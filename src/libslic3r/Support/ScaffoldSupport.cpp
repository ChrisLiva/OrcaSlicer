#include "ScaffoldSupport.hpp"

#include <algorithm>
#include <chrono>

#include "ClipperUtils.hpp"
#include "Model.hpp"
#include "Print.hpp"
#include "TriangleMesh.hpp"
#include "TriangleMeshSlicer.hpp"
#include "libslic3r/SLA/SupportTreeBuilder.hpp"
#include "libslic3r/SLA/SupportTreeBuildsteps.hpp"
#include "libslic3r/SLA/SupportTreeMesher.hpp"

#include <boost/log/trivial.hpp>

namespace Slic3r::ScaffoldSupport {

namespace {

uint32_t ms_since(const std::chrono::steady_clock::time_point &start)
{
    return uint32_t(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count());
}

// The SLA builder's config for a cage at zero elevation: tips as fine as one support line, pillars at the tree
// branch diameter, bridges no longer than the scaffold bridge length, and nothing anchored on the model.
sla::SupportTreeConfig tree_config(const Params &params)
{
    sla::SupportTreeConfig cfg;
    cfg.head_front_radius_mm        = params.toolpath_width_mm;
    cfg.head_penetration_mm         = params.toolpath_width_mm;
    cfg.head_fallback_radius_mm     = params.toolpath_width_mm;
    cfg.head_back_radius_mm         = params.pillar_diameter_mm / 2.;
    cfg.head_width_mm               = 1.;
    cfg.base_radius_mm              = params.pillar_diameter_mm;
    cfg.base_height_mm              = 0.5;
    cfg.object_elevation_mm         = 0.;
    cfg.ground_facing_only          = false;
    cfg.allow_model_anchors         = false;
    cfg.pillar_connection_mode      = sla::PillarConnectionMode::zigzag;
    cfg.max_bridge_length_mm        = params.bridge_length_mm;
    cfg.max_pillar_link_distance_mm = params.bridge_length_mm;
    cfg.max_bridges_on_pillar       = 3;
    cfg.bridge_slope                = M_PI / 4.;
    cfg.safety_distance_mm          = params.xy_distance_mm;
    cfg.pillar_link_slenderness     = 0.;
    return cfg;
}

} // namespace

Output draw(const PrintObject &object, const std::vector<std::vector<SupportNode *>> &contacts,
            const std::vector<SupportNode *> & /* dropped */, const std::vector<LayerHeightData> &layer_heights,
            const ModelSupportRisk::Field &risk, const Params &params, const std::function<void()> &throw_on_cancel)
{
    Output out;
    out.layers.resize(layer_heights.size());
    for (const LayerHeightData &plan : layer_heights) {
        if (plan.print_z > params.pad_thickness_mm + EPSILON)
            break;
        ++ out.pad_layers;
    }

    // One tip per contact. Its grade is the width of the disc it fuses to the model with: two support lines, or
    // four where the model under it hangs off a neck at least eight lines wide. The pin is half the grade.
    const double w          = params.toolpath_width_mm;
    const bool   risk_known = risk.status == ModelSupportRisk::Field::Status::Complete;
    sla::SupportPoints points;
    for (size_t i = 0; i < std::min(contacts.size(), layer_heights.size()); ++ i)
        for (const SupportNode *node : contacts[i]) {
            // An interior tip is kept only where its overhang holds a disc as wide as the longest bridge: under a
            // narrower overhang the tips on its rim already hold it.
            if (node->placement == SupportNode::Placement::Interior &&
                (node->overhang.empty() || offset_ex(node->overhang, -scale_(params.max_bridge_length_mm / 2.)).empty()))
                continue;
            double grade = 2. * w;
            if (risk_known) {
                const ModelSupportRisk::Sample s = ModelSupportRisk::sample(risk, size_t(node->obj_layer_nr + 1), node->position);
                if (s.status == ModelSupportRisk::Sample::Status::Known && s.neck_width_mm >= 8. * w)
                    grade = 4. * w;
            }
            const Vec2d xy = unscale(node->position);
            points.emplace_back(float(xy.x()), float(xy.y()), float(node->print_z - params.z_offset_mm), float(grade / 2.));
        }
    out.counts.tips_placed = points.size();

    // The object in the frame its slices are in: XY centred, bottom on z 0. The builder's mesh index points
    // into it, so it lives for the whole call.
    TriangleMesh mesh = object.model_object()->raw_mesh();
    mesh.transform(object.trafo_centered());

    const auto build_start = std::chrono::steady_clock::now();
    sla::SupportableMesh sm(mesh.its, points, tree_config(params));
    // The copy the SupportableMesh holds drops any ground offset its source carried, so the offset goes on
    // the copy: pillars end on the pad's top face.
    sm.emesh.ground_level_offset(params.pad_thickness_mm);
    sla::SupportTreeBuilder builder;
    sla::JobController      ctl;
    ctl.stopcondition = [&object] { return object.print()->canceled(); };
    ctl.cancelfn      = throw_on_cancel;
    builder.set_ctl(ctl);
    if (sla::SupportTreeBuildsteps::execute(builder, sm))
        throw_on_cancel();

    // What became of each tip. A head the builder kept carries its point's index as its id; one it gave up on
    // lost the id and is found by the position it was built at; a point with no head was filtered out.
    enum class Tip : uint8_t { Filtered, Unrouted, Routed };
    std::vector<Tip>    tips(points.size(), Tip::Filtered);
    std::vector<size_t> by_pos(points.size());
    for (size_t i = 0; i < by_pos.size(); ++ i)
        by_pos[i] = i;
    const auto pos_less = [](const Vec3d &a, const Vec3d &b) { return std::lexicographical_compare(a.data(), a.data() + 3, b.data(), b.data() + 3); };
    const auto pos_of   = [&points](size_t i) { return Vec3d(points[i].pos.cast<double>()); };
    std::sort(by_pos.begin(), by_pos.end(), [&](size_t a, size_t b) { return pos_less(pos_of(a), pos_of(b)); });
    for (const sla::Head &head : builder.heads()) {
        if (head.is_valid()) {
            if (size_t(head.id) < tips.size())
                tips[head.id] = Tip::Routed;
            continue;
        }
        auto it = std::lower_bound(by_pos.begin(), by_pos.end(), head.pos,
                                   [&](size_t i, const Vec3d &pos) { return pos_less(pos_of(i), pos); });
        for (; it != by_pos.end() && (pos_of(*it) - head.pos).norm() <= 1e-6; ++ it)
            if (tips[*it] == Tip::Filtered) {
                tips[*it] = Tip::Unrouted;
                break;
            }
    }
    for (size_t i = 0; i < tips.size(); ++ i) {
        if (tips[i] == Tip::Routed) {
            ++ out.counts.tips_routed;
            continue;
        }
        ++ out.counts.tips_dropped;
        const Vec3f &p = points[i].pos;
        BOOST_LOG_TRIVIAL(debug) << "scaffold tip dropped at (" << p.x() << ", " << p.y() << ", " << p.z()
                                 << "): " << (tips[i] == Tip::Unrouted ? "unrouted" : "filtered");
    }

    // The cage and the pad under it as one mesh. A cage with no part routed gets no pad.
    indexed_triangle_set cage = builder.retrieve_mesh(sla::MeshType::Support);
    if (! cage.indices.empty()) {
        sla::PadConfig pad;
        pad.wall_thickness_mm    = params.pad_thickness_mm;
        pad.wall_height_mm       = 0.;
        pad.brim_size_mm         = 1.6;
        pad.embed_object.enabled = false;
        builder.add_pad({}, pad);
        its_merge(cage, builder.retrieve_mesh(sla::MeshType::Pad));
    }
    out.stage_ms.build = ms_since(build_start);

    // Each planned layer is the cage's section through the layer's middle.
    const auto slice_start = std::chrono::steady_clock::now();
    if (! cage.indices.empty() && ! layer_heights.empty()) {
        std::vector<float> zs;
        zs.reserve(layer_heights.size());
        for (const LayerHeightData &plan : layer_heights)
            zs.push_back(float(plan.print_z - 0.5 * plan.height - params.z_offset_mm));
        std::vector<ExPolygons> slices = slice_mesh_ex(cage, zs, 0.f, throw_on_cancel);
        for (size_t i = 0; i < std::min(slices.size(), out.layers.size()); ++ i)
            out.layers[i].base = union_ex(slices[i]);
    }

    // Each routed tip's own head, sliced on its top interface layer, the highest planned layer whose top is at
    // or under the tip, and on the layers under it up to the interface count, prints as interface.
    if (params.interface_layers > 0) {
        for (const sla::Head &head : builder.heads()) {
            if (! head.is_valid())
                continue;
            throw_on_cancel();
            const double tip_z = head.pos.z() + params.z_offset_mm;
            const size_t above = size_t(std::upper_bound(layer_heights.begin(), layer_heights.end(), tip_z + EPSILON,
                                                         [](double z, const LayerHeightData &plan) { return z < plan.print_z; }) -
                                        layer_heights.begin());
            const size_t count = std::min(above, params.interface_layers);
            if (count == 0)
                continue;
            // The slicer wants its heights ascending.
            const size_t       first = above - count;
            std::vector<float> zs;
            for (size_t i = first; i < above; ++ i)
                zs.push_back(float(layer_heights[i].print_z - params.z_offset_mm));
            const std::vector<ExPolygons> rings = slice_mesh_ex(sla::get_mesh(head, 45), zs, 0.f);
            for (size_t k = 0; k < std::min(count, rings.size()); ++ k)
                append(out.layers[first + k].interface_, rings[k]);
        }
        for (LayerAreas &layer : out.layers)
            if (! layer.interface_.empty()) {
                layer.interface_ = union_ex(layer.interface_);
                layer.base       = diff_ex(layer.base, layer.interface_);
            }
    }
    out.stage_ms.slice = ms_since(slice_start);
    return out;
}

} // namespace Slic3r::ScaffoldSupport
