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
            const ModelSupportRisk::Field & /* risk */, const Params &params, const std::function<void()> &throw_on_cancel)
{
    Output out;
    out.layers.resize(layer_heights.size());
    for (const LayerHeightData &plan : layer_heights) {
        if (plan.print_z > params.pad_thickness_mm + EPSILON)
            break;
        ++ out.pad_layers;
    }

    // One tip per contact, at the small grade: the pin is one support line wide.
    sla::SupportPoints points;
    for (size_t i = 0; i < std::min(contacts.size(), layer_heights.size()); ++ i)
        for (const SupportNode *node : contacts[i]) {
            const Vec2d xy = unscale(node->position);
            points.emplace_back(float(xy.x()), float(xy.y()), float(node->print_z - params.z_offset_mm), float(params.toolpath_width_mm));
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
    for (const sla::Head &head : builder.heads())
        if (head.is_valid())
            ++ out.counts.tips_routed;
    out.counts.tips_dropped = out.counts.tips_placed - out.counts.tips_routed;

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
    out.stage_ms.slice = ms_since(slice_start);
    return out;
}

} // namespace Slic3r::ScaffoldSupport
