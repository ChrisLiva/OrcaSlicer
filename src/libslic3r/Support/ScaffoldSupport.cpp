#include "ScaffoldSupport.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <unordered_map>

#include "ClipperUtils.hpp"
#include "Model.hpp"
#include "Print.hpp"
#include "TriangleMesh.hpp"
#include "TriangleMeshSlicer.hpp"
#include "SupportComponents.hpp"
#include "libslic3r/SLA/SupportTreeBuilder.hpp"
#include "libslic3r/SLA/SupportTreeBuildsteps.hpp"
#include "libslic3r/SLA/SupportTreeMesher.hpp"

#include <boost/functional/hash.hpp>
#include <boost/log/trivial.hpp>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

namespace Slic3r::ScaffoldSupport {

namespace {

// The head's length between the pin's sphere and the pillar's radius. The cone meets the pillar this far under the
// sphere: a tip's neck bottoms at about this plus one toolpath width under the tip for the small grade and three
// for the large, and the wall test reads the shallower depth.
constexpr double head_width_mm = 1.;

uint32_t ms_since(const std::chrono::steady_clock::time_point &start)
{
    return uint32_t(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count());
}

// The SLA builder's config for a cage at zero elevation: tips as fine as one support line, pillars at the tree
// branch diameter, bridges and braces no longer than the scaffold bridge length, a brace on every pillar standing
// more than the brace slenderness in diameters unbraced, and nothing anchored on the model.
sla::SupportTreeConfig tree_config(const Params &params)
{
    sla::SupportTreeConfig cfg;
    cfg.head_front_radius_mm        = params.toolpath_width_mm;
    cfg.head_penetration_mm         = params.toolpath_width_mm;
    cfg.head_fallback_radius_mm     = params.toolpath_width_mm;
    cfg.head_back_radius_mm         = params.pillar_diameter_mm / 2.;
    cfg.head_width_mm               = head_width_mm;
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
    cfg.pillar_link_slenderness     = params.brace_slenderness;
    return cfg;
}

// The seed a contact was placed for, or the largest id for a node the seed pass never named.
uint64_t seed_id(const SupportNode &node) { return node.source_ids.empty() ? std::numeric_limits<uint64_t>::max() : node.source_ids.front(); }

// Where a tip stands. `node` is the contact it stands for, or null for a tip the hold floor seeded under an island
// the front half left without one.
struct TipSite
{
    Point              position;
    double             print_z      = 0.;
    int                obj_layer_nr = 0;
    uint64_t           seed         = std::numeric_limits<uint64_t>::max();
    const SupportNode *node         = nullptr;
};

TipSite site_of(const SupportNode &node) { return { node.position, node.print_z, node.obj_layer_nr, seed_id(node), &node }; }

// The tips a model island needs for its unjoined height: one for a sliver, two up to 5 mm, three above. A slab z is
// a sum of layer heights, so a band edge carries an epsilon.
size_t hold_floor(double height_mm) { return height_mm <= 1. + EPSILON ? 1 : height_mm <= 5. + EPSILON ? 2 : 3; }

// The point of `piece` furthest inside it, to the binary search's 0.01 mm: the middle of its deepest inward offset.
Point inscribed_point(const ExPolygon &piece)
{
    ExPolygons deepest { piece };
    double     inside = 0., outside = 0.5 * double(get_extents(piece).size().minCoeff());
    while (outside - inside > scale_(0.01)) {
        const double depth = 0.5 * (inside + outside);
        if (ExPolygons shrunk = offset_ex(piece, -float(depth)); shrunk.empty())
            outside = depth;
        else {
            inside  = depth;
            deepest = std::move(shrunk);
        }
    }
    const Point middle = deepest.front().contour.centroid();
    return deepest.front().contains(middle) ? middle : deepest.front().contour.points.front();
}

// How many tips a pillar diameter apart `piece` holds: the points of a hexagonal grid at that spacing inside the piece
// shrunk by half of it. A tip stands anywhere in a piece whatever its width, so every piece holds at least one, even
// where the shrunk piece is empty.
size_t tips_fitting(const ExPolygon &piece, double pillar_diameter_mm)
{
    const ExPolygons room = offset_ex(piece, -float(scale_(pillar_diameter_mm / 2.)));
    if (room.empty())
        return 1;
    const BoundingBox bbox  = get_extents(room);
    const coord_t     step  = coord_t(scale_(pillar_diameter_mm));
    const coord_t     pitch = coord_t(std::lround(double(step) * std::sqrt(3.) / 2.));
    size_t            fit   = 0;
    size_t            row   = 0;
    for (coord_t y = bbox.min.y(); y <= bbox.max.y(); y += pitch, ++ row)
        for (coord_t x = bbox.min.x() + (row % 2 == 0 ? 0 : step / 2); x <= bbox.max.x(); x += step)
            if (std::any_of(room.begin(), room.end(), [p = Point(x, y)](const ExPolygon &expoly) { return expoly.contains(p); }))
                ++ fit;
    return std::max<size_t>(fit, 1);
}

// Holds each mid-air island of the model with its floor of tips a pillar diameter apart, and returns how many islands
// stay short of the floor or of the tips their birth piece fits, whichever is fewer. A tip belongs to the island that
// owns the model piece over it, on the overhang's own layer, one above the node's. An island that never joins and
// starts at most 1 mm under the object's top is mesh debris: no floor, no tip, not counted. An island with no tip and
// no dropped contact gets one tip seeded at the deepest point of its birth piece, at the piece's bottom, unless
// `at_wall` skips it; short of the floor, the dropped contacts under it come back.
size_t restore_hold_floor(const PrintObject &object, std::vector<TipSite> &tips, const std::vector<TipSite> &dropped,
                          double pillar_diameter_mm, const std::function<bool(const TipSite &)> &at_wall)
{
    using namespace SupportAnalysis;
    const std::vector<Slab> slabs = model_slabs_of(object);
    if (slabs.empty())
        return 0;
    const IslandMap map = island_joins(slabs, slabs.front().bottom_z);
    if (map.islands.empty())
        return 0;
    const auto island_of = [&](const TipSite &tip) {
        const size_t s = size_t(tip.obj_layer_nr + 1);
        if (tip.obj_layer_nr + 1 < 0 || s >= map.components.slab_range.size())
            return size_t(-1);
        for (size_t p = map.components.slab_range[s].first; p < map.components.slab_range[s].second; ++ p)
            if (map.components.pieces[p].polygon.contains(tip.position))
                return map.island_of_piece[p];
        return size_t(-1);
    };
    std::vector<std::vector<TipSite>> kept(map.islands.size()), spare(map.islands.size());
    for (const TipSite &tip : tips)
        if (const size_t k = island_of(tip); k < kept.size())
            kept[k].push_back(tip);
    for (const TipSite &tip : dropped)
        if (const size_t k = island_of(tip); k < spare.size())
            spare[k].push_back(tip);

    const double min_spacing = scale_(pillar_diameter_mm);
    const auto   xy_distance = [](const TipSite &a, const TipSite &b) { return (a.position - b.position).cast<double>().norm(); };
    size_t       under_held  = 0;
    for (size_t k = 0; k < map.islands.size(); ++ k) {
        const IslandJoin &join   = map.islands[k];
        const bool        joins  = join.join_slab < slabs.size();
        const Slab       &birth  = slabs[join.birth_slab];
        const double      height = (joins ? slabs[join.join_slab].bottom_z : slabs.back().print_z) - birth.bottom_z;
        if (! joins && height <= 1. + EPSILON) {
            BOOST_LOG_TRIVIAL(debug) << "scaffold island skipped at " << birth.bottom_z << ": debris";
            continue;
        }
        const size_t floor = hold_floor(height);
        // The piece the island starts from: the one piece of its birth slab it owns, since that piece has nothing under
        // it to share a set with.
        const ExPolygon *birth_piece = nullptr;
        for (size_t p = map.components.slab_range[join.birth_slab].first; p < map.components.slab_range[join.birth_slab].second; ++ p)
            if (map.island_of_piece[p] == k)
                birth_piece = &map.components.pieces[p].polygon;
        if (kept[k].empty() && spare[k].empty() && birth_piece != nullptr) {
            const TipSite seeded { inscribed_point(*birth_piece), birth.bottom_z, int(join.birth_slab) - 1 };
            if (! at_wall(seeded)) {
                const Vec2d xy = unscale(seeded.position);
                BOOST_LOG_TRIVIAL(debug) << "scaffold tip seeded at (" << xy.x() << ", " << xy.y() << ", " << seeded.print_z << ")";
                kept[k].push_back(seeded);
                tips.push_back(seeded);
            }
        }
        // Greedy, lowest first: a tip counts where it stands a pillar diameter from every tip counted before it.
        std::vector<TipSite> counted;
        const auto count = [&](const TipSite &tip) {
            if (std::all_of(counted.begin(), counted.end(), [&](const TipSite &c) { return xy_distance(tip, c) >= min_spacing; }))
                counted.push_back(tip);
        };
        std::sort(kept[k].begin(), kept[k].end(), [](const TipSite &a, const TipSite &b) {
            return a.print_z != b.print_z ? a.print_z < b.print_z : a.seed < b.seed;
        });
        for (const TipSite &tip : kept[k])
            if (counted.size() < floor)
                count(tip);
        // Short of the floor: the dropped contacts come back lowest first, among equals the one furthest from the
        // tips the island already has.
        std::vector<TipSite> &candidates = spare[k];
        std::sort(candidates.begin(), candidates.end(), [](const TipSite &a, const TipSite &b) { return a.print_z < b.print_z; });
        size_t next = 0;
        while (counted.size() < floor && next < candidates.size()) {
            size_t tie_end = next;
            while (tie_end < candidates.size() && candidates[tie_end].print_z <= candidates[next].print_z + EPSILON)
                ++ tie_end;
            const auto nearest_kept = [&](const TipSite &tip) {
                double d = std::numeric_limits<double>::max();
                for (const TipSite &t : kept[k])
                    d = std::min(d, xy_distance(tip, t));
                return d;
            };
            size_t best = next;
            double best_d = nearest_kept(candidates[next]);
            for (size_t i = next + 1; i < tie_end; ++ i)
                if (const double d = nearest_kept(candidates[i]); d > best_d) {
                    best   = i;
                    best_d = d;
                }
            std::swap(candidates[next], candidates[best]);
            const TipSite restored = candidates[next ++];
            kept[k].push_back(restored);
            tips.push_back(restored);
            count(restored);
        }
        // An island counts as under-held only where its birth piece has room for more tips than it got.
        if (counted.size() < floor && (birth_piece == nullptr || counted.size() < tips_fitting(*birth_piece, pillar_diameter_mm)))
            ++ under_held;
    }
    return under_held;
}

// Merges the tips standing within the builder's alias distance `sla::D_SP` of each other in 3-D, xy from the node's
// position and z its print z. The front half hands the same overhang spot on consecutive layers, and the builder's
// `filter` keeps one point of each such pair, so the other read as a drop. Tips are visited lowest first, among
// equals by seed id, and a tip within the distance of a tip already kept merges into it; a merged tip absorbs none.
// A chain at the layer pitch therefore keeps every tip standing further than the distance from the kept tips under
// it, and three tips all within the distance of each other keep one where the builder's pairs would keep two: no
// two kept tips are aliases, so the builder filters none. The kept tips sit in a grid of cells the distance wide,
// so a tip reads the 27 cells around its own. `tips` keeps its order.
void merge_aliases(std::vector<TipSite> &tips)
{
    using Cell = std::array<int64_t, 3>;
    std::unordered_map<Cell, std::vector<Vec3d>, boost::hash<Cell>> kept;
    const auto aliased = [&kept](const Vec3d &p, const Cell &c) {
        const auto within = [&p](const Vec3d &q) { return (q - p).norm() <= sla::D_SP; };
        for (int64_t dx = -1; dx <= 1; ++ dx)
            for (int64_t dy = -1; dy <= 1; ++ dy)
                for (int64_t dz = -1; dz <= 1; ++ dz)
                    if (const auto it = kept.find(Cell{c[0] + dx, c[1] + dy, c[2] + dz});
                        it != kept.end() && std::any_of(it->second.begin(), it->second.end(), within))
                        return true;
        return false;
    };
    std::vector<size_t> order(tips.size());
    for (size_t i = 0; i < order.size(); ++ i)
        order[i] = i;
    std::sort(order.begin(), order.end(), [&tips](size_t a, size_t b) {
        const TipSite &ta = tips[a], &tb = tips[b];
        if (ta.print_z != tb.print_z)
            return ta.print_z < tb.print_z;
        if (ta.seed != tb.seed)
            return ta.seed < tb.seed;
        return ta.position.x() != tb.position.x() ? ta.position.x() < tb.position.x() : ta.position.y() < tb.position.y();
    });
    std::vector<bool> keep(tips.size(), false);
    for (const size_t i : order) {
        const Vec2d xy = unscale(tips[i].position);
        const Vec3d p(xy.x(), xy.y(), tips[i].print_z);
        const Cell  c{int64_t(std::floor(p.x() / sla::D_SP)), int64_t(std::floor(p.y() / sla::D_SP)),
                      int64_t(std::floor(p.z() / sla::D_SP))};
        if (aliased(p, c)) {
            BOOST_LOG_TRIVIAL(debug) << "scaffold tip merged at (" << p.x() << ", " << p.y() << ", " << p.z() << ")";
            continue;
        }
        keep[i] = true;
        kept[c].push_back(p);
    }
    size_t next = 0;
    for (size_t i = 0; i < tips.size(); ++ i)
        if (keep[i])
            tips[next ++] = tips[i];
    tips.resize(next);
}

} // namespace

Output draw(const PrintObject &object, const std::vector<std::vector<SupportNode *>> &contacts,
            const std::vector<SupportNode *> &dropped, const std::vector<LayerHeightData> &layer_heights,
            const ModelSupportRisk::Field &risk, const Params &params, const std::function<void()> &throw_on_cancel)
{
    Output out;
    out.layers.resize(layer_heights.size());
    for (const LayerHeightData &plan : layer_heights) {
        if (plan.print_z > params.pad_thickness_mm + EPSILON)
            break;
        ++ out.pad_layers;
    }

    // One tip per contact. An interior tip is kept only where its overhang holds a disc as wide as the longest bridge:
    // under a narrower overhang the tips on its rim already hold it.
    std::vector<TipSite> nodes;
    for (size_t i = 0; i < std::min(contacts.size(), layer_heights.size()); ++ i)
        for (const SupportNode *node : contacts[i])
            if (node->placement != SupportNode::Placement::Interior ||
                (! node->overhang.empty() && ! offset_ex(node->overhang, -scale_(params.max_bridge_length_mm / 2.)).empty()))
                nodes.push_back(site_of(*node));

    // A tip whose centre stands within the xy distance of the model at its neck's bottom is skipped, and so is a
    // dropped contact the hold floor could restore there: the seam clips support inside that band, so the neck
    // would be cut while its ring survived. Under a slope the head tilts along the underside's normal and the
    // slope recedes at least its rise by that depth, so the neck clears the band; beside a wall it does not, and
    // the wall anchors that band as it does under the legacy tree. A neck bottoming under the first layer stands
    // by the pad and is kept.
    const double neck_depth_mm = head_width_mm + params.toolpath_width_mm;
    const auto   reference_layer = [&object, neck_depth_mm](const TipSite &tip) {
        const double z = tip.print_z - neck_depth_mm;
        if (object.layer_count() == 0 || z <= object.get_layer(0)->bottom_z())
            return -1;
        const auto it = std::lower_bound(object.layers().begin(), object.layers().end(), z,
                                         [](const Layer *layer, double z) { return layer->print_z < z; });
        return it == object.layers().end() ? -1 : int(it - object.layers().begin());
    };
    std::vector<TipSite> spare;
    for (const SupportNode *node : dropped)
        spare.push_back(site_of(*node));
    std::vector<int> wall_layers;
    for (const TipSite &tip : nodes)
        if (const int l = reference_layer(tip); l >= 0)
            wall_layers.push_back(l);
    for (const TipSite &tip : spare)
        if (const int l = reference_layer(tip); l >= 0)
            wall_layers.push_back(l);
    std::sort(wall_layers.begin(), wall_layers.end());
    wall_layers.erase(std::unique(wall_layers.begin(), wall_layers.end()), wall_layers.end());
    const auto wall_band = [&object, &params](int layer) { return offset_ex(object.get_layer(layer)->lslices, scale_(params.xy_distance_mm)); };
    std::vector<ExPolygons> wall_bands(wall_layers.size());
    tbb::parallel_for(tbb::blocked_range<size_t>(0, wall_layers.size()), [&](const tbb::blocked_range<size_t> &range) {
        for (size_t i = range.begin(); i < range.end(); ++ i)
            wall_bands[i] = wall_band(wall_layers[i]);
    });
    // A tip the hold floor seeds may stand over a layer no other tip does, so its band is offset on the spot.
    const auto at_wall = [&](const TipSite &tip) {
        const int l = reference_layer(tip);
        if (l < 0)
            return false;
        const auto        it     = std::lower_bound(wall_layers.begin(), wall_layers.end(), l);
        const bool        listed = it != wall_layers.end() && *it == l;
        const ExPolygons  own    = listed ? ExPolygons() : wall_band(l);
        const ExPolygons &band   = listed ? wall_bands[size_t(it - wall_layers.begin())] : own;
        if (std::none_of(band.begin(), band.end(), [&tip](const ExPolygon &expoly) { return expoly.contains(tip.position); }))
            return false;
        const Vec2d xy = unscale(tip.position);
        BOOST_LOG_TRIVIAL(debug) << "scaffold tip skipped at (" << xy.x() << ", " << xy.y() << ", " << tip.print_z << "): wall";
        return true;
    };
    nodes.erase(std::remove_if(nodes.begin(), nodes.end(), at_wall), nodes.end());
    spare.erase(std::remove_if(spare.begin(), spare.end(), at_wall), spare.end());

    const auto islands_start      = std::chrono::steady_clock::now();
    out.counts.islands_under_held = restore_hold_floor(object, nodes, spare, params.pillar_diameter_mm, at_wall);
    out.stage_ms.island_joins     = ms_since(islands_start);
    // The floor counted tips a pillar diameter apart, so no alias counted there twice; the merge comes after it so
    // that a restored contact standing on a kept one merges too.
    merge_aliases(nodes);

    // A tip's grade is the width of the disc it fuses to the model with: two support lines, or four where the model
    // under it hangs off a neck at least eight lines wide. The pin is half the grade. A seeded tip starts a feature
    // and keeps the small grade.
    const double w          = params.toolpath_width_mm;
    const bool   risk_known = risk.status == ModelSupportRisk::Field::Status::Complete;
    std::vector<double> grades(nodes.size(), 2. * w);
    if (risk_known)
        tbb::parallel_for(tbb::blocked_range<size_t>(0, nodes.size()), [&](const tbb::blocked_range<size_t> &range) {
            for (size_t i = range.begin(); i < range.end(); ++ i) {
                if (nodes[i].node == nullptr)
                    continue;
                const ModelSupportRisk::Sample s = ModelSupportRisk::sample(risk, size_t(nodes[i].obj_layer_nr + 1), nodes[i].position);
                if (s.status == ModelSupportRisk::Sample::Status::Known && s.neck_width_mm >= 8. * w)
                    grades[i] = 4. * w;
            }
        });
    // A head's id is its point's index, so the points keep the order of `nodes`.
    sla::SupportPoints points;
    for (size_t i = 0; i < nodes.size(); ++ i) {
        const Vec2d xy = unscale(nodes[i].position);
        points.emplace_back(float(xy.x()), float(xy.y()), float(nodes[i].print_z - params.z_offset_mm), float(grades[i] / 2.));
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
    out.counts.pillars_unbraced = builder.unbraced_pillars;

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
