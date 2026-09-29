#include "ScaffoldSupport.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <unordered_map>

#include "ClipperUtils.hpp"
#include "ExtrusionEntity.hpp"
#include "Print.hpp"
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
// more than the brace slenderness in diameters unbraced, and nothing anchored on the model. Heads facing the model
// route in the points' order, so one slice builds one tree, and one no route reaches retries thin and, holding an
// island, along leaning axes.
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
    cfg.bridge_slope                = max_head_tilt_rad;
    cfg.safety_distance_mm          = params.xy_distance_mm;
    cfg.pillar_link_slenderness     = params.brace_slenderness;
    cfg.pillar_link_radius_mm       = params.brace_diameter_mm / 2.;
    cfg.route_in_order              = true;
    cfg.retry_thin_head             = true;
    cfg.island_axis_retry           = true;
    return cfg;
}

// The seed a contact was placed for, or the largest id for a node the seed pass never named.
uint64_t seed_id(const SupportNode &node) { return node.source_ids.empty() ? std::numeric_limits<uint64_t>::max() : node.source_ids.front(); }

TipSite site_of(const SupportNode &node) { return { node.position, node.print_z, node.obj_layer_nr, seed_id(node), node.is_pinned }; }

// How far a tip's neck runs from the tip along its axis, where the planner and the wall skip read its end.
double neck_depth_mm(const Params &params) { return head_width_mm + params.toolpath_width_mm; }

// The plan input `prepare_plan` builds for `object` at the widths, distances and neck depth of `params`, with the full
// head `tree_config` gives the builder.
PlanInput plan_input(const PrintObject &object, const Params &params, double threshold_rad, const std::vector<Polygons> &blockers,
                     const ObjectMesh *mesh)
{
    PlanInput                    input = prepare_plan(object, params.toolpath_width_mm, params.xy_distance_mm, neck_depth_mm(params),
                                                      params.max_bridge_length_mm, threshold_rad, blockers, mesh);
    const sla::SupportTreeConfig cfg   = tree_config(params);
    input.head                         = { cfg.head_back_radius_mm, cfg.head_fullwidth(), cfg.safety_distance_mm };
    return input;
}

// The model piece over `tip`, on the overhang's own layer, one above the node's; npos where none holds it.
size_t piece_of(const SupportAnalysis::IslandMap &map, const TipSite &tip)
{
    const size_t s = size_t(tip.obj_layer_nr + 1);
    if (tip.obj_layer_nr + 1 < 0 || s >= map.components.slab_range.size())
        return size_t(-1);
    for (size_t p = map.components.slab_range[s].first; p < map.components.slab_range[s].second; ++ p)
        if (map.components.pieces[p].polygon.contains(tip.position))
            return p;
    return size_t(-1);
}

// The index of the piece island `k` starts from, the one piece of its birth slab it owns, since that piece has nothing
// under it to share a set with; npos where none does.
size_t birth_piece_index(const SupportAnalysis::IslandMap &map, size_t k)
{
    const size_t birth_slab = map.islands[k].birth_slab;
    size_t       birth      = size_t(-1);
    for (size_t p = map.components.slab_range[birth_slab].first; p < map.components.slab_range[birth_slab].second; ++ p)
        if (map.island_of_piece[p] == k)
            birth = p;
    return birth;
}

// Where each mid-air island of the model prints with no tip of a baked list holding it, read by the planner's island
// rule, so a list and an automatic slice hold an island alike. A tip belongs to the island that owns the model piece
// over it, on the overhang's own layer, one above the node's, and is marked `holds_island`. It holds the island only
// on the birth piece, the island's one piece with nothing under it, as the planner holds a birth with the enforced tip
// on that piece and stands no tip higher on the part in for it. An island with no tip on its birth piece is read by
// the birth rule, `read_births`: one continuing the slab below, debris, or a nub the list's tips or the bed hold at its
// merge needs none. Any other prints unheld, named at the tip the rule would stand under it, or, where no neck clears,
// at its birth point, the deepest point of its birth piece at the piece's bottom, as the plan names it.
std::vector<Vec3d> unheld_islands(const PlanInput &input, std::vector<TipSite> &tips)
{
    using namespace SupportAnalysis;
    std::vector<Vec3d> bare;
    if (input.slabs.empty())
        return bare;
    // `build_components` over the plan's own slabs and ground, so the map's pieces are the plan input's, index for index.
    const IslandMap   map = island_joins(input.slabs, input.slabs.front().bottom_z);
    std::vector<char> held(map.islands.size(), 0);
    for (TipSite &tip : tips)
        if (const size_t p = piece_of(map, tip); p != size_t(-1))
            if (const size_t k = map.island_of_piece[p]; k < held.size()) {
                tip.holds_island = true;
                if (map.components.pieces[p].below.empty())
                    held[k] = 1;
            }
    std::vector<size_t> births;
    for (size_t k = 0; k < map.islands.size(); ++ k)
        if (const size_t p = birth_piece_index(map, k); ! held[k] && p != size_t(-1))
            births.push_back(p);
    const std::vector<BirthRead> reads = read_births(input, births, tips);
    for (size_t i = 0; i < births.size(); ++ i) {
        const Piece &piece = map.components.pieces[births[i]];
        if (reads[i].hold == BirthHold::Tip) {
            const Vec2d xy = unscale(reads[i].site.position);
            bare.emplace_back(xy.x(), xy.y(), reads[i].site.print_z);
        } else if (reads[i].hold == BirthHold::NoNeck) {
            const Vec2d xy = unscale(inscribed_point(piece.polygon));
            bare.emplace_back(xy.x(), xy.y(), piece.bottom_z);
        } else
            continue;
        BOOST_LOG_TRIVIAL(debug) << "scaffold island at (" << bare.back().x() << ", " << bare.back().y() << ", " << bare.back().z()
                                 << ") unheld: " << (reads[i].hold == BirthHold::Tip ? "no point" : "no neck");
    }
    return bare;
}

// A tip's point as the alias merge measures it: xy from its position, z its print z.
Vec3d alias_point(const TipSite &tip)
{
    const Vec2d xy = unscale(tip.position);
    return { xy.x(), xy.y(), tip.print_z };
}

// The kept tips of `merge_aliases` by cell of its grid, each with its 3-D point and its index in `tips`.
using AliasCell = std::array<int64_t, 3>;
using AliasGrid = std::unordered_map<AliasCell, std::vector<std::pair<Vec3d, size_t>>, boost::hash<AliasCell>>;

// The index of the kept tip `p` is an alias of, or npos.
size_t alias_of(const AliasGrid &kept, const Vec3d &p, const AliasCell &c)
{
    for (int64_t dx = -1; dx <= 1; ++ dx)
        for (int64_t dy = -1; dy <= 1; ++ dy)
            for (int64_t dz = -1; dz <= 1; ++ dz)
                if (const auto it = kept.find(AliasCell{c[0] + dx, c[1] + dy, c[2] + dz}); it != kept.end())
                    for (const auto &[q, i] : it->second)
                        if ((q - p).norm() <= sla::D_SP)
                            return i;
    return size_t(-1);
}

// Merges the tips standing within the builder's alias distance `sla::D_SP` of each other in 3-D, xy from the node's
// position and z its print z. The front half hands the same overhang spot on consecutive layers, and the builder's
// `filter` keeps one point of each such pair, so the other read as a drop. Tips are visited lowest first, among
// equals by seed id, and a tip within the distance of a tip already kept merges into it; a merged tip absorbs none.
// A chain at the layer pitch therefore keeps every tip standing further than the distance from the kept tips under
// it, and three tips all within the distance of each other keep one where the builder's pairs would keep two: no
// two kept tips are aliases, so the builder filters none. A kept tip is enforced when a tip merged into it was, and
// takes the larger head grade of the two, so a Heavy point merged into a Light one keeps its Heavy head, holds an
// island when either did, and keeps its own axis. The kept tips sit in a grid of cells the distance wide, so a tip
// reads the 27 cells around its own.
// `tips` keeps its order.
void merge_aliases(std::vector<TipSite> &tips)
{
    AliasGrid kept;
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
        const Vec3d     p = alias_point(tips[i]);
        const AliasCell c{int64_t(std::floor(p.x() / sla::D_SP)), int64_t(std::floor(p.y() / sla::D_SP)),
                          int64_t(std::floor(p.z() / sla::D_SP))};
        if (const size_t into = alias_of(kept, p, c); into != size_t(-1)) {
            BOOST_LOG_TRIVIAL(debug) << "scaffold tip merged at (" << p.x() << ", " << p.y() << ", " << p.z() << ")";
            tips[into].enforced = tips[into].enforced || tips[i].enforced;
            tips[into].grade_mm = std::max(tips[into].grade_mm, tips[i].grade_mm);
            tips[into].holds_island = tips[into].holds_island || tips[i].holds_island;
            continue;
        }
        keep[i] = true;
        kept[c].emplace_back(p, i);
    }
    size_t next = 0;
    for (size_t i = 0; i < tips.size(); ++ i)
        if (keep[i])
            tips[next ++] = tips[i];
    tips.resize(next);
}

// Where `tip`'s neck ends, `depth_mm` from the tip along its axis, or straight down where it has none.
Vec3d neck_end(const TipSite &tip, double depth_mm)
{
    const Vec3d axis = tip.axis.isZero() ? Vec3d(0., 0., -1.) : Vec3d(tip.axis.cast<double>().normalized());
    return alias_point(tip) + depth_mm * axis;
}

// The first object layer at or above the bottom of `tip`'s neck, or -1 where the neck bottoms under the first layer.
int reference_layer(const PrintObject &object, const TipSite &tip, double depth_mm)
{
    const double z = neck_end(tip, depth_mm).z();
    if (object.layer_count() == 0 || z <= object.get_layer(0)->bottom_z())
        return -1;
    const auto it = std::lower_bound(object.layers().begin(), object.layers().end(), z,
                                     [](const Layer *layer, double z) { return layer->print_z < z; });
    return it == object.layers().end() ? -1 : int(it - object.layers().begin());
}

// The wall skip, true for a tip whose neck's end stands within the xy distance of the model: the seam clips support
// inside that band, so the neck would be cut while its ring survived, and the wall anchors that band as it does under
// the legacy tree. The skip reads the neck's end along the tip's axis, straight down where it has none, not the built
// neck: under a slope the head tilts along the underside's normal, and the neck check exempts a cut head whose tilted
// neck bottoms outside the band. A neck bottoming under the first layer stands by the pad and is kept, and so is an
// enforced tip: the enforcer asked for it there, and `clip_base` keeps its head out of the model alone. The bands of
// the layers under `tips` are offset up front, and the skip reads only those tips.
std::function<bool(const TipSite &)> wall_skip(const PrintObject &object, const std::vector<TipSite> &tips, const Params &params)
{
    const double     depth = neck_depth_mm(params);
    std::vector<int> wall_layers;
    for (const TipSite &tip : tips)
        if (const int l = reference_layer(object, tip, depth); l >= 0)
            wall_layers.push_back(l);
    std::sort(wall_layers.begin(), wall_layers.end());
    wall_layers.erase(std::unique(wall_layers.begin(), wall_layers.end()), wall_layers.end());
    std::vector<ExPolygons> wall_bands(wall_layers.size());
    tbb::parallel_for(tbb::blocked_range<size_t>(0, wall_layers.size()), [&](const tbb::blocked_range<size_t> &range) {
        for (size_t i = range.begin(); i < range.end(); ++ i)
            wall_bands[i] = offset_ex(object.get_layer(wall_layers[i])->lslices, scale_(params.xy_distance_mm));
    });
    return [&object, depth, wall_layers = std::move(wall_layers), wall_bands = std::move(wall_bands)](const TipSite &tip) {
        const int l = reference_layer(object, tip, depth);
        if (l < 0 || tip.enforced)
            return false;
        const ExPolygons &band = wall_bands[size_t(std::lower_bound(wall_layers.begin(), wall_layers.end(), l) - wall_layers.begin())];
        const Vec3d       end  = neck_end(tip, depth);
        const Point       at   = Point::new_scale(end.x(), end.y());
        if (std::none_of(band.begin(), band.end(), [&at](const ExPolygon &expoly) { return expoly.contains(at); }))
            return false;
        const Vec2d xy = unscale(tip.position);
        BOOST_LOG_TRIVIAL(debug) << "scaffold tip skipped at (" << xy.x() << ", " << xy.y() << ", " << tip.print_z << "): wall";
        return true;
    };
}

// The leading planned layers whose base is the pad.
size_t pad_layer_count(const std::vector<LayerHeightData> &layer_heights, const Params &params)
{
    size_t count = 0;
    for (const LayerHeightData &plan : layer_heights) {
        if (plan.print_z > params.pad_thickness_mm + EPSILON)
            break;
        ++ count;
    }
    return count;
}

// A tip's grade is the width of the disc it fuses to the model with, which the pin's radius halves: the planner's for
// every tip it placed, the size's for a baked point, and two support lines for an enforced one, which asks for none.
std::vector<double> tip_grades(const std::vector<TipSite> &nodes, const Params &params)
{
    std::vector<double> grades(nodes.size(), 2. * params.toolpath_width_mm);
    for (size_t i = 0; i < nodes.size(); ++ i)
        if (nodes[i].grade_mm > 0.)
            grades[i] = nodes[i].grade_mm;
    return grades;
}

// A post's disc, the tip's disc, at its point's xy.
ExPolygons post_disc(const sla::SupportPoint &pt)
{
    Polygon disc = make_circle(scale_(pt.head_front_radius), scale_(0.01));
    disc.translate(Point::new_scale(pt.pos.x(), pt.pos.y()));
    return ExPolygons{ ExPolygon(std::move(disc)) };
}

// Every pillar the builder stood on the pad's top widens toward it over its whole height by the taper, as a tree
// branch widens by the branch diameter angle: a sideways push on the model bends a pillar most at its foot, the more
// so the taller it stands. A foot reaches half a line past the midpoint to its nearest pillar or post, so neighbouring
// feet fuse along a line: a wider foot would take its neighbour in, and the base walls, laid along the union's outline
// only, would leave that neighbour nothing to stand on where the feet part higher up. A pillar the builder gave no
// base, one whose foot stands too near the model, runs down into the pad and stays straight, and so do the posts,
// the pillars from `built` on.
void taper_pillars(sla::SupportTreeBuilder &builder, size_t built, const Params &params)
{
    if (params.taper <= 0.)
        return;
    const std::vector<sla::Pillar> &pillars = builder.pillars();
    for (size_t pid = 0; pid < built; ++ pid) {
        const sla::Pillar &pillar = pillars[pid];
        if (std::abs(pillar.endpt.z() - builder.ground_level) >= EPSILON)
            continue;
        double foot = pillar.r + pillar.height * params.taper;
        for (size_t other = 0; other < pillars.size(); ++ other)
            if (other != pid)
                foot = std::min(foot, 0.5 * ((pillars[other].endpt - pillar.endpt).head<2>().norm() + params.toolpath_width_mm));
        if (foot > pillar.r)
            builder.add_pillar_base(long(pid), pillar.height, foot);
    }
}

// A build's slices: the cage through each planned layer's middle less every ringed or enforced head's own head over
// its rings, each routed head's own head through the tops of its ring layers, `first` the lowest, and every ringed or
// enforced head's own head, its neck, through the middles of the layers it reaches under its rings, `neck_first` the
// lowest. A head over its rings is its pin, which the builder sinks into the model over the tip by the penetration:
// where the model is thinner than that, the pin comes out on the model's top face with nothing under it, a floating
// piece that holds no ring, walls no hole and is no neck. A neck prints outside the model alone once `exempt`: an
// enforced tip's from the start, another head's once the neck check cut it and its lowest slice clears the band. A
// head whose lowest neck slice meets the band keeps no neck. The rings are indexed as the build's heads, whose ids
// index `tip_of`, then as the posts, whose rings are the post's disc on the layers a head's rings would take; a post
// has no neck.
struct Rings { size_t first = 0; std::vector<ExPolygons> slices; size_t neck_first = 0; std::vector<ExPolygons> neck;
               bool exempt = false; };

// What became of each tip. A head the builder kept carries its point's index as its id; one it gave up on
// lost the id and is found by the position it was built at; a point with no head was filtered out. A head
// whose rings, or exempt neck, would float is cut: `Unrouted` where the builder left it with no pillar and no
// bridge, which a side head whose ground pillar fails keeps, else `Neck`.
std::vector<ScaffoldTipResult> outcomes(const sla::SupportTreeBuilder &builder, const sla::SupportPoints &pts)
{
    std::vector<ScaffoldTipResult> tips(pts.size(), ScaffoldTipResult::Filtered);
    std::vector<size_t>            by_pos(pts.size());
    for (size_t i = 0; i < by_pos.size(); ++ i)
        by_pos[i] = i;
    const auto pos_less = [](const Vec3d &a, const Vec3d &b) { return std::lexicographical_compare(a.data(), a.data() + 3, b.data(), b.data() + 3); };
    const auto pos_of   = [&pts](size_t i) { return Vec3d(pts[i].pos.cast<double>()); };
    std::sort(by_pos.begin(), by_pos.end(), [&](size_t a, size_t b) { return pos_less(pos_of(a), pos_of(b)); });
    for (const sla::Head &head : builder.heads()) {
        if (head.is_valid()) {
            if (size_t(head.id) < tips.size())
                tips[head.id] = ScaffoldTipResult::Routed;
            continue;
        }
        auto it = std::lower_bound(by_pos.begin(), by_pos.end(), head.pos,
                                   [&](size_t i, const Vec3d &pos) { return pos_less(pos_of(i), pos); });
        for (; it != by_pos.end() && (pos_of(*it) - head.pos).norm() <= 1e-6; ++ it)
            if (tips[*it] == ScaffoldTipResult::Filtered) {
                tips[*it] = ScaffoldTipResult::Unrouted;
                break;
            }
    }
    return tips;
}

// A ring entry's point among those the build ran on, and why its tip goes when it is cut; a post is a `Neck`.
size_t point_of(const sla::SupportTreeBuilder &builder, const std::vector<size_t> &posts, size_t h)
{
    const std::vector<sla::Head> &heads = builder.heads();
    return h < heads.size() ? size_t(heads[h].id) : posts[h - heads.size()];
}
ScaffoldTipResult cut_reason(const sla::SupportTreeBuilder &builder, size_t h)
{
    if (h >= builder.heads().size())
        return ScaffoldTipResult::Neck;
    const sla::Head &head = builder.heads()[h];
    return head.pillar_id < 0 && head.bridge_id < 0 ? ScaffoldTipResult::Unrouted : ScaffoldTipResult::Neck;
}

// Takes the points of the heads `cut` names out of `left` and `index_of`, the rest in their order, and records in
// `tips` why each went.
void drop_cut(const sla::SupportTreeBuilder &builder, const std::vector<size_t> &posted, const std::vector<char> &cut,
              sla::SupportPoints &left, std::vector<size_t> &index_of, std::vector<ScaffoldTipResult> &tips)
{
    std::vector<char> gone(left.size(), 0);
    for (size_t h = 0; h < cut.size(); ++ h)
        if (cut[h]) {
            gone[point_of(builder, posted, h)] = 1;
            tips[index_of[point_of(builder, posted, h)]] = cut_reason(builder, h);
        }
    size_t next = 0;
    for (size_t k = 0; k < left.size(); ++ k)
        if (! gone[k]) {
            left[next]     = left[k];
            index_of[next] = index_of[k];
            ++ next;
        }
    left.resize(next);
    index_of.resize(next);
}

// Each tip's outcome after the last build. A post counts as routed, and a head the last build still cuts loses its
// rings, which stay out of the output.
void settle_tips(const sla::SupportTreeBuilder &builder, const sla::SupportPoints &left, const std::vector<size_t> &index_of,
                 const std::vector<size_t> &posted, const std::vector<char> &cut, std::vector<Rings> &rings, std::vector<ScaffoldTipResult> &tips)
{
    const std::vector<ScaffoldTipResult> last = outcomes(builder, left);
    for (size_t k = 0; k < last.size(); ++ k)
        tips[index_of[k]] = last[k];
    for (const size_t k : posted)
        tips[index_of[k]] = ScaffoldTipResult::Routed;
    for (size_t h = 0; h < cut.size(); ++ h)
        if (cut[h]) {
            tips[index_of[point_of(builder, posted, h)]] = cut_reason(builder, h);
            rings[h].slices.clear();
            rings[h].neck.clear();
        }
}

// Why a tip went, as the drop and island lines name it.
const char *drop_reason(ScaffoldTipResult tip)
{
    switch (tip) {
    case ScaffoldTipResult::Filtered: return "filtered";
    case ScaffoldTipResult::Unrouted: return "unrouted";
    case ScaffoldTipResult::Neck: return "neck";
    case ScaffoldTipResult::Wall: return "wall";
    default: return "other";
    }
}

// Each island the slice leaves unheld with its cause at debug level, a tipped island naming its one tip's result and a
// hung one its holders, and at info level the plan's islands by how they are held once routed: its own tip, hanging
// from the parts it met, or unheld. Debris needs no hold and is left out.
void log_islands(const Plan &plan, const PlanOutcome &after)
{
    if (plan.islands.empty())
        return;
    std::vector<char> unrouted(plan.islands.size(), 0);
    for (const size_t k : after.islands)
        unrouted[k] = 1;
    size_t tipped = 0, hung = 0, no_neck = 0;
    for (size_t k = 0; k < plan.islands.size(); ++ k) {
        const Island &island = plan.islands[k];
        const char   *cause  = nullptr;
        if (island.reason == IslandReason::NoNeck) {
            ++ no_neck;
            cause = "no neck";
        } else if (unrouted[k])
            cause = island.reason == IslandReason::Tip ? drop_reason(after.tips[island.holders.front()]) : "holders unrouted";
        else if (island.reason != IslandReason::Debris)
            ++ (island.reason == IslandReason::Tip ? tipped : hung);
        if (cause != nullptr)
            BOOST_LOG_TRIVIAL(debug) << "scaffold island at (" << island.birth.x() << ", " << island.birth.y() << ", " << island.birth.z()
                                     << ") unheld: " << cause;
    }
    BOOST_LOG_TRIVIAL(info) << "scaffold islands: " << tipped << " tipped, " << hung << " hung, " << no_neck + after.islands.size()
                            << " unheld (" << no_neck << " no neck, " << after.islands.size() << " not routed)";
}

// The floating pieces on each planned layer above the pad, with their extents.
using Adrift = std::vector<std::vector<std::pair<BoundingBox, const ExPolygon *>>>;

// What the steps of `draw` share: its inputs, each planned layer's middle in the mesh's frame, the pad's layer count
// and the tallest post.
struct DrawContext
{
    const std::vector<TipSite>         &nodes;
    const std::vector<LayerHeightData> &layer_heights;
    const std::vector<LayerClip>       &clips;
    const Params                       &params;
    const std::function<void()>        &throw_on_cancel;
    std::vector<float>                  middles;
    size_t                              pad_layers  = 0;
    double                              post_max_mm = 0.;

    size_t                  layers_under_tip(double mesh_z) const;
    std::vector<size_t>     add_posts(sla::SupportTreeBuilder &builder, const sla::SupportPoints &pts) const;
    void                    slice_build(const sla::SupportTreeBuilder &builder, const std::vector<size_t> &tip_of,
                                        const sla::SupportPoints &pts, const std::vector<size_t> &posts,
                                        std::vector<ExPolygons> &cage, std::vector<Rings> &rings) const;
    std::vector<ExPolygons> exempt_heads(const std::vector<Rings> &rings) const;
    std::vector<char>       find_stranded(std::vector<ExPolygons> &cage, const std::vector<Rings> &rings,
                                          const std::vector<ExPolygons> &exempt) const;
    std::vector<char>       check_necks(const sla::SupportTreeBuilder &builder, std::vector<ExPolygons> &cage,
                                        std::vector<Rings> &rings) const;
    std::pair<ExPolygons, ExPolygons> seam_areas(const ExPolygons &section, const ExPolygons &exempt, const ExPolygons &interface_,
                                                 size_t i) const;
    SupportAnalysis::Slab   seam_slab(size_t i, const ExPolygons &base_i, const ExPolygons &rings_i, double ground) const;
    Adrift                  adrift_pieces(const std::vector<SupportAnalysis::Slab> &slabs, const std::vector<std::vector<bool>> &floating) const;
    std::vector<std::vector<std::pair<size_t, size_t>>> ring_holds(const std::vector<Rings> &rings, const Adrift &adrift) const;
    std::vector<std::vector<char>> hole_walls(std::vector<ExPolygons> &cage, const std::vector<ExPolygons> &interface_,
                                              const std::vector<ExPolygons> &exempt, const Adrift &adrift,
                                              const std::vector<std::vector<char>> &ringed) const;
    void                    neck_adrift(const std::vector<Rings> &rings, const Adrift &adrift, const std::vector<std::vector<char>> &ringed,
                                        const std::vector<std::vector<char>> &walls, std::vector<char> &cut) const;
    void                    write_output(sla::SupportTreeBuilder &builder, std::vector<ExPolygons> &cage, const std::vector<Rings> &rings,
                                         uint32_t build_ms, Output &out) const;
};

// How many planned layers have their top at or under a tip at `mesh_z`. The rings of the tip's head or post print
// on the highest of them, the layer the tip's z tops, and the ones under it up to the interface count.
size_t DrawContext::layers_under_tip(double mesh_z) const
{
    const double tip_z = mesh_z + params.z_offset_mm;
    return size_t(std::upper_bound(layer_heights.begin(), layer_heights.end(), tip_z + EPSILON,
                                   [](double z, const LayerHeightData &plan) { return z < plan.print_z; }) -
                  layer_heights.begin());
}

// A point the build gave a valid head is left to the head. The result lists the points of `pts` that got a post,
// ascending.
std::vector<size_t> DrawContext::add_posts(sla::SupportTreeBuilder &builder, const sla::SupportPoints &pts) const
{
    std::vector<char>   headed(pts.size(), 0);
    std::vector<size_t> posted;
    for (const sla::Head &head : builder.heads())
        if (head.is_valid() && size_t(head.id) < headed.size())
            headed[size_t(head.id)] = 1;
    for (size_t k = 0; k < pts.size(); ++ k) {
        const Vec3d  tip = pts[k].pos.cast<double>();
        const double h   = tip.z() - builder.ground_level;
        if (headed[k] || h <= 0. || h > post_max_mm)
            continue;
        const size_t     above = layers_under_tip(tip.z());
        const size_t     clear = std::min(above, std::max(above - std::min(above, params.interface_layers), pad_layers));
        const ExPolygons disc  = post_disc(pts[k]);
        bool             outside = true;
        for (size_t i = 0; i < clear && outside; ++ i)
            outside = intersection_ex(disc, clips[i].band).empty();
        if (outside) {
            builder.add_pillar(Vec3d(tip.x(), tip.y(), builder.ground_level), h, double(pts[k].head_front_radius));
            posted.push_back(k);
        }
    }
    return posted;
}

void DrawContext::slice_build(const sla::SupportTreeBuilder &builder, const std::vector<size_t> &tip_of,
                              const sla::SupportPoints &pts, const std::vector<size_t> &posts,
                              std::vector<ExPolygons> &cage, std::vector<Rings> &rings) const
{
    const indexed_triangle_set &merged = builder.retrieve_mesh(sla::MeshType::Support);
    cage = merged.indices.empty() ? std::vector<ExPolygons>() : slice_mesh_ex(merged, middles, 0.f, throw_on_cancel);
    cage.resize(layer_heights.size());
    const std::vector<sla::Head> &heads = builder.heads();
    // Each ringed or enforced head's slices over its rings, as (the lowest layer, the slices).
    std::vector<std::pair<size_t, std::vector<ExPolygons>>> pins(heads.size());
    rings.assign(heads.size() + posts.size(), Rings());
    for (size_t j = 0; j < posts.size(); ++ j) {
        const size_t above = layers_under_tip(pts[posts[j]].pos.z());
        Rings       &r     = rings[heads.size() + j];
        r.first            = above - std::min(above, params.interface_layers);
        r.slices.assign(above - r.first, post_disc(pts[posts[j]]));
    }
    tbb::parallel_for(tbb::blocked_range<size_t>(0, heads.size()), [&](const tbb::blocked_range<size_t> &range) {
        for (size_t h = range.begin(); h < range.end(); ++ h) {
            if (! heads[h].is_valid())
                continue;
            const size_t above = layers_under_tip(heads[h].pos.z());
            const size_t count = std::min(above, params.interface_layers);
            rings[h].first     = above - count;
            const bool enforced = nodes[tip_of[size_t(heads[h].id)]].enforced;
            if (count == 0 && ! enforced)
                continue;
            const indexed_triangle_set head = sla::get_mesh(heads[h], 45);
            // The slicer wants its heights ascending.
            if (count > 0) {
                std::vector<float> zs;
                for (size_t i = rings[h].first; i < above; ++ i)
                    zs.push_back(float(layer_heights[i].print_z - params.z_offset_mm));
                rings[h].slices = slice_mesh_ex(head, zs, 0.f);
            }
            const auto [low, high] = std::minmax_element(head.vertices.begin(), head.vertices.end(),
                                                         [](const Vec3f &a, const Vec3f &b) { return a.z() < b.z(); });
            const float  bottom    = low->z();
            const size_t over      = size_t(std::lower_bound(middles.begin(), middles.end(), high->z()) - middles.begin());
            if (over > above)
                pins[h] = { above, slice_mesh_ex(head, std::vector<float>(middles.begin() + above, middles.begin() + over), 0.f) };
            rings[h].neck_first = std::min(size_t(std::lower_bound(middles.begin(), middles.end(), bottom) - middles.begin()),
                                           rings[h].first);
            if (rings[h].neck_first < rings[h].first)
                rings[h].neck = slice_mesh_ex(head, std::vector<float>(middles.begin() + rings[h].neck_first,
                                                                       middles.begin() + rings[h].first), 0.f);
            rings[h].exempt = enforced;
            if (! enforced && ! rings[h].neck.empty() && ! intersection_ex(rings[h].neck.front(), clips[rings[h].neck_first].band).empty())
                rings[h].neck.clear();
        }
    });
    std::vector<ExPolygons> over_rings(layer_heights.size());
    for (const auto &[lowest, slices] : pins)
        for (size_t k = 0; k < slices.size(); ++ k)
            append(over_rings[lowest + k], slices[k]);
    tbb::parallel_for(tbb::blocked_range<size_t>(0, layer_heights.size()), [&](const tbb::blocked_range<size_t> &range) {
        for (size_t i = range.begin(); i < range.end(); ++ i)
            cage[i] = over_rings[i].empty() ? union_ex(cage[i]) : diff_ex(union_ex(cage[i]), over_rings[i]);
    });
}

// The exempt heads' slices per planned layer: the enforced tips' heads and the heads the neck check exempted.
std::vector<ExPolygons> DrawContext::exempt_heads(const std::vector<Rings> &rings) const
{
    std::vector<ExPolygons> out(layer_heights.size());
    for (const Rings &r : rings)
        if (r.exempt)
            for (size_t k = 0; k < r.neck.size(); ++ k)
                append(out[r.neck_first + k], r.neck[k]);
    return out;
}

// The heads whose rings would print over nothing, and the cage holes whose walls would. What the seam lays on
// each planned layer above the pad, its areas with the holes `TreeSupport::fill_small_holes` fills filled, goes
// through the connectivity rule the floating pass applies, as the outlines the printed lines cover, since that
// pass reads `polygons_covered_by_width`; the pad's top is the ground every pillar stands on. The floating pass
// reads the scaffold as rooted on the plate alone, and with that flag the rule reads no model slab.
// - Base prints walls and no infill: `Params::base_cover` lays them through the call generate_toolpaths makes,
//   a loop on each contour and hole of the area closed inward by half a support line with its seam anchor, so
//   a wide area's inside prints nothing and a loop too short for its seam clip prints nothing either.
// - A ring prints through `make_perimeter_and_infill` with the interface flow: one loop on the ring shrunk by half
//   the interface spacing, laid through `extrusion_entities_append_loops` as here, with its infill inside, which
//   is counted as solid since its angle comes later from `normalize_interface_ids`. A part of a ring narrower
//   than one spacing prints nothing, and a ring that touches a neighbour only across such a neck prints apart
//   from it.
// A head is cut where one of its rings lies in a floating piece. A floating piece that holds no ring and is the
// wall of a hole in the cage's section stands over the unprinted inside of the section below: a gap among fused
// necks 2 mm across or wider keeps its hole while the same gap narrower on the layers around it is filled. That
// hole is filled too, which removes the wall and nothing a rooted piece stands on. An exempt head is cut where
// its own neck under its rings lies in a floating piece that holds no ring and walls no hole the fill removes: the
// head runs through the model or narrows under a line there, so the neck exempt from the band would print in
// mid-air.
std::vector<char> DrawContext::find_stranded(std::vector<ExPolygons> &cage, const std::vector<Rings> &rings,
                                             const std::vector<ExPolygons> &exempt) const
{
    std::vector<char> cut(rings.size(), 0);
    const size_t lowest = pad_layers;
    if (lowest >= layer_heights.size())
        return cut;
    std::vector<ExPolygons> interface_(layer_heights.size() - lowest);
    for (const Rings &r : rings)
        for (size_t k = 0; k < r.slices.size(); ++ k)
            if (r.first + k >= lowest)
                append(interface_[r.first + k - lowest], r.slices[k]);
    const double ground = layer_heights[lowest].print_z - layer_heights[lowest].height;
    std::vector<SupportAnalysis::Slab> slabs(interface_.size());
    tbb::parallel_for(tbb::blocked_range<size_t>(0, slabs.size()), [&](const tbb::blocked_range<size_t> &range) {
        for (size_t s = range.begin(); s < range.end(); ++ s) {
            const size_t i                 = lowest + s;
            const auto   [base_i, rings_i] = seam_areas(cage[i], exempt[i], interface_[s], i);
            slabs[s]                       = seam_slab(i, base_i, rings_i, ground);
        }
    });
    const std::vector<std::vector<bool>> floating = SupportAnalysis::floating_pieces(slabs, {}, true, 0.);

    const Adrift adrift = adrift_pieces(slabs, floating);
    if (std::all_of(adrift.begin(), adrift.end(), [](const auto &pieces) { return pieces.empty(); }))
        return cut;
    const std::vector<std::vector<std::pair<size_t, size_t>>> holds = ring_holds(rings, adrift);
    std::vector<std::vector<char>> ringed(slabs.size());
    for (size_t s = 0; s < slabs.size(); ++ s)
        ringed[s].assign(adrift[s].size(), 0);
    for (size_t h = 0; h < rings.size(); ++ h) {
        cut[h] = ! holds[h].empty();
        for (const auto &[s, j] : holds[h])
            ringed[s][j] = 1;
    }
    const std::vector<std::vector<char>> walls = hole_walls(cage, interface_, exempt, adrift, ringed);
    neck_adrift(rings, adrift, ringed, walls, cut);
    return cut;
}

// The heads whose rings or exempt neck would float, after the neck exemption. A head the check cuts for its neck
// whose lowest neck slice clears the band, one that kept its `neck`, takes the exemption: its neck prints outside the
// model alone, and the same build is checked again, which cuts it only where its rings or its own neck still float.
// A head the check never cuts keeps its neck clipped by the band. `find_stranded` erases the holes it fills from
// `cage`, so the second check runs on a copy of the cage as sliced.
std::vector<char> DrawContext::check_necks(const sla::SupportTreeBuilder &builder, std::vector<ExPolygons> &cage,
                                           std::vector<Rings> &rings) const
{
    std::vector<ExPolygons> sliced = cage;
    std::vector<char>       cut    = find_stranded(cage, rings, exempt_heads(rings));
    bool                    exempted = false;
    for (size_t h = 0; h < builder.heads().size(); ++ h)
        if (cut[h] && cut_reason(builder, h) == ScaffoldTipResult::Neck && ! rings[h].neck.empty() && ! rings[h].exempt) {
            rings[h].exempt = true;
            exempted        = true;
        }
    if (! exempted)
        return cut;
    cage = std::move(sliced);
    return find_stranded(cage, rings, exempt_heads(rings));
}

// The base and the rings the seam lays on the planned layer `i` above the pad, with their small holes filled.
std::pair<ExPolygons, ExPolygons> DrawContext::seam_areas(const ExPolygons &section, const ExPolygons &exempt,
                                                          const ExPolygons &interface_, size_t i) const
{
    ExPolygons rings_i = diff_ex(union_ex(interface_), clips[i].model);
    ExPolygons base_i  = diff_ex(clip_base(section, exempt, clips[i]), rings_i);
    for (ExPolygon &area : base_i)
        TreeSupport::fill_small_holes(area);
    for (ExPolygon &area : rings_i)
        TreeSupport::fill_small_holes(area);
    return std::make_pair(std::move(base_i), std::move(rings_i));
}

// The outlines the lines the seam prints on the planned layer `i` cover, as a slab over `ground`.
SupportAnalysis::Slab DrawContext::seam_slab(size_t i, const ExPolygons &base_i, const ExPolygons &rings_i, double ground) const
{
    const float      inset   = float(scale_(0.5 * Flow::rounded_rectangle_extrusion_spacing(
                                            float(params.interface_width_mm), float(layer_heights[i].height))));
    Polygons         printed = params.base_cover(base_i, layer_heights[i].height);
    const ExPolygons loop_area = offset_ex(rings_i, -inset, jtSquare);
    ExtrusionEntityCollection loops;
    extrusion_entities_append_loops(loops.entities, to_polygons(loop_area), erSupportMaterialInterface, 0.,
                                    float(params.interface_width_mm), float(layer_heights[i].height));
    loops.polygons_covered_by_width(printed, 0.f);
    polygons_append(printed, to_polygons(offset_ex(loop_area, -0.5f * float(scale_(params.interface_width_mm)))));
    SupportAnalysis::Slab slab;
    slab.bottom_z = layer_heights[i].print_z - layer_heights[i].height - ground;
    slab.print_z  = layer_heights[i].print_z - ground;
    slab.polygons = union_ex(printed);
    return slab;
}

Adrift DrawContext::adrift_pieces(const std::vector<SupportAnalysis::Slab> &slabs, const std::vector<std::vector<bool>> &floating) const
{
    Adrift adrift(slabs.size());
    for (size_t s = 0; s < slabs.size(); ++ s)
        for (size_t k = 0; k < slabs[s].polygons.size(); ++ k)
            if (floating[s][k])
                adrift[s].emplace_back(get_extents(slabs[s].polygons[k]), &slabs[s].polygons[k]);
    return adrift;
}

// The floating pieces each head's rings lie in, as (layer above the pad, index into `adrift`).
std::vector<std::vector<std::pair<size_t, size_t>>> DrawContext::ring_holds(const std::vector<Rings> &rings, const Adrift &adrift) const
{
    const size_t lowest = pad_layers;
    std::vector<std::vector<std::pair<size_t, size_t>>> holds(rings.size());
    tbb::parallel_for(tbb::blocked_range<size_t>(0, rings.size()), [&](const tbb::blocked_range<size_t> &range) {
        for (size_t h = range.begin(); h < range.end(); ++ h)
            for (size_t k = 0; k < rings[h].slices.size(); ++ k) {
                if (rings[h].first + k < lowest || rings[h].slices[k].empty())
                    continue;
                const size_t      s   = rings[h].first + k - lowest;
                const BoundingBox box = get_extents(rings[h].slices[k]);
                for (size_t j = 0; j < adrift[s].size(); ++ j)
                    if (adrift[s][j].first.overlap(box) &&
                        ! intersection_ex(rings[h].slices[k], ExPolygons{ *adrift[s][j].second }).empty())
                        holds[h].emplace_back(s, j);
            }
    });
    return holds;
}

// Which floating pieces that hold no ring wall a hole in the cage's section, whose holes this erases from `cage`.
// On a layer with an exempt head the band or the model can cut a filled hole out of the printed base again,
// and the wall around it stays in mid-air: there a piece counts as a hole's wall only when the base the seam
// prints after the fill borders no hole at the piece.
std::vector<std::vector<char>> DrawContext::hole_walls(std::vector<ExPolygons> &cage, const std::vector<ExPolygons> &interface_,
                                                       const std::vector<ExPolygons> &exempt, const Adrift &adrift,
                                                       const std::vector<std::vector<char>> &ringed) const
{
    const size_t lowest    = pad_layers;
    const float  half_line = float(scale_(0.5 * params.toolpath_width_mm));
    // A hole's wall covers from the hole's edge to a line into the section.
    const auto walls_hole = [half_line](const Polygon &hole, const BoundingBox &piece_box, const ExPolygon &piece) {
        BoundingBox box = get_extents(hole);
        box.offset(2 * half_line);
        if (! box.overlap(piece_box))
            return false;
        Polygon gap = hole;
        gap.make_counter_clockwise();
        return ! intersection_ex(offset_ex(ExPolygon(gap), 2.f * half_line), ExPolygons{ piece }).empty();
    };
    std::vector<std::vector<char>> walls(adrift.size());
    tbb::parallel_for(tbb::blocked_range<size_t>(0, adrift.size()), [&](const tbb::blocked_range<size_t> &range) {
        for (size_t s = range.begin(); s < range.end(); ++ s) {
            walls[s].assign(adrift[s].size(), 0);
            for (size_t j = 0; j < adrift[s].size(); ++ j) {
                if (ringed[s][j])
                    continue;
                for (ExPolygon &section : cage[lowest + s])
                    section.holes.erase(std::remove_if(section.holes.begin(), section.holes.end(), [&](const Polygon &hole) {
                        const bool wall = walls_hole(hole, adrift[s][j].first, *adrift[s][j].second);
                        walls[s][j] = walls[s][j] || wall;
                        return wall;
                    }), section.holes.end());
            }
            if (exempt[lowest + s].empty() || std::find(walls[s].begin(), walls[s].end(), char(1)) == walls[s].end())
                continue;
            const ExPolygons base = seam_areas(cage[lowest + s], exempt[lowest + s], interface_[s], lowest + s).first;
            for (size_t j = 0; j < adrift[s].size(); ++ j)
                for (const ExPolygon &area : base)
                    for (const Polygon &hole : area.holes)
                        if (walls[s][j] && walls_hole(hole, adrift[s][j].first, *adrift[s][j].second))
                            walls[s][j] = 0;
        }
    });
    return walls;
}

// The exempt heads whose own neck lies in a floating piece that holds no ring and walls no hole.
void DrawContext::neck_adrift(const std::vector<Rings> &rings, const Adrift &adrift, const std::vector<std::vector<char>> &ringed,
                              const std::vector<std::vector<char>> &walls, std::vector<char> &cut) const
{
    const size_t lowest = pad_layers;
    tbb::parallel_for(tbb::blocked_range<size_t>(0, rings.size()), [&](const tbb::blocked_range<size_t> &range) {
        for (size_t h = range.begin(); h < range.end(); ++ h)
            for (size_t k = 0; k < rings[h].neck.size() && rings[h].exempt && ! cut[h]; ++ k) {
                if (rings[h].neck_first + k < lowest || rings[h].neck[k].empty())
                    continue;
                const size_t      s   = rings[h].neck_first + k - lowest;
                const BoundingBox box = get_extents(rings[h].neck[k]);
                for (size_t j = 0; j < adrift[s].size() && ! cut[h]; ++ j)
                    if (! ringed[s][j] && ! walls[s][j] && adrift[s][j].first.overlap(box) &&
                        ! intersection_ex(rings[h].neck[k], ExPolygons{ *adrift[s][j].second }).empty())
                        cut[h] = 1;
            }
    });
}

void DrawContext::write_output(sla::SupportTreeBuilder &builder, std::vector<ExPolygons> &cage, const std::vector<Rings> &rings,
                               uint32_t build_ms, Output &out) const
{
    // The pad under the cage, on the layers it spans. A cage with no part routed gets no pad.
    const auto pad_start = std::chrono::steady_clock::now();
    indexed_triangle_set pad_mesh;
    if (! builder.retrieve_mesh(sla::MeshType::Support).indices.empty()) {
        sla::PadConfig pad;
        pad.wall_thickness_mm    = params.pad_thickness_mm;
        pad.wall_height_mm       = 0.;
        pad.brim_size_mm         = 1.6;
        pad.embed_object.enabled = false;
        builder.add_pad({}, pad);
        pad_mesh = builder.retrieve_mesh(sla::MeshType::Pad);
    }
    out.stage_ms.build = build_ms + ms_since(pad_start);

    // The last build's slices are the output: each planned layer is the cage's and the pad's section through the
    // layer's middle, each routed head's rings print as interface, and the exempt heads under their rings go out
    // apart for `clip_base`.
    const auto output_start = std::chrono::steady_clock::now();
    if (! pad_mesh.indices.empty()) {
        const float  pad_top = std::max_element(pad_mesh.vertices.begin(), pad_mesh.vertices.end(),
                                                [](const Vec3f &a, const Vec3f &b) { return a.z() < b.z(); })->z();
        const size_t spans   = size_t(std::upper_bound(middles.begin(), middles.end(), pad_top) - middles.begin());
        const std::vector<ExPolygons> pad_slices =
            slice_mesh_ex(pad_mesh, std::vector<float>(middles.begin(), middles.begin() + spans), 0.f, throw_on_cancel);
        for (size_t i = 0; i < pad_slices.size(); ++ i)
            if (! pad_slices[i].empty()) {
                append(cage[i], pad_slices[i]);
                cage[i] = union_ex(cage[i]);
            }
    }
    std::vector<ExPolygons> necks = exempt_heads(rings);
    for (size_t i = 0; i < out.layers.size(); ++ i) {
        out.layers[i].base         = std::move(cage[i]);
        out.layers[i].exempt_heads = std::move(necks[i]);
    }
    for (const Rings &r : rings)
        for (size_t k = 0; k < r.slices.size(); ++ k)
            append(out.layers[r.first + k].interface_, r.slices[k]);
    for (LayerAreas &layer : out.layers)
        if (! layer.interface_.empty()) {
            layer.interface_ = union_ex(layer.interface_);
            layer.base       = diff_ex(layer.base, layer.interface_);
        }
    out.stage_ms.slice += ms_since(output_start);
}

} // namespace

ExPolygons clip_base(const ExPolygons &base, const ExPolygons &exempt_heads, const LayerClip &clip)
{
    ExPolygons out = diff_ex(base, clip.band);
    if (! exempt_heads.empty())
        out = union_ex(out, diff_ex(exempt_heads, clip.model));
    return out;
}

Tips place_tips(const PrintObject &object, const std::vector<std::vector<SupportNode *>> &contacts, const Params &params,
                double threshold_rad, const std::vector<Polygons> &blockers)
{
    std::vector<TipSite> enforced;
    for (const std::vector<SupportNode *> &layer : contacts)
        for (const SupportNode *node : layer)
            if (node->is_pinned)
                enforced.push_back(site_of(*node));
    Tips tips;
    tips.mesh             = std::make_shared<const ObjectMesh>(object);
    const PlanInput input = plan_input(object, params, threshold_rad, blockers, tips.mesh.get());
    const auto start = std::chrono::steady_clock::now();
    tips.plan               = plan_tips(input, enforced);
    tips.island_joins_ms    = ms_since(start);
    tips.islands_under_held = tips.plan.islands_unheld;
    for (const Island &island : tips.plan.islands)
        if (island.reason == IslandReason::NoNeck)
            tips.bare_islands.push_back(island.birth);
    for (const PlannedTip &tip : tips.plan.tips)
        tips.sites.push_back(tip.site);
    for (const Island &island : tips.plan.islands)
        if (! island.rooted)
            for (const size_t tip : island.holders)
                tips.sites[tip].holds_island = true;
    // The planner keeps its tips out of the band on its lattice; the wall skip reads the band exactly.
    const std::function<bool(const TipSite &)> at_wall = wall_skip(object, tips.sites, params);
    tips.sites.erase(std::remove_if(tips.sites.begin(), tips.sites.end(), at_wall), tips.sites.end());
    merge_aliases(tips.sites);
    return tips;
}

ScaffoldPoint point_of(const PrintObject &object, const Params &params, const TipSite &site, double grade_mm)
{
    const Vec2d xy  = unscale(site.position);
    const Vec3d raw = object.trafo_centered().inverse() * Vec3d(xy.x(), xy.y(), site.print_z - params.z_offset_mm);
    return { raw.cast<float>(), grade_mm > 3. * params.toolpath_width_mm ? ScaffoldHeadSize::Heavy : ScaffoldHeadSize::Light,
             site.enforced };
}

Tips baked_tips(const PrintObject &object, const ScaffoldPoints &points, const Params &params, double threshold_rad)
{
    Tips tips;
    if (object.layer_count() == 0)
        return tips;
    // A point stands where a contact would: on the bottom of the object layer holding it, the first whose top is above
    // it, and `obj_layer_nr` one under that layer, the layer a contact under its overhang is filed on.
    std::vector<TipSite> sites;
    const Transform3d   &trafo = object.trafo_centered();
    for (size_t i = 0; i < points.size(); ++ i) {
        const Vec3d  p = trafo * points[i].pos.cast<double>();
        const double z = p.z() + params.z_offset_mm;
        auto         it = std::upper_bound(object.layers().begin(), object.layers().end(), z + EPSILON,
                                           [](double z, const Layer *layer) { return z < layer->print_z; });
        if (it == object.layers().end())
            -- it;
        TipSite site { Point::new_scale(p.x(), p.y()), (*it)->bottom_z(), int(it - object.layers().begin()) - 1 };
        site.enforced = points[i].enforced;
        site.grade_mm = (points[i].size == ScaffoldHeadSize::Heavy ? 4. : 2.) * params.toolpath_width_mm;
        site.source   = int(i);
        sites.push_back(site);
    }

    // A point keeps no axis, so each one not enforced leans its neck as the planner would lean it at that spot, its head
    // fitted on the object's mesh as the planner fits it. A list ignores support blockers as it ignores paint, so the
    // plan input reads none.
    tips.mesh             = std::make_shared<const ObjectMesh>(object);
    const PlanInput input = plan_input(object, params, threshold_rad, {}, tips.mesh.get());
    for (TipSite &site : sites)
        if (! site.enforced)
            site.axis = neck_axis(input, site);
    const std::function<bool(const TipSite &)> at_wall = wall_skip(object, sites, params);
    sites.erase(std::remove_if(sites.begin(), sites.end(), [&](const TipSite &site) {
                    if (! at_wall(site))
                        return false;
                    tips.wall_skipped.push_back(site.source);
                    return true;
                }), sites.end());
    // The islands the list leaves unheld are counted and named, and none is given a tip.
    const auto islands_start = std::chrono::steady_clock::now();
    tips.bare_islands        = unheld_islands(input, sites);
    tips.islands_under_held  = tips.bare_islands.size();
    tips.island_joins_ms     = ms_since(islands_start);
    merge_aliases(sites);
    tips.sites = std::move(sites);
    return tips;
}

PlanOutcome unheld_after_routing(const Plan &plan, const std::vector<TipSite> &sites, const std::vector<ScaffoldTipResult> &results)
{
    PlanOutcome out;
    out.tips.assign(plan.tips.size(), ScaffoldTipResult::Wall);
    for (size_t i = 0; i < plan.tips.size(); ++ i) {
        const Vec3d p = alias_point(plan.tips[i].site);
        for (size_t j = 0; j < sites.size() && out.tips[i] != ScaffoldTipResult::Routed; ++ j)
            if ((alias_point(sites[j]) - p).norm() <= sla::D_SP)
                out.tips[i] = results[j];
        if (plan.tips[i].need == TipNeed::Underside && out.tips[i] != ScaffoldTipResult::Routed)
            out.underside_mm2 += plan.tips[i].answered_mm2;
    }
    for (size_t k = 0; k < plan.islands.size(); ++ k) {
        const Island &island = plan.islands[k];
        if (! island.rooted && ! island.holders.empty() &&
            std::none_of(island.holders.begin(), island.holders.end(), [&out](size_t tip) { return out.tips[tip] == ScaffoldTipResult::Routed; }))
            out.islands.push_back(k);
    }
    return out;
}

bool baked_pose_valid(const Matrix3d &pose, const Matrix3d &linear)
{
    const Matrix3d M            = linear * pose.inverse();
    const bool     keeps_length = (M.transpose() * M - Matrix3d::Identity()).norm() < 1e-6;
    const bool     keeps_z      = (M * Vec3d::UnitZ() - Vec3d::UnitZ()).norm() < 1e-6;
    return keeps_length && keeps_z;
}

Output draw(const PrintObject &object, const Tips &chosen, const std::vector<LayerHeightData> &layer_heights,
            const std::vector<LayerClip> &clips, const Params &params, const std::function<void()> &throw_on_cancel)
{
    Output out;
    out.layers.resize(layer_heights.size());
    out.pad_layers                = pad_layer_count(layer_heights, params);
    out.counts.islands_slender    = chosen.plan.islands_slender;
    out.stage_ms.island_joins     = chosen.island_joins_ms;
    const std::vector<TipSite> &nodes = chosen.sites;

    std::vector<double> grades = tip_grades(nodes, params);
    // A head's id is its point's index, so the points keep the order of `nodes`. A tip holding an island makes its point
    // one that starts an island, whose head the builder retries along leaning axes.
    sla::SupportPoints points;
    for (size_t i = 0; i < nodes.size(); ++ i) {
        const Vec2d xy = unscale(nodes[i].position);
        points.emplace_back(float(xy.x()), float(xy.y()), float(nodes[i].print_z - params.z_offset_mm), float(grades[i] / 2.),
                            nodes[i].holds_island);
    }
    out.counts.tips_placed = points.size();

    // The object in the frame its slices are in, so an object lifted off the bed or sunk into it keeps its offset: the
    // plan's, or for a baked list one built here. The builder's mesh index points into it, so it lives for the whole call.
    const std::shared_ptr<const ObjectMesh> model = chosen.mesh ? chosen.mesh : std::make_shared<const ObjectMesh>(object);

    // The builder grounds its pillars at the mesh's lowest z less the elevation, so the elevation is that z: the
    // pillars stand on the pad on the bed however high the object floats. A lifted object's elevation is positive,
    // which lets a pillar drop straight under its underside rather than steering its foot out from under the model.
    sla::SupportTreeConfig cfg = tree_config(params);
    cfg.object_elevation_mm    = model->mesh.bounding_box().min.z();
    sla::SupportableMesh sm(model->aabb, sla::SupportPoints{}, cfg);
    // The copy the SupportableMesh holds drops any ground offset its source carried, so the offset goes on
    // the copy: pillars end on the pad's top face.
    sm.emesh.ground_level_offset(params.pad_thickness_mm);
    sla::JobController ctl;
    ctl.stopcondition = [&object] { return object.print()->canceled(); };
    ctl.cancelfn      = throw_on_cancel;
    // A fresh builder per run: the builder's move assignment carries neither its junctions nor its anchors. Each point's
    // tip hands its axis along, which the points left after a cut renumber.
    const auto build = [&sm, &ctl, &throw_on_cancel, &nodes](const sla::SupportPoints &pts, const std::vector<size_t> &index_of) {
        auto builder = std::make_unique<sla::SupportTreeBuilder>();
        builder->set_ctl(ctl);
        sm.pts = pts;
        sm.head_axes.clear();
        for (const size_t k : index_of)
            sm.head_axes.push_back(nodes[k].axis);
        if (sla::SupportTreeBuildsteps::execute(*builder, sm))
            throw_on_cancel();
        return builder;
    };

    // A tip too near the ground for a head, the underside of an object lifted off the bed among them, stands on a post:
    // a straight pillar as wide as the tip's disc from the pad's top up to the tip, which the cage and the pad take in
    // like any other pillar. A post is no taller than a head pointing straight down over a pillar base. Its disc stays
    // out of the band on every layer under its rings and on every pad layer, where the seam would cut the post or the
    // pad under it; the neck check reads the rings above the pad.
    const double post_max_mm = cfg.head_width_mm + 2. * cfg.head_back_radius_mm + 2. * cfg.head_front_radius_mm -
                               cfg.head_penetration_mm + cfg.base_height_mm;
    std::vector<float> middles;
    middles.reserve(layer_heights.size());
    for (const LayerHeightData &plan : layer_heights)
        middles.push_back(float(plan.print_z - 0.5 * plan.height - params.z_offset_mm));
    const DrawContext ctx{ nodes, layer_heights, clips, params, throw_on_cancel, std::move(middles), out.pad_layers, post_max_mm };

    // The builder runs until no head is cut, each time without the heads the last run cut, so no pillar or bridge
    // stands for a head that is gone; a run re-routes the neighbours of what it lost, which can strand another
    // ring. The points keep their order, so a head's id is its index among the points left. With no interface layer
    // there is no ring to strand, but an enforced head still prints its neck up to the tip outside the band, so the
    // check runs while any tip is enforced and one run stands only when none is. Each run builds and slices the whole
    // cage, 2.5 to 3 s on plate 3 of the corpus, whose runs cut 12 heads and then none. Plate 4 takes four runs, so
    // six leave two spare, and the heads the sixth still cuts are dropped without another run, their rings left out of
    // the output.
    constexpr size_t max_builds = 6;
    const bool       check      = params.interface_layers > 0 ||
                                  std::any_of(nodes.begin(), nodes.end(), [](const TipSite &tip) { return tip.enforced; });
    std::vector<ScaffoldTipResult> tips(points.size(), ScaffoldTipResult::Filtered);
    sla::SupportPoints             left = points;
    std::vector<size_t>            index_of(points.size());
    for (size_t i = 0; i < index_of.size(); ++ i)
        index_of[i] = i;
    std::unique_ptr<sla::SupportTreeBuilder> builder;
    std::vector<size_t>                      posted;
    std::vector<ExPolygons>                  cage;
    std::vector<Rings>                       rings;
    std::vector<char>                        cut;
    uint32_t                                 build_ms = 0;
    for (size_t run = 1;; ++ run) {
        const auto run_start = std::chrono::steady_clock::now();
        builder              = build(left, index_of);
        const size_t built   = builder->pillars().size();
        posted               = ctx.add_posts(*builder, left);
        taper_pillars(*builder, built, params);
        build_ms += ms_since(run_start);
        const auto slice_start = std::chrono::steady_clock::now();
        ctx.slice_build(*builder, index_of, left, posted, cage, rings);
        cut = check ? ctx.check_necks(*builder, cage, rings) : std::vector<char>(rings.size(), 0);
        out.stage_ms.slice += ms_since(slice_start);
        const size_t cut_count = size_t(std::count(cut.begin(), cut.end(), char(1)));
        BOOST_LOG_TRIVIAL(debug) << "scaffold build " << run << ": " << left.size() << " points, " << cut_count << " heads cut";
        if (cut_count == 0 || run == max_builds)
            break;
        drop_cut(*builder, posted, cut, left, index_of, tips);
    }
    settle_tips(*builder, left, index_of, posted, cut, rings, tips);
    out.counts.pillars_unbraced = builder->unbraced_pillars;
    for (size_t i = 0; i < tips.size(); ++ i) {
        if (tips[i] == ScaffoldTipResult::Routed) {
            ++ out.counts.tips_routed;
            continue;
        }
        ++ out.counts.tips_dropped;
        const Vec3f &p = points[i].pos;
        BOOST_LOG_TRIVIAL(debug) << "scaffold tip dropped at (" << p.x() << ", " << p.y() << ", " << p.z()
                                 << "): " << drop_reason(tips[i]);
    }
    // What routing left of the plan: an island whose holders all dropped prints with no tip holding it, and the
    // underside a dropped head answered hangs. A baked list has no plan, so its dropped points count only as dropped.
    const PlanOutcome after        = unheld_after_routing(chosen.plan, nodes, tips);
    out.counts.islands_under_held  = chosen.islands_under_held + after.islands.size();
    out.counts.underside_unmet_mm2 = chosen.plan.underside_unmet_mm2 + after.underside_mm2;
    out.bare_islands               = chosen.bare_islands;
    for (const size_t k : after.islands)
        out.bare_islands.push_back(chosen.plan.islands[k].birth);
    log_islands(chosen.plan, after);

    ctx.write_output(*builder, cage, rings, build_ms, out);
    out.results = std::move(tips);
    out.grades  = std::move(grades);
    return out;
}

} // namespace Slic3r::ScaffoldSupport
