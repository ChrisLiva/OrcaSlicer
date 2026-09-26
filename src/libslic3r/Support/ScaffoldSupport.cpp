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

TipSite site_of(const SupportNode &node) { return { node.position, node.print_z, node.obj_layer_nr, seed_id(node), &node, node.is_pinned }; }

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

// The island that owns the model piece over `tip`, on the overhang's own layer, one above the node's; npos where none
// does.
size_t island_of(const SupportAnalysis::IslandMap &map, const TipSite &tip)
{
    const size_t s = size_t(tip.obj_layer_nr + 1);
    if (tip.obj_layer_nr + 1 < 0 || s >= map.components.slab_range.size())
        return size_t(-1);
    for (size_t p = map.components.slab_range[s].first; p < map.components.slab_range[s].second; ++ p)
        if (map.components.pieces[p].polygon.contains(tip.position))
            return map.island_of_piece[p];
    return size_t(-1);
}

// The piece island `k` starts from: the one piece of its birth slab it owns, since that piece has nothing under it to
// share a set with.
const ExPolygon *birth_piece_of(const SupportAnalysis::IslandMap &map, size_t k)
{
    const size_t     birth_slab  = map.islands[k].birth_slab;
    const ExPolygon *birth_piece = nullptr;
    for (size_t p = map.components.slab_range[birth_slab].first; p < map.components.slab_range[birth_slab].second; ++ p)
        if (map.island_of_piece[p] == k)
            birth_piece = &map.components.pieces[p].polygon;
    return birth_piece;
}

// Greedy, lowest first: a tip counts where it stands a pillar diameter from every tip counted before it.
void count_spaced(std::vector<TipSite> &counted, const TipSite &tip, double min_spacing)
{
    if (std::all_of(counted.begin(), counted.end(), [&](const TipSite &c) { return tip.position.distance_to(c.position) >= min_spacing; }))
        counted.push_back(tip);
}

// Short of the floor: the dropped contacts come back lowest first, among equals the one furthest from the tips the
// island already has.
void restore_dropped(std::vector<TipSite> &kept, std::vector<TipSite> &candidates, std::vector<TipSite> &counted, size_t floor,
                     double min_spacing, std::vector<TipSite> &tips)
{
    std::sort(candidates.begin(), candidates.end(), [](const TipSite &a, const TipSite &b) { return a.print_z < b.print_z; });
    const auto nearest_kept = [&kept](const TipSite &tip) {
        double d = std::numeric_limits<double>::max();
        for (const TipSite &t : kept)
            d = std::min(d, tip.position.distance_to(t.position));
        return d;
    };
    size_t next = 0;
    while (counted.size() < floor && next < candidates.size()) {
        size_t tie_end = next;
        while (tie_end < candidates.size() && candidates[tie_end].print_z <= candidates[next].print_z + EPSILON)
            ++ tie_end;
        size_t best = next;
        double best_d = nearest_kept(candidates[next]);
        for (size_t i = next + 1; i < tie_end; ++ i)
            if (const double d = nearest_kept(candidates[i]); d > best_d) {
                best   = i;
                best_d = d;
            }
        std::swap(candidates[next], candidates[best]);
        const TipSite restored = candidates[next ++];
        kept.push_back(restored);
        tips.push_back(restored);
        count_spaced(counted, restored, min_spacing);
    }
}

// Holds island `k` with its floor of tips, `kept` the tips standing under it and `spare` its dropped contacts, and
// returns whether it stays short of the floor or of the tips its birth piece fits, whichever is fewer. Debris and an
// island hanging from a wall read false.
bool hold_island(const std::vector<SupportAnalysis::Slab> &slabs, const SupportAnalysis::IslandMap &map, size_t k,
                 std::vector<TipSite> &kept, std::vector<TipSite> &spare, std::vector<TipSite> &tips, double pillar_diameter_mm,
                 const std::function<bool(const TipSite &)> &at_wall)
{
    using namespace SupportAnalysis;
    const IslandJoin &join   = map.islands[k];
    const bool        joins  = join.join_slab < slabs.size();
    const Slab       &birth  = slabs[join.birth_slab];
    const IslandJoin &part   = map.islands[join.part];
    const double      top_z  = slabs[part.top_slab].print_z;
    const double      height = (joins ? slabs[join.join_slab].bottom_z : top_z) - birth.bottom_z;
    if (! joins && top_z - slabs[part.birth_slab].bottom_z <= 1. + EPSILON) {
        BOOST_LOG_TRIVIAL(debug) << "scaffold island skipped at " << birth.bottom_z << ": debris";
        return false;
    }
    const size_t     floor       = hold_floor(height);
    const ExPolygon *birth_piece = birth_piece_of(map, k);
    if (kept.empty() && spare.empty() && birth_piece != nullptr) {
        const TipSite seeded { inscribed_point(*birth_piece), birth.bottom_z, int(join.birth_slab) - 1 };
        if (! at_wall(seeded)) {
            const Vec2d xy = unscale(seeded.position);
            BOOST_LOG_TRIVIAL(debug) << "scaffold tip seeded at (" << xy.x() << ", " << xy.y() << ", " << seeded.print_z << ")";
            kept.push_back(seeded);
            tips.push_back(seeded);
        } else if (joins && height <= 1. + EPSILON) {
            BOOST_LOG_TRIVIAL(debug) << "scaffold island held at " << birth.bottom_z << ": wall";
            return false;
        }
    }
    const double         min_spacing = scale_(pillar_diameter_mm);
    std::vector<TipSite> counted;
    std::sort(kept.begin(), kept.end(), [](const TipSite &a, const TipSite &b) {
        return a.print_z != b.print_z ? a.print_z < b.print_z : a.seed < b.seed;
    });
    for (const TipSite &tip : kept)
        if (counted.size() < floor)
            count_spaced(counted, tip, min_spacing);
    restore_dropped(kept, spare, counted, floor, min_spacing, tips);
    // An island counts as under-held only where its birth piece has room for more tips than it got.
    return counted.size() < floor && (birth_piece == nullptr || counted.size() < tips_fitting(*birth_piece, pillar_diameter_mm));
}

// Holds each mid-air island of the model with its floor of tips a pillar diameter apart, and returns how many islands
// stay short of the floor or of the tips their birth piece fits, whichever is fewer. A tip belongs to the island that
// owns the model piece over it, on the overhang's own layer, one above the node's. An island that never joins measures
// to the top of the part it ends up in, and a whole part at most 1 mm tall is mesh debris: no floor, no tip, not
// counted, so a short leg merging into a taller floating part keeps its floor. An island with no tip and no dropped
// contact gets one tip seeded at the deepest point of its birth piece, at the piece's bottom, unless `at_wall` skips
// it; an island `at_wall` skips that joins within 1 mm hangs from that wall and is not counted. Short of the floor,
// the dropped contacts under it come back.
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
    std::vector<std::vector<TipSite>> kept(map.islands.size()), spare(map.islands.size());
    for (const TipSite &tip : tips)
        if (const size_t k = island_of(map, tip); k < kept.size())
            kept[k].push_back(tip);
    for (const TipSite &tip : dropped)
        if (const size_t k = island_of(map, tip); k < spare.size())
            spare[k].push_back(tip);

    size_t under_held = 0;
    for (size_t k = 0; k < map.islands.size(); ++ k)
        if (hold_island(slabs, map, k, kept[k], spare[k], tips, pillar_diameter_mm, at_wall))
            ++ under_held;
    return under_held;
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
// two kept tips are aliases, so the builder filters none. A kept tip is enforced when a tip merged into it was. The
// kept tips sit in a grid of cells the distance wide, so a tip reads the 27 cells around its own. `tips` keeps its
// order.
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
        const Vec2d     xy = unscale(tips[i].position);
        const Vec3d     p(xy.x(), xy.y(), tips[i].print_z);
        const AliasCell c{int64_t(std::floor(p.x() / sla::D_SP)), int64_t(std::floor(p.y() / sla::D_SP)),
                          int64_t(std::floor(p.z() / sla::D_SP))};
        if (const size_t into = alias_of(kept, p, c); into != size_t(-1)) {
            BOOST_LOG_TRIVIAL(debug) << "scaffold tip merged at (" << p.x() << ", " << p.y() << ", " << p.z() << ")";
            tips[into].enforced = tips[into].enforced || tips[i].enforced;
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

// One tip per contact. An interior tip is kept only where its overhang holds a disc as wide as the longest bridge:
// under a narrower overhang the tips on its rim already hold it.
std::vector<TipSite> contact_tips(const std::vector<std::vector<SupportNode *>> &contacts, double max_bridge_length_mm)
{
    std::vector<TipSite> nodes;
    for (const std::vector<SupportNode *> &layer : contacts)
        for (const SupportNode *node : layer)
            if (node->placement != SupportNode::Placement::Interior ||
                (! node->overhang.empty() && ! offset_ex(node->overhang, -scale_(max_bridge_length_mm / 2.)).empty()))
                nodes.push_back(site_of(*node));
    return nodes;
}

// The first object layer at or above the bottom of `tip`'s neck, `neck_depth_mm` under the tip, or -1 where the neck
// bottoms under the first layer.
int reference_layer(const PrintObject &object, const TipSite &tip, double neck_depth_mm)
{
    const double z = tip.print_z - neck_depth_mm;
    if (object.layer_count() == 0 || z <= object.get_layer(0)->bottom_z())
        return -1;
    const auto it = std::lower_bound(object.layers().begin(), object.layers().end(), z,
                                     [](const Layer *layer, double z) { return layer->print_z < z; });
    return it == object.layers().end() ? -1 : int(it - object.layers().begin());
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

// A tip's grade is the width of the disc it fuses to the model with: two support lines, or four where the model
// under it hangs off a neck at least eight lines wide. The pin is half the grade. A seeded tip starts a feature
// and keeps the small grade.
std::vector<double> tip_grades(const std::vector<TipSite> &nodes, const ModelSupportRisk::Field &risk, const Params &params)
{
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

// A build's slices: the cage through each planned layer's middle, each routed head's own head through the tops of
// its ring layers, `first` the lowest, and an enforced tip's own head through the middles of the layers it reaches
// under its rings, `neck_first` the lowest. The rings are indexed as the build's heads, whose ids index `tip_of`,
// then as the posts, whose rings are the post's disc on the layers a head's rings would take.
struct Rings { size_t first = 0; std::vector<ExPolygons> slices; size_t neck_first = 0; std::vector<ExPolygons> neck; };

// What became of each tip. A head the builder kept carries its point's index as its id; one it gave up on
// lost the id and is found by the position it was built at; a point with no head was filtered out. A head
// whose rings, or enforced neck, would float is cut: `Unrouted` where the builder left it with no pillar and no
// bridge, which a side head whose ground pillar fails keeps, else `Neck`.
enum class Tip : uint8_t { Filtered, Unrouted, Neck, Routed };
std::vector<Tip> outcomes(const sla::SupportTreeBuilder &builder, const sla::SupportPoints &pts)
{
    std::vector<Tip>    tips(pts.size(), Tip::Filtered);
    std::vector<size_t> by_pos(pts.size());
    for (size_t i = 0; i < by_pos.size(); ++ i)
        by_pos[i] = i;
    const auto pos_less = [](const Vec3d &a, const Vec3d &b) { return std::lexicographical_compare(a.data(), a.data() + 3, b.data(), b.data() + 3); };
    const auto pos_of   = [&pts](size_t i) { return Vec3d(pts[i].pos.cast<double>()); };
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
    return tips;
}

// A ring entry's point among those the build ran on, and why its tip goes when it is cut; a post is a `Neck`.
size_t point_of(const sla::SupportTreeBuilder &builder, const std::vector<size_t> &posts, size_t h)
{
    const std::vector<sla::Head> &heads = builder.heads();
    return h < heads.size() ? size_t(heads[h].id) : posts[h - heads.size()];
}
Tip cut_reason(const sla::SupportTreeBuilder &builder, size_t h)
{
    if (h >= builder.heads().size())
        return Tip::Neck;
    const sla::Head &head = builder.heads()[h];
    return head.pillar_id < 0 && head.bridge_id < 0 ? Tip::Unrouted : Tip::Neck;
}

// Takes the points of the heads `cut` names out of `left` and `index_of`, the rest in their order, and records in
// `tips` why each went.
void drop_cut(const sla::SupportTreeBuilder &builder, const std::vector<size_t> &posted, const std::vector<char> &cut,
              sla::SupportPoints &left, std::vector<size_t> &index_of, std::vector<Tip> &tips)
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
                 const std::vector<size_t> &posted, const std::vector<char> &cut, std::vector<Rings> &rings, std::vector<Tip> &tips)
{
    const std::vector<Tip> last = outcomes(builder, left);
    for (size_t k = 0; k < last.size(); ++ k)
        tips[index_of[k]] = last[k];
    for (const size_t k : posted)
        tips[index_of[k]] = Tip::Routed;
    for (size_t h = 0; h < cut.size(); ++ h)
        if (cut[h]) {
            tips[index_of[point_of(builder, posted, h)]] = cut_reason(builder, h);
            rings[h].slices.clear();
            rings[h].neck.clear();
        }
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
    rings.assign(heads.size() + posts.size(), Rings());
    for (size_t j = 0; j < posts.size(); ++ j) {
        const size_t above = layers_under_tip(pts[posts[j]].pos.z());
        Rings       &r     = rings[heads.size() + j];
        r.first            = above - std::min(above, params.interface_layers);
        r.slices.assign(above - r.first, post_disc(pts[posts[j]]));
    }
    tbb::parallel_for(tbb::blocked_range<size_t>(0, layer_heights.size()), [&](const tbb::blocked_range<size_t> &range) {
        for (size_t i = range.begin(); i < range.end(); ++ i)
            cage[i] = union_ex(cage[i]);
    });
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
            if (enforced) {
                const float bottom = std::min_element(head.vertices.begin(), head.vertices.end(),
                                                      [](const Vec3f &a, const Vec3f &b) { return a.z() < b.z(); })->z();
                rings[h].neck_first = std::min(size_t(std::lower_bound(middles.begin(), middles.end(), bottom) - middles.begin()),
                                               rings[h].first);
                if (rings[h].neck_first < rings[h].first)
                    rings[h].neck = slice_mesh_ex(head, std::vector<float>(middles.begin() + rings[h].neck_first,
                                                                           middles.begin() + rings[h].first), 0.f);
            }
        }
    });
}

// The heads exempt from the band's slices per planned layer, today the enforced heads.
std::vector<ExPolygons> DrawContext::exempt_heads(const std::vector<Rings> &rings) const
{
    std::vector<ExPolygons> out(layer_heights.size());
    for (const Rings &r : rings)
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
// hole is filled too, which removes the wall and nothing a rooted piece stands on. An enforced head is cut where
// its own neck under its rings lies in a floating piece that holds no ring and walls no hole the fill removes: the
// head runs through the model or narrows under a line there, so the neck the enforcer exempts from the band would
// print in mid-air.
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
// On a layer with an enforced head the band or the model can cut a filled hole out of the printed base again,
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

// The enforced heads whose own neck lies in a floating piece that holds no ring and walls no hole.
void DrawContext::neck_adrift(const std::vector<Rings> &rings, const Adrift &adrift, const std::vector<std::vector<char>> &ringed,
                              const std::vector<std::vector<char>> &walls, std::vector<char> &cut) const
{
    const size_t lowest = pad_layers;
    tbb::parallel_for(tbb::blocked_range<size_t>(0, rings.size()), [&](const tbb::blocked_range<size_t> &range) {
        for (size_t h = range.begin(); h < range.end(); ++ h)
            for (size_t k = 0; k < rings[h].neck.size() && ! cut[h]; ++ k) {
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
    // layer's middle, each routed head's rings print as interface, and the heads exempt from the band under their
    // rings go out apart for `clip_base`.
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

Tips choose_tips(const PrintObject &object, const std::vector<std::vector<SupportNode *>> &contacts,
                 const std::vector<SupportNode *> &dropped, const Params &params)
{
    std::vector<TipSite> nodes = contact_tips(contacts, params.max_bridge_length_mm);

    // A tip whose centre stands within the xy distance of the model at its neck's bottom is skipped, and so is a
    // dropped contact the hold floor could restore there: the seam clips support inside that band, so the neck
    // would be cut while its ring survived. Under a slope the head tilts along the underside's normal and the
    // slope recedes at least its rise by that depth, so the neck clears the band; beside a wall it does not, and
    // the wall anchors that band as it does under the legacy tree. A neck bottoming under the first layer stands
    // by the pad and is kept, and so is an enforced tip: the enforcer asked for it there, and `clip_base` keeps its
    // head out of the model alone.
    const double neck_depth_mm = head_width_mm + params.toolpath_width_mm;
    std::vector<TipSite> spare;
    for (const SupportNode *node : dropped)
        spare.push_back(site_of(*node));
    std::vector<int> wall_layers;
    for (const TipSite &tip : nodes)
        if (const int l = reference_layer(object, tip, neck_depth_mm); l >= 0)
            wall_layers.push_back(l);
    for (const TipSite &tip : spare)
        if (const int l = reference_layer(object, tip, neck_depth_mm); l >= 0)
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
        const int l = reference_layer(object, tip, neck_depth_mm);
        if (l < 0 || tip.enforced)
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

    Tips       tips;
    const auto islands_start = std::chrono::steady_clock::now();
    tips.islands_under_held  = restore_hold_floor(object, nodes, spare, params.pillar_diameter_mm, at_wall);
    tips.island_joins_ms     = ms_since(islands_start);
    // The floor counted tips a pillar diameter apart, so no alias counted there twice; the merge comes after it so
    // that a restored contact standing on a kept one merges too.
    merge_aliases(nodes);
    tips.sites = std::move(nodes);
    return tips;
}

Output draw(const PrintObject &object, const Tips &chosen, const std::vector<LayerHeightData> &layer_heights,
            const std::vector<LayerClip> &clips, const ModelSupportRisk::Field &risk, const Params &params,
            const std::function<void()> &throw_on_cancel)
{
    Output out;
    out.layers.resize(layer_heights.size());
    out.pad_layers                = pad_layer_count(layer_heights, params);
    out.counts.islands_under_held = chosen.islands_under_held;
    out.stage_ms.island_joins     = chosen.island_joins_ms;
    const std::vector<TipSite> &nodes = chosen.sites;

    const std::vector<double> grades = tip_grades(nodes, risk, params);
    // A head's id is its point's index, so the points keep the order of `nodes`.
    sla::SupportPoints points;
    for (size_t i = 0; i < nodes.size(); ++ i) {
        const Vec2d xy = unscale(nodes[i].position);
        points.emplace_back(float(xy.x()), float(xy.y()), float(nodes[i].print_z - params.z_offset_mm), float(grades[i] / 2.));
    }
    out.counts.tips_placed = points.size();

    // The object in the frame its slices are in: XY centred, the bed on z 0, so an object lifted off the bed or
    // sunk into it keeps its offset. The builder's mesh index points into it, so it lives for the whole call.
    TriangleMesh mesh = object.model_object()->raw_mesh();
    mesh.transform(object.trafo_centered());

    // The builder grounds its pillars at the mesh's lowest z less the elevation, so the elevation is that z: the
    // pillars stand on the pad on the bed however high the object floats. A lifted object's elevation is positive,
    // which lets a pillar drop straight under its underside rather than steering its foot out from under the model.
    sla::SupportTreeConfig cfg = tree_config(params);
    cfg.object_elevation_mm    = mesh.bounding_box().min.z();
    sla::SupportableMesh sm(mesh.its, sla::SupportPoints{}, cfg);
    // The copy the SupportableMesh holds drops any ground offset its source carried, so the offset goes on
    // the copy: pillars end on the pad's top face.
    sm.emesh.ground_level_offset(params.pad_thickness_mm);
    sla::JobController ctl;
    ctl.stopcondition = [&object] { return object.print()->canceled(); };
    ctl.cancelfn      = throw_on_cancel;
    // A fresh builder per run: the builder's move assignment carries neither its junctions nor its anchors.
    const auto build = [&sm, &ctl, &throw_on_cancel](const sla::SupportPoints &pts) {
        auto builder = std::make_unique<sla::SupportTreeBuilder>();
        builder->set_ctl(ctl);
        sm.pts = pts;
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
    // cage, 2.5 to 3 s on plate 3 of the corpus, whose runs cut about 90, 10, 3, 2 and then no head: six runs leave one
    // spare, and the heads the sixth still cuts are dropped without another run, their rings left out of the output.
    constexpr size_t max_builds = 6;
    const bool       check      = params.interface_layers > 0 ||
                                  std::any_of(nodes.begin(), nodes.end(), [](const TipSite &tip) { return tip.enforced; });
    std::vector<Tip>    tips(points.size(), Tip::Filtered);
    sla::SupportPoints  left = points;
    std::vector<size_t> index_of(points.size());
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
        builder              = build(left);
        const size_t built   = builder->pillars().size();
        posted               = ctx.add_posts(*builder, left);
        taper_pillars(*builder, built, params);
        build_ms += ms_since(run_start);
        const auto slice_start = std::chrono::steady_clock::now();
        ctx.slice_build(*builder, index_of, left, posted, cage, rings);
        cut = check ? ctx.find_stranded(cage, rings, ctx.exempt_heads(rings)) : std::vector<char>(rings.size(), 0);
        out.stage_ms.slice += ms_since(slice_start);
        const size_t cut_count = size_t(std::count(cut.begin(), cut.end(), char(1)));
        BOOST_LOG_TRIVIAL(debug) << "scaffold build " << run << ": " << left.size() << " points, " << cut_count << " heads cut";
        if (cut_count == 0 || run == max_builds)
            break;
        drop_cut(*builder, posted, cut, left, index_of, tips);
    }
    settle_tips(*builder, left, index_of, posted, cut, rings, tips);
    out.counts.pillars_unbraced = builder->unbraced_pillars;
    static constexpr const char *reason[] = { "filtered", "unrouted", "neck" };
    for (size_t i = 0; i < tips.size(); ++ i) {
        if (tips[i] == Tip::Routed) {
            ++ out.counts.tips_routed;
            continue;
        }
        ++ out.counts.tips_dropped;
        const Vec3f &p = points[i].pos;
        BOOST_LOG_TRIVIAL(debug) << "scaffold tip dropped at (" << p.x() << ", " << p.y() << ", " << p.z()
                                 << "): " << reason[size_t(tips[i])];
    }

    ctx.write_output(*builder, cage, rings, build_ms, out);
    return out;
}

} // namespace Slic3r::ScaffoldSupport
