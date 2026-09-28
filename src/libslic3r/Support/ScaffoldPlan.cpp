#include "ScaffoldPlan.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <optional>
#include <queue>
#include "ClipperUtils.hpp"
#include "Geometry/ConvexHull.hpp"
#include "Layer.hpp"
#include "Model.hpp"
#include "Print.hpp"
#include "DisjointSets.hpp"
#include <boost/log/trivial.hpp>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

namespace Slic3r::ScaffoldSupport {

namespace {

constexpr float  unreached        = std::numeric_limits<float>::infinity();
// A part reaching no farther than this from its anchors stands however thin: they hold all of it.
constexpr double stability_min_mm = 3.;
// How much higher a part whose stability search ended short is searched again: a face a little higher may take a tip.
constexpr double stability_retry_mm = 1.;
// A birth tip under an island that carries its part higher than this takes the heavy disc where the section fuses it.
constexpr double heavy_birth_mm   = 2.;
// An island that never merges and stands no taller than this is mesh debris, and one whose birth stands at a wall
// and merges within it hangs from that wall when the part it meets is held.
constexpr double debris_mm        = 1.;
// How far past what the layer below carries an underside may hang, in support lines: the first line past a held edge
// bonds its side to a held line and the second to a line that hangs by one, but a third would lie against a line that
// itself hangs by two.
constexpr double reach_lines      = 2.;
// How far from straight down a face may turn and still take a head, on the normal the builder aims the head along: the
// builder tilts a head at most 45 degrees from down, so a head on such a face meets it within 15 degrees of its normal.
constexpr double face_cap_deg     = 60.;
// The directions the bridge hold tries through a cell, 2.8 degrees apart: a line through the middle of a 9 mm chord
// stays within one cell of it.
constexpr int    bridge_directions = 64;

using Heap = std::priority_queue<std::pair<float, size_t>, std::vector<std::pair<float, size_t>>, std::greater<>>;

// One object layer's pieces on the lattice of cell side `cell` with its corner at `origin`, each cell labelled by the
// piece whose area holds its centre.
LayerGrid rasterize(const ExPolygons &pieces, const Point &origin, coord_t cell)
{
    LayerGrid grid;
    if (pieces.empty())
        return grid;
    const double      c    = double(cell);
    const double      ox   = double(origin.x()), oy = double(origin.y());
    const BoundingBox bbox = get_extents(pieces);
    grid.x0 = int(std::floor((double(bbox.min.x()) - ox) / c));
    grid.y0 = int(std::floor((double(bbox.min.y()) - oy) / c));
    grid.w  = int(std::floor((double(bbox.max.x()) - ox) / c)) - grid.x0 + 1;
    grid.h  = int(std::floor((double(bbox.max.y()) - oy) / c)) - grid.y0 + 1;
    grid.cells.assign(size_t(grid.w) * size_t(grid.h), 0);
    std::vector<std::vector<double>> crossings(size_t(grid.h));
    for (size_t k = 0; k < pieces.size() && k < 0xfffe; ++ k) {
        for (std::vector<double> &row : crossings)
            row.clear();
        // Where each edge crosses the rows whose centre it spans, half open so a vertex on a centre counts once.
        const auto add = [&](const Polygon &poly) {
            const Points &pts = poly.points;
            for (size_t i = 0; i < pts.size(); ++ i) {
                const Point &a = pts[i], &b = pts[(i + 1) % pts.size()];
                if (a.y() == b.y())
                    continue;
                const Point &lo = a.y() < b.y() ? a : b, &hi = a.y() < b.y() ? b : a;
                const int    j0 = std::max(grid.y0, int(std::ceil((double(lo.y()) - oy) / c - 0.5)));
                const int    j1 = std::min(grid.y0 + grid.h, int(std::ceil((double(hi.y()) - oy) / c - 0.5)));
                for (int j = j0; j < j1; ++ j) {
                    const double y = oy + (j + 0.5) * c;
                    crossings[size_t(j - grid.y0)].push_back(double(lo.x()) + (y - double(lo.y())) * double(hi.x() - lo.x()) / double(hi.y() - lo.y()));
                }
            }
        };
        add(pieces[k].contour);
        for (const Polygon &hole : pieces[k].holes)
            add(hole);
        for (int r = 0; r < grid.h; ++ r) {
            std::vector<double> &row = crossings[size_t(r)];
            std::sort(row.begin(), row.end());
            for (size_t i = 0; i + 1 < row.size(); i += 2) {
                const int i0 = std::max(grid.x0, int(std::ceil((row[i] - ox) / c - 0.5)));
                const int i1 = std::min(grid.x0 + grid.w, int(std::ceil((row[i + 1] - ox) / c - 0.5)));
                for (int x = i0; x < i1; ++ x)
                    grid.cells[size_t(r) * size_t(grid.w) + size_t(x - grid.x0)] = uint16_t(k + 1);
            }
        }
    }
    return grid;
}

// Per piece that starts a part (nothing below it): the slab where its part first overlaps another part, the slab
// count where it never does, and how far it stands free, to that slab or to its part's top. `carry_mm` is how far it
// carries its part by the elder rule: where parts meet, the one born lowest, a rooted part first, carries on, and every
// other part's eldest birth ends there; a birth that never ends carries to its part's top.
struct Births
{
    std::vector<size_t> merge_slab;
    std::vector<double> free_mm, carry_mm;
};

Births walk_births(const PlanInput &in)
{
    const std::vector<SupportAnalysis::Piece> &pieces = in.components.pieces;
    const size_t                               n      = in.slabs.size();
    Births births;
    births.merge_slab.assign(pieces.size(), n);
    births.free_mm.assign(pieces.size(), 0.);
    births.carry_mm.assign(pieces.size(), -1.);
    DisjointSets                     sets(pieces.size());
    std::vector<std::vector<size_t>> open(pieces.size());   // per root: its births that have not merged yet
    std::vector<size_t>              eldest(pieces.size()); // per root: its birth born lowest, the lower index on a tie
    std::vector<double>              top(pieces.size(), 0.);
    const auto older = [&pieces](size_t a, size_t b) {
        return pieces[a].bottom_z != pieces[b].bottom_z ? pieces[a].bottom_z < pieces[b].bottom_z : a < b;
    };
    for (size_t s = 0; s < n; ++ s) {
        const auto [first, last] = in.components.slab_range[s];
        for (size_t p = first; p < last; ++ p) {
            const SupportAnalysis::Piece &piece = pieces[p];
            if (piece.below.empty()) {
                open[p].push_back(p);
                eldest[p] = p;
                top[p]    = piece.print_z;
                continue;
            }
            std::vector<size_t> roots;
            for (size_t q : piece.below)
                if (const size_t r = sets.find(q); std::find(roots.begin(), roots.end(), r) == roots.end())
                    roots.push_back(r);
            const size_t root  = roots.front();
            size_t       elder = eldest[root];
            if (roots.size() > 1) {
                for (size_t r : roots) {
                    for (size_t birth : open[r])
                        births.merge_slab[birth] = s;
                    open[r].clear();
                    if (older(eldest[r], elder))
                        elder = eldest[r];
                }
                for (size_t r : roots)
                    if (eldest[r] != elder)
                        births.carry_mm[eldest[r]] = in.slabs[s].bottom_z - pieces[eldest[r]].bottom_z;
            }
            for (size_t i = 1; i < roots.size(); ++ i) {
                top[root] = std::max(top[root], top[roots[i]]);
                sets.join(root, roots[i]);
            }
            sets.join(root, p);
            eldest[root] = elder;
            top[root]    = std::max(top[root], piece.print_z);
        }
    }
    for (size_t p = 0; p < pieces.size(); ++ p)
        if (pieces[p].below.empty()) {
            const double part_top = top[sets.find(p)];
            births.free_mm[p] = (births.merge_slab[p] < n ? in.slabs[births.merge_slab[p]].bottom_z : part_top) - pieces[p].bottom_z;
            if (births.carry_mm[p] < 0.)
                births.carry_mm[p] = part_top - pieces[p].bottom_z;
        }
    return births;
}

// The narrowest width of a convex hull: the least, over its edges, of the farthest point from the edge.
double min_width_mm(const Polygon &hull)
{
    if (hull.size() < 3)
        return 0.;
    double best = std::numeric_limits<double>::max();
    for (size_t i = 0; i < hull.size(); ++ i) {
        const Vec2d a = unscale(hull[i]), b = unscale(hull[(i + 1) % hull.size()]);
        const Vec2d d = b - a;
        if (d.squaredNorm() <= 0.)
            continue;
        const Vec2d nrm = Vec2d(-d.y(), d.x()).normalized();
        double      far = 0.;
        for (const Point &p : hull)
            far = std::max(far, std::abs((unscale(p) - a).dot(nrm)));
        best = std::min(best, far);
    }
    return best == std::numeric_limits<double>::max() ? 0. : best;
}

// The walk over the object's layers, bottom-up: births first, then the underside runs, then each part's stability.
class Sweep
{
public:
    Sweep(const PlanInput &in, const NeedParams &need) : m_in(in), m_need(need), m_sets(in.components.pieces.size()),
        m_anchor_z(in.components.pieces.size(), -std::numeric_limits<double>::max()),
        m_rearm_z(in.components.pieces.size(), -std::numeric_limits<double>::max()),
        m_anchors(in.components.pieces.size()), m_waiting(in.components.pieces.size()), m_rooted(in.components.pieces.size(), 0),
        m_slender(in.components.pieces.size(), 0), m_flaps(0), m_reach_mm(reach_lines * in.toolpath_width_mm)
    {
        m_births = walk_births(in);
        const auto disc = [](double r) {
            std::vector<std::pair<int, int>> out;
            for (int dy = -int(r) - 1; dy <= int(r) + 1; ++ dy)
                for (int dx = -int(r) - 1; dx <= int(r) + 1; ++ dx)
                    if (double(dx * dx + dy * dy) <= r * r)
                        out.emplace_back(dx, dy);
            return out;
        };
        // Cells within the xy distance of a wall, under a small head, and within two lines, all by offset.
        m_wall_disc  = disc((in.xy_distance_mm + 0.71 * in.cell_mm) / in.cell_mm);
        m_head_disc  = disc(in.toolpath_width_mm / in.cell_mm);
        m_two_lines = disc(m_reach_mm / in.cell_mm);
        // A small head's own disc, pi w^2, in cells.
        m_flap_floor = size_t(std::lround(M_PI * in.toolpath_width_mm * in.toolpath_width_mm / (in.cell_mm * in.cell_mm)));
    }

    Plan run(const std::vector<TipSite> &enforced)
    {
        const size_t n = m_in.slabs.size();
        std::vector<std::vector<TipSite>> enforced_on(n);
        for (const TipSite &tip : enforced)
            if (const int l = tip.obj_layer_nr + 1; l >= 0 && size_t(l) < n)
                enforced_on[size_t(l)].push_back(tip);
            else
                m_plan.tips.push_back({ tip, TipNeed::Enforced });
        for (size_t l = 0; l < n; ++ l) {
            join_parts(l);
            layer(l, enforced_on[l]);
            stability(l);
        }
        prune_pending(std::numeric_limits<float>::max());
        // Lowest first, and the islands' holders follow their tips.
        std::vector<size_t> order(m_plan.tips.size());
        std::iota(order.begin(), order.end(), size_t(0));
        std::stable_sort(order.begin(), order.end(), [this](size_t i, size_t j) {
            const TipSite &a = m_plan.tips[i].site, &b = m_plan.tips[j].site;
            return a.print_z != b.print_z ? a.print_z < b.print_z :
                   a.position.x() != b.position.x() ? a.position.x() < b.position.x() : a.position.y() < b.position.y();
        });
        std::vector<PlannedTip> sorted;
        std::vector<size_t>     rank(order.size());
        sorted.reserve(order.size());
        for (size_t i = 0; i < order.size(); ++ i) {
            rank[order[i]] = i;
            sorted.push_back(m_plan.tips[order[i]]);
        }
        m_plan.tips = std::move(sorted);
        for (Island &island : m_plan.islands) {
            for (size_t &tip : island.holders)
                tip = rank[tip];
            std::sort(island.holders.begin(), island.holders.end());
        }
        size_t count[4] = {};
        for (const PlannedTip &tip : m_plan.tips)
            ++ count[size_t(tip.need)];
        BOOST_LOG_TRIVIAL(debug) << "scaffold plan: " << count[size_t(TipNeed::Birth)] << " birth, " << count[size_t(TipNeed::Underside)]
                                 << " underside, " << count[size_t(TipNeed::Stability)] << " stability, " << count[size_t(TipNeed::Enforced)]
                                 << " enforced tips; " << m_plan.islands_unheld << " islands unheld, " << m_plan.islands_slender
                                 << " slender, " << m_plan.underside_unmet_mm2 << " mm2 underside unmet";
        return std::move(m_plan);
    }

private:
    const PlanInput         &m_in;
    const NeedParams        &m_need;
    Births                   m_births;
    DisjointSets             m_sets;       // pieces joined as the walk climbs, for stability
    std::vector<double>      m_anchor_z;   // per root: its highest anchor, a tip or a birth
    std::vector<double>      m_rearm_z;    // per root: where its last stability search ended short
    struct Anchor { Vec3d at; size_t tip; };   // a tip's point and its index in the plan
    std::vector<std::vector<Anchor>> m_anchors;   // per root: its tips
    // Per root: the births that took no tip on the understanding that the part they merge into holds them, as indices
    // into the plan's islands. The merge settles them.
    std::vector<std::vector<size_t>> m_waiting;
    // Per island of the plan: its birth slab and piece, and the piece's deepest point.
    struct BirthPiece { size_t slab, piece; Point deepest; };
    std::vector<BirthPiece>  m_island_pieces;
    std::vector<char>        m_rooted, m_slender;
    std::vector<std::pair<int, int>> m_wall_disc;
    std::vector<std::pair<int, int>> m_head_disc;    // the cells under a small head, two support lines across
    std::vector<std::pair<int, int>> m_two_lines;    // the cells within two lines: the reach, the wall zone and the rim
    // Flaps: the underside hanging past the step, connected on a layer and across layers, with each one's cell count
    // at its root. A flap's count is its projected hanging area, since a cell faces down on one layer only.
    DisjointSets             m_flaps;
    std::vector<size_t>      m_flap_cells;
    size_t                   m_flap_floor = 0;   // a flap with fewer cells prints as it hangs
    const double             m_reach_mm;         // how far an underside may hang past the step, `reach_lines` lines
    Plan                     m_plan;
    // The layer being walked and the one under it: their window on the lattice, each cell's run and its flap, -1 off
    // any flap.
    LayerGrid                m_prev_grid;
    std::vector<float>       m_prev_run;
    std::vector<int>         m_prev_flap;
    const LayerGrid         *m_grid = nullptr;
    std::vector<float>       m_dist, m_tip_dist;
    float                    m_step = 0.f;   // the self-support step of the layer being walked
    // Underside cells that hang past the reach with no head answering them yet: lattice x and y, bottom z and run.
    struct Pending { int x, y; float z, run; };
    std::vector<Pending>     m_pending;
    std::vector<size_t>      m_heads;        // the cells under the heads placed on the layer being walked
    std::vector<std::pair<int, int>> m_layer_heads;   // those heads' own cells
    // Per cell of the layer being walked, what its underside walk has read, -1 before it has: whether it lies in the
    // wall zone, on the rim, and whether a head may stand there.
    std::vector<int8_t>      m_zone, m_rim, m_site_ok;
    size_t                   m_wall_bottom = 0, m_wall_top = 0;   // the layers a wall cell of the layer walked stays empty on

    const std::vector<SupportAnalysis::Piece> &pieces() const { return m_in.components.pieces; }
    Point centre(int x, int y) const { return m_in.origin + Point(coord_t((x + 0.5) * m_in.cell), coord_t((y + 0.5) * m_in.cell)); }
    std::pair<int, int> cell_of(const Point &p) const
    {
        return { int(std::floor(double(p.x() - m_in.origin.x()) / double(m_in.cell))),
                 int(std::floor(double(p.y() - m_in.origin.y()) / double(m_in.cell))) };
    }
    float run_of(size_t i) const { return std::max(0.f, m_dist[i] - m_step); }
    // The run a cell holds within: the reach, and half a cell for where the lattice samples it.
    float reach_run() const { return float(m_reach_mm + 0.5 * m_in.cell_mm); }

    void join_parts(size_t l)
    {
        const auto [first, last] = m_in.components.slab_range[l];
        for (size_t p = first; p < last; ++ p) {
            const SupportAnalysis::Piece &piece = pieces()[p];
            if (piece.below.empty()) {
                m_rooted[p]   = piece.bottom_z <= m_in.slabs.front().bottom_z + EPSILON;
                // A birth anchors its own part, whether a tip holds it or it prints without one.
                m_anchor_z[p] = piece.bottom_z;
                continue;
            }
            std::vector<size_t> roots;
            for (size_t q : piece.below)
                if (const size_t r = m_sets.find(q); std::find(roots.begin(), roots.end(), r) == roots.end())
                    roots.push_back(r);
            if (roots.size() > 1)
                settle(roots);
            const size_t root = roots.front();
            for (size_t i = 1; i < roots.size(); ++ i) {
                const size_t r = roots[i];
                m_anchor_z[root] = std::max(m_anchor_z[root], m_anchor_z[r]);
                m_rearm_z[root]  = std::max(m_rearm_z[root], m_rearm_z[r]);
                append(m_anchors[root], std::move(m_anchors[r]));
                m_rooted[root]  = m_rooted[root] || m_rooted[r];
                m_slender[root] = m_slender[root] || m_slender[r];
                m_sets.join(root, r);
            }
            m_sets.join(root, p);
        }
    }

    // Parts meeting: a birth that waited for this merge is held when a tip or the bed already holds one of the parts, and
    // hangs from every tip on the parts held. When none is, the first waiting birth a tip can stand under takes its tip
    // after all, which holds the rest; one that no tip can reach is counted.
    void settle(const std::vector<size_t> &roots)
    {
        std::vector<size_t> holders;
        bool                rooted = false;
        for (size_t r : roots) {
            rooted = rooted || m_rooted[r];
            for (const Anchor &anchor : m_anchors[r])
                holders.push_back(anchor.tip);
        }
        bool placed = rooted || ! holders.empty();
        for (size_t r : roots) {
            for (const size_t k : m_waiting[r]) {
                if (placed)
                    break;
                const BirthPiece &birth = m_island_pieces[k];
                if (const std::optional<Point> spot = birth_spot(birth.slab, birth.piece, birth.deepest)) {
                    BOOST_LOG_TRIVIAL(debug) << "scaffold island at " << pieces()[birth.piece].bottom_z << " joins nothing held: tip";
                    holders = { place(site_at(birth.slab, *spot, birth_grade(birth.slab, birth.piece, *spot)), TipNeed::Birth, nullptr) };
                    m_plan.islands[k].reason = IslandReason::Tip;
                    placed = true;
                }
            }
        }
        for (size_t r : roots) {
            for (const size_t k : m_waiting[r]) {
                Island &island = m_plan.islands[k];
                if (! placed) {
                    island.reason = IslandReason::NoNeck;
                    ++ m_plan.islands_unheld;
                    continue;
                }
                island.holders = holders;
                island.rooted  = rooted;
            }
            m_waiting[r].clear();
        }
    }

    // Whether a tip may stand at `p` on object layer `l`: not under a blocker, and its neck, one head width and one
    // toolpath width under it, not within the xy distance of the model, as the wall skip reads it.
    bool eligible(size_t l, const Point &p) const
    {
        const std::pair<int, int> cell = cell_of(p);
        const int                 x = cell.first, y = cell.second;
        if (m_in.blocked[l].at(x, y) != 0)
            return false;
        const double z = m_in.slabs[l].bottom_z - m_in.neck_depth_mm;
        if (z <= m_in.slabs.front().bottom_z)
            return true;
        const auto it = std::lower_bound(m_in.slabs.begin(), m_in.slabs.end(), z,
                                         [](const SupportAnalysis::Slab &slab, double z) { return slab.print_z < z; });
        if (it == m_in.slabs.end())
            return true;
        const LayerGrid &wall = m_in.material[size_t(it - m_in.slabs.begin())];
        if (std::any_of(m_wall_disc.begin(), m_wall_disc.end(), [&](const std::pair<int, int> &d) { return wall.at(x + d.first, y + d.second) != 0; }))
            return false;
        // The lattice places `p` and the wall only to within a cell each, and a tip on the model's edge, as every
        // stability tip is, stands where that decides: the wall skip reads the band itself, so the planner does too.
        const ExPolygons &band = m_in.wall_band[size_t(it - m_in.slabs.begin())];
        return std::none_of(band.begin(), band.end(), [&p](const ExPolygon &expoly) { return expoly.contains(p); });
    }

    // Whether the builder aims a head at `p` on the bottom of slab `l` within the face cap of straight down: it aims the
    // head along `sla::normals` at the point, the faces' normal averaged within the head's radius, one toolpath width.
    bool faces_down(size_t l, const Point &p) const
    {
        if (m_in.mesh == nullptr)
            return true;
        const Vec2d   xy = unscale(p);
        sla::PointSet at(1, 3);
        at.row(0) = Vec3d(xy.x(), xy.y(), m_in.slabs[l].bottom_z - m_in.z_offset_mm);
        const sla::PointSet normal = sla::normals(at, m_in.mesh->aabb, m_in.toolpath_width_mm);
        return normal.rows() == 0 || -normal(0, 2) >= std::cos(face_cap_deg * M_PI / 180.);
    }

    // The root of the piece of slab `l` holding `p`, or npos.
    size_t root_at(size_t l, const Point &p)
    {
        const auto [x, y] = cell_of(p);
        if (const uint16_t label = m_in.material[l].at(x, y); label != 0)
            return m_sets.find(m_in.components.slab_range[l].first + label - 1);
        for (size_t q = m_in.components.slab_range[l].first; q < m_in.components.slab_range[l].second; ++ q)
            if (pieces()[q].polygon.contains(p))
                return m_sets.find(q);
        return size_t(-1);
    }

    // Returns the tip's index in the plan.
    size_t place(const TipSite &site, TipNeed need, Heap *heap)
    {
        const size_t index = m_plan.tips.size();
        m_plan.tips.push_back({ site, need });
        const size_t l      = size_t(site.obj_layer_nr + 1);
        const Vec2d  xy     = unscale(site.position);
        const size_t root   = root_at(l, site.position);
        if (root != size_t(-1)) {
            m_anchors[root].push_back({ Vec3d(xy.x(), xy.y(), site.print_z), index });
            m_anchor_z[root] = std::max(m_anchor_z[root], site.print_z);
        }
        if (heap == nullptr)
            return index;
        // Its head anchors the cells under its disc on the layer being walked.
        const double radius = 0.5 * (site.grade_mm > 0. ? site.grade_mm : 2. * m_in.toolpath_width_mm) / m_in.cell_mm;
        const auto [cx, cy] = cell_of(site.position);
        m_layer_heads.emplace_back(cx, cy);
        const LayerGrid &g  = *m_grid;
        for (int dy = -int(radius) - 1; dy <= int(radius) + 1; ++ dy)
            for (int dx = -int(radius) - 1; dx <= int(radius) + 1; ++ dx)
                if (double(dx * dx + dy * dy) <= std::max(radius * radius, 0.5)) {
                    const int x = cx + dx, y = cy + dy;
                    if (g.at(x, y) == 0)
                        continue;
                    const size_t i = size_t(y - g.y0) * size_t(g.w) + size_t(x - g.x0);
                    m_heads.push_back(i);
                    if (m_dist[i] > 0.f) {
                        m_dist[i] = 0.f;
                        heap->emplace(0.f, i);
                    }
                }
        return index;
    }

    TipSite site_at(size_t l, const Point &p, double grade_mm) const
    {
        TipSite site { p, m_in.slabs[l].bottom_z, int(l) - 1 };
        site.grade_mm = grade_mm;
        return site;
    }

    // The disc a birth tip at `p` under piece `piece` of slab `l` fuses with. The heavy one, four lines across, goes
    // where the island carries its part higher than `heavy_birth_mm` and the model the pin reaches, the slabs within one
    // toolpath width over the tip, fills at least twice as many cells of the heavy disc as of the small one; the small
    // one, two lines across, goes everywhere else, where the heavy disc would add scar and no hold.
    double birth_grade(size_t l, size_t piece, const Point &p) const
    {
        const double w = m_in.toolpath_width_mm;
        if (m_births.carry_mm[piece] <= heavy_birth_mm)
            return 2. * w;
        const auto [cx, cy] = cell_of(p);
        const int  r        = int(std::ceil(2. * w / m_in.cell_mm)) + 1;
        size_t     small = 0, heavy = 0;
        for (int dy = -r; dy <= r; ++ dy)
            for (int dx = -r; dx <= r; ++ dx) {
                const double d = (centre(cx + dx, cy + dy) - p).cast<double>().norm() * SCALING_FACTOR;
                if (d > 2. * w)
                    continue;
                bool filled = false;
                for (size_t k = l; k < m_in.slabs.size() && m_in.slabs[k].bottom_z < m_in.slabs[l].bottom_z + w && ! filled; ++ k)
                    filled = m_in.material[k].at(cx + dx, cy + dy) != 0;
                heavy += filled;
                small += filled && d <= w;
            }
        return heavy >= 2 * std::max<size_t>(small, 1) ? 4. * w : 2. * w;
    }

    // Dijkstra through the layer's material, from whatever the heap holds, over the 16 moves of a king and a knight: a
    // path overstates a straight distance by at most 3 %, where the king's 8 alone overstate it by 8 %. A knight's move
    // needs material on the two cells it passes between.
    void spread(Heap &heap)
    {
        static constexpr int moves[16][2] = { { 1, 0 }, { -1, 0 }, { 0, 1 }, { 0, -1 }, { 1, 1 }, { 1, -1 }, { -1, 1 }, { -1, -1 },
                                              { 2, 1 }, { 2, -1 }, { -2, 1 }, { -2, -1 }, { 1, 2 }, { 1, -2 }, { -1, 2 }, { -1, -2 } };
        const LayerGrid &g = *m_grid;
        const float      c = float(m_in.cell_mm);
        const float      cost[3] = { c, c * float(M_SQRT2), c * std::sqrt(5.f) };
        const auto       solid = [&g](int x, int y) { return x >= 0 && y >= 0 && x < g.w && y < g.h && g.cells[size_t(y) * size_t(g.w) + size_t(x)] != 0; };
        while (! heap.empty()) {
            const auto [d, i] = heap.top();
            heap.pop();
            if (d > m_dist[i])
                continue;
            const int x = int(i % size_t(g.w)), y = int(i / size_t(g.w));
            for (const auto &m : moves) {
                const int nx = x + m[0], ny = y + m[1];
                if (! solid(nx, ny))
                    continue;
                const bool knight = std::abs(m[0]) + std::abs(m[1]) == 3;
                if (knight && ! (solid(x + m[0] / 2, y + m[1] / 2) && solid(x + (m[0] - m[0] / 2), y + (m[1] - m[1] / 2))))
                    continue;
                const float  nd = d + cost[knight ? 2 : (m[0] != 0 && m[1] != 0 ? 1 : 0)];
                const size_t j  = size_t(ny) * size_t(g.w) + size_t(nx);
                if (nd < m_dist[j]) {
                    m_dist[j] = nd;
                    heap.emplace(nd, j);
                }
            }
        }
    }

    void layer(size_t l, const std::vector<TipSite> &enforced)
    {
        const LayerGrid &g = m_in.material[l];
        m_grid = &g;
        if (g.w == 0) {
            for (const TipSite &tip : enforced)
                place(tip, TipNeed::Enforced, nullptr);
            m_prev_grid = g;
            m_prev_run.clear();
            m_prev_flap.clear();
            return;
        }
        const LayerGrid *below  = l > 0 ? &m_in.material[l - 1] : nullptr;
        const double     a      = m_in.self_support_mm[l];
        const double     a_cells = a / m_in.cell_mm;
        const size_t     cells  = g.cells.size();
        m_step = float(a);
        // 1: material right under it, 2: within the self-support step of the layer below, 3: underside.
        std::vector<uint8_t> kind(cells, 0);
        m_dist.assign(cells, unreached);
        Heap heap;
        const auto prev_run = [this](int x, int y) {
            const LayerGrid &p = m_prev_grid;
            if (x < p.x0 || y < p.y0 || x >= p.x0 + p.w || y >= p.y0 + p.h || m_prev_run.empty())
                return 0.f;
            return m_prev_run[size_t(y - p.y0) * size_t(p.w) + size_t(x - p.x0)];
        };
        const bool rooted_layer = m_in.slabs[l].bottom_z <= m_in.slabs.front().bottom_z + EPSILON;
        for (int y = 0; y < g.h; ++ y)
            for (int x = 0; x < g.w; ++ x) {
                const size_t i = size_t(y) * size_t(g.w) + size_t(x);
                if (g.cells[i] == 0)
                    continue;
                const int gx = g.x0 + x, gy = g.y0 + y;
                if (rooted_layer || (below != nullptr && below->at(gx, gy) != 0)) {
                    kind[i]   = 1;
                    m_dist[i] = rooted_layer ? 0.f : prev_run(gx, gy);
                    heap.emplace(m_dist[i], i);
                    continue;
                }
                kind[i] = 3;
                if (below != nullptr)
                    for (int dy = -int(a_cells); dy <= int(a_cells) && kind[i] == 3; ++ dy)
                        for (int dx = -int(a_cells); dx <= int(a_cells); ++ dx)
                            if (double(dx * dx + dy * dy) <= a_cells * a_cells && below->at(gx + dx, gy + dy) != 0) {
                                kind[i] = 2;
                                break;
                            }
            }

        m_heads.clear();
        m_layer_heads.clear();
        std::vector<size_t> enforced_tips;
        for (const TipSite &tip : enforced)
            enforced_tips.push_back(place(tip, TipNeed::Enforced, &heap));
        births(l, enforced_tips, heap);
        spread(heap);
        spread_tips();
        std::vector<int> flap = flaps(kind);
        underside(l, kind, flap, heap);

        // What the layer above hangs from. Material within the reach of a head on this layer spans to that head as a
        // line bridges to its anchor, so it holds what grows on it as the layer below holds; the rest passes its run
        // on, a line bridged between held ends included.
        const float reach = reach_run();
        m_prev_grid = g;
        m_prev_run.assign(cells, 0.f);
        for (size_t i = 0; i < cells; ++ i)
            if (kind[i] != 0 && m_tip_dist[i] > reach)
                m_prev_run[i] = m_dist[i] == unreached ? 0.f : run_of(i);
        m_prev_flap = std::move(flap);
    }

    // The flaps of the layer being walked, each cell's flap key or -1: the 8-connected pieces of its cells that the
    // layer below does not stand under and that hang past the step, each continuing a flap of the layer below with a
    // hanging cell within two cells, one toolpath width, of one of its own. Read after the births and before any
    // underside head on the layer.
    std::vector<int> flaps(const std::vector<uint8_t> &kind)
    {
        const LayerGrid &g     = *m_grid;
        const size_t     cells = g.cells.size();
        std::vector<int> flap(cells, -1);
        const auto hangs = [&](size_t i) { return kind[i] >= 2 && run_of(i) > 0.f; };
        const auto prev  = [this](int x, int y) {
            const LayerGrid &p = m_prev_grid;
            if (m_prev_flap.empty() || x < p.x0 || y < p.y0 || x >= p.x0 + p.w || y >= p.y0 + p.h)
                return -1;
            return m_prev_flap[size_t(y - p.y0) * size_t(p.w) + size_t(x - p.x0)];
        };
        std::vector<size_t> stack;
        for (size_t seed = 0; seed < cells; ++ seed) {
            if (flap[seed] >= 0 || ! hangs(seed))
                continue;
            const int key = int(m_flaps.parent.size());
            m_flaps.parent.push_back(size_t(key));
            m_flap_cells.push_back(0);
            std::vector<int> continued;
            flap[seed] = key;
            stack.assign(1, seed);
            while (! stack.empty()) {
                const size_t i = stack.back();
                stack.pop_back();
                ++ m_flap_cells[size_t(key)];
                const int x = int(i % size_t(g.w)), y = int(i / size_t(g.w));
                for (int dy = -2; dy <= 2; ++ dy)
                    for (int dx = -2; dx <= 2; ++ dx) {
                        if (const int old = prev(g.x0 + x + dx, g.y0 + y + dy); old >= 0)
                            continued.push_back(old);
                        if (std::abs(dx) > 1 || std::abs(dy) > 1 || x + dx < 0 || y + dy < 0 || x + dx >= g.w || y + dy >= g.h)
                            continue;
                        const size_t j = size_t(y + dy) * size_t(g.w) + size_t(x + dx);
                        if (flap[j] < 0 && hangs(j)) {
                            flap[j] = key;
                            stack.push_back(j);
                        }
                    }
            }
            std::sort(continued.begin(), continued.end());
            continued.erase(std::unique(continued.begin(), continued.end()), continued.end());
            for (const int old : continued)
                if (const size_t ro = m_flaps.find(size_t(old)), rk = m_flaps.find(size_t(key)); ro != rk) {
                    m_flaps.join(rk, ro);
                    m_flap_cells[rk] += m_flap_cells[ro];
                }
        }
        return flap;
    }

    // The underside walk of the layer being walked, after its births. Each cell carries a run, how far it hangs past
    // the step from what anchors it. A cell the layer below does not stand under is due where its run passes the reach,
    // plus half a cell for where the lattice samples it, and each due cell is answered in turn:
    // - A due cell on a flap smaller than a small head's disc prints as it hangs: the scar would outweigh the sag.
    // - A due cell a straight line bridges takes no head. Orca bridges a bottom along one direction it picks without
    //   the heads, and its perimeters follow the layer's contour, so a cell in the wall zone, within two lines of the
    //   contour, bridges along any line that stays within a cell of the zone and ends on held cells on both sides
    //   within `bridge_mm`; a cell farther in than the zone bridges only where every direction ends so, on a held cell
    //   or on the contour once the zone is answered: no cell of it left due, and none hanging where no head could
    //   stand or across a gap from one.
    // - The rest calls for heads, the wall zone first while any of it is due. A head covers the cells within the reach
    //   plus the step plus its own radius, in 3-D, and goes where it covers the most due cells, and pending cells of
    //   the layers under it, among the cells the layer below does not stand under that cover at least one due cell:
    //   first on the rim, within two lines of a wall that stays empty for as long as a frontier at the step takes to
    //   advance two lines, with its disc wholly on the layer's material, then on the rim, then with its disc on the
    //   material, then anywhere. Among equals it goes nearest the due cell hanging farthest, then lowest in y, then x.
    //   A head stands only where it may, not within the reach of a head already on the layer, and where the builder
    //   aims it within the face cap of straight down. Due cells no head can answer hang.
    // - A due cell a head covered in 2-D that the head leaves due hangs across a gap from it, and hangs.
    // A cell that hangs stays pending while it lies within a head's cover under the layer walked; what falls out of
    // that cover past one and a half reaches counts as unmet, and each head keeps what of that it answers: due cells
    // it brings within the reach and pending cells its cover takes.
    void underside(size_t l, const std::vector<uint8_t> &kind, const std::vector<int> &flap, Heap &heap)
    {
        const LayerGrid &g      = *m_grid;
        const size_t     cells  = g.cells.size();
        const float      reach  = reach_run(), far = unmet_run();
        const double     cover  = m_reach_mm + double(m_step) + m_in.toolpath_width_mm;
        const double     cover_cells = cover / m_in.cell_mm;
        const float      z      = float(m_in.slabs[l].bottom_z);
        prune_pending(z - float(cover));
        std::vector<char> hanging(cells, 0), bridged(cells, 0);
        std::vector<uint8_t> witness(cells, 0);
        bool first = true, zone_hangs = false;
        for (;;) {
            std::vector<size_t> due;
            for (size_t i = 0; i < cells; ++ i)
                if (kind[i] >= 2 && ! hanging[i] && ! bridged[i] && run_of(i) > reach)
                    due.push_back(i);
            if (due.empty())
                break;
            if (first) {
                m_zone.assign(cells, -1);
                m_rim.assign(cells, -1);
                m_site_ok.assign(cells, -1);
                // A wall stays empty for as long as a frontier at the step takes to advance two lines, up to the layer
                // whose steps above this one add up to two lines: a boundary that advances slower is a wall.
                m_wall_bottom = m_wall_top = l;
                for (double rise = 0.; m_wall_top + 1 < m_in.slabs.size() && rise < 2. * m_in.toolpath_width_mm - EPSILON;)
                    rise += m_in.self_support_mm[++ m_wall_top];
                first = false;
            }
            const auto hang = [&](size_t i) {
                hanging[i] = 1;
                m_pending.push_back({ g.x0 + int(i % size_t(g.w)), g.y0 + int(i / size_t(g.w)), z, run_of(i) });
            };
            due.erase(std::remove_if(due.begin(), due.end(), [&](size_t i) {
                if (flap[i] >= 0 && m_flap_cells[m_flaps.find(size_t(flap[i]))] >= m_flap_floor)
                    return false;
                hang(i);
                return true;
            }), due.end());
            if (m_in.bridge_mm > 0.)
                bridge(due, bridged, witness, zone_hangs);
            due.erase(std::remove_if(due.begin(), due.end(), [&](size_t i) { return bridged[i] != 0; }), due.end());
            if (due.empty())
                break;
            std::vector<size_t> scored;
            for (size_t i : due)
                if (in_zone(g.x0 + int(i % size_t(g.w)), g.y0 + int(i / size_t(g.w))))
                    scored.push_back(i);
            const bool zone = ! scored.empty();
            if (! zone)
                scored = due;
            const std::optional<size_t> site = head_site(l, kind, scored, cover_cells, z);
            if (! site) {
                for (size_t i : scored)
                    hang(i);
                zone_hangs = zone_hangs || zone;
                continue;
            }
            const int bx = g.x0 + int(*site % size_t(g.w)), by = g.y0 + int(*site / size_t(g.w));
            std::vector<size_t> far_due;
            for (size_t i : due)
                if (run_of(i) > far)
                    far_due.push_back(i);
            const size_t tip = place(site_at(l, centre(bx, by), 2. * m_in.toolpath_width_mm), TipNeed::Underside, &heap);
            spread(heap);
            spread_tips();
            size_t answered = size_t(std::count_if(far_due.begin(), far_due.end(), [&](size_t i) { return run_of(i) <= reach; }));
            m_pending.erase(std::remove_if(m_pending.begin(), m_pending.end(), [&](const Pending &q) {
                const double dx = double(q.x - bx), dy = double(q.y - by), dz = double(z - q.z) / m_in.cell_mm;
                if (dx * dx + dy * dy + dz * dz > cover_cells * cover_cells)
                    return false;
                answered += q.run > far;
                return true;
            }), m_pending.end());
            m_plan.tips[tip].answered_mm2 = double(answered) * m_in.cell_mm * m_in.cell_mm;
            // What it covered in 2-D and left due hangs across a gap from it.
            for (size_t i : scored) {
                const double dx = double(g.x0 + int(i % size_t(g.w)) - bx), dy = double(g.y0 + int(i / size_t(g.w)) - by);
                if (dx * dx + dy * dy <= cover_cells * cover_cells && run_of(i) > reach) {
                    hang(i);
                    zone_hangs = zone_hangs || zone;
                }
            }
        }
    }

    bool material(int x, int y) const { return m_grid->at(x, y) != 0; }
    size_t local(int x, int y) const { return size_t(y - m_grid->y0) * size_t(m_grid->w) + size_t(x - m_grid->x0); }
    // Whether lattice cell (x, y) of the layer being walked lies in its wall zone: material within two lines of empty.
    bool in_zone(int x, int y)
    {
        if (! material(x, y))
            return false;
        int8_t &zone = m_zone[local(x, y)];
        if (zone < 0)
            zone = std::any_of(m_two_lines.begin(), m_two_lines.end(), [&](const std::pair<int, int> &d) { return ! material(x + d.first, y + d.second); });
        return zone != 0;
    }
    // Within a cell of the wall zone, where a perimeter following the contour runs.
    bool near_zone(int x, int y)
    {
        for (int dy = -1; dy <= 1; ++ dy)
            for (int dx = -1; dx <= 1; ++ dx)
                if (in_zone(x + dx, y + dy))
                    return true;
        return false;
    }
    bool held(int x, int y) const { return material(x, y) && run_of(local(x, y)) <= reach_run(); }

    // Marks in `bridged` the cells of `due` a straight line bridges on the layer being walked: one sample every half
    // cell along each of `bridge_directions` directions, both ways from the cell. `witness` keeps, per inside cell,
    // the direction that last failed it, which the next test tries first. `zone_hangs` says a cell of the wall zone
    // hangs unanswered, which keeps every inside cell's line off the contour.
    void bridge(const std::vector<size_t> &due, std::vector<char> &bridged, std::vector<uint8_t> &witness, bool zone_hangs)
    {
        const LayerGrid &g     = *m_grid;
        const double     limit = m_in.bridge_mm / m_in.cell_mm;
        const int        steps = int(limit / 0.5) + 1;
        // The distance in cells from (x, y) along direction `k`, one way, to the first sample `end` accepts, or -1
        // where `stop` accepts one first or none comes within `steps` samples.
        const auto walk = [&](int x, int y, int k, double sign, const auto &end, const auto &stop) {
            const double th = M_PI * double(k) / double(bridge_directions), ux = sign * std::cos(th), uy = sign * std::sin(th);
            for (int j = 1; j <= steps; ++ j) {
                const double s  = 0.5 * double(j);
                const int    sx = int(std::floor(double(x) + ux * s + 0.5)), sy = int(std::floor(double(y) + uy * s + 0.5));
                if (stop(sx, sy))
                    return -1.;
                if (end(sx, sy))
                    return s;
            }
            return -1.;
        };
        std::vector<size_t> inside;
        bool                zone_done = ! zone_hangs;
        for (const size_t i : due) {
            const int x = g.x0 + int(i % size_t(g.w)), y = g.y0 + int(i / size_t(g.w));
            if (! in_zone(x, y)) {
                inside.push_back(i);
                continue;
            }
            const auto gap  = [this](int sx, int sy) { return ! near_zone(sx, sy); };
            const auto hold = [this](int sx, int sy) { return held(sx, sy); };
            for (int k = 0; k < bridge_directions && ! bridged[i]; ++ k) {
                const double one = walk(x, y, k, 1., hold, gap);
                if (one < 0.)
                    continue;
                const double other = walk(x, y, k, -1., hold, gap);
                bridged[i] = other >= 0. && one + other <= limit;
            }
            zone_done = zone_done && bridged[i];
        }
        for (const size_t i : inside) {
            const int  x    = g.x0 + int(i % size_t(g.w)), y = g.y0 + int(i / size_t(g.w));
            const auto ends = [this, zone_done](int sx, int sy) { return held(sx, sy) || (zone_done && ! material(sx, sy)); };
            const auto gap  = [this, zone_done](int sx, int sy) { return ! zone_done && ! material(sx, sy); };
            bool       all  = true;
            for (int n = 0; n < bridge_directions && all; ++ n) {
                const int    k   = (int(witness[i]) + n) % bridge_directions;
                const double one = walk(x, y, k, 1., ends, gap);
                const double other = one < 0. ? -1. : walk(x, y, k, -1., ends, gap);
                if (other < 0. || one + other > limit) {
                    witness[i] = uint8_t(k);
                    all        = false;
                }
            }
            bridged[i] = all;
        }
    }

    // Whether lattice cell (x, y) of the layer being walked lies on its rim: within two lines of a wall, a cell empty on
    // the layers from this one to `m_wall_top`.
    bool on_rim(int x, int y)
    {
        int8_t &rim = m_rim[local(x, y)];
        if (rim < 0) {
            const auto wall = [&](int wx, int wy) {
                for (size_t k = m_wall_bottom; k <= m_wall_top; ++ k)
                    if (m_in.material[k].at(wx, wy) != 0)
                        return false;
                return true;
            };
            rim = std::any_of(m_two_lines.begin(), m_two_lines.end(), [&](const std::pair<int, int> &d) { return wall(x + d.first, y + d.second); });
        }
        return rim != 0;
    }

    // Where the next underside head goes on layer `l` for the due cells `scored`, as `underside` lays out, or none.
    std::optional<size_t> head_site(size_t l, const std::vector<uint8_t> &kind, const std::vector<size_t> &scored, double cover_cells, float z)
    {
        const LayerGrid &g     = *m_grid;
        const size_t     cells = g.cells.size();
        // Per cell, the due cells within its cover in 2-D and the pending cells within it in 3-D.
        std::vector<uint32_t> due_near(cells, 0), pending_near(cells, 0);
        const int r = int(std::ceil(cover_cells));
        const auto stamp = [&](int cx, int cy, double r2, std::vector<uint32_t> &count) {
            for (int dy = -r; dy <= r; ++ dy)
                for (int dx = -r; dx <= r; ++ dx) {
                    const int x = cx + dx - g.x0, y = cy + dy - g.y0;
                    if (x >= 0 && y >= 0 && x < g.w && y < g.h && double(dx * dx + dy * dy) <= r2)
                        ++ count[size_t(y) * size_t(g.w) + size_t(x)];
                }
        };
        size_t deepest = scored.front();
        for (const size_t i : scored) {
            stamp(g.x0 + int(i % size_t(g.w)), g.y0 + int(i / size_t(g.w)), cover_cells * cover_cells, due_near);
            if (run_of(i) > run_of(deepest))
                deepest = i;
        }
        for (const Pending &q : m_pending)
            if (const double dz = double(z - q.z) / m_in.cell_mm; std::abs(dz) <= cover_cells)
                stamp(q.x, q.y, cover_cells * cover_cells - dz * dz, pending_near);
        const int fx = int(deepest % size_t(g.w)), fy = int(deepest / size_t(g.w));
        std::vector<size_t> candidates;
        for (size_t i = 0; i < cells; ++ i)
            if (kind[i] >= 2 && due_near[i] > 0)
                candidates.push_back(i);
        const auto from_deepest = [&](size_t i) {
            const int dx = int(i % size_t(g.w)) - fx, dy = int(i / size_t(g.w)) - fy;
            return dx * dx + dy * dy;
        };
        std::sort(candidates.begin(), candidates.end(), [&](size_t a, size_t b) {
            const uint32_t na = due_near[a] + pending_near[a], nb = due_near[b] + pending_near[b];
            return na != nb ? na > nb : from_deepest(a) != from_deepest(b) ? from_deepest(a) < from_deepest(b) : a < b;
        });
        const double spacing = m_reach_mm / m_in.cell_mm;
        const auto   inset   = [&](int x, int y) {
            return std::all_of(m_head_disc.begin(), m_head_disc.end(), [&](const std::pair<int, int> &d) { return material(x + d.first, y + d.second); });
        };
        const auto may_stand = [&](size_t i, int x, int y) {
            if (std::any_of(m_layer_heads.begin(), m_layer_heads.end(), [&](const std::pair<int, int> &h) {
                    return double((h.first - x) * (h.first - x) + (h.second - y) * (h.second - y)) <= spacing * spacing; }))
                return false;
            int8_t &ok = m_site_ok[i];
            if (ok < 0)
                ok = eligible(l, centre(x, y)) && faces_down(l, centre(x, y));
            return ok != 0;
        };
        // Rim and inset, rim, inset, anywhere.
        for (int pass = 0; pass < 4; ++ pass) {
            const bool rim_pass = pass < 2, inset_pass = pass == 0 || pass == 2;
            for (const size_t i : candidates) {
                const int x = g.x0 + int(i % size_t(g.w)), y = g.y0 + int(i / size_t(g.w));
                if ((rim_pass && ! on_rim(x, y)) || (inset_pass && ! inset(x, y)))
                    continue;
                if (may_stand(i, x, y))
                    return i;
            }
        }
        return std::nullopt;
    }

    // The run past which underside no head answers counts as unmet: one and a half reaches.
    float unmet_run() const { return 1.5f * float(m_reach_mm); }

    // Drops the pending cells below `z_min`, which no head placed from here on covers, counting those that hang past one
    // and a half reaches as unmet.
    void prune_pending(float z_min)
    {
        const float far = unmet_run();
        m_pending.erase(std::remove_if(m_pending.begin(), m_pending.end(), [&](const Pending &q) {
            if (q.z >= z_min)
                return false;
            if (q.run > far)
                m_plan.underside_unmet_mm2 += m_in.cell_mm * m_in.cell_mm;
            return true;
        }), m_pending.end());
    }

    // `m_tip_dist` from every head placed on the layer being walked, through its material.
    void spread_tips()
    {
        const LayerGrid &g = *m_grid;
        m_tip_dist.assign(g.cells.size(), unreached);
        Heap heap;
        for (size_t i : m_heads)
            if (m_tip_dist[i] > 0.f) {
                m_tip_dist[i] = 0.f;
                heap.emplace(0.f, i);
            }
        std::swap(m_dist, m_tip_dist);
        spread(heap);
        std::swap(m_dist, m_tip_dist);
    }

    // Where a tip stands under birth piece `p` of slab `l`: its deepest point `spot`, or where that stands at a wall, the
    // eligible cell of the piece nearest it. None where every cell stands at a wall.
    std::optional<Point> birth_spot(size_t l, size_t p, const Point &spot) const
    {
        if (eligible(l, spot))
            return spot;
        const LayerGrid &g     = m_in.material[l];
        const uint16_t   label = uint16_t(p - m_in.components.slab_range[l].first + 1);
        double           best  = std::numeric_limits<double>::max();
        std::optional<Point> found;
        for (size_t i = 0; i < g.cells.size(); ++ i)
            if (g.cells[i] == label) {
                const Point c = centre(g.x0 + int(i % size_t(g.w)), g.y0 + int(i / size_t(g.w)));
                if (const double d = (c - spot).cast<double>().norm(); d < best && eligible(l, c)) {
                    best  = d;
                    found = c;
                }
            }
        return found;
    }

    // Each island starting on slab `l` takes one tip at its deepest point, graded by `birth_grade`, unless an enforced
    // tip already stands on its birth piece, among `enforced` the plan's tips on the slab, or it is debris. An island
    // that merges before it could droop, or one no tip can reach that merges within the debris height, waits for its
    // merge, which `settle` reads: it goes without a tip only when the part it meets is held. Any other island no tip can
    // reach is counted. An island left without a tip prints as it hangs, so its cells anchor what grows on them. Every
    // island goes into the plan with how it is held.
    void births(size_t l, const std::vector<size_t> &enforced, Heap &heap)
    {
        const LayerGrid &g     = *m_grid;
        const size_t     first = m_in.components.slab_range[l].first, last = m_in.components.slab_range[l].second;
        const auto accept = [&](size_t p) {
            const uint16_t label = uint16_t(p - first + 1);
            for (size_t i = 0; i < g.cells.size(); ++ i)
                if (g.cells[i] == label && m_dist[i] > 0.f) {
                    m_dist[i] = 0.f;
                    heap.emplace(0.f, i);
                }
        };
        for (size_t p = first; p < last; ++ p) {
            const SupportAnalysis::Piece &piece = pieces()[p];
            if (! piece.below.empty() || m_rooted[p])
                continue;
            const Point  deepest = inscribed_point(piece.polygon);
            const Vec2d  xy      = unscale(deepest);
            const size_t k       = m_plan.islands.size();
            m_plan.islands.push_back({ Vec3d(xy.x(), xy.y(), m_in.slabs[l].bottom_z) });
            m_island_pieces.push_back({ l, p, deepest });
            Island &island = m_plan.islands.back();
            if (const auto tip = std::find_if(enforced.begin(), enforced.end(),
                                              [&](size_t t) { return piece.polygon.contains(m_plan.tips[t].site.position); });
                tip != enforced.end()) {
                island.holders = { *tip };
                island.reason  = IslandReason::Tip;
                continue;
            }
            const bool   merges = m_births.merge_slab[p] < m_in.slabs.size();
            const double free   = m_births.free_mm[p];
            if (! merges && free <= debris_mm + EPSILON) {
                BOOST_LOG_TRIVIAL(debug) << "scaffold island skipped at " << piece.bottom_z << ": debris";
                island.reason = IslandReason::Debris;
                accept(p);
                continue;
            }
            const BoundingBox box = get_extents(piece.polygon);
            if (merges && free <= m_need.micro_merge_mm + EPSILON && unscale<double>(box.size().maxCoeff()) <= 2. * m_in.toolpath_width_mm + EPSILON) {
                BOOST_LOG_TRIVIAL(debug) << "scaffold island at " << piece.bottom_z << " waits for its merge: micro";
                island.reason = IslandReason::Hung;
                m_waiting[p].push_back(k);
                accept(p);
                continue;
            }
            const std::optional<Point> spot = birth_spot(l, p, deepest);
            if (! spot) {
                if (merges && free <= debris_mm + EPSILON) {
                    BOOST_LOG_TRIVIAL(debug) << "scaffold island at " << piece.bottom_z << " waits for its merge: wall";
                    island.reason = IslandReason::Hung;
                    m_waiting[p].push_back(k);
                } else
                    ++ m_plan.islands_unheld;
                accept(p);
                continue;
            }
            island.holders = { place(site_at(l, *spot, birth_grade(l, p, *spot)), TipNeed::Birth, &heap) };
            island.reason  = IslandReason::Tip;
        }
    }

    // How far `q` stands from part `r`'s nearest tip in 3-D, or above its highest anchor while no tip holds it.
    double from_anchors(size_t r, const Vec3d &q) const
    {
        if (m_anchors[r].empty())
            return q.z() - m_anchor_z[r];
        double near = std::numeric_limits<double>::max();
        for (const Anchor &anchor : m_anchors[r])
            near = std::min(near, (anchor.at - q).norm());
        return near;
    }
    // The lever the nozzle works a part by on layer top `z`: the farthest corner of its section's hull from its
    // anchors, whether the section stands above them or reaches out sideways.
    double lever(size_t r, const Polygon &hull, double z) const
    {
        double far = 0.;
        for (const Point &p : hull)
            far = std::max(far, from_anchors(r, Vec3d(unscale<double>(p.x()), unscale<double>(p.y()), z)));
        return far;
    }

    // A part turns slender when its section reaches farther from its anchors than the stability minimum and than
    // `slender_ratio` of the section's narrowest width, the window: a blade hanging from its point turns slender as it
    // widens, not only as it climbs. It takes a small tip on its down-facing surface within the window's height under
    // this layer, at the corner of a face farthest from its anchors that a tip can stand under, at least half the window
    // from them, since a tip beside an anchor shortens no lever. A part with no such corner is counted once; it, and a
    // part still slender with its new tip, is measured again `stability_retry_mm` higher.
    void stability(size_t l)
    {
        const auto [first, last] = m_in.components.slab_range[l];
        std::vector<size_t> roots;
        for (size_t p = first; p < last; ++ p)
            if (const size_t r = m_sets.find(p); std::find(roots.begin(), roots.end(), r) == roots.end())
                roots.push_back(r);
        const double top = m_in.slabs[l].print_z;
        for (size_t r : roots) {
            if (m_rooted[r] || top - m_rearm_z[r] <= stability_retry_mm + EPSILON)
                continue;
            Points section;
            for (size_t p = first; p < last; ++ p)
                if (m_sets.find(p) == r)
                    append(section, pieces()[p].polygon.contour.points);
            const Polygon hull   = Geometry::convex_hull(section);
            const double  window = std::max(stability_min_mm, m_need.slender_ratio * min_width_mm(hull));
            if (lever(r, hull, top) <= window + EPSILON)
                continue;
            struct Corner { double score; Point p; size_t layer; };
            std::vector<Corner> corners;
            for (size_t k = l + 1; k-- > 0 && m_in.slabs[k].bottom_z > top - window - EPSILON;)
                for (const ExPolygon &face : m_in.down_facing[k])
                    if (root_at(k, face.contour.points.front()) == r)
                        for (const Point &p : face.contour.points)
                            corners.push_back({ from_anchors(r, Vec3d(unscale<double>(p.x()), unscale<double>(p.y()), m_in.slabs[k].bottom_z)), p, k });
            std::stable_sort(corners.begin(), corners.end(), [](const Corner &a, const Corner &b) { return a.score > b.score; });
            auto spot = corners.end();
            for (auto it = corners.begin(); it != corners.end() && it->score >= 0.5 * window; ++ it)
                if (eligible(it->layer, it->p)) {
                    spot = it;
                    break;
                }
            if (spot != corners.end())
                place(site_at(spot->layer, spot->p, 2. * m_in.toolpath_width_mm), TipNeed::Stability, nullptr);
            else {
                if (! m_slender[r])
                    ++ m_plan.islands_slender;
                m_slender[r] = 1;
                BOOST_LOG_TRIVIAL(debug) << "scaffold part slender at " << top << ": no down-facing point";
            }
            if (spot == corners.end() || lever(r, hull, top) > window + EPSILON)
                m_rearm_z[r] = top;
        }
    }
};

} // namespace

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

ObjectMesh::ObjectMesh(const PrintObject &object) : mesh([&object] {
    TriangleMesh framed = object.model_object()->raw_mesh();
    framed.transform(object.trafo_centered());
    return framed;
}()), aabb(mesh) {}

PlanInput prepare_plan(const PrintObject &object, double toolpath_width_mm, double xy_distance_mm, double neck_depth_mm,
                       double bridge_mm, double threshold_rad, const std::vector<Polygons> &blockers, const ObjectMesh *mesh)
{
    PlanInput in;
    in.slabs             = SupportAnalysis::model_slabs_of(object);
    in.toolpath_width_mm = toolpath_width_mm;
    in.xy_distance_mm    = xy_distance_mm;
    in.neck_depth_mm     = neck_depth_mm;
    in.bridge_mm         = bridge_mm;
    in.mesh              = mesh;
    in.z_offset_mm       = object.slicing_parameters().object_print_z_min;
    if (in.slabs.empty() || toolpath_width_mm <= 0.)
        return in;
    in.components = SupportAnalysis::build_components(in.slabs, in.slabs.front().bottom_z);
    in.cell       = std::max<coord_t>(1, coord_t(scale_(toolpath_width_mm / 2.)));
    in.cell_mm    = unscale<double>(in.cell);
    BoundingBox bbox;
    for (const SupportAnalysis::Slab &slab : in.slabs)
        if (! slab.polygons.empty())
            bbox.merge(get_extents(slab.polygons));
    in.origin = bbox.defined ? bbox.min : Point(0, 0);
    const size_t n = in.slabs.size();
    in.material.resize(n);
    in.blocked.resize(n);
    in.down_facing.resize(n);
    in.wall_band.resize(n);
    in.self_support_mm.assign(n, 0.);
    const double tan_threshold = std::tan(threshold_rad);
    tbb::parallel_for(tbb::blocked_range<size_t>(0, n), [&](const tbb::blocked_range<size_t> &range) {
        for (size_t l = range.begin(); l < range.end(); ++ l) {
            in.material[l] = rasterize(in.slabs[l].polygons, in.origin, in.cell);
            if (l < blockers.size() && ! blockers[l].empty())
                in.blocked[l] = rasterize(union_ex(blockers[l]), in.origin, in.cell);
            in.down_facing[l] = l == 0 ? in.slabs[l].polygons : diff_ex(in.slabs[l].polygons, in.slabs[l - 1].polygons);
            in.wall_band[l]   = offset_ex(in.slabs[l].polygons, float(scale_(xy_distance_mm)));
            if (l > 0 && tan_threshold > 0.)
                in.self_support_mm[l] = (in.slabs[l - 1].print_z - in.slabs[l - 1].bottom_z) / tan_threshold;
        }
    });
    return in;
}

Plan plan_tips(const PlanInput &input, const std::vector<TipSite> &enforced, const NeedParams &need)
{
    if (input.slabs.empty() || input.cell <= 0)
        return {};
    return Sweep(input, need).run(enforced);
}

} // namespace Slic3r::ScaffoldSupport
