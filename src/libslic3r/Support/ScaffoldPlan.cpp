#include "ScaffoldPlan.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <optional>
#include <queue>
#include "ClipperUtils.hpp"
#include "Geometry/ConvexHull.hpp"
#include "Layer.hpp"
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
// A birth tip under an island standing taller than this before it merges carries the island's weight: heavy disc.
constexpr double heavy_birth_mm   = 2.;
// An island that never merges and stands no taller than this is mesh debris, and one whose birth stands at a wall
// and merges within it hangs from that wall when the part it meets is held.
constexpr double debris_mm        = 1.;

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
// count where it never does, and how far it stands free, to that slab or to its part's top.
struct Births
{
    std::vector<size_t> merge_slab;
    std::vector<double> free_mm;
};

Births walk_births(const PlanInput &in)
{
    const std::vector<SupportAnalysis::Piece> &pieces = in.components.pieces;
    const size_t                               n      = in.slabs.size();
    Births births;
    births.merge_slab.assign(pieces.size(), n);
    births.free_mm.assign(pieces.size(), 0.);
    DisjointSets                     sets(pieces.size());
    std::vector<std::vector<size_t>> open(pieces.size());   // per root: its births that have not merged yet
    std::vector<double>              top(pieces.size(), 0.);
    for (size_t s = 0; s < n; ++ s) {
        const auto [first, last] = in.components.slab_range[s];
        for (size_t p = first; p < last; ++ p) {
            const SupportAnalysis::Piece &piece = pieces[p];
            if (piece.below.empty()) {
                open[p].push_back(p);
                top[p] = piece.print_z;
                continue;
            }
            std::vector<size_t> roots;
            for (size_t q : piece.below)
                if (const size_t r = sets.find(q); std::find(roots.begin(), roots.end(), r) == roots.end())
                    roots.push_back(r);
            if (roots.size() > 1)
                for (size_t r : roots) {
                    for (size_t birth : open[r])
                        births.merge_slab[birth] = s;
                    open[r].clear();
                }
            const size_t root = roots.front();
            for (size_t i = 1; i < roots.size(); ++ i) {
                top[root] = std::max(top[root], top[roots[i]]);
                sets.join(root, roots[i]);
            }
            sets.join(root, p);
            top[root] = std::max(top[root], piece.print_z);
        }
    }
    for (size_t p = 0; p < pieces.size(); ++ p)
        if (pieces[p].below.empty())
            births.free_mm[p] = (births.merge_slab[p] < n ? in.slabs[births.merge_slab[p]].bottom_z : top[sets.find(p)]) - pieces[p].bottom_z;
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
        m_slender(in.components.pieces.size(), 0)
    {
        m_births = walk_births(in);
        // Cells within the self-support step of the layer below, and within the xy distance of a wall, by offset.
        const double wall_r = (in.xy_distance_mm + 0.71 * in.cell_mm) / in.cell_mm;
        for (int dy = -int(wall_r) - 1; dy <= int(wall_r) + 1; ++ dy)
            for (int dx = -int(wall_r) - 1; dx <= int(wall_r) + 1; ++ dx)
                if (double(dx * dx + dy * dy) <= wall_r * wall_r)
                    m_wall_disc.emplace_back(dx, dy);
        const double head_r = in.toolpath_width_mm / in.cell_mm;
        for (int dy = -int(head_r); dy <= int(head_r); ++ dy)
            for (int dx = -int(head_r); dx <= int(head_r); ++ dx)
                if (double(dx * dx + dy * dy) <= head_r * head_r)
                    m_head_disc.emplace_back(dx, dy);
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
    std::vector<std::pair<int, int>> m_head_disc;   // the cells under a small head, two support lines across
    Plan                     m_plan;
    // The layer being walked and the one under it: their window on the lattice and each cell's run.
    LayerGrid                m_prev_grid;
    std::vector<float>       m_prev_run;
    const LayerGrid         *m_grid = nullptr;
    std::vector<float>       m_dist, m_tip_dist;
    // Underside cells that hang past the reach with no head answering them yet: lattice x and y, bottom z and run.
    struct Pending { int x, y; float z, run; };
    std::vector<Pending>     m_pending;
    std::vector<size_t>      m_heads;      // the cells under the heads placed on the layer being walked

    const std::vector<SupportAnalysis::Piece> &pieces() const { return m_in.components.pieces; }
    Point centre(int x, int y) const { return m_in.origin + Point(coord_t((x + 0.5) * m_in.cell), coord_t((y + 0.5) * m_in.cell)); }
    std::pair<int, int> cell_of(const Point &p) const
    {
        return { int(std::floor(double(p.x() - m_in.origin.x()) / double(m_in.cell))),
                 int(std::floor(double(p.y() - m_in.origin.y()) / double(m_in.cell))) };
    }

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
                    const double grade = m_births.free_mm[birth.piece] > heavy_birth_mm ? 4. * m_in.toolpath_width_mm : 0.;
                    holders = { place(site_at(birth.slab, *spot, grade), TipNeed::Birth, nullptr) };
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
        site.contact  = true;
        site.grade_mm = grade_mm;
        return site;
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
            return;
        }
        const LayerGrid *below  = l > 0 ? &m_in.material[l - 1] : nullptr;
        const double     a      = m_in.self_support_mm[l];
        const double     a_cells = a / m_in.cell_mm;
        const size_t     cells  = g.cells.size();
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
        std::vector<size_t> enforced_tips;
        for (const TipSite &tip : enforced)
            enforced_tips.push_back(place(tip, TipNeed::Enforced, &heap));
        births(l, enforced_tips, heap);
        spread(heap);
        spread_tips();

        // Tips where a run exceeds the reach, on any cell the layer below does not stand under, the step it carries
        // included: on a shallow slope that step is most of the surface that hangs. A head may stand on any such cell,
        // due or not, so it can stand in from an edge and still cover what hangs there. A head covers the underside within the reach plus the self-support step
        // plus its own radius, in 3-D, and has to answer at least a disc of half the reach of it: underside that hangs
        // past the reach and no head answers stays pending while it lies within that cover under the layer walked, so a
        // shallow frontier, which comes due a scattered cell at a time, calls for a head once enough of it hangs, and a
        // sliver too small to be worth a scar prints as it hangs. Each head goes on the eligible due cell covering the
        // most due and pending cells, candidates sampled a third of the cover apart, among equals the first listed,
        // lowest in y, then x. A cell whose small head's disc lies wholly on the layer's material goes before one on an
        // edge, where half the disc would hang and the scar would sit on the corner; a feature narrower than the disc
        // takes its head on the edge. What hangs past one and a half reaches and no head covers counts as unmet, and a
        // head keeps what of that it answers: due cells it brings within the reach and pending cells its cover takes.
        const float  reach  = float(m_need.reach_mm + 0.5 * m_in.cell_mm), aa = float(a), far = unmet_run();
        const auto   run_of = [&](size_t i) { return std::max(0.f, m_dist[i] - aa); };
        const double cover  = m_need.reach_mm + a + m_in.toolpath_width_mm;
        const double cover_cells = cover / m_in.cell_mm;
        const int    stride      = std::max(1, int(std::lround(cover_cells / 3.)));
        const size_t least       = std::max<size_t>(1, size_t(std::lround(M_PI * 0.25 * m_need.reach_mm * m_need.reach_mm / (m_in.cell_mm * m_in.cell_mm))));
        const float  z           = float(m_in.slabs[l].bottom_z);
        prune_pending(z - float(cover));
        std::vector<char> hanging(cells, 0);
        for (;;) {
            std::vector<size_t> due;
            for (size_t i = 0; i < cells; ++ i)
                if (kind[i] >= 2 && ! hanging[i] && run_of(i) > reach)
                    due.push_back(i);
            if (due.empty())
                break;
            // What a head at cell `c` of this layer covers: due cells on the layer and pending cells under it.
            const auto covers = [&](size_t c) {
                const int    cx = g.x0 + int(c % size_t(g.w)), cy = g.y0 + int(c / size_t(g.w));
                const double r2 = cover_cells * cover_cells;
                size_t       n  = 0;
                for (size_t j : due) {
                    const double dx = double(g.x0 + int(j % size_t(g.w)) - cx), dy = double(g.y0 + int(j / size_t(g.w)) - cy);
                    n += dx * dx + dy * dy <= r2;
                }
                for (const Pending &q : m_pending) {
                    const double dx = double(q.x - cx), dy = double(q.y - cy), dz = double(z - q.z) / m_in.cell_mm;
                    n += dx * dx + dy * dy + dz * dz <= r2;
                }
                return n;
            };
            std::vector<size_t> sites;
            for (size_t i = 0; i < cells; ++ i)
                if (kind[i] >= 2)
                    sites.push_back(i);
            size_t best = 0, best_n = 0;
            for (int pass = 0; pass < 3 && best_n < least; ++ pass)
                for (size_t i : sites) {
                    const int x = g.x0 + int(i % size_t(g.w)), y = g.y0 + int(i / size_t(g.w));
                    if (pass == 0 && (x % stride != 0 || y % stride != 0))
                        continue;
                    if (pass < 2 && std::any_of(m_head_disc.begin(), m_head_disc.end(), [&](const std::pair<int, int> &d) { return g.at(x + d.first, y + d.second) == 0; }))
                        continue;
                    const size_t n = covers(i);
                    if (n > best_n && eligible(l, centre(x, y))) {
                        best   = i;
                        best_n = n;
                    }
                }
            if (best_n < least) {
                for (size_t i : due) {
                    hanging[i] = 1;
                    m_pending.push_back({ g.x0 + int(i % size_t(g.w)), g.y0 + int(i / size_t(g.w)), z, run_of(i) });
                }
                break;
            }
            const int bx = g.x0 + int(best % size_t(g.w)), by = g.y0 + int(best / size_t(g.w));
            std::vector<size_t> far_due;
            for (size_t i : due)
                if (run_of(i) > far)
                    far_due.push_back(i);
            const size_t tip = place(site_at(l, centre(bx, by), 0.), TipNeed::Underside, &heap);
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
        }

        // What the layer above hangs from. Material within the reach of a head on this layer spans to that head as a
        // line bridges to its anchor, so it holds what grows on it as the layer below holds; the rest passes its run on.
        m_prev_grid = g;
        m_prev_run.assign(cells, 0.f);
        for (size_t i = 0; i < cells; ++ i)
            if (kind[i] != 0 && m_tip_dist[i] > reach)
                m_prev_run[i] = m_dist[i] == unreached ? 0.f : run_of(i);
    }

    // The run past which underside no head answers counts as unmet: one and a half reaches.
    float unmet_run() const { return 1.5f * float(m_need.reach_mm); }

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

    // Each island starting on slab `l` takes one tip at its deepest point, unless an enforced tip already stands on its
    // birth piece, among `enforced` the plan's tips on the slab, or it is debris. An island that merges before it could
    // droop, or one no tip can reach that merges within the debris height, waits for its merge, which `settle` reads: it
    // goes without a tip only when the part it meets is held. Any other island no tip can reach is counted. An island
    // left without a tip prints as it hangs, so its cells anchor what grows on them. Every island goes into the plan
    // with how it is held.
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
            island.holders = { place(site_at(l, *spot, free > heavy_birth_mm ? 4. * m_in.toolpath_width_mm : 0.), TipNeed::Birth, &heap) };
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
    // widens, not only as it climbs. It takes a heavy tip on its down-facing surface within the window's height under
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
                place(site_at(spot->layer, spot->p, 4. * m_in.toolpath_width_mm), TipNeed::Stability, nullptr);
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

PlanInput prepare_plan(const PrintObject &object, double toolpath_width_mm, double xy_distance_mm, double neck_depth_mm,
                       double threshold_rad, const std::vector<Polygons> &blockers)
{
    PlanInput in;
    in.slabs             = SupportAnalysis::model_slabs_of(object);
    in.toolpath_width_mm = toolpath_width_mm;
    in.xy_distance_mm    = xy_distance_mm;
    in.neck_depth_mm     = neck_depth_mm;
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
