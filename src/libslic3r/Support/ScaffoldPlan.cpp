#include "ScaffoldPlan.hpp"
#include <algorithm>
#include <cmath>
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
// A part standing at most this high over its highest anchor stands however thin: its anchors hold its whole height.
constexpr double stability_min_mm = 3.;
// A birth tip under an island standing taller than this before it merges carries the island's weight: heavy disc.
constexpr double heavy_birth_mm   = 2.;
// An island that never merges and stands no taller than this is mesh debris, and one whose birth stands at a wall
// and merges within it hangs from that wall.
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

// The narrowest width of `points`' convex hull: the least, over the hull's edges, of the farthest point from the edge.
double min_width_mm(const Points &points)
{
    const Polygon hull = Geometry::convex_hull(points);
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
        m_anchors(in.components.pieces.size()), m_rooted(in.components.pieces.size(), 0),
        m_slender(in.components.pieces.size(), 0)
    {
        m_births = walk_births(in);
        // Cells within the self-support step of the layer below, and within the xy distance of a wall, by offset.
        const double wall_r = (in.xy_distance_mm + 0.71 * in.cell_mm) / in.cell_mm;
        for (int dy = -int(wall_r) - 1; dy <= int(wall_r) + 1; ++ dy)
            for (int dx = -int(wall_r) - 1; dx <= int(wall_r) + 1; ++ dx)
                if (double(dx * dx + dy * dy) <= wall_r * wall_r)
                    m_wall_disc.emplace_back(dx, dy);
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
        std::stable_sort(m_plan.tips.begin(), m_plan.tips.end(), [](const PlannedTip &a, const PlannedTip &b) {
            return a.site.print_z != b.site.print_z ? a.site.print_z < b.site.print_z :
                   a.site.position.x() != b.site.position.x() ? a.site.position.x() < b.site.position.x() :
                                                                a.site.position.y() < b.site.position.y();
        });
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
    std::vector<double>      m_anchor_z;   // per root: its highest anchor
    std::vector<std::vector<Vec3d>> m_anchors;
    std::vector<char>        m_rooted, m_slender;
    std::vector<std::pair<int, int>> m_wall_disc;
    Plan                     m_plan;
    // The layer being walked and the one under it: their window on the lattice and each cell's run.
    LayerGrid                m_prev_grid;
    std::vector<float>       m_prev_run;
    const LayerGrid         *m_grid = nullptr;
    std::vector<float>       m_dist;

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
            size_t root = m_sets.find(piece.below.front());
            for (size_t q : piece.below)
                if (const size_t r = m_sets.find(q); r != root) {
                    m_anchor_z[root] = std::max(m_anchor_z[root], m_anchor_z[r]);
                    append(m_anchors[root], std::move(m_anchors[r]));
                    m_rooted[root]  = m_rooted[root] || m_rooted[r];
                    m_slender[root] = m_slender[root] && m_slender[r];
                    m_sets.join(root, r);
                }
            m_sets.join(root, p);
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
        return std::none_of(m_wall_disc.begin(), m_wall_disc.end(), [&](const std::pair<int, int> &d) { return wall.at(x + d.first, y + d.second) != 0; });
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

    void place(const TipSite &site, TipNeed need, Heap *heap)
    {
        m_plan.tips.push_back({ site, need });
        const size_t l      = size_t(site.obj_layer_nr + 1);
        const Vec2d  xy     = unscale(site.position);
        const size_t root   = root_at(l, site.position);
        if (root != size_t(-1)) {
            m_anchors[root].emplace_back(xy.x(), xy.y(), site.print_z);
            m_anchor_z[root] = std::max(m_anchor_z[root], site.print_z);
        }
        if (heap == nullptr)
            return;
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
                    if (m_dist[i] > 0.f) {
                        m_dist[i] = 0.f;
                        heap->emplace(0.f, i);
                    }
                }
    }

    TipSite site_at(size_t l, const Point &p, double grade_mm) const
    {
        TipSite site { p, m_in.slabs[l].bottom_z, int(l) - 1 };
        site.contact  = true;
        site.grade_mm = grade_mm;
        return site;
    }

    // Octile Dijkstra through the layer's material, from whatever the heap holds.
    void spread(Heap &heap)
    {
        const LayerGrid &g    = *m_grid;
        const float      step = float(m_in.cell_mm), diag = float(m_in.cell_mm * M_SQRT2);
        while (! heap.empty()) {
            const auto [d, i] = heap.top();
            heap.pop();
            if (d > m_dist[i])
                continue;
            const int x = int(i % size_t(g.w)), y = int(i / size_t(g.w));
            for (int dy = -1; dy <= 1; ++ dy)
                for (int dx = -1; dx <= 1; ++ dx) {
                    if ((dx | dy) == 0)
                        continue;
                    const int nx = x + dx, ny = y + dy;
                    if (nx < 0 || ny < 0 || nx >= g.w || ny >= g.h)
                        continue;
                    const size_t j = size_t(ny) * size_t(g.w) + size_t(nx);
                    if (g.cells[j] == 0)
                        continue;
                    const float nd = d + (dx != 0 && dy != 0 ? diag : step);
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

        births(l, heap);
        for (const TipSite &tip : enforced)
            place(tip, TipNeed::Enforced, &heap);
        spread(heap);

        // Tips where a run exceeds the reach, until none does or no eligible cell is left to answer it.
        const float reach = float(m_need.reach_mm), aa = float(a);
        const auto  run_of = [&](size_t i) { return std::max(0.f, m_dist[i] - aa); };
        for (;;) {
            std::vector<std::pair<float, size_t>> due;
            for (size_t i = 0; i < cells; ++ i)
                if (kind[i] == 3 && run_of(i) > reach)
                    due.emplace_back(std::abs(run_of(i) - 2.f * reach), i);
            if (due.empty())
                break;
            std::sort(due.begin(), due.end());
            bool placed = false;
            for (const auto &[key, i] : due) {
                const Point p = centre(g.x0 + int(i % size_t(g.w)), g.y0 + int(i / size_t(g.w)));
                if (eligible(l, p)) {
                    place(site_at(l, p, 0.), TipNeed::Underside, &heap);
                    spread(heap);
                    placed = true;
                    break;
                }
            }
            if (! placed) {
                // Under a blocker or at a wall: the need goes unmet, counted once, and the cells print as they hang.
                m_plan.underside_unmet_mm2 += double(due.size()) * m_in.cell_mm * m_in.cell_mm;
                for (const auto &[key, i] : due) {
                    m_dist[i] = 0.f;
                    heap.emplace(0.f, i);
                }
                spread(heap);
            }
        }

        m_prev_grid = g;
        m_prev_run.assign(cells, 0.f);
        for (size_t i = 0; i < cells; ++ i)
            if (kind[i] != 0)
                m_prev_run[i] = m_dist[i] == unreached ? 0.f : run_of(i);
    }

    // Each island starting on slab `l` takes one tip at its deepest point, unless it is debris, merges before it could
    // droop, or hangs from a wall; an island no tip can reach is counted. An island left without a tip prints as it
    // hangs, so its cells anchor what grows on them.
    void births(size_t l, Heap &heap)
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
            const bool   merges = m_births.merge_slab[p] < m_in.slabs.size();
            const double free   = m_births.free_mm[p];
            if (! merges && free <= debris_mm + EPSILON) {
                BOOST_LOG_TRIVIAL(debug) << "scaffold island skipped at " << piece.bottom_z << ": debris";
                accept(p);
                continue;
            }
            const BoundingBox box = get_extents(piece.polygon);
            if (merges && free <= m_need.micro_merge_mm + EPSILON && unscale<double>(box.size().maxCoeff()) <= 2. * m_in.toolpath_width_mm + EPSILON) {
                BOOST_LOG_TRIVIAL(debug) << "scaffold island skipped at " << piece.bottom_z << ": micro";
                accept(p);
                continue;
            }
            // The deepest point, or where it stands at a wall, the eligible cell of the piece nearest it.
            Point spot = inscribed_point(piece.polygon);
            if (! eligible(l, spot)) {
                const uint16_t label = uint16_t(p - first + 1);
                double         best  = std::numeric_limits<double>::max();
                Point          found = spot;
                for (size_t i = 0; i < g.cells.size(); ++ i)
                    if (g.cells[i] == label) {
                        const Point c = centre(g.x0 + int(i % size_t(g.w)), g.y0 + int(i / size_t(g.w)));
                        if (const double d = (c - spot).cast<double>().norm(); d < best && eligible(l, c)) {
                            best  = d;
                            found = c;
                        }
                    }
                if (best == std::numeric_limits<double>::max()) {
                    if (merges && free <= debris_mm + EPSILON)
                        BOOST_LOG_TRIVIAL(debug) << "scaffold island held at " << piece.bottom_z << ": wall";
                    else
                        ++ m_plan.islands_unheld;
                    accept(p);
                    continue;
                }
                spot = found;
            }
            place(site_at(l, spot, free > heavy_birth_mm ? 4. * m_in.toolpath_width_mm : 0.), TipNeed::Birth, &heap);
        }
    }

    // A part turns slender when it stands over its highest anchor by more than the stability minimum and by more than
    // `slender_ratio` of its section's narrowest width: it takes a heavy tip on its down-facing surface within that
    // last stretch, the point farthest from its anchors, or is counted once when it has none.
    void stability(size_t l)
    {
        const auto [first, last] = m_in.components.slab_range[l];
        std::vector<size_t> roots;
        for (size_t p = first; p < last; ++ p)
            if (const size_t r = m_sets.find(p); std::find(roots.begin(), roots.end(), r) == roots.end())
                roots.push_back(r);
        const double top = m_in.slabs[l].print_z;
        for (size_t r : roots) {
            if (m_rooted[r] || m_slender[r] || top - m_anchor_z[r] <= stability_min_mm + EPSILON)
                continue;
            Points hull_points;
            for (size_t p = first; p < last; ++ p)
                if (m_sets.find(p) == r)
                    append(hull_points, pieces()[p].polygon.contour.points);
            const double window = m_need.slender_ratio * min_width_mm(hull_points);
            if (top - m_anchor_z[r] <= window + EPSILON)
                continue;
            double best = -1.;
            Point  spot;
            size_t spot_layer = 0;
            for (size_t k = l + 1; k-- > 0 && m_in.slabs[k].bottom_z >= top - window - EPSILON;)
                for (const ExPolygon &face : m_in.down_facing[k]) {
                    const Point p = face.contour.points[face.contour.size() / 2];
                    if (root_at(k, p) != r || ! eligible(k, p))
                        continue;
                    const Vec3d q(unscale<double>(p.x()), unscale<double>(p.y()), m_in.slabs[k].bottom_z);
                    double      near = std::numeric_limits<double>::max();
                    for (const Vec3d &anchor : m_anchors[r])
                        near = std::min(near, (anchor - q).norm());
                    if (near > best) {
                        best       = near;
                        spot       = p;
                        spot_layer = k;
                    }
                }
            if (best < 0.) {
                m_slender[r] = 1;
                ++ m_plan.islands_slender;
                BOOST_LOG_TRIVIAL(debug) << "scaffold part slender at " << top << ": no down-facing point";
                continue;
            }
            place(site_at(spot_layer, spot, 4. * m_in.toolpath_width_mm), TipNeed::Stability, nullptr);
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

NeedParams need_params(double density)
{
    static constexpr NeedParams tiers[3] = { { 1.5, 4., 0.3 }, { 1., 3., 0.12 }, { 0.6, 2., 0. } };
    const double d = std::clamp(density, 0., 2.);
    const size_t i = d >= 1. ? 1 : 0;
    const double t = d - double(i);
    const auto   mix = [t](double a, double b) { return a + (b - a) * t; };
    return { mix(tiers[i].reach_mm, tiers[i + 1].reach_mm), mix(tiers[i].slender_ratio, tiers[i + 1].slender_ratio),
             mix(tiers[i].micro_merge_mm, tiers[i + 1].micro_merge_mm) };
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
    in.self_support_mm.assign(n, 0.);
    const double tan_threshold = std::tan(threshold_rad);
    tbb::parallel_for(tbb::blocked_range<size_t>(0, n), [&](const tbb::blocked_range<size_t> &range) {
        for (size_t l = range.begin(); l < range.end(); ++ l) {
            in.material[l] = rasterize(in.slabs[l].polygons, in.origin, in.cell);
            if (l < blockers.size() && ! blockers[l].empty())
                in.blocked[l] = rasterize(union_ex(blockers[l]), in.origin, in.cell);
            in.down_facing[l] = l == 0 ? in.slabs[l].polygons : diff_ex(in.slabs[l].polygons, in.slabs[l - 1].polygons);
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
