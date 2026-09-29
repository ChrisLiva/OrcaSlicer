#include "ScaffoldPlan.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <numeric>
#include <optional>
#include <queue>
#include "AABBTreeLines.hpp"
#include "ClipperUtils.hpp"
#include "Layer.hpp"
#include "Model.hpp"
#include "Print.hpp"
#include "DisjointSets.hpp"
#include "../SLA/SupportTreeBuildsteps.hpp"
#include <boost/log/trivial.hpp>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

namespace Slic3r::ScaffoldSupport {

namespace {

constexpr float  unreached        = std::numeric_limits<float>::infinity();
// A part reaching no farther than this from its anchors stands however thin: they hold all of it.
constexpr double stability_min_mm = 3.;
// How many of its thicknesses a stem may reach from what holds it. A rod bends as F L^3 / (E t^4), and the nozzle
// pushes a section with a force that grows with the section it prints, so a fixed reach per thickness holds a fixed
// deflection; 3 meets the 3 mm floor at a 1 mm stem.
constexpr double stability_ratio  = 3.;
// How much thicker than a stem a section it grows out of must be to clamp it: twice as thick is 16 times as stiff.
constexpr double junction_ratio   = 2.;
// How far under a section its stem's thickness is read, and how much higher a branch whose stability search ended
// short is searched again: a face a little higher may take a tip.
constexpr double stability_retry_mm = 1.;
// A birth tip under an island that carries its part higher than this takes the heavy disc where the section fuses it.
constexpr double heavy_birth_mm   = 2.;
// An island that never merges and stands no taller than this is mesh debris.
constexpr double debris_mm        = 1.;
// A birth no wider than two support lines that merges having stood free less than this is a nub: every reference birth
// that stands free this long or longer carries a contact, whatever the layer height.
constexpr double nub_free_mm      = 0.1;
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
// The area in mm2 a piece may leave outside a grown set and still lie within it: Clipper's rounding of the two outlines.
constexpr double within_mm2       = 1e-4;

using Heap = std::priority_queue<std::pair<float, size_t>, std::vector<std::pair<float, size_t>>, std::greater<>>;

// The deepest inward offset of `piece` that leaves any of it, scaled, found to 0.01 mm, and what it leaves: the offset
// is the radius of the largest circle inside the piece, and the circle's centre lies in what is left.
std::pair<double, ExPolygons> deepest_offset(const ExPolygon &piece)
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
    return { inside, std::move(deepest) };
}

// The radius of the largest circle inside `piece`, in mm.
double inscribed_radius(const ExPolygon &piece) { return unscale<double>(deepest_offset(piece).first); }

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

// The lattice offsets within `r` cells of a cell.
std::vector<std::pair<int, int>> disc(double r)
{
    std::vector<std::pair<int, int>> out;
    for (int dy = -int(r) - 1; dy <= int(r) + 1; ++ dy)
        for (int dx = -int(r) - 1; dx <= int(r) + 1; ++ dx)
            if (double(dx * dx + dy * dy) <= r * r)
                out.emplace_back(dx, dy);
    return out;
}

std::pair<int, int> cell_of(const PlanInput &in, const Point &p)
{
    return { int(std::floor(double(p.x() - in.origin.x()) / double(in.cell))), int(std::floor(double(p.y() - in.origin.y()) / double(in.cell))) };
}
Point centre_of(const PlanInput &in, int x, int y) { return in.origin + Point(coord_t((x + 0.5) * in.cell), coord_t((y + 0.5) * in.cell)); }

// The piece of slab `l` holding `p`, or npos. The lattice labels a cell by the piece holding its centre, so a point off
// the centre, as a corner on an outline is, may read a neighbour's label: the outlines decide, and the label stands only
// for a point that no outline holds.
size_t piece_at(const PlanInput &in, size_t l, const Point &p)
{
    const auto [first, last] = in.components.slab_range[l];
    const auto [x, y]        = cell_of(in, p);
    const uint16_t label     = in.material[l].at(x, y);
    const size_t   labelled  = label != 0 ? first + label - 1 : size_t(-1);
    if (labelled != size_t(-1) && in.components.pieces[labelled].polygon.contains(p))
        return labelled;
    for (size_t q = first; q < last; ++ q)
        if (q != labelled && in.components.pieces[q].polygon.contains(p))
            return q;
    return labelled;
}

// Whether every point of `piece` lies within `mm` of `set`, grown with round joins, which reach no farther than `mm`.
bool within(const ExPolygon &piece, const Polygons &set, double mm)
{
    double left = 0.;
    for (const ExPolygon &out : diff_ex(ExPolygons{ piece }, offset(set, float(scale_(mm)), ClipperLib::jtRound, scale_(0.01))))
        left += out.area();
    return left * SCALING_FACTOR * SCALING_FACTOR < within_mm2;
}

// How far a nub may lie from what holds it at merge slab `s`: that slab's step plus the reach, so the merge layer
// bridges to it no farther than to any underside the planner leaves bare.
double hang_mm(const PlanInput &in, size_t s) { return in.self_support_mm[s] + reach_lines * in.toolpath_width_mm; }

// What the parts other than `own` that `held` holds, by root in `sets`, have under merge piece `q` within `hang` mm of
// `nub`, with those parts' roots added to `holding`.
Polygons footing(const PlanInput &in, DisjointSets &sets, size_t q, size_t own, const ExPolygon &nub, double hang,
                 const std::function<bool(size_t)> &held, std::vector<size_t> &holding)
{
    const Polygons    reach    = offset(nub, float(scale_(hang)), ClipperLib::jtRound, scale_(0.01));
    const BoundingBox near_box = get_extents(reach);
    Polygons          out;
    for (const size_t b : in.components.pieces[q].below)
        if (const size_t rb = sets.find(b); rb != own && held(rb)) {
            const Polygons part = ClipperUtils::clip_clipper_polygons_with_subject_bbox(in.components.pieces[b].polygon, near_box);
            if (part.empty() || intersection(part, reach).empty())
                continue;
            append(out, part);
            if (std::find(holding.begin(), holding.end(), rb) == holding.end())
                holding.push_back(rb);
        }
    return out;
}

// The disc a birth tip at `p` under piece `piece` of slab `l` fuses with. The heavy one, four lines across, goes where the
// island carries its part higher than `heavy_birth_mm` and the model the pin reaches, the slabs within one toolpath width
// over the tip, fills at least twice as many cells of the heavy disc as of the small one; the small one, two lines
// across, goes everywhere else, where the heavy disc would add scar and no hold.
double birth_grade(const PlanInput &in, const Births &births, size_t l, size_t piece, const Point &p)
{
    const double w = in.toolpath_width_mm;
    if (births.carry_mm[piece] <= heavy_birth_mm)
        return 2. * w;
    const auto [cx, cy] = cell_of(in, p);
    const int  r        = int(std::ceil(2. * w / in.cell_mm)) + 1;
    size_t     small = 0, heavy = 0;
    for (int dy = -r; dy <= r; ++ dy)
        for (int dx = -r; dx <= r; ++ dx) {
            const double d = (centre_of(in, cx + dx, cy + dy) - p).cast<double>().norm() * SCALING_FACTOR;
            if (d > 2. * w)
                continue;
            bool filled = false;
            for (size_t k = l; k < in.slabs.size() && in.slabs[k].bottom_z < in.slabs[l].bottom_z + w && ! filled; ++ k)
                filled = in.material[k].at(cx + dx, cy + dy) != 0;
            heavy += filled;
            small += filled && d <= w;
        }
    return heavy >= 2 * std::max<size_t>(small, 1) ? 4. * w : 2. * w;
}

const Vec3d straight_down(0., 0., -1.);

// The neck a head hangs from a tip by, read on the plan's lattice: one neck depth from a point on the bottom of a slab
// along a unit axis pointing down.
class Necks
{
public:
    explicit Necks(const PlanInput &in)
        : m_in(in), m_wall_disc(disc((in.xy_distance_mm + 0.71 * in.cell_mm) / in.cell_mm)),
          m_step(std::asin(std::min(1., in.cell_mm / in.neck_depth_mm)))
    {}

    // Whether a neck from `p` on the bottom of slab `l` along `axis` clears the model, as the wall skip reads it: no
    // blocker covers `p`'s cell, and the neck's end, on the first slab whose top reaches it, stands clear of the
    // lattice's material by the xy distance plus half a cell's diagonal and outside the exact band. The lattice
    // places the end and the wall only to within a cell each, and a tip on the model's edge, as every stability tip
    // is, stands where that decides: the wall skip reads the band itself, so the neck does too. An end under the
    // first slab stands by the pad. The shaft also crosses no material on the slabs between its end and its tip, a
    // thin shelf the end has passed included. Straight down, the run of material right under the tip is the tip's
    // own face to the lattice's resolution and is not crossed: a tip on a face's edge stands over the slab below's
    // contour, whose cell reads material about half the time, and a steep face's column stays in its part for a few
    // slabs. A leaning neck has no such allowance, so any material on its shaft is crossed: a birth's stands over no
    // face of its own, and a stability tip's clears only where its shaft leaves its face's column at once.
    bool clear(size_t l, const Point &p, const Vec3d &axis) const
    {
        const auto [x, y] = cell_of(m_in, p);
        if (m_in.blocked[l].at(x, y) != 0)
            return false;
        const double depth = m_in.neck_depth_mm;
        const double z     = m_in.slabs[l].bottom_z + depth * axis.z();
        const size_t k     = end_slab(z);
        if (z > m_in.slabs.front().bottom_z && k < m_in.slabs.size()) {
            const Point               end  = along(p, axis, depth);
            const std::pair<int, int> cell = cell_of(m_in, end);
            const LayerGrid          &wall = m_in.material[k];
            if (std::any_of(m_wall_disc.begin(), m_wall_disc.end(),
                            [&](const std::pair<int, int> &d) { return wall.at(cell.first + d.first, cell.second + d.second) != 0; }))
                return false;
            const ExPolygons &band = m_in.wall_band[k];
            if (std::any_of(band.begin(), band.end(), [&end](const ExPolygon &expoly) { return expoly.contains(end); }))
                return false;
        }
        const double top    = m_in.slabs[l].bottom_z;
        const size_t bottom = z <= m_in.slabs.front().bottom_z ? 0 : k + 1;
        bool         face   = axis.x() == 0. && axis.y() == 0.;
        for (size_t j = l; j -- > bottom;) {
            const double mid = 0.5 * (m_in.slabs[j].bottom_z + m_in.slabs[j].print_z);
            const auto [sx, sy] = cell_of(m_in, along(p, axis, (top - mid) / -axis.z()));
            const bool solid = m_in.material[j].at(sx, sy) != 0;
            if (solid && ! face)
                return false;
            face = face && solid;
        }
        return true;
    }

    // Whether the builder keeps the full head of a tip at `p` on the bottom of slab `l` whose pin is `pin_mm` in radius
    // along `axis`: the builder's own pinhead test, `sla::pinhead_mesh_intersect`, reads the head clear of the object's
    // mesh over its length at the point, axis and pin the builder reads, each a float. Where the head meets the model
    // along a handed axis, the filter searches another pose or falls back to a thin head, whose rings the neck check cuts
    // where they float. The filter also refuses a full head whose back reaches under the pad's top, which is left to the
    // builder: a tip that low stands on a post or a thin head by the pad. With no mesh or no head every axis fits.
    bool head_fits(size_t l, const Point &p, const Vec3d &axis, double pin_mm) const
    {
        const HeadShape &head = m_in.head;
        if (m_in.mesh == nullptr || head.length_mm <= 0.)
            return true;
        const Vec2d xy = unscale(p);
        const Vec3d at = Vec3f(float(xy.x()), float(xy.y()), float(m_in.slabs[l].bottom_z - m_in.z_offset_mm)).cast<double>();
        return sla::pinhead_mesh_intersect(m_in.mesh->aabb, at, axis.cast<float>().cast<double>().normalized(), double(float(pin_mm)),
                                           head.back_mm, head.length_mm, head.safety_mm)
                   .distance() > head.length_mm;
    }

    // Where a birth tip stands, the axis its neck takes, zero for straight down, and whether the builder's full head fits
    // along that axis. A straight neck always fits, since the builder aims its head along the mesh normal instead.
    struct Spot { Point at; Vec3f axis; bool fits; };

    // Where a tip stands under birth piece `piece` of slab `l`, whose deepest point is `deepest`; `pin_mm` gives the pin
    // radius of a tip at a candidate. The candidates are `deepest` and then the piece's cells, nearest it first. The
    // first candidate whose neck clears straight down wins; failing that, the least lean, in tilt steps up to the cap,
    // at which a candidate's neck clears and its head fits, at the first such candidate along the azimuth whose end
    // stands farthest from the model. Where no lean fits the head, the least lean at which a candidate's neck alone
    // clears, by the same order, which does not fit: the builder's own pose search and its axis retry for a head holding
    // an island may still route a head there. None where no candidate's neck clears.
    std::optional<Spot> search(size_t l, size_t piece, const Point &deepest, const std::function<double(const Point &)> &pin_mm) const
    {
        if (clear(l, deepest, straight_down))
            return Spot { deepest, Vec3f::Zero(), true };
        const LayerGrid   &g     = m_in.material[l];
        const uint16_t     label = uint16_t(piece - m_in.components.slab_range[l].first + 1);
        std::vector<Point> candidates { deepest };
        for (size_t i = 0; i < g.cells.size(); ++ i)
            if (g.cells[i] == label)
                candidates.push_back(centre_of(m_in, g.x0 + int(i % size_t(g.w)), g.y0 + int(i / size_t(g.w))));
        std::stable_sort(candidates.begin() + 1, candidates.end(), [&deepest](const Point &a, const Point &b) {
            return (a - deepest).cast<double>().squaredNorm() < (b - deepest).cast<double>().squaredNorm();
        });
        for (auto it = candidates.begin() + 1; it != candidates.end(); ++ it)
            if (clear(l, *it, straight_down))
                return Spot { *it, Vec3f::Zero(), true };
        std::vector<double> pins(candidates.size(), -1.);
        for (const double tilt : tilts())
            for (size_t i = 0; i < candidates.size(); ++ i) {
                if (pins[i] < 0.)
                    pins[i] = pin_mm(candidates[i]);
                if (const std::optional<Vec3d> axis = lean(l, candidates[i], tilt, pins[i]))
                    return Spot { candidates[i], axis->cast<float>(), true };
            }
        for (const double tilt : tilts())
            for (const Point &c : candidates)
                if (const std::optional<Vec3d> axis = lean(l, c, tilt, std::nullopt))
                    return Spot { c, axis->cast<float>(), false };
        return std::nullopt;
    }

    // The least lean past straight down, in tilt steps up to the cap, at which a neck from `p` on the bottom of slab `l`
    // clears and the head of a pin `pin_mm` in radius fits, or with no pin the neck alone clears, along the azimuth whose
    // end stands farthest from the model; none where no lean does.
    std::optional<Vec3f> leaning(size_t l, const Point &p, std::optional<double> pin_mm) const
    {
        for (const double tilt : tilts())
            if (const std::optional<Vec3d> axis = lean(l, p, tilt, pin_mm))
                return axis->cast<float>().eval();
        return std::nullopt;
    }

    // The axis a neck from `p` on the bottom of slab `l` takes by the same rule, `p` the only candidate: zero where it
    // clears straight down or no lean does. A `birth` tip where no lean fits the head takes the least lean its neck
    // alone clears, as `search` does.
    Vec3f axis_at(size_t l, const Point &p, double pin_mm, bool birth) const
    {
        if (clear(l, p, straight_down))
            return Vec3f::Zero();
        std::optional<Vec3f> axis = leaning(l, p, pin_mm);
        if (! axis && birth)
            axis = leaning(l, p, std::nullopt);
        return axis.value_or(Vec3f::Zero());
    }

private:
    const PlanInput                 &m_in;
    std::vector<std::pair<int, int>> m_wall_disc;   // cells within the xy distance of a wall, by offset
    const double                     m_step;        // the lean that moves a neck's end one cell sideways

    // The first slab whose top reaches `z`, or the slab count past the last.
    size_t end_slab(double z) const
    {
        return size_t(std::lower_bound(m_in.slabs.begin(), m_in.slabs.end(), z,
                                       [](const SupportAnalysis::Slab &slab, double z) { return slab.print_z < z; }) - m_in.slabs.begin());
    }
    static Point along(const Point &p, const Vec3d &axis, double length)
    {
        return p + Point::new_scale(length * axis.x(), length * axis.y());
    }
    // The leans past straight down, one step apart up to the cap, which is the last.
    std::vector<double> tilts() const
    {
        std::vector<double> out;
        if (m_in.max_tilt_rad <= 0. || m_step <= 0.)
            return out;
        for (int i = 1; double(i) * m_step < m_in.max_tilt_rad - EPSILON; ++ i)
            out.push_back(double(i) * m_step);
        out.push_back(m_in.max_tilt_rad);
        return out;
    }
    // Among the azimuths at lean `tilt`, one cell apart at the neck's end, the axis whose neck clears and along which the
    // head of a pin `pin_mm` in radius fits, or with no pin whose neck clears, the one whose end stands farthest from the
    // material of the slab it ends on, the first on a tie; none where no azimuth does. The head test casts rays, so it
    // runs on the clearing axes in that order and stops at the first that fits.
    std::optional<Vec3d> lean(size_t l, const Point &p, double tilt, std::optional<double> pin_mm) const
    {
        const double depth = m_in.neck_depth_mm;
        const int    n     = std::max(1, int(std::ceil(2. * M_PI * depth * std::sin(tilt) / m_in.cell_mm)));
        const size_t k     = end_slab(m_in.slabs[l].bottom_z - depth * std::cos(tilt));
        std::vector<std::pair<double, Vec3d>> clearing;   // (room at the end, axis)
        for (int j = 0; j < n; ++ j) {
            const double phi  = 2. * M_PI * double(j) / double(n);
            const Vec3d  axis(std::sin(tilt) * std::cos(phi), std::sin(tilt) * std::sin(phi), -std::cos(tilt));
            if (clear(l, p, axis))
                clearing.emplace_back(room(k, along(p, axis, depth)), axis);
        }
        std::stable_sort(clearing.begin(), clearing.end(), [](const auto &a, const auto &b) { return a.first > b.first; });
        for (const auto &[r, axis] : clearing)
            if (! pin_mm || head_fits(l, p, axis, *pin_mm))
                return axis;
        return std::nullopt;
    }
    // How far `e` stands from the nearest material cell of slab `k`, up to one neck depth.
    double room(size_t k, const Point &e) const
    {
        const double cap = m_in.neck_depth_mm;
        if (k >= m_in.slabs.size())
            return cap;
        const int        r        = int(std::ceil(cap / m_in.cell_mm));
        const auto       [ex, ey] = cell_of(m_in, e);
        const LayerGrid &g        = m_in.material[k];
        double           near     = cap;
        for (int dy = -r; dy <= r; ++ dy)
            for (int dx = -r; dx <= r; ++ dx)
                if (g.at(ex + dx, ey + dy) != 0)
                    near = std::min(near, (centre_of(m_in, ex + dx, ey + dy) - e).cast<double>().norm() * SCALING_FACTOR);
        return near;
    }
};

// How the birth rule reads birth piece `p` of slab `l` on its own, before a merge settles it. It continues the slab
// below as an overhang does where every point of it lies within that slab's self-support step of the slab's material,
// which `layer` reads as a cell the step carries: a rib stepping out less than that a layer is no island. It is debris
// where it never merges and stands at most `debris_mm`, and a nub where it merges having stood free less than
// `nub_free_mm` and is no wider than two support lines. Any other birth needs a neck.
enum class BirthKind : uint8_t { Overhang, Debris, Nub, Neck };
BirthKind birth_kind(const PlanInput &in, const Births &births, size_t l, size_t p)
{
    const ExPolygon &piece = in.components.pieces[p].polygon;
    if (l > 0 && in.self_support_mm[l] > 0.) {
        BoundingBox box = get_extents(piece);
        box.offset(coord_t(scale_(in.self_support_mm[l])) + 1);
        if (within(piece, ClipperUtils::clip_clipper_polygons_with_subject_bbox(in.slabs[l - 1].polygons, box), in.self_support_mm[l]))
            return BirthKind::Overhang;
    }
    const bool   merges = births.merge_slab[p] < in.slabs.size();
    const double free   = births.free_mm[p];
    if (! merges && free <= debris_mm + EPSILON)
        return BirthKind::Debris;
    if (merges && free < nub_free_mm - EPSILON &&
        unscale<double>(get_extents(piece).size().maxCoeff()) <= 2. * in.toolpath_width_mm + EPSILON)
        return BirthKind::Nub;
    return BirthKind::Neck;
}

// A section that holds what grows on it wherever it stands: the bed under a rooted part, or the section a stem grows
// out of. A point stands from it by its distance in x and y to the outline, 0 inside, and its height over it.
struct Footing
{
    AABBTreeLines::LinesDistancer<Line> outline;
    double                              z = 0.;

    double from(const Vec3d &q) const
    {
        const double xy = std::max(0., outline.distance_from_lines<true>(Point::new_scale(q.x(), q.y())) * SCALING_FACTOR);
        return std::hypot(xy, q.z() - z);
    }
};

// The walk over the object's layers, bottom-up: births first, then the underside runs, then each piece's stability.
class Sweep
{
public:
    explicit Sweep(const PlanInput &in) : m_in(in), m_sets(in.components.pieces.size()),
        m_anchor_z(in.components.pieces.size(), -std::numeric_limits<double>::max()),
        m_rearm_z(in.components.pieces.size(), -std::numeric_limits<double>::max()),
        m_anchors(in.components.pieces.size()), m_waiting(in.components.pieces.size()), m_rooted(in.components.pieces.size(), 0),
        m_slender(in.components.pieces.size(), 0), m_thickness_mm(in.components.pieces.size(), 0.), m_corners(in.slabs.size()),
        m_necks(in), m_flaps(0), m_reach_mm(reach_lines * in.toolpath_width_mm)
    {
        m_births = walk_births(in);
        tbb::parallel_for(tbb::blocked_range<size_t>(0, in.slabs.size()), [&](const tbb::blocked_range<size_t> &range) {
            for (size_t l = range.begin(); l < range.end(); ++ l)
                for (size_t p = in.components.slab_range[l].first; p < in.components.slab_range[l].second; ++ p)
                    m_thickness_mm[p] = 2. * inscribed_radius(in.components.pieces[p].polygon);
        });
        // Cells under a small head and within two lines, by offset.
        m_head_disc  = disc(in.toolpath_width_mm / in.cell_mm);
        m_two_lines = disc(m_reach_mm / in.cell_mm);
        // A small head's own disc, pi w^2, in cells.
        m_flap_floor = size_t(std::lround(M_PI * in.toolpath_width_mm * in.toolpath_width_mm / (in.cell_mm * in.cell_mm)));
        if (const ExPolygons &bed = in.slabs.front().polygons; ! bed.empty())
            m_bed = std::make_unique<Footing>(Footing { AABBTreeLines::LinesDistancer<Line>(to_lines(bed)), in.slabs.front().bottom_z });
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
    Births                   m_births;
    DisjointSets             m_sets;       // pieces joined as the walk climbs, for stability
    std::vector<double>      m_anchor_z;   // per root: its highest anchor, a tip or a birth
    // Per piece: where the last stability search of its branch ended short, the highest over the pieces under it.
    std::vector<double>      m_rearm_z;
    struct Anchor { Vec3d at; size_t tip; };   // a tip's point and its index in the plan
    std::vector<std::vector<Anchor>> m_anchors;   // per root: its tips
    // Per root: the births that took no tip on the understanding that the part they merge into holds them, as indices
    // into the plan's islands. The merge settles them.
    std::vector<std::vector<size_t>> m_waiting;
    // Per island of the plan: its birth slab and piece, and the piece's deepest point.
    struct BirthPiece { size_t slab, piece; Point deepest; };
    std::vector<BirthPiece>  m_island_pieces;
    // Per root: whether the bed holds its part. Per piece: whether its branch has been counted slender, set where any
    // piece under it was.
    std::vector<char>        m_rooted, m_slender;
    std::vector<double>      m_thickness_mm;   // per piece: its largest inscribed circle's diameter
    std::unique_ptr<Footing> m_bed;            // the first slab's outline, which holds every rooted part
    // A corner of a slab's down-facing surface that steps past the slab below: the piece of the slab holding it, and
    // its slice angle, how far from straight down the face turns there over the two slabs under it. `ok` is whether a
    // tip may stand there with its neck straight down and its head within the face cap, and `leans` whether one may
    // with its neck leaning along `axis`, which stays zero on a corner whose neck clears straight down; each is -1
    // until read.
    struct Corner { Point p; size_t piece; double slice_deg; int8_t ok = -1, leans = -1; Vec3f axis = Vec3f::Zero(); };
    std::vector<std::optional<std::vector<Corner>>> m_corners;   // per slab, read on first use
    Necks                    m_necks;
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
    Point centre(int x, int y) const { return centre_of(m_in, x, y); }
    std::pair<int, int> cell_of(const Point &p) const { return ScaffoldSupport::cell_of(m_in, p); }
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
            for (size_t q : piece.below) {
                if (const size_t r = m_sets.find(q); std::find(roots.begin(), roots.end(), r) == roots.end())
                    roots.push_back(r);
                m_rearm_z[p] = std::max(m_rearm_z[p], m_rearm_z[q]);
                m_slender[p] = m_slender[p] || m_slender[q];
            }
            if (roots.size() > 1)
                settle(l, p, roots);
            const size_t root = roots.front();
            for (size_t i = 1; i < roots.size(); ++ i) {
                const size_t r = roots[i];
                m_anchor_z[root] = std::max(m_anchor_z[root], m_anchor_z[r]);
                append(m_anchors[root], std::move(m_anchors[r]));
                m_rooted[root] = m_rooted[root] || m_rooted[r];
                m_sets.join(root, r);
            }
            m_sets.join(root, p);
        }
    }

    // Parts meeting at piece `q` of slab `s`: each nub that waited for this merge is settled, lowest first. A nub whose
    // own part holds a tip, an underside head its layer placed, is held by that tip. One lying wholly within the merge
    // slab's step plus the reach of what the held parts, those a tip or the bed holds, have under the merge hangs from
    // their tips: the merge layer bridges no farther to it than to any underside the planner leaves bare. Any other nub
    // takes its birth tip now, which holds its part for the nubs after it and takes the pending cells under its cover
    // unless its head does not fit, `place_unanchored`, or counts as unheld where no neck clears.
    void settle(size_t s, size_t q, const std::vector<size_t> &roots)
    {
        std::vector<std::pair<size_t, size_t>> waiting;   // (island, its part's root)
        for (const size_t r : roots) {
            for (const size_t k : m_waiting[r])
                waiting.emplace_back(k, r);
            m_waiting[r].clear();
        }
        std::sort(waiting.begin(), waiting.end());
        const std::function<bool(size_t)> held = [this](size_t r) { return m_rooted[r] || ! m_anchors[r].empty(); };
        const auto tips = [this](size_t r) {
            std::vector<size_t> out;
            for (const Anchor &anchor : m_anchors[r])
                out.push_back(anchor.tip);
            return out;
        };
        const double hang = hang_mm(m_in, s);
        for (const auto &[k, r] : waiting) {
            Island           &island = m_plan.islands[k];
            const BirthPiece &birth  = m_island_pieces[k];
            const ExPolygon  &nub    = pieces()[birth.piece].polygon;
            const double      z      = pieces()[birth.piece].bottom_z;
            if (! m_anchors[r].empty()) {
                BOOST_LOG_TRIVIAL(debug) << "scaffold island at " << z << " held by its own tip";
                island.holders = tips(r);
                island.rooted  = m_rooted[r];
                island.reason  = IslandReason::Tip;
                continue;
            }
            std::vector<size_t> holding;
            if (const Polygons under = footing(m_in, m_sets, q, r, nub, hang, held, holding); ! under.empty() && within(nub, under, hang)) {
                BOOST_LOG_TRIVIAL(debug) << "scaffold island at " << z << " hangs from a held part";
                island.holders.clear();
                island.rooted = false;
                for (const size_t rb : holding) {
                    append(island.holders, tips(rb));
                    island.rooted = island.rooted || m_rooted[rb];
                }
                island.reason = IslandReason::Hung;
                continue;
            }
            if (const std::optional<Necks::Spot> spot =
                    m_necks.search(birth.slab, birth.piece, birth.deepest, birth_pin(birth.slab, birth.piece))) {
                BOOST_LOG_TRIVIAL(debug) << "scaffold island at " << z << " joins nothing held near it: tip";
                TipSite site = site_at(birth.slab, spot->at, birth_grade(m_in, m_births, birth.slab, birth.piece, spot->at));
                site.axis    = spot->axis;
                island.holders = { spot->fits ? place(site, TipNeed::Birth, nullptr) : place_unanchored(site) };
                island.reason  = IslandReason::Tip;
                if (spot->fits) {
                    const auto [x, y] = cell_of(site.position);
                    take_pending(x, y, float(site.print_z), cover_mm(birth.slab) / m_in.cell_mm);
                }
            } else {
                island.reason = IslandReason::NoNeck;
                ++ m_plan.islands_unheld;
            }
        }
    }

    // Whether a tip may stand at `p` on object layer `l` with its neck straight down.
    bool eligible(size_t l, const Point &p) const { return m_necks.clear(l, p, straight_down); }

    // Whether the face a head's pin meets at `p` on the bottom of slab `l` stands within the face cap of straight down:
    // its normal as `sla::normals` reads it at the point at the head's radius, one toolpath width, the normal of the
    // face nearest the point, or where that nearest point lies within the radius of a vertex or an edge, the average of
    // the faces sharing it. The builder aims the head of a tip with a zero axis along that normal, and one with an axis
    // along the axis, whose pin still meets that face. The cap holds its own angle: on a square edge where a 45 degree
    // face meets an upright one the average reads the cap exactly, and float rounding must not decide it.
    bool faces_down(size_t l, const Point &p) const
    {
        if (m_in.mesh == nullptr)
            return true;
        const Vec2d   xy = unscale(p);
        sla::PointSet at(1, 3);
        at.row(0) = Vec3d(xy.x(), xy.y(), m_in.slabs[l].bottom_z - m_in.z_offset_mm);
        const sla::PointSet normal = sla::normals(at, m_in.mesh->aabb, m_in.toolpath_width_mm);
        return normal.rows() == 0 || -normal(0, 2) >= std::cos(face_cap_deg * M_PI / 180.) - EPSILON;
    }

    // The root of the piece of slab `l` holding `p`, or npos.
    size_t root_at(size_t l, const Point &p)
    {
        const size_t q = piece_at(m_in, l, p);
        return q == size_t(-1) ? q : m_sets.find(q);
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

    // A birth tip whose full head does not fit along its lean holds its own island and nothing else, since the builder
    // routes few such tips: it anchors no part and no cell and takes no pending cells, so the walk reads its island as one
    // no neck clears. A nub meeting its part takes its own tip, and a tip the builder cannot route leaves every other hold
    // where the plan would place it with no tip there. Returns the tip's index in the plan.
    size_t place_unanchored(const TipSite &site)
    {
        m_plan.tips.push_back({ site, TipNeed::Birth });
        return m_plan.tips.size() - 1;
    }

    TipSite site_at(size_t l, const Point &p, double grade_mm) const
    {
        TipSite site { p, m_in.slabs[l].bottom_z, int(l) - 1 };
        site.grade_mm = grade_mm;
        return site;
    }

    // The pin radius of a birth tip at `p` under piece `piece` of slab `l`, half its `birth_grade`.
    std::function<double(const Point &)> birth_pin(size_t l, size_t piece) const
    {
        return [this, l, piece](const Point &p) { return 0.5 * birth_grade(m_in, m_births, l, piece, p); };
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
        const double     cover  = cover_mm(l);
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
            const size_t answered = size_t(std::count_if(far_due.begin(), far_due.end(), [&](size_t i) { return run_of(i) <= reach; })) +
                                    take_pending(bx, by, z, cover_cells);
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

    // How far a head on slab `l` covers: the reach plus the slab's step, as `m_step` holds it, plus one toolpath width.
    double cover_mm(size_t l) const { return m_reach_mm + double(float(m_in.self_support_mm[l])) + m_in.toolpath_width_mm; }

    // Takes the pending cells within `cover_cells` in 3-D of a head at lattice cell (x, y) at height `z`, and returns
    // how many of them hung past one and a half reaches.
    size_t take_pending(int x, int y, float z, double cover_cells)
    {
        const float far   = unmet_run();
        size_t      taken = 0;
        m_pending.erase(std::remove_if(m_pending.begin(), m_pending.end(), [&](const Pending &q) {
            const double dx = double(q.x - x), dy = double(q.y - y), dz = double(z - q.z) / m_in.cell_mm;
            if (dx * dx + dy * dy + dz * dz > cover_cells * cover_cells)
                return false;
            taken += q.run > far;
            return true;
        }), m_pending.end());
        return taken;
    }

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

    // Each island starting on slab `l` is read by the birth rule, unless an enforced tip already stands on its birth
    // piece, among `enforced` the plan's tips on the slab. A piece `birth_kind` reads as continuing the slab below is no
    // island. Debris takes no tip. A nub prints its layer as it hangs and waits for its merge, which `settle` reads; its
    // cells hang from the nearest other material on their layer, what the merge bridges them from, so one standing far
    // from it can take an underside head on its own layer. Any other island takes one tip where `Necks::search` finds a
    // neck, graded by `birth_grade`, and counts as unheld where none clears. An island left without a tip otherwise, or
    // held only by a tip whose head does not fit, `place_unanchored`, prints as it hangs, so its cells anchor what grows
    // on them. Every island goes into the plan with how it is held.
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
        // A nub's cells start at their distance to the nearest other material on the layer, up to where a cell is due
        // however far that stands.
        const auto hang = [&](size_t p) {
            const uint16_t label = uint16_t(p - first + 1);
            const double   cap   = double(reach_run()) + double(m_step) + m_in.cell_mm;
            const int      r     = int(std::ceil(cap / m_in.cell_mm));
            for (size_t i = 0; i < g.cells.size(); ++ i) {
                if (g.cells[i] != label)
                    continue;
                const int x = g.x0 + int(i % size_t(g.w)), y = g.y0 + int(i / size_t(g.w));
                double    near = cap;
                for (int dy = -r; dy <= r; ++ dy)
                    for (int dx = -r; dx <= r; ++ dx)
                        if (const uint16_t other = g.at(x + dx, y + dy); other != 0 && other != label)
                            near = std::min(near, std::sqrt(double(dx * dx + dy * dy)) * m_in.cell_mm);
                if (float(near) < m_dist[i]) {
                    m_dist[i] = float(near);
                    heap.emplace(m_dist[i], i);
                }
            }
        };
        for (size_t p = first; p < last; ++ p) {
            const SupportAnalysis::Piece &piece = pieces()[p];
            if (! piece.below.empty() || m_rooted[p])
                continue;
            const auto held = std::find_if(enforced.begin(), enforced.end(),
                                           [&](size_t t) { return piece.polygon.contains(m_plan.tips[t].site.position); });
            const BirthKind kind = held != enforced.end() ? BirthKind::Neck : birth_kind(m_in, m_births, l, p);
            if (kind == BirthKind::Overhang) {
                BOOST_LOG_TRIVIAL(debug) << "scaffold island skipped at " << piece.bottom_z << ": continues the layer below";
                accept(p);
                continue;
            }
            const Point  deepest = inscribed_point(piece.polygon);
            const Vec2d  xy      = unscale(deepest);
            const size_t k       = m_plan.islands.size();
            m_plan.islands.push_back({ Vec3d(xy.x(), xy.y(), m_in.slabs[l].bottom_z) });
            m_island_pieces.push_back({ l, p, deepest });
            Island &island = m_plan.islands.back();
            if (held != enforced.end()) {
                island.holders = { *held };
                island.reason  = IslandReason::Tip;
                continue;
            }
            if (kind == BirthKind::Debris) {
                BOOST_LOG_TRIVIAL(debug) << "scaffold island skipped at " << piece.bottom_z << ": debris";
                island.reason = IslandReason::Debris;
                accept(p);
                continue;
            }
            if (kind == BirthKind::Nub) {
                BOOST_LOG_TRIVIAL(debug) << "scaffold island at " << piece.bottom_z << " waits for its merge: nub";
                island.reason = IslandReason::Hung;
                m_waiting[p].push_back(k);
                hang(p);
                continue;
            }
            const std::optional<Necks::Spot> spot = m_necks.search(l, p, deepest, birth_pin(l, p));
            if (! spot) {
                BOOST_LOG_TRIVIAL(debug) << "scaffold island at " << piece.bottom_z << ": no neck";
                island.reason = IslandReason::NoNeck;
                ++ m_plan.islands_unheld;
                accept(p);
                continue;
            }
            TipSite site = site_at(l, spot->at, birth_grade(m_in, m_births, l, p, spot->at));
            site.axis    = spot->axis;
            if (! site.axis.isZero())
                BOOST_LOG_TRIVIAL(debug) << "scaffold island at " << piece.bottom_z << " leans its neck "
                                         << std::acos(std::clamp(-double(site.axis.z()), -1., 1.)) * 180. / M_PI << " degrees";
            island.reason = IslandReason::Tip;
            if (spot->fits)
                island.holders = { place(site, TipNeed::Birth, &heap) };
            else {
                island.holders = { place_unanchored(site) };
                accept(p);
            }
        }
    }

    // How far `q` stands from what holds part `r`: its nearest tip in 3-D or the nearest of `footings`, or, for a part
    // with neither, how high `q` stands over its highest anchor. The tips are read only until one stands within
    // `enough`, whose distance is then returned: the caller only asks whether `q` stands farther than that.
    double from_anchors(size_t r, const Vec3d &q, const std::vector<const Footing *> &footings,
                        double enough = -std::numeric_limits<double>::max()) const
    {
        if (m_anchors[r].empty() && footings.empty())
            return q.z() - m_anchor_z[r];
        double near = std::numeric_limits<double>::max();
        for (const Anchor &anchor : m_anchors[r])
            if ((near = std::min(near, (anchor.at - q).norm())) <= enough)
                return near;
        for (const Footing *footing : footings)
            near = std::min(near, footing->from(q));
        return near;
    }
    // The lever the nozzle works piece `p` of part `r` by at its top `z`: how far the farthest point of its outline
    // stands from what holds the part, and that point's index.
    std::pair<double, size_t> lever(size_t r, size_t p, double z, const std::vector<const Footing *> &footings) const
    {
        const Points &outline = pieces()[p].polygon.contour.points;
        double        far     = -std::numeric_limits<double>::max();
        size_t        at      = 0;
        for (size_t i = 0; i < outline.size(); ++ i)
            if (const double d = from_anchors(r, Vec3d(unscale<double>(outline[i].x()), unscale<double>(outline[i].y()), z), footings, far);
                d > far) {
                far = d;
                at  = i;
            }
        return { far, at };
    }

    // Extends `stem`, per slab from slab `l` down the pieces a piece of `l` reaches through the pieces under each,
    // sorted, until it holds the slab `depth` slabs under `l` or the pieces all start.
    void grow_stem(std::vector<std::vector<size_t>> &stem, size_t l, size_t depth) const
    {
        while (stem.size() <= depth && stem.size() <= l) {
            std::vector<size_t> next;
            for (const size_t q : stem.back())
                append(next, pieces()[q].below);
            if (next.empty())
                return;
            sort_remove_duplicates(next);
            stem.push_back(std::move(next));
        }
    }

    // The corners of slab `k`'s down-facing surface, read once per slab: each vertex of a face's outline that steps
    // past slab k-1, with the piece holding it and its slice angle, atan of the two slabs' rise over its distance to
    // slab k-2.
    std::vector<Corner> &corners(size_t k)
    {
        std::optional<std::vector<Corner>> &out = m_corners[k];
        if (out)
            return *out;
        out.emplace();
        if (k == 0 || m_in.down_facing[k].empty())
            return *out;
        using Distancer = AABBTreeLines::LinesDistancer<Line>;
        const size_t    two = k >= 2 ? k - 2 : 0;
        const Distancer below(to_lines(m_in.slabs[k - 1].polygons)), under(to_lines(m_in.slabs[two].polygons));
        const auto      away = [](const Distancer &lines, const Point &p) {
            return lines.get_lines().empty() ? std::numeric_limits<double>::max() :
                                               std::max(0., lines.distance_from_lines<true>(p) * SCALING_FACTOR);
        };
        const double rise = m_in.slabs[k].bottom_z - m_in.slabs[two].bottom_z;
        for (const ExPolygon &face : m_in.down_facing[k])
            if (const size_t piece = piece_at(m_in, k, face.contour.points.front()); piece != size_t(-1))
                for (const Point &p : face.contour.points)
                    if (away(below, p) > EPSILON)
                        out->push_back({ p, piece, std::atan2(rise, away(under, p)) * 180. / M_PI });
        return *out;
    }

    // Each piece of slab `l` is measured against its own window, the larger of `stability_min_mm` and `stability_ratio`
    // times its stem's thickness: the inscribed diameter of the piece or, where thicker, of what it grows out of
    // `stability_retry_mm` under its top. Its lever is how far the farthest point of its outline at its top stands from
    // what holds its part: the part's tips, the bed under a rooted part and, no deeper than the lever, the first section
    // the stem grows out of that is `junction_ratio` times as thick, which clamps it. A piece whose lever passes its
    // window takes a small tip at a corner of its stem's down-facing surface above that section and within the window
    // under its top, at least half the window from what holds the part, since a tip beside an anchor shortens no lever,
    // and within the window of the farthest point. The corner whose face turns least from straight down over the two
    // slabs under it goes first, then the one farthest from what holds the part, and it must be eligible with its face
    // within the face cap, `faces_down`. Where no corner both clears straight down and faces within the cap, the first
    // corner in that order whose face stands within the cap and whose neck clears leaning with the builder's full head
    // fitting takes the least such lean, and hands it to the builder as its axis. Unlike a birth, a corner whose neck
    // clears only along leans the head does not fit takes no tip: a stability tip holds its part only as an anchor, and
    // one the builder cannot route would still displace the holds the plan places after it. A branch with no such corner
    // counts slender once. A piece with no such corner, or still past its window with the new tip, is measured again
    // `stability_retry_mm` higher.
    void stability(size_t l)
    {
        const auto [first, last] = m_in.components.slab_range[l];
        const double top   = m_in.slabs[l].print_z;
        const double grade = 2. * m_in.toolpath_width_mm;   // the small disc a stability tip fuses with
        for (size_t p = first; p < last; ++ p) {
            if (top - m_rearm_z[p] <= stability_retry_mm + EPSILON)
                continue;
            const size_t                 r = m_sets.find(p);
            std::vector<const Footing *> footings;
            if (m_rooted[r] && m_bed)
                footings.push_back(m_bed.get());
            double lever_mm;
            size_t far_at;
            std::tie(lever_mm, far_at) = lever(r, p, top, footings);
            if (lever_mm <= stability_min_mm + EPSILON)
                continue;
            std::vector<std::vector<size_t>> stem { { p } };
            size_t below = 0;
            while (below < l && m_in.slabs[l - below].print_z > top - stability_retry_mm + EPSILON)
                ++ below;
            grow_stem(stem, l, below);
            below = std::min(below, stem.size() - 1);
            double thickness = m_thickness_mm[p];
            for (const size_t q : stem[below])
                thickness = std::max(thickness, m_thickness_mm[q]);
            const double window = std::max(stability_min_mm, stability_ratio * thickness);
            if (lever_mm <= window + EPSILON)
                continue;
            size_t deep = below;
            while (deep < l && m_in.slabs[l - deep - 1].print_z > top - lever_mm + EPSILON)
                ++ deep;
            grow_stem(stem, l, deep);
            std::optional<Footing> junction;
            size_t                 junction_at = stem.size();
            for (size_t i = below + 1; i < stem.size() && ! junction; ++ i)
                if (std::any_of(stem[i].begin(), stem[i].end(), [&](size_t q) { return m_thickness_mm[q] >= junction_ratio * thickness; })) {
                    ExPolygons section;
                    for (const size_t q : stem[i])
                        section.push_back(pieces()[q].polygon);
                    junction.emplace(Footing { AABBTreeLines::LinesDistancer<Line>(to_lines(section)), m_in.slabs[l - i].print_z });
                    footings.push_back(&*junction);
                    junction_at = i;
                    std::tie(lever_mm, far_at) = lever(r, p, top, footings);
                }
            if (lever_mm <= window + EPSILON)
                continue;
            const Vec2d far_xy   = unscale(pieces()[p].polygon.contour.points[far_at]);
            const Vec3d far(far_xy.x(), far_xy.y(), top);
            // A part held by nothing but its birth reads every point of the outline at its height alike, so none is the
            // farthest for a corner to stand near.
            const bool  held_far = ! m_anchors[r].empty() || ! footings.empty();
            struct Candidate { Corner *corner; size_t slab; double from; };
            std::vector<Candidate> candidates;
            for (size_t i = 0; i < std::min(junction_at, stem.size()); ++ i) {
                const size_t k = l - i;
                if (k == 0 || m_in.slabs[k].bottom_z <= top - window - EPSILON)
                    break;
                for (Corner &corner : corners(k)) {
                    if (! std::binary_search(stem[i].begin(), stem[i].end(), corner.piece))
                        continue;
                    const Vec2d xy = unscale(corner.p);
                    const Vec3d q(xy.x(), xy.y(), m_in.slabs[k].bottom_z);
                    if (held_far && (q - far).norm() > window + EPSILON)
                        continue;
                    if (const double from = from_anchors(r, q, footings); from >= 0.5 * window - EPSILON)
                        candidates.push_back({ &corner, k, from });
                }
            }
            // The slices read a flat face's angle only to float rounding, so corners order by the whole degree.
            std::stable_sort(candidates.begin(), candidates.end(), [](const Candidate &a, const Candidate &b) {
                const long da = std::lround(a.corner->slice_deg), db = std::lround(b.corner->slice_deg);
                return da != db ? da < db : a.from > b.from;
            });
            auto spot = std::find_if(candidates.begin(), candidates.end(), [this](const Candidate &c) {
                if (c.corner->ok < 0)
                    c.corner->ok = eligible(c.slab, c.corner->p) && faces_down(c.slab, c.corner->p);
                return c.corner->ok != 0;
            });
            if (spot == candidates.end())
                spot = std::find_if(candidates.begin(), candidates.end(), [this, grade](const Candidate &c) {
                    if (c.corner->leans < 0) {
                        const std::optional<Vec3f> axis =
                            faces_down(c.slab, c.corner->p) ? m_necks.leaning(c.slab, c.corner->p, 0.5 * grade) : std::nullopt;
                        c.corner->leans = axis.has_value();
                        c.corner->axis  = axis.value_or(Vec3f::Zero());
                    }
                    return c.corner->leans != 0;
                });
            if (spot == candidates.end()) {
                if (! m_slender[p])
                    ++ m_plan.islands_slender;
                m_slender[p] = 1;
                BOOST_LOG_TRIVIAL(debug) << "scaffold part slender at " << top << ": lever " << lever_mm << " mm past its window of "
                                         << window << " mm, no face to hold";
                m_rearm_z[p] = top;
                continue;
            }
            TipSite site = site_at(spot->slab, spot->corner->p, grade);
            site.axis    = spot->corner->axis;
            place(site, TipNeed::Stability, nullptr);
            BOOST_LOG_TRIVIAL(debug) << "scaffold stability tip at " << m_in.slabs[spot->slab].bottom_z << " for the section at " << top
                                     << ": lever " << lever_mm << " mm past its window of " << window << " mm, face "
                                     << spot->corner->slice_deg << " degrees from down, neck leaning "
                                     << (site.axis.isZero() ? 0. : std::acos(std::clamp(-double(site.axis.z()), -1., 1.)) * 180. / M_PI)
                                     << " degrees";
            if (lever(r, p, top, footings).first > window + EPSILON)
                m_rearm_z[p] = top;
        }
    }
};

} // namespace

Point inscribed_point(const ExPolygon &piece)
{
    const ExPolygons deepest = deepest_offset(piece).second;
    const Point      middle  = deepest.front().contour.centroid();
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

Vec3f neck_axis(const PlanInput &input, const TipSite &site)
{
    const int l = site.obj_layer_nr + 1;
    if (input.slabs.empty() || input.cell <= 0 || l < 0 || size_t(l) >= input.slabs.size())
        return Vec3f::Zero();
    // The pin `draw` gives the tip: half its grade, or of the small disc where it has none.
    const double pin_mm = 0.5 * (site.grade_mm > 0. ? site.grade_mm : 2. * input.toolpath_width_mm);
    // A site on a birth piece off the bed holds its island as the planner's birth tip there does, so it leans as one. An
    // input with no pieces reads none.
    bool birth = false;
    if (l > 0 && size_t(l) < input.components.slab_range.size())
        if (const size_t q = piece_at(input, size_t(l), site.position); q != size_t(-1))
            birth = input.components.pieces[q].below.empty();
    return Necks(input).axis_at(size_t(l), site.position, pin_mm, birth);
}

std::vector<BirthRead> read_births(const PlanInput &input, const std::vector<size_t> &pieces, const std::vector<TipSite> &tips)
{
    std::vector<BirthRead> out(pieces.size());
    if (input.slabs.empty() || input.cell <= 0)
        return out;
    const Births births = walk_births(input);
    const Necks  necks(input);
    const auto  &ranges  = input.components.slab_range;
    const auto  &all     = input.components.pieces;
    const auto   slab_of = [&ranges](size_t p) {
        return size_t(std::upper_bound(ranges.begin(), ranges.end(), p,
                                       [](size_t p, const std::pair<size_t, size_t> &range) { return p < range.first; }) - ranges.begin()) - 1;
    };
    // The tip the neck search stands under birth `i`, if any clears, and whether the builder's full head fits there.
    const auto tip = [&](size_t i) {
        const size_t p = pieces[i], l = slab_of(p);
        const auto pin = [&](const Point &c) { return 0.5 * birth_grade(input, births, l, p, c); };
        const auto spot = necks.search(l, p, inscribed_point(all[p].polygon), pin);
        if (spot) {
            out[i].hold      = BirthHold::Tip;
            out[i].site      = { spot->at, input.slabs[l].bottom_z, int(l) - 1 };
            out[i].site.axis = spot->axis;
        }
        return spot && spot->fits;
    };
    std::vector<std::vector<size_t>> waiting(all.size());   // per root: its nubs, as indices into `pieces`
    bool                             nubs = false;
    for (size_t i = 0; i < pieces.size(); ++ i)
        switch (birth_kind(input, births, slab_of(pieces[i]), pieces[i])) {
        case BirthKind::Overhang: out[i].hold = BirthHold::Overhang; break;
        case BirthKind::Debris: out[i].hold = BirthHold::Debris; break;
        case BirthKind::Nub: waiting[pieces[i]].push_back(i); nubs = true; break;
        case BirthKind::Neck: tip(i); break;
        }
    if (! nubs)
        return out;
    // The nubs settle at their merges bottom up as the plan's `settle` settles them, a part held where the bed roots it
    // or one of `tips` stands on it under the merge slab. A nub the rule tips holds its part for the nubs after it, as
    // the tip the plan would place there would, unless that tip's full head does not fit, which the plan anchors nowhere.
    std::vector<std::vector<size_t>> on_piece(all.size());   // per piece: the tips standing on it
    for (size_t t = 0; t < tips.size(); ++ t)
        if (const int l = tips[t].obj_layer_nr + 1; l >= 0 && size_t(l) < input.slabs.size())
            if (const size_t q = piece_at(input, size_t(l), tips[t].position); q != size_t(-1))
                on_piece[q].push_back(t);
    // Per root: whether a tip, the bed or the rule's tip holds its part, whether the bed does, and the tips on it.
    std::vector<char>                held(all.size(), 0), rooted(all.size(), 0);
    std::vector<std::vector<size_t>> points(all.size());
    DisjointSets                      sets(all.size());
    const std::function<bool(size_t)> holds = [&held](size_t r) { return held[r] != 0; };
    for (size_t s = 0; s < input.slabs.size(); ++ s) {
        for (size_t q = ranges[s].first; q < ranges[s].second; ++ q) {
            const SupportAnalysis::Piece &piece = all[q];
            if (piece.below.empty()) {
                held[q] = rooted[q] = piece.bottom_z <= input.slabs.front().bottom_z + EPSILON;
                continue;
            }
            std::vector<size_t> roots;
            for (const size_t b : piece.below)
                if (const size_t r = sets.find(b); std::find(roots.begin(), roots.end(), r) == roots.end())
                    roots.push_back(r);
            if (roots.size() > 1) {
                std::vector<std::pair<size_t, size_t>> due;   // (birth piece, index into `pieces`)
                for (const size_t r : roots) {
                    for (const size_t i : waiting[r])
                        due.emplace_back(pieces[i], i);
                    waiting[r].clear();
                }
                std::sort(due.begin(), due.end());
                const double hang = hang_mm(input, s);
                for (const auto &[p, i] : due) {
                    const size_t        r = sets.find(p);
                    std::vector<size_t> holding;
                    if (held[r]) {
                        out[i].hold = BirthHold::Nub;
                        holding     = { r };
                    } else if (const Polygons under = footing(input, sets, q, r, all[p].polygon, hang, holds, holding);
                               ! under.empty() && within(all[p].polygon, under, hang)) {
                        out[i].hold = BirthHold::Nub;
                    } else {
                        held[r] = tip(i);
                        continue;
                    }
                    for (const size_t rb : holding) {
                        append(out[i].holders, points[rb]);
                        out[i].rooted = out[i].rooted || rooted[rb];
                    }
                }
            }
            const size_t root = roots.front();
            for (size_t k = 1; k < roots.size(); ++ k) {
                held[root]   = held[root] || held[roots[k]];
                rooted[root] = rooted[root] || rooted[roots[k]];
                append(points[root], std::move(points[roots[k]]));
                sets.join(root, roots[k]);
            }
            sets.join(root, q);
        }
        // A tip on this slab holds its part from the next merge on, as the plan places a slab's tips after its merges.
        for (size_t q = ranges[s].first; q < ranges[s].second; ++ q)
            if (! on_piece[q].empty()) {
                const size_t r = sets.find(q);
                held[r]        = 1;
                append(points[r], on_piece[q]);
            }
    }
    return out;
}

Plan plan_tips(const PlanInput &input, const std::vector<TipSite> &enforced)
{
    if (input.slabs.empty() || input.cell <= 0)
        return {};
    return Sweep(input).run(enforced);
}

} // namespace Slic3r::ScaffoldSupport
