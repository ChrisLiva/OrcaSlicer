#include "MiniatureSupport.hpp"

#include "../ClipperUtils.hpp"
#include "../Geometry/ConvexHull.hpp"
#include "../Polygon.hpp"

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>

namespace Slic3r {
namespace MiniatureSupport {

namespace {

// The ground one contact patch covers, tessellated once per contact: the repository's own circle at
// the geometry tolerance, drawn around the origin so a candidate position is a translation of it.
Polygon contact_patch(double radius_mm)
{
    const double radius = scaled<double>(radius_mm);
    return radius > 0. ? make_circle(radius, double(SCALED_EPSILON)) : Polygon();
}

// How many mm2 of `region` that patch covers when it is centred on `position`. Material the region
// does not hold is not this region's to be damaged.
double patch_area_mm2(const ExPolygons &region, const Polygon &patch, const Point &position)
{
    if (patch.points.size() < 3)
        return 0.;
    Polygon moved = patch;
    moved.translate(position);
    double area = 0.;
    for (const ExPolygon &piece : intersection_ex(region, ExPolygons{ ExPolygon(moved) }))
        area += piece.area();
    return area * SCALING_FACTOR * SCALING_FACTOR;
}

// How one reading ranks against another as somewhere to stand. A printable measurement beats one
// below the printable width, which is still a measurement and beats no reading at all. Ranking first
// keeps an unmeasured position, whose weight is not a number about anything, from ever reading as
// cheaper than measured ground.
int standing_rank(const ModelSupportRisk::Sample &sample)
{
    switch (sample.status) {
    case ModelSupportRisk::Sample::Status::Known:               return 2;
    case ModelSupportRisk::Sample::Status::BelowPrintableWidth: return 1;
    default:                                                    return 0;
    }
}

// Whether a contact at `position` still holds every one of `required`, which is a lattice cut down to
// the cells one contact is the only cover for.
bool covers_all(const ExPolygon &polygon, const Witnesses &required, const Point &position, double reach_mm)
{
    if (required.empty())
        return true;
    const std::vector<bool> held = covered_by_contact(polygon, required, position, reach_mm);
    return std::find(held.begin(), held.end(), false) == held.end();
}

// Moves what a thinning pass kept onto the least damaging model section each contact can legally
// reach, updating `position`, `cover` and `cover_count` in step so the assignment that follows reads
// the ground the contacts actually ended up on. Runs region by region and, inside a region, in source
// id order: the greedy order is the problem's own.
void relocate_contacts(const Problem &problem, const ModelSupportRisk::Field &risk,
                       const std::vector<std::vector<size_t>> &seeds_of_region, const std::vector<char> &kept,
                       std::vector<std::vector<bool>> &cover, std::vector<std::vector<int>> &cover_count,
                       std::vector<Point> &position)
{
    for (size_t r = 0; r < problem.regions.size(); ++ r) {
        const RequiredRegion &region = problem.regions[r];
        if (seeds_of_region[r].empty() || region.legal_reach_mm <= 0.)
            continue;
        const auto      lattice_ptr = region_witnesses(region, problem.extrusion_width_mm);
        const Witnesses &lattice    = *lattice_ptr;
        if (lattice.empty())
            continue;
        const ExPolygons region_polys{ region.polygon };

        // What this region offers to stand on: the centre of every lattice square the region
        // itself holds. The lattice is the frozen one the witnesses are counted on, so the
        // positions are a function of the problem and nothing else.
        std::vector<Point> candidates;
        candidates.reserve(lattice.size());
        for (const WitnessCell &cell : lattice.cells) {
            const Point centre(cell.index.x() * lattice.pitch + lattice.pitch / 2,
                               cell.index.y() * lattice.pitch + lattice.pitch / 2);
            if (region.polygon.contains(centre))
                candidates.push_back(centre);
        }
        if (candidates.empty())
            continue;
        // One contact to a position: two anchors collapsed onto one point anchor once.
        std::vector<char> occupied(candidates.size(), 0);
        for (size_t s : seeds_of_region[r])
            if (kept[s])
                for (size_t k = 0; k < candidates.size(); ++ k)
                    if (candidates[k] == position[s])
                        occupied[k] = 1;

        const std::vector<size_t> &seeds  = seeds_of_region[r];
        const double               reach2 = scaled<double>(region.legal_reach_mm) * scaled<double>(region.legal_reach_mm);

        // Which positions the decisions below can ask about at all: a candidate no contact of this
        // region may legally reach, and one another contact already stands on, is never read. A
        // reading costs a search over the whole risk field, so the ones that will not be read are
        // worth this much arithmetic to leave out.
        std::vector<char> reachable(candidates.size(), 0);
        for (const size_t s : seeds) {
            if (! kept[s] || problem.seeds[s].pinned)
                continue;
            for (size_t k = 0; k < candidates.size(); ++ k) {
                if (occupied[k] || reachable[k])
                    continue;
                const double dx = double(candidates[k].x() - problem.seeds[s].position.x());
                const double dy = double(candidates[k].y() - problem.seeds[s].position.y());
                if (dx * dx + dy * dy <= reach2)
                    reachable[k] = 1;
            }
        }
        std::vector<size_t> wanted;
        for (size_t k = 0; k < candidates.size(); ++ k)
            if (reachable[k])
                wanted.push_back(k);

        // What standing still is worth for each contact, and the model reading at each position it
        // may move onto: independent searches over a const field, so they are all taken here, before
        // any decision is made. Only the readings change hands - the loop below still runs region by
        // region and, inside a region, in source id order, and a contact still takes a position out
        // from under the contacts that follow it.
        std::vector<ModelSupportRisk::Sample> here(seeds.size());
        std::vector<ModelSupportRisk::Sample> sample(candidates.size());
        tbb::parallel_for(tbb::blocked_range<size_t>(0, seeds.size() + wanted.size()),
            [&](const tbb::blocked_range<size_t> &range) {
                for (size_t j = range.begin(); j < range.end(); ++ j)
                    if (j < seeds.size()) {
                        const size_t s = seeds[j];
                        if (kept[s] && ! problem.seeds[s].pinned)
                            here[j] = ModelSupportRisk::sample(risk, region.object_layer, position[s]);
                    } else {
                        const size_t k = wanted[j - seeds.size()];
                        sample[k] = ModelSupportRisk::sample(risk, region.object_layer, candidates[k]);
                    }
            });

        for (size_t i = 0; i < seeds.size(); ++ i) {
            const size_t       s    = seeds[i];
            const ContactSeed &seed = problem.seeds[s];
            // A pinned contact is one the generator was asked for, at the place it was asked for.
            if (! kept[s] || seed.pinned)
                continue;
            // What the contact damages where it lands: the ground its own tip covers, never less
            // than the ground one extrusion covers, because nothing prints narrower than that.
            // The floor is on the measurement alone - the contact's own radius stays what the
            // generator drew, and no weight here may widen a tip or thin a stem.
            const Polygon patch = contact_patch(std::max(seed.radius_mm, 0.5 * problem.extrusion_width_mm));

            // The cells this contact is the region's only cover for: whatever it moves onto has
            // to hold every one of them, so the region gives up nothing by the move.
            Witnesses required;
            required.pitch = lattice.pitch;
            for (size_t c = 0; c < cover[s].size(); ++ c)
                if (cover[s][c] && cover_count[r][c] <= 1)
                    required.cells.push_back(lattice.cells[c]);

            // What standing still is worth, which is what anywhere else has to beat.
            const ModelSupportRisk::Sample &here_sample = here[i];
            const int    here_rank = standing_rank(here_sample);
            const double here_risk = here_sample.risk_per_mm2 * patch_area_mm2(region_polys, patch, position[s]);

            // Every position that is an improvement at all: measured no worse than where the
            // contact stands, within the region's reach of its source, and not already taken.
            struct Placement
            {
                int    rank  = 0;
                double risk  = 0.;
                double move2 = 0.;   // squared, scaled: only its order is read
                size_t index = 0;
            };
            std::vector<Placement> options;
            for (size_t k = 0; k < candidates.size(); ++ k) {
                if (occupied[k])
                    continue;
                // No further from where the generator placed the contact than the region allows.
                const double dx = double(candidates[k].x() - seed.position.x());
                const double dy = double(candidates[k].y() - seed.position.y());
                if (dx * dx + dy * dy > reach2)
                    continue;
                const ModelSupportRisk::Sample &there = sample[k];
                const int                       rank  = standing_rank(there);
                if (rank < here_rank)
                    continue;
                const double cost = there.risk_per_mm2 * patch_area_mm2(region_polys, patch, candidates[k]);
                // Weights of two different ranks are not numbers about the same thing, so only a
                // move within one rank is decided by the weight at all.
                if (rank == here_rank && ! (cost < here_risk))
                    continue;
                options.push_back({ rank, cost, dx * dx + dy * dy, k });
            }
            // Better measured ground first, then less damage, then the shorter move, then the
            // lattice's own order, which the region hands out with its cells.
            std::sort(options.begin(), options.end(), [](const Placement &a, const Placement &b) {
                if (a.rank != b.rank)
                    return a.rank > b.rank;
                if (a.risk != b.risk)
                    return a.risk < b.risk;
                if (a.move2 != b.move2)
                    return a.move2 < b.move2;
                return a.index < b.index;
            });

            size_t best = std::numeric_limits<size_t>::max();
            for (const Placement &option : options)
                if (covers_all(region.polygon, required, candidates[option.index], region.legal_reach_mm)) {
                    best = option.index;
                    break;
                }
            if (best == std::numeric_limits<size_t>::max())
                continue;

            for (size_t c = 0; c < cover[s].size(); ++ c)
                if (cover[s][c])
                    -- cover_count[r][c];
            position[s]    = candidates[best];
            occupied[best] = 1;
            cover[s]       = covered_by_contact(region.polygon, lattice, position[s], region.legal_reach_mm);
            for (size_t c = 0; c < cover[s].size(); ++ c)
                if (cover[s][c])
                    ++ cover_count[r][c];
        }
    }
}

} // namespace

bool holds_an_extrusion(const ExPolygons &area, double extrusion_width_mm)
{
    return ! offset_ex(area, -0.5f * float(scale_(extrusion_width_mm))).empty();
}

Witnesses witness_cells(const ExPolygon &polygon, double extrusion_width_mm)
{
    Witnesses witnesses;
    if (extrusion_width_mm <= 0. || polygon.contour.points.size() < 3)
        return witnesses;
    // Half an extrusion width, floored at one scaled unit: a lattice coarser than the material that
    // has to cover it could hide a gap a whole extrusion wide inside one cell.
    witnesses.pitch      = std::max<coord_t>(1, coord_t(std::llround(scaled<double>(0.5 * extrusion_width_mm))));
    const coord_t pitch  = witnesses.pitch;
    const BoundingBox bbox = get_extents(polygon);
    if (! bbox.defined)
        return witnesses;

    // Floor division, so the lattice runs continuously through the origin instead of folding at it:
    // the lines sit at multiples of the pitch in scaled object coordinates, and two regions that
    // overlap share cell boundaries however far from the origin they lie.
    const auto lattice_index = [pitch](coord_t v) { return v >= 0 ? v / pitch : - ((pitch - 1 - v) / pitch); };

    const ExPolygons source{ polygon };
    for (coord_t iy = lattice_index(bbox.min.y()); iy * pitch < bbox.max.y(); ++ iy)
        for (coord_t ix = lattice_index(bbox.min.x()); ix * pitch < bbox.max.x(); ++ ix) {
            Polygon square;
            square.points = { Point(ix * pitch, iy * pitch), Point((ix + 1) * pitch, iy * pitch),
                              Point((ix + 1) * pitch, (iy + 1) * pitch), Point(ix * pitch, (iy + 1) * pitch) };
            // The square clipped to the region, kept whatever is left of it: a boundary sliver is a
            // place material still has to reach, and dropping it would let a contact claim ground it
            // never covers.
            ExPolygons clipped = intersection_ex(source, ExPolygons{ ExPolygon(square) });
            if (clipped.empty())
                continue;
            WitnessCell cell;
            cell.index = Point(ix, iy);
            cell.area  = std::move(clipped);
            for (const ExPolygon &piece : cell.area) {
                append(cell.corners, piece.contour.points);
                for (const Polygon &hole : piece.holes)
                    append(cell.corners, hole.points);
            }
            cell.bbox = get_extents(cell.area);
            witnesses.cells.emplace_back(std::move(cell));
        }
    return witnesses;
}

std::shared_ptr<const Witnesses> region_witnesses(const RequiredRegion &region, double extrusion_width_mm)
{
    // A region with no room for one extrusion is a sliver, not a place a line can be laid: it carries no
    // cells whatever lattice was frozen with it, so nothing can be counted for or against it.
    if (! region.printable)
        return std::make_shared<const Witnesses>();
    if (region.witnesses)
        return region.witnesses;
    return std::make_shared<const Witnesses>(witness_cells(region.polygon, extrusion_width_mm));
}

double legal_reach(double branch_distance_mm, double max_bridge_length_mm)
{
    const bool spacing = branch_distance_mm > 0., bridging = max_bridge_length_mm > 0.;
    if (spacing && bridging)
        return 0.5 * std::min(branch_distance_mm, max_bridge_length_mm);
    if (spacing)
        return 0.5 * branch_distance_mm;
    if (bridging)
        return 0.5 * max_bridge_length_mm;
    // Neither limit is active, so there is no active spacing to read a reach off: a contact anchors
    // where it sits and nowhere else, rather than everywhere.
    return 0.;
}

std::vector<bool> covered_by_contact(const ExPolygon &polygon, const Witnesses &witnesses,
                                     const Point &position, double reach_mm)
{
    std::vector<bool> covered(witnesses.size(), false);
    if (reach_mm <= 0. || witnesses.empty())
        return covered;
    const double reach2 = scaled<double>(reach_mm) * scaled<double>(reach_mm);
    // A convex region with no hole holds every segment between two of its own points, so nothing in
    // it can block a line of sight and the hull test below has only one answer. Between two of its
    // own points: a contact outside the region still leaves it on the way in, so the fast path is
    // its to take only while it stands inside.
    const bool       unobstructed = polygon.holes.empty() && polygon_is_convex(polygon.contour) &&
                                    polygon.contains(position);
    const ExPolygons region{ polygon };
    for (size_t i = 0; i < witnesses.cells.size(); ++ i) {
        const WitnessCell &cell = witnesses.cells[i];
        // The complete cell within the reach, never just its centre: a polygon's farthest point from
        // anywhere outside it is one of its vertices, so the vertices settle it exactly.
        bool within = true;
        for (const Point &p : cell.corners) {
            const double dx = double(p.x() - position.x()), dy = double(p.y() - position.y());
            if (dx * dx + dy * dy > reach2) {
                within = false;
                break;
            }
        }
        if (! within)
            continue;
        if (unobstructed) {
            covered[i] = true;
            continue;
        }
        // The straight segment from the contact to every point of the cell stays inside the region
        // exactly when the convex hull of the contact and the cell does: the hull is convex, so it
        // holds all of those segments, and it fits inside the region only when none of them leaves
        // it. A hole, a notch or a second connected piece therefore blocks the contact outright,
        // where a disk dilation would have spanned it.
        Points hull_points = cell.corners;
        hull_points.push_back(position);
        const ExPolygons outside = diff_ex(ExPolygons{ ExPolygon(Geometry::convex_hull(hull_points)) }, region);
        // Clipper leaves hairline slivers where the hull runs along the region's own boundary. A
        // real leak is as wide as the gap it crosses; anything narrower than one scaled epsilon is
        // rounding, not geometry.
        if (outside.empty() || offset_ex(outside, - float(SCALED_EPSILON)).empty())
            covered[i] = true;
    }
    return covered;
}

CoverageRule::CoverageRule(const Problem &problem) : m_problem(problem)
{
    const size_t count = problem.regions.size();
    m_lattice.reserve(count);
    for (const RequiredRegion &region : problem.regions)
        m_lattice.push_back(region_witnesses(region, problem.extrusion_width_mm));

    m_order.resize(count);
    for (size_t r = 0; r < count; ++ r)
        m_order[r] = r;
    // Component first, so one component's regions are contiguous; then contact z, so the carry loop
    // walks a component upward; then the region index, so equal z is still one fixed order.
    std::sort(m_order.begin(), m_order.end(), [&problem](size_t a, size_t b) {
        const RequiredRegion &ra = problem.regions[a], &rb = problem.regions[b];
        if (ra.component != rb.component)
            return ra.component < rb.component;
        if (ra.contact_z_mm != rb.contact_z_mm)
            return ra.contact_z_mm < rb.contact_z_mm;
        return a < b;
    });
    m_group_first.assign(count, 0);
    m_group_last.assign(count, 0);
    for (size_t i = 0; i < m_order.size();) {
        size_t j = i;
        while (j < m_order.size() && problem.regions[m_order[j]].component == problem.regions[m_order[i]].component)
            ++ j;
        for (size_t k = i; k < j; ++ k) {
            m_group_first[m_order[k]] = i;
            m_group_last[m_order[k]]  = j;
        }
        i = j;
    }
}

std::vector<CoveredCell> CoverageRule::cells(size_t seed, const Point &position) const
{
    std::vector<CoveredCell> out;
    if (seed >= m_problem.seeds.size())
        return out;
    const size_t own = size_t(m_problem.seeds[seed].region_id);
    if (own >= m_problem.regions.size())
        return out;

    // The own region, under the whole reach and line-of-sight test `covered_by_contact` runs: what
    // this contact anchors is what it legally covers there.
    const RequiredRegion &region = m_problem.regions[own];
    const std::vector<bool> held = covered_by_contact(region.polygon, *m_lattice[own], position,
                                                      region.legal_reach_mm);
    for (size_t c = 0; c < held.size(); ++ c)
        if (held[c])
            out.push_back({ own, c });

    const double d = m_problem.contact_min_distance_mm;
    if (d <= 0.)
        return out;

    // What it carries: a cell of a region of its own overhang component, above it and strictly under
    // the distance in 3-D, every corner of the cell included. Material printed for this contact
    // stands under such a cell, so the cell keeps a contact behind it while this contact is retained.
    // Credit flows upward only, and the carried clause has no visibility test: within the distance
    // the branch is the same one.
    const double      z0       = region.contact_z_mm;
    const double      scaled_d = scaled<double>(d);
    const double      d2       = scaled_d * scaled_d;
    const BoundingBox disc     = BoundingBox(position, position).inflated(scaled_d);
    for (size_t i = m_group_first[own]; i < m_group_last[own]; ++ i) {
        const size_t r = m_order[i];
        if (r == own)
            continue;
        const double dz = m_problem.regions[r].contact_z_mm - z0;
        if (dz < 0. || dz >= d)
            continue;
        const double     dz2     = scaled<double>(dz) * scaled<double>(dz);
        const Witnesses &lattice = *m_lattice[r];
        for (size_t c = 0; c < lattice.cells.size(); ++ c) {
            const WitnessCell &cell = lattice.cells[c];
            if (! disc.overlap(cell.bbox))
                continue;
            bool within = true;
            for (const Point &p : cell.corners) {
                const double dx = double(p.x() - position.x()), dy = double(p.y() - position.y());
                if (dx * dx + dy * dy + dz2 >= d2) {
                    within = false;
                    break;
                }
            }
            if (within)
                out.push_back({ r, c });
        }
    }
    return out;
}

Selection select_contacts(const Problem &problem, const ModelSupportRisk::Field &risk)
{
    Selection selection;
    if (problem.seeds.empty())
        return selection;

    const size_t                     region_count = problem.regions.size();
    std::vector<std::vector<size_t>> seeds_of_region(region_count);
    for (size_t s = 0; s < problem.seeds.size(); ++ s)
        if (problem.seeds[s].region_id < region_count)
            seeds_of_region[size_t(problem.seeds[s].region_id)].push_back(s);

    std::vector<char> kept(problem.seeds.size(), 1);
    size_t            restored = 0;

    if (problem.contact_min_distance_mm > 0.) {
        // The contact z a seed stands at is its own region's.
        const auto z_of = [&problem](size_t s) {
            return problem.regions[size_t(problem.seeds[s].region_id)].contact_z_mm;
        };

        // Decimation, over the source positions and by the requested distance alone, which is what main's
        // `decimate_contact_nodes` did: contact z ascending, radius descending, object layer ascending,
        // then position, so the order is a function of the frozen problem and a contact is only ever
        // judged against ones already settled at its z or below it. A seed whose region is past the end
        // of the problem is in no component and is never decimated.
        std::vector<size_t> order;
        order.reserve(problem.seeds.size());
        for (size_t s = 0; s < problem.seeds.size(); ++ s)
            if (problem.seeds[s].region_id < region_count)
                order.push_back(s);
        std::stable_sort(order.begin(), order.end(), [&problem](size_t a, size_t b) {
            const RequiredRegion &ra = problem.regions[size_t(problem.seeds[a].region_id)];
            const RequiredRegion &rb = problem.regions[size_t(problem.seeds[b].region_id)];
            if (ra.contact_z_mm != rb.contact_z_mm)
                return ra.contact_z_mm < rb.contact_z_mm;
            if (problem.seeds[a].radius_mm != problem.seeds[b].radius_mm)
                return problem.seeds[a].radius_mm > problem.seeds[b].radius_mm;
            if (ra.object_layer != rb.object_layer)
                return ra.object_layer < rb.object_layer;
            return problem.seeds[a].position < problem.seeds[b].position;
        });

        // The contacts already settled, per overhang component, which is the set a contact may be
        // crowded by: a contact of another component anchors nothing here however close it stands.
        std::map<size_t, std::vector<size_t>> settled_of_component;
        const double                          distance2 = problem.contact_min_distance_mm * problem.contact_min_distance_mm;
        for (const size_t s : order) {
            // A pinned contact is one the generator was asked for: it is kept, and it crowds nobody,
            // the way main's rule skipped a pinned node before reaching its grid at all.
            if (problem.seeds[s].pinned)
                continue;
            std::vector<size_t> &settled = settled_of_component[problem.regions[size_t(problem.seeds[s].region_id)].component];
            const double         z       = z_of(s);
            for (const size_t k : settled) {
                const double dx = unscale<double>(problem.seeds[s].position.x() - problem.seeds[k].position.x());
                const double dy = unscale<double>(problem.seeds[s].position.y() - problem.seeds[k].position.y());
                const double dz = z - z_of(k);
                // Strict, as main's was: a contact at exactly the requested distance is not crowded.
                if (dx * dx + dy * dy + dz * dz < distance2) {
                    kept[s] = 0;
                    break;
                }
            }
            if (kept[s])
                settled.push_back(s);
        }

        // The add-back, read at the contacts' own positions, which is where the decimation judged them.
        // A printable region that carries contacts of its own and has a witness cell no survivor stands
        // behind under `CoverageRule` keeps its lowest-id contact. A region too narrow to hold one
        // extrusion carries no cells at all (`region_witnesses` states that), so nothing comes back for
        // a sliver: it is not a place a line can be laid.
        const CoverageRule             rule(problem);
        std::vector<std::vector<char>> covered(region_count);
        for (size_t r = 0; r < region_count; ++ r)
            covered[r].assign(region_witnesses(problem.regions[r], problem.extrusion_width_mm)->size(), 0);
        for (size_t s = 0; s < problem.seeds.size(); ++ s)
            if (kept[s])
                for (const CoveredCell &cell : rule.cells(s, problem.seeds[s].position))
                    covered[cell.region][cell.cell] = 1;
        for (size_t r = 0; r < region_count; ++ r) {
            if (! problem.regions[r].printable || seeds_of_region[r].empty())
                continue;
            if (std::all_of(covered[r].begin(), covered[r].end(), [](char c) { return c != 0; }))
                continue;
            size_t lowest = seeds_of_region[r].front();
            for (const size_t s : seeds_of_region[r])
                if (problem.seeds[s].id < problem.seeds[lowest].id)
                    lowest = s;
            if (! kept[lowest]) {
                kept[lowest] = 1;
                ++ restored;
            }
        }
    }

    // What each retained contact legally covers on its own region and how many retained contacts stand
    // behind each cell: the coverage placement below may not give up. The lattice is thrown away again,
    // only the cover vectors indexed by it outlive this loop.
    std::vector<std::vector<bool>> cover(problem.seeds.size());
    std::vector<std::vector<int>>  cover_count(region_count);
    for (size_t r = 0; r < region_count; ++ r) {
        if (seeds_of_region[r].empty())
            continue;
        const auto      lattice_ptr = region_witnesses(problem.regions[r], problem.extrusion_width_mm);
        const Witnesses &lattice    = *lattice_ptr;
        cover_count[r].assign(lattice.size(), 0);
        for (size_t s : seeds_of_region[r]) {
            if (! kept[s])
                continue;
            cover[s] = covered_by_contact(problem.regions[r].polygon, lattice, problem.seeds[s].position,
                                          problem.regions[r].legal_reach_mm);
            for (size_t c = 0; c < cover[s].size(); ++ c)
                if (cover[s][c])
                    ++ cover_count[r][c];
        }
    }

    // Where each retained contact ends up. It stands where the generator put it until a legal position
    // over less fragile model material is found for it. An unmeasured field is the absence of a reading,
    // not a reading of no risk: with nothing to compare positions by, every contact stays put.
    std::vector<Point> position(problem.seeds.size());
    for (size_t s = 0; s < problem.seeds.size(); ++ s)
        position[s] = problem.seeds[s].position;
    if (risk.status == ModelSupportRisk::Field::Status::Complete)
        relocate_contacts(problem, risk, seeds_of_region, kept, cover, cover_count, position);

    for (size_t s = 0; s < problem.seeds.size(); ++ s)
        if (kept[s]) {
            selection.retained.push_back(problem.seeds[s]);
            selection.retained.back().position = position[s];
        }
    selection.seeds_restored = restored;
    return selection;
}

} // namespace MiniatureSupport
} // namespace Slic3r
