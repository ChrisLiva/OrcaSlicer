#include <catch2/catch_all.hpp>

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Geometry/ConvexHull.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Support/MiniatureSupport.hpp"
#include "libslic3r/Support/ModelSupportRisk.hpp"
#include "libslic3r/libslic3r.h"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <vector>

using namespace Slic3r;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

// An axis-aligned rectangle in mm, as an ExPolygon in scaled object coordinates.
ExPolygon rect_mm(double x0, double y0, double x1, double y1)
{
    ExPolygon ex;
    ex.contour.points = { Point(scale_(x0), scale_(y0)), Point(scale_(x1), scale_(y0)),
                          Point(scale_(x1), scale_(y1)), Point(scale_(x0), scale_(y1)) };
    return ex;
}

Point pt_mm(double x, double y) { return Point(scale_(x), scale_(y)); }

// One required region of a written-out problem, taking the id its insertion order gives it.
void add_region(MiniatureSupport::Problem &problem, const ExPolygon &polygon, size_t layer, double contact_z,
                double reach)
{
    MiniatureSupport::RequiredRegion region;
    region.id             = uint64_t(problem.regions.size());
    region.object_layer   = layer;
    region.polygon        = polygon;
    region.contact_z_mm   = contact_z;
    region.legal_reach_mm = reach;
    // Read off the polygon and the problem's width the way `build_required_regions` reads it, so a
    // hand-written sliver is non-printable here too without the leg having to say so.
    region.printable      = MiniatureSupport::holds_an_extrusion(ExPolygons{ polygon }, problem.extrusion_width_mm);
    // A written-out region is its own component, the same value as its id, unless the leg assigns one.
    region.component      = problem.regions.size();
    problem.regions.push_back(region);
}

// One contact seed of a written-out problem, likewise dense from zero in insertion order.
void add_seed(MiniatureSupport::Problem &problem, uint64_t region_id, double x, double y, bool pinned = false,
              bool critical = false, double radius_mm = 0.4)
{
    MiniatureSupport::ContactSeed seed;
    seed.id        = uint64_t(problem.seeds.size());
    seed.region_id = region_id;
    seed.position  = pt_mm(x, y);
    seed.radius_mm = radius_mm;
    seed.pinned    = pinned;
    seed.critical  = critical;
    problem.seeds.push_back(seed);
}

// One 1 mm slab of a model, from parts Clipper unions into whatever islands they make.
ModelSupportRisk::Slice slab_mm(double bottom_mm, std::initializer_list<ExPolygon> parts)
{
    ModelSupportRisk::Slice slice;
    slice.bottom_z_mm = bottom_mm;
    slice.top_z_mm    = bottom_mm + 1.;
    Polygons all;
    for (const ExPolygon &part : parts)
        append(all, to_polygons(part));
    slice.solids = union_ex(all);
    return slice;
}

// A 6 x 6 mm guard block standing on the bed, carrying a `length` mm long, 0.8 mm thick blade off its
// +x face on the upper slab alone. Layer 1 is one island: the model is 6 mm across over the guard and
// 0.8 mm across over the blade, so one overhang band laid along y 2.6..3.4 crosses both.
std::vector<ModelSupportRisk::Slice> guard_and_blade(double length)
{
    const ExPolygon guard = rect_mm(0., 0., 6., 6.);
    const ExPolygon blade = rect_mm(6., 2.6, 6. + length, 3.4);
    return { slab_mm(0., { guard }), slab_mm(1., { guard, blade }) };
}

// A 12 x 6 mm guard (`rect_mm(-6., ...)`) and the same 3 mm blade, plus a 3 x 3 mm pad hanging half a
// millimetre off the blade's tip and
// standing on a 0.4 x 0.4 mm foot of its own. The pad is thick material reached only through a joint
// narrower than one extrusion, and the half millimetre between blade and pad is model nowhere at all,
// so one band laid along y = 3 crosses printable ground, unmeasurable ground and unprintable ground.
std::vector<ModelSupportRisk::Slice> blade_and_stilted_pad()
{
    const ExPolygon guard = rect_mm(-6., 0., 6., 6.);
    const ExPolygon blade = rect_mm(6., 2.6, 9., 3.4);
    const ExPolygon foot  = rect_mm(10.8, 2.8, 11.2, 3.2);
    const ExPolygon pad   = rect_mm(9.5, 1.5, 12.5, 4.5);
    return { slab_mm(0., { guard, foot }), slab_mm(1., { guard, blade, pad }) };
}

// A field nothing was ever measured into: every query answers Unknown, which is what the thinning
// legs below run against and what a canceled or malformed measurement leaves behind.
const ModelSupportRisk::Field &unmeasured_field()
{
    static const ModelSupportRisk::Field field;
    return field;
}

// Which of `region`'s witness cells the contacts at `positions` legally cover between them.
std::vector<bool> covered_union(const MiniatureSupport::RequiredRegion &region, double extrusion_width_mm,
                                const std::vector<Point> &positions)
{
    const MiniatureSupport::Witnesses lattice = MiniatureSupport::witness_cells(region.polygon, extrusion_width_mm);
    std::vector<bool>                 covered(lattice.size(), false);
    for (const Point &position : positions) {
        const std::vector<bool> one =
            MiniatureSupport::covered_by_contact(region.polygon, lattice, position, region.legal_reach_mm);
        for (size_t c = 0; c < covered.size() && c < one.size(); ++ c)
            covered[c] = covered[c] || one[c];
    }
    return covered;
}

// Where each seed of `problem` ended up: its own position when the selection dropped it, and the
// position the selection retained it at when it kept it.
std::vector<Point> retained_positions(const MiniatureSupport::Problem &problem, const MiniatureSupport::Selection &selection)
{
    std::vector<Point> positions;
    positions.reserve(problem.seeds.size());
    for (const MiniatureSupport::ContactSeed &seed : problem.seeds)
        positions.push_back(seed.position);
    for (const MiniatureSupport::ContactSeed &seed : selection.retained)
        if (seed.id < positions.size())
            positions[size_t(seed.id)] = seed.position;
    return positions;
}

} // namespace

TEST_CASE("A contact moves only inside its own region and within its legal reach and over every witness it held", "[MiniatureSupport][MiniatureContacts]")
{
    // A 6 x 6 mm guard block standing on the bed with a 0.8 mm blade off its +x face, and one overhang
    // band laid along the blade's centre line from 4 mm inside the guard to 2 mm out along the blade.
    // The band is the same 0.8 mm strip end to end; what changes across it is the model under it.
    const ModelSupportRisk::Field field =
        ModelSupportRisk::build(guard_and_blade(3.), 0.42, []() { return false; });
    REQUIRE(field.status == ModelSupportRisk::Field::Status::Complete);
    // The two ends of the band, measured: 0.8 mm of blade against several millimetres of guard. The
    // move below is worth nothing unless the field really reads the two ends apart.
    const ModelSupportRisk::Sample on_blade = ModelSupportRisk::sample(field, 1, pt_mm(7.5, 3.0));
    const ModelSupportRisk::Sample on_guard = ModelSupportRisk::sample(field, 1, pt_mm(2.4, 3.0));
    REQUIRE(on_blade.status == ModelSupportRisk::Sample::Status::Known);
    REQUIRE(on_guard.status == ModelSupportRisk::Sample::Status::Known);
    REQUIRE(on_guard.local_width_mm > on_blade.local_width_mm + 1.);
    REQUIRE(on_guard.risk_per_mm2 < on_blade.risk_per_mm2);

    // One region and one contact on the thin end of it. The reach is longer than the band's own
    // diagonal, so every position in the band covers every cell of it and coverage never decides
    // this leg: what the contact may do is bounded by the region and by the reach alone.
    const ExPolygon band = rect_mm(2., 2.6, 8., 3.4);
    const auto      one_seed = [&band](bool pinned, bool critical) {
        MiniatureSupport::Problem problem;
        problem.extrusion_width_mm = 0.42;
        add_region(problem, band, 1, 1.0, 7.);
        add_seed(problem, 0, 7.5, 3.0, pinned, critical);
        return problem;
    };

    SECTION("a contact on the blade moves onto the guard, keeping its region, its layer and every cell it covered")
    {
        const MiniatureSupport::Problem   problem   = one_seed(false, false);
        const MiniatureSupport::Selection selection = MiniatureSupport::select_contacts(problem, field);
        REQUIRE(selection.retained.size() == 1);
        const MiniatureSupport::ContactSeed &kept = selection.retained.front();
        // The same contact, for the same region, on the same object layer: only where it stands changed.
        CHECK(kept.id == 0);
        CHECK(kept.region_id == 0);
        CHECK(kept.position != problem.seeds.front().position);
        // Onto the guard. The blade starts at x = 6, and the guard's own width is only readable well
        // inside it, so the assertion is that the contact left the thin material behind.
        CHECK(unscale<double>(kept.position.x()) < 5.);
        // Still a place the region holds, and no further from its source than the region allows.
        CHECK(band.contains(kept.position));
        CHECK((kept.position - problem.seeds.front().position).cast<double>().norm() * SCALING_FACTOR
              <= problem.regions.front().legal_reach_mm + 1e-6);

        // And the band gave up nothing: every cell the contact covered where it was, it covers where
        // it went.
        const std::vector<bool> before = covered_union(problem.regions.front(), problem.extrusion_width_mm,
                                                       { problem.seeds.front().position });
        const std::vector<bool> after  = covered_union(problem.regions.front(), problem.extrusion_width_mm,
                                                       { kept.position });
        REQUIRE(before.size() == after.size());
        for (size_t c = 0; c < before.size(); ++ c)
            if (before[c])
                CHECK(after[c]);
    }

    SECTION("a pinned contact stays where the generator put it and a critical one may still move")
    {
        const MiniatureSupport::Problem   pinned    = one_seed(true, false);
        const MiniatureSupport::Selection unmoved   = MiniatureSupport::select_contacts(pinned, field);
        REQUIRE(unmoved.retained.size() == 1);
        CHECK(unmoved.retained.front().position == pinned.seeds.front().position);

        // A critical contact is the last anchor of something, and it stays that anchor wherever in its
        // own region it stands: its region and its height are what the anchor names.
        const MiniatureSupport::Problem   critical = one_seed(false, true);
        const MiniatureSupport::Selection moved    = MiniatureSupport::select_contacts(critical, field);
        REQUIRE(moved.retained.size() == 1);
        CHECK(moved.retained.front().critical);
        CHECK(moved.retained.front().position != critical.seeds.front().position);
    }

    SECTION("no contact moves further than its region's legal reach")
    {
        // A 9 mm band over a 6 mm blade, seeded every 0.35 mm, with a 1 mm reach. Each contact's cells
        // are held by four neighbours, so coverage lets every one of them move and the reach is the
        // only bound left standing.
        const ModelSupportRisk::Field longer =
            ModelSupportRisk::build(guard_and_blade(6.), 0.42, []() { return false; });
        REQUIRE(longer.status == ModelSupportRisk::Field::Status::Complete);
        MiniatureSupport::Problem problem;
        problem.extrusion_width_mm = 0.42;
        add_region(problem, rect_mm(2., 2.6, 11., 3.4), 1, 1.0, 1.);
        for (double x = 2.5; x < 10.6; x += 0.35)
            add_seed(problem, 0, x, 3.0);
        REQUIRE(problem.seeds.size() > 20);

        const MiniatureSupport::Selection selection = MiniatureSupport::select_contacts(problem, longer);
        REQUIRE(selection.retained.size() == problem.seeds.size());
        size_t moved = 0;
        for (const MiniatureSupport::ContactSeed &kept : selection.retained) {
            const Point &source = problem.seeds[size_t(kept.id)].position;
            const double travel = (kept.position - source).cast<double>().norm() * SCALING_FACTOR;
            INFO("seed " << kept.id << " travelled " << travel << " mm");
            CHECK(travel <= problem.regions.front().legal_reach_mm + 1e-6);
            if (kept.position != source)
                ++ moved;
        }
        CHECK(moved > 0);
    }

    SECTION("a field that measured nothing moves nothing")
    {
        // An unmeasured field is the absence of a measurement, not a reading of zero risk everywhere:
        // nothing about it says one position is better than another, so every contact stays put.
        const MiniatureSupport::Problem   problem   = one_seed(false, false);
        const MiniatureSupport::Selection selection = MiniatureSupport::select_contacts(problem, unmeasured_field());
        REQUIRE(selection.retained.size() == 1);
        CHECK(selection.retained.front().position == problem.seeds.front().position);
    }
}

TEST_CASE("Placement takes measured printable ground only and settles what it cannot tell apart by the shorter move", "[MiniatureSupport][MiniatureContacts]")
{
    const ModelSupportRisk::Field field =
        ModelSupportRisk::build(blade_and_stilted_pad(), 0.42, []() { return false; });
    REQUIRE(field.status == ModelSupportRisk::Field::Status::Complete);

    // The three kinds of ground the band below crosses, read off the field before anything is placed
    // on them. The pad is the trap: thick material, reached only over a 0.4 mm joint, and cheaper by
    // the weight than the 0.8 mm blade the contact starts on.
    const ModelSupportRisk::Sample blade = ModelSupportRisk::sample(field, 1, pt_mm(7.5, 3.0));
    const ModelSupportRisk::Sample gap   = ModelSupportRisk::sample(field, 1, pt_mm(9.25, 3.0));
    const ModelSupportRisk::Sample pad   = ModelSupportRisk::sample(field, 1, pt_mm(11.0, 3.0));
    REQUIRE(blade.status == ModelSupportRisk::Sample::Status::Known);
    REQUIRE(gap.status == ModelSupportRisk::Sample::Status::Unknown);
    REQUIRE(pad.status == ModelSupportRisk::Sample::Status::BelowPrintableWidth);
    REQUIRE(pad.risk_per_mm2 < blade.risk_per_mm2);

    // A 0.2 mm band along the blade's centre line, out over the gap and across the pad.
    const ExPolygon band = rect_mm(6.2, 2.9, 12.2, 3.1);
    MiniatureSupport::Problem problem;
    problem.extrusion_width_mm = 0.42;
    add_region(problem, band, 1, 1.0, 7.);
    add_seed(problem, 0, 7.5, 3.0);

    SECTION("neither an unmeasured position nor one below the printable width takes over a printable placement")
    {
        const MiniatureSupport::Selection selection = MiniatureSupport::select_contacts(problem, field);
        REQUIRE(selection.retained.size() == 1);
        const Point &where = selection.retained.front().position;
        INFO("contact ended at " << unscale<double>(where.x()) << ", " << unscale<double>(where.y()));
        // Unmeasured geometry is not zero damage and cheap unprintable material is not a place to
        // stand: the contact keeps printable ground it can be measured on.
        CHECK(ModelSupportRisk::sample(field, 1, where).status == ModelSupportRisk::Sample::Status::Known);
        CHECK(unscale<double>(where.x()) < 9.);
    }

    SECTION("a contact already standing on the best ground its region offers does not move")
    {
        const Point settled = MiniatureSupport::select_contacts(problem, field).retained.front().position;
        // The same problem with the contact placed where the last one ended: there is nothing left to
        // gain, so the shorter move wins and the shortest move is none.
        MiniatureSupport::Problem already;
        already.extrusion_width_mm = problem.extrusion_width_mm;
        add_region(already, band, 1, 1.0, 7.);
        MiniatureSupport::ContactSeed seed;
        seed.position  = settled;
        seed.radius_mm = 0.4;
        already.seeds.push_back(seed);
        const MiniatureSupport::Selection again = MiniatureSupport::select_contacts(already, field);
        REQUIRE(again.retained.size() == 1);
        CHECK(again.retained.front().position == settled);
    }

    SECTION("two contacts never share a position and one problem gives one answer twice")
    {
        MiniatureSupport::Problem pair;
        pair.extrusion_width_mm = 0.42;
        add_region(pair, band, 1, 1.0, 7.);
        add_seed(pair, 0, 7.2, 3.0);
        add_seed(pair, 0, 7.8, 3.0);
        const MiniatureSupport::Selection first  = MiniatureSupport::select_contacts(pair, field);
        const MiniatureSupport::Selection second = MiniatureSupport::select_contacts(pair, field);
        REQUIRE(first.retained.size() == 2);
        REQUIRE(second.retained.size() == 2);
        // Two anchors collapsed onto one point anchor once.
        CHECK(first.retained[0].position != first.retained[1].position);
        CHECK(first.retained[0].position == second.retained[0].position);
        CHECK(first.retained[1].position == second.retained[1].position);
    }
}

TEST_CASE("Placement weighs a contact over the width it will be printed at and never rewrites its radius", "[MiniatureSupport][MiniatureContacts]")
{
    const ModelSupportRisk::Field field =
        ModelSupportRisk::build(guard_and_blade(3.), 0.42, []() { return false; });
    REQUIRE(field.status == ModelSupportRisk::Field::Status::Complete);
    const ExPolygon band = rect_mm(2., 2.6, 8., 3.4);
    const auto      one_seed = [&band](double radius_mm) {
        MiniatureSupport::Problem problem;
        problem.extrusion_width_mm = 0.42;
        add_region(problem, band, 1, 1.0, 7.);
        add_seed(problem, 0, 7.5, 3.0, false, false, radius_mm);
        return problem;
    };

    SECTION("a contact the generator drew at no radius is still weighed over the printable tip")
    {
        // Nothing prints narrower than one extrusion, so a contact carrying no radius of its own still
        // damages the model over half an extrusion width. Weighing it over its nominal zero would make
        // every position on the model cost exactly nothing and leave the contact on the blade.
        const MiniatureSupport::Problem   problem   = one_seed(0.);
        const MiniatureSupport::Selection selection = MiniatureSupport::select_contacts(problem, field);
        REQUIRE(selection.retained.size() == 1);
        CHECK(unscale<double>(selection.retained.front().position.x()) < 5.);
        // The floor is on the weighing and on nothing else: the contact still carries the radius the
        // generator drew, and what the branch under it grows to is not this pass's to decide.
        CHECK_THAT(selection.retained.front().radius_mm, WithinAbs(0., 1e-12));
    }

    SECTION("the contact keeps the radius the generator gave it")
    {
        // Placement answers where a contact stands and nothing else: it cannot thin a stem that has to
        // carry a branch, and it cannot widen a tip to spread its damage over stronger material.
        for (double radius_mm : { 0.4, 1.1 }) {
            DYNAMIC_SECTION("radius " << radius_mm << " mm")
            {
                const MiniatureSupport::Problem   problem   = one_seed(radius_mm);
                const MiniatureSupport::Selection selection = MiniatureSupport::select_contacts(problem, field);
                REQUIRE(selection.retained.size() == 1);
                CHECK(selection.retained.front().position != problem.seeds.front().position);
                CHECK_THAT(selection.retained.front().radius_mm, WithinAbs(radius_mm, 1e-12));
            }
        }
    }
}

TEST_CASE("The frozen witness lattice halves the extrusion width and aligns to scaled coordinates and keeps every clipped cell", "[MiniatureSupport][MiniatureContacts]")
{
    // A 1.05 x 0.55 mm rectangle whose lower-left corner sits at (0.209, -0.209) mm: no edge of it
    // lines up with a 0.21 mm lattice, the region straddles y = 0, and its left column is 0.001 mm
    // wide, the narrowest sliver a lattice can cut.
    ExPolygon region;
    region.contour.points = { Point(scale_(0.209), scale_(-0.209)), Point(scale_(1.259), scale_(-0.209)),
                              Point(scale_(1.259), scale_(0.341)),  Point(scale_(0.209), scale_(0.341)) };

    const MiniatureSupport::Witnesses cells = MiniatureSupport::witness_cells(region, 0.42);
    // Half the resolved support extrusion width, and nothing else.
    REQUIRE(cells.pitch == coord_t(scale_(0.21)));
    // 6 lattice columns and 3 rows meet the rectangle, and every square that meets it keeps a cell.
    REQUIRE(cells.size() == 18);

    double                             sum = 0., smallest = std::numeric_limits<double>::max();
    std::set<std::pair<coord_t, coord_t>> seen;
    for (const MiniatureSupport::WitnessCell &cell : cells.cells) {
        INFO("cell " << cell.index.x() << "," << cell.index.y());
        // One cell per lattice square, never two.
        REQUIRE(seen.insert({ cell.index.x(), cell.index.y() }).second);
        REQUIRE(! cell.area.empty());
        double a = 0.;
        for (const ExPolygon &piece : cell.area) {
            a += piece.area();
            // The square the cell came from is anchored at the scaled coordinate origin: its corner
            // is index * pitch, so a region that starts anywhere gets the same lattice lines.
            const BoundingBox b = get_extents(piece);
            REQUIRE(b.min.x() >= cell.index.x() * cells.pitch);
            REQUIRE(b.max.x() <= (cell.index.x() + 1) * cells.pitch);
            REQUIRE(b.min.y() >= cell.index.y() * cells.pitch);
            REQUIRE(b.max.y() <= (cell.index.y() + 1) * cells.pitch);
        }
        REQUIRE(a > 0.);
        sum += a;
        smallest = std::min(smallest, a);
    }
    // The cells tile the region: their areas add up to it exactly.
    REQUIRE_THAT(sum * SCALING_FACTOR * SCALING_FACTOR, WithinRel(1.05 * 0.55, 1e-6));
    // And the narrow boundary slivers are kept rather than dropped: the thinnest cell is the
    // 0.001 x 0.131 mm corner of the left column, a thousandth of a whole square.
    REQUIRE(smallest < 0.01 * double(cells.pitch) * double(cells.pitch));

    // Twice the extrusion width is twice the pitch, so the same region resolves to strictly fewer cells.
    const MiniatureSupport::Witnesses coarse = MiniatureSupport::witness_cells(region, 0.84);
    REQUIRE(coarse.pitch == coord_t(scale_(0.42)));
    REQUIRE(coarse.size() < cells.size());

    // Nothing to witness: no polygon, or no resolved width to derive a pitch from.
    REQUIRE(MiniatureSupport::witness_cells(ExPolygon(), 0.42).empty());
    REQUIRE(MiniatureSupport::witness_cells(region, 0.).empty());

    // A region too narrow for one extrusion carries no lattice at all: `witness_cells` would keep the
    // clipped slivers of every square the sliver meets, and `region_witnesses` hands back nothing
    // instead, so nothing is ever counted for or against a place no line can be laid in.
    MiniatureSupport::Problem narrow;
    narrow.extrusion_width_mm = 0.42;
    add_region(narrow, rect_mm(0., 0., 0.3, 1.), 1, 1.0, 1.);
    REQUIRE(narrow.regions[0].printable == false);
    REQUIRE(MiniatureSupport::region_witnesses(narrow.regions[0], 0.42)->empty());

    MiniatureSupport::Problem wide;
    wide.extrusion_width_mm = 0.42;
    add_region(wide, rect_mm(0., 0., 0.6, 1.), 1, 1.0, 1.);
    REQUIRE(wide.regions[0].printable == true);
    REQUIRE(! MiniatureSupport::region_witnesses(wide.regions[0], 0.42)->empty());
}

TEST_CASE("Legal reach halves the smaller active limit and no influence crosses a hole or a notch", "[MiniatureSupport][MiniatureContacts]")
{
    // Both limits positive: half the smaller of the two, whichever way round they are given.
    REQUIRE_THAT(MiniatureSupport::legal_reach(5., 10.), WithinAbs(2.5, 1e-12));
    REQUIRE_THAT(MiniatureSupport::legal_reach(10., 5.), WithinAbs(2.5, 1e-12));
    // One of them switched off: half the one that is left.
    REQUIRE_THAT(MiniatureSupport::legal_reach(5., 0.), WithinAbs(2.5, 1e-12));
    REQUIRE_THAT(MiniatureSupport::legal_reach(0., 3.), WithinAbs(1.5, 1e-12));
    // Neither positive: no reach at all rather than an unbounded one.
    REQUIRE_THAT(MiniatureSupport::legal_reach(0., 0.), WithinAbs(0., 1e-12));
    REQUIRE_THAT(MiniatureSupport::legal_reach(-2., -1.), WithinAbs(0., 1e-12));

    // A 10 x 2 mm bar with a contact 1 mm in from its left end and a 2 mm reach.
    ExPolygon bar;
    bar.contour.points = { Point(scale_(0.), scale_(0.)), Point(scale_(10.), scale_(0.)),
                           Point(scale_(10.), scale_(2.)), Point(scale_(0.), scale_(2.)) };
    const MiniatureSupport::Witnesses bar_cells = MiniatureSupport::witness_cells(bar, 0.42);
    const Point                       contact(scale_(1.), scale_(1.));
    const std::vector<bool>           reached = MiniatureSupport::covered_by_contact(bar, bar_cells, contact, 2.);
    REQUIRE(reached.size() == bar_cells.size());

    size_t covered = 0, centre_only = 0;
    for (size_t i = 0; i < bar_cells.size(); ++ i) {
        const MiniatureSupport::WitnessCell &cell = bar_cells.cells[i];
        double farthest = 0.;
        for (const Point &p : cell.corners)
            farthest = std::max(farthest, (p - contact).cast<double>().norm());
        const double centre = (cell.area.front().contour.centroid() - contact).cast<double>().norm();
        if (reached[i]) {
            // The complete cell lies within the reach, so its farthest vertex does.
            ++ covered;
            REQUIRE(farthest <= scale_(2.) + 1e-6);
        } else if (centre <= scale_(2.))
            // A cell the reach cuts through: sampling its centre alone would have called it covered.
            ++ centre_only;
    }
    REQUIRE(covered > 0);
    REQUIRE(centre_only > 0);
    // A non-positive reach anchors nothing, whatever the geometry says.
    const std::vector<bool> none = MiniatureSupport::covered_by_contact(bar, bar_cells, contact, 0.);
    REQUIRE(std::find(none.begin(), none.end(), true) == none.end());

    // A 10 x 6 mm plate with a 6 x 4 mm hole through it. The contact sits at (1, 3), left of the hole,
    // and the reach is 10 mm, more than the 9.49 mm to the plate's far corners: everything beyond the
    // hole is inside the reach as a disk, and none of it may be claimed.
    ExPolygon plate;
    plate.contour.points = { Point(scale_(0.), scale_(0.)), Point(scale_(10.), scale_(0.)),
                             Point(scale_(10.), scale_(6.)), Point(scale_(0.), scale_(6.)) };
    Polygon hole;
    hole.points = { Point(scale_(2.), scale_(1.)), Point(scale_(2.), scale_(5.)),
                    Point(scale_(8.), scale_(5.)), Point(scale_(8.), scale_(1.)) };
    plate.holes.push_back(hole);
    const MiniatureSupport::Witnesses plate_cells = MiniatureSupport::witness_cells(plate, 0.42);
    const Point                       plate_contact(scale_(1.), scale_(3.));
    const std::vector<bool>           plate_reached = MiniatureSupport::covered_by_contact(plate, plate_cells, plate_contact, 10.);
    size_t                            plate_covered = 0, beyond = 0;
    for (size_t i = 0; i < plate_cells.size(); ++ i) {
        const MiniatureSupport::WitnessCell &cell = plate_cells.cells[i];
        const bool far_side = cell.bbox.min.x() >= coord_t(scale_(8.));
        if (far_side)
            ++ beyond;
        if (plate_reached[i]) {
            ++ plate_covered;
            INFO("cell " << cell.index.x() << "," << cell.index.y());
            REQUIRE(! far_side);
        }
    }
    REQUIRE(plate_covered > 0);
    REQUIRE(beyond > 0);

    // A U: the same 10 x 6 mm outline with a 4 x 4 mm notch cut out of its top edge. The contact sits
    // in the left arm and the right arm is well inside a 9 mm disk, but every straight line to it
    // leaves the region through the notch.
    ExPolygon u;
    u.contour.points = { Point(scale_(0.), scale_(0.)), Point(scale_(10.), scale_(0.)),
                         Point(scale_(10.), scale_(6.)), Point(scale_(7.), scale_(6.)),
                         Point(scale_(7.), scale_(2.)),  Point(scale_(3.), scale_(2.)),
                         Point(scale_(3.), scale_(6.)),  Point(scale_(0.), scale_(6.)) };
    const MiniatureSupport::Witnesses u_cells = MiniatureSupport::witness_cells(u, 0.42);
    const Point                       u_contact(scale_(1.5), scale_(4.));
    const std::vector<bool>           u_reached = MiniatureSupport::covered_by_contact(u, u_cells, u_contact, 9.);
    size_t                            u_covered = 0, other_arm = 0;
    for (size_t i = 0; i < u_cells.size(); ++ i) {
        const MiniatureSupport::WitnessCell &cell = u_cells.cells[i];
        const bool right_arm = cell.bbox.min.x() >= coord_t(scale_(7.)) && cell.bbox.min.y() >= coord_t(scale_(2.));
        if (right_arm)
            ++ other_arm;
        if (u_reached[i]) {
            ++ u_covered;
            INFO("cell " << cell.index.x() << "," << cell.index.y());
            REQUIRE(! right_arm);
        }
    }
    REQUIRE(u_covered > 0);
    REQUIRE(other_arm > 0);

    // A convex region with no hole holds every segment between two of its own points, but only its
    // own points: a contact 1 mm off the bar's left end anchors nothing in it, because every segment
    // from the contact into the bar starts outside the bar.
    const Point             outside_contact(scale_(-1.), scale_(1.));
    const std::vector<bool> from_outside = MiniatureSupport::covered_by_contact(bar, bar_cells, outside_contact, 6.);
    REQUIRE(from_outside.size() == bar_cells.size());
    size_t within_reach = 0;
    for (size_t i = 0; i < bar_cells.size(); ++ i) {
        double farthest = 0.;
        for (const Point &p : bar_cells.cells[i].corners)
            farthest = std::max(farthest, (p - outside_contact).cast<double>().norm());
        if (farthest <= scale_(6.))
            ++ within_reach;
    }
    // Cells a disk of that reach would have swept up, so the check is not vacuous.
    REQUIRE(within_reach > 0);
    REQUIRE(std::find(from_outside.begin(), from_outside.end(), true) == from_outside.end());
}

TEST_CASE("Contact selection decimates by main's distance rule and restores one contact per uncovered printable region",
          "[MiniatureSupport][MiniatureContacts]")
{
    // The problems here are written out rather than sliced, so the geometry each rule runs on is a
    // stated number and not whatever a fixture happens to produce. Nothing is measured off the model,
    // so no contact may be moved and what every leg sees is the decimation and the add-back alone.
    const auto ids = [](const MiniatureSupport::Selection &s) {
        std::vector<uint64_t> out;
        for (const MiniatureSupport::ContactSeed &seed : s.retained)
            out.push_back(seed.id);
        std::sort(out.begin(), out.end());
        return out;
    };

    // Leg 1, the order and the bound. Seven contacts 0.5 mm apart along one 3 x 1 mm band: walking them
    // up the order, every second one lies strictly inside the 1 mm distance of the one already kept and
    // goes, while the next at exactly 1 mm does not. A critical contact decimates like any other, so the
    // one marked critical here is dropped, and the band's cells all lie within reach of the survivors, so
    // nothing is put back.
    MiniatureSupport::Problem row;
    row.extrusion_width_mm      = 0.42;
    add_region(row, rect_mm(0., 0., 3., 1.), 1, 1.0, 5.);
    for (int i = 0; i <= 6; ++ i)
        add_seed(row, 0, 0.5 * double(i), 0.5, false, i == 1);
    row.contact_min_distance_mm = 1.0;
    const MiniatureSupport::Selection decimated = MiniatureSupport::select_contacts(row, unmeasured_field());
    REQUIRE(ids(decimated) == std::vector<uint64_t>{ 0, 2, 4, 6 });
    REQUIRE(decimated.seeds_restored == 0);

    // Leg 2, radius before id. Two contacts 0.5 mm apart on the same band: the fatter one is settled
    // first and the thinner one falls to it, whichever id they carry.
    MiniatureSupport::Problem radius;
    radius.extrusion_width_mm      = 0.42;
    add_region(radius, rect_mm(0., 0., 3., 1.), 1, 1.0, 5.);
    add_seed(radius, 0, 0.,  0.5, false, false, 0.4);
    add_seed(radius, 0, 0.5, 0.5, false, false, 0.8);
    radius.contact_min_distance_mm = 1.0;
    const MiniatureSupport::Selection fatter = MiniatureSupport::select_contacts(radius, unmeasured_field());
    REQUIRE(ids(fatter) == std::vector<uint64_t>{ 1 });

    // Leg 3, contact z before id, with the lower region listed second. The upper region's contact is
    // judged against the lower one already settled below it, 1 mm away in z and inside the 2 mm distance,
    // and the lower contact carries every cell of the upper region, so nothing is put back.
    MiniatureSupport::Problem stacked;
    stacked.extrusion_width_mm      = 0.42;
    add_region(stacked, rect_mm(0., 0., 0.5, 0.5), 2, 2.0, 5.);
    add_region(stacked, rect_mm(0., 0., 0.5, 0.5), 1, 1.0, 5.);
    stacked.regions[1].component = 0;
    add_seed(stacked, 0, 0.25, 0.25);
    add_seed(stacked, 1, 0.25, 0.25);
    stacked.contact_min_distance_mm = 2.0;
    const MiniatureSupport::Selection lower_first = MiniatureSupport::select_contacts(stacked, unmeasured_field());
    REQUIRE(ids(lower_first) == std::vector<uint64_t>{ 1 });
    REQUIRE(lower_first.seeds_restored == 0);

    // Leg 4, a pinned contact. It is kept whatever stands near it and it crowds nobody: the contact
    // 0.5 mm from it survives, and the one 0.7 mm from that survivor does not.
    MiniatureSupport::Problem pinned;
    pinned.extrusion_width_mm      = 0.42;
    add_region(pinned, rect_mm(0., 0., 3., 1.), 1, 1.0, 5.);
    add_seed(pinned, 0, 0.,  0.5, true);
    add_seed(pinned, 0, 0.5, 0.5);
    add_seed(pinned, 0, 1.2, 0.5);
    pinned.contact_min_distance_mm = 1.0;
    const MiniatureSupport::Selection with_pin = MiniatureSupport::select_contacts(pinned, unmeasured_field());
    REQUIRE(ids(with_pin) == std::vector<uint64_t>{ 0, 1 });

    // Leg 5, the add-back. A 4.6 mm bar between two pads one layer under it, all one component: both of
    // the bar's contacts stand within 1.5 mm in 3-D of a pad contact and are decimated, but the bar's
    // middle cells are over 2 mm from either pad and nothing is left behind them, so the bar's lowest
    // contact comes back.
    const auto bar_problem = []() {
        MiniatureSupport::Problem p;
        p.extrusion_width_mm = 0.42;
        add_region(p, rect_mm(0., 0., 0.5, 0.5), 1, 1.0, 5.);      // left pad
        add_region(p, rect_mm(0.6, 0., 5.2, 0.6), 2, 1.2, 5.);     // the bar
        add_region(p, rect_mm(5.3, 0., 5.8, 0.5), 1, 1.0, 5.);     // right pad
        p.regions[1].component = p.regions[2].component = 0;
        add_seed(p, 0, 0.25, 0.25);
        add_seed(p, 1, 0.6,  0.3);
        add_seed(p, 1, 5.2,  0.3);
        add_seed(p, 2, 5.55, 0.25);
        p.contact_min_distance_mm = 1.5;
        return p;
    };
    const MiniatureSupport::Problem  bar       = bar_problem();
    const MiniatureSupport::Selection restored = MiniatureSupport::select_contacts(bar, unmeasured_field());
    REQUIRE(ids(restored) == std::vector<uint64_t>{ 0, 1, 3 });
    REQUIRE(restored.seeds_restored == 1);

    // Leg 6, a requested distance of nothing is a preference for nothing: every contact stays where the
    // generator put it and nothing was put back, because nothing was taken.
    MiniatureSupport::Problem loose  = bar_problem();
    loose.contact_min_distance_mm    = 0.;
    const MiniatureSupport::Selection whole = MiniatureSupport::select_contacts(loose, unmeasured_field());
    REQUIRE(ids(whole) == std::vector<uint64_t>{ 0, 1, 2, 3 });
    REQUIRE(whole.seeds_restored == 0);
    const std::vector<Point> unmoved = retained_positions(loose, whole);
    for (size_t s = 0; s < loose.seeds.size(); ++ s) {
        INFO("seed " << s);
        REQUIRE(unmoved[s] == loose.seeds[s].position);
    }

    // Leg 7, the same problem with no legal reach at all. The carry reads the contact distance and never
    // the reach, so the decimation is unchanged and the add-back still finds the bar's cells uncovered;
    // with nothing measured off the model, no contact moves either.
    MiniatureSupport::Problem reachless = bar_problem();
    for (MiniatureSupport::RequiredRegion &region : reachless.regions)
        region.legal_reach_mm = 0.;
    const MiniatureSupport::Selection unreached = MiniatureSupport::select_contacts(reachless, unmeasured_field());
    REQUIRE(ids(unreached) == std::vector<uint64_t>{ 0, 1, 3 });
    REQUIRE(unreached.seeds_restored == 1);
    const std::vector<Point> still = retained_positions(reachless, unreached);
    for (size_t s = 0; s < reachless.seeds.size(); ++ s) {
        INFO("seed " << s);
        REQUIRE(still[s] == reachless.seeds[s].position);
    }

    // Leg 8, a problem with nothing in it selects nothing.
    MiniatureSupport::Problem empty;
    empty.extrusion_width_mm = 0.42;
    const MiniatureSupport::Selection nothing = MiniatureSupport::select_contacts(empty, unmeasured_field());
    REQUIRE(nothing.retained.empty());
    REQUIRE(nothing.seeds_restored == 0);
}
