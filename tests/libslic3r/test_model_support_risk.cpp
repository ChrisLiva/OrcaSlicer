#include <catch2/catch_all.hpp>

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Support/ModelSupportRisk.hpp"
#include "libslic3r/libslic3r.h"

#include <cmath>
#include <limits>
#include <vector>

using namespace Slic3r;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

// An axis-aligned rectangle in mm, as an ExPolygon in scaled object coordinates.
ExPolygon rect_mm(double x0, double y0, double x1, double y1)
{
    ExPolygon out;
    out.contour.points = { Point(scale_(x0), scale_(y0)), Point(scale_(x1), scale_(y0)),
                           Point(scale_(x1), scale_(y1)), Point(scale_(x0), scale_(y1)) };
    return out;
}

Point pt_mm(double x, double y) { return Point(scale_(x), scale_(y)); }

} // namespace

TEST_CASE("Local model width reads the solid the query sits in and not the overhang under it", "[ModelSupportRisk]")
{
    SECTION("a 4 mm strip and a 0.8 mm strip of the same length measure their own widths")
    {
        // The same 20 mm long band, once 4 mm deep and once 0.8 mm deep. The query sits at the middle
        // of each, where the medial axis runs down the spine and the width is the strip's depth.
        double thick = 0., thin = 0.;
        REQUIRE(ModelSupportRisk::local_width(rect_mm(0., 0., 20., 4.), pt_mm(10., 2.), &thick));
        REQUIRE(ModelSupportRisk::local_width(rect_mm(0., 0., 20., 0.8), pt_mm(10., 0.4), &thin));
        CHECK_THAT(thick, WithinAbs(4.0, 0.05));
        CHECK_THAT(thin, WithinAbs(0.8, 0.05));
        CHECK(thick > thin + 1.);
    }

    SECTION("the query's own clearance caps the width, so a tip is not read as the body's thickness")
    {
        // 0.6 mm from the far end of the 4 mm strip: the strip is still 4 mm deep, but no circle
        // wider than 1.2 mm fits inside the solid around the query.
        double at_tip = 0.;
        REQUIRE(ModelSupportRisk::local_width(rect_mm(0., 0., 20., 4.), pt_mm(19.4, 2.), &at_tip));
        CHECK(at_tip <= 1.2 + 1e-6);
        CHECK(at_tip > 0.);
    }

    SECTION("a solid narrow nowhere reads the widest circle that fits around the query")
    {
        // MedialAxis drops every skeleton edge whose two walls are more than PI/8 off facing unless
        // the edge is shorter than min_width, and min_width is zero here, so a 4 mm square has no
        // skeleton at all. Its width is still 4 mm in the middle and 1 mm half a millimetre in.
        double middle = 0., near_wall = 0.;
        REQUIRE(ModelSupportRisk::local_width(rect_mm(0., 0., 4., 4.), pt_mm(2., 2.), &middle));
        REQUIRE(ModelSupportRisk::local_width(rect_mm(0., 0., 4., 4.), pt_mm(0.5, 2.), &near_wall));
        CHECK_THAT(middle, WithinAbs(4.0, 1e-6));
        CHECK_THAT(near_wall, WithinAbs(1.0, 1e-6));
        // And a query the square does not hold stays unmeasured all the same.
        double outside = 0.;
        CHECK_FALSE(ModelSupportRisk::local_width(rect_mm(0., 0., 4., 4.), pt_mm(9., 2.), &outside));
    }

    SECTION("a query outside the solid and a degenerate solid are answered as unmeasured")
    {
        double width = -7.;
        CHECK_FALSE(ModelSupportRisk::local_width(rect_mm(0., 0., 20., 4.), pt_mm(30., 2.), &width));
        CHECK_FALSE(ModelSupportRisk::local_width(ExPolygon(), pt_mm(1., 1.), &width));
        CHECK_FALSE(ModelSupportRisk::local_width(rect_mm(0., 0., 20., 0.), pt_mm(10., 0.), &width));
        // Nothing was written: the caller's value survives an unmeasured query untouched.
        CHECK(width == -7.);
    }
}

namespace {

// One 1 mm slab of a model, from parts that Clipper unions into whatever islands they make.
ModelSupportRisk::Slice slab(double bottom_mm, std::initializer_list<ExPolygon> parts)
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

const ExPolygon torso_mm = rect_mm(0., 0., 4., 4.);
const ExPolygon arm_mm   = rect_mm(4., 1.6, 8., 2.4);   // 0.8 mm thick: the hand attachment
const ExPolygon blade_mm = rect_mm(8., 0., 9., 3.);     // 1.0 mm thick, hanging in mid air

// Six slabs. The blade appears in mid air on slabs 3 and 4, and only on slab 5 does the arm join it
// to the torso, so the blade's own material never touches anything below it.
std::vector<ModelSupportRisk::Slice> hanging_weapon()
{
    std::vector<ModelSupportRisk::Slice> slices;
    for (size_t l = 0; l < 6; ++ l) {
        if (l == 5)
            slices.push_back(slab(double(l), { torso_mm, arm_mm, blade_mm }));
        else if (l >= 3)
            slices.push_back(slab(double(l), { torso_mm, blade_mm }));
        else
            slices.push_back(slab(double(l), { torso_mm }));
    }
    return slices;
}

} // namespace

TEST_CASE("A hanging weapon reaches the bed upward through its hand and the hand is the neck", "[ModelSupportRisk]")
{
    const auto never_stop = []() { return false; };

    SECTION("the blade travels up to the arm and down the torso, and reports the arm as its neck")
    {
        const ModelSupportRisk::Field field = ModelSupportRisk::build(hanging_weapon(), 0.42, never_stop);
        REQUIRE(field.status == ModelSupportRisk::Field::Status::Complete);

        // Mid-blade on the lowest slab the blade exists on: nothing under it, so the only path to the
        // bed climbs to the arm on slab 5 and comes back down the torso.
        const ModelSupportRisk::Sample sample = ModelSupportRisk::sample(field, 3, pt_mm(8.5, 1.5));
        REQUIRE(sample.status == ModelSupportRisk::Sample::Status::Known);
        CHECK_THAT(sample.local_width_mm, WithinAbs(1.0, 0.05));
        // The 0.8 mm arm, not the 1.0 mm blade it came from and not the 4 mm torso it ends on.
        CHECK_THAT(sample.neck_width_mm, WithinAbs(0.8, 0.05));
        // Two 1 mm slabs of climbing before the arm is even reached.
        CHECK(sample.lever_mm >= 2. - 1e-6);
        CHECK(sample.lever_mm < 12.);
        CHECK(std::isfinite(sample.risk_per_mm2));
    }

    SECTION("a blade with no arm at all reaches no bed and is unmeasured, not scored")
    {
        // The same model with slab 5's arm taken out: the blade is an island in mid air.
        std::vector<ModelSupportRisk::Slice> slices = hanging_weapon();
        slices[5] = slab(5., { torso_mm, blade_mm });
        const ModelSupportRisk::Field field = ModelSupportRisk::build(slices, 0.42, never_stop);
        REQUIRE(field.status == ModelSupportRisk::Field::Status::Complete);
        CHECK(ModelSupportRisk::sample(field, 3, pt_mm(8.5, 1.5)).status == ModelSupportRisk::Sample::Status::Unknown);
        // The torso itself still resolves, so the unknown is the blade's and not the field's.
        CHECK(ModelSupportRisk::sample(field, 3, pt_mm(2., 2.)).status == ModelSupportRisk::Sample::Status::Known);
    }

    SECTION("a model lifted onto a raft roots at its own first slab")
    {
        // A raft under the object puts its first slab 5 mm up; the raft is the ground it stands on.
        std::vector<ModelSupportRisk::Slice> slices = hanging_weapon();
        for (ModelSupportRisk::Slice &slice : slices) {
            slice.bottom_z_mm += 5.;
            slice.top_z_mm    += 5.;
        }
        const ModelSupportRisk::Field field = ModelSupportRisk::build(slices, 0.42, never_stop);
        REQUIRE(field.status == ModelSupportRisk::Field::Status::Complete);
        CHECK(ModelSupportRisk::sample(field, 0, pt_mm(2., 2.)).status == ModelSupportRisk::Sample::Status::Known);
        // The blade still reaches that ground up through the arm and down the torso.
        CHECK(ModelSupportRisk::sample(field, 3, pt_mm(8.5, 1.5)).status == ModelSupportRisk::Sample::Status::Known);
    }

    SECTION("a stopped build is canceled, which is not the same answer as unmeasured geometry")
    {
        const ModelSupportRisk::Field field = ModelSupportRisk::build(hanging_weapon(), 0.42, []() { return true; });
        CHECK(field.status == ModelSupportRisk::Field::Status::Canceled);
    }

    SECTION("nonfinite or inverted slab bounds are invalid input, not a measurement")
    {
        std::vector<ModelSupportRisk::Slice> nan_z = hanging_weapon();
        nan_z[2].top_z_mm = std::numeric_limits<double>::quiet_NaN();
        CHECK(ModelSupportRisk::build(nan_z, 0.42, never_stop).status == ModelSupportRisk::Field::Status::Invalid);

        std::vector<ModelSupportRisk::Slice> inverted = hanging_weapon();
        inverted[2].top_z_mm = inverted[2].bottom_z_mm - 1.;
        CHECK(ModelSupportRisk::build(inverted, 0.42, never_stop).status == ModelSupportRisk::Field::Status::Invalid);

        // No slices at all is an empty model, not malformed input, and it answers every query with
        // no measurement rather than with a zero.
        const ModelSupportRisk::Field empty = ModelSupportRisk::build({}, 0.42, never_stop);
        CHECK(empty.status == ModelSupportRisk::Field::Status::Complete);
        CHECK(ModelSupportRisk::sample(empty, 0, pt_mm(1., 1.)).status == ModelSupportRisk::Sample::Status::Unknown);
    }
}


TEST_CASE("The risk weight rises with thinner material or a narrower neck or a longer lever", "[ModelSupportRisk]")
{
    const double w = 0.42;   // the resolved support extrusion width

    SECTION("the weight is the stated dimensionless expression, and material at or over the extrusion width is not rewarded for being thicker")
    {
        // (w / max(t, w)) * (1 + L / max(n, w)) at t = 2, n = 1, L = 3.
        CHECK_THAT(ModelSupportRisk::risk_weight(w, 2., 1., 3.), WithinRel((0.42 / 2.) * (1. + 3. / 1.), 1e-12));
        // Nothing anywhere: a contact on thick material with no lever weighs w / t alone.
        CHECK_THAT(ModelSupportRisk::risk_weight(w, 2., 1., 0.), WithinRel(0.42 / 2., 1e-12));
        // t below w clamps to w, so the expression cannot report more than 1 for the width term. The
        // Sample's BelowPrintableWidth status is what keeps that from reading as printable.
        CHECK_THAT(ModelSupportRisk::risk_weight(w, 0.2, 1., 0.), WithinRel(1.0, 1e-12));
        CHECK_THAT(ModelSupportRisk::risk_weight(w, 0.05, 1., 0.), WithinRel(1.0, 1e-12));
    }

    SECTION("thinner material, a narrower neck and a longer lever each raise it on their own")
    {
        const double base = ModelSupportRisk::risk_weight(w, 4., 2., 5.);
        CHECK(ModelSupportRisk::risk_weight(w, 2., 2., 5.) > base);   // half the local width
        CHECK(ModelSupportRisk::risk_weight(w, 4., 0.8, 5.) > base);  // a narrower neck
        CHECK(ModelSupportRisk::risk_weight(w, 4., 2., 9.) > base);   // a longer lever
        // Two identical 4 mm ends, one held by a 0.8 mm neck and one by a 2 mm neck at equal lever.
        CHECK(ModelSupportRisk::risk_weight(w, 4., 0.8, 6.) > ModelSupportRisk::risk_weight(w, 4., 2., 6.));
    }

    SECTION("a misuse answers NaN, so it can never read as an absence of risk")
    {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        CHECK(std::isnan(ModelSupportRisk::risk_weight(0., 2., 1., 3.)));
        CHECK(std::isnan(ModelSupportRisk::risk_weight(-1., 2., 1., 3.)));
        CHECK(std::isnan(ModelSupportRisk::risk_weight(w, nan, 1., 3.)));
        CHECK(std::isnan(ModelSupportRisk::risk_weight(w, 2., nan, 3.)));
        CHECK(std::isnan(ModelSupportRisk::risk_weight(w, 2., 1., nan)));
    }

    SECTION("a sample under one extrusion width keeps both measured widths and says so")
    {
        // The blade is 1.0 mm and its arm 0.8 mm, so a 1.2 mm extrusion cannot print either.
        const ModelSupportRisk::Field field = ModelSupportRisk::build(hanging_weapon(), 1.2, []() { return false; });
        REQUIRE(field.status == ModelSupportRisk::Field::Status::Complete);
        const ModelSupportRisk::Sample thin = ModelSupportRisk::sample(field, 3, pt_mm(8.5, 1.5));
        REQUIRE(thin.status == ModelSupportRisk::Sample::Status::BelowPrintableWidth);
        CHECK_THAT(thin.local_width_mm, WithinAbs(1.0, 0.05));
        CHECK_THAT(thin.neck_width_mm, WithinAbs(0.8, 0.05));
        CHECK(thin.risk_per_mm2 > 0.);
    }
}

namespace {

// A 20 x 4 mm strip on the bed, a 2 mm thick rectangular frame over it, and a slab with nothing on
// it at all. The strip is the ground the frame's path ends on, and the frame's hole gives a query a
// rim to sit just inside of.
std::vector<ModelSupportRisk::Slice> strip_and_frame()
{
    ExPolygon frame = rect_mm(0., 0., 20., 12.);
    frame.holes.push_back(rect_mm(2., 2., 18., 10.).contour);
    frame.holes.front().reverse();
    return { slab(0., { rect_mm(0., 0., 20., 4.) }), slab(1., { frame }), slab(2., {}) };
}

} // namespace

TEST_CASE("A query within half an extrusion width of a solid reads that solid and not an absence", "[ModelSupportRisk]")
{
    const auto never_stop = []() { return false; };
    // At a 0.42 mm extrusion the rule reaches 0.21 mm off the boundary.
    const ModelSupportRisk::Field field = ModelSupportRisk::build(strip_and_frame(), 0.42, never_stop);
    REQUIRE(field.status == ModelSupportRisk::Field::Status::Complete);
    // The frame kept its hole through the union, so the rim query below really sits in a void.
    REQUIRE(field.slices[1].solids.size() == 1);
    REQUIRE(field.slices[1].solids.front().holes.size() == 1);
    REQUIRE(field.slices[2].solids.empty());

    // Mid-strip, well inside it: the reading the tree already gives, and the width every query below
    // is measured against.
    const ModelSupportRisk::Sample inside = ModelSupportRisk::sample(field, 0, pt_mm(10., 2.));
    REQUIRE(inside.status == ModelSupportRisk::Sample::Status::Known);
    CHECK_THAT(inside.local_width_mm, WithinAbs(4.0, 0.05));

    // 0.05 mm off the strip's long edge: inside the bound, so it stands on the strip and reads the
    // strip's own width rather than nothing.
    const ModelSupportRisk::Sample near_edge = ModelSupportRisk::sample(field, 0, pt_mm(10., -0.05));
    CHECK(near_edge.status == ModelSupportRisk::Sample::Status::Known);
    CHECK_THAT(near_edge.local_width_mm, WithinAbs(4.0, 0.05));
    CHECK(near_edge.risk_per_mm2 > 0.);

    // 0.30 mm off the same edge: past the bound, and still no measurement.
    const ModelSupportRisk::Sample far_edge = ModelSupportRisk::sample(field, 0, pt_mm(10., -0.30));
    CHECK(far_edge.status == ModelSupportRisk::Sample::Status::Unknown);
    CHECK(far_edge.missing == ModelSupportRisk::Sample::Missing::OutsideSolids);

    // 0.05 mm inside the frame's hole: a void is the same distance as a void outside the outline,
    // and the 2 mm wall the query hangs off is what it reads.
    const ModelSupportRisk::Sample in_hole = ModelSupportRisk::sample(field, 1, pt_mm(10., 9.95));
    CHECK(in_hole.status == ModelSupportRisk::Sample::Status::Known);
    CHECK_THAT(in_hole.local_width_mm, WithinAbs(2.0, 0.05));

    // A slab with no islands has nothing to attach to, however near the query is to a solid one
    // layer down.
    CHECK(ModelSupportRisk::sample(field, 2, pt_mm(10., -0.05)).status == ModelSupportRisk::Sample::Status::Unknown);
    CHECK(ModelSupportRisk::sample(field, 2, pt_mm(10., 2.)).missing == ModelSupportRisk::Sample::Missing::OutsideSolids);
}

namespace {

// Two 4 x 20 mm ends held off a 6 x 20 mm trunk by necks 6 mm long, 0.8 mm on the left and 2.0 mm on
// the right. Only the trunk stands on the bed, so both ends hang off their own neck and differ in
// nothing else. `left_neck_mm` is the left neck's thickness, so the pair can be made identical.
std::vector<ModelSupportRisk::Slice> two_necks(double left_neck_mm)
{
    const ExPolygon left_end   = rect_mm(0., 0., 4., 20.);
    const ExPolygon left_neck  = rect_mm(4., 10. - left_neck_mm * 0.5, 10., 10. + left_neck_mm * 0.5);
    const ExPolygon trunk      = rect_mm(10., 0., 16., 20.);
    const ExPolygon right_neck = rect_mm(16., 9., 22., 11.);
    const ExPolygon right_end  = rect_mm(22., 0., 26., 20.);
    return { slab(0., { trunk }), slab(1., { trunk }),
             slab(2., { left_end, left_neck, trunk, right_neck, right_end }) };
}

} // namespace

TEST_CASE("Model risk ranks a narrow neck or a long lever or thin material worse and unknown geometry as unknown", "[ModelSupportRisk]")
{
    const auto   never_stop = []() { return false; };
    const double w          = 0.42;

    SECTION("the same band reports its own width under a 4 mm solid and under a 0.8 mm solid")
    {
        const ExPolygon bar  = rect_mm(0., 0., 20., 4.);
        const ExPolygon band = rect_mm(0., 1.6, 20., 2.4);
        const ModelSupportRisk::Field thick =
            ModelSupportRisk::build({ slab(0., { bar }), slab(1., { bar }), slab(2., { bar }) }, w, never_stop);
        const ModelSupportRisk::Field thin =
            ModelSupportRisk::build({ slab(0., { bar }), slab(1., { bar }), slab(2., { band }) }, w, never_stop);
        const ModelSupportRisk::Sample over_thick = ModelSupportRisk::sample(thick, 2, pt_mm(10., 2.));
        const ModelSupportRisk::Sample over_thin  = ModelSupportRisk::sample(thin, 2, pt_mm(10., 2.));
        REQUIRE(over_thick.status == ModelSupportRisk::Sample::Status::Known);
        REQUIRE(over_thin.status == ModelSupportRisk::Sample::Status::Known);
        CHECK_THAT(over_thick.local_width_mm, WithinAbs(4.0, 0.05));
        CHECK_THAT(over_thin.local_width_mm, WithinAbs(0.8, 0.05));
        // The same contact area on the thinner solid is worth more risk.
        CHECK(over_thin.risk_per_mm2 > over_thick.risk_per_mm2);
    }

    SECTION("two identical 4 mm ends rank by the neck that holds them")
    {
        const ModelSupportRisk::Field field = ModelSupportRisk::build(two_necks(0.8), w, never_stop);
        REQUIRE(field.status == ModelSupportRisk::Field::Status::Complete);
        const ModelSupportRisk::Sample narrow = ModelSupportRisk::sample(field, 2, pt_mm(2., 10.));
        const ModelSupportRisk::Sample wide   = ModelSupportRisk::sample(field, 2, pt_mm(24., 10.));
        REQUIRE(narrow.status == ModelSupportRisk::Sample::Status::Known);
        REQUIRE(wide.status == ModelSupportRisk::Sample::Status::Known);
        // The ends are the same 4 mm bar; only the neck differs.
        CHECK_THAT(narrow.local_width_mm, WithinAbs(wide.local_width_mm, 0.05));
        CHECK_THAT(narrow.neck_width_mm, WithinAbs(0.8, 0.05));
        CHECK_THAT(wide.neck_width_mm, WithinAbs(2.0, 0.05));
        CHECK(narrow.risk_per_mm2 > wide.risk_per_mm2);

        // Made identical, the two ends rank identically: nothing but the neck was ever separating them.
        const ModelSupportRisk::Field matched = ModelSupportRisk::build(two_necks(2.0), w, never_stop);
        const ModelSupportRisk::Sample was_narrow = ModelSupportRisk::sample(matched, 2, pt_mm(2., 10.));
        const ModelSupportRisk::Sample still_wide = ModelSupportRisk::sample(matched, 2, pt_mm(24., 10.));
        REQUIRE(was_narrow.status == ModelSupportRisk::Sample::Status::Known);
        CHECK_THAT(was_narrow.risk_per_mm2, WithinRel(still_wide.risk_per_mm2, 1e-9));
    }

    SECTION("the same widths further from the neck weigh more")
    {
        const ModelSupportRisk::Field field = ModelSupportRisk::build(two_necks(0.8), w, never_stop);
        // Both queries sit on the 4 mm end's spine; the second is 8 mm further down it from the neck.
        const ModelSupportRisk::Sample close = ModelSupportRisk::sample(field, 2, pt_mm(2., 10.));
        const ModelSupportRisk::Sample far   = ModelSupportRisk::sample(field, 2, pt_mm(2., 2.));
        REQUIRE(close.status == ModelSupportRisk::Sample::Status::Known);
        REQUIRE(far.status == ModelSupportRisk::Sample::Status::Known);
        CHECK_THAT(far.local_width_mm, WithinAbs(close.local_width_mm, 0.05));
        CHECK_THAT(far.neck_width_mm, WithinAbs(close.neck_width_mm, 0.05));
        CHECK(far.lever_mm > close.lever_mm + 4.);
        CHECK(far.risk_per_mm2 > close.risk_per_mm2);
    }

    SECTION("a slice that reaches nothing below it, and a slice with no area, are unmeasured")
    {
        const ExPolygon bar = rect_mm(0., 0., 20., 4.);
        // The band lies 30 mm away in y: it overlaps no material under it, so no path reaches the bed.
        const ModelSupportRisk::Field adrift = ModelSupportRisk::build(
            { slab(0., { bar }), slab(1., { bar }), slab(2., { rect_mm(0., 30., 20., 30.8) }) }, w, never_stop);
        REQUIRE(adrift.status == ModelSupportRisk::Field::Status::Complete);
        CHECK(ModelSupportRisk::sample(adrift, 2, pt_mm(10., 30.4)).status == ModelSupportRisk::Sample::Status::Unknown);

        // A slice whose polygon encloses nothing: there is no width there to read.
        ModelSupportRisk::Slice flat;
        flat.bottom_z_mm = 2.;
        flat.top_z_mm    = 3.;
        flat.solids.push_back(rect_mm(0., 2., 20., 2.));
        const ModelSupportRisk::Field degenerate =
            ModelSupportRisk::build({ slab(0., { bar }), slab(1., { bar }), flat }, w, never_stop);
        REQUIRE(degenerate.status == ModelSupportRisk::Field::Status::Complete);
        CHECK(ModelSupportRisk::sample(degenerate, 2, pt_mm(10., 2.)).status == ModelSupportRisk::Sample::Status::Unknown);
    }
}
