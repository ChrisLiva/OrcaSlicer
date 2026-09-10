#include <catch2/catch_all.hpp>

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/AABBMesh.hpp"
#include "libslic3r/Support/ModelSupportRisk.hpp"
#include "libslic3r/TriangleMesh.hpp"
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

TEST_CASE("Local model width reads the solid the query sits in, not the overhang under it", "[ModelSupportRisk]")
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


TEST_CASE("The risk weight rises with thinner material, a narrower neck and a longer lever", "[ModelSupportRisk]")
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

TEST_CASE("Model risk ranks a narrow neck, a long lever and thin material worse, and unknown geometry as unknown", "[ModelSupportRisk]")
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

namespace {

// One axis-aligned box in mm, merged into `out`. The access cases are built from solid boxes, so a
// cavity is six of them rather than one inverted shell and every face keeps an outward normal.
void add_box(indexed_triangle_set &out, const Vec3d &min, const Vec3d &size)
{
    const indexed_triangle_set box  = its_make_cube(size.x(), size.y(), size.z());
    const int                  base = int(out.vertices.size());
    for (const Vec3f &vertex : box.vertices)
        out.vertices.push_back(vertex + min.cast<float>());
    for (const Vec3i32 &face : box.indices)
        out.indices.push_back(face + Vec3i32(base, base, base));
}

// A 10 x 10 x 2 mm plate at z 8..10: a contact placed at (5, 5, 8) hangs under the middle of it with
// nothing else anywhere, which is the open case the closed cavity, the windowed cavity with a limb
// across it, and the unmeasured leg are measured against.
indexed_triangle_set open_plate()
{
    indexed_triangle_set its;
    add_box(its, Vec3d(0., 0., 8.), Vec3d(10., 10., 2.));
    return its;
}

// A closed 6 x 6 x 6 mm shell of 1 mm walls around a 4 mm cavity whose centre is (3, 3, 3), with a
// `window` mm square hole through the middle of its +x wall when `window` is positive.
indexed_triangle_set cavity(double window)
{
    indexed_triangle_set its;
    add_box(its, Vec3d(0., 0., 0.), Vec3d(6., 6., 1.));   // floor
    add_box(its, Vec3d(0., 0., 5.), Vec3d(6., 6., 1.));   // ceiling
    add_box(its, Vec3d(0., 0., 1.), Vec3d(6., 1., 4.));   // -y
    add_box(its, Vec3d(0., 5., 1.), Vec3d(6., 1., 4.));   // +y
    add_box(its, Vec3d(0., 1., 1.), Vec3d(1., 4., 4.));   // -x
    if (window <= 0.) {
        add_box(its, Vec3d(5., 1., 1.), Vec3d(1., 4., 4.));
        return its;
    }
    const double lo = 3. - 0.5 * window, hi = 3. + 0.5 * window;
    add_box(its, Vec3d(5., 1., 1.),  Vec3d(1., lo - 1., 4.));
    add_box(its, Vec3d(5., hi, 1.),  Vec3d(1., 5. - hi, 4.));
    add_box(its, Vec3d(5., lo, 1.),  Vec3d(1., window, lo - 1.));
    add_box(its, Vec3d(5., lo, hi),  Vec3d(1., window, 5. - hi));
    return its;
}

// `layers` slabs, each carrying the same footprint.
std::vector<ExPolygons> stacked(const ExPolygon &footprint, size_t layers)
{
    return std::vector<ExPolygons>(layers, ExPolygons{ footprint });
}

// Print_z values from `first` upward in `step` mm, which is how a support stack hands its slabs over:
// each slab runs from the print_z under it.
std::vector<double> layer_tops(double first, double step, size_t layers)
{
    std::vector<double> tops;
    for (size_t i = 0; i < layers; ++ i)
        tops.push_back(first + double(i) * step);
    return tops;
}

const std::vector<ExPolygons> no_support;
const std::vector<double>     no_layers;

} // namespace

TEST_CASE("Removal access answers with the first clear direction of the fixed order", "[ModelSupportRisk]")
{
    const double probe = 0.3;

    SECTION("a contact under an open plate leaves along the first direction the order names")
    {
        const indexed_triangle_set     plate = open_plate();
        const AABBMesh                 mesh(plate);
        const ModelSupportRisk::Access access =
            ModelSupportRisk::assess_access(mesh, no_support, no_layers, Vec3d(5., 5., 8.), probe);
        REQUIRE(access.status == ModelSupportRisk::Access::Status::Clear);
        // The 26 directions run in lexicographic order of their integer triples, so the answer is
        // (-1, -1, -1) and not the straight drop that would read as the obvious way out.
        CHECK(access.direction.isApprox(Vec3d(-1., -1., -1.).normalized()));
        // The plate the contact hangs off is what the probe starts against, and it is excused only
        // within the probe radius of the contact: a direction that runs along the plate's underside
        // stays against it the whole way and is not clear.
        CHECK(access.direction.z() < 0.);
    }

    SECTION("a closed cavity is blocked, and a window through one wall is the way out")
    {
        const indexed_triangle_set shut = cavity(0.);
        const AABBMesh             closed(shut);
        CHECK(ModelSupportRisk::assess_access(closed, no_support, no_layers, Vec3d(3., 3., 3.), probe).status ==
              ModelSupportRisk::Access::Status::Blocked);

        const indexed_triangle_set     open_wall = cavity(1.2);
        const AABBMesh                 windowed(open_wall);
        const ModelSupportRisk::Access access =
            ModelSupportRisk::assess_access(windowed, no_support, no_layers, Vec3d(3., 3., 3.), probe);
        REQUIRE(access.status == ModelSupportRisk::Access::Status::Clear);
        // Five axis directions come before (1, 0, 0) in the order and meet the cavity's walls square-on,
        // and the diagonals before it meet them at an angle, so the 1.2 mm window is found as the axis
        // it is on.
        CHECK(access.direction.isApprox(Vec3d(1., 0., 0.)));
    }

    SECTION("a thin limb across the window blocks the direction its centre ray misses")
    {
        indexed_triangle_set its = cavity(1.2);
        // 0.1 mm thick, 0.19 mm off the axis the probe runs along: the centre line passes it and the
        // 0.3 mm capsule does not.
        add_box(its, Vec3d(5.2, 2.4, 3.19), Vec3d(0.2, 1.2, 0.1));
        const AABBMesh mesh(its);
        REQUIRE_FALSE(mesh.query_ray_hit(Vec3d(3. + probe, 3., 3.), Vec3d(1., 0., 0.)).is_hit());
        CHECK(ModelSupportRisk::assess_access(mesh, no_support, no_layers, Vec3d(3., 3., 3.), probe).status ==
              ModelSupportRisk::Access::Status::Blocked);
    }

    SECTION("what cannot be measured is unknown, and unknown is never blocked")
    {
        const indexed_triangle_set plate = open_plate();
        const AABBMesh             mesh(plate);
        const auto     unknown = [](const ModelSupportRisk::Access &access) {
            return access.status == ModelSupportRisk::Access::Status::Unknown && access.direction == Vec3d::Zero();
        };
        // A mesh with no triangles at all.
        const indexed_triangle_set nothing;
        const AABBMesh             empty(nothing);
        CHECK(unknown(ModelSupportRisk::assess_access(empty, no_support, no_layers, Vec3d(5., 5., 8.), probe)));
        // A nonfinite contact, and a probe radius that is not a radius.
        const double nan = std::numeric_limits<double>::quiet_NaN();
        CHECK(unknown(ModelSupportRisk::assess_access(mesh, no_support, no_layers, Vec3d(5., nan, 8.), probe)));
        CHECK(unknown(ModelSupportRisk::assess_access(mesh, no_support, no_layers, Vec3d(5., 5., 8.), 0.)));
        CHECK(unknown(ModelSupportRisk::assess_access(mesh, no_support, no_layers, Vec3d(5., 5., 8.), nan)));
        // Support layers whose two halves do not describe the same stack.
        CHECK(unknown(ModelSupportRisk::assess_access(mesh, stacked(rect_mm(0., 0., 1., 1.), 2), layer_tops(1., 0.2, 3),
                                                      Vec3d(5., 5., 8.), probe)));
        // A contact inside solid material: every probe would start buried, which is a start that
        // cannot leave the source patch rather than a way out that does not exist.
        indexed_triangle_set solid;
        add_box(solid, Vec3d(0., 0., 0.), Vec3d(6., 6., 6.));
        const AABBMesh buried(solid);
        CHECK(unknown(ModelSupportRisk::assess_access(buried, no_support, no_layers, Vec3d(3., 3., 3.), probe)));
    }
}

TEST_CASE("Printed support blocks removal access, bar the contact's own material at the contact", "[ModelSupportRisk]")
{
    const double               probe = 0.3;
    const indexed_triangle_set plate = open_plate();
    const AABBMesh             mesh(plate);
    const Vec3d                contact(5., 5., 8.);

    // Seven 0.2 mm slabs from z 6.0 to 7.4, well clear of the probe radius around the contact.
    const std::vector<double> tops = layer_tops(6.2, 0.2, 7);

    SECTION("two supports with a corridor between them leave the contact reachable")
    {
        std::vector<ExPolygons> corridor(tops.size());
        for (ExPolygons &layer : corridor)
            layer = ExPolygons{ rect_mm(1., 1., 4.4, 9.), rect_mm(5.6, 1., 9., 9.) };
        const ModelSupportRisk::Access access =
            ModelSupportRisk::assess_access(mesh, corridor, tops, contact, probe);
        REQUIRE(access.status == ModelSupportRisk::Access::Status::Clear);
        CHECK(access.direction.z() < 0.);
    }

    SECTION("the same two supports joined into one cage take the corridor away")
    {
        std::vector<ExPolygons> cage(tops.size());
        for (ExPolygons &layer : cage)
            layer = ExPolygons{ rect_mm(1., 1., 9., 9.) };
        CHECK(ModelSupportRisk::assess_access(mesh, cage, tops, contact, probe).status ==
              ModelSupportRisk::Access::Status::Blocked);
    }

    SECTION("the contact's own material inside the probe radius is not what stops it")
    {
        // Two slabs spanning z 7.85..7.95 carrying a 0.2 mm square under the contact: every point of
        // it lies inside the 0.3 mm sphere around the contact, which is the source component the
        // probe starts on.
        const std::vector<double> tip_tops = { 7.9, 7.95 };
        const std::vector<ExPolygons> tip = stacked(rect_mm(4.9, 4.9, 5.1, 5.1), tip_tops.size());
        CHECK(ModelSupportRisk::assess_access(mesh, tip, tip_tops, contact, probe).status ==
              ModelSupportRisk::Access::Status::Clear);

        // The same component carried on down to the plate is material outside that sphere, and it
        // stops the probe like any other support does.
        std::vector<ExPolygons> column = tip;
        std::vector<double>     column_tops = tip_tops;
        for (double z = 7.5; z > 5.9; z -= 0.2) {
            column.insert(column.begin(), ExPolygons{ rect_mm(3.5, 3.5, 6.5, 6.5) });
            column_tops.insert(column_tops.begin(), z);
        }
        REQUIRE(column.size() == column_tops.size());
        CHECK(ModelSupportRisk::assess_access(mesh, column, column_tops, contact, probe).status ==
              ModelSupportRisk::Access::Status::Blocked);
    }
}
