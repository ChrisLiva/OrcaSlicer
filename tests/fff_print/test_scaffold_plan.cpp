#include <catch2/catch_all.hpp>
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/SLA/IndexedMesh.hpp"
#include "libslic3r/Support/ScaffoldPlan.hpp"
#include <algorithm>
#include <memory>
#include <set>
#include "test_helpers.hpp"

using namespace Slic3r;
using namespace Slic3r::Test;
using namespace Slic3r::ScaffoldSupport;

namespace {

constexpr double width_mm = 0.42, xy_mm = 0.35, neck_mm = 1. + width_mm;

// `points` as a convex solid.
TriangleMesh hull(const std::vector<Vec3d> &points)
{
    std::vector<Vec3f> corners;
    for (const Vec3d &p : points)
        corners.push_back(p.cast<float>());
    return TriangleMesh(its_convex_hull(corners));
}

TriangleMesh box(double x, double y, double z, double sx, double sy, double sz)
{
    TriangleMesh mesh = make_cube(sx, sy, sz);
    mesh.translate(float(x), float(y), float(z));
    return mesh;
}

TriangleMesh merged(std::initializer_list<TriangleMesh> parts)
{
    TriangleMesh out;
    for (const TriangleMesh &part : parts)
        out.merge(part);
    return out;
}

// Slices `mesh` at `layer_mm`, 0.1 mm unless a case asks for another, with no support, so the plan reads plain object
// layers, and plans on the object's mesh with the config's 10 mm `max_bridge_length`, as `place_tips` does. A fixture
// coordinate maps into the sliced frame by the offset between the mesh's bounding box and the layers' extents; every
// fixture touches z 0.
struct Sliced
{
    Print                       print;
    Model                       model;
    std::unique_ptr<ObjectMesh> mesh;
    PlanInput                   input;
    Vec2d                       shift = Vec2d::Zero();   // sliced frame minus fixture frame, mm

    Sliced(const TriangleMesh &fixture, double layer_mm = 0.1)
    {
        TriangleMesh      copy   = fixture;
        const std::string height = std::to_string(layer_mm);
        init_print({ std::move(copy) }, print, model, fixture_config({ { "enable_support", "0" }, { "layer_height", height },
                                                                        { "initial_layer_print_height", height },
                                                                        { "layer_change_gcode", "G92 E0" } }));
        print.process();
        const PrintObject &object = *print.objects().front();
        BoundingBox        extents;
        // Above the first layer, which the elephant foot compensation may shrink.
        for (const Layer *layer : object.layers())
            if (layer->id() > 0 && ! layer->lslices.empty())
                extents.merge(get_extents(layer->lslices));
        const BoundingBoxf3 bb = fixture.bounding_box();
        shift = unscale(extents.min) - Vec2d(bb.min.x(), bb.min.y());
        mesh  = std::make_unique<ObjectMesh>(object);
        input = prepare_plan(object, width_mm, xy_mm, neck_mm, object.config().max_bridge_length.value, Geometry::deg2rad(21.), {},
                             mesh.get());
    }

    Plan plan() const { return plan_tips(input, {}); }
    // A tip's xy in the fixture frame.
    Vec2d at(const PlannedTip &tip) const { return unscale(tip.site.position) - shift; }
    // How far from straight down the builder aims a head at `tip`: the normal it reads off the mesh within one head
    // radius, one toolpath width, of the tip.
    double from_down_deg(const PlannedTip &tip) const
    {
        const Vec2d   xy = unscale(tip.site.position);
        sla::PointSet p(1, 3);
        p.row(0) = Vec3d(xy.x(), xy.y(), tip.site.print_z - input.z_offset_mm);
        return Geometry::rad2deg(std::acos(std::clamp(-sla::normals(p, mesh->aabb, width_mm)(0, 2), -1., 1.)));
    }
};

bool inside(const Sliced &s, const PlannedTip &tip, const BoundingBoxf3 &box)
{
    const Vec2d xy = s.at(tip);
    return xy.x() >= box.min.x() && xy.x() <= box.max.x() && xy.y() >= box.min.y() && xy.y() <= box.max.y() &&
           tip.site.print_z >= box.min.z() && tip.site.print_z <= box.max.z();
}

size_t count_in(const Sliced &s, const Plan &plan, const BoundingBoxf3 &box, std::set<TipNeed> needs = {})
{
    return size_t(std::count_if(plan.tips.begin(), plan.tips.end(), [&](const PlannedTip &tip) {
        return (needs.empty() || needs.count(tip.need) > 0) && inside(s, tip, box);
    }));
}

// The islands of `plan` whose birth point lies in `box`, in the fixture frame.
std::vector<const Island *> islands_in(const Sliced &s, const Plan &plan, const BoundingBoxf3 &box)
{
    std::vector<const Island *> found;
    for (const Island &island : plan.islands)
        if (const Vec3d p(island.birth.x() - s.shift.x(), island.birth.y() - s.shift.y(), island.birth.z()); box.contains(p))
            found.push_back(&island);
    return found;
}

// How far `axis` leans from straight down, in degrees.
double lean_deg(const Vec3f &axis) { return Geometry::rad2deg(std::acos(std::clamp(-double(axis.z()), -1., 1.))); }

} // namespace

TEST_CASE("The plan lattice samples each layer's area", "[ScaffoldPlan]")
{
    // A 6 x 6 mm column: the cells of one layer cover its 36 mm2 to half a cell's rounding along its 24 mm rim.
    Sliced s(box(0, 0, 0, 6, 6, 2));
    REQUIRE(s.input.material.size() > 5);
    const LayerGrid &g     = s.input.material[5];
    const size_t     cells = size_t(std::count_if(g.cells.begin(), g.cells.end(), [](uint16_t c) { return c != 0; }));
    CHECK_THAT(double(cells) * s.input.cell_mm * s.input.cell_mm, Catch::Matchers::WithinAbs(36., 24. * s.input.cell_mm / 2.));
}

TEST_CASE("A rod hanging under a slab takes one tip at its lowest point", "[ScaffoldPlan]")
{
    // A 4 x 4 x 10 column carries a 10 x 10 x 2 slab at z 8..10, and a 1 x 1 mm rod hangs from the slab at x 7..8,
    // y 4.5..5.5 down to z 3: the rod starts in mid-air and stands 5 mm before it meets the slab.
    Sliced s(merged({ box(0, 0, 0, 4, 4, 10), box(0, 0, 8, 10, 10, 2), box(7, 4.5, 3, 1, 1, 5) }));
    const BoundingBoxf3 rod(Vec3d(6.8, 4.3, 0.), Vec3d(8.2, 5.7, 7.9));
    const Plan          plan = s.plan();
    REQUIRE(count_in(s, plan, rod) == 1);
    const PlannedTip &tip = *std::find_if(plan.tips.begin(), plan.tips.end(), [&](const PlannedTip &t) { return inside(s, t, rod); });
    CHECK(tip.need == TipNeed::Birth);
    CHECK_THAT(tip.site.print_z, Catch::Matchers::WithinAbs(3., 0.1 + 1e-6));
}

TEST_CASE("A ledge takes a tip only when it hangs two support lines past the step", "[ScaffoldPlan]")
{
    // A 6 x 6 x 10 column standing on the bed carries a 1 mm ledge off its +x face at z 3 and a 1.5 mm ledge off its -x
    // face at z 6, each 4 mm wide. At 0.1 mm layers and the 21 degree threshold the layer below carries 0.26 mm of each,
    // so the first hangs 0.74 mm past that and the second 1.24 mm, either side of the reach, two lines or 0.84 mm.
    Sliced s(merged({ box(0, 0, 0, 6, 6, 10), box(6, 1, 3, 1., 4, 0.5), box(-1.5, 1, 6, 1.5, 4, 0.5) }));
    const Plan plan = s.plan();
    CHECK(count_in(s, plan, BoundingBoxf3(Vec3d(6., 0.9, 2.9), Vec3d(7.1, 5.1, 3.6))) == 0);
    CHECK(count_in(s, plan, BoundingBoxf3(Vec3d(-1.6, 0.9, 5.9), Vec3d(0., 5.1, 6.6)), { TipNeed::Underside }) > 0);
    CHECK(plan.underside_unmet_mm2 == 0.);
}

TEST_CASE("A flap smaller than a head's disc prints as it hangs", "[ScaffoldPlan]")
{
    // Off a 6 x 6 x 10 column standing on the bed, a fin 0.3 mm wide at z 3 and a ledge 1.2 mm wide at z 6 both stick
    // 1.6 mm out, past the reach at their ends. What the fin hangs past the 0.26 mm step, about 0.40 mm2, is less than
    // the small head's disc, pi w^2 or 0.55 mm2, and the scar a head would leave; the ledge hangs about 1.61 mm2.
    Sliced s(merged({ box(0, 0, 0, 6, 6, 10), box(6, 2.85, 3, 1.6, 0.3, 0.5), box(-1.6, 2.4, 6, 1.6, 1.2, 0.5) }));
    const Plan plan = s.plan();
    CHECK(count_in(s, plan, BoundingBoxf3(Vec3d(6., 2.6, 2.9), Vec3d(7.7, 3.4, 3.6)), { TipNeed::Underside }) == 0);
    CHECK(count_in(s, plan, BoundingBoxf3(Vec3d(-1.7, 2.2, 5.9), Vec3d(0., 3.8, 6.6)), { TipNeed::Underside }) == 1);
}

namespace {
// A 3 mm wide ledge sticking 1.6 mm out of the +x face of a 6 x 6 x 10 column standing on the bed, at x 6..7.6,
// y 1.5..4.5, z 5..5.5.
TriangleMesh cantilever() { return merged({ box(0, 0, 0, 6, 6, 10), box(6, 1.5, 5, 1.6, 3, 0.5) }); }
const BoundingBoxf3 cantilever_ledge(Vec3d(6., 1.4, 4.9), Vec3d(7.7, 4.6, 5.6));
} // namespace

TEST_CASE("A cantilever takes its head at its far edge", "[ScaffoldPlan]")
{
    // Nothing reaches the ledge's far edge from the other side, so no line bridges to it: its heads stand along its rim,
    // within two lines, the rim's width, plus a cell of its far edge or a side.
    Sliced     s(cantilever());
    const Plan plan = s.plan();
    size_t     heads = 0;
    for (const PlannedTip &tip : plan.tips)
        if (tip.need == TipNeed::Underside && inside(s, tip, cantilever_ledge)) {
            ++ heads;
            const Vec2d  xy   = s.at(tip);
            const double edge = std::min({ 7.6 - xy.x(), xy.y() - 1.5, 4.5 - xy.y() });
            INFO("tip at (" << xy.x() << ", " << xy.y() << "), " << edge << " mm from the far or side edge");
            CHECK(edge <= 2. * width_mm + 0.5 * width_mm);
        }
    CHECK(heads > 0);
}

TEST_CASE("A shallow ramp between two walls bridges between rim heads", "[ScaffoldPlan]")
{
    // Two 1 x 8 x 10 walls standing on the bed 4 mm apart carry a 1 mm thick floor between them whose underside rises
    // 10 degrees along y, from z 3. Each layer its edge advances 0.57 mm, 0.31 mm past the step, and the middle of that
    // edge hangs 2 mm from the walls, far past the reach, while the walls hold its ends: a line along the edge bridges
    // the middle. With `max_bridge_length` 0 nothing bridges, and the middle takes heads.
    const double       rise = 8. * std::tan(Geometry::deg2rad(10.));
    std::vector<Vec3d> ramp;
    for (double x : { 0.9, 5.1 })
        for (const Vec3d &p : { Vec3d(x, 0., 3.), Vec3d(x, 8., 3. + rise), Vec3d(x, 0., 4.), Vec3d(x, 8., 4. + rise) })
            ramp.push_back(p);
    Sliced              s(merged({ box(0, 0, 0, 1, 8, 10), box(5, 0, 0, 1, 8, 10), hull(ramp) }));
    const BoundingBoxf3 middle(Vec3d(2., -0.1, 2.9), Vec3d(4., 8.1, 5.5));
    const size_t        bridged = count_in(s, s.plan(), middle, { TipNeed::Underside });
    s.input.bridge_mm           = 0.;
    const size_t        held    = count_in(s, s.plan(), middle, { TipNeed::Underside });
    INFO("underside tips in the middle 2 mm: " << bridged << " with the bridge hold, " << held << " without");
    CHECK(bridged == 0);
    CHECK(held > 0);
}

TEST_CASE("A flat face born whole takes a ring on its rim", "[ScaffoldPlan]")
{
    // A disc 9 mm across floats at z 1..2.5 under a column 3 mm across up to z 4, beside a post that roots the object.
    // The disc starts whole in mid-air: its birth tip stands at its centre and the rest of its underside hangs past
    // the reach from it. Orca bridges a bottom along one direction it picks without the heads, so no line counts as
    // bridged by the direction it happens to take: the rim, which the perimeters follow, takes a ring of heads, and
    // the inside bridges to that ring, or to the rim, in every direction within `max_bridge_length`. With that
    // length 0 the inside takes heads.
    TriangleMesh disc = make_cylinder(4.5, 1.5), column = make_cylinder(1.5, 1.6);
    disc.translate(0.f, 0.f, 1.f);
    column.translate(0.f, 0.f, 2.4f);
    Sliced     s(merged({ disc, column, box(-9, -1, 0, 2, 2, 1) }));
    const auto from_rim = [&](const Plan &plan) {
        std::vector<double> depths;
        for (const PlannedTip &tip : plan.tips)
            if (tip.need == TipNeed::Underside)
                depths.push_back(4.5 - s.at(tip).norm());
        return depths;
    };
    const std::vector<double> ring = from_rim(s.plan());
    s.input.bridge_mm              = 0.;
    const std::vector<double> held = from_rim(s.plan());
    INFO(ring.size() << " underside tips, deepest " << (ring.empty() ? 0. : *std::max_element(ring.begin(), ring.end())) << " mm in; "
                     << held.size() << " without the bridge hold, deepest "
                     << (held.empty() ? 0. : *std::max_element(held.begin(), held.end())) << " mm in");
    REQUIRE_FALSE(ring.empty());
    for (const double depth : ring)
        CHECK(depth <= 2. * width_mm + 0.5 * width_mm);
    CHECK(std::any_of(held.begin(), held.end(), [](double depth) { return depth > 2. * width_mm + 0.5 * width_mm; }));
}

TEST_CASE("A face whose rim no head can hold takes heads inside instead of bridging to the rim", "[ScaffoldPlan]")
{
    // A disc 9 mm across floats at z 3..5 over a frame standing on the bed up to z 2, 9 x 9 outside with a 3 x 3
    // opening. A head's neck, 1.42 mm deep, meets the frame everywhere but over the opening, and a head covers at most
    // 1.52 mm, so no head reaches the disc's wall zone, within two lines of its edge: the zone hangs and its contour
    // sags with it. Every chord of the disc is shorter than `max_bridge_length`, yet no line of the inside ends on that
    // contour: the inside takes heads over the opening, and the zone counts as unmet.
    TriangleMesh disc = make_cylinder(4.5, 2);
    disc.translate(0.f, 0.f, 3.f);
    Sliced     s(merged({ disc, box(-4.5, -4.5, 0, 9, 3, 2), box(-4.5, 1.5, 0, 9, 3, 2), box(-4.5, -1.5, 0, 3, 3, 2),
                          box(1.5, -1.5, 0, 3, 3, 2) }));
    const Plan plan = s.plan();
    const auto heads_in = [&](double half) {
        return count_in(s, plan, BoundingBoxf3(Vec3d(-half, -half, 2.9), Vec3d(half, half, 3.1)), { TipNeed::Underside });
    };
    const size_t inside = heads_in(1.5), all = heads_in(4.6);
    INFO(inside << " underside heads over the opening of " << all << "; unmet " << plan.underside_unmet_mm2 << " mm2");
    CHECK(inside > 0);
    CHECK(inside == all);
    CHECK(plan.underside_unmet_mm2 > 0.);
}

TEST_CASE("A rim head beside a wall stays off the wall", "[ScaffoldPlan]")
{
    // A ledge 0.6 mm wide sticks 1.8 mm out of the +x face of a 6 x 6 x 10 column standing on the bed at z 5, and its
    // end face leans back over it at 45 degrees, so the ledge's far corners are acute. It is too narrow for a head's
    // disc to stand wholly inside it, so its heads go on its rim, where the builder aims a head along the faces'
    // normal averaged within one head radius: at the far corners that normal reads the end face and points 77
    // degrees from down. A rim head goes only where that normal stands within 60 degrees of down.
    std::vector<Vec3d> ledge;
    for (double y : { 2.7, 3.3 })
        for (const Vec3d &p : { Vec3d(5.9, y, 5.), Vec3d(7.8, y, 5.), Vec3d(5.9, y, 5.6), Vec3d(7.2, y, 5.6) })
            ledge.push_back(p);
    Sliced     s(merged({ box(0, 0, 0, 6, 6, 10), hull(ledge) }));
    const Plan plan  = s.plan();
    size_t     heads = 0;
    for (const PlannedTip &tip : plan.tips)
        if (tip.need == TipNeed::Underside && inside(s, tip, BoundingBoxf3(Vec3d(6., 2.5, 4.9), Vec3d(8., 3.5, 5.7)))) {
            ++ heads;
            const Vec2d xy = s.at(tip);
            INFO("tip at (" << xy.x() << ", " << xy.y() << ", " << tip.site.print_z << ")");
            CHECK(s.from_down_deg(tip) <= 60.);
        }
    CHECK(heads > 0);
}

TEST_CASE("The head loop ends when a covered cell hangs across a gap", "[ScaffoldPlan]")
{
    // A U-shaped ledge sticks out of the +x face of a 6 x 6 x 10 column standing on the bed at z 5: its near arm leaves
    // the column at y 1.5..2.3 and runs 4 mm out, turns and runs back at y 2.8..3.6, ending 0.4 mm short of the column.
    // The far arm hangs from the turn, and a head at its free end covers near arm cells across the 0.5 mm gap while the
    // way to them through material runs round the turn. With `max_bridge_length` 0 no line holds those cells: they
    // hang across the gap instead of calling a head of their own, within one and a half reaches or under the cover of
    // the head the near arm takes at the turn, so the near arm takes that one head and nothing counts as unmet.
    Sliced s(merged({ box(0, 0, 0, 6, 6, 10), box(6, 1.5, 5, 4, 0.8, 0.5), box(9.2, 1.5, 5, 0.8, 2.1, 0.5),
                      box(6.4, 2.8, 5, 3.6, 0.8, 0.5) }));
    s.input.bridge_mm = 0.;
    const Plan   plan     = s.plan();
    const size_t near_arm = count_in(s, plan, BoundingBoxf3(Vec3d(6., 1.4, 4.9), Vec3d(10.1, 2.35, 5.6)), { TipNeed::Underside });
    const size_t far_arm  = count_in(s, plan, BoundingBoxf3(Vec3d(6., 2.75, 4.9), Vec3d(10.1, 3.7, 5.6)), { TipNeed::Underside });
    INFO("near arm " << near_arm << " heads, far arm " << far_arm << ", unmet " << plan.underside_unmet_mm2 << " mm2");
    CHECK(near_arm == 1);
    CHECK(far_arm > 0);
    CHECK_THAT(plan.underside_unmet_mm2, Catch::Matchers::WithinAbs(0., 1e-12));
}

TEST_CASE("A birth carrying its part takes the heavy disc where the section allows it", "[ScaffoldPlan]")
{
    // Rods hang 5 mm from a slab carried by a column standing on the bed, 2 x 2 mm and 0.3 x 0.3 mm. Each birth tip
    // carries its rod's 5 mm, past the 2 mm a small disc carries, but only the thick rod's section fuses the heavy
    // disc, four lines across, to twice the small disc's area: the thin rod keeps the small disc. Beside them a
    // 3 x 1 mm panel starts at z 3 and a 0.3 mm nub at z 3.3 joins it at z 4.5, and the panel stands on to the slab:
    // the panel's birth, the elder, carries 6 mm and takes the heavy disc, and the nub's ends at the join.
    Sliced s(merged({ box(0, 0, 0, 4, 4, 11), box(0, 0, 9, 16, 10, 2), box(6, 1, 4, 2, 2, 5), box(10, 1.85, 4, 0.3, 0.3, 5),
                      box(12, 6, 3, 3, 1, 6), box(13.35, 7.5, 3.3, 0.3, 0.3, 1.5), box(13.35, 6.5, 4.5, 0.3, 1.2, 0.4) }));
    const Plan plan = s.plan();
    const auto grade_in = [&](const BoundingBoxf3 &box) {
        std::vector<double> grades;
        for (const PlannedTip &tip : plan.tips)
            if (tip.need == TipNeed::Birth && inside(s, tip, box))
                grades.push_back(tip.site.grade_mm);
        return grades;
    };
    const std::vector<double> thick = grade_in(BoundingBoxf3(Vec3d(5.9, 0.9, 3.9), Vec3d(8.1, 3.1, 4.1))),
                              thin  = grade_in(BoundingBoxf3(Vec3d(9.8, 1.6, 3.9), Vec3d(10.5, 2.4, 4.1))),
                              panel = grade_in(BoundingBoxf3(Vec3d(11.9, 5.9, 2.9), Vec3d(15.1, 7.1, 3.1))),
                              nub   = grade_in(BoundingBoxf3(Vec3d(13.2, 7.3, 3.2), Vec3d(13.8, 7.9, 3.4)));
    REQUIRE(thick.size() == 1);
    REQUIRE(thin.size() == 1);
    REQUIRE(panel.size() == 1);
    REQUIRE(nub.size() == 1);
    CHECK_THAT(thick.front(), Catch::Matchers::WithinAbs(4. * width_mm, 1e-9));
    CHECK_THAT(thin.front(), Catch::Matchers::WithinAbs(2. * width_mm, 1e-9));
    CHECK_THAT(panel.front(), Catch::Matchers::WithinAbs(4. * width_mm, 1e-9));
    CHECK_THAT(nub.front(), Catch::Matchers::WithinAbs(2. * width_mm, 1e-9));
}

TEST_CASE("Without the bridge hold a floating plate takes tips within the reach of its whole underside", "[ScaffoldPlan]")
{
    // A 12 x 12 x 2 plate at z 5..7 beside a column that roots the object: the plate is one island, too tall for debris.
    // With `max_bridge_length` 0 no line bridges its underside, so the heads cover all of it.
    Sliced s(merged({ box(-6, 0, 0, 3, 3, 8), box(0, 0, 5, 12, 12, 2) }));
    s.input.bridge_mm = 0.;
    const BoundingBoxf3 plate(Vec3d(-0.1, -0.1, 4.9), Vec3d(12.1, 12.1, 5.1));
    const Plan          plan  = s.plan();
    const double        reach = 2. * width_mm;
    CHECK(count_in(s, plan, plate) > 0);
    // Every underside sample on a 0.5 mm grid lies within one and a half reaches of a tip's head, the most a sliver too
    // small for a head of its own may hang: measured from the rim of the head's disc, two support lines across, plus
    // the 0.26 mm the self-support step grants, plus half a cell's diagonal for where the sample falls.
    double worst = 0.;
    for (double x = 0.1; x < 12.; x += 0.5)
        for (double y = 0.1; y < 12.; y += 0.5) {
            double near = std::numeric_limits<double>::max();
            for (const PlannedTip &tip : plan.tips)
                if (tip.site.print_z < 5.1)
                    near = std::min(near, (s.at(tip) - Vec2d(x, y)).norm());
            worst = std::max(worst, near);
        }
    CHECK(worst <= 1.5 * reach + width_mm + 0.26 + 0.15);
    CHECK(plan.underside_unmet_mm2 < 0.01 * 144.);
}

TEST_CASE("A shallower underside takes more tips and one steeper than the threshold takes none", "[ScaffoldPlan]")
{
    // A 4 x 4 x 14 column standing on the bed grows a 6 mm flare off its +x face from z 4, its underside rising at
    // `deg` from horizontal. Only the flare's own underside takes tips; nothing starts in mid-air.
    const auto flare_tips = [](double deg) {
        const double rise = 6. * std::tan(Geometry::deg2rad(deg));
        std::vector<Vec3d> flare;
        for (double y : { 0., 4. })
            for (const Vec3d &p : { Vec3d(4., y, 4.), Vec3d(10., y, 4. + rise), Vec3d(4., y, 5. + rise), Vec3d(10., y, 5. + rise) })
                flare.push_back(p);
        Sliced s(merged({ box(0, 0, 0, 4, 4, 14), hull(flare) }));
        const Plan plan = s.plan();
        CHECK(plan.islands_unheld == 0);
        return count_in(s, plan, BoundingBoxf3(Vec3d(4., -0.1, 3.9), Vec3d(10.1, 4.1, 14.)), { TipNeed::Underside });
    };
    const size_t shallow = flare_tips(10.), middle = flare_tips(15.), steep = flare_tips(30.);
    INFO("10 deg " << shallow << ", 15 deg " << middle << ", 30 deg " << steep);
    CHECK(shallow > middle);
    CHECK(middle > 0);
    CHECK(steep == 0);
}

TEST_CASE("A flare's runs restart at a solid column however its column is held", "[ScaffoldPlan]")
{
    // The same 15 degree flare 5 mm up a 4 x 4 column, once on a column standing on the bed and once on a column
    // floating from z 2 beside a post that roots the object: the flare hangs off solid material either way.
    const auto flare_tips = [](double base) {
        const double       rise = 6. * std::tan(Geometry::deg2rad(15.));
        std::vector<Vec3d> flare;
        for (double y : { 0., 4. })
            for (const Vec3d &p : { Vec3d(4., y, base + 5.), Vec3d(10., y, base + 5. + rise), Vec3d(4., y, base + 6. + rise),
                                    Vec3d(10., y, base + 6. + rise) })
                flare.push_back(p);
        Sliced s(merged({ box(0, 0, base, 4, 4, 12), hull(flare), box(-8, 0, 0, 2, 2, 1) }));
        return count_in(s, s.plan(), BoundingBoxf3(Vec3d(4., -0.1, base + 4.9), Vec3d(10.1, 4.1, base + 12.)));
    };
    CHECK(flare_tips(0.) == flare_tips(2.));
}

namespace {
// A 1.5 mm thick blade whose lower edge rises at `edge_deg` from its point at z 2, steeper than the threshold, so it
// has no underside above its point: 16 / tan(edge_deg) across in x, 19.07 mm at 40 degrees, it stands 16 mm free
// before a block from 2 mm short of its whole-mm run, x 17..22 at 40 degrees, z 18..21, on a rooted column past the
// block takes it in. Its edge faces `edge_deg` from down, and where it meets an upright side at a square corner the
// builder's normal reads the two faces averaged: 57 degrees from down at 40, within the face cap, and 63 at 50, past it.
TriangleMesh standing_blade(double edge_deg = 40.)
{
    const double       run = 16. / std::tan(Geometry::deg2rad(edge_deg)), block = std::floor(run) - 2.;
    std::vector<Vec3d> blade;
    for (double y : { 0., 1.5 })
        for (const Vec3d &p : { Vec3d(0., y, 2.), Vec3d(run, y, 18.), Vec3d(0., y, 5.), Vec3d(run, y, 21.) })
            blade.push_back(p);
    return merged({ hull(blade), box(block, -1, 18, 5, 3.5, 3), box(block + 5., -1, 0, 3, 3.5, 21) });
}

// A 1 x 1 mm rod rising 45 degrees out of the top of a 6 x 6 x 8 block standing on the bed: the hull of the square at
// x 2.5..3.5, y 2.5..3.5, z 8..9 and the same square moved 10 mm along x and 10 mm up.
TriangleMesh leaning_rod()
{
    std::vector<Vec3d> rod;
    for (double along : { 0., 10. })
        for (double x : { 2.5, 3.5 })
            for (double y : { 2.5, 3.5 })
                for (double z : { 8., 9. })
                    rod.emplace_back(x + along, y, z + along);
    return merged({ box(0, 0, 0, 6, 6, 8), hull(rod) });
}

// A panel hanging from its lowest corner at (0, 0..1, 2) beside a 5 x 5 post that roots the object: its lower edge,
// the keel, rises 40 degrees to x 8, and the panel stands to z 12, 1 mm thick at the keel and drafted 3 degrees
// outward on both sides.
const double keel_deg = 40.;
TriangleMesh hanging_panel()
{
    const double       rise = std::tan(Geometry::deg2rad(keel_deg)), draft = std::tan(Geometry::deg2rad(3.));
    std::vector<Vec3d> panel;
    for (const Vec2d &xz : { Vec2d(0., 2.), Vec2d(8., 2. + 8. * rise), Vec2d(8., 12.), Vec2d(0.3, 12.) }) {
        const double out = draft * (xz.y() - 2.);
        panel.emplace_back(xz.x(), -out, xz.y());
        panel.emplace_back(xz.x(), 1. + out, xz.y());
    }
    return merged({ hull(panel), box(-9, 0, 0, 5, 5, 13) });
}

// The Stability tips of `plan` in the fixture frame, lowest first.
std::vector<Vec3d> stability_tips(const Sliced &s, const Plan &plan)
{
    std::vector<Vec3d> tips;
    for (const PlannedTip &tip : plan.tips)
        if (tip.need == TipNeed::Stability) {
            const Vec2d xy = s.at(tip);
            tips.emplace_back(xy.x(), xy.y(), tip.site.print_z);
        }
    std::sort(tips.begin(), tips.end(), [](const Vec3d &a, const Vec3d &b) { return a.z() < b.z(); });
    return tips;
}

// A stem's stability window: three times its thickness, and never under 3 mm.
double window_of(double thickness_mm) { return std::max(3., 3. * thickness_mm); }
} // namespace

TEST_CASE("A blade standing free takes tips up its height", "[ScaffoldPlan]")
{
    Sliced         s(standing_blade());
    const Plan     plan = s.plan();
    std::set<long> heights;
    double         low = 1e9, high = -1e9;
    bool           point = false;
    for (const PlannedTip &tip : plan.tips) {
        const Vec2d xy = s.at(tip);
        if (xy.x() > 16.9 || tip.site.print_z > 17.9)
            continue;
        heights.insert(std::lround(tip.site.print_z));
        low   = std::min(low, tip.site.print_z);
        high  = std::max(high, tip.site.print_z);
        point = point || (tip.need == TipNeed::Birth && tip.site.print_z < 2.2);
    }
    INFO("tips on the blade " << heights.size() << " heights from " << low << " to " << high << ", slender " << plan.islands_slender);
    CHECK(point);
    CHECK(heights.size() >= 3);
    CHECK(high - low >= 8.);
}

TEST_CASE("Stability and Underside tips take the small disc", "[ScaffoldPlan]")
{
    // A stability tip steadies a part that already stands on its birth, and an underside tip holds a line or two, so
    // each takes the small disc, two lines across: on the cantilever's ledge, up the leaning rod and on the panel's keel.
    for (const TriangleMesh &fixture : { cantilever(), leaning_rod(), hanging_panel() }) {
        Sliced     s(fixture);
        const Plan plan  = s.plan();
        size_t     small = 0;
        for (const PlannedTip &tip : plan.tips)
            if (tip.need == TipNeed::Stability || tip.need == TipNeed::Underside) {
                ++ small;
                CHECK_THAT(tip.site.grade_mm, Catch::Matchers::WithinAbs(2. * width_mm, 1e-9));
            }
        CHECK(small > 0);
    }
}

TEST_CASE("A squat floating block takes no stability tip", "[ScaffoldPlan]")
{
    // The post that roots the object stands 8 mm, within its window of three times its 3 mm: a thinner post would count
    // as slender itself.
    Sliced s(merged({ box(0, 0, 3, 8, 8, 4), box(-5, 0, 0, 3, 3, 8) }));
    const Plan plan = s.plan();
    CHECK(std::none_of(plan.tips.begin(), plan.tips.end(), [](const PlannedTip &t) { return t.need == TipNeed::Stability; }));
    CHECK(plan.islands_slender == 0);
}

TEST_CASE("A blade widening from its point takes stability tips along its spine", "[ScaffoldPlan]")
{
    // A 1.5 mm thick blade hangs from its point at z 1: its front edge stands at x 0 and its spine runs out to x -10 at
    // z 8, rising 35 degrees, steeper than the threshold, so the spine has no underside. It stands 10 mm free before a
    // crossbar at z 11..13 on a rooted column takes it in. The blade reaches out from its point faster than it climbs,
    // so its tips follow the spine, each within the stability window of the last: 4.5 mm, three blade thicknesses.
    std::vector<Vec3d> blade;
    for (double y : { 0., 1.5 })
        for (const Vec3d &p : { Vec3d(0., y, 1.), Vec3d(-10., y, 8.), Vec3d(-10., y, 11.), Vec3d(0., y, 11.) })
            blade.push_back(p);
    Sliced s(merged({ hull(blade), box(-11, -1, 11, 13, 3.5, 2), box(2, -1, 0, 3, 3.5, 13) }));
    const double       window = window_of(1.5);
    const Plan         plan   = s.plan();
    std::vector<Vec3d> tips;
    for (const PlannedTip &tip : plan.tips)
        if (const Vec2d xy = s.at(tip); xy.x() < 0.5 && tip.site.print_z < 8.5) {
            tips.emplace_back(xy.x(), xy.y(), tip.site.print_z);
            if (tip.need == TipNeed::Stability)
                CHECK_THAT(tip.site.grade_mm, Catch::Matchers::WithinAbs(2. * width_mm, 1e-9));
        }
    std::sort(tips.begin(), tips.end(), [](const Vec3d &a, const Vec3d &b) { return a.z() < b.z(); });
    double apart = 0., end = std::numeric_limits<double>::max();
    for (size_t i = 0; i < tips.size(); ++ i) {
        if (i > 0)
            apart = std::max(apart, (tips[i] - tips[i - 1]).norm());
        end = std::min(end, (tips[i] - Vec3d(-10., 0.75, 8.)).norm());
    }
    INFO(tips.size() << " tips on the blade, at most " << apart << " mm apart, the spine's end " << end << " mm from one, window "
                     << window);
    REQUIRE(tips.size() >= 2);
    CHECK(tips.front().z() < 1.2);
    // A layer and a corner's offset along the face over the window.
    CHECK(apart <= window + 0.5);
    CHECK(end <= window + 0.5);
}

TEST_CASE("A rod leaning out of a block on the bed takes stability tips along its underside", "[ScaffoldPlan]")
{
    // The rod grows out of a block the bed roots, and its underside faces 45 degrees from down, steeper than the
    // threshold, so no underside tip holds it. It is 1 mm thick and may reach 3 mm from what holds it: the block it
    // grows out of, six times as thick, and its own tips. So it takes stability tips up its underside, each at least
    // half that window from the block and within a window of the next, the last within a window of its end.
    Sliced                   s(leaning_rod());
    const Plan               plan = s.plan();
    const std::vector<Vec3d> tips = stability_tips(s, plan);
    const double             window = window_of(1.);
    const Vec3d              end(13., 3., 18.5);
    INFO(tips.size() << " stability tips, slender " << plan.islands_slender);
    REQUIRE(tips.size() >= 3);
    // Its keel corners read exactly the face cap, a 45 degree face averaged with an upright one, and count as within
    // it: the rod is held from its first window over the block on, and no stretch of it is left slender.
    CHECK(plan.islands_slender == 0);
    double nearest_end = std::numeric_limits<double>::max();
    for (size_t i = 0; i < tips.size(); ++ i) {
        const Vec3d  &tip        = tips[i];
        const double  from_block = (tip - tip.cwiseMax(Vec3d::Zero()).cwiseMin(Vec3d(6., 6., 8.))).norm();
        INFO("tip " << i << " at (" << tip.x() << ", " << tip.y() << ", " << tip.z() << "), " << from_block << " mm from the block");
        CHECK(from_block >= 0.5 * window);
        if (i == 0)
            CHECK(from_block <= window + 0.5);
        else
            CHECK((tip - tips[i - 1]).norm() <= window + 0.5);
        nearest_end = std::min(nearest_end, (tip - end).norm());
    }
    CHECK(nearest_end <= window + 0.5);
}

TEST_CASE("A panel hanging from its corner takes stability tips on its keel", "[ScaffoldPlan]")
{
    // The keel faces 40 degrees from down and the drafted sides 87: a head on a side would meet a wall, so the panel's
    // holds stand on its keel, within 0.2 mm in x and z of the line it rises along from its corner.
    Sliced                   s(hanging_panel());
    const std::vector<Vec3d> tips = stability_tips(s, s.plan());
    REQUIRE_FALSE(tips.empty());
    const double rise = std::tan(Geometry::deg2rad(keel_deg));
    for (const Vec3d &tip : tips) {
        const double off = std::abs(tip.z() - 2. - tip.x() * rise) * std::cos(Geometry::deg2rad(keel_deg));
        INFO("tip at (" << tip.x() << ", " << tip.y() << ", " << tip.z() << "), " << off << " mm off the keel");
        CHECK(off <= 0.2);
    }
}

TEST_CASE("A stability tip stands only where the builder aims its head within 60 degrees of down", "[ScaffoldPlan]")
{
    // The builder aims a head along the mesh normal it reads at the tip, the faces within a head's radius averaged. A
    // blade whose edge faces 50 degrees from down reads 63 degrees at its square corners, past the face cap, so it
    // takes stability tips only where its edge face stands alone within that radius. The leaning rod reads the cap
    // exactly at its corners and the panel about 55 degrees on its keel. The cap holds its own angle to rounding.
    const std::pair<const char *, TriangleMesh> fixtures[] = { { "blade at 50 degrees", standing_blade(50.) },
                                                               { "leaning rod", leaning_rod() },
                                                               { "hanging panel", hanging_panel() } };
    for (const auto &[name, fixture] : fixtures) {
        Sliced     s(fixture);
        const Plan plan = s.plan();
        size_t     held = 0;
        INFO(name);
        for (const PlannedTip &tip : plan.tips)
            if (tip.need == TipNeed::Stability) {
                ++ held;
                INFO("tip at z " << tip.site.print_z);
                CHECK(s.from_down_deg(tip) <= 60.01);
            }
        CHECK(held > 0);
    }
}

TEST_CASE("A figure standing on a base takes no stability tip", "[ScaffoldPlan]")
{
    // A 16 x 16 x 2 base carries two 2 x 2 legs to z 10.5, a 7 x 4 x 10 torso from z 10 and a 4 x 4 x 3 head on it.
    // Near-vertical legs, torso and head offer no face a head can hold within 60 degrees of down, so they stay bare.
    // Each leg reaches past its window, three times its 2 mm, over the base it grows out of about 6 mm up, and counts
    // as slender once; the torso and head above them carry the legs' count on.
    Sliced     s(merged({ box(-8, -8, 0, 16, 16, 2), box(-3, -1, 2, 2, 2, 8.5), box(1, -1, 2, 2, 2, 8.5), box(-3.5, -2, 10, 7, 4, 10),
                          box(-2, -2, 20, 4, 4, 3) }));
    const Plan plan = s.plan();
    const std::vector<Vec3d> tips = stability_tips(s, plan);
    INFO(tips.size() << " stability tips, the first at z " << (tips.empty() ? 0. : tips.front().z()));
    CHECK(tips.empty());
    CHECK(plan.islands_slender == 2);
}

TEST_CASE("A floating cone takes no stability tip at its apex", "[ScaffoldPlan]")
{
    // A cone of radius 4 stands 8 mm on its flat base at z 3 beside a 5 x 5 post that roots the object. Its tip
    // narrows to nothing, but it grows out of the cone's own wider sections below it, which hold it within its window.
    TriangleMesh cone = make_cone(4., 8.);
    cone.translate(0.f, 0.f, 3.f);
    Sliced     s(merged({ cone, box(-11, -2.5, 0, 5, 5, 12) }));
    const Plan plan = s.plan();
    CHECK(stability_tips(s, plan).empty());
    CHECK(plan.islands_slender == 0);
}

TEST_CASE("A level spear takes a tip at least every bridge length", "[ScaffoldPlan]")
{
    // A spear 1.1 mm across runs level 15 mm out of the +x face of a 3 x 3 x 10 column standing on the bed, its axis at
    // z 8. Held level, it shares each slab piece with the column, so it reads the column's thickness; what holds it is
    // the underside rule's bridge: a head at least every `max_bridge_length`, 10 mm, from the column out, and one whose
    // cover, the reach plus the 0.26 mm step plus a toolpath width, and half a cell for where the lattice samples it,
    // takes in its end.
    TriangleMesh spear = make_cylinder(0.55, 15.);
    spear.rotate_y(float(M_PI / 2.));
    spear.translate(3.f, 1.5f, 8.f);
    Sliced     s(merged({ box(0, 0, 0, 3, 3, 10), spear }));
    const Plan plan = s.plan();
    std::vector<double> held { 3. };
    for (const PlannedTip &tip : plan.tips)
        if (const Vec2d xy = s.at(tip); xy.x() > 3. && std::abs(xy.y() - 1.5) < 0.6 && tip.site.print_z > 7.3 && tip.site.print_z < 8.1)
            held.push_back(xy.x());
    std::sort(held.begin(), held.end());
    INFO(held.size() - 1 << " tips under the spear, the last at x " << held.back());
    REQUIRE(held.size() > 1);
    for (size_t i = 1; i < held.size(); ++ i)
        CHECK(held[i] - held[i - 1] <= s.input.bridge_mm);
    CHECK(held.back() >= 18. - (3. * width_mm + 0.26 + 0.5 * s.input.cell_mm));
}

namespace {
// A 4 x 4 x 10 column standing on the bed carries a 12 x 10 x 2 slab at z 8..10. Three 0.6 x 0.6 mm nubs hang under the
// slab at y 1.7..2.3: A at x 4.3..4.9 and B at x 7..7.6 from z 7.93, C at x 9..9.6 from z 7.83. At 0.05 mm layers A and
// B print one layer, 0.05 mm free, before the slab takes them in, and C three, 0.15 mm free; at 0.1 mm layers A and B
// print one layer 0.1 mm free. A's far edge stands 0.9 mm off the column and B's 3.6 mm.
TriangleMesh nubs_under_slab()
{
    return merged({ box(0, 0, 0, 4, 4, 10), box(0, 0, 8, 12, 10, 2), box(4.3, 1.7, 7.93, 0.6, 0.6, 0.07), box(7, 1.7, 7.93, 0.6, 0.6, 0.07),
                    box(9, 1.7, 7.83, 0.6, 0.6, 0.17) });
}
const BoundingBoxf3 nub_a(Vec3d(4.1, 1.5, 7.8), Vec3d(5.1, 2.5, 7.99)), nub_b(Vec3d(6.8, 1.5, 7.8), Vec3d(7.8, 2.5, 7.99)),
                    nub_c(Vec3d(8.8, 1.5, 7.8), Vec3d(9.8, 2.5, 7.99));
} // namespace

TEST_CASE("A one-layer nub beside a held part hangs from it and one farther off or taller takes a tip", "[ScaffoldPlan]")
{
    // At 0.05 mm layers the merge layer carries a = 0.13 mm, and a nub hangs from the part it meets when all of it lies
    // within a + R = 0.97 mm of that part's material under the merge: A does, and hangs from the column the bed roots.
    // B stands farther off and takes its birth tip at the merge, and C stands free longer than the nub bound, 0.1 mm.
    Sliced     s(nubs_under_slab(), 0.05);
    const Plan plan = s.plan();
    CHECK(count_in(s, plan, nub_a) == 0);
    CHECK(count_in(s, plan, nub_b, { TipNeed::Birth }) == 1);
    CHECK(count_in(s, plan, nub_c, { TipNeed::Birth }) == 1);
    CHECK(plan.islands_unheld == 0);
    const std::vector<const Island *> a = islands_in(s, plan, nub_a);
    REQUIRE(a.size() == 1);
    CHECK(a.front()->reason == IslandReason::Hung);
    CHECK(a.front()->rooted);
}

TEST_CASE("The same nub takes a tip at 0.1 mm layers", "[ScaffoldPlan]")
{
    // At 0.1 mm layers nub A stands 0.1 mm free before the slab takes it in, a birth the reference tips: the hang holds
    // only a nub standing free less than 0.1 mm, whatever the layer height.
    Sliced     s(nubs_under_slab(), 0.1);
    const Plan plan = s.plan();
    CHECK(count_in(s, plan, nub_a, { TipNeed::Birth }) == 1);
    CHECK(plan.islands_unheld == 0);
}

TEST_CASE("Two nubs that meet only each other take one tip", "[ScaffoldPlan]")
{
    // Two 0.6 mm nubs 0.3 mm apart at z 5..5.07 meet each other one 0.05 mm layer up and grow on as a strand into the
    // slab of a column standing on the bed. Neither part is held where they meet, so the first takes its birth tip,
    // which holds its part, and the second, within a + R of it, hangs from it.
    Sliced s(merged({ box(0, 0, 0, 4, 4, 10), box(0, 0, 8, 12, 10, 2), box(7, 6, 5, 0.6, 0.6, 0.07), box(7.9, 6, 5, 0.6, 0.6, 0.07),
                      box(7, 6, 5.07, 1.5, 0.6, 2.93) }),
             0.05);
    const Plan          plan = s.plan();
    const BoundingBoxf3 pair(Vec3d(6.8, 5.8, 4.9), Vec3d(8.7, 6.8, 7.95));
    CHECK(count_in(s, plan, pair, { TipNeed::Birth }) == 1);
    CHECK(plan.islands_unheld == 0);
    const std::vector<const Island *> paired = islands_in(s, plan, BoundingBoxf3(Vec3d(6.8, 5.8, 4.9), Vec3d(8.7, 6.8, 5.1)));
    REQUIRE(paired.size() == 2);
    const auto tipped = std::find_if(paired.begin(), paired.end(), [](const Island *i) { return i->reason == IslandReason::Tip; });
    const auto hung   = std::find_if(paired.begin(), paired.end(), [](const Island *i) { return i->reason == IslandReason::Hung; });
    REQUIRE(tipped != paired.end());
    REQUIRE(hung != paired.end());
    REQUIRE((*tipped)->holders.size() == 1);
    CHECK(plan.tips[(*tipped)->holders.front()].need == TipNeed::Birth);
    CHECK(inside(s, plan.tips[(*tipped)->holders.front()], pair));
    CHECK((*hung)->holders == (*tipped)->holders);
    CHECK_FALSE((*hung)->rooted);
}

TEST_CASE("A birth sliver within the self-support step takes no tip and counts nothing", "[ScaffoldPlan]")
{
    // A fin 0.12 mm thick stands 0.12 mm off the +x face of a 4 x 4 x 10 column standing on the bed, from z 3 to 10. The
    // gap is wider than the slicer's closing radius shuts, so the fin starts as its own piece with nothing under it, but
    // its farthest point, 0.24 mm off the column, lies within the 0.26 mm the layer below carries at 0.1 mm layers: it
    // continues that layer as an overhang does and is no island.
    Sliced              s(merged({ box(0, 0, 0, 4, 4, 10), box(4.12, 1, 3, 0.12, 2, 7) }));
    const Plan          plan = s.plan();
    const BoundingBoxf3 fin(Vec3d(4., 0.8, 2.9), Vec3d(4.5, 3.2, 10.));
    CHECK(count_in(s, plan, fin) == 0);
    CHECK(islands_in(s, plan, fin).empty());
    CHECK(plan.islands_unheld == 0);
}

TEST_CASE("A far nub held by its own underside head gets no second tip", "[ScaffoldPlan]")
{
    // A nub 0.82 mm square and 0.07 mm tall hangs under the slab of the nub fixture's column at x 5.05..5.87, 1.05 to
    // 1.87 mm off the column: at 0.05 mm layers it prints one layer before the slab takes it in. Its cells hang from the
    // column, the material nearest them on their layer, past the reach plus half a cell, 0.95 mm, and its 16 lattice
    // cells cover more than a small head's disc, 13 cells, so its own layer takes an underside head. That head holds it
    // at the merge, where it would otherwise stand too far from the column to hang.
    Sliced s(merged({ box(0, 0, 0, 4, 4, 10), box(0, 0, 8, 12, 10, 2), box(5.05, 1.69, 7.93, 0.82, 0.82, 0.07) }), 0.05);
    const Plan          plan = s.plan();
    const BoundingBoxf3 nub(Vec3d(4.9, 1.5, 7.8), Vec3d(6.0, 2.7, 7.99));
    REQUIRE(count_in(s, plan, nub) == 1);
    const auto tip = std::find_if(plan.tips.begin(), plan.tips.end(), [&](const PlannedTip &t) { return inside(s, t, nub); });
    CHECK(tip->need == TipNeed::Underside);
    const std::vector<const Island *> found = islands_in(s, plan, nub);
    REQUIRE(found.size() == 1);
    CHECK(found.front()->holders == std::vector<size_t>{ size_t(tip - plan.tips.begin()) });
    CHECK(plan.islands_unheld == 0);
}

TEST_CASE("A birth over a shelf takes a tilted tip", "[ScaffoldPlan]")
{
    // A 0.8 mm rod hangs from the slab of a 4 x 4 column standing on the bed down to z 4, at x 7..7.8, over a shelf off
    // the column at z 2..3 that ends at x 7.5, under the rod. Every neck straight down from the rod ends within the xy
    // distance of the shelf, and one leaning away from it, past x 8, clears it: the birth tip takes the least lean that
    // clears, which no smaller lean at its spot does, and hands the builder that axis. With no lean allowed no tip
    // stands under the rod and the plan counts its island unheld.
    Sliced s(merged({ box(0, 0, 0, 4, 4, 10), box(0, 0, 8, 12, 10, 2), box(4, 3, 2, 3.5, 3, 1), box(7, 4.6, 4, 0.8, 0.8, 4) }));
    const BoundingBoxf3 rod(Vec3d(6.8, 4.4, 3.9), Vec3d(8., 5.6, 4.1));
    const Plan          plan = s.plan();
    REQUIRE(count_in(s, plan, rod, { TipNeed::Birth }) == 1);
    const PlannedTip &tip = *std::find_if(plan.tips.begin(), plan.tips.end(), [&](const PlannedTip &t) { return inside(s, t, rod); });
    const double      lean = lean_deg(tip.site.axis);
    INFO("birth tip at (" << s.at(tip).x() << ", " << s.at(tip).y() << ", " << tip.site.print_z << ") leaning " << lean
                          << " degrees along (" << tip.site.axis.x() << ", " << tip.site.axis.y() << ", " << tip.site.axis.z() << ")");
    CHECK_THAT(double(tip.site.axis.norm()), Catch::Matchers::WithinAbs(1., 1e-6));
    CHECK(lean > 0.);
    CHECK(lean <= 45. + 1e-6);
    // The planner hands the tip the axis the neck search reads at its spot, the one a baked list recomputes, and at one
    // tilt step less nothing clears there.
    const Vec3f again = neck_axis(s.input, tip.site);
    CHECK_THAT(double((again - tip.site.axis).norm()), Catch::Matchers::WithinAbs(0., 1e-6));
    const double step = std::asin(s.input.cell_mm / s.input.neck_depth_mm);
    s.input.max_tilt_rad = Geometry::deg2rad(lean) - step + 1e-9;
    CHECK(neck_axis(s.input, tip.site).isZero());

    s.input.max_tilt_rad = 0.;
    const Plan upright = s.plan();
    CHECK(count_in(s, upright, rod, { TipNeed::Birth }) == 0);
    CHECK(upright.islands_unheld == 1);
    const std::vector<const Island *> at_rod = islands_in(s, upright, rod);
    REQUIRE(at_rod.size() == 1);
    CHECK(at_rod.front()->reason == IslandReason::NoNeck);
}

TEST_CASE("A birth whose straight neck crosses a thin shelf leans past the shelf", "[ScaffoldPlan]")
{
    // The rod of the shelf case hangs to z 4 over a shelf 0.3 mm thick at z 3..3.3 off the column that runs under the
    // whole rod to x 8.2. A neck straight down from the rod ends at z 2.58 under the shelf, clear of the model, but its
    // shaft crosses the shelf: the birth tip leans away from the column past the shelf's edge, and with no lean allowed
    // no neck clears.
    Sliced s(merged({ box(0, 0, 0, 4, 4, 10), box(0, 0, 8, 12, 10, 2), box(4, 3, 3, 4.2, 4, 0.3), box(7, 4.6, 4, 0.8, 0.8, 4) }));
    const BoundingBoxf3 rod(Vec3d(6.8, 4.4, 3.9), Vec3d(8., 5.6, 4.1));
    const Plan          plan = s.plan();
    REQUIRE(count_in(s, plan, rod, { TipNeed::Birth }) == 1);
    const PlannedTip &tip = *std::find_if(plan.tips.begin(), plan.tips.end(), [&](const PlannedTip &t) { return inside(s, t, rod); });
    INFO("birth tip at (" << s.at(tip).x() << ", " << s.at(tip).y() << ", " << tip.site.print_z << ") leaning " << lean_deg(tip.site.axis)
                          << " degrees along (" << tip.site.axis.x() << ", " << tip.site.axis.y() << ", " << tip.site.axis.z() << ")");
    CHECK_FALSE(tip.site.axis.isZero());
    CHECK(tip.site.axis.x() > 0.f);

    s.input.max_tilt_rad = 0.;
    const Plan upright = s.plan();
    CHECK(count_in(s, upright, rod, { TipNeed::Birth }) == 0);
    CHECK(upright.islands_unheld == 1);
}

TEST_CASE("A leaning neck takes the azimuth whose end stands farthest from the model", "[ScaffoldPlan]")
{
    // A lattice built by hand, 0.21 mm cells and 0.1 mm slabs, so every cell a neck reads is known exactly. The tip
    // stands at the centre of cell (0, 0) on the bottom of slab 40, z 4, over a one-cell plate on slab 34, whose middle
    // lies 0.55 mm under it, and beside a wall three cells thick from cell x 6 on, 1.26 mm off, on every slab under the
    // tip. Straight down and at the first lean, 8.5 degrees, the shaft stays in the plate's cell; at 17 degrees it
    // leaves it at every azimuth, and every end stands clear of the wall. The end leaning toward the wall stands 0.85 mm
    // from it and one leaning away beyond the neck depth, so the lean points away from the wall.
    PlanInput in;
    in.cell           = scaled<coord_t>(0.21);
    in.cell_mm        = unscale<double>(in.cell);
    in.neck_depth_mm  = neck_mm;
    in.xy_distance_mm = xy_mm;
    const auto grid = [](int x0, int y0, int w, int h) {
        LayerGrid g;
        g.x0 = x0, g.y0 = y0, g.w = w, g.h = h;
        g.cells.assign(size_t(w) * size_t(h), 1);
        return g;
    };
    const LayerGrid wall = grid(6, -10, 3, 21);
    ExPolygon       wall_outline;
    wall_outline.contour = Polygon({ Point(6 * in.cell, -10 * in.cell), Point(9 * in.cell, -10 * in.cell), Point(9 * in.cell, 11 * in.cell),
                                     Point(6 * in.cell, 11 * in.cell) });
    for (int j = 0; j <= 40; ++ j) {
        in.slabs.push_back({ 0.1 * j, 0.1 * (j + 1), {} });
        in.blocked.emplace_back();
        in.material.push_back(j == 40 ? LayerGrid() : wall);
        in.wall_band.push_back(j == 40 ? ExPolygons() : offset_ex(wall_outline, scaled<float>(xy_mm)));
    }
    LayerGrid &plate = in.material[34];
    plate            = grid(0, -10, 9, 21);
    std::fill(plate.cells.begin(), plate.cells.end(), 0);
    for (int y = -10; y <= 10; ++ y)
        for (int x = 6; x <= 8; ++ x)
            plate.cells[size_t(y + 10) * size_t(plate.w) + size_t(x)] = 1;
    plate.cells[size_t(10) * size_t(plate.w)] = 1;

    const TipSite site { Point(in.cell / 2, in.cell / 2), 4., 39 };
    const Vec3f   axis = neck_axis(in, site);
    INFO("axis (" << axis.x() << ", " << axis.y() << ", " << axis.z() << ") leaning " << lean_deg(axis) << " degrees");
    CHECK_THAT(lean_deg(axis), Catch::Matchers::WithinAbs(Geometry::rad2deg(2. * std::asin(in.cell_mm / neck_mm)), 1e-3));
    // More than half the lean points away from the wall: an end leaning toward it, or across it, stands nearer.
    CHECK(axis.x() < -0.5f * std::sin(float(Geometry::deg2rad(lean_deg(axis)))));
}

namespace {
// The birth piece of `s`'s plan input whose outline holds `box`'s middle at its height, in the fixture frame.
size_t birth_piece(const Sliced &s, const BoundingBoxf3 &box)
{
    const Vec3d middle = box.center();
    const Point at     = Point::new_scale(middle.x() + s.shift.x(), middle.y() + s.shift.y());
    const auto &pieces = s.input.components.pieces;
    for (size_t p = 0; p < pieces.size(); ++ p)
        if (pieces[p].below.empty() && pieces[p].bottom_z >= box.min.z() && pieces[p].bottom_z <= box.max.z() && pieces[p].polygon.contains(at))
            return p;
    return size_t(-1);
}
} // namespace

TEST_CASE("A list's tipless nub hangs only where the planner would hang it", "[ScaffoldPlan]")
{
    // A baked list reads each tipless birth by the planner's rule, so a nub hangs only from a part the list's tips or
    // the bed hold within its hang. Under the nub fixture's slab at 0.05 mm layers A hangs from the column the bed roots,
    // while B stands too far off and C stands free too long: both need their tip.
    {
        Sliced       s(nubs_under_slab(), 0.05);
        const size_t a = birth_piece(s, nub_a), b = birth_piece(s, nub_b), c = birth_piece(s, nub_c);
        REQUIRE(a != size_t(-1));
        REQUIRE(b != size_t(-1));
        REQUIRE(c != size_t(-1));
        const std::vector<BirthRead> reads = read_births(s.input, { a, b, c }, {});
        CHECK(reads[0].hold == BirthHold::Nub);
        CHECK(reads[1].hold == BirthHold::Tip);
        CHECK(reads[2].hold == BirthHold::Tip);
    }
    // The far nub of the underside head case needs its tip until the list holds the head its own layer places.
    {
        Sliced s(merged({ box(0, 0, 0, 4, 4, 10), box(0, 0, 8, 12, 10, 2), box(5.05, 1.69, 7.93, 0.82, 0.82, 0.07) }), 0.05);
        const size_t nub = birth_piece(s, BoundingBoxf3(Vec3d(4.9, 1.5, 7.8), Vec3d(6.0, 2.7, 7.99)));
        REQUIRE(nub != size_t(-1));
        std::vector<TipSite> list;
        for (const PlannedTip &tip : s.plan().tips)
            list.push_back(tip.site);
        CHECK(read_births(s.input, { nub }, {}).front().hold == BirthHold::Tip);
        CHECK(read_births(s.input, { nub }, list).front().hold == BirthHold::Nub);
    }
    // Of two nubs meeting only each other the first needs its tip and the second hangs from it, as the plan places one.
    {
        Sliced s(merged({ box(0, 0, 0, 4, 4, 10), box(0, 0, 8, 12, 10, 2), box(7, 6, 5, 0.6, 0.6, 0.07), box(7.9, 6, 5, 0.6, 0.6, 0.07),
                          box(7, 6, 5.07, 1.5, 0.6, 2.93) }),
                 0.05);
        const size_t first = birth_piece(s, BoundingBoxf3(Vec3d(6.9, 5.9, 4.9), Vec3d(7.7, 6.7, 5.1)));
        const size_t second = birth_piece(s, BoundingBoxf3(Vec3d(7.8, 5.9, 4.9), Vec3d(8.6, 6.7, 5.1)));
        REQUIRE(first != size_t(-1));
        REQUIRE(second != size_t(-1));
        const std::vector<BirthRead> reads = read_births(s.input, { first, second }, {});
        const size_t tipped = size_t(std::count_if(reads.begin(), reads.end(), [](const BirthRead &r) { return r.hold == BirthHold::Tip; }));
        const size_t hung   = size_t(std::count_if(reads.begin(), reads.end(), [](const BirthRead &r) { return r.hold == BirthHold::Nub; }));
        CHECK(tipped == 1);
        CHECK(hung == 1);
    }
}
