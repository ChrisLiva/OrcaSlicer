#include <catch2/catch_all.hpp>
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Support/ScaffoldPlan.hpp"
#include <algorithm>
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

// Slices `mesh` at 0.1 mm with no support, so the plan reads plain object layers. A fixture coordinate maps into the
// sliced frame by the offset between the mesh's bounding box and the layers' extents; every fixture touches z 0.
struct Sliced
{
    Print     print;
    Model     model;
    PlanInput input;
    Vec2d     shift = Vec2d::Zero();   // sliced frame minus fixture frame, mm

    Sliced(const TriangleMesh &mesh)
    {
        TriangleMesh copy = mesh;
        init_print({ std::move(copy) }, print, model, fixture_config({ { "enable_support", "0" }, { "layer_height", "0.1" },
                                                                        { "initial_layer_print_height", "0.1" },
                                                                        { "layer_change_gcode", "G92 E0" } }));
        print.process();
        const PrintObject &object = *print.objects().front();
        BoundingBox        extents;
        // Above the first layer, which the elephant foot compensation may shrink.
        for (const Layer *layer : object.layers())
            if (layer->id() > 0 && ! layer->lslices.empty())
                extents.merge(get_extents(layer->lslices));
        const BoundingBoxf3 bb = mesh.bounding_box();
        shift = unscale(extents.min) - Vec2d(bb.min.x(), bb.min.y());
        input = prepare_plan(object, width_mm, xy_mm, neck_mm, Geometry::deg2rad(21.), {});
    }

    Plan plan() const { return plan_tips(input, {}); }
    // A tip's xy in the fixture frame.
    Vec2d at(const PlannedTip &tip) const { return unscale(tip.site.position) - shift; }
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
    // It carries 5 mm of rod: the heavy disc.
    CHECK_THAT(tip.site.grade_mm, Catch::Matchers::WithinAbs(4. * width_mm, 1e-9));
}

TEST_CASE("Ledges narrower than the reach take no tip", "[ScaffoldPlan]")
{
    // A 6 x 6 x 10 column standing on the bed with flat ledges sticking out of its faces 0.3, 0.5 and 0.8 mm at
    // z 3, 5 and 7. At 0.1 mm layers and the 21 degree threshold the layer below carries 0.26 mm of each, so the
    // widest hangs 0.54 mm past it, inside the 1 mm reach.
    Sliced s(merged({ box(0, 0, 0, 6, 6, 10), box(6, 1, 3, 0.3, 4, 0.5), box(1, 6, 5, 4, 0.5, 0.5), box(-0.8, 1, 7, 0.8, 4, 0.5) }));
    const Plan plan = s.plan();
    CHECK(plan.tips.empty());
    CHECK(plan.underside_unmet_mm2 == 0.);
}

TEST_CASE("A ledge hanging a head's worth past the reach takes tips", "[ScaffoldPlan]")
{
    // A 2 mm ledge hangs 1.74 mm past what the layer below carries. A head has to answer a disc of half the reach, and
    // the 4 mm strip past the 1 mm reach holds that much.
    Sliced s(merged({ box(0, 0, 0, 6, 6, 10), box(6, 1, 5, 2, 4, 0.5) }));
    const BoundingBoxf3 ledge(Vec3d(6., 0.9, 4.9), Vec3d(8.1, 5.1, 5.6));
    CHECK(count_in(s, s.plan(), ledge) > 0);
}

TEST_CASE("A floating plate takes tips within the reach of its whole underside", "[ScaffoldPlan]")
{
    // A 12 x 12 x 2 plate at z 5..7 beside a column that roots the object: the plate is one island, too tall for debris.
    Sliced s(merged({ box(-6, 0, 0, 3, 3, 8), box(0, 0, 5, 12, 12, 2) }));
    const BoundingBoxf3 plate(Vec3d(-0.1, -0.1, 4.9), Vec3d(12.1, 12.1, 5.1));
    const Plan          plan  = s.plan();
    const double        reach = NeedParams().reach_mm;
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

TEST_CASE("A blade standing free takes heavy tips up its height", "[ScaffoldPlan]")
{
    // A 1.5 mm thick blade whose lower edge rises at 60 degrees from its point at z 2, steeper than the threshold, so
    // it has no underside above its point: 9.24 mm across in x, it stands 16 mm free before a block at x 7..12,
    // z 18..21 on a rooted column at x 12..15 takes it in.
    const double       run = 16. / std::tan(Geometry::deg2rad(60.));
    std::vector<Vec3d> blade;
    for (double y : { 0., 1.5 })
        for (const Vec3d &p : { Vec3d(0., y, 2.), Vec3d(run, y, 18.), Vec3d(0., y, 5.), Vec3d(run, y, 21.) })
            blade.push_back(p);
    Sliced s(merged({ hull(blade), box(7, -1, 18, 5, 3.5, 3), box(12, -1, 0, 3, 3.5, 21) }));
    const Plan plan = s.plan();
    std::set<long> heights;
    double         low = 1e9, high = -1e9;
    bool           heavy_point = false;
    for (const PlannedTip &tip : plan.tips) {
        const Vec2d xy = s.at(tip);
        if (xy.x() > 10.5 || tip.site.print_z > 18.)
            continue;
        heights.insert(std::lround(tip.site.print_z));
        low  = std::min(low, tip.site.print_z);
        high = std::max(high, tip.site.print_z);
        if (tip.need == TipNeed::Birth && tip.site.print_z < 2.2 && tip.site.grade_mm >= 4. * width_mm - 1e-9)
            heavy_point = true;
        if (tip.need == TipNeed::Stability)
            CHECK(tip.site.grade_mm >= 4. * width_mm - 1e-9);
    }
    INFO("tips on the blade " << heights.size() << " heights from " << low << " to " << high << ", slender " << plan.islands_slender);
    CHECK(heavy_point);
    CHECK(heights.size() >= 3);
    CHECK(high - low >= 8.);
}

TEST_CASE("A squat floating block takes no stability tip", "[ScaffoldPlan]")
{
    Sliced s(merged({ box(0, 0, 3, 8, 8, 4), box(-5, 0, 0, 2, 2, 8) }));
    const Plan plan = s.plan();
    CHECK(std::none_of(plan.tips.begin(), plan.tips.end(), [](const PlannedTip &t) { return t.need == TipNeed::Stability; }));
    CHECK(plan.islands_slender == 0);
}

TEST_CASE("A blade widening from its point takes stability tips along its spine", "[ScaffoldPlan]")
{
    // A 1.5 mm thick blade hangs from its point at z 1: its front edge stands at x 0 and its spine runs out to x -10 at
    // z 8, rising 35 degrees, steeper than the threshold, so the spine has no underside. It stands 10 mm free before a
    // crossbar at z 11..13 on a rooted column takes it in. The blade reaches out from its point faster than it climbs,
    // so its tips follow the spine, each within the stability window of the last.
    std::vector<Vec3d> blade;
    for (double y : { 0., 1.5 })
        for (const Vec3d &p : { Vec3d(0., y, 1.), Vec3d(-10., y, 8.), Vec3d(-10., y, 11.), Vec3d(0., y, 11.) })
            blade.push_back(p);
    Sliced s(merged({ hull(blade), box(-11, -1, 11, 13, 3.5, 2), box(2, -1, 0, 3, 3.5, 13) }));
    const double       window = std::max(3., NeedParams().slender_ratio * 1.5);
    const Plan         plan   = s.plan();
    std::vector<Vec3d> tips;
    for (const PlannedTip &tip : plan.tips)
        if (const Vec2d xy = s.at(tip); xy.x() < 0.5 && tip.site.print_z < 8.5)
            tips.emplace_back(xy.x(), xy.y(), tip.site.print_z);
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

TEST_CASE("Small island starts go without a tip only when they meet a held part", "[ScaffoldPlan]")
{
    // A 12 x 10 slab at z 8..10 on a column standing on the bed. Under it hangs a 0.6 mm nub, z 7.9..8, that meets the
    // slab one layer up: it prints as it hangs. Beside it two 0.6 mm nubs at z 5..5.1 meet each other one layer up and
    // grow on as a strand into the slab: neither part is held where they meet, so one of them takes a tip.
    Sliced s(merged({ box(0, 0, 0, 4, 4, 10), box(0, 0, 8, 12, 10, 2), box(7, 2, 7.9, 0.6, 0.6, 0.1), box(7, 6, 5, 0.6, 0.6, 0.1),
                      box(7.9, 6, 5, 0.6, 0.6, 0.1), box(7, 6, 5.1, 1.5, 0.6, 2.9) }));
    const Plan plan = s.plan();
    const BoundingBoxf3 pair(Vec3d(6.8, 5.8, 4.9), Vec3d(8.7, 6.8, 7.95));
    CHECK(count_in(s, plan, BoundingBoxf3(Vec3d(6.8, 1.8, 7.7), Vec3d(7.8, 2.8, 7.95))) == 0);
    CHECK(count_in(s, plan, pair, { TipNeed::Birth }) == 1);
    CHECK(plan.islands_unheld == 0);

    // The plan lists every island with how it is held: the lone nub hangs from the slab the column roots, and of the
    // pair one takes the birth tip and the other hangs from it.
    const auto island_in = [&](double x0, double y0, double x1, double y1) {
        std::vector<const Island *> found;
        for (const Island &island : plan.islands)
            if (const Vec2d xy = Vec2d(island.birth.x(), island.birth.y()) - s.shift;
                xy.x() >= x0 && xy.x() <= x1 && xy.y() >= y0 && xy.y() <= y1)
                found.push_back(&island);
        return found;
    };
    const std::vector<const Island *> lone = island_in(6.8, 1.8, 7.8, 2.8), paired = island_in(6.8, 5.8, 8.7, 6.8);
    REQUIRE(lone.size() == 1);
    CHECK(lone.front()->reason == IslandReason::Hung);
    CHECK(lone.front()->rooted);
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
