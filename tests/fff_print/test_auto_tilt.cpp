#include <catch2/catch_all.hpp>

#include "libslic3r/AutoTilt.hpp"
#include "libslic3r/AutoTiltScorer.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Support/TreeSupport.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include <tbb/global_control.h>

#include <algorithm>
#include <functional>
#include <initializer_list>
#include <stdexcept>
#include <vector>

#include "test_helpers.hpp"

using namespace Slic3r;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

// The runner every test but the refusing one passes: the scorer's "main thread" is this thread.
AutoTilt::MainThreadRunner inline_runner()
{
    return [](const std::function<void()> &fn) { fn(); };
}

// Tree supports on, threshold 60 (so a face counts as an overhang past 29 deg from vertical),
// 0.2 mm layers. `line_width` is set because it defaults to an absolute 0, which zeroes
// `extrusion_width_scaled` in TreeSupport::detect_overhangs and kills sharp-tail propagation.
DynamicPrintConfig fixture_config(std::initializer_list<ConfigBase::SetDeserializeItem> extra = {})
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "enable_support",              "1" },
        { "support_type",                "tree(auto)" },
        { "support_threshold_angle",     "60" },
        { "nozzle_diameter",             "0.4" },
        { "line_width",                  "0.42" },
        { "layer_height",                "0.2" },
        { "initial_layer_print_height",  "0.2" },
        { "support_on_build_plate_only", "0" },
    });
    if (extra.size() > 0)
        config.set_deserialize_strict(extra);
    return config;
}

// An 8 mm square column carrying an 8 x 1.5 x 44 mm fin that leans 40 deg off vertical out of the
// column's y = 0 face near the bottom. At the root pose the fin's underside is 40 deg from vertical,
// past the detector's 29 deg crossover, so it is an overhang; the column's walls are vertical and are
// not, and its 8 x 8 base costs little once a tilt lifts it off the plate.
TriangleMesh fin_fixture()
{
    TriangleMesh base = make_cube(8, 8, 20);
    TriangleMesh fin  = make_cube(8, 1.5, 44);
    fin.rotate_x(float(Geometry::deg2rad(40.)));
    fin.translate(0.f, 0.f, 2.f);
    base.merge(fin);
    return base;
}

// The 9-pose grid the search behaviors use: three tilts by three leans, root included.
AutoTilt::Constants coarse_constants()
{
    AutoTilt::Constants k;
    k.tilts_deg = { 0, -20, -40 };
    k.leans_deg = { -15, 0, 15 };
    return k;
}

} // namespace

TEST_CASE("The auto-tilt scorer slices at the search layer height", "[AutoTilt]")
{
    const double              print_h = GENERATE(0.06, 0.2);
    const AutoTilt::Constants k;
    const double              expected_h = std::max(print_h, k.h_search_min_mm);

    Slic3r::Model            model  = Slic3r::Test::model("cube", Slic3r::Test::cube(20));
    const DynamicPrintConfig config = fixture_config({ { "layer_height", print_h } });

    AutoTilt::ContactScorer scorer(*model.objects.front(), config, k, inline_runner());
    REQUIRE_THAT(scorer.search_layer_height_mm(), WithinAbs(expected_h, 1e-12));

    scorer.score(AutoTilt::Pose{});
    REQUIRE_THAT(double(scorer.print().objects().front()->layer_count()), WithinAbs(20.0 / expected_h, 2.0));
}

TEST_CASE("The auto-tilt scorer overrides the object's own layer height and overhang filter", "[AutoTilt]")
{
    const AutoTilt::Constants k;

    Slic3r::Model model = Slic3r::Test::model("cube", Slic3r::Test::cube(20));
    ModelObject  *obj   = model.objects.front();

    // Two independent ways to override the layer height. update_layer_height_profile prefers the
    // profile over the ranges, so clearing only one of them would still slice at 0.1 mm.
    ModelConfig range_config;
    range_config.set("layer_height", 0.1);
    obj->layer_config_ranges[t_layer_height_range(0.0, 10.0)] = range_config;
    obj->layer_height_profile.set(std::vector<coordf_t>{ 0.0, 0.1, 20.0, 0.1 });

    const DynamicPrintConfig config = fixture_config({ { "support_remove_small_overhang", "1" } });

    AutoTilt::ContactScorer scorer(*obj, config, k, inline_runner());
    scorer.score(AutoTilt::Pose{});

    const PrintObject *po = scorer.print().objects().front();
    REQUIRE(po->config().support_remove_small_overhang.value == false);
    REQUIRE_THAT(double(po->layer_count()), WithinAbs(20.0 / 0.2, 2.0));
}

TEST_CASE("The auto-tilt scorer excludes the first layer's overhangs", "[AutoTilt]")
{
    const AutoTilt::Constants k;
    const DynamicPrintConfig  config = fixture_config();

    // A reference print of the same cube, sliced and detected directly: its layer 0 island is 4 mm
    // across, under the sharp-tail size threshold, so the detector does put a polygon there.
    Slic3r::Print ref_print;
    Slic3r::Model ref_model;
    Slic3r::Test::init_print({ Slic3r::Test::cube(4) }, ref_print, ref_model, config);
    PrintObject *ref_po = ref_print.objects_mutable().front();
    ref_po->slice();
    TreeSupport ts(*ref_po, ref_po->slicing_parameters());
    ts.m_scoring_mode = true;
    ts.detect_overhangs();
    REQUIRE_FALSE(ref_po->layers()[0]->loverhangs.empty());

    Slic3r::Model           model = Slic3r::Test::model("cube", Slic3r::Test::cube(4));
    AutoTilt::ContactScorer scorer(*model.objects.front(), config, k, inline_runner());
    const AutoTilt::Contact c = scorer.score(AutoTilt::Pose{});
    REQUIRE_THAT(c.volume_mm3, WithinAbs(0.0, 0.0));
    REQUIRE_THAT(c.score_mm3, WithinAbs(0.0, 0.0));
}

TEST_CASE("The auto-tilt scorer reports contact volume, weighted score and object volume", "[AutoTilt]")
{
    const AutoTilt::Constants k;

    Slic3r::Model           model = Slic3r::Test::model("fin", fin_fixture());
    AutoTilt::ContactScorer scorer(*model.objects.front(), fixture_config(), k, inline_runner());
    const AutoTilt::Contact c = scorer.score(AutoTilt::Pose{});

    REQUIRE(c.volume_mm3 > 0.0);
    REQUIRE(c.score_mm3 >= c.volume_mm3); // every fragility weight is at least 1
    // 1280 mm3 of column (8 x 8 x 20) plus 528 mm3 of fin (8 x 1.5 x 44), less the wedge of fin
    // that sits inside the column.
    REQUIRE_THAT(c.object_volume_mm3, WithinRel(1280.0 + 528.0, 0.05));
}

TEST_CASE("The auto-tilt search picks the pose that removes the most support contact", "[AutoTilt]")
{
    const AutoTilt::Constants k = coarse_constants();

    Slic3r::Model           model = Slic3r::Test::model("fin", fin_fixture());
    AutoTilt::ContactScorer scorer(*model.objects.front(), fixture_config(), k, inline_runner());

    const AutoTilt::SearchResult r =
        AutoTilt::search(AutoTilt::grid(k), scorer, k, []() { return false; }, [](size_t, size_t) {});

    REQUIRE(r.outcome == AutoTilt::SearchResult::Outcome::Improved);
    REQUIRE_THAT(r.best.tilt_deg, WithinAbs(-20.0, 1e-9));
    REQUIRE(r.best_contact.score_mm3 < r.root.score_mm3);
    REQUIRE(r.improvement >= r.required_improvement);
}

TEST_CASE("The auto-tilt search returns the same numbers whatever the worker count", "[AutoTilt]")
{
    const AutoTilt::Constants k = coarse_constants();

    auto run_search = [&k]() {
        Slic3r::Model           model = Slic3r::Test::model("fin", fin_fixture());
        AutoTilt::ContactScorer scorer(*model.objects.front(), fixture_config(), k, inline_runner());
        return AutoTilt::search(AutoTilt::grid(k), scorer, k, []() { return false; }, [](size_t, size_t) {});
    };

    AutoTilt::SearchResult single_threaded;
    {
        tbb::global_control gc(tbb::global_control::max_allowed_parallelism, 1);
        single_threaded = run_search();
    }
    const AutoTilt::SearchResult many_threaded = run_search();

    REQUIRE_THAT(many_threaded.root.score_mm3, WithinAbs(single_threaded.root.score_mm3, 0.0));
    REQUIRE_THAT(many_threaded.root.volume_mm3, WithinAbs(single_threaded.root.volume_mm3, 0.0));
    REQUIRE_THAT(many_threaded.best_contact.score_mm3, WithinAbs(single_threaded.best_contact.score_mm3, 0.0));
    REQUIRE(many_threaded.best == single_threaded.best);
}

TEST_CASE("Benchmark auto-tilt candidate scoring", "[AutoTilt][!benchmark]")
{
    const AutoTilt::Constants k;

    Slic3r::Model           model = Slic3r::Test::model("fin", fin_fixture());
    AutoTilt::ContactScorer scorer(*model.objects.front(), fixture_config(), k, inline_runner());

    BENCHMARK("score root pose") { return scorer.score(AutoTilt::Pose{}); };
}

TEST_CASE("The auto-tilt scorer applies its Print through the injected main-thread runner", "[AutoTilt]")
{
    const AutoTilt::Constants k;

    Slic3r::Model           model = Slic3r::Test::model("cube", Slic3r::Test::cube(20));
    AutoTilt::ContactScorer scorer(*model.objects.front(), fixture_config(), k,
                                   [](const std::function<void()> &) { throw std::runtime_error("runner refused"); });

    REQUIRE_THROWS_WITH(scorer.score(AutoTilt::Pose{}), "runner refused");
}
