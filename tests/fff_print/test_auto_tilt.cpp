#include <catch2/catch_all.hpp>

#include "libslic3r/AutoTilt.hpp"
#include "libslic3r/AutoTiltEvaluation.hpp"
#include "libslic3r/AutoTiltScorer.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Support/SupportAnalysis.hpp"
#include "libslic3r/Support/TreeSupport.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include <nlohmann/json.hpp>
#include <tbb/global_control.h>

#include <algorithm>
#include <cctype>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <initializer_list>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <string>
#include <utility>
#include <vector>

#include "support_validation.hpp"
#include "test_helpers.hpp"
#include "test_utils.hpp"

using namespace Slic3r;
using namespace Slic3r::Test;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

// The runner every test but the refusing one passes: the scorer's "main thread" is this thread.
AutoTilt::MainThreadRunner inline_runner()
{
    return [](const std::function<void()> &fn) { fn(); };
}

// Two 4 mm legs 16 mm apart under a 20 x 20 x 3 mm slab. The slab's underside spans the gap as a
// flat overhang that is neither a sharp tail nor a cantilever, so the legacy tree generator hands it
// interface roofs. The fin fixture cannot stand in here: its overhang is one sharp-tail cluster from
// end to end, and draw_circles() routes sharp-tail nodes to base_areas, never to the roof areas.
TriangleMesh table_fixture()
{
    TriangleMesh mesh = make_cube(4, 4, 20);
    TriangleMesh leg  = make_cube(4, 4, 20);
    leg.translate(16.f, 0.f, 0.f);
    TriangleMesh slab = make_cube(20, 20, 3);
    slab.translate(0.f, 0.f, 20.f);
    mesh.merge(leg);
    mesh.merge(slab);
    return mesh;
}

// The table fixture carrying a thin shelf low on its left leg: 6 x 4 x 0.8 mm reaching into the gap
// between the legs, its underside a flat overhang at z = 2 mm. The object is 23 mm tall, so that
// overhang sits at about a tenth of its height and the search's bottom exclusion plane at a fifth
// drops it from the estimate - while the print still has to lay support under it.
TriangleMesh low_shelf_fixture()
{
    TriangleMesh mesh  = table_fixture();
    TriangleMesh shelf = make_cube(6, 4, 0.8);
    shelf.translate(4.f, 0.f, 2.f);
    mesh.merge(shelf);
    return mesh;
}

// One captured plate's worth of evaluation input: the model as it stands, the config it would slice
// under, and the instances the pose is allowed to move. Copies the model, so the caller keeps its own
// pointers and the ids on both sides are the same ones.
AutoTilt::EvaluationInput plate_input(const Slic3r::Model &model, const DynamicPrintConfig &config,
                                      const std::vector<ObjectID> &affected)
{
    AutoTilt::PlateInput plate;
    plate.plate_index           = 0;
    plate.model                 = model;
    plate.full_config           = config;
    plate.affected_instance_ids = affected;

    AutoTilt::EvaluationInput input;
    input.object_id = model.objects.empty() ? ObjectID() : model.objects.front()->id();
    input.plates.push_back(std::move(plate));
    return input;
}

// The reason codes of an evaluation, joined, so a failing status names itself in the test output.
std::string reasons_of(const AutoTilt::PoseEvaluation &evaluation)
{
    std::string out;
    for (const std::string &code : evaluation.reason_codes)
        out += (out.empty() ? "" : ", ") + code;
    return out.empty() ? "none" : out;
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

TEST_CASE("The auto-tilt scorer ignores contact below the bottom exclusion plane", "[AutoTilt]")
{
    // The column HEAD sums with the exclusion off, one score per pose of the 9-pose grid.
    const std::vector<std::pair<AutoTilt::Pose, double>> expected_score = {
        { { 0., 0. }, 73.4894 },     { { 0., -15. }, 94.4571 },  { { 0., 15. }, 94.4571 },
        { { -20., -15. }, 44.0795 }, { { -20., 0. }, 37.6070 },  { { -20., 15. }, 44.0795 },
        { { -40., -15. }, 63.4913 }, { { -40., 0. }, 74.3701 },  { { -40., 15. }, 63.4913 },
    };

    AutoTilt::Constants k0       = coarse_constants();
    k0.bottom_exclusion_fraction = 0.;

    const std::vector<AutoTilt::Pose> g = AutoTilt::grid(k0);
    REQUIRE(g.size() == expected_score.size());

    Slic3r::Model           model0 = Slic3r::Test::model("fin", fin_fixture());
    AutoTilt::ContactScorer scorer0(*model0.objects.front(), fixture_config(), k0, inline_runner());

    double root0 = 0., minus20_0 = 0.;
    for (const AutoTilt::Pose &p : g) {
        const auto it = std::find_if(expected_score.begin(), expected_score.end(),
                                     [&p](const std::pair<AutoTilt::Pose, double> &e) { return e.first == p; });
        REQUIRE(it != expected_score.end());

        const AutoTilt::Contact c = scorer0.score(p);
        INFO("pose tilt " << p.tilt_deg << " lean " << p.lean_deg);
        CHECK_THAT(c.score_mm3, WithinRel(it->second, 1e-5));
        // object_volume_mm3 is the sliced sum of area(lslices)*height, an approximation that moves
        // with the layer discretisation as the object turns, so only the root pose pins it tightly.
        CHECK_THAT(c.object_volume_mm3, p.is_root() ? WithinRel(1797.32, 1e-5) : WithinRel(1797.32, 0.05));

        if (p.is_root())
            root0 = c.score_mm3;
        if (p == AutoTilt::Pose{ -20., 0. })
            minus20_0 = c.score_mm3;
    }
    REQUIRE(root0 > 0.);
    REQUIRE(minus20_0 > 0.);

    // The bottom fifth of the root pose's height excluded. The fin's overhang starts at z = 2 mm on a
    // fixture about 36.7 mm tall, so the plane at about 7.3 mm cuts contact off both poses, and it
    // cuts more off the root than off the tilt that already lifted the fin's foot clear.
    AutoTilt::Constants k20       = coarse_constants();
    k20.bottom_exclusion_fraction = 0.20;

    // A fresh scorer per step: score() re-appends sharp tails and cantilevers, so it is not
    // idempotent for a repeated identical pose.
    Slic3r::Model           model20 = Slic3r::Test::model("fin", fin_fixture());
    AutoTilt::ContactScorer scorer20(*model20.objects.front(), fixture_config(), k20, inline_runner());
    const AutoTilt::Contact root20  = scorer20.score(AutoTilt::Pose{});
    const AutoTilt::Contact minus20 = scorer20.score(AutoTilt::Pose{ -20., 0. });
    INFO("root " << root0 << " -> " << root20.score_mm3 << ", (-20, 0) " << minus20_0 << " -> " << minus20.score_mm3);
    CHECK(root20.score_mm3 < root0 * (1. - 1e-5));
    CHECK(minus20.score_mm3 < minus20_0 * (1. - 1e-5));

    Slic3r::Model           model20s = Slic3r::Test::model("fin", fin_fixture());
    AutoTilt::ContactScorer scorer20s(*model20s.objects.front(), fixture_config(), k20, inline_runner());
    const AutoTilt::SearchResult r20 =
        AutoTilt::search(AutoTilt::grid(k20), scorer20s, k20, []() { return false; }, [](size_t, size_t) {});
    CHECK(r20.outcome == AutoTilt::SearchResult::Outcome::Improved);
    CHECK_THAT(r20.best.tilt_deg, WithinAbs(-20.0, 1e-9));

    // The plane raised to the full height of the root pose: no contact survives it, so the search
    // sees a root under the negligible-volume floor and stops after that one pose.
    AutoTilt::Constants k1       = coarse_constants();
    k1.bottom_exclusion_fraction = 1.0;

    Slic3r::Model           model1 = Slic3r::Test::model("fin", fin_fixture());
    AutoTilt::ContactScorer scorer1(*model1.objects.front(), fixture_config(), k1, inline_runner());
    const AutoTilt::Contact root1 = scorer1.score(AutoTilt::Pose{});
    CHECK_THAT(root1.volume_mm3, WithinAbs(0.0, 0.0));
    CHECK_THAT(root1.score_mm3, WithinAbs(0.0, 0.0));
    CHECK_THAT(root1.object_volume_mm3, WithinRel(1797.32, 1e-5)); // the denominator stays whole

    Slic3r::Model           model1s = Slic3r::Test::model("fin", fin_fixture());
    AutoTilt::ContactScorer scorer1s(*model1s.objects.front(), fixture_config(), k1, inline_runner());
    const AutoTilt::SearchResult r1 =
        AutoTilt::search(AutoTilt::grid(k1), scorer1s, k1, []() { return false; }, [](size_t, size_t) {});
    CHECK(r1.outcome == AutoTilt::SearchResult::Outcome::BelowFloor);
    CHECK(r1.evaluated == 1);
}

TEST_CASE("The auto-tilt scorer records one row per contact polygon when asked", "[AutoTilt]")
{
    // The plane is switched on explicitly rather than left at the shipping default, so that this
    // fixture produces both excluded and charged rows and the sums below have something to prove.
    AutoTilt::Constants k;
    k.bottom_exclusion_fraction = 0.20;

    Slic3r::Model           model = Slic3r::Test::model("fin", fin_fixture());
    AutoTilt::ContactScorer scorer(*model.objects.front(), fixture_config(), k, inline_runner());

    std::vector<AutoTilt::ContactScorer::PolygonRecord> records;
    scorer.records = &records;
    const AutoTilt::Contact c = scorer.score(AutoTilt::Pose{});

    REQUIRE_FALSE(records.empty());

    // Summed in vector order, so these reproduce score()'s own float arithmetic term for term.
    double score_sum = 0., volume_sum = 0.;
    size_t excluded_rows = 0;
    for (const AutoTilt::ContactScorer::PolygonRecord &r : records) {
        CHECK(r.layer >= 1); // layer 0 sits on the plate and is never recorded
        CHECK(r.print_z_mm > 0.);
        CHECK(r.weight >= 1.);
        if (r.perimeter_mm > 0.)
            CHECK_THAT(r.t_mm, WithinRel(2. * r.area_mm2 / r.perimeter_mm, 1e-12));
        if (r.excluded) {
            ++excluded_rows;
            continue;
        }
        volume_sum += r.area_mm2 * k.h_ref_mm;
        score_sum += r.weight * r.area_mm2 * k.h_ref_mm;
    }
    INFO("records " << records.size() << ", excluded " << excluded_rows);
    REQUIRE(excluded_rows > 0);
    CHECK_THAT(score_sum, WithinRel(c.score_mm3, 1e-9));
    CHECK_THAT(volume_sum, WithinRel(c.volume_mm3, 1e-9));

    // A second pose appends to the same vector: score() adds rows and never clears them.
    const size_t n = records.size();
    scorer.score(AutoTilt::Pose{ -20., 0. });
    CHECK(records.size() > n);
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

TEST_CASE("A processed tree-support print measures its emitted contact through the support analysis", "[AutoTilt]")
{
    // The area the toolpaths of one print actually cover, at the width they were emitted with.
    const auto printed_mm2 = [](const PrintObject &po) {
        double covered = 0.;
        for (const SupportLayer *sl : po.support_layers())
            for (const ExPolygon &poly : union_ex(sl->support_fills.polygons_covered_by_width(0.f)))
                covered += poly.area() * SCALING_FACTOR * SCALING_FACTOR;
        return covered;
    };

    // Each nozzle with the support width it prints at. The measurement reads emitted extrusion
    // footprints rather than the areas the router drew, so it has to hold at every width; the three
    // legacy tree styles route those areas differently and all three end in printed material.
    struct Nozzle { const char *diameter; const char *support_width; };
    const Nozzle nozzles[3] = { { "0.25", "0.25" }, { "0.4", "0.42" }, { "0.6", "0.62" } };
    std::vector<double> widths;
    for (const Nozzle &nozzle : nozzles) {
        double style_width = 0.;
        for (const char *style : { "tree_slim", "tree_strong", "tree_hybrid" }) {
            INFO("nozzle " << nozzle.diameter << " support_line_width " << nozzle.support_width << " style " << style);
            // support_style, because fixture_config() leaves it unset and SupportParameters resolves
            // that to organic, which the analysis does not run under; support_top_z_distance at
            // PrintConfig.cpp's own default, so the gap the tips are planned against is stated here.
            Slic3r::Print print;
            Slic3r::Model model;
            Slic3r::Test::init_print({ table_fixture() }, print, model,
                fixture_config({ { "support_style", style }, { "support_top_z_distance", "0.2" },
                                 { "nozzle_diameter", nozzle.diameter }, { "support_line_width", nozzle.support_width } }));
            print.request_legacy_support_analysis();
            print.process();

            const PrintObject                                   &po     = *print.objects().front();
            const std::shared_ptr<const SupportAnalysis::Report> report = po.support_analysis();
            REQUIRE(report != nullptr);
            REQUIRE(report->coverage_available);
            REQUIRE(report->key.extrusion_width_mm > 0.);
            REQUIRE(report->measured_contact_mm2 > 0.);
            REQUIRE(report->provenance.traced_regions > 0);
            REQUIRE(report->support_volume_mm3 > 0.);

            // Whatever the style drew, what is counted is the material the toolpaths cover: the
            // measurement is bounded by it and never reaches the drawn areas above it.
            const double printed = printed_mm2(po);
            REQUIRE(printed > 0.);
            REQUIRE(report->measured_contact_mm2 < printed);

            // The three styles resolve one support extrusion width per nozzle between them.
            if (style_width == 0.)
                style_width = report->key.extrusion_width_mm;
            REQUIRE_THAT(report->key.extrusion_width_mm, WithinAbs(style_width, 1e-9));

            if (std::string(style) == "tree_hybrid") {
                // The slab's underside is a big flat overhang, so Hybrid routes it as a polygon
                // contact the generator pins. A pinned contact is one no thinning pass may drop, so
                // it is critical, and its material is counted the same way as any branch tip's:
                // through the extrusions the toolpaths laid, not through the polygon that was drawn.
                REQUIRE(! report->pinned_anchor_ids.empty());
                for (uint64_t id : report->pinned_anchor_ids)
                    REQUIRE(std::binary_search(report->critical_anchor_ids.begin(), report->critical_anchor_ids.end(), id));
                size_t pinned_regions_traced = 0;
                for (const SupportAnalysis::RegionCoverage &region : report->coverage)
                    for (uint64_t id : region.anchor_ids)
                        if (std::binary_search(report->pinned_anchor_ids.begin(), report->pinned_anchor_ids.end(), id) &&
                            region.anchored) {
                            ++ pinned_regions_traced;
                            break;
                        }
                REQUIRE(pinned_regions_traced > 0);
            }
        }
        widths.push_back(style_width);
    }
    // A wider nozzle prints a wider support line, and the key carries the width the problem resolved.
    REQUIRE(widths.size() == 3);
    REQUIRE(widths[0] < widths[1]);
    REQUIRE(widths[1] < widths[2]);

    // A region whose tip was drawn inside the planned gap and never printed measures no contact at
    // all and reports its contacts as missing: unknown geometry stays explicit and never reads as
    // zero material against a region that was in fact anchored.
    Slic3r::Print fin_print;
    Slic3r::Model fin_model;
    Slic3r::Test::init_print({ fin_fixture() }, fin_print, fin_model,
        fixture_config({ { "support_style", "tree_slim" }, { "support_top_z_distance", "0.2" } }));
    fin_print.request_legacy_support_analysis();
    fin_print.process();
    const std::shared_ptr<const SupportAnalysis::Report> fin_report = fin_print.objects().front()->support_analysis();
    REQUIRE(fin_report != nullptr);
    REQUIRE(fin_report->coverage_available);

    size_t unanchored = 0;
    for (const SupportAnalysis::RegionCoverage &region : fin_report->coverage) {
        if (region.anchored)
            continue;
        ++ unanchored;
        INFO("region " << region.region_id);
        REQUIRE_THAT(region.covered_mm2, WithinAbs(0., 1e-12));
        REQUIRE(region.covered_count() == 0);
        for (uint64_t id : region.anchor_ids)
            REQUIRE(std::binary_search(fin_report->missing_anchor_ids.begin(), fin_report->missing_anchor_ids.end(), id));
    }
    REQUIRE(unanchored > 0);
    REQUIRE(! fin_report->missing_anchor_ids.empty());
    REQUIRE(fin_report->has_reason(SupportAnalysis::Reason::MissingAnchor));
    // And the print as a whole still measured contact, so the missing region is a missing region and
    // not a print that emitted nothing.
    REQUIRE(fin_report->measured_contact_mm2 > 0.);
}

TEST_CASE("Generated evaluation poses every affected instance about its own root and pivot", "[AutoTilt]")
{
    // One object with two instances that share nothing but the object: the second stands 60 mm away,
    // half again as big, mirrored in X and lifted 3 mm off the plate. A second object beside them is
    // the neighbour nobody selected. A third instance sits on a second captured plate, so the pose
    // has to answer over the whole captured input and not over one plate's model.
    Slic3r::Model  model = Slic3r::Test::model("fin", fin_fixture());
    ModelObject   *obj   = model.objects.front();
    obj->add_instance(Vec3d(60., 0., 3.), Vec3d(1.5, 1.5, 1.5), Vec3d(0., 0., 0.), Vec3d(-1., 1., 1.));
    ModelObject *neighbour = model.add_object();
    neighbour->add_volume(Slic3r::Test::cube(10));
    neighbour->add_instance()->set_offset(Vec3d(-40., 0., 0.));

    Slic3r::Model second_plate = Slic3r::Test::model("fin", fin_fixture());
    ModelObject  *far_obj      = second_plate.objects.front();

    const std::vector<ObjectID> affected{ obj->instances[0]->id(), obj->instances[1]->id() };
    const AutoTilt::Pose        pose{ -12., 8. };

    AutoTilt::EvaluationInput input = plate_input(model, fixture_config(), affected);
    input.plates.push_back(AutoTilt::PlateInput{});
    input.plates.back().plate_index           = 1;
    input.plates.back().model                 = second_plate;
    input.plates.back().full_config           = fixture_config();
    input.plates.back().affected_instance_ids = { far_obj->instances.front()->id() };

    const std::vector<AutoTilt::InstanceSnapshot> posed = AutoTilt::posed_instances(input, pose);

    // One entry per affected instance, over every captured plate, in the order they were asked for,
    // and nothing for the instance nobody selected.
    REQUIRE(posed.size() == 3);
    REQUIRE(posed[0].id == affected[0]);
    REQUIRE(posed[1].id == affected[1]);
    REQUIRE(posed[2].id == far_obj->instances.front()->id());
    for (const AutoTilt::InstanceSnapshot &entry : posed)
        REQUIRE(entry.id != neighbour->instances.front()->id());

    // Each instance is dropped by its own transformed model-part hull minimum: both stand on the
    // plate, and because they are different sizes the two drops are different numbers, which is what
    // one whole-object ensure_on_bed() call could not have produced.
    std::vector<double> drop;
    for (size_t i = 0; i < 2; ++ i) {
        const BoundingBoxf3 box = AutoTilt::posed_hull_box(input.plates.front().model, posed[i]);
        REQUIRE(box.defined);
        REQUIRE_THAT(box.min.z(), WithinAbs(0., 1e-9));
        drop.push_back(posed[i].matrix.translation().z());
    }
    REQUIRE(std::abs(drop[0] - drop[1]) > 1e-6);

    // A snapshot is read against the model that carries its instance and against no other: the model
    // of the plate this instance does not sit on has nothing to say about it.
    REQUIRE_FALSE(AutoTilt::posed_hull_box(input.plates.back().model, posed[0]).defined);

    // The same requested tilt and lean relative to each instance's own root, with that instance's own
    // scale and mirror still in it: the linear part of posed * root^-1 is the requested rotation and
    // nothing else, whatever the root carried.
    const Transform3d rotation =
        Geometry::rotation_transform(Vec3d(Geometry::deg2rad(pose.tilt_deg), Geometry::deg2rad(pose.lean_deg), 0.));
    for (size_t i = 0; i < 2; ++ i) {
        INFO("instance " << i);
        const Transform3d root     = obj->instances[i]->get_transformation().get_matrix();
        const Matrix3d    relative = posed[i].matrix.linear() * root.linear().inverse();
        REQUIRE((relative - rotation.linear()).cwiseAbs().maxCoeff() < 1e-9);
        // The root's own scale and mirror survive: the determinant of the posed linear part is the
        // root's, sign and all, because a rotation contributes exactly 1.
        REQUIRE_THAT(posed[i].matrix.linear().determinant(), WithinRel(root.linear().determinant(), 1e-9));
    }
    REQUIRE(posed[1].matrix.linear().determinant() < 0.); // the mirrored instance is still mirrored

    // An instance that is not dropped keeps the height it was given: the Z correction is the
    // instance's own auto-drop, never a blanket flattening of the pose onto the plate.
    far_obj->instances.front()->auto_drop = false;
    far_obj->instances.front()->set_offset(Vec3d(0., 0., 5.));
    input.plates.back().model             = second_plate;
    const std::vector<AutoTilt::InstanceSnapshot> lifted = AutoTilt::posed_instances(input, pose);
    REQUIRE(lifted.size() == 3);
    REQUIRE(AutoTilt::posed_hull_box(input.plates.back().model, lifted[2]).min.z() > 0.);
}

TEST_CASE("Generated evaluation slices each pose in fresh Print state under the captured settings", "[AutoTilt]")
{
    // Two things the coarse contact scorer erases before it slices: the object's own layer height and
    // its own small-overhang filter. The generated evaluation is the final measurement, so it slices
    // under both.
    Slic3r::Model model = Slic3r::Test::model("fin", fin_fixture());
    ModelObject  *obj   = model.objects.front();
    obj->config.set("layer_height", 0.1);
    obj->config.set("support_remove_small_overhang", true);
    obj->ensure_on_bed();
    // The neighbour nobody selected, far enough away to collide with nothing.
    ModelObject *neighbour = model.add_object();
    neighbour->add_volume(Slic3r::Test::cube(10));
    neighbour->add_instance()->set_offset(Vec3d(-40., 0., 0.));
    neighbour->ensure_on_bed();

    const ObjectID    instance_id    = obj->instances.front()->id();
    const ObjectID    neighbour_id   = neighbour->instances.front()->id();
    const Transform3d neighbour_root = neighbour->instances.front()->get_transformation().get_matrix();

    const AutoTilt::EvaluationInput input =
        plate_input(model, fixture_config({ { "support_style", "tree_slim" }, { "layer_change_gcode", "G92 E0" } }), { instance_id });

    // `Print::apply` rewrites the model tree, so it goes through the injected runner and a runner
    // that refuses stops the evaluation where it stands.
    {
        AutoTilt::GeneratedEvaluator refusing(input, [](const std::function<void()> &) { throw std::runtime_error("runner refused"); });
        REQUIRE_THROWS_WITH(refusing.evaluate(AutoTilt::Pose{ -10., 0. }, {}), "runner refused");
    }

    AutoTilt::GeneratedEvaluator     evaluator(input, inline_runner());
    const AutoTilt::PoseEvaluation   tilted = evaluator.evaluate(AutoTilt::Pose{ -10., 0. }, {});
    INFO("reasons: " << reasons_of(tilted));
    REQUIRE(tilted.status != AutoTilt::PoseEvaluation::Status::Invalid);
    REQUIRE(tilted.status != AutoTilt::PoseEvaluation::Status::Canceled);
    REQUIRE(tilted.status != AutoTilt::PoseEvaluation::Status::Unknown);

    // What was actually sliced kept the object's own overrides: the profile the scorer flattens and
    // the filter it turns off are both still in force.
    const PrintObject *po = nullptr;
    for (const PrintObject *candidate : evaluator.print(0).objects())
        if (candidate->model_object()->id() == obj->id())
            po = candidate;
    REQUIRE(po != nullptr);
    REQUIRE(po->config().support_remove_small_overhang.value == true);
    REQUIRE_THAT(po->config().layer_height.value, WithinAbs(0.1, 1e-12));
    REQUIRE(po->layer_count() > 2);
    REQUIRE_THAT(po->layers().back()->height, WithinAbs(0.1, 1e-9));

    // The instance nobody selected sits exactly where it was captured.
    const auto instance_matrix = [&evaluator](ObjectID id) {
        for (const ModelObject *object : evaluator.model(0).objects)
            for (const ModelInstance *instance : object->instances)
                if (instance->id() == id)
                    return instance->get_transformation().get_matrix();
        return Transform3d(Transform3d::Identity());
    };
    REQUIRE(instance_matrix(neighbour_id).isApprox(neighbour_root, 1e-12));

    // Fresh state per pose: the next pose is taken from the captured root, never from the model the
    // pose before it left behind, so evaluating the root after a tilt lands back on the root.
    const AutoTilt::PoseEvaluation root = evaluator.evaluate(AutoTilt::Pose{}, {});
    REQUIRE(root.status != AutoTilt::PoseEvaluation::Status::Invalid);
    const std::vector<AutoTilt::InstanceSnapshot> expected = AutoTilt::posed_instances(input, AutoTilt::Pose{});
    REQUIRE(expected.size() == 1);
    REQUIRE(instance_matrix(instance_id).isApprox(expected.front().matrix, 1e-12));
}

TEST_CASE("Generated evaluation measures legacy support read-only and refuses Organic", "[AutoTilt]")
{
    Slic3r::Model model = Slic3r::Test::model("fin", fin_fixture());
    model.objects.front()->ensure_on_bed();
    const std::vector<ObjectID> affected{ model.objects.front()->instances.front()->id() };

    const auto config = [](std::initializer_list<ConfigBase::SetDeserializeItem> extra) {
        DynamicPrintConfig out = fixture_config({ { "support_top_z_distance", "0.2" }, { "layer_change_gcode", "G92 E0" } });
        out.set_deserialize_strict(extra);
        return out;
    };
    const auto evaluate = [&](const DynamicPrintConfig &cfg) {
        AutoTilt::GeneratedEvaluator evaluator(plate_input(model, cfg, affected), inline_runner());
        return evaluator.evaluate(AutoTilt::Pose{ -6., 0. }, {});
    };

    // Organic keeps its own search and its own estimate. The evaluator answers for the legacy tree
    // generator alone, so it says Unknown rather than handing back a legacy report with an Organic
    // label on it - and the unset style, which SupportParameters resolves to organic, is the same
    // answer for the same reason.
    for (const char *style : { "organic", "default" }) {
        INFO("support_style " << style);
        const AutoTilt::PoseEvaluation organic = evaluate(config({ { "support_style", style } }));
        REQUIRE(organic.status == AutoTilt::PoseEvaluation::Status::Unknown);
        REQUIRE(organic.instances.empty());
        REQUIRE(std::find(organic.reason_codes.begin(), organic.reason_codes.end(),
                          std::string("organic_or_non_tree_support")) != organic.reason_codes.end());
    }

    // Legacy with the miniature contact mode off: the one generated pass is measured where it
    // stands, nothing thinned it, and the required regions the support never reached are reported
    // rather than passed.
    const AutoTilt::PoseEvaluation plain = evaluate(config({ { "support_style", "tree_slim" }, { "support_miniature_contacts", "0" } }));
    INFO("reasons: " << reasons_of(plain));
    REQUIRE(plain.instances.size() == 1);
    const SupportAnalysis::Report &measured = plain.instances.front();
    REQUIRE(measured.coverage_available);
    REQUIRE(measured.contact_risk_available);
    REQUIRE(! measured.key.region_ids.empty());
    REQUIRE(! measured.missing_anchor_ids.empty());
    REQUIRE(plain.status == AutoTilt::PoseEvaluation::Status::UnresolvedCoverage);
    REQUIRE(std::find(plain.reason_codes.begin(), plain.reason_codes.end(),
                      std::string("required_region_unsupported")) != plain.reason_codes.end());

    // With the mode on, the generator thins the contacts of the one problem it prepared, and the
    // report the object hands out still names the regions that problem carried.
    const AutoTilt::PoseEvaluation mode_on =
        evaluate(config({ { "support_style", "tree_slim" }, { "support_miniature_contacts", "1" },
                          { "support_contact_min_distance", "1" } }));
    INFO("reasons: " << reasons_of(mode_on));
    REQUIRE(mode_on.instances.size() == 1);
    REQUIRE(! mode_on.instances.front().key.region_ids.empty());

    // And the rule every one of these reports is read under: Complete says every domain the
    // measurement was asked for came back, and nothing less says it.
    for (const AutoTilt::PoseEvaluation *evaluation : { &plain, &mode_on })
        for (const SupportAnalysis::Report &report : evaluation->instances)
            REQUIRE((report.status == SupportAnalysis::Report::Status::Complete) ==
                    (report.coverage_available && report.stability.available && report.damage.available));
}

TEST_CASE("Generated evaluation reads an object with no overhang as complete", "[AutoTilt]")
{
    // A cube standing on its face has no overhang, so the measured pass requires no region: nothing
    // is left to cover, and the object stands on the plate by itself.
    Slic3r::Model model = Slic3r::Test::model("cube", cube(8.));
    model.objects.front()->instances.front()->set_offset(Vec3d(100., 100., 0.));
    model.objects.front()->ensure_on_bed();
    const DynamicPrintConfig config =
        fixture_config({ { "support_style", "tree_slim" }, { "support_top_z_distance", "0.2" }, { "layer_change_gcode", "G92 E0" } });

    AutoTilt::GeneratedEvaluator evaluator(plate_input(model, config, { model.objects.front()->instances.front()->id() }),
                                           inline_runner());
    const AutoTilt::PoseEvaluation evaluation = evaluator.evaluate(AutoTilt::Pose{}, {});
    INFO("reasons: " << reasons_of(evaluation));
    REQUIRE(evaluation.instances.size() == 1);
    CHECK(evaluation.status == AutoTilt::PoseEvaluation::Status::Complete);

    const SupportAnalysis::Report &report = evaluation.instances.front();
    CHECK(report.status == SupportAnalysis::Report::Status::Complete);
    CHECK(report.coverage_available);
    CHECK(report.stability.available);
    CHECK(report.missing_anchor_ids.empty());
    CHECK(report.has_reason(SupportAnalysis::Reason::NoProblem));
    CHECK(SupportAnalysis::stability_admissible(report.stability));
    CHECK(std::find(evaluation.reason_codes.begin(), evaluation.reason_codes.end(),
                    std::string("required_region_unsupported")) == evaluation.reason_codes.end());
}

TEST_CASE("Generated evaluation counts every affected instance and refreshes the footprint after brim", "[AutoTilt]")
{
    // Two copies of the selected object, and a neighbour object nothing selected that needs support
    // of its own. One rotation moves both copies; the neighbour is somebody else's material.
    // Placed well inside the 200 x 200 mm default bed: TreeSupport clips its branches to the machine
    // border, so an object hanging off the plate would report support it never had to lay.
    Slic3r::Model model = Slic3r::Test::model("fin", fin_fixture());
    ModelObject  *obj   = model.objects.front();
    obj->instances.front()->set_offset(Vec3d(70., 100., 0.));
    obj->add_instance(Vec3d(100., 100., 0.), Vec3d(1., 1., 1.), Vec3d(0., 0., 0.), Vec3d(1., 1., 1.));
    obj->ensure_on_bed();
    ModelObject *neighbour = model.add_object();
    neighbour->add_volume(fin_fixture());
    neighbour->add_instance()->set_offset(Vec3d(135., 100., 0.));
    neighbour->ensure_on_bed();

    const std::vector<ObjectID> affected{ obj->instances[0]->id(), obj->instances[1]->id() };
    const auto run = [&](const char *brim) {
        const DynamicPrintConfig config =
            fixture_config({ { "support_style", "tree_slim" }, { "support_top_z_distance", "0.2" },
                             { "layer_change_gcode", "G92 E0" }, { "brim_type", brim }, { "brim_width", "5" } });
        return plate_input(model, config, affected);
    };

    AutoTilt::GeneratedEvaluator    evaluator(run("outer_only"), inline_runner());
    const AutoTilt::PoseEvaluation  brimmed = evaluator.evaluate(AutoTilt::Pose{ -6., 0. }, {});
    INFO("reasons: " << reasons_of(brimmed));

    // One measurement per affected physical instance, and the material totals are those measurements
    // summed: output two copies print is charged for two copies.
    REQUIRE(brimmed.instances.size() == 2);
    double summed_support = 0., summed_raft = 0.;
    for (const SupportAnalysis::Report &report : brimmed.instances) {
        summed_support += report.support_volume_mm3;
        summed_raft    += report.raft_volume_mm3;
    }
    REQUIRE(summed_support > 0.);
    REQUIRE_THAT(brimmed.support_volume_mm3, WithinRel(summed_support, 1e-12));
    REQUIRE_THAT(brimmed.raft_volume_mm3, WithinAbs(summed_raft, 1e-12));

    // The neighbour printed support of its own, and none of it is charged to the selected model.
    const PrintObject *neighbour_object = nullptr;
    for (const PrintObject *candidate : evaluator.print(0).objects())
        if (candidate->model_object()->id() == neighbour->id())
            neighbour_object = candidate;
    REQUIRE(neighbour_object != nullptr);
    REQUIRE(neighbour_object->support_analysis() != nullptr);
    REQUIRE(neighbour_object->support_analysis()->support_volume_mm3 > 0.);
    REQUIRE(brimmed.support_volume_mm3 < summed_support + neighbour_object->support_analysis()->support_volume_mm3);

    // Each instance keeps its own safety measures: two separate stability measurements, neither of
    // them an average of the pair.
    for (const SupportAnalysis::Report &report : brimmed.instances)
        REQUIRE(report.stability.available);

    // The brim is ground the object stands on that the support measurement never saw: that measurement
    // runs inside the support generation, and the print lays the brim after it. The object still holds
    // the report as it was generated, so the same print says both numbers and the difference between
    // them is the brim and nothing else.
    const PrintObject *selected = nullptr;
    for (const PrintObject *candidate : evaluator.print(0).objects())
        if (candidate->model_object()->id() == obj->id())
            selected = candidate;
    REQUIRE(selected != nullptr);
    REQUIRE(selected->support_analysis() != nullptr);
    const SupportAnalysis::Stability &as_generated = selected->support_analysis()->stability;
    REQUIRE(as_generated.available);
    for (size_t i = 0; i < 2; ++ i) {
        INFO("instance " << i);
        REQUIRE(brimmed.instances[i].stability.available);
        REQUIRE(brimmed.instances[i].stability.bed_footprint_mm2 > as_generated.bed_footprint_mm2);
        // The margin is read off that footprint, so it moved with it rather than staying the number
        // taken before the brim existed.
        REQUIRE(brimmed.instances[i].stability.min_bed_margin > as_generated.min_bed_margin);
    }
}

TEST_CASE("Generated evaluation reproduces an independent full Print and refuses a pose the plate cannot hold", "[AutoTilt]")
{
    const auto base_config = [](std::initializer_list<ConfigBase::SetDeserializeItem> extra) {
        DynamicPrintConfig out = fixture_config({ { "support_style", "tree_slim" }, { "layer_change_gcode", "G92 E0" },
                                                  { "brim_type", "no_brim" } });
        out.set_deserialize_strict(extra);
        return out;
    };

    // Variable layer regions are settings, and settings are authoritative: 0.08 mm below 10 mm and
    // 0.16 mm above it survive into the Z values the generated support was laid at.
    {
        Slic3r::Model model = Slic3r::Test::model("fin", fin_fixture());
        ModelObject  *obj   = model.objects.front();
        obj->instances.front()->set_offset(Vec3d(100., 100., 0.));
        obj->ensure_on_bed();
        ModelConfig fine, coarse;
        fine.set("layer_height", 0.08);
        coarse.set("layer_height", 0.16);
        obj->layer_config_ranges[t_layer_height_range(0.0, 10.0)]  = fine;
        obj->layer_config_ranges[t_layer_height_range(10.0, 60.0)] = coarse;

        AutoTilt::GeneratedEvaluator evaluator(
            plate_input(model, base_config({ { "support_top_z_distance", "0" } }), { obj->instances.front()->id() }),
            inline_runner());
        const AutoTilt::PoseEvaluation evaluation = evaluator.evaluate(AutoTilt::Pose{}, {});
        INFO("reasons: " << reasons_of(evaluation));
        REQUIRE(evaluation.instances.size() == 1);

        const PrintObject *po = evaluator.print(0).objects().front();
        size_t             object_fine = 0, object_coarse = 0, support_fine = 0, support_coarse = 0;
        for (const Layer *layer : po->layers()) {
            object_fine   += std::abs(layer->height - 0.08) < 1e-6 ? 1 : 0;
            object_coarse += std::abs(layer->height - 0.16) < 1e-6 ? 1 : 0;
        }
        for (const SupportLayer *layer : po->support_layers()) {
            support_fine   += std::abs(layer->height - 0.08) < 1e-6 ? 1 : 0;
            support_coarse += std::abs(layer->height - 0.16) < 1e-6 ? 1 : 0;
        }
        REQUIRE(object_fine > 0);
        REQUIRE(object_coarse > 0);
        REQUIRE(support_fine > 0);
        REQUIRE(support_coarse > 0);

        // A zero planned gap puts the tips on the model, so what is reported is material the print
        // actually laid rather than a drawn area that never became an extrusion.
        REQUIRE(evaluation.support_volume_mm3 > 0.);
    }

    // Two instances at different scales: one rotation moves both, each contributes its own
    // measurement, and each carries the same requested rotation relative to its own root.
    {
        Slic3r::Model model = Slic3r::Test::model("fin", fin_fixture());
        ModelObject  *obj   = model.objects.front();
        obj->instances.front()->set_offset(Vec3d(70., 100., 0.));
        obj->add_instance(Vec3d(120., 100., 0.), Vec3d(1.4, 1.4, 1.4), Vec3d(0., 0., 0.), Vec3d(1., 1., 1.));
        obj->ensure_on_bed();
        const std::vector<ObjectID> affected{ obj->instances[0]->id(), obj->instances[1]->id() };

        AutoTilt::GeneratedEvaluator   evaluator(plate_input(model, base_config({ { "support_top_z_distance", "0.2" } }), affected),
                                                 inline_runner());
        const AutoTilt::Pose           pose{ -8., 4. };
        const AutoTilt::PoseEvaluation evaluation = evaluator.evaluate(pose, {});
        INFO("reasons: " << reasons_of(evaluation));

        // Two different scales are two different PrintObjects, and both of them printed support.
        REQUIRE(evaluation.instances.size() == 2);
        REQUIRE(evaluator.print(0).objects().size() == 2);
        for (const SupportAnalysis::Report &report : evaluation.instances)
            REQUIRE(report.support_volume_mm3 > 0.);

        const Transform3d rotation =
            Geometry::rotation_transform(Vec3d(Geometry::deg2rad(pose.tilt_deg), Geometry::deg2rad(pose.lean_deg), 0.));
        for (size_t i = 0; i < 2; ++ i) {
            INFO("instance " << i);
            Transform3d live = Transform3d::Identity();
            for (const ModelObject *object : evaluator.model(0).objects)
                for (const ModelInstance *instance : object->instances)
                    if (instance->id() == affected[i])
                        live = instance->get_transformation().get_matrix();
            const Transform3d root     = obj->instances[i]->get_transformation().get_matrix();
            const Matrix3d    relative = live.linear() * root.linear().inverse();
            REQUIRE((relative - rotation.linear()).cwiseAbs().maxCoeff() < 1e-9);
        }
    }

    // A pose the plate cannot hold is refused before anything is sliced: the L-shaped printable
    // ground is not the rectangle its bounding box would be, and a second plate's exclusion volume is
    // just as final as the first plate's ground.
    {
        Slic3r::Model model = Slic3r::Test::model("fin", fin_fixture());
        ModelObject  *obj   = model.objects.front();
        obj->instances.front()->set_offset(Vec3d(100., 100., 0.));
        obj->add_instance(Vec3d(60., 60., 0.), Vec3d(1., 1., 1.), Vec3d(0., 0., 0.), Vec3d(1., 1., 1.));
        obj->ensure_on_bed();
        const DynamicPrintConfig config = base_config({ { "support_top_z_distance", "0.2" } });

        // The L: everything the 200 x 200 mm plate can print except its far quarter, which is where
        // the first instance stands. A bounding-box test would have called that inside.
        ExPolygon l_shape;
        for (const Vec2d &point : { Vec2d(0., 0.), Vec2d(200., 0.), Vec2d(200., 80.), Vec2d(80., 80.),
                                    Vec2d(80., 200.), Vec2d(0., 200.) })
            l_shape.contour.points.emplace_back(scaled<coord_t>(point.x()), scaled<coord_t>(point.y()));

        AutoTilt::EvaluationInput outside = plate_input(model, config, { obj->instances[0]->id() });
        outside.plates.front().printable_regions = ExPolygons{ l_shape };
        AutoTilt::GeneratedEvaluator   refuser(outside, inline_runner());
        const AutoTilt::PoseEvaluation refused = refuser.evaluate(AutoTilt::Pose{ -8., 0. }, {});
        REQUIRE(refused.status == AutoTilt::PoseEvaluation::Status::Invalid);
        REQUIRE(std::find(refused.reason_codes.begin(), refused.reason_codes.end(),
                          std::string("outside_printable_region")) != refused.reason_codes.end());
        // Before processing: nothing was applied, so the private Print holds no object at all.
        REQUIRE(refuser.print(0).objects().empty());
        REQUIRE(refused.instances.empty());

        // A second plate, whose own instance reaches into a volume nothing may print in.
        AutoTilt::EvaluationInput two_plates = plate_input(model, config, { obj->instances[0]->id() });
        AutoTilt::PlateInput      second     = two_plates.plates.front();
        second.plate_index                   = 1;
        second.affected_instance_ids         = { obj->instances[1]->id() };
        second.exclusions.push_back(BoundingBoxf3(Vec3d(40., 40., 0.), Vec3d(90., 90., 40.)));
        two_plates.plates.push_back(std::move(second));
        AutoTilt::GeneratedEvaluator   two(two_plates, inline_runner());
        const AutoTilt::PoseEvaluation excluded = two.evaluate(AutoTilt::Pose{ -8., 0. }, {});
        REQUIRE(excluded.status == AutoTilt::PoseEvaluation::Status::Invalid);
        REQUIRE(std::find(excluded.reason_codes.begin(), excluded.reason_codes.end(),
                          std::string("exclusion_area")) != excluded.reason_codes.end());
        REQUIRE(two.print(1).objects().empty());
    }

    // Seven runs of the evaluator against seven runs of a full Print initialized on its own, over one
    // fixture. Legacy tree generation is not run-to-run reproducible, so what has to match exactly is
    // what the measurement asserts about the pose - its status and the anchors it never reached -
    // while the unsupported-path count and every continuous metric are compared as the band each path
    // spans, on this fixture's own variability rather than any other's.
    {
        Slic3r::Model model = Slic3r::Test::model("table", table_fixture());
        ModelObject  *obj   = model.objects.front();
        obj->instances.front()->set_offset(Vec3d(100., 100., 0.));
        obj->ensure_on_bed();
        const std::vector<ObjectID> affected{ obj->instances.front()->id() };
        const DynamicPrintConfig    config = base_config({ { "support_top_z_distance", "0.2" } });
        const AutoTilt::EvaluationInput input = plate_input(model, config, affected);

        // The same posed model an evaluator would slice, handed to a Print this test builds itself.
        Slic3r::Model posed = input.plates.front().model;
        for (const AutoTilt::InstanceSnapshot &instance : AutoTilt::posed_instances(input, AutoTilt::Pose{}))
            for (ModelObject *object : posed.objects)
                for (ModelInstance *live : object->instances)
                    if (live->id() == instance.id) {
                        live->set_transformation(Geometry::Transformation(instance.matrix));
                        object->invalidate_bounding_box();
                    }

        struct Path { std::vector<double> support, contact, margin, unsupported; };
        Path                          evaluated, reference;
        std::vector<int>              status;
        std::vector<std::vector<uint64_t>> missing;
        const auto record = [&](Path &path, const SupportAnalysis::Report &report) {
            path.support.push_back(report.support_volume_mm3);
            path.contact.push_back(report.measured_contact_mm2);
            path.margin.push_back(report.stability.min_bed_margin);
            path.unsupported.push_back(double(report.stability.unsupported_paths));
            status.push_back(int(report.status));
            missing.push_back(report.missing_anchor_ids);
        };

        AutoTilt::GeneratedEvaluator evaluator(input, inline_runner());
        for (size_t run = 0; run < 7; ++ run) {
            const AutoTilt::PoseEvaluation evaluation = evaluator.evaluate(AutoTilt::Pose{}, {});
            INFO("evaluator run " << run << ", reasons: " << reasons_of(evaluation));
            REQUIRE(evaluation.instances.size() == 1);
            record(evaluated, evaluation.instances.front());
        }
        for (size_t run = 0; run < 7; ++ run) {
            INFO("reference run " << run);
            Slic3r::Print full;
            Slic3r::Model copy = posed;
            full.set_status_silent();
            full.apply(copy, config);
            REQUIRE(full.validate().string.empty());
            full.request_legacy_support_analysis();
            full.process();
            const std::shared_ptr<const SupportAnalysis::Report> report = full.objects().front()->support_analysis();
            REQUIRE(report != nullptr);
            record(reference, *report);
        }

        // What the measurement asserts, exactly, on every one of the fourteen runs. The count of
        // paths left standing on nothing is not among them: a single leg of table_fixture flips
        // between 0 and 1 across generator runs, so it is compared as a band below.
        for (size_t i = 1; i < status.size(); ++ i) {
            INFO("run " << i);
            REQUIRE(status[i] == status.front());
            REQUIRE(missing[i] == missing.front());
        }

        // And each continuous metric as a band: the two bands have to overlap, and the two medians
        // have to sit no further apart than the wider of the two bands plus the numeric tolerance.
        const auto band = [](std::vector<double> values) {
            std::sort(values.begin(), values.end());
            return std::array<double, 3>{ values.front(), values.back(), values[values.size() / 2] };
        };
        const std::pair<const char *, std::pair<const std::vector<double> *, const std::vector<double> *>> metrics[3] = {
            { "support_volume_mm3", { &evaluated.support, &reference.support } },
            { "measured_contact_mm2", { &evaluated.contact, &reference.contact } },
            { "min_bed_margin", { &evaluated.margin, &reference.margin } },
        };
        // Bands of zero would agree with anything, so the two paths have to have measured something.
        for (const std::vector<double> *values : { &evaluated.support, &reference.support, &evaluated.contact, &reference.contact })
            for (double value : *values)
                REQUIRE(value > 0.);

        for (const auto &metric : metrics) {
            const std::array<double, 3> a = band(*metric.second.first);
            const std::array<double, 3> b = band(*metric.second.second);
            INFO(metric.first << ": evaluator [" << a[0] << ", " << a[1] << "] median " << a[2]
                              << " | full Print [" << b[0] << ", " << b[1] << "] median " << b[2]);
            REQUIRE(a[0] <= b[1]);
            REQUIRE(b[0] <= a[1]);
            const double width   = std::max(a[1] - a[0], b[1] - b[0]);
            const double epsilon = std::max(1e-9, 1e-6 * std::max(std::abs(a[2]), std::abs(b[2])));
            REQUIRE(std::abs(a[2] - b[2]) <= width + epsilon);
        }

        // The unsupported-path count as a band too: one generator run in a hundred leaves a leg of
        // this fixture standing on nothing, so a lone flip on either side is inside the two bands and
        // leaves both medians where they were. An integer count needs no epsilon, and a majority is a
        // majority, so the medians have to be the same count - four of seven runs disagreeing moves
        // one of them and fails here.
        {
            const std::array<double, 3> a = band(evaluated.unsupported);
            const std::array<double, 3> b = band(reference.unsupported);
            INFO("unsupported_paths: evaluator [" << a[0] << ", " << a[1] << "] median " << a[2]
                                                  << " | full Print [" << b[0] << ", " << b[1] << "] median " << b[2]);
            REQUIRE(a[0] <= b[1]);
            REQUIRE(b[0] <= a[1]);
            REQUIRE(a[2] == b[2]);
        }
    }
}

TEST_CASE("Generated evaluation slices a plate other than the first on its own origin", "[AutoTilt]")
{
    // The second plate of a 200 mm bed sits 1.2 bed widths along x, and an instance on it carries that
    // offset. The machine border the tree generator clips every support area to is placed by the plate
    // origin, so a pose Print left at the first plate's origin clips this object's support to nothing.
    Slic3r::Model model = Slic3r::Test::model("fin", fin_fixture());
    ModelObject  *obj   = model.objects.front();
    obj->instances.front()->set_offset(Vec3d(340., 100., 0.));
    obj->ensure_on_bed();
    const ObjectID id = obj->instances.front()->id();

    const DynamicPrintConfig config = fixture_config({ { "support_style", "tree_slim" }, { "layer_change_gcode", "G92 E0" },
                                                       { "brim_type", "no_brim" }, { "support_top_z_distance", "0" } });
    AutoTilt::EvaluationInput input = plate_input(model, config, { id });
    input.plates.front().plate_index  = 1;
    input.plates.front().plate_origin = Vec3d(240., 0., 0.);

    AutoTilt::GeneratedEvaluator   evaluator(input, inline_runner());
    const AutoTilt::PoseEvaluation evaluation = evaluator.evaluate(AutoTilt::Pose{}, {});
    INFO("reasons: " << reasons_of(evaluation));
    REQUIRE(evaluation.instances.size() == 1);
    REQUIRE(evaluation.support_volume_mm3 > 0.);
}

namespace {

// The settings the cancellation cases slice under: legacy tree support, a stated contact gap, and
// the layer-change G-code Print::validate insists on.
DynamicPrintConfig cancellation_config()
{
    return fixture_config({ { "support_style", "tree_slim" }, { "layer_change_gcode", "G92 E0" },
                            { "support_top_z_distance", "0.2" }, { "brim_type", "no_brim" } });
}

} // namespace

TEST_CASE("A generated evaluation canceled from another thread stops its Print and starts no later pose", "[AutoTilt]")
{
    Slic3r::Model model = Slic3r::Test::model("table", table_fixture());
    model.objects.front()->ensure_on_bed();
    const std::vector<ObjectID> affected{ model.objects.front()->instances.front()->id() };

    // Two captured plates, so a run that is stopped on the first can be told from a run that simply
    // finished: the second plate is work the evaluator would otherwise go on to.
    AutoTilt::EvaluationInput input = plate_input(model, cancellation_config(), affected);
    input.plates.push_back(input.plates.front());
    input.plates.back().plate_index = 1;

    std::mutex              mutex;
    std::condition_variable slicing;
    bool                    started = false;
    const auto              signal  = [&mutex, &slicing, &started]() {
        {
            std::lock_guard<std::mutex> lock(mutex);
            started = true;
        }
        slicing.notify_all();
    };

    // The Print's status callback is the one hook that fires from inside Print::process(), so the
    // driver is released while real slicing work is running rather than before it starts.
    size_t                       plate = 0;
    AutoTilt::GeneratedEvaluator evaluator(input, [&](const std::function<void()> &fn) {
        fn();
        evaluator.print(plate ++).set_status_callback([&signal](const PrintBase::SlicingStatus &) { signal(); });
    });

    // The driver waits and cancels and does nothing else: Catch2's assertions are not thread safe,
    // so every expectation below is checked on this thread after the join.
    std::thread driver([&mutex, &slicing, &started, &evaluator]() {
        std::unique_lock<std::mutex> lock(mutex);
        slicing.wait(lock, [&started]() { return started; });
        lock.unlock();
        evaluator.cancel();
    });

    const AutoTilt::PoseEvaluation out = evaluator.evaluate(AutoTilt::Pose{ -8., 0. }, {});
    signal(); // releases the driver even if the pose never reached the slicer
    driver.join();

    REQUIRE(out.status == AutoTilt::PoseEvaluation::Status::Canceled);
    REQUIRE(std::find(out.reason_codes.begin(), out.reason_codes.end(), std::string("canceled")) != out.reason_codes.end());
    REQUIRE(out.instances.empty());
    REQUIRE_THAT(out.support_volume_mm3, WithinAbs(0., 1e-12));
    // Real Print work had started - the plate was applied and slicing had reported progress - and it
    // was stopped where it stood: the generator never reached the support layers this fixture prints.
    REQUIRE_FALSE(evaluator.print(0).objects().empty());
    REQUIRE(evaluator.print(0).objects().front()->support_layers().empty());
    // The second plate was never applied: the cancellation stopped the run rather than ending it.
    REQUIRE(evaluator.print(1).objects().empty());

    // And the latch outlives the call it was set in, so no later pose starts on a canceled evaluator.
    const AutoTilt::PoseEvaluation later = evaluator.evaluate(AutoTilt::Pose{}, {});
    REQUIRE(later.status == AutoTilt::PoseEvaluation::Status::Canceled);
    REQUIRE(later.instances.empty());
    REQUIRE(evaluator.print(1).objects().empty());
}

TEST_CASE("Generated evaluation checks the stop predicate before it applies a private Print", "[AutoTilt]")
{
    Slic3r::Model model = Slic3r::Test::model("table", table_fixture());
    model.objects.front()->ensure_on_bed();
    const std::vector<ObjectID> affected{ model.objects.front()->instances.front()->id() };

    const AutoTilt::EvaluationInput input    = plate_input(model, cancellation_config(), affected);
    const AutoTilt::EvaluationInput captured = input;

    // Let the entry check through and stop on the next one, which falls inside the geometry loop the
    // pose is admitted by - before the private clone and apply, not after them.
    size_t                       calls = 0;
    bool                         ran   = false;
    AutoTilt::GeneratedEvaluator evaluator(input, [&ran](const std::function<void()> &fn) { ran = true; fn(); });
    const AutoTilt::PoseEvaluation out = evaluator.evaluate(AutoTilt::Pose{ -8., 0. }, [&calls]() { return ++ calls > 1; });

    REQUIRE(out.status == AutoTilt::PoseEvaluation::Status::Canceled);
    REQUIRE_FALSE(ran);
    REQUIRE(evaluator.model(0).objects.empty());
    REQUIRE(evaluator.print(0).objects().empty());
    REQUIRE(out.instances.empty());

    // Nothing the evaluation was given moved: a stopped pose leaves the scene it was measured on
    // exactly as it found it, which is the state a later result would have to match.
    REQUIRE(AutoTilt::evaluation_inputs_unchanged(captured, input));

    // A stop predicate is one call's business and not a latch, so the same evaluator answers the next
    // pose in full. Only cancel() stops an evaluator for good, and only cancel() may be called from
    // a thread that is not the one running evaluate().
    const AutoTilt::PoseEvaluation next = evaluator.evaluate(AutoTilt::Pose{}, {});
    REQUIRE(next.status != AutoTilt::PoseEvaluation::Status::Canceled);
    REQUIRE_FALSE(evaluator.print(0).objects().empty());
}

TEST_CASE("A fragile contact in the lower fifth changes the verified ranking and not the estimate", "[AutoTilt]")
{
    const DynamicPrintConfig config = fixture_config({ { "support_style", "tree_slim" },
                                                       { "support_top_z_distance", "0.2" },
                                                       { "layer_change_gcode", "G92 E0" },
                                                       { "brim_type", "no_brim" } });
    // The same object twice, once carrying the low shelf and once without it, so what the shelf costs
    // can be read off the difference rather than asserted about one number.
    const auto placed = [&config](TriangleMesh &&mesh, Slic3r::Model &model) {
        model = Slic3r::Test::model("table", std::move(mesh));
        ModelObject *object = model.objects.front();
        object->instances.front()->set_offset(Vec3d(100., 100., 0.));
        object->ensure_on_bed();
        return plate_input(model, config, { object->instances.front()->id() });
    };
    Slic3r::Model                   shelf_model, plain_model;
    const AutoTilt::EvaluationInput shelf = placed(low_shelf_fixture(), shelf_model);
    const AutoTilt::EvaluationInput plain = placed(table_fixture(), plain_model);

    // Four poses: full evaluation slices every one of them, so this is what a test can afford. The
    // bottom exclusion plane keeps the fraction the search ships with.
    AutoTilt::Constants k;
    k.tilts_deg = { 0., -6. };
    k.leans_deg = { 0., 10. };
    const std::vector<AutoTilt::Pose> legal = AutoTilt::grid(k);
    REQUIRE(legal.size() == 4);

    // The estimate, with the plane where it ships and with it switched off. The shelf's overhang is
    // the only contact below a fifth of this object's height, so the difference between those two
    // readings is the shelf - and with the plane on, the estimate cannot tell the shelf model from
    // the plain one at all.
    AutoTilt::Constants k0       = k;
    k0.bottom_exclusion_fraction = 0.;
    AutoTilt::LegacyShortlistScorer excluding(shelf, k, inline_runner());
    AutoTilt::LegacyShortlistScorer whole(shelf, k0, inline_runner());
    AutoTilt::LegacyShortlistScorer without_shelf(plain, k, inline_runner());
    const double estimate_excluding = excluding.score(AutoTilt::Pose{}).score_mm3;
    const double estimate_whole     = whole.score(AutoTilt::Pose{}).score_mm3;
    const double estimate_plain     = without_shelf.score(AutoTilt::Pose{}).score_mm3;
    INFO("estimate with the plane " << estimate_excluding << ", without it " << estimate_whole
                                    << ", on the plain object " << estimate_plain);
    REQUIRE(estimate_whole > 0.);
    REQUIRE(estimate_excluding < estimate_whole * (1. - 1e-5));
    REQUIRE_THAT(estimate_excluding, WithinRel(estimate_plain, 1e-4));

    // The verified path over the same four poses.
    AutoTilt::GeneratedEvaluator         evaluator(shelf, inline_runner());
    AutoTilt::LegacyShortlistScorer      cheap(shelf, k, inline_runner());
    const AutoTilt::VerifiedSearchResult verified =
        AutoTilt::search_verified(legal, cheap, evaluator, k, [] { return false; }, [](size_t, size_t) {});
    INFO("verified outcome " << int(verified.outcome) << ", root status " << int(verified.root.status)
                             << ", selected " << verified.selected.pose.tilt_deg << "/" << verified.selected.pose.lean_deg
                             << ", reasons: " << reasons_of(verified.selected));

    REQUIRE(verified.root.status == AutoTilt::PoseEvaluation::Status::Complete);
    REQUIRE(verified.root.instances.size() == 1);
    REQUIRE(verified.cheap_scored == 3);
    REQUIRE(verified.verified == 4); // the root plus three finalists, which is this whole grid

    // The shelf the estimate dropped is a required region of the print all the same, and the root
    // pose was measured for it: full evaluation never skips what the exclusion plane hides. The
    // plain object, whose estimate is the same number, carries no requirement down there at all, so
    // this is the shelf and not some property the two objects share.
    AutoTilt::GeneratedEvaluator   plain_evaluator(plain, inline_runner());
    const AutoTilt::PoseEvaluation plain_root = plain_evaluator.evaluate(AutoTilt::Pose{}, {});
    REQUIRE(plain_root.status == AutoTilt::PoseEvaluation::Status::Complete);
    const auto low_regions = [](const SupportAnalysis::Report &report) {
        size_t n = 0;
        for (size_t i = 0; i < report.key.region_ids.size(); ++ i)
            if (report.key.region_contact_z[i] < 0.20 * 23.) // a fifth of the object's own height
                ++ n;
        return n;
    };
    const SupportAnalysis::Report &root_report = verified.root.instances.front();
    INFO("required regions below the plane: with the shelf " << low_regions(root_report) << ", without it "
                                                             << low_regions(plain_root.instances.front()));
    REQUIRE(low_regions(root_report) > 0);
    REQUIRE(low_regions(plain_root.instances.front()) == 0);
    REQUIRE(verified.root.support_volume_mm3 > 0.);
    REQUIRE(AutoTilt::objectives(verified.root).damage.available); // the ranking had a full tuple to compare

    // The two rankings then disagree, and it is the verified one the object is moved on. The estimate
    // reads a 6 degree tilt as the pose that removes the most contact; the print, sliced, leaves
    // required regions of that pose unsupported, so nothing is applied and the object stays as it is.
    AutoTilt::LegacyShortlistScorer estimating(shelf, k, inline_runner());
    const AutoTilt::SearchResult    estimated =
        AutoTilt::search(legal, estimating, k, [] { return false; }, [](size_t, size_t) {});
    INFO("estimate outcome " << int(estimated.outcome) << ", best " << estimated.best.tilt_deg << "/" << estimated.best.lean_deg);
    REQUIRE(estimated.outcome == AutoTilt::SearchResult::Outcome::Improved);
    REQUIRE_FALSE(estimated.best.is_root());
    REQUIRE(verified.outcome == AutoTilt::VerifiedSearchResult::Outcome::NoImprovement);
    REQUIRE(verified.selected.pose.is_root());

    // The shortlist score is one answer over every instance the pose would move: a second copy of the
    // object on the plate is a second copy of the contact, and the estimate says so.
    Slic3r::Model  two_model = Slic3r::Test::model("table", low_shelf_fixture());
    ModelObject   *two_object = two_model.objects.front();
    two_object->instances.front()->set_offset(Vec3d(70., 100., 0.));
    two_object->add_instance(Vec3d(120., 100., 0.), Vec3d(1., 1., 1.), Vec3d(0., 0., 0.), Vec3d(1., 1., 1.));
    two_object->ensure_on_bed();
    AutoTilt::LegacyShortlistScorer both(
        plate_input(two_model, config, { two_object->instances[0]->id(), two_object->instances[1]->id() }), k,
        inline_runner());
    REQUIRE(both.instance_count() == 2);
    REQUIRE_THAT(both.score(AutoTilt::Pose{}).score_mm3, WithinRel(2. * estimate_excluding, 1e-6));

    // An id the captured model does not carry resolves to no scorer at all, and the sum over nothing
    // is zero: the count is what says how many instances a score covers, never the score itself.
    AutoTilt::LegacyShortlistScorer none(plate_input(shelf_model, config, { ObjectID() }), k, inline_runner());
    REQUIRE(none.instance_count() == 0);
    REQUIRE_THAT(none.score(AutoTilt::Pose{}).score_mm3, WithinAbs(0., 0.));
}


// ---------------------------------------------------------------------------------------------
// Validation harness (hidden, see TEST_CASE "Auto-tilt validation harness over a corpus").
// ---------------------------------------------------------------------------------------------

namespace {

constexpr double harness_nan = std::numeric_limits<double>::quiet_NaN();

// Replays contacts the harness already measured, so search() drives the ranking without re-slicing.
// A pose the scorer could not measure carries an infinite score and can never win.
class TableScorer : public AutoTilt::Scorer
{
public:
    TableScorer(const std::vector<AutoTilt::Pose> &poses, const std::vector<AutoTilt::Contact> &contacts, const std::vector<char> &ok)
        : m_poses(poses), m_contacts(contacts), m_ok(ok)
    {}

    AutoTilt::Contact score(const AutoTilt::Pose &pose) override
    {
        const auto it = std::find(m_poses.begin(), m_poses.end(), pose);
        const size_t i = size_t(it - m_poses.begin());
        if (it == m_poses.end() || ! m_ok[i])
            return AutoTilt::Contact{ std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), 0. };
        return m_contacts[i];
    }

private:
    const std::vector<AutoTilt::Pose>    &m_poses;
    const std::vector<AutoTilt::Contact> &m_contacts;
    const std::vector<char>              &m_ok;
};

// Index of the pose the search would apply: the winner on Improved, the root on every other outcome.
size_t chosen_index(const AutoTilt::SearchResult &r, const std::vector<AutoTilt::Pose> &poses)
{
    const AutoTilt::Pose p  = r.outcome == AutoTilt::SearchResult::Outcome::Improved ? r.best : AutoTilt::Pose{};
    const auto           it = std::find(poses.begin(), poses.end(), p);
    return it == poses.end() ? 0 : size_t(it - poses.begin());
}

// One scoring column's sweep: every pose measured once, then search() ranked over the recorded table.
struct Column
{
    std::vector<AutoTilt::Contact> contacts;
    std::vector<char>              ok;
    AutoTilt::SearchResult         result;
    size_t                         chosen  = 0;
    double                         seconds = 0.;
};

Column sweep(AutoTilt::ContactScorer &scorer, const std::vector<AutoTilt::Pose> &poses, const AutoTilt::Constants &k)
{
    Column col;
    col.contacts.resize(poses.size());
    col.ok.assign(poses.size(), 0);
    // Only the scoring costs wall clock worth reporting; the table search() below runs in microseconds.
    const auto start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < poses.size(); ++ i) {
        try {
            col.contacts[i] = scorer.score(poses[i]);
            col.ok[i]       = 1;
        } catch (const std::exception &) {
            col.ok[i] = 0;
        }
    }
    col.seconds = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count();
    TableScorer table(poses, col.contacts, col.ok);
    col.result = AutoTilt::search(poses, table, k, []() { return false; }, [](size_t, size_t) {});
    col.chosen = chosen_index(col.result, poses);
    return col;
}

std::string pose_text(const AutoTilt::Pose &p)
{
    std::ostringstream os;
    os << p.tilt_deg << "/" << p.lean_deg;
    return os.str();
}

// Replays evaluations the exhaustive sweep already measured, so the production selection runs over
// the recorded table without slicing any pose a second time. A pose the sweep never measured is one
// the search cannot settle on.
class ReplayVerifier : public AutoTilt::Verifier
{
public:
    ReplayVerifier(const std::vector<AutoTilt::Pose> &poses, const std::vector<AutoTilt::PoseEvaluation> &evaluations)
        : m_poses(poses), m_evaluations(evaluations)
    {}

    AutoTilt::PoseEvaluation evaluate(const AutoTilt::Pose &pose, const AutoTilt::StopPredicate &) override
    {
        const auto it = std::find(m_poses.begin(), m_poses.end(), pose);
        if (it == m_poses.end()) {
            AutoTilt::PoseEvaluation unknown;
            unknown.pose = pose;
            return unknown;
        }
        return m_evaluations[size_t(it - m_poses.begin())];
    }

private:
    const std::vector<AutoTilt::Pose>            &m_poses;
    const std::vector<AutoTilt::PoseEvaluation>  &m_evaluations;
};

// One captured plate whose printable ground is the bed the config declares, so a pose the plate
// cannot hold is refused by the same exact containment test the production evaluation applies.
AutoTilt::EvaluationInput corpus_input(const Slic3r::Model &model, const DynamicPrintConfig &config)
{
    std::vector<ObjectID> affected;
    for (const ModelObject *object : model.objects)
        for (const ModelInstance *instance : object->instances)
            affected.push_back(instance->id());
    AutoTilt::EvaluationInput input = plate_input(model, config, affected);
    input.plates.front().printable_regions = ExPolygons{ ExPolygon(Polygon(get_bed_shape(config))) };
    return input;
}

// The field-by-field extremes of what a pose measured over its repeats. `optimistic` takes the best
// reading of every field and `pessimistic` the worst, so a comparison between two poses only calls
// one of them better where the gap is wider than either spread: legacy generation is not run-to-run
// reproducible, and one reading decides nothing.
struct ObjectiveEnvelope
{
    AutoTilt::Objectives optimistic;
    AutoTilt::Objectives pessimistic;
    bool                 measured = false;
};

ObjectiveEnvelope objective_envelope(const std::vector<AutoTilt::PoseEvaluation> &repeats)
{
    ObjectiveEnvelope out;
    for (const AutoTilt::PoseEvaluation &evaluation : repeats) {
        const AutoTilt::Objectives o = AutoTilt::objectives(evaluation);
        if (! o.damage.available)
            return ObjectiveEnvelope{};
        if (! out.measured) {
            out.optimistic  = o;
            out.pessimistic = o;
            out.measured    = true;
            continue;
        }
        out.optimistic.damage.unknown_contacts     = std::min(out.optimistic.damage.unknown_contacts, o.damage.unknown_contacts);
        out.optimistic.damage.inaccessible_groups  = std::min(out.optimistic.damage.inaccessible_groups, o.damage.inaccessible_groups);
        out.optimistic.damage.max_group_risk       = std::min(out.optimistic.damage.max_group_risk, o.damage.max_group_risk);
        out.optimistic.damage.total_group_risk     = std::min(out.optimistic.damage.total_group_risk, o.damage.total_group_risk);
        out.optimistic.volume_mm3                  = std::min(out.optimistic.volume_mm3, o.volume_mm3);
        out.pessimistic.damage.unknown_contacts    = std::max(out.pessimistic.damage.unknown_contacts, o.damage.unknown_contacts);
        out.pessimistic.damage.inaccessible_groups = std::max(out.pessimistic.damage.inaccessible_groups, o.damage.inaccessible_groups);
        out.pessimistic.damage.max_group_risk      = std::max(out.pessimistic.damage.max_group_risk, o.damage.max_group_risk);
        out.pessimistic.damage.total_group_risk    = std::max(out.pessimistic.damage.total_group_risk, o.damage.total_group_risk);
        out.pessimistic.volume_mm3                 = std::max(out.pessimistic.volume_mm3, o.volume_mm3);
    }
    return out;
}

// The exhaustive validation mode over one manifest case, one style and one feature mode: every
// entry the unchanged grid returns is evaluated `repeats` times under the actual settings, each
// evaluation writes its own row, and one selection row records what the production search settled
// on against the root it started from and against the best pose the grid held.
void run_exhaustive_case(const SupportValidation::Manifest &manifest, const SupportValidation::ManifestCase &entry,
                         const std::string &style, const std::string &mode, const AutoTilt::Constants &k,
                         std::ostream &rows)
{
    const std::vector<AutoTilt::Pose> poses = AutoTilt::grid(k);
    // The evaluator validates each posed plate before slicing it, and relative extruder addressing
    // makes Print::validate demand a per-layer reset; the loaded config still outranks this default.
    const SupportValidation::CorpusObject object =
        SupportValidation::case_object(manifest, entry, fixture_config({ { "layer_change_gcode", "G92 E0" } }), style, mode);
    const AutoTilt::EvaluationInput input = corpus_input(object.model, object.config);

    AutoTilt::GeneratedEvaluator                        evaluator(input, inline_runner());
    std::vector<std::vector<AutoTilt::PoseEvaluation>>  measured(poses.size());
    std::vector<AutoTilt::PoseEvaluation>               representative(poses.size());
    size_t                                              invalid_poses = 0;

    for (size_t i = 0; i < poses.size(); ++ i) {
        for (size_t repeat = 0; repeat < entry.repeats; ++ repeat) {
            const auto               start      = std::chrono::high_resolution_clock::now();
            const AutoTilt::PoseEvaluation evaluation = evaluator.evaluate(poses[i], []() { return false; });
            SupportValidation::CaseResult  row;
            row.harness           = "auto_tilt";
            row.case_id           = entry.id;
            row.style             = style;
            row.feature_mode      = mode;
            row.pose              = poses[i];
            row.repeat            = repeat;
            // Status, metrics, containment, the selection the instances came to and their reason
            // codes, read the one way both harnesses read them.
            SupportValidation::read_evaluation_analyses(evaluation, row);
            row.source_sha256     = entry.sha256;
            row.config_digest     = entry.config_digest;
            row.build_revision    = SupportValidation::build_revision();
            row.elapsed_s         = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count();
            row.peak_memory_bytes = SupportValidation::peak_memory_bytes();
            SupportValidation::write_result(row, rows);
            measured[i].push_back(evaluation);
        }
        representative[i] = measured[i].front();
        if (representative[i].status == AutoTilt::PoseEvaluation::Status::Invalid)
            ++ invalid_poses;
    }

    // What the production search settles on, over the table already measured: the cheap scorer
    // shortlists and the recorded evaluations rank, exactly as `search_verified` ranks them live.
    AutoTilt::ContactScorer      scorer(*object.model.objects.front(), object.config, k, inline_runner());
    ReplayVerifier               verifier(poses, representative);
    const AutoTilt::VerifiedSearchResult result =
        AutoTilt::search_verified(poses, scorer, verifier, k, []() { return false; }, [](size_t, size_t) {});

    std::vector<ObjectiveEnvelope> envelopes(poses.size());
    std::vector<AutoTilt::Objectives> quality(poses.size());
    std::vector<char>                 comparable(poses.size(), 0);
    for (size_t i = 0; i < poses.size(); ++ i) {
        envelopes[i]  = objective_envelope(measured[i]);
        quality[i]    = envelopes[i].optimistic;
        comparable[i] = envelopes[i].measured ? 1 : 0;
    }
    // The root is judged on its worst reading and every candidate on its best, so a pose is only
    // called worse than the root where the gap outruns both spreads.
    if (envelopes.front().measured)
        quality.front() = envelopes.front().pessimistic;

    const auto index_of = [&poses](const AutoTilt::Pose &pose) {
        const auto it = std::find(poses.begin(), poses.end(), pose);
        return it == poses.end() ? size_t(0) : size_t(it - poses.begin());
    };
    const size_t chosen = result.outcome == AutoTilt::VerifiedSearchResult::Outcome::Improved ?
        index_of(result.selected.pose) : size_t(0);

    size_t best = 0;
    for (size_t i = 0; i < poses.size(); ++ i)
        if (comparable[i] && (! comparable[best] || AutoTilt::compare_objectives(quality[best], envelopes[i].optimistic) > 0))
            best = i;

    SupportValidation::CaseResult summary;
    summary.row_type        = "selection";
    summary.harness         = "auto_tilt";
    summary.case_id         = entry.id;
    summary.style           = style;
    summary.feature_mode    = mode;
    summary.pose            = poses[chosen];
    summary.repeat          = 0;
    // The selection row is still a measurement of the pose the search settled on, so it carries what
    // a pose row carries - including the selection those instances came to, which the gates read.
    SupportValidation::read_evaluation_analyses(representative[chosen], summary);
    summary.source_sha256   = entry.sha256;
    summary.config_digest   = entry.config_digest;
    summary.build_revision  = SupportValidation::build_revision();
    summary.peak_memory_bytes = SupportValidation::peak_memory_bytes();
    summary.selection_summary.present         = true;
    summary.selection_summary.root_pose       = poses.front();
    summary.selection_summary.selected_pose   = poses[chosen];
    summary.selection_summary.best_pose       = poses[best];
    summary.selection_summary.grid_entries    = poses.size();
    summary.selection_summary.evaluated_poses = poses.size() - invalid_poses;
    summary.selection_summary.invalid_poses   = invalid_poses;
    summary.selection_summary.false_move      = SupportValidation::is_false_move(quality, comparable, chosen);
    const double regret = SupportValidation::regret_of(envelopes[chosen].optimistic, envelopes[best].optimistic);
    summary.selection_summary.regret_available = std::isfinite(regret);
    summary.selection_summary.regret           = std::isfinite(regret) ? regret : 0.;
    summary.selection_summary.discrete_worse  =
        SupportValidation::discrete_worse(envelopes[chosen].optimistic, envelopes[best].optimistic);
    // Why the search settled where it did, on top of the reason codes the chosen pose's own
    // evaluation already put on the row.
    for (const std::string &code : result.reason_codes)
        summary.reason_codes.push_back(code);
    SupportValidation::write_result(summary, rows);

    std::cout << "case " << entry.id << " " << style << "/" << mode
              << " grid=" << poses.size() << " invalid=" << invalid_poses
              << " selected=" << pose_text(poses[chosen]) << " best=" << pose_text(poses[best])
              << " false_move=" << (summary.selection_summary.false_move ? 1 : 0)
              << " regret=" << summary.selection_summary.regret
              << " discrete_worse=" << (summary.selection_summary.discrete_worse ? 1 : 0)
              << " peak_memory_bytes=" << summary.peak_memory_bytes
              << std::endl;
}

// Scores and reports one model with the cheap scorer. Writes `<stem>.autotilt.csv` beside the corpus when
// `corpus_dir` is set; prints the summary line either way.
void run_harness_model(size_t index, const std::string &stem, const Model &base, const DynamicPrintConfig &config,
                       const std::string &corpus_dir, const AutoTilt::Constants &k)
{
    const auto                        start = std::chrono::high_resolution_clock::now();
    const std::vector<AutoTilt::Pose> poses = AutoTilt::grid(k);
    const size_t                      n     = poses.size();
    const ModelObject                &obj   = *base.objects.front();

    AutoTilt::ContactScorer plain(obj, config, k, inline_runner());

    const Column col_plain = sweep(plain, poses, k);

    // The root-pose shape diagnostics describe the root pose alone, off a scorer of their own: score()
    // accumulates sharp-tail and cantilever state across calls, so a swept scorer would fold nine poses
    // of it into the root's rows.
    AutoTilt::ContactScorer                            diag(obj, config, k, inline_runner());
    std::vector<AutoTilt::ContactScorer::PolygonRecord> records;
    diag.records                   = &records;
    const AutoTilt::Contact root_c = diag.score(AutoTilt::Pose{});

    if (! corpus_dir.empty()) {
        const std::filesystem::path csv_path = std::filesystem::path(corpus_dir) / (stem + ".autotilt.csv");
        std::ofstream               csv(csv_path.string());
        csv << std::setprecision(12);
        csv << "tilt,lean,score_mm3,volume_mm3\n";
        for (size_t i = 0; i < n; ++ i)
            csv << poses[i].tilt_deg << "," << poses[i].lean_deg << ","
                << (col_plain.ok[i] ? col_plain.contacts[i].score_mm3 : harness_nan) << ","
                << (col_plain.ok[i] ? col_plain.contacts[i].volume_mm3 : harness_nan) << "\n";
        csv.close();

        size_t        lines = 0;
        std::ifstream back(csv_path.string());
        for (std::string line; std::getline(back, line); )
            ++ lines;
        REQUIRE(lines == n + 1);

        // One row per overhang polygon the root pose slices, excluded rows included: where to put the
        // exclusion plane is read off these, so the rows it would drop have to be in the file.
        const std::filesystem::path poly_path = std::filesystem::path(corpus_dir) / (stem + ".autotilt.polygons.csv");
        std::ofstream               poly(poly_path.string());
        poly << std::setprecision(12);
        poly << "layer,print_z_mm,area_mm2,perimeter_mm,t_mm,weight,root_height_fraction,type_floor,excluded\n";
        for (const AutoTilt::ContactScorer::PolygonRecord &r : records)
            poly << r.layer << "," << r.print_z_mm << "," << r.area_mm2 << "," << r.perimeter_mm << ","
                 << r.t_mm << "," << r.weight << "," << r.root_height_fraction << ","
                 << (r.type_floor ? 1 : 0) << "," << (r.excluded ? 1 : 0) << "\n";
        poly.close();

        size_t        poly_lines = 0;
        std::ifstream poly_back(poly_path.string());
        for (std::string line; std::getline(poly_back, line); )
            ++ poly_lines;
        REQUIRE(poly_lines == records.size() + 1);
    }

    // Also the height the search-height check below compares against.
    const double print_h = obj.config.has("layer_height") ? obj.config.opt_float("layer_height") : config.opt_float("layer_height");
    const Vec3d  bbox    = obj.instance_bounding_box(0).size();
    std::ostringstream   fields;
    fields << std::fixed << std::setprecision(2) << bbox.x() << "x" << bbox.y() << "x" << bbox.z();
    const std::string bbox_text = fields.str();
    fields.str(std::string());
    fields << std::setprecision(1) << col_plain.seconds;
    const std::string sweep_text = fields.str();
    fields.str(std::string());
    fields << std::setprecision(1) << std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count();
    const std::string elapsed_text = fields.str();

    std::cout << "model " << index << " " << stem
              << " chosen_plain=" << pose_text(poses[col_plain.chosen])
              << " facets=" << obj.facets_count()
              << " bbox_mm=" << bbox_text
              << " layer_height_mm=" << print_h
              << " sweep_s=" << sweep_text
              << " elapsed_s=" << elapsed_text
              << std::endl;

    // The root-pose shape diagnostics the corpus CSV carries, over the root pose's rows. `R` is what
    // the exclusion plane keeps; the percentiles read off the finite thicknesses in it (t is NaN for
    // a degenerate zero-perimeter contour), and `excluded_share` weighs what the plane drops against
    // every row, kept or not.
    std::vector<double> t_kept, wa_kept;
    double              area_h_all = 0., area_h_excluded = 0.;
    size_t              kept = 0, t_above_ref = 0;
    for (const AutoTilt::ContactScorer::PolygonRecord &r : records) {
        area_h_all += r.area_mm2 * k.h_ref_mm;
        if (r.excluded) {
            area_h_excluded += r.area_mm2 * k.h_ref_mm;
            continue;
        }
        ++ kept;
        wa_kept.push_back(r.weight * r.area_mm2);
        if (std::isfinite(r.t_mm)) {
            t_kept.push_back(r.t_mm);
            if (r.t_mm >= k.t_ref_mm)
                ++ t_above_ref;
        }
    }
    std::sort(t_kept.begin(), t_kept.end());
    const size_t t_n        = t_kept.size();
    // Nearest rank: ceil(P*n) is at least 1 for P > 0 and n >= 1.
    const auto   percentile = [&](double p) {
        return t_n == 0 ? harness_nan : t_kept[size_t(std::ceil(p * double(t_n))) - 1];
    };
    double top3_share = harness_nan, excluded_share = harness_nan;
    if (kept > 3) {
        std::partial_sort(wa_kept.begin(), wa_kept.begin() + 3, wa_kept.end(), std::greater<double>());
        top3_share = std::accumulate(wa_kept.begin(), wa_kept.begin() + 3, 0.) /
                     std::accumulate(wa_kept.begin(), wa_kept.end(), 0.);
    } else if (kept > 0) {
        top3_share = 1.;
    }
    if (kept > 0)
        excluded_share = area_h_excluded / area_h_all;

    std::cout << "model " << index << " " << stem
              << " diag root_volume_mm3=" << root_c.volume_mm3
              << " root_volume_fraction=" << (root_c.volume_mm3 / root_c.object_volume_mm3)
              << " t_p10=" << percentile(0.10)
              << " t_p50=" << percentile(0.50)
              << " t_p90=" << percentile(0.90)
              << " share_t_above_t_ref=" << (t_n == 0 ? harness_nan : double(t_above_ref) / double(t_n))
              << " top3_share=" << top3_share
              << " excluded_share=" << excluded_share
              << " h_search_mm=" << diag.search_layer_height_mm()
              << std::endl;

    // Below the search floor the search slices coarser than the print does, so the winner has to be
    // shown against the same sweep run at the print's own height.
    if (print_h < k.h_search_min_mm) {
        AutoTilt::Constants k_print = k;
        k_print.h_search_min_mm     = print_h;
        AutoTilt::ContactScorer at_print(obj, config, k_print, inline_runner());
        const Column           col_print = sweep(at_print, poses, k_print);
        std::cout << "model " << index << " " << stem
                  << " winner_at_h_search=" << pose_text(poses[col_plain.chosen])
                  << " winner_at_print_h=" << pose_text(poses[col_print.chosen])
                  << std::endl;
    }
}

} // namespace

TEST_CASE("The auto-tilt corpus stem keeps a one-object file's name and sanitises a multi-object one", "[AutoTilt]")
{
    CHECK(corpus_stem("/c/Ratmen_Test.3mf", 1, "anything at all") == "Ratmen_Test");
    CHECK(corpus_stem("/c/Ratmen_Test.3mf", 3, "10_Dark Elves 1.stl") == "Ratmen_Test#10_Dark_Elves_1.stl");
}

TEST_CASE("The corpus false-move predicate compares the selected pose with the root", "[AutoTilt]")
{
    // What the verified search ranks on: the four removal-damage fields and then the material. Index
    // 0 is the root pose, index 1 the pose the search selected, and lower is better throughout.
    const AutoTilt::Objectives root{ { 0, 0, 4., 10., true }, 100. };
    const std::vector<char>    measured{ 1, 1 };

    // A pose the print measures as carrying more removal risk than the root: the harm the harness
    // counts, whatever a third pose nobody selected would have scored.
    CHECK(SupportValidation::is_false_move({ root, AutoTilt::Objectives{ { 0, 0, 4., 11., true }, 100. } }, measured, 1));
    // The material alone can make it one, once every damage field ties.
    CHECK(SupportValidation::is_false_move({ root, AutoTilt::Objectives{ { 0, 0, 4., 10., true }, 120. } }, measured, 1));
    // A contact nobody could answer for is worse whatever the pose saved after it.
    CHECK(SupportValidation::is_false_move({ root, AutoTilt::Objectives{ { 1, 0, 0.5, 1., true }, 10. } }, measured, 1));

    // Better, and tied, are both moves the print does not regret.
    CHECK_FALSE(SupportValidation::is_false_move({ root, AutoTilt::Objectives{ { 0, 0, 2., 5., true }, 100. } }, measured, 1));
    CHECK_FALSE(SupportValidation::is_false_move({ root, AutoTilt::Objectives{ { 0, 0, 4., 10., true }, 100. } }, measured, 1));

    // A pose the evaluation refused leaves nothing to compare at either end, so no false move is
    // claimed rather than one guessed at.
    CHECK_FALSE(SupportValidation::is_false_move({ root, AutoTilt::Objectives{ { 0, 0, 4., 11., true }, 100. } }, { 0, 1 }, 1));
    CHECK_FALSE(SupportValidation::is_false_move({ root, AutoTilt::Objectives{ { 0, 0, 4., 11., true }, 100. } }, { 1, 0 }, 1));
}

TEST_CASE("Repeated generations decide an improvement by envelope and write one row each", "[AutoTilt]")
{
    // Legacy generation is not run-to-run reproducible (AGENTS.md "Testing"), so one pose is seven
    // readings and what it is worth is the spread they covered, never the reading that came last.
    const SupportValidation::Envelope root = SupportValidation::envelope_of({ 10.4, 10.0, 10.2, 10.1, 10.3, 10.2, 10.1 });
    CHECK_THAT(root.min, WithinAbs(10.0, 1e-12));
    CHECK_THAT(root.max, WithinAbs(10.4, 1e-12));

    SECTION("a candidate whose worst reading clears the root's best by the gain is an improvement") {
        const SupportValidation::Envelope candidate =
            SupportValidation::envelope_of({ 9.4, 9.0, 9.2, 9.1, 9.3, 9.2, 9.1 });
        // The spread the two legs left between them is 10.0 - 9.4 = 0.6 mm3; a gain inside it is
        // cleared and a gain wider than it is not.
        CHECK(SupportValidation::envelope_clears(root, candidate, 0.5));
        CHECK_FALSE(SupportValidation::envelope_clears(root, candidate, 0.7));
    }
    SECTION("overlapping envelopes are inconclusive, not a verified improvement") {
        const SupportValidation::Envelope candidate =
            SupportValidation::envelope_of({ 10.2, 9.8, 10.0, 9.9, 10.1, 10.0, 9.9 });
        CHECK_FALSE(SupportValidation::envelope_clears(root, candidate, 0.5));
        CHECK_FALSE(SupportValidation::envelope_clears(root, candidate, 0.0));
    }
    SECTION("a worse candidate is never an improvement, whatever gain is asked for") {
        const SupportValidation::Envelope candidate =
            SupportValidation::envelope_of({ 12.0, 11.5, 11.8, 11.6, 11.9, 11.7, 11.6 });
        CHECK_FALSE(SupportValidation::envelope_clears(root, candidate, 0.5));
    }
    SECTION("a gain nobody configured decides nothing") {
        const SupportValidation::Envelope candidate =
            SupportValidation::envelope_of({ 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 });
        CHECK_FALSE(SupportValidation::envelope_clears(root, candidate, 0.));
        CHECK_FALSE(SupportValidation::envelope_clears(root, candidate, -1.));
    }

    SECTION("one repeat writes one row, and the row survives being read back") {
        // A hostile row: a sub-microsecond runtime, a case id carrying non-ASCII and a slash, a
        // 64-bit memory reading and a reason code nothing escaped by hand would survive.
        SupportValidation::CaseResult result;
        result.harness            = "auto_tilt";
        result.case_id            = u8"\u5c0f\u3055\u306a\u30df\u30cb \u2014 spear/\u00e9p\u00e9e";
        result.style              = "tree_slim";
        result.feature_mode       = "on";
        result.pose               = AutoTilt::Pose{ -12., 4. };
        result.repeat             = 6;
        result.status             = SupportValidation::Outcome::Complete;
        result.measured           = true;
        result.reason_codes       = { u8"Missing\u00c4nchor", "VolumeReduced" };
        result.elapsed_s          = 0.000123456789;
        result.peak_memory_bytes  = 9007199254740993LL;
        result.source_sha256      = std::string(64, 'a');
        result.config_digest      = std::string(64, 'b');
        result.build_revision     = "deadbeef";
        result.metrics.support_volume_mm3 = 123.5;
        result.metrics.coverage_available = true;

        std::ostringstream os;
        SupportValidation::write_result(result, os);
        const std::string line = os.str();
        // One JSON object per line: exactly one newline, at the end.
        REQUIRE(line.find('\n') == line.size() - 1);

        const nlohmann::json back = nlohmann::json::parse(line);
        CHECK(back.at("row_type").get<std::string>() == "pose");
        CHECK(back.at("harness").get<std::string>() == "auto_tilt");
        CHECK(back.at("case_id").get<std::string>() == result.case_id);
        CHECK(back.at("style").get<std::string>() == "tree_slim");
        CHECK(back.at("feature_mode").get<std::string>() == "on");
        CHECK_THAT(back.at("pose").at("tilt_deg").get<double>(), WithinAbs(-12., 0.));
        CHECK_THAT(back.at("pose").at("lean_deg").get<double>(), WithinAbs(4., 0.));
        CHECK(back.at("repeat").get<size_t>() == 6);
        CHECK(back.at("status").get<std::string>() == "complete");
        CHECK(back.at("measured").get<bool>());
        CHECK(back.at("reason_codes").at(0).get<std::string>() == result.reason_codes[0]);
        CHECK_THAT(back.at("elapsed_s").get<double>(), WithinAbs(0.000123456789, 0.));
        CHECK(back.at("peak_memory_available").get<bool>());
        CHECK(back.at("peak_memory_bytes").get<long long>() == 9007199254740993LL);
        CHECK(back.at("source_sha256").get<std::string>() == result.source_sha256);
        CHECK(back.at("config_digest").get<std::string>() == result.config_digest);
        CHECK(back.at("build_revision").get<std::string>() == "deadbeef");
        CHECK_THAT(back.at("metrics").at("support_volume_mm3").get<double>(), WithinAbs(123.5, 0.));
        CHECK(back.at("metrics").at("coverage_available").get<bool>());

        // An unmeasured peak is an explicit marker and a null, never a zero reading.
        result.peak_memory_bytes = -1;
        std::ostringstream unavailable;
        SupportValidation::write_result(result, unavailable);
        const nlohmann::json marked = nlohmann::json::parse(unavailable.str());
        CHECK_FALSE(marked.at("peak_memory_available").get<bool>());
        CHECK(marked.at("peak_memory_bytes").is_null());
    }
}

TEST_CASE("A manifest case naming the Organic style is measured as an estimate, never as a generated result", "[AutoTilt]")
{
    // A case's style string goes straight into set_deserialize_strict (case_object), so it has to be
    // a value s_keys_map_SupportMaterialStyle carries. A spelling the enum does not know throws
    // there, and the row that comes back says the slice failed rather than what the style measures.
    ScopedTemporaryDir root("orca-manifest");
    const std::string  model = (root.path() / "case.stl").string();
    const TriangleMesh mesh  = cube(8.);
    REQUIRE(its_write_stl_binary(model.c_str(), "case", mesh.its));

    SupportValidation::Manifest manifest;
    manifest.version    = 1;
    manifest.model_root = root.string();
    SupportValidation::ManifestCase entry;
    entry.id            = "organic_estimate_case";
    entry.model         = "case.stl";
    entry.selectors     = { "index:0" };
    entry.repeats       = 1;
    entry.sha256        = std::string(64, 'a');
    entry.config_digest = std::string(64, 'b');

    const SupportValidation::CaseResult organic =
        SupportValidation::measure_case(manifest, entry, fixture_config(), "organic", "off", 0);
    CHECK(organic.measured);
    CHECK(organic.status == SupportValidation::Outcome::OrganicEstimate);
    CHECK(organic.estimate_only);
    CHECK_FALSE(organic.verified);

    // The same case under a legacy style is a generated result the print measured, so the estimate
    // branch is the style's doing and not something every row gets.
    const SupportValidation::CaseResult legacy =
        SupportValidation::measure_case(manifest, entry, fixture_config(), "tree_slim", "off", 0);
    CHECK(legacy.measured);
    CHECK(legacy.status != SupportValidation::Outcome::OrganicEstimate);
    CHECK(legacy.status != SupportValidation::Outcome::Unknown);
    CHECK_FALSE(legacy.estimate_only);
}

TEST_CASE("The exhaustive sweep measures a selected pose against the best pose in the grid", "[AutoTilt]")
{
    // What the exhaustive mode visits: the unchanged grid, whole. Nothing here trims it, and the
    // harness records Invalid for the entries exact plate containment refuses rather than dropping
    // them.
    CHECK(AutoTilt::grid(AutoTilt::Constants()).size() == 77);

    // The best pose in the grid, and the pose the production search settled on. Lower is better on
    // every field; the two counts are the discrete safety classification and the three doubles are
    // the continuous objectives, compared in that order.
    const AutoTilt::Objectives best{ { 0, 0, 4., 10., true }, 100. };

    SECTION("regret is the first continuous objective that differs, relative to the best") {
        // The worst group differs first, so nothing after it is read.
        CHECK_THAT(SupportValidation::regret_of(AutoTilt::Objectives{ { 0, 0, 4.4, 50., true }, 500. }, best),
                   WithinRel(0.10, 1e-12));
        // Tied there, the sum of the groups decides.
        CHECK_THAT(SupportValidation::regret_of(AutoTilt::Objectives{ { 0, 0, 4., 11., true }, 500. }, best),
                   WithinRel(0.10, 1e-12));
        // Tied on both, the material decides.
        CHECK_THAT(SupportValidation::regret_of(AutoTilt::Objectives{ { 0, 0, 4., 10., true }, 105. }, best),
                   WithinRel(0.05, 1e-12));
    }
    SECTION("a pose that ties the best on every continuous objective regrets nothing") {
        CHECK_THAT(SupportValidation::regret_of(best, best), WithinAbs(0., 1e-12));
    }
    SECTION("a best of zero is divided by the floor rather than by nothing") {
        const AutoTilt::Objectives zero{ { 0, 0, 0., 0., true }, 0. };
        CHECK(SupportValidation::regret_of(AutoTilt::Objectives{ { 0, 0, 1e-9, 0., true }, 0. }, zero) > 0.9);
    }
    SECTION("an unmeasured end measures no regret") {
        const AutoTilt::Objectives unavailable{ { 0, 0, 4., 10., false }, 100. };
        CHECK(std::isnan(SupportValidation::regret_of(unavailable, best)));
        CHECK(std::isnan(SupportValidation::regret_of(best, unavailable)));
    }
    SECTION("a worse discrete classification is reported whatever the scalar regret says") {
        // Fewer mm3 than the best pose, and a contact nobody could answer for: the classification is
        // worse, and the material it saved does not buy that back.
        const AutoTilt::Objectives cheaper_but_unknown{ { 1, 0, 1., 1., true }, 10. };
        CHECK(SupportValidation::discrete_worse(cheaper_but_unknown, best));
        CHECK(SupportValidation::discrete_worse(AutoTilt::Objectives{ { 0, 1, 4., 10., true }, 100. }, best));
        CHECK_FALSE(SupportValidation::discrete_worse(AutoTilt::Objectives{ { 0, 0, 40., 100., true }, 1000. }, best));
        // An unmeasured end classifies nothing, so it is not a worse classification either.
        CHECK_FALSE(SupportValidation::discrete_worse(AutoTilt::Objectives{ { 1, 0, 1., 1., false }, 10. }, best));
    }
}

// Hidden ([.]): with a manifest one case costs 77 poses times its repeats in full Print::process()
// passes, so hours of wall clock, and the corpus it reads lives outside the repo under
// $ORCA_AUTOTILT_CORPUS (tests/AGENTS.md). Without a manifest it is the cheap-scorer diagnostic over
// the built-in fin, which scripts/validate_miniature_supports.py accepts as no acceptance run at all.
TEST_CASE("Auto-tilt validation harness over a corpus", "[AutoTilt][.]")
{
    // $ORCA_AUTOTILT_COARSE trades the 77-pose grid for the 9-pose one, presence-tested the way
    // test_multifilament.cpp's ORCA_UPDATE_WIPE_TOWER_TEMP_TRACE check does. Thresholds keep their
    // production defaults.
    const AutoTilt::Constants k = std::getenv("ORCA_AUTOTILT_COARSE") != nullptr ? coarse_constants() : AutoTilt::Constants();

    SupportValidation::use_os_temporary_dir();

    const char       *env        = std::getenv("ORCA_AUTOTILT_CORPUS");
    const std::string corpus_dir = env != nullptr ? std::string(env) : std::string();

    // With a manifest the harness runs the exhaustive validation mode: every entry of the grid
    // above, every declared repeat, one row each, and one selection row per case, style and feature
    // mode. Without one it stays the cheap-scorer diagnostic below, which the acceptance command
    // rejects because it satisfies no manifest.
    const char *manifest_env = std::getenv("ORCA_MINIATURE_MANIFEST");
    if (manifest_env != nullptr && *manifest_env != '\0') {
        const SupportValidation::Manifest manifest = SupportValidation::load_manifest(manifest_env);
        const char                       *results  = std::getenv("ORCA_AUTOTILT_RESULTS");
        REQUIRE(results != nullptr);
        std::ofstream rows(results);
        REQUIRE(rows.good());
        for (const SupportValidation::ManifestCase &entry : manifest.cases)
            for (const std::string &style : entry.styles)
                for (const std::string &mode : entry.feature_modes)
                    run_exhaustive_case(manifest, entry, style, mode, k, rows);
        rows.close();
        REQUIRE(rows.good());
        return;
    }

    // Model 0 is always the built-in fin fixture, under a legacy tree style: the evaluator refuses
    // Organic, which the default style resolves to, so the diagnostic would measure nothing there.
    const Model model0 = Slic3r::Test::model("fin_fixture", fin_fixture());
    run_harness_model(0, "fin_fixture", model0, fixture_config({ { "support_style", "tree_slim" } }), corpus_dir, k);

    if (corpus_dir.empty()) {
        std::cout << "corpus dir not set, model 0 only" << std::endl;
        return;
    }

    // The scorers cache m_root_box = instance_bounding_box(0) in their constructor
    // (ContactScorer's constructor), so each corpus object arrives centred on the bed and dropped
    // onto it, or every root-height fraction would be measured off a shifted box.
    SupportValidation::for_each_corpus_object(corpus_dir, fixture_config(), 1,
        [&](size_t index, const SupportValidation::CorpusObject &object) {
            run_harness_model(index, object.stem, object.model, object.config, corpus_dir, k);
        });
}

TEST_CASE("The affected instances resolve one support generator or report a mixed operation", "[AutoTilt]")
{
    Slic3r::Model model = Slic3r::Test::model("table", table_fixture());
    model.objects.front()->ensure_on_bed();
    const std::vector<ObjectID> affected{ model.objects.front()->instances.front()->id() };

    SECTION("an unset style resolves to Organic, which is what SupportParameters makes of it") {
        // Reading support_style off the config would have found smsDefault and called it legacy.
        const AutoTilt::EvaluationInput input = plate_input(model, fixture_config(), affected);
        REQUIRE(AutoTilt::affected_support_generator(input) == AutoTilt::SupportGenerator::Organic);
    }
    SECTION("a legacy tree style resolves to Legacy") {
        const AutoTilt::EvaluationInput input = plate_input(model, fixture_config({ { "support_style", "tree_slim" } }), affected);
        REQUIRE(AutoTilt::affected_support_generator(input) == AutoTilt::SupportGenerator::Legacy);
    }
    SECTION("an object override outranks the plate config") {
        Slic3r::Model overridden = model;
        overridden.objects.front()->config.set_key_value("support_style", new ConfigOptionEnum<SupportMaterialStyle>(smsTreeOrganic));
        const AutoTilt::EvaluationInput input =
            plate_input(overridden, fixture_config({ { "support_style", "tree_slim" } }), { overridden.objects.front()->instances.front()->id() });
        REQUIRE(AutoTilt::affected_support_generator(input) == AutoTilt::SupportGenerator::Organic);
    }
    SECTION("instances that disagree across plates are a mixed operation") {
        // One rotation moves every instance of the object at once, so a verified legacy answer cannot
        // be claimed for the plate whose copy runs Organic, and rotating one plate is not on offer.
        AutoTilt::EvaluationInput input = plate_input(model, fixture_config({ { "support_style", "tree_slim" } }), affected);
        input.plates.push_back(input.plates.front());
        input.plates.back().plate_index = 1;
        input.plates.back().full_config = fixture_config();
        REQUIRE(AutoTilt::affected_support_generator(input) == AutoTilt::SupportGenerator::Mixed);
    }
    SECTION("supports switched off resolve to Unknown") {
        const AutoTilt::EvaluationInput input = plate_input(model, fixture_config({ { "enable_support", "0" } }), affected);
        REQUIRE(AutoTilt::affected_support_generator(input) == AutoTilt::SupportGenerator::Unknown);
    }
    SECTION("a normal support type resolves to Unknown, since no tree generator runs") {
        const AutoTilt::EvaluationInput input =
            plate_input(model, fixture_config({ { "support_type", "normal(auto)" } }), affected);
        REQUIRE(AutoTilt::affected_support_generator(input) == AutoTilt::SupportGenerator::Unknown);
    }
}
