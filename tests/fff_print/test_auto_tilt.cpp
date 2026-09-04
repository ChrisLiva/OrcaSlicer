#include <catch2/catch_all.hpp>

#include "libslic3r/AutoTilt.hpp"
#include "libslic3r/AutoTiltScorer.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Support/TreeSupport.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/Utils.hpp"

#include <tbb/global_control.h>

#include <algorithm>
#include <cctype>
#include <chrono>
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
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
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

TEST_CASE("A processed tree-support print exposes its roof areas through SupportLayer", "[AutoTilt]")
{
    // support_style stays at its default in fixture_config(), and SupportParameters resolves that to
    // organic, whose generator never fills roof_areas. Only the legacy tree generator's draw_circles()
    // writes them, so the accessor seam has to be walked under a legacy style.
    Slic3r::Print print;
    Slic3r::Test::init_and_process_print({ table_fixture() }, print, fixture_config({ { "support_style", "tree_slim" } }));

    size_t layers_with_roof = 0, layers_with_roof_1st = 0;
    for (const SupportLayer *sl : print.objects().front()->support_layers()) {
        if (! sl->tree_roof_areas().empty())
            ++ layers_with_roof;
        if (! sl->tree_roof_1st_layer().empty())
            ++ layers_with_roof_1st;
    }

    REQUIRE(layers_with_roof > 0);
    REQUIRE(layers_with_roof_1st > 0);

    // The fin's root contact is one sharp-tail cluster from end to end: draw_circles() routes those
    // nodes past roof_areas entirely, but the 0.2 mm support_top_z_distance still buys a gap layer,
    // so the cluster does land in roof_gap_areas. That is the routing a roof-only truth cannot see.
    Slic3r::Print fin_print;
    Slic3r::Test::init_and_process_print({ fin_fixture() }, fin_print, fixture_config({ { "support_style", "tree_slim" } }));

    const PrintObject *fin_po      = fin_print.objects().front();
    const double       fin_first_z = fin_po->slicing_parameters().first_print_layer_height;

    size_t layers_with_roof_gap = 0;
    for (const SupportLayer *sl : fin_po->support_layers())
        if (sl->print_z > fin_first_z + EPSILON && ! sl->tree_roof_gap_areas().empty())
            ++ layers_with_roof_gap;

    REQUIRE(layers_with_roof_gap > 0);
}

// ---------------------------------------------------------------------------------------------
// Validation harness (hidden, see the TEST_CASE at the bottom of this file).
// ---------------------------------------------------------------------------------------------

namespace {

constexpr double harness_nan = std::numeric_limits<double>::quiet_NaN();

// Ground truth for one pose: the tree generator's contact-tip area (`roof_gap_areas`, every tip
// node sharp-tail or not) a full Print::process() lays against the object, at the model's own print
// layer height. `ok` is false when process() refused the pose.
struct TruthValue
{
    double weighted   = harness_nan;
    double unweighted = harness_nan;
    bool   ok         = false;
};

TruthValue ground_truth(const Model &base, const DynamicPrintConfig &config, const AutoTilt::Pose &pose, const AutoTilt::Constants &k)
{
    Model        posed_model = base;
    ModelObject *obj         = posed_model.objects.front();
    const Transform3d   root     = obj->instances.front()->get_transformation().get_matrix();
    const BoundingBoxf3 root_box = obj->instance_bounding_box(0);
    const Vec3d         pivot    = root_box.center();
    obj->instances.front()->set_transformation(Geometry::Transformation(AutoTilt::candidate_transform(root, pivot, pose)));
    obj->invalidate_bounding_box();
    obj->ensure_on_bed();
    const Transform3d posed = obj->instances.front()->get_transformation().get_matrix();

    TruthValue    t;
    Slic3r::Print print;
    print.set_status_silent();
    try {
        print.apply(posed_model, config);
        print.process();
    } catch (const std::exception &) {
        // Print::validate rejects some poses outright (exclusion areas, off-bed); the plan says to
        // skip the pose and keep the model rather than losing the whole run.
        return t;
    }

    const PrintObject *po      = print.objects().front();
    const double       first_z = po->slicing_parameters().first_print_layer_height;

    // Serial, ascending layer, ascending polygon: two poses must sum the same floats in the same order.
    double weighted = 0., unweighted = 0.;
    for (const SupportLayer *sl : po->support_layers()) {
        if (sl->print_z <= first_z + EPSILON)
            continue; // plate contact, not object contact
        for (const ExPolygon &p : sl->tree_roof_gap_areas()) {
            // The same exclusion plane the scorer applies, read off the `k` the harness was handed.
            // A zero-area polygon adds 0 either way, and centroid() returns an integer Point, so the
            // fraction is finite, as in the scorer.
            const Vec3d centroid_world =
                to_3d(unscale(p.contour.centroid()) + unscale(po->instances().front().shift), sl->print_z);
            if (AutoTilt::root_height_fraction(root, posed, root_box, centroid_world) < k.bottom_exclusion_fraction)
                continue;
            const double a = p.area() * SCALING_FACTOR * SCALING_FACTOR;
            double       perimeter_scaled = p.contour.length();
            for (const Polygon &hole : p.holes)
                perimeter_scaled += hole.length();
            const double perimeter = unscale<double>(perimeter_scaled);
            unweighted += a * k.h_ref_mm;
            weighted += AutoTilt::fragility_weight(a, perimeter, false, k) * a * k.h_ref_mm;
        }
    }
    t.weighted   = weighted;
    t.unweighted = unweighted;
    t.ok         = true;
    return t;
}

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

// Ranks ascending, ties sharing their average rank.
std::vector<double> average_ranks(const std::vector<double> &v)
{
    std::vector<size_t> idx(v.size());
    std::iota(idx.begin(), idx.end(), size_t(0));
    std::stable_sort(idx.begin(), idx.end(), [&v](size_t a, size_t b) { return v[a] < v[b]; });
    std::vector<double> ranks(v.size(), 0.);
    size_t              i = 0;
    while (i < idx.size()) {
        size_t j = i;
        while (j + 1 < idx.size() && v[idx[j + 1]] == v[idx[i]])
            ++ j;
        const double avg = 0.5 * (double(i) + double(j)) + 1.;
        for (size_t t = i; t <= j; ++ t)
            ranks[idx[t]] = avg;
        i = j + 1;
    }
    return ranks;
}

// Spearman rho: Pearson over average ranks, which is the tie-correct form of the standard formula.
// Zero variance on either side (every pose tied) leaves rho undefined; report 0, not nan.
double spearman(const std::vector<double> &x, const std::vector<double> &y)
{
    if (x.size() < 2 || x.size() != y.size())
        return 0.;
    const std::vector<double> rx = average_ranks(x), ry = average_ranks(y);
    const double              n  = double(rx.size());
    const double mx = std::accumulate(rx.begin(), rx.end(), 0.) / n, my = std::accumulate(ry.begin(), ry.end(), 0.) / n;
    double num = 0., dx = 0., dy = 0.;
    for (size_t i = 0; i < rx.size(); ++ i) {
        const double a = rx[i] - mx, b = ry[i] - my;
        num += a * b;
        dx += a * a;
        dy += b * b;
    }
    if (dx <= 0. || dy <= 0.)
        return 0.;
    return std::max(-1., std::min(1., num / std::sqrt(dx * dy)));
}

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

// (truth[chosen] - truth_min) / truth_min, over the poses whose ground truth landed. When the best
// pose leaves no contact at all, any contact the search settled for is infinitely worse in relative
// terms, so say so rather than scaling the gap by an arbitrary epsilon. nan means unmeasured.
double regret_of(const std::vector<double> &truth, const std::vector<char> &truth_ok, size_t chosen)
{
    double truth_min = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < truth.size(); ++ i)
        if (truth_ok[i])
            truth_min = std::min(truth_min, truth[i]);
    if (! truth_ok[chosen] || ! std::isfinite(truth_min))
        return harness_nan;
    if (truth_min > 0)
        return (truth[chosen] - truth_min) / truth_min;
    return truth[chosen] == 0 ? 0. : std::numeric_limits<double>::infinity();
}

std::string pose_text(const AutoTilt::Pose &p)
{
    std::ostringstream os;
    os << p.tilt_deg << "/" << p.lean_deg;
    return os.str();
}

// Scores, ground-truths and reports one model. Writes `<stem>.autotilt.csv` beside the corpus when
// `corpus_dir` is set; prints the summary line either way.
void run_harness_model(size_t index, const std::string &stem, const Model &base, const DynamicPrintConfig &config,
                       const std::string &corpus_dir, const AutoTilt::Constants &k)
{
    const auto                        start = std::chrono::high_resolution_clock::now();
    const std::vector<AutoTilt::Pose> poses = AutoTilt::grid(k);
    const size_t                      n     = poses.size();
    const ModelObject                &obj   = *base.objects.front();

    AutoTilt::ContactScorer plain(obj, config, k, inline_runner());
    AutoTilt::ContactScorer shadow(obj, config, k, inline_runner());
    shadow.shadow_correction = true;

    const Column col_plain  = sweep(plain, poses, k);
    const Column col_shadow = sweep(shadow, poses, k);

    // §14's diagnostics describe the root pose alone, off a scorer of their own: score() accumulates
    // sharp-tail and cantilever state across calls, so a swept scorer would fold nine poses of it into
    // the root's rows. shadow_correction stays false, where the recorded and the charged sets coincide.
    AutoTilt::ContactScorer                            diag(obj, config, k, inline_runner());
    std::vector<AutoTilt::ContactScorer::PolygonRecord> records;
    diag.records                   = &records;
    const AutoTilt::Contact root_c = diag.score(AutoTilt::Pose{});

    // roof_gap_areas is the gap the generator leaves between a contact tip and the object, so a
    // config that prints the tips straight onto the object files none of it: there is no truth to
    // measure, and 77 Print::process() passes would only produce zeros. Score, report, say so.
    const bool truth_available = config.opt_float("support_top_z_distance") > 0.;

    std::vector<double> truth_w(n, harness_nan), truth_u(n, harness_nan);
    std::vector<char>   truth_ok(n, 0);
    size_t              measured = 0;
    if (truth_available)
        for (size_t i = 0; i < n; ++ i) {
            const TruthValue t = ground_truth(base, config, poses[i], k);
            truth_w[i]         = t.weighted;
            truth_u[i]         = t.unweighted;
            truth_ok[i]        = t.ok ? 1 : 0;
            if (t.ok)
                ++ measured;
            else
                std::cout << "model " << index << " " << stem << " pose " << pose_text(poses[i]) << " skipped" << std::endl;
        }

    if (! corpus_dir.empty()) {
        const std::filesystem::path csv_path = std::filesystem::path(corpus_dir) / (stem + ".autotilt.csv");
        std::ofstream               csv(csv_path.string());
        csv << std::setprecision(12);
        csv << "tilt,lean,score_mm3,volume_mm3,shadow_corrected_score_mm3,truth_weighted_mm3,truth_unweighted_mm3\n";
        for (size_t i = 0; i < n; ++ i)
            csv << poses[i].tilt_deg << "," << poses[i].lean_deg << ","
                << (col_plain.ok[i] ? col_plain.contacts[i].score_mm3 : harness_nan) << ","
                << (col_plain.ok[i] ? col_plain.contacts[i].volume_mm3 : harness_nan) << ","
                << (col_shadow.ok[i] ? col_shadow.contacts[i].score_mm3 : harness_nan) << ","
                << truth_w[i] << "," << truth_u[i] << "\n";
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

    // Spearman and the truth extremes only see the poses both sides could measure.
    std::vector<double> sc_plain, sc_shadow, tr_plain, tr_shadow;
    double              truth_min = std::numeric_limits<double>::infinity(), truth_max = 0.;
    for (size_t i = 0; i < n; ++ i) {
        if (! truth_ok[i])
            continue;
        truth_min = std::min(truth_min, truth_w[i]);
        truth_max = std::max(truth_max, truth_w[i]);
        if (col_plain.ok[i]) {
            sc_plain.push_back(col_plain.contacts[i].score_mm3);
            tr_plain.push_back(truth_w[i]);
        }
        if (col_shadow.ok[i]) {
            sc_shadow.push_back(col_shadow.contacts[i].score_mm3);
            tr_shadow.push_back(truth_w[i]);
        }
    }

    const double regret_plain  = truth_available ? regret_of(truth_w, truth_ok, col_plain.chosen) : harness_nan;
    const double regret_shadow = truth_available ? regret_of(truth_w, truth_ok, col_shadow.chosen) : harness_nan;
    const double rho_plain     = truth_available ? spearman(sc_plain, tr_plain) : harness_nan;
    const double rho_shadow    = truth_available ? spearman(sc_shadow, tr_shadow) : harness_nan;
    // The root is pose 0 of the grid. A false move is the search leaving a root that was already best.
    const bool false_move = truth_ok[0] && truth_w[0] <= truth_min &&
                            (col_plain.result.outcome == AutoTilt::SearchResult::Outcome::Improved);

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
              << " regret_plain=" << regret_plain
              << " regret_shadow=" << regret_shadow
              << " spearman_plain=" << rho_plain
              << " spearman_shadow=" << rho_shadow
              << " false_move=" << (false_move ? 1 : 0)
              << " truth_min_mm3=" << truth_min
              << " truth_max_mm3=" << truth_max
              << " poses_with_truth=" << measured << "/" << n
              << " chosen_plain=" << pose_text(poses[col_plain.chosen])
              << " chosen_shadow=" << pose_text(poses[col_shadow.chosen])
              << " facets=" << obj.facets_count()
              << " bbox_mm=" << bbox_text
              << " layer_height_mm=" << print_h
              << " sweep_s=" << sweep_text
              << " elapsed_s=" << elapsed_text
              << (truth_available ? "" : " truth_unavailable=support_top_z_distance_0")
              << std::endl;

    // §14's shape diagnostics, over the root pose's rows. `R` is what the exclusion plane keeps; the
    // percentiles read off the finite thicknesses in it (t is NaN for a degenerate zero-perimeter
    // contour), and `excluded_share` weighs what the plane drops against every row, kept or not.
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

    if (truth_available)
        REQUIRE(measured > 0);
    if (truth_ok[col_plain.chosen]) {
        REQUIRE(! std::isnan(regret_plain));
        REQUIRE(regret_plain >= 0.);
    }
    if (truth_ok[col_shadow.chosen]) {
        REQUIRE(! std::isnan(regret_shadow));
        REQUIRE(regret_shadow >= 0.);
    }
    if (truth_available) {
        REQUIRE(rho_plain >= -1.);
        REQUIRE(rho_plain <= 1.);
        REQUIRE(rho_shadow >= -1.);
        REQUIRE(rho_shadow <= 1.);
    }

    // Below the search floor the search slices coarser than the print does, so the winner has to be
    // shown to survive dropping the search height onto the print height.
    if (print_h < k.h_search_min_mm) {
        AutoTilt::Constants k_print = k;
        k_print.h_search_min_mm     = print_h;
        AutoTilt::ContactScorer at_print(obj, config, k_print, inline_runner());
        const Column           col_print = sweep(at_print, poses, k_print);
        const double           r_print   = regret_of(truth_w, truth_ok, col_print.chosen);
        std::cout << "model " << index << " " << stem
                  << " winner_at_h_search=" << pose_text(poses[col_plain.chosen])
                  << " winner_at_print_h=" << pose_text(poses[col_print.chosen])
                  << " both_within_regret=" << ((std::isfinite(regret_plain) && regret_plain <= 0.10 &&
                                                 std::isfinite(r_print) && r_print <= 0.10) ? 1 : 0)
                  << std::endl;
    }
}

// Every .stl and .3mf directly under `dir`, in name order, so two runs walk the same models in the
// same order. A corpus directory that has gone missing yields nothing rather than throwing.
std::vector<std::filesystem::path> corpus_files(const std::string &dir)
{
    std::vector<std::filesystem::path> out;
    std::error_code                    ec;
    for (std::filesystem::directory_iterator it(dir, ec), end; ! ec && it != end; it.increment(ec)) {
        if (! it->is_regular_file(ec))
            continue;
        std::string ext = it->path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return char(std::tolower(c)); });
        if (ext == ".stl" || ext == ".3mf")
            out.push_back(it->path());
    }
    std::sort(out.begin(), out.end());
    return out;
}

// The name a corpus object's CSV and log lines carry. Slic3r::sanitize_filename (Utils.hpp:279)
// replaces only /\:*?"<>| and would leave the spaces of an object name like `10_Dark Elves 1.stl`
// in a path that gets typed on a command line, so this keeps a tighter character class.
std::string corpus_stem(const std::filesystem::path &path, size_t object_count, const std::string &object_name)
{
    const std::string file = path.stem().string();
    if (object_count == 1)
        return file;
    std::string s = object_name;
    for (char &c : s) {
        const bool keep = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') ||
                          c == '.' || c == '_' || c == '-';
        if (! keep)
            c = '_';
    }
    return file + "#" + s;
}

// The config a corpus model is both scored and ground-truthed under: whatever the file carries, over
// the fixture's defaults (an .stl carries nothing), with only what ground truth needs forced back in.
// Ground truth reads roof_gap_areas off the tree generator, so a file that turned supports off,
// picked a normal support type, or left the organic style (which never fills roof_gap_areas) would
// make both the truth and the scorer return nothing; each of those three is forced only when the
// loaded value cannot produce roof areas, so a legacy tree style the file already chose is left alone.
DynamicPrintConfig corpus_config(const DynamicPrintConfig &loaded)
{
    DynamicPrintConfig config = fixture_config();
    if (! loaded.keys().empty())
        config.apply(loaded);
    const auto *enable = config.option<ConfigOptionBool>("enable_support");
    if (enable == nullptr || ! enable->value)
        config.set_deserialize_strict({ { "enable_support", "1" } });
    const auto *type = config.option<ConfigOptionEnum<SupportType>>("support_type");
    if (type == nullptr || ! is_tree(type->value))
        config.set_deserialize_strict({ { "support_type", "tree(auto)" } });
    const auto *style = config.option<ConfigOptionEnum<SupportMaterialStyle>>("support_style");
    if (style == nullptr || style->value == smsDefault || style->value == smsTreeOrganic)
        config.set_deserialize_strict({ { "support_style", "tree_slim" } });
    return config;
}

} // namespace

TEST_CASE("The auto-tilt corpus stem keeps a one-object file's name and sanitises a multi-object one", "[AutoTilt]")
{
    CHECK(corpus_stem("/c/Ratmen_Test.3mf", 1, "anything at all") == "Ratmen_Test");
    CHECK(corpus_stem("/c/Ratmen_Test.3mf", 3, "10_Dark Elves 1.stl") == "Ratmen_Test#10_Dark_Elves_1.stl");
}

// Hidden ([.]): one model costs 77 full Print::process() passes plus 154 scorer slices, and a third
// 77-pose sweep on a model printing finer than 0.12 mm, so hours of wall clock; the corpus it reads
// lives outside the repo under $ORCA_AUTOTILT_CORPUS
// (tests/AGENTS.md:44). It measures the scorer's regret against ground truth; it does not pin it.
TEST_CASE("Auto-tilt validation harness over a corpus", "[AutoTilt][.]")
{
    // $ORCA_AUTOTILT_COARSE trades the 77-pose grid for the 9-pose one, presence-tested the way
    // tests/fff_print/test_multifilament.cpp:648 does. Thresholds keep their production defaults.
    const AutoTilt::Constants k = std::getenv("ORCA_AUTOTILT_COARSE") != nullptr ? coarse_constants() : AutoTilt::Constants();

    // Model::get_backup_path() builds from temporary_dir(), which is "" in a test process, so each
    // 3mf load logs two "Failed to create backup path /orcaslicer_model/...: Read-only file system"
    // errors that read like a failure but are caught and non-fatal. Point it at the OS temp dir the
    // way the app does at src/OrcaSlicer.cpp:1330.
    Slic3r::set_temporary_dir(std::filesystem::temp_directory_path().string());

    const char       *env        = std::getenv("ORCA_AUTOTILT_CORPUS");
    const std::string corpus_dir = env != nullptr ? std::string(env) : std::string();

    // Model 0 is always the built-in fin fixture, under a legacy tree style: the organic generator
    // never fills roof_gap_areas, so ground truth would be identically zero under the default style.
    const Model model0 = Slic3r::Test::model("fin_fixture", fin_fixture());
    run_harness_model(0, "fin_fixture", model0, fixture_config({ { "support_style", "tree_slim" } }), corpus_dir, k);

    if (corpus_dir.empty()) {
        std::cout << "corpus dir not set, model 0 only" << std::endl;
        return;
    }

    size_t index = 1;
    for (const std::filesystem::path &path : corpus_files(corpus_dir)) {
        const std::string      stem = path.stem().string();
        DynamicPrintConfig     loaded;
        std::unique_ptr<Model> model;
        try {
            // The 3mf importer creates no object without LoadModel and reads no config without
            // LoadConfig (Format/bbs_3mf.cpp:1419, :1422), so the default strategy hands back an
            // empty model; load the way the CLI does (src/OrcaSlicer.cpp:1648).
            model.reset(new Model(Model::read_from_file(path.string(), &loaded, nullptr,
                LoadStrategy::LoadModel | LoadStrategy::LoadConfig | LoadStrategy::AddDefaultInstances)));
        } catch (const std::exception &e) {
            std::cout << "model " << index << " " << stem << " skipped: " << e.what() << std::endl;
            ++ index;
            continue;
        }
        if (model->objects.empty()) {
            std::cout << "model " << index << " " << stem << " skipped: no printable instance" << std::endl;
            ++ index;
            continue;
        }
        const DynamicPrintConfig config = corpus_config(loaded);
        // One harness model per object: the scorer poses one object's one instance, so ground truth
        // has to print exactly that. The scorers cache m_root_box = instance_bounding_box(0) in their
        // constructor (AutoTiltScorer.cpp:24), so the instance is centred on the bed and dropped onto
        // it first, or every root-height fraction would be measured off a shifted box.
        for (const ModelObject *src : model->objects) {
            const std::string obj_stem = corpus_stem(path, model->objects.size(), src->name);
            if (src->instances.empty()) {
                std::cout << "model " << index << " " << obj_stem << " skipped: no printable instance" << std::endl;
                ++ index;
                continue;
            }
            Model        one;
            ModelObject *obj = one.add_object(*src);
            while (obj->instances.size() > 1)
                obj->delete_last_instance();
            one.center_instances_around_point(unscale(BoundingBox(get_bed_shape(config)).center()));
            obj->ensure_on_bed();

            run_harness_model(index, obj_stem, one, config, corpus_dir, k);
            ++ index;
        }
    }
}
