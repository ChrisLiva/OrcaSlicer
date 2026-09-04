#include <catch2/catch_all.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <utility>
#include <vector>

#include "libslic3r/AutoTilt.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Geometry.hpp"

using namespace Slic3r;
using namespace Slic3r::AutoTilt;

using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

TEST_CASE("grid lists the root pose first and then every tilt/lean pair tilt-major", "[AutoTilt]")
{
    const Constants         k;
    const std::vector<Pose> g = grid(k);

    REQUIRE(g.size() == 147);
    REQUIRE(g[0].is_root());
    REQUIRE(g[1] == Pose{0, -15});
    REQUIRE(g[7] == Pose{-2, -15});

    SECTION("every tilt/lean pair appears exactly once") {
        for (double tilt : k.tilts_deg) {
            for (double lean : k.leans_deg) {
                const Pose p{tilt, lean};
                const auto n = std::count_if(g.begin(), g.end(), [&p](const Pose &q) { return q == p; });
                REQUIRE(n == 1);
            }
        }
    }

    SECTION("a smaller Constants yields a smaller grid") {
        Constants small;
        small.tilts_deg = {0, -20, -40};
        small.leans_deg = {-15, 0, 15};
        REQUIRE(grid(small).size() == 9);
    }
}

TEST_CASE("candidate_transform tilts toward +Y and leans toward +X about the pivot", "[AutoTilt]")
{
    const Transform3d identity = Transform3d::Identity();
    const Vec3d       origin   = Vec3d::Zero();

    SECTION("a negative tilt lifts the far end of the +Z axis toward +Y") {
        const Vec3d p = candidate_transform(identity, origin, Pose{-30, 0}) * Vec3d(0, 0, 10);
        REQUIRE_THAT(p.y(), WithinAbs(5.0, 1e-9));
        REQUIRE_THAT(p.z(), WithinAbs(10.0 * std::cos(Geometry::deg2rad(-30.)), 1e-9));
    }
    SECTION("a positive lean swings the +Z axis toward +X") {
        const Vec3d p = candidate_transform(identity, origin, Pose{0, 30}) * Vec3d(0, 0, 10);
        REQUIRE_THAT(p.x(), WithinAbs(5.0, 1e-9));
    }
    SECTION("the pivot is a fixed point") {
        const Vec3d pivot = Vec3d(1, 2, 3);
        const Vec3d p     = candidate_transform(identity, pivot, Pose{-30, 15}) * pivot;
        REQUIRE_THAT(p.x(), WithinAbs(pivot.x(), 1e-12));
        REQUIRE_THAT(p.y(), WithinAbs(pivot.y(), 1e-12));
        REQUIRE_THAT(p.z(), WithinAbs(pivot.z(), 1e-12));
    }
    SECTION("a pure tilt leaves the X axis fixed, so no Z rotation creeps in") {
        const Vec3d p = candidate_transform(identity, origin, Pose{-30, 0}) * Vec3d(1, 0, 0);
        REQUIRE_THAT(p.x(), WithinAbs(1.0, 1e-12));
        REQUIRE_THAT(p.y(), WithinAbs(0.0, 1e-12));
        REQUIRE_THAT(p.z(), WithinAbs(0.0, 1e-12));
    }
    SECTION("a pure lean leaves the Y axis fixed, so no Z rotation creeps in") {
        const Vec3d p = candidate_transform(identity, origin, Pose{0, 30}) * Vec3d(0, 1, 0);
        REQUIRE_THAT(p.x(), WithinAbs(0.0, 1e-12));
        REQUIRE_THAT(p.y(), WithinAbs(1.0, 1e-12));
        REQUIRE_THAT(p.z(), WithinAbs(0.0, 1e-12));
    }
    SECTION("the root pose reproduces a mirrored, non-uniformly scaled instance matrix") {
        const Transform3d root = Geometry::translation_transform(Vec3d(7, 8, 9)) *
                                 Geometry::scale_transform(Vec3d(2, 0.5, 1)) *
                                 Geometry::scale_transform(Vec3d(-1, 1, 1));
        const Transform3d out = candidate_transform(root, Vec3d(1, 2, 3), Pose{0, 0});
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c)
                REQUIRE_THAT(out.matrix()(r, c), WithinAbs(root.matrix()(r, c), 1e-12));
    }
}

TEST_CASE("root_height_fraction reads a posed point's height in the root frame as a fraction of the root height", "[AutoTilt]")
{
    REQUIRE(Constants{}.bottom_exclusion_fraction == 0.20);

    // A 10x10x50 root box sitting at the root translation, so a mesh-local Z of 5 mm is 0.1 of the height.
    const Transform3d   root = Geometry::translation_transform(Vec3d(7, 8, 9));
    const BoundingBoxf3 root_box(Vec3d(7, 8, 9), Vec3d(17, 18, 59));
    const Vec3d         pivot = root_box.center();

    SECTION("every pose reports the same root-frame height for the same mesh point") {
        for (const Pose &pose : {Pose{0, 0}, Pose{-30, 0}, Pose{-20, 15}, Pose{-40, -15}}) {
            CAPTURE(pose.tilt_deg, pose.lean_deg);
            // The 3.7 mm stands in for the Z shift ensure_on_bed() pre-multiplies onto a tilted instance.
            const Transform3d posed = Geometry::translation_transform(Vec3d(0, 0, 3.7)) *
                                      candidate_transform(root, pivot, pose);
            REQUIRE_THAT(root_height_fraction(root, posed, root_box, posed * Vec3d(3, 3, 5)), WithinAbs(0.1, 1e-9));
            REQUIRE_THAT(root_height_fraction(root, posed, root_box, posed * Vec3d(10, 10, 50)), WithinAbs(1.0, 1e-9));
            REQUIRE_THAT(root_height_fraction(root, posed, root_box, posed * Vec3d(0, 0, 0)), WithinAbs(0.0, 1e-9));
        }
    }

    SECTION("a zero-height box yields 0") {
        const BoundingBoxf3 flat(Vec3d(0, 0, 0), Vec3d(10, 10, 0));
        REQUIRE_THAT(root_height_fraction(root, root, flat, Vec3d(5, 5, 0)), WithinAbs(0.0, 1e-12));
    }

    SECTION("a non-finite point yields a fraction that excludes nothing") {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        REQUIRE(std::isnan(root_height_fraction(root, root, root_box, Vec3d(nan, nan, nan))));
        REQUIRE_FALSE(root_height_fraction(root, root, root_box, Vec3d(nan, nan, nan)) < 0.20);
    }
}

TEST_CASE("fragility_weight scales with thinness and floors sharp contacts", "[AutoTilt]")
{
    const Constants k; // t_ref 2 mm, w_max 8, w_sharp 4

    // t = 2*area/perimeter, w = clamp(t_ref/t, 1, w_max).
    REQUIRE_THAT(fragility_weight(20, 24, false, k), WithinRel(1.2, 1e-9));   // t = 1.6667 mm
    REQUIRE_THAT(fragility_weight(10, 22, false, k), WithinRel(2.2, 1e-9));   // t = 0.9091 mm
    REQUIRE_THAT(fragility_weight(2, 20.4, false, k), WithinRel(8.0, 1e-9));  // raw 10.2, clamped to w_max
    REQUIRE_THAT(fragility_weight(100, 40, false, k), WithinRel(1.0, 1e-9));  // raw 0.4, clamped up to 1
    REQUIRE_THAT(fragility_weight(100, 40, true, k), WithinRel(4.0, 1e-9));   // a floor contact never sits below w_sharp
    REQUIRE_THAT(fragility_weight(2, 20.4, true, k), WithinRel(8.0, 1e-9));   // w_sharp does not pull a thin contact down
    REQUIRE_THAT(fragility_weight(5, 0, false, k), WithinRel(1.0, 1e-9));     // no perimeter, no thickness to measure
}

namespace {

// Scorer stand-in: every pose in `legal` starts at `fill`, individual poses are overridden with
// set(), and every call is logged so the tests can pin call order and call count.
class FakeScorer : public Scorer
{
public:
    FakeScorer(const std::vector<Pose> &legal, const Contact &fill)
    {
        for (const Pose &p : legal)
            m_contacts[key(p)] = fill;
    }

    void set(const Pose &p, const Contact &c) { m_contacts[key(p)] = c; }

    Contact score(const Pose &p) override
    {
        m_calls.push_back(p);
        return m_contacts.at(key(p));
    }

    const std::vector<Pose> &calls() const { return m_calls; }

private:
    static std::pair<double, double> key(const Pose &p) { return {p.tilt_deg, p.lean_deg}; }

    std::map<std::pair<double, double>, Contact> m_contacts;
    std::vector<Pose>                            m_calls;
};

const StopPredicate never_stop = [] { return false; };
const ProgressSink  no_progress = [](size_t, size_t) {};

} // namespace

TEST_CASE("search reports no improvement when every pose scores the same", "[AutoTilt]")
{
    const Constants         k;
    const std::vector<Pose> legal = grid(k);
    // volume 100 sits far above the negligible floor of 0.0002 * 1000 = 0.2, so the whole grid runs.
    FakeScorer              scorer(legal, Contact{100, 100, 1000});

    const SearchResult r = search(legal, scorer, k, never_stop, no_progress);

    REQUIRE(r.outcome == SearchResult::Outcome::NoImprovement);
    REQUIRE(r.best.is_root());
    REQUIRE(r.evaluated == legal.size());
}

TEST_CASE("search breaks a tie between two admissible poses by the smaller deviation", "[AutoTilt]")
{
    const Constants         k;
    const std::vector<Pose> legal = grid(k);
    FakeScorer              scorer(legal, Contact{100, 100, 1000});
    // Both halve the root score, so both clear their own threshold whatever the constants ask.
    scorer.set(Pose{-4, 0}, Contact{50, 50, 1000});
    scorer.set(Pose{-10, 0}, Contact{50, 50, 1000});

    const SearchResult r = search(legal, scorer, k, never_stop, no_progress);

    REQUIRE(r.outcome == SearchResult::Outcome::Improved);
    REQUIRE(r.best == Pose{-4, 0});
}

TEST_CASE("the shipped thresholds ask 5% of a lean and 15% of the steepest tilt", "[AutoTilt]")
{
    // The values Constants ships, asserted in one place so moving them lands here and nowhere else.
    const Constants         k;
    const std::vector<Pose> legal = grid(k);

    SECTION("a lean-only pose clears at 5%") {
        FakeScorer scorer(legal, Contact{100, 100, 1000});
        scorer.set(Pose{0, 5}, Contact{94, 94, 1000}); // 6% against the 5% base, no tilt to add to it

        const SearchResult r = search(legal, scorer, k, never_stop, no_progress);

        REQUIRE(r.outcome == SearchResult::Outcome::Improved);
        REQUIRE(r.best == Pose{0, 5});
        REQUIRE_THAT(r.required_improvement, WithinAbs(0.05, 1e-12));
    }

    SECTION("the steepest tilt has to earn 15%") {
        FakeScorer scorer(legal, Contact{100, 100, 1000});
        scorer.set(Pose{-40, 0}, Contact{85.1, 85.1, 1000}); // 14.9% against 5% + 0.25%/deg * 40 = 15%

        const SearchResult r = search(legal, scorer, k, never_stop, no_progress);

        REQUIRE(r.outcome == SearchResult::Outcome::NoImprovement);
        REQUIRE(r.best == Pose{-40, 0});
        REQUIRE_THAT(r.required_improvement, WithinAbs(0.15, 1e-12));
    }
}

TEST_CASE("search rejects a gain that misses the threshold for its tilt", "[AutoTilt]")
{
    Constants k;
    k.threshold_base              = 0.10;
    k.threshold_per_degree        = 0.0025;
    const std::vector<Pose> legal = grid(k);
    FakeScorer              scorer(legal, Contact{100, 100, 1000});
    scorer.set(Pose{-40, 0}, Contact{95, 95, 1000}); // 5% gain against 10% + 0.25%/deg * 40 = 20%

    const SearchResult r = search(legal, scorer, k, never_stop, no_progress);

    REQUIRE(r.outcome == SearchResult::Outcome::NoImprovement);
    REQUIRE(r.best == Pose{-40, 0});
    REQUIRE_THAT(r.improvement, WithinAbs(0.05, 1e-12));
    REQUIRE_THAT(r.required_improvement, WithinAbs(0.20, 1e-12));
}

TEST_CASE("search requires a 10.5% gain from a 2 degree tilt", "[AutoTilt]")
{
    Constants k; // 10% + 0.25%/deg * 2
    k.threshold_base              = 0.10;
    k.threshold_per_degree        = 0.0025;
    const std::vector<Pose> legal = grid(k);

    SECTION("a 10.4% gain misses the threshold") {
        FakeScorer scorer(legal, Contact{100, 100, 1000});
        scorer.set(Pose{-2, 0}, Contact{89.6, 89.6, 1000});

        const SearchResult r = search(legal, scorer, k, never_stop, no_progress);

        REQUIRE(r.outcome == SearchResult::Outcome::NoImprovement);
        REQUIRE(r.best == Pose{-2, 0});
        REQUIRE_THAT(r.required_improvement, WithinAbs(0.105, 1e-12));
    }

    SECTION("a 10.6% gain clears it") {
        FakeScorer scorer(legal, Contact{100, 100, 1000});
        scorer.set(Pose{-2, 0}, Contact{89.4, 89.4, 1000});

        const SearchResult r = search(legal, scorer, k, never_stop, no_progress);

        REQUIRE(r.outcome == SearchResult::Outcome::Improved);
        REQUIRE(r.best == Pose{-2, 0});
    }
}

TEST_CASE("search ranks inside the admissible set, not over the whole grid", "[AutoTilt]")
{
    Constants k;
    k.threshold_base              = 0.10;
    k.threshold_per_degree        = 0.0025;
    const std::vector<Pose> legal = grid(k);
    FakeScorer              scorer(legal, Contact{100, 100, 1000});
    scorer.set(Pose{-40, 0}, Contact{81, 81, 1000});  // the lowest score, but 19% < 20% required
    scorer.set(Pose{-10, 0}, Contact{85, 85, 1000});  // 15% clears the 12.5% required

    const SearchResult r = search(legal, scorer, k, never_stop, no_progress);

    REQUIRE(r.outcome == SearchResult::Outcome::Improved);
    REQUIRE(r.best == Pose{-10, 0});
}

TEST_CASE("search stops after the root when its contact volume is negligible", "[AutoTilt]")
{
    const Constants         k;
    const std::vector<Pose> legal = grid(k);
    // 1 mm3 of contact against a 100000 mm3 object is 0.001%, below the 0.02% floor.
    FakeScorer              scorer(legal, Contact{0, 0, 100000});
    scorer.set(Pose{}, Contact{10, 1, 100000});
    scorer.set(Pose{-20, 0}, Contact{0, 0, 100000});

    const SearchResult r = search(legal, scorer, k, never_stop, no_progress);

    REQUIRE(r.outcome == SearchResult::Outcome::BelowFloor);
    REQUIRE(r.evaluated == 1);
    REQUIRE(scorer.calls().size() == 1);
    REQUIRE(scorer.calls().front().is_root());
}

TEST_CASE("search scores the root first wherever it sits in the list", "[AutoTilt]")
{
    const Constants   k;
    std::vector<Pose> legal = grid(k);
    std::rotate(legal.begin(), legal.end() - 3, legal.end()); // root moves to index 3
    REQUIRE_FALSE(legal.front().is_root());
    REQUIRE(legal[3].is_root());
    FakeScorer scorer(legal, Contact{100, 100, 1000});

    const SearchResult r = search(legal, scorer, k, never_stop, no_progress);

    REQUIRE(scorer.calls().front().is_root());
    REQUIRE(r.evaluated == legal.size());
}

TEST_CASE("search stops at the next candidate boundary when the stop predicate fires", "[AutoTilt]")
{
    const Constants         k;
    const std::vector<Pose> legal = grid(k);
    FakeScorer              scorer(legal, Contact{100, 100, 1000});
    size_t                  asked = 0;
    const StopPredicate     stop_on_third = [&asked] { return ++asked == 3; };

    const SearchResult r = search(legal, scorer, k, stop_on_third, no_progress);

    REQUIRE(r.outcome == SearchResult::Outcome::Canceled);
    REQUIRE(r.evaluated == 3); // the root plus two candidates
    REQUIRE(scorer.calls().back() == legal[2]);
}

TEST_CASE("search reports progress once per scored candidate", "[AutoTilt]")
{
    Constants k;
    k.tilts_deg = {0, -20, -40};
    k.leans_deg = {-15, 0, 15};
    const std::vector<Pose> legal = grid(k);
    REQUIRE(legal.size() == 9);
    FakeScorer                             scorer(legal, Contact{100, 100, 1000});
    std::vector<std::pair<size_t, size_t>> ticks;
    const ProgressSink sink = [&ticks](size_t done, size_t total) { ticks.emplace_back(done, total); };

    search(legal, scorer, k, never_stop, sink);

    REQUIRE(ticks.size() == 9);
    for (size_t i = 0; i < ticks.size(); ++i) {
        CAPTURE(i);
        REQUIRE(ticks[i].first == i + 1);
        REQUIRE(ticks[i].second == 9);
    }
}

TEST_CASE("instances_unchanged matches instances by ObjectID and compares matrices exactly", "[AutoTilt]")
{
    const Transform3d a = Transform3d::Identity();
    const Transform3d b = Geometry::translation_transform(Vec3d(5, 0, 0));
    const std::vector<InstanceSnapshot> cached{{ObjectID(1), a}, {ObjectID(2), b}};

    SECTION("an identical list is unchanged") {
        REQUIRE(instances_unchanged(cached, {{ObjectID(1), a}, {ObjectID(2), b}}));
    }
    SECTION("the smallest matrix difference counts as changed") {
        Transform3d moved      = b;
        moved.matrix()(0, 3) += 1e-12;
        REQUIRE_FALSE(instances_unchanged(cached, {{ObjectID(1), a}, {ObjectID(2), moved}}));
    }
    SECTION("a missing instance counts as changed") {
        REQUIRE_FALSE(instances_unchanged(cached, {{ObjectID(1), a}}));
    }
    SECTION("an extra instance counts as changed") {
        REQUIRE_FALSE(instances_unchanged(cached, {{ObjectID(1), a}, {ObjectID(2), b}, {ObjectID(3), a}}));
    }
    SECTION("reordering the same instances is not a change") {
        // Index-wise comparison would see id 2's matrix against id 1's and report a change.
        REQUIRE(instances_unchanged(cached, {{ObjectID(2), b}, {ObjectID(1), a}}));
    }
}
