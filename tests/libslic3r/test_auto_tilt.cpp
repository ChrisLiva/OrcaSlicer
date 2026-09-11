#include <catch2/catch_all.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>

#include "libslic3r/AutoTilt.hpp"
#include "libslic3r/AutoTiltEvaluation.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/BrimEarsPoint.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Slicing.hpp"
#include "libslic3r/TriangleMesh.hpp"

using namespace Slic3r;
using namespace Slic3r::AutoTilt;

using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

TEST_CASE("grid lists the root pose first and then every tilt/lean pair tilt-major", "[AutoTilt]")
{
    const Constants         k;
    const std::vector<Pose> g = grid(k);

    REQUIRE(g.size() == 77);
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

// Verifier stand-in: hands back the PoseEvaluation the test registered for a pose, and logs every
// call, so the tests can pin what was verified and how often. A pose nothing was registered for
// verifies as Unknown, which is what an evaluator answers for a pose it cannot measure at all.
class FakeVerifier : public Verifier
{
public:
    void set(const Pose &p, const PoseEvaluation &e) { m_evaluations[key(p)] = e; }

    PoseEvaluation evaluate(const Pose &p, const StopPredicate &) override
    {
        m_calls.push_back(p);
        const auto     it  = m_evaluations.find(key(p));
        PoseEvaluation out = it == m_evaluations.end() ? PoseEvaluation{} : it->second;
        out.pose           = p;
        return out;
    }

    const std::vector<Pose> &calls() const { return m_calls; }

private:
    static std::pair<double, double> key(const Pose &p) { return {p.tilt_deg, p.lean_deg}; }

    std::map<std::pair<double, double>, PoseEvaluation> m_evaluations;
    std::vector<Pose>                                   m_calls;
};

SupportAnalysis::Damage damage_of(size_t unknown_contacts, size_t inaccessible_groups, double max_risk, double total_risk)
{
    SupportAnalysis::Damage damage;
    damage.unknown_contacts    = unknown_contacts;
    damage.inaccessible_groups = inaccessible_groups;
    damage.max_group_risk      = max_risk;
    damage.total_group_risk    = total_risk;
    damage.available           = true;
    return damage;
}

// One affected instance measured in full: complete, nothing left uncovered, and stability admissible
// on its own terms. The damage numbers are what the ranking tests vary.
SupportAnalysis::Report measured_instance(const SupportAnalysis::Damage &damage)
{
    SupportAnalysis::Report report;
    report.status                    = SupportAnalysis::Report::Status::Complete;
    report.coverage_available        = true;
    report.stability.available       = true;
    report.stability.min_bed_margin  = 0.5;
    report.stability.max_slenderness = 4.;
    report.damage                    = damage;
    return report;
}

// One pose measured in full over one affected instance, printing the material asked for.
PoseEvaluation measured_pose(const SupportAnalysis::Damage &damage, double support_mm3, double raft_mm3 = 0.)
{
    PoseEvaluation out;
    out.status             = PoseEvaluation::Status::Complete;
    out.instances          = { measured_instance(damage) };
    out.support_volume_mm3 = support_mm3;
    out.raft_volume_mm3    = raft_mm3;
    return out;
}

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

TEST_CASE("the shipped thresholds ask 5% of a lean and 10% of the steepest tilt", "[AutoTilt]")
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

    SECTION("the steepest tilt has to earn 10%") {
        FakeScorer scorer(legal, Contact{100, 100, 1000});
        scorer.set(Pose{-20, 0}, Contact{90.1, 90.1, 1000}); // 9.9% against 5% + 0.25%/deg * 20 = 10%

        const SearchResult r = search(legal, scorer, k, never_stop, no_progress);

        REQUIRE(r.outcome == SearchResult::Outcome::NoImprovement);
        REQUIRE(r.best == Pose{-20, 0});
        REQUIRE_THAT(r.required_improvement, WithinAbs(0.10, 1e-12));
    }
}

TEST_CASE("search rejects a gain that misses the threshold for its tilt", "[AutoTilt]")
{
    Constants k;
    k.threshold_base              = 0.10;
    k.threshold_per_degree        = 0.0025;
    const std::vector<Pose> legal = grid(k);
    FakeScorer              scorer(legal, Contact{100, 100, 1000});
    scorer.set(Pose{-20, 0}, Contact{95, 95, 1000}); // 5% gain against 10% + 0.25%/deg * 20 = 15%

    const SearchResult r = search(legal, scorer, k, never_stop, no_progress);

    REQUIRE(r.outcome == SearchResult::Outcome::NoImprovement);
    REQUIRE(r.best == Pose{-20, 0});
    REQUIRE_THAT(r.improvement, WithinAbs(0.05, 1e-12));
    REQUIRE_THAT(r.required_improvement, WithinAbs(0.15, 1e-12));
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
    scorer.set(Pose{-20, 0}, Contact{86, 86, 1000});  // the lowest score, but 14% < 15% required
    scorer.set(Pose{-10, 0}, Contact{87, 87, 1000});  // 13% clears the 12.5% required

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

TEST_CASE("verified search evaluates the root in full before it considers any pose", "[AutoTilt]")
{
    const Constants         k;
    const std::vector<Pose> legal = grid(k);

    SECTION("a stop that fires before the first pose measures nothing") {
        FakeScorer   scorer(legal, Contact{100, 100, 1000});
        FakeVerifier verifier;

        const VerifiedSearchResult r = search_verified(legal, scorer, verifier, k, [] { return true; }, no_progress);

        REQUIRE(r.outcome == VerifiedSearchResult::Outcome::Canceled);
        REQUIRE(r.verified == 0);
        REQUIRE(verifier.calls().empty());
        REQUIRE(scorer.calls().empty());
        REQUIRE(r.selected.pose.is_root());
    }

    SECTION("a cheap root below the negligible-contact floor is verified all the same") {
        // The estimate search stops on this root: no contact at all against a 100000 mm3 object. A
        // fragile feature the bottom exclusion plane dropped reads exactly like that, so the verified
        // path measures the root anyway and still finds the pose the print measures as better.
        FakeScorer scorer(legal, Contact{0, 0, 100000});
        scorer.set(Pose{-4, 0}, Contact{-1, 0, 100000}); // the best cheap score of the grid
        FakeVerifier verifier;
        verifier.set(Pose{}, measured_pose(damage_of(0, 0, 4., 10.), 100.));
        verifier.set(Pose{-4, 0}, measured_pose(damage_of(0, 0, 2., 5.), 100.));

        const VerifiedSearchResult r = search_verified(legal, scorer, verifier, k, never_stop, no_progress);

        REQUIRE(r.outcome == VerifiedSearchResult::Outcome::Improved);
        REQUIRE(r.selected.pose == Pose{-4, 0});
        REQUIRE(r.verified == 6);
    }

    SECTION("an unknown root analysis leaves the object where it stands") {
        FakeScorer   scorer(legal, Contact{100, 100, 1000});
        FakeVerifier verifier; // nothing registered, so every pose measures Unknown

        const VerifiedSearchResult r = search_verified(legal, scorer, verifier, k, never_stop, no_progress);

        REQUIRE(r.outcome == VerifiedSearchResult::Outcome::VerificationUnavailable);
        REQUIRE(r.selected.pose.is_root());
        REQUIRE(r.verified == 1);
        REQUIRE(verifier.calls().size() == 1);
        REQUIRE(scorer.calls().empty()); // no cheap sweep is worth running for a root nothing measured
    }

    SECTION("unresolved root coverage does not stop the search") {
        FakeScorer     scorer(legal, Contact{100, 100, 1000});
        PoseEvaluation root = measured_pose(damage_of(0, 0, 4., 10.), 100.);
        root.status         = PoseEvaluation::Status::UnresolvedCoverage;
        FakeVerifier verifier;
        verifier.set(Pose{}, root);
        verifier.set(Pose{-4, 0}, measured_pose(damage_of(0, 0, 1., 1.), 1.));

        const VerifiedSearchResult r = search_verified(legal, scorer, verifier, k, never_stop, no_progress);

        REQUIRE(r.outcome == VerifiedSearchResult::Outcome::Improved);
        REQUIRE(r.selected.pose == Pose{-4, 0});
        REQUIRE(r.verified > 1);
    }
}

TEST_CASE("verified search shortlists five finalists by cheap score, deviation and grid order", "[AutoTilt]")
{
    const Constants         k;
    const std::vector<Pose> legal = grid(k);
    REQUIRE(legal.size() == 77); // the legal angular grid the estimate search already sweeps

    SECTION("the five best cheap scores become the finalists and the sixth is left alone") {
        FakeScorer scorer(legal, Contact{100, 100, 1000});
        scorer.set(Pose{-2, 0}, Contact{10, 10, 1000});
        scorer.set(Pose{-20, -15}, Contact{20, 20, 1000});
        scorer.set(Pose{-6, 5}, Contact{30, 30, 1000});
        scorer.set(Pose{0, -5}, Contact{40, 40, 1000});
        scorer.set(Pose{-4, -10}, Contact{50, 50, 1000});
        scorer.set(Pose{-8, 0}, Contact{60, 60, 1000}); // the sixth best, and never sliced
        FakeVerifier verifier;
        verifier.set(Pose{}, measured_pose(damage_of(0, 0, 4., 10.), 100.));

        const VerifiedSearchResult r = search_verified(legal, scorer, verifier, k, never_stop, no_progress);

        const std::vector<Pose> expected{Pose{-2, 0}, Pose{-20, -15}, Pose{-6, 5}, Pose{0, -5}, Pose{-4, -10}};
        REQUIRE(r.shortlist == expected);
        REQUIRE(r.cheap_scored == 76); // every legal pose but the root
        REQUIRE(r.verified == 6);      // the root plus the five finalists
        REQUIRE(verifier.calls().size() == 6);
        REQUIRE(verifier.calls().front().is_root());
        for (const Pose &finalist : expected)
            REQUIRE(std::count(verifier.calls().begin(), verifier.calls().end(), finalist) == 1);
        REQUIRE(std::count(verifier.calls().begin(), verifier.calls().end(), Pose{-8, 0}) == 0);
    }

    SECTION("equal cheap scores go to the smaller deviation and then to the earlier grid position") {
        FakeScorer scorer(legal, Contact{100, 100, 1000});
        for (const Pose &p : {Pose{0, -5}, Pose{-2, 0}, Pose{0, 5}, Pose{-4, 0}, Pose{-2, -5}, Pose{-6, 0}})
            scorer.set(p, Contact{10, 10, 1000});
        FakeVerifier verifier;
        verifier.set(Pose{}, measured_pose(damage_of(0, 0, 4., 10.), 100.));

        const VerifiedSearchResult r = search_verified(legal, scorer, verifier, k, never_stop, no_progress);

        // Deviations 2, 4, 5, 5, 5.39: the two at 5 keep the order the grid lists them in, and the
        // 6 degree tilt is the sixth of six equal scores.
        const std::vector<Pose> expected{Pose{-2, 0}, Pose{-4, 0}, Pose{0, -5}, Pose{0, 5}, Pose{-2, -5}};
        REQUIRE(r.shortlist == expected);
    }

    SECTION("a cheap score that is not a number orders nothing and is not shortlisted") {
        FakeScorer   scorer(legal, Contact{100, 100, 1000});
        const double nan = std::numeric_limits<double>::quiet_NaN();
        scorer.set(Pose{-2, 0}, Contact{nan, 10, 1000});
        scorer.set(Pose{-4, 0}, Contact{- std::numeric_limits<double>::infinity(), 10, 1000});
        scorer.set(Pose{-6, 0}, Contact{10, 10, 1000});
        FakeVerifier verifier;
        verifier.set(Pose{}, measured_pose(damage_of(0, 0, 4., 10.), 100.));

        const VerifiedSearchResult r = search_verified(legal, scorer, verifier, k, never_stop, no_progress);

        REQUIRE(r.shortlist.front() == Pose{-6, 0});
        REQUIRE(std::count(r.shortlist.begin(), r.shortlist.end(), Pose{-2, 0}) == 0);
        REQUIRE(std::count(r.shortlist.begin(), r.shortlist.end(), Pose{-4, 0}) == 0);
        REQUIRE(r.cheap_scored == 76); // both were scored; neither was ranked
    }

    SECTION("a finalist the full evaluation rejects is not retried and no sixth pose replaces it") {
        FakeScorer scorer(legal, Contact{100, 100, 1000});
        scorer.set(Pose{-2, 0}, Contact{10, 10, 1000});
        scorer.set(Pose{-20, -15}, Contact{20, 20, 1000});
        scorer.set(Pose{-6, 5}, Contact{30, 30, 1000});
        scorer.set(Pose{0, -5}, Contact{40, 40, 1000});
        scorer.set(Pose{-4, -10}, Contact{50, 50, 1000});
        scorer.set(Pose{-8, 0}, Contact{60, 60, 1000}); // the sixth best cheap score
        FakeVerifier   verifier;
        verifier.set(Pose{}, measured_pose(damage_of(0, 0, 4., 10.), 100.));
        // The best cheap score of the whole grid, and a pose the plate cannot print: a cheap score
        // grants no validity, and the budget does not grow to look for a replacement.
        PoseEvaluation rejected = measured_pose(damage_of(0, 0, 0., 0.), 0.);
        rejected.status         = PoseEvaluation::Status::Invalid;
        verifier.set(Pose{-2, 0}, rejected);

        const VerifiedSearchResult r = search_verified(legal, scorer, verifier, k, never_stop, no_progress);

        REQUIRE(r.shortlist.front() == Pose{-2, 0});
        REQUIRE(r.outcome == VerifiedSearchResult::Outcome::NoImprovement);
        REQUIRE(r.selected.pose.is_root());
        REQUIRE(std::count(verifier.calls().begin(), verifier.calls().end(), Pose{-2, 0}) == 1);
        REQUIRE(verifier.calls().size() == 6);
        REQUIRE(std::count(verifier.calls().begin(), verifier.calls().end(), Pose{-8, 0}) == 0);
    }
}

TEST_CASE("a verified candidate is measured, standing and no less stable than the root", "[AutoTilt]")
{
    const Constants         k;
    const std::vector<Pose> legal{Pose{}, Pose{-4, 0}};
    const PoseEvaluation    root = measured_pose(damage_of(0, 0, 4., 10.), 100.);

    // Half the root's risk on every count, so nothing but admissibility can keep it out.
    PoseEvaluation candidate = measured_pose(damage_of(0, 0, 2., 5.), 50.);

    const auto outcome_with = [&](const PoseEvaluation &c) {
        FakeScorer   scorer(legal, Contact{100, 100, 1000});
        FakeVerifier verifier;
        verifier.set(Pose{}, root);
        verifier.set(Pose{-4, 0}, c);
        return search_verified(legal, scorer, verifier, k, never_stop, no_progress).outcome;
    };

    SECTION("the candidate that gives nothing up is the one that wins") {
        REQUIRE(outcome_with(candidate) == VerifiedSearchResult::Outcome::Improved);
    }

    SECTION("an incomplete pose measurement cannot win") {
        for (const PoseEvaluation::Status status : {PoseEvaluation::Status::Unknown, PoseEvaluation::Status::Invalid}) {
            PoseEvaluation c = candidate;
            c.status         = status;
            REQUIRE(outcome_with(c) == VerifiedSearchResult::Outcome::NoImprovement);
        }
    }

    SECTION("a pose that leaves a required region open still wins on what it measured") {
        PoseEvaluation c                  = candidate;
        c.status                          = PoseEvaluation::Status::UnresolvedCoverage;
        c.instances[0].missing_anchor_ids = {17};
        REQUIRE(outcome_with(c) == VerifiedSearchResult::Outcome::Improved);
    }

    SECTION("a pose whose support printed nothing against its required regions cannot win") {
        // What SupportAnalysis::measure reports when nothing was emitted: no material, so no damage
        // and no volume, which would otherwise rank above every pose that printed anything.
        PoseEvaluation c        = candidate;
        c.status                = PoseEvaluation::Status::UnresolvedCoverage;
        c.instances[0].status   = SupportAnalysis::Report::Status::UnresolvedCoverage;
        REQUIRE(outcome_with(c) == VerifiedSearchResult::Outcome::NoImprovement);
    }

    SECTION("an incomplete instance report cannot win") {
        PoseEvaluation c        = candidate;
        c.instances[0].status   = SupportAnalysis::Report::Status::Unknown;
        REQUIRE(outcome_with(c) == VerifiedSearchResult::Outcome::NoImprovement);
    }

    SECTION("material standing on air, a group that never printed, or a centroid off its footprint cannot win") {
        PoseEvaluation paths = candidate;
        paths.instances[0].stability.unsupported_paths = 1;
        REQUIRE(outcome_with(paths) == VerifiedSearchResult::Outcome::NoImprovement);

        PoseEvaluation unrooted = candidate;
        unrooted.instances[0].stability.unrooted_groups = 1;
        REQUIRE(outcome_with(unrooted) == VerifiedSearchResult::Outcome::NoImprovement);

        PoseEvaluation tipping = candidate;
        tipping.instances[0].stability.min_bed_margin = -0.01;
        REQUIRE(outcome_with(tipping) == VerifiedSearchResult::Outcome::NoImprovement);

        PoseEvaluation unmeasured = candidate;
        unmeasured.instances[0].stability.available = false;
        REQUIRE(outcome_with(unmeasured) == VerifiedSearchResult::Outcome::NoImprovement);
    }

    SECTION("stability that stands on its own but is worse than the root's cannot win") {
        // Both readings are admissible on their own terms. The comparison is against the root, so a
        // pose that is measurably less stable than the pose the object already holds does not win.
        PoseEvaluation slimmer = candidate;
        slimmer.instances[0].stability.min_bed_margin = 0.2; // the root holds 0.5
        REQUIRE(outcome_with(slimmer) == VerifiedSearchResult::Outcome::NoImprovement);

        PoseEvaluation slenderer = candidate;
        slenderer.instances[0].stability.max_slenderness = 5.; // the root holds 4
        REQUIRE(outcome_with(slenderer) == VerifiedSearchResult::Outcome::NoImprovement);
    }

    SECTION("a safer copy never stands in for one that got worse") {
        PoseEvaluation two_root = root;
        two_root.instances.push_back(measured_instance(damage_of(0, 0, 4., 10.)));
        PoseEvaluation two = candidate;
        two.instances.push_back(measured_instance(damage_of(0, 0, 2., 5.)));
        two.instances[0].stability.min_bed_margin = 0.9; // one copy stands better
        two.instances[1].stability.min_bed_margin = 0.1; // and the other stands worse

        FakeScorer   scorer(legal, Contact{100, 100, 1000});
        FakeVerifier verifier;
        verifier.set(Pose{}, two_root);
        verifier.set(Pose{-4, 0}, two);
        REQUIRE(search_verified(legal, scorer, verifier, k, never_stop, no_progress).outcome ==
                VerifiedSearchResult::Outcome::NoImprovement);
    }

    SECTION("a pose that measured a different set of instances is not comparable") {
        PoseEvaluation two_root = root;
        two_root.instances.push_back(measured_instance(damage_of(0, 0, 4., 10.)));

        FakeScorer   scorer(legal, Contact{100, 100, 1000});
        FakeVerifier verifier;
        verifier.set(Pose{}, two_root);
        verifier.set(Pose{-4, 0}, candidate); // one instance against the root's two
        REQUIRE(search_verified(legal, scorer, verifier, k, never_stop, no_progress).outcome ==
                VerifiedSearchResult::Outcome::NoImprovement);
    }

    SECTION("region ids from another pose's problem are never matched against the root's") {
        // Two poses prepare two problems, and a region id in one names nothing in the other. The
        // candidate is compared on its per-instance counts and normalized metrics alone.
        PoseEvaluation posed_root = root;
        posed_root.instances[0].key.region_ids = {1, 2};
        posed_root.instances[0].coverage.resize(2);
        posed_root.instances[0].coverage[0].emitted_path = true;
        posed_root.instances[0].coverage[1].emitted_path = true;

        PoseEvaluation c        = candidate;
        c.instances[0].key.region_ids = {7, 8};
        c.instances[0].coverage.resize(2);
        c.instances[0].coverage[0].emitted_path = false;
        c.instances[0].coverage[1].emitted_path = false;

        FakeScorer   scorer(legal, Contact{100, 100, 1000});
        FakeVerifier verifier;
        verifier.set(Pose{}, posed_root);
        verifier.set(Pose{-4, 0}, c);
        REQUIRE(search_verified(legal, scorer, verifier, k, never_stop, no_progress).outcome ==
                VerifiedSearchResult::Outcome::Improved);
    }
}

TEST_CASE("verified ranking compares damage, then volume, and gates the first improved continuous objective", "[AutoTilt]")
{
    const Constants k; // 5% base plus 0.25% per degree of tilt

    // One root and one candidate, so the ranking has nothing to hide behind: the shortlist is the
    // single non-root pose, and what comes back is what the gate made of it.
    const auto against_root = [&k](const Pose &pose, const PoseEvaluation &root, const PoseEvaluation &candidate) {
        const std::vector<Pose> legal{Pose{}, pose};
        FakeScorer              scorer(legal, Contact{100, 100, 1000});
        FakeVerifier            verifier;
        verifier.set(Pose{}, root);
        verifier.set(pose, candidate);
        return search_verified(legal, scorer, verifier, k, never_stop, no_progress);
    };

    SECTION("one contact fewer that nothing could answer for clears no percentage at all") {
        // The steepest tilt in the grid, which the continuous gate would charge 10% for.
        const VerifiedSearchResult r = against_root(Pose{-20, 0}, measured_pose(damage_of(2, 1, 4., 10.), 100.),
                                                    measured_pose(damage_of(1, 1, 4., 10.), 100.));
        REQUIRE(r.outcome == VerifiedSearchResult::Outcome::Improved);
        REQUIRE_THAT(r.required_improvement, WithinAbs(0., 1e-12));
    }

    SECTION("one group fewer that nothing can reach clears no percentage either") {
        const VerifiedSearchResult r = against_root(Pose{-20, 0}, measured_pose(damage_of(0, 2, 4., 10.), 100.),
                                                    measured_pose(damage_of(0, 1, 4., 10.), 100.));
        REQUIRE(r.outcome == VerifiedSearchResult::Outcome::Improved);
        REQUIRE_THAT(r.required_improvement, WithinAbs(0., 1e-12));
    }

    SECTION("a lower worst group is charged against that same worst group") {
        const PoseEvaluation root = measured_pose(damage_of(0, 0, 4., 10.), 100.);

        const VerifiedSearchResult missed = against_root(Pose{-20, 0}, root, measured_pose(damage_of(0, 0, 3.61, 10.), 100.));
        REQUIRE(missed.outcome == VerifiedSearchResult::Outcome::NoImprovement);
        REQUIRE_THAT(missed.improvement, WithinAbs(0.0975, 1e-12)); // of the root's own 4, not of its 10
        REQUIRE_THAT(missed.required_improvement, WithinAbs(0.10, 1e-12));

        const VerifiedSearchResult cleared = against_root(Pose{-20, 0}, root, measured_pose(damage_of(0, 0, 3.59, 10.), 100.));
        REQUIRE(cleared.outcome == VerifiedSearchResult::Outcome::Improved);
        REQUIRE_THAT(cleared.improvement, WithinAbs(0.1025, 1e-12));
    }

    SECTION("with the counts and the worst group tied, the summed risk is what is charged") {
        const PoseEvaluation root = measured_pose(damage_of(0, 0, 4., 10.), 100.);

        const VerifiedSearchResult missed = against_root(Pose{-20, 0}, root, measured_pose(damage_of(0, 0, 4., 9.3), 100.));
        REQUIRE(missed.outcome == VerifiedSearchResult::Outcome::NoImprovement);
        REQUIRE_THAT(missed.improvement, WithinAbs(0.07, 1e-12));

        const VerifiedSearchResult cleared = against_root(Pose{-20, 0}, root, measured_pose(damage_of(0, 0, 4., 8.9), 100.));
        REQUIRE(cleared.outcome == VerifiedSearchResult::Outcome::Improved);
        REQUIRE_THAT(cleared.improvement, WithinAbs(0.11, 1e-12));
    }

    SECTION("with every damage field tied, the material is what is charged") {
        const PoseEvaluation root = measured_pose(damage_of(0, 0, 4., 10.), 90., 10.);

        // A lean-only pose asks the 5% base, and the raft counts as material the print lays.
        const VerifiedSearchResult missed = against_root(Pose{0, 5}, root, measured_pose(damage_of(0, 0, 4., 10.), 86., 10.));
        REQUIRE(missed.outcome == VerifiedSearchResult::Outcome::NoImprovement);
        REQUIRE_THAT(missed.improvement, WithinAbs(0.04, 1e-12));
        REQUIRE_THAT(missed.required_improvement, WithinAbs(0.05, 1e-12));

        const VerifiedSearchResult cleared = against_root(Pose{0, 5}, root, measured_pose(damage_of(0, 0, 4., 10.), 84., 10.));
        REQUIRE(cleared.outcome == VerifiedSearchResult::Outcome::Improved);
        REQUIRE_THAT(cleared.improvement, WithinAbs(0.06, 1e-12));
    }

    SECTION("a regression in an earlier objective is not bought back by a later one") {
        const VerifiedSearchResult r = against_root(Pose{-4, 0}, measured_pose(damage_of(0, 0, 4., 10.), 100.),
                                                    measured_pose(damage_of(0, 1, 0.5, 1.), 1.));
        REQUIRE(r.outcome == VerifiedSearchResult::Outcome::NoImprovement);
        REQUIRE(r.selected.pose.is_root());
    }

    SECTION("an exact tie on every objective retains the root") {
        const VerifiedSearchResult r = against_root(Pose{-4, 0}, measured_pose(damage_of(0, 0, 4., 10.), 100.),
                                                    measured_pose(damage_of(0, 0, 4., 10.), 100.));
        REQUIRE(r.outcome == VerifiedSearchResult::Outcome::NoImprovement);
        REQUIRE(r.selected.pose.is_root());
        REQUIRE_THAT(r.improvement, WithinAbs(0., 1e-12));
    }

    SECTION("a root that measured nothing to improve on invents no percentage") {
        // Nothing weighed and nothing printed on either side: the fraction has no denominator, and a
        // pose that ties the root is not one to move the object for.
        const VerifiedSearchResult tied = against_root(Pose{-4, 0}, measured_pose(damage_of(0, 0, 0., 0.), 0.),
                                                       measured_pose(damage_of(0, 0, 0., 0.), 0.));
        REQUIRE(tied.outcome == VerifiedSearchResult::Outcome::NoImprovement);
        REQUIRE(std::isfinite(tied.improvement));
        REQUIRE_THAT(tied.improvement, WithinAbs(0., 1e-12));

        // The strictly lower earlier objective is what wins there, and it is a count, not a fraction.
        const VerifiedSearchResult counted = against_root(Pose{-4, 0}, measured_pose(damage_of(1, 0, 0., 0.), 0.),
                                                          measured_pose(damage_of(0, 0, 0., 0.), 0.));
        REQUIRE(counted.outcome == VerifiedSearchResult::Outcome::Improved);
    }

    SECTION("the ranking runs damage, then volume, then deviation, then grid order") {
        const std::vector<Pose> legal = grid(k);
        const PoseEvaluation    root  = measured_pose(damage_of(0, 0, 4., 10.), 100.);

        const auto rank_of = [&](const std::map<std::pair<double, double>, PoseEvaluation> &candidates) {
            FakeScorer   scorer(legal, Contact{100, 100, 1000});
            FakeVerifier verifier;
            verifier.set(Pose{}, root);
            double cheap = 1.;
            for (const auto &entry : candidates) {
                const Pose pose{entry.first.first, entry.first.second};
                scorer.set(pose, Contact{cheap, cheap, 1000}); // both shortlisted, the other finalist slots grid fill; none of it decisive
                cheap += 1.;
                verifier.set(pose, entry.second);
            }
            return search_verified(legal, scorer, verifier, k, never_stop, no_progress).selected.pose;
        };

        // Same damage, less material: the material decides.
        REQUIRE(rank_of({{{-8., 0.}, measured_pose(damage_of(0, 0, 2., 5.), 50.)},
                         {{-10., 0.}, measured_pose(damage_of(0, 0, 2., 5.), 40.)}}) == Pose{-10, 0});
        // Lower damage on more material: the damage decides.
        REQUIRE(rank_of({{{-8., 0.}, measured_pose(damage_of(0, 0, 2., 5.), 50.)},
                         {{-10., 0.}, measured_pose(damage_of(0, 0, 2., 6.), 40.)}}) == Pose{-8, 0});
        // Everything tied: the pose closer to the root.
        REQUIRE(rank_of({{{-8., 0.}, measured_pose(damage_of(0, 0, 2., 5.), 50.)},
                         {{-10., 0.}, measured_pose(damage_of(0, 0, 2., 5.), 50.)}}) == Pose{-8, 0});
        // Tied at the same deviation of 10 degrees: the pose the grid lists first.
        REQUIRE(rank_of({{{-10., 0.}, measured_pose(damage_of(0, 0, 2., 5.), 50.)},
                         {{0., 10.}, measured_pose(damage_of(0, 0, 2., 5.), 50.)}}) == Pose{0, 10});
    }
}

TEST_CASE("the verified oracle answers to the full evaluation and never to the cheap one", "[AutoTilt]")
{
    const Constants k;

    SECTION("five finalists, and the one the print measures best is the one applied") {
        const std::vector<Pose> legal = grid(k);
        FakeScorer              scorer(legal, Contact{100, 100, 1000});
        FakeVerifier            verifier;
        verifier.set(Pose{}, measured_pose(damage_of(0, 0, 4., 10.), 100.));

        // The best cheap score of the whole grid, and the print says it is worse than the root.
        scorer.set(Pose{-2, 0}, Contact{10, 10, 1000});
        verifier.set(Pose{-2, 0}, measured_pose(damage_of(0, 0, 4., 11.), 60.));
        // Nothing printed at all, and a required region left open for it: material saved buys back
        // no coverage.
        scorer.set(Pose{-4, 0}, Contact{20, 20, 1000});
        PoseEvaluation uncovered       = measured_pose(damage_of(0, 0, 0., 0.), 0.);
        uncovered.status               = PoseEvaluation::Status::UnresolvedCoverage;
        uncovered.instances[0].status  = SupportAnalysis::Report::Status::UnresolvedCoverage;
        verifier.set(Pose{-4, 0}, uncovered);
        // Half the removal risk on the same material.
        scorer.set(Pose{-6, 0}, Contact{30, 30, 1000});
        verifier.set(Pose{-6, 0}, measured_pose(damage_of(0, 0, 2., 5.), 100.));
        // The same removal risk on half the material: a real saving, and not the one that decides.
        scorer.set(Pose{-8, 0}, Contact{40, 40, 1000});
        verifier.set(Pose{-8, 0}, measured_pose(damage_of(0, 0, 4., 10.), 50.));
        // A pose the evaluator does not answer for.
        scorer.set(Pose{-10, 0}, Contact{50, 50, 1000});
        PoseEvaluation unknown = measured_pose(damage_of(0, 0, 1., 1.), 1.);
        unknown.status         = PoseEvaluation::Status::Unknown;
        verifier.set(Pose{-10, 0}, unknown);
        // The sixth best cheap score, which nothing ever slices.
        scorer.set(Pose{-12, 0}, Contact{60, 60, 1000});

        const VerifiedSearchResult r = search_verified(legal, scorer, verifier, k, never_stop, no_progress);

        REQUIRE(r.outcome == VerifiedSearchResult::Outcome::Improved);
        REQUIRE(r.selected.pose == Pose{-6, 0});
        // What was actually measured, so a five-finalist answer is never read as a swept grid.
        REQUIRE(r.cheap_scored == 76);
        REQUIRE(r.verified == 6);
        REQUIRE(r.shortlist.size() == 5);
        REQUIRE(std::count(verifier.calls().begin(), verifier.calls().end(), Pose{-12, 0}) == 0);
    }

    SECTION("a grid with fewer poses than finalists runs to an answer") {
        const std::vector<Pose> legal{Pose{}, Pose{0, -5}, Pose{-4, 0}};
        FakeScorer              scorer(legal, Contact{100, 100, 1000});
        scorer.set(Pose{-4, 0}, Contact{10, 10, 1000});
        FakeVerifier verifier;
        verifier.set(Pose{}, measured_pose(damage_of(0, 0, 4., 10.), 100.));
        verifier.set(Pose{0, -5}, measured_pose(damage_of(0, 0, 4., 10.), 100.));
        verifier.set(Pose{-4, 0}, measured_pose(damage_of(0, 0, 2., 5.), 100.));

        const VerifiedSearchResult r = search_verified(legal, scorer, verifier, k, never_stop, no_progress);

        REQUIRE(r.shortlist.size() == 2);
        REQUIRE(r.verified == 3);
        REQUIRE(r.outcome == VerifiedSearchResult::Outcome::Improved);
        REQUIRE(r.selected.pose == Pose{-4, 0});
    }

    SECTION("a measurement that is not a number cannot win") {
        const std::vector<Pose> legal{Pose{}, Pose{-4, 0}};
        FakeScorer              scorer(legal, Contact{100, 100, 1000});
        FakeVerifier            verifier;
        verifier.set(Pose{}, measured_pose(damage_of(0, 0, 4., 10.), 100.));
        // Nothing printed and a risk nobody can read: an unreadable number is not a low one.
        PoseEvaluation nonsense = measured_pose(damage_of(0, 0, std::numeric_limits<double>::quiet_NaN(), 5.), 0.);
        verifier.set(Pose{-4, 0}, nonsense);

        const VerifiedSearchResult r = search_verified(legal, scorer, verifier, k, never_stop, no_progress);

        REQUIRE(r.outcome == VerifiedSearchResult::Outcome::NoImprovement);
        REQUIRE(r.selected.pose.is_root());
    }

    SECTION("a cancellation part way through the finalists leaves the object where it stands") {
        const std::vector<Pose> legal{Pose{}, Pose{-4, 0}, Pose{-6, 0}};
        FakeScorer              scorer(legal, Contact{100, 100, 1000});
        scorer.set(Pose{-4, 0}, Contact{10, 10, 1000});
        FakeVerifier verifier;
        verifier.set(Pose{}, measured_pose(damage_of(0, 0, 4., 10.), 100.));
        verifier.set(Pose{-4, 0}, measured_pose(damage_of(0, 0, 1., 1.), 1.));
        verifier.set(Pose{-6, 0}, measured_pose(damage_of(0, 0, 1., 1.), 1.));

        // The root, both cheap scores and the first finalist, and then the user says stop.
        size_t              asked = 0;
        const StopPredicate stop_on_fifth = [&asked] { return ++ asked >= 5; };

        const VerifiedSearchResult r = search_verified(legal, scorer, verifier, k, stop_on_fifth, no_progress);

        REQUIRE(r.outcome == VerifiedSearchResult::Outcome::Canceled);
        REQUIRE(r.selected.pose.is_root());
    }

    SECTION("an evaluation that came back canceled cannot win either") {
        const std::vector<Pose> legal{Pose{}, Pose{-4, 0}};
        FakeScorer              scorer(legal, Contact{100, 100, 1000});
        FakeVerifier            verifier;
        verifier.set(Pose{}, measured_pose(damage_of(0, 0, 4., 10.), 100.));
        PoseEvaluation stopped = measured_pose(damage_of(0, 0, 1., 1.), 1.);
        stopped.status         = PoseEvaluation::Status::Canceled;
        verifier.set(Pose{-4, 0}, stopped);

        const VerifiedSearchResult r = search_verified(legal, scorer, verifier, k, never_stop, no_progress);

        REQUIRE(r.outcome == VerifiedSearchResult::Outcome::Canceled);
        REQUIRE(r.selected.pose.is_root());
    }
}

namespace {

// One captured plate: the selected object with two instances, a neighbour object nobody selected, a
// plate config, the plate's printable ground and one exclusion volume. A Model copy keeps every
// ObjectID, so the "live" side of a comparison is this input copied and then edited in one place.
AutoTilt::EvaluationInput captured_plate()
{
    AutoTilt::PlateInput plate;
    plate.plate_index = 0;

    ModelObject *object = plate.model.add_object();
    object->name        = "selected";
    object->add_volume(make_cube(10., 10., 10.));
    object->add_instance()->set_offset(Vec3d(0., 0., 0.));
    object->add_instance()->set_offset(Vec3d(30., 0., 0.));

    ModelObject *neighbour = plate.model.add_object();
    neighbour->name        = "neighbour";
    neighbour->add_volume(make_cube(5., 5., 5.));
    neighbour->add_instance()->set_offset(Vec3d(-30., 0., 0.));

    plate.full_config.set_key_value("support_top_z_distance", new ConfigOptionFloat(0.2));
    plate.full_config.set_key_value("support_style", new ConfigOptionEnum<SupportMaterialStyle>(smsTreeSlim));
    plate.affected_instance_ids = { object->instances[0]->id(), object->instances[1]->id() };
    plate.printable_regions.emplace_back(Polygon{ Point::new_scale(-100., -100.), Point::new_scale(100., -100.),
                                                  Point::new_scale(100., 100.), Point::new_scale(-100., 100.) });
    plate.exclusions.emplace_back(Vec3d(60., 60., 0.), Vec3d(80., 80., 20.));
    plate.mesh_identity = AutoTilt::mesh_identities(plate.model);

    AutoTilt::EvaluationInput input;
    input.object_id = object->id();
    input.plates.push_back(std::move(plate));
    return input;
}

// The selected object of a captured input's first plate, by the id the input names.
ModelObject &selected_object(AutoTilt::EvaluationInput &input)
{
    for (ModelObject *object : input.plates.front().model.objects)
        if (object->id() == input.object_id)
            return *object;
    throw std::runtime_error("the captured input names no object of its own plate");
}

} // namespace

TEST_CASE("pose_admissible reads the plate's printable polygon and height, not its bounding box", "[AutoTilt]")
{
    AutoTilt::EvaluationInput input = captured_plate();

    // The L: everything inside the printable ground's 200 x 200 mm bounding box except the quadrant
    // at positive x and positive y, which a bounding-box test would have called inside.
    input.plates.front().printable_regions.clear();
    input.plates.front().printable_regions.emplace_back(Polygon{ Point::new_scale(-100., -100.), Point::new_scale(100., -100.),
                                                                 Point::new_scale(100., 0.), Point::new_scale(0., 0.),
                                                                 Point::new_scale(0., 100.), Point::new_scale(-100., 100.) });

    // make_cube builds from the origin corner, so an instance at offset o spans [o, o + 10] on every
    // axis; these two are the pair the captured plate names in affected_instance_ids.
    ModelObject &object = selected_object(input);
    object.instances[0]->set_offset(Vec3d(-50., -50., 0.));
    object.instances[1]->set_offset(Vec3d(50., -50., 0.));
    REQUIRE(AutoTilt::pose_admissible(input, AutoTilt::Pose{}));

    // Inside the L's bounding box, outside the L itself, and clear of the plate's one exclusion, so
    // only a test that reads the polygon refuses it.
    object.instances[1]->set_offset(Vec3d(30., 30., 0.));
    REQUIRE_FALSE(AutoTilt::pose_admissible(input, AutoTilt::Pose{}));

    // The height the printer reaches, against the 10 mm cube that stands under it.
    object.instances[1]->set_offset(Vec3d(50., -50., 0.));
    input.plates.front().printable_height_mm = 5.;
    REQUIRE_FALSE(AutoTilt::pose_admissible(input, AutoTilt::Pose{}));

    input.plates.front().printable_height_mm = 10.5;
    REQUIRE(AutoTilt::pose_admissible(input, AutoTilt::Pose{}));
}

TEST_CASE("evaluation_inputs_unchanged rejects a changed plate config or membership or plate geometry", "[AutoTilt]")
{
    const AutoTilt::EvaluationInput captured = captured_plate();

    SECTION("the same capture read twice is unchanged") {
        REQUIRE(AutoTilt::evaluation_inputs_unchanged(captured, captured));
    }
    SECTION("a config key only one side carries is a change") {
        // Complete equality, not the partial equals()/diff() pair, which ignore keys the other side
        // does not have: a plate override that appeared while the search ran is exactly that case.
        AutoTilt::EvaluationInput live = captured;
        live.plates.front().full_config.set_key_value("raft_layers", new ConfigOptionInt(3));
        REQUIRE_FALSE(AutoTilt::evaluation_inputs_unchanged(captured, live));
        REQUIRE_FALSE(AutoTilt::evaluation_inputs_unchanged(live, captured));
    }
    SECTION("the selected object leaving the plate is a change") {
        AutoTilt::EvaluationInput live = captured;
        live.plates.front().affected_instance_ids.pop_back();
        REQUIRE_FALSE(AutoTilt::evaluation_inputs_unchanged(captured, live));
    }
    SECTION("the same instances read in another order are not a change") {
        AutoTilt::EvaluationInput live = captured;
        std::swap(live.plates.front().affected_instance_ids[0], live.plates.front().affected_instance_ids[1]);
        REQUIRE(AutoTilt::evaluation_inputs_unchanged(captured, live));
    }
    SECTION("a plate the search never captured is a change") {
        AutoTilt::EvaluationInput live = captured;
        live.plates.push_back(live.plates.front());
        live.plates.back().plate_index = 1;
        REQUIRE_FALSE(AutoTilt::evaluation_inputs_unchanged(captured, live));
    }
    SECTION("the same instances printed on another plate are a change") {
        AutoTilt::EvaluationInput live = captured;
        live.plates.front().plate_index = 2;
        REQUIRE_FALSE(AutoTilt::evaluation_inputs_unchanged(captured, live));
    }
    SECTION("a plate that moved is a change") {
        AutoTilt::EvaluationInput live = captured;
        live.plates.front().plate_origin = Vec3d(240., 0., 0.);
        REQUIRE_FALSE(AutoTilt::evaluation_inputs_unchanged(captured, live));
    }
    SECTION("a moved exclusion volume is a change") {
        AutoTilt::EvaluationInput live = captured;
        live.plates.front().exclusions.front().translate(-40., 0., 0.);
        REQUIRE_FALSE(AutoTilt::evaluation_inputs_unchanged(captured, live));
    }
    SECTION("a printable ground of another shape is a change") {
        AutoTilt::EvaluationInput live = captured;
        live.plates.front().printable_regions.front().contour.points.back().x() -= scaled<coord_t>(20.);
        REQUIRE_FALSE(AutoTilt::evaluation_inputs_unchanged(captured, live));
    }
    SECTION("another object selected on the same plate is a change") {
        AutoTilt::EvaluationInput live = captured;
        live.object_id = live.plates.front().model.objects.back()->id();
        REQUIRE_FALSE(AutoTilt::evaluation_inputs_unchanged(captured, live));
    }
}

TEST_CASE("evaluation_inputs_unchanged compares object volume and instance records by ObjectID", "[AutoTilt]")
{
    const AutoTilt::EvaluationInput captured = captured_plate();

    // Every section edits one value of a copy that otherwise carries the captured ids, so what the
    // comparison answers to is that value and never the identity of a freshly allocated clone.
    AutoTilt::EvaluationInput live   = captured;
    ModelObject              &object = selected_object(live);

    SECTION("a copy that changed nothing is unchanged") {
        REQUIRE(AutoTilt::evaluation_inputs_unchanged(captured, live));
    }
    SECTION("the smallest instance move is a change") {
        Geometry::Transformation moved = object.instances[1]->get_transformation();
        moved.set_offset(moved.get_offset() + Vec3d(1e-12, 0., 0.));
        object.instances[1]->set_transformation(moved);
        REQUIRE_FALSE(AutoTilt::evaluation_inputs_unchanged(captured, live));
    }
    SECTION("the same instances listed in another order are not a change") {
        std::swap(object.instances[0], object.instances[1]);
        REQUIRE(AutoTilt::evaluation_inputs_unchanged(captured, live));
    }
    SECTION("an instance made unprintable is a change") {
        object.instances[1]->printable = false;
        REQUIRE_FALSE(AutoTilt::evaluation_inputs_unchanged(captured, live));
    }
    SECTION("an instance that no longer drops to the bed is a change") {
        object.instances[1]->auto_drop = false;
        REQUIRE_FALSE(AutoTilt::evaluation_inputs_unchanged(captured, live));
    }
    SECTION("an instance added to the object is a change") {
        object.add_instance()->set_offset(Vec3d(60., 0., 0.));
        REQUIRE_FALSE(AutoTilt::evaluation_inputs_unchanged(captured, live));
    }
    SECTION("a moved volume is a change") {
        object.volumes.front()->set_offset(Vec3d(0., 0., 1.));
        REQUIRE_FALSE(AutoTilt::evaluation_inputs_unchanged(captured, live));
    }
    SECTION("a model part turned into a modifier is a change") {
        object.volumes.front()->set_type(ModelVolumeType::PARAMETER_MODIFIER);
        REQUIRE_FALSE(AutoTilt::evaluation_inputs_unchanged(captured, live));
    }
    SECTION("an object override is a change") {
        object.config.set("support_top_z_distance", 0.3);
        REQUIRE_FALSE(AutoTilt::evaluation_inputs_unchanged(captured, live));
    }
    SECTION("a volume override is a change") {
        object.volumes.front()->config.set("wall_loops", 4);
        REQUIRE_FALSE(AutoTilt::evaluation_inputs_unchanged(captured, live));
    }
    SECTION("an object made unprintable is a change") {
        object.printable = false;
        REQUIRE_FALSE(AutoTilt::evaluation_inputs_unchanged(captured, live));
    }
    SECTION("a layer height profile is a change") {
        object.layer_height_profile.set(std::vector<coordf_t>{0., 0.2, 10., 0.08});
        REQUIRE_FALSE(AutoTilt::evaluation_inputs_unchanged(captured, live));
    }
    SECTION("a layer config range is a change") {
        object.layer_config_ranges[t_layer_height_range(0., 5.)].set("layer_height", 0.08);
        REQUIRE_FALSE(AutoTilt::evaluation_inputs_unchanged(captured, live));
    }
    SECTION("repainted supports are a change") {
        object.volumes.front()->supported_facets.reset();
        REQUIRE_FALSE(AutoTilt::evaluation_inputs_unchanged(captured, live));
    }
    SECTION("repainted seams are a change") {
        object.volumes.front()->seam_facets.reset();
        REQUIRE_FALSE(AutoTilt::evaluation_inputs_unchanged(captured, live));
    }
    SECTION("repainted material segmentation is a change") {
        object.volumes.front()->mmu_segmentation_facets.reset();
        REQUIRE_FALSE(AutoTilt::evaluation_inputs_unchanged(captured, live));
    }
    SECTION("repainted fuzzy skin is a change") {
        object.volumes.front()->fuzzy_skin_facets.reset();
        REQUIRE_FALSE(AutoTilt::evaluation_inputs_unchanged(captured, live));
    }
    SECTION("an added brim ear is a change") {
        object.brim_points.emplace_back(Vec3f(1.f, 1.f, 0.f), 5.f);
        REQUIRE_FALSE(AutoTilt::evaluation_inputs_unchanged(captured, live));
    }
    SECTION("an object that left the plate is a change") {
        live.plates.front().model.delete_object(size_t(1));
        live.plates.front().mesh_identity = AutoTilt::mesh_identities(live.plates.front().model);
        REQUIRE_FALSE(AutoTilt::evaluation_inputs_unchanged(captured, live));
    }
}

TEST_CASE("The stale-input oracle rejects a changed gap or a replaced mesh or a moved neighbour", "[AutoTilt]")
{
    const AutoTilt::EvaluationInput captured = captured_plate();

    SECTION("a support gap the measurement was not taken under is a change") {
        AutoTilt::EvaluationInput live = captured;
        live.plates.front().full_config.set_key_value("support_top_z_distance", new ConfigOptionFloat(0.1));
        REQUIRE_FALSE(AutoTilt::evaluation_inputs_unchanged(captured, live));
    }
    SECTION("a mesh replaced under the same volume id is a change") {
        // Same id, same size, same transform, same overrides: the recorded mesh pointer is the only
        // thing that says this volume is no longer the geometry the pose was measured on.
        AutoTilt::EvaluationInput live = captured;
        selected_object(live).volumes.front()->set_mesh(make_cube(10., 10., 10.));
        live.plates.front().mesh_identity = AutoTilt::mesh_identities(live.plates.front().model);
        REQUIRE_FALSE(AutoTilt::evaluation_inputs_unchanged(captured, live));
    }
    SECTION("a neighbour nobody selected moving is a change") {
        // The pose was scored against this plate's collision and print-order context, and the
        // neighbour is part of it even though no rotation would ever touch it.
        AutoTilt::EvaluationInput live = captured;
        live.plates.front().model.objects.back()->instances.front()->set_offset(Vec3d(-25., 0., 0.));
        REQUIRE_FALSE(AutoTilt::evaluation_inputs_unchanged(captured, live));
    }
}
