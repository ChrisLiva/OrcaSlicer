#pragma once

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ObjectID.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Support/SupportAnalysis.hpp"

namespace Slic3r { namespace AutoTilt {

// Hands a piece of work to the main thread and returns once it has run. `Print::apply` mutates
// ObjectIDs and the model tree, so it may not run on a worker; slicing and measuring may.
using MainThreadRunner = std::function<void(const std::function<void()> &)>;

// Every bound, step and threshold the search uses. Passed into grid(), search()
// and fragility_weight() so no loop bakes a number in.
struct Constants
{
    std::vector<double> tilts_deg {0, -2, -4, -6, -8, -10, -12, -14, -16, -18, -20};
    std::vector<double> leans_deg {-15, -10, -5, 0, 5, 10, 15};
    double threshold_base             = 0.05;   // fraction of the root score
    double threshold_per_degree       = 0.0025; // added per degree of |tilt|, so the -20 deg cap asks 10%
    double negligible_volume_fraction = 0.0002; // of object volume
    double t_ref_mm                   = 2.0;
    double w_max                      = 8.0;
    double w_sharp                    = 4.0;
    double h_ref_mm                   = 0.12; // unit of account for both reported mm3 figures
    double h_search_min_mm            = 0.12; // floor of the search layer height
    double bottom_exclusion_fraction  = 0.20; // contact whose centroid sits below this fraction of the root pose's height is ignored; 0 turns it off
};

struct Pose
{
    double tilt_deg = 0;
    double lean_deg = 0;

    bool   is_root() const;
    double deviation_deg() const; // sqrt(tilt^2 + lean^2)
};

bool operator==(const Pose &lhs, const Pose &rhs);

// Root pose first, then tilt-major over Constants (tilt outer loop, lean inner), skipping the root.
std::vector<Pose> grid(const Constants &k);

// Translate(pivot) * rotation_transform({rad(tilt), rad(lean), 0}) * Translate(-pivot) * root
Transform3d candidate_transform(const Transform3d &root, const Vec3d &pivot, const Pose &pose);

// Height of a posed-world point in the root pose, as a fraction of the root pose's Z extent:
// (z of root * posed^-1 * point - root_box.min.z) / (root_box.max.z - root_box.min.z). A zero-height box yields 0.
double root_height_fraction(const Transform3d &root, const Transform3d &posed, const BoundingBoxf3 &root_box, const Vec3d &posed_point);

// t = 2*area/perimeter; w = clamp(t_ref/t, 1, w_max); if type_floor, w = max(w, w_sharp).
// perimeter <= 0 yields 1.
double fragility_weight(double area_mm2, double perimeter_mm, bool type_floor, const Constants &k);

struct Contact
{
    double score_mm3         = 0;
    double volume_mm3        = 0;
    double object_volume_mm3 = 0;
};

class Scorer
{
public:
    virtual ~Scorer()               = default;
    virtual Contact score(const Pose &) = 0;
};

struct SearchResult
{
    enum class Outcome { Canceled, BelowFloor, NoImprovement, Improved };

    Outcome outcome = Outcome::Canceled;
    Contact root;
    Pose    best;         // the applied pose when Improved; the lowest-score pose otherwise
    Contact best_contact;
    double  improvement          = 0; // (root.score - best.score) / root.score
    double  required_improvement = 0; // threshold_base + threshold_per_degree * |best.tilt|
    size_t  evaluated            = 0; // candidates actually scored
};

// What one pose came to when it was sliced under the actual settings and the support the slicer
// generated for it was measured. `instances` carries one measurement per affected physical instance,
// in the order the instances were captured, so a safer instance never stands in for one that got
// worse; the two volumes are those measurements summed over the instances that print them. Complete
// says every affected instance was measured in full; UnresolvedCoverage says the generated support
// left some required region of some instance open; Invalid says the pose never reached processing;
// Unknown says something the evaluator does not answer for, the Organic style among them.
struct PoseEvaluation
{
    enum class Status { Complete, Canceled, Invalid, Unknown, UnresolvedCoverage };

    Status                               status = Status::Unknown;
    Pose                                 pose;
    std::vector<SupportAnalysis::Report> instances;
    double                               support_volume_mm3 = 0.;
    double                               raft_volume_mm3    = 0.;
    std::vector<std::string>             reason_codes;
};

using StopPredicate = std::function<bool()>;
using ProgressSink  = std::function<void(size_t done, size_t total)>;

// Precondition: `legal` contains the root pose. Scores the root first, then the others in list order.
SearchResult search(const std::vector<Pose> &legal,
                    Scorer                  &scorer,
                    const Constants         &k,
                    const StopPredicate     &stop,
                    const ProgressSink      &progress);

// One evaluated pose reduced to what the verified ranking compares. The Damage fields are aggregated
// over every affected instance - counts added up, the worst group the worst of them, the sums added -
// and the volume is the support plus the raft every instance prints. `damage.available` is false
// unless the pose measured every affected instance in full, because a domain nothing measured is not
// damage of zero.
struct Objectives
{
    SupportAnalysis::Damage damage;
    double                  volume_mm3 = 0.; // support plus raft
};

Objectives objectives(const PoseEvaluation &evaluation);

// Lexicographic over the Damage fields in the order they are written, then over the volume: negative
// where `is` is the better of the two, positive where it is the worse, zero where they tie. Counts
// compare exactly, the weights and the volume within the numeric tolerance. An unavailable tuple is
// never the better one, and two of them tie, because neither was measured.
int compare_objectives(const Objectives &was, const Objectives &is);

// Measures one pose under the actual print settings. The search holds one and never assumes what it
// answers: a pose it cannot measure is a pose that cannot win.
class Verifier
{
public:
    virtual ~Verifier() = default;
    virtual PoseEvaluation evaluate(const Pose &, const StopPredicate &) = 0;
};

// How many cheap-scored poses the verified search takes to full evaluation.
constexpr size_t verified_finalist_count = 5;

// What the verified search settled on. `root` is the one evaluation of the pose the object already
// stands in, taken once and compared against throughout; `selected` is the pose to apply on Improved
// and a copy of `root` on every other outcome. `shortlist` is the finalists the cheap sweep chose,
// best cheap score first, and the two counts say how much was actually measured, so no caller can
// read a `verified_finalist_count`-finalist answer as a swept grid. Canceled says the search stopped
// before it settled;
// VerificationUnavailable says the root itself was never measured. Coverage decides no outcome: a
// root or candidate that leaves a required region open is ranked on what it measured.
struct VerifiedSearchResult
{
    enum class Outcome { Canceled, NoImprovement, Improved, VerificationUnavailable };

    Outcome                  outcome = Outcome::Canceled;
    PoseEvaluation           root;
    PoseEvaluation           selected;
    std::vector<Pose>        shortlist;
    size_t                   cheap_scored         = 0;  // poses the cheap scorer answered for
    size_t                   verified             = 0;  // full evaluations run, the root included
    double                   improvement          = 0.; // of the best-ranked verified candidate
    double                   required_improvement = 0.;
    std::vector<std::string> reason_codes;
};

// Answers VerificationUnavailable with "no_root_pose" unless `legal` contains the root pose. Evaluates
// the root in full first, cheap-scores every other legal pose, and verifies the best
// `verified_finalist_count` of them under the actual settings. The cheap score orders the shortlist
// and grants nothing else: a pose wins only on what the full evaluation measured.
VerifiedSearchResult search_verified(const std::vector<Pose> &legal,
                                     Scorer                  &scorer,
                                     Verifier                &verifier,
                                     const Constants         &k,
                                     const StopPredicate     &stop,
                                     const ProgressSink      &progress);

struct InstanceSnapshot
{
    ObjectID    id;
    Transform3d matrix;
};

}} // namespace Slic3r::AutoTilt
