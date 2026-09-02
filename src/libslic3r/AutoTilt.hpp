#pragma once

#include <cstddef>
#include <functional>
#include <vector>

#include "libslic3r/ObjectID.hpp"
#include "libslic3r/Point.hpp"

namespace Slic3r { namespace AutoTilt {

// Every bound, step and threshold the search uses. Passed into grid(), search()
// and fragility_weight() so no loop bakes a number in.
struct Constants
{
    std::vector<double> tilts_deg {0, -5, -10, -15, -20, -25, -30, -35, -40};
    std::vector<double> leans_deg {-15, -10, -5, 0, 5, 10, 15};
    double threshold_base             = 0.10;   // fraction of the root score
    double threshold_per_degree       = 0.0025; // added per degree of |tilt|
    double negligible_volume_fraction = 0.0002; // of object volume
    double t_ref_mm                   = 2.0;
    double w_max                      = 8.0;
    double w_sharp                    = 4.0;
    double h_ref_mm                   = 0.12; // unit of account for both reported mm3 figures
    double h_search_min_mm            = 0.12; // floor of the search layer height
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

using StopPredicate = std::function<bool()>;
using ProgressSink  = std::function<void(size_t done, size_t total)>;

// Precondition: `legal` contains the root pose. Scores the root first, then the others in list order.
SearchResult search(const std::vector<Pose> &legal,
                    Scorer                  &scorer,
                    const Constants         &k,
                    const StopPredicate     &stop,
                    const ProgressSink      &progress);

struct InstanceSnapshot
{
    ObjectID    id;
    Transform3d matrix;
};

// Same count; every cached id present in live; and for each cached entry, the live entry matched by
// ObjectID, never by position, carries an exactly equal matrix (Eigen ==, no epsilon).
bool instances_unchanged(const std::vector<InstanceSnapshot> &cached, const std::vector<InstanceSnapshot> &live);

}} // namespace Slic3r::AutoTilt
