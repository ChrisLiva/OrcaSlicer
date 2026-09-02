#include "AutoTilt.hpp"

#include <algorithm>
#include <cmath>

#include "Geometry.hpp"

namespace Slic3r { namespace AutoTilt {

bool Pose::is_root() const { return this->tilt_deg == 0. && this->lean_deg == 0.; }

double Pose::deviation_deg() const { return std::sqrt(this->tilt_deg * this->tilt_deg + this->lean_deg * this->lean_deg); }

bool operator==(const Pose &lhs, const Pose &rhs) { return lhs.tilt_deg == rhs.tilt_deg && lhs.lean_deg == rhs.lean_deg; }

std::vector<Pose> grid(const Constants &k)
{
    std::vector<Pose> out;
    out.reserve(k.tilts_deg.size() * k.leans_deg.size());
    out.push_back(Pose{});
    for (double tilt : k.tilts_deg)
        for (double lean : k.leans_deg) {
            const Pose p{tilt, lean};
            if (! p.is_root())
                out.push_back(p);
        }
    return out;
}

Transform3d candidate_transform(const Transform3d &root, const Vec3d &pivot, const Pose &pose)
{
    // Rotate about the pivot in world space, then keep whatever the instance already carries
    // (mirroring and non-uniform scale included) by post-multiplying the root matrix.
    const Vec3d rotation_rad(Geometry::deg2rad(pose.tilt_deg), Geometry::deg2rad(pose.lean_deg), 0.);
    return Geometry::translation_transform(pivot) * Geometry::rotation_transform(rotation_rad) *
           Geometry::translation_transform(- pivot) * root;
}

double fragility_weight(double area_mm2, double perimeter_mm, bool type_floor, const Constants &k)
{
    if (perimeter_mm <= 0.)
        return 1.;
    // Mean thickness of a slab of the given area and perimeter; the thinner it is, the more a
    // detaching support hurts, so the weight rises as t falls below t_ref.
    const double t = 2. * area_mm2 / perimeter_mm;
    double       w = t > 0. ? std::min(std::max(k.t_ref_mm / t, 1.), k.w_max) : k.w_max;
    if (type_floor)
        w = std::max(w, k.w_sharp);
    return w;
}

namespace {

struct Record
{
    Pose    pose;
    Contact contact;
};

// Lower score wins; equal scores go to the pose closer to the root; equal again keeps the earlier one.
bool ranks_before(const Record &lhs, const Record &rhs)
{
    if (lhs.contact.score_mm3 != rhs.contact.score_mm3)
        return lhs.contact.score_mm3 < rhs.contact.score_mm3;
    return lhs.pose.deviation_deg() < rhs.pose.deviation_deg();
}

// Fraction of the root score a pose removes. A non-positive root leaves nothing to improve on.
double gain(const Contact &root, const Contact &candidate)
{
    return root.score_mm3 > 0. ? (root.score_mm3 - candidate.score_mm3) / root.score_mm3 : 0.;
}

// The gain a pose has to clear to be worth applying; steeper tilts have to earn more.
double required_gain(const Pose &pose, const Constants &k)
{
    return k.threshold_base + k.threshold_per_degree * std::abs(pose.tilt_deg);
}

} // namespace

SearchResult search(const std::vector<Pose> &legal,
                    Scorer                  &scorer,
                    const Constants         &k,
                    const StopPredicate     &stop,
                    const ProgressSink      &progress)
{
    SearchResult result;
    std::vector<Record> records;
    records.reserve(legal.size());

    // The root anchors every comparison below, so it is scored first no matter where it sits.
    const auto root_it = std::find_if(legal.begin(), legal.end(), [](const Pose &p) { return p.is_root(); });
    if (root_it == legal.end())
        return result;

    const Pose root_pose = *root_it;
    result.root          = scorer.score(root_pose);
    result.evaluated     = 1;
    records.push_back(Record{root_pose, result.root});
    if (progress)
        progress(result.evaluated, legal.size());

    // Too little support contact to be worth moving the object; nothing below can beat it.
    if (result.root.volume_mm3 < k.negligible_volume_fraction * result.root.object_volume_mm3) {
        result.outcome              = SearchResult::Outcome::BelowFloor;
        result.best                 = root_pose;
        result.best_contact         = result.root;
        result.required_improvement = required_gain(root_pose, k);
        return result;
    }

    for (const Pose &p : legal) {
        if (p.is_root())
            continue;
        if (stop && stop()) {
            result.outcome      = SearchResult::Outcome::Canceled;
            result.best         = root_pose;
            result.best_contact = result.root;
            return result;
        }
        const Contact c = scorer.score(p);
        ++ result.evaluated;
        records.push_back(Record{p, c});
        if (progress)
            progress(result.evaluated, legal.size());
    }

    // Filter to the poses whose gain clears their own threshold, then rank inside that set. The root
    // always has a zero gain against a positive threshold, so it is never admissible.
    const Record *best = nullptr;
    for (const Record &rec : records)
        if (gain(result.root, rec.contact) >= required_gain(rec.pose, k))
            if (best == nullptr || ranks_before(rec, *best))
                best = &rec;

    if (best != nullptr) {
        result.outcome = SearchResult::Outcome::Improved;
    } else {
        // Nothing worth applying; report the lowest-scoring pose so the caller can say by how much it missed.
        result.outcome = SearchResult::Outcome::NoImprovement;
        best           = &records.front();
        for (const Record &rec : records)
            if (ranks_before(rec, *best))
                best = &rec;
    }

    result.best                 = best->pose;
    result.best_contact         = best->contact;
    result.improvement          = gain(result.root, best->contact);
    result.required_improvement = required_gain(best->pose, k);
    return result;
}

bool instances_unchanged(const std::vector<InstanceSnapshot> &cached, const std::vector<InstanceSnapshot> &live)
{
    if (cached.size() != live.size())
        return false;
    for (const InstanceSnapshot &c : cached) {
        // Match by id, not by position: the caller may hand back the same instances in another order.
        const auto it = std::find_if(live.begin(), live.end(), [&c](const InstanceSnapshot &l) { return l.id == c.id; });
        if (it == live.end())
            return false;
        if (! (it->matrix.matrix() == c.matrix.matrix()))
            return false;
    }
    return true;
}

}} // namespace Slic3r::AutoTilt
