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

double root_height_fraction(const Transform3d &root, const Transform3d &posed, const BoundingBoxf3 &root_box, const Vec3d &posed_point)
{
    // Undo the pose, then apply the root, so the point lands where it sat before the search moved it.
    const Vec3d  p = root * posed.inverse() * posed_point;
    // Read min/max rather than size(): a box built with equal Z is marked undefined, and its size() is not usable.
    const double h = root_box.max.z() - root_box.min.z();
    return h > 0. ? (p.z() - root_box.min.z()) / h : 0.;
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

namespace {

// Which objective the candidate first differs from the root on, and which side is the better one.
// `field` is the damage field compare_damage named; `volume` says the damage tied and only the
// material moved. Neither is set on a tie or where either tuple was never measured.
struct Objective
{
    SupportAnalysis::DamageField field  = SupportAnalysis::DamageField::None;
    bool                         volume = false;
    int                          order  = 0;
    bool differs() const { return field != SupportAnalysis::DamageField::None || volume; }
    const char *code() const;
};

const char *Objective::code() const
{
    switch (field) {
    case SupportAnalysis::DamageField::UnknownContacts:    return "gain_unknown_contacts";
    case SupportAnalysis::DamageField::InaccessibleGroups: return "gain_inaccessible_groups";
    case SupportAnalysis::DamageField::MaxGroupRisk:       return "gain_max_group_risk";
    case SupportAnalysis::DamageField::TotalGroupRisk:     return "gain_total_group_risk";
    case SupportAnalysis::DamageField::None:               break;
    }
    return volume ? "gain_volume" : "gain_none";
}

Objective first_objective(const Objectives &was, const Objectives &is)
{
    Objective out;
    out.order = compare_objectives(was, is);
    if (out.order == 0 || ! was.damage.available || ! is.damage.available)
        return out;
    out.field  = SupportAnalysis::compare_damage(was.damage, is.damage).field;
    out.volume = out.field == SupportAnalysis::DamageField::None;
    return out;
}

// What one verified candidate takes off the root, and what it has to take off to be worth applying.
struct Gain
{
    bool        improved = false;
    double      gain     = 0.;
    double      required = 0.;
    const char *code     = "gain_none";
};

// The gain gate, on the terms the estimate search already uses: the same
// threshold_base + threshold_per_degree * |tilt| percentage, charged against the first objective the
// candidate strictly improved. A count is not a percentage - one contact nobody can answer for, or
// one group nothing can reach, is worth removing whatever fraction of the total it is - so the
// discrete fields clear no gate. A continuous field is charged against the root's own reading of
// that same field, and a root reading of zero yields no percentage at all rather than an infinite
// one, so a candidate needs a strictly lower earlier objective to win there.
Gain gain_over_root(const Objectives &root, const Objectives &candidate, const Pose &pose, const Constants &k)
{
    Gain  out;
    out.required = k.threshold_base + k.threshold_per_degree * std::abs(pose.tilt_deg);

    const Objective d = first_objective(root, candidate);
    out.code          = d.code();
    if (! d.differs() || d.order > 0)
        return out; // a tie, an unmeasured side, or a regression in the first objective that differs

    const auto discrete = [](size_t was, size_t is) { return was > 0 ? double(was - is) / double(was) : 0.; };
    switch (d.field) {
    case SupportAnalysis::DamageField::UnknownContacts:
        out.gain     = discrete(root.damage.unknown_contacts, candidate.damage.unknown_contacts);
        out.required = 0.;
        out.improved = true;
        return out;
    case SupportAnalysis::DamageField::InaccessibleGroups:
        out.gain     = discrete(root.damage.inaccessible_groups, candidate.damage.inaccessible_groups);
        out.required = 0.;
        out.improved = true;
        return out;
    case SupportAnalysis::DamageField::MaxGroupRisk:
        out.gain = root.damage.max_group_risk > 0. ?
            (root.damage.max_group_risk - candidate.damage.max_group_risk) / root.damage.max_group_risk : 0.;
        break;
    case SupportAnalysis::DamageField::TotalGroupRisk:
        out.gain = root.damage.total_group_risk > 0. ?
            (root.damage.total_group_risk - candidate.damage.total_group_risk) / root.damage.total_group_risk : 0.;
        break;
    case SupportAnalysis::DamageField::None:
        // the damage tied and only the material moved
        out.gain = root.volume_mm3 > 0. ? (root.volume_mm3 - candidate.volume_mm3) / root.volume_mm3 : 0.;
        break;
    }
    out.improved = out.gain >= out.required;
    return out;
}

// Whether one verified pose may stand against the root at all. Coverage and stability are
// constraints, not scores: measured in full, no required region left open, nothing standing on air,
// and the centroid over what it stands on. Compared instance by instance, in capture order.
bool candidate_admissible(const PoseEvaluation &root, const PoseEvaluation &candidate)
{
    if (candidate.status != PoseEvaluation::Status::Complete)
        return false;
    if (candidate.instances.empty() || candidate.instances.size() != root.instances.size())
        return false;
    for (size_t i = 0; i < candidate.instances.size(); ++ i) {
        const SupportAnalysis::Report &report = candidate.instances[i];
        if (report.status != SupportAnalysis::Report::Status::Complete || ! report.missing_anchor_ids.empty())
            return false;
        if (! SupportAnalysis::stability_admissible(report.stability))
            return false;
        // Ids are compared only inside one pose's source problem: stability_no_worse matches groups
        // by id where the two CoverageKeys are equal and compares only counts and normalized metrics
        // where they are not, which is what two different poses always are.
        if (! SupportAnalysis::stability_no_worse(root.instances[i], report))
            return false;
    }
    return true;
}

void add_reason(VerifiedSearchResult &out, const char *code)
{
    if (std::find(out.reason_codes.begin(), out.reason_codes.end(), code) == out.reason_codes.end())
        out.reason_codes.emplace_back(code);
}

// One cheap-scored pose, kept with the place it holds in the legal list so the shortlist order can
// fall back on it.
struct Cheap
{
    Pose   pose;
    double score = 0.;
    size_t order = 0;
};

// One verified candidate that cleared admissibility, with what it measured and what it takes off the
// root.
struct Candidate
{
    PoseEvaluation evaluation;
    Objectives     objectives;
    Gain           gain;
    size_t         order = 0;
};

// Lower objectives win; a tie goes to the pose closer to the root; a tie again to the earlier pose
// in the grid.
bool ranks_before(const Candidate &lhs, const Candidate &rhs)
{
    const int order = compare_objectives(rhs.objectives, lhs.objectives);
    if (order != 0)
        return order < 0;
    const double dl = lhs.evaluation.pose.deviation_deg(), dr = rhs.evaluation.pose.deviation_deg();
    if (dl != dr)
        return dl < dr;
    return lhs.order < rhs.order;
}

} // namespace

Objectives objectives(const PoseEvaluation &evaluation)
{
    Objectives out;
    // A pose that was not measured in full is not a pose with zero damage: nothing about it is
    // comparable, and `available` says so rather than a field standing in for the answer.
    if (evaluation.status != PoseEvaluation::Status::Complete || evaluation.instances.empty())
        return out;
    for (const SupportAnalysis::Report &report : evaluation.instances) {
        // A reading that is not a number is not a low one: an available field has to be finite, so a
        // domain that came back unreadable leaves the whole tuple unavailable rather than comparing
        // as a tie against everything.
        if (! report.damage.available ||
            ! std::isfinite(report.damage.max_group_risk) || ! std::isfinite(report.damage.total_group_risk))
            return Objectives{};
        out.damage.unknown_contacts    += report.damage.unknown_contacts;
        out.damage.inaccessible_groups += report.damage.inaccessible_groups;
        out.damage.max_group_risk       = std::max(out.damage.max_group_risk, report.damage.max_group_risk);
        out.damage.total_group_risk    += report.damage.total_group_risk;
    }
    out.volume_mm3 = evaluation.support_volume_mm3 + evaluation.raft_volume_mm3;
    if (! std::isfinite(out.volume_mm3))
        return Objectives{};
    out.damage.available = true;
    return out;
}

int compare_objectives(const Objectives &was, const Objectives &is)
{
    if (! was.damage.available || ! is.damage.available)
        return was.damage.available == is.damage.available ? 0 : (is.damage.available ? -1 : 1);
    const SupportAnalysis::DamageComparison d = SupportAnalysis::compare_damage(was.damage, is.damage);
    if (d.order != 0)
        return d.order;
    if (std::abs(is.volume_mm3 - was.volume_mm3) > SupportAnalysis::comparison_epsilon(was.volume_mm3, is.volume_mm3))
        return is.volume_mm3 < was.volume_mm3 ? -1 : 1;
    return 0;
}

VerifiedSearchResult search_verified(const std::vector<Pose> &legal,
                                     Scorer                  &scorer,
                                     Verifier                &verifier,
                                     const Constants         &k,
                                     const StopPredicate     &stop,
                                     const ProgressSink      &progress)
{
    VerifiedSearchResult result;

    const auto root_it = std::find_if(legal.begin(), legal.end(), [](const Pose &p) { return p.is_root(); });
    if (root_it == legal.end()) {
        result.outcome = VerifiedSearchResult::Outcome::VerificationUnavailable;
        add_reason(result, "no_root_pose");
        return result;
    }
    const Pose root_pose = *root_it;
    result.root.pose     = root_pose;
    result.selected      = result.root;

    // The budget, known before any of it is spent: the root, every other legal pose cheap-scored,
    // and the finalists that sweep can produce.
    const size_t others = legal.size() - 1;
    const size_t total  = 1 + others + std::min(verified_finalist_count, others);
    size_t       done   = 0;
    const auto   tick   = [&progress, &done, total]() {
        if (progress)
            progress(++ done, total);
    };
    const auto stopped = [&stop]() { return stop && stop(); };
    // The one answer to a stop, wherever it comes from: the object stays in the pose it already
    // holds, and the outcome says nothing was settled.
    const auto canceled = [&result]() {
        result.outcome  = VerifiedSearchResult::Outcome::Canceled;
        result.selected = result.root;
        add_reason(result, "canceled");
    };

    // Before any work at all: a search stopped here measured nothing, and nothing is what it says.
    if (stopped()) {
        canceled();
        return result;
    }

    // The root, once, and in full. The estimate search stops on a root whose cheap contact reading is
    // negligible; this one does not, because a fragile feature the bottom exclusion plane dropped
    // reads exactly like a root with nothing on it, and that is the case full evaluation is for.
    result.root      = verifier.evaluate(root_pose, stop);
    result.root.pose = root_pose;
    result.selected  = result.root;
    ++ result.verified;
    tick();
    switch (result.root.status) {
    case PoseEvaluation::Status::Complete:
        break;
    case PoseEvaluation::Status::Canceled:
        canceled();
        return result;
    case PoseEvaluation::Status::UnresolvedCoverage:
        // The object's own finding, not a candidate's: what the root pose leaves open is what the
        // user is told about, and no pose is applied to cover it up.
        result.outcome = VerifiedSearchResult::Outcome::UnresolvedCoverage;
        add_reason(result, "root_coverage_unresolved");
        return result;
    default:
        result.outcome = VerifiedSearchResult::Outcome::VerificationUnavailable;
        add_reason(result, "root_analysis_unavailable");
        return result;
    }

    // The cheap sweep over every other legal pose. It orders the shortlist and grants nothing else.
    std::vector<Cheap> cheap;
    cheap.reserve(others);
    for (size_t i = 0; i < legal.size(); ++ i) {
        if (legal[i].is_root())
            continue;
        if (stopped()) {
            canceled();
            return result;
        }
        const Contact c = scorer.score(legal[i]);
        ++ result.cheap_scored;
        tick();
        // A score that is not a number orders nothing, so the pose carrying it is not shortlisted.
        if (! std::isfinite(c.score_mm3)) {
            add_reason(result, "cheap_score_not_finite");
            continue;
        }
        cheap.push_back(Cheap{legal[i], c.score_mm3, i});
    }

    std::stable_sort(cheap.begin(), cheap.end(), [](const Cheap &a, const Cheap &b) {
        if (a.score != b.score)
            return a.score < b.score;
        const double da = a.pose.deviation_deg(), db = b.pose.deviation_deg();
        if (da != db)
            return da < db;
        return a.order < b.order;
    });

    std::vector<size_t> shortlist_order;
    for (const Cheap &c : cheap) {
        if (result.shortlist.size() >= verified_finalist_count)
            break;
        if (std::find(result.shortlist.begin(), result.shortlist.end(), c.pose) != result.shortlist.end())
            continue; // the same pose twice is one finalist, not two
        result.shortlist.push_back(c.pose);
        shortlist_order.push_back(c.order);
    }

    // Each finalist once, at the settings the print would use. A finalist that fails is a finalist
    // that failed: it is not retried, and no pose past `verified_finalist_count` takes its place.
    std::vector<Candidate> admissible;
    const Objectives       root_objectives = objectives(result.root);
    for (size_t i = 0; i < result.shortlist.size(); ++ i) {
        if (stopped()) {
            canceled();
            return result;
        }
        PoseEvaluation evaluation = verifier.evaluate(result.shortlist[i], stop);
        evaluation.pose           = result.shortlist[i];
        ++ result.verified;
        tick();
        if (evaluation.status == PoseEvaluation::Status::Canceled) {
            canceled();
            return result;
        }
        if (! candidate_admissible(result.root, evaluation)) {
            add_reason(result, "candidate_inadmissible");
            continue;
        }
        Candidate candidate;
        candidate.objectives = objectives(evaluation);
        candidate.gain       = gain_over_root(root_objectives, candidate.objectives, evaluation.pose, k);
        candidate.order      = shortlist_order[i];
        candidate.evaluation = std::move(evaluation);
        admissible.push_back(std::move(candidate));
    }

    // Rank inside the set that cleared its own gate, never over the whole shortlist: a pose with
    // better objectives that did not earn its tilt must not keep a pose that did out of the answer.
    const Candidate *best = nullptr;
    for (const Candidate &candidate : admissible)
        if (candidate.gain.improved && (best == nullptr || ranks_before(candidate, *best)))
            best = &candidate;

    if (best != nullptr) {
        result.outcome              = VerifiedSearchResult::Outcome::Improved;
        result.selected             = best->evaluation;
        result.improvement          = best->gain.gain;
        result.required_improvement = best->gain.required;
        add_reason(result, best->gain.code);
        return result;
    }

    // Nothing worth applying. The best-ranked candidate that was measured at all is reported with
    // its own numbers, so the caller can say by how much the object stayed where it is.
    result.outcome = VerifiedSearchResult::Outcome::NoImprovement;
    for (const Candidate &candidate : admissible)
        if (best == nullptr || ranks_before(candidate, *best))
            best = &candidate;
    if (best != nullptr) {
        result.improvement          = best->gain.gain;
        result.required_improvement = best->gain.required;
    } else {
        result.required_improvement = k.threshold_base;
        add_reason(result, "no_candidate_measured");
    }
    return result;
}

}} // namespace Slic3r::AutoTilt
