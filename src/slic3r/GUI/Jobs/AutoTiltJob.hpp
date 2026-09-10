#pragma once

#include <cstddef>
#include <exception>
#include <memory>
#include <string>
#include <vector>

#include "Job.hpp"
#include "libslic3r/AutoTilt.hpp"
#include "libslic3r/AutoTiltEvaluation.hpp"
#include "libslic3r/AutoTiltScorer.hpp"
#include "libslic3r/ObjectID.hpp"
#include "libslic3r/Point.hpp"

namespace Slic3r { namespace GUI {

class Plater;

// Searches the tilt/lean grid for a pose that cuts tree-support contact on one object, then applies
// the winner. Only ids, matrices and value copies cross the worker boundary; finalize() re-resolves
// the live model tree by ObjectID on the main thread and re-checks every precondition before it
// touches anything.
class AutoTiltJob : public Job
{
public:
    // State half of the precondition (supports on, tree style, every instance auto-dropped and on a
    // plate, root pose fits) for the object at obj_idx. Main thread.
    static bool state_preconditions_hold(Plater &plater, int obj_idx);
    // Menu greying: worker idle, exactly one object selected, and state_preconditions_hold. Main thread.
    static bool can_start(Plater &plater);

    // Main thread. Captures the affected plates whole, builds the config, runs the fit pre-pass and
    // clones the object into the scorer. Returns false, after showing the relevant notification, when
    // the job must not be queued.
    bool prepare(Plater &plater);

    void process(Ctl &ctl) override;
    void finalize(bool canceled, std::exception_ptr &eptr) override;

private:
    // Which search answered, so finalize() applies what that search settled on and never reads an
    // estimate as a measurement. VerificationUnavailable is the mixed-generator case: the affected
    // instances do not all run the same support generator, so there is nothing to apply.
    enum class ResultKind { None, Estimated, Verified, VerificationUnavailable };

    // `text` with the pre-pass skipped clause appended when the pre-pass skipped at least one pose.
    std::string with_skipped_clause(const std::string &text) const;
    void        push_result(const std::string &text) const;
    // The notification one applied Organic pose deserves: the cheap contact estimate that picked it,
    // labelled as the estimate it is.
    std::string estimated_text(const AutoTilt::Pose &pose, double improvement) const;
    // The notification one applied legacy pose deserves: the support volume the slicer actually
    // generated for the root and for the winner, which way the estimated removal risk moved, the pose
    // itself, and how much of the grid was measured.
    std::string verified_text() const;
    // " Reason: <phrase>." for the first reason code the verified search or its root evaluation
    // recorded, empty when neither recorded one worth showing.
    std::string reason_detail() const;
    // Main thread. Moves every affected instance onto `pose` when the live scene still matches the
    // capture the pose was measured on, and says whether it did.
    bool        apply_pose(const AutoTilt::Pose &pose);

    AutoTilt::Constants                      m_k;
    Plater                                  *m_plater = nullptr;
    ObjectID                                 m_object_id;
    // Every plate the selected object stands on, as it stood when the search started: the model, the
    // config that plate would slice under, the affected ids, and the ground a candidate has to fit.
    AutoTilt::EvaluationInput                m_captured;
    AutoTilt::SupportGenerator               m_generator = AutoTilt::SupportGenerator::Unknown;
    std::vector<AutoTilt::Pose>              m_legal;
    size_t                                   m_total_poses = 0;
    size_t                                   m_skipped     = 0;
    // The shortlist scorer the generator on this object calls for: the legacy branch ranks over
    // every affected instance of every captured plate, Organic over the one live object.
    std::unique_ptr<AutoTilt::Scorer>        m_scorer;
    ResultKind                               m_kind = ResultKind::None;
    AutoTilt::SearchResult                   m_result;   // Organic: the estimate
    AutoTilt::VerifiedSearchResult           m_verified; // legacy: measured under the actual settings
    // Read at call time by the main-thread runner handed to the scorer; null until process() sets it,
    // which is how that runner tells a request made while prepare() builds the scorer - already on
    // the main thread - from one score() makes on the worker.
    Ctl                                     *m_ctl = nullptr;
};

}} // namespace Slic3r::GUI
