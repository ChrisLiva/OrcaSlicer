#pragma once

#include <cstddef>
#include <exception>
#include <memory>
#include <string>
#include <vector>

#include "Job.hpp"
#include "libslic3r/AutoTilt.hpp"
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

    // Main thread. Caches ids/matrices, builds the config, runs the fit pre-pass and clones the
    // object into the scorer. Returns false, after showing the relevant notification, when the job
    // must not be queued.
    bool prepare(Plater &plater);

    void process(Ctl &ctl) override;
    void finalize(bool canceled, std::exception_ptr &eptr) override;

private:
    // One transform per instance, always built from the cached root pose, never from the live matrix.
    std::vector<Transform3d> candidate_transforms(const AutoTilt::Pose &pose) const;
    // `text` with the pre-pass skipped clause appended when the pre-pass skipped at least one pose.
    std::string with_skipped_clause(const std::string &text) const;
    void        push_result(const std::string &text) const;

    AutoTilt::Constants                      m_k;
    Plater                                  *m_plater = nullptr;
    ObjectID                                 m_object_id;
    std::vector<AutoTilt::InstanceSnapshot>  m_instances;
    std::vector<Transform3d>                 m_root_matrix;
    std::vector<Vec3d>                       m_pivot;
    std::vector<AutoTilt::Pose>              m_legal;
    size_t                                   m_total_poses = 0;
    size_t                                   m_skipped     = 0;
    std::unique_ptr<AutoTilt::ContactScorer> m_scorer;
    AutoTilt::SearchResult                   m_result;
    // Read at call time by the main-thread runner handed to the scorer; null until process() sets it.
    Ctl                                     *m_ctl = nullptr;
};

}} // namespace Slic3r::GUI
