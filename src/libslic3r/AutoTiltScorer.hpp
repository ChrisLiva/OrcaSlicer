#pragma once

#include <functional>

#include "libslic3r/AutoTilt.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"

namespace Slic3r { namespace AutoTilt {

// Hands a piece of work to the main thread and returns once it has run. `Print::apply` mutates
// ObjectIDs and the model tree, so it may not run on a worker; everything else in score() may.
using MainThreadRunner = std::function<void(const std::function<void()> &)>;

// Scores a candidate pose by slicing a private clone of one ModelObject and measuring the tree
// support contact the slicer's own overhang detector finds on it.
class ContactScorer : public Scorer
{
public:
    // Main thread only (clones the object and bumps ObjectIDs). `full_config` is the print
    // preset's full config already overlaid with the plate config. Keeps instance 0 only.
    ContactScorer(const ModelObject &object, const DynamicPrintConfig &full_config, const Constants &k, MainThreadRunner run_on_main);

    Contact score(const Pose &pose) override; // worker thread; `Print::apply` goes through run_on_main

    double search_layer_height_mm() const { return m_h_search; } // max(print layer height, k.h_search_min_mm)

    // Off in every shipping path. On, score() charges each overhang only for the part that does not
    // already sit over solid object below it (spec §11); the validation harness measures whether that
    // correction tracks ground truth better than the plain sum does.
    bool shadow_correction = false;

    // No root/pivot accessors: the GUI job caches its own `root_matrix[i]` and `pivot[i]` per instance
    // from the live ModelObject, and this clone keeps instance 0 only, so exposing the scorer's copies
    // would create a second source of truth that nothing reads.
    const Print &print() const { return m_print; } // read-only view for tests

private:
    Constants          m_k;
    MainThreadRunner   m_run_on_main;
    Model              m_model;
    Print              m_print;
    DynamicPrintConfig m_config;
    Transform3d        m_root      = Transform3d::Identity();
    Vec3d              m_pivot     = Vec3d::Zero();
    BoundingBoxf3      m_root_box;  // the object's bounding box in the root pose, set once in the ctor
    double             m_h_search  = 0.;
};

}} // namespace Slic3r::AutoTilt
