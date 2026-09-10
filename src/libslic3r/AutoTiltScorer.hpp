#pragma once

#include <functional>
#include <memory>
#include <vector>

#include "libslic3r/AutoTilt.hpp"
#include "libslic3r/AutoTiltEvaluation.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"

namespace Slic3r { namespace AutoTilt {

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

    struct PolygonRecord {
        size_t layer;            // object layer index, >= 1
        double print_z_mm, area_mm2, perimeter_mm, t_mm, weight, root_height_fraction;
        bool   type_floor, excluded;
    };
    // when set, score() appends (never clears) one row per loverhangs polygon on layers >= 1, excluded ones
    // included
    std::vector<PolygonRecord> *records = nullptr;

    // No root/pivot accessors: AutoTilt::posed_instances reads root and pivot per instance off the
    // captured model, and this clone keeps instance 0 only, so exposing the scorer's copies would
    // create a second source of truth that nothing reads.
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

// Cheap legacy ranking over every affected instance of every captured plate: one ContactScorer per
// instance the pose would move, each posed about its own root and its own pivot and sliced under its
// own plate's settings, and their contacts added up. What it produces orders a shortlist and grants
// nothing else - it is an estimate of contact the overhang detector found, not a measurement of the
// support the print would lay - so none of it is ever a number the user is shown.
class LegacyShortlistScorer : public Scorer
{
public:
    // The per-instance clones rewrite the model tree and hand out ObjectIDs, so the constructor does
    // that work through `run_on_main` and hands the same runner to each scorer it builds.
    LegacyShortlistScorer(const EvaluationInput &input, const Constants &k, MainThreadRunner run_on_main);

    Contact score(const Pose &pose) override; // worker thread; sums the instances in capture order

    size_t instance_count() const { return m_scorers.size(); }

private:
    std::vector<std::unique_ptr<ContactScorer>> m_scorers;
};

}} // namespace Slic3r::AutoTilt
