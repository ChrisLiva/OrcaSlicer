#pragma once

#include <atomic>
#include <cstddef>
#include <limits>
#include <memory>
#include <mutex>
#include <vector>

#include "libslic3r/AutoTilt.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/ObjectID.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Print.hpp"

namespace Slic3r { namespace AutoTilt {

// Which mesh one volume's id stood for at capture time. The mesh behind a ModelVolume is immutable
// and replaced wholesale, so the pointer is the mesh's identity; nothing else about a volume changes
// when the user swaps its geometry for another of the same size. Recorded from the live model rather
// than read back off a captured copy, so the comparison never rests on where a copy allocated.
struct VolumeMeshIdentity
{
    ObjectID            volume_id;
    const TriangleMesh *mesh = nullptr;
};

// The mesh every volume of `model` points at right now, in volume-id order.
std::vector<VolumeMeshIdentity> mesh_identities(const Model &model);

// One plate as it stood when the search was started: the whole plate's model, so the collision and
// print-order context of the pose is the real one, the config that plate would slice under, the
// instances of the selected object that sit on it, and the geometry a candidate has to stay inside.
// `printable_regions` is the plate's printable ground, which need not be a rectangle; `exclusions`
// are the volumes on it nothing may reach into.
struct PlateInput
{
    size_t                          plate_index = 0;
    // Where the plate sits in the scene, in mm; instances on it carry this offset.
    Vec3d                           plate_origin = Vec3d::Zero();
    Model                           model;
    DynamicPrintConfig              full_config;
    // PresetBundle::is_bbl_vendor() for the printer the plate slices on. Print::validate reads it through
    // Print::is_BBL_printer(), which a bare Print leaves false.
    bool                            bbl_printer = false;
    std::vector<ObjectID>           affected_instance_ids;
    ExPolygons                      printable_regions;
    std::vector<BoundingBoxf3>      exclusions;
    // Top of the plate's build volume; infinity where no height binds.
    double                          printable_height_mm = std::numeric_limits<double>::infinity();
    // The meshes `model`'s volumes stood for when the plate was captured, taken from the live model.
    std::vector<VolumeMeshIdentity> mesh_identity;
};

// Every plate the selected object has instances on. A pose is one answer over all of them, because
// one rotation moves every instance of the object at once.
struct EvaluationInput
{
    ObjectID                object_id;
    std::vector<PlateInput> plates;
};

// Whether the scene a result was measured on is still the scene it would be applied to. Every value
// the measurement rests on is compared: the plate a candidate has to fit, the config it would slice
// under - whole, so a key that only one side carries is a difference - and the printer vendor flag
// that validates it, the mesh behind each volume,
// every object, volume and instance transform, override, paint timestamp, layer profile and range,
// and which plate each instance sits on. Instances and affected ids are matched by id and sorted
// before comparison, so reordering those is not a change, while volumes compare in list order; a
// missing id, an extra one, or a single differing value is. A pose measured against a `captured`
// this rejects may not be applied.
bool evaluation_inputs_unchanged(const EvaluationInput &captured, const EvaluationInput &live);

// Where every affected instance of `input` stands under `pose`, over all of its plates, one entry
// per affected id in capture order. The requested tilt and lean are applied about that instance's
// own pivot on top of that instance's own root matrix, so whatever the root carried - its place on
// the plate, its scale, its mirror - is still in the result; an auto-dropped instance is then
// translated by its own transformed model-part hull minimum, so every copy lands on the plate by its
// own geometry rather than by instance zero's. An id no captured model carries yields no entry, so a
// caller can tell a model that moved under it from one that did not, and every instance not named is
// left exactly where it is. The private evaluation, the final fit check and the GUI application all
// apply these matrices and no others.
std::vector<InstanceSnapshot> posed_instances(const EvaluationInput &input, const Pose &pose);

// The model-part convex hulls of `snapshot`'s instance under `snapshot.matrix`, in the model that
// carries that instance: the box a plate's printable ground and exclusion volumes are read against.
// Undefined where the model carries no such instance or the object has no model part.
BoundingBoxf3 posed_hull_box(const Model &model, const InstanceSnapshot &snapshot);

// The plan footprint of one object's model parts under `matrix`: the convex hull of every model-part
// convex hull's vertices projected to the plate, in scaled print coordinates. Read against a plate's
// printable ground, which need not be a rectangle, so a box would not do.
Polygon posed_footprint(const ModelObject &object, const Transform3d &matrix);

// Which constraint of one plate a posed instance breaks, or nullptr where it fits:
// "outside_printable_region" where the footprint leaves a non-empty printable_regions or the hull box
// rises above printable_height_mm, "exclusion_area" where the hull box intersects an exclusion.
// Checked in that order.
const char *plate_refusal(const ExPolygons &printable_regions, const std::vector<BoundingBoxf3> &exclusions,
                          double printable_height_mm, const Polygon &footprint, const BoundingBoxf3 &hull_box);

// Whether every affected instance of every captured plate fits its plate under pose: each affected id
// resolves to an instance the plate's model carries with a defined hull box, and plate_refusal is
// nullptr for all of them.
bool pose_admissible(const EvaluationInput &input, const Pose &pose);

// Which support generator the affected instances of a captured input would actually run. Resolved
// through SupportParameters over each instance's own effective object config, never off the raw
// `support_style` value, which leaves the unset "default" pointing at nothing. Mixed says the
// affected instances disagree, and an operation whose instances disagree is one no single search
// answers for: a verified legacy answer cannot be claimed for the Organic ones, and rotating only
// some of them is not an option because one pose moves every instance of the object at once. Unknown
// says at least one affected instance runs no tree support at all, or could not be resolved.
enum class SupportGenerator { Legacy, Organic, Mixed, Unknown };

// Main thread: this applies each captured plate to a throwaway Print, which hands out ObjectIDs.
SupportGenerator affected_support_generator(const EvaluationInput &input);

// Measures one pose by slicing the captured plates whole, under their own settings, and reading the
// support the generator actually emitted. Private models and private Prints throughout: nothing it
// owns is the model the user is looking at, and no setting of that model is flattened on the way in.
class GeneratedEvaluator : public Verifier
{
public:
    GeneratedEvaluator(const EvaluationInput &input, MainThreadRunner run_on_main);

    // Worker thread. Model cloning and `Print::apply` go through the injected runner, as they must;
    // the geometric admissibility tests, slicing and the measurement all run right here.
    PoseEvaluation evaluate(const Pose &pose, const StopPredicate &stop) override;

    // Any thread. Stops the Print that is processing right now, and every pose started after it.
    void cancel();

    // Read-only views of what the last evaluate() posed and sliced, one per captured plate in
    // capture order. For tests: nothing in the search reads them.
    const Model &model(size_t plate) const { return *m_models.at(plate); }
    const Print &print(size_t plate) const { return *m_prints.at(plate); }
    // Tests install a status callback through this.
    Print &print(size_t plate) { return *m_prints.at(plate); }

private:
    bool stopped(const StopPredicate &stop) const;

    EvaluationInput                     m_input;
    MainThreadRunner                    m_run_on_main;
    std::vector<std::unique_ptr<Model>> m_models;
    std::vector<std::unique_ptr<Print>> m_prints;

    // The latch that stops every pose started after a cancellation. Atomic, so a worker deep in a
    // pose reads it without taking the mutex the canceling thread may be holding.
    std::atomic<bool>                   m_cancel_requested{false};
    // The Print a worker is processing right now, published under the mutex so cancel() can reach it
    // from another thread and never reaches one that has already been destroyed.
    mutable std::mutex                  m_mutex;
    Print                              *m_active = nullptr;
};

}} // namespace Slic3r::AutoTilt
