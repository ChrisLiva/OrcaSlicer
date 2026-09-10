#include "AutoTiltEvaluation.hpp"

#include <algorithm>
#include <atomic>
#include <utility>

#include "ClipperUtils.hpp"
#include "Geometry.hpp"
#include "Geometry/ConvexHull.hpp"
#include "Layer.hpp"
#include "PrintBase.hpp"
#include "PrintConfig.hpp"
#include "Support/SupportParameters.hpp"
#include "TriangleMesh.hpp"
#include "libslic3r.h"

namespace Slic3r { namespace AutoTilt {

Polygon posed_footprint(const ModelObject &object, const Transform3d &matrix)
{
    Points points;
    for (const ModelVolume *volume : object.volumes) {
        if (! volume->is_model_part())
            continue;
        const TriangleMesh &hull  = volume->get_convex_hull();
        const Transform3d   world = matrix * volume->get_matrix();
        points.reserve(points.size() + hull.its.vertices.size());
        for (const Vec3f &vertex : hull.its.vertices) {
            const Vec3d point = world * vertex.cast<double>();
            points.emplace_back(scaled<coord_t>(point.x()), scaled<coord_t>(point.y()));
        }
    }
    return Geometry::convex_hull(points);
}

const char *plate_refusal(const ExPolygons &printable_regions, const std::vector<BoundingBoxf3> &exclusions,
                          double printable_height_mm, const Polygon &footprint, const BoundingBoxf3 &hull_box)
{
    if ((! printable_regions.empty() && ! diff(Polygons{ footprint }, to_polygons(printable_regions)).empty()) ||
        hull_box.max.z() > printable_height_mm)
        return "outside_printable_region";
    for (const BoundingBoxf3 &exclusion : exclusions)
        if (hull_box.intersects(exclusion))
            return "exclusion_area";
    return nullptr;
}

namespace {

// The object one instance id belongs to, or nullptr where the model carries no such instance.
const ModelObject *object_of_instance(const Model &model, const ObjectID &id)
{
    for (const ModelObject *object : model.objects)
        for (const ModelInstance *instance : object->instances)
            if (instance->id() == id)
                return object;
    return nullptr;
}

// Whether the generator this object would run is the legacy tree one this evaluator answers for.
// SupportParameters resolves the style, the unset "default" among them, so the rule is read from the
// one place that owns it rather than restated here.
bool legacy_tree_support(const PrintObject &object)
{
    if (! object.config().enable_support.value || ! is_tree(object.config().support_type.value))
        return false;
    return SupportParameters(object).support_style != smsTreeOrganic;
}

// The print object one posed instance ended up in, and which of that object's copies it is. Matched
// by ObjectID rather than by position: Print::apply groups instances into print objects by their
// transform, so a pose can move an instance from one of them to another.
const PrintObject *print_object_of_instance(const Print &print, const ObjectID &id, size_t &index)
{
    for (const PrintObject *object : print.objects())
        for (size_t i = 0; i < object->instances().size(); ++ i)
            if (object->instances()[i].model_instance != nullptr && object->instances()[i].model_instance->id() == id) {
                index = i;
                return object;
            }
    return nullptr;
}

// The bed adhesion the print laid under one physical instance, brought back into the object's own
// canonical unshifted instance frame. The support measurement was taken in that frame, and the brim
// is emitted in plate coordinates, so the instance's own shift and the plate's origin come back out
// here: one generated pass shared by several copies stands on different ground under each of them.
Polygons instance_adhesion(const Print &print, const PrintObject &object, size_t instance_index)
{
    const auto it = print.get_brimMapByInstance().find(ObjectInstanceID{ object.id(), instance_index });
    if (it == print.get_brimMapByInstance().end() || instance_index >= object.instances().size())
        return {};
    Polygons    adhesion = it->second.polygons_covered_by_width(0.f);
    const Vec3d origin   = print.get_plate_origin();
    const Point shift    = object.instances()[instance_index].shift_without_plate_offset() +
                           Point(scaled<coord_t>(origin.x()), scaled<coord_t>(origin.y()));
    for (Polygon &polygon : adhesion)
        polygon.translate(- shift);
    return adhesion;
}

// Adds `code` once: a pose that fails the same way on two plates says so once, so the codes read as
// a set of conditions rather than as a tally of instances.
void add_reason(PoseEvaluation &out, const char *code)
{
    if (std::find(out.reason_codes.begin(), out.reason_codes.end(), code) == out.reason_codes.end())
        out.reason_codes.emplace_back(code);
}

// The worse of two outcomes, so the whole pose reads as its weakest plate. Complete is the only
// outcome a pose can win on; everything below it keeps a pose out of the comparison, and Canceled
// outranks the rest because a canceled run measured nothing at all.
PoseEvaluation::Status worse(PoseEvaluation::Status a, PoseEvaluation::Status b)
{
    const auto rank = [](PoseEvaluation::Status status) {
        switch (status) {
        case PoseEvaluation::Status::Complete:           return 0;
        case PoseEvaluation::Status::UnresolvedCoverage: return 1;
        case PoseEvaluation::Status::Unknown:            return 2;
        case PoseEvaluation::Status::Invalid:            return 3;
        case PoseEvaluation::Status::Canceled:           return 4;
        }
        return 2;
    };
    return rank(a) >= rank(b) ? a : b;
}

// Publishes one Print as the evaluator's active one for the length of a scope and clears it on the
// way out, whether the scope ends by returning or by throwing. A cancellation that latched before the
// Print existed is applied to it as it is published, so the window between the latch and the
// publication loses nothing.
class ActivePrint
{
public:
    ActivePrint(std::mutex &mutex, Print *&slot, Print &print, const std::atomic<bool> &canceled) : m_mutex(mutex), m_slot(slot)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_slot = &print;
        if (canceled.load())
            print.cancel();
    }
    ~ActivePrint()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_slot = nullptr;
    }
    ActivePrint(const ActivePrint &)            = delete;
    ActivePrint &operator=(const ActivePrint &) = delete;

private:
    std::mutex &m_mutex;
    Print      *&m_slot;
};

// `ids` sorted, so a collection the UI reordered reads the same both times. Every collection this
// comparison walks is keyed by ObjectID, and nothing about a pose depends on the order they arrive
// in, so ordering is the one difference that is not a difference.
std::vector<ObjectID> sorted_ids(std::vector<ObjectID> ids)
{
    std::sort(ids.begin(), ids.end());
    return ids;
}

// Exact box equality, `defined` included: a box nobody set is not a box at the origin.
bool box_unchanged(const BoundingBoxf3 &a, const BoundingBoxf3 &b)
{
    return a.defined == b.defined && (! a.defined || (a.min == b.min && a.max == b.max));
}

// Everything about one plate that is not its model: which plate it is and where it sits, what it
// would slice under, which instances the pose may move, the ground it has to stay on, the volumes it
// may not reach into, and the mesh behind every volume id on it.
bool plate_shell_unchanged(const PlateInput &a, const PlateInput &b)
{
    if (a.plate_index != b.plate_index || a.plate_origin != b.plate_origin)
        return false;
    if (sorted_ids(a.affected_instance_ids) != sorted_ids(b.affected_instance_ids))
        return false;
    // Complete equality. `equals()` and `diff()` ignore keys the other side does not carry, so an
    // override that appeared or vanished while the search ran would read as no change at all.
    if (! (a.full_config == b.full_config))
        return false;
    if (a.printable_regions != b.printable_regions || a.printable_height_mm != b.printable_height_mm ||
        a.exclusions.size() != b.exclusions.size())
        return false;
    for (size_t i = 0; i < a.exclusions.size(); ++ i)
        if (! box_unchanged(a.exclusions[i], b.exclusions[i]))
            return false;
    // mesh_identities() sorts by volume id, so this compares the same volume on both sides.
    if (a.mesh_identity.size() != b.mesh_identity.size())
        return false;
    for (size_t i = 0; i < a.mesh_identity.size(); ++ i)
        if (a.mesh_identity[i].volume_id != b.mesh_identity[i].volume_id || a.mesh_identity[i].mesh != b.mesh_identity[i].mesh)
            return false;
    return true;
}

// One instance's own state, matched by id so a list the UI reordered still compares like with like.
// The matrix is compared exactly, Eigen's ==, because the pose is built on top of it and a rounding
// error's worth of drift is still a scene the measurement was not taken on.
bool instances_unchanged_by_id(const ModelObject &a, const ModelObject &b)
{
    if (a.instances.size() != b.instances.size())
        return false;
    std::vector<const ModelInstance *> left(a.instances.begin(), a.instances.end());
    std::vector<const ModelInstance *> right(b.instances.begin(), b.instances.end());
    const auto by_id = [](const ModelInstance *x, const ModelInstance *y) { return x->id() < y->id(); };
    std::sort(left.begin(), left.end(), by_id);
    std::sort(right.begin(), right.end(), by_id);
    for (size_t i = 0; i < left.size(); ++ i) {
        if (left[i]->id() != right[i]->id())
            return false;
        if (left[i]->get_transformation().get_matrix().matrix() != right[i]->get_transformation().get_matrix().matrix())
            return false;
        if (left[i]->printable != right[i]->printable || left[i]->auto_drop != right[i]->auto_drop)
            return false;
    }
    return true;
}

// The volumes of one object, in list order. Order is part of the comparison here rather than sorted
// away: the paint helpers below walk both volume lists positionally, so the captured order is the
// only order their answer can be read under, and a reordered list is reported as changed instead of
// silently compared against the wrong volume.
bool volumes_unchanged(const ModelObject &a, const ModelObject &b)
{
    if (a.volumes.size() != b.volumes.size())
        return false;
    for (size_t i = 0; i < a.volumes.size(); ++ i) {
        const ModelVolume &x = *a.volumes[i], &y = *b.volumes[i];
        if (x.id() != y.id() || x.type() != y.type())
            return false;
        if (x.get_matrix().matrix() != y.get_matrix().matrix())
            return false;
        if (! (x.config.get() == y.config.get()))
            return false;
    }
    return true;
}

// Everything one object carries that the measurement rests on. The paint of all five kinds is read
// through the helpers Print::apply itself uses, so the timestamps that decide a repaint are compared
// by the code that owns that rule rather than copied into a record here.
bool object_unchanged(const ModelObject &a, const ModelObject &b)
{
    if (a.id() != b.id() || a.printable != b.printable)
        return false;
    if (! (a.config.get() == b.config.get()))
        return false;
    if (a.layer_height_profile.get() != b.layer_height_profile.get() ||
        ! a.layer_height_profile.timestamp_matches(b.layer_height_profile))
        return false;
    if (a.layer_config_ranges.size() != b.layer_config_ranges.size())
        return false;
    for (auto left = a.layer_config_ranges.begin(), right = b.layer_config_ranges.begin(); left != a.layer_config_ranges.end();
         ++ left, ++ right)
        if (left->first != right->first || ! (left->second.get() == right->second.get()))
            return false;
    if (! volumes_unchanged(a, b))
        return false;
    // Synchronized volume lists are these helpers' precondition, and volumes_unchanged() has just
    // proved it: same count, same ids, same order, same types.
    if (model_custom_supports_data_changed(a, b) || model_custom_seam_data_changed(a, b) ||
        model_mmu_segmentation_data_changed(a, b) || model_fuzzy_skin_data_changed(a, b) ||
        model_brim_points_data_changed(a, b))
        return false;
    return instances_unchanged_by_id(a, b);
}

// The plate's whole model, objects in list order: that order is the print order a sequential plate
// runs in and the collision context every candidate was measured against, so it is a value here and
// not something to sort away.
bool plate_model_unchanged(const Model &a, const Model &b)
{
    if (a.objects.size() != b.objects.size())
        return false;
    for (size_t i = 0; i < a.objects.size(); ++ i)
        if (! object_unchanged(*a.objects[i], *b.objects[i]))
            return false;
    return true;
}

bool plate_unchanged(const PlateInput &a, const PlateInput &b)
{
    return plate_shell_unchanged(a, b) && plate_model_unchanged(a.model, b.model);
}

} // namespace

std::vector<VolumeMeshIdentity> mesh_identities(const Model &model)
{
    std::vector<VolumeMeshIdentity> out;
    for (const ModelObject *object : model.objects)
        for (const ModelVolume *volume : object->volumes)
            out.push_back(VolumeMeshIdentity{ volume->id(), volume->mesh_ptr().get() });
    std::sort(out.begin(), out.end(),
              [](const VolumeMeshIdentity &a, const VolumeMeshIdentity &b) { return a.volume_id < b.volume_id; });
    return out;
}

bool evaluation_inputs_unchanged(const EvaluationInput &captured, const EvaluationInput &live)
{
    if (captured.object_id != live.object_id || captured.plates.size() != live.plates.size())
        return false;
    // Plates are captured in plate order, so entry i on both sides is the same plate; its index is
    // compared as a value, so a scene the user re-plated is a change even where nothing else moved.
    for (size_t i = 0; i < captured.plates.size(); ++ i)
        if (! plate_unchanged(captured.plates[i], live.plates[i]))
            return false;
    return true;
}

std::vector<InstanceSnapshot> posed_instances(const EvaluationInput &input, const Pose &pose)
{
    std::vector<InstanceSnapshot> out;
    for (const PlateInput &plate : input.plates)
        for (const ObjectID &id : plate.affected_instance_ids)
            for (const ModelObject *object : plate.model.objects) {
                const ModelInstance *instance = nullptr;
                size_t               index    = 0;
                for (size_t i = 0; i < object->instances.size(); ++ i)
                    if (object->instances[i]->id() == id) {
                        instance = object->instances[i];
                        index    = i;
                        break;
                    }
                if (instance == nullptr)
                    continue;
                // This instance's own root and its own pivot, which is the object's bounding box
                // under this instance's live matrix - the convention the GUI job caches per instance.
                InstanceSnapshot snapshot{ id, candidate_transform(instance->get_transformation().get_matrix(),
                                                                   object->instance_bounding_box(index).center(), pose) };
                if (instance->auto_drop) {
                    const BoundingBoxf3 box = posed_hull_box(plate.model, snapshot);
                    if (box.defined)
                        // The transformed convex hull has the same extreme z as the transformed mesh,
                        // so this drop is exact rather than conservative, and it is this instance's
                        // own: ensure_on_bed() would have applied instance zero's to all of them.
                        snapshot.matrix = Geometry::translation_transform(Vec3d(0., 0., - box.min.z())) * snapshot.matrix;
                }
                out.push_back(snapshot);
                break;
            }
    return out;
}

BoundingBoxf3 posed_hull_box(const Model &model, const InstanceSnapshot &snapshot)
{
    BoundingBoxf3      box;
    const ModelObject *object = object_of_instance(model, snapshot.id);
    if (object == nullptr)
        return box;
    for (const ModelVolume *volume : object->volumes)
        if (volume->is_model_part())
            box.merge(volume->get_convex_hull().transformed_bounding_box(snapshot.matrix * volume->get_matrix()));
    return box;
}

bool pose_admissible(const EvaluationInput &input, const Pose &pose)
{
    const std::vector<InstanceSnapshot> all_posed = posed_instances(input, pose);
    for (const PlateInput &plate : input.plates) {
        // Membership is the plate's own affected list, never "whichever ids this plate's model
        // happens to carry": two plates can be captured from models that carry the same instance ids.
        for (const ObjectID &id : plate.affected_instance_ids) {
            const InstanceSnapshot *posed = nullptr;
            for (const InstanceSnapshot &snapshot : all_posed)
                if (snapshot.id == id) {
                    posed = &snapshot;
                    break;
                }
            if (posed == nullptr)
                return false;
            const ModelObject   *object   = object_of_instance(plate.model, id);
            const BoundingBoxf3  hull_box = posed_hull_box(plate.model, *posed);
            if (object == nullptr || ! hull_box.defined)
                return false;
            if (plate_refusal(plate.printable_regions, plate.exclusions, plate.printable_height_mm,
                              posed_footprint(*object, posed->matrix), hull_box) != nullptr)
                return false;
        }
    }
    return true;
}

SupportGenerator affected_support_generator(const EvaluationInput &input)
{
    SupportGenerator resolved = SupportGenerator::Unknown;
    bool             first    = true;
    for (const PlateInput &plate : input.plates) {
        // A clone, because Print::apply rewrites the model tree it is given; the clone keeps every
        // id, so the affected instances are found in the print by the ids the capture named.
        Model clone(plate.model);
        Print print;
        print.set_status_silent();
        print.apply(clone, plate.full_config);
        for (const ObjectID &id : plate.affected_instance_ids) {
            size_t             index  = 0;
            const PrintObject *object = print_object_of_instance(print, id, index);
            SupportGenerator   here   = SupportGenerator::Unknown;
            if (object != nullptr && object->config().enable_support.value && is_tree(object->config().support_type.value))
                // SupportParameters owns the rule that turns the unset style into a generator, the
                // tree default into Organic among it, so the answer is read from there and not from
                // the raw option value.
                here = SupportParameters(*object).support_style == smsTreeOrganic ? SupportGenerator::Organic : SupportGenerator::Legacy;
            if (first) {
                resolved = here;
                first    = false;
            } else if (here != resolved) {
                // Unknown beside anything is still an operation nothing answers for, and a legacy
                // instance beside an Organic one is the mixed case the caller must refuse outright.
                return here == SupportGenerator::Unknown || resolved == SupportGenerator::Unknown ? SupportGenerator::Unknown :
                                                                                                    SupportGenerator::Mixed;
            }
        }
    }
    return resolved;
}

GeneratedEvaluator::GeneratedEvaluator(const EvaluationInput &input, MainThreadRunner run_on_main)
    : m_input(input), m_run_on_main(std::move(run_on_main))
{
    m_models.resize(m_input.plates.size());
    m_prints.resize(m_input.plates.size());
    for (size_t i = 0; i < m_input.plates.size(); ++ i) {
        m_models[i] = std::make_unique<Model>();
        m_prints[i] = std::make_unique<Print>();
        m_prints[i]->set_status_silent();
    }
}

void GeneratedEvaluator::cancel()
{
    // The latch first, so a worker that is between two poses sees the cancellation even if it never
    // looks at m_active; then the Print that is running right now, which only PrintBase::cancel() can
    // stop from another thread.
    m_cancel_requested.store(true);
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_active != nullptr)
        m_active->cancel();
}

bool GeneratedEvaluator::stopped(const StopPredicate &stop) const
{
    // The predicate belongs to the worker that called evaluate() and is asked only here; a thread
    // that is not that worker stops this evaluator through cancel() and the latch.
    return m_cancel_requested.load() || (stop && stop());
}

PoseEvaluation GeneratedEvaluator::evaluate(const Pose &pose, const StopPredicate &stop)
{
    PoseEvaluation out;
    out.pose   = pose;
    out.status = PoseEvaluation::Status::Complete;

    if (this->stopped(stop)) {
        out.status = PoseEvaluation::Status::Canceled;
        add_reason(out, "canceled");
        return out;
    }

    // Where every affected instance of every captured plate stands under the pose: one helper, the
    // same matrices the final fit check and the GUI application will use. An id the captured model no
    // longer carries is not something to guess at.
    const std::vector<InstanceSnapshot> all_posed = posed_instances(m_input, pose);

    for (size_t p = 0; p < m_input.plates.size(); ++ p) {
        const PlateInput &plate = m_input.plates[p];

        // This plate's own affected instances, in capture order. Membership is the plate's affected
        // list, never "whichever ids this plate's model happens to carry": two plates can be captured
        // from models that carry the same instance ids, and only one of them prints each of them.
        std::vector<InstanceSnapshot> posed;
        for (const ObjectID &id : plate.affected_instance_ids)
            for (const InstanceSnapshot &snapshot : all_posed)
                if (snapshot.id == id) {
                    posed.push_back(snapshot);
                    break;
                }
        if (posed.size() != plate.affected_instance_ids.size()) {
            out.status = worse(out.status, PoseEvaluation::Status::Unknown);
            add_reason(out, "instance_missing");
            continue;
        }

        // The plate has to be able to print what the pose asks for, before anything is sliced: a
        // candidate that leaves the printable ground, rises above the printable height or reaches
        // into an exclusion is not a candidate. plate_refusal owns which of those it broke, so the
        // pre-pass and the GUI application read the same rule through pose_admissible.
        bool admissible = true;
        for (const InstanceSnapshot &instance : posed) {
            if (this->stopped(stop)) {
                out.status = PoseEvaluation::Status::Canceled;
                add_reason(out, "canceled");
                return out;
            }
            const ModelObject   *object   = object_of_instance(plate.model, instance.id);
            const BoundingBoxf3  hull_box = posed_hull_box(plate.model, instance);
            if (object == nullptr || ! hull_box.defined) {
                out.status = worse(out.status, PoseEvaluation::Status::Unknown);
                add_reason(out, "instance_missing");
                admissible = false;
                break;
            }
            if (const char *refusal = plate_refusal(plate.printable_regions, plate.exclusions, plate.printable_height_mm,
                                                   posed_footprint(*object, instance.matrix), hull_box)) {
                out.status = worse(out.status, PoseEvaluation::Status::Invalid);
                add_reason(out, refusal);
                admissible = false;
                break;
            }
        }
        if (! admissible)
            continue;

        // Fresh private state for this pose: the captured plate cloned whole, the affected instances
        // moved on the clone, and a Print that has never seen another pose. Cloning and apply rewrite
        // the model tree and hand out ObjectIDs, so both belong to the main thread.
        m_run_on_main([this, p, &plate, &posed]() {
            m_models[p] = std::make_unique<Model>(plate.model);
            for (const InstanceSnapshot &instance : posed)
                for (ModelObject *object : m_models[p]->objects)
                    for (ModelInstance *live : object->instances)
                        if (live->id() == instance.id) {
                            live->set_transformation(Geometry::Transformation(instance.matrix));
                            object->invalidate_bounding_box();
                        }
            m_prints[p] = std::make_unique<Print>();
            m_prints[p]->set_status_silent();
            // TreeSupport places the machine border it clips every support area to by this origin.
            m_prints[p]->set_plate_origin(plate.plate_origin);
            m_prints[p]->apply(*m_models[p], plate.full_config);
        });

        Print &print = *m_prints[p];
        if (print.objects().empty()) {
            out.status = worse(out.status, PoseEvaluation::Status::Unknown);
            add_reason(out, "apply_produced_no_object");
            continue;
        }

        // The posed model against the plate's own print-order and collision context. A warning is a
        // warning; a returned message is the print refusing the arrangement, which is not something
        // to slice anyway.
        std::vector<StringObjectException> warnings;
        if (! print.validate(&warnings).string.empty()) {
            out.status = worse(out.status, PoseEvaluation::Status::Invalid);
            add_reason(out, "validate_rejected");
            continue;
        }

        // Only the legacy tree generator measures itself. Organic keeps its own search and is not
        // something this evaluator answers for, so it is Unknown rather than a legacy report wearing
        // an Organic label.
        bool needs_request = false, unsupported = false;
        for (const InstanceSnapshot &instance : posed) {
            size_t             index  = 0;
            const PrintObject *object = print_object_of_instance(print, instance.id, index);
            if (object == nullptr) {
                out.status = worse(out.status, PoseEvaluation::Status::Unknown);
                add_reason(out, "apply_produced_no_object");
                unsupported = true;
                break;
            }
            if (! legacy_tree_support(*object)) {
                out.status = worse(out.status, PoseEvaluation::Status::Unknown);
                add_reason(out, "organic_or_non_tree_support");
                unsupported = true;
                break;
            }
            // With the miniature contact mode on the generator measures itself already; with it off a
            // legacy pass measures only when it is asked to. Asking adds the floating pass, which takes
            // out the support extrusions resting on nothing, and otherwise leaves the output alone.
            if (! object->config().support_miniature_contacts.value)
                needs_request = true;
        }
        if (unsupported)
            continue;
        if (needs_request)
            // Print fans the request over every object of the print, because a shared object reads
            // the owner's pass and the owner is the one that generates it.
            print.request_legacy_support_analysis();

        bool canceled = this->stopped(stop);
        if (! canceled) {
            // Publishes this Print for the duration of its processing and unpublishes it on every
            // exit path, a thrown one included, so no cancel() can ever reach a destroyed Print. A
            // cancellation that latched while the model was being applied is delivered right here.
            ActivePrint published(m_mutex, m_active, print, m_cancel_requested);
            try {
                print.process();
            } catch (const CanceledException &) {
                canceled = true;
            }
        }
        if (canceled || this->stopped(stop)) {
            out.status = PoseEvaluation::Status::Canceled;
            add_reason(out, "canceled");
            return out;
        }

        // One measurement per affected physical instance, in capture order: a generated pass shared
        // by several copies is counted once per copy that prints it, and no copy stands in for
        // another.
        for (const InstanceSnapshot &instance : posed) {
            size_t             index  = 0;
            const PrintObject *object = print_object_of_instance(print, instance.id, index);
            const std::shared_ptr<const SupportAnalysis::Report> measured =
                object == nullptr ? nullptr : object->support_analysis();
            if (measured == nullptr) {
                out.status = worse(out.status, PoseEvaluation::Status::Unknown);
                add_reason(out, "analysis_missing");
                continue;
            }
            // The ground this copy stands on, taken again now that the print has laid its brim: the
            // support measurement ran before any of it existed.
            const std::shared_ptr<const SupportAnalysis::EmittedSupport> emitted = object->emitted_support();
            const SupportAnalysis::Report report =
                emitted == nullptr ? *measured :
                                     SupportAnalysis::refresh_bed_footprint(*measured, *object, *emitted,
                                                                            instance_adhesion(print, *object, index));
            out.support_volume_mm3 += report.support_volume_mm3;
            out.raft_volume_mm3    += report.raft_volume_mm3;
            out.instances.push_back(report);

            switch (report.status) {
            case SupportAnalysis::Report::Status::Complete:
                if (! report.missing_anchor_ids.empty()) {
                    out.status = worse(out.status, PoseEvaluation::Status::UnresolvedCoverage);
                    add_reason(out, "required_region_unsupported");
                }
                break;
            case SupportAnalysis::Report::Status::UnresolvedCoverage:
                out.status = worse(out.status, PoseEvaluation::Status::UnresolvedCoverage);
                add_reason(out, "required_region_unsupported");
                break;
            case SupportAnalysis::Report::Status::Unknown:
                out.status = worse(out.status, PoseEvaluation::Status::Unknown);
                add_reason(out, "analysis_incomplete");
                break;
            }
        }
    }

    if (out.instances.empty() && out.status == PoseEvaluation::Status::Complete) {
        out.status = PoseEvaluation::Status::Unknown;
        add_reason(out, "nothing_measured");
    }
    return out;
}

}} // namespace Slic3r::AutoTilt
