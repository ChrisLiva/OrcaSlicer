#include "AutoTiltJob.hpp"

#include <cmath>
#include <cstdlib>

#include "Worker.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "slic3r/GUI/GUI_App.hpp"
#include "slic3r/GUI/I18N.hpp"
#include "slic3r/GUI/NotificationManager.hpp"
#include "slic3r/GUI/PartPlate.hpp"
#include "slic3r/GUI/Plater.hpp"
#include "slic3r/GUI/format.hpp"

namespace Slic3r { namespace GUI {

// The plate each instance sits on, in instance order. Empty when any instance is off every plate,
// which is itself a precondition failure.
static std::vector<PartPlate *> resolve_plates(Plater &plater, int obj_idx, size_t instance_count)
{
    PartPlateList           &plates = plater.get_partplate_list();
    std::vector<PartPlate *> out;
    out.reserve(instance_count);
    for (size_t i = 0; i < instance_count; ++i) {
        const int  plate_idx = plates.find_instance(obj_idx, int(i));
        PartPlate *plate     = plate_idx < 0 ? nullptr : plates.get_plate(plate_idx);
        if (plate == nullptr)
            return {};
        out.push_back(plate);
    }
    return out;
}

// Does the object, posed by T (one transform per instance) and dropped back onto the bed, stay
// inside its plate and clear of that plate's exclude areas?
//
// The per-volume hull box repeats the body of ModelObject::instance_convex_hull_bounding_box; that
// helper takes an instance index and reads the live matrix, so it cannot serve a candidate transform.
static bool pose_fits(const ModelObject &obj, const std::vector<Transform3d> &T, const std::vector<PartPlate *> &plate)
{
    if (T.empty() || T.size() != plate.size())
        return false;

    std::vector<BoundingBoxf3> boxes(T.size());
    for (size_t i = 0; i < T.size(); ++i)
        for (const ModelVolume *v : obj.volumes)
            if (v->is_model_part())
                boxes[i].merge(v->get_convex_hull().transformed_bounding_box(T[i] * v->get_matrix()));
    for (const BoundingBoxf3 &box : boxes)
        if (!box.defined)
            return false;

    // ModelObject::ensure_on_bed translates every auto_drop instance by -min_z(), and
    // ModelObject::min_z() reads instances.front()'s matrix. The transformed convex hull has the
    // same extreme z as the transformed mesh, so dz is exact, not conservative.
    const double dz = -boxes[0].min.z();
    for (size_t i = 0; i < boxes.size(); ++i) {
        BoundingBoxf3 box = boxes[i];
        box.translate(0., 0., dz);
        if (!plate[i]->get_plate_box().contains(box))
            return false;
        for (const BoundingBoxf3 &ex : plate[i]->get_exclude_areas())
            if (box.intersects(ex))
                return false;
    }
    return true;
}

// One entry per instance: the ObjectID and the live matrix. Cached by prepare(), recomputed by
// finalize() so the two can be compared without holding on to anything from the model tree.
static std::vector<AutoTilt::InstanceSnapshot> snapshot_instances(const ModelObject &obj)
{
    std::vector<AutoTilt::InstanceSnapshot> out;
    out.reserve(obj.instances.size());
    for (const ModelInstance *inst : obj.instances)
        out.push_back(AutoTilt::InstanceSnapshot{inst->id(), inst->get_transformation().get_matrix()});
    return out;
}

// The object's own value when it carries one, otherwise the edited print preset's.
static const DynamicPrintConfig &effective_config(const ModelObject &obj, const DynamicPrintConfig &global, const t_config_option_key &key)
{
    return obj.config.has(key) ? obj.config.get() : global;
}

bool AutoTiltJob::state_preconditions_hold(Plater &plater, int obj_idx)
{
    const ModelObjectPtrs &objects = plater.model().objects;
    if (obj_idx < 0 || obj_idx >= int(objects.size()))
        return false;
    const ModelObject *obj = objects[obj_idx];
    if (obj == nullptr || obj->instances.empty())
        return false;

    const DynamicPrintConfig &global  = wxGetApp().preset_bundle->prints.get_edited_preset().config;
    const ConfigOptionBool   *enabled = effective_config(*obj, global, "enable_support").option<ConfigOptionBool>("enable_support");
    if (enabled == nullptr || !enabled->value)
        return false;
    const auto *type = effective_config(*obj, global, "support_type").option<ConfigOptionEnum<SupportType>>("support_type");
    if (type == nullptr || !is_tree(type->value))
        return false;

    // The fit pre-pass drops every candidate back onto the bed, which only happens for auto-dropped
    // instances, and it needs a plate to measure against.
    for (const ModelInstance *inst : obj->instances)
        if (!inst->auto_drop)
            return false;
    const std::vector<PartPlate *> plates = resolve_plates(plater, obj_idx, obj->instances.size());
    if (plates.empty())
        return false;

    std::vector<Transform3d> T;
    T.reserve(obj->instances.size());
    for (const ModelInstance *inst : obj->instances)
        T.push_back(inst->get_transformation().get_matrix());
    return pose_fits(*obj, T, plates);
}

bool AutoTiltJob::can_start(Plater &plater)
{
    if (!plater.get_ui_job_worker().is_idle())
        return false;
    if (!plater.is_single_full_object_selection())
        return false;
    return state_preconditions_hold(plater, plater.get_selected_object_idx());
}

std::vector<Transform3d> AutoTiltJob::candidate_transforms(const AutoTilt::Pose &pose) const
{
    std::vector<Transform3d> T;
    T.reserve(m_root_matrix.size());
    for (size_t i = 0; i < m_root_matrix.size(); ++i)
        T.push_back(AutoTilt::candidate_transform(m_root_matrix[i], m_pivot[i], pose));
    return T;
}

std::string AutoTiltJob::with_skipped_clause(const std::string &text) const
{
    if (m_skipped == 0)
        return text;
    return text + GUI::format(_L(" %1% of %2% angles were skipped, they leave the plate."), m_skipped, m_total_poses);
}

void AutoTiltJob::push_result(const std::string &text) const
{
    NotificationManager *notify = m_plater == nullptr ? nullptr : m_plater->get_notification_manager();
    if (notify != nullptr)
        notify->push_notification(NotificationType::AutoTiltResult, NotificationManager::NotificationLevel::RegularNotificationLevel, text);
}

bool AutoTiltJob::prepare(Plater &plater)
{
    m_plater = &plater;

    const int obj_idx = plater.get_selected_object_idx();
    if (!state_preconditions_hold(plater, obj_idx)) {
        push_result(_u8L("Auto-tilt needs tree supports to be enabled."));
        return false;
    }

    const ModelObject &obj = *plater.model().objects[obj_idx];
    m_object_id            = obj.id();
    m_instances            = snapshot_instances(obj);
    m_root_matrix.clear();
    m_pivot.clear();
    for (size_t i = 0; i < obj.instances.size(); ++i) {
        m_root_matrix.push_back(obj.instances[i]->get_transformation().get_matrix());
        m_pivot.push_back(obj.instance_bounding_box(i).center());
    }

    const std::vector<PartPlate *> plates = resolve_plates(plater, obj_idx, obj.instances.size());
    assert(!plates.empty()); // state_preconditions_hold already proved every instance sits on a plate

    DynamicPrintConfig cfg = wxGetApp().preset_bundle->full_config(false);
    cfg.apply(*plates[0]->config());

    const std::vector<AutoTilt::Pose> all = AutoTilt::grid(m_k);
    m_total_poses                         = all.size();
    m_legal.clear();
    for (const AutoTilt::Pose &pose : all)
        if (pose_fits(obj, candidate_transforms(pose), plates))
            m_legal.push_back(pose);
    m_skipped = m_total_poses - m_legal.size();
    // AutoTilt::search needs the root in the list, and the root pose is exactly what
    // state_preconditions_hold measured, so the pre-pass can never have dropped it.
    assert(!m_legal.empty() && m_legal.front().is_root());

    if (m_legal.size() == 1) {
        push_result(GUI::format(_L("No other angle fits on this plate, %1% of %2% were skipped."), m_skipped, m_total_poses));
        return false;
    }

    // Reads m_ctl at call time: it is null until process() sets it, and prepare() never scores, so
    // the runner only ever fires from the worker thread with a live controller.
    AutoTilt::MainThreadRunner run_on_main = [this](const std::function<void()> &fn) { m_ctl->call_on_main_thread(fn).wait(); };
    m_scorer = std::make_unique<AutoTilt::ContactScorer>(obj, cfg, m_k, std::move(run_on_main));
    return true;
}

void AutoTiltJob::process(Ctl &ctl)
{
    m_ctl    = &ctl;
    m_result = AutoTilt::search(
        m_legal, *m_scorer, m_k, [&ctl] { return ctl.was_canceled(); },
        [&ctl](size_t done, size_t total) { ctl.update_status(int(100 * done / total), GUI::format(_L("Testing angle %1% of %2%"), done, total)); });
}

void AutoTiltJob::finalize(bool canceled, std::exception_ptr &eptr)
{
    // Silent on cancel; an exception stays in eptr so the worker rethrows it.
    if (canceled || eptr)
        return;

    // Resolve the object by id, never by the index prepare() saw: the user may have deleted,
    // reordered or edited objects while the search ran.
    const ModelObjectPtrs &objects = m_plater->model().objects;
    ModelObject           *obj     = nullptr;
    int                    obj_idx = -1;
    for (size_t i = 0; i < objects.size(); ++i)
        if (objects[i]->id() == m_object_id) {
            obj     = objects[i];
            obj_idx = int(i);
            break;
        }

    const auto discard = [this]() {
        push_result(with_skipped_clause(_u8L("Auto-tilt discarded: the model or printer changed while the search was running.")));
    };

    if (obj == nullptr || !AutoTilt::instances_unchanged(m_instances, snapshot_instances(*obj)) ||
        !state_preconditions_hold(*m_plater, obj_idx)) {
        discard();
        return;
    }

    switch (m_result.outcome) {
    case AutoTilt::SearchResult::Outcome::Improved: {
        const std::vector<Transform3d> T      = candidate_transforms(m_result.best);
        const std::vector<PartPlate *> plates = resolve_plates(*m_plater, obj_idx, obj->instances.size());
        if (plates.empty() || !pose_fits(*obj, T, plates)) {
            discard();
            return;
        }

        Plater::TakeSnapshot snapshot(m_plater, _u8L("Auto-tilt for supports"));
        for (size_t i = 0; i < obj->instances.size(); ++i)
            obj->instances[i]->set_transformation(Geometry::Transformation(T[i]));
        obj->invalidate_bounding_box();
        obj->ensure_on_bed();
        m_plater->update();

        const int      reduced   = int(std::lround(100. * m_result.improvement));
        const double   tilt      = std::abs(m_result.best.tilt_deg);
        const double   lean      = std::abs(m_result.best.lean_deg);
        const wxString lean_word = m_result.best.lean_deg > 0 ? _L("right") : _L("left");
        std::string    text;
        if (tilt != 0. && lean != 0.)
            // xgettext:no-c-format, no-boost-format
            text = GUI::format(_L("Support contact reduced %1%%%. Tilted %2%° back, leaned %3%° %4%."), reduced, tilt, lean, lean_word);
        else if (lean == 0.)
            // xgettext:no-c-format, no-boost-format
            text = GUI::format(_L("Support contact reduced %1%%%. Tilted %2%° back."), reduced, tilt);
        else
            // xgettext:no-c-format, no-boost-format
            text = GUI::format(_L("Support contact reduced %1%%%. Leaned %2%° %3%."), reduced, lean, lean_word);
        push_result(with_skipped_clause(text));
        break;
    }
    case AutoTilt::SearchResult::Outcome::BelowFloor:
        push_result(with_skipped_clause(_u8L("Already nearly support-free — no tilt needed.")));
        break;
    case AutoTilt::SearchResult::Outcome::NoImprovement:
        // xgettext:no-c-format, no-boost-format
        push_result(with_skipped_clause(GUI::format(_L("No better angle found. The best of %1% angles cut support contact %2%%%, below the threshold for a %3%° tilt."),
                                                    m_legal.size(), int(std::lround(100. * m_result.improvement)), std::abs(m_result.best.tilt_deg))));
        break;
    default: // Canceled is already handled above
        break;
    }
}

}} // namespace Slic3r::GUI
