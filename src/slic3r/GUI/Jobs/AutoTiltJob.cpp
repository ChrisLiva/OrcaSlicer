#include "AutoTiltJob.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <mutex>
#include <thread>

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

// The cheap precondition gate, read straight off the live model so opening a menu costs no clone:
// the object under its own live matrices, each auto-dropped instance dropped by its own transformed
// hull, which is the same rule AutoTilt::posed_instances applies to a candidate.
static bool root_pose_fits(const ModelObject &obj, const std::vector<PartPlate *> &plates)
{
    if (obj.instances.size() != plates.size())
        return false;
    std::vector<BoundingBoxf3> boxes(obj.instances.size());
    for (size_t i = 0; i < obj.instances.size(); ++i) {
        const Transform3d matrix = obj.instances[i]->get_transformation().get_matrix();
        boxes[i]                 = obj.instance_convex_hull_bounding_box(i);
        if (boxes[i].defined && obj.instances[i]->auto_drop)
            boxes[i].translate(0., 0., -boxes[i].min.z());
        if (!boxes[i].defined)
            return false;
        // The same rule a candidate pose is held to, read off the live plate: the shared printable
        // polygon capture_inputs would store, the exclude areas and the top of the build volume.
        // A degenerate shared polygon leaves the polygon test out, exactly as capture_inputs does.
        Polygon    p = plates[i]->get_shared_printable_polygon();
        ExPolygons ground;
        if (p.points.size() > 2)
            ground.emplace_back(std::move(p));
        if (AutoTilt::plate_refusal(ground, plates[i]->get_exclude_areas(), plates[i]->get_build_volume().max.z(),
                                    AutoTilt::posed_footprint(obj, matrix), boxes[i]) != nullptr)
            return false;
    }
    return true;
}

// The object's own value when it carries one, otherwise the edited print preset's.
static const DynamicPrintConfig &effective_config(const ModelObject &obj, const DynamicPrintConfig &global, const t_config_option_key &key)
{
    return obj.config.has(key) ? obj.config.get() : global;
}

// Relays the job controller's cancellation to the evaluator, which owns a Print that nothing but
// PrintBase::cancel() can stop from another thread. It reads Ctl::was_canceled() at most every 25 ms
// on its own thread and joins in its destructor, so it never outlives the controller or the evaluator
// it holds, on any exit path including a thrown one. It touches nothing else: no GUI call, no
// assertion, no predicate of the caller's, and it is never detached.
class CancellationRelay
{
public:
    CancellationRelay(Job::Ctl &ctl, AutoTilt::GeneratedEvaluator &evaluator)
    {
        m_thread = std::thread([this, &ctl, &evaluator]() {
            std::unique_lock<std::mutex> lock(m_mutex);
            while (!m_done) {
                if (ctl.was_canceled()) {
                    lock.unlock();
                    evaluator.cancel();
                    return;
                }
                m_wake.wait_for(lock, std::chrono::milliseconds(25), [this]() { return m_done; });
            }
        });
    }
    ~CancellationRelay()
    {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_done = true;
        }
        m_wake.notify_all();
        if (m_thread.joinable())
            m_thread.join();
    }
    CancellationRelay(const CancellationRelay &)            = delete;
    CancellationRelay &operator=(const CancellationRelay &) = delete;

private:
    std::mutex              m_mutex;
    std::condition_variable m_wake;
    bool                    m_done = false;
    std::thread             m_thread;
};

// Every plate the selected object stands on, captured whole: the plate's own model with every
// instance that prints elsewhere removed, the config that plate would slice under, the affected ids,
// the printable ground, the exclusion volumes, and the mesh each of its volumes stood for. The clone
// keeps every ObjectID, so the capture and the live scene are matched by id and never by position.
static AutoTilt::EvaluationInput capture_inputs(Plater &plater, int obj_idx)
{
    AutoTilt::EvaluationInput input;
    const ModelObjectPtrs    &objects = plater.model().objects;
    const ModelObject        &obj     = *objects[obj_idx];
    input.object_id        = obj.id();

    PartPlateList &plates = plater.get_partplate_list();
    // Which plate each instance of every object prints on, so membership is asked of PartPlateList
    // once and every later question is answered from the same table.
    std::vector<std::vector<int>> plate_of(objects.size());
    for (size_t i = 0; i < objects.size(); ++i) {
        plate_of[i].reserve(objects[i]->instances.size());
        for (size_t j = 0; j < objects[i]->instances.size(); ++j)
            plate_of[i].push_back(plates.find_instance(int(i), int(j)));
    }

    // The plates the selected object reaches, in plate order, each with the affected ids that print
    // on it. An instance off every plate leaves the capture empty: it is a precondition failure.
    std::vector<int> affected_plates;
    for (size_t j = 0; j < obj.instances.size(); ++j) {
        const int plate_idx = plate_of[obj_idx][j];
        if (plate_idx < 0 || plates.get_plate(plate_idx) == nullptr)
            return AutoTilt::EvaluationInput{};
        if (std::find(affected_plates.begin(), affected_plates.end(), plate_idx) == affected_plates.end())
            affected_plates.push_back(plate_idx);
    }
    std::sort(affected_plates.begin(), affected_plates.end());

    const std::vector<AutoTilt::VolumeMeshIdentity> scene_meshes = AutoTilt::mesh_identities(plater.model());

    for (int plate_idx : affected_plates) {
        PartPlate           *plate = plates.get_plate(plate_idx);
        AutoTilt::PlateInput captured;
        captured.plate_index = size_t(plate_idx);
        captured.plate_origin = plate->get_origin();

        for (size_t j = 0; j < obj.instances.size(); ++j)
            if (plate_of[obj_idx][j] == plate_idx)
                captured.affected_instance_ids.push_back(obj.instances[j]->id());

        // The plate's own model: the whole scene cloned, then every instance that prints on another
        // plate removed, and every object left without one with it. What stays is the collision and
        // print-order context this plate would actually slice.
        captured.model = plater.model();
        for (size_t i = objects.size(); i-- > 0;) {
            ModelObject *cloned = captured.model.objects[i];
            for (size_t j = cloned->instances.size(); j-- > 0;)
                if (plate_of[i][j] != plate_idx)
                    cloned->delete_instance(j);
            if (cloned->instances.empty())
                captured.model.delete_object(i);
        }

        // The pair BackgroundSlicingProcess slices a plate under: the whole edited configuration,
        // then that plate's own overrides on top of it.
        captured.full_config = wxGetApp().preset_bundle->full_config(false);
        captured.full_config.apply(*plate->config());

        // The plate's printable ground, which need not be a rectangle, in plate coordinates. On a
        // printer carrying extruder_printable_area this is the area every extruder shares, not the
        // whole bed, so a pose the assigned extruder cannot reach stays outside the printable region.
        Polygon ground = plate->get_shared_printable_polygon();
        if (ground.points.size() > 2)
            captured.printable_regions.emplace_back(std::move(ground));
        captured.exclusions = plate->get_exclude_areas();
        // The top of the build volume, so a tilt that raises the object above what the printer
        // reaches is refused.
        captured.printable_height_mm = plate->get_build_volume().max.z();

        for (const AutoTilt::VolumeMeshIdentity &identity : scene_meshes) {
            bool on_plate = false;
            for (const ModelObject *cloned : captured.model.objects)
                for (const ModelVolume *volume : cloned->volumes)
                    on_plate = on_plate || volume->id() == identity.volume_id;
            if (on_plate)
                captured.mesh_identity.push_back(identity);
        }

        input.plates.push_back(std::move(captured));
    }
    return input;
}

bool AutoTiltJob::state_preconditions_hold(Plater &plater, int obj_idx)
{
    const ModelObjectPtrs &objects = plater.model().objects;
    if (obj_idx < 0 || obj_idx >= int(objects.size()))
        return false;
    const ModelObject &obj = *objects[obj_idx];
    if (obj.instances.empty())
        return false;

    const DynamicPrintConfig &global  = wxGetApp().preset_bundle->prints.get_edited_preset().config;
    const ConfigOptionBool   *enabled = effective_config(obj, global, "enable_support").option<ConfigOptionBool>("enable_support");
    if (enabled == nullptr || !enabled->value)
        return false;
    const auto *type = effective_config(obj, global, "support_type").option<ConfigOptionEnum<SupportType>>("support_type");
    if (type == nullptr || !is_tree(type->value))
        return false;

    // The fit pre-pass drops every candidate back onto the bed, which only happens for auto-dropped
    // instances, and it needs a plate to measure against.
    for (const ModelInstance *inst : obj.instances)
        if (!inst->auto_drop)
            return false;
    const std::vector<PartPlate *> plates = resolve_plates(plater, obj_idx, obj.instances.size());
    if (plates.empty())
        return false;
    return root_pose_fits(obj, plates);
}

bool AutoTiltJob::can_start(Plater &plater)
{
    if (!plater.get_ui_job_worker().is_idle())
        return false;
    if (!plater.is_single_full_object_selection())
        return false;
    return state_preconditions_hold(plater, plater.get_selected_object_idx());
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

// What the pose does to the object, and nothing about what it buys: the same sentence serves the
// measured legacy result and the estimated Organic one, so neither borrows the other's claim.
static std::string pose_text(const AutoTilt::Pose &pose)
{
    const double   tilt      = std::abs(pose.tilt_deg);
    const double   lean      = std::abs(pose.lean_deg);
    const wxString lean_word = pose.lean_deg > 0 ? _L("right") : _L("left");
    if (tilt != 0. && lean != 0.)
        return GUI::format(_L("Tilted %1%° back, leaned %2%° %3%."), tilt, lean, lean_word);
    if (lean == 0.)
        return GUI::format(_L("Tilted %1%° back."), tilt);
    return GUI::format(_L("Leaned %1%° %2%."), lean, lean_word);
}

// The qualification every Organic result carries: its percentage comes from the cheap contact
// estimate, and no support was generated to check it.
static std::string organic_note()
{
    return _u8L("Support generation was not verified for Organic.");
}

// Which way the estimated removal risk moved between the two measured poses. The volume field is
// zeroed on both tuples first, so compare_objectives ranks the Damage fields and nothing
// else: a pose that only cut volume must not read as one that made removal safer. A tuple no
// evaluation filled in says so rather than passing as a tie.
static std::string damage_direction_text(const AutoTilt::PoseEvaluation &root, const AutoTilt::PoseEvaluation &selected)
{
    AutoTilt::Objectives was = AutoTilt::objectives(root);
    AutoTilt::Objectives is  = AutoTilt::objectives(selected);
    if (!was.damage.available || !is.damage.available)
        return _u8L("Estimated removal risk: not measured.");
    was.volume_mm3 = 0.;
    is.volume_mm3  = 0.;
    const int order = AutoTilt::compare_objectives(was, is);
    if (order < 0)
        return _u8L("Estimated removal risk: lower than at the current angle.");
    if (order > 0)
        return _u8L("Estimated removal risk: higher than at the current angle.");
    return _u8L("Estimated removal risk: unchanged from the current angle.");
}

// The user-facing half of one machine reason code. The codes come from AutoTilt::search_verified and
// from the root evaluation it carries; an unmapped code falls through to the code itself, so a code a
// later change adds shows up in the notification instead of vanishing from it.
static std::string reason_phrase(const std::string &code)
{
    if (code == "no_root_pose")
        return _u8L("the current orientation was not among the angles tested");
    if (code == "required_region_unsupported")
        return _u8L("a required support region stayed unreachable");
    if (code == "root_analysis_unavailable" || code == "analysis_missing" || code == "analysis_incomplete")
        return _u8L("the support analysis did not finish");
    if (code == "nothing_measured")
        return _u8L("no instance was measured");
    if (code == "cheap_score_not_finite")
        return _u8L("some angles could not be scored");
    if (code == "candidate_inadmissible")
        return _u8L("no angle tested gave a complete support measurement");
    if (code == "no_candidate_measured")
        return _u8L("no candidate angle could be measured");
    if (code == "instance_missing")
        return _u8L("an instance was missing from the test slice");
    if (code == "outside_printable_region")
        return _u8L("an instance left the printable area");
    if (code == "exclusion_area")
        return _u8L("an instance reached into an exclusion area");
    if (code == "apply_produced_no_object" || code == "validate_rejected")
        return _u8L("the print settings rejected the test slice");
    if (code == "organic_or_non_tree_support")
        return _u8L("the affected copies do not all use a legacy tree style");
    if (code == "gain_unknown_contacts")
        return _u8L("contacts of unknown removal risk did not drop enough");
    if (code == "gain_inaccessible_groups")
        return _u8L("support groups with no removal access did not drop enough");
    if (code == "gain_max_group_risk")
        return _u8L("the worst group's estimated removal risk did not drop enough");
    if (code == "gain_total_group_risk")
        return _u8L("the total estimated removal risk did not drop enough");
    if (code == "gain_volume")
        return _u8L("generated support volume did not drop enough");
    if (code == "gain_none")
        return _u8L("no measured objective differed");
    return code;
}

// search_verified files every root it cannot use under "root_analysis_unavailable", which names none
// of the reasons the root carries, so there the root's own code comes first. On any other outcome the
// root was used, and a code it carries (an open required region) is not why the orientation was kept.
std::string AutoTiltJob::reason_detail() const
{
    const bool root_first = m_verified.outcome == AutoTilt::VerifiedSearchResult::Outcome::VerificationUnavailable;
    const std::vector<std::string> &first  = root_first ? m_verified.root.reason_codes : m_verified.reason_codes;
    const std::vector<std::string> &second = root_first ? m_verified.reason_codes : m_verified.root.reason_codes;
    for (const std::vector<std::string> *codes : {&first, &second})
        for (const std::string &code : *codes)
            // Cancellation is already silent, so it is never the reason a result is shown for.
            if (code != "canceled")
                return " " + GUI::format(_L("Reason: %1%."), reason_phrase(code));
    return {};
}

std::string AutoTiltJob::estimated_text(const AutoTilt::Pose &pose, double improvement) const
{
    const int reduced = int(std::lround(100. * improvement));
    // xgettext:no-c-format, no-boost-format
    return GUI::format(_L("Estimated support contact reduction %1%%%."), reduced) + " " + pose_text(pose);
}

std::string AutoTiltJob::verified_text() const
{
    const AutoTilt::PoseEvaluation &root       = m_verified.root;
    const AutoTilt::PoseEvaluation &selected   = m_verified.selected;
    // Both totals already carry the raft, which prints with the support and has to be cleaned off the
    // same object; the raft is then named on its own so neither number hides inside the other.
    const double                    root_total = root.support_volume_mm3 + root.raft_volume_mm3;
    const double                    sel_total  = selected.support_volume_mm3 + selected.raft_volume_mm3;

    std::string text;
    if (root_total > 0.) {
        // A percentage of nothing is not a percentage, so it is only ever charged against a root that
        // actually generated support.
        const int percent = int(std::lround(100. * (root_total - sel_total) / root_total));
        if (percent > 0)
            // xgettext:no-c-format, no-boost-format
            text = GUI::format(_L("Generated support: %1% mm³, down from %2% mm³ (%3%%% less)."), int(std::lround(sel_total)),
                               int(std::lround(root_total)), percent);
        else if (percent < 0)
            // xgettext:no-c-format, no-boost-format
            text = GUI::format(_L("Generated support: %1% mm³, up from %2% mm³ (%3%%% more)."), int(std::lround(sel_total)),
                               int(std::lround(root_total)), -percent);
        else
            text = GUI::format(_L("Generated support: %1% mm³, unchanged from %2% mm³."), int(std::lround(sel_total)),
                               int(std::lround(root_total)));
    } else {
        text = GUI::format(_L("Generated support: %1% mm³; the current angle generated none."), int(std::lround(sel_total)));
    }
    if (selected.raft_volume_mm3 > 0. || root.raft_volume_mm3 > 0.)
        text += " " + GUI::format(_L("Raft included: %1% mm³, against %2% mm³ at the current angle."),
                                  int(std::lround(selected.raft_volume_mm3)), int(std::lround(root.raft_volume_mm3)));

    text += " " + damage_direction_text(root, selected);
    text += " " + pose_text(selected.pose);
    // The finalists actually measured, capped by AutoTilt::verified_finalist_count and shorter when
    // the plate left fewer legal angles or the shortlist dropped non-finite and duplicate poses, so
    // the count is never larger than what was sliced.
    text += " " + GUI::format(_L("%1% finalist angles were checked with generated support; physical support removal is not verified."),
                              m_verified.shortlist.size());
    return text;
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

    m_captured = capture_inputs(plater, obj_idx);
    if (m_captured.plates.empty()) {
        push_result(_u8L("Auto-tilt needs tree supports to be enabled."));
        return false;
    }
    // A plate or object override that puts some copies on the legacy generator and the rest on
    // Organic is one operation no single search answers for.
    m_generator = AutoTilt::affected_support_generator(m_captured);
    if (m_generator == AutoTilt::SupportGenerator::Mixed) {
        push_result(_u8L("Auto-tilt cannot verify this object: its copies do not all use the same support generator."));
        return false;
    }
    if (m_generator == AutoTilt::SupportGenerator::Unknown) {
        push_result(_u8L("Auto-tilt needs tree supports to be enabled."));
        return false;
    }

    const std::vector<AutoTilt::Pose> all = AutoTilt::grid(m_k);
    m_total_poses                         = all.size();
    m_legal.clear();
    for (const AutoTilt::Pose &pose : all)
        if (AutoTilt::pose_admissible(m_captured, pose))
            m_legal.push_back(pose);
    m_skipped = m_total_poses - m_legal.size();
    // Both searches need the root in the list, and the root pose is exactly what
    // state_preconditions_hold measured, so the pre-pass can only have dropped it if the capture and
    // the live scene disagree about where this object stands - in which case there is nothing to run.
    if (m_legal.empty() || !m_legal.front().is_root()) {
        push_result(_u8L("Auto-tilt discarded: the model or printer changed while the search was running."));
        return false;
    }

    if (m_legal.size() == 1) {
        push_result(GUI::format(_L("No other angle fits on this plate, %1% of %2% were skipped."), m_skipped, m_total_poses));
        return false;
    }

    AutoTilt::MainThreadRunner run_on_main = [this](const std::function<void()> &fn) {
        if (m_ctl == nullptr)
            fn();
        else
            m_ctl->call_on_main_thread(fn).wait();
    };
    if (m_generator == AutoTilt::SupportGenerator::Legacy)
        m_scorer = std::make_unique<AutoTilt::LegacyShortlistScorer>(m_captured, m_k, std::move(run_on_main));
    else
        m_scorer = std::make_unique<AutoTilt::ContactScorer>(obj, m_captured.plates.front().full_config, m_k,
                                                             std::move(run_on_main));
    return true;
}

void AutoTiltJob::process(Ctl &ctl)
{
    m_ctl = &ctl;
    const AutoTilt::StopPredicate stop     = [&ctl] { return ctl.was_canceled(); };
    const AutoTilt::ProgressSink  progress = [&ctl](size_t done, size_t total) {
        ctl.update_status(int(100 * done / total), GUI::format(_L("Testing angle %1% of %2%"), done, total));
    };

    if (m_generator == AutoTilt::SupportGenerator::Legacy) {
        // The legacy generator measures itself, so the winner is a pose that was sliced under the
        // actual settings rather than one the coarse scorer liked. The relay is declared after the
        // evaluator and destroyed before it, so the thread that may call cancel() is always joined
        // while the evaluator it holds is still alive.
        AutoTilt::GeneratedEvaluator evaluator(m_captured, [this](const std::function<void()> &fn) {
            m_ctl->call_on_main_thread(fn).wait();
        });
        CancellationRelay relay(ctl, evaluator);
        m_verified = AutoTilt::search_verified(m_legal, *m_scorer, evaluator, m_k, stop, progress);
        return;
    }
    if (m_generator == AutoTilt::SupportGenerator::Organic)
        // Organic keeps the search it has: the generated evaluation does not answer for it.
        m_result = AutoTilt::search(m_legal, *m_scorer, m_k, stop, progress);
}

bool AutoTiltJob::apply_pose(const AutoTilt::Pose &pose)
{
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
    if (obj == nullptr || !state_preconditions_hold(*m_plater, obj_idx))
        return false;

    // The whole scene as it stands right now, read on the main thread immediately before anything is
    // moved: every value the measurement rested on has to still be the value it rested on.
    if (!AutoTilt::evaluation_inputs_unchanged(m_captured, capture_inputs(*m_plater, obj_idx)))
        return false;

    if (!AutoTilt::pose_admissible(m_captured, pose))
        return false;

    // Exactly the transforms the evaluation measured and the fit check just accepted, matched to the
    // live instances by ObjectID. Each already carries its own auto-drop, so no whole-object
    // ensure_on_bed() may follow: that would put instance zero's Z offset on every copy.
    const std::vector<AutoTilt::InstanceSnapshot> posed = AutoTilt::posed_instances(m_captured, pose);
    Plater::TakeSnapshot                          snapshot(m_plater, _u8L("Auto-tilt for supports"));
    for (const AutoTilt::InstanceSnapshot &entry : posed)
        for (ModelInstance *inst : obj->instances)
            if (inst->id() == entry.id)
                inst->set_transformation(Geometry::Transformation(entry.matrix));
    obj->invalidate_bounding_box();
    m_plater->update();
    return true;
}

void AutoTiltJob::finalize(bool canceled, std::exception_ptr &eptr)
{
    // Silent on cancel; an exception stays in eptr so the worker rethrows it.
    if (canceled || eptr)
        return;

    const auto discard = [this]() {
        push_result(with_skipped_clause(_u8L("Auto-tilt discarded: the model or printer changed while the search was running.")));
    };

    const auto push_estimate = [this](const std::string &text) { push_result(with_skipped_clause(text) + " " + organic_note()); };

    // prepare() refused a Mixed or Unknown generator before the job was queued, so only these two reach here.
    switch (m_generator) {
    case AutoTilt::SupportGenerator::Legacy:
        switch (m_verified.outcome) {
        case AutoTilt::VerifiedSearchResult::Outcome::Improved:
            if (!apply_pose(m_verified.selected.pose))
                discard();
            else
                push_result(with_skipped_clause(verified_text()));
            break;
        case AutoTilt::VerifiedSearchResult::Outcome::NoImprovement:
            push_result(with_skipped_clause(_u8L("No verified improvement found. Model orientation was kept.") + reason_detail()));
            break;
        case AutoTilt::VerifiedSearchResult::Outcome::VerificationUnavailable:
            push_result(with_skipped_clause(_u8L("Support verification was unavailable. Model orientation was kept.") + reason_detail()));
            break;
        default: // Canceled is already handled above
            break;
        }
        break;

    case AutoTilt::SupportGenerator::Organic:
        switch (m_result.outcome) {
        case AutoTilt::SearchResult::Outcome::Improved:
            if (!apply_pose(m_result.best))
                discard();
            else
                push_estimate(estimated_text(m_result.best, m_result.improvement));
            break;
        case AutoTilt::SearchResult::Outcome::BelowFloor:
            push_estimate(_u8L("Already nearly support-free — no tilt needed."));
            break;
        case AutoTilt::SearchResult::Outcome::NoImprovement:
            // xgettext:no-c-format, no-boost-format
            push_estimate(GUI::format(_L("No better angle found. The best of %1% angles gave an estimated support contact reduction of "
                                         "%2%%%, below the threshold for a %3%° tilt."),
                                      m_legal.size(), int(std::lround(100. * m_result.improvement)), std::abs(m_result.best.tilt_deg)));
            break;
        default: // Canceled is already handled above
            break;
        }
        break;

    default:
        break;
    }
}

}} // namespace Slic3r::GUI
