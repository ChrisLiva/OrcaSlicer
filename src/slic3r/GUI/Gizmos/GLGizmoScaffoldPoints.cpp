#include "GLGizmoScaffoldPoints.hpp"
#include <glad/gl.h>
#include "slic3r/GUI/GLCanvas3D.hpp"
#include "slic3r/GUI/GUI.hpp"
#include "slic3r/GUI/Camera.hpp"
#include "slic3r/GUI/Gizmos/GLGizmosCommon.hpp"
#include "slic3r/GUI/GUI_App.hpp"
#include "slic3r/GUI/MainFrame.hpp"
#include "slic3r/GUI/MsgDialog.hpp"
#include "slic3r/GUI/Plater.hpp"
#include "slic3r/GUI/format.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "libslic3r/SLA/SupportTree.hpp"
#include "libslic3r/Support/ScaffoldRetune.hpp"
#include "GLGizmoUtils.hpp"

namespace Slic3r { namespace GUI {

static const ColorRGBA SELECTED_COLOR    = {0.0f, 0.5f, 0.5f, 1.f};
static const ColorRGBA HOVER_COLOR       = {0.f, 1.f, 1.f, 1.f};
static const ColorRGBA UNSLICED_COLOR    = {1.f, 1.f, 1.f, 1.f};
static const ColorRGBA BARE_ISLAND_COLOR = {1.f, 0.9f, 0.1f, 1.f};
// The cone marks the direction a branch arrives from, so it keeps one size whatever the head.
static constexpr double CONE_RADIUS = 0.25;
static constexpr double CONE_HEIGHT = 0.75;
static constexpr double DISC_THICKNESS = 0.05;

static ColorRGBA result_color(ScaffoldTipResult result)
{
    switch (result) {
    case ScaffoldTipResult::Routed:   return {0.2f, 0.8f, 0.2f, 1.f};
    case ScaffoldTipResult::Filtered: return {0.6f, 0.6f, 0.6f, 1.f};
    case ScaffoldTipResult::Unrouted: return {1.f, 0.3f, 0.3f, 1.f};
    case ScaffoldTipResult::Neck:     return {1.f, 0.6f, 0.1f, 1.f};
    case ScaffoldTipResult::Merged:   return {0.3f, 0.5f, 1.f, 1.f};
    case ScaffoldTipResult::Wall:     return {0.7f, 0.3f, 0.9f, 1.f};
    }
    return UNSLICED_COLOR;
}

// A contact point needs a face a branch can reach from below, by the SLA support tree's rule for which faces take a head.
static bool faces_up(const Vec3d &world_normal)
{
    return std::acos(std::clamp(world_normal.normalized().z(), -1., 1.)) < M_PI - sla::SupportTreeConfig::normal_cutoff_angle;
}

// The record of `po`'s finished support step, or none: a step still running holds the previous pass's record or none.
// PrintObject reads and writes the slot atomically, since an object sharing another's layers has its record re-copied
// after its step reads as done.
static std::shared_ptr<const ScaffoldRecord> finished_record(const PrintObject *po)
{
    return po != nullptr && po->is_step_done(posSupportMaterial) ? po->scaffold_record() : nullptr;
}

// Looked up by id, never through the selection: the object may be gone or another one selected by now.
static ModelObject *model_object_by_id(ObjectID id)
{
    for (ModelObject *object : wxGetApp().model().objects)
        if (object->id() == id) return object;
    return nullptr;
}

static const ModelInstance *instance_by_id(const ModelObject *mo, ObjectID id)
{
    if (mo != nullptr)
        for (const ModelInstance *instance : mo->instances)
            if (instance->id() == id) return instance;
    return nullptr;
}

GLGizmoScaffoldPoints::GLGizmoScaffoldPoints(GLCanvas3D &parent, const std::string &icon_filename, unsigned int sprite_id)
    : GLGizmoBase(parent, icon_filename, sprite_id)
{
    const indexed_triangle_set sphere = its_make_sphere(1.0, double(PI) / 12.0);
    m_sphere.mesh_raycaster = std::make_unique<MeshRaycaster>(std::make_shared<const TriangleMesh>(sphere));
    m_sphere.model.init_from(sphere);
    m_cone.init_from(its_make_cone(1.0, 1.0, double(PI) / 12.0));
    m_disc.init_from(its_make_cylinder(1.0, 1.0, double(PI) / 12.0));
    m_disc.set_color(BARE_ISLAND_COLOR);
}

bool GLGizmoScaffoldPoints::on_init()
{
    const wxString ctrl  = GUI::shortkey_ctrl_prefix();
    const wxString alt   = GUI::shortkey_alt_prefix();
    const wxString shift = GUI::shortkey_shift_prefix();

    m_desc["head_size"]        = _L("Head size");
    m_desc["light"]            = _L("Light");
    m_desc["heavy"]            = _L("Heavy");
    m_desc["create"]           = _L("Create");
    m_desc["generate"]         = _L("Generate");
    m_desc["generate_tooltip"] = _L("Replace the list with the contact points the last slice routed.");
    m_desc["revert"]           = _L("Revert to auto");
    m_desc["revert_tooltip"]   = _L("Clear the list so that the next slice places its contact points automatically.");
    m_desc["remove"]           = _L("Remove");
    m_desc["remove_selected"]  = _L("Selected");
    m_desc["remove_all"]       = _L("All");
    m_desc["apply"]            = _L("Apply");
    m_desc["discard"]          = _L("Discard");
    m_desc["refused"]          = _L("That face points up. Place points on undersides and walls.");
    m_desc["density"]          = _L("Density");
    m_desc["density_tooltip"]  = _L("Slide to place fewer or more contact points, then Apply to slice with them.");
    m_desc["density_light"]    = _L("Light");
    m_desc["density_medium"]   = _L("Medium");
    m_desc["density_heavy"]    = _L("Heavy");
    m_desc["density_unsliced"] = _L("Slice the plate with automatic points to use the density slider.");
    m_desc["density_edited"]   = _L("The points were edited by hand. Discard the edits or revert to auto to use the density slider.");

    m_shortcuts = {
        {_L("Left mouse button"),  _L("Add or Select")},
        {_L("Right mouse button"), _L("Remove")},
        {_L("Drag"),               _L("Move point")},
        {shift + _L("Drag"),       _L("Select by rectangle")},
        {alt + _L("Drag"),         _L("Deselect by rectangle")},
        {ctrl + "A",               _L("Select all points")},
        {ctrl + _L("Mouse wheel"), m_desc["head_size"]},
    };

    return true;
}

void GLGizmoScaffoldPoints::data_changed(bool is_serializing)
{
    if (!m_c->selection_info()) return;

    // Only a change of object reloads the cache: any other reload would wipe edits not yet applied. Undo and redo load
    // the cache their snapshot holds, which belongs to the object it selects.
    const ModelObject *mo = m_c->selection_info()->model_object();
    if (m_state == On && mo && mo->id() != m_old_mo_id) {
        if (!is_serializing) {
            // The selection has already moved, so the close check would compare the new object with itself: edits left
            // on the object it moved from are put to the user here, and written only into that object.
            if (cache_differs_from_model(model_object_by_id(m_old_mo_id))) {
                ScaffoldPoints points;
                for (const CacheEntry &entry : m_editing_cache) points.push_back(entry.point);
                wxGetApp().CallAfter([this, object_id = m_old_mo_id, instance_id = m_old_instance_id, points = std::move(points),
                                      status = cache_status()]() {
                    MessageDialog dlg(wxGetApp().mainframe, _L("Apply your scaffold point edits?"), _L("Scaffold Points"),
                                      wxICON_QUESTION | wxYES | wxNO);
                    if (dlg.ShowModal() != wxID_YES) return;
                    ModelObject         *edited   = model_object_by_id(object_id);
                    const ModelInstance *instance = instance_by_id(edited, instance_id);
                    if (instance != nullptr) write_points(*edited, *instance, points, status);
                });
            }
            reload_cache();
            m_generate_failure.clear();
            // The slider starts at the density the object's last auto slice ran at.
            const std::shared_ptr<const ScaffoldSupport::Candidates> candidates = density_candidates();
            m_density = candidates ? float(ScaffoldSupport::density_of(*candidates)) : 1.f;
        }
        m_old_mo_id = mo->id();
    }
    if (m_state == On && mo) {
        const int active = m_c->selection_info()->get_active_instance();
        if (active >= 0 && active < int(mo->instances.size())) m_old_instance_id = mo->instances[active]->id();
    }
    // A slice finishing or failing reloads the scene, which lands here.
    finish_pending_generate();
    if (m_state == On) update_results();
}

void GLGizmoScaffoldPoints::on_render()
{
    ModelObject     *mo        = m_c->selection_info()->model_object();
    const Selection &selection = m_parent.get_selection();

    // If current m_c->m_model_object does not match selection, ask GLCanvas3D to turn us off. The tool can stay open
    // with no selection while it asks about unapplied edits, so the object index is checked before it is used.
    const int obj_idx = selection.get_object_idx();
    if (m_state == On && (mo == nullptr || obj_idx < 0 || obj_idx >= int(selection.get_model()->objects.size()) ||
                          mo != selection.get_model()->objects[obj_idx] ||
                          m_c->selection_info()->get_active_instance() != selection.get_instance_idx())) {
        if (!m_close_asked) m_parent.post_event(SimpleEvent(EVT_GLCANVAS_RESETGIZMOS));
        return;
    }

    glsafe(::glEnable(GL_BLEND));
    glsafe(::glEnable(GL_DEPTH_TEST));

    if (selection.is_from_single_instance()) render_points(selection);

    m_selection_rectangle.render(m_parent);
    m_c->object_clipper()->render_cut();

    glsafe(::glDisable(GL_BLEND));
}

void GLGizmoScaffoldPoints::render_points(const Selection &selection)
{
    if (m_editing_cache.empty() && m_bare_islands.empty()) return;

    GLShaderProgram *shader = wxGetApp().get_shader("gouraud_light");
    if (shader == nullptr) return;
    shader->start_using();
    ScopeGuard guard([shader]() { shader->stop_using(); });

    const Camera      &camera                          = wxGetApp().plater()->get_camera();
    const Transform3d &view_matrix                     = camera.get_view_matrix();
    const GLVolume    *vol                             = selection.get_volume(*selection.get_volume_idxs().begin());
    const Transform3d  instance_scaling_matrix_inverse = vol->get_instance_transformation().get_scaling_factor_matrix().inverse();
    const Transform3d &instance_matrix                 = vol->get_instance_transformation().get_matrix();
    const double       width                           = support_width();

    shader->set_uniform("projection_matrix", camera.get_projection_matrix());
    shader->set_uniform("emission_factor", 0.5f);

    auto render_at = [shader, &view_matrix](GLModel &model, const Transform3d &matrix) {
        shader->set_uniform("view_model_matrix", view_matrix * matrix);
        const Matrix3d view_normal_matrix = view_matrix.matrix().block(0, 0, 3, 3) * matrix.matrix().block(0, 0, 3, 3).inverse().transpose();
        shader->set_uniform("view_normal_matrix", view_normal_matrix);
        model.render();
    };

    if (vol->is_left_handed()) glsafe(::glFrontFace(GL_CW));

    for (size_t i = 0; i < m_editing_cache.size(); ++i) {
        const CacheEntry &entry = m_editing_cache[i];
        const ColorRGBA   color = size_t(m_hover_id) == i ? HOVER_COLOR :
                                  entry.selected          ? SELECTED_COLOR :
                                  entry.result            ? result_color(*entry.result) :
                                                            UNSLICED_COLOR;
        m_sphere.model.set_color(color);
        m_cone.set_color(color);

        // Inverse matrix of the instance scaling is applied so that the mark does not scale with the object.
        const Transform3d point_matrix = instance_matrix * Geometry::translation_transform(entry.point.pos.cast<double>()) *
                                         instance_scaling_matrix_inverse;
        Eigen::Quaterniond q;
        q.setFromTwoVectors(Vec3d::UnitZ(), instance_scaling_matrix_inverse * entry.normal.cast<double>());

        const double radius = entry.point.size == ScaffoldHeadSize::Heavy ? 2. * width : width;
        render_at(m_cone, point_matrix * q *
                              Geometry::assemble_transform((CONE_HEIGHT + radius) * Vec3d::UnitZ(), Vec3d(PI, 0., 0.),
                                                           Vec3d(CONE_RADIUS, CONE_RADIUS, CONE_HEIGHT)));

        const Transform3d sphere_matrix = point_matrix * Geometry::scale_transform(radius);
        if (i < m_grabbers.size()) m_grabbers[i].raycasters[0]->set_transform(sphere_matrix);
        render_at(m_sphere.model, sphere_matrix);
    }

    if (vol->is_left_handed()) glsafe(::glFrontFace(GL_CCW));

    // A bare island lies flat on its layer, so its disc stays level whatever the instance's rotation.
    for (const Vec3f &island : m_bare_islands)
        render_at(m_disc, Geometry::translation_transform(instance_matrix * island.cast<double>()) *
                              Geometry::scale_transform(Vec3d(2. * width, 2. * width, DISC_THICKNESS)));
}

bool GLGizmoScaffoldPoints::on_mouse(const wxMouseEvent &mouse_event)
{
    const Vec2d mouse_pos(mouse_event.GetX(), mouse_event.GetY());
    // when control is down we allow scene pan and rotation even when clicking over some object
    const bool control_down           = mouse_event.CmdDown();
    const bool grabber_contains_mouse = get_hover_id() != -1;

    if (mouse_event.LeftDown()) {
        if ((!control_down || grabber_contains_mouse) &&
            gizmo_event(SLAGizmoEventType::LeftDown, mouse_pos, mouse_event.ShiftDown(), mouse_event.AltDown(), false))
            return true;
    } else if (mouse_event.RightDown()) {
        if (!control_down && m_parent.get_selection().get_object_idx() != -1 &&
            gizmo_event(SLAGizmoEventType::RightDown, mouse_pos, false, false, false))
            return true;
    } else if (mouse_event.Dragging()) {
        if (m_parent.get_move_volume_id() != -1)
            // don't allow dragging objects with the gizmo on
            return true;
        if (!control_down && gizmo_event(SLAGizmoEventType::Dragging, mouse_pos, mouse_event.ShiftDown(), mouse_event.AltDown(), false)) {
            m_parent.set_as_dirty();
            return true;
        }
        if (control_down && (mouse_event.LeftIsDown() || mouse_event.RightIsDown())) {
            // CTRL has been pressed while already dragging -> stop current action
            gizmo_event(mouse_event.LeftIsDown() ? SLAGizmoEventType::LeftUp : SLAGizmoEventType::RightUp, mouse_pos, mouse_event.ShiftDown(),
                        mouse_event.AltDown(), true);
            return false;
        }
    } else if (mouse_event.LeftUp()) {
        if (gizmo_event(SLAGizmoEventType::LeftUp, mouse_pos, mouse_event.ShiftDown(), mouse_event.AltDown(), control_down) &&
            !m_parent.is_mouse_dragging())
            return true;
    }
    return use_grabbers(mouse_event);
}

// Called from GLCanvas3D to inform the gizmo about a mouse or keyboard event. Returns true when the gizmo took the event,
// so that the canvas does not make different sense of it.
bool GLGizmoScaffoldPoints::gizmo_event(SLAGizmoEventType action, const Vec2d &mouse_position, bool shift_down, bool alt_down, bool control_down)
{
    const ModelObject *mo = m_c->selection_info()->model_object();
    if (mo == nullptr) return false;

    // left down with shift or alt: start the selection rectangle, or toggle the hovered point
    if (action == SLAGizmoEventType::LeftDown && (shift_down || alt_down || control_down)) {
        if (m_hover_id == -1) {
            if (shift_down || alt_down)
                m_selection_rectangle.start_dragging(mouse_position, shift_down ? GLSelectionRectangle::Select : GLSelectionRectangle::Deselect);
        } else if (m_editing_cache[m_hover_id].selected)
            unselect_point(m_hover_id);
        else if (!alt_down)
            select_point(m_hover_id);
        return true;
    }

    // left down without selection rectangle: place a point on the mesh
    if (action == SLAGizmoEventType::LeftDown && !m_selection_rectangle.is_dragging() && !shift_down) {
        // A hovered point starts a drag, which the grabbers handle.
        if (m_hover_id != -1) return false;

        // A click with points selected clears the selection instead of adding a point.
        if (!m_selection_empty) {
            select_point(NoPoints);
            return true;
        }

        Vec3f pos, normal;
        Vec3d world_normal;
        if (!unproject_on_mesh(mouse_position, pos, normal, world_normal)) return false;

        m_click_refused = faces_up(world_normal);
        if (!m_click_refused) {
            Plater::TakeSnapshot snapshot(wxGetApp().plater(), "Add scaffold point");
            CacheEntry &entry = m_editing_cache.emplace_back(ScaffoldPoint{pos, m_new_point_size, true});
            entry.normal      = normal;
            update_raycasters();
        }
        m_wait_for_up_event = true;
        m_parent.set_as_dirty();
        return true;
    }

    // left up with selection rectangle: select the visible points inside the rectangle
    if ((action == SLAGizmoEventType::LeftUp || action == SLAGizmoEventType::ShiftUp || action == SLAGizmoEventType::AltUp) &&
        m_selection_rectangle.is_dragging()) {
        const GLSelectionRectangle::EState rectangle_status = m_selection_rectangle.get_state();

        const int                      active_inst = m_c->selection_info()->get_active_instance();
        const Geometry::Transformation trafo       = mo->instances[active_inst]->get_transformation();
        std::vector<Vec3d>             points;
        for (const CacheEntry &entry : m_editing_cache) points.push_back(trafo.get_matrix() * entry.point.pos.cast<double>());

        const std::vector<unsigned int> points_idxs = m_selection_rectangle.contains(points);
        m_selection_rectangle.stop_dragging();
        std::vector<Vec3f> points_inside;
        for (unsigned int idx : points_idxs) points_inside.push_back(points[idx].cast<float>());

        // Check the base of each point's cone too, so the points don't hide under every small irregularity of the model.
        const size_t orig_pts_num = points_inside.size();
        for (unsigned int idx : points_idxs)
            points_inside.emplace_back((trafo.get_matrix() * (m_editing_cache[idx].point.pos + m_editing_cache[idx].normal).cast<double>()).cast<float>());

        if (const MeshRaycaster *raycaster = m_c->raycaster() ? m_c->raycaster()->raycaster() : nullptr)
            for (size_t idx : raycaster->get_unobscured_idxs(trafo, wxGetApp().plater()->get_camera(), points_inside,
                                                             m_c->object_clipper()->get_clipping_plane())) {
                if (idx >= orig_pts_num) // a cone base: take the index of the point it belongs to
                    idx -= orig_pts_num;
                if (rectangle_status == GLSelectionRectangle::Deselect)
                    unselect_point(points_idxs[idx]);
                else
                    select_point(points_idxs[idx]);
            }
        return true;
    }

    // left up with no selection rectangle
    if (action == SLAGizmoEventType::LeftUp) {
        m_wait_for_up_event = false;
        return true;
    }

    // dragging the selection rectangle
    if (action == SLAGizmoEventType::Dragging) {
        // A point has been placed and the button not released yet: keep GLCanvas from rotating the scene.
        if (m_wait_for_up_event) return true;
        if (m_selection_rectangle.is_dragging()) {
            m_selection_rectangle.dragging(mouse_position);
            return true;
        }
        return false;
    }

    if (action == SLAGizmoEventType::Delete) {
        // Taken even with nothing selected, so that Delete never falls through to deleting the object.
        delete_selected_points();
        return true;
    }

    if (action == SLAGizmoEventType::RightDown) {
        if (m_hover_id == -1) return false;
        select_point(NoPoints);
        select_point(m_hover_id);
        delete_selected_points();
        return true;
    }

    if (action == SLAGizmoEventType::SelectAll) {
        select_point(AllPoints);
        return true;
    }

    if ((action == SLAGizmoEventType::MouseWheelUp || action == SLAGizmoEventType::MouseWheelDown) && control_down) {
        toggle_head_size();
        return true;
    }

    return false;
}

bool GLGizmoScaffoldPoints::unproject_on_mesh(const Vec2d &mouse_pos, Vec3f &pos, Vec3f &normal, Vec3d &world_normal) const
{
    const Camera        &camera    = wxGetApp().plater()->get_camera();
    const Selection     &selection = m_parent.get_selection();
    const ClippingPlane *clp       = m_c->object_clipper()->get_position() != 0. ? m_c->object_clipper()->get_clipping_plane() : nullptr;

    double closest = std::numeric_limits<double>::max();
    bool   found   = false;
    for (unsigned int idx : selection.get_volume_idxs()) {
        const GLVolume    *v  = selection.get_volume(idx);
        const ModelVolume *mv = get_model_volume(*v, wxGetApp().model());
        if (mv == nullptr || !mv->is_model_part() || !v->mesh_raycaster) continue;

        const Transform3d volume_trafo = v->get_volume_transformation().get_matrix();
        const Transform3d world_trafo  = v->get_instance_transformation().get_matrix() * volume_trafo;
        Vec3f             hit, hit_normal;
        if (!v->mesh_raycaster->unproject_on_mesh(mouse_pos, world_trafo, camera, hit, hit_normal, clp)) continue;

        const double dist = (camera.get_position() - world_trafo * hit.cast<double>()).norm();
        if (dist >= closest) continue;
        closest      = dist;
        found        = true;
        pos          = (volume_trafo * hit.cast<double>()).cast<float>();
        normal       = (volume_trafo.linear().inverse().transpose() * hit_normal.cast<double>()).normalized().cast<float>();
        world_normal = (world_trafo.linear().inverse().transpose() * hit_normal.cast<double>()).normalized();
    }
    return found;
}

void GLGizmoScaffoldPoints::on_start_dragging()
{
    if (m_hover_id != -1) {
        select_point(NoPoints);
        select_point(m_hover_id);
        m_point_before_drag = m_editing_cache[m_hover_id];
    } else
        m_point_before_drag.reset();
}

void GLGizmoScaffoldPoints::on_dragging(const UpdateData &data)
{
    if (m_hover_id == -1) return;

    Vec3f pos, normal;
    Vec3d world_normal;
    // The point follows the mesh and stays where it was over a face that points up, as a click there adds nothing.
    if (!unproject_on_mesh(data.mouse_pos.cast<double>(), pos, normal, world_normal) || faces_up(world_normal)) return;
    m_editing_cache[m_hover_id].point.pos = pos;
    m_editing_cache[m_hover_id].normal    = normal;
    // The last slice placed the point elsewhere.
    m_editing_cache[m_hover_id].result.reset();
}

void GLGizmoScaffoldPoints::on_stop_dragging()
{
    if (m_hover_id != -1 && m_point_before_drag && m_editing_cache[m_hover_id].point.pos != m_point_before_drag->point.pos) {
        // Momentarily restore the point so that the snapshot holds the state before the move.
        const CacheEntry moved      = m_editing_cache[m_hover_id];
        m_editing_cache[m_hover_id] = *m_point_before_drag;
        Plater::TakeSnapshot snapshot(wxGetApp().plater(), "Move scaffold point");
        m_editing_cache[m_hover_id] = moved;
    }
    m_point_before_drag.reset();
}

void GLGizmoScaffoldPoints::toggle_head_size()
{
    auto other = [](ScaffoldHeadSize size) { return size == ScaffoldHeadSize::Light ? ScaffoldHeadSize::Heavy : ScaffoldHeadSize::Light; };

    const auto first = std::find_if(m_editing_cache.begin(), m_editing_cache.end(), [](const CacheEntry &e) { return e.selected; });
    if (first == m_editing_cache.end())
        m_new_point_size = other(m_new_point_size);
    else {
        Plater::TakeSnapshot   snapshot(wxGetApp().plater(), "Change scaffold point size");
        const ScaffoldHeadSize size = other(first->point.size);
        for (CacheEntry &entry : m_editing_cache)
            if (entry.selected) entry.point.size = size;
    }
    m_parent.set_as_dirty();
}

void GLGizmoScaffoldPoints::apply_changes()
{
    ModelObject *mo     = m_c->selection_info()->model_object();
    const int    active = m_c->selection_info()->get_active_instance();
    if (mo == nullptr || active < 0 || active >= int(mo->instances.size())) return;

    ScaffoldPoints points;
    for (const CacheEntry &entry : m_editing_cache) points.push_back(entry.point);
    m_generate_failure.clear();
    write_points(*mo, *mo->instances[active], std::move(points), cache_status());
}

void GLGizmoScaffoldPoints::write_points(ModelObject &mo, const ModelInstance &instance, ScaffoldPoints points, ScaffoldPointsStatus status)
{
    Plater::TakeSnapshot snapshot(wxGetApp().plater(), "Apply scaffold points");
    mo.scaffold_points          = std::move(points);
    mo.scaffold_points_status   = status;
    mo.scaffold_points_pose     = instance.get_matrix().linear();
    mo.scaffold_points_mesh_box = mo.raw_mesh_bounding_box();
    wxGetApp().plater()->set_plater_dirty(true);
    select_plate_of(mo, instance);
    wxGetApp().plater()->reslice();
}

void GLGizmoScaffoldPoints::revert_to_auto()
{
    ModelObject *mo = m_c->selection_info()->model_object();
    if (mo == nullptr) return;

    Plater::TakeSnapshot snapshot(wxGetApp().plater(), "Revert scaffold points to auto");
    mo->clear_scaffold_points();
    m_editing_cache.clear();
    m_selection_empty = true;
    update_raycasters();
    m_generate_failure.clear();
    wxGetApp().plater()->set_plater_dirty(true);
    m_parent.post_event(SimpleEvent(EVT_GLCANVAS_SCHEDULE_BACKGROUND_PROCESS));
    m_parent.set_as_dirty();
}

void GLGizmoScaffoldPoints::generate()
{
    ModelObject *mo     = m_c->selection_info()->model_object();
    const int    active = m_c->selection_info()->get_active_instance();
    if (mo == nullptr || active < 0 || active >= int(mo->instances.size()) || m_generate_pending) return;

    if (mo->scaffold_points_status == ScaffoldPointsStatus::UserModified && !mo->scaffold_points.empty()) {
        MessageDialog dlg(wxGetApp().mainframe, _L("Generate will replace your edited scaffold points. Continue?"), _L("Scaffold Points"),
                          wxICON_WARNING | wxYES | wxNO);
        if (dlg.ShowModal() != wxID_YES) return;
    }

    Plater::TakeSnapshot snapshot(wxGetApp().plater(), "Generate scaffold points");
    m_generate_failure.clear();
    m_generate_stash = {mo->id(), mo->instances[active]->id(), mo->scaffold_points, mo->scaffold_points_status, mo->scaffold_points_pose};

    select_plate_of(*mo, *mo->instances[active]);
    if (const std::shared_ptr<const ScaffoldRecord> record = finished_record(selected_print_object()); record && !record->baked && !record->stale) {
        // The plate's last slice placed its contact points automatically: copy them now.
        m_generate_pending = true;
        finish_pending_generate();
        return;
    }

    mo->scaffold_points_status = ScaffoldPointsStatus::NoPoints;
    wxGetApp().plater()->reslice();
    // Set only once reslice() has returned: it reloads the scene before it marks the slice as running, and a pending
    // Generate seen from that reload would read an unfinished step and no slice, and give up.
    m_generate_pending = true;
    // reslice() can return without starting a slice, and then no reload reaches data_changed.
    finish_pending_generate();
}

void GLGizmoScaffoldPoints::finish_pending_generate()
{
    if (!m_generate_pending) return;

    ModelObject         *mo       = model_object_by_id(m_generate_stash.object_id);
    const ModelInstance *instance = instance_by_id(mo, m_generate_stash.instance_id);
    const PrintObject   *po       = instance ? print_object_of(*mo, m_generate_stash.instance_id) : nullptr;

    // Waits for the whole slice, G-code export included: the copy invalidates the support step, which would cancel it.
    if (instance != nullptr && wxGetApp().plater()->is_background_process_slicing()) return;
    if (instance != nullptr && (po == nullptr || !po->is_step_done(posSupportMaterial))) {
        restore_generate_stash();
        m_generate_failure = _L("Generate stopped: the slice did not finish. No points written.");
        return;
    }

    // The slice finished: its tips are copied only when they were taken from the object as it stands now, placed
    // automatically, in the pose the selected instance holds.
    const std::shared_ptr<const ScaffoldRecord> record = finished_record(po);
    auto model_part_meshes = [](const ModelObject &object) {
        std::vector<const TriangleMesh *> meshes;
        for (const ModelVolume *mv : object.volumes)
            if (mv->is_model_part()) meshes.push_back(mv->mesh_ptr().get());
        return meshes;
    };
    const bool unchanged = record && !record->baked && !record->stale && model_part_meshes(*mo) == model_part_meshes(*po->model_object()) &&
                           (instance->get_matrix().linear() - record->pose).cwiseAbs().maxCoeff() <= 1e-9;
    if (!unchanged) {
        restore_generate_stash();
        m_generate_failure = _L("Generate stopped: the object changed during the slice. No points written.");
        return;
    }

    m_generate_pending           = false;
    mo->scaffold_points          = scaffold_points_from(*record);
    mo->scaffold_points_status   = ScaffoldPointsStatus::AutoGenerated;
    mo->scaffold_points_pose     = record->pose;
    mo->scaffold_points_mesh_box = mo->raw_mesh_bounding_box();
    wxGetApp().plater()->set_plater_dirty(true);
    if (const ModelObject *selected = m_c->selection_info()->model_object(); selected != nullptr && selected->id() == mo->id())
        reload_cache();
    m_parent.post_event(SimpleEvent(EVT_GLCANVAS_SCHEDULE_BACKGROUND_PROCESS));
    m_parent.set_as_dirty();
}

void GLGizmoScaffoldPoints::restore_generate_stash()
{
    m_generate_pending = false;
    if (ModelObject *mo = model_object_by_id(m_generate_stash.object_id)) {
        mo->scaffold_points        = m_generate_stash.points;
        mo->scaffold_points_status = m_generate_stash.status;
        mo->scaffold_points_pose   = m_generate_stash.pose;
        // The Print took the cleared status; it takes the restored list back.
        m_parent.post_event(SimpleEvent(EVT_GLCANVAS_SCHEDULE_BACKGROUND_PROCESS));
    }
    m_parent.set_as_dirty();
}

void GLGizmoScaffoldPoints::update_results()
{
    const std::shared_ptr<const ScaffoldRecord> record = finished_record(selected_print_object());
    if (!record) {
        m_bare_islands.clear();
        return;
    }
    m_bare_islands = record->bare_islands;

    // A slice that did not build from the list says nothing about its points.
    if (!record->baked || record->stale) {
        for (CacheEntry &entry : m_editing_cache) entry.result.reset();
        return;
    }
    // A baked record holds one tip per list point in list order; a cache edited since then keeps the results it has,
    // and a point added since shows as not yet sliced.
    if (record->tips.size() != m_editing_cache.size()) return;
    for (size_t i = 0; i < m_editing_cache.size(); ++i)
        if (record->tips[i].pos != m_editing_cache[i].point.pos) return;
    for (size_t i = 0; i < m_editing_cache.size(); ++i) m_editing_cache[i].result = record->tips[i].result;
}

void GLGizmoScaffoldPoints::delete_selected_points()
{
    if (!has_selected_points()) return;

    Plater::TakeSnapshot snapshot(wxGetApp().plater(), "Delete scaffold points");
    m_editing_cache.erase(std::remove_if(m_editing_cache.begin(), m_editing_cache.end(), [](const CacheEntry &e) { return e.selected; }),
                          m_editing_cache.end());
    select_point(NoPoints);
    update_raycasters();
    m_parent.set_as_dirty();
}

bool GLGizmoScaffoldPoints::has_selected_points() const
{
    return std::any_of(m_editing_cache.begin(), m_editing_cache.end(), [](const CacheEntry &e) { return e.selected; });
}

void GLGizmoScaffoldPoints::unselect_point(int i)
{
    m_editing_cache[i].selected = false;
    m_selection_empty           = !has_selected_points();
}

void GLGizmoScaffoldPoints::select_point(int i)
{
    if (i == AllPoints || i == NoPoints) {
        for (CacheEntry &entry : m_editing_cache) entry.selected = (i == AllPoints);
        m_selection_empty = (i == NoPoints) || m_editing_cache.empty();
    } else {
        m_editing_cache[i].selected = true;
        m_selection_empty           = false;
    }
}

std::shared_ptr<const ScaffoldSupport::Candidates> GLGizmoScaffoldPoints::density_candidates() const
{
    const PrintObject *po = selected_print_object();
    return po != nullptr && po->is_step_done(posSlice) ? po->scaffold_candidates() : nullptr;
}

void GLGizmoScaffoldPoints::retune()
{
    const PrintObject                                       *po         = selected_print_object();
    const std::shared_ptr<const ScaffoldSupport::Candidates> candidates = density_candidates();
    if (po == nullptr || candidates == nullptr) return;

    if (candidates != m_density_candidates) {
        m_density_candidates = candidates;
        m_density_grades     = ScaffoldSupport::grades_of(*candidates);
    }
    ScaffoldPoints points = ScaffoldSupport::retune_points(*po, *candidates, m_density, m_density_grades);
    m_editing_cache.clear();
    for (const ScaffoldPoint &point : points) {
        CacheEntry &entry = m_editing_cache.emplace_back(point);
        entry.normal      = normal_at(point.pos);
    }
    m_density_points  = std::move(points);
    m_selection_empty = true;
    m_click_refused   = false;
    update_raycasters();
    m_parent.set_as_dirty();
}

ScaffoldPointsStatus GLGizmoScaffoldPoints::cache_status() const
{
    if (!m_density_points || m_density_points->size() != m_editing_cache.size()) return ScaffoldPointsStatus::UserModified;
    for (size_t i = 0; i < m_editing_cache.size(); ++i)
        if (m_editing_cache[i].point != (*m_density_points)[i]) return ScaffoldPointsStatus::UserModified;
    return ScaffoldPointsStatus::AutoGenerated;
}

bool GLGizmoScaffoldPoints::has_hand_edits(const ModelObject *mo) const
{
    return cache_differs_from_model(mo) && cache_status() == ScaffoldPointsStatus::UserModified;
}

bool GLGizmoScaffoldPoints::cache_differs_from_model(const ModelObject *mo) const
{
    if (mo == nullptr) return false;
    if (mo->scaffold_points.size() != m_editing_cache.size()) return true;
    for (size_t i = 0; i < m_editing_cache.size(); ++i)
        if (m_editing_cache[i].point != mo->scaffold_points[i]) return true;
    return false;
}

void GLGizmoScaffoldPoints::reload_cache()
{
    const ModelObject *mo = m_c->selection_info()->model_object();
    m_editing_cache.clear();
    if (mo != nullptr)
        for (const ScaffoldPoint &point : mo->scaffold_points) {
            CacheEntry &entry = m_editing_cache.emplace_back(point);
            entry.normal      = normal_at(point.pos);
        }
    m_density_points.reset();
    m_selection_empty = true;
    update_raycasters();
}

Vec3f GLGizmoScaffoldPoints::normal_at(const Vec3f &pos) const
{
    // A contact point sits on an underside, so a point off every model part faces down.
    Vec3f              normal = -Vec3f::UnitZ();
    const ModelObject *mo     = m_c->selection_info()->model_object();
    if (mo == nullptr || m_c->raycaster() == nullptr) return normal;

    // The pool holds one raycaster per model part, or one per volume once a gizmo has cleared its model-part flag.
    const std::vector<const MeshRaycaster *> raycasters = m_c->raycaster()->raycasters();
    const bool                               per_volume = raycasters.size() == mo->volumes.size();
    float                                    best       = std::numeric_limits<float>::max();
    size_t                                   next       = 0;
    for (const ModelVolume *mv : mo->volumes) {
        if (!per_volume && !mv->is_model_part()) continue;
        if (next == raycasters.size()) break;
        const MeshRaycaster *raycaster = raycasters[next++];
        if (!mv->is_model_part()) continue;

        const Transform3d &trafo = mv->get_matrix();
        Vec3f              local_normal;
        const Vec3f        hit  = raycaster->get_closest_point((trafo.inverse() * pos.cast<double>()).cast<float>(), &local_normal);
        const float        dist = ((trafo * hit.cast<double>()).cast<float>() - pos).squaredNorm();
        if (dist < best) {
            best   = dist;
            normal = (trafo.linear().inverse().transpose() * local_normal.cast<double>()).normalized().cast<float>();
        }
    }
    return normal;
}

void GLGizmoScaffoldPoints::select_plate_of(const ModelObject &mo, const ModelInstance &instance)
{
    const ModelObjectPtrs &objects  = wxGetApp().model().objects;
    const int              obj_idx  = int(std::find(objects.begin(), objects.end(), &mo) - objects.begin());
    const int              inst_idx = int(std::find(mo.instances.begin(), mo.instances.end(), &instance) - mo.instances.begin());
    if (obj_idx == int(objects.size()) || inst_idx == int(mo.instances.size())) return;

    Plater    *plater    = wxGetApp().plater();
    const int  plate_idx = plater->get_partplate_list().find_instance_belongs(obj_idx, inst_idx);
    if (plate_idx >= 0 && plate_idx != plater->get_partplate_list().get_curr_plate_index()) plater->select_plate(plate_idx);
}

const PrintObject *GLGizmoScaffoldPoints::print_object_of(const ModelObject &mo, ObjectID instance_id) const
{
    const Print *print = m_parent.fff_print();
    if (print == nullptr) return nullptr;
    for (const PrintObject *po : print->objects())
        if (po->model_object()->id() == mo.id())
            for (const PrintInstance &pi : po->instances())
                if (pi.model_instance->id() == instance_id) return po;
    return nullptr;
}

const PrintObject *GLGizmoScaffoldPoints::selected_print_object() const
{
    const ModelObject *mo = m_c->selection_info() ? m_c->selection_info()->model_object() : nullptr;
    if (mo == nullptr) return nullptr;

    const int active = m_c->selection_info()->get_active_instance();
    if (active < 0 || active >= int(mo->instances.size())) return nullptr;
    return print_object_of(*mo, mo->instances[active]->id());
}

double GLGizmoScaffoldPoints::support_width() const
{
    if (const std::shared_ptr<const ScaffoldRecord> record = finished_record(selected_print_object()); record && record->toolpath_width_mm > 0.)
        return record->toolpath_width_mm;
    // Before the first slice: the support width the slice would derive from the nozzle.
    const double nozzle_diameter = wxGetApp().preset_bundle->printers.get_edited_preset().config.option<ConfigOptionFloats>("nozzle_diameter")->get_at(0);
    return Flow::auto_extrusion_width(frSupportMaterial, float(nozzle_diameter));
}

void GLGizmoScaffoldPoints::on_render_input_window(float x, float y, float bottom_limit)
{
    static float last_y = 0.0f;
    static float last_h = 0.0f;

    // A cancelled or failed slice reloads the scene while it still reads as running, so no later data_changed sees it
    // end; the next frame does.
    finish_pending_generate();

    const ModelObject *mo = m_c->selection_info()->model_object();
    if (!mo) return;

    const float win_h = ImGui::GetWindowHeight();
    y                 = std::min(y, bottom_limit - win_h);
    GizmoImguiSetNextWIndowPos(x, y, ImGuiCond_Always, 0.0f, 0.0f);

    const float f_scale = m_parent.get_scale();
    ImGuiWrapper::push_toolbar_style(f_scale);
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(6.0f, 4.0f * f_scale));
    GizmoImguiBegin(get_name(),
                    ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar);

    const float           space_size   = m_imgui->get_style_scaling() * 8;
    std::vector<wxString> captions     = {m_desc["head_size"], m_desc["create"], m_desc["density"], m_desc["remove"]};
    const float           caption_size = m_imgui->find_widest_text(captions) + space_size + ImGui::GetStyle().WindowPadding.x;

    // adjust window position to avoid overlap the view toolbar
    if (last_h != win_h || last_y != y) {
        // ask canvas for another frame to render the window in the correct position
        m_imgui->set_requires_extra_frame();
        last_h = win_h;
        last_y = y;
    }

    ImGui::AlignTextToFramePadding();
    m_imgui->text(m_desc["head_size"]);
    ImGui::SameLine(caption_size);
    if (m_imgui->radio_button(m_desc["light"], m_new_point_size == ScaffoldHeadSize::Light)) m_new_point_size = ScaffoldHeadSize::Light;
    ImGui::SameLine();
    if (m_imgui->radio_button(m_desc["heavy"], m_new_point_size == ScaffoldHeadSize::Heavy)) m_new_point_size = ScaffoldHeadSize::Heavy;

    ImGui::Separator();

    ImGui::AlignTextToFramePadding();
    m_imgui->text(m_desc["create"]);
    ImGui::SameLine(caption_size);
    // Each runs after the frame: Generate may ask a question and both Generate and Apply reslice, which reloads the scene.
    m_imgui->disabled_begin(m_generate_pending);
    if (m_imgui->button(m_desc["generate"], m_desc["generate_tooltip"])) wxGetApp().CallAfter([this]() {
        if (m_state == On) generate();
    });
    m_imgui->disabled_end();
    ImGui::SameLine();
    m_imgui->disabled_begin(m_generate_pending ||
                            (mo->scaffold_points_status == ScaffoldPointsStatus::NoPoints && mo->scaffold_points.empty() && m_editing_cache.empty()));
    if (m_imgui->button(m_desc["revert"], m_desc["revert_tooltip"])) revert_to_auto();
    m_imgui->disabled_end();

    // The slider selects again from the contacts the last auto slice placed; hand edits and a user list stay unless the
    // user discards or reverts them.
    const bool density_ready = density_candidates() != nullptr;
    // A slice running reads the layers the slider reads, and replaces the candidates when it places points itself.
    const bool density_open  = density_ready && !m_generate_pending && !wxGetApp().plater()->is_background_process_slicing() &&
                               !has_hand_edits(mo) &&
                               !(mo->scaffold_points_status == ScaffoldPointsStatus::UserModified && !cache_differs_from_model(mo));
    ImGui::AlignTextToFramePadding();
    m_imgui->text(m_desc["density"]);
    ImGui::SameLine(caption_size);
    m_imgui->disabled_begin(!density_open);
    const float slider_width = m_imgui->calc_text_size(m_desc["density_light"] + m_desc["density_medium"] + m_desc["density_heavy"]).x +
                               4.f * space_size;
    ImGui::PushItemWidth(slider_width);
    // The value before this frame's move, which the drag's snapshot holds with the points from before the drag.
    const float density_before = m_density;
    const bool  density_moved  = m_imgui->slider_float("##scaffold_density", &m_density, 0.f, 2.f, "", 1.f, true, m_desc["density_tooltip"], false);
    if (ImGui::IsItemActivated()) {
        const float density_after = m_density;
        m_density                 = density_before;
        Plater::TakeSnapshot snapshot(wxGetApp().plater(), "Scaffold density");
        m_density = density_after;
    }
    ImGui::PopItemWidth();
    const ImVec2 slider_min = ImGui::GetItemRectMin(), slider_max = ImGui::GetItemRectMax();
    m_imgui->disabled_end();
    if (density_moved && density_open) retune();
    // The tier names under the slider, at its ends and its middle.
    ImGui::SetCursorPosX(caption_size);
    const float cursor_y = ImGui::GetCursorPosY();
    m_imgui->text(m_desc["density_light"]);
    ImGui::SameLine();
    ImGui::SetCursorPos({caption_size + 0.5f * (slider_max.x - slider_min.x - m_imgui->calc_text_size(m_desc["density_medium"]).x), cursor_y});
    m_imgui->text(m_desc["density_medium"]);
    ImGui::SameLine();
    ImGui::SetCursorPos({caption_size + slider_max.x - slider_min.x - m_imgui->calc_text_size(m_desc["density_heavy"]).x, cursor_y});
    m_imgui->text(m_desc["density_heavy"]);

    ImGui::AlignTextToFramePadding();
    m_imgui->text(m_desc["remove"]);
    ImGui::SameLine(caption_size);
    m_imgui->disabled_begin(!has_selected_points());
    if (m_imgui->button(m_desc["remove_selected"])) delete_selected_points();
    m_imgui->disabled_end();
    ImGui::SameLine();
    m_imgui->disabled_begin(m_editing_cache.empty());
    if (m_imgui->button(m_desc["remove_all"])) {
        select_point(AllPoints);
        delete_selected_points();
    }
    m_imgui->disabled_end();

    ImGui::Separator();

    const std::vector<std::pair<ColorRGBA, wxString>> legend = {
        {result_color(ScaffoldTipResult::Routed), _L("Routed")},     {result_color(ScaffoldTipResult::Filtered), _L("Filtered")},
        {result_color(ScaffoldTipResult::Unrouted), _L("Unrouted")}, {result_color(ScaffoldTipResult::Neck), _L("Neck")},
        {result_color(ScaffoldTipResult::Merged), _L("Merged")},     {result_color(ScaffoldTipResult::Wall), _L("Wall")},
        {UNSLICED_COLOR, _L("Not yet sliced")},                      {SELECTED_COLOR, _L("Selected")},
        {BARE_ISLAND_COLOR, _L("Bare island")},
    };
    const float swatch = ImGui::GetTextLineHeight();
    float       widest = 0.f;
    for (const auto &[color, label] : legend) widest = std::max(widest, m_imgui->calc_text_size(label).x);
    const float column = swatch + widest + 2.f * space_size;
    for (size_t i = 0; i < legend.size(); ++i) {
        if (i % 2 == 1) ImGui::SameLine(ImGui::GetStyle().WindowPadding.x + column);
        const ImVec2 pos = ImGui::GetCursorScreenPos();
        ImGui::GetWindowDrawList()->AddRectFilled(pos, ImVec2(pos.x + swatch, pos.y + swatch), ImGuiWrapper::to_ImU32(legend[i].first));
        ImGui::Dummy(ImVec2(swatch, swatch));
        ImGui::SameLine();
        m_imgui->text(legend[i].second);
    }

    ImGui::Separator();

    const std::shared_ptr<const ScaffoldRecord> record = finished_record(selected_print_object());
    if (record) {
        const size_t routed = std::count_if(record->tips.begin(), record->tips.end(),
                                            [](const ScaffoldRecord::Tip &tip) { return tip.result == ScaffoldTipResult::Routed; });
        m_imgui->text(format_wxstr(_L("Points %1% · routed %2% · dropped %3% · bare islands %4%"), record->tips.size(), routed,
                                   record->tips.size() - routed, record->bare_islands.size()));
    } else
        m_imgui->text(format_wxstr(_L("Points %1%"), m_editing_cache.size()));

    std::vector<wxString> warnings;
    if (record && record->stale) warnings.push_back(_L("Scaffold points are stale for this pose; auto contacts used."));
    if (mo->scaffold_points_status != ScaffoldPointsStatus::NoPoints)
        warnings.push_back(_L("Paint and blockers are ignored while the list is in use."));
    if (cache_differs_from_model(mo)) warnings.push_back(_L("Unapplied edits"));
    if (m_click_refused) warnings.push_back(m_desc["refused"]);
    if (!density_ready)
        warnings.push_back(m_desc["density_unsliced"]);
    else if (has_hand_edits(mo) || (mo->scaffold_points_status == ScaffoldPointsStatus::UserModified && !cache_differs_from_model(mo)))
        warnings.push_back(m_desc["density_edited"]);
    if (!m_generate_failure.empty()) warnings.push_back(m_generate_failure);
    if (!warnings.empty()) {
        ImGui::PushStyleColor(ImGuiCol_Text, ImGuiWrapper::COL_WARNING);
        const float parent_width = ImGui::GetContentRegionAvail().x;
        for (const wxString &warning : warnings) m_imgui->text_wrapped(warning, parent_width);
        ImGui::PopStyleColor(1);
    }

    ImGui::Separator();

    GLGizmoUtils::render_tooltip_button(m_imgui, m_parent, m_shortcuts, x, y);

    ImGui::SameLine();
    GLGizmoUtils::begin_right_aligned_buttons({m_desc["apply"], m_desc["discard"]});
    m_imgui->disabled_begin(m_generate_pending || !cache_differs_from_model(mo));
    if (m_imgui->button(m_desc["apply"])) wxGetApp().CallAfter([this]() {
        if (m_state == On) apply_changes();
    });
    m_imgui->disabled_end();
    ImGui::SameLine();
    m_imgui->disabled_begin(!cache_differs_from_model(mo));
    if (m_imgui->button(m_desc["discard"])) reload_cache();
    m_imgui->disabled_end();

    GizmoImguiEnd();
    ImGui::PopStyleVar(1); // ImGuiStyleVar_FramePadding
    ImGuiWrapper::pop_toolbar_style();
}

bool GLGizmoScaffoldPoints::on_is_activable() const
{
    const Selection &selection = m_parent.get_selection();
    if (!selection.is_single_full_instance()) return false;

    // The object's own value where it sets one, the print preset's otherwise.
    const DynamicPrintConfig &obj_cfg = selection.get_model()->objects[selection.get_object_idx()]->config.get();
    const DynamicPrintConfig &glb_cfg = wxGetApp().preset_bundle->prints.get_edited_preset().config;
    auto config_of = [&obj_cfg, &glb_cfg](const char *key) -> const DynamicPrintConfig & { return obj_cfg.has(key) ? obj_cfg : glb_cfg; };

    return config_of("enable_support").opt_bool("enable_support") && is_tree(config_of("support_type").opt_enum<SupportType>("support_type")) &&
           config_of("support_style").opt_enum<SupportMaterialStyle>("support_style") == smsTreeScaffold;
}

std::string GLGizmoScaffoldPoints::on_get_name() const
{
    if (!on_is_activable() && m_state == EState::Off)
        return _u8L("Scaffold Points") + ":\n" + _u8L("Please select a single object that uses Tree Scaffold support.");
    return _u8L("Scaffold Points");
}

CommonGizmosDataID GLGizmoScaffoldPoints::on_get_requirements() const
{
    return CommonGizmosDataID(int(CommonGizmosDataID::SelectionInfo) | int(CommonGizmosDataID::InstancesHider) |
                              int(CommonGizmosDataID::Raycaster) | int(CommonGizmosDataID::ObjectClipper));
}

void GLGizmoScaffoldPoints::on_set_state()
{
    if (m_state == m_old_state) return;

    if (m_state == On && m_old_state != On)
        wxGetApp().plater()->enter_gizmos_stack();
    if (m_state == Off && m_old_state != Off) {
        // A Generate still waiting on its slice writes nothing once the tool closes.
        if (m_generate_pending) restore_generate_stash();
        // Unapplied edits: refuse to close so that the gizmo is still active when the question is answered, and close it
        // then. The dialog runs through CallAfter, because otherwise on OSX it was shown several times when clicked into.
        if (m_close_asked || cache_differs_from_model(m_c->selection_info()->model_object())) {
            m_state = m_old_state;
            if (m_close_asked) return;
            m_close_asked = true;
            // The edits belong to the object loaded into the cache; an answer never writes them into another one.
            wxGetApp().CallAfter([this, edited_id = m_old_mo_id]() {
                MessageDialog dlg(wxGetApp().mainframe, _L("Apply your scaffold point edits?"), _L("Scaffold Points"),
                                  wxICON_QUESTION | wxYES | wxNO | wxCANCEL);
                const int          ret = dlg.ShowModal();
                const ModelObject *mo  = m_c->selection_info() ? m_c->selection_info()->model_object() : nullptr;
                if (mo != nullptr && mo->id() == edited_id) {
                    if (ret == wxID_YES)
                        apply_changes();
                    else if (ret == wxID_NO)
                        reload_cache();
                } else
                    // The edited object is no longer selected: its edits are dropped so that the tool can close.
                    reload_cache();
                m_close_asked = false;
                if (ret != wxID_CANCEL && m_parent.get_gizmos_manager().get_current_type() == GLGizmosManager::ScaffoldPoints) {
                    m_parent.get_gizmos_manager().reset_all_states();
                    m_parent.set_as_dirty();
                }
            });
            return;
        }
        wxGetApp().plater()->leave_gizmos_stack();
        // Reopening the tool reloads the list from the model.
        m_old_mo_id = ObjectID();
    }
    m_old_state = m_state;
}

void GLGizmoScaffoldPoints::on_load(cereal::BinaryInputArchive &ar)
{
    bool           has_density_points = false;
    ScaffoldPoints density_points;
    ar(m_new_point_size, m_editing_cache, m_selection_empty, m_density, has_density_points, density_points);
    m_density_points = has_density_points ? std::optional<ScaffoldPoints>(std::move(density_points)) : std::nullopt;
    // Undo and redo change the number of points, and each point needs its grabber to be picked.
    update_raycasters();
    // The snapshot holds no results, so they are matched against the last slice again.
    update_results();
}

void GLGizmoScaffoldPoints::on_save(cereal::BinaryOutputArchive &ar) const
{
    ar(m_new_point_size, m_editing_cache, m_selection_empty, m_density, m_density_points.has_value(),
       m_density_points ? *m_density_points : ScaffoldPoints());
}

void GLGizmoScaffoldPoints::on_register_raycasters_for_picking() { update_raycasters(); }

void GLGizmoScaffoldPoints::on_unregister_raycasters_for_picking()
{
    m_parent.remove_raycasters_for_picking(SceneRaycaster::EType::Gizmo);
    m_grabbers.clear();
}

void GLGizmoScaffoldPoints::update_raycasters()
{
    // One grabber per point, picked by the point's sphere.
    if (m_editing_cache.size() < m_grabbers.size()) {
        for (auto it = m_grabbers.begin() + m_editing_cache.size(); it != m_grabbers.end(); ++it)
            if (it->picking_id >= 0) it->unregister_raycasters_for_picking();
        m_grabbers.erase(m_grabbers.begin() + m_editing_cache.size(), m_grabbers.end());
    } else
        while (m_grabbers.size() < m_editing_cache.size()) {
            const auto id = m_grabbers.size();
            auto      &g  = m_grabbers.emplace_back();
            g.register_raycasters_for_picking(id);
            g.raycasters[0] = m_parent.add_raycaster_for_picking(SceneRaycaster::EType::Gizmo, id, *m_sphere.mesh_raycaster, Transform3d::Identity());
        }
}

}} // namespace Slic3r::GUI
