#include "GLGizmoScaffoldPoints.hpp"
#include <glad/gl.h>
#include "slic3r/GUI/GLCanvas3D.hpp"
#include "slic3r/GUI/GUI.hpp"
#include "slic3r/GUI/Camera.hpp"
#include "slic3r/GUI/Gizmos/GLGizmosCommon.hpp"
#include "slic3r/GUI/GUI_App.hpp"
#include "slic3r/GUI/Plater.hpp"
#include "slic3r/GUI/format.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "GLGizmoUtils.hpp"

namespace Slic3r { namespace GUI {

static const ColorRGBA SELECTED_COLOR    = {0.0f, 0.5f, 0.5f, 1.f};
static const ColorRGBA HOVER_COLOR       = {0.f, 1.f, 1.f, 1.f};
static const ColorRGBA UNSLICED_COLOR    = {1.f, 1.f, 1.f, 1.f};
static const ColorRGBA BARE_ISLAND_COLOR = {1.f, 0.9f, 0.1f, 1.f};
// The cone marks the direction a branch arrives from, so it keeps one size whatever the head.
static constexpr double CONE_RADIUS = 0.25;
static constexpr double CONE_HEIGHT = 0.75;

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

GLGizmoScaffoldPoints::GLGizmoScaffoldPoints(GLCanvas3D &parent, const std::string &icon_filename, unsigned int sprite_id)
    : GLGizmoBase(parent, icon_filename, sprite_id)
{
    const indexed_triangle_set sphere = its_make_sphere(1.0, double(PI) / 12.0);
    m_sphere.mesh_raycaster = std::make_unique<MeshRaycaster>(std::make_shared<const TriangleMesh>(sphere));
    m_sphere.model.init_from(sphere);
    m_cone.init_from(its_make_cone(1.0, 1.0, double(PI) / 12.0));
}

bool GLGizmoScaffoldPoints::on_init()
{
    const wxString ctrl = GUI::shortkey_ctrl_prefix();

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

    m_shortcuts = {
        {_L("Left mouse button"),  _L("Add or Select")},
        {_L("Right mouse button"), _L("Remove")},
        {ctrl + _L("Mouse wheel"), m_desc["head_size"]},
    };

    return true;
}

void GLGizmoScaffoldPoints::data_changed(bool is_serializing)
{
    if (!m_c->selection_info()) return;

    // Only a change of object reloads the cache: any other reload would wipe edits not yet applied.
    const ModelObject *mo = m_c->selection_info()->model_object();
    if (m_state == On && mo && mo->id() != m_old_mo_id && !is_serializing) {
        reload_cache();
        m_old_mo_id = mo->id();
    }
}

void GLGizmoScaffoldPoints::on_render()
{
    ModelObject     *mo        = m_c->selection_info()->model_object();
    const Selection &selection = m_parent.get_selection();

    // If current m_c->m_model_object does not match selection, ask GLCanvas3D to turn us off
    if (m_state == On && (mo != selection.get_model()->objects[selection.get_object_idx()] ||
                          m_c->selection_info()->get_active_instance() != selection.get_instance_idx())) {
        m_parent.post_event(SimpleEvent(EVT_GLCANVAS_RESETGIZMOS));
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
    if (m_editing_cache.empty()) return;

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
}

bool GLGizmoScaffoldPoints::gizmo_event(SLAGizmoEventType action, const Vec2d &mouse_position, bool shift_down, bool alt_down, bool control_down)
{
    if (action == SLAGizmoEventType::Delete) {
        // Taken even with nothing selected, so that Delete never falls through to deleting the object.
        delete_selected_points();
        return true;
    }
    return false;
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

bool GLGizmoScaffoldPoints::cache_differs_from_model() const
{
    const ModelObject *mo = m_c->selection_info()->model_object();
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

const PrintObject *GLGizmoScaffoldPoints::selected_print_object() const
{
    const ModelObject *mo    = m_c->selection_info() ? m_c->selection_info()->model_object() : nullptr;
    const Print       *print = m_parent.fff_print();
    if (mo == nullptr || print == nullptr) return nullptr;

    const int active = m_c->selection_info()->get_active_instance();
    if (active < 0 || active >= int(mo->instances.size())) return nullptr;
    const ObjectID instance_id = mo->instances[active]->id();

    for (const PrintObject *po : print->objects())
        if (po->model_object()->id() == mo->id())
            for (const PrintInstance &pi : po->instances())
                if (pi.model_instance->id() == instance_id) return po;
    return nullptr;
}

double GLGizmoScaffoldPoints::support_width() const
{
    if (const PrintObject *po = selected_print_object())
        if (const std::shared_ptr<const ScaffoldRecord> record = po->scaffold_record(); record && record->toolpath_width_mm > 0.)
            return record->toolpath_width_mm;
    // Before the first slice: the support width the slice would derive from the nozzle.
    const double nozzle_diameter = wxGetApp().preset_bundle->printers.get_edited_preset().config.option<ConfigOptionFloats>("nozzle_diameter")->get_at(0);
    return Flow::auto_extrusion_width(frSupportMaterial, float(nozzle_diameter));
}

void GLGizmoScaffoldPoints::on_render_input_window(float x, float y, float bottom_limit)
{
    static float last_y = 0.0f;
    static float last_h = 0.0f;

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
    std::vector<wxString> captions     = {m_desc["head_size"], m_desc["create"], m_desc["remove"]};
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
    m_imgui->disabled_begin(true);
    m_imgui->button(m_desc["generate"], m_desc["generate_tooltip"]);
    ImGui::SameLine();
    m_imgui->button(m_desc["revert"], m_desc["revert_tooltip"]);
    m_imgui->disabled_end();

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

    const PrintObject                          *po     = selected_print_object();
    const std::shared_ptr<const ScaffoldRecord> record = po ? po->scaffold_record() : nullptr;
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
    if (cache_differs_from_model()) warnings.push_back(_L("Unapplied edits"));
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
    m_imgui->disabled_begin(true);
    m_imgui->button(m_desc["apply"]);
    m_imgui->disabled_end();
    ImGui::SameLine();
    m_imgui->disabled_begin(!cache_differs_from_model());
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
        wxGetApp().plater()->leave_gizmos_stack();
        // Reopening the tool reloads the list from the model.
        m_old_mo_id = ObjectID();
    }
    m_old_state = m_state;
}

void GLGizmoScaffoldPoints::on_load(cereal::BinaryInputArchive &ar) { ar(m_new_point_size, m_editing_cache, m_selection_empty); }

void GLGizmoScaffoldPoints::on_save(cereal::BinaryOutputArchive &ar) const { ar(m_new_point_size, m_editing_cache, m_selection_empty); }

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
