#ifndef slic3r_GLGizmoScaffoldPoints_hpp_
#define slic3r_GLGizmoScaffoldPoints_hpp_

#include "GLGizmoBase.hpp"
#include "slic3r/GUI/GLSelectionRectangle.hpp"
#include "slic3r/GUI/I18N.hpp"
#include "libslic3r/ObjectID.hpp"
#include "libslic3r/ScaffoldPoints.hpp"

#include <optional>

namespace Slic3r {

class PrintObject;

namespace GUI {

enum class SLAGizmoEventType : unsigned char;

// Edits ModelObject::scaffold_points, the baked contact list a Tree Scaffold slice builds from.
class GLGizmoScaffoldPoints : public GLGizmoBase
{
private:
    struct CacheEntry {
        ScaffoldPoint                    point;
        Vec3f                            normal = Vec3f::Zero(); // object frame; zero until looked up on the mesh
        bool                             selected = false;
        std::optional<ScaffoldTipResult> result;                 // what the last slice did with this point

        CacheEntry() = default;
        explicit CacheEntry(const ScaffoldPoint &p) : point(p) {}

        template<class Archive> void serialize(Archive &ar) { ar(point, normal, selected); }
    };

public:
    GLGizmoScaffoldPoints(GLCanvas3D &parent, const std::string &icon_filename, unsigned int sprite_id);
    virtual ~GLGizmoScaffoldPoints() = default;

    void data_changed(bool is_serializing) override;
    bool on_mouse(const wxMouseEvent &mouse_event) override;
    bool gizmo_event(SLAGizmoEventType action, const Vec2d &mouse_position, bool shift_down, bool alt_down, bool control_down);
    bool is_selection_rectangle_dragging() const override { return m_selection_rectangle.is_dragging(); }

    bool        wants_enter_leave_snapshots() const override { return true; }
    std::string get_gizmo_entering_text() const override { return _u8L("Entering Scaffold Points"); }
    std::string get_gizmo_leaving_text() const override { return _u8L("Leaving Scaffold Points"); }

protected:
    bool               on_init() override;
    void               on_render() override;
    void               on_dragging(const UpdateData &data) override;
    void               on_start_dragging() override;
    void               on_stop_dragging() override;
    void               on_render_input_window(float x, float y, float bottom_limit) override;
    void               on_set_state() override;
    void               on_set_hover_id() override
    {
        if ((int) m_editing_cache.size() <= m_hover_id)
            m_hover_id = -1;
    }
    std::string        on_get_name() const override;
    bool               on_is_activable() const override;
    CommonGizmosDataID on_get_requirements() const override;
    void               on_load(cereal::BinaryInputArchive &ar) override;
    void               on_save(cereal::BinaryOutputArchive &ar) const override;
    void               on_register_raycasters_for_picking() override;
    void               on_unregister_raycasters_for_picking() override;

private:
    enum { AllPoints = -2, NoPoints };

    void  render_points(const Selection &selection);
    void  reload_cache();
    Vec3f normal_at(const Vec3f &pos) const;
    void  update_raycasters();
    // The closest hit of the mouse ray on a model part of the selected instance: position and normal in the object
    // frame, and the normal in world space.
    bool  unproject_on_mesh(const Vec2d &mouse_pos, Vec3f &pos, Vec3f &normal, Vec3d &world_normal) const;
    void  select_point(int i);
    void  unselect_point(int i);
    void  delete_selected_points();
    // Ctrl+wheel: flips the head size of the selected points, or the preset for new points when none is selected.
    void  toggle_head_size();
    // Writes the editing cache into the ModelObject as a user-modified list.
    void  apply_changes();
    bool  has_selected_points() const;
    bool  cache_differs_from_model() const;
    // The Print's copy of the selected object on the current plate, holding the selected instance.
    const PrintObject *selected_print_object() const;
    // w: a Light head has radius w and a Heavy one 2w.
    double             support_width() const;

    ScaffoldHeadSize        m_new_point_size = ScaffoldHeadSize::Light;
    std::vector<CacheEntry> m_editing_cache;
    ObjectID                m_old_mo_id;
    EState                  m_old_state = Off;
    bool                    m_selection_empty = true;
    bool                    m_wait_for_up_event = false;
    bool                    m_click_refused     = false; // the last click hit a face that points up
    bool                    m_close_asked       = false; // the apply-on-close dialog is queued or showing
    std::optional<CacheEntry> m_point_before_drag;

    PickingModel m_sphere;
    GLModel      m_cone;

    GLSelectionRectangle m_selection_rectangle;

    std::map<std::string, wxString>            m_desc;
    std::vector<std::pair<wxString, wxString>> m_shortcuts;
};

} // namespace GUI
} // namespace Slic3r

#endif // slic3r_GLGizmoScaffoldPoints_hpp_
