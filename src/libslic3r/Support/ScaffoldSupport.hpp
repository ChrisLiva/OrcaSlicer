#pragma once
#include <limits>
#include "TreeSupport.hpp"          // SupportNode, LayerHeightData
#include "ModelSupportRisk.hpp"
#include "../ScaffoldPoints.hpp"
#include "ScaffoldRetune.hpp"
#include "ScaffoldPlan.hpp"
namespace Slic3r::ScaffoldSupport {
struct Params {   // filled by TreeSupport from its config and support params
    double toolpath_width_mm = 0., pillar_diameter_mm = 0., xy_distance_mm = 0., bridge_length_mm = 0.,
           brace_slenderness = 0., max_bridge_length_mm = 0.,
           brace_diameter_mm = 0.,   // a brace between two pillars, capped at the pillar diameter; 0 is the pillar's
           pad_thickness_mm = 0.,    // 0.6 mm rounded up to whole planned layers by TreeSupport
           z_offset_mm = 0.,         // m_slicing_params.object_print_z_min: print z minus mesh z
           interface_width_mm = 0.,  // the interface flow's line width, the width a ring's loop prints at
           taper = 0.;               // radius a pillar gains per mm below its top, the branch diameter angle in radians
    size_t interface_layers = 0;     // support_interface_top_layers
    // What the lines TreeSupport lays on base areas of a layer this high cover, as the floating pass reads them.
    std::function<Polygons(const ExPolygons &base, double height)> base_cover;
};
// Before the 2D clip TreeSupport runs. `exempt_heads` are the heads exempt from the band, the enforced tips' heads and
// the heads the neck check exempted, sliced like the base on the layers below their rings.
struct LayerAreas { ExPolygons base; ExPolygons interface_; ExPolygons exempt_heads; };
// The 2D clip TreeSupport runs on a planned layer, computed once for `draw` and the seam: interface stays out of
// `model`, the object over the layer's whole height, and base stays out of `band`, that model grown by the xy distance
// together with the bottom gap's trim.
struct LayerClip { ExPolygons model, band; };
// The base the seam prints on a planned layer before the bed clip, which the neck check reads the same way: `base`
// outside the band, and the heads exempt from the band outside the model alone, so an enforced tip fuses beside a wall
// and an exempted head's neck reaches its rings through the band.
ExPolygons clip_base(const ExPolygons &base, const ExPolygons &exempt_heads, const LayerClip &clip);
struct Counts { size_t tips_placed = 0, tips_routed = 0, tips_dropped = 0, islands_under_held = 0,
                pillars_unbraced = 0, islands_slender = 0; double underside_unmet_mm2 = 0.; };
struct StageMs { uint32_t island_joins = 0, build = 0, slice = 0; };   // for TreeSupport's profiler
struct Output { std::vector<LayerAreas> layers;   // one entry per planned layer
                size_t pad_layers = 0;            // the leading planned layers whose base is the pad
                Counts counts; StageMs stage_ms;
                std::vector<ScaffoldTipResult> results;   // what became of each tip, indexed like `Tips::sites`
                std::vector<double>            grades;    // each tip's disc width in mm, indexed like `Tips::sites`
              };
// The tips `draw` builds heads for, with what the planner could not meet and how long it took. From a baked list, also
// the `source` of each point the wall skip took out, and where the hold floor would have seeded a tip, x and y in mm and
// print z.
struct Tips { std::vector<TipSite> sites; size_t islands_under_held = 0; uint32_t island_joins_ms = 0;
              std::vector<int> wall_skipped; std::vector<Vec3d> bare_islands;
              size_t islands_slender = 0; double underside_unmet_mm2 = 0.; };
// The density a `scaffold_density` tier stands for.
double tier_density(ScaffoldDensity tier);
// The key a tip's grade is kept under.
std::array<int64_t, 3> grade_key(const TipSite &site);

// What an auto slice keeps so that `retune_points` can place its tips at another density without slicing again: the
// contacts an enforcer asked for, the planner's input, the risk field the grades read, the grade the slice gave each
// tip it drew, and the parameters and density the slice ran with.
struct Candidates {
    std::vector<TipSite>             enforced;
    std::shared_ptr<const PlanInput> plan;
    ModelSupportRisk::Field          risk;
    TipGrades                        grades;
    Params                           params;   // without `base_cover`, which reads the generator that built it
    double                           density = 1.;
};
// contacts: TreeSupport's contact_nodes before plan_layer_heights re-distributes them, of which the planner keeps the
// ones an enforcer asked for. threshold_rad and blockers: the overhang detector's threshold and the support blockers per
// object layer. The risk field is TreeSupport's to move in once its measurement is done.
Candidates collect_candidates(const PrintObject &object, const std::vector<std::vector<SupportNode *>> &contacts, const Params &params,
                              double threshold_rad, const std::vector<Polygons> &blockers);
// The planner at `density`, then the wall skip and the alias merge. Reads no planned layer, so TreeSupport plans a layer
// topped at every tip's z.
Tips place_tips(const PrintObject &object, const Candidates &candidates, double density);
// A tip as a list point in ModelObject::raw_mesh()'s frame: a grade over three toolpath widths reads Heavy.
ScaffoldPoint point_of(const PrintObject &object, const Params &params, const TipSite &site, double grade_mm);
// A baked list in the builder's frame: each point mapped through trafo_centered(), z + params.z_offset_mm, snapped
// to the bottom of the object layer holding it; wall skip (enforced exempt), island count seeding nothing, alias merge.
Tips baked_tips(const PrintObject &object, const ScaffoldPoints &points, const Params &params);
// Whether a list baked under the linear part `pose` still holds under `linear`: the change between them keeps lengths
// and keeps the Z axis, as a turn about Z or a mirror in X or Y does, and a tilt, a Z mirror or a scale does not.
bool baked_pose_valid(const Matrix3d &pose, const Matrix3d &linear);
// clips: the clip the seam runs on each planned layer, which the neck check applies the same way.
Output draw(const PrintObject &object, const Tips &chosen, const std::vector<LayerHeightData> &layer_heights,
            const std::vector<LayerClip> &clips, const ModelSupportRisk::Field &risk, const Params &params,
            const std::function<void()> &throw_on_cancel);
}
