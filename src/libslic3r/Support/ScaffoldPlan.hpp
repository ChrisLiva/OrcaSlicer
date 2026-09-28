#pragma once
#include <cstdint>
#include <limits>
#include <vector>
#include "../ExPolygon.hpp"
#include "../Point.hpp"
#include "../Polygon.hpp"
#include "SupportComponents.hpp"
namespace Slic3r {
class PrintObject;
namespace ScaffoldSupport {
// Where a tip stands. `contact` is true for a tip `draw` grades against the risk field, and false for one that keeps
// the small grade. `enforced` is a contact a support enforcer asked for, painted facets or an enforcer volume
// (`SupportNode::is_pinned`), which the wall skip keeps and whose head `clip_base` clips by the model alone.
// `grade_mm` is the disc width the tip asks for, 0 to let `draw` grade it, and `source` the point's index in a baked
// list, -1 for any other tip. Both come after `enforced`, since tips are built by position.
struct TipSite
{
    Point              position;
    double             print_z      = 0.;
    int                obj_layer_nr = 0;
    uint64_t           seed         = std::numeric_limits<uint64_t>::max();
    bool               contact      = false;
    bool               enforced     = false;
    double             grade_mm     = 0.;
    int                source       = -1;
};

// The deepest point of `piece`, where the hold floor and the birth need stand a tip.
Point inscribed_point(const ExPolygon &piece);

// The need a planned tip answers: an enforcer asked for it, an island starts there, the underside droops too far from
// its anchors there, or a part standing free has grown too tall over its anchors.
enum class TipNeed : uint8_t { Enforced, Birth, Underside, Stability };

// What a density asks of the planner. `reach_mm` is how far an underside may hang past its anchors, `slender_ratio`
// how many section widths a part may stand over its highest anchor, and `micro_merge_mm` how soon an island no wider
// than two support lines has to merge to print without a tip.
struct NeedParams { double reach_mm = 1., slender_ratio = 3., micro_merge_mm = 0.12; };
// Linear between Light at 0, Medium at 1 and Heavy at 2.
NeedParams need_params(double density);

// One object layer on the plan's lattice, within the layer's own bounding box: per cell 0 where there is no material,
// else 1 + the index of the piece holding it among its slab's pieces.
struct LayerGrid
{
    int                   x0 = 0, y0 = 0, w = 0, h = 0;
    std::vector<uint16_t> cells;
    uint16_t at(int x, int y) const
    {
        return x < x0 || y < y0 || x >= x0 + w || y >= y0 + h ? 0 : cells[size_t(y - y0) * size_t(w) + size_t(x - x0)];
    }
};

// What the planner reads of a slice, whatever the density: built once per slice and kept for the density slider.
struct PlanInput
{
    Point                              origin;             // the lattice's cell (0, 0) corner, scaled
    coord_t                            cell    = 0;        // the cell side, half the support toolpath width, scaled
    double                             cell_mm = 0.;
    std::vector<SupportAnalysis::Slab> slabs;              // one per object layer, `model_slabs_of`
    SupportAnalysis::Components        components;         // `build_components` over the slabs, ground at the first
    std::vector<LayerGrid>             material;           // one per object layer, labelled by piece
    std::vector<LayerGrid>             blocked;            // one per object layer, 1 under a support blocker
    std::vector<ExPolygons>            down_facing;        // one per object layer: its material the layer below lacks
    std::vector<double>                self_support_mm;    // one per object layer: the step the layer below carries
    double toolpath_width_mm = 0., xy_distance_mm = 0., neck_depth_mm = 0.;
};
// `threshold_rad` is the overhang detector's threshold angle and `blockers` the support blockers per object layer, as
// TreeSupport gathers them.
PlanInput prepare_plan(const PrintObject &object, double toolpath_width_mm, double xy_distance_mm, double neck_depth_mm,
                       double threshold_rad, const std::vector<Polygons> &blockers);

struct PlannedTip { TipSite site; TipNeed need; };
// The tips the needs call for, lowest first, and what they could not meet: islands whose birth took no tip, parts
// left slender and underside area left hanging past the reach.
struct Plan
{
    std::vector<PlannedTip> tips;
    size_t                  islands_unheld = 0, islands_slender = 0;
    double                  underside_unmet_mm2 = 0.;
};
// `enforced` are the contacts an enforcer asked for; each becomes a tip and an anchor.
Plan plan_tips(const PlanInput &input, const std::vector<TipSite> &enforced, const NeedParams &need);
} // namespace ScaffoldSupport
} // namespace Slic3r
