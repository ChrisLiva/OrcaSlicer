#pragma once
#include <limits>
#include "TreeSupport.hpp"          // SupportNode, LayerHeightData
#include "ModelSupportRisk.hpp"
namespace Slic3r::ScaffoldSupport {
struct Params {   // filled by TreeSupport from its config and support params
    double toolpath_width_mm, pillar_diameter_mm, xy_distance_mm, bridge_length_mm, brace_slenderness,
           max_bridge_length_mm,
           pad_thickness_mm,    // 0.6 mm rounded up to whole planned layers by TreeSupport
           z_offset_mm,         // m_slicing_params.object_print_z_min: print z minus mesh z
           interface_width_mm;  // the interface flow's line width, the width a ring's loop prints at
    size_t interface_layers;    // support_interface_top_layers
    // What the lines TreeSupport lays on base areas of a layer this high cover, as the floating pass reads them.
    std::function<Polygons(const ExPolygons &base, double height)> base_cover;
};
// Before the 2D clip TreeSupport runs. `enforced_heads` are the heads of the enforced tips, sliced like the base on the
// layers below their rings.
struct LayerAreas { ExPolygons base; ExPolygons interface_; ExPolygons enforced_heads; };
// The 2D clip TreeSupport runs on a planned layer, computed once for `draw` and the seam: interface stays out of
// `model`, the object over the layer's whole height, and base stays out of `band`, that model grown by the xy distance
// together with the bottom gap's trim.
struct LayerClip { ExPolygons model, band; };
// The base the seam prints on a planned layer before the bed clip, which the neck check reads the same way: `base`
// outside the band, and the enforced heads outside the model alone, so an enforced tip fuses beside a wall.
ExPolygons clip_base(const ExPolygons &base, const ExPolygons &enforced_heads, const LayerClip &clip);
struct Counts { size_t tips_placed = 0, tips_routed = 0, tips_dropped = 0, islands_under_held = 0,
                pillars_unbraced = 0; };
struct StageMs { uint32_t island_joins = 0, build = 0, slice = 0; };   // for TreeSupport's profiler
struct Output { std::vector<LayerAreas> layers;   // one entry per planned layer
                size_t pad_layers = 0;            // the leading planned layers whose base is the pad
                Counts counts; StageMs stage_ms; };
// Where a tip stands. `node` is the contact it stands for, or null for a tip the hold floor seeded under an island
// the front half left without one. `enforced` is a contact a support enforcer asked for, painted facets or an enforcer
// volume (`SupportNode::is_pinned`), which the wall skip keeps and whose head `clip_base` clips by the model alone.
struct TipSite
{
    Point              position;
    double             print_z      = 0.;
    int                obj_layer_nr = 0;
    uint64_t           seed         = std::numeric_limits<uint64_t>::max();
    const SupportNode *node         = nullptr;
    bool               enforced     = false;
};
// The tips `draw` builds heads for, with what the hold floor counted and how long its island map took.
struct Tips { std::vector<TipSite> sites; size_t islands_under_held = 0; uint32_t island_joins_ms = 0; };
// contacts: TreeSupport's contact_nodes before plan_layer_heights re-distributes them, the nodes the contact selection
// kept. dropped: the nodes the erase loop after select_contacts took out, the hold floor's candidates. Reads no planned
// layer, so TreeSupport plans a layer topped at every tip's z.
Tips choose_tips(const PrintObject &object, const std::vector<std::vector<SupportNode *>> &contacts,
                 const std::vector<SupportNode *> &dropped, const Params &params);
// clips: the clip the seam runs on each planned layer, which the neck check applies the same way.
Output draw(const PrintObject &object, const Tips &chosen, const std::vector<LayerHeightData> &layer_heights,
            const std::vector<LayerClip> &clips, const ModelSupportRisk::Field &risk, const Params &params,
            const std::function<void()> &throw_on_cancel);
}
