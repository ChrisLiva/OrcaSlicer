#pragma once
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
struct LayerAreas { ExPolygons base; ExPolygons interface_; };   // before the 2D clip TreeSupport runs
// The 2D clip TreeSupport runs on a planned layer, computed once for `draw` and the seam: interface stays out of
// `model`, the object over the layer's whole height, and base stays out of `band`, that model grown by the xy distance
// together with the bottom gap's trim.
struct LayerClip { ExPolygons model, band; };
struct Counts { size_t tips_placed = 0, tips_routed = 0, tips_dropped = 0, islands_under_held = 0,
                pillars_unbraced = 0; };
struct StageMs { uint32_t island_joins = 0, build = 0, slice = 0; };   // for TreeSupport's profiler
struct Output { std::vector<LayerAreas> layers;   // one entry per planned layer
                size_t pad_layers = 0;            // the leading planned layers whose base is the pad
                Counts counts; StageMs stage_ms; };
// contacts: TreeSupport's contact_nodes after plan_layer_heights re-distributes them, one entry per planned layer.
// dropped: the nodes the erase loop after select_contacts took out, the hold floor's candidates.
// clips: the clip the seam runs on each planned layer, which the neck check applies the same way.
Output draw(const PrintObject &object, const std::vector<std::vector<SupportNode *>> &contacts,
            const std::vector<SupportNode *> &dropped, const std::vector<LayerHeightData> &layer_heights,
            const std::vector<LayerClip> &clips, const ModelSupportRisk::Field &risk, const Params &params,
            const std::function<void()> &throw_on_cancel);
}
