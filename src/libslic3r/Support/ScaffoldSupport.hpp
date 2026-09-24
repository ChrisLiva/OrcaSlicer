#pragma once
#include "TreeSupport.hpp"          // SupportNode, LayerHeightData
#include "ModelSupportRisk.hpp"
namespace Slic3r::ScaffoldSupport {
struct Params {   // filled by TreeSupport from its config and support params
    double toolpath_width_mm, pillar_diameter_mm, xy_distance_mm, bridge_length_mm, brace_slenderness,
           max_bridge_length_mm,
           pad_thickness_mm,    // 0.6 mm rounded up to whole planned layers by TreeSupport
           z_offset_mm;         // m_slicing_params.object_print_z_min: print z minus mesh z
    size_t interface_layers;    // support_interface_top_layers
};
struct LayerAreas { ExPolygons base; ExPolygons interface_; };   // before the 2D clip TreeSupport runs
struct Counts { size_t tips_placed = 0, tips_routed = 0, tips_dropped = 0, islands_under_held = 0,
                pillars_unbraced = 0; };
struct StageMs { uint32_t island_joins = 0, build = 0, slice = 0; };   // for TreeSupport's profiler
struct Output { std::vector<LayerAreas> layers;   // one entry per planned layer
                size_t pad_layers = 0;            // the leading planned layers whose base is the pad
                Counts counts; StageMs stage_ms; };
// contacts: TreeSupport's contact_nodes after plan_layer_heights re-distributes them, one entry per planned layer.
// dropped: the nodes the erase loop after select_contacts took out, the hold floor's candidates.
Output draw(const PrintObject &object, const std::vector<std::vector<SupportNode *>> &contacts,
            const std::vector<SupportNode *> &dropped, const std::vector<LayerHeightData> &layer_heights,
            const ModelSupportRisk::Field &risk, const Params &params, const std::function<void()> &throw_on_cancel);
}
