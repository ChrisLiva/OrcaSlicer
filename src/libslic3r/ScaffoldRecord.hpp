#pragma once

#include "ScaffoldPoints.hpp"

namespace Slic3r {

enum class ScaffoldTipResult : uint8_t { Routed, Filtered, Unrouted, Neck, Merged, Wall };

// What one scaffold slice of one PrintObject did with its tips, in the object raw-mesh frame.
struct ScaffoldRecord {
    struct Tip { Vec3f pos; ScaffoldHeadSize size; bool enforced; ScaffoldTipResult result; };
    std::vector<Tip>   tips;          // baked: one per list point, in list order; auto: the tips handed to draw
    std::vector<Vec3f> bare_islands;  // where an island prints with no tip holding it
    bool               baked = false, stale = false;
    Matrix3d           pose = Matrix3d::Identity();   // linear part of the first model instance's matrix
    double             toolpath_width_mm = 0.;        // w, the slice's support toolpath width
};

// The routed tips of a record, as Generate bakes them.
inline ScaffoldPoints scaffold_points_from(const ScaffoldRecord &record)
{
    ScaffoldPoints points;
    for (const ScaffoldRecord::Tip &tip : record.tips)
        if (tip.result == ScaffoldTipResult::Routed)
            points.push_back({ tip.pos, tip.size, tip.enforced });
    return points;
}

} // namespace Slic3r
