#ifndef slic3r_RemovalAccess_hpp_
#define slic3r_RemovalAccess_hpp_

#include "../ExPolygon.hpp"
#include "../Point.hpp"

#include <cstdint>
#include <vector>

namespace Slic3r {

class AABBMesh;

namespace RemovalAccess {

// Whether a straight probe of the stated radius can reach one placed contact from outside the print,
// and along which direction. A geometric reachability estimate for an idealised straight tool: it
// says nothing about the force a removal takes, about what a real tool's handle needs behind it, or
// about whether cutting there is safe.
struct Access
{
    enum class Status : uint8_t { Clear, Blocked, Unknown };

    // Which step did not answer, for a status of Unknown, on the same terms as `ModelSupportRisk::Sample::Missing`.
    // AllBuried is the geometric one: every escape direction starts inside the model.
    enum class Missing : uint8_t { None, NoMesh, BadContact, SlabMismatch, SlabOrder, BadBounds, AllBuried };

    Status  status    = Status::Unknown;
    Missing missing   = Missing::None;
    // The direction found clear, normalized. Zero for every other answer.
    Vec3d   direction = Vec3d::Zero();
};

// The name of one Access::Missing, for a log line.
const char *missing_name(Access::Missing missing);

// Which way `contact` can be reached through the model and the support already printed around it, in
// posed PrintObject coordinates and millimetres. The 26 directions with coordinates in {-1, 0, 1} bar
// the zero vector are tried in lexicographic order of those integer triples and the first clear one
// is the answer, so the order is part of the result.
//
// `clearance_mm` is an analysis probe radius, not a tool: the caller passes half the resolved support
// extrusion width. For a direction d the probe runs from `contact + d * clearance_mm` to
// `contact + d * (clearance_mm + 2 * diagonal)`, the diagonal of the model and support taken together,
// so it leaves the print in every direction it is not stopped in. The direction is clear when that
// capsule stays further than the probe radius from every model triangle, counting only closest points
// outside the closed sphere of that radius around the contact itself - the model the contact is
// placed on cannot be what stops it - and when it shares no volume with a support slab, which is
// excused inside that same sphere. A collision farther along the capsule stops the direction whatever
// sits at the contact.
//
// `support_by_layer` holds the printed support footprints the contact is not being removed with, in
// scaled object coordinates, and `layer_z_mm` the print_z of the same layers, ascending: the material
// of the contact's own component is what a removal takes away, so the caller leaves it out rather
// than being told a group is caged by itself. A layer's slab runs from the print_z of the layer under
// it, and the first from as far below its own as the second layer stands above it. Every slab is
// grown by the probe radius in Z before its plan-view test, so a probe grazing the top or bottom of a
// stack reads blocked rather than clear: the estimate errs toward saying a contact cannot be reached.
//
// Unknown, never Blocked, for a mesh with no triangles, nonfinite bounds or arguments, a layer list
// whose two halves disagree, and a contact whose every probe would start inside the model, which is a
// start that cannot leave the source patch.
Access assess_access(const AABBMesh &model, const std::vector<ExPolygons> &support_by_layer,
                     const std::vector<double> &layer_z_mm, const Vec3d &contact, double clearance_mm);

} // namespace RemovalAccess
} // namespace Slic3r

#endif // slic3r_RemovalAccess_hpp_
