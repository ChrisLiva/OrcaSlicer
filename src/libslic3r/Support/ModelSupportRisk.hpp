#ifndef slic3r_ModelSupportRisk_hpp_
#define slic3r_ModelSupportRisk_hpp_

#include "../ExPolygon.hpp"
#include "../Point.hpp"
#include "../Polyline.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

namespace Slic3r {

class AABBMesh;

namespace ModelSupportRisk {

// One sliced layer of the model itself: the Z slab it occupies and the complete solid islands on it,
// as slicing left them. Not the overhang polygons - a weapon tip is thin whether or not anything
// under it was ever detected as an overhang.
struct Slice
{
    double     bottom_z_mm = 0.;
    double     top_z_mm    = 0.;
    ExPolygons solids;
};

// What the field found under one queried point. `local_width_mm` is the model's own width there,
// `neck_width_mm` the narrowest constriction between it and the object's first slab, which the plate
// or a raft carries, along the path the model is actually held by, `lever_mm` how far the query sits
// from that constriction, and `risk_per_mm2` the dimensionless weight one mm2 of contact there
// carries. A relative geometric ranking, not a force estimate and not a material model.
struct Sample
{
    // BelowPrintableWidth is a measured result and keeps its widths: the feature is thinner than one
    // extrusion, which the weight's own clamping would otherwise present as merely printable.
    // Unknown is the absence of a measurement, and its numbers mean nothing.
    enum class Status : uint8_t { Known, BelowPrintableWidth, Unknown };

    Status status         = Status::Unknown;
    double local_width_mm = 0.;
    double neck_width_mm  = 0.;
    double lever_mm       = 0.;
    double risk_per_mm2   = 0.;
};

// The model's solid geometry and its connectivity, built once and queried many times. Immutable after
// build() returns and owning every polygon it answers from, so it outlives the Print layers it was
// measured off and holds no pointer into them.
struct Field
{
    enum class Status : uint8_t { Complete, Canceled, Invalid };

    // One medial sample of one island: a point on that island's medial axis and the model width
    // there. Nodes are numbered in (layer, island, medial order), which is the search's final
    // tie-break.
    struct Node
    {
        size_t layer    = 0;
        size_t island   = 0;
        Point  position;
        double width_mm = 0.;
        bool   root     = false;   // its island is part of the object's first slab
    };

    // One undirected connection: two medial samples of one island, or two islands on adjacent layers
    // whose solids actually overlap. `width_mm` is the width of the connection itself - the overlap
    // width across layers, and infinite inside one island, where the constriction belongs to the
    // samples the run links and not to the run. `length_mm` is its length in mm.
    struct Edge
    {
        size_t a         = 0;
        size_t b         = 0;
        double width_mm  = 0.;
        double length_mm = 0.;
    };

    Status                           status             = Status::Invalid;
    double                           extrusion_width_mm = 0.;
    std::vector<Slice>               slices;            // the geometry queried against, owned
    std::vector<Node>                nodes;
    std::vector<Edge>                edges;
    std::vector<std::vector<size_t>> adjacency;         // node index -> edge indices
    // The flat island index of (layer, island) is island_base[layer] + island.
    std::vector<size_t>              island_base;
    std::vector<std::vector<size_t>> island_nodes;      // flat island index -> node indices
    std::vector<ThickPolylines>      island_medial;     // flat island index -> its medial axis
};

// The model width at `query` inside one solid, in mm: the medial-axis width interpolated at the
// nearest projection of the query that the solid itself contains the sight line to, capped by twice
// the query's own clearance from the solid's boundary. A solid whose skeleton comes back empty (one
// that is narrow nowhere) answers twice the clearance itself, for a query it contains. Answers false,
// leaving *width_mm alone, for a degenerate polygon, a query no medial segment answers for, a medial
// axis whose width vector does not carry two entries per segment, and any nonfinite value.
bool local_width(const ExPolygon &solid, const Point &query, double *width_mm);

// Builds the field over the model's own slices. Islands on adjacent layers connect in both directions
// only where their intersection has positive area, and every island carries its own medial samples so
// a weapon attached to a hand on one layer is not read as uniformly thick. `stop` is polled as the
// build runs: a true answer abandons it and returns Canceled, which is a different thing from a
// geometrically unknown sample. Nonfinite or inverted slab bounds return Invalid.
Field build(const std::vector<Slice> &slices, double extrusion_width_mm, const std::function<bool()> &stop);

// What the model under `query` on `layer` is worth carrying a contact on. Unknown wherever the field
// is not Complete, the query lies in no solid, the local width cannot be measured, or no path over
// the model's own solids reaches the object's first slab, which the plate or a raft carries.
Sample sample(const Field &field, size_t layer, const Point &query);

// The dimensionless weight `(w / max(t, w)) * (1 + L / max(n, w))`: w the resolved support extrusion
// width, t the local model width, n the neck width and L the lever length, all mm. Thinner material
// and a longer lever off a narrower neck weigh more. Returns NaN for a non-positive or nonfinite w or
// any nonfinite argument, so a misuse can never read as an absence of risk.
double risk_weight(double extrusion_width_mm, double local_width_mm, double neck_width_mm, double lever_mm);

// Whether a straight probe of the stated radius can reach one placed contact from outside the print,
// and along which direction. A geometric reachability estimate for an idealised straight tool: it
// says nothing about the force a removal takes, about what a real tool's handle needs behind it, or
// about whether cutting there is safe.
struct Access
{
    enum class Status : uint8_t { Clear, Blocked, Unknown };

    Status status    = Status::Unknown;
    // The direction found clear, normalized. Zero for every other answer.
    Vec3d  direction = Vec3d::Zero();
};

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

} // namespace ModelSupportRisk
} // namespace Slic3r

#endif // slic3r_ModelSupportRisk_hpp_
