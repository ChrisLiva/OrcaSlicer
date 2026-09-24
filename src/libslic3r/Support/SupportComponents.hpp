#ifndef slic3r_SupportComponents_hpp_
#define slic3r_SupportComponents_hpp_

#include "../ExPolygon.hpp"
#include "../Point.hpp"
#include "../Polygon.hpp"

#include <cstddef>
#include <utility>
#include <vector>

namespace Slic3r {

class PrintObject;

// The slab connectivity graph the support analysis is built on: printed material described as one
// closed Z interval per layer with the ground it covers, the components positive-area overlap between
// touching slabs joins it into, and which of those components stand on something. A geometric
// connectivity rule throughout: nothing here weighs material or estimates a force.
namespace SupportAnalysis {

// One printed slab: the closed Z interval [bottom_z, print_z] one layer occupies and the ground its
// material covers on it. Support material and model material are described the same way, so one
// connectivity walk serves both.
struct Slab
{
    double     bottom_z = 0.;
    double     print_z  = 0.;
    ExPolygons polygons;
};

// One polygon of one slab, and the polygons of the neighbouring slabs whose material it continues
// into.
struct Piece
{
    ExPolygon           polygon;
    double              bottom_z  = 0.;
    double              print_z   = 0.;
    std::vector<size_t> below;
    std::vector<size_t> above;
    size_t              component = 0;
};

struct Components
{
    std::vector<Piece>                    pieces;
    std::vector<std::pair<size_t, size_t>> slab_range;  // [first, last) piece index of each slab
    size_t                                count = 0;
    std::vector<char>                     bed_rooted;   // one per component
};

// A slab whose underside is on the plate. `EmittedLayer::bottom_z` and a `Layer`'s print_z minus its
// height are both the plan's own arithmetic, so the plate slab's bottom is zero to floating-point
// noise and every slab above it starts a whole layer up: nothing about the slab's own height enters
// this, which is what keeps a thin first layer under a thicker one (0.1 mm under 0.25 mm) from
// letting the floating slab above it pass as plate-rooted.
bool on_bed(double bottom_z);

// What positive-area overlap between the polygons of consecutive touching slabs joins up. Two slabs
// take part only when their closed Z intervals touch, so material separated by a slab that printed
// nothing is separate however far the two overlap in plan, and an overlap that has no area joins
// nothing either. `slabs` is ordered by ascending print_z. A component is rooted where one of its pieces
// starts at `ground_z`: the plate for support, whose first slab (the raft's, where there is one) starts
// there, and the object's own first slab for the model, which the plate or a raft carries.
Components build_components(const std::vector<Slab> &slabs, double ground_z);

// Which components of `support` stand on something. The plate first, then a termination against the
// object, which counts only where the settings permit resting on the model at all and where the model
// it rests on is itself connected to the object's own first layer, which the plate or a raft carries.
// `model` is the object's own sliced body, in the same frame as the support and already grouped into
// its own components.
std::vector<char> rooted_components(const Components &support, const std::vector<Slab> &model_slabs,
                                    const Components &model, bool on_build_plate_only, double bottom_gap_mm);

// The middle of a piece's section in mm: the centroid of its outline, half way up its own slab.
Vec3d piece_middle(const Piece &piece);

// The piece of `slab` a printed part sits in, or npos when the part has no outline or no piece holds
// its first point.
size_t piece_containing(const Components &support, size_t slab, const ExPolygon &part);

// The ground the object stands on: its own first layer where that layer is the one on the plate,
// every printed support slab that starts on the plate, the raft among them, and whatever bed adhesion
// the print laid afterwards. A support slab that starts on the plate roots its own component by
// construction, so which components hold something up does not enter this. A skirt is neither support
// nor the object's, so it never gets here.
ExPolygons bed_ground(const std::vector<Slab> &model_slabs, const std::vector<Slab> &support_slabs,
                      const Polygons &adhesion);

// The object's own body, sliced: one slab per object layer, in the object's canonical unshifted
// instance frame. It is what a branch may terminate against, what has to be connected to its own first
// layer before such a termination holds anything up, and that first layer, which the plate or a raft
// carries, is what the object stands on.
std::vector<Slab> model_slabs_of(const PrintObject &object);

// Which polygons of `support` the print would lay in mid-air: one flag per polygon of each slab, set
// where the polygon's connected component of support material neither starts on the plate nor, where
// `on_build_plate_only` is off, comes down onto model material connected to the object's first layer
// within `bottom_gap_mm` plus its own slab of its underside. Components join through positive-area
// overlap between the polygons of consecutive slabs whose Z intervals touch, so a slab that printed
// nothing separates what is above it from what is below. A geometric connectivity rule, not a force
// estimate: it is what the stability measurement counts as unsupported paths, offered to a generator
// so the material it counts is never printed.
std::vector<std::vector<bool>> floating_pieces(const std::vector<Slab> &support, const std::vector<Slab> &model,
                                               bool on_build_plate_only, double bottom_gap_mm);

// Where a mid-air island of the model is born and where it first meets the rooted body.
struct IslandJoin
{
    size_t birth_slab = 0;
    size_t join_slab  = 0; // the slab count when the island never joins
};

struct IslandMap
{
    Components             components;      // build_components' pieces, slab_range and below lists
    std::vector<size_t>    island_of_piece; // one per piece; size_t(-1) where no island owns it
    std::vector<IslandJoin> islands;
};

// The islands of `model_slabs` walked bottom-up: each piece with nothing below it whose connected set is
// not yet rooted at `ground_z` starts an island, which joins at the first slab whose overlaps connect it
// to a rooted set. A piece belongs to the oldest island still open in its set at its own slab.
IslandMap island_joins(const std::vector<Slab> &model_slabs, double ground_z);

} // namespace SupportAnalysis
} // namespace Slic3r

#endif // slic3r_SupportComponents_hpp_
