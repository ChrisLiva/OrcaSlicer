#pragma once
#include <cstdint>
#include <limits>
#include <vector>
#include "../ExPolygon.hpp"
#include "../Point.hpp"
#include "../Polygon.hpp"
#include "../TriangleMesh.hpp"
#include "../SLA/IndexedMesh.hpp"
#include "SupportComponents.hpp"
namespace Slic3r {
class PrintObject;
namespace ScaffoldSupport {
// Where a tip stands. `enforced` is a contact a support enforcer asked for, painted facets or an enforcer volume
// (`SupportNode::is_pinned`), which the wall skip keeps and whose head `clip_base` clips by the model alone. `grade_mm`
// is the disc width the tip asks for, 0 for the small disc `draw` gives it, and `source` the point's index in a baked
// list, -1 for any other tip. `axis` is the unit direction the tip's neck leans along, pointing down, which the wall
// skip reads the band at the neck's end by and the builder aims the head along; zero, as on every tip but a birth or
// stability tip whose neck clears only leaning, leaves the head to the mesh normal. `holds_island` marks a tip holding
// a part the bed does not hold, whose head the builder retries along leaning axes where no other route reaches the pad.
// These come after `enforced`, since tips are built by position.
struct TipSite
{
    Point              position;
    double             print_z      = 0.;
    int                obj_layer_nr = 0;
    uint64_t           seed         = std::numeric_limits<uint64_t>::max();
    bool               enforced     = false;
    double             grade_mm     = 0.;
    int                source       = -1;
    Vec3f              axis         = Vec3f::Zero();
    bool               holds_island = false;
};

// The deepest point of `piece`, where the birth need stands a tip and an island's birth point lies.
Point inscribed_point(const ExPolygon &piece);

// The most a head leans from straight down: the builder saturates a head's direction at this bridge slope, so the
// planner leans a neck no farther.
constexpr double max_head_tilt_rad = M_PI / 4.;

// The need a planned tip answers: an enforcer asked for it, an island starts there, the underside droops too far from
// its anchors there, or a stem reaches too far from what holds it for its thickness.
enum class TipNeed : uint8_t { Enforced, Birth, Underside, Stability };

// The object's mesh in the frame its slices are in, XY centred and the bed on z 0, with the AABB tree the builder aims
// its heads by. `place_tips` builds it once: the planner reads the faces a head meets off it and `draw` builds the tree
// on it. `aabb` points into `mesh`, so the pair never moves.
struct ObjectMesh
{
    explicit ObjectMesh(const PrintObject &object);
    ObjectMesh(const ObjectMesh &)            = delete;
    ObjectMesh &operator=(const ObjectMesh &) = delete;
    TriangleMesh     mesh;
    sla::IndexedMesh aabb;
};

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

// The builder's full head: the back's radius, the length the filter tests the head clear over, from the tip's point,
// where the pin meets the model, to the far side of the back sphere, and the clearance it keeps from the model, as the
// builder's config sets them. A leaning neck is accepted only where this head, at the tip's pin, clears the mesh along
// its axis, so the builder keeps the axis and the full head; a zero length tests no head.
struct HeadShape
{
    double back_mm = 0., length_mm = 0., safety_mm = 0.;
};

// What the planner reads of a slice, built once per slice.
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
    std::vector<ExPolygons>            wall_band;          // one per object layer: its material grown by the xy distance, as the seam clips it
    std::vector<double>                self_support_mm;    // one per object layer: the step the layer below carries
    double toolpath_width_mm = 0., xy_distance_mm = 0., neck_depth_mm = 0.;
    double bridge_mm         = 0.;        // the longest line an underside bridges between held ends, 0 for none
    const ObjectMesh *mesh   = nullptr;   // the faces a head meets; with none the planner reads no face and fits no head
    double z_offset_mm       = 0.;        // print z minus mesh z, the object's lift
    double max_tilt_rad      = max_head_tilt_rad;   // how far a birth's or stability tip's neck may lean, 0 for straight down only
    HeadShape head;                       // the head a leaning neck must fit, which `prepare_plan` leaves to its caller
};
// `bridge_mm` is `max_bridge_length`, `threshold_rad` the overhang detector's threshold angle and `blockers` the support
// blockers per object layer, as TreeSupport gathers them. `mesh` is the object's, which the plan reads through the
// lifetime of the input.
PlanInput prepare_plan(const PrintObject &object, double toolpath_width_mm, double xy_distance_mm, double neck_depth_mm,
                       double bridge_mm, double threshold_rad, const std::vector<Polygons> &blockers,
                       const ObjectMesh *mesh = nullptr);

// `answered_mm2` is, for an Underside tip, the underside past one and a half reaches its head answered on its layer,
// which hangs again when the head does not route.
struct PlannedTip { TipSite site; TipNeed need; double answered_mm2 = 0.; };

// How the plan holds an island: a tip stands under its birth piece, its own, an enforced one or, for a nub, the
// underside head its own layer placed; it is a nub that waited for its merge and hangs from the held parts it met
// within its hang; no tip can stand under it; or it is debris, which prints as it hangs and needs no hold.
enum class IslandReason : uint8_t { Tip, Hung, NoNeck, Debris };
// A birth piece off the bed with nothing under it, unless every point of it lies within the slab below's step of that
// slab's material: such a piece continues the slab below as an overhang does and is no island. `birth` is its deepest
// point at its bottom, x and y in mm and print z. `holders` index `Plan::tips`: an island's own tips, or for one that
// hangs, every tip on the held parts within its hang, whatever need placed it. `rooted` is true where the bed held one
// of those parts.
struct Island
{
    Vec3d               birth = Vec3d::Zero();
    std::vector<size_t> holders;
    bool                rooted = false;
    IslandReason        reason = IslandReason::NoNeck;
};

// The tips the needs call for, lowest first, every island with how it is held, and what the needs could not meet:
// islands no tip can stand under, branches left slender with no face a head can hold, and underside area left hanging
// past the reach.
struct Plan
{
    std::vector<PlannedTip> tips;
    std::vector<Island>     islands;
    size_t                  islands_unheld = 0, islands_slender = 0;
    double                  underside_unmet_mm2 = 0.;
};
// `enforced` are the contacts an enforcer asked for; each becomes a tip and an anchor.
Plan plan_tips(const PlanInput &input, const std::vector<TipSite> &enforced);

// The axis a neck from `site` leans along by the planner's search, `site` the only spot tried: zero where it clears
// straight down or no lean up to `input.max_tilt_rad` clears with the head fitting at the pin of `site`'s grade, else
// the least lean that does, one tilt step apart, and at it the azimuth whose end stands farthest from the model. A baked
// point carries no axis, so its list reads it again.
Vec3f neck_axis(const PlanInput &input, const TipSite &site);

// What the birth rule makes of a birth piece with no tip of a list the planner did not place: the piece continues the
// slab below as an overhang does, it is debris, it is a nub held at its merge, or it needs a neck, which the tip at
// `site` has, `site.axis` included, or which none clears. A nub is held as the plan's merge holds it: by a tip on its
// own part, or by the tips or the bed holding a part it lies within the hang of. Any other nub needs a neck as any
// birth does, and one that gets it holds its part for the nubs meeting it after it.
enum class BirthHold : uint8_t { Overhang, Debris, Nub, Tip, NoNeck };
struct BirthRead { BirthHold hold = BirthHold::NoNeck; TipSite site; };
// `pieces` index `input.components.pieces`, each one a birth piece; the reads follow their order. `tips` are the list's
// tips, which hold the parts they stand on.
std::vector<BirthRead> read_births(const PlanInput &input, const std::vector<size_t> &pieces, const std::vector<TipSite> &tips);
} // namespace ScaffoldSupport
} // namespace Slic3r
