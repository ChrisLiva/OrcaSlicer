#ifndef slic3r_MiniatureSupport_hpp_
#define slic3r_MiniatureSupport_hpp_

#include "../BoundingBox.hpp"
#include "../ExPolygon.hpp"
#include "../Point.hpp"
#include "ModelSupportRisk.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace Slic3r {
namespace MiniatureSupport {

// One cell of a region's frozen witness lattice: a lattice square clipped to the region polygon. A
// hole may cut one square into several pieces; they stay one cell, so a cell split by a hole is only
// ever covered as a whole.
struct WitnessCell
{
    Point       index;     // the lattice square this cell came from: its corner is index * pitch
    ExPolygons  area;      // that square clipped to the region polygon, never empty
    Points      corners;   // every vertex of `area`, the points the reach and visibility tests run on
    BoundingBox bbox;
};

// A region's frozen witness lattice. Frozen in the sense that it is a pure function of the region
// polygon and the resolved extrusion width, both of which the prepared problem holds by value: two
// attempts made from one problem lay the same cells in the same order and can be compared cell by cell.
struct Witnesses
{
    coord_t                  pitch = 0;   // the lattice square side, scaled
    std::vector<WitnessCell> cells;       // lattice order: y ascending, then x ascending

    size_t size() const { return cells.size(); }
    bool   empty() const { return cells.empty(); }
};

// One place a legacy tree support has to anchor to: a single connected overhang polygon on a single
// object layer, as overhang detection left it. Two polygons on different layers are two regions, so
// material printed under one of them can never stand in for the other.
struct RequiredRegion
{
    // Dense, counted from zero over the whole problem in (object layer, polygon order). A value, so
    // it survives the node pool, the layer vectors and the attempt that produced them.
    uint64_t  id             = 0;
    size_t    object_layer   = 0;   // the object layer the overhang polygon was detected on
    ExPolygon polygon;              // the overhang polygon itself, frozen
    double    contact_z_mm   = 0.;  // the underside of that layer: where a tip has to reach
    double    legal_reach_mm = 0.;  // how far from the polygon a contact may sit and still anchor it
    bool      critical       = false;
    // Whether the region has room for one extrusion of the problem's resolved width (`holds_an_extrusion`);
    // a region that has not carries no witness lattice (`region_witnesses`).
    bool      printable      = true;
    // The directed overhang component the seeding pass filed the region under: the root index of the
    // union-find over regions linked downward within two layers. A region built by hand is its own
    // component unless the test says otherwise.
    size_t    component      = 0;
    // The region's frozen lattice, laid once when the problem is built; null on a region built by hand.
    std::shared_ptr<const Witnesses> witnesses;
};

// One contact the generator placed for a region. Position and radius are what the contact pass chose;
// `pinned` is the generator's own "keep this one" mark (a painted enforcer, a Hybrid big overhang).
struct ContactSeed
{
    uint64_t id        = 0;   // dense, counted from zero over the whole problem
    uint64_t region_id = 0;
    Point    position;
    double   radius_mm = 0.;
    bool     pinned    = false;
    bool     critical  = false;
};

// The frozen anchoring problem the legacy generation runs against. Every id in it is a value assigned
// by a deterministic rule over frozen inputs, so a slice repeated from one prepared problem hands out
// the same ids and its report reads against the same regions and seeds.
struct Problem
{
    std::vector<RequiredRegion> regions;
    std::vector<ContactSeed>    seeds;
    double                      extrusion_width_mm = 0.;   // resolved support extrusion width
    // `support_contact_min_distance` with the mode on, 0 otherwise: the thinning distance and the
    // carry distance in one number.
    double                      contact_min_distance_mm = 0.;

    bool empty() const { return regions.empty() && seeds.empty(); }
};

// Whether `area` has room for one extrusion of `extrusion_width_mm`: some of it survives an erosion by
// half that width. A region narrower than that is a sliver the overhang detector filed - the band a
// shallow slope adds per layer, a speck - not a place a line can be laid or an overhang that needs one.
bool holds_an_extrusion(const ExPolygons &area, double extrusion_width_mm);

// Lays a lattice of half the resolved support extrusion width over scaled object coordinates (the
// lines sit at multiples of the pitch, so where a region happens to start never moves its cells),
// clips every square to `polygon` and keeps every nonempty clipped cell, boundary slivers included.
// An empty polygon or a non-positive width has no cells at all.
Witnesses witness_cells(const ExPolygon &polygon, double extrusion_width_mm);

// The lattice `region` was frozen with, or a freshly laid one when it carries none. `witness_cells`
// reads nothing but the polygon and the width, so both branches hand back the same cells in the same
// order and a region built by hand costs only the lattice it did not keep. A region whose `printable`
// is false carries no cells at all, whatever lattice was frozen with it.
std::shared_ptr<const Witnesses> region_witnesses(const RequiredRegion &region, double extrusion_width_mm);

// How far from a contact a region may still be anchored by it: half the smaller of the two limits
// the contact sampler already runs under, half whichever one is positive, or zero when neither is.
// A conservative geometric policy read off the active spacing and bridging limits, not a calibrated
// unsupported-span guarantee.
double legal_reach(double branch_distance_mm, double max_bridge_length_mm);

// Which cells of `witnesses` a contact at `position` legally covers inside `polygon`: the complete
// cell no further than `reach_mm` from the contact, and the straight segment from the contact to
// every point of the cell inside the region, so a hole, a notch or a second connected piece blocks
// the contact outright instead of being spanned by a disk. Conservative where the region is not
// convex: a cell the contact can see around a corner is reported uncovered rather than covered.
// One entry per cell, in lattice order; a non-positive reach covers nothing.
std::vector<bool> covered_by_contact(const ExPolygon &polygon, const Witnesses &witnesses,
                                     const Point &position, double reach_mm);

// `region` indexes `problem.regions` and `cell` indexes that region's witness lattice. Both are
// positions, never `RequiredRegion::id` or `ContactSeed::id`: the prepared problem hands those out
// dense from zero so the two coincide, and the rule still addresses by position.
struct CoveredCell { size_t region; size_t cell; };

// The cells a contact stands behind, own region and carried. Built once from the frozen problem:
// regions grouped by component and sorted by contact z, each region's cached witness lattice.
class CoverageRule
{
public:
    explicit CoverageRule(const Problem &problem);
    // `seed` indexes `problem.seeds` and names the region, hence its z and component; `position`
    // may be the relocated one. A seed index past the end, or one whose `region_id` is past the end
    // of `problem.regions`, gives back an empty vector. The rule holds `problem` by reference and
    // never outlives it.
    std::vector<CoveredCell> cells(size_t seed, const Point &position) const;

private:
    const Problem                                &m_problem;
    // One entry per region, in region order: the frozen lattice or the one laid for a region built by
    // hand, held so the cells stay alive as long as the rule does.
    std::vector<std::shared_ptr<const Witnesses>> m_lattice;
    // Region indices sorted by component, then contact z, then index, and per region the half-open
    // range of that order holding its own component: the regions a contact may carry, in z order.
    std::vector<size_t>                           m_order;
    std::vector<size_t>                           m_group_first;
    std::vector<size_t>                           m_group_last;
};

// What one selection kept, addressed by source contact id, which the prepared problem hands out dense from zero.
struct Selection
{
    std::vector<ContactSeed> retained;      // the kept contacts by value, in ascending source id
    // How many of `retained` the decimation had dropped and the add-back turned back on: one contact
    // per printable region the retained set left a witness cell of uncovered. Zero where the pass put
    // nothing back.
    size_t                   seeds_restored = 0;
};

// What one prepared problem's contacts come to, under three rules in this order.
//
// Decimation first, over the source positions and by `contact_min_distance_mm` alone: contacts are
// walked by contact z ascending, radius descending, object layer ascending, then position, and one
// goes when a contact already kept of its own overhang component lies strictly within the distance of
// it in 3-D. The bound is strict, a pinned contact is kept and crowds nobody, and a distance of 0 or
// less decimates nothing at all.
//
// The add-back then reads what the survivors stand behind under `CoverageRule`, at the contacts' own
// positions: a printable region that carries contacts of its own and has a witness cell no survivor
// covers keeps its lowest-id contact, and `Selection::seeds_restored` counts what came back. A region
// too narrow to hold one extrusion (`printable` false) carries no cells, so nothing is put back for it.
//
// Placement runs last, over the retained set and against `risk` measured off the model itself. A contact
// may stand where it is or on its region's own frozen lattice, no further from its source than the region
// allows, and only where the region gives up no covered cell. Pinned contacts never move. Nothing moves at
// all unless `risk` is a complete measurement: an unmeasured field is the absence of a reading, never a
// reading of no risk.
//
// Deterministic over the frozen problem and field alone.
Selection select_contacts(const Problem &problem, const ModelSupportRisk::Field &risk);

} // namespace MiniatureSupport
} // namespace Slic3r

#endif // slic3r_MiniatureSupport_hpp_
