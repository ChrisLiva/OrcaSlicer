#ifndef slic3r_SupportAnalysis_hpp_
#define slic3r_SupportAnalysis_hpp_

#include "MiniatureSupport.hpp"
#include "ModelSupportRisk.hpp"

#include "../ExPolygon.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace Slic3r {

class PrintObject;

namespace SupportAnalysis {

// Why a report is not Complete, or what the measurement found missing. Codes are values, so two
// reports can be compared without reading prose.
enum class Reason : uint8_t {
    NoProblem,              // the prepared problem named no required region
    EmittedMaterialMissing, // no emitted footprint was recorded, so coverage cannot be resolved
    MissingAnchor,          // some required region reached no emitted material
    StabilityUnavailable,
    DamageUnavailable,
};

// One printed slab: the closed Z interval [bottom_z, print_z] one layer occupies and the ground its
// material covers on it. Support material and model material are described the same way, so one
// connectivity walk serves both.
struct Slab
{
    double     bottom_z = 0.;
    double     print_z  = 0.;
    ExPolygons polygons;
};

// What one routed node area became on its support layer. GapAbove is the planned gap between a tip
// and the model: a virtual layer that is drawn and never extruded.
enum class Termination : uint8_t { Base, Roof, RoofFirstLayer, Floor, GapAbove };

// One routed node's area on one support layer, with every source it carries, recorded before
// draw_circles unions the layer's areas together and the source identity is gone.
struct AttributedArea
{
    std::vector<uint64_t> source_ids;         // sorted, unique
    ExPolygon             area;               // the polygon the router clipped, pre-union
    Termination           termination     = Termination::Base;
    bool                  virtual_gap     = false;  // never extruded: the planned top gap
    bool                  to_buildplate   = false;  // routed-source root fact
    // The width of the section that was drawn here, not the radius the router planned for the node.
    double                min_diameter_mm = 0.;
};

// One support layer's slab, what was routed onto it, and what was actually printed on it.
struct EmittedLayer
{
    size_t                      support_layer_index = 0;
    double                      print_z             = 0.;
    double                      bottom_z            = 0.;
    std::vector<AttributedArea> areas;
    ExPolygons                  emitted;                    // union of this layer's support extrusion footprints
    bool                        emitted_available   = false;
};

// What one generation attempt emitted, layer by layer. TreeSupport fills `areas` in draw_circles and
// `emitted` after generate_toolpaths, so the two halves describe the same attempt.
struct EmittedSupport
{
    std::vector<EmittedLayer> layers;
    double                    top_gap_mm          = 0.;  // planned gap between a source contact and the tip under it
    double                    max_layer_height_mm = 0.;  // the widest slab a tip may occupy under that gap

    bool empty() const { return layers.empty(); }
};

// What proves two reports were measured from one prepared problem: the region ids in order, the
// object layer and contact Z each came from, the polygon each one is, and the resolved extrusion
// width. Compared by value throughout, never by address and never by a hash of any of it.
struct CoverageKey
{
    std::vector<uint64_t> region_ids;
    std::vector<size_t>   region_layers;
    std::vector<double>   region_contact_z;
    ExPolygons            witnesses;
    double                extrusion_width_mm = 0.;

    bool operator==(const CoverageKey &other) const;
    bool operator!=(const CoverageKey &other) const { return ! (*this == other); }

    static CoverageKey from_problem(const MiniatureSupport::Problem &problem);
};

// What the measurement found for one required region.
struct RegionCoverage
{
    uint64_t              region_id = 0;
    std::vector<uint64_t> anchor_ids;              // the region's contact seeds, sorted
    // One bit per cell of the region's frozen witness lattice (MiniatureSupport::witness_cells): set
    // when the whole cell lies within the legal influence of a contact whose material was printed for
    // its own region, or within the contact distance in 3-D of such a contact one or more bands down
    // in the same overhang component. The complete cell, never its centre, and never a proximity guess.
    std::vector<bool>     covered;
    double                covered_mm2  = 0.;
    double                tip_print_z  = 0.;       // the print_z of the highest qualifying intersection
    bool                  anchored     = false;
    bool                  critical     = false;
    // What `RequiredRegion::printable` said of the region this coverage was measured for: false when the
    // overhang has no room for one support extrusion, and then the region carries no witness cells at all.
    bool                  printable    = true;
    // Some area the router carried this region's own sources into reached printed material, or a
    // reached contact of this region's overhang component carries one of its cells. Broader than
    // `anchored`, which also demands the region's own top-gap interval, and it is what keeps a group
    // in a comparison rather than letting it drop out.
    bool                  emitted_path = false;

    size_t cell_count() const { return covered.size(); }
    size_t covered_count() const;
};

// What the emitted material's own geometry says about whether the print can stand up. Geometric
// estimates, not force guarantees. `available` is false whenever any of it could not be measured,
// and a field is then not a zero result but no result at all.
struct Stability
{
    // Printed support components that reach neither the plate nor a termination the active settings
    // permit: material the print lays in mid-air.
    size_t unsupported_paths  = 0;
    // Source groups the router carried into drawn material none of which was ever printed: routed
    // provenance with no emitted path at all.
    size_t unrooted_groups    = 0;
    // The smallest signed distance from the sliced volume's centroid to the convex hull of what the
    // object stands on, over the object's own height. Negative when the centroid is outside the
    // hull. A balance proxy, not an adhesion or acceleration model.
    double min_bed_margin     = 0.;
    // The longest unbraced run of a branch over the thinnest cross-section printed along it.
    double max_slenderness    = 0.;
    // Explanatory, so a margin can be read: the area of that connected footprint and the height it
    // is normalized by.
    double bed_footprint_mm2  = 0.;
    double object_height_mm   = 0.;
    bool   available          = false;
};

// What the model around one placed contact is worth carrying it: the readings ModelSupportRisk::Sample
// defines, taken at the contact.
struct ContactRisk
{
    uint64_t                 seed_id = 0;
    ModelSupportRisk::Sample sample;
};

// What taking the generated support off the model would put at risk, group by group. A removal group
// is a connected component of printed support material and the contacts the router carried into it.
// Geometric estimates and a ranking of them: no force is calculated here and no cut is called safe.
struct Damage
{
    // Contacts the measurement could not answer for: the model under them was never measured, or no
    // probe could tell whether anything reaches them. Not damage of zero, and first in the order.
    size_t unknown_contacts     = 0;
    // Groups every probe met something on the way to: material that would have to be cut out through
    // the model or another group rather than pulled away from it.
    size_t inaccessible_groups  = 0;
    // The worst single group, and every group added up: contact area weighted by what the model
    // carries there, times the run each group reaches from its own root over the neck holding it.
    double max_group_risk       = 0.;
    double total_group_risk     = 0.;
    bool   available            = false;
};

enum class DamageField : uint8_t { None, UnknownContacts, InaccessibleGroups, MaxGroupRisk, TotalGroupRisk };

struct DamageComparison
{
    int         order = 0;
    DamageField field = DamageField::None;
};

// The tolerance two floating metrics compare within: relative to the larger of the two, never finer than 1e-9.
double comparison_epsilon(double a, double b);

// Where two measured Damage domains first differ, in the order the fields are written: `order` negative where
// `is` is the better of the two, positive where it is the worse, zero where they tie; `field` names the
// first field that differed, None on a tie. Reads neither side's `available`: callers settle that first.
DamageComparison compare_damage(const Damage &was, const Damage &is);

// How far source identity survived into printed material: what the intersection of the attributed
// masks with the actual extrusion footprints carried.
struct Provenance
{
    size_t emitted_areas       = 0;  // attributed areas whose intersection with printed material is non-empty
    size_t multi_source_areas  = 0;  // of those, the ones carrying more than one source region
    size_t max_sources_in_area = 0;  // the most source regions one printed area carries
    size_t traced_regions      = 0;  // regions that reached printed material of their own
};

// One measurement of one generated support pass against the problem it was generated for.
struct Report
{
    enum class Status : uint8_t { Complete, Unknown, UnresolvedCoverage };

    Status                      status = Status::Unknown;
    CoverageKey                 key;
    std::vector<RegionCoverage> coverage;             // one entry per region, in key order
    std::vector<uint64_t>       missing_anchor_ids;   // sorted
    std::vector<uint64_t>       critical_anchor_ids;  // sorted
    std::vector<uint64_t>       pinned_anchor_ids;    // sorted; the contacts the generator pinned
    Stability                   stability;
    Damage                      damage;
    double                      support_volume_mm3   = 0.;
    double                      raft_volume_mm3      = 0.;
    // The area of the union of every qualifying intersection, taken once: material two sources share
    // counts once globally while each source stays traceable to it.
    double                      measured_contact_mm2 = 0.;
    bool                        coverage_available   = false;
    // One entry per contact seed, in ascending seed id. Available on its own terms: contact estimates
    // can guide where a contact goes, and they are not a removal assessment for a whole group.
    std::vector<ContactRisk>    contact_risk;
    bool                        contact_risk_available = false;
    // What the contact selection did with the prepared problem's seeds, counted where the selection
    // ran rather than re-derived from the toolpaths: every seed the problem carried, the ones the
    // thinning kept outright, the ones it put back after a cell or an anchor came up short, and their
    // sum, the seeds this pass generated support for. A report built by hand leaves them at zero.
    size_t                      seeds_candidate = 0;
    size_t                      seeds_kept      = 0;
    size_t                      seeds_restored  = 0;
    size_t                      seeds_retained  = 0;
    Provenance                  provenance;
    std::vector<Reason>         reasons;
    bool has_reason(Reason reason) const;
};

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

// The model's own weakness field for contacts of `extrusion_width_mm`, built off the object's sliced layers
// (bottom_z, print_z, lslices per layer) and canceled through the object's Print. Status::Invalid where the
// width is not positive.
ModelSupportRisk::Field measure_model_risk(const PrintObject &object, double extrusion_width_mm);

// Measures one generated pass against a field the caller already built off the same object: volumes off the
// object's emitted support extrusions, coverage off the intersection of the attributed masks with those
// extrusions' width footprints. Infers nothing from spatial proximity: a footprint counts for a region only
// when it carries one of that region's own sources and sits in that region's planned top-gap interval.
Report measure(const PrintObject &object, const MiniatureSupport::Problem &problem, const EmittedSupport &emitted,
               const ModelSupportRisk::Field &risk);

// The same measurement with the field built here, for a caller holding no prebuilt one.
Report measure(const PrintObject &object, const MiniatureSupport::Problem &problem, const EmittedSupport &emitted);

// The same measurement with the ground taken again, now that the whole print has laid the bed
// adhesion under one physical instance of the object. The support measurement runs inside the support
// generation, which is over before the print emits any brim, so the footprint it took holds the
// object's own first layer, its raft and its plate-level support and nothing the print laid after.
// `adhesion` is that material in the object's own canonical unshifted instance frame, which is the
// frame the measurement was taken in: the caller applies the instance's own shift, because one
// generated pass shared by several copies stands on different ground under each of them. Returns a
// new report - an installed measurement is immutable - and leaves a report whose stability was never
// measured exactly as it was, because ground added to no measurement is still no measurement.
Report refresh_bed_footprint(const Report &report, const PrintObject &object, const EmittedSupport &emitted,
                             const Polygons &adhesion);

// The minimum width of one printed cross-section: the smallest distance between two parallel lines
// that enclose it, taken on its convex hull, which is the diameter of a branch drawn as a circle and
// the thickness of a strip. An empty or degenerate section measures zero, which the stability
// measurement reports as unknown rather than as infinitely slender.
double cross_section_width_mm(const ExPolygon &section);

// Whether one measured pass left a requirement of its own prepared problem open, read on its own
// terms with nothing to compare it against. Not "every witness cell covered" and not "every contact
// printed at its planned tip": no generated pass covers every cell of every detected overhang, and a
// tip drawn against the model's xy clearance prints its first material a few layers under the planned
// gap, so either reading would mark every miniature slice unresolved. What it asks is whether an
// overhang that needs support got none - a critical region the detector filed as wide enough for one
// support extrusion (`RegionCoverage::printable`) whose sources reached no printed material at all -
// or whether the pass routed a group that never printed. A region narrower than an extrusion is a
// sliver the detector filed, the band a shallow slope adds per layer or a speck, and the wall beside
// it carries it whether or not a branch reached it. A measurement that required nothing is resolved.
bool support_unresolved(const Report &report);

} // namespace SupportAnalysis
} // namespace Slic3r

#endif // slic3r_SupportAnalysis_hpp_
