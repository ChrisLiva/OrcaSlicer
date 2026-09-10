#include "SupportAnalysis.hpp"

#include "../AABBMesh.hpp"
#include "../ClipperUtils.hpp"
#include "../Geometry/ConvexHull.hpp"
#include "../Layer.hpp"
#include "../Print.hpp"

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>
#include <queue>
#include <set>

namespace Slic3r {
namespace SupportAnalysis {

size_t RegionCoverage::covered_count() const
{
    return size_t(std::count(covered.begin(), covered.end(), true));
}

bool CoverageKey::operator==(const CoverageKey &other) const
{
    // Value by value, the witness polygons included: two problems that name the same regions on the
    // same layers with the same geometry are the same problem, and nothing else is.
    return region_ids == other.region_ids && region_layers == other.region_layers &&
           region_contact_z == other.region_contact_z && witnesses == other.witnesses &&
           extrusion_width_mm == other.extrusion_width_mm;
}

CoverageKey CoverageKey::from_problem(const MiniatureSupport::Problem &problem)
{
    CoverageKey key;
    key.region_ids.reserve(problem.regions.size());
    key.region_layers.reserve(problem.regions.size());
    key.region_contact_z.reserve(problem.regions.size());
    key.witnesses.reserve(problem.regions.size());
    for (const MiniatureSupport::RequiredRegion &region : problem.regions) {
        key.region_ids.push_back(region.id);
        key.region_layers.push_back(region.object_layer);
        key.region_contact_z.push_back(region.contact_z_mm);
        key.witnesses.push_back(region.polygon);
    }
    key.extrusion_width_mm = problem.extrusion_width_mm;
    return key;
}

namespace {

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
bool on_bed(double bottom_z)
{
    return bottom_z <= EPSILON;
}

// What positive-area overlap between the polygons of consecutive touching slabs joins up. Two slabs
// take part only when their closed Z intervals touch, so material separated by a slab that printed
// nothing is separate however far the two overlap in plan, and an overlap that has no area joins
// nothing either. `slabs` is ordered by ascending print_z. A component is rooted where one of its pieces
// starts at `ground_z`: the plate for support, whose first slab (the raft's, where there is one) starts
// there, and the object's own first slab for the model, which the plate or a raft carries.
Components build_components(const std::vector<Slab> &slabs, double ground_z)
{
    Components out;
    out.slab_range.resize(slabs.size());
    for (size_t s = 0; s < slabs.size(); ++ s) {
        out.slab_range[s].first = out.pieces.size();
        for (const ExPolygon &polygon : slabs[s].polygons) {
            Piece piece;
            piece.polygon  = polygon;
            piece.bottom_z = slabs[s].bottom_z;
            piece.print_z  = slabs[s].print_z;
            out.pieces.emplace_back(std::move(piece));
        }
        out.slab_range[s].second = out.pieces.size();
    }

    std::vector<size_t> parent(out.pieces.size());
    std::iota(parent.begin(), parent.end(), size_t(0));
    const std::function<size_t(size_t)> find = [&parent](size_t x) {
        while (parent[x] != x) {
            parent[x] = parent[parent[x]];
            x         = parent[x];
        }
        return x;
    };

    // Two pieces whose boxes do not overlap intersect in nothing, so the clip runs on the pairs whose
    // boxes do: a few per piece on a layer of hundreds, where the pairwise clip is hundreds per piece.
    std::vector<BoundingBox> boxes;
    boxes.reserve(out.pieces.size());
    for (const Piece &piece : out.pieces)
        boxes.push_back(get_extents(piece.polygon));
    for (size_t s = 0; s + 1 < slabs.size(); ++ s) {
        if (slabs[s + 1].bottom_z > slabs[s].print_z + 1e-6)
            continue; // the two slabs do not touch: nothing printed between them
        for (size_t i = out.slab_range[s].first; i < out.slab_range[s].second; ++ i)
            for (size_t j = out.slab_range[s + 1].first; j < out.slab_range[s + 1].second; ++ j) {
                if (! boxes[i].overlap(boxes[j]))
                    continue;
                if (intersection_ex(ExPolygons{ out.pieces[i].polygon }, ExPolygons{ out.pieces[j].polygon }).empty())
                    continue;
                out.pieces[i].above.push_back(j);
                out.pieces[j].below.push_back(i);
                parent[find(i)] = find(j);
            }
    }

    std::vector<size_t> label(out.pieces.size(), size_t(-1));
    for (size_t i = 0; i < out.pieces.size(); ++ i) {
        const size_t root = find(i);
        if (label[root] == size_t(-1))
            label[root] = out.count ++;
        out.pieces[i].component = label[root];
    }
    out.bed_rooted.assign(out.count, 0);
    for (const Piece &piece : out.pieces)
        if (piece.bottom_z <= ground_z + EPSILON)
            out.bed_rooted[piece.component] = 1;
    return out;
}

// Which components of `support` stand on something. The plate first, then a termination against the
// object, which counts only where the settings permit resting on the model at all and where the model
// it rests on is itself connected to the object's own first layer, which the plate or a raft carries.
// `model` is the object's own sliced body, in the same frame as the support and already grouped into
// its own components.
std::vector<char> rooted_components(const Components &support, const std::vector<Slab> &model_slabs, const Components &model,
                                    bool on_build_plate_only, double bottom_gap)
{
    std::vector<char> rooted = support.bed_rooted;
    if (on_build_plate_only)
        return rooted;
    std::vector<double> model_tops(model_slabs.size(), 0.);
    for (size_t s = 0; s < model_slabs.size(); ++ s)
        model_tops[s] = model_slabs[s].print_z;
    for (const Piece &piece : support.pieces) {
        if (rooted[piece.component])
            continue;
        // The object material this slab comes down onto: no further below its underside than the
        // gap the settings leave between a support bottom and the object, plus the slab itself,
        // because the surface it stands over is only known to the layer it was sliced at.
        const double reach = bottom_gap + (piece.print_z - piece.bottom_z);
        size_t       s     = size_t(std::lower_bound(model_tops.begin(), model_tops.end(),
                                                     piece.bottom_z - reach - 1e-6) - model_tops.begin());
        for (; s < model_slabs.size() && model_slabs[s].print_z <= piece.bottom_z + 1e-6; ++ s) {
            for (size_t m = model.slab_range[s].first; m < model.slab_range[s].second; ++ m) {
                // The object it rests on has to be standing up itself: material that is floating
                // holds nothing.
                if (! model.bed_rooted[model.pieces[m].component])
                    continue;
                if (intersection_ex(ExPolygons{ piece.polygon }, ExPolygons{ model.pieces[m].polygon }).empty())
                    continue;
                rooted[piece.component] = 1;
                break;
            }
            if (rooted[piece.component])
                break;
        }
    }
    return rooted;
}

// The middle of a piece's section in mm: the centroid of its outline, half way up its own slab.
Vec3d piece_middle(const Piece &piece)
{
    const Point point = piece.polygon.contour.centroid();
    return Vec3d(point.x() * SCALING_FACTOR, point.y() * SCALING_FACTOR, 0.5 * (piece.bottom_z + piece.print_z));
}

// The piece of `slab` a printed part sits in, or npos when the part has no outline or no piece holds
// its first point.
size_t piece_containing(const Components &support, size_t slab, const ExPolygon &part)
{
    if (part.contour.points.empty())
        return size_t(-1);
    for (size_t k = support.slab_range[slab].first; k < support.slab_range[slab].second; ++ k)
        if (support.pieces[k].polygon.contains(part.contour.points.front(), true))
            return k;
    return size_t(-1);
}

// What the support extrusions covered, layer by layer, at the width they were emitted with. The raft
// the object stands on comes first and comes from the support layers themselves: it is emitted
// support material that carries no routed provenance, so the attributed record does not hold it, and
// the ground it covers is read off its own extrusions the same way. `slab_of_layer` names the emitted
// layer each slab came from, or npos for a raft slab.
std::vector<Slab> printed_support_slabs(const PrintObject &object, const EmittedSupport &emitted,
                                        std::vector<size_t> &slab_of_layer)
{
    std::vector<Slab> slabs;
    slab_of_layer.clear();
    const ConstSupportLayerPtrsAdaptor support_layers = object.support_layers();
    for (size_t i = 0; i < object.support_raft_layers() && i < support_layers.size(); ++ i) {
        const SupportLayer *layer = support_layers[i];
        Slab slab;
        slab.print_z  = layer->print_z;
        slab.bottom_z = layer->print_z - layer->height;
        slab.polygons = union_ex(layer->support_fills.polygons_covered_by_width(0.f));
        if (slab.polygons.empty())
            continue;
        slabs.emplace_back(std::move(slab));
        slab_of_layer.push_back(size_t(-1));
    }
    for (size_t l = 0; l < emitted.layers.size(); ++ l) {
        const EmittedLayer &layer = emitted.layers[l];
        if (! layer.emitted_available || layer.emitted.empty())
            continue;
        Slab slab;
        slab.print_z  = layer.print_z;
        slab.bottom_z = layer.bottom_z;
        slab.polygons = layer.emitted;
        slabs.emplace_back(std::move(slab));
        slab_of_layer.push_back(l);
    }
    return slabs;
}

// The ground the object stands on: its own first layer where that layer is the one on the plate,
// every printed support slab that starts on the plate, the raft among them, and whatever bed adhesion
// the print laid afterwards. A support slab that starts on the plate roots its own component by
// construction, so which components hold something up does not enter this. A skirt is neither support
// nor the object's, so it never gets here.
ExPolygons bed_ground(const std::vector<Slab> &model_slabs, const std::vector<Slab> &support_slabs, const Polygons &adhesion)
{
    ExPolygons ground;
    if (! model_slabs.empty() && on_bed(model_slabs.front().bottom_z))
        ground = model_slabs.front().polygons;
    for (const Slab &slab : support_slabs)
        if (on_bed(slab.bottom_z))
            append(ground, slab.polygons);
    append(ground, union_ex(adhesion));
    return union_ex(ground);
}

// Where the sliced volume's mass sits in plan: each slice weighted by the layer height it fills,
// holes carrying their own negative area so a hollow section pulls nothing toward its middle. False
// where there is no mass at all, which is no centroid rather than the origin.
bool volume_centroid(const PrintObject &object, Point &centroid)
{
    double weight = 0.;
    Vec2d  moment(0., 0.);
    for (const Layer *layer : object.layers())
        for (const ExPolygon &slice : layer->lslices) {
            const double contour_weight = slice.contour.area() * layer->height;
            weight += contour_weight;
            moment += contour_weight * slice.contour.centroid().cast<double>();
            for (const Polygon &hole : slice.holes) {
                const double hole_weight = hole.area() * layer->height;  // holes have negative area
                weight += hole_weight;
                moment += hole_weight * hole.centroid().cast<double>();
            }
        }
    if (weight <= 0.)
        return false;
    centroid = Point(Vec2d(moment / weight));
    return true;
}

// The signed distance from `centroid` to the convex hull of `ground`, over `height`: positive inside,
// negative outside, a length against a length so two objects of different sizes are read on one
// scale. False where there is no footprint or no height, which is an unknown margin, not a zero one.
bool bed_margin(const ExPolygons &ground, const Point &centroid, double height, double &margin)
{
    const Polygon hull = Geometry::convex_hull(ground);
    if (hull.points.size() < 3 || height <= 0.)
        return false;
    double       nearest = std::numeric_limits<double>::max();
    const size_t n       = hull.points.size();
    for (size_t i = 0; i < n; ++ i)
        nearest = std::min(nearest, Line(hull.points[i], hull.points[(i + 1) % n]).distance_to(centroid));
    margin = (hull.contains(centroid) ? nearest : - nearest) * SCALING_FACTOR / height;
    return true;
}

// The emitted material's own geometry, measured in the owning object's canonical unshifted instance
// frame: the object's layers and its support layers are both already in that frame, so a pose
// evaluation transforms this result per physical instance rather than measuring in a shifted one.
void measure_stability(Report &report, const PrintObject &object, const EmittedSupport &emitted,
                       const std::vector<std::vector<ExPolygons>> &printed,
                       const std::vector<uint64_t> &region_of_seed, const std::vector<Slab> &support_slabs,
                       const std::vector<size_t> &slab_of_layer, const Components &support)
{
    Stability &stability = report.stability;

    const std::vector<Slab> model_slabs = model_slabs_of(object);
    if (model_slabs.empty()) {
        stability = Stability();
        return; // nothing sliced: there is no frame to measure anything in
    }
    const Components model = build_components(model_slabs, model_slabs.front().bottom_z);

    // What holds each component up: the plate, or model material where the settings permit resting on
    // it. The same rule the generator removes mid-air material by, so what it left standing is what is
    // measured here.
    const std::vector<char> rooted = rooted_components(support, model_slabs, model, object.config().support_on_build_plate_only.value,
                                                       std::max(0., object.config().support_bottom_z_distance.value));
    for (size_t c = 0; c < support.count; ++ c)
        if (! rooted[c])
            ++ stability.unsupported_paths;

    // The connected ground the object stands on. What the print lays on the plate after the support
    // is generated - the brim - is not here yet, and joins it where the footprint is refreshed.
    const ExPolygons connected  = bed_ground(model_slabs, support_slabs, Polygons{});
    stability.bed_footprint_mm2 = area(connected) * SCALING_FACTOR * SCALING_FACTOR;
    stability.object_height_mm  = model_slabs.back().print_z - model_slabs.front().bottom_z;

    Point centroid;
    if (! volume_centroid(object, centroid) ||
        ! bed_margin(connected, centroid, stability.object_height_mm, stability.min_bed_margin)) {
        stability = Stability();
        return; // no footprint, no mass or no height: the margin is unknown, not zero
    }

    // What each printed piece of a branch is as thick as: the thinnest section the router drew into
    // it, which is one branch's own section rather than the width of everything the layer printed
    // unioned together. Where nothing routed into a piece, the raft under the object among them, the
    // piece's own section stands.
    std::vector<double> diameter(support.pieces.size(), 0.);
    for (size_t s = 0; s < support_slabs.size(); ++ s) {
        const size_t l = slab_of_layer[s];
        if (l == size_t(-1))
            continue;  // a raft slab: no routed area drew it, so its own section is its width
        for (size_t a = 0; a < emitted.layers[l].areas.size(); ++ a) {
            const double section = emitted.layers[l].areas[a].min_diameter_mm;
            if (section <= 0.)
                continue;
            for (const ExPolygon &part : printed[l][a]) {
                const size_t k = piece_containing(support, s, part);
                if (k != size_t(-1))
                    diameter[k] = diameter[k] > 0. ? std::min(diameter[k], section) : section;
            }
        }
    }
    for (size_t k = 0; k < support.pieces.size(); ++ k)
        if (diameter[k] <= 0.)
            diameter[k] = cross_section_width_mm(support.pieces[k].polygon);

    // A path runs between the places a branch is held: a tip with nothing above it, a junction where
    // branches meet or part, and the bottom where it roots. Everything in between is one unbraced
    // run, measured through the middle of the sections it passes through.
    const auto braced = [&support](size_t k) {
        return support.pieces[k].above.size() != 1 || support.pieces[k].below.size() != 1;
    };
    for (size_t k = 0; k < support.pieces.size(); ++ k) {
        if (! braced(k))
            continue;
        for (size_t start : support.pieces[k].below) {
            double length = 0., thinnest = diameter[k];
            size_t here = k, there = start;
            for (;;) {
                length  += (piece_middle(support.pieces[here]) - piece_middle(support.pieces[there])).norm();
                thinnest = std::min(thinnest, diameter[there]);
                if (braced(there))
                    break;
                here  = there;
                there = support.pieces[there].below.front();
            }
            if (thinnest <= 0.) {
                // A section the measurement cannot size leaves slenderness unknown, and one unknown
                // measure leaves the domain unknown rather than partly filled in.
                stability = Stability();
                return;
            }
            stability.max_slenderness = std::max(stability.max_slenderness, length / thinnest);
        }
    }

    // Routed provenance against printed material, group by group. A group the router carried into
    // drawn areas none of which was ever printed has no emitted path at all, which is a separate
    // failure from material that printed and floats.
    std::vector<char> routed(report.coverage.size(), 0), reached(report.coverage.size(), 0);
    for (size_t l = 0; l < emitted.layers.size(); ++ l)
        for (size_t a = 0; a < emitted.layers[l].areas.size(); ++ a) {
            const AttributedArea &area     = emitted.layers[l].areas[a];
            const bool            on_paper = ! area.virtual_gap && ! printed[l][a].empty();
            for (uint64_t seed_id : area.source_ids) {
                if (seed_id >= region_of_seed.size())
                    continue;
                const uint64_t region_id = region_of_seed[size_t(seed_id)];
                if (region_id >= routed.size())
                    continue;
                routed[size_t(region_id)] = 1;
                if (on_paper)
                    reached[size_t(region_id)] = 1;
            }
        }
    for (size_t r = 0; r < routed.size(); ++ r) {
        report.coverage[r].emitted_path = reached[r] != 0;
        if (routed[r] && ! reached[r])
            ++ stability.unrooted_groups;
    }

    stability.available = true;
}

// The removal groups of one emitted pass, and what taking one of them off the model would put at
// risk. A group is a connected component of printed support material - what the extrusions actually
// covered, so the roof gap the router draws over a tip and never extrudes neither joins two groups
// nor divides one - and every contact the router carried into a component's printed material belongs
// to that component, merged branches and shared roots included. One printed area is weighed once
// whatever it carries, at the worst estimate of the contacts sharing it. Geometric estimates
// throughout: a ranking of what removing a group would put at risk, never a force calculation and
// never a safe cutting procedure.
void measure_damage(Report &report, const PrintObject &object, const MiniatureSupport::Problem &problem,
                    const EmittedSupport &emitted, const std::vector<std::vector<ExPolygons>> &printed,
                    const std::vector<uint64_t> &region_of_seed, const std::vector<Slab> &support_slabs,
                    const std::vector<size_t> &slab_of_layer, const Components &support)
{
    Damage      &damage = report.damage;
    const double width  = problem.extrusion_width_mm;
    // Contacts were placed and what the model carries them on was never measured: the groups they
    // make cannot be weighed, and an unweighed group is not a group that costs nothing.
    if (! problem.seeds.empty() && (! report.contact_risk_available || ! (width > 0.)))
        return;

    // The estimate each contact carries, by id.
    std::vector<const ModelSupportRisk::Sample *> sample_of_seed(problem.seeds.size(), nullptr);
    for (const ContactRisk &contact : report.contact_risk)
        if (contact.seed_id < sample_of_seed.size())
            sample_of_seed[size_t(contact.seed_id)] = &contact.sample;

    // Which slab each emitted layer became, so printed material can be found in the component graph.
    std::vector<size_t> slab_of_emitted(emitted.layers.size(), size_t(-1));
    for (size_t s = 0; s < slab_of_layer.size(); ++ s)
        if (slab_of_layer[s] != size_t(-1))
            slab_of_emitted[slab_of_layer[s]] = s;

    std::vector<double>             group_weight(support.count, 0.);
    std::vector<std::set<uint64_t>> group_contacts(support.count);
    std::vector<size_t>             piece_of_seed(problem.seeds.size(), size_t(-1));

    for (size_t l = 0; l < emitted.layers.size(); ++ l) {
        const EmittedLayer &layer = emitted.layers[l];
        const size_t        s     = slab_of_emitted[l];
        if (s == size_t(-1))
            continue;
        for (size_t a = 0; a < layer.areas.size(); ++ a) {
            const AttributedArea &attributed = layer.areas[a];
            if (attributed.virtual_gap || printed[l][a].empty())
                continue;
            // The contacts this printed material actually touches the model with: material further
            // down a branch is what holds a contact up, not what it is attached by.
            std::vector<uint64_t> here;
            for (uint64_t seed_id : attributed.source_ids) {
                if (seed_id >= region_of_seed.size())
                    continue;
                const uint64_t region_id = region_of_seed[size_t(seed_id)];
                if (region_id >= problem.regions.size())
                    continue;
                const double tip_top = problem.regions[size_t(region_id)].contact_z_mm - emitted.top_gap_mm;
                const double tol     = std::max(1e-9, 1e-6 * std::abs(tip_top));
                if (layer.print_z > tip_top + tol || layer.print_z < tip_top - emitted.max_layer_height_mm - tol)
                    continue;
                here.push_back(seed_id);
            }
            if (here.empty())
                continue;
            // The piece of printed support this material is part of, and so the group it belongs to.
            size_t piece = size_t(-1);
            for (const ExPolygon &part : printed[l][a]) {
                piece = piece_containing(support, s, part);
                if (piece != size_t(-1))
                    break;
            }
            if (piece == size_t(-1))
                continue;
            const size_t group = support.pieces[piece].component;
            double       worst = 0.;
            for (uint64_t seed_id : here) {
                const ModelSupportRisk::Sample *sample = sample_of_seed[size_t(seed_id)];
                // A contact the model was never measured under is already counted as unknown, and it
                // weighs nothing here rather than weighing zero.
                if (sample == nullptr || sample->status == ModelSupportRisk::Sample::Status::Unknown)
                    continue;
                worst = std::max(worst, sample->risk_per_mm2);
                group_contacts[group].insert(seed_id);
                if (piece_of_seed[size_t(seed_id)] == size_t(-1))
                    piece_of_seed[size_t(seed_id)] = piece;
            }
            group_weight[group] += area(printed[l][a]) * SCALING_FACTOR * SCALING_FACTOR * worst;
        }
    }

    // How far each piece of support stands from the ground its own group is held by, run through the
    // printed material itself: the plate where the group reaches it, and the lowest material the
    // group has where it does not. Measured through the middle of the sections a run passes through.
    std::vector<double> lowest(support.count, std::numeric_limits<double>::max());
    for (const Piece &piece : support.pieces)
        lowest[piece.component] = std::min(lowest[piece.component], piece.bottom_z);
    std::vector<double> run(support.pieces.size(), std::numeric_limits<double>::max());
    std::priority_queue<std::pair<double, size_t>, std::vector<std::pair<double, size_t>>,
                        std::greater<std::pair<double, size_t>>> frontier;
    for (size_t k = 0; k < support.pieces.size(); ++ k) {
        const Piece &piece = support.pieces[k];
        if (! on_bed(piece.bottom_z) && piece.bottom_z > lowest[piece.component] + 1e-9)
            continue;
        run[k] = 0.;
        frontier.emplace(0., k);
    }
    while (! frontier.empty()) {
        const std::pair<double, size_t> here = frontier.top();
        frontier.pop();
        if (here.first > run[here.second] + 1e-12)
            continue;
        for (const std::vector<size_t> *side : { &support.pieces[here.second].below, &support.pieces[here.second].above })
            for (size_t there : *side) {
                const double step = here.first +
                                    (piece_middle(support.pieces[there]) - piece_middle(support.pieces[here.second])).norm();
                if (step < run[there]) {
                    run[there] = step;
                    frontier.emplace(step, there);
                }
            }
    }

    // What stands in the way of reaching each group. Its own material is what is being removed, so it
    // is never what makes it unreachable; every other group's printed material is.
    std::vector<double> slab_tops;
    slab_tops.reserve(support_slabs.size());
    for (const Slab &slab : support_slabs)
        slab_tops.push_back(slab.print_z);
    bool any_group = false;
    for (size_t g = 0; g < support.count; ++ g)
        any_group = any_group || ! group_contacts[g].empty();
    indexed_triangle_set model;
    if (any_group && object.model_object() != nullptr) {
        model = object.model_object()->raw_indexed_triangle_set();
        // The contacts and slab tops are print_z, which a raft lifts, while trafo_centered() puts the
        // object's bottom at z 0.
        its_transform(model, Geometry::translation_transform(Vec3d(0., 0., object.slicing_parameters().object_print_z_min)) *
                                 object.trafo_centered());
    }
    const AABBMesh mesh(model);

    for (size_t g = 0; g < support.count; ++ g) {
        // Support the router carried no contact into holds nothing off the model: there is no removal
        // to estimate the damage of.
        if (group_contacts[g].empty())
            continue;
        // A long run off a narrow neck is a lever: the same contact area weighs more the further the
        // group holding it reaches from its own root. The neck is the model's, clamped at one
        // extrusion the same way the contact estimate clamps it.
        double lever = 0.;
        for (uint64_t seed_id : group_contacts[g]) {
            const size_t piece = piece_of_seed[size_t(seed_id)];
            if (piece >= run.size() || run[piece] == std::numeric_limits<double>::max())
                continue;
            const ModelSupportRisk::Sample *sample = sample_of_seed[size_t(seed_id)];
            if (sample == nullptr)
                continue;
            lever = std::max(lever, run[piece] / std::max(sample->neck_width_mm, width));
        }
        const double risk = group_weight[g] * (1. + lever);
        damage.total_group_risk += risk;
        damage.max_group_risk    = std::max(damage.max_group_risk, risk);

        // Whether anything can reach the group at all, tried at its contacts in turn until one of
        // them answers. A count of what could not be reached and a count of what could not be
        // answered for, both kept beside the weight rather than folded into it.
        std::vector<ExPolygons> others(support_slabs.size());
        for (size_t s = 0; s < support_slabs.size(); ++ s)
            for (size_t k = support.slab_range[s].first; k < support.slab_range[s].second; ++ k)
                if (support.pieces[k].component != g)
                    others[s].push_back(support.pieces[k].polygon);
        bool   reached = false;
        size_t unknown = 0;
        for (uint64_t seed_id : group_contacts[g]) {
            if (seed_id >= problem.seeds.size())
                continue;
            const MiniatureSupport::ContactSeed &seed = problem.seeds[size_t(seed_id)];
            if (seed.region_id >= problem.regions.size())
                continue;
            const Vec3d where(unscale<double>(seed.position.x()), unscale<double>(seed.position.y()),
                              problem.regions[size_t(seed.region_id)].contact_z_mm);
            const ModelSupportRisk::Access access =
                ModelSupportRisk::assess_access(mesh, others, slab_tops, where, 0.5 * width);
            if (access.status == ModelSupportRisk::Access::Status::Clear) {
                reached = true;
                break;
            }
            if (access.status == ModelSupportRisk::Access::Status::Unknown)
                ++ unknown;
        }
        if (reached)
            continue;
        // Unknown is not blocked: a group the probe could not answer for is an unknown contact, and
        // only a group every probe actually met something in is one nothing can reach.
        if (unknown > 0)
            damage.unknown_contacts += unknown;
        else
            ++ damage.inaccessible_groups;
    }
    damage.available = true;
}

// What the model itself says about each contact the generator placed: the width of the solid under
// the contact, the narrowest constriction between that solid and the bed, and how far the contact
// sits from it.
void measure_contact_risk(Report &report, const PrintObject &object, const MiniatureSupport::Problem &problem,
                          const ModelSupportRisk::Field &field)
{
    if (problem.seeds.empty() || ! (problem.extrusion_width_mm > 0.))
        return;
    // A canceled or malformed field is no measurement, and an absent measurement is not a zero one.
    if (field.status != ModelSupportRisk::Field::Status::Complete)
        return;

    // One reading per seed, and each is a search over the whole field from a point no other reading
    // depends on. The field is const here and the sampler keeps no state across calls, so the seeds
    // are measured together, one output slot each; what the report counts and how it is ordered is
    // decided afterwards, serially, off the filled slots.
    report.contact_risk.resize(problem.seeds.size());
    tbb::parallel_for(tbb::blocked_range<size_t>(0, problem.seeds.size()),
        [&](const tbb::blocked_range<size_t> &range) {
            for (size_t i = range.begin(); i < range.end(); ++ i) {
                const MiniatureSupport::ContactSeed &seed = problem.seeds[i];
                report.contact_risk[i].seed_id = seed.id;
                // The layer the contact's own region was detected on: the model solid a tip has to reach.
                const size_t layer = seed.region_id < problem.regions.size() ?
                                         problem.regions[size_t(seed.region_id)].object_layer : object.layers().size();
                report.contact_risk[i].sample = ModelSupportRisk::sample(field, layer, seed.position);
            }
        });
    for (const ContactRisk &entry : report.contact_risk)
        if (entry.sample.status == ModelSupportRisk::Sample::Status::Unknown)
            ++ report.damage.unknown_contacts;
    std::sort(report.contact_risk.begin(), report.contact_risk.end(),
              [](const ContactRisk &a, const ContactRisk &b) { return a.seed_id < b.seed_id; });
    report.contact_risk_available = true;
}

} // namespace

// The object's own body, sliced. It is what a branch may terminate against, what has to be connected
// to its own first layer before such a termination holds anything up, and that first layer, which the
// plate or a raft carries, is what the object stands on.
std::vector<Slab> model_slabs_of(const PrintObject &object)
{
    std::vector<Slab> slabs;
    slabs.reserve(object.layers().size());
    for (const Layer *layer : object.layers()) {
        Slab slab;
        slab.print_z  = layer->print_z;
        slab.bottom_z = layer->print_z - layer->height;
        slab.polygons = layer->lslices;
        slabs.emplace_back(std::move(slab));
    }
    return slabs;
}

std::vector<std::vector<bool>> floating_pieces(const std::vector<Slab> &support, const std::vector<Slab> &model,
                                               bool on_build_plate_only, double bottom_gap_mm)
{
    const Components        components = build_components(support, 0.);
    const std::vector<char> rooted     = model.empty() ? components.bed_rooted :
        rooted_components(components, model, build_components(model, model.front().bottom_z), on_build_plate_only,
                          std::max(0., bottom_gap_mm));
    std::vector<std::vector<bool>> floating(support.size());
    for (size_t s = 0; s < support.size(); ++ s) {
        floating[s].assign(support[s].polygons.size(), false);
        for (size_t k = components.slab_range[s].first; k < components.slab_range[s].second; ++ k)
            floating[s][k - components.slab_range[s].first] = ! rooted[components.pieces[k].component];
    }
    return floating;
}

double cross_section_width_mm(const ExPolygon &section)
{
    // The convex hull, so a notch in the outline cannot report a branch as thinner than it is, and
    // rotating calipers over it: the width in one direction is how far the hull reaches from the
    // edge that faces it, and the minimum over the edges is the section's own width.
    const Polygon hull = Geometry::convex_hull(section.contour.points);
    if (hull.points.size() < 3)
        return 0.;
    double       narrowest = std::numeric_limits<double>::max();
    const size_t n         = hull.points.size();
    for (size_t i = 0; i < n; ++ i) {
        const Line edge(hull.points[i], hull.points[(i + 1) % n]);
        if (edge.a == edge.b)
            continue;
        double reach = 0.;
        for (const Point &point : hull.points)
            reach = std::max(reach, edge.perp_distance_to(point));
        narrowest = std::min(narrowest, reach);
    }
    return narrowest == std::numeric_limits<double>::max() ? 0. : narrowest * SCALING_FACTOR;
}

bool Report::has_reason(Reason reason) const
{
    return std::find(reasons.begin(), reasons.end(), reason) != reasons.end();
}

// Measured off the object's own sliced layers, which are the model whatever support was or was not
// emitted, so it is taken before anything about the emitted material is read. Built only here, on the
// analysis path: a slice that never asked for a report never pays for a field.
ModelSupportRisk::Field measure_model_risk(const PrintObject &object, double extrusion_width_mm)
{
    if (! (extrusion_width_mm > 0.))
        return ModelSupportRisk::Field{};
    std::vector<ModelSupportRisk::Slice> slices;
    slices.reserve(object.layers().size());
    for (const Layer *layer : object.layers()) {
        ModelSupportRisk::Slice slice;
        slice.bottom_z_mm = layer->bottom_z();
        slice.top_z_mm    = layer->print_z;
        slice.solids      = layer->lslices;
        slices.push_back(std::move(slice));
    }
    const Print *print = object.print();
    return ModelSupportRisk::build(slices, extrusion_width_mm,
                                   [print]() { return print != nullptr && print->canceled(); });
}

Report measure(const PrintObject &object, const MiniatureSupport::Problem &problem, const EmittedSupport &emitted,
               const ModelSupportRisk::Field &risk)
{
    Report report;
    report.key = CoverageKey::from_problem(problem);

    report.coverage.resize(problem.regions.size());
    for (size_t i = 0; i < problem.regions.size(); ++ i) {
        report.coverage[i].region_id = problem.regions[i].id;
        report.coverage[i].critical  = problem.regions[i].critical;
        report.coverage[i].printable = problem.regions[i].printable;
    }
    // Seeds are handed out in one dense order over the whole problem, so a seed's region is found by
    // its recorded region id rather than by where it happens to sit.
    for (const MiniatureSupport::ContactSeed &seed : problem.seeds) {
        if (seed.region_id < report.coverage.size())
            report.coverage[seed.region_id].anchor_ids.push_back(seed.id);
        if (seed.critical)
            report.critical_anchor_ids.push_back(seed.id);
        if (seed.pinned)
            report.pinned_anchor_ids.push_back(seed.id);
    }
    for (RegionCoverage &region : report.coverage)
        std::sort(region.anchor_ids.begin(), region.anchor_ids.end());
    std::sort(report.critical_anchor_ids.begin(), report.critical_anchor_ids.end());
    std::sort(report.pinned_anchor_ids.begin(), report.pinned_anchor_ids.end());

    if (problem.regions.empty())
        report.reasons.push_back(Reason::NoProblem);

    // The model's own weakness under each contact. It reads the object's slices alone, so it holds
    // whatever the generator emitted and is taken before any of that is looked at.
    measure_contact_risk(report, object, problem, risk);

    // The material metric, summed straight off the extrusions the generator emitted. SupportLayer
    // carries the support and interface paths alone, so the object's own perimeters and infill, its
    // brim, the print's skirt, the purge and the wipe tower are all outside it by construction, and
    // nothing here reads a G-code file or a statistic that only export fills in. The leading support
    // layers are the raft the object stands on: reported separately, and their sum is the metric.
    {
        size_t index = 0;
        for (const SupportLayer *layer : object.support_layers()) {
            (index < object.support_raft_layers() ? report.raft_volume_mm3 : report.support_volume_mm3) +=
                layer->support_fills.total_volume();
            ++ index;
        }
    }

    bool any_emitted = false;
    for (const EmittedLayer &layer : emitted.layers)
        if (layer.emitted_available) {
            any_emitted = true;
            break;
        }
    std::vector<uint64_t> region_of_seed(problem.seeds.size(), 0);
    std::vector<Point>    seed_position(problem.seeds.size());
    for (const MiniatureSupport::ContactSeed &seed : problem.seeds)
        if (seed.id < region_of_seed.size()) {
            region_of_seed[size_t(seed.id)] = seed.region_id;
            seed_position[size_t(seed.id)]  = seed.position;
        }

    // What each attributed area actually got printed on, taken once: the stability measurement and
    // the coverage measurement both read it, and the intersection is the expensive half of either.
    std::vector<std::vector<ExPolygons>> printed(emitted.layers.size());
    tbb::parallel_for(tbb::blocked_range<size_t>(0, emitted.layers.size()),
        [&](const tbb::blocked_range<size_t> &range) {
            for (size_t l = range.begin(); l < range.end(); ++ l) {
                const EmittedLayer &layer = emitted.layers[l];
                printed[l].resize(layer.areas.size());
                if (! layer.emitted_available || layer.emitted.empty())
                    continue;
                for (size_t a = 0; a < layer.areas.size(); ++ a) {
                    const AttributedArea &area = layer.areas[a];
                    // The planned gap between a tip and the model is drawn and never extruded: counting it
                    // would report material where none was laid.
                    if (area.virtual_gap || area.source_ids.empty())
                        continue;
                    // Only the emitted material that reaches this area's box: the prefilter drops the
                    // emitted vertices lying beyond one side of the box, so the clip carries tens of
                    // points instead of the layer's thousands. The same shape as intersection() over a
                    // clipped clip in ClipperUtils.cpp. Clipping against a reduced set moves what
                    // ClipperLib rounds, so the result differs from clipping against the whole layer by
                    // up to one scaled unit on a vertex. Measured on the plate 3 fixture over 61534
                    // areas: 7.4e-7 mm2 of area at worst, and 17 areas where two pieces meeting at a
                    // one-unit neck came back joined or split, one polygon more or fewer. The fixture
                    // gates that pin the stability and coverage values hold under that change.
                    printed[l][a] = intersection_ex(ExPolygons{ area.area },
                        ClipperUtils::clip_clipper_polygons_with_subject_bbox(layer.emitted,
                            get_extents(area.area).inflated(SCALED_EPSILON)));
                }
            }
        });

    // What printed support material joins up, taken once: stability reads it to find what stands on
    // nothing and damage reads it to find what would come off together.
    std::vector<size_t>     slab_of_layer;
    const std::vector<Slab> support_slabs = printed_support_slabs(object, emitted, slab_of_layer);
    const Components        support       = build_components(support_slabs, 0.);

    // Stability reads the emitted material and the object alone, so it is measured whatever the
    // prepared problem asked for, and it is what it is even where coverage stays unresolved.
    if (any_emitted)
        measure_stability(report, object, emitted, printed, region_of_seed, support_slabs, slab_of_layer, support);
    if (! report.stability.available)
        report.reasons.push_back(Reason::StabilityUnavailable);

    // Removal damage reads the same material and the contact estimates already taken off the model.
    measure_damage(report, object, problem, emitted, printed, region_of_seed, support_slabs, slab_of_layer, support);
    if (! report.damage.available)
        report.reasons.push_back(Reason::DamageUnavailable);

    if (problem.regions.empty() || ! any_emitted) {
        // Nothing to intersect against. Coverage is unresolved rather than zero: an unmeasured domain
        // may not read as a measured absence.
        report.coverage_available = false;
        report.status             = Report::Status::UnresolvedCoverage;
        report.reasons.push_back(Reason::EmittedMaterialMissing);
        return report;
    }

    // Per region: every support layer whose printed material carries one of that region's own sources
    // and sits in that region's planned top-gap interval. Nothing else can anchor it, however close it
    // happens to lie. The interval is one slab deep, so two contacts of one region whose tips print a
    // layer apart inside it both anchor it: the tip clipped against the model a layer higher than its
    // neighbour's is still the tip the plan asked for.
    const size_t            region_count = problem.regions.size();
    std::vector<ExPolygons> region_material(region_count);
    std::vector<double>     region_tip_z(region_count, 0.);
    std::vector<char>       region_hit(region_count, 0);
    std::vector<std::set<uint64_t>> region_reached(region_count);

    for (size_t l = 0; l < emitted.layers.size(); ++ l) {
        const EmittedLayer &layer = emitted.layers[l];
        if (! layer.emitted_available || layer.emitted.empty())
            continue;
        for (size_t a = 0; a < layer.areas.size(); ++ a) {
            const AttributedArea &area          = layer.areas[a];
            const ExPolygons     &printed_here  = printed[l][a];
            if (printed_here.empty())
                continue;
            std::set<uint64_t> regions_here;
            for (uint64_t seed_id : area.source_ids)
                if (seed_id < region_of_seed.size())
                    regions_here.insert(region_of_seed[size_t(seed_id)]);
            ++ report.provenance.emitted_areas;
            if (regions_here.size() > 1)
                ++ report.provenance.multi_source_areas;
            report.provenance.max_sources_in_area = std::max(report.provenance.max_sources_in_area, regions_here.size());

            for (uint64_t region_id : regions_here) {
                if (region_id >= region_count)
                    continue;
                const MiniatureSupport::RequiredRegion &region = problem.regions[size_t(region_id)];
                const double tip_top    = region.contact_z_mm - emitted.top_gap_mm;
                const double tip_bottom = tip_top - emitted.max_layer_height_mm;
                const double tol        = std::max(1e-9, 1e-6 * std::abs(tip_top));
                if (layer.print_z > tip_top + tol || layer.print_z < tip_bottom - tol)
                    continue;
                if (! region_hit[size_t(region_id)] || layer.print_z > region_tip_z[size_t(region_id)])
                    region_tip_z[size_t(region_id)] = layer.print_z;
                region_hit[size_t(region_id)] = 1;
                append(region_material[size_t(region_id)], printed_here);
                for (uint64_t seed_id : area.source_ids)
                    if (seed_id < region_of_seed.size() && region_of_seed[size_t(seed_id)] == region_id)
                        region_reached[size_t(region_id)].insert(seed_id);
            }
        }
    }

    // The union of every qualifying intersection, taken once: material two regions share is one piece
    // of printed support, and counting it twice would make a shared branch look like two.
    // Pass one, the material each region's own contacts printed. The region's frozen witness lattice,
    // the one selection ran against, is sized here for every region, anchored or not: a contact of a
    // region one band down writes into a higher region's bitmap under the rule below, and that bitmap
    // has to exist before it can be written into. Everything else stays behind the anchored gate.
    ExPolygons all_material;
    for (size_t i = 0; i < region_count; ++ i) {
        RegionCoverage &region_report = report.coverage[i];
        const auto lattice_ptr = MiniatureSupport::region_witnesses(problem.regions[i], problem.extrusion_width_mm);
        region_report.covered.assign(lattice_ptr->size(), false);
        region_report.anchored        = region_hit[i] != 0 && ! region_material[i].empty();
        if (! region_report.anchored)
            continue;
        ++ report.provenance.traced_regions;
        const ExPolygons material     = union_ex(region_material[i]);
        region_report.tip_print_z     = region_tip_z[i];
        region_report.covered_mm2     = area(material) * SCALING_FACTOR * SCALING_FACTOR;
        append(all_material, material);
    }

    // Pass two, the cells those contacts stand behind, through the one rule selection thins by: the
    // region's own cells within its legal reach, and the cells of the bands above it in its overhang
    // component within the problem's contact distance in 3-D. A cell counts as covered only when the
    // whole of it lies behind a contact whose material was actually printed, so a cell the measurement
    // cannot account for stays uncovered rather than being sampled away at its centre.
    const MiniatureSupport::CoverageRule rule(problem);
    std::vector<char>                    carried(region_count, 0);
    for (size_t i = 0; i < region_count; ++ i)
        for (uint64_t seed_id : region_reached[i]) {
            if (seed_id >= seed_position.size())
                continue;
            for (const MiniatureSupport::CoveredCell &cell : rule.cells(size_t(seed_id), seed_position[size_t(seed_id)])) {
                if (cell.region >= region_count)
                    continue;
                RegionCoverage &covered_report = report.coverage[cell.region];
                if (cell.cell >= covered_report.covered.size())
                    continue;
                covered_report.covered[cell.cell] = true;
                if (cell.region != i)
                    carried[cell.region] = 1;
            }
        }

    // Pass three. `measure_stability` has already recorded whether each region's own sources reached
    // printed material; a region whose cells a reached contact of its component carries has a path
    // through that contact too, so the two are ORed rather than one overwriting the other.
    for (size_t i = 0; i < region_count; ++ i)
        report.coverage[i].emitted_path = report.coverage[i].emitted_path || carried[i] != 0;

    report.measured_contact_mm2 = area(union_ex(all_material)) * SCALING_FACTOR * SCALING_FACTOR;

    for (const MiniatureSupport::ContactSeed &seed : problem.seeds)
        if (seed.region_id >= region_count || region_reached[size_t(seed.region_id)].count(seed.id) == 0)
            report.missing_anchor_ids.push_back(seed.id);
    std::sort(report.missing_anchor_ids.begin(), report.missing_anchor_ids.end());
    if (! report.missing_anchor_ids.empty())
        report.reasons.push_back(Reason::MissingAnchor);

    report.coverage_available = true;
    // Complete when every domain the measurement was asked for came back available - coverage
    // against the problem's regions, the emitted material's stability, and the removal damage - and
    // Unknown as soon as one of them did not: no caller may treat an unmeasured domain as a passed
    // one. Coverage that could not be resolved at all leaves earlier, as UnresolvedCoverage.
    report.status = report.coverage_available && report.stability.available && report.damage.available ?
                        Report::Status::Complete : Report::Status::Unknown;
    return report;
}

Report measure(const PrintObject &object, const MiniatureSupport::Problem &problem, const EmittedSupport &emitted)
{
    return measure(object, problem, emitted, measure_model_risk(object, problem.extrusion_width_mm));
}

Report refresh_bed_footprint(const Report &report, const PrintObject &object, const EmittedSupport &emitted,
                             const Polygons &adhesion)
{
    Report out = report;
    if (! out.stability.available)
        return out;  // ground added to a measurement that was never taken is still no measurement

    const std::vector<Slab> model_slabs = model_slabs_of(object);
    std::vector<size_t>     slab_of_layer;
    const std::vector<Slab> support_slabs = printed_support_slabs(object, emitted, slab_of_layer);
    const ExPolygons        connected     = bed_ground(model_slabs, support_slabs, adhesion);

    Point  centroid;
    double margin = 0.;
    if (model_slabs.empty() || ! volume_centroid(object, centroid) ||
        ! bed_margin(connected, centroid, out.stability.object_height_mm, margin)) {
        // The refreshed ground is what the object actually stands on. Where it cannot be read, the
        // domain is unknown rather than left holding the number taken before the brim existed.
        out.stability = Stability();
        if (! out.has_reason(Reason::StabilityUnavailable))
            out.reasons.push_back(Reason::StabilityUnavailable);
        out.status = Report::Status::Unknown;
        return out;
    }
    out.stability.bed_footprint_mm2 = area(connected) * SCALING_FACTOR * SCALING_FACTOR;
    out.stability.min_bed_margin    = margin;
    return out;
}

double comparison_epsilon(double a, double b)
{
    return std::max(1e-9, 1e-6 * std::max(std::abs(a), std::abs(b)));
}

DamageComparison compare_damage(const Damage &was, const Damage &is)
{
    if (is.unknown_contacts != was.unknown_contacts)
        return { is.unknown_contacts < was.unknown_contacts ? -1 : 1, DamageField::UnknownContacts };
    if (is.inaccessible_groups != was.inaccessible_groups)
        return { is.inaccessible_groups < was.inaccessible_groups ? -1 : 1, DamageField::InaccessibleGroups };
    if (std::abs(is.max_group_risk - was.max_group_risk) >
        comparison_epsilon(was.max_group_risk, is.max_group_risk))
        return { is.max_group_risk < was.max_group_risk ? -1 : 1, DamageField::MaxGroupRisk };
    if (std::abs(is.total_group_risk - was.total_group_risk) >
        comparison_epsilon(was.total_group_risk, is.total_group_risk))
        return { is.total_group_risk < was.total_group_risk ? -1 : 1, DamageField::TotalGroupRisk };
    return { 0, DamageField::None };
}

bool stability_no_worse(const Report &reference, const Report &candidate)
{
    const Stability &was = reference.stability, &is = candidate.stability;
    // An unmeasured domain establishes nothing. Non-regression has to be shown, not assumed from a
    // field that was never filled in.
    if (! was.available || ! is.available)
        return false;
    // Counts are counts: they compare exactly, with no tolerance to hide one behind.
    if (is.unsupported_paths > was.unsupported_paths || is.unrooted_groups > was.unrooted_groups)
        return false;
    if (is.min_bed_margin < was.min_bed_margin - comparison_epsilon(was.min_bed_margin, is.min_bed_margin))
        return false;
    if (is.max_slenderness > was.max_slenderness + comparison_epsilon(was.max_slenderness, is.max_slenderness))
        return false;
    // Equal keys mean one prepared problem, whose groups answer one for one: a group may not leave
    // the comparison by losing the provenance that put it in. Different keys are different poses,
    // whose source ids name different regions, so nothing is matched by id across them and only the
    // aggregate measures above are compared.
    if (reference.key == candidate.key)
        for (size_t i = 0; i < reference.coverage.size() && i < candidate.coverage.size(); ++ i)
            if (reference.coverage[i].emitted_path && ! candidate.coverage[i].emitted_path)
                return false;
    return true;
}

bool stability_admissible(const Stability &stability)
{
    return stability.available && stability.unsupported_paths == 0 && stability.unrooted_groups == 0 &&
           stability.min_bed_margin >= 0.;
}

// The declaration in SupportAnalysis.hpp states what this predicate asks and why it asks no more
// than that. It reads `RegionCoverage::printable`, which `measure` copies off the prepared problem's
// own `RequiredRegion::printable`, so the width clause is decided once, where the region was filed.
bool support_unresolved(const Report &report)
{
    for (const RegionCoverage &region : report.coverage)
        if (region.printable && region.critical && ! region.emitted_path)
            return true;
    return report.stability.unrooted_groups > 0;
}

} // namespace SupportAnalysis
} // namespace Slic3r
