#include "SupportAnalysis.hpp"

#include "RemovalAccess.hpp"
#include "SupportComponents.hpp"
#include "../AABBMesh.hpp"
#include "../ClipperUtils.hpp"
#include "../Geometry/ConvexHull.hpp"
#include "../Layer.hpp"
#include "../Print.hpp"

#include <boost/log/trivial.hpp>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <sstream>

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

// The removal groups of one emitted pass (`Damage` states what they are), and what taking one of them
// off the model would put at risk. A group is what the extrusions actually covered, so the roof gap
// the router draws over a tip and never extrudes neither joins two groups nor divides one, and every
// contact the router carried into a component's printed material belongs to that component, merged
// branches and shared roots included. One printed area is weighed once whatever it carries, at the
// worst estimate of the contacts sharing it.
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

    // What the access probe answered, over every group it probed: a count per outcome, and per reason
    // where it could not answer. Logged once below, because 1400 contacts nothing can answer for and
    // 1400 contacts a tool can reach are the same number in the damage tuple.
    size_t probed_clear = 0, probed_blocked = 0, probed_unknown = 0;
    std::map<std::string, size_t> access_missing;
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
            lever = std::max(lever, run[piece] / std::max(sample_of_seed[size_t(seed_id)]->neck_width_mm, width));
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
            const MiniatureSupport::ContactSeed &seed = problem.seeds[size_t(seed_id)];
            if (seed.region_id >= problem.regions.size())
                continue;
            const Vec3d where(unscale<double>(seed.position.x()), unscale<double>(seed.position.y()),
                              problem.regions[size_t(seed.region_id)].contact_z_mm);
            const RemovalAccess::Access access =
                RemovalAccess::assess_access(mesh, others, slab_tops, where, 0.5 * width);
            if (access.status == RemovalAccess::Access::Status::Clear) {
                ++ probed_clear;
                reached = true;
                break;
            }
            if (access.status == RemovalAccess::Access::Status::Unknown) {
                ++ unknown;
                ++ probed_unknown;
                ++ access_missing[RemovalAccess::missing_name(access.missing)];
            } else {
                ++ probed_blocked;
            }
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
    {
        std::ostringstream reasons;
        for (const std::pair<const std::string, size_t> &entry : access_missing)
            reasons << " " << entry.first << " " << entry.second;
        BOOST_LOG_TRIVIAL(info) << "support analysis removal access: " << probed_clear << " clear, " << probed_blocked
                                << " blocked, " << probed_unknown << " unknown of " << problem.seeds.size()
                                << " seeds in " << support.count << " groups, unknown by reason:" << reasons.str();
    }
    damage.available = true;
}

// What the model itself says about each contact the generator placed (ModelSupportRisk::Sample).
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
    size_t known = 0, below_width = 0, unknown = 0;
    std::map<std::string, size_t> sample_missing;
    for (const ContactRisk &entry : report.contact_risk)
        switch (entry.sample.status) {
        case ModelSupportRisk::Sample::Status::Known:               ++ known; break;
        case ModelSupportRisk::Sample::Status::BelowPrintableWidth: ++ below_width; break;
        case ModelSupportRisk::Sample::Status::Unknown:
            ++ unknown;
            ++ report.damage.unknown_contacts;
            ++ sample_missing[ModelSupportRisk::missing_name(entry.sample.missing)];
            break;
        }
    {
        std::ostringstream reasons;
        for (const std::pair<const std::string, size_t> &entry : sample_missing)
            reasons << " " << entry.first << " " << entry.second;
        BOOST_LOG_TRIVIAL(info) << "support analysis contact risk: " << known << " known, " << below_width
                                << " below printable width, " << unknown << " unknown of " << problem.seeds.size()
                                << " seeds, unknown by reason:" << reasons.str();
    }
    std::sort(report.contact_risk.begin(), report.contact_risk.end(),
              [](const ContactRisk &a, const ContactRisk &b) { return a.seed_id < b.seed_id; });
    report.contact_risk_available = true;
}

} // namespace

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
                    // Only the emitted material that reaches this area's box, through
                    // ClipperUtils::clip_clipper_polygons_with_subject_bbox, so the clip carries tens of
                    // points instead of the layer's thousands. Clipping against a reduced set moves what
                    // ClipperLib rounds, so a vertex may differ from the whole-layer clip by up to one
                    // scaled unit (AGENTS.md, Testing, keeps the measurement); the fixture gates that pin
                    // the stability and coverage values hold under that.
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

    // Stability reads the emitted material and the object alone, so it is measured wherever anything
    // was emitted, whatever the prepared problem asked for, and it is what it is even where coverage
    // stays unresolved. With no required region it is measured even when nothing was emitted: it then
    // reads the object and whatever raft it stands on.
    if (any_emitted || problem.regions.empty())
        measure_stability(report, object, emitted, printed, region_of_seed, support_slabs, slab_of_layer, support);
    if (! report.stability.available)
        report.reasons.push_back(Reason::StabilityUnavailable);

    // Removal damage reads the same material and the contact estimates already taken off the model.
    measure_damage(report, object, problem, emitted, printed, region_of_seed, support_slabs, slab_of_layer, support);
    if (! report.damage.available)
        report.reasons.push_back(Reason::DamageUnavailable);

    // A problem with no required region has nothing to cover, so its coverage is complete by
    // construction: the passes below walk zero regions and zero seeds, and the final rule decides the
    // status. Required regions with nothing emitted have nothing to intersect against, and their
    // coverage is unresolved rather than zero: an unmeasured domain may not read as a measured absence.
    if (! problem.regions.empty() && ! any_emitted) {
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

    // Pass two, the cells those contacts stand behind, through `CoverageRule`, the one rule selection
    // thins by. A cell counts as covered only when the whole of it lies behind a contact whose material
    // was actually printed, so a cell the measurement cannot account for stays uncovered.
    const MiniatureSupport::CoverageRule rule(problem);
    std::vector<char>                    carried(region_count, 0);
    for (size_t i = 0; i < region_count; ++ i)
        for (uint64_t seed_id : region_reached[i])
            for (const MiniatureSupport::CoveredCell &cell : rule.cells(size_t(seed_id), seed_position[size_t(seed_id)])) {
                report.coverage[cell.region].covered[cell.cell] = true;
                if (cell.region != i)
                    carried[cell.region] = 1;
            }

    // Pass three. Where stability was measured, `measure_stability` recorded whether each region's own
    // sources reached printed material; a region whose cells a reached contact of its component
    // carries has a path through that contact too, so the two are ORed rather than one overwriting
    // the other.
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
    report.status = report.stability.available && report.damage.available ? Report::Status::Complete : Report::Status::Unknown;
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

bool support_unresolved(const Report &report)
{
    for (const RegionCoverage &region : report.coverage)
        if (region.printable && region.critical && ! region.emitted_path)
            return true;
    return report.stability.unrooted_groups > 0;
}

} // namespace SupportAnalysis
} // namespace Slic3r
