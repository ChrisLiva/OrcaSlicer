#include "ModelSupportRisk.hpp"

#include "../ClipperUtils.hpp"
#include "../AABBMesh.hpp"
#include "../BoundingBox.hpp"
#include "../Geometry/MedialAxis.hpp"
#include "../libslic3r.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <queue>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

namespace Slic3r {
namespace ModelSupportRisk {

namespace {

// Twice the query's own clearance: no circle inside the solid centred on the query is wider, so no
// medial width the query reads may exceed it. Contours and holes alike, in scaled units.
double clearance_scaled(const ExPolygon &solid, const Point &query)
{
    double best = std::numeric_limits<double>::max();
    const auto walk = [&best, &query](const Polygon &polygon) {
        const Points &pts = polygon.points;
        for (size_t i = 0; i < pts.size(); ++ i)
            best = std::min(best, Line(pts[i], pts[(i + 1) % pts.size()]).distance_to(query));
    };
    walk(solid.contour);
    for (const Polygon &hole : solid.holes)
        walk(hole);
    return best;
}

// The width over an already-built medial axis: project the query onto every segment, keep only the
// projections the solid contains the sight line to, interpolate that segment's own width pair, and
// take the nearest such projection, with segment order as the tie-break.
bool width_from_medial(const ExPolygon &solid, const ThickPolylines &medial, const Point &query, double *width_mm)
{
    bool   found   = false;
    double best_d2 = std::numeric_limits<double>::max();
    double best_w  = 0.;
    for (const ThickPolyline &polyline : medial) {
        const Points &pts = polyline.points;
        if (pts.size() < 2)
            continue;
        // Two entries per segment is what ThickPolyline promises; anything else is not a width vector
        // this can read, and reading it as one would invent a number.
        if (polyline.width.size() != (pts.size() - 1) * 2)
            return false;
        for (size_t i = 0; i + 1 < pts.size(); ++ i) {
            const Vec2d  a   = pts[i].cast<double>();
            const Vec2d  d   = pts[i + 1].cast<double>() - a;
            const double len2 = d.squaredNorm();
            double       t    = 0.;
            if (len2 > 0.)
                t = std::clamp((query.cast<double>() - a).dot(d) / len2, 0., 1.);
            const Vec2d  proj_d = a + t * d;
            const Point  proj(coord_t(std::lround(proj_d.x())), coord_t(std::lround(proj_d.y())));
            // The sight line has to stay inside this very solid: a projection reached only by
            // crossing a hole or leaving the island describes some other piece of the model.
            if (proj != query && ! solid.contains(Line(query, proj)))
                continue;
            const double w0 = polyline.width[2 * i], w1 = polyline.width[2 * i + 1];
            if (! std::isfinite(w0) || ! std::isfinite(w1))
                return false;
            const double d2 = (proj_d - query.cast<double>()).squaredNorm();
            if (! found || d2 < best_d2) {
                found   = true;
                best_d2 = d2;
                best_w  = w0 + t * (w1 - w0);
            }
        }
    }
    if (! found)
        return false;
    const double capped = std::min(best_w, 2. * clearance_scaled(solid, query)) * SCALING_FACTOR;
    if (! std::isfinite(capped) || capped < 0.)
        return false;
    *width_mm = capped;
    return true;
}

// A polygon with no interior to measure: nothing about it is a width.
bool degenerate(const ExPolygon &solid)
{
    return solid.contour.points.size() < 3 || ! (std::abs(solid.area()) > 0.);
}

// The medial axis of one complete solid, taken straight off Geometry::MedialAxis so the ExPolygon
// wrapper's endpoint pruning cannot erase a thin weapon tip. Empty for a degenerate polygon, and
// empty as well for a solid that is narrow nowhere: MedialAxis::validate_edge drops every skeleton
// edge whose two boundary walls are more than PI/8 off facing unless the edge is shorter than
// min_width, and the min_width of zero this is built with makes that test reject them all. So an
// empty skeleton here says "no narrow spine", not "no geometry".
ThickPolylines medial_axis_of(const ExPolygon &solid)
{
    ThickPolylines lines;
    if (degenerate(solid))
        return lines;
    const BoundingBox bbox = get_extents(solid);
    const double      diagonal = (bbox.max - bbox.min).cast<double>().norm();
    if (! std::isfinite(diagonal) || diagonal <= 0.)
        return lines;
    Geometry::MedialAxis(0., diagonal + SCALED_EPSILON, solid).build(&lines);
    return lines;
}

// The width measurement over a skeleton already in hand, so a query costs no second Voronoi.
bool measured_width(const ExPolygon &solid, const ThickPolylines &medial, const Point &query, double *width_mm)
{
    if (width_mm == nullptr || degenerate(solid))
        return false;
    if (! medial.empty())
        return width_from_medial(solid, medial, query, width_mm);
    // A solid with no narrow spine at all: the widest circle that fits inside it around the query is
    // its width there, which is the same cap a medial reading is held to anyway. A query the solid
    // does not hold is still unmeasured.
    if (! solid.contains(query))
        return false;
    const double clearance = 2. * clearance_scaled(solid, query) * SCALING_FACTOR;
    if (! std::isfinite(clearance) || clearance <= 0.)
        return false;
    *width_mm = clearance;
    return true;
}

} // namespace

bool local_width(const ExPolygon &solid, const Point &query, double *width_mm)
{
    if (width_mm == nullptr || degenerate(solid))
        return false;
    return measured_width(solid, medial_axis_of(solid), query, width_mm);
}

namespace {

// A point the solid itself holds: its centroid where that lies inside, otherwise the first vertex of
// its medial axis, which lies inside by construction.
bool interior_point(const ExPolygon &solid, Point &out)
{
    if (degenerate(solid))
        return false;
    const Point centroid = solid.contour.centroid();
    if (solid.contains(centroid)) {
        out = centroid;
        return true;
    }
    for (const ThickPolyline &polyline : medial_axis_of(solid))
        if (! polyline.points.empty()) {
            out = polyline.points.front();
            return true;
        }
    return false;
}

// Twice the largest circle the connection holds: how thick a joint between two layers is, read off
// the same medial axis the widths come from, and off the clearance at an interior point where the
// skeleton filter left nothing to read.
double joint_width_mm(const ExPolygon &joint)
{
    double widest = 0.;
    for (const ThickPolyline &polyline : medial_axis_of(joint))
        for (double w : polyline.width)
            if (std::isfinite(w))
                widest = std::max(widest, w);
    if (widest > 0.)
        return widest * SCALING_FACTOR;
    Point core;
    if (! interior_point(joint, core))
        return 0.;
    return 2. * clearance_scaled(joint, core) * SCALING_FACTOR;
}

// Where two islands are joined: a point the overlap itself holds, so the connection lands where the
// material actually overlaps.
Point inside_point(const ExPolygon &joint)
{
    Point core;
    return interior_point(joint, core) ? core : joint.contour.centroid();
}

// Disjoint sets over node indices, so the medial branches of one island can be checked for being one
// connected piece and joined where the skeleton came back in pieces.
struct DisjointSets
{
    std::vector<size_t> parent;

    explicit DisjointSets(size_t n) : parent(n) { std::iota(parent.begin(), parent.end(), size_t(0)); }
    size_t find(size_t i) { while (parent[i] != i) { parent[i] = parent[parent[i]]; i = parent[i]; } return i; }
    bool   join(size_t a, size_t b) { a = find(a); b = find(b); if (a == b) return false; parent[b] = a; return true; }
};

// An island's medial skeleton can come back in pieces even though the solid is one connected body.
// Joining the pieces at their closest node pair keeps the island one piece of the graph, at the cost
// of a straight edge no wider than the two samples it links. Skipped for a skeleton too large to
// pair off, which leaves the island's far pieces unresolved rather than slow.
constexpr size_t max_joinable_island_nodes = 4000;

} // namespace

Field build(const std::vector<Slice> &slices, double extrusion_width_mm, const std::function<bool()> &stop)
{
    Field field;
    field.extrusion_width_mm = extrusion_width_mm;
    const auto stopped = [&stop]() { return bool(stop) && stop(); };

    for (const Slice &slice : slices)
        if (! std::isfinite(slice.bottom_z_mm) || ! std::isfinite(slice.top_z_mm) || slice.top_z_mm < slice.bottom_z_mm) {
            field.status = Field::Status::Invalid;
            return field;
        }
    if (stopped()) {
        field.status = Field::Status::Canceled;
        return field;
    }

    field.slices = slices;
    field.island_base.resize(slices.size(), 0);
    size_t island_count = 0;
    for (size_t l = 0; l < slices.size(); ++ l) {
        field.island_base[l] = island_count;
        island_count        += slices[l].solids.size();
    }
    field.island_nodes.resize(island_count);
    field.island_medial.resize(island_count);

    // medial_axis_of is pure and each island owns one slot, so the whole field's medial axes come out
    // of one parallel pass; the welding loop below stays serial because it shares field.nodes/edges.
    {
        std::vector<std::pair<size_t, size_t>> islands;
        islands.reserve(island_count);
        for (size_t l = 0; l < slices.size(); ++ l)
            for (size_t i = 0; i < slices[l].solids.size(); ++ i)
                islands.emplace_back(l, i);
        if (stopped()) {
            field.status = Field::Status::Canceled;
            return field;
        }
        tbb::parallel_for(tbb::blocked_range<size_t>(0, island_count), [&](const tbb::blocked_range<size_t> &range) {
            for (size_t flat = range.begin(); flat < range.end(); ++ flat)
                field.island_medial[flat] = medial_axis_of(slices[islands[flat].first].solids[islands[flat].second]);
        });
    }

    const auto add_edge = [&field](size_t a, size_t b, double width_mm, double length_mm) {
        if (a == b)
            return;
        const size_t index = field.edges.size();
        field.edges.push_back(Field::Edge{ a, b, width_mm, length_mm });
        field.adjacency[a].push_back(index);
        field.adjacency[b].push_back(index);
    };

    // The ground is the object's first slab, on the plate or on the raft the plate carries: an island
    // whose slab starts there stands on it, and every medial sample of it is a place a path may end.
    const double ground_z_mm      = slices.empty() ? 0. : slices.front().bottom_z_mm;
    const double ground_tolerance = 1e-6;

    for (size_t l = 0; l < slices.size(); ++ l) {
        if (stopped()) {
            field.status = Field::Status::Canceled;
            return field;
        }
        const bool root = slices[l].bottom_z_mm <= ground_z_mm + ground_tolerance;
        for (size_t i = 0; i < slices[l].solids.size(); ++ i) {
            const ExPolygon &solid = slices[l].solids[i];
            const size_t     flat  = field.island_base[l] + i;

            // One node per medial vertex, welded by position so branches meeting at a junction share
            // it and the island's own constrictions become edges of the graph.
            std::map<std::pair<coord_t, coord_t>, size_t> welded;
            const size_t                                  first_edge = field.edges.size();
            for (const ThickPolyline &polyline : field.island_medial[flat]) {
                const Points &pts = polyline.points;
                if (pts.size() < 2 || polyline.width.size() != (pts.size() - 1) * 2)
                    continue;
                std::vector<size_t> mapped(pts.size(), 0);
                for (size_t j = 0; j < pts.size(); ++ j) {
                    // A vertex sits between two segments and carries the narrower of the two width
                    // entries that meet on it.
                    double here = j == 0 ? polyline.width[0] :
                                  j + 1 == pts.size() ? polyline.width[2 * (pts.size() - 2) + 1] :
                                  std::min(polyline.width[2 * (j - 1) + 1], polyline.width[2 * j]);
                    if (! std::isfinite(here))
                        here = 0.;
                    const auto key = std::make_pair(pts[j].x(), pts[j].y());
                    auto       it  = welded.find(key);
                    if (it == welded.end()) {
                        it = welded.emplace(key, field.nodes.size()).first;
                        field.nodes.push_back(Field::Node{ l, i, pts[j], here * SCALING_FACTOR, root });
                        field.adjacency.emplace_back();
                        field.island_nodes[flat].push_back(it->second);
                    } else
                        field.nodes[it->second].width_mm = std::min(field.nodes[it->second].width_mm, here * SCALING_FACTOR);
                    mapped[j] = it->second;
                }
                // The samples carry the island's own widths, so a run between two of them constrains
                // nothing of its own: a segment that tapers is narrow at its narrow end, not along
                // all of it.
                for (size_t j = 0; j + 1 < pts.size(); ++ j)
                    add_edge(mapped[j], mapped[j + 1], std::numeric_limits<double>::infinity(),
                             (pts[j + 1] - pts[j]).cast<double>().norm() * SCALING_FACTOR);
            }

            // A solid narrow nowhere has no spine for the skeleton to report, and it is still part of
            // the model a path runs through. One sample at an interior point, as wide as the circle
            // that fits there, keeps it in the graph at its own thickness.
            Point core;
            if (field.island_nodes[flat].empty() && interior_point(solid, core)) {
                const double width = 2. * clearance_scaled(solid, core) * SCALING_FACTOR;
                if (std::isfinite(width) && width > 0.) {
                    field.island_nodes[flat].push_back(field.nodes.size());
                    field.nodes.push_back(Field::Node{ l, i, core, width, root });
                    field.adjacency.emplace_back();
                }
            }

            // One solid is one connected body, so its samples have to be one connected piece of the
            // graph whatever the skeleton came back as.
            const std::vector<size_t> &island = field.island_nodes[flat];
            if (island.size() < 2 || island.size() > max_joinable_island_nodes)
                continue;
            DisjointSets sets(field.nodes.size());
            for (size_t e = first_edge; e < field.edges.size(); ++ e)
                sets.join(field.edges[e].a, field.edges[e].b);
            const size_t anchor = island.front();
            for (const size_t stray : island) {
                if (sets.find(stray) == sets.find(anchor))
                    continue;
                // The closest pair between what is already joined and the piece this node belongs to.
                size_t best_a = anchor, best_b = stray;
                double best_d = std::numeric_limits<double>::max();
                for (const size_t here : island) {
                    if (sets.find(here) != sets.find(anchor))
                        continue;
                    for (const size_t there : island) {
                        if (sets.find(there) != sets.find(stray))
                            continue;
                        const double d = (field.nodes[there].position - field.nodes[here].position).cast<double>().norm();
                        if (d < best_d) {
                            best_d = d;
                            best_a = here;
                            best_b = there;
                        }
                    }
                }
                add_edge(best_a, best_b, std::numeric_limits<double>::infinity(), best_d * SCALING_FACTOR);
                sets.join(anchor, stray);
            }
        }
    }

    // Adjacent layers, both directions: a weapon hanging under its hand reaches the object's first
    // slab by climbing to the hand first. Only a real overlap connects them, never a bounding box that
    // happens to.
    for (size_t l = 0; l + 1 < slices.size(); ++ l) {
        if (stopped()) {
            field.status = Field::Status::Canceled;
            return field;
        }
        const double dz = std::abs((slices[l + 1].bottom_z_mm + slices[l + 1].top_z_mm) * 0.5 -
                                   (slices[l].bottom_z_mm + slices[l].top_z_mm) * 0.5);
        for (size_t i = 0; i < slices[l].solids.size(); ++ i) {
            const BoundingBox below = get_extents(slices[l].solids[i]);
            for (size_t j = 0; j < slices[l + 1].solids.size(); ++ j) {
                if (! below.overlap(get_extents(slices[l + 1].solids[j])))
                    continue;
                const ExPolygons overlap = intersection_ex(ExPolygons{ slices[l].solids[i] },
                                                           ExPolygons{ slices[l + 1].solids[j] });
                const std::vector<size_t> &lower = field.island_nodes[field.island_base[l] + i];
                const std::vector<size_t> &upper = field.island_nodes[field.island_base[l + 1] + j];
                if (lower.empty() || upper.empty())
                    continue;
                for (const ExPolygon &piece : overlap) {
                    if (! (std::abs(piece.area()) > 0.))
                        continue;
                    const Point  where = inside_point(piece);
                    const double width = joint_width_mm(piece);
                    const auto   nearest = [&field, &where](const std::vector<size_t> &candidates) {
                        size_t best   = candidates.front();
                        double best_d = std::numeric_limits<double>::max();
                        for (const size_t node : candidates) {
                            const double d = (field.nodes[node].position - where).cast<double>().squaredNorm();
                            if (d < best_d) {
                                best_d = d;
                                best   = node;
                            }
                        }
                        return best;
                    };
                    const size_t a = nearest(lower), b = nearest(upper);
                    const double dxy = (field.nodes[b].position - field.nodes[a].position).cast<double>().norm() * SCALING_FACTOR;
                    add_edge(a, b, width, std::sqrt(dz * dz + dxy * dxy));
                }
            }
        }
    }

    field.status = Field::Status::Complete;
    return field;
}

Sample sample(const Field &field, size_t layer, const Point &query)
{
    Sample result;
    if (field.status != Field::Status::Complete || layer >= field.slices.size())
        return result;
    const Slice &slice  = field.slices[layer];
    size_t       island = slice.solids.size();
    for (size_t i = 0; i < slice.solids.size(); ++ i)
        if (slice.solids[i].contains(query)) {
            island = i;
            break;
        }
    if (island == slice.solids.size())
        return result;

    const size_t flat = field.island_base[layer] + island;
    double       local = 0.;
    if (! measured_width(slice.solids[island], field.island_medial[flat], query, &local))
        return result;
    const std::vector<size_t> &samples = field.island_nodes[flat];
    if (samples.empty())
        return result;

    // The query joins the graph at its island's nearest medial sample. The connecting run is a
    // candidate bottleneck like any other, so a query out on a thin tip is not credited with the
    // body's width.
    size_t attach   = samples.front();
    double attach_d = std::numeric_limits<double>::max();
    for (const size_t node : samples) {
        const double d = (field.nodes[node].position - query).cast<double>().squaredNorm();
        if (d < attach_d) {
            attach_d = d;
            attach   = node;
        }
    }

    // Widest bottleneck first, shortest path among equal bottlenecks, then node order - and node
    // order is (layer, island, medial order), so the answer does not depend on how the search ran.
    struct Label
    {
        double bottleneck = 0.;
        double length     = 0.;
        double lever      = 0.;
        size_t node       = 0;
    };
    const auto worse = [](const Label &a, const Label &b) {
        if (a.bottleneck != b.bottleneck)
            return a.bottleneck < b.bottleneck;
        if (a.length != b.length)
            return a.length > b.length;
        return a.node > b.node;
    };
    std::priority_queue<Label, std::vector<Label>, decltype(worse)> frontier(worse);
    std::vector<double> best_bottleneck(field.nodes.size(), -1.);
    std::vector<double> best_length(field.nodes.size(), std::numeric_limits<double>::max());
    std::vector<char>   settled(field.nodes.size(), 0);

    Label start;
    start.node       = attach;
    start.bottleneck = field.nodes[attach].width_mm;
    start.length     = std::sqrt(attach_d) * SCALING_FACTOR;
    start.lever      = start.length;
    best_bottleneck[attach] = start.bottleneck;
    best_length[attach]     = start.length;
    frontier.push(start);

    bool   rooted = false;
    double neck   = 0.;
    double lever  = 0.;
    while (! frontier.empty()) {
        const Label label = frontier.top();
        frontier.pop();
        if (settled[label.node])
            continue;
        settled[label.node] = 1;
        if (field.nodes[label.node].root) {
            rooted = true;
            neck   = label.bottleneck;
            lever  = label.lever;
            break;
        }
        for (const size_t e : field.adjacency[label.node]) {
            const Field::Edge &edge = field.edges[e];
            const size_t       next = edge.a == label.node ? edge.b : edge.a;
            if (settled[next])
                continue;
            Label step;
            step.node       = next;
            step.bottleneck = label.bottleneck;
            step.lever      = label.lever;
            // Along the path the connection comes first, then the sample at its far end. The lever
            // runs to where the bottleneck first appears, so a later constriction of the same width
            // does not move it further out.
            if (edge.width_mm < step.bottleneck) {
                step.bottleneck = edge.width_mm;
                step.lever      = label.length + edge.length_mm * 0.5;
            }
            step.length = label.length + edge.length_mm;
            if (field.nodes[next].width_mm < step.bottleneck) {
                step.bottleneck = field.nodes[next].width_mm;
                step.lever      = step.length;
            }
            if (step.bottleneck > best_bottleneck[next] ||
                (step.bottleneck == best_bottleneck[next] && step.length < best_length[next])) {
                best_bottleneck[next] = step.bottleneck;
                best_length[next]     = step.length;
                frontier.push(step);
            }
        }
    }
    if (! rooted)
        return result;

    const double weight = risk_weight(field.extrusion_width_mm, local, neck, lever);
    if (! std::isfinite(weight))
        return result;
    result.local_width_mm = local;
    result.neck_width_mm  = neck;
    result.lever_mm       = lever;
    result.risk_per_mm2   = weight;
    // Below one extrusion width the model cannot be printed as drawn, and the weight's own clamping
    // would otherwise hand back exactly the number a printable feature gets.
    result.status = local < field.extrusion_width_mm || neck < field.extrusion_width_mm ?
                        Sample::Status::BelowPrintableWidth : Sample::Status::Known;
    return result;
}

double risk_weight(double extrusion_width_mm, double local_width_mm, double neck_width_mm, double lever_mm)
{
    if (! std::isfinite(extrusion_width_mm) || extrusion_width_mm <= 0. || ! std::isfinite(local_width_mm) ||
        ! std::isfinite(neck_width_mm) || ! std::isfinite(lever_mm))
        return std::numeric_limits<double>::quiet_NaN();
    // Both widths clamp at one extrusion: below it the model cannot be printed as drawn at all, and
    // no ratio taken there means anything. What that clamping hides, Sample::BelowPrintableWidth says.
    return (extrusion_width_mm / std::max(local_width_mm, extrusion_width_mm)) *
           (1. + lever_mm / std::max(neck_width_mm, extrusion_width_mm));
}

namespace {

// The 26 directions with coordinates in {-1, 0, 1} bar the zero vector, normalized, in lexicographic
// order of the integer triples. The first one found clear is the answer, so this order is part of it.
const std::vector<Vec3d> &escape_directions()
{
    static const std::vector<Vec3d> directions = []() {
        std::vector<Vec3d> out;
        out.reserve(26);
        for (int x = -1; x <= 1; ++ x)
            for (int y = -1; y <= 1; ++ y)
                for (int z = -1; z <= 1; ++ z)
                    if (x != 0 || y != 0 || z != 0)
                        out.push_back(Vec3d(double(x), double(y), double(z)).normalized());
        return out;
    }();
    return directions;
}

// The point of triangle abc nearest to p, by the barycentric region test: the interior projection
// where the projection lands inside, and the nearest edge point or vertex otherwise.
Vec3d closest_point_on_triangle(const Vec3d &p, const Vec3d &a, const Vec3d &b, const Vec3d &c)
{
    const Vec3d  ab = b - a, ac = c - a, ap = p - a;
    const double d1 = ab.dot(ap), d2 = ac.dot(ap);
    if (d1 <= 0. && d2 <= 0.)
        return a;
    const Vec3d  bp = p - b;
    const double d3 = ab.dot(bp), d4 = ac.dot(bp);
    if (d3 >= 0. && d4 <= d3)
        return b;
    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0. && d1 >= 0. && d3 <= 0.)
        return a + ab * (d1 / (d1 - d3));
    const Vec3d  cp = p - c;
    const double d5 = ab.dot(cp), d6 = ac.dot(cp);
    if (d6 >= 0. && d5 <= d6)
        return c;
    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0. && d2 >= 0. && d6 <= 0.)
        return a + ac * (d2 / (d2 - d6));
    const double va = d3 * d6 - d5 * d4;
    if (va <= 0. && (d4 - d3) >= 0. && (d5 - d6) >= 0.)
        return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
    const double denom = 1. / (va + vb + vc);
    return a + ab * (vb * denom) + ac * (vc * denom);
}

// The nearest pair of points of two segments, clamped at both ends, parallel segments included.
void closest_points_on_segments(const Vec3d &p0, const Vec3d &p1, const Vec3d &q0, const Vec3d &q1,
                                Vec3d &on_p, Vec3d &on_q)
{
    const Vec3d  u = p1 - p0, v = q1 - q0, w = p0 - q0;
    const double a = u.dot(u), b = u.dot(v), c = v.dot(v), d = u.dot(w), e = v.dot(w);
    const double denom = a * c - b * b;
    double       s = 0., t = 0.;
    if (denom > 1e-18) {
        s = std::clamp((b * e - c * d) / denom, 0., 1.);
        t = (b * s + e) / (c > 0. ? c : 1.);
        if (t < 0.) {
            t = 0.;
            s = a > 0. ? std::clamp(- d / a, 0., 1.) : 0.;
        } else if (t > 1.) {
            t = 1.;
            s = a > 0. ? std::clamp((b - d) / a, 0., 1.) : 0.;
        }
    } else {
        // Parallel: one end of each segment against the other segment is the whole of the answer.
        t = c > 0. ? std::clamp(- e / c, 0., 1.) : 0.;
        s = a > 0. ? std::clamp((b * t - d) / a, 0., 1.) : 0.;
    }
    on_p = p0 + u * s;
    on_q = q0 + v * t;
}

// The exact minimum distance between the segment [a, b] and the triangle, and where on the triangle
// it is reached: zero where the segment pierces the triangle, and otherwise the nearest of the two
// endpoints against the triangle and the segment against each of the three edges.
double segment_triangle_distance(const Vec3d &a, const Vec3d &b, const Vec3d &v0, const Vec3d &v1,
                                 const Vec3d &v2, Vec3d *on_triangle)
{
    const Vec3d  normal = (v1 - v0).cross(v2 - v0);
    const double da = normal.dot(a - v0), db = normal.dot(b - v0);
    if (normal.squaredNorm() > 0. && ((da <= 0. && db >= 0.) || (da >= 0. && db <= 0.)) && da != db) {
        const Vec3d crossing = a + (b - a) * (da / (da - db));
        // Inside the triangle when it lies on the same side of all three edges.
        const Vec3d n0 = (v1 - v0).cross(crossing - v0), n1 = (v2 - v1).cross(crossing - v1),
                    n2 = (v0 - v2).cross(crossing - v2);
        if (n0.dot(normal) >= 0. && n1.dot(normal) >= 0. && n2.dot(normal) >= 0.) {
            if (on_triangle != nullptr)
                *on_triangle = crossing;
            return 0.;
        }
    }
    double best = std::numeric_limits<double>::max();
    Vec3d  best_point = v0;
    const auto keep = [&best, &best_point](const Vec3d &from, const Vec3d &on) {
        const double d = (on - from).norm();
        if (d < best) {
            best       = d;
            best_point = on;
        }
    };
    keep(a, closest_point_on_triangle(a, v0, v1, v2));
    keep(b, closest_point_on_triangle(b, v0, v1, v2));
    const Vec3d edges[3][2] = { { v0, v1 }, { v1, v2 }, { v2, v0 } };
    for (const Vec3d (&edge)[2] : edges) {
        Vec3d on_segment, on_edge;
        closest_points_on_segments(a, b, edge[0], edge[1], on_segment, on_edge);
        keep(on_segment, on_edge);
    }
    if (on_triangle != nullptr)
        *on_triangle = best_point;
    return best;
}

// The distance in mm between the 2-D segment [a, b] and `polygon`, zero where the segment enters it,
// with the nearest point of the polygon and where on the segment it was reached. Contours and holes
// alike: a hole's wall is polygon boundary the segment has to keep clear of just the same.
double segment_polygon_distance_mm(const Vec2d &a, const Vec2d &b, const ExPolygon &polygon,
                                   Vec2d *on_polygon, double *at)
{
    const Point start(coord_t(std::lround(scaled<double>(a.x()))), coord_t(std::lround(scaled<double>(a.y()))));
    if (polygon.contains(start)) {
        *on_polygon = a;
        *at         = 0.;
        return 0.;
    }
    double best = std::numeric_limits<double>::max();
    const auto walk = [&](const Polygon &ring) {
        const Points &pts = ring.points;
        for (size_t i = 0; i < pts.size(); ++ i) {
            const Vec2d e0 = unscale(pts[i]), e1 = unscale(pts[(i + 1) % pts.size()]);
            Vec3d on_segment, on_edge;
            closest_points_on_segments(Vec3d(a.x(), a.y(), 0.), Vec3d(b.x(), b.y(), 0.),
                                       Vec3d(e0.x(), e0.y(), 0.), Vec3d(e1.x(), e1.y(), 0.), on_segment, on_edge);
            const double d = (on_edge - on_segment).norm();
            if (d < best) {
                best        = d;
                *on_polygon = Vec2d(on_edge.x(), on_edge.y());
                const double span = (b - a).norm();
                *at               = span > 0. ? std::clamp((Vec2d(on_segment.x(), on_segment.y()) - a).norm() / span, 0., 1.) : 0.;
            }
        }
    };
    walk(polygon.contour);
    for (const Polygon &hole : polygon.holes)
        walk(hole);
    return best;
}

// Whether the model stops the probe running from `start` to `end`. The ray down the middle of the
// capsule is the broad phase: every hit past the start lies outside the sphere the contact's own
// material is excused inside of, so one of them settles the direction. What that ray misses and the
// capsule's wall does not is found by measuring the segment against each triangle whose own extents
// reach the capsule at all.
bool blocked_by_model(const AABBMesh &mesh, const indexed_triangle_set &its, const Vec3d &contact,
                      const Vec3d &start, const Vec3d &end, const Vec3d &far_start, double r)
{
    const Vec3d  along  = end - start;
    const double length = along.norm();
    if (length <= 0.)
        return false;
    for (const AABBMesh::hit_result &hit : mesh.query_ray_hits(start, along / length))
        if (hit.is_hit() && hit.distance() > 1e-9 && hit.distance() <= length)
            return true;
    const Vec3d lo = start.cwiseMin(end) - Vec3d(r, r, r), hi = start.cwiseMax(end) + Vec3d(r, r, r);
    for (const Vec3i32 &face : its.indices) {
        const Vec3d v0 = its.vertices[face(0)].cast<double>(), v1 = its.vertices[face(1)].cast<double>(),
                    v2 = its.vertices[face(2)].cast<double>();
        const Vec3d tri_lo = v0.cwiseMin(v1).cwiseMin(v2), tri_hi = v0.cwiseMax(v1).cwiseMax(v2);
        if ((tri_hi.array() < lo.array()).any() || (tri_lo.array() > hi.array()).any())
            continue;
        Vec3d on_triangle(0., 0., 0.);
        if (segment_triangle_distance(start, end, v0, v1, v2, &on_triangle) > r)
            continue;
        if ((on_triangle - contact).norm() <= r &&
            segment_triangle_distance(far_start, end, v0, v1, v2, nullptr) > r)
            continue;   // the model the contact is placed on, and nothing of it further along
        return true;
    }
    return false;
}

// One printed support layer as the probe meets it: the closed Z interval it occupies and the ground
// it covers on it.
struct AccessSlab
{
    double            bottom = 0.;
    double            top    = 0.;
    const ExPolygons *polygons = nullptr;
};

// The part of [a, b] whose Z lies in [lo, hi], as parameters into the segment. Answers false where
// the segment never enters that band.
bool clip_to_z(const Vec3d &a, const Vec3d &b, double lo, double hi, double &t0, double &t1)
{
    t0 = 0.;
    t1 = 1.;
    const double dz = b.z() - a.z();
    if (std::abs(dz) < 1e-12)
        return a.z() >= lo && a.z() <= hi;
    double enter = (lo - a.z()) / dz, leave = (hi - a.z()) / dz;
    if (enter > leave)
        std::swap(enter, leave);
    t0 = std::max(0., enter);
    t1 = std::min(1., leave);
    return t0 <= t1;
}

// Whether the capsule of radius `r` around [a, b] shares volume with the slab's material, and where
// on that material it was met. The slab is grown by `r` in Z before the plan-view test, so material
// the probe passes just over or just under its cap reads as met.
bool slab_meets(const AccessSlab &slab, const Vec3d &a, const Vec3d &b, double r, Vec3d &where)
{
    if (slab.polygons->empty())
        return false;
    double t0 = 0., t1 = 1.;
    if (! clip_to_z(a, b, slab.bottom - r, slab.top + r, t0, t1))
        return false;
    const Vec3d from = a + (b - a) * t0, to = a + (b - a) * t1;
    const Vec2d from2(from.x(), from.y()), to2(to.x(), to.y());
    const BoundingBox reach(Points{
        Point(coord_t(std::lround(scaled<double>(std::min(from2.x(), to2.x()) - r))),
              coord_t(std::lround(scaled<double>(std::min(from2.y(), to2.y()) - r)))),
        Point(coord_t(std::lround(scaled<double>(std::max(from2.x(), to2.x()) + r))),
              coord_t(std::lround(scaled<double>(std::max(from2.y(), to2.y()) + r)))) });
    for (const ExPolygon &polygon : *slab.polygons) {
        if (! reach.overlap(get_extents(polygon)))
            continue;
        Vec2d  on_polygon(0., 0.);
        double at = 0.;
        if (segment_polygon_distance_mm(from2, to2, polygon, &on_polygon, &at) >= r)
            continue;   // touching is not shared volume
        const double z = from.z() + (to.z() - from.z()) * at;
        where = Vec3d(on_polygon.x(), on_polygon.y(), std::clamp(z, slab.bottom, slab.top));
        return true;
    }
    return false;
}

} // namespace

Access assess_access(const AABBMesh &model, const std::vector<ExPolygons> &support_by_layer,
                     const std::vector<double> &layer_z_mm, const Vec3d &contact, double clearance_mm)
{
    Access out;
    const indexed_triangle_set *its = model.get_triangle_mesh();
    if (its == nullptr || its->indices.empty() || its->vertices.empty())
        return out;   // no mesh to test against: unknown, and never a way out
    if (! contact.allFinite() || ! std::isfinite(clearance_mm) || clearance_mm <= 0.)
        return out;
    if (support_by_layer.size() != layer_z_mm.size())
        return out;
    for (size_t i = 0; i < layer_z_mm.size(); ++ i)
        if (! std::isfinite(layer_z_mm[i]) || (i > 0 && layer_z_mm[i] < layer_z_mm[i - 1]))
            return out;

    const double r = clearance_mm;
    // A slab runs from the print_z of the layer under it; the first from as far below its own as the
    // second layer stands above it, which is the height that stack was printed at.
    std::vector<AccessSlab> slabs;
    slabs.reserve(layer_z_mm.size());
    for (size_t i = 0; i < layer_z_mm.size(); ++ i) {
        AccessSlab slab;
        slab.top      = layer_z_mm[i];
        slab.bottom   = i > 0 ? layer_z_mm[i - 1] :
                        (layer_z_mm.size() > 1 ? layer_z_mm[0] - (layer_z_mm[1] - layer_z_mm[0]) : layer_z_mm[0] - r);
        slab.polygons = &support_by_layer[i];
        slabs.push_back(slab);
    }

    // How far the probe has to travel to be out of the print: the diagonal of the model and the
    // support taken together, twice over.
    BoundingBoxf3 bounds;
    for (const Vec3f &vertex : its->vertices)
        bounds.merge(vertex.cast<double>());
    for (const AccessSlab &slab : slabs)
        for (const ExPolygon &polygon : *slab.polygons) {
            const BoundingBox extents = get_extents(polygon);
            bounds.merge(Vec3d(unscale<double>(extents.min.x()), unscale<double>(extents.min.y()), slab.bottom));
            bounds.merge(Vec3d(unscale<double>(extents.max.x()), unscale<double>(extents.max.y()), slab.top));
        }
    if (! bounds.defined || ! bounds.min.allFinite() || ! bounds.max.allFinite())
        return out;
    const double span = 2. * (bounds.max - bounds.min).norm();
    if (! std::isfinite(span) || span <= 0.)
        return out;

    size_t buried = 0;
    for (const Vec3d &direction : escape_directions()) {
        const Vec3d  start = contact + direction * r;
        const Vec3d  end   = contact + direction * (r + span);
        // Where the probe would start inside the model there is nothing to measure from. Only where
        // that holds of every direction is the contact one the measurement cannot answer for.
        // Where the ray forward finds nothing the ray back settles it: a start inside the solid has
        // material behind it whichever way a corner-grazing hit was missed forward.
        const AABBMesh::hit_result first = model.query_ray_hit(start, direction);
        if (first.is_hit() ? first.is_inside() : model.query_ray_hit(start, - direction).is_inside()) {
            ++ buried;
            continue;
        }
        // The model at the contact is what the contact is placed on, so it is excused inside the
        // probe radius of it - and only there. Past three radii nothing inside that sphere can reach
        // the capsule, so the far part of the probe is tested with nothing excused at all.
        const Vec3d far_start = contact + direction * std::min(3. * r, r + span);
        if (blocked_by_model(model, *its, contact, start, end, far_start, r))
            continue;
        bool blocked = false;
        for (const AccessSlab &slab : slabs) {
            Vec3d where(0., 0., 0.);
            if (! slab_meets(slab, start, end, r, where))
                continue;
            if ((where - contact).norm() <= r && ! slab_meets(slab, far_start, end, r, where))
                continue;   // the source component's own material, inside the starting sphere
            blocked = true;
            break;
        }
        if (blocked)
            continue;
        out.status    = Access::Status::Clear;
        out.direction = direction;
        return out;
    }
    if (buried == escape_directions().size())
        return out;   // a start that cannot leave the source patch
    out.status = Access::Status::Blocked;
    return out;
}

} // namespace ModelSupportRisk
} // namespace Slic3r
