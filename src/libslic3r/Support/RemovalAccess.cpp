#include "RemovalAccess.hpp"

#include "../AABBMesh.hpp"
#include "../AABBTreeIndirect.hpp"
#include "../BoundingBox.hpp"
#include "../libslic3r.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace Slic3r {
namespace RemovalAccess {

namespace {

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
    keep(a, AABBTreeIndirect::detail::closest_point_to_triangle(a, v0, v1, v2));
    keep(b, AABBTreeIndirect::detail::closest_point_to_triangle(b, v0, v1, v2));
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
// capsule's wall does not is found by measuring the segment against each triangle the mesh's AABB tree
// finds inside the capsule's bounding box.
bool blocked_by_model(const AABBMesh &mesh, const indexed_triangle_set &its, const Vec3d &contact,
                      const Vec3d &start, const Vec3d &end, const Vec3d &far_start, double r)
{
    const Vec3d  along  = end - start;
    const double length = along.norm();
    for (const AABBMesh::hit_result &hit : mesh.query_ray_hits(start, along / length))
        if (hit.is_hit() && hit.distance() > 1e-9 && hit.distance() <= length)
            return true;
    const Vec3d lo = start.cwiseMin(end) - Vec3d(r, r, r), hi = start.cwiseMax(end) + Vec3d(r, r, r);
    for (const size_t face_id : mesh.faces_in_box(lo, hi)) {
        const Vec3i32 &face = its.indices[face_id];
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
// on that material it was met.
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
    if (its == nullptr || its->indices.empty() || its->vertices.empty()) {
        out.missing = Access::Missing::NoMesh;   // no mesh to test against: unknown, and never a way out
        return out;
    }
    if (! contact.allFinite() || ! std::isfinite(clearance_mm) || clearance_mm <= 0.) {
        out.missing = Access::Missing::BadContact;
        return out;
    }
    if (support_by_layer.size() != layer_z_mm.size()) {
        out.missing = Access::Missing::SlabMismatch;
        return out;
    }
    for (size_t i = 0; i < layer_z_mm.size(); ++ i)
        if (! std::isfinite(layer_z_mm[i]) || (i > 0 && layer_z_mm[i] < layer_z_mm[i - 1])) {
            out.missing = Access::Missing::SlabOrder;
            return out;
        }

    const double r = clearance_mm;
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
    if (! bounds.defined || ! bounds.min.allFinite() || ! bounds.max.allFinite()) {
        out.missing = Access::Missing::BadBounds;
        return out;
    }
    const double span = 2. * (bounds.max - bounds.min).norm();
    if (! std::isfinite(span) || span <= 0.) {
        out.missing = Access::Missing::BadBounds;
        return out;
    }

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
    if (buried == escape_directions().size()) {
        out.missing = Access::Missing::AllBuried;   // a start that cannot leave the source patch
        return out;
    }
    out.status = Access::Status::Blocked;
    return out;
}

const char *missing_name(Access::Missing missing)
{
    switch (missing) {
    case Access::Missing::None:         return "none";
    case Access::Missing::NoMesh:       return "no_mesh";
    case Access::Missing::BadContact:   return "bad_contact";
    case Access::Missing::SlabMismatch: return "slab_mismatch";
    case Access::Missing::SlabOrder:    return "slab_order";
    case Access::Missing::BadBounds:    return "bad_bounds";
    case Access::Missing::AllBuried:    return "all_buried";
    }
    return "none";
}

} // namespace RemovalAccess
} // namespace Slic3r
