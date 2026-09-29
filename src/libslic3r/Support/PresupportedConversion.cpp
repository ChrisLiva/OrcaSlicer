#include "PresupportedConversion.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <optional>
#include <vector>

#include <Eigen/Eigenvalues>

#include "AABBTreeIndirect.hpp"
#include "Model.hpp"
#include "ScaffoldPlan.hpp"
#include "TriangleMesh.hpp"
#include "Format/bbs_3mf.hpp"

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

namespace Slic3r {
namespace PresupportedConversion {

namespace {

// All lengths in mm, in the instance's world frame.
constexpr double plate_mm            = 0.05;  // a shell whose bottom is this close to the mesh's bottom stands on the plate
constexpr double flat_height_mm      = 1.0;   // a shell on the plate no taller than this is an artist raft piece
constexpr double gap_floor           = 1e-3;  // the gap search stops at shells this small against the largest's box
constexpr double min_volume_mm3      = 1e-5;  // a closed shell with less volume is a sliver of the figure's own mesh
constexpr size_t max_primitive_faces = 5000;  // a shell with more faces is no support primitive, and its hull test would cost
constexpr double plane_slack_mm      = 1e-4;  // how far a convex shell's vertex may stand outside one of its face planes
constexpr double level_mm            = 1e-4;  // vertices this close in height along the axis form one ring
constexpr double flat_mm             = 1e-3;  // a ring this close to the end's extreme is a flat end
constexpr double fit_slack           = 0.01;  // a sphere fits an end while every vertex lies within this share of r...
constexpr double fit_floor_mm        = 1e-4;  // ...or this far, whichever is more
constexpr double min_cap_depth       = 0.3;   // a rounded end runs at least this share of its radius along the axis
constexpr double max_end_radius_mm   = 2.0;   // a fitted sphere wider than this is a flat piece, not a rounded end
constexpr double tip_ratio           = 0.9;   // a tip's narrow end radius is under this share of its wide end's
constexpr double contact_slack_mm    = 0.05;  // an end sits on the figure when its centre is within r + this of it
constexpr double overlap_mm          = 0.01;  // two supports touch when one's vertex lies inside the other or this close outside
constexpr double duplicate_site_mm   = 0.01;  // tips this close, and within duplicate_axis_deg, are one tip
constexpr double duplicate_axis_deg  = 1.0;
constexpr double heavy_diameter_mm   = 0.45;  // a contact this wide or wider is a Heavy tip
constexpr double no_azimuth          = 1e-3;  // an axis whose horizontal part is shorter has only the fit's noise for azimuth

struct Planes
{
    std::vector<Vec3d>  normals; // outward
    std::vector<double> offsets;
    bool                holds(const Vec3d &p, double slack) const
    {
        for (size_t i = 0; i < normals.size(); ++i)
            if (normals[i].dot(p) - offsets[i] > slack)
                return false;
        return true;
    }
};

struct End
{
    enum class Kind : uint8_t { None, Flat, Round } kind = Kind::None;
    Vec3d  centre   = Vec3d::Zero();
    double r        = 0.;
    double figure_d = std::numeric_limits<double>::infinity(); // the centre's distance to the figure
    bool   usable() const { return kind != Kind::None && r < max_end_radius_mm; }
    bool   on_figure() const { return usable() && figure_d <= r + contact_slack_mm; }
};

struct Shell
{
    indexed_triangle_set its;   // in the volume's mesh frame, as the split returns it
    std::vector<Vec3d>   world; // its vertices in the instance's world frame
    BoundingBoxf3        box;
    bool                 figure    = false;
    bool                 raft      = false; // a raft piece on the plate
    bool                 primitive = false; // closed, convex and with volume, as every artist support piece is
    Planes               planes;
    End                  ends[2];
};

double box_volume(const BoundingBoxf3 &box)
{
    const Vec3d size = box.size();
    return size.x() * size.y() * size.z();
}

// Closed, convex and with volume; `planes` gets the face planes turned away from the centroid.
bool primitive(Shell &shell)
{
    if (shell.its.indices.size() > max_primitive_faces || its_num_open_edges(shell.its) != 0)
        return false;
    const Vec3d centroid = std::accumulate(shell.world.begin(), shell.world.end(), Vec3d(Vec3d::Zero())) / double(shell.world.size());
    double      volume   = 0.;
    for (const stl_triangle_vertex_indices &f : shell.its.indices) {
        const Vec3d &a = shell.world[f[0]], &b = shell.world[f[1]], &c = shell.world[f[2]];
        volume += a.dot(b.cross(c)) / 6.;
        Vec3d        n = (b - a).cross(c - a);
        const double l = n.norm();
        if (l < 1e-12)
            continue;
        n /= l;
        if (n.dot(a - centroid) < 0.)
            n = -n;
        shell.planes.normals.push_back(n);
        shell.planes.offsets.push_back(n.dot(a));
    }
    if (std::abs(volume) <= min_volume_mm3)
        return false;
    return std::all_of(shell.world.begin(), shell.world.end(), [&shell](const Vec3d &p) { return shell.planes.holds(p, plane_slack_mm); });
}

// The least-squares sphere through `pts`, |p - c|^2 = r^2 read as 2 p.c + (r^2 - c.c) = p.p about the first point.
bool fit_sphere(const std::vector<Vec3d> &pts, Vec3d &centre, double &r)
{
    const Vec3d                     o   = pts.front();
    Eigen::Matrix4d                 ata = Eigen::Matrix4d::Zero();
    Eigen::Vector4d                 atb = Eigen::Vector4d::Zero();
    for (const Vec3d &p : pts) {
        const Vec3d           q = p - o;
        const Eigen::Vector4d row(2. * q.x(), 2. * q.y(), 2. * q.z(), 1.);
        ata += row * row.transpose();
        atb += row * q.squaredNorm();
    }
    const Eigen::FullPivLU<Eigen::Matrix4d> lu(ata);
    if (lu.rank() < 4)
        return false;
    const Eigen::Vector4d x = lu.solve(atb);
    const double          r2 = x[3] + x.head<3>().squaredNorm();
    if (! (r2 > 0.))
        return false;
    centre = o + x.head<3>();
    r      = std::sqrt(r2);
    return true;
}

// The end of `pts` farthest along `sign * axis` from `c`: rings of vertices walked inward from the extreme while one
// sphere holds them all. Three rings on a sphere, the extreme vertex counting as one, make a rounded end, since any two
// coaxial rings lie on some sphere; a first ring of three or more vertices that no sphere continues is a flat end.
End fit_end(const std::vector<Vec3d> &pts, const Vec3d &c, const Vec3d &axis, double sign)
{
    std::vector<std::pair<double, size_t>> order(pts.size());
    for (size_t i = 0; i < pts.size(); ++i)
        order[i] = { -sign * (pts[i] - c).dot(axis), i };
    std::sort(order.begin(), order.end());
    const double top = order.front().first;
    // Ring boundaries: order[rings[k] .. rings[k + 1]) is ring k.
    std::vector<size_t> rings { 0 };
    for (size_t i = 1; i < order.size(); ++i)
        if (order[i].first - order[rings.back()].first > level_mm)
            rings.push_back(i);
    rings.push_back(order.size());

    End                end;
    std::vector<Vec3d> held;
    size_t             held_rings = 0;
    for (size_t k = 0; k + 1 < rings.size(); ++k) {
        std::vector<Vec3d> trial = held;
        for (size_t i = rings[k]; i < rings[k + 1]; ++i)
            trial.push_back(pts[order[i].second]);
        Vec3d  centre;
        double r = 0.;
        if (trial.size() >= 4) {
            if (! fit_sphere(trial, centre, r))
                break;
            const double slack = std::max(fit_floor_mm, fit_slack * r);
            if (std::any_of(trial.begin(), trial.end(), [&](const Vec3d &p) { return std::abs((p - centre).norm() - r) > slack; }))
                break;
            end.centre = centre;
            end.r      = r;
        }
        held = std::move(trial);
        ++held_rings;
    }
    const double depth = held.empty() ? 0. : order[held.size() - 1].first - top;
    if (held_rings >= 3 && end.r > 0. && depth >= min_cap_depth * end.r) {
        end.kind = End::Kind::Round;
        return end;
    }
    end = End();
    if (rings[1] >= 3 && order[rings[1] - 1].first - top <= flat_mm) {
        end.kind = End::Kind::Flat;
        for (size_t i = 0; i < rings[1]; ++i)
            end.centre += pts[order[i].second];
        end.centre /= double(rings[1]);
        for (size_t i = 0; i < rings[1]; ++i)
            end.r = std::max(end.r, (pts[order[i].second] - end.centre).norm());
    }
    return end;
}

// Both ends of a shell along its principal axis.
void fit_ends(Shell &shell)
{
    const std::vector<Vec3d> &pts = shell.world;
    const Vec3d               c   = std::accumulate(pts.begin(), pts.end(), Vec3d(Vec3d::Zero())) / double(pts.size());
    Matrix3d                  cov = Matrix3d::Zero();
    for (const Vec3d &p : pts)
        cov += (p - c) * (p - c).transpose();
    const Eigen::SelfAdjointEigenSolver<Matrix3d> eig(cov);
    const Vec3d                                   axis = eig.eigenvectors().col(2); // eigenvalues ascend
    shell.ends[0] = fit_end(pts, c, axis, 1.);
    shell.ends[1] = fit_end(pts, c, axis, -1.);
}

struct Tip
{
    size_t shell;
    Vec3d  site;   // the narrow end's centre
    Vec3d  axis;   // unit, from the site toward the wide end
    double diameter;
};

// The tip a shell makes: two rounded ends of different radii, apart by more than the wide one, the narrow one on the figure.
std::optional<Tip> tip_of(const Shell &shell, size_t idx)
{
    const End &a = shell.ends[0], &b = shell.ends[1];
    if (a.kind != End::Kind::Round || b.kind != End::Kind::Round || ! a.usable() || ! b.usable())
        return std::nullopt;
    const End   &narrow = a.r < b.r ? a : b, &wide = a.r < b.r ? b : a;
    const Vec3d  span   = wide.centre - narrow.centre;
    if (narrow.r >= tip_ratio * wide.r || span.norm() <= wide.r || ! narrow.on_figure())
        return std::nullopt;
    return Tip { idx, narrow.centre, span.normalized(), 2. * narrow.r };
}

size_t find_root(std::vector<size_t> &parent, size_t i)
{
    while (parent[i] != i)
        i = parent[i] = parent[parent[i]];
    return i;
}

// Each support's cluster root: supports join where a vertex of one lies inside the other, found over boxes swept in x.
std::vector<size_t> clusters(const std::vector<Shell> &shells, const std::vector<size_t> &supports)
{
    std::vector<size_t> parent(supports.size());
    std::iota(parent.begin(), parent.end(), 0);
    std::vector<size_t> by_x(supports.size());
    std::iota(by_x.begin(), by_x.end(), 0);
    std::sort(by_x.begin(), by_x.end(), [&](size_t l, size_t r) { return shells[supports[l]].box.min.x() < shells[supports[r]].box.min.x(); });
    const auto inside = [](const Shell &a, const Shell &b) {
        return std::any_of(a.world.begin(), a.world.end(), [&b](const Vec3d &p) { return b.planes.holds(p, overlap_mm); });
    };
    for (size_t i = 0; i < by_x.size(); ++i) {
        const Shell &a = shells[supports[by_x[i]]];
        for (size_t j = i + 1; j < by_x.size(); ++j) {
            const Shell &b = shells[supports[by_x[j]]];
            if (b.box.min.x() > a.box.max.x() + overlap_mm)
                break;
            if (b.box.min.y() > a.box.max.y() + overlap_mm || a.box.min.y() > b.box.max.y() + overlap_mm ||
                b.box.min.z() > a.box.max.z() + overlap_mm || a.box.min.z() > b.box.max.z() + overlap_mm)
                continue;
            const size_t ra = find_root(parent, by_x[i]), rb = find_root(parent, by_x[j]);
            if (ra != rb && (inside(a, b) || inside(b, a)))
                parent[ra] = rb;
        }
    }
    std::vector<size_t> root(supports.size());
    for (size_t i = 0; i < supports.size(); ++i)
        root[i] = find_root(parent, i);
    return root;
}

// Marks the raft and the figure. A shell on the plate is raft when it is no taller than a flat piece, or when it is a
// primitive lower than its shorter horizontal side, as a raft box is. Past the raft and the primitives standing on it, the figure is the
// shells above the widest ratio gap between consecutive box volumes, largest first; the largest alone when no gap is
// searched. A shell that starts on the plate stands on no raft piece, and a shell that is no primitive is never set
// aside as standing on one, so a figure on the plate or on a raft stays a candidate.
void mark_figure(std::vector<Shell> &shells, double z_min)
{
    std::vector<size_t> flat, rest;
    for (size_t i = 0; i < shells.size(); ++i) {
        const Vec3d size = shells[i].box.size();
        if (shells[i].box.min.z() <= z_min + plate_mm &&
            (size.z() <= flat_height_mm || (shells[i].primitive && size.z() < std::min(size.x(), size.y())))) {
            shells[i].raft = true;
            flat.push_back(i);
        }
    }
    for (size_t i = 0; i < shells.size(); ++i) {
        const BoundingBoxf3 &b = shells[i].box;
        if (shells[i].raft)
            continue;
        const bool resting = shells[i].primitive && b.min.z() > z_min + plate_mm && std::any_of(flat.begin(), flat.end(), [&](size_t f) {
            const BoundingBoxf3 &base = shells[f].box;
            return b.min.z() <= base.max.z() + overlap_mm && b.min.x() <= base.max.x() + overlap_mm && b.max.x() >= base.min.x() - overlap_mm &&
                   b.min.y() <= base.max.y() + overlap_mm && b.max.y() >= base.min.y() - overlap_mm;
        });
        if (! resting)
            rest.push_back(i);
    }
    if (rest.empty())
        return;
    std::stable_sort(rest.begin(), rest.end(), [&](size_t l, size_t r) { return box_volume(shells[l].box) > box_volume(shells[r].box); });
    const double largest = box_volume(shells[rest.front()].box);
    size_t       above   = 0;
    double       best    = 1.;
    for (size_t i = 0; i + 1 < rest.size(); ++i) {
        const double v = box_volume(shells[rest[i]].box);
        if (v < gap_floor * largest)
            break;
        const double ratio = v / std::max(box_volume(shells[rest[i + 1]].box), 1e-12);
        if (ratio > best) {
            best  = ratio;
            above = i;
        }
    }
    for (size_t i = 0; i <= above; ++i)
        shells[rest[i]].figure = true;
}

// `axis` moved onto the cone of the builder's head tilt cap about straight down when it leans past it, azimuth kept.
bool clamp_axis(Vec3d &axis)
{
    const double cap = ScaffoldSupport::max_head_tilt_rad;
    if (-axis.z() >= std::cos(cap) - EPSILON)
        return false;
    Vec2d h = axis.head<2>();
    h       = h.norm() > no_azimuth ? Vec2d(h.normalized()) : Vec2d(1., 0.); // straight up has no azimuth: lean it toward +x
    axis    = Vec3d(std::sin(cap) * h.x(), std::sin(cap) * h.y(), -std::cos(cap));
    return true;
}

} // namespace

Summary convert(ModelObject &object, size_t instance_idx, const std::function<void()> &before_change)
{
    Summary      summary;
    ModelVolume *part = nullptr;
    for (ModelVolume *volume : object.volumes)
        if (volume->is_model_part()) {
            if (part != nullptr) {
                summary.refusal = Refusal::SeveralParts;
                return summary;
            }
            part = volume;
        }
    if (part == nullptr || instance_idx >= object.instances.size()) {
        summary.refusal = Refusal::SeveralParts;
        return summary;
    }

    // The same split ModelObject::split makes: faces joined by a shared edge.
    std::vector<Shell> shells;
    for (indexed_triangle_set &its : its_split(part->mesh().its))
        shells.push_back(Shell { std::move(its) });
    if (shells.size() < 2) {
        summary.refusal = Refusal::NoSupports;
        return summary;
    }
    const ModelInstance &instance = *object.instances[instance_idx];
    const Transform3d    to_world = instance.get_matrix() * part->get_matrix();
    tbb::parallel_for(tbb::blocked_range<size_t>(0, shells.size()), [&](const tbb::blocked_range<size_t> &range) {
        for (size_t i = range.begin(); i < range.end(); ++i) {
            Shell &shell = shells[i];
            shell.world.reserve(shell.its.vertices.size());
            for (const Vec3f &v : shell.its.vertices)
                shell.world.push_back(to_world * v.cast<double>());
            shell.box       = BoundingBoxf3(shell.world);
            shell.primitive = primitive(shell);
        }
    });
    const double z_min = std::min_element(shells.begin(), shells.end(), [](const Shell &l, const Shell &r) {
                             return l.box.min.z() < r.box.min.z();
                         })->box.min.z();
    mark_figure(shells, z_min);

    // Supports: primitives outside the figure, and every raft piece, which leaves the mesh whatever its shape. Any
    // other shell, such as a sliver of the figure's own mesh, stays with the figure.
    tbb::parallel_for(tbb::blocked_range<size_t>(0, shells.size()), [&](const tbb::blocked_range<size_t> &range) {
        for (size_t i = range.begin(); i < range.end(); ++i)
            if (shells[i].primitive && ! shells[i].figure)
                fit_ends(shells[i]);
    });
    std::vector<size_t> supports;
    for (size_t i = 0; i < shells.size(); ++i) {
        if (shells[i].primitive && ! shells[i].figure)
            supports.push_back(i);
        else if (! shells[i].raft)
            shells[i].figure = true;
    }
    if (supports.empty()) {
        summary.refusal = Refusal::NoSupports;
        return summary;
    }

    // A cluster reaches the plate where a member stands on the raft or on the plate itself: a trunk sunk into a raft
    // piece reaches it whether or not the piece is convex enough to join the trunk's cluster.
    double raft_top = z_min + plate_mm;
    for (const Shell &shell : shells)
        if (shell.raft)
            raft_top = std::max(raft_top, shell.box.max.z());
    const std::vector<size_t> root = clusters(shells, supports);
    std::vector<size_t>       cluster_of(shells.size(), size_t(-1));
    std::vector<bool>         on_plate(supports.size(), false);
    for (size_t k = 0; k < supports.size(); ++k) {
        cluster_of[supports[k]] = root[k];
        on_plate[root[k]]       = on_plate[root[k]] || shells[supports[k]].box.min.z() <= raft_top + overlap_mm;
    }

    // Each end's distance to the figure, over a tree of the figure in the world frame, and the tips that makes. A
    // cluster off the plate that holds no tip and touches the figure less than twice is no support but a separate
    // part of the figure, a spear, a staff or an eye, which goes back to the figure; the tips are then read again
    // against it, since an artist may have tipped that part. A sphere's two ends are one sphere and one contact.
    std::vector<Tip>    tips;
    std::vector<bool>   has_tip;
    std::vector<size_t> contacts;
    for (bool returned = true; returned;) {
        indexed_triangle_set figure_world;
        for (const Shell &shell : shells)
            if (shell.figure) {
                const auto base = int(figure_world.vertices.size());
                for (const Vec3d &v : shell.world)
                    figure_world.vertices.push_back(v.cast<float>());
                for (const stl_triangle_vertex_indices &f : shell.its.indices)
                    figure_world.indices.emplace_back(f + stl_triangle_vertex_indices(base, base, base));
            }
        const AABBTreeIndirect::Tree3f tree = AABBTreeIndirect::build_aabb_tree_over_indexed_triangle_set(figure_world.vertices, figure_world.indices);
        tbb::parallel_for(tbb::blocked_range<size_t>(0, supports.size()), [&](const tbb::blocked_range<size_t> &range) {
            for (size_t k = range.begin(); k < range.end(); ++k)
                for (End &end : shells[supports[k]].ends)
                    if (end.usable()) {
                        size_t       face;
                        Vec3d        closest;
                        const double d2 = AABBTreeIndirect::squared_distance_to_indexed_triangle_set(figure_world.vertices, figure_world.indices,
                                                                                                     tree, end.centre, face, closest);
                        end.figure_d = d2 < 0. ? std::numeric_limits<double>::infinity() : std::sqrt(d2);
                    }
        });
        tips.clear();
        has_tip.assign(supports.size(), false);
        contacts.assign(supports.size(), 0);
        for (size_t k = 0; k < supports.size(); ++k) {
            const Shell &shell = shells[supports[k]];
            if (shell.figure)
                continue;
            if (std::optional<Tip> tip = tip_of(shell, supports[k])) {
                tips.push_back(*tip);
                has_tip[root[k]] = true;
            }
            const End &a = shell.ends[0], &b = shell.ends[1];
            const bool one_sphere = a.kind == End::Kind::Round && b.kind == End::Kind::Round && (a.centre - b.centre).norm() <= 0.5 * a.r;
            contacts[root[k]] += one_sphere ? size_t(a.on_figure() || b.on_figure()) : size_t(a.on_figure()) + size_t(b.on_figure());
        }
        returned = false;
        for (size_t k = 0; k < supports.size(); ++k)
            if (! shells[supports[k]].figure && ! on_plate[root[k]] && ! has_tip[root[k]] && contacts[root[k]] < 2)
                returned = shells[supports[k]].figure = true;
    }
    if (tips.empty()) {
        summary.refusal = Refusal::NoArtistTips;
        return summary;
    }
    for (size_t k = 0; k < supports.size(); ++k)
        if (root[k] == k && ! shells[supports[k]].figure && ! on_plate[k] && ! has_tip[k])
            ++summary.micro_struts_dropped;

    // One point per distinct tip, the first of each run of duplicates kept.
    const double         same_axis = std::cos(duplicate_axis_deg * M_PI / 180.);
    std::vector<Tip>     distinct;
    for (const Tip &tip : tips) {
        if (std::any_of(distinct.begin(), distinct.end(), [&](const Tip &t) {
                return (t.site - tip.site).norm() <= duplicate_site_mm && t.axis.dot(tip.axis) >= same_axis;
            }))
            ++summary.duplicates_removed;
        else
            distinct.push_back(tip);
    }
    const Transform3d to_raw      = instance.get_matrix().inverse();
    const Matrix3d    axis_to_raw = instance.get_matrix().linear().inverse();
    ScaffoldPoints    points;
    for (Tip &tip : distinct) {
        summary.axes_clamped          += size_t(clamp_axis(tip.axis));
        summary.tips_rooted_on_figure += size_t(! on_plate[cluster_of[tip.shell]]);
        points.push_back({ (to_raw * tip.site).cast<float>(),
                           tip.diameter >= heavy_diameter_mm - EPSILON ? ScaffoldHeadSize::Heavy : ScaffoldHeadSize::Light, true,
                           (axis_to_raw * tip.axis).normalized().cast<float>() });
    }
    summary.tips_converted = points.size();

    // The figure in the part's own frame, the part and instance transforms kept, so it stays where the artist posed it.
    indexed_triangle_set figure;
    for (const Shell &shell : shells)
        if (shell.figure)
            its_merge(figure, shell.its);
    if (before_change)
        before_change();
    for (ModelInstance *inst : object.instances)
        inst->auto_drop = false;
    part->set_mesh(std::move(figure));
    part->calculate_convex_hull();
    part->invalidate_convex_hull_2d();
    // Paint is indexed by triangle, and the Print and the canvas key a volume's mesh on its id.
    summary.paint_removed = part->is_any_painted();
    part->reset_extra_facets();
    part->set_new_unique_id();
    object.invalidate_bounding_box();
    object.scaffold_points          = std::move(points);
    object.scaffold_points_status   = ScaffoldPointsStatus::UserModified;
    object.scaffold_points_pose     = instance.get_matrix().linear();
    object.scaffold_points_mesh_box = object.raw_mesh_bounding_box();
    save_object_mesh(object);
    return summary;
}

} // namespace PresupportedConversion
} // namespace Slic3r
