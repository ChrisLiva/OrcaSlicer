#include <catch2/catch_all.hpp>

#include "libslic3r/AABBMesh.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Support/RemovalAccess.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/libslic3r.h"

#include <cmath>
#include <limits>
#include <vector>

using namespace Slic3r;

namespace {

// An axis-aligned rectangle in mm, as an ExPolygon in scaled object coordinates.
ExPolygon rect_mm(double x0, double y0, double x1, double y1)
{
    ExPolygon out;
    out.contour.points = { Point(scale_(x0), scale_(y0)), Point(scale_(x1), scale_(y0)),
                           Point(scale_(x1), scale_(y1)), Point(scale_(x0), scale_(y1)) };
    return out;
}

// One axis-aligned box in mm, merged into `out`. The access cases are built from solid boxes, so a
// cavity is six of them rather than one inverted shell and every face keeps an outward normal.
void add_box(indexed_triangle_set &out, const Vec3d &min, const Vec3d &size)
{
    const indexed_triangle_set box  = its_make_cube(size.x(), size.y(), size.z());
    const int                  base = int(out.vertices.size());
    for (const Vec3f &vertex : box.vertices)
        out.vertices.push_back(vertex + min.cast<float>());
    for (const Vec3i32 &face : box.indices)
        out.indices.push_back(face + Vec3i32(base, base, base));
}

// A 10 x 10 x 2 mm plate at z 8..10: a contact placed at (5, 5, 8) hangs under the middle of it with
// nothing else anywhere, which is the open case the closed cavity, the windowed cavity with a limb
// across it, and the unmeasured leg are measured against.
indexed_triangle_set open_plate()
{
    indexed_triangle_set its;
    add_box(its, Vec3d(0., 0., 8.), Vec3d(10., 10., 2.));
    return its;
}

// A closed 6 x 6 x 6 mm shell of 1 mm walls around a 4 mm cavity whose centre is (3, 3, 3), with a
// `window` mm square hole through the middle of its +x wall when `window` is positive.
indexed_triangle_set cavity(double window)
{
    indexed_triangle_set its;
    add_box(its, Vec3d(0., 0., 0.), Vec3d(6., 6., 1.));   // floor
    add_box(its, Vec3d(0., 0., 5.), Vec3d(6., 6., 1.));   // ceiling
    add_box(its, Vec3d(0., 0., 1.), Vec3d(6., 1., 4.));   // -y
    add_box(its, Vec3d(0., 5., 1.), Vec3d(6., 1., 4.));   // +y
    add_box(its, Vec3d(0., 1., 1.), Vec3d(1., 4., 4.));   // -x
    if (window <= 0.) {
        add_box(its, Vec3d(5., 1., 1.), Vec3d(1., 4., 4.));
        return its;
    }
    const double lo = 3. - 0.5 * window, hi = 3. + 0.5 * window;
    add_box(its, Vec3d(5., 1., 1.),  Vec3d(1., lo - 1., 4.));
    add_box(its, Vec3d(5., hi, 1.),  Vec3d(1., 5. - hi, 4.));
    add_box(its, Vec3d(5., lo, 1.),  Vec3d(1., window, lo - 1.));
    add_box(its, Vec3d(5., lo, hi),  Vec3d(1., window, 5. - hi));
    return its;
}

// `layers` slabs, each carrying the same footprint.
std::vector<ExPolygons> stacked(const ExPolygon &footprint, size_t layers)
{
    return std::vector<ExPolygons>(layers, ExPolygons{ footprint });
}

// Print_z values from `first` upward in `step` mm, which is how a support stack hands its slabs over:
// each slab runs from the print_z under it.
std::vector<double> layer_tops(double first, double step, size_t layers)
{
    std::vector<double> tops;
    for (size_t i = 0; i < layers; ++ i)
        tops.push_back(first + double(i) * step);
    return tops;
}

const std::vector<ExPolygons> no_support;
const std::vector<double>     no_layers;

} // namespace

TEST_CASE("Removal access answers with the first clear direction of the fixed order", "[RemovalAccess]")
{
    const double probe = 0.3;

    SECTION("a contact under an open plate leaves along the first direction the order names")
    {
        const indexed_triangle_set     plate = open_plate();
        const AABBMesh                 mesh(plate);
        const RemovalAccess::Access access =
            RemovalAccess::assess_access(mesh, no_support, no_layers, Vec3d(5., 5., 8.), probe);
        REQUIRE(access.status == RemovalAccess::Access::Status::Clear);
        // The 26 directions run in lexicographic order of their integer triples, so the answer is
        // (-1, -1, -1) and not the straight drop that would read as the obvious way out.
        CHECK(access.direction.isApprox(Vec3d(-1., -1., -1.).normalized()));
        // The plate the contact hangs off is what the probe starts against, and it is excused only
        // within the probe radius of the contact: a direction that runs along the plate's underside
        // stays against it the whole way and is not clear.
        CHECK(access.direction.z() < 0.);
    }

    SECTION("a closed cavity is blocked, and a window through one wall is the way out")
    {
        const indexed_triangle_set shut = cavity(0.);
        const AABBMesh             closed(shut);
        CHECK(RemovalAccess::assess_access(closed, no_support, no_layers, Vec3d(3., 3., 3.), probe).status ==
              RemovalAccess::Access::Status::Blocked);

        const indexed_triangle_set     open_wall = cavity(1.2);
        const AABBMesh                 windowed(open_wall);
        const RemovalAccess::Access access =
            RemovalAccess::assess_access(windowed, no_support, no_layers, Vec3d(3., 3., 3.), probe);
        REQUIRE(access.status == RemovalAccess::Access::Status::Clear);
        // Five axis directions come before (1, 0, 0) in the order and meet the cavity's walls square-on,
        // and the diagonals before it meet them at an angle, so the 1.2 mm window is found as the axis
        // it is on.
        CHECK(access.direction.isApprox(Vec3d(1., 0., 0.)));
    }

    SECTION("a thin limb across the window blocks the direction its centre ray misses")
    {
        indexed_triangle_set its = cavity(1.2);
        // 0.1 mm thick, 0.19 mm off the axis the probe runs along: the centre line passes it and the
        // 0.3 mm capsule does not.
        add_box(its, Vec3d(5.2, 2.4, 3.19), Vec3d(0.2, 1.2, 0.1));
        const AABBMesh mesh(its);
        REQUIRE_FALSE(mesh.query_ray_hit(Vec3d(3. + probe, 3., 3.), Vec3d(1., 0., 0.)).is_hit());
        CHECK(RemovalAccess::assess_access(mesh, no_support, no_layers, Vec3d(3., 3., 3.), probe).status ==
              RemovalAccess::Access::Status::Blocked);
    }

    SECTION("what cannot be measured is unknown, and unknown is never blocked")
    {
        const indexed_triangle_set plate = open_plate();
        const AABBMesh             mesh(plate);
        const auto     unknown = [](const RemovalAccess::Access &access) {
            return access.status == RemovalAccess::Access::Status::Unknown && access.direction == Vec3d::Zero();
        };
        // A mesh with no triangles at all.
        const indexed_triangle_set nothing;
        const AABBMesh             empty(nothing);
        CHECK(unknown(RemovalAccess::assess_access(empty, no_support, no_layers, Vec3d(5., 5., 8.), probe)));
        // A nonfinite contact, and a probe radius that is not a radius.
        const double nan = std::numeric_limits<double>::quiet_NaN();
        CHECK(unknown(RemovalAccess::assess_access(mesh, no_support, no_layers, Vec3d(5., nan, 8.), probe)));
        CHECK(unknown(RemovalAccess::assess_access(mesh, no_support, no_layers, Vec3d(5., 5., 8.), 0.)));
        CHECK(unknown(RemovalAccess::assess_access(mesh, no_support, no_layers, Vec3d(5., 5., 8.), nan)));
        // Support layers whose two halves do not describe the same stack.
        CHECK(unknown(RemovalAccess::assess_access(mesh, stacked(rect_mm(0., 0., 1., 1.), 2), layer_tops(1., 0.2, 3),
                                                      Vec3d(5., 5., 8.), probe)));
        // A contact inside solid material: every probe would start buried, which is a start that
        // cannot leave the source patch rather than a way out that does not exist.
        indexed_triangle_set solid;
        add_box(solid, Vec3d(0., 0., 0.), Vec3d(6., 6., 6.));
        const AABBMesh buried(solid);
        CHECK(unknown(RemovalAccess::assess_access(buried, no_support, no_layers, Vec3d(3., 3., 3.), probe)));
    }
}

TEST_CASE("Printed support blocks removal access except the contact's own material at the contact", "[RemovalAccess]")
{
    const double               probe = 0.3;
    const indexed_triangle_set plate = open_plate();
    const AABBMesh             mesh(plate);
    const Vec3d                contact(5., 5., 8.);

    // Seven 0.2 mm slabs from z 6.0 to 7.4, well clear of the probe radius around the contact.
    const std::vector<double> tops = layer_tops(6.2, 0.2, 7);

    SECTION("two supports with a corridor between them leave the contact reachable")
    {
        std::vector<ExPolygons> corridor(tops.size());
        for (ExPolygons &layer : corridor)
            layer = ExPolygons{ rect_mm(1., 1., 4.4, 9.), rect_mm(5.6, 1., 9., 9.) };
        const RemovalAccess::Access access =
            RemovalAccess::assess_access(mesh, corridor, tops, contact, probe);
        REQUIRE(access.status == RemovalAccess::Access::Status::Clear);
        CHECK(access.direction.z() < 0.);
    }

    SECTION("the same two supports joined into one cage take the corridor away")
    {
        std::vector<ExPolygons> cage(tops.size());
        for (ExPolygons &layer : cage)
            layer = ExPolygons{ rect_mm(1., 1., 9., 9.) };
        CHECK(RemovalAccess::assess_access(mesh, cage, tops, contact, probe).status ==
              RemovalAccess::Access::Status::Blocked);
    }

    SECTION("the contact's own material inside the probe radius is not what stops it")
    {
        // Two slabs spanning z 7.85..7.95 carrying a 0.2 mm square under the contact: every point of
        // it lies inside the 0.3 mm sphere around the contact, which is the source component the
        // probe starts on.
        const std::vector<double> tip_tops = { 7.9, 7.95 };
        const std::vector<ExPolygons> tip = stacked(rect_mm(4.9, 4.9, 5.1, 5.1), tip_tops.size());
        CHECK(RemovalAccess::assess_access(mesh, tip, tip_tops, contact, probe).status ==
              RemovalAccess::Access::Status::Clear);

        // The same component carried on down to the plate is material outside that sphere, and it
        // stops the probe like any other support does.
        std::vector<ExPolygons> column = tip;
        std::vector<double>     column_tops = tip_tops;
        for (double z = 7.5; z > 5.9; z -= 0.2) {
            column.insert(column.begin(), ExPolygons{ rect_mm(3.5, 3.5, 6.5, 6.5) });
            column_tops.insert(column_tops.begin(), z);
        }
        REQUIRE(column.size() == column_tops.size());
        CHECK(RemovalAccess::assess_access(mesh, column, column_tops, contact, probe).status ==
              RemovalAccess::Access::Status::Blocked);
    }
}
