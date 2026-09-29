#include <catch2/catch_all.hpp>

#include "libslic3r/AABBTreeIndirect.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Format/STL.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/MeshBoolean.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Preset.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Support/PresupportedConversion.hpp"
#include "libslic3r/Support/ScaffoldSupport.hpp"
#include "libslic3r/Support/SupportAnalysis.hpp"
#include "libslic3r/Support/SupportComponents.hpp"
#include "libslic3r/Support/SupportParameters.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <set>
#include <sstream>
#include <string>

#include <nlohmann/json.hpp>
#include <openssl/evp.h>

#include "support_validation.hpp"
#include "test_helpers.hpp"

template<> struct Catch::StringMaker<Slic3r::ScaffoldTipResult>
{
    static std::string convert(Slic3r::ScaffoldTipResult result)
    {
        static const char *names[] = { "Routed", "Filtered", "Unrouted", "Neck", "Merged", "Wall" };
        return names[size_t(result)];
    }
};

using namespace Slic3r::Test;
using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {

// A 6.4 x 2 x 4 mm wall at x -0.9..5.5, y -2.3..-0.3 carrying, off its y -0.3 face, a 4.6 x 0.6 x 1 mm bar
// at x 0..4.6, y -0.3..0.3, z 2.2..3.2: bar_fixture in test_miniature_contacts.cpp without its two pads.
TriangleMesh wall_bar_fixture()
{
    TriangleMesh wall = make_cube(6.4, 2., 4.);
    wall.translate(-0.9f, -2.3f, 0.f);
    TriangleMesh bar = make_cube(4.6, 0.6, 1.);
    bar.translate(0.f, -0.3f, 2.2f);
    wall.merge(bar);
    return wall;
}

// A 10 x 10 x 8 mm block at x 0..10, y 0..10 standing on the plate, and three bodies that start in mid-air at z 3:
// cube A, 4 x 4 x 4 at x 14..18, y 3..7, z 3..7; stick B, 3 x 0.6 x 2 at x 22..25, y 4.7..5.3, z 3..5; post C,
// 0.8 x 0.8 x 2 at x 30..30.8, y 4.6..5.4, z 3..5. A 6 x 2 x 1 bar at x 9..15, y 4..6, z 6..7 reaches into the
// block's +x face at x 10 and cube A's -x face at x 14, so A meets the rooted block through it at z 6; B and C
// never meet it.
TriangleMesh islands_fixture()
{
    TriangleMesh block = make_cube(10., 10., 8.);
    TriangleMesh cube  = make_cube(4., 4., 4.);
    cube.translate(14.f, 3.f, 3.f);
    TriangleMesh bar = make_cube(6., 2., 1.);
    bar.translate(9.f, 4.f, 6.f);
    TriangleMesh stick = make_cube(3., 0.6, 2.);
    stick.translate(22.f, 4.7f, 3.f);
    TriangleMesh post = make_cube(0.8, 0.8, 2.);
    post.translate(30.f, 4.6f, 3.f);
    block.merge(cube);
    block.merge(bar);
    block.merge(stick);
    block.merge(post);
    return block;
}

// The islands fixture's block with a 4 x 4 x 4 mm cube at x 14..18, y 3..7, z 3.5..7.5 that a 6 x 2 x 1 mm bar at
// x 9..15, y 4..6, z 3.5..4.5 joins to the block at the cube's first layer. Under the cube a 0.4 x 0.4 x 0.5 mm spike
// at x 14..14.4, y 4.8..5.2, z 3..3.5 starts an island that joins the rooted body 0.6 mm up, too thin for a line, so
// the front half seeds nothing under it. A 0.3 x 0.3 x 0.2 mm speck at x 35..35.3, y 5..5.3, z 3..3.2 fills one
// slab and never joins. A 4 x 0.2 x 0.6 mm sliver at x 3..7, y 10.15..10.35, z 5..5.6 hangs 0.15 mm off the block's
// +y face, inside its 0.35 mm xy band, until a 4 x 0.45 x 0.2 mm ledge at y 9.9..10.35, z 5.6..5.8 joins it to the
// block 0.6 mm up. A second sliver, 4 x 0.2 x 1.4 mm at x 3..7, y -0.35..-0.15, z 4..5.4, hangs as far off the block's
// -y face until a 4 x 0.45 x 0.2 mm ledge at y -0.35..0.1, z 5.4..5.6 joins it 1.4 mm up, too high for the wall to hold
// it. A floating part that never meets the rooted body stands at x 37..48: a 4 x 4 x 3 mm cube at
// x 40..44, y 3..7, z 3..6 carries off its +x face a 4 x 0.4 x 0.4 mm arm at x 44..48, y 4.8..5.2, z 5.6..6, and under
// the arm's end a 0.4 x 0.4 x 0.4 mm leg at x 47.4..47.8, y 4.8..5.2, z 5.2..5.6 starts an island that the arm merges
// into the cube's 0.4 mm up. Arm and leg are too thin for a line, so the front half seeds nothing under them. A
// 3 x 3 x 0.4 mm flange off the cube's -x face at x 37..40, y 3.5..6.5, z 5.6..6 does get contacts, and they carry
// the planned support layers, which end at the highest contact, above the leg.
TriangleMesh seeded_islands_fixture()
{
    TriangleMesh block = make_cube(10., 10., 8.);
    TriangleMesh cube  = make_cube(4., 4., 4.);
    cube.translate(14.f, 3.f, 3.5f);
    TriangleMesh bar = make_cube(6., 2., 1.);
    bar.translate(9.f, 4.f, 3.5f);
    TriangleMesh spike = make_cube(0.4, 0.4, 0.5);
    spike.translate(14.f, 4.8f, 3.f);
    TriangleMesh debris = make_cube(0.3, 0.3, 0.2);
    debris.translate(35.f, 5.f, 3.f);
    TriangleMesh sliver = make_cube(4., 0.2, 0.6);
    sliver.translate(3.f, 10.15f, 5.f);
    TriangleMesh ledge = make_cube(4., 0.45, 0.2);
    ledge.translate(3.f, 9.9f, 5.6f);
    TriangleMesh tall_sliver = make_cube(4., 0.2, 1.4);
    tall_sliver.translate(3.f, -0.35f, 4.f);
    TriangleMesh tall_ledge = make_cube(4., 0.45, 0.2);
    tall_ledge.translate(3.f, -0.35f, 5.4f);
    TriangleMesh part = make_cube(4., 4., 3.);
    part.translate(40.f, 3.f, 3.f);
    TriangleMesh arm = make_cube(4., 0.4, 0.4);
    arm.translate(44.f, 4.8f, 5.6f);
    TriangleMesh leg = make_cube(0.4, 0.4, 0.4);
    leg.translate(47.4f, 4.8f, 5.2f);
    TriangleMesh flange = make_cube(3., 3., 0.4);
    flange.translate(37.f, 3.5f, 5.6f);
    block.merge(cube);
    block.merge(bar);
    block.merge(spike);
    block.merge(debris);
    block.merge(sliver);
    block.merge(ledge);
    block.merge(tall_sliver);
    block.merge(tall_ledge);
    block.merge(part);
    block.merge(arm);
    block.merge(leg);
    block.merge(flange);
    return block;
}

// A 6 x 6 x 14 mm column at x 0..6, y 0..6 carrying a 6 x 0.6 x 1 mm bar off its +x face at x 6..12, y 2.7..3.3,
// z 4..5 (a 0.6 mm neck) and a 12 x 12 x 2 mm slab off its top at x 6..18, y -3..9, z 12..14 (a 6 mm neck where it
// meets the column).
TriangleMesh shelf_fixture()
{
    TriangleMesh column = make_cube(6., 6., 14.);
    TriangleMesh bar    = make_cube(6., 0.6, 1.);
    bar.translate(6.f, 2.7f, 4.f);
    TriangleMesh slab = make_cube(12., 12., 2.);
    slab.translate(6.f, -3.f, 12.f);
    column.merge(bar);
    column.merge(slab);
    return column;
}

// A 6 x 6 x 30 mm column at x 0..6, y 0..6 carrying a 12 x 12 x 2 mm slab off its top at x 6..18, y -3..9, z 28..30:
// pillars under the slab stand about 27 mm with no bridge, 23 diameters at 1.2 mm.
TriangleMesh tall_shelf_fixture()
{
    TriangleMesh column = make_cube(6., 6., 30.);
    TriangleMesh slab   = make_cube(12., 12., 2.);
    slab.translate(6.f, -3.f, 28.f);
    column.merge(slab);
    return column;
}

// The config every scaffold journey slices under. The miniature checkbox and plate-only rooting are off, so what
// the style forces on is the style's own doing. The printable area is centred because init_print leaves the
// instance on the origin.
DynamicPrintConfig scaffold_config(std::initializer_list<ConfigBase::SetDeserializeItem> extra = {})
{
    DynamicPrintConfig config = fixture_config({ { "support_style", "tree_scaffold" },
                                                 { "printable_area", "-100x-100,100x-100,100x100,-100x100" },
                                                 { "tree_support_branch_diameter", "1.2" },
                                                 { "support_remove_small_overhang", "0" },
                                                 { "support_miniature_contacts", "0" },
                                                 { "support_on_build_plate_only", "0" },
                                                 { "support_top_z_distance", "0.2" } });
    config.set_deserialize_strict(extra);
    return config;
}

// Every extrusion in `collection` and the collections nested in it.
void collect_entities(const ExtrusionEntityCollection &collection, std::vector<const ExtrusionEntity *> &out)
{
    for (const ExtrusionEntity *e : collection.entities)
        if (e->is_collection())
            collect_entities(*static_cast<const ExtrusionEntityCollection *>(e), out);
        else
            out.push_back(e);
}

// What a support layer's extrusions of `role` cover, as one union.
ExPolygons role_footprint(const SupportLayer &sl, ExtrusionRole role)
{
    std::vector<const ExtrusionEntity *> entities;
    collect_entities(sl.support_fills, entities);
    Polygons covered;
    for (const ExtrusionEntity *e : entities)
        if (e->role() == role)
            e->polygons_covered_by_width(covered, 0.f);
    return union_ex(covered);
}

// The highest support layer whose top is at or under `z`, or npos.
template<class Layers> size_t top_layer_under(const Layers &layers, double z)
{
    size_t found = size_t(-1);
    for (size_t i = 0; i < layers.size(); ++ i)
        if (layers[i]->print_z <= z + EPSILON)
            found = i;
    return found;
}

// A rectangle given in the fixture's own mm, placed in the object's centred frame. `origin` is where the fixture's
// point (0, 0) landed.
struct FixtureBox
{
    Point  origin;
    double x0, y0, x1, y1;

    bool contains(const Point &p) const
    {
        const Vec2d q = (p - origin).cast<double>() * SCALING_FACTOR;
        return q.x() >= x0 && q.x() <= x1 && q.y() >= y0 && q.y() <= y1;
    }
    Polygon polygon() const
    {
        Polygon poly({ Point::new_scale(x0, y0), Point::new_scale(x1, y0), Point::new_scale(x1, y1), Point::new_scale(x0, y1) });
        poly.translate(origin);
        return poly;
    }
};

// The object's slices on every object layer that overlaps the height `sl` prints across.
ExPolygons model_over(const PrintObject &object, const SupportLayer &sl)
{
    ExPolygons model;
    for (const Layer *layer : object.layers())
        if (std::min(layer->print_z, sl.print_z) - std::max(layer->bottom_z(), sl.print_z - sl.height) > EPSILON)
            append(model, layer->lslices);
    return model;
}

// Every extrusion role the support layers print.
template<class Layers> std::set<ExtrusionRole> support_roles(const Layers &layers)
{
    std::set<ExtrusionRole> roles;
    for (const SupportLayer *sl : layers) {
        std::vector<const ExtrusionEntity *> entities;
        collect_entities(sl->support_fills, entities);
        for (const ExtrusionEntity *e : entities)
            roles.insert(e->role());
    }
    return roles;
}

// What a support layer's extrusions cover.
ExPolygons footprint(const SupportLayer &sl) { return union_ex(sl.support_fills.polygons_covered_by_width(0.f)); }

double area_mm2(const ExPolygons &polys)
{
    double a = 0.;
    for (const ExPolygon &p : polys)
        a += p.area() * SCALING_FACTOR * SCALING_FACTOR;
    return a;
}

// How many of a support layer's extrusions reach further into its base area than a second wall of width `w` would.
size_t reaches_inside(const SupportLayer &sl, double w)
{
    const ExPolygons interior = offset_ex(sl.base_areas, -scale_(2.5 * w));
    std::vector<const ExtrusionEntity *> entities;
    collect_entities(sl.support_fills, entities);
    size_t count = 0;
    for (const ExtrusionEntity *e : entities) {
        Polygons covered;
        e->polygons_covered_by_width(covered, 0.f);
        if (! intersection_ex(covered, interior).empty())
            ++ count;
    }
    return count;
}

// A disc's width is the diameter of the largest circle its outer contour inscribes; holes are ignored on purpose,
// since a printed disc is a ring. The inscribed circle is one disc's whatever fused with it, and a tilted head's
// slice, an ellipse along the tilt, inscribes its short axis, the sphere chord the grade sets. The contour is
// closed by half a line width first: the line ends notch it, and a notch shrinks the circle a whole disc holds.
struct Disc { double width_mm, box_min_mm, box_max_mm; Point centroid; };
std::vector<Disc> discs(const SupportLayer &sl, double w)
{
    std::vector<Disc> out;
    for (const ExPolygon &poly : role_footprint(sl, erSupportMaterialInterface)) {
        const BoundingBox box    = get_extents(poly.contour);
        const Polygons    closed = offset(offset(poly.contour, scale_(0.5 * w)), -scale_(0.5 * w));
        double lo = 0., hi = 0.5 * unscale<double>(std::max(box.size().x(), box.size().y()));
        for (int i = 0; i < 24; ++ i) {
            const double mid = 0.5 * (lo + hi);
            (offset(closed, -scale_(mid)).empty() ? hi : lo) = mid;
        }
        out.push_back({ 2. * lo, unscale<double>(std::min(box.size().x(), box.size().y())),
                        unscale<double>(std::max(box.size().x(), box.size().y())), poly.contour.centroid() });
    }
    return out;
}

// The interface discs on one of the four support layers under a tip: how many stand in the tip's region, and on the
// tip's own layer how many of those are graded, standing outside the excluded box, with the narrowest and the widest.
struct RingLayer
{
    double print_z;
    size_t in_region = 0, graded = 0;
    Disc   narrowest { std::numeric_limits<double>::max(), 0., 0., Point() };
    Disc   widest { std::numeric_limits<double>::lowest(), 0., 0., Point() };
};

// The rings on `layers[top]`, the highest support layer whose top reaches the tip, and the three layers below it, in
// that order. `top` must be 3 or more.
template<class Layers>
std::vector<RingLayer> rings_under(const Layers &layers, size_t top, const FixtureBox &region, const FixtureBox *excluded, double w)
{
    std::vector<RingLayer> out;
    for (size_t k = 0; k < 4; ++ k) {
        const SupportLayer &sl = *layers[top - k];
        RingLayer           ring { sl.print_z };
        for (const Disc &disc : discs(sl, w)) {
            if (! region.contains(disc.centroid))
                continue;
            ++ ring.in_region;
            if (k == 0 && ! (excluded != nullptr && excluded->contains(disc.centroid))) {
                ++ ring.graded;
                if (disc.width_mm < ring.narrowest.width_mm)
                    ring.narrowest = disc;
                if (disc.width_mm > ring.widest.width_mm)
                    ring.widest = disc;
            }
        }
        out.push_back(ring);
    }
    return out;
}

// Where an enforced tip's head may enter the xy band: `area` on the layers at most a reach under `tip_z`.
struct Reach { Polygon area; double tip_z; };

// The areas of the reaches whose tip stands at or over `print_z` and at most `reach_z` above it.
Polygons reach_at(const std::vector<Reach> &reaches, double print_z, double reach_z)
{
    Polygons reach;
    for (const Reach &spot : reaches)
        if (print_z <= spot.tip_z + EPSILON && print_z > spot.tip_z - reach_z)
            reach.push_back(spot.area);
    return reach;
}

// Stands every object of `model` in its mesh's own orientation, centred on the bed and resting on it.
void stand_upright(Model &model, const DynamicPrintConfig &config)
{
    for (ModelObject *mo : model.objects)
        for (ModelInstance *instance : mo->instances)
            instance->set_rotation(Vec3d::Zero());
    model.center_instances_around_point(unscale(BoundingBox(get_bed_shape(config)).center()));
    for (ModelObject *mo : model.objects)
        mo->ensure_on_bed();
}

// A 40 x 40 x 4 mm block, a 4 x 4 x 10 mm post on it at x 18..22, y 0..4, z 4..14, and a 4 x 40 x 2 mm lip off the
// post's top at x 18..22, y 0..40, z 12..14: the lip's underside hangs 8 mm over the block, which reaches 18 mm past
// it on both x sides, so a tip under the lip's middle has no ground within the 12 mm scaffold bridge length.
TriangleMesh blocked_lip_fixture()
{
    TriangleMesh block = make_cube(40., 40., 4.);
    TriangleMesh post  = make_cube(4., 4., 10.);
    post.translate(18.f, 0.f, 4.f);
    TriangleMesh lip = make_cube(4., 40., 2.);
    lip.translate(18.f, 0.f, 12.f);
    block.merge(post);
    block.merge(lip);
    return block;
}

// A 20 x 20 x 12 mm box open at the top, at x 0..20, y 0..20: a 1 mm floor and four 1 mm walls. A head inside it faces
// the floor, and a pillar or a bridge only walks down, never over a wall, so nothing inside routes to the pad.
TriangleMesh open_box_fixture()
{
    TriangleMesh box   = make_cube(20., 20., 1.);
    TriangleMesh south = make_cube(20., 1., 12.);
    TriangleMesh north = make_cube(20., 1., 12.);
    north.translate(0.f, 19.f, 0.f);
    TriangleMesh west = make_cube(1., 20., 12.);
    TriangleMesh east = make_cube(1., 20., 12.);
    east.translate(19.f, 0.f, 0.f);
    for (const TriangleMesh *wall : { &south, &north, &west, &east })
        box.merge(*wall);
    return box;
}

// The open box with a 20 x 2 x 1 mm bar across its top at y 9..11, z 12..13, lying on the walls at x 0..1 and 19..20,
// and a 1 x 1 mm rod hanging from the bar's middle at x 9.5..10.5, y 9.5..10.5 down to z 6: the rod starts in mid-air
// 5 mm over the floor and meets the rooted box through the bar 6 mm up.
TriangleMesh boxed_rod_fixture()
{
    TriangleMesh box = open_box_fixture();
    TriangleMesh bar = make_cube(20., 2., 1.);
    bar.translate(0.f, 9.f, 12.f);
    TriangleMesh rod = make_cube(1., 1., 6.);
    rod.translate(9.5f, 9.5f, 6.f);
    box.merge(bar);
    box.merge(rod);
    return box;
}

// The open box with a 3 x 8 x 1 mm ledge off its west wall's inner face at x 1..4, y 6..14, z 6..7: the ledge hangs
// 3 mm from the wall, past the reach, over the floor.
TriangleMesh boxed_ledge_fixture()
{
    TriangleMesh box   = open_box_fixture();
    TriangleMesh ledge = make_cube(3., 8., 1.);
    ledge.translate(1.f, 6.f, 6.f);
    box.merge(ledge);
    return box;
}

// A 6 x 6 x 14 mm column at x 0..6, y 0..6 carrying a 6 x 6 x 2 mm slab off its +x face at x 6..12, z 12..14, and a
// 0.3 x 1 mm rod hanging from the slab 0.15 mm off the column's face, at x 6.15..6.45, y 2.5..3.5, down to z 6: the rod
// starts in mid-air beside the wall, and a neck straight down from any point of it ends within the column's xy band.
TriangleMesh leaning_rod_fixture()
{
    TriangleMesh column = make_cube(6., 6., 14.);
    TriangleMesh slab   = make_cube(6., 6., 2.);
    slab.translate(6.f, 0.f, 12.f);
    TriangleMesh rod = make_cube(0.3, 1., 6.);
    rod.translate(6.15f, 2.5f, 6.f);
    column.merge(slab);
    column.merge(rod);
    return column;
}

// A 6 x 6 x 14 mm column at x 0..6, y 0..6 carrying off its +x face a plank 2 mm thick whose underside falls at
// 45 degrees from z 10 at the face, `length_mm` along x, across y 0..6. A head on the underside aims along the
// underside's normal, down and back toward the column, so a tip under a millimetre from the face tilts its neck into
// the column's xy band a few layers under its rings.
TriangleMesh slope_fixture(double length_mm)
{
    TriangleMesh       column = make_cube(6., 6., 14.);
    std::vector<Vec3f> corners;
    for (const float y : { 0.f, 6.f })
        for (const float z : { 0.f, 2.f }) {
            corners.emplace_back(5.f, y, 11.f + z);
            corners.emplace_back(float(6. + length_mm), y, float(10. - length_mm) + z);
        }
    column.merge(TriangleMesh(its_convex_hull(corners)));
    return column;
}

// The hull of a sphere of radius `r0` at `a` and one of `r1` at `b`, each sphere's poles on the line through both:
// an artist tip where the radii differ, a strut piece where they match.
TriangleMesh swept_spheres(const Vec3d &a, double r0, const Vec3d &b, double r1)
{
    const Eigen::Quaterniond turn = Eigen::Quaterniond::FromTwoVectors(Vec3d::UnitZ(), b - a);
    std::vector<Vec3f>       points;
    for (const auto &[centre, r] : { std::make_pair(a, r0), std::make_pair(b, r1) })
        for (const Vec3f &v : its_make_sphere(r, PI / 8.).vertices)
            points.push_back((centre + turn * v.cast<double>()).cast<float>());
    return TriangleMesh(its_convex_hull(points));
}

// An 8-sided rod of radius `r` from `a` to `b` with flat ends: an artist trunk or brace.
TriangleMesh artist_rod(const Vec3d &a, const Vec3d &b, double r)
{
    TriangleMesh rod(its_make_cylinder(r, (b - a).norm(), PI / 4.));
    Transform3d  place = Transform3d::Identity();
    place.translate(a);
    place.rotate(Eigen::Quaterniond::FromTwoVectors(Vec3d::UnitZ(), b - a));
    rod.transform(place);
    return rod;
}

// An artist tip of contact diameter `diameter` at `site` whose wide end, 1 mm across, stands 3 mm off along `axis`.
struct FixtureTip { Vec3d site, axis; double diameter; };
TriangleMesh artist_tip(const FixtureTip &tip) { return swept_spheres(tip.site, tip.diameter / 2., tip.site + 3. * tip.axis, 0.5); }

// A pre-supported model: a 20 x 12 x 4 mm figure slab at z 6..10, x 0..20, y 0..12, one shell, with two more figure
// shells, an 8-sided 1 mm staff standing on its top face at (4, 8) up to z 18 and a 2 mm eye sphere set in its y 0
// face at (10, 0, 8), both convex like an artist's primitives. Artist tips on the slab's underside each stand on a
// 0.8 mm trunk sunk in a 3 x 3 x 0.5 mm raft pad on the plate: a 0.3 mm tip straight up at (4, 4), written twice; a
// 0.5 mm tip straight up at (10, 6); a 0.3 mm tip at (16, 8) leaning 60 degrees toward -x; a 0.45 mm tip at (7, 9)
// and a 0.4 mm one at (13, 3), either side of the Heavy cut. A brace joins the first two trunks at z 2. On the slab's
// top face a 0.3 mm tip at (16, 3) stands on the figure pointing down, and under the slab a 0.2 mm micro strut, two
// capsules and a rod, joins (2, 10) to (4, 10). Each piece is its own shell, as the artist's closed primitives are.
const FixtureTip presupported_light { Vec3d(4., 4., 6.), -Vec3d::UnitZ(), 0.3 };
const FixtureTip presupported_heavy { Vec3d(10., 6., 6.), -Vec3d::UnitZ(), 0.5 };
const FixtureTip presupported_leaning { Vec3d(16., 8., 6.), Vec3d(-std::sin(PI / 3.), 0., -std::cos(PI / 3.)), 0.3 };
const FixtureTip presupported_at_cut { Vec3d(7., 9., 6.), -Vec3d::UnitZ(), 0.45 };
const FixtureTip presupported_under_cut { Vec3d(13., 3., 6.), -Vec3d::UnitZ(), 0.4 };
const FixtureTip presupported_rooted { Vec3d(16., 3., 10.), Vec3d::UnitZ(), 0.3 };
TriangleMesh presupported_staff() { return artist_rod(Vec3d(4., 8., 10.), Vec3d(4., 8., 18.), 0.5); }
TriangleMesh presupported_eye()
{
    TriangleMesh eye(its_make_sphere(1., PI / 8.));
    eye.translate(10.f, 0.f, 8.f);
    return eye;
}
TriangleMesh presupported_slab()
{
    TriangleMesh slab = make_cube(20., 12., 4.);
    slab.translate(0.f, 0.f, 6.f);
    return slab;
}
// The fixture around `model` in place of the slab, its trunks standing on no pad when `pads` is false.
TriangleMesh presupported_fixture(TriangleMesh model = presupported_slab(), bool pads = true)
{
    model.merge(presupported_staff());
    model.merge(presupported_eye());
    for (const FixtureTip &tip : { presupported_light, presupported_light, presupported_heavy, presupported_leaning, presupported_at_cut,
                                   presupported_under_cut, presupported_rooted })
        model.merge(artist_tip(tip));
    for (const FixtureTip &tip : { presupported_light, presupported_heavy, presupported_leaning, presupported_at_cut, presupported_under_cut }) {
        const Vec3d wide = tip.site + 3. * tip.axis;
        model.merge(artist_rod(Vec3d(wide.x(), wide.y(), 0.4), wide, 0.4));
        if (pads) {
            TriangleMesh pad = make_cube(3., 3., 0.5);
            pad.translate(float(wide.x() - 1.5), float(wide.y() - 1.5), 0.f);
            model.merge(pad);
        }
    }
    model.merge(artist_rod(Vec3d(4., 4., 2.), Vec3d(10., 6., 2.), 0.3));
    model.merge(swept_spheres(Vec3d(2., 10., 6.), 0.1, Vec3d(2.5, 10., 5.4), 0.1));
    model.merge(artist_rod(Vec3d(2.5, 10., 5.4), Vec3d(3.5, 10., 5.4), 0.08));
    model.merge(swept_spheres(Vec3d(3.5, 10., 5.4), 0.1, Vec3d(4., 10., 6.), 0.1));
    return model;
}

// A 6 x 6 x 14 mm column at x 0..6, y 0..6 carrying off its +x face a 6 x 6 mm sheet 0.1 mm thick at x 6..12,
// z 8..8.1, and off its -x face two 2 x 2.5 x 1 mm bars at x -2..0, one at y 0..2.5 from z 8.12, one at y 3.5..6 from
// z 8.18. At 0.06 mm layers over a 0.2 mm first layer each underside tops an object layer, and a scaffold contact,
// which has no top gap, bounds a planned layer at its z: the bars plan a support layer at z 8.12..8.18, over the sheet
// but under the 0.22 mm a head's pin reaches over its tip, so there the pin of a head under the sheet comes out above
// the sheet with no sheet in that layer's band.
TriangleMesh thin_sheet_fixture()
{
    TriangleMesh column = make_cube(6., 6., 14.);
    TriangleMesh sheet  = make_cube(6., 6., 0.1);
    sheet.translate(6.f, 0.f, 8.f);
    TriangleMesh low_bar = make_cube(2., 2.5, 1.);
    low_bar.translate(-2.f, 0.f, 8.12f);
    TriangleMesh high_bar = make_cube(2., 2.5, 1.);
    high_bar.translate(-2.f, 3.5f, 8.18f);
    column.merge(sheet);
    column.merge(low_bar);
    column.merge(high_bar);
    return column;
}

// A 12 x 20 x 3 mm shelf at x -10..2, y -10..10, a 2 x 6 x 7 mm column on it at x -10..-8, y -3..3, z 3..10, a bar
// off the column's top at x -10..1, z 9..10, and a 2 x 6 x 1.6 mm lip hanging from the bar at x -1..1, y -3..3,
// z 7.4..9: the lip starts in mid-air and joins the column through the bar. A head straight down from (0, 0, 7.4)
// hangs its junction 2.4 mm over the shelf, 2 mm in from its +x edge, and every walk from there meets the shelf before
// its scan ring clears the edge; leaning 45 degrees toward +x its walk starts higher and further out and clears it.
TriangleMesh island_lip_fixture()
{
    TriangleMesh shelf = make_cube(12., 20., 3.);
    shelf.translate(-10.f, -10.f, 0.f);
    TriangleMesh column = make_cube(2., 6., 7.);
    column.translate(-10.f, -3.f, 3.f);
    TriangleMesh bar = make_cube(11., 6., 1.);
    bar.translate(-10.f, -3.f, 9.f);
    TriangleMesh lip = make_cube(2., 6., 1.6);
    lip.translate(-1.f, -3.f, 7.4f);
    for (const TriangleMesh *part : { &column, &bar, &lip })
        shelf.merge(*part);
    return shelf;
}

// A 24 x 12 x 4 mm base at x 0..24, y 0..12, a 2 x 2 x 6 mm column on it at x 0..2, y 5..7, z 4..10, and a 16 x 6 x
// 1 mm bar off the column's top at x 0..16, y 3..9, z 9..10, from which two bodies hang: a 4 x 4 mm block at x 4..8,
// y 4..8 down to z 6, standing free 3 mm before it meets the bar, and a 1 x 1 mm rod at x 11.5..12.5, y 5.5..6.5 down
// to z 4.6, 0.6 mm over the base. A neck from the block's underside ends clear of the base, while one from the rod's,
// straight down or leaning up to 45 degrees, ends inside it.
TriangleMesh hanging_islands_fixture()
{
    TriangleMesh base   = make_cube(24., 12., 4.);
    TriangleMesh column = make_cube(2., 2., 6.);
    column.translate(0.f, 5.f, 4.f);
    TriangleMesh bar = make_cube(16., 6., 1.);
    bar.translate(0.f, 3.f, 9.f);
    TriangleMesh block = make_cube(4., 4., 3.);
    block.translate(4.f, 4.f, 6.f);
    TriangleMesh rod = make_cube(1., 1., 4.4);
    rod.translate(11.5f, 5.5f, 4.6f);
    for (const TriangleMesh *part : { &column, &bar, &block, &rod })
        base.merge(*part);
    return base;
}

// The hanging islands' base, column and bar, with a tee hanging from the bar: a 6 x 6 x 1.6 mm ledge at x 5..11, y 3..9,
// z 7.4..9, and under its middle a 1.5 x 1.5 mm stem at x 7.25..8.75, y 5.25..6.75 down to z 6. The tee is one island,
// born at the stem's bottom 2 mm over the base, and its ledge hangs 1.4 mm higher around the stem.
TriangleMesh hanging_tee_fixture()
{
    TriangleMesh base   = make_cube(24., 12., 4.);
    TriangleMesh column = make_cube(2., 2., 6.);
    column.translate(0.f, 5.f, 4.f);
    TriangleMesh bar = make_cube(16., 6., 1.);
    bar.translate(0.f, 3.f, 9.f);
    TriangleMesh ledge = make_cube(6., 6., 1.6);
    ledge.translate(5.f, 3.f, 7.4f);
    TriangleMesh stem = make_cube(1.5, 1.5, 1.4);
    stem.translate(7.25f, 5.25f, 6.f);
    for (const TriangleMesh *part : { &column, &bar, &ledge, &stem })
        base.merge(*part);
    return base;
}

// The hanging islands' column on a 20 x 12 x 4 mm base at x 0..20, y 0..12, a 28 x 6 x 1 mm bar off the column's top at
// x 0..28, y 3..9, z 9..10, and a 12 x 4 x 3 mm slab hanging from the bar at x 14..26, y 4..8, down to z 6: the slab is
// one island, born 2 mm over the base at x 14..20 and 6 mm over the bed past the base's +x edge. A head under the slab
// 4 mm in from that edge has no route, since no pillar stands on the model and every walk down from its junction meets
// the base before it clears the edge, while one 4 mm past the edge drops straight to the pad.
TriangleMesh overhung_slab_fixture()
{
    TriangleMesh base   = make_cube(20., 12., 4.);
    TriangleMesh column = make_cube(2., 2., 6.);
    column.translate(0.f, 5.f, 4.f);
    TriangleMesh bar = make_cube(28., 6., 1.);
    bar.translate(0.f, 3.f, 9.f);
    TriangleMesh slab = make_cube(12., 4., 3.);
    slab.translate(14.f, 4.f, 6.f);
    for (const TriangleMesh *part : { &column, &bar, &slab })
        base.merge(*part);
    return base;
}

// The overhung slab with a 2.5 x 4 x 2 mm ledge off the slab's +x end at x 25.5..28, y 4..8, z 7..9, and under the ledge
// a 0.6 x 0.6 x 0.07 mm nub 0.2 mm off the slab's +x face, at x 26.2..26.8, y 5.7..6.3, from z 6.93. At 0.05 mm layers
// the nub prints one layer, z 6.95..7, before the ledge takes it into the slab's part, and all of it lies within its hang
// of the slab under that layer: it hangs from whatever holds the slab.
TriangleMesh overhung_nub_fixture()
{
    TriangleMesh mesh  = overhung_slab_fixture();
    TriangleMesh ledge = make_cube(2.5, 4., 2.);
    ledge.translate(25.5f, 4.f, 7.f);
    TriangleMesh nub = make_cube(0.6, 0.6, 0.07);
    nub.translate(26.2f, 5.7f, 6.93f);
    mesh.merge(ledge);
    mesh.merge(nub);
    return mesh;
}

// A `sx` x `sy` x `sz` mm box with its lowest corner at (x, y, z).
TriangleMesh box(double x, double y, double z, double sx, double sy, double sz)
{
    TriangleMesh mesh = make_cube(sx, sy, sz);
    mesh.translate(float(x), float(y), float(z));
    return mesh;
}

// The overhung slab's base, column and bar with the slab split at x 19.5..20.5 into two slabs at z 6..8.5, one over the
// base and one past its edge, that a block at x 14..26, z 8.5..9 joins before the bar takes their part into the column's
// at z 9. Two 0.6 x 0.6 x 0.07 mm nubs at y 5.7..6.3 each stand 0.2 mm off a face under a ledge at y 4..8 that takes it
// in one 0.05 mm layer up: one at x 26.2..26.8 from z 8.73, off the block's +x face, under a ledge at x 26..27.5,
// z 8.8..9, where it hangs from the two slabs' part, and one at x 28.2..28.8 from z 9.43, off the bar's +x end, under a
// ledge at x 28..30.5, z 9.5..10, where it hangs from the part the bed roots.
TriangleMesh joined_nubs_fixture()
{
    TriangleMesh mesh = make_cube(20., 12., 4.);
    for (const TriangleMesh &part : { box(0., 5., 4., 2., 2., 6.), box(0., 3., 9., 28., 6., 1.), box(14., 4., 6., 5.5, 4., 2.5),
                                      box(20.5, 4., 6., 5.5, 4., 2.5), box(14., 4., 8.5, 12., 4., 0.5), box(26., 4., 8.8, 1.5, 4., 0.2),
                                      box(26.2, 5.7, 8.73, 0.6, 0.6, 0.07), box(28., 4., 9.5, 2.5, 4., 0.5),
                                      box(28.2, 5.7, 9.43, 0.6, 0.6, 0.07) })
        mesh.merge(part);
    return mesh;
}

// A 4 x 4 x 10 mm column carrying a 12 x 10 x 2 mm slab at z 8..10, and under the slab two 0.6 x 0.6 x 0.07 mm nubs at
// x 7..7.6 and 7.9..8.5, y 6..6.6, from z 5, which a strand at x 7..8.5 from z 5.07 joins one 0.05 mm layer up and
// carries into the slab: they meet only each other.
TriangleMesh paired_nubs_fixture()
{
    TriangleMesh mesh = make_cube(4., 4., 10.);
    for (const TriangleMesh &part : { box(0., 0., 8., 12., 10., 2.), box(7., 6., 5., 0.6, 0.6, 0.07), box(7.9, 6., 5., 0.6, 0.6, 0.07),
                                      box(7., 6., 5.07, 1.5, 0.6, 2.93) })
        mesh.merge(part);
    return mesh;
}

// A JSON config written to the OS temp directory and removed when the guard leaves scope.
struct ScratchJson
{
    std::filesystem::path path;

    ScratchJson(const std::string &name, const std::string &body) : path(std::filesystem::temp_directory_path() / name)
    {
        std::ofstream out(path);
        out << body;
    }
    ~ScratchJson() { std::filesystem::remove(path); }
};

// A second apply of `model` under the config rebuilt the way init_print built it, as paint_and_reapply does.
Print::ApplyStatus reapply(Print &print, Model &model, const DynamicPrintConfig &config)
{
    DynamicPrintConfig full = DynamicPrintConfig::full_print_config();
    full.apply(config);
    full.set_key_value("gcode_comments", new ConfigOptionBool(true));
    return print.apply(model, full);
}

// Slices `mesh` from a baked list of light points on a grid, `step` mm apart over x0..x1 by y0..y1 at z, in the fixture's
// frame: a test of the builder reads the same pillars whatever the automatic placement does.
void process_grid(Print &print, Model &model, const DynamicPrintConfig &config, const TriangleMesh &mesh, double z, double x0, double x1,
                  double y0, double y1, double step)
{
    init_print({ TriangleMesh(mesh) }, print, model, config);
    ModelObject &mo = *model.objects.front();
    for (double x = x0; x <= x1 + EPSILON; x += step)
        for (double y = y0; y <= y1 + EPSILON; y += step)
            mo.scaffold_points.push_back({ Vec3f(float(x), float(y), float(z)), ScaffoldHeadSize::Light, false });
    mo.scaffold_points_status   = ScaffoldPointsStatus::UserModified;
    mo.scaffold_points_pose     = mo.instances.front()->get_matrix().linear();
    mo.scaffold_points_mesh_box = mo.raw_mesh_bounding_box();
    reapply(print, model, config);
    print.process();
}

// Whether the support step of `po` carries a warning whose text holds `text`.
bool warns(const PrintObject &po, const std::string &text)
{
    const std::vector<PrintStateBase::Warning> warnings = po.step_state_with_warnings(posSupportMaterial).warnings;
    return std::any_of(warnings.begin(), warnings.end(),
                       [&text](const PrintStateBase::Warning &w) { return w.message.find(text) != std::string::npos; });
}

// Whether any piece of `area` holds `p`.
bool holds(const ExPolygons &area, const Point &p)
{
    return std::any_of(area.begin(), area.end(), [&p](const ExPolygon &expoly) { return expoly.contains(p); });
}

// A tip at `at`, fixture xy offset by the object's lowest corner above the first layer, which the elephant foot
// compensation may shrink: on the bottom of the first layer from `from_z` up holding it, filed one layer under that
// layer, as a contact under an overhang is.
ScaffoldSupport::TipSite site_under(const PrintObject &object, const Vec2d &at, double from_z = 0.)
{
    const Point p    = get_extents(object.layers()[1]->lslices).min + Point::new_scale(at.x(), at.y());
    size_t      over = 0;
    while (over < object.layer_count() && (object.layers()[over]->bottom_z() < from_z - EPSILON || ! holds(object.layers()[over]->lslices, p)))
        ++ over;
    REQUIRE(over < object.layer_count());
    return { p, object.layers()[over]->bottom_z(), int(over) - 1 };
}

// Whether a bare island of `bare` stands at `site`, within a lattice cell's rounding of the birth's deepest point.
bool bare_at(const std::vector<Vec3d> &bare, const ScaffoldSupport::TipSite &site)
{
    return std::any_of(bare.begin(), bare.end(), [&site](const Vec3d &p) {
        return (p.head<2>() - unscale(site.position)).norm() <= 0.3 && std::abs(p.z() - site.print_z) <= 1e-6;
    });
}

// `draw` on the object's own layers as the planned layers, each clipped by its slices and those grown by the 0.35 mm
// xy distance, with a 0.42 mm line, 1.2 mm pillars and 10 mm bridges.
struct DrawOnLayers
{
    std::vector<LayerHeightData>            plan;
    std::vector<ScaffoldSupport::LayerClip> clips;
    ScaffoldSupport::Params                 params;

    explicit DrawOnLayers(const PrintObject &object)
    {
        for (const Layer *layer : object.layers()) {
            plan.emplace_back(layer->print_z, layer->height, size_t(layer->id()));
            clips.push_back({ layer->lslices, offset_ex(layer->lslices, scale_(0.35)) });
        }
        params.toolpath_width_mm    = 0.42;
        params.pillar_diameter_mm   = 1.2;
        params.xy_distance_mm       = 0.35;
        params.bridge_length_mm     = 10.;
        params.brace_slenderness    = 10.;
        params.max_bridge_length_mm = 10.;
        params.brace_diameter_mm    = 0.84;
        params.interface_width_mm   = 0.42;
        params.base_cover           = [](const ExPolygons &base, double) { return to_polygons(base); };
        params.pad_thickness_mm     = std::find_if(plan.begin(), plan.end(), [](const LayerHeightData &l) { return l.print_z >= 0.6 - EPSILON; })->print_z;
    }
    ScaffoldSupport::Output operator()(const PrintObject &object, const ScaffoldSupport::Tips &tips) const
    {
        return ScaffoldSupport::draw(object, tips, plan, clips, params, [] {});
    }
};

} // namespace

TEST_CASE("A tree scaffold style round-trips through the config and lists last", "[ScaffoldSupport]")
{
    DynamicPrintConfig style;
    style.set_deserialize_strict({ { "support_style", "tree_scaffold" } });
    CHECK(style.opt_enum<SupportMaterialStyle>("support_style") == smsTreeScaffold);
    CHECK(int(smsTreeScaffold) == int(smsTreeHybrid) + 1);
    CHECK(style.opt_serialize("support_style") == "tree_scaffold");

    const ConfigOptionDef *style_def = print_config_def.get("support_style");
    REQUIRE(style_def != nullptr);
    CHECK(style_def->enum_values.back() == "tree_scaffold");
    CHECK(style_def->enum_labels.back() == "Tree Scaffold");

    const DynamicPrintConfig defaults = DynamicPrintConfig::full_print_config();
    CHECK_THAT(defaults.opt_float("scaffold_bridge_length"), WithinAbs(12., 1e-9));
    CHECK_THAT(defaults.opt_float("scaffold_brace_slenderness"), WithinAbs(15., 1e-9));
    CHECK_THAT(defaults.option<ConfigOptionPercent>("scaffold_brace_diameter")->value, WithinAbs(60., 1e-9));

    const ConfigOptionDef *bridge_def = print_config_def.get("scaffold_bridge_length");
    REQUIRE(bridge_def != nullptr);
    CHECK_THAT(bridge_def->min, WithinAbs(3., 1e-9));
    CHECK_THAT(bridge_def->max, WithinAbs(30., 1e-9));
    CHECK(bridge_def->mode == comAdvanced);
    const ConfigOptionDef *brace_def = print_config_def.get("scaffold_brace_slenderness");
    REQUIRE(brace_def != nullptr);
    CHECK_THAT(brace_def->min, WithinAbs(5., 1e-9));
    CHECK_THAT(brace_def->max, WithinAbs(40., 1e-9));
    CHECK(brace_def->mode == comAdvanced);
    CHECK(brace_def->sidetext.empty());
    const ConfigOptionDef *brace_diameter_def = print_config_def.get("scaffold_brace_diameter");
    REQUIRE(brace_diameter_def != nullptr);
    CHECK(brace_diameter_def->type == coPercent);
    CHECK_THAT(brace_diameter_def->min, WithinAbs(10., 1e-9));
    CHECK_THAT(brace_diameter_def->max, WithinAbs(100., 1e-9));
    CHECK(brace_diameter_def->mode == comAdvanced);

    const std::vector<std::string> &options = Preset::print_options();
    CHECK(std::find(options.begin(), options.end(), "scaffold_bridge_length") != options.end());
    CHECK(std::find(options.begin(), options.end(), "scaffold_brace_slenderness") != options.end());
    CHECK(std::find(options.begin(), options.end(), "scaffold_brace_diameter") != options.end());

    // Print::apply needs the Model the print was built from, so the print is built through init_print.
    const DynamicPrintConfig config = fixture_config({ { "support_style", "tree_slim" } });
    Print print;
    Model model;
    init_print({ make_cube(10, 10, 10) }, print, model, config);
    print.process();
    REQUIRE(print.objects().front()->is_step_done(posSupportMaterial));

    DynamicPrintConfig changed = config;
    changed.set_deserialize_strict({ { "scaffold_bridge_length", "20" } });
    CHECK(print.apply(model, changed) != PrintBase::APPLY_STATUS_UNCHANGED);
    CHECK_FALSE(print.objects().front()->is_step_done(posSupportMaterial));
}

TEST_CASE("A project saved with scaffold_density loads without it", "[ScaffoldSupport]")
{
    // Presets and 3MF project and object configs written by a build that had the key still carry it: it loads as an
    // unknown key, with no substitution, and the rest of the file loads.
    DynamicPrintConfig        config = DynamicPrintConfig::full_print_config();
    ConfigSubstitutionContext context(ForwardCompatibilitySubstitutionRule::Enable);
    config.set_deserialize("scaffold_density", "heavy", context);
    CHECK_FALSE(config.has("scaffold_density"));
    CHECK(context.unrecogized_keys == std::vector<std::string>{ "scaffold_density" });
    CHECK(context.substitutions.empty());
    CHECK(print_config_def.get("scaffold_density") == nullptr);
}

TEST_CASE("A normal support type resets the scaffold style to grid", "[ScaffoldSupport]")
{
    // The bar's underside is 0.6 mm wide, under two extrusion widths, so the normal generator would drop it as a
    // small overhang and print no support at all.
    Print print;
    init_and_process_print({ wall_bar_fixture() }, print,
                           fixture_config({ { "support_type", "normal(auto)" },
                                            { "support_style", "tree_scaffold" },
                                            { "support_threshold_angle", "60" },
                                            { "support_remove_small_overhang", "0" } }));
    REQUIRE(print.objects().size() == 1);
    CHECK(SupportParameters(*print.objects().front()).support_style == smsGrid);
    CHECK(print.objects().front()->support_layers().size() > 0);
}

TEST_CASE("An unknown support style and an unknown scaffold key load with a substitution and an unrecognized key", "[ScaffoldSupport]")
{
    const ScratchJson file("orca_scaffold_unknown_style.json", R"({"support_style": "no_such_style", "scaffold_no_such_key": "1"})");

    DynamicPrintConfig                 config = DynamicPrintConfig::full_print_config();
    std::map<std::string, std::string> key_values;
    std::string                        reason;
    const ConfigSubstitutions substitutions =
        config.load_from_json(file.path.string(), ForwardCompatibilitySubstitutionRule::Enable, key_values, reason);
    REQUIRE(substitutions.size() == 1);
    REQUIRE(substitutions.front().opt_def != nullptr);
    CHECK(substitutions.front().opt_def->opt_key == "support_style");
    CHECK(config.opt_enum<SupportMaterialStyle>("support_style") == smsDefault);

    DynamicPrintConfig        context_config = DynamicPrintConfig::full_print_config();
    ConfigSubstitutionContext context(ForwardCompatibilitySubstitutionRule::Enable);
    key_values.clear();
    reason.clear();
    context_config.load_from_json(file.path.string(), context, true, key_values, reason);
    REQUIRE(context.unrecogized_keys.size() == 1);
    CHECK(context.unrecogized_keys.front() == "scaffold_no_such_key");
}

TEST_CASE("A config without the scaffold keys loads with their defaults", "[ScaffoldSupport]")
{
    const ScratchJson file("orca_scaffold_older_build.json", R"({"support_style": "tree_slim", "layer_height": "0.06"})");

    DynamicPrintConfig                 config = DynamicPrintConfig::full_print_config();
    std::map<std::string, std::string> key_values;
    std::string                        reason;
    const ConfigSubstitutions substitutions =
        config.load_from_json(file.path.string(), ForwardCompatibilitySubstitutionRule::Enable, key_values, reason);
    CHECK(substitutions.empty());
    CHECK_THAT(config.opt_float("layer_height"), WithinAbs(0.06, 1e-9));
    CHECK(config.opt_enum<SupportMaterialStyle>("support_style") == smsTreeSlim);
    CHECK_THAT(config.opt_float("scaffold_bridge_length"), WithinAbs(12., 1e-9));
    CHECK_THAT(config.opt_float("scaffold_brace_slenderness"), WithinAbs(15., 1e-9));
    CHECK_THAT(config.option<ConfigOptionPercent>("scaffold_brace_diameter")->value, WithinAbs(60., 1e-9));
}

TEST_CASE("Island joins name the slab each mid-air island first meets the rooted body", "[ScaffoldSupport]")
{
    // 8 mm at fixture_config's 0.2 mm layers and 0.2 mm first layer: 40 slabs, the three islands born at z 3.0..3.2 (slab 15).
    Print print;
    init_and_process_print({ islands_fixture() }, print,
                           fixture_config({ { "enable_support", "0" },
                                            { "printable_area", "-100x-100,100x-100,100x100,-100x100" } }));
    REQUIRE(print.objects().size() == 1);
    const std::vector<SupportAnalysis::Slab> slabs = SupportAnalysis::model_slabs_of(*print.objects().front());
    REQUIRE(slabs.size() == 40);
    const SupportAnalysis::IslandMap map = SupportAnalysis::island_joins(slabs, slabs.front().bottom_z);
    const SupportAnalysis::Components &components = map.components;
    REQUIRE(map.island_of_piece.size() == components.pieces.size());
    REQUIRE(map.islands.size() == 3);

    // The birth piece of each island is the piece on its birth slab that the island owns.
    std::vector<std::pair<double, size_t>> by_x;
    for (size_t i = 0; i < map.islands.size(); ++ i) {
        const size_t s = map.islands[i].birth_slab;
        for (size_t p = components.slab_range[s].first; p < components.slab_range[s].second; ++ p)
            if (map.island_of_piece[p] == i)
                by_x.emplace_back(SupportAnalysis::piece_middle(components.pieces[p]).x(), i);
    }
    REQUIRE(by_x.size() == 3);
    std::sort(by_x.begin(), by_x.end());
    const size_t a = by_x[0].second, b = by_x[1].second, c = by_x[2].second;
    CHECK(map.islands[a].birth_slab == 15);
    CHECK(map.islands[a].join_slab == 30);
    CHECK(map.islands[b].birth_slab == 15);
    CHECK(map.islands[b].join_slab == slabs.size());
    CHECK(map.islands[c].birth_slab == 15);
    CHECK(map.islands[c].join_slab == slabs.size());
    // A owns its pieces until the bar roots it at slab 30; B and C end at z 5, the top of slab 24. None merges into
    // another, so each is its own part.
    CHECK(map.islands[a].top_slab == 29);
    CHECK(map.islands[b].top_slab == 24);
    CHECK(map.islands[c].top_slab == 24);
    CHECK(map.islands[a].part == a);
    CHECK(map.islands[b].part == b);
    CHECK(map.islands[c].part == c);

    // The piece of slab `s` whose outline holds `point`, or npos.
    const auto piece_holding = [&](size_t s, const Point &point) {
        for (size_t p = components.slab_range[s].first; p < components.slab_range[s].second; ++ p)
            if (components.pieces[p].polygon.contains(point))
                return p;
        return size_t(-1);
    };
    REQUIRE(components.slab_range[0].second - components.slab_range[0].first == 1);
    const Point block_middle = components.pieces[components.slab_range[0].first].polygon.contour.centroid();
    for (size_t s = 0; s < slabs.size(); ++ s) {
        const size_t block = piece_holding(s, block_middle);
        REQUIRE(block != size_t(-1));
        CHECK(map.island_of_piece[block] == size_t(-1));
    }
    Point a_middle;
    for (size_t p = components.slab_range[15].first; p < components.slab_range[15].second; ++ p)
        if (map.island_of_piece[p] == a)
            a_middle = components.pieces[p].polygon.contour.centroid();
    const size_t a_on_20 = piece_holding(20, a_middle);
    REQUIRE(a_on_20 != size_t(-1));
    CHECK(map.island_of_piece[a_on_20] == a);
    const size_t a_on_31 = piece_holding(31, a_middle);
    REQUIRE(a_on_31 != size_t(-1));
    CHECK(map.island_of_piece[a_on_31] == size_t(-1));
}

TEST_CASE("A result row carries the scaffold counts", "[ScaffoldSupport]")
{
    SupportAnalysis::Report r;
    r.tips_placed             = 7;
    r.tips_routed             = 5;
    r.tips_dropped            = 2;
    r.islands_under_held      = 1;
    r.pillars_unbraced        = 3;
    r.islands_slender         = 6;
    r.underside_unmet_mm2     = 0.25;
    r.floating_pieces_removed = 4;

    const SupportValidation::Metrics m = SupportValidation::metrics_of(r);
    CHECK(m.tips_placed == 7);
    CHECK(m.tips_routed == 5);
    CHECK(m.tips_dropped == 2);
    CHECK(m.islands_under_held == 1);
    CHECK(m.pillars_unbraced == 3);
    CHECK(m.islands_slender == 6);
    CHECK_THAT(m.underside_unmet_mm2, WithinAbs(0.25, 1e-12));
    CHECK(m.floating_pieces_removed == 4);

    SupportValidation::CaseResult row;
    row.metrics = m;
    std::ostringstream os;
    SupportValidation::write_result(row, os);
    const nlohmann::json j = nlohmann::json::parse(os.str());
    REQUIRE(j.contains("metrics"));
    const nlohmann::json &metrics = j.at("metrics");
    for (const char *key : { "support_volume_mm3", "raft_volume_mm3", "missing_critical_anchors", "invalid_paths",
                             "unrooted_groups", "min_bed_margin", "max_slenderness", "unknown_contacts", "inaccessible_groups",
                             "max_group_risk", "total_group_risk", "coverage_available", "stability_available",
                             "damage_available", "tips_placed", "tips_routed", "tips_dropped", "islands_under_held",
                             "pillars_unbraced", "islands_slender", "underside_unmet_mm2", "floating_pieces_removed" }) {
        INFO(key);
        CHECK(metrics.contains(key));
    }
    CHECK(metrics.value("tips_placed", size_t(0)) == 7);
    CHECK(metrics.value("tips_routed", size_t(0)) == 5);
    CHECK(metrics.value("tips_dropped", size_t(0)) == 2);
    CHECK(metrics.value("islands_under_held", size_t(0)) == 1);
    CHECK(metrics.value("pillars_unbraced", size_t(0)) == 3);
    CHECK(metrics.value("islands_slender", size_t(0)) == 6);
    CHECK_THAT(metrics.value("underside_unmet_mm2", 0.), WithinAbs(0.25, 1e-12));
    CHECK(metrics.value("floating_pieces_removed", size_t(0)) == 4);
}

TEST_CASE("A scaffold on the shelf fixture prints a pad with dense faces and clear base walls and nothing floating", "[ScaffoldSupport]")
{
    Print print;
    init_and_process_print({ shelf_fixture() }, print, scaffold_config());
    REQUIRE(print.objects().size() == 1);
    const PrintObject &object = *print.objects().front();
    REQUIRE(object.support_analysis() != nullptr);
    REQUIRE(object.emitted_support() != nullptr);
    const SupportAnalysis::Report &report = *object.support_analysis();

    // The style runs the miniature front half with a zero gap although the checkbox is off.
    CHECK(report.seeds_candidate > 0);
    CHECK_THAT(object.emitted_support()->top_gap_mm, WithinAbs(0., 1e-9));

    // Every support extrusion is base or interface material, and the pad's first layer joins the slices the skirt reads.
    const auto layers = object.support_layers();
    REQUIRE(layers.size() > 0);
    CHECK_FALSE(layers.front()->lslices.empty());
    CHECK(support_roles(layers) == std::set<ExtrusionRole>{ erSupportMaterial, erSupportMaterialInterface });

    // The pad is 0.6 mm rounded up to whole planned layers: the layers up to the first whose top reaches 0.6 mm.
    // The tree plans its own support layer heights, so that is three layers here, and the layer above it holds
    // only the pillar feet. Every foot stands on the pad, and the 1.6 mm brim reaches past the feet at the pad's top
    // face: the pad's sides slope at 45 degrees, so its first layer is narrower by nearly the pad's thickness.
    const double w = 0.42;
    size_t pad_top = 0;
    while (pad_top < layers.size() && layers[pad_top]->print_z < 0.6 - EPSILON)
        ++ pad_top;
    REQUIRE(pad_top + 1 < layers.size());
    CHECK(pad_top == 2);
    const ExPolygons f0 = footprint(*layers[0]), f_above = footprint(*layers[pad_top + 1]);
    REQUIRE_FALSE(f0.empty());
    REQUIRE_FALSE(f_above.empty());
    const double a0 = area_mm2(f0);
    // The pad's first layer and its top print at the first layer's density, so the bed holds the pad and every foot
    // lands on printed material; the layers between only carry the top and print half as dense.
    for (size_t i = 1; i < pad_top; ++ i) {
        INFO("pad layer " << i << " print_z " << layers[i]->print_z);
        CHECK(area_mm2(footprint(*layers[i])) >= 0.5 * a0);
        CHECK(area_mm2(footprint(*layers[i])) <= 0.7 * a0);
    }
    CHECK(area_mm2(footprint(*layers[pad_top])) >= 0.9 * a0);
    CHECK(area_mm2(f_above) < 0.5 * a0);
    CHECK(diff_ex(f_above, offset_ex(f0, scale_(w))).empty());
    const BoundingBox box_top = get_extents(footprint(*layers[pad_top])), box_above = get_extents(f_above);
    // A foot widens halfway to its nearest pillar, so sparse pillars stand wide feet nearer the brim's rim.
    CHECK(box_above.min.x() - box_top.min.x() >= scale_(1.2));
    CHECK(box_above.min.y() - box_top.min.y() >= scale_(1.2));
    CHECK(box_top.max.x() - box_above.max.x() >= scale_(1.2));
    CHECK(box_top.max.y() - box_above.max.y() >= scale_(1.2));

    // No base comes within the xy distance of the model at its own height, and all of it is on the bed. The
    // interface fuses to the model and is held to not entering it further down.
    const Point shift = object.instances().front().shift;
    for (const SupportLayer *sl : layers) {
        const ExPolygons model   = model_over(object, *sl);
        const ExPolygons printed = footprint(*sl);
        INFO("print_z " << sl->print_z);
        CHECK(intersection_ex(role_footprint(*sl, erSupportMaterial), offset_ex(union_ex(model), scale_(0.35 - 0.02))).empty());
        for (const ExPolygon &poly : printed) {
            BoundingBox box = get_extents(poly);
            box.translate(shift);
            CHECK(box.min.x() >= scale_(-100.));
            CHECK(box.min.y() >= scale_(-100.));
            CHECK(box.max.x() <= scale_(100.));
            CHECK(box.max.y() <= scale_(100.));
        }
    }

    // The pad prints through the sheath and the cage above it prints walls only: no extrusion above the pad
    // reaches further into its layer's base area than a second wall would, while the pad's infill does.
    for (size_t i = pad_top + 1; i < layers.size(); ++ i) {
        INFO("print_z " << layers[i]->print_z);
        CHECK(reaches_inside(*layers[i], w) == 0);
    }
    CHECK(reaches_inside(*layers[1], w) > 0);

    // Each tip fuses to its overhang through a ring of interface on the three layers under it: the highest layer
    // whose top reaches the tip and the two below, and the layer under those carries none. Every tip here holds an
    // underside, the bar's and the slab's, so each reads the small grade, two lines across. The column's first layer
    // places the fixture in the object's centred frame.
    const Point      origin    = get_extents(object.layers().front()->lslices).min;
    const FixtureBox bar_strip { origin, 6., 2.4, 12., 3.6 };
    const FixtureBox slab_box  { origin, 6., -3., 18., 9. };
    // A head at the bar's 0.6 mm edge tilts, so its lower rings stand up to half a millimetre off the bar's side.
    const FixtureBox bar_rings { origin, 6., 2.1, 12., 3.9 };
    const auto check_rings = [&](double tip_z, const FixtureBox &region, const FixtureBox *excluded, double min_w, double max_w) {
        const size_t top = top_layer_under(layers, tip_z);
        REQUIRE(top != size_t(-1));
        REQUIRE(top >= 3);
        const std::vector<RingLayer> rings = rings_under(layers, top, region, excluded, w);
        for (size_t k = 0; k < rings.size(); ++ k) {
            INFO("tip z " << tip_z << " layer " << k << " under the tip, print_z " << rings[k].print_z);
            if (k < 3)
                CHECK(rings[k].in_region > 0);
            else
                CHECK(rings[k].in_region == 0);
        }
        const Disc &narrowest = rings.front().narrowest, &widest = rings.front().widest;
        INFO("tip z " << tip_z << ": graded discs inscribe " << narrowest.width_mm << " mm (bounding box " << narrowest.box_min_mm << " x "
                      << narrowest.box_max_mm << " mm) to " << widest.width_mm << " mm (bounding box " << widest.box_min_mm << " x "
                      << widest.box_max_mm << " mm)");
        CHECK(rings.front().graded > 0);
        CHECK(narrowest.width_mm >= min_w);
        CHECK(widest.width_mm <= max_w);
    };
    check_rings(4., bar_rings, nullptr, 1.8 * w, 2.2 * w);
    check_rings(12., slab_box, &bar_strip, 1.8 * w, 2.2 * w);

    // A bar tip within the xy distance of the column face is skipped, since its neck would be clipped there while
    // its ring survived; the bar keeps the tips further out.
    {
        const SupportLayer &sl = *layers[top_layer_under(layers, 4.)];
        size_t under_bar = 0;
        for (const Disc &disc : discs(sl, w)) {
            const double x = unscale<double>(disc.centroid.x() - origin.x());
            INFO("interface centroid at fixture x " << x << " on print_z " << sl.print_z);
            CHECK(x >= 6.35);
            if (bar_strip.contains(disc.centroid))
                ++ under_bar;
        }
        CHECK(under_bar > 0);
    }

    // The interface fuses to the model but never enters it. What its extrusions cover reaches under 0.04 mm past
    // the drawn area.
    for (const SupportLayer *sl : layers) {
        INFO("print_z " << sl->print_z);
        const ExPolygons inside = offset_ex(union_ex(model_over(object, *sl)), -scale_(0.05));
        CHECK(intersection_ex(role_footprint(*sl, erSupportMaterialInterface), inside).empty());
    }

    // The cage roots on the plate and the floating pass finds nothing to take out of it.
    CHECK(report.floating_pieces_removed == 0);
    CHECK(report.stability.unsupported_paths == 0);

    // The report states what the scaffold did and claims no coverage it never measured.
    INFO("tips placed " << report.tips_placed << " routed " << report.tips_routed << " dropped " << report.tips_dropped
                        << " support " << report.support_volume_mm3 << " mm3");
    CHECK(report.status == SupportAnalysis::Report::Status::Unknown);
    CHECK_FALSE(report.coverage_available);
    CHECK(report.coverage.empty());
    CHECK(report.missing_anchor_ids.empty());
    CHECK(report.tips_placed > 0);
    CHECK(report.tips_routed > 0);
    CHECK(report.tips_placed == report.tips_routed + report.tips_dropped);
    CHECK(report.support_volume_mm3 > 0.);
    CHECK(report.stability.available);
}

TEST_CASE("A cancel during the scaffold build throws and leaves no support layer", "[ScaffoldSupport]")
{
    Print print;
    Model model;
    init_print({ shelf_fixture() }, print, model, scaffold_config());
    print.set_status_callback([&print](const PrintBase::SlicingStatus &s) {
        if (s.percent == 60)
            print.cancel();
    });
    REQUIRE_THROWS_AS(print.process(), CanceledException);
    REQUIRE(print.objects().front()->support_layers().empty());
}

TEST_CASE("With no interface layers the tip layers print as base", "[ScaffoldSupport]")
{
    Print print;
    init_and_process_print({ shelf_fixture() }, print, scaffold_config({ { "support_interface_top_layers", "0" } }));
    REQUIRE(print.objects().size() == 1);
    const PrintObject &object = *print.objects().front();
    const auto         layers = object.support_layers();
    REQUIRE(layers.size() > 0);
    for (const SupportLayer *sl : layers) {
        INFO("print_z " << sl->print_z);
        CHECK(role_footprint(*sl, erSupportMaterialInterface).empty());
    }
    const size_t top = top_layer_under(layers, 4.);
    REQUIRE(top != size_t(-1));
    const FixtureBox bar_strip { get_extents(object.layers().front()->lslices).min, 6., 2.4, 12., 3.6 };
    CHECK_FALSE(intersection_ex(role_footprint(*layers[top], erSupportMaterial), ExPolygons{ ExPolygon(bar_strip.polygon()) }).empty());
}

TEST_CASE("A tip with no path to the pad is dropped and counted", "[ScaffoldSupport]")
{
    Print print;
    init_and_process_print({ blocked_lip_fixture() }, print, scaffold_config());
    REQUIRE(print.objects().size() == 1);
    const PrintObject &object = *print.objects().front();
    REQUIRE(object.support_analysis() != nullptr);
    const SupportAnalysis::Report &report = *object.support_analysis();
    INFO("tips placed " << report.tips_placed << " routed " << report.tips_routed << " dropped " << report.tips_dropped);
    CHECK(report.tips_dropped >= 1);
    CHECK(report.tips_placed == report.tips_routed + report.tips_dropped);
    CHECK(report.floating_pieces_removed == 0);
}

TEST_CASE("Island holders that fail to route count their islands", "[ScaffoldSupport]")
{
    using namespace ScaffoldSupport;
    using Result = ScaffoldTipResult;
    const TipSite held { Point::new_scale(5., 5.), 3., 29 };
    const auto    moved = [&held](double dx) {
        TipSite site = held;
        site.position += Point::new_scale(dx, 0.);
        return site;
    };
    const auto unheld = [](const Plan &plan, const std::vector<TipSite> &sites, const std::vector<Result> &results) {
        return unheld_after_routing(plan, sites, results).islands.size();
    };

    // One birth tip holds its own island and a nub that waited for its merge and met the tip's part. An island no tip
    // can stand under is the plan's own count, and one the bed holds through the part it met needs no tip.
    Plan birth;
    birth.tips.push_back({ held, TipNeed::Birth });
    birth.islands.push_back({ Vec3d(5., 5., 3.), { 0 }, false, IslandReason::Tip });
    birth.islands.push_back({ Vec3d(5.6, 5., 3.4), { 0 }, false, IslandReason::Hung });
    birth.islands.push_back({ Vec3d(9., 9., 4.), {}, false, IslandReason::NoNeck });
    birth.islands.push_back({ Vec3d(1., 1., 2.), {}, true, IslandReason::Hung });
    CHECK(unheld(birth, { held }, { Result::Unrouted }) == 2);
    CHECK(unheld(birth, { held }, { Result::Routed }) == 0);
    // The wall skip took the tip out, so no drawn site stands for it.
    CHECK(unheld(birth, {}, {}) == 2);
    // The alias merge folded it into a tip 0.05 mm off, within sla::D_SP, and that one routed; one 0.2 mm off is another.
    CHECK(unheld(birth, { moved(0.05) }, { Result::Routed }) == 0);
    CHECK(unheld(birth, { moved(0.2) }, { Result::Routed }) == 2);
    // The match is 3-D: a routed site over the tip's xy but 0.2 mm higher stands for another tip.
    TipSite above = held;
    above.print_z += 0.2;
    CHECK(unheld(birth, { above }, { Result::Routed }) == 2);
    // Two drawn sites within sla::D_SP of the tip: it reads routed when either one routed, whichever comes first.
    CHECK(unheld(birth, { held, moved(0.05) }, { Result::Unrouted, Result::Routed }) == 0);
    CHECK(unheld(birth, { held, moved(0.05) }, { Result::Routed, Result::Unrouted }) == 0);
    const PlanOutcome failed = unheld_after_routing(birth, { held }, { Result::Unrouted });
    REQUIRE(failed.tips.size() == 1);
    CHECK(failed.tips.front() == Result::Unrouted);
    CHECK(unheld_after_routing(birth, {}, {}).tips.front() == Result::Wall);

    // A nub whose own slab took an underside head hangs from that head the same way, and the head that fails hands the
    // underside it answered back.
    Plan underside;
    underside.tips.push_back({ held, TipNeed::Underside, 0.25 });
    underside.islands.push_back({ Vec3d(5., 5., 3.), { 0 }, false, IslandReason::Hung });
    const PlanOutcome dropped = unheld_after_routing(underside, { held }, { Result::Unrouted });
    CHECK(dropped.islands.size() == 1);
    CHECK_THAT(dropped.underside_mm2, WithinAbs(0.25, 1e-12));
    const PlanOutcome routed = unheld_after_routing(underside, { held }, { Result::Routed });
    CHECK(routed.islands.empty());
    CHECK_THAT(routed.underside_mm2, WithinAbs(0., 1e-12));
}

TEST_CASE("A birth tip the builder cannot route lists its island as bare and warns", "[ScaffoldSupport]")
{
    // At 0.2 mm layers the rod's birth slab is z 6.0..6.2. Its tip stands at the rod's middle over the box floor, where
    // no route leaves the box, so the rod prints from mid-air and the slice says where.
    Print print;
    init_and_process_print({ boxed_rod_fixture() }, print, scaffold_config());
    REQUIRE(print.objects().size() == 1);
    const PrintObject &object = *print.objects().front();
    REQUIRE(object.support_analysis() != nullptr);
    const std::shared_ptr<const ScaffoldRecord> record = object.scaffold_record();
    REQUIRE(record != nullptr);
    CHECK_FALSE(record->baked);
    const Vec3f rod_birth(10.f, 10.f, 6.f);
    const auto  at_rod = [&rod_birth](const Vec3f &p) {
        return (p.head<2>() - rod_birth.head<2>()).norm() <= 0.3f && std::abs(p.z() - rod_birth.z()) <= 0.25f;
    };
    const auto tip = std::find_if(record->tips.begin(), record->tips.end(), [&](const ScaffoldRecord::Tip &t) { return at_rod(t.pos); });
    REQUIRE(tip != record->tips.end());
    CHECK(tip->result == ScaffoldTipResult::Unrouted);

    const SupportAnalysis::Report &report = *object.support_analysis();
    INFO("tips placed " << report.tips_placed << " routed " << report.tips_routed << " dropped " << report.tips_dropped
                        << ", islands under-held " << report.islands_under_held << ", bare islands " << record->bare_islands.size());
    CHECK(report.islands_under_held >= 1);
    CHECK(record->bare_islands.size() == report.islands_under_held);
    CHECK(std::any_of(record->bare_islands.begin(), record->bare_islands.end(), at_rod));
    CHECK(warns(object, "islands print with no tip holding them."));
}

TEST_CASE("An unrouted Underside head adds its cells back to unmet", "[ScaffoldSupport]")
{
    // The ledge hangs 3 mm off the wall, past one and a half reaches, so its heads answer underside the plan would
    // otherwise count unmet; the box leaves them no route.
    Print print;
    init_and_process_print({ boxed_ledge_fixture() }, print, scaffold_config());
    REQUIRE(print.objects().size() == 1);
    const PrintObject &object = *print.objects().front();
    REQUIRE(object.support_analysis() != nullptr);
    const std::shared_ptr<const ScaffoldRecord> record = object.scaffold_record();
    REQUIRE(record != nullptr);
    const SupportAnalysis::Report &report = *object.support_analysis();
    INFO("tips placed " << report.tips_placed << " routed " << report.tips_routed << " dropped " << report.tips_dropped
                        << ", underside unmet " << report.underside_unmet_mm2 << " mm2");
    const auto under_ledge = [](const ScaffoldRecord::Tip &t) {
        return t.pos.x() > 1.f && t.pos.x() < 4.f && t.pos.y() > 6.f && t.pos.y() < 14.f && std::abs(t.pos.z() - 6.f) <= 0.25f;
    };
    REQUIRE(std::any_of(record->tips.begin(), record->tips.end(), under_ledge));
    CHECK(std::none_of(record->tips.begin(), record->tips.end(),
                       [&](const ScaffoldRecord::Tip &t) { return under_ledge(t) && t.result == ScaffoldTipResult::Routed; }));
    CHECK(report.underside_unmet_mm2 > 0.);
}

TEST_CASE("A planner axis moves the wall skip's neck reading", "[ScaffoldSupport]")
{
    // The rod's birth tip leans its neck away from the column, and the wall skip reads the band at the leaning neck's
    // end, so the tip stays where straight down it would stand in the band. The builder aims the head along that axis,
    // and the head routes. A baked point with no axis reads the same lean again off the plan's input, so a point baked
    // at the tip stays too; one storing a straight-down axis keeps that axis, and the wall skip takes it at the band.
    Print                    print;
    Model                    model;
    const DynamicPrintConfig config = scaffold_config();
    init_print({ leaning_rod_fixture() }, print, model, config);
    print.process();
    REQUIRE(print.objects().size() == 1);
    const std::shared_ptr<const ScaffoldRecord> record = print.objects().front()->scaffold_record();
    REQUIRE(record != nullptr);
    const auto at_rod = [](const ScaffoldRecord::Tip &t) {
        return t.pos.x() > 6.f && t.pos.x() < 6.6f && t.pos.y() > 2.3f && t.pos.y() < 3.7f && std::abs(t.pos.z() - 6.f) <= 0.25f;
    };
    const auto tip = std::find_if(record->tips.begin(), record->tips.end(), at_rod);
    REQUIRE(tip != record->tips.end());
    INFO("the rod's tip at (" << tip->pos.x() << ", " << tip->pos.y() << ", " << tip->pos.z() << ") reads " << Catch::StringMaker<ScaffoldTipResult>::convert(tip->result));
    CHECK(tip->result == ScaffoldTipResult::Routed);

    ModelObject &mo             = *model.objects.front();
    mo.scaffold_points          = { { tip->pos, ScaffoldHeadSize::Light, false } };
    mo.scaffold_points_status   = ScaffoldPointsStatus::AutoGenerated;
    mo.scaffold_points_pose     = record->pose;
    mo.scaffold_points_mesh_box = mo.raw_mesh_bounding_box();
    REQUIRE(reapply(print, model, config) != Print::APPLY_STATUS_UNCHANGED);
    print.process();
    const std::shared_ptr<const ScaffoldRecord> baked = print.objects().front()->scaffold_record();
    REQUIRE(baked != nullptr);
    CHECK(baked->baked);
    REQUIRE(baked->tips.size() == 1);
    CHECK(baked->tips.front().result != ScaffoldTipResult::Wall);

    mo.scaffold_points = { { tip->pos, ScaffoldHeadSize::Light, false, -Vec3f::UnitZ() } };
    REQUIRE(reapply(print, model, config) != Print::APPLY_STATUS_UNCHANGED);
    print.process();
    const std::shared_ptr<const ScaffoldRecord> stored = print.objects().front()->scaffold_record();
    REQUIRE(stored != nullptr);
    CHECK(stored->baked);
    REQUIRE(stored->tips.size() == 1);
    CHECK(stored->tips.front().result == ScaffoldTipResult::Wall);
}

TEST_CASE("A support blocker leaves a baked list's points as the list alone builds them", "[ScaffoldSupport]")
{
    // The rod's birth tip, baked, leans its neck away from the column as the auto slice leaned it. A support blocker
    // volume over the rod's bottom, x 6.05..6.75, y 2.3..3.7, z 5..7, would keep a planned tip off the rod, but a list
    // ignores blockers: the point keeps its lean, stays through the wall skip and routes as it did with no blocker, and
    // the slice warns of nothing dropped.
    Print                    print;
    Model                    model;
    const DynamicPrintConfig config = scaffold_config();
    init_print({ leaning_rod_fixture() }, print, model, config);
    print.process();
    REQUIRE(print.objects().size() == 1);
    const std::shared_ptr<const ScaffoldRecord> record = print.objects().front()->scaffold_record();
    REQUIRE(record != nullptr);
    const auto tip = std::find_if(record->tips.begin(), record->tips.end(), [](const ScaffoldRecord::Tip &t) {
        return t.pos.x() > 6.f && t.pos.x() < 6.6f && t.pos.y() > 2.3f && t.pos.y() < 3.7f && std::abs(t.pos.z() - 6.f) <= 0.25f;
    });
    REQUIRE(tip != record->tips.end());

    ModelObject &mo             = *model.objects.front();
    mo.scaffold_points          = { { tip->pos, ScaffoldHeadSize::Light, false } };
    mo.scaffold_points_status   = ScaffoldPointsStatus::AutoGenerated;
    mo.scaffold_points_pose     = record->pose;
    mo.scaffold_points_mesh_box = mo.raw_mesh_bounding_box();
    REQUIRE(reapply(print, model, config) != Print::APPLY_STATUS_UNCHANGED);
    print.process();
    const std::shared_ptr<const ScaffoldRecord> open = print.objects().front()->scaffold_record();
    REQUIRE(open != nullptr);
    REQUIRE(open->baked);
    REQUIRE(open->tips.size() == 1);
    REQUIRE(open->tips.front().result == ScaffoldTipResult::Routed);

    // The model part's volume carries the offset that centres its mesh, so a volume added uncentred stands in the
    // fixture's own frame.
    TriangleMesh blocker = make_cube(0.7, 1.4, 2.);
    blocker.translate(6.05f, 2.3f, 5.f);
    mo.add_volume(std::move(blocker), ModelVolumeType::SUPPORT_BLOCKER, false);
    REQUIRE(reapply(print, model, config) != Print::APPLY_STATUS_UNCHANGED);
    print.process();
    REQUIRE(print.objects().size() == 1);
    const PrintObject &object = *print.objects().front();
    const std::vector<Polygons> blockers = object.slice_support_blockers();
    REQUIRE(std::any_of(blockers.begin(), blockers.end(), [](const Polygons &layer) { return ! layer.empty(); }));
    const std::shared_ptr<const ScaffoldRecord> blocked = object.scaffold_record();
    REQUIRE(blocked != nullptr);
    CHECK(blocked->baked);
    REQUIRE(blocked->tips.size() == 1);
    INFO("the baked point under a blocker reads " << Catch::StringMaker<ScaffoldTipResult>::convert(blocked->tips.front().result));
    CHECK(blocked->tips.front().result == ScaffoldTipResult::Routed);
    CHECK_FALSE(warns(object, "Scaffold points for"));
}

TEST_CASE("Draw aims each head along its tip's axis", "[ScaffoldSupport]")
{
    // One tip under a 12 x 10 x 2 mm slab at z 8..10 carried by a 4 x 4 mm column, at (9, 5), where the mesh normal
    // points straight down and nothing stands within a head's reach. Handed an axis leaning 45 degrees toward +y, the
    // builder aims the head along it: 1.2 mm under the tip the head's slice holds the point 1.2 mm along +y, and not
    // the point straight under the tip, which a head along the normal would hold.
    Print        print;
    Model        model;
    TriangleMesh fixture = make_cube(4., 4., 10.), slab = make_cube(12., 10., 2.);
    slab.translate(0.f, 0.f, 8.f);
    fixture.merge(slab);
    init_print({ fixture }, print, model, fixture_config({ { "enable_support", "0" }, { "layer_change_gcode", "G92 E0" } }));
    print.process();
    REQUIRE(print.objects().size() == 1);
    const PrintObject &object = *print.objects().front();
    REQUIRE(object.layer_count() > 1);
    ScaffoldSupport::Tips    tips;
    ScaffoldSupport::TipSite site = site_under(object, Vec2d(9., 5.));
    site.axis = Vec3f(0.f, float(std::sin(M_PI / 4.)), float(-std::cos(M_PI / 4.)));
    tips.sites.push_back(site);

    const DrawOnLayers            draw(object);
    const ScaffoldSupport::Output out = draw(object, tips);
    REQUIRE(out.results.size() == 1);
    CHECK(out.results.front() == ScaffoldTipResult::Routed);

    const double z     = site.print_z - 1.2;
    const auto   layer = std::find_if(draw.plan.begin(), draw.plan.end(), [z](const LayerHeightData &l) { return l.print_z - l.height <= z && z < l.print_z; });
    REQUIRE(layer != draw.plan.end());
    const ScaffoldSupport::LayerAreas &areas = out.layers[size_t(layer - draw.plan.begin())];
    ExPolygons                         head  = areas.base;
    append(head, areas.interface_);
    append(head, areas.exempt_heads);
    head = union_ex(head);
    INFO("head slice at z " << layer->print_z << ": " << head.size() << " pieces");
    CHECK(holds(head, site.position + Point::new_scale(0., 1.2)));
    CHECK_FALSE(holds(head, site.position));
}

TEST_CASE("A tip holding an island leans its head out where no other route reaches the pad", "[ScaffoldSupport]")
{
    // Under the lip every route from a head straight down meets the shelf, and so does the thin head's. The tip holds
    // the lip, an island, so the builder retries its head along leaning axes, and one reaches the pad past the shelf's
    // edge. A tip holding no island is dropped there. A baked point at the tip stands under the lip's island, so its
    // list marks it and it routes as the tip does. Two baked points 0.08 mm apart across the lip's -x edge are aliases:
    // the merge keeps the one outside the lip, which holds the island for the point merged into it. The need planner
    // holds the lip with a tip of its own, which `place_tips` marks, so an auto slice routes it and leaves no island
    // under-held.
    Print print;
    Model model;
    init_print({ island_lip_fixture() }, print, model, fixture_config({ { "enable_support", "0" }, { "layer_change_gcode", "G92 E0" } }));
    print.process();
    REQUIRE(print.objects().size() == 1);
    const PrintObject &object = *print.objects().front();
    REQUIRE(object.layer_count() > 1);
    const DrawOnLayers draw(object);
    for (const bool island : { false, true }) {
        ScaffoldSupport::Tips tips;
        tips.sites.push_back(site_under(object, Vec2d(10., 10.), 5.));
        tips.sites.front().holds_island = island;
        const ScaffoldSupport::Output out = draw(object, tips);
        INFO((island ? "holding an island" : "holding none"));
        REQUIRE(out.results.size() == 1);
        CHECK(out.results.front() == (island ? ScaffoldTipResult::Routed : ScaffoldTipResult::Unrouted));
    }

    const ScaffoldSupport::TipSite site  = site_under(object, Vec2d(10., 10.), 5.);
    const ScaffoldPoints           point = { ScaffoldSupport::point_of(object, draw.params, site, 2. * draw.params.toolpath_width_mm) };
    const ScaffoldSupport::Tips    baked = ScaffoldSupport::baked_tips(object, point, draw.params, M_PI / 6.);
    REQUIRE(baked.sites.size() == 1);
    CHECK(baked.sites.front().holds_island);
    const ScaffoldSupport::Output out = draw(object, baked);
    REQUIRE(out.results.size() == 1);
    CHECK(out.results.front() == ScaffoldTipResult::Routed);

    const ScaffoldSupport::TipSite inside  = site_under(object, Vec2d(9.05, 10.), 5.);
    ScaffoldSupport::TipSite       outside = inside;
    outside.position -= Point::new_scale(0.08, 0.);
    const ScaffoldPoints        pair   = { ScaffoldSupport::point_of(object, draw.params, outside, 2. * draw.params.toolpath_width_mm),
                                           ScaffoldSupport::point_of(object, draw.params, inside, 2. * draw.params.toolpath_width_mm) };
    const ScaffoldSupport::Tips merged = ScaffoldSupport::baked_tips(object, pair, draw.params, M_PI / 6.);
    REQUIRE(merged.sites.size() == 1);
    CHECK((unscale(merged.sites.front().position) - unscale(outside.position)).norm() < 0.01);
    CHECK(merged.sites.front().holds_island);

    const ScaffoldSupport::Tips placed = ScaffoldSupport::place_tips(object, {}, draw.params, M_PI / 6., {});
    const auto lip = std::find_if(placed.plan.islands.begin(), placed.plan.islands.end(), [](const ScaffoldSupport::Island &island) {
        return ! island.rooted && ! island.holders.empty();
    });
    REQUIRE(lip != placed.plan.islands.end());
    const ScaffoldSupport::Output planned = draw(object, placed);
    REQUIRE(planned.results.size() == placed.sites.size());
    for (const size_t holder : lip->holders) {
        const ScaffoldSupport::TipSite &at   = placed.plan.tips[holder].site;
        const auto                      kept = std::find_if(placed.sites.begin(), placed.sites.end(), [&at](const ScaffoldSupport::TipSite &t) {
            return t.position == at.position && std::abs(t.print_z - at.print_z) < EPSILON;
        });
        REQUIRE(kept != placed.sites.end());
        const Vec2d xy = unscale(kept->position);
        INFO("the lip's holder at (" << xy.x() << ", " << xy.y() << ", " << kept->print_z << ") reads "
                                     << Catch::StringMaker<ScaffoldTipResult>::convert(planned.results[size_t(kept - placed.sites.begin())]));
        CHECK(kept->holds_island);
    }
    CHECK(ScaffoldSupport::unheld_after_routing(placed.plan, placed.sites, planned.results).islands.empty());
}

TEST_CASE("A baked list holds an island by the planner's rule and names every island it leaves unheld", "[ScaffoldSupport]")
{
    // At 0.2 mm layers the block is born on the slab at z 6 and the rod on the one at z 4.6. The planner holds the
    // block with one birth tip however far it stands free and however many more tips its underside has room for, and
    // no neck clears under the rod, so the plan counts the rod's island unheld and names its birth point. A list reads
    // the islands by the same rule: one point under the block's middle holds it, a point holds the rod only where it
    // survives the wall skip, as a point placed with the tool does, and every unheld island is named, the block's at
    // the tip the rule would stand under it.
    Print print;
    Model model;
    init_print({ hanging_islands_fixture() }, print, model, fixture_config({ { "enable_support", "0" }, { "layer_change_gcode", "G92 E0" } }));
    print.process();
    REQUIRE(print.objects().size() == 1);
    const PrintObject &object = *print.objects().front();
    const DrawOnLayers draw(object);
    const double       threshold = M_PI / 6.;
    const ScaffoldSupport::TipSite block = site_under(object, Vec2d(6., 6.), 5.);
    const ScaffoldSupport::TipSite rod   = site_under(object, Vec2d(12., 6.), 4.5);
    REQUIRE_THAT(block.print_z, WithinAbs(6., 1e-6));
    REQUIRE_THAT(rod.print_z, WithinAbs(4.6, 1e-6));

    const ScaffoldSupport::Tips placed = ScaffoldSupport::place_tips(object, {}, draw.params, threshold, {});
    CHECK(placed.islands_under_held == 1);
    CHECK(placed.bare_islands.size() == 1);
    CHECK(bare_at(placed.bare_islands, rod));

    const ScaffoldPoints        one  = { ScaffoldSupport::point_of(object, draw.params, block, 2. * draw.params.toolpath_width_mm) };
    const ScaffoldSupport::Tips held = ScaffoldSupport::baked_tips(object, one, draw.params, threshold);
    REQUIRE(held.sites.size() == 1);
    CHECK(held.sites.front().holds_island);
    CHECK(held.islands_under_held == 1);
    CHECK(held.bare_islands.size() == 1);
    CHECK(bare_at(held.bare_islands, rod));

    // A point under the rod necks into the base: a copied one is wall-skipped and leaves the rod unheld, while one
    // placed with the tool is enforced, stays and holds it.
    ScaffoldPoints              pair    = { one.front(), ScaffoldSupport::point_of(object, draw.params, rod, 2. * draw.params.toolpath_width_mm) };
    const ScaffoldSupport::Tips skipped = ScaffoldSupport::baked_tips(object, pair, draw.params, threshold);
    CHECK(skipped.wall_skipped == std::vector<int>{ 1 });
    CHECK(skipped.islands_under_held == 1);
    CHECK(skipped.bare_islands.size() == 1);
    CHECK(bare_at(skipped.bare_islands, rod));
    pair.back().enforced = true;
    const ScaffoldSupport::Tips enforced = ScaffoldSupport::baked_tips(object, pair, draw.params, threshold);
    CHECK(enforced.wall_skipped.empty());
    CHECK(enforced.sites.size() == 2);
    CHECK(enforced.islands_under_held == 0);
    CHECK(enforced.bare_islands.empty());

    const ScaffoldSupport::Tips empty = ScaffoldSupport::baked_tips(object, {}, draw.params, threshold);
    CHECK(empty.islands_under_held == 2);
    CHECK(empty.bare_islands.size() == 2);
    CHECK(bare_at(empty.bare_islands, block));
    CHECK(bare_at(empty.bare_islands, rod));

    // A list Generate copied from the auto slice counts and names what the auto slice does.
    ScaffoldPoints copied;
    for (const ScaffoldSupport::TipSite &site : placed.sites)
        copied.push_back(ScaffoldSupport::point_of(object, draw.params, site, site.grade_mm));
    const ScaffoldSupport::Tips baked = ScaffoldSupport::baked_tips(object, copied, draw.params, threshold);
    CHECK(baked.sites.size() == placed.sites.size());
    CHECK(baked.islands_under_held == placed.islands_under_held);
    CHECK(baked.bare_islands.size() == placed.bare_islands.size());
    CHECK(bare_at(baked.bare_islands, rod));
}

TEST_CASE("A baked point holds an island only on its birth piece", "[ScaffoldSupport]")
{
    // At 0.2 mm layers the tee is born on the slab at z 6 under its stem, and its ledge's underside starts at z 7.4. The
    // planner holds the tee with one birth tip under the stem. A point under the ledge stands on the tee's part and is
    // marked as holding it, but the stem's 1.4 mm under it print on nothing, so a list holding only the ledge
    // reads the tee under-held and names it at the tip the rule would stand under the stem, while a point there holds it.
    Print print;
    Model model;
    init_print({ hanging_tee_fixture() }, print, model, fixture_config({ { "enable_support", "0" }, { "layer_change_gcode", "G92 E0" } }));
    print.process();
    REQUIRE(print.objects().size() == 1);
    const PrintObject &object = *print.objects().front();
    const DrawOnLayers draw(object);
    const double       threshold = M_PI / 6.;
    const ScaffoldSupport::TipSite stem  = site_under(object, Vec2d(8., 6.), 5.);
    const ScaffoldSupport::TipSite ledge = site_under(object, Vec2d(5.5, 3.5), 5.);
    REQUIRE_THAT(stem.print_z, WithinAbs(6., 1e-6));
    REQUIRE_THAT(ledge.print_z, WithinAbs(7.4, 1e-6));

    const ScaffoldSupport::Tips placed = ScaffoldSupport::place_tips(object, {}, draw.params, threshold, {});
    CHECK(placed.islands_under_held == 0);
    const auto tee = std::find_if(placed.plan.islands.begin(), placed.plan.islands.end(),
                                  [](const ScaffoldSupport::Island &island) { return std::abs(island.birth.z() - 6.) < 1e-6; });
    REQUIRE(tee != placed.plan.islands.end());
    REQUIRE(tee->holders.size() == 1);
    CHECK(placed.plan.tips[tee->holders.front()].need == ScaffoldSupport::TipNeed::Birth);
    CHECK_THAT(placed.plan.tips[tee->holders.front()].site.print_z, WithinAbs(6., 1e-6));

    const ScaffoldPoints        on_ledge   = { ScaffoldSupport::point_of(object, draw.params, ledge, 2. * draw.params.toolpath_width_mm) };
    const ScaffoldSupport::Tips ledge_only = ScaffoldSupport::baked_tips(object, on_ledge, draw.params, threshold);
    REQUIRE(ledge_only.sites.size() == 1);
    CHECK(ledge_only.sites.front().holds_island);
    CHECK(ledge_only.islands_under_held == 1);
    REQUIRE(ledge_only.bare_islands.size() == 1);
    const Vec3d &bare = ledge_only.bare_islands.front();
    INFO("bare island at (" << bare.x() << ", " << bare.y() << ", " << bare.z() << ")");
    CHECK((bare.head<2>() - unscale(stem.position)).norm() <= 0.3);
    CHECK_THAT(bare.z(), WithinAbs(6., 1e-6));

    const ScaffoldPoints        both = { on_ledge.front(), ScaffoldSupport::point_of(object, draw.params, stem, 2. * draw.params.toolpath_width_mm) };
    const ScaffoldSupport::Tips held = ScaffoldSupport::baked_tips(object, both, draw.params, threshold);
    REQUIRE(held.sites.size() == 2);
    CHECK(held.islands_under_held == 0);
    CHECK(held.bare_islands.empty());
}

TEST_CASE("A baked island whose points on its birth piece all fail to route reads under-held and bare", "[ScaffoldSupport]")
{
    // At 0.2 mm layers the slab is born on the object layer at z 6. A point under it 4 mm in from the base's edge holds
    // the island by the planner's rule, but its head has no route, so once it drops the island prints with no tip
    // holding it: draw counts it under-held and names it at its birth point, the slab's middle, as the plan names an
    // island whose holders all fail. The same list with a point 4 mm past the edge, which routes, reads the island held.
    // An alias of the stranded point merges into it and hands its place to it, so a routable point under the bar past
    // the slab, which holds no island, listed after the two, leaves the island under-held.
    Print print;
    Model model;
    init_print({ overhung_slab_fixture() }, print, model, fixture_config({ { "enable_support", "0" }, { "layer_change_gcode", "G92 E0" } }));
    print.process();
    REQUIRE(print.objects().size() == 1);
    const PrintObject &object = *print.objects().front();
    const DrawOnLayers draw(object);
    const double       threshold = M_PI / 6.;
    const ScaffoldSupport::TipSite over   = site_under(object, Vec2d(16., 6.), 5.);
    const ScaffoldSupport::TipSite beyond = site_under(object, Vec2d(24., 6.), 5.);
    const ScaffoldSupport::TipSite middle = site_under(object, Vec2d(20., 6.), 5.);
    REQUIRE_THAT(over.print_z, WithinAbs(6., 1e-6));
    REQUIRE_THAT(beyond.print_z, WithinAbs(6., 1e-6));
    const auto point = [&](const ScaffoldSupport::TipSite &site) {
        return ScaffoldSupport::point_of(object, draw.params, site, 2. * draw.params.toolpath_width_mm);
    };
    const auto results = [](const ScaffoldSupport::Output &out) {
        std::string text;
        for (const ScaffoldTipResult result : out.results)
            text += Catch::StringMaker<ScaffoldTipResult>::convert(result) + " ";
        return text;
    };

    const ScaffoldSupport::Tips stranded = ScaffoldSupport::baked_tips(object, { point(over) }, draw.params, threshold);
    REQUIRE(stranded.sites.size() == 1);
    CHECK(stranded.sites.front().holds_island);
    CHECK(stranded.islands_under_held == 0);
    const ScaffoldSupport::Output dropped = draw(object, stranded);
    INFO("the point over the base reads " << results(dropped));
    REQUIRE(dropped.results.size() == 1);
    CHECK(dropped.results.front() != ScaffoldTipResult::Routed);
    CHECK(dropped.counts.islands_under_held == 1);
    REQUIRE(dropped.bare_islands.size() == 1);
    const Vec3d &bare = dropped.bare_islands.front();
    INFO("bare island at (" << bare.x() << ", " << bare.y() << ", " << bare.z() << ")");
    CHECK((bare.head<2>() - unscale(middle.position)).norm() <= 0.3);
    CHECK_THAT(bare.z(), WithinAbs(6., 1e-6));

    const ScaffoldSupport::Tips   held   = ScaffoldSupport::baked_tips(object, { point(over), point(beyond) }, draw.params, threshold);
    REQUIRE(held.sites.size() == 2);
    const ScaffoldSupport::Output routed = draw(object, held);
    INFO("the points read " << results(routed));
    REQUIRE(routed.results.size() == 2);
    CHECK(routed.results.front() != ScaffoldTipResult::Routed);
    CHECK(routed.results.back() == ScaffoldTipResult::Routed);
    CHECK(routed.counts.islands_under_held == 0);
    CHECK(routed.bare_islands.empty());

    ScaffoldSupport::TipSite alias = over;
    alias.position += Point::new_scale(0.05, 0.);
    const ScaffoldSupport::TipSite under_bar = site_under(object, Vec2d(27.5, 6.), 8.5);
    REQUIRE_THAT(under_bar.print_z, WithinAbs(9., 1e-6));
    const ScaffoldSupport::Tips merged =
        ScaffoldSupport::baked_tips(object, { point(over), point(alias), point(under_bar) }, draw.params, threshold);
    REQUIRE(merged.sites.size() == 2);
    CHECK_FALSE(merged.sites.back().holds_island);
    const ScaffoldSupport::Output still_dropped = draw(object, merged);
    INFO("with the stranded point's alias the points read " << results(still_dropped));
    REQUIRE(still_dropped.results.size() == 2);
    CHECK(still_dropped.results.front() != ScaffoldTipResult::Routed);
    CHECK(still_dropped.results.back() == ScaffoldTipResult::Routed);
    CHECK(still_dropped.counts.islands_under_held == 1);
    CHECK(still_dropped.bare_islands.size() == 1);
}

TEST_CASE("A baked nub counts under-held once the points on the part it hangs from all fail to route", "[ScaffoldSupport]")
{
    // At 0.05 mm layers the slab is born at z 6 and the nub at z 6.95, and the nub merges into the slab's part at z 7 with
    // no point on its own birth piece. The plan hangs it from the slab's tips, so it prints unheld once they all fail. A
    // list reads it the same way: a point on the slab holds the slab and the nub, and when that point has no route both
    // print with no tip holding them and are named at their birth points. With no point nothing holds the slab at the
    // nub's merge, so the nub needs the tip the rule stands under it, which the list lacks, and counts with the slab.
    Print print;
    Model model;
    init_print({ overhung_nub_fixture() }, print, model,
               fixture_config({ { "enable_support", "0" }, { "layer_height", "0.05" }, { "initial_layer_print_height", "0.05" },
                                { "layer_change_gcode", "G92 E0" } }));
    print.process();
    REQUIRE(print.objects().size() == 1);
    const PrintObject &object = *print.objects().front();
    const DrawOnLayers draw(object);
    const double       threshold = M_PI / 6.;
    const ScaffoldSupport::TipSite over   = site_under(object, Vec2d(16., 6.), 5.);
    const ScaffoldSupport::TipSite beyond = site_under(object, Vec2d(24., 6.), 5.);
    const ScaffoldSupport::TipSite nub    = site_under(object, Vec2d(26.5, 6.), 6.5);
    REQUIRE_THAT(over.print_z, WithinAbs(6., 1e-6));
    REQUIRE_THAT(beyond.print_z, WithinAbs(6., 1e-6));
    REQUIRE_THAT(nub.print_z, WithinAbs(6.95, 1e-6));
    const auto point = [&](const ScaffoldSupport::TipSite &site) {
        return ScaffoldSupport::point_of(object, draw.params, site, 2. * draw.params.toolpath_width_mm);
    };

    const ScaffoldSupport::Tips placed = ScaffoldSupport::place_tips(object, {}, draw.params, threshold, {});
    const auto hung = std::find_if(placed.plan.islands.begin(), placed.plan.islands.end(), [](const ScaffoldSupport::Island &island) {
        return std::abs(island.birth.z() - 6.95) < 1e-6;
    });
    REQUIRE(hung != placed.plan.islands.end());
    CHECK(hung->reason == ScaffoldSupport::IslandReason::Hung);
    CHECK_FALSE(hung->rooted);
    CHECK_FALSE(hung->holders.empty());

    const ScaffoldSupport::Tips held = ScaffoldSupport::baked_tips(object, { point(beyond) }, draw.params, threshold);
    REQUIRE(held.sites.size() == 1);
    CHECK(held.islands_under_held == 0);
    REQUIRE(held.held_islands.size() == 2);
    for (const ScaffoldSupport::HeldIsland &island : held.held_islands)
        CHECK(island.holders == std::vector<size_t>{ 0 });
    const ScaffoldSupport::Output routed = draw(object, held);
    REQUIRE(routed.results.size() == 1);
    CHECK(routed.results.front() == ScaffoldTipResult::Routed);
    CHECK(routed.counts.islands_under_held == 0);
    CHECK(routed.bare_islands.empty());

    const ScaffoldSupport::Tips stranded = ScaffoldSupport::baked_tips(object, { point(over) }, draw.params, threshold);
    REQUIRE(stranded.sites.size() == 1);
    CHECK(stranded.islands_under_held == 0);
    CHECK(stranded.held_islands.size() == 2);
    const ScaffoldSupport::Output dropped = draw(object, stranded);
    REQUIRE(dropped.results.size() == 1);
    CHECK(dropped.results.front() != ScaffoldTipResult::Routed);
    CHECK(dropped.counts.islands_under_held == 2);
    CHECK(dropped.bare_islands.size() == 2);
    CHECK(bare_at(dropped.bare_islands, nub));

    const ScaffoldSupport::Tips empty = ScaffoldSupport::baked_tips(object, {}, draw.params, threshold);
    CHECK(empty.islands_under_held == 2);
    CHECK(empty.bare_islands.size() == 2);
    CHECK(bare_at(empty.bare_islands, nub));
}

TEST_CASE("A baked nub hanging from a part the bed roots counts nothing once the points on that part fail to route", "[ScaffoldSupport]")
{
    // At 0.05 mm layers both slabs are born at z 6 and joined at z 8.5, the first nub is born at z 8.75 and hangs from
    // their part at z 8.8, and the second is born at z 9.45 and hangs at z 9.5 from the part the bar joined them into
    // with the column the bed roots. A point under the slab over the base has no route: once it drops, that slab and the
    // first nub, which it holds, print with no tip holding them, as does the second slab, which has no point. The second
    // nub needs no point, since the bed holds its part.
    Print print;
    Model model;
    init_print({ joined_nubs_fixture() }, print, model,
               fixture_config({ { "enable_support", "0" }, { "layer_height", "0.05" }, { "initial_layer_print_height", "0.05" },
                                { "layer_change_gcode", "G92 E0" } }));
    print.process();
    REQUIRE(print.objects().size() == 1);
    const PrintObject &object = *print.objects().front();
    const DrawOnLayers draw(object);
    const double       threshold = M_PI / 6.;
    const ScaffoldSupport::TipSite over   = site_under(object, Vec2d(16., 6.), 5.);
    const ScaffoldSupport::TipSite first  = site_under(object, Vec2d(26.5, 6.), 8.6);
    const ScaffoldSupport::TipSite second = site_under(object, Vec2d(28.5, 6.), 9.3);
    REQUIRE_THAT(over.print_z, WithinAbs(6., 1e-6));
    REQUIRE_THAT(first.print_z, WithinAbs(8.75, 1e-6));
    REQUIRE_THAT(second.print_z, WithinAbs(9.45, 1e-6));
    const auto island_at = [](const ScaffoldSupport::Plan &plan, double z) {
        return std::find_if(plan.islands.begin(), plan.islands.end(),
                            [z](const ScaffoldSupport::Island &island) { return std::abs(island.birth.z() - z) < 1e-6; });
    };

    const ScaffoldSupport::Tips placed      = ScaffoldSupport::place_tips(object, {}, draw.params, threshold, {});
    const auto                  first_plan  = island_at(placed.plan, 8.75);
    const auto                  second_plan = island_at(placed.plan, 9.45);
    REQUIRE(first_plan != placed.plan.islands.end());
    REQUIRE(second_plan != placed.plan.islands.end());
    CHECK(first_plan->reason == ScaffoldSupport::IslandReason::Hung);
    CHECK_FALSE(first_plan->rooted);
    CHECK(second_plan->reason == ScaffoldSupport::IslandReason::Hung);
    CHECK(second_plan->rooted);

    const ScaffoldPoints        list     = { ScaffoldSupport::point_of(object, draw.params, over, 2. * draw.params.toolpath_width_mm) };
    const ScaffoldSupport::Tips stranded = ScaffoldSupport::baked_tips(object, list, draw.params, threshold);
    REQUIRE(stranded.sites.size() == 1);
    CHECK(stranded.islands_under_held == 1);
    CHECK(stranded.held_islands.size() == 2);
    const ScaffoldSupport::Output dropped = draw(object, stranded);
    REQUIRE(dropped.results.size() == 1);
    CHECK(dropped.results.front() != ScaffoldTipResult::Routed);
    CHECK(dropped.counts.islands_under_held == 3);
    CHECK(dropped.bare_islands.size() == 3);
    CHECK(bare_at(dropped.bare_islands, first));
    CHECK_FALSE(bare_at(dropped.bare_islands, second));
}

TEST_CASE("A baked list with no point counts both of two nubs that meet only each other", "[ScaffoldSupport]")
{
    // At 0.05 mm layers both nubs are born at z 5 and meet at z 5.05. The plan tips one and hangs the other from it. A
    // list with no point lacks that tip, so the one the rule tips is under-held, named at the tip the rule stands under
    // it, and the other, whose part only that tip would hold, is under-held too, named at its birth point.
    Print print;
    Model model;
    init_print({ paired_nubs_fixture() }, print, model,
               fixture_config({ { "enable_support", "0" }, { "layer_height", "0.05" }, { "initial_layer_print_height", "0.05" },
                                { "layer_change_gcode", "G92 E0" } }));
    print.process();
    REQUIRE(print.objects().size() == 1);
    const PrintObject &object = *print.objects().front();
    const DrawOnLayers draw(object);
    const double       threshold = M_PI / 6.;

    const ScaffoldSupport::Tips placed = ScaffoldSupport::place_tips(object, {}, draw.params, threshold, {});
    CHECK(placed.islands_under_held == 0);
    CHECK(std::count_if(placed.plan.islands.begin(), placed.plan.islands.end(), [](const ScaffoldSupport::Island &island) {
              return island.reason == ScaffoldSupport::IslandReason::Hung && std::abs(island.birth.z() - 5.) < 1e-6;
          }) == 1);

    const ScaffoldSupport::Tips empty = ScaffoldSupport::baked_tips(object, {}, draw.params, threshold);
    CHECK(empty.islands_under_held == 2);
    CHECK(empty.bare_islands.size() == 2);
    for (const Vec2d &nub : { Vec2d(7.3, 6.3), Vec2d(8.2, 6.3) }) {
        const ScaffoldSupport::TipSite site = site_under(object, nub, 4.5);
        INFO("nub at (" << nub.x() << ", " << nub.y() << ")");
        REQUIRE_THAT(site.print_z, WithinAbs(5., 1e-6));
        CHECK(bare_at(empty.bare_islands, site));
    }
}

TEST_CASE("A head whose neck bottoms in the xy band is dropped while one whose neck clears it keeps its head and no ring floats",
          "[ScaffoldSupport]")
{
    // At the corpus's 0.22 mm support line and 0.5 mm xy distance a small-grade neck, 0.44 mm across under its pin, fits
    // wholly inside the column's band. A head's neck under its rings is clipped by that band. When that leaves a ring over
    // nothing and the neck's lowest slice clears the band, the neck is clipped by the model alone and the head stays; a
    // head whose neck bottoms in the column's band is dropped and the cage is built without it. The automatic placement
    // leaves the plank's underside beside the column to the column, so on either plank every head stands; a baked list
    // with points 0.6 and 1.3 mm off the face of the 4 mm plank drops one or both of them.
    // The 4 mm plank at the corpus's 0.06 mm layers and two interface layers adds rings whose printed loop stops short
    // of a neighbour the drawn ring touches, and base whose printed walls leave its inside and its slivers bare: the
    // drop reads the rings and the base as their lines cover them.
    const auto [length_mm, layer_height, interface_layers, baked, min_dropped, max_dropped] = GENERATE(table<double, std::string, std::string, bool, size_t, size_t>(
        { { 2., "0.2", "3", false, 0, 0 }, { 4., "0.06", "2", false, 0, 0 }, { 4., "0.06", "2", true, 1, 2 } }));
    const DynamicPrintConfig config = scaffold_config({ { "support_line_width", "0.22" },
                                                        { "support_object_xy_distance", "0.5" },
                                                        { "layer_height", layer_height },
                                                        { "support_interface_top_layers", interface_layers } });
    Print print;
    Model model;
    init_print({ slope_fixture(length_mm) }, print, model, config);
    if (baked) {
        // On the underside, which falls from z 10 at the column face x 6 at 45 degrees, across the plank's middle.
        ModelObject &mo = *model.objects.front();
        for (const double x : { 6.6, 7.3, 8.5, 9.5 })
            mo.scaffold_points.push_back({ Vec3f(float(x), 3.f, float(16. - x)), ScaffoldHeadSize::Light, false });
        mo.scaffold_points_status   = ScaffoldPointsStatus::UserModified;
        mo.scaffold_points_pose     = mo.instances.front()->get_matrix().linear();
        mo.scaffold_points_mesh_box = mo.raw_mesh_bounding_box();
        reapply(print, model, config);
    }
    print.process();
    REQUIRE(print.objects().size() == 1);
    const PrintObject &object = *print.objects().front();
    REQUIRE(object.support_analysis() != nullptr);
    const SupportAnalysis::Report &report = *object.support_analysis();
    INFO("plank " << length_mm << " mm at " << layer_height << " mm layers" << (baked ? ", baked" : "") << ": tips placed " << report.tips_placed << " routed "
                  << report.tips_routed << " dropped " << report.tips_dropped << " floating removed " << report.floating_pieces_removed);
    CHECK(report.floating_pieces_removed == 0);
    CHECK(report.tips_dropped >= min_dropped);
    CHECK(report.tips_dropped <= max_dropped);
    CHECK(report.tips_routed > 0);
    CHECK(report.tips_placed == report.tips_routed + report.tips_dropped);

    // The contacts nearest the column plan layers that carry no area once the wall skip and the neck check have taken
    // their tips out. Such a layer is zeroed and erased, so every support layer left has a height and an area.
    const auto layers = object.support_layers();
    for (const SupportLayer *sl : layers) {
        INFO("support layer at print_z " << sl->print_z);
        CHECK(sl->height > 0.);
        CHECK(sl->base_areas.size() + sl->tree_roof_1st_layer().size() > 0);
    }

    // Base inside the band stands only under a ring: within `reach_xy` in x and in y of a ring's centroid, on the layers
    // at most `reach_z` under it. So a neck clipped by the model alone enters the band and a pillar or a bridge never does.
    // The rings print as interface on both rows.
    const double       reach_xy = 1.5, reach_z = 2.5;
    const coord_t      r = scale_(reach_xy);
    std::vector<Reach> reaches;
    for (const SupportLayer *sl : layers)
        for (const ExPolygon &ring : role_footprint(*sl, erSupportMaterialInterface)) {
            const Point c = ring.contour.centroid();
            reaches.push_back({ BoundingBox(c - Point(r, r), c + Point(r, r)).polygon(), sl->print_z });
        }
    for (const SupportLayer *sl : layers) {
        const ExPolygons in_band =
            intersection_ex(role_footprint(*sl, erSupportMaterial), offset_ex(union_ex(model_over(object, *sl)), scale_(0.5 - 0.02)));
        INFO("support layer at print_z " << sl->print_z);
        CHECK(diff_ex(in_band, reach_at(reaches, sl->print_z, reach_z)).empty());
    }
}

TEST_CASE("A head under a sheet thinner than its pin leaves nothing floating over the sheet", "[ScaffoldSupport]")
{
    // The builder sinks each head's pin into the model over its tip. Over a sheet thinner than that reach the pin comes
    // out on the sheet's top face, where no support stands under it, so the cage prints no head over its rings.
    Print print;
    init_and_process_print({ thin_sheet_fixture() }, print,
                           scaffold_config({ { "support_line_width", "0.22" },
                                             { "support_object_xy_distance", "0.5" },
                                             { "layer_height", "0.06" },
                                             { "min_layer_height", "0.04" },
                                             { "support_top_z_distance", "0.06" },
                                             { "support_bottom_z_distance", "0" },
                                             { "support_interface_top_layers", "2" } }));
    REQUIRE(print.objects().size() == 1);
    const PrintObject &object = *print.objects().front();
    REQUIRE(object.support_analysis() != nullptr);
    const SupportAnalysis::Report &report = *object.support_analysis();
    INFO("tips placed " << report.tips_placed << " routed " << report.tips_routed << " dropped " << report.tips_dropped
                        << " floating removed " << report.floating_pieces_removed);
    CHECK(report.tips_routed > 0);
    CHECK(report.tips_placed == report.tips_routed + report.tips_dropped);
    CHECK(report.floating_pieces_removed == 0);

    // The bars route tips of their own, so the sheet's own heads are read off the rings under it.
    const auto   layers = object.support_layers();
    const size_t top    = top_layer_under(layers, 8.);
    REQUIRE(top != size_t(-1));
    const FixtureBox under_sheet { get_extents(object.layers().front()->lslices).min, 6.5, 0., 12., 6. };
    CHECK_FALSE(intersection_ex(role_footprint(*layers[top], erSupportMaterialInterface), ExPolygons{ ExPolygon(under_sheet.polygon()) }).empty());
}

TEST_CASE("A painted enforcer beside a wall fuses its tip where the paint asks", "[ScaffoldSupport]")
{
    // At the corpus's 0.22 mm support line, 0.5 mm xy distance and 0.06 mm layers, a bar tip within 0.5 mm of the column
    // face stands in the column's band at its neck's bottom, and a tip on the column's face always does, so the wall skip
    // takes both. A support enforcer, painted facets or an enforcer volume, pins the contacts it covers, and paint does
    // it here: painting the bar's underside pins every contact on it, and painting the column's +y face puts a vertical
    // enforcer point at the centroid of each of its two facets. An enforced tip is not skipped, and its own head below its
    // rings is clipped by the model alone: under two 0.06 mm rings the band would cut a tilted neck and strand the rings.
    // With no interface layers the tips print as base, and the neck check still reads the enforced heads. Support layers
    // follow the object's layers, as the corpus's dense contacts plan them under most tips; a lone tip here would get
    // 0.29 mm layers, and its tilted neck clears the band under two rings that deep.
    const std::string interface_layers = GENERATE(as<std::string>{}, "2", "0");
    // The role a tip's top layer prints in.
    const ExtrusionRole      tip_role = interface_layers == "0" ? erSupportMaterial : erSupportMaterialInterface;
    const DynamicPrintConfig config   = scaffold_config({ { "support_line_width", "0.22" },
                                                          { "support_object_xy_distance", "0.5" },
                                                          { "layer_height", "0.06" },
                                                          { "independent_support_layer_height", "0" },
                                                          { "support_interface_top_layers", interface_layers } });
    const auto run = [&config](Model &model, Print &print, bool painted) {
        init_print({ shelf_fixture() }, print, model, config);
        if (painted)
            paint_and_reapply(print, model, config, [](ModelVolume &mv) {
                // add_volume centred the mesh on its bounding box, whose minimum is the fixture's (0, -3, 0).
                const Vec3f shift = mv.mesh().bounding_box().min.cast<float>() - Vec3f(0.f, -3.f, 0.f);
                const auto  all   = [&shift](const Vec3f &a, const Vec3f &b, const Vec3f &c, int axis, float value) {
                    return std::abs(a[axis] - shift[axis] - value) < 1e-3f && std::abs(b[axis] - shift[axis] - value) < 1e-3f &&
                           std::abs(c[axis] - shift[axis] - value) < 1e-3f;
                };
                // The bar's underside at z 4 and the column's +y face at y 6, two facets each.
                REQUIRE(paint_enforcers(mv, [&all](const Vec3f &a, const Vec3f &b, const Vec3f &c) {
                            return all(a, b, c, 2, 4.f) || all(a, b, c, 1, 6.f);
                        }) == 4);
            });
        print.process();
        REQUIRE(print.objects().size() == 1);
        REQUIRE(print.objects().front()->support_analysis() != nullptr);
        return print.objects().front();
    };
    // The tip polygons on the layer the bar's underside tops whose centroid stands under the bar within 0.4 mm of the
    // column face, inside the 0.5 mm band. The column's first layer places the fixture in the object's centred frame.
    const auto in_band_under_bar = [tip_role](const PrintObject &object) {
        const auto   layers = object.support_layers();
        const size_t top    = top_layer_under(layers, 4.);
        REQUIRE(top != size_t(-1));
        const Point      origin = get_extents(object.layers().front()->lslices).min;
        const FixtureBox band { origin, 6., 2.4, 6.4, 3.6 };
        size_t           count = 0;
        for (const ExPolygon &poly : role_footprint(*layers[top], tip_role)) {
            const Vec2d q = (poly.contour.centroid() - origin).cast<double>() * SCALING_FACTOR;
            UNSCOPED_INFO("tip polygon centroid (" << q.x() << ", " << q.y() << ") on print_z " << layers[top]->print_z);
            if (band.contains(poly.contour.centroid()))
                ++ count;
        }
        return count;
    };

    INFO("support_interface_top_layers " << interface_layers);
    Model plain_model;
    Print plain_print;
    const PrintObject &plain = *run(plain_model, plain_print, false);
    CHECK(in_band_under_bar(plain) == 0);

    Model painted_model;
    Print painted_print;
    const PrintObject             &painted = *run(painted_model, painted_print, true);
    const SupportAnalysis::Report &report  = *painted.support_analysis();
    INFO("painted: tips placed " << report.tips_placed << " routed " << report.tips_routed << " dropped " << report.tips_dropped
                                 << " floating removed " << report.floating_pieces_removed);
    CHECK(in_band_under_bar(painted) >= 1);
    CHECK(report.floating_pieces_removed == 0);
    CHECK(report.tips_placed == report.tips_routed + report.tips_dropped);

    // Each vertical enforcer point is a contact on the first object layer whose top reaches it, at that layer's bottom,
    // and its tip's top layer prints on the support layer that z tops, against the face. The points carry the slicer's
    // scaled xy and z in mm.
    std::vector<Polygons>                enforcers;
    std::vector<std::pair<Vec3f, Vec3f>> vertical_points;
    painted.project_and_append_custom_facets(false, EnforcerBlockerType::ENFORCER, enforcers, &vertical_points);
    REQUIRE(vertical_points.size() == 2);
    const auto layers = painted.support_layers();
    // Where an enforced tip's head may enter the band: within `reach_xy` in x and in y of the painted spot, on the layers
    // at most `reach_z` under its tip. The heads here enter it up to 0.6 mm off their spot and 1.1 mm under their tip.
    // The bar's painted underside tops its tips at z 4.
    const double reach_xy = 1.5, reach_z = 2.5;
    const Point       origin = get_extents(painted.layers().front()->lslices).min;
    std::vector<Reach> reaches { { FixtureBox{ origin, 6. - reach_xy, 2.7 - reach_xy, 12. + reach_xy, 3.3 + reach_xy }.polygon(), 4. } };
    for (const std::pair<Vec3f, Vec3f> &pt_and_normal : vertical_points) {
        const Vec3f &pt = pt_and_normal.first;
        const auto   it = std::find_if(painted.layers().begin(), painted.layers().end(),
                                       [&pt](const Layer *layer) { return float(layer->print_z) >= pt.z(); });
        REQUIRE(it != painted.layers().end());
        const double tip_z = (*it)->bottom_z();
        const size_t top   = top_layer_under(layers, tip_z);
        REQUIRE(top != size_t(-1));
        const Point  spot(coord_t(pt.x()), coord_t(pt.y()));
        const Vec2d  xy = unscale(spot);
        INFO("vertical enforcer point (" << xy.x() << ", " << xy.y() << ", " << pt.z() << "), tip z " << tip_z << ", support layer print_z "
                                         << layers[top]->print_z);
        CHECK_THAT(layers[top]->print_z, WithinAbs(tip_z, 1e-4));
        size_t at_spot = 0;
        for (const ExPolygon &poly : role_footprint(*layers[top], tip_role)) {
            const Vec2d c = unscale(poly.contour.centroid());
            UNSCOPED_INFO("tip polygon centroid (" << c.x() << ", " << c.y() << ")");
            if ((poly.contour.centroid() - spot).cast<double>().norm() <= scale_(1.))
                ++ at_spot;
        }
        CHECK(at_spot >= 1);
        const coord_t r = scale_(reach_xy);
        reaches.push_back({ BoundingBox(spot - Point(r, r), spot + Point(r, r)).polygon(), tip_z });
    }

    // Pillars, bridges and braces keep the band, and only an enforced tip's own head under its tip enters it: base inside
    // the model over the layer's height grown by the xy distance stands within an enforced spot's reach. A clip that
    // skipped the band for all of a layer's base would print the heads' spheres beside the bar above its underside.
    for (const SupportLayer *sl : layers) {
        const ExPolygons in_band =
            intersection_ex(role_footprint(*sl, erSupportMaterial), offset_ex(union_ex(model_over(painted, *sl)), scale_(0.5 - 0.02)));
        INFO("support layer at print_z " << sl->print_z);
        CHECK(diff_ex(in_band, reach_at(reaches, sl->print_z, reach_z)).empty());
    }
}

TEST_CASE("Unseeded feature starts get a scaffold tip while floating debris gets none and slivers beside a wall lean theirs", "[ScaffoldSupport]")
{
    // The spike is too thin to extrude, so no overhang and no contact starts under it, and the cube's contacts stand
    // on the body the bar roots. The spike's island joins 0.6 mm up and takes one birth tip at the spike's middle.
    // The speck never joins and its own height is one 0.2 mm slab: debris, left alone. Each sliver hangs 0.15 mm off
    // the block, its far side 0.35 mm off, past the 0.11 mm the layer below carries at 0.2 mm layers and the 61 degree
    // threshold, so it is an island; a neck straight down from it ends in the block's xy band, and one leaning away
    // from the block clears it, so each takes a birth tip leaning its neck, and both route. The floating part's leg
    // reaches 0.4 mm to the arm that merges it into the cube's island, but the part it ends up in stands 3 mm tall and
    // its top sits 0.8 mm over the leg's bottom: the leg is no debris and gets its tip.
    Print print;
    init_and_process_print({ seeded_islands_fixture() }, print, scaffold_config({ { "support_remove_small_overhang", "1" } }));
    REQUIRE(print.objects().size() == 1);
    const PrintObject &object = *print.objects().front();
    REQUIRE(object.support_analysis() != nullptr);
    const SupportAnalysis::Report &report = *object.support_analysis();
    INFO("tips placed " << report.tips_placed << " routed " << report.tips_routed << " dropped " << report.tips_dropped
                        << " floating removed " << report.floating_pieces_removed);
    CHECK(report.islands_under_held == 0);
    CHECK(report.floating_pieces_removed == 0);

    // The block's first layer places the fixture's (0, 0) in the object's centred frame. The speck is born on slab 15
    // (z 3.0..3.2 at 0.2 mm layers), never meets the rooted body, and tops out on its birth slab.
    const Point                              origin = get_extents(object.layers().front()->lslices).min;
    const std::vector<SupportAnalysis::Slab> slabs  = SupportAnalysis::model_slabs_of(object);
    REQUIRE_FALSE(slabs.empty());
    const SupportAnalysis::IslandMap map = SupportAnalysis::island_joins(slabs, slabs.front().bottom_z);
    // The island owning the lowest piece that holds the fixture's point (x, y), or npos.
    const auto island_under = [&](double x, double y) {
        const Point point = origin + Point::new_scale(x, y);
        for (size_t s = 0; s < slabs.size(); ++ s)
            for (size_t p = map.components.slab_range[s].first; p < map.components.slab_range[s].second; ++ p)
                if (map.components.pieces[p].polygon.contains(point))
                    return map.island_of_piece[p];
        return size_t(-1);
    };
    const size_t speck = island_under(35.15, 5.15);
    REQUIRE(speck < map.islands.size());
    CHECK(map.islands[speck].birth_slab == 15);
    CHECK(map.islands[speck].join_slab == slabs.size());
    CHECK(map.islands[speck].top_slab == map.islands[speck].birth_slab);
    // The leg is born on slab 26 (z 5.2..5.4) and owns its pieces up to slab 27; on slab 28 the arm merges it into the
    // older cube's island, which owns the part up to its top, slab 29 (z 5.8..6.0). Neither meets the rooted body.
    const size_t leg = island_under(47.6, 5.), part = island_under(42., 5.);
    REQUIRE(leg < map.islands.size());
    REQUIRE(part < map.islands.size());
    CHECK(map.islands[leg].birth_slab == 26);
    CHECK(map.islands[leg].join_slab == slabs.size());
    CHECK(map.islands[leg].top_slab == 27);
    CHECK(map.islands[leg].part == part);
    CHECK(map.islands[part].birth_slab == 15);
    CHECK(map.islands[part].join_slab == slabs.size());
    CHECK(map.islands[part].top_slab == 29);
    CHECK(map.islands[part].part == part);
    CHECK(map.islands[speck].part == speck);

    // The seeded tip stands at the spike's bottom, z 3, and a tip's z tops a planned layer, so its disc prints on the
    // layer right under it with no gap.
    const auto   layers = object.support_layers();
    const size_t top    = top_layer_under(layers, 3.);
    REQUIRE(top != size_t(-1));
    INFO("the layer under the spike prints at z " << layers[top]->print_z);
    CHECK_THAT(layers[top]->print_z, WithinAbs(3., 1e-4));
    const FixtureBox under_spike { origin, 13.7, 4.5, 14.7, 5.5 };
    const FixtureBox under_debris { origin, 34.7, 4.7, 35.6, 5.6 };
    const FixtureBox under_sliver { origin, 2.7, 10., 7.3, 10.65 };
    const FixtureBox under_tall_sliver { origin, 2.7, -0.65, 7.3, 0. };
    size_t at_spike = 0;
    for (const ExPolygon &poly : role_footprint(*layers[top], erSupportMaterialInterface)) {
        const Point c = poly.contour.centroid();
        const Vec2d q = (c - origin).cast<double>() * SCALING_FACTOR;
        INFO("interface polygon centroid (" << q.x() << ", " << q.y() << ")");
        if (under_spike.contains(c))
            ++ at_spike;
    }
    CHECK(at_spike >= 1);
    // The leg's seeded tip stands at its bottom, z 5.2, where no contact stands, and prints its disc on the layer whose
    // top is the tip.
    const size_t under_leg_top = top_layer_under(layers, 5.2);
    REQUIRE(under_leg_top != size_t(-1));
    INFO("the layer under the leg prints at z " << layers[under_leg_top]->print_z);
    CHECK_THAT(layers[under_leg_top]->print_z, WithinAbs(5.2, 1e-4));
    const FixtureBox under_leg { origin, 47.1, 4.5, 48.1, 5.5 };
    size_t at_leg = 0;
    for (const ExPolygon &poly : role_footprint(*layers[under_leg_top], erSupportMaterialInterface))
        if (under_leg.contains(poly.contour.centroid()))
            ++ at_leg;
    CHECK(at_leg >= 1);
    size_t at_debris = 0, at_sliver = 0, at_tall_sliver = 0;
    for (const SupportLayer *layer : layers)
        for (const ExPolygon &poly : role_footprint(*layer, erSupportMaterialInterface)) {
            if (under_debris.contains(poly.contour.centroid()))
                ++ at_debris;
            if (under_sliver.contains(poly.contour.centroid()))
                ++ at_sliver;
            if (under_tall_sliver.contains(poly.contour.centroid()))
                ++ at_tall_sliver;
        }
    CHECK(at_debris == 0);
    CHECK(at_sliver >= 1);
    CHECK(at_tall_sliver >= 1);
}

TEST_CASE("Slender scaffold pillars get braces and unreachable ones stand unbraced", "[ScaffoldSupport]")
{
    // Runs that share a bridge length share their pillars: the key also bounds head clustering and routing, and only
    // the linking pass after routing reads the slenderness. A baked 6 x 6 grid 2 mm apart stands the pillars under the slab. The pillars stand straight, so a brace shows as a section
    // of its own between them rather than inside a widened foot.
    struct Reading { size_t unbraced = 0, floating = 0, mid_polygons = 0; double volume_mm3 = 0.; };
    const auto read = [](const char *slenderness, const char *bridge_length) {
        Print print;
        Model model;
        process_grid(print, model,
                     scaffold_config({ { "scaffold_brace_slenderness", slenderness }, { "scaffold_bridge_length", bridge_length },
                                       { "tree_support_branch_diameter_angle", "0" } }),
                     tall_shelf_fixture(), 28., 7., 17., -2., 8., 2.);
        REQUIRE(print.objects().size() == 1);
        const PrintObject &object = *print.objects().front();
        REQUIRE(object.support_analysis() != nullptr);
        const SupportAnalysis::Report &report = *object.support_analysis();
        Reading r { report.pillars_unbraced, report.floating_pieces_removed, 0, report.support_volume_mm3 };
        for (const SupportLayer *sl : object.support_layers())
            if (sl->print_z > 5. && sl->print_z < 25.)
                r.mid_polygons += role_footprint(*sl, erSupportMaterial).size();
        INFO("slenderness " << slenderness << " bridge length " << bridge_length << ": unbraced " << r.unbraced << " floating removed "
                            << r.floating << " support " << r.volume_mm3 << " mm3 tips placed " << report.tips_placed << " routed "
                            << report.tips_routed << " dropped " << report.tips_dropped << " mid polygons " << r.mid_polygons);
        CHECK(r.floating == 0);
        return r;
    };

    // Every pillar is past 15 diameters, and a neighbour within 12 mm takes a chain whose slices sit between the pad
    // and the slab.
    const Reading a = read("15", "12");
    const Reading c = read("40", "12");
    INFO("mid polygons at 15: " << a.mid_polygons << ", at 40: " << c.mid_polygons);
    CHECK(a.unbraced == 0);
    CHECK(c.unbraced == 0);
    CHECK(a.mid_polygons > c.mid_polygons);

    // At a 3 mm reach a pillar with no neighbour that close stays up unbraced, and no pillar is taken away.
    const Reading b = read("15", "3");
    const Reading d = read("40", "3");
    INFO("support at 3 mm reach: " << b.volume_mm3 << " mm3 at 15, " << d.volume_mm3 << " mm3 at 40");
    CHECK(b.unbraced >= 1);
    CHECK(d.unbraced == 0);
    CHECK(b.volume_mm3 >= 0.98 * d.volume_mm3);
}

TEST_CASE("Scaffold braces print at their share of the pillar diameter and never under two support lines", "[ScaffoldSupport]")
{
    // The tall shelf's straight 1.2 mm pillars stand past 15 diameters, so every one takes braces, and the brace
    // diameter changes only the linking pass after routing: the same pillars stand under every run, and the printed
    // footprint between the pad and the slab differs by the braces' sections alone. The fixture's 0.42 mm lines put the
    // two-line floor at 0.84 mm, so 60 % stays at that floor and 10 %, a 0.12 mm brace that would print nothing, prints
    // there too.
    struct Reading { size_t unbraced = 0, floating = 0; double mid_mm2 = 0.; };
    const auto read = [](const char *slenderness, const char *brace_diameter) {
        Print print;
        Model model;
        process_grid(print, model,
                     scaffold_config({ { "scaffold_brace_slenderness", slenderness }, { "scaffold_brace_diameter", brace_diameter },
                                       { "tree_support_branch_diameter_angle", "0" } }),
                     tall_shelf_fixture(), 28., 7., 17., -2., 8., 2.);
        REQUIRE(print.objects().size() == 1);
        const PrintObject &object = *print.objects().front();
        REQUIRE(object.support_analysis() != nullptr);
        const SupportAnalysis::Report &report = *object.support_analysis();
        Reading r { report.pillars_unbraced, report.floating_pieces_removed, 0. };
        for (const SupportLayer *sl : object.support_layers())
            if (sl->print_z > 5. && sl->print_z < 25.)
                r.mid_mm2 += area_mm2(role_footprint(*sl, erSupportMaterial));
        INFO("slenderness " << slenderness << " brace " << brace_diameter << ": unbraced " << r.unbraced << " floating removed "
                            << r.floating << " mid footprint " << r.mid_mm2 << " mm2 over its layers");
        CHECK(r.floating == 0);
        return r;
    };

    const Reading full     = read("15", "100%");
    const Reading slim     = read("15", "60%");
    const Reading thinnest = read("15", "10%");
    const Reading no_brace = read("40", "60%");
    CHECK(full.unbraced == 0);
    CHECK(slim.unbraced == 0);
    CHECK(thinnest.unbraced == 0);
    CHECK(slim.mid_mm2 < full.mid_mm2);
    // 10 % prints at the floor 60 % prints at, so its braces add about as much; unfloored they would add nothing.
    CHECK(thinnest.mid_mm2 - no_brace.mid_mm2 > 0.5 * (slim.mid_mm2 - no_brace.mid_mm2));
}

TEST_CASE("Scaffold pillars widen toward the pad by the branch diameter angle", "[ScaffoldSupport]")
{
    // Under the tall shelf's slab the pillars stand about 27 mm, from the pad's top at 0.6 mm to the heads under z 28.
    // At 5 degrees a pillar gains 0.087 mm of radius per mm below its top, so its 0.6 mm radius reaches about 1.7 mm
    // at z 15 and would reach 2.8 mm at z 2, 22 times the straight disc's area, had the feet not stopped where they
    // meet their neighbours; at 0 degrees it keeps its width down to its base cone. The taper widens the cage the
    // builder routed, so the same tips route under both.
    struct Reading { size_t routed = 0, floating = 0; std::map<int, double> base_mm2; };
    const auto read = [](const char *angle) {
        Print print;
        Model model;
        process_grid(print, model, scaffold_config({ { "tree_support_branch_diameter_angle", angle } }), tall_shelf_fixture(), 28., 7., 17.,
                     -2., 8., 2.);
        REQUIRE(print.objects().size() == 1);
        const PrintObject &object = *print.objects().front();
        REQUIRE(object.support_analysis() != nullptr);
        const SupportAnalysis::Report &report = *object.support_analysis();
        Reading r { report.tips_routed, report.floating_pieces_removed, {} };
        // The area inside the printed outlines under the slab: a pillar prints a ring, so its hole counts.
        const FixtureBox under_slab { get_extents(object.layers().front()->lslices).min, 6.5, -3., 18., 9. };
        const auto       layers = object.support_layers();
        for (int z : { 2, 15, 20 }) {
            const size_t i = top_layer_under(layers, z);
            REQUIRE(i != size_t(-1));
            r.base_mm2[z] = area_mm2(intersection_ex(layers[i]->base_areas, { ExPolygon(under_slab.polygon()) }));
        }
        INFO("angle " << angle << ": routed " << r.routed << " floating removed " << r.floating << " base at z 2, 15, 20: "
                      << r.base_mm2[2] << ", " << r.base_mm2[15] << ", " << r.base_mm2[20] << " mm2");
        CHECK(r.floating == 0);
        return r;
    };

    // A factor of 4 at z 2 leaves room for the feet stopping at their neighbours and for the band beside the column.
    const Reading straight = read("0");
    const Reading tapered  = read("5");
    INFO("base at z 2: " << straight.base_mm2.at(2) << " mm2 straight, " << tapered.base_mm2.at(2) << " mm2 tapered");
    CHECK(tapered.routed == straight.routed);
    CHECK(tapered.base_mm2.at(2) > tapered.base_mm2.at(15));
    // Over the last few millimetres under the slab the heads' bridges to their pillars outweigh the taper, so the
    // pillars are read up to z 20.
    CHECK(tapered.base_mm2.at(15) > tapered.base_mm2.at(20));
    CHECK(tapered.base_mm2.at(2) > 4. * straight.base_mm2.at(2));
}

TEST_CASE("A scaffold under an object lifted off the bed stands on the bed and holds the underside", "[ScaffoldSupport]")
{
    // The shelf fixture lifted 1.5 mm with auto-drop off: the column's 6 x 6 mm underside at z 1.5 is an overhang
    // too near the bed for a head, which needs 2.62 mm above the pad's top, so its tips stand on posts.
    const double lift = 1.5;
    Print print;
    Model model;
    const DynamicPrintConfig config = scaffold_config();
    init_print({ shelf_fixture() }, print, model, config);
    ModelInstance &instance = *model.objects.front()->instances.front();
    instance.auto_drop      = false;
    instance.set_offset(Z, lift);
    DynamicPrintConfig full = DynamicPrintConfig::full_print_config();
    full.apply(config);
    print.apply(model, full);
    // An object with no support under its lowest layer fails G-code layer collection with an empty first layer.
    REQUIRE_NOTHROW(print.process());
    REQUIRE(print.objects().size() == 1);
    const PrintObject &object = *print.objects().front();
    REQUIRE(object.support_analysis() != nullptr);
    const SupportAnalysis::Report &report = *object.support_analysis();
    INFO("tips placed " << report.tips_placed << " routed " << report.tips_routed << " dropped " << report.tips_dropped);
    CHECK(report.tips_placed > 0);
    CHECK(report.tips_dropped == 0);
    CHECK(report.floating_pieces_removed == 0);

    // The pad prints on the bed, not on a ground raised by the lift.
    const auto layers = object.support_layers();
    REQUIRE(layers.size() > 0);
    CHECK_THAT(layers.front()->print_z, WithinAbs(0.2, EPSILON));
    CHECK(layers.front()->has_extrusions());

    // Under the column's underside every support layer from the pad's top up prints inside the column's footprint,
    // and the one the underside rests on prints interface there.
    const auto first_solid = std::find_if(object.layers().begin(), object.layers().end(),
                                          [](const Layer *l) { return ! l->lslices.empty(); });
    REQUIRE(first_solid != object.layers().end());
    const FixtureBox column { get_extents((*first_solid)->lslices).min, 0., 0., 6., 6. };
    const ExPolygons underside { ExPolygon(column.polygon()) };
    const size_t     top = top_layer_under(layers, lift + 0.2);
    REQUIRE(top != size_t(-1));
    for (size_t i = 0; i <= top; ++ i) {
        if (layers[i]->print_z < 0.6 + EPSILON)
            continue;
        INFO("print_z " << layers[i]->print_z);
        CHECK_FALSE(intersection_ex(union_ex(layers[i]->support_fills.polygons_covered_by_width(0.f)), underside).empty());
    }
    CHECK_FALSE(intersection_ex(role_footprint(*layers[top], erSupportMaterialInterface), underside).empty());
}

TEST_CASE("A baked scaffold list builds the tips it holds and reports each point's result", "[ScaffoldSupport]")
{
    Print print;
    Model model;
    const DynamicPrintConfig config = scaffold_config();
    init_print({ shelf_fixture() }, print, model, config);
    ModelObject &mo = *model.objects.front();

    // Writes a list onto `target` as Apply does, the mesh box stamped, and applies the model again.
    const auto bake = [&](ModelObject &target, ScaffoldPoints points, ScaffoldPointsStatus status, const Matrix3d &pose) {
        target.scaffold_points          = std::move(points);
        target.scaffold_points_status   = status;
        target.scaffold_points_pose     = pose;
        target.scaffold_points_mesh_box = target.raw_mesh_bounding_box();
        REQUIRE(reapply(print, model, config) != Print::APPLY_STATUS_UNCHANGED);
    };
    // Slices the plate and reads the report and the record of the PrintObject printing `target`.
    struct Slice { SupportAnalysis::Report report; std::shared_ptr<const ScaffoldRecord> record; };
    const auto slice = [&print](const ModelObject &target) {
        print.process();
        const PrintObject *po = nullptr;
        for (const PrintObject *candidate : print.objects())
            if (candidate->model_object()->id() == target.id())
                po = candidate;
        REQUIRE(po != nullptr);
        REQUIRE(po->support_analysis() != nullptr);
        REQUIRE(po->scaffold_record() != nullptr);
        return Slice{ *po->support_analysis(), po->scaffold_record() };
    };

    // The auto slice records every tip it handed to draw with what became of it, in the raw-mesh frame the list is
    // kept in: the shelf's tips lie on the mesh, where the centred frame draw builds in would put them at negative x.
    print.process();
    REQUIRE(print.objects().size() == 1);
    const PrintObject &object = *print.objects().front();
    REQUIRE(object.support_analysis() != nullptr);
    const SupportAnalysis::Report &report = *object.support_analysis();
    const std::shared_ptr<const ScaffoldRecord> record = object.scaffold_record();
    REQUIRE(record != nullptr);
    CHECK_FALSE(record->baked);
    CHECK_FALSE(record->stale);
    REQUIRE(report.tips_placed > 0);
    CHECK(record->tips.size() == report.tips_placed);
    const size_t routed = size_t(std::count_if(record->tips.begin(), record->tips.end(),
                                               [](const ScaffoldRecord::Tip &tip) { return tip.result == ScaffoldTipResult::Routed; }));
    CHECK(routed == report.tips_routed);
    BoundingBoxf3 mesh_box = mo.raw_mesh_bounding_box();
    mesh_box.offset(0.5);
    for (const ScaffoldRecord::Tip &tip : record->tips) {
        INFO("tip at (" << tip.pos.x() << ", " << tip.pos.y() << ", " << tip.pos.z() << ")");
        CHECK(mesh_box.contains(tip.pos.cast<double>()));
    }
    const size_t         auto_placed     = report.tips_placed;
    const size_t         auto_candidates = report.seeds_candidate;
    const ScaffoldPoints auto_list       = scaffold_points_from(*record);
    const Matrix3d       auto_pose       = record->pose;
    CHECK(auto_candidates > 0);

    // Bake: the routed tips come back as a list. Writing it invalidates the support and keeps the slices, and the
    // slice builds a tip per point without placing a contact seed.
    bake(mo, auto_list, ScaffoldPointsStatus::AutoGenerated, auto_pose);
    REQUIRE(print.objects().size() == 1);
    CHECK(print.objects().front()->is_step_done(posSlice));
    CHECK_FALSE(print.objects().front()->is_step_done(posSupportMaterial));
    const Slice baked = slice(mo);
    INFO("baked: tips placed " << baked.report.tips_placed << " routed " << baked.report.tips_routed);
    CHECK(baked.record->baked);
    CHECK(baked.record->tips.size() == auto_list.size());
    CHECK(baked.report.tips_placed == auto_list.size());
    CHECK(baked.report.tips_routed >= size_t(std::ceil(0.98 * double(baked.report.tips_placed))));
    CHECK(baked.report.seeds_candidate == 0);

    // Hand-added points on the slab, x 6..18, y -3..9, z 12..14: an enforced one on its underside 1 mm clear of every
    // baked point routes, an enforced one on its top face faces up and the builder filters it, one 0.05 mm from a baked
    // point under the slab is an alias of it within sla::D_SP, and one at (3, 3, 12) stands inside the column, where no
    // neck leaning up to 45 degrees ends outside the column's xy band.
    const Vec3d slab_max = mo.raw_mesh_bounding_box().max;
    REQUIRE_THAT(slab_max.x(), WithinAbs(18., 1e-6));
    REQUIRE_THAT(slab_max.y(), WithinAbs(9., 1e-6));
    REQUIRE_THAT(slab_max.z(), WithinAbs(14., 1e-6));
    const auto partner = std::find_if(auto_list.begin(), auto_list.end(),
                                      [](const ScaffoldPoint &pt) { return pt.pos.x() >= 8.f && pt.pos.z() > 11.f; });
    REQUIRE(partner != auto_list.end());
    const size_t j = size_t(partner - auto_list.begin());
    const auto clear_of_baked = [&auto_list](const Vec3f &p) {
        return std::all_of(auto_list.begin(), auto_list.end(), [&p](const ScaffoldPoint &pt) { return (pt.pos - p).norm() >= 1.f; });
    };
    std::optional<Vec3f> under_slab;
    for (float x = 8.f; ! under_slab && x <= 17.f; x += 0.5f)
        for (float y = -2.f; ! under_slab && y <= 8.f; y += 0.5f)
            if (clear_of_baked(Vec3f(x, y, 12.f)))
                under_slab = Vec3f(x, y, 12.f);
    REQUIRE(under_slab.has_value());
    ScaffoldPoints hand = auto_list;
    const size_t   n    = hand.size();
    hand.push_back({ *under_slab, ScaffoldHeadSize::Light, true });
    hand.push_back({ Vec3f(12.f, 3.f, 14.f), ScaffoldHeadSize::Light, true });
    hand.push_back({ auto_list[j].pos + Vec3f(0.05f, 0.f, 0.f), ScaffoldHeadSize::Light, false });
    hand.push_back({ Vec3f(3.f, 3.f, 12.f), ScaffoldHeadSize::Light, false });
    bake(mo, hand, ScaffoldPointsStatus::UserModified, auto_pose);
    const Slice edited = slice(mo);
    REQUIRE(edited.record->tips.size() == hand.size());
    const std::vector<ScaffoldRecord::Tip> &tips = edited.record->tips;
    INFO("underside point (" << under_slab->x() << ", " << under_slab->y() << ", 12), alias partner " << j);
    CHECK(tips[n].result == ScaffoldTipResult::Routed);
    CHECK(tips[n + 1].result == ScaffoldTipResult::Filtered);
    CHECK(size_t(tips[j].result == ScaffoldTipResult::Merged) + size_t(tips[n + 2].result == ScaffoldTipResult::Merged) == 1);
    CHECK(tips[n + 3].result == ScaffoldTipResult::Wall);
    // Each point that did not route counts in the slice's warning: the filtered, merged and wall points at least.
    CHECK(warns(*print.objects().front(), " dropped, 0 islands left without a tip."));
    CHECK_FALSE(warns(*print.objects().front(), " 0 dropped,"));

    // Paint places no tip on a baked slice: enforcers on the slab's underside leave the placed count as it was.
    paint_and_reapply(print, model, config, [](ModelVolume &mv) {
        // add_volume centred the mesh on its bounding box, whose minimum is the fixture's (0, -3, 0).
        const float lift = mv.mesh().bounding_box().min.z();
        REQUIRE(paint_enforcers(mv, [lift](const Vec3f &a, const Vec3f &b, const Vec3f &c) {
                    return std::abs(a.z() - lift - 12.f) < 1e-3f && std::abs(b.z() - lift - 12.f) < 1e-3f &&
                           std::abs(c.z() - lift - 12.f) < 1e-3f;
                }) == 2);
    });
    const Slice painted = slice(mo);
    CHECK(painted.record->baked);
    CHECK(painted.report.tips_placed == edited.report.tips_placed);

    // Another style records nothing, and the scaffold style reads the list again.
    REQUIRE(reapply(print, model, scaffold_config({ { "support_style", "tree_slim" } })) != Print::APPLY_STATUS_UNCHANGED);
    print.process();
    REQUIRE(print.objects().size() == 1);
    CHECK(print.objects().front()->scaffold_record() == nullptr);
    REQUIRE(reapply(print, model, config) != Print::APPLY_STATUS_UNCHANGED);
    const Slice back = slice(mo);
    CHECK(back.record->baked);
    CHECK(back.report.tips_placed == painted.report.tips_placed);

    // An empty list builds no tip.
    bake(mo, {}, ScaffoldPointsStatus::AutoGenerated, auto_pose);
    const Slice empty = slice(mo);
    CHECK(empty.record->baked);
    CHECK(empty.report.tips_placed == 0);
    CHECK(empty.record->tips.empty());

    // Revert: with the paint cleared and no list, the auto slice places what it placed at first.
    paint_and_reapply(print, model, config, [](ModelVolume &mv) { mv.supported_facets.reset(); });
    bake(mo, {}, ScaffoldPointsStatus::NoPoints, Matrix3d::Identity());
    const Slice reverted = slice(mo);
    CHECK_FALSE(reverted.record->baked);
    CHECK(reverted.report.tips_placed == auto_placed);

    // A copy sharing the source's meshes prints its own list: the source's list builds on the source only.
    bake(mo, auto_list, ScaffoldPointsStatus::AutoGenerated, auto_pose);
    // Model::add_object sets extruder 1 on the copy, as the GUI's objects carry it; without it on the source too the
    // configs differ and Print::process never shares the layers this leg is about.
    mo.config.set_key_value("extruder", new ConfigOptionInt(1));
    ModelObject &copy = *model.add_object(mo);
    copy.instances.front()->set_offset(copy.instances.front()->get_offset() + Vec3d(40., 0., 0.));
    bake(copy, {}, ScaffoldPointsStatus::AutoGenerated, auto_pose);
    REQUIRE(print.objects().size() == 2);
    const Slice source = slice(mo);
    const Slice copied = slice(copy);
    CHECK(source.record->baked);
    CHECK(source.record->tips.size() == auto_list.size());
    CHECK(copied.record->baked);
    CHECK(copied.record->tips.empty());
    model.delete_object(model.objects.size() - 1);
    REQUIRE(reapply(print, model, config) != Print::APPLY_STATUS_UNCHANGED);

    // Heavy heads print more support than light ones on the same points. The heavy list goes first: the auto list
    // may be all light already.
    ScaffoldPoints light = auto_list, heavy = auto_list;
    for (ScaffoldPoint &pt : light)
        pt.size = ScaffoldHeadSize::Light;
    for (ScaffoldPoint &pt : heavy)
        pt.size = ScaffoldHeadSize::Heavy;
    bake(mo, heavy, ScaffoldPointsStatus::AutoGenerated, auto_pose);
    const Slice heavier = slice(mo);
    bake(mo, light, ScaffoldPointsStatus::AutoGenerated, auto_pose);
    const Slice lighter = slice(mo);
    CHECK(heavier.record->baked);
    CHECK(heavier.report.support_volume_mm3 > lighter.report.support_volume_mm3);

    // Bare islands: the auto slice tips the fixture's islands and names as many as it counts unheld, while an empty
    // list gives no tip to an island the planner's rule tips, and the record names each such island.
    Print                    islands_print;
    Model                    islands_model;
    const DynamicPrintConfig islands_config = scaffold_config({ { "support_remove_small_overhang", "1" } });
    init_print({ seeded_islands_fixture() }, islands_print, islands_model, islands_config);
    islands_print.process();
    REQUIRE(islands_print.objects().size() == 1);
    REQUIRE(islands_print.objects().front()->scaffold_record() != nullptr);
    REQUIRE(islands_print.objects().front()->support_analysis() != nullptr);
    CHECK(islands_print.objects().front()->scaffold_record()->bare_islands.size() ==
          islands_print.objects().front()->support_analysis()->islands_under_held);
    ModelObject &islands = *islands_model.objects.front();
    islands.scaffold_points_status   = ScaffoldPointsStatus::AutoGenerated;
    islands.scaffold_points_pose     = islands_print.objects().front()->scaffold_record()->pose;
    islands.scaffold_points_mesh_box = islands.raw_mesh_bounding_box();
    REQUIRE(reapply(islands_print, islands_model, islands_config) != Print::APPLY_STATUS_UNCHANGED);
    islands_print.process();
    REQUIRE(islands_print.objects().size() == 1);
    const std::shared_ptr<const ScaffoldRecord> bare = islands_print.objects().front()->scaffold_record();
    REQUIRE(bare != nullptr);
    CHECK(bare->baked);
    CHECK(bare->tips.empty());
    CHECK_FALSE(bare->bare_islands.empty());
    CHECK(warns(*islands_print.objects().front(), "0 dropped, " + std::to_string(bare->bare_islands.size()) + " islands left without a tip."));
}

TEST_CASE("A list copied from an auto slice's routed tips slices on its support layers and routes what it routed", "[ScaffoldSupport]")
{
    // The support layers are planned at the contact layers as well as at the tips' tops, so a baked slice runs the contact
    // pass though it builds no tip from it, and a copied point stands at its tip's position to the scaled unit, since the
    // builder's routing turns on single units. Every tip of the auto slice routes on the shelf and on a 4 mm plank whose
    // underside falls at 45 degrees, where the contacts bound layers no tip tops, so the list Generate copies holds them
    // all: sliced from it, the support layers stand where the auto slice's stood, every point routes and the island
    // count is the auto slice's. The volume matches to the list's float positions, which hold the scaled unit only
    // within 16 mm of the raw origin: the shelf reaches 18 mm.
    const bool               slope = GENERATE(false, true);
    Print                    print;
    Model                    model;
    const DynamicPrintConfig config = scaffold_config();
    INFO((slope ? "the sloped plank" : "the shelf"));
    init_print({ slope ? slope_fixture(4.) : shelf_fixture() }, print, model, config);
    print.process();
    REQUIRE(print.objects().size() == 1);
    const auto layers_of = [](const PrintObject &object) {
        std::vector<std::pair<double, double>> out;
        for (const SupportLayer *layer : object.support_layers())
            out.emplace_back(layer->print_z, layer->height);
        return out;
    };
    REQUIRE(print.objects().front()->support_analysis() != nullptr);
    const SupportAnalysis::Report               auto_report = *print.objects().front()->support_analysis();
    const std::shared_ptr<const ScaffoldRecord> record      = print.objects().front()->scaffold_record();
    REQUIRE(record != nullptr);
    REQUIRE(auto_report.tips_placed > 0);
    REQUIRE(auto_report.tips_routed == auto_report.tips_placed);
    const std::vector<std::pair<double, double>> auto_layers = layers_of(*print.objects().front());

    ModelObject &mo             = *model.objects.front();
    mo.scaffold_points          = scaffold_points_from(*record);
    mo.scaffold_points_status   = ScaffoldPointsStatus::AutoGenerated;
    mo.scaffold_points_pose     = record->pose;
    mo.scaffold_points_mesh_box = mo.raw_mesh_bounding_box();
    REQUIRE(reapply(print, model, config) != Print::APPLY_STATUS_UNCHANGED);
    print.process();
    REQUIRE(print.objects().size() == 1);
    const PrintObject &object = *print.objects().front();
    REQUIRE(object.support_analysis() != nullptr);
    const std::shared_ptr<const ScaffoldRecord> baked = object.scaffold_record();
    REQUIRE(baked != nullptr);
    REQUIRE(baked->baked);
    CHECK(baked->tips.size() == auto_report.tips_routed);
    CHECK(std::all_of(baked->tips.begin(), baked->tips.end(), [](const ScaffoldRecord::Tip &tip) { return tip.result == ScaffoldTipResult::Routed; }));
    CHECK(object.support_analysis()->islands_under_held == auto_report.islands_under_held);

    const std::vector<std::pair<double, double>> baked_layers = layers_of(object);
    INFO("support layers: auto " << auto_layers.size() << ", baked " << baked_layers.size());
    REQUIRE(baked_layers.size() == auto_layers.size());
    for (size_t i = 0; i < auto_layers.size(); ++ i) {
        INFO("support layer " << i);
        CHECK_THAT(baked_layers[i].first, WithinAbs(auto_layers[i].first, 1e-9));
        CHECK_THAT(baked_layers[i].second, WithinAbs(auto_layers[i].second, 1e-9));
    }
    CHECK_THAT(object.support_analysis()->support_volume_mm3, Catch::Matchers::WithinRel(auto_report.support_volume_mm3, 1e-4));
}

TEST_CASE("A support blocker added after the bake leaves a baked slice's support layers as they were", "[ScaffoldSupport]")
{
    // A baked slice's contacts bound its planned layers, and it reads their overhangs with no paint and no blocker, as the
    // list ignores both. On the 4 mm plank whose underside falls at 45 degrees a blocker over the underside, x 6.5..11.5,
    // y -1..7, z 4..9, would take the contacts off the layers it covers and move the support layers there; with the list
    // in use the support layers, each point's result and the support volume stay as they were.
    Print                    print;
    Model                    model;
    const DynamicPrintConfig config = scaffold_config();
    init_print({ slope_fixture(4.) }, print, model, config);
    print.process();
    REQUIRE(print.objects().size() == 1);
    const std::shared_ptr<const ScaffoldRecord> record = print.objects().front()->scaffold_record();
    REQUIRE(record != nullptr);
    ModelObject &mo             = *model.objects.front();
    mo.scaffold_points          = scaffold_points_from(*record);
    mo.scaffold_points_status   = ScaffoldPointsStatus::AutoGenerated;
    mo.scaffold_points_pose     = record->pose;
    mo.scaffold_points_mesh_box = mo.raw_mesh_bounding_box();
    REQUIRE(reapply(print, model, config) != Print::APPLY_STATUS_UNCHANGED);
    // The support layers, each point's result and the support volume of the slice `print` last processed.
    struct Slice
    {
        std::vector<std::pair<double, double>> layers;
        std::vector<ScaffoldTipResult>         results;
        double                                 volume_mm3 = 0.;
    };
    const auto slice = [&print]() {
        print.process();
        REQUIRE(print.objects().size() == 1);
        const PrintObject &object = *print.objects().front();
        REQUIRE(object.support_analysis() != nullptr);
        REQUIRE(object.scaffold_record() != nullptr);
        REQUIRE(object.scaffold_record()->baked);
        Slice out;
        for (const SupportLayer *layer : object.support_layers())
            out.layers.emplace_back(layer->print_z, layer->height);
        for (const ScaffoldRecord::Tip &tip : object.scaffold_record()->tips)
            out.results.push_back(tip.result);
        out.volume_mm3 = object.support_analysis()->support_volume_mm3;
        return out;
    };
    const Slice open = slice();

    // The model part's volume carries the offset that centres its mesh, so a volume added uncentred stands in the
    // fixture's own frame.
    TriangleMesh blocker = box(6.5, -1., 4., 5., 8., 5.);
    mo.add_volume(std::move(blocker), ModelVolumeType::SUPPORT_BLOCKER, false);
    REQUIRE(reapply(print, model, config) != Print::APPLY_STATUS_UNCHANGED);
    const Slice blocked = slice();
    const std::vector<Polygons> blockers = print.objects().front()->slice_support_blockers();
    REQUIRE(std::any_of(blockers.begin(), blockers.end(), [](const Polygons &layer) { return ! layer.empty(); }));

    INFO("support layers: open " << open.layers.size() << ", blocked " << blocked.layers.size());
    REQUIRE(blocked.layers.size() == open.layers.size());
    for (size_t i = 0; i < open.layers.size(); ++ i) {
        INFO("support layer " << i);
        CHECK_THAT(blocked.layers[i].first, WithinAbs(open.layers[i].first, 1e-9));
        CHECK_THAT(blocked.layers[i].second, WithinAbs(open.layers[i].second, 1e-9));
    }
    CHECK(blocked.results == open.results);
    CHECK_THAT(blocked.volume_mm3, WithinAbs(open.volume_mm3, 1e-9));
}

TEST_CASE("A baked list goes stale under a tilt and stays valid under a Z rotation", "[ScaffoldSupport]")
{
    Print print;
    Model model;
    const DynamicPrintConfig config = scaffold_config();
    init_print({ shelf_fixture() }, print, model, config);
    ModelObject *mo = model.objects.front();

    print.process();
    REQUIRE(print.objects().size() == 1);
    const std::shared_ptr<const ScaffoldRecord> auto_record = print.objects().front()->scaffold_record();
    REQUIRE(auto_record != nullptr);
    REQUIRE_FALSE(auto_record->tips.empty());
    mo->scaffold_points          = scaffold_points_from(*auto_record);
    mo->scaffold_points_status   = ScaffoldPointsStatus::AutoGenerated;
    mo->scaffold_points_pose     = auto_record->pose;
    mo->scaffold_points_mesh_box = mo->raw_mesh_bounding_box();

    // Two instances of one object in two poses print as two PrintObjects, each judging the list against its own pose:
    // a quarter turn about Z keeps the list, a 30 degree tilt about X leaves it stale.
    mo->add_instance(*mo->instances.front());
    ModelInstance &turned = *mo->instances[0];
    ModelInstance &tilted = *mo->instances[1];
    turned.set_rotation(Vec3d(0., 0., PI / 2.));
    turned.set_offset(turned.get_offset() + Vec3d(30., 0., 0.));
    tilted.set_rotation(Vec3d(PI / 6., 0., 0.));
    tilted.set_offset(tilted.get_offset() - Vec3d(30., 0., 0.));
    mo->ensure_on_bed();

    // Slices the plate and reads the record of the PrintObject printing instance `k`: model_instance points into the
    // Print's own copy of the model, which keeps the instance's id.
    const auto slice = [&]() {
        REQUIRE(reapply(print, model, config) != Print::APPLY_STATUS_UNCHANGED);
        print.process();
        REQUIRE(print.objects().size() == 2);
    };
    const auto object_of = [&](size_t k) -> const PrintObject & {
        const PrintObject *po = nullptr;
        for (const PrintObject *candidate : print.objects())
            if (candidate->instances().front().model_instance->id() == mo->instances[k]->id())
                po = candidate;
        REQUIRE(po != nullptr);
        REQUIRE(po->scaffold_record() != nullptr);
        return *po;
    };
    const auto warns_stale = [](const PrintObject &po) { return warns(po, "stale"); };

    slice();
    CHECK(object_of(0).scaffold_record()->baked);
    CHECK_FALSE(object_of(0).scaffold_record()->stale);
    CHECK_FALSE(warns_stale(object_of(0)));
    // The list maps through the turned instance's rotation, so support prints at every routed point's place in that
    // instance's frame. A tip mapped without the turn lands in open air and still routes to the bed, so where the
    // support prints, not the routed count, is what tells the two apart.
    {
        const PrintObject &po = object_of(0);
        REQUIRE(po.support_analysis() != nullptr);
        CHECK(po.support_analysis()->tips_placed == mo->scaffold_points.size());
        const std::shared_ptr<const ScaffoldRecord> record = po.scaffold_record();
        size_t routed = 0, printed_at = 0;
        for (const ScaffoldRecord::Tip &tip : record->tips) {
            if (tip.result != ScaffoldTipResult::Routed)
                continue;
            ++ routed;
            const Vec3d q    = po.trafo_centered() * tip.pos.cast<double>();
            const Point at   = Point::new_scale(q.x(), q.y());
            bool        near = false;
            for (const SupportLayer *sl : po.support_layers())
                if (std::abs(sl->print_z - q.z()) <= 0.4) {
                    Polygons covered;
                    std::vector<const ExtrusionEntity *> entities;
                    collect_entities(sl->support_fills, entities);
                    for (const ExtrusionEntity *e : entities)
                        e->polygons_covered_by_width(covered, 0.f);
                    near = near || ! intersection_ex(offset(covered, scale_(0.5)), Polygons{ Polygon({ at, at + Point(1, 0), at + Point(0, 1) }) }).empty();
                }
            printed_at += near;
        }
        REQUIRE(routed > 0);
        CHECK(printed_at == routed);
    }
    CHECK_FALSE(object_of(1).scaffold_record()->baked);
    CHECK(object_of(1).scaffold_record()->stale);
    CHECK(warns_stale(object_of(1)));

    // A scale changes the lengths the list was placed at.
    turned.set_scaling_factor(Vec3d::Constant(1.1));
    slice();
    CHECK_FALSE(object_of(0).scaffold_record()->baked);
    CHECK(object_of(0).scaffold_record()->stale);

    // A part moved inside the object moves the raw mesh under the list, which the stamped mesh box catches.
    turned.set_scaling_factor(Vec3d::Ones());
    mo->volumes.front()->set_offset(mo->volumes.front()->get_offset() + Vec3d(1., 0., 0.));
    slice();
    CHECK(object_of(0).scaffold_record()->stale);
    CHECK(object_of(1).scaffold_record()->stale);
}

TEST_CASE("Converting a pre-supported model leaves the figure and one scaffold point per artist tip", "[ScaffoldSupport]")
{
    // The instance turned about Z or not: the points come back in the raw-mesh frame the fixture was built in, and an
    // axis is clamped to the head tilt cap in the world, where the builder reads it.
    const double turn = GENERATE(0., PI / 6.);
    Print        print;
    Model        model;
    init_print({ presupported_fixture() }, print, model, scaffold_config());
    ModelObject   &mo       = *model.objects.front();
    ModelInstance &instance = *mo.instances.front();
    instance.set_rotation(Vec3d(0., 0., turn));
    // A second instance shares the mesh: no bed drop may take its lift either.
    ModelInstance &second = *mo.add_instance(instance);
    second.set_offset(instance.get_offset() + Vec3d(40., 0., 0.));
    mo.ensure_on_bed();
    const Vec3d world_before = mo.instance_bounding_box(0).min;
    REQUIRE_THAT(world_before.z(), WithinAbs(0., 1e-6));

    const PresupportedConversion::Summary summary = PresupportedConversion::convert(mo, 0);
    CHECK(summary.refusal == PresupportedConversion::Refusal::None);
    CHECK(summary.tips_converted == 6);
    CHECK(summary.duplicates_removed == 1);
    CHECK(summary.axes_clamped == 2);
    CHECK(summary.micro_struts_dropped == 1);
    CHECK(summary.tips_rooted_on_figure == 1);

    // The figure alone is left, its three shells, where the artist posed it, and no bed drop takes its lift.
    REQUIRE(mo.volumes.size() == 1);
    CHECK(mo.volumes.front()->mesh().facets_count() == 12 + presupported_staff().facets_count() + presupported_eye().facets_count());
    const BoundingBoxf3 raw = mo.raw_mesh_bounding_box();
    for (int c = 0; c < 3; ++c) {
        CHECK_THAT(raw.min[c], WithinAbs(Vec3d(0., -1., 6.)[c], 1e-5));
        CHECK_THAT(raw.max[c], WithinAbs(Vec3d(20., 12., 18.)[c], 1e-5));
    }
    for (size_t i = 0; i < mo.instances.size(); ++i) {
        CHECK_FALSE(mo.instances[i]->auto_drop);
        CHECK_THAT(mo.instance_bounding_box(i).min.z(), WithinAbs(6., 1e-5));
    }
    mo.ensure_on_bed();
    for (size_t i = 0; i < mo.instances.size(); ++i)
        CHECK_THAT(mo.instance_bounding_box(i).min.z(), WithinAbs(6., 1e-5));

    // The list: every point enforced, at its tip's contact site, aimed from the site toward the tip's wide end, a lean
    // past 45 degrees moved onto the cap with its azimuth kept and the rooted tip's straight-up axis leaning toward
    // the world's +x, Heavy from a 0.45 mm contact.
    CHECK(mo.scaffold_points_status == ScaffoldPointsStatus::UserModified);
    CHECK(mo.scaffold_points_pose.isApprox(instance.get_matrix().linear()));
    CHECK(mo.scaffold_points_mesh_box.min.isApprox(raw.min));
    CHECK(mo.scaffold_points_mesh_box.max.isApprox(raw.max));
    const double cap      = std::sin(PI / 4.);
    const Vec3d  up_leans = instance.get_matrix().linear().inverse() * Vec3d(cap, 0., -cap);
    const struct { FixtureTip tip; Vec3d axis; ScaffoldHeadSize size; } expected[] = {
        { presupported_light, -Vec3d::UnitZ(), ScaffoldHeadSize::Light },
        { presupported_heavy, -Vec3d::UnitZ(), ScaffoldHeadSize::Heavy },
        { presupported_leaning, Vec3d(-cap, 0., -cap), ScaffoldHeadSize::Light },
        { presupported_at_cut, -Vec3d::UnitZ(), ScaffoldHeadSize::Heavy },
        { presupported_under_cut, -Vec3d::UnitZ(), ScaffoldHeadSize::Light },
        { presupported_rooted, up_leans, ScaffoldHeadSize::Light },
    };
    REQUIRE(mo.scaffold_points.size() == std::size(expected));
    for (const auto &e : expected) {
        const auto point = std::find_if(mo.scaffold_points.begin(), mo.scaffold_points.end(),
                                        [&e](const ScaffoldPoint &p) { return (p.pos.cast<double>() - e.tip.site).norm() < 1e-3; });
        INFO("tip at (" << e.tip.site.x() << ", " << e.tip.site.y() << ", " << e.tip.site.z() << ")");
        REQUIRE(point != mo.scaffold_points.end());
        CHECK(point->enforced);
        CHECK(point->size == e.size);
        for (int c = 0; c < 3; ++c)
            CHECK_THAT(point->axis[c], WithinAbs(e.axis[c], 1e-4));
    }
}

TEST_CASE("Converting a pre-supported model strips a raft taller than a flat piece and keeps a figure that stands on a raft", "[ScaffoldSupport]")
{
    // "thick raft": the fixture's trunks sunk in one 2 mm raft box at x 2.5..15, y 1.5..10.5 in place of their pads,
    // taller than a flat raft piece and lower than it is wide. "figure on raft": the slab joined to a 2 x 2 mm foot
    // post at x 17.5..19.5, y 9.5..11.5 from z 0.5 into the slab, one non-convex shell, standing on a 3 x 3 x 0.5 mm
    // pad of its own clear of the trunks, the fixture's pads, trunks, tips and brace kept.
    const std::string kind   = GENERATE(as<std::string>{}, "thick raft", "figure on raft");
    TriangleMesh      figure = presupported_slab();
    TriangleMesh      fixture;
    double            figure_bottom = 6.;
    if (kind == "thick raft") {
        fixture           = presupported_fixture(figure, false);
        TriangleMesh raft = make_cube(12.5, 9., 2.);
        raft.translate(2.5f, 1.5f, 0.f);
        fixture.merge(raft);
    } else {
        TriangleMesh post = make_cube(2., 2., 6.5);
        post.translate(17.5f, 9.5f, 0.5f);
        MeshBoolean::cgal::plus(figure, post);
        REQUIRE(its_split(figure.its).size() == 1);
        figure_bottom = 0.5;
        fixture       = presupported_fixture(figure);
        TriangleMesh pad = make_cube(3., 3., 0.5);
        pad.translate(17.f, 9.f, 0.f);
        fixture.merge(pad);
    }
    CAPTURE(kind);
    Model        model;
    ModelObject &mo = *model.add_object();
    mo.add_volume(fixture);
    mo.add_instance();

    const PresupportedConversion::Summary summary = PresupportedConversion::convert(mo, 0);
    CHECK(summary.refusal == PresupportedConversion::Refusal::None);
    CHECK(summary.tips_converted == 6);
    // Every trunk reaches the plate through the raft; only the tip on the slab's top face stands on the figure.
    CHECK(summary.tips_rooted_on_figure == 1);

    // The figure alone is left, with its staff and eye, and neither raft piece.
    REQUIRE(mo.volumes.size() == 1);
    CHECK(mo.volumes.front()->mesh().facets_count() == figure.facets_count() + presupported_staff().facets_count() + presupported_eye().facets_count());
    const BoundingBoxf3 raw = mo.raw_mesh_bounding_box();
    for (int c = 0; c < 3; ++c) {
        CHECK_THAT(raw.min[c], WithinAbs(Vec3d(0., -1., figure_bottom)[c], 1e-5));
        CHECK_THAT(raw.max[c], WithinAbs(Vec3d(20., 12., 18.)[c], 1e-5));
    }
}

TEST_CASE("Converting a model with no separable artist tip leaves it unchanged and says why", "[ScaffoldSupport]")
{
    using PresupportedConversion::Refusal;
    const std::string kind = GENERATE(as<std::string>{}, "plain", "welded", "tipless", "two parts");
    Model        model;
    ModelObject &mo = *model.add_object();
    TriangleMesh figure = make_cube(20., 12., 4.);
    figure.translate(0.f, 0.f, 6.f);
    Refusal refusal = Refusal::NoSupports;
    if (kind == "plain")
        mo.add_volume(figure);
    else if (kind == "welded") {
        // A tip sharing the slab's surface makes one shell with it, which no split separates.
        std::vector<Vec3f> points = figure.its.vertices;
        append(points, artist_tip(presupported_light).its.vertices);
        mo.add_volume(TriangleMesh(its_convex_hull(points)));
    } else if (kind == "tipless") {
        // A trunk on a raft pad under the slab, with no tip on it.
        TriangleMesh pad = make_cube(3., 3., 0.5);
        pad.translate(2.5f, 2.5f, 0.f);
        figure.merge(pad);
        figure.merge(artist_rod(Vec3d(4., 4., 0.4), Vec3d(4., 4., 6.), 0.4));
        mo.add_volume(figure);
        refusal = Refusal::NoArtistTips;
    } else {
        mo.add_volume(presupported_fixture());
        mo.add_volume(make_cube(2., 2., 2.));
        refusal = Refusal::SeveralParts;
    }
    mo.add_instance();
    // A list the user placed before, which a refusal must keep.
    const ScaffoldPoints placed = { { Vec3f(5.f, 5.f, 6.f), ScaffoldHeadSize::Heavy, true } };
    mo.scaffold_points          = placed;
    mo.scaffold_points_status   = ScaffoldPointsStatus::UserModified;
    mo.scaffold_points_mesh_box = mo.raw_mesh_bounding_box();
    const size_t   facets    = mo.volumes.front()->mesh().facets_count();
    const ObjectID volume_id = mo.volumes.front()->id();

    DYNAMIC_SECTION(kind) {
        size_t                                changes = 0;
        const PresupportedConversion::Summary summary = PresupportedConversion::convert(mo, 0, [&changes]() { ++ changes; });
        CHECK(summary.refusal == refusal);
        CHECK(summary.tips_converted == 0);
        CHECK(changes == 0);
        CHECK(mo.volumes.front()->mesh().facets_count() == facets);
        CHECK(mo.volumes.front()->id() == volume_id);
        CHECK(mo.scaffold_points_status == ScaffoldPointsStatus::UserModified);
        CHECK(mo.scaffold_points == placed);
        CHECK(mo.instances.front()->auto_drop);
    }
}

TEST_CASE("A converted pre-supported model slices its list and aims each head along its point's axis", "[ScaffoldSupport]")
{
    // The instance turned a quarter about Z or not: the builder reads each stored axis through the instance's turn.
    const double             turn = GENERATE(0., PI / 2.);
    Print                    print;
    Model                    model;
    const DynamicPrintConfig config = scaffold_config();
    init_print({ presupported_fixture() }, print, model, config);
    ModelObject &mo = *model.objects.front();
    mo.instances.front()->set_rotation(Vec3d(0., 0., turn));
    REQUIRE(PresupportedConversion::convert(mo, 0).refusal == PresupportedConversion::Refusal::None);
    REQUIRE(reapply(print, model, config) != Print::APPLY_STATUS_UNCHANGED);
    REQUIRE_NOTHROW(print.process());
    REQUIRE(print.objects().size() == 1);
    const PrintObject &object = *print.objects().front();
    const std::shared_ptr<const ScaffoldRecord> record = object.scaffold_record();
    REQUIRE(record != nullptr);
    CHECK(record->baked);
    CHECK_FALSE(record->stale);
    REQUIRE(record->tips.size() == mo.scaffold_points.size());

    // The lifted figure's pad prints on the bed.
    const auto layers = object.support_layers();
    REQUIRE(layers.size() > 0);
    CHECK_THAT(layers.front()->print_z, WithinAbs(0.2, EPSILON));

    const auto result_at = [&](const FixtureTip &tip) {
        const auto found = std::find_if(record->tips.begin(), record->tips.end(),
                                        [&tip](const ScaffoldRecord::Tip &t) { return (t.pos.cast<double>() - tip.site).norm() < 1e-3; });
        REQUIRE(found != record->tips.end());
        INFO("tip at (" << tip.site.x() << ", " << tip.site.y() << ", " << tip.site.z() << ") reads "
                        << Catch::StringMaker<ScaffoldTipResult>::convert(found->result));
        return found->result;
    };
    CHECK(result_at(presupported_light) == ScaffoldTipResult::Routed);
    CHECK(result_at(presupported_heavy) == ScaffoldTipResult::Routed);
    CHECK(result_at(presupported_leaning) == ScaffoldTipResult::Routed);
    CHECK(result_at(presupported_at_cut) == ScaffoldTipResult::Routed);
    CHECK(result_at(presupported_under_cut) == ScaffoldTipResult::Routed);
    // The tip rooted on the slab's top face points down onto it, where no head fits.
    CHECK(result_at(presupported_rooted) != ScaffoldTipResult::Routed);

    // 1.2 mm under the leaning tip, clamped to 45 degrees toward the fixture's -x, its head holds the point 1.2 mm
    // along the fixture's -x, turned with the instance, and not the point straight under the site, where a head along
    // the mesh normal would stand.
    const Vec3d  site  = object.trafo_centered() * presupported_leaning.site;
    const Vec3d  along = object.trafo_centered().linear() * Vec3d(-1.2, 0., 0.);
    const double z     = site.z() - 1.2;
    const auto   layer = std::find_if(layers.begin(), layers.end(), [z](const SupportLayer *l) { return l->print_z - l->height <= z && z < l->print_z; });
    REQUIRE(layer != layers.end());
    // A head prints as a ring, so each piece's outer contour is what it holds.
    const ExPolygons head    = footprint(**layer);
    const auto       encloses = [&head](const Point &p) {
        return std::any_of(head.begin(), head.end(), [&p](const ExPolygon &piece) { return piece.contour.contains(p); });
    };
    INFO("support at z " << (*layer)->print_z << ": " << head.size() << " pieces");
    CHECK(encloses(Point::new_scale(site.x() + along.x(), site.y() + along.y())));
    CHECK_FALSE(encloses(Point::new_scale(site.x(), site.y())));
}

// Hidden ([.]): four full Print::process() passes over a 993k-facet miniature at 0.06 mm layers, minutes in
// total, and the model lives outside the repo under $ORCA_MINIATURE_CORPUS (docs/miniature_support_validation.md).
// It gates the style on plate 3 of the corpus against a tree-slim slice measured in the same run.
TEST_CASE("Scaffold support over corpus plate 3 in two poses", "[ScaffoldSupport][.]")
{
    SupportValidation::use_os_temporary_dir();

    const char *env = std::getenv("ORCA_MINIATURE_CORPUS");
    if (env == nullptr || *env == '\0') {
        std::cout << "corpus dir not set" << std::endl;
        return;
    }

    SupportValidation::Manifest m;
    m.version    = 1;
    m.model_root = env;
    SupportValidation::ManifestCase c;
    c.id            = "plate3";
    c.model         = "elf_test.3mf";
    c.sha256        = "201c541805e94a2914c3cf0a0aebaee68ee3f6199cc096ae36993069fc781ba6";
    c.selectors     = { "name:10_Dark Elves 3_test.stl" };
    c.styles        = { "tree_slim", "tree_scaffold" };
    c.feature_modes = { "on" };
    c.repeats       = 1;

    const char   *results = std::getenv("ORCA_SCAFFOLD_RESULTS");
    std::ofstream file;
    if (results != nullptr && *results != '\0') {
        file.open(results);
        REQUIRE(file.good());
    }
    std::ostream &rows = file.is_open() ? static_cast<std::ostream &>(file) : std::cout;

    struct Reading { SupportValidation::Metrics metrics; double elapsed_s = 0.; double island_joins_s = 0.; };
    std::map<std::string, std::map<std::string, Reading>> readings; // pose, then style
    for (const std::string pose : { "stored", "upright" })
        for (const std::string &style : c.styles) {
            SupportValidation::CorpusObject object = SupportValidation::case_object(m, c, fixture_config({ { "support_top_z_distance", "0.2" } }), style, "on");
            // The project's own settings carry 0.06 mm layers; the base's 0.2 means the file's config never loaded.
            REQUIRE_THAT(object.config.opt_float("layer_height"), WithinAbs(0.06, 1e-9));
            if (pose == "upright")
                stand_upright(object.model, object.config);

            Print print;
            print.set_status_silent();
            print.apply(object.model, object.config);
            print.request_legacy_support_analysis();
            const auto start = std::chrono::steady_clock::now();
            print.process();
            Reading &r  = readings[pose][style];
            r.elapsed_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

            SupportValidation::CaseResult row;
            row.harness      = "scaffold_support";
            row.case_id      = c.id;
            row.style        = style;
            row.feature_mode = pose;
            row.elapsed_s    = r.elapsed_s;
            SupportValidation::read_print_analyses(print, row);
            r.metrics = row.metrics;

            if (style == "tree_scaffold") {
                // Zero here means accumulate_metrics dropped the count on its way into the row.
                REQUIRE(row.metrics.tips_placed > 0);
                const auto slabs      = SupportAnalysis::model_slabs_of(*print.objects().front());
                const auto join_start = std::chrono::steady_clock::now();
                SupportAnalysis::island_joins(slabs, slabs.front().bottom_z);
                r.island_joins_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - join_start).count();
            }
            SupportValidation::write_result(row, rows);
            std::cout << pose << " " << style << ": tips placed " << r.metrics.tips_placed << " / routed " << r.metrics.tips_routed
                      << " / dropped " << r.metrics.tips_dropped << ", floating removed " << r.metrics.floating_pieces_removed
                      << ", islands under-held " << r.metrics.islands_under_held << ", pillars unbraced " << r.metrics.pillars_unbraced
                      << ", support " << r.metrics.support_volume_mm3 << " mm3, process " << r.elapsed_s << " s, island_joins "
                      << r.island_joins_s << " s" << std::endl;
        }
    if (file.is_open()) {
        file.close();
        REQUIRE(file.good());
    }

    // Every slice is written before any cap is read, so a failing pose still leaves all four rows behind.
    for (const std::string pose : { "stored", "upright" }) {
        const Reading &slim     = readings[pose]["tree_slim"];
        const Reading &scaffold = readings[pose]["tree_scaffold"];
        INFO("pose " << pose);
        {
            INFO("tips dropped " << scaffold.metrics.tips_dropped << " of " << scaffold.metrics.tips_placed << " placed");
            REQUIRE(scaffold.metrics.tips_dropped <= scaffold.metrics.tips_placed / 5);
        }
        {
            INFO("floating pieces removed " << scaffold.metrics.floating_pieces_removed << " (tree slim " << slim.metrics.floating_pieces_removed << ")");
            REQUIRE(scaffold.metrics.floating_pieces_removed == 0);
        }
        {
            const double wall_cap = 1.5;
            INFO("process wall " << scaffold.elapsed_s << " s against tree slim " << slim.elapsed_s << " s, cap " << wall_cap << "x");
            REQUIRE(scaffold.elapsed_s <= wall_cap * slim.elapsed_s);
        }
        {
            INFO("island_joins " << scaffold.island_joins_s << " s against a 2.0 s cap");
            REQUIRE(scaffold.island_joins_s <= 2.0);
        }
        {
            INFO("support " << scaffold.metrics.support_volume_mm3 << " mm3 against tree slim " << slim.metrics.support_volume_mm3 << " mm3");
            REQUIRE(scaffold.metrics.support_volume_mm3 <= 2.5 * slim.metrics.support_volume_mm3);
        }
        // Upright the plan holds every island, and the builder leaves the birth tips of three islands born at z 15.4
        // unrouted, one of them leaning its neck 41.4 degrees off the wall beside it, so those three print with no tip
        // holding them.
        if (pose == "upright") {
            INFO("islands under-held " << scaffold.metrics.islands_under_held);
            REQUIRE(scaffold.metrics.islands_under_held <= 3);
        }
    }
}

// Hidden ([.]): one full Print::process() of plate 1 of the corpus, about a minute. The planner places tips only where
// the print needs them: the hand draped over the raised knee takes a tip on each fingertip that starts in mid-air rather
// than one on every knuckle, and the sword, which stands free from its point at z 1.1 to z 17.1 before it meets the
// cloth, takes a heavy tip at its point and anchors up its height. Boxes are in the frame the slice builds in,
// trafo_centered() over the raw mesh with z the mesh's own.
TEST_CASE("Need-driven tips hold corpus plate 1's hand and sword with few contacts", "[ScaffoldSupport][.]")
{
    SupportValidation::use_os_temporary_dir();
    const char *env = std::getenv("ORCA_MINIATURE_CORPUS");
    if (env == nullptr || *env == '\0') {
        std::cout << "corpus dir not set" << std::endl;
        return;
    }
    SupportValidation::Manifest m;
    m.version    = 1;
    m.model_root = env;
    SupportValidation::ManifestCase c;
    c.id            = "plate1";
    c.model         = "elf_test.3mf";
    c.sha256        = "201c541805e94a2914c3cf0a0aebaee68ee3f6199cc096ae36993069fc781ba6";
    c.selectors     = { "name:10_Dark Elves 1.stl" };
    c.styles        = { "tree_scaffold" };
    c.feature_modes = { "on" };
    c.repeats       = 1;
    SupportValidation::CorpusObject object = SupportValidation::case_object(m, c, fixture_config({ { "support_top_z_distance", "0.2" } }),
                                                                            "tree_scaffold", "on");
    Print print;
    print.set_status_silent();
    print.apply(object.model, object.config);
    print.process();
    REQUIRE(print.objects().size() == 1);
    const PrintObject &po = *print.objects().front();
    REQUIRE(po.support_analysis() != nullptr);
    const std::shared_ptr<const ScaffoldRecord> record = po.scaffold_record();
    REQUIRE(record != nullptr);
    const SupportAnalysis::Report &report = *po.support_analysis();
    CHECK(report.floating_pieces_removed == 0);

    // Three islands print with no tip holding them, and the record names each: two born at z 35.2 whose birth tips drop
    // their necks straight and do not route, and one born at z 40.4 beside a wall, where no neck leaning up to 45 degrees
    // both clears the band and fits the builder's full head, so its birth tip takes the least lean its neck alone clears,
    // 20.7 degrees, and does not route either. The island born at z 42.7 beside it leans its birth tip's neck 41.4
    // degrees, where the head fits, and routes.
    CHECK(report.islands_under_held <= 3);
    CHECK(record->bare_islands.size() == report.islands_under_held);

    // The sword hangs point-down from the raised hand: its blade stands free from z 1.04 and widens from its point to
    // about 7 mm by z 6.5, then rises nearly vertical to the guard at z 15.8. The widening edge takes a stability tip at
    // z 3.56. Above it the stability tip at z 12.32 drops its neck straight, and those at 8.24 and 12.44, where no corner
    // of the blade's faces both clears straight down and holds its head within the face cap, lean theirs off the blade.
    // The blade counts only the tips the builder routes, since a dropped head holds nothing in the print.
    size_t              hand = 0, low_blade = 0;
    std::vector<double> sword;
    bool                heavy_point = false;
    for (const ScaffoldRecord::Tip &tip : record->tips) {
        const Vec3d p = po.trafo_centered() * tip.pos.cast<double>();
        if (p.x() > 8. && p.x() < 15.5 && p.z() > 20. && p.z() < 29.)
            ++ hand;
        if (tip.result == ScaffoldTipResult::Routed && p.x() > -10. && p.x() < -0.2 && p.y() > 2.5 && p.z() < 15.) {
            sword.push_back(p.z());
            low_blade += p.z() < 7.5;
            heavy_point = heavy_point || (p.z() < 1.5 && tip.size == ScaffoldHeadSize::Heavy);
        }
    }
    std::sort(sword.begin(), sword.end());
    double gap = 15. - (sword.empty() ? 0. : sword.back());
    for (size_t i = 1; i < sword.size(); ++ i)
        gap = std::max(gap, sword[i] - sword[i - 1]);
    std::ostringstream blade;
    for (const double z : sword)
        blade << " " << z;
    std::cout << "plate 1: " << record->tips.size() << " points, " << hand << " on the hand, blade tips at z" << blade.str() << ", largest gap "
              << gap << " mm, islands under-held " << report.islands_under_held << ", slender " << report.islands_slender << std::endl;
    INFO(record->tips.size() << " points, " << hand << " on the hand, " << sword.size() << " on the blade under z 15, " << low_blade
                             << " under z 7.5, largest gap " << gap << " mm");
    CHECK(hand <= 10);
    CHECK(heavy_point);
    // No stretch of blade under the guard longer than the blade's stability window, three of its 2.25 mm thicknesses
    // rounded up to 7 mm, goes without a tip, and the widening lower blade takes two.
    CHECK(gap <= 7.);
    CHECK(low_blade >= 2);
    // No more contacts than the resin reference's 117 for this figure.
    CHECK(record->tips.size() <= 117);
}

// Hidden ([.]): two full Print::process() passes over plate 1 of the corpus, about a minute. The list Generate copies
// from the auto slice keeps its 90 routed tips, and sliced from it they route as they did: every point routes, and the
// islands under-held and named are the auto slice's three. The support volume differs only by the planned layers the
// auto slice's five unrouted tips topped, which the list does not hold: 1296.51 against 1296.61 mm3. A copy planned
// without the contacts' layers lost the birth tip at print z 42.08 to the neck check and read 49 mm3 less.
TEST_CASE("A list Generate copies from corpus plate 1's auto slice routes what the auto slice routed", "[ScaffoldSupport][.]")
{
    SupportValidation::use_os_temporary_dir();
    const char *env = std::getenv("ORCA_MINIATURE_CORPUS");
    if (env == nullptr || *env == '\0') {
        std::cout << "corpus dir not set" << std::endl;
        return;
    }
    SupportValidation::Manifest m;
    m.version    = 1;
    m.model_root = env;
    SupportValidation::ManifestCase c;
    c.id            = "plate1";
    c.model         = "elf_test.3mf";
    c.sha256        = "201c541805e94a2914c3cf0a0aebaee68ee3f6199cc096ae36993069fc781ba6";
    c.selectors     = { "name:10_Dark Elves 1.stl" };
    c.styles        = { "tree_scaffold" };
    c.feature_modes = { "on" };
    c.repeats       = 1;
    SupportValidation::CorpusObject object = SupportValidation::case_object(m, c, fixture_config({ { "support_top_z_distance", "0.2" } }),
                                                                            "tree_scaffold", "on");
    Print print;
    print.set_status_silent();
    print.apply(object.model, object.config);
    print.process();
    REQUIRE(print.objects().size() == 1);
    REQUIRE(print.objects().front()->support_analysis() != nullptr);
    const SupportAnalysis::Report               auto_report = *print.objects().front()->support_analysis();
    const std::shared_ptr<const ScaffoldRecord> record      = print.objects().front()->scaffold_record();
    REQUIRE(record != nullptr);
    REQUIRE_FALSE(record->baked);
    const ScaffoldPoints copied = scaffold_points_from(*record);
    REQUIRE(copied.size() == auto_report.tips_routed);

    ModelObject &mo             = *object.model.objects.front();
    mo.scaffold_points          = copied;
    mo.scaffold_points_status   = ScaffoldPointsStatus::AutoGenerated;
    mo.scaffold_points_pose     = record->pose;
    mo.scaffold_points_mesh_box = mo.raw_mesh_bounding_box();
    REQUIRE(print.apply(object.model, object.config) != Print::APPLY_STATUS_UNCHANGED);
    print.process();
    REQUIRE(print.objects().size() == 1);
    const PrintObject &po = *print.objects().front();
    REQUIRE(po.support_analysis() != nullptr);
    const SupportAnalysis::Report              &report = *po.support_analysis();
    const std::shared_ptr<const ScaffoldRecord> baked  = po.scaffold_record();
    REQUIRE(baked != nullptr);
    REQUIRE(baked->baked);
    REQUIRE(baked->tips.size() == copied.size());
    std::ostringstream unrouted;
    for (const ScaffoldRecord::Tip &tip : baked->tips)
        if (tip.result != ScaffoldTipResult::Routed) {
            const Vec3d p = po.trafo_centered() * tip.pos.cast<double>();
            unrouted << " (" << p.x() << ", " << p.y() << ", " << p.z() << ") " << Catch::StringMaker<ScaffoldTipResult>::convert(tip.result);
        }
    std::cout << "plate 1 copied: " << report.tips_routed << " of " << baked->tips.size() << " routed, islands under-held "
              << report.islands_under_held << " (auto " << auto_report.islands_under_held << "), support " << report.support_volume_mm3
              << " mm3 (auto " << auto_report.support_volume_mm3 << " mm3)" << std::endl;
    INFO("points not routed:" << unrouted.str());
    CHECK(report.tips_routed == copied.size());
    CHECK(report.islands_under_held == auto_report.islands_under_held);
    CHECK(baked->bare_islands.size() == record->bare_islands.size());
    CHECK_THAT(report.support_volume_mm3, Catch::Matchers::WithinRel(auto_report.support_volume_mm3, 2e-4));
}

namespace {

// The lowercase hex sha256 of the file at `path`.
std::string sha256_of(const std::filesystem::path &path)
{
    std::ifstream in(path, std::ios::binary);
    REQUIRE(in.good());
    EVP_MD_CTX *ctx = EVP_MD_CTX_new();
    EVP_DigestInit_ex(ctx, EVP_sha256(), nullptr);
    std::vector<char> buffer(1 << 20);
    while (in) {
        in.read(buffer.data(), std::streamsize(buffer.size()));
        EVP_DigestUpdate(ctx, buffer.data(), size_t(in.gcount()));
    }
    unsigned char digest[EVP_MAX_MD_SIZE];
    unsigned int  length = 0;
    EVP_DigestFinal_ex(ctx, digest, &length);
    EVP_MD_CTX_free(ctx);
    std::ostringstream hex;
    for (unsigned int i = 0; i < length; ++ i)
        hex << std::hex << std::setw(2) << std::setfill('0') << int(digest[i]);
    return hex.str();
}

// The distances from a sample of `mesh`'s vertices to `reference` once a rigid transform fits the one onto the other:
// principal axes align them, the proper rotation of the four sign choices that lands closest kept, then closest-point
// iterations refine the fit. Sorted ascending.
std::vector<double> rigid_fit_distances(const indexed_triangle_set &mesh, const indexed_triangle_set &reference)
{
    const AABBTreeIndirect::Tree3f tree = AABBTreeIndirect::build_aabb_tree_over_indexed_triangle_set(reference.vertices, reference.indices);
    const size_t                   step = std::max<size_t>(1, mesh.vertices.size() / 20000);
    Eigen::Matrix3Xd               sample(3, (mesh.vertices.size() + step - 1) / step);
    for (size_t i = 0, k = 0; i < mesh.vertices.size(); i += step, ++ k)
        sample.col(Eigen::Index(k)) = mesh.vertices[i].cast<double>();
    const auto frame = [](const std::vector<Vec3f> &vertices) {
        Vec3d c = Vec3d::Zero();
        for (const Vec3f &v : vertices)
            c += v.cast<double>();
        c /= double(vertices.size());
        Matrix3d cov = Matrix3d::Zero();
        for (const Vec3f &v : vertices)
            cov += (v.cast<double>() - c) * (v.cast<double>() - c).transpose();
        Matrix3d axes = Eigen::SelfAdjointEigenSolver<Matrix3d>(cov).eigenvectors();
        if (axes.determinant() < 0.)
            axes.col(0) = -axes.col(0);
        return std::make_pair(c, axes);
    };
    const auto closest = [&](const Eigen::Matrix3Xd &points, Eigen::Matrix3Xd &hits, std::vector<double> &distances) {
        hits.resize(3, points.cols());
        distances.assign(size_t(points.cols()), 0.);
        for (Eigen::Index k = 0; k < points.cols(); ++ k) {
            size_t      face;
            Vec3d       hit;
            const Vec3d p = points.col(k);
            distances[size_t(k)] = std::sqrt(AABBTreeIndirect::squared_distance_to_indexed_triangle_set(reference.vertices, reference.indices, tree, p, face, hit));
            hits.col(k)          = hit;
        }
    };
    const auto [c_mesh, a_mesh] = frame(mesh.vertices);
    const auto [c_ref, a_ref]   = frame(reference.vertices);
    Transform3d         best    = Transform3d::Identity();
    double              best_d  = std::numeric_limits<double>::infinity();
    Eigen::Matrix3Xd    hits;
    std::vector<double> distances;
    for (const Vec3d flip : { Vec3d(1., 1., 1.), Vec3d(-1., -1., 1.), Vec3d(-1., 1., -1.), Vec3d(1., -1., -1.) }) {
        Transform3d fit = Transform3d::Identity();
        fit.linear()      = a_ref * flip.asDiagonal() * a_mesh.transpose();
        fit.translation() = c_ref - fit.linear() * c_mesh;
        closest(fit * sample, hits, distances);
        const double mean = std::accumulate(distances.begin(), distances.end(), 0.) / double(distances.size());
        if (mean < best_d) {
            best_d = mean;
            best   = fit;
        }
    }
    for (int iteration = 0; iteration < 30; ++ iteration) {
        const Eigen::Matrix3Xd moved = best * sample;
        closest(moved, hits, distances);
        best = Transform3d(Eigen::umeyama(moved, hits, false)) * best;
    }
    closest(best * sample, hits, distances);
    std::sort(distances.begin(), distances.end());
    return distances;
}

} // namespace

// Hidden ([.]): three pre-supported miniatures of 25 to 86 MB and their unsupported files, seconds to load each,
// live outside the repo under $ORCA_MINIATURE_CORPUS/resin_examples. The counts are what an independent analysis of
// the meshes measured: its distinct tips, the duplicates the files repeat, the axes past 45 degrees (less two Ratmen
// axes the artist set on the cap, which that analysis read at 45.00001 degrees), the 0.5 mm
// contacts, the tips on a support standing on the figure and the tipless supports off the plate touching the figure
// twice or more. Each file's hash is checked first, so a changed corpus reads as a changed corpus.
TEST_CASE("Converting the reference pre-supported miniatures keeps each artist tip and the figure alone", "[ScaffoldSupport][.]")
{
    const char *env = std::getenv("ORCA_MINIATURE_CORPUS");
    if (env == nullptr || *env == '\0') {
        std::cout << "corpus dir not set" << std::endl;
        return;
    }
    const std::filesystem::path dir = std::filesystem::path(env) / "resin_examples";
    struct Pair
    {
        std::string supported, supported_sha256, bare, bare_sha256;
        size_t      tips, duplicates, clamped, heavy, rooted, struts;
    };
    const Pair pairs[] = {
        { "STL_10_Dark Elves 1_Supported.stl", "5afa2901bb68d6f84026f69f33d73ebfa1abc8e2f201adde05d47eb8cb15721e", "10_Dark Elves 1.stl",
          "16eb732d9eebecca3756247c3542eca0087524a3d4bb875e25d356df35b48749", 117, 2, 18, 32, 5, 14 },
        { "STL_10_Dark Elves 2_Supported.stl", "b90d3731eaae9d4fb217d487029f750b0572da29b62f0a39bfef4cbc4705dd01", "10_Dark Elves 2.stl",
          "0266162a464e73c50978f117fb07a18f74a3c0b780da0174e1ebc64b283848a6", 90, 0, 12, 34, 0, 22 },
        { "STL_AOFQ_Ratmen_Cleric_supported.stl", "dbd69b9d023185bc98e6ecb50e646a6b967fd7c4229baf4d0f0b5ebae4f66496", "AOFQ_Ratmen_Cleric.stl",
          "385f89bdbf730398da13fe2ebf58f973ad92fffd0f506efb94cc7ef6c5e47756", 214, 1, 73, 25, 0, 17 },
    };
    for (const Pair &pair : pairs) {
        DYNAMIC_SECTION(pair.supported) {
            REQUIRE(sha256_of(dir / pair.supported) == pair.supported_sha256);
            REQUIRE(sha256_of(dir / pair.bare) == pair.bare_sha256);
            Model supported, bare;
            REQUIRE(load_stl((dir / pair.supported).string().c_str(), &supported));
            REQUIRE(load_stl((dir / pair.bare).string().c_str(), &bare));
            supported.add_default_instances();
            ModelObject &mo = *supported.objects.front();

            const auto                            start   = std::chrono::steady_clock::now();
            const PresupportedConversion::Summary summary = PresupportedConversion::convert(mo, 0);
            const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
            std::cout << pair.supported << ": " << summary.tips_converted << " tips, " << summary.duplicates_removed << " duplicates, "
                      << summary.axes_clamped << " clamped, " << summary.tips_rooted_on_figure << " rooted, " << summary.micro_struts_dropped
                      << " micro struts in " << seconds << " s" << std::endl;
            REQUIRE(summary.refusal == PresupportedConversion::Refusal::None);
            CHECK(summary.tips_converted == pair.tips);
            CHECK(summary.duplicates_removed == pair.duplicates);
            CHECK(summary.axes_clamped == pair.clamped);
            CHECK(summary.tips_rooted_on_figure == pair.rooted);
            CHECK(summary.micro_struts_dropped == pair.struts);
            CHECK(size_t(std::count_if(mo.scaffold_points.begin(), mo.scaffold_points.end(),
                                       [](const ScaffoldPoint &p) { return p.size == ScaffoldHeadSize::Heavy; })) == pair.heavy);
            CHECK(seconds < 10.);

            // The figure is the unsupported file rigidly re-posed: after a fit, nearly every vertex of each lies on the
            // other, so the conversion neither kept a support nor dropped a part of the figure, and the few faces the
            // two files hold apart lie within a tenth of a millimetre.
            const indexed_triangle_set figure = mo.raw_mesh().its, reference = bare.objects.front()->raw_mesh().its;
            for (const bool reverse : { false, true }) {
                const std::vector<double> d = reverse ? rigid_fit_distances(reference, figure) : rigid_fit_distances(figure, reference);
                INFO((reverse ? "unsupported onto figure" : "figure onto unsupported") << ": median " << d[d.size() / 2]
                     << " mm, 99th percentile " << d[d.size() * 99 / 100] << " mm, max " << d.back() << " mm");
                CHECK(d[d.size() * 99 / 100] < 1e-3);
                CHECK(d.back() < 0.1);
            }
        }
    }
}
