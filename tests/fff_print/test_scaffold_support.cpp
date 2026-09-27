#include <catch2/catch_all.hpp>

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Preset.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Support/SupportAnalysis.hpp"
#include "libslic3r/Support/SupportComponents.hpp"
#include "libslic3r/Support/SupportParameters.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <string>

#include <nlohmann/json.hpp>

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

// A 40 x 12 x 1 mm slab at x 0..40, y 0..12, z 3..4 that touches nothing, and a 2 x 2 x 4 mm post at x 42..44,
// y 0..2 that keeps the object on the plate. The slab's underside is an overhang island 12 mm wide, and 40 mm long so
// the interior grid at a 7 mm step still lands candidates inside it.
TriangleMesh floating_slab_fixture()
{
    TriangleMesh post = make_cube(2., 2., 4.);
    post.translate(42.f, 0.f, 0.f);
    TriangleMesh slab = make_cube(40., 12., 1.);
    slab.translate(0.f, 0.f, 3.f);
    post.merge(slab);
    return post;
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

    const std::vector<std::string> &options = Preset::print_options();
    CHECK(std::find(options.begin(), options.end(), "scaffold_bridge_length") != options.end());
    CHECK(std::find(options.begin(), options.end(), "scaffold_brace_slenderness") != options.end());

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
    r.floating_pieces_removed = 4;

    const SupportValidation::Metrics m = SupportValidation::metrics_of(r);
    CHECK(m.tips_placed == 7);
    CHECK(m.tips_routed == 5);
    CHECK(m.tips_dropped == 2);
    CHECK(m.islands_under_held == 1);
    CHECK(m.pillars_unbraced == 3);
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
                             "pillars_unbraced", "floating_pieces_removed" }) {
        INFO(key);
        CHECK(metrics.contains(key));
    }
    CHECK(metrics.value("tips_placed", size_t(0)) == 7);
    CHECK(metrics.value("tips_routed", size_t(0)) == 5);
    CHECK(metrics.value("tips_dropped", size_t(0)) == 2);
    CHECK(metrics.value("islands_under_held", size_t(0)) == 1);
    CHECK(metrics.value("pillars_unbraced", size_t(0)) == 3);
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
    CHECK(box_above.min.x() - box_top.min.x() >= scale_(1.4));
    CHECK(box_above.min.y() - box_top.min.y() >= scale_(1.4));
    CHECK(box_top.max.x() - box_above.max.x() >= scale_(1.4));
    CHECK(box_top.max.y() - box_above.max.y() >= scale_(1.4));

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
    // whose top reaches the tip and the two below, and the layer under those carries none. A tip on the bar's 0.6 mm
    // neck reads the small grade, two lines across, and a tip on the slab's 6 mm neck the large one, four lines.
    // The column's first layer places the fixture in the object's centred frame.
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
    check_rings(12., slab_box, &bar_strip, 3.4 * w, 4.2 * w);

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

TEST_CASE("A head whose neck bottoms in the xy band is dropped while one whose neck clears it keeps its head and no ring floats",
          "[ScaffoldSupport]")
{
    // At the corpus's 0.22 mm support line and 0.5 mm xy distance a small-grade neck, 0.44 mm across under its pin, fits
    // wholly inside the column's band. A head's neck under its rings is clipped by that band. When that leaves a ring over
    // nothing and the neck's lowest slice clears the band, the neck is clipped by the model alone and the head stays; a
    // head whose neck bottoms in the column's band is dropped and the cage is built without it. On the 2 mm plank every
    // tip keeps its head; on the 4 mm plank the two tips nearest the column face, 0.6 and 1.3 mm off it, are dropped.
    // The 4 mm plank at the corpus's 0.06 mm layers and two interface layers adds rings whose printed loop stops short
    // of a neighbour the drawn ring touches, and base whose printed walls leave its inside and its slivers bare: the
    // drop reads the rings and the base as their lines cover them.
    const auto [length_mm, layer_height, interface_layers, min_dropped, max_dropped] = GENERATE(
        table<double, std::string, std::string, size_t, size_t>({ { 2., "0.2", "3", 0, 0 }, { 4., "0.06", "2", 1, 2 } }));
    Print print;
    init_and_process_print({ slope_fixture(length_mm) }, print,
                           scaffold_config({ { "support_line_width", "0.22" },
                                             { "support_object_xy_distance", "0.5" },
                                             { "layer_height", layer_height },
                                             { "support_interface_top_layers", interface_layers } }));
    REQUIRE(print.objects().size() == 1);
    const PrintObject &object = *print.objects().front();
    REQUIRE(object.support_analysis() != nullptr);
    const SupportAnalysis::Report &report = *object.support_analysis();
    INFO("plank " << length_mm << " mm at " << layer_height << " mm layers: tips placed " << report.tips_placed << " routed "
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

TEST_CASE("Interior tips survive only where a bridge span fits", "[ScaffoldSupport]")
{
    // A 12 mm wide slab holds no 14 mm disc, so at a 14 mm bridge length every tip under it stands on the rim; a 10 mm
    // disc fits, so at 10 mm the interior grid keeps a tip well inside the rim.
    struct Reading { size_t polygons = 0; double max_depth_mm = -1e9, min_depth_mm = 1e9; };
    const auto read = [](const char *bridge_length) {
        Print print;
        init_and_process_print({ floating_slab_fixture() }, print, scaffold_config({ { "max_bridge_length", bridge_length } }));
        REQUIRE(print.objects().size() == 1);
        const PrintObject &object = *print.objects().front();
        Reading r;

        // The first layer holds only the post, which places the fixture's (0, 0) in the object's centred frame.
        const Point      origin = get_extents(object.layers().front()->lslices).min - Point::new_scale(42., 0.);
        const FixtureBox slab { origin, 0., 0., 40., 12. };
        const FixtureBox near_slab { origin, -4., -4., 41.9, 16. };
        const auto layers = object.support_layers();
        const size_t top = top_layer_under(layers, 3.);
        REQUIRE(top != size_t(-1));
        INFO("bridge length " << bridge_length << " top interface print_z " << layers[top]->print_z);
        // Tips closer than a disc print as one polygon, so each polygon's centroid is read, and its depth is the
        // distance inside the slab's outline, negative outside it.
        for (const ExPolygon &poly : role_footprint(*layers[top], erSupportMaterialInterface)) {
            const Point c = poly.contour.centroid();
            if (! near_slab.contains(c))
                continue;
            const Vec2d  q     = (c - origin).cast<double>() * SCALING_FACTOR;
            const double dx    = std::max({ slab.x0 - q.x(), 0., q.x() - slab.x1 });
            const double dy    = std::max({ slab.y0 - q.y(), 0., q.y() - slab.y1 });
            const double depth = slab.contains(c) ? std::min({ q.x() - slab.x0, slab.x1 - q.x(), q.y() - slab.y0, slab.y1 - q.y() })
                                                  : -std::hypot(dx, dy);
            ++ r.polygons;
            r.max_depth_mm = std::max(r.max_depth_mm, depth);
            r.min_depth_mm = std::min(r.min_depth_mm, depth);
        }
        return r;
    };

    const Reading wide = read("14");
    INFO("at 14: " << wide.polygons << " polygons, depth " << wide.min_depth_mm << " to " << wide.max_depth_mm << " mm");
    REQUIRE(wide.polygons > 0);
    CHECK(wide.max_depth_mm <= 1.0);
    CHECK(wide.min_depth_mm >= -1.0);

    const Reading narrow = read("10");
    INFO("at 10: " << narrow.polygons << " polygons, depth " << narrow.min_depth_mm << " to " << narrow.max_depth_mm << " mm");
    // No tip count is compared: the key also moves the contour tips and the interior grid step before the thinning.
    CHECK(narrow.max_depth_mm >= 2.0);
}

TEST_CASE("The hold floor restores dropped contacts under tall islands", "[ScaffoldSupport]")
{
    // At a 3.5 mm contact distance the decimation keeps one end of stick B's 3 mm underside. B and post C stand 2 mm
    // from their birth at z 3 to their own tops at z 5 without joining, and cube A 3 mm to its join at z 6, so each
    // wants two tips a pillar diameter (1.2 mm) apart: B gets its other end back, and C, 0.8 mm across, holds only
    // one, which is all its footprint fits, so no island is under-held.
    Print print;
    init_and_process_print({ islands_fixture() }, print, scaffold_config({ { "support_contact_min_distance", "3.5" } }));
    REQUIRE(print.objects().size() == 1);
    const PrintObject &object = *print.objects().front();
    REQUIRE(object.support_analysis() != nullptr);
    const SupportAnalysis::Report &report = *object.support_analysis();
    INFO("tips placed " << report.tips_placed << " routed " << report.tips_routed << " dropped " << report.tips_dropped
                        << " floating removed " << report.floating_pieces_removed);
    CHECK(report.islands_under_held == 0);

    // The islands' undersides are at z 3, the top of a planned layer. The block's first layer places the fixture's
    // (0, 0) in the object's centred frame. A head at the stick's end tilts, so its disc's centroid sits just past
    // the end: the box under B reaches 0.3 mm past the stick on every side.
    const auto   layers = object.support_layers();
    const size_t top    = top_layer_under(layers, 3.);
    REQUIRE(top != size_t(-1));
    CHECK_THAT(layers[top]->print_z, WithinAbs(3., 1e-6));
    const Point      origin = get_extents(object.layers().front()->lslices).min;
    const FixtureBox under_a { origin, 14., 3., 18., 7. };
    const FixtureBox under_b { origin, 21.7, 4.4, 25.3, 5.6 };
    std::vector<Point> at_a, at_b;
    for (const ExPolygon &poly : role_footprint(*layers[top], erSupportMaterialInterface)) {
        const Point c = poly.contour.centroid();
        const Vec2d q = (c - origin).cast<double>() * SCALING_FACTOR;
        INFO("interface polygon centroid (" << q.x() << ", " << q.y() << ")");
        if (under_a.contains(c))
            at_a.push_back(c);
        if (under_b.contains(c))
            at_b.push_back(c);
    }
    CHECK(at_a.size() >= 2);
    REQUIRE(at_b.size() >= 2);
    double spread_mm = 0.;
    for (size_t i = 0; i < at_b.size(); ++ i)
        for (size_t j = i + 1; j < at_b.size(); ++ j)
            spread_mm = std::max(spread_mm, unscale<double>((at_b[i] - at_b[j]).cast<double>().norm()));
    INFO("the interface polygons under B spread " << spread_mm << " mm");
    CHECK(spread_mm >= 1.2);
}

TEST_CASE("Unseeded feature starts get a scaffold tip while floating debris and wall-held slivers get none", "[ScaffoldSupport]")
{
    // The spike is too thin to extrude, so no overhang and no contact starts under it, and the cube's contacts stand
    // on the body the bar roots. The spike's island joins 0.6 mm up and wants one tip, which the hold floor seeds at
    // the spike's middle. The speck never joins and its own height is one 0.2 mm slab: debris, left alone. The
    // sliver's seed stands inside the block's xy band, where the wall skip removes it, and the sliver joins the
    // block 0.6 mm up: the wall holds it, so it gets no tip and is not under-held. The tall sliver's seed stands in the
    // same band, but it joins 1.4 mm up, so it gets no tip and is the one island counted under-held. The floating
    // part's leg reaches 0.4 mm to the arm that merges it into the cube's island, but the part it ends up in stands
    // 3 mm tall and its top sits 0.8 mm over the leg's bottom: the leg is no debris and gets its seed.
    Print print;
    init_and_process_print({ seeded_islands_fixture() }, print, scaffold_config({ { "support_remove_small_overhang", "1" } }));
    REQUIRE(print.objects().size() == 1);
    const PrintObject &object = *print.objects().front();
    REQUIRE(object.support_analysis() != nullptr);
    const SupportAnalysis::Report &report = *object.support_analysis();
    INFO("tips placed " << report.tips_placed << " routed " << report.tips_routed << " dropped " << report.tips_dropped
                        << " floating removed " << report.floating_pieces_removed);
    CHECK(report.islands_under_held == 1);
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
    size_t at_debris = 0, at_sliver = 0;
    for (const SupportLayer *layer : layers)
        for (const ExPolygon &poly : role_footprint(*layer, erSupportMaterialInterface)) {
            if (under_debris.contains(poly.contour.centroid()))
                ++ at_debris;
            if (under_sliver.contains(poly.contour.centroid()))
                ++ at_sliver;
        }
    CHECK(at_debris == 0);
    CHECK(at_sliver == 0);
}

TEST_CASE("Slender scaffold pillars get braces and unreachable ones stand unbraced", "[ScaffoldSupport]")
{
    // Runs that share a bridge length share their pillars: the key also bounds head clustering and routing, and only
    // the linking pass after routing reads the slenderness. The pillars stand straight, so a brace shows as a section
    // of its own between them rather than inside a widened foot.
    struct Reading { size_t unbraced = 0, floating = 0, mid_polygons = 0; double volume_mm3 = 0.; };
    const auto read = [](const char *slenderness, const char *bridge_length) {
        Print print;
        init_and_process_print({ tall_shelf_fixture() }, print,
                               scaffold_config({ { "scaffold_brace_slenderness", slenderness }, { "scaffold_bridge_length", bridge_length },
                                                 { "tree_support_branch_diameter_angle", "0" } }));
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
        init_and_process_print({ tall_shelf_fixture() }, print, scaffold_config({ { "tree_support_branch_diameter_angle", angle } }));
        REQUIRE(print.objects().size() == 1);
        const PrintObject &object = *print.objects().front();
        REQUIRE(object.support_analysis() != nullptr);
        const SupportAnalysis::Report &report = *object.support_analysis();
        Reading r { report.tips_routed, report.floating_pieces_removed, {} };
        // The area inside the printed outlines under the slab: a pillar prints a ring, so its hole counts.
        const FixtureBox under_slab { get_extents(object.layers().front()->lslices).min, 6.5, -3., 18., 9. };
        const auto       layers = object.support_layers();
        for (int z : { 2, 15, 25 }) {
            const size_t i = top_layer_under(layers, z);
            REQUIRE(i != size_t(-1));
            r.base_mm2[z] = area_mm2(intersection_ex(layers[i]->base_areas, { ExPolygon(under_slab.polygon()) }));
        }
        INFO("angle " << angle << ": routed " << r.routed << " floating removed " << r.floating << " base at z 2, 15, 25: "
                      << r.base_mm2[2] << ", " << r.base_mm2[15] << ", " << r.base_mm2[25] << " mm2");
        CHECK(r.floating == 0);
        return r;
    };

    // A factor of 4 at z 2 leaves room for the feet stopping at their neighbours and for the band beside the column.
    const Reading straight = read("0");
    const Reading tapered  = read("5");
    INFO("base at z 2: " << straight.base_mm2.at(2) << " mm2 straight, " << tapered.base_mm2.at(2) << " mm2 tapered");
    CHECK(tapered.routed == straight.routed);
    CHECK(tapered.base_mm2.at(2) > tapered.base_mm2.at(15));
    CHECK(tapered.base_mm2.at(15) > tapered.base_mm2.at(25));
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

    // A second apply of `m` under the config rebuilt the way init_print built it, as paint_and_reapply does.
    const auto reapply = [](Print &p, Model &m, const DynamicPrintConfig &cfg) {
        DynamicPrintConfig full = DynamicPrintConfig::full_print_config();
        full.apply(cfg);
        full.set_key_value("gcode_comments", new ConfigOptionBool(true));
        return p.apply(m, full);
    };
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
    // point under the slab is an alias of it within sla::D_SP, and one at x 6.1 stands in the column's xy band.
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
    hand.push_back({ Vec3f(6.1f, 3.f, 12.f), ScaffoldHeadSize::Light, false });
    bake(mo, hand, ScaffoldPointsStatus::UserModified, auto_pose);
    const Slice edited = slice(mo);
    REQUIRE(edited.record->tips.size() == hand.size());
    const std::vector<ScaffoldRecord::Tip> &tips = edited.record->tips;
    INFO("underside point (" << under_slab->x() << ", " << under_slab->y() << ", 12), alias partner " << j);
    CHECK(tips[n].result == ScaffoldTipResult::Routed);
    CHECK(tips[n + 1].result == ScaffoldTipResult::Filtered);
    CHECK(size_t(tips[j].result == ScaffoldTipResult::Merged) + size_t(tips[n + 2].result == ScaffoldTipResult::Merged) == 1);
    CHECK(tips[n + 3].result == ScaffoldTipResult::Wall);

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

    // Heavy heads print more support than light ones on the same points.
    ScaffoldPoints heavy = auto_list;
    for (ScaffoldPoint &pt : heavy)
        pt.size = ScaffoldHeadSize::Heavy;
    bake(mo, heavy, ScaffoldPointsStatus::AutoGenerated, auto_pose);
    const Slice heavier = slice(mo);
    CHECK(heavier.record->baked);
    CHECK(heavier.report.support_volume_mm3 > baked.report.support_volume_mm3);

    // Bare islands: an empty list seeds no tip under an island the hold floor would hold, and the record names where
    // it would have. The auto slice seeds them and names none.
    Print                    islands_print;
    Model                    islands_model;
    const DynamicPrintConfig islands_config = scaffold_config({ { "support_remove_small_overhang", "1" } });
    init_print({ seeded_islands_fixture() }, islands_print, islands_model, islands_config);
    islands_print.process();
    REQUIRE(islands_print.objects().size() == 1);
    REQUIRE(islands_print.objects().front()->scaffold_record() != nullptr);
    CHECK(islands_print.objects().front()->scaffold_record()->bare_islands.empty());
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
        if (pose == "upright") {
            INFO("islands under-held " << scaffold.metrics.islands_under_held);
            REQUIRE(scaffold.metrics.islands_under_held == 0);
        }
    }
}
