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
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>

#include <nlohmann/json.hpp>

#include "support_validation.hpp"
#include "test_helpers.hpp"

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
// the front half seeds nothing under it; a 0.3 x 0.3 x 0.3 mm cube at x 35..35.3, y 5..5.3, z 7.2..7.5 never joins
// and stands 0.8 mm under the object's top at z 8.
TriangleMesh seeded_islands_fixture()
{
    TriangleMesh block = make_cube(10., 10., 8.);
    TriangleMesh cube  = make_cube(4., 4., 4.);
    cube.translate(14.f, 3.f, 3.5f);
    TriangleMesh bar = make_cube(6., 2., 1.);
    bar.translate(9.f, 4.f, 3.5f);
    TriangleMesh spike = make_cube(0.4, 0.4, 0.5);
    spike.translate(14.f, 4.8f, 3.f);
    TriangleMesh debris = make_cube(0.3, 0.3, 0.3);
    debris.translate(35.f, 5.f, 7.2f);
    block.merge(cube);
    block.merge(bar);
    block.merge(spike);
    block.merge(debris);
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

TEST_CASE("A scaffold on the shelf fixture prints a solid pad and clear base walls and nothing floating", "[ScaffoldSupport]")
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
    std::set<ExtrusionRole> roles;
    for (const SupportLayer *sl : layers) {
        std::vector<const ExtrusionEntity *> entities;
        collect_entities(sl->support_fills, entities);
        for (const ExtrusionEntity *e : entities)
            roles.insert(e->role());
    }
    CHECK(roles == std::set<ExtrusionRole>{ erSupportMaterial, erSupportMaterialInterface });

    // What each support layer's extrusions cover.
    const auto footprint = [](const SupportLayer &sl) { return union_ex(sl.support_fills.polygons_covered_by_width(0.f)); };
    const auto area_mm2 = [](const ExPolygons &polys) {
        double a = 0.;
        for (const ExPolygon &p : polys)
            a += p.area() * SCALING_FACTOR * SCALING_FACTOR;
        return a;
    };

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
    for (size_t i = 1; i <= pad_top; ++ i) {
        INFO("pad layer " << i << " print_z " << layers[i]->print_z);
        CHECK(area_mm2(footprint(*layers[i])) >= 0.5 * a0);
    }
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
        ExPolygons model;
        for (const Layer *layer : object.layers())
            if (std::min(layer->print_z, sl->print_z) - std::max(layer->bottom_z(), sl->print_z - sl->height) > EPSILON)
                append(model, layer->lslices);
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

    // The pad prints solid through the sheath and the cage above it prints walls only: no extrusion above the pad
    // reaches further into its layer's base area than a second wall would, while the pad's infill does.
    const auto reaches_inside = [w](const SupportLayer &sl) {
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
    };
    for (size_t i = pad_top + 1; i < layers.size(); ++ i) {
        INFO("print_z " << layers[i]->print_z);
        CHECK(reaches_inside(*layers[i]) == 0);
    }
    CHECK(reaches_inside(*layers[1]) > 0);

    // Each tip fuses to its overhang through a ring of interface on the three layers under it: the highest layer
    // whose top reaches the tip and the two below, and the layer under those carries none. A tip on the bar's 0.6 mm
    // neck reads the small grade, two lines across, and a tip on the slab's 6 mm neck the large one, four lines.
    // The column's first layer places the fixture in the object's centred frame.
    const Point      origin    = get_extents(object.layers().front()->lslices).min;
    const FixtureBox bar_strip { origin, 6., 2.4, 12., 3.6 };
    const FixtureBox slab_box  { origin, 6., -3., 18., 9. };
    // A head at the bar's 0.6 mm edge tilts, so its lower rings stand up to half a millimetre off the bar's side.
    const FixtureBox bar_rings { origin, 6., 2.1, 12., 3.9 };
    // A disc's width is the diameter of the largest circle its outer contour inscribes; holes are ignored on purpose,
    // since a printed disc is a ring. The inscribed circle is one disc's whatever fused with it, and a tilted head's
    // slice, an ellipse along the tilt, inscribes its short axis, the sphere chord the grade sets. The contour is
    // closed by half a line width first: the line ends notch it, and a notch shrinks the circle a whole disc holds.
    struct Disc { double width_mm, box_min_mm, box_max_mm; Point centroid; };
    const auto discs = [w](const SupportLayer &sl) {
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
    };
    const auto rings_under = [&](double tip_z, const FixtureBox &region, const FixtureBox *excluded, double min_w, double max_w) {
        const size_t top = top_layer_under(layers, tip_z);
        REQUIRE(top != size_t(-1));
        REQUIRE(top >= 3);
        for (size_t k = 0; k < 4; ++ k) {
            const SupportLayer &sl = *layers[top - k];
            INFO("tip z " << tip_z << " layer " << k << " under the tip, print_z " << sl.print_z);
            size_t in_region = 0, graded = 0;
            for (const Disc &disc : discs(sl)) {
                INFO("disc inscribes " << disc.width_mm << " mm, bounding box " << disc.box_min_mm << " x " << disc.box_max_mm << " mm");
                if (! region.contains(disc.centroid))
                    continue;
                ++ in_region;
                if (k == 0 && ! (excluded != nullptr && excluded->contains(disc.centroid))) {
                    ++ graded;
                    CHECK(disc.width_mm >= min_w);
                    CHECK(disc.width_mm <= max_w);
                }
            }
            if (k < 3)
                CHECK(in_region > 0);
            else
                CHECK(in_region == 0);
            if (k == 0)
                CHECK(graded > 0);
        }
    };
    rings_under(4., bar_rings, nullptr, 1.8 * w, 2.2 * w);
    rings_under(12., slab_box, &bar_strip, 3.4 * w, 4.2 * w);

    // A bar tip within the xy distance of the column face is skipped, since its neck would be clipped there while
    // its ring survived; the bar keeps the tips further out.
    {
        const SupportLayer &sl = *layers[top_layer_under(layers, 4.)];
        size_t under_bar = 0;
        for (const Disc &disc : discs(sl)) {
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
        ExPolygons model;
        for (const Layer *layer : object.layers())
            if (std::min(layer->print_z, sl->print_z) - std::max(layer->bottom_z(), sl->print_z - sl->height) > EPSILON)
                append(model, layer->lslices);
        INFO("print_z " << sl->print_z);
        CHECK(intersection_ex(role_footprint(*sl, erSupportMaterialInterface), offset_ex(union_ex(model), -scale_(0.05))).empty());
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
    // At a 3.5 mm contact distance the decimation keeps one end of stick B's 3 mm underside. B and post C stand 5 mm
    // from their birth at z 3 to the object's top without joining, and cube A 3 mm to its join at z 6, so each wants
    // two tips a pillar diameter (1.2 mm) apart: B gets its other end back, and C, 0.8 mm across, holds only one,
    // which is all its footprint fits, so no island is under-held.
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

TEST_CASE("Unseeded feature starts get a scaffold tip and floating debris gets none", "[ScaffoldSupport]")
{
    // The spike is too thin to extrude, so no overhang and no contact starts under it, and the cube's contacts stand
    // on the body the bar roots. The spike's island joins 0.6 mm up and wants one tip, which the hold floor seeds at
    // the spike's middle. The speck never joins and stands under 1 mm below the object's top: debris, left alone.
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

    // The seeded tip stands at the spike's bottom, z 3, and prints its disc on the highest planned layer whose top is
    // at or under it. The block's first layer places the fixture's (0, 0) in the object's centred frame.
    const auto   layers = object.support_layers();
    const size_t top    = top_layer_under(layers, 3.);
    REQUIRE(top != size_t(-1));
    INFO("the layer under the spike prints at z " << layers[top]->print_z);
    const Point      origin = get_extents(object.layers().front()->lslices).min;
    const FixtureBox under_spike { origin, 13.7, 4.5, 14.7, 5.5 };
    const FixtureBox under_debris { origin, 34.7, 4.7, 35.6, 5.6 };
    size_t at_spike = 0;
    for (const ExPolygon &poly : role_footprint(*layers[top], erSupportMaterialInterface)) {
        const Point c = poly.contour.centroid();
        const Vec2d q = (c - origin).cast<double>() * SCALING_FACTOR;
        INFO("interface polygon centroid (" << q.x() << ", " << q.y() << ")");
        if (under_spike.contains(c))
            ++ at_spike;
    }
    CHECK(at_spike >= 1);
    size_t at_debris = 0;
    for (const SupportLayer *layer : layers)
        for (const ExPolygon &poly : role_footprint(*layer, erSupportMaterialInterface))
            if (under_debris.contains(poly.contour.centroid()))
                ++ at_debris;
    CHECK(at_debris == 0);
}

TEST_CASE("Slender scaffold pillars get braces and unreachable ones stand unbraced", "[ScaffoldSupport]")
{
    // Runs that share a bridge length share their pillars: the key also bounds head clustering and routing, and only
    // the linking pass after routing reads the slenderness.
    struct Reading { size_t unbraced = 0, floating = 0, mid_polygons = 0; double volume_mm3 = 0.; };
    const auto read = [](const char *slenderness, const char *bridge_length) {
        Print print;
        init_and_process_print({ tall_shelf_fixture() }, print,
                               scaffold_config({ { "scaffold_brace_slenderness", slenderness }, { "scaffold_bridge_length", bridge_length } }));
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
    c.sha256        = "0252d6ebf9fa9ff0fc006404920cd41d17a2d1b3b51f6b3a08049b35fd627ed7";
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
            if (pose == "upright") {
                for (ModelObject *mo : object.model.objects)
                    for (ModelInstance *instance : mo->instances)
                        instance->set_rotation(Vec3d::Zero());
                object.model.center_instances_around_point(unscale(BoundingBox(get_bed_shape(object.config)).center()));
                for (ModelObject *mo : object.model.objects)
                    mo->ensure_on_bed();
            }

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
            REQUIRE(scaffold.metrics.tips_dropped <= scaffold.metrics.tips_placed / 10);
        }
        {
            INFO("floating pieces removed " << scaffold.metrics.floating_pieces_removed << " (tree slim " << slim.metrics.floating_pieces_removed << ")");
            REQUIRE(scaffold.metrics.floating_pieces_removed == 0);
        }
        {
            INFO("process wall " << scaffold.elapsed_s << " s against tree slim " << slim.elapsed_s << " s");
            REQUIRE(scaffold.elapsed_s <= 1.25 * slim.elapsed_s);
        }
        {
            INFO("island_joins " << scaffold.island_joins_s << " s against a 2.0 s cap");
            REQUIRE(scaffold.island_joins_s <= 2.0);
        }
        {
            INFO("support " << scaffold.metrics.support_volume_mm3 << " mm3 against tree slim " << slim.metrics.support_volume_mm3 << " mm3");
            REQUIRE(scaffold.metrics.support_volume_mm3 <= 2.0 * slim.metrics.support_volume_mm3);
        }
        if (pose == "upright") {
            INFO("islands under-held " << scaffold.metrics.islands_under_held);
            REQUIRE(scaffold.metrics.islands_under_held == 0);
        }
    }
}
