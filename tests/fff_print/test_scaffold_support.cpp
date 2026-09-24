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
#include <filesystem>
#include <fstream>
#include <map>
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

    // Every support extrusion is base material, and the pad's first layer joins the slices the skirt reads.
    const auto layers = object.support_layers();
    REQUIRE(layers.size() > 0);
    CHECK_FALSE(layers.front()->lslices.empty());
    for (const SupportLayer *sl : layers) {
        std::vector<const ExtrusionEntity *> entities;
        collect_entities(sl->support_fills, entities);
        for (const ExtrusionEntity *e : entities) {
            INFO("print_z " << sl->print_z);
            CHECK(e->role() == erSupportMaterial);
        }
    }

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

    // Nothing printed comes within the xy distance of the model at its own height, and all of it is on the bed.
    const Point shift = object.instances().front().shift;
    for (const SupportLayer *sl : layers) {
        ExPolygons model;
        for (const Layer *layer : object.layers())
            if (std::min(layer->print_z, sl->print_z) - std::max(layer->bottom_z(), sl->print_z - sl->height) > EPSILON)
                append(model, layer->lslices);
        const ExPolygons printed = footprint(*sl);
        INFO("print_z " << sl->print_z);
        CHECK(intersection_ex(printed, offset_ex(union_ex(model), scale_(0.35 - 0.02))).empty());
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
