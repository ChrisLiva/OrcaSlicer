#include <catch2/catch_all.hpp>

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
