#include <catch2/catch_all.hpp>

#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Preset.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Support/SupportParameters.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>

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
