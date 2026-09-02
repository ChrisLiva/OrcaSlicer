#include <catch2/catch_all.hpp>

#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Support/TreeSupport.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "test_helpers.hpp" // get access to init_print, etc

using namespace Slic3r::Test;
using namespace Slic3r;

TEST_CASE("Three raft layers are created", "[SupportMaterial]")
{
	Slic3r::Print print;
	Slic3r::Test::init_and_process_print({ cube(20) }, print, {
        { "enable_support", 1 },
        { "raft_layers",    3 }
		});
    REQUIRE(print.objects().front()->support_layers().size() == 3);
}

TEST_CASE("Enforced support layers are generated", "[SupportMaterial]")
{
    // enforce_support_layers forces support on the first N layers even with support off.
    Slic3r::Print baseline;
    Slic3r::Test::init_and_process_print({ TestMesh::overhang }, baseline, {
        { "enable_support",         0 },
        { "enforce_support_layers", 0 }
    });
    REQUIRE(baseline.objects().front()->support_layers().empty());

    Slic3r::Print enforced;
    Slic3r::Test::init_and_process_print({ TestMesh::overhang }, enforced, {
        { "enable_support",         0 },
        { "enforce_support_layers", 100 }
    });
    REQUIRE(enforced.objects().front()->support_layers().size() > 0);
}

SCENARIO("Support layer Z honors contact distance", "[SupportMaterial]")
{
    // Box h = 20mm, hole bottom at 5mm, hole height 10mm (top edge at 15mm).
    TriangleMesh mesh = Slic3r::Test::mesh(Slic3r::Test::TestMesh::cube_with_hole);
    mesh.rotate_x(float(M_PI / 2));

	auto check = [](Slic3r::Print &print, bool &first_support_layer_height_ok, bool &layer_height_minimum_ok, bool &layer_height_maximum_ok)
	{
        ConstSupportLayerPtrsAdaptor support_layers = print.objects().front()->support_layers();

		first_support_layer_height_ok = support_layers.front()->print_z == print.config().initial_layer_print_height.value;

		layer_height_minimum_ok = true;
		layer_height_maximum_ok = true;
		double min_layer_height = print.config().min_layer_height.values.front();
		double max_layer_height = print.config().nozzle_diameter.values.front();
		if (print.config().max_layer_height.values.front() > EPSILON)
			max_layer_height = std::min(max_layer_height, print.config().max_layer_height.values.front());
		for (size_t i = 1; i < support_layers.size(); ++ i) {
			if (support_layers[i]->print_z - support_layers[i - 1]->print_z < min_layer_height - EPSILON)
				layer_height_minimum_ok = false;
			if (support_layers[i]->print_z - support_layers[i - 1]->print_z > max_layer_height + EPSILON)
				layer_height_maximum_ok = false;
		}
	};

    GIVEN("A print object having one modelObject") {
        WHEN("Layer height = 0.2 and first layer height = 0.4") {
			Slic3r::Print print;
			Slic3r::Test::init_and_process_print({ mesh }, print, {
                { "enable_support",             1 },
                { "layer_height",               0.2 },
                { "initial_layer_print_height", 0.4 },
                { "dont_support_bridges",       false },
			});
			bool first_layer_ok, layer_min_ok, layer_max_ok;
            check(print, first_layer_ok, layer_min_ok, layer_max_ok);
            THEN("First layer height is honored")			{ REQUIRE(first_layer_ok == true); }
            THEN("No null or negative support layers")		{ REQUIRE(layer_min_ok == true); }
            THEN("No layers thicker than nozzle diameter")	{ REQUIRE(layer_max_ok == true); }
        }
        WHEN("Layer height = 0.2 and first layer height = 0.3") {
			Slic3r::Print print;
			Slic3r::Test::init_and_process_print({ mesh }, print, {
                { "enable_support",             1 },
                { "layer_height",               0.2 },
                { "initial_layer_print_height", 0.3 },
                { "dont_support_bridges",       false },
            });
            bool first_layer_ok, layer_min_ok, layer_max_ok;
            check(print, first_layer_ok, layer_min_ok, layer_max_ok);
            THEN("First layer height is honored")			{ REQUIRE(first_layer_ok == true); }
            THEN("No null or negative support layers")		{ REQUIRE(layer_min_ok == true); }
            THEN("No layers thicker than nozzle diameter")	{ REQUIRE(layer_max_ok == true); }
        }
    }
}

// extrude_support once held a `static` lambda capturing `this`, so a second export in the
// same process dereferenced a returned stack frame (ASan: stack-use-after-return).
TEST_CASE("Support G-code emission survives a second slice in the same process", "[SupportMaterial][Regression]")
{
    const std::string first = slice({ TestMesh::overhang }, { { "enable_support", 1 } });
    REQUIRE(! layers_with_role(first, "support").empty());

    const std::string second = slice({ TestMesh::overhang }, { { "enable_support", 1 } });
    REQUIRE(! layers_with_role(second, "support").empty());
}

namespace {

// A 6x6x30 mm column carrying a 2 mm dia. rod tilted 70 degrees off vertical, so the rod's
// lowest slice overlaps nothing below it: a sharp tail that propagates up the whole rod.
TriangleMesh sharp_tail_spike()
{
    TriangleMesh column = make_cube(6, 6, 30);
    TriangleMesh rod    = make_cylinder(1.0, 20.0);
    rod.rotate_x(float(Geometry::deg2rad(110.)));
    rod.translate(3, 0, 30);
    column.merge(rod);
    return column;
}

// print_z of every layer above the first that carries at least one sharp-tail overhang band.
std::vector<coordf_t> sharp_tail_band_zs(double layer_height, bool scoring_mode)
{
    Slic3r::Print print;
    Slic3r::Model model;
    Slic3r::Test::init_print({ sharp_tail_spike() }, print, model, {
        { "enable_support",             1 },
        { "support_type",               "tree(auto)" },
        { "support_threshold_angle",    45 },
        { "nozzle_diameter",            0.4 },
        { "line_width",                 0.42 },
        { "initial_layer_print_height", 0.2 },
        { "layer_height",               layer_height }
    });

    PrintObject *po = print.objects_mutable().front();
    po->slice();
    TreeSupport ts(*po, po->slicing_parameters());
    ts.m_scoring_mode = scoring_mode;
    ts.detect_overhangs();

    std::vector<coordf_t> zs;
    for (const Layer *layer : po->layers()) {
        if (layer->id() == 0)
            continue;
        for (size_t i = 0; i < layer->loverhangs.size(); ++i) {
            // overhang_types is keyed on the addresses of this vector's elements; operator[] would
            // insert Detected for a missing key and read back as a mis-classification.
            auto it = ts.overhang_types.find(&layer->loverhangs[i]);
            if (it != ts.overhang_types.end() && it->second == TreeSupport::SharpTail) {
                zs.push_back(layer->print_z);
                break;
            }
        }
    }
    std::sort(zs.begin(), zs.end());
    return zs;
}

// Same band set: equal size and every band within 1 um of its counterpart.
bool same_bands(const std::vector<coordf_t> &a, const std::vector<coordf_t> &b)
{
    if (a.size() != b.size())
        return false;
    for (size_t i = 0; i < a.size(); ++i)
        if (std::abs(a[i] - b[i]) > 1e-6)
            return false;
    return true;
}

// Largest gap from a band in `bands` to the nearest band in `reference`.
coordf_t max_distance_to_nearest(const std::vector<coordf_t> &bands, const std::vector<coordf_t> &reference)
{
    coordf_t worst = 0;
    for (coordf_t z : bands) {
        coordf_t nearest = std::numeric_limits<coordf_t>::max();
        for (coordf_t z_ref : reference)
            nearest = std::min(nearest, std::abs(z - z_ref));
        worst = std::max(worst, nearest);
    }
    return worst;
}

} // namespace

TEST_CASE("Scoring mode admits sharp-tail bands at a layer-height-stable pitch", "[SupportMaterial]")
{
    // Scoring mode gates bands on crossing a 0.5 mm boundary, so the same spike yields the same
    // band count and the same band heights whether it is sliced at 0.06 mm or at 0.12 mm.
    const std::vector<coordf_t> on_006 = sharp_tail_band_zs(0.06, true);
    const std::vector<coordf_t> on_012 = sharp_tail_band_zs(0.12, true);

    // The tilted rod spans ~7.7 mm of z, so a 0.5 mm pitch admits far more than 5 bands.
    REQUIRE(on_006.size() >= 5);
    REQUIRE(std::abs(int(on_006.size()) - int(on_012.size())) <= 1);
    // Bands are co-located: the two runs detach the tail on their own grids, so a 0.12 mm band can
    // sit at most one layer of each grid away from its 0.06 mm counterpart.
    REQUIRE(max_distance_to_nearest(on_012, on_006) <= 0.12 + 0.06 + 1e-6);

    // With the flag off the truncating `int(accum_height * 10) % 5` gate stays exactly as it is:
    // roughly one band per 0.5 mm at 0.06 mm and roughly one per 0.6 mm at 0.12 mm.
    const std::vector<coordf_t> off_006 = sharp_tail_band_zs(0.06, false);
    const std::vector<coordf_t> off_012 = sharp_tail_band_zs(0.12, false);

    REQUIRE(off_006.size() > off_012.size() + 1);
    REQUIRE_FALSE(same_bands(off_012, on_012));
}
