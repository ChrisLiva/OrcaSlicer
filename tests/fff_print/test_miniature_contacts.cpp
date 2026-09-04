#include <catch2/catch_all.hpp>

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Support/TreeSupport.hpp"
#include "libslic3r/Utils.hpp"

#include <tbb/global_control.h>

#include <algorithm>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <random>
#include <utility>
#include <vector>

#include "test_helpers.hpp"

using namespace Slic3r::Test;
using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {

// One contact cluster per ExPolygon of union_ex(sl->tree_roof_gap_areas()) on every support layer whose
// print_z exceeds first_print_layer_height (plate contact is not object contact), the measure
// tests/fff_print/test_auto_tilt.cpp:421-441 uses. Areas in mm2, sorted ascending; centroids as
// (scaled Point, print_z) in layer order so two runs compare exactly.
struct ContactClusters {
    std::vector<double>                    areas_mm2;   // sorted ascending
    std::vector<std::pair<Point, double>>  centroids;   // (contour centroid, print_z), layer order
    double                                 total_mm2 = 0.;
    size_t count() const { return areas_mm2.size(); }
};

ContactClusters contact_clusters(const PrintObject &po)
{
    ContactClusters clusters;
    const double    first_z = po.slicing_parameters().first_print_layer_height;
    for (const SupportLayer *sl : po.support_layers()) {
        if (sl->print_z <= first_z + EPSILON)
            continue; // plate contact, not object contact
        for (const ExPolygon &p : union_ex(sl->tree_roof_gap_areas())) {
            clusters.areas_mm2.push_back(p.area() * SCALING_FACTOR * SCALING_FACTOR);
            clusters.centroids.emplace_back(p.contour.centroid(), sl->print_z);
        }
    }
    std::sort(clusters.areas_mm2.begin(), clusters.areas_mm2.end());
    // Serial, in sorted order: two runs sum the same floats in the same order.
    for (double a : clusters.areas_mm2)
        clusters.total_mm2 += a;
    return clusters;
}

// Same count, every centroid Point equal and print_z within 1e-9, every sorted area within 1e-9 mm2.
bool same_clusters(const ContactClusters &a, const ContactClusters &b)
{
    if (a.count() != b.count())
        return false;
    for (size_t i = 0; i < a.areas_mm2.size(); ++ i)
        if (std::abs(a.areas_mm2[i] - b.areas_mm2[i]) > 1e-9)
            return false;
    for (size_t i = 0; i < a.centroids.size(); ++ i) {
        if (a.centroids[i].first != b.centroids[i].first)
            return false;
        if (std::abs(a.centroids[i].second - b.centroids[i].second) > 1e-9)
            return false;
    }
    return true;
}

} // namespace

TEST_CASE("Miniature contacts leave a stock slice untouched when off and thin the contact set when on", "[MiniatureContacts]")
{
    // support_style, because the default resolves to organic and the organic generator never fills
    // roof areas; support_top_z_distance explicitly, because at 0 the tips land outside
    // roof_gap_areas and the measure comes back empty. 0.2 is the value PrintConfig.cpp defaults to.
    const DynamicPrintConfig base = fixture_config({ { "support_style", "tree_slim" }, { "support_top_z_distance", "0.2" } });

    // Leg 1, stock: the fin under tree_slim fills roof_gap_areas on layers above the first.
    Slic3r::Print stock_print;
    init_and_process_print({ fin_fixture() }, stock_print, base);
    const ContactClusters stock = contact_clusters(*stock_print.objects().front());
    REQUIRE(stock.count() > 0);

    // Leg 2, the mode explicitly off. PrintConfigDef::handle_legacy() clears any key print_config_def
    // does not carry, so set_deserialize_strict drops an unknown key instead of throwing; reading both
    // values back is what proves the keys are declared and that this leg is more than a copy of leg 1.
    DynamicPrintConfig off_config = base;
    off_config.set_deserialize_strict({ { "support_miniature_contacts", "0" }, { "support_contact_min_distance", "1" } });
    REQUIRE(off_config.option("support_miniature_contacts") != nullptr);
    REQUIRE(off_config.option("support_contact_min_distance") != nullptr);
    REQUIRE(off_config.opt_bool("support_miniature_contacts") == false);
    REQUIRE_THAT(off_config.opt_float("support_contact_min_distance"), WithinAbs(1., 1e-9));

    Slic3r::Print off_print;
    init_and_process_print({ fin_fixture() }, off_print, off_config);
    const ContactClusters off = contact_clusters(*off_print.objects().front());
    REQUIRE(off.count() == stock.count());
    REQUIRE(same_clusters(off, stock));

    // Legs 3 (the mode on) and 4 (the mode on under one TBB thread) arrive with the wiring step.
}
