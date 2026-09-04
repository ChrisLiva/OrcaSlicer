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

// A 30 x 8 x 10 mm base carrying a 25 x 1.2 x 2 mm lip on its +y face: make_cube builds from the
// origin corner (src/libslic3r/TriangleMesh.cpp:886-894), so the lip spans x 2.5..27.5, y 8..9.2,
// z 8..10. At threshold 60 and 0.2 mm layers the band on the lip's first layer is the lip minus the
// base offset by 0.2 / tan 61 deg = 0.1109 mm (TreeSupport.cpp:706-708, :846-848), 25 x 1.089 mm.
// Today's cull erodes it by one 0.42 mm line width to 24.16 x 0.249 mm and 0.249 < 0.84 discards it
// (:1027-1029); eroded by half a line width it is 24.58 x 0.669 mm and survives. Its far point sits
// 0.68 mm from the base boundary, under the 3 mm cantilever test (:905), and the 30 x 8 base clears
// the 6 x 6 layer-0 sharp-tail threshold (:702, :833-843).
TriangleMesh lip_fixture()
{
    TriangleMesh base = make_cube(30, 8, 10);
    TriangleMesh lip  = make_cube(25, 1.2, 2);
    lip.translate(2.5f, 8.f, 8.f);
    base.merge(lip);
    return base;
}

// Every overhang band detect_overhangs() leaves on a layer above the first, as (print_z, band bbox),
// under fixture_config's tree_slim settings overlaid with `extra`. fixture_config takes one
// initializer_list, so `extra` is applied to the config it returns.
std::vector<std::pair<double, BoundingBox>> lip_overhangs(std::initializer_list<ConfigBase::SetDeserializeItem> extra)
{
    DynamicPrintConfig config = fixture_config({ { "support_style", "tree_slim" }, { "support_top_z_distance", "0.2" } });
    // set_deserialize_strict drops a key print_config_def does not carry instead of throwing, so every
    // key spelled here has to match a declared option exactly.
    config.set_deserialize_strict(extra);

    Slic3r::Print print;
    Slic3r::Model model;
    init_print({ lip_fixture() }, print, model, config);

    PrintObject *po = print.objects_mutable().front();
    po->slice();
    TreeSupport ts(*po, po->slicing_parameters());
    ts.detect_overhangs();

    std::vector<std::pair<double, BoundingBox>> bands;
    for (const Layer *layer : po->layers()) {
        if (layer->id() == 0)
            continue;
        for (const ExPolygon &expoly : layer->loverhangs)
            bands.emplace_back(layer->print_z, get_extents(expoly));
    }
    return bands;
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

TEST_CASE("Miniature contacts keep a long thin overhang lip that the small-overhang cull would discard", "[MiniatureContacts]")
{
    // Leg 1, stock: the bounding-box cull fails the band on its 0.249 mm y extent and drops it.
    const auto culled = lip_overhangs({ { "support_miniature_contacts", "0" } });
    REQUIRE(culled.empty());

    // Leg 2, the mode on: half a line width is the room one extrusion needs, so the band survives on
    // the layer whose bottom is the lip's z = 8.0, the 41st layer at 0.2 mm with a 0.2 mm first layer.
    const auto kept = lip_overhangs({ { "support_miniature_contacts", "1" } });
    REQUIRE(kept.size() == 1);
    REQUIRE_THAT(kept[0].first, WithinAbs(8.2, 1e-6));
    const Vec2d sz = unscale(kept[0].second.size());
    REQUIRE_THAT(sz.x(), WithinAbs(25.0, 0.05));
    REQUIRE_THAT(sz.y(), WithinAbs(1.089, 0.05));

    // Leg 3, the mode on with the cull switched off: clusters are still built (TreeSupport.cpp:995-1004),
    // but the whole sharp-tail/small-overhang classification sits under the support_remove_small_overhang
    // gate at :1012, so every cluster is kept and the predicate never runs.
    const auto on_no_cull = lip_overhangs({ { "support_miniature_contacts", "1" }, { "support_remove_small_overhang", "0" } });
    REQUIRE(on_no_cull.size() == 1);

    // Leg 4, the same with the mode off: the band's survival here is the cull's doing, not the mode's.
    const auto off_no_cull = lip_overhangs({ { "support_miniature_contacts", "0" }, { "support_remove_small_overhang", "0" } });
    REQUIRE(off_no_cull.size() == 1);
}
