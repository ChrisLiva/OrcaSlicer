#include <catch2/catch_all.hpp>

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
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
#include <string>
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
// base offset by 0.2 / tan 61 deg = 0.1109 mm (detect_overhangs' thresh_angle is
// support_threshold_angle + 1, its lower_layer_offset is lower_layer->height / tan(threshold_rad)),
// 25 x 1.089 mm. Today's cull erodes it by one 0.42 mm line width to 24.16 x 0.249 mm and 0.249 < 0.84
// discards it (the bounding-box test in detect_overhangs' else branch); eroded by half a line width it
// is 24.58 x 0.669 mm and survives. Its far point sits 0.68 mm from the base boundary, under the 3 mm
// cantilever test (dist_max > scale_(3) in detect_overhangs), and the 30 x 8 base clears the 6 x 6
// layer-0 sharp-tail threshold (length_thresh_well_supported, applied in detect_overhangs' branch for
// layer->lower_layer == nullptr).
TriangleMesh lip_fixture()
{
    TriangleMesh base = make_cube(30, 8, 10);
    TriangleMesh lip  = make_cube(25, 1.2, 2);
    lip.translate(2.5f, 8.f, 8.f);
    base.merge(lip);
    return base;
}

// lip_fixture plus a 0.5 x 0.5 x 0.8 mm peg under the lip's -x, -y corner: model x 2.5..3.0, y 8.1..8.6,
// z 7.4..8.2, so its top 0.2 mm is inside the lip and its bottom hangs 0.6 mm (3 layers) under the lip's
// underside. In the centred object frame the lip corner contact node sits at (-12.5, 3.51) print_z 8.0
// and every peg corner node at print_z 7.4 lies within 0.60..0.92 mm of it in 3-D, inside the 1 mm
// distance, while the peg's overhang polygon (print_z 7.6) is 3 outer-vector layers under the lip band
// (print_z 8.2), outside the 2-layer island window.
TriangleMesh peg_fixture()
{
    TriangleMesh base = lip_fixture();
    TriangleMesh peg  = make_cube(0.5, 0.5, 0.8);
    peg.translate(2.5f, 8.1f, 7.4f);
    base.merge(peg);
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

// Measures one model both ways for the corpus harness (hidden, see the TEST_CASE at the bottom of
// this file): runs `run` with the mode off and then on, prints one line per mode, and writes
// `<stem>.miniature.csv` beside the corpus when `corpus_dir` is set.
void report(size_t index, const std::string &stem, const std::string &corpus_dir,
            const std::function<ContactClusters(bool mode_on)> &run)
{
    const ContactClusters off = run(false);
    const ContactClusters on  = run(true);

    // areas_mm2 is sorted ascending, so both read straight off it. An empty set has no area at all.
    const auto median = [](const std::vector<double> &a) {
        const size_t n = a.size();
        return n == 0 ? 0. : (n % 2 ? a[n / 2] : 0.5 * (a[n / 2 - 1] + a[n / 2]));
    };
    const auto p90 = [](const std::vector<double> &a) {
        const size_t n = a.size();
        return n == 0 ? 0. : a[(n - 1) * 9 / 10];
    };

    const std::pair<const char *, const ContactClusters *> modes[2] = { { "off", &off }, { "on", &on } };
    std::cout << std::setprecision(6);
    for (const auto &mode : modes)
        std::cout << "model " << index << " " << stem << " mode=" << mode.first
                  << " clusters=" << mode.second->count()
                  << " area_median=" << median(mode.second->areas_mm2)
                  << " area_p90=" << p90(mode.second->areas_mm2)
                  << " area_total=" << mode.second->total_mm2 << std::endl;

    if (corpus_dir.empty())
        return;

    const std::filesystem::path csv_path = std::filesystem::path(corpus_dir) / (stem + ".miniature.csv");
    std::ofstream               csv(csv_path.string());
    csv << std::setprecision(12);
    csv << "model,stem,mode,clusters,area_median_mm2,area_p90_mm2,area_total_mm2\n";
    for (const auto &mode : modes)
        csv << index << "," << stem << "," << mode.first << "," << mode.second->count() << ","
            << median(mode.second->areas_mm2) << "," << p90(mode.second->areas_mm2) << ","
            << mode.second->total_mm2 << "\n";
    csv.close();

    // Read the file back: a header and one row per mode, or the disk took less than was written.
    size_t        lines = 0;
    std::ifstream back(csv_path.string());
    for (std::string line; std::getline(back, line); )
        ++ lines;
    REQUIRE(lines == 3);
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

    // Leg 3, the mode on: decimation drops every contact within 1 mm of a stronger neighbour, so the
    // set thins. Each removed node takes its gap area with it, and every sliver has positive area.
    DynamicPrintConfig on_config = base;
    on_config.set_deserialize_strict({ { "support_miniature_contacts", "1" }, { "support_contact_min_distance", "1" } });

    Slic3r::Print on_print;
    init_and_process_print({ fin_fixture() }, on_print, on_config);
    const ContactClusters on = contact_clusters(*on_print.objects().front());
    REQUIRE(on.count() < off.count());
    REQUIRE(on.total_mm2 < off.total_mm2);

    // Leg 4, the mode on under one TBB worker: generate_contact_points() runs in a tbb::parallel_for,
    // so the decimation post-pass has to reach the same survivors whatever the worker count.
    ContactClusters on_one;
    {
        tbb::global_control gc(tbb::global_control::max_allowed_parallelism, 1);
        Slic3r::Print       on_one_print;
        init_and_process_print({ fin_fixture() }, on_one_print, on_config);
        on_one = contact_clusters(*on_one_print.objects().front());
    }
    REQUIRE(on_one.count() == on.count());
    REQUIRE(same_clusters(on_one, on));
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

    // Leg 3, the mode on with the cull switched off: clusters are still built (detect_overhangs'
    // find_and_insert_cluster loop), but the whole sharp-tail/small-overhang classification sits under
    // its is_auto(stype) && config_remove_small_overhangs gate, so every cluster is kept and the
    // predicate never runs.
    const auto on_no_cull = lip_overhangs({ { "support_miniature_contacts", "1" }, { "support_remove_small_overhang", "0" } });
    REQUIRE(on_no_cull.size() == 1);

    // Leg 4, the same with the mode off: the band's survival here is the cull's doing, not the mode's.
    const auto off_no_cull = lip_overhangs({ { "support_miniature_contacts", "0" }, { "support_remove_small_overhang", "0" } });
    REQUIRE(off_no_cull.size() == 1);

    // Leg 5, the mode on under organic: the organic generator (TreeSupport3D::generate_support_areas)
    // reaches this same detect_overhangs(), and the mode is a legacy-tree feature, so it must not touch
    // organic geometry. The band is culled exactly as in leg 1.
    const auto organic = lip_overhangs({ { "support_style", "organic" }, { "support_miniature_contacts", "1" } });
    REQUIRE(organic.empty());
}

TEST_CASE("Miniature contacts keep a small overhang island that sits within the contact distance of a larger one", "[MiniatureContacts]")
{
    // support_remove_small_overhang = 0 is the user's own cull setting, and the leg needs it: with the
    // stock cull on, the peg's 0.25 mm2 overhang polygon is discarded with the mode off and the mode-off
    // leg has no baseline to compare against.
    const auto clusters_for = [](const char *mode) {
        const DynamicPrintConfig config = fixture_config({ { "support_style", "tree_slim" },
                                                           { "support_top_z_distance", "0.2" },
                                                           { "support_remove_small_overhang", "0" },
                                                           { "support_miniature_contacts", mode },
                                                           { "support_contact_min_distance", "1" } });
        Slic3r::Print print;
        init_and_process_print({ peg_fixture() }, print, config);
        return contact_clusters(*print.objects().front());
    };

    // Centroids are scaled Points in the centred object frame paired with their print_z in mm.
    const auto count_in_box = [](const ContactClusters &cl, double x0, double x1, double y0, double y1, double z) {
        int n = 0;
        for (const std::pair<Point, double> &c : cl.centroids) {
            const double x = unscale<double>(c.first.x());
            const double y = unscale<double>(c.first.y());
            if (x >= x0 && x <= x1 && y >= y0 && y <= y1 && std::abs(c.second - z) < 1e-6)
                ++ n;
        }
        return n;
    };

    // Leg 1, the mode off: the peg is its own overhang island and keeps one contact cluster under it at
    // print_z 7.4, while the lip keeps its cluster at print_z 8.0.
    const ContactClusters off = clusters_for("0");
    REQUIRE(count_in_box(off, -12.8, -11.7, 3.2, 4.3, 7.4) == 1);
    REQUIRE(count_in_box(off, -13.0, 13.0, 3.3, 6.0, 8.0) >= 1);

    // Leg 2, the mode on: the peg's contact sits 0.60..0.92 mm from the lip's corner contact, inside the
    // 1 mm support_contact_min_distance, so the pre-change radius-first, island-blind order suppressed it
    // and left the peg unsupported. Decimation is per overhang island, so both clusters must survive.
    const ContactClusters on = clusters_for("1");
    REQUIRE(count_in_box(on, -12.8, -11.7, 3.2, 4.3, 7.4) == 1);
    REQUIRE(count_in_box(on, -13.0, 13.0, 3.3, 6.0, 8.0) >= 1);
}

TEST_CASE("decimate_contact_nodes honours pinning, identity, the strict bound and a non-positive distance", "[MiniatureContacts]")
{
    // The nodes live in a deque so their addresses stay put: decimation erases pointers from the
    // per-layer vectors and never touches the nodes themselves.
    std::deque<SupportNode>                pool;
    std::vector<std::vector<SupportNode*>> nodes(2);
    auto add = [&](size_t layer, double x_mm, double y_mm, double z_mm, double radius, bool pinned, int island = -1) -> SupportNode* {
        pool.emplace_back();
        SupportNode *n = &pool.back();
        n->position    = Point(scale_(x_mm), scale_(y_mm));
        n->print_z     = z_mm;
        n->radius      = radius;
        n->is_pinned   = pinned;
        n->island      = island;
        nodes[layer].push_back(n);
        return n;
    };

    // Layer 0, in push order: A B C D E G G H, all on the x axis at z 0.
    SupportNode *A = add(0, 0.0, 0., 0., 0.6, false);
                     add(0, 0.5, 0., 0., 0.5, false); // B, 0.5 mm from the larger-radius A
    SupportNode *C = add(0, 3.0, 0., 0., 0.4, false);
    SupportNode *D = add(0, 3.5, 0., 0., 0.4, true);  // pinned: kept, and suppresses nobody
    SupportNode *E = add(0, 6.0, 0., 0., 0.4, false);
    SupportNode *G = add(0, 9.0, 0., 0., 0.4, false);
    nodes[0].push_back(G);                            // the same pointer listed twice, as the Hybrid
                                                      // big-overhang path does: insert_point emplaces the
                                                      // node in generate_contact_points, then its ePolygon
                                                      // caller emplaces the same pointer again
                     add(0, 9.5, 0., 0., 0.4, false); // H, 0.5 mm from G
    // Layer 1: F sits exactly 1.0 mm above E.
    SupportNode *F = add(1, 6.0, 0., 1.0, 0.4, false);

    // Islands. P and Q are 0.5 mm apart but in different islands, so both survive; T is the pendant tip:
    // the lowest node of island 3, so it beats U, the thicker node 0.58 mm above it in the same island;
    // V is 0.72 mm from T in another island and survives.
    SupportNode *P = add(0, 12.0, 0., 0.,  0.4,  false, 1);
    SupportNode *Q = add(0, 12.5, 0., 0.,  0.6,  false, 2);
    SupportNode *T = add(0, 20.0, 0., 0.,  0.22, false, 3);
                     add(1, 20.3, 0., 0.5, 0.6,  false, 3); // U, suppressed by T
    SupportNode *V = add(1, 20.6, 0., 0.4, 0.6,  false, 4);

    const std::vector<std::vector<SupportNode*>> original = nodes;
    // B falls to the larger-radius A at 0.5 mm; C keeps its place because its only neighbour inside
    // 1.0 mm is the pinned D; D is pinned; G's second entry survives because a node never suppresses
    // itself; H falls to G at 0.5 mm; P and Q both survive because they sit in different islands.
    const std::vector<SupportNode*> expected0{ A, C, D, E, G, G, P, Q, T };
    // E and F are exactly 1.0 mm apart and the bound is strict, so both survive. Layer-1 push order is
    // F, U, V; U falls to T, the lower node of its own island, and V keeps its place in another island.
    const std::vector<SupportNode*> expected1{ F, V };

    decimate_contact_nodes(nodes, 1.0);
    REQUIRE(nodes.size() == 2);
    REQUIRE(nodes[0] == expected0);
    REQUIRE(nodes[1] == expected1);

    // Idempotent: a second pass over the survivors changes nothing.
    decimate_contact_nodes(nodes, 1.0);
    REQUIRE(nodes[0] == expected0);
    REQUIRE(nodes[1] == expected1);

    // A non-positive distance is a no-op, on the untouched input.
    std::vector<std::vector<SupportNode*>> copy = original;
    decimate_contact_nodes(copy, 0.);
    REQUIRE(copy == original);
    decimate_contact_nodes(copy, -1.);
    REQUIRE(copy == original);
    REQUIRE(copy[0].size() == 11);
}

TEST_CASE("assign_contact_islands links overhang polygons on the same or a nearby layer within the higher layer's dilation and nothing else", "[MiniatureContacts]")
{
    // The nodes live in a deque so their addresses stay put: the pass writes SupportNode::island and
    // leaves the per-layer vectors alone.
    std::deque<SupportNode>                pool;
    std::vector<std::vector<SupportNode*>> nodes(12);
    // An axis-aligned square in scaled units, lower-left corner at (x_mm, y_mm).
    const auto square = [](double x_mm, double y_mm, double side_mm) {
        ExPolygon ex;
        ex.contour.points = { Point(scale_(x_mm),           scale_(y_mm)),
                              Point(scale_(x_mm + side_mm), scale_(y_mm)),
                              Point(scale_(x_mm + side_mm), scale_(y_mm + side_mm)),
                              Point(scale_(x_mm),           scale_(y_mm + side_mm)) };
        return ex;
    };
    auto add = [&](size_t layer, const ExPolygon &overhang) -> SupportNode* {
        pool.emplace_back();
        SupportNode *n = &pool.back();
        n->overhang    = overhang;
        nodes[layer].push_back(n);
        return n;
    };

    SupportNode *A0   = add(0, square(0., 0., 1.));
    SupportNode *B0   = add(0, square(5., 0., 1.));
    nodes[0].push_back(B0);                             // the same pointer listed twice: still one node
    SupportNode *Bdup = add(0, square(5., 0., 1.));     // a distinct node on B0's square
    SupportNode *A1   = add(1, square(0.2, 0., 1.));    // overlaps A0 outright, 1 index above it
    SupportNode *C    = add(2, ExPolygon());            // no polygon at all
    SupportNode *A2   = add(3, square(1.25, 0., 1.));   // 2 indices above A1, XY gap 0.05 < 0.3
    SupportNode *D    = add(4, square(1.25, 1.35, 1.)); // 1 index above A2, XY gap 0.35 > 0.3
    SupportNode *A3   = add(6, square(0., 0., 1.));     // 3 indices above A2, outside the 2-layer window
    // Layers 7..11 carry their own dilation (the vector below). x 20 and beyond keeps every square here
    // out of the A chain's reach, so each pair is judged on its own gap.
    SupportNode *H7   = add(7,  square(20.,  0., 1.));  // dilation 0.2 on this layer, 0.8 on H8's
    SupportNode *H8   = add(8,  square(21.5, 0., 1.));  // 0.5 mm from H7 in x: the higher layer's 0.8 spans it
    SupportNode *L9   = add(9,  square(30.,  0., 1.));  // dilation 0.8 on this layer, 0.2 on L10's
    SupportNode *L10  = add(10, square(31.5, 0., 1.));  // 0.5 mm from L9 in x: the lower layer's 0.8 does not count
    SupportNode *S11a = add(11, square(40.,  0., 1.));  // two different polygons on one layer, 0.5 mm apart
    SupportNode *S11b = add(11, square(41.5, 0., 1.));  // linked to S11a by layer 11's own 0.8

    // 0.3 on the A chain's layers as before; layers 7..11 pair a small and a large value so which
    // index the gap rule reads shows in the ids.
    std::vector<coord_t> dilation(12, scale_(0.3));
    dilation[7]  = scale_(0.2); dilation[8]  = scale_(0.8);
    dilation[9]  = scale_(0.8); dilation[10] = scale_(0.2);
    dilation[11] = scale_(0.8);
    assign_contact_islands(nodes, dilation, 2);

    // Ids are dense and first-seen over (layer asc, index asc): A0's chain is 0, B0's pair 1, D 2,
    // A3 3, the H pair 4, L9 5, L10 6, the S pair 7.
    REQUIRE(A0->island == 0);
    REQUIRE(A1->island == 0);
    REQUIRE(A2->island == 0);
    REQUIRE(B0->island == 1);
    REQUIRE(Bdup->island == 1);
    REQUIRE(C->island == -1);
    REQUIRE(D->island == 2);
    REQUIRE(A3->island == 3);
    REQUIRE(H7->island == 4);
    REQUIRE(H8->island == 4);
    REQUIRE(L9->island == 5);
    REQUIRE(L10->island == 6);
    REQUIRE(S11a->island == 7);
    REQUIRE(S11b->island == 7);

    // Idempotent, and the pass never edits the vectors.
    assign_contact_islands(nodes, dilation, 2);
    REQUIRE(A0->island == 0);
    REQUIRE(A1->island == 0);
    REQUIRE(A2->island == 0);
    REQUIRE(B0->island == 1);
    REQUIRE(Bdup->island == 1);
    REQUIRE(C->island == -1);
    REQUIRE(D->island == 2);
    REQUIRE(A3->island == 3);
    REQUIRE(H7->island == 4);
    REQUIRE(H8->island == 4);
    REQUIRE(L9->island == 5);
    REQUIRE(L10->island == 6);
    REQUIRE(S11a->island == 7);
    REQUIRE(S11b->island == 7);
    REQUIRE(nodes[0].size() == 4);
}

TEST_CASE("decimate_contact_nodes keeps the same survivors as a brute-force reference over a total order", "[MiniatureContacts]")
{
    for (int seed = 0; seed < 10; ++ seed) {
        INFO("seed " << seed);
        std::mt19937                           rng(0x5EED + seed); // fixed seed: the run is deterministic
        std::uniform_int_distribution<coord_t> dist_xy(0, scale_(20.));
        std::uniform_int_distribution<int>     dist_k(0, 49);
        std::uniform_int_distribution<int>     dist_r(0, 2);
        std::uniform_int_distribution<int>     dist_pin(0, 19);
        std::uniform_int_distribution<int>     dist_island(-1, 3);

        // k runs 0..49 and layer = k / 5, so the 500 nodes address exactly these 10 layers and
        // REQUIRE(actual.size() == 10) asserts the outer vector kept its size.
        std::deque<SupportNode>                pool;
        std::vector<std::vector<SupportNode*>> input(10);
        for (int i = 0; i < 500; ++ i) {
            pool.emplace_back();
            SupportNode *n = &pool.back();
            n->position    = Point(dist_xy(rng), dist_xy(rng));
            const int k    = dist_k(rng);
            n->print_z     = 0.2 * k;
            n->radius      = 0.4 + 0.1 * dist_r(rng);
            n->is_pinned   = dist_pin(rng) == 0;
            n->island      = dist_island(rng);
            input[k / 5].push_back(n);
        }

        // The reference: the same total order, then an O(n^2) sweep keeping every survivor as a
        // blocker. A pinned node is kept and never becomes a blocker.
        struct Ref { size_t layer; size_t index; SupportNode *node; };
        std::vector<Ref> refs;
        for (size_t layer = 0; layer < input.size(); ++ layer)
            for (size_t index = 0; index < input[layer].size(); ++ index)
                refs.push_back({ layer, index, input[layer][index] });
        std::stable_sort(refs.begin(), refs.end(), [](const Ref &a, const Ref &b) {
            if (a.node->print_z != b.node->print_z)
                return a.node->print_z < b.node->print_z;
            if (a.node->radius != b.node->radius)
                return a.node->radius > b.node->radius;
            if (a.layer != b.layer)
                return a.layer < b.layer;
            return a.node->position < b.node->position;
        });

        std::vector<std::vector<bool>> keep(input.size());
        for (size_t layer = 0; layer < input.size(); ++ layer)
            keep[layer].assign(input[layer].size(), false);
        std::vector<const SupportNode*> blockers;
        for (const Ref &r : refs) {
            if (r.node->is_pinned) {
                keep[r.layer][r.index] = true;
                continue;
            }
            bool dropped = false;
            for (const SupportNode *b : blockers) {
                if (b == r.node)
                    continue;
                if (b->island != r.node->island)
                    continue; // a kept node suppresses only inside its own island
                const double dx = unscale<double>(b->position.x() - r.node->position.x());
                const double dy = unscale<double>(b->position.y() - r.node->position.y());
                const double dz = b->print_z - r.node->print_z;
                if (sqr(dx) + sqr(dy) + sqr(dz) < 1.0) {
                    dropped = true;
                    break;
                }
            }
            if (dropped)
                continue;
            keep[r.layer][r.index] = true;
            blockers.push_back(r.node);
        }

        std::vector<std::vector<SupportNode*>> expected(input.size());
        size_t                                 total_survivors = 0;
        for (size_t layer = 0; layer < input.size(); ++ layer)
            for (size_t index = 0; index < input[layer].size(); ++ index)
                if (keep[layer][index]) {
                    expected[layer].push_back(input[layer][index]);
                    ++ total_survivors;
                }

        std::vector<std::vector<SupportNode*>> actual = input;
        decimate_contact_nodes(actual, 1.0);
        REQUIRE(actual.size() == 10);
        for (size_t layer = 0; layer < expected.size(); ++ layer) {
            INFO("layer " << layer);
            REQUIRE(actual[layer] == expected[layer]);
        }
        // 500 nodes in a 20 x 20 x 10 mm box make 124,750 pairs, each within 1.0 mm with probability
        // about 0.001: some pair collides in every run, so decimation always removes something.
        REQUIRE(total_survivors < 500);
    }
}

// Hidden ([.]): one corpus model costs two full Print::process() passes on a miniature at fine
// layers, minutes each, and the corpus it reads lives outside the repo under
// $ORCA_MINIATURE_CORPUS (tests/AGENTS.md:44). It measures how far decimation thins the contact
// set, model by model; it does not pin it.
TEST_CASE("Miniature contact decimation over a corpus", "[MiniatureContacts][.]")
{
    // Model::get_backup_path() builds from temporary_dir(), which is "" in a test process, so each
    // 3mf load logs two "Failed to create backup path /orcaslicer_model/...: Read-only file system"
    // errors that read like a failure but are caught and non-fatal. Point it at the OS temp dir the
    // way the app does at src/OrcaSlicer.cpp:1330.
    Slic3r::set_temporary_dir(std::filesystem::temp_directory_path().string());

    const char       *env        = std::getenv("ORCA_MINIATURE_CORPUS");
    const std::string corpus_dir = env != nullptr ? std::string(env) : std::string();

    // Model 0 is always the built-in fin fixture, under the same two settings the pinning test
    // spells out: the organic generator never fills roof_gap_areas, and at a zero top gap the tips
    // land outside them, so either default would measure zero both ways.
    report(0, "fin_fixture", corpus_dir, [&](bool on) {
        Slic3r::Print print;
        init_and_process_print({ fin_fixture() }, print,
            fixture_config({ { "support_style", "tree_slim" }, { "support_top_z_distance", "0.2" },
                             { "support_miniature_contacts", on ? "1" : "0" } }));
        return contact_clusters(*print.objects().front());
    });

    if (corpus_dir.empty()) {
        std::cout << "corpus dir not set, model 0 only" << std::endl;
        return;
    }

    size_t index = 1;
    for (const std::filesystem::path &path : corpus_files(corpus_dir)) {
        const std::string      stem = path.stem().string();
        DynamicPrintConfig     loaded;
        std::unique_ptr<Model> model;
        try {
            // The 3mf importer creates no object without LoadModel and reads no config without
            // LoadConfig (Format/bbs_3mf.cpp:1419, :1422), so the default strategy hands back an
            // empty model; load the way the CLI does (src/OrcaSlicer.cpp:1648).
            model.reset(new Model(Model::read_from_file(path.string(), &loaded, nullptr,
                LoadStrategy::LoadModel | LoadStrategy::LoadConfig | LoadStrategy::AddDefaultInstances)));
        } catch (const std::exception &e) {
            std::cout << "model " << index << " " << stem << " skipped: " << e.what() << std::endl;
            ++ index;
            continue;
        }
        if (model->objects.empty()) {
            std::cout << "model " << index << " " << stem << " skipped: no printable instance" << std::endl;
            ++ index;
            continue;
        }
        // corpus_config leaves a Strong or Hybrid style the file chose alone and forces tree_slim
        // only over default or organic. support_contact_min_distance stays at whatever the loaded
        // config carries, which is 1.0 for any file saved before this key existed.
        const DynamicPrintConfig config = corpus_config(fixture_config({ { "support_style", "tree_slim" },
            { "support_top_z_distance", "0.2" } }), loaded);
        // roof_gap_areas is the gap the generator leaves between a contact tip and the object, so a
        // config that prints the tips straight onto the object files none of it: both modes would
        // measure an empty set, whatever decimation did.
        if (config.opt_float("support_top_z_distance") <= 0.) {
            std::cout << "model " << index << " " << stem
                      << " skipped: support_top_z_distance is 0, roof_gap_areas would be empty" << std::endl;
            continue;
        }
        // One harness model per object: the print measured has to hold exactly the one object, and
        // its single instance is centred on the bed and dropped onto it before slicing.
        for (const ModelObject *src : model->objects) {
            const std::string obj_stem = corpus_stem(path, model->objects.size(), src->name);
            if (src->instances.empty()) {
                std::cout << "model " << index << " " << obj_stem << " skipped: no printable instance" << std::endl;
                ++ index;
                continue;
            }
            Model        one;
            ModelObject *obj = one.add_object(*src);
            while (obj->instances.size() > 1)
                obj->delete_last_instance();
            one.center_instances_around_point(unscale(BoundingBox(get_bed_shape(config)).center()));
            obj->ensure_on_bed();

            report(index, obj_stem, corpus_dir, [&](bool on) {
                DynamicPrintConfig mode_config = config;
                mode_config.set_deserialize_strict({ { "support_miniature_contacts", on ? "1" : "0" } });
                Slic3r::Print print;
                print.set_status_silent();
                print.apply(one, mode_config);
                print.process();
                return contact_clusters(*print.objects().front());
            });
            ++ index;
        }
    }
}
