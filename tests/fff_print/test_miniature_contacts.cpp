#include <catch2/catch_all.hpp>

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Geometry/ConvexHull.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Support/MiniatureSupport.hpp"
#include "libslic3r/Support/ModelSupportRisk.hpp"
#include "libslic3r/Support/SupportAnalysis.hpp"
#include "libslic3r/Support/TreeSupport.hpp"

#include <tbb/global_control.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iterator>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "support_validation.hpp"
#include "test_helpers.hpp"

using namespace Slic3r::Test;
using namespace Slic3r;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

// The corpus measurements both hidden validation harnesses share, by their short names
// (tests/fff_print/support_validation.hpp).
using SupportValidation::ContactClusters;
using SupportValidation::contact_clusters;
using SupportValidation::same_clusters;
using SupportValidation::Envelope;
using SupportValidation::envelope_of;
using SupportValidation::SupportMetrics;
using SupportValidation::support_metrics;

// The per-object-layer annotations overhang detection leaves on the object: sharp tails, the height
// each of them has accumulated, and cantilevers. They are part of what one support pass produces, so
// a pass that hands its output back to the object has to hand these back with it, whole.
struct Annotations
{
    size_t layers_with_sharp_tails = 0, layers_with_cantilevers = 0;
    size_t sharp_tails = 0, sharp_tails_height = 0, cantilevers = 0;
    double sharp_tail_mm2 = 0., cantilever_mm2 = 0.;
};

Annotations annotations(const PrintObject &po)
{
    Annotations a;
    for (const Layer *layer : po.layers()) {
        if (! layer->sharp_tails.empty())
            ++ a.layers_with_sharp_tails;
        if (! layer->cantilevers.empty())
            ++ a.layers_with_cantilevers;
        a.sharp_tails        += layer->sharp_tails.size();
        a.sharp_tails_height += layer->sharp_tails_height.size();
        a.cantilevers        += layer->cantilevers.size();
        for (const ExPolygon &p : layer->sharp_tails)
            a.sharp_tail_mm2 += p.area() * SCALING_FACTOR * SCALING_FACTOR;
        for (const ExPolygon &p : layer->cantilevers)
            a.cantilever_mm2 += p.area() * SCALING_FACTOR * SCALING_FACTOR;
    }
    return a;
}

// Same counts and the same areas to within 1e-9 mm2.
bool same_annotations(const Annotations &a, const Annotations &b)
{
    if (a.layers_with_sharp_tails != b.layers_with_sharp_tails)
        return false;
    if (a.layers_with_cantilevers != b.layers_with_cantilevers)
        return false;
    if (a.sharp_tails != b.sharp_tails)
        return false;
    if (a.sharp_tails_height != b.sharp_tails_height)
        return false;
    if (a.cantilevers != b.cantilevers)
        return false;
    if (std::abs(a.sharp_tail_mm2 - b.sharp_tail_mm2) > 1e-9)
        return false;
    return std::abs(a.cantilever_mm2 - b.cantilever_mm2) <= 1e-9;
}

// What a finished support pass has to leave behind whatever it generated: every support layer belongs
// to the object that owns the pass (`owner` is that object, which is the shared owner when `po` reads
// its layers through one), the layers are ordered by print_z with no repeat, and every sharp tail
// carries the accumulated height detect_overhangs() indexes it by.
void require_valid_support_layers(const PrintObject &po, const PrintObject &owner)
{
    double previous_z = -1.;
    for (const SupportLayer *sl : po.support_layers()) {
        REQUIRE(sl->object() == &owner);
        REQUIRE(sl->print_z > previous_z);
        previous_z = sl->print_z;
    }
    for (const Layer *layer : po.layers())
        REQUIRE(layer->sharp_tails.size() == layer->sharp_tails_height.size());
}

// Entity count and volume per extrusion role, over one collection and the collections nested in it.
void collect_roles(const ExtrusionEntityCollection &collection, std::map<int, std::pair<size_t, double>> &out)
{
    for (const ExtrusionEntity *e : collection.entities)
        if (e->is_collection())
            collect_roles(*static_cast<const ExtrusionEntityCollection*>(e), out);
        else {
            std::pair<size_t, double> &slot = out[int(e->role())];
            ++ slot.first;
            slot.second += e->total_volume();
        }
}

// Every extrusion the object's own regions carry, by role. Support material lives in its own
// collection on SupportLayer and the skirt and the brim belong to the Print rather than the object,
// so what this walks is the part of the output one machine reproduces exactly from run to run.
std::map<int, std::pair<size_t, double>> role_totals(const PrintObject &po)
{
    std::map<int, std::pair<size_t, double>> totals;
    for (const Layer *layer : po.layers())
        for (const LayerRegion *region : layer->regions()) {
            collect_roles(region->perimeters, totals);
            collect_roles(region->fills, totals);
        }
    return totals;
}

// The same roles, the same entity count in each, and the same volume to within 1e-9 mm3.
bool same_roles(const std::map<int, std::pair<size_t, double>> &a, const std::map<int, std::pair<size_t, double>> &b)
{
    if (a.size() != b.size())
        return false;
    auto ib = b.begin();
    for (auto ia = a.begin(); ia != a.end(); ++ ia, ++ ib) {
        if (ia->first != ib->first)
            return false;
        if (ia->second.first != ib->second.first)
            return false;
        if (std::abs(ia->second.second - ib->second.second) > 1e-9)
            return false;
    }
    return true;
}

// Two legs measuring the same varying metric: their spreads have to overlap, and their medians have
// to sit no further apart than the wider leg's own spread. Both bounds come from the run at hand, so
// nothing here carries a host-specific range from one machine to another.
void require_same_envelope(const std::string &name, const Envelope &a, const Envelope &b)
{
    INFO(name << " stock [" << a.min << ", " << a.max << "] median " << a.median
              << " vs off [" << b.min << ", " << b.max << "] median " << b.median);
    REQUIRE(a.min <= b.max + 1e-9);
    REQUIRE(b.min <= a.max + 1e-9);
    REQUIRE(std::abs(a.median - b.median) <= std::max(a.width(), b.width()) + 1e-9);
}

// A 30 x 8 x 10 mm base carrying a 25 x 1.2 x 2 mm lip on its +y face: make_cube builds from the
// origin corner (its_make_cube()), so the lip spans x 2.5..27.5, y 8..9.2, z 8..10. At threshold 60
// and 0.2 mm layers the band on the lip's first layer is the lip minus the base offset by
// 0.2 / tan 61 deg = 0.1109 mm (detect_overhangs' thresh_angle is support_threshold_angle + 1, its
// lower_layer_offset is lower_layer->height / tan(threshold_rad)),
// 25 x 1.089 mm. With the mode off the cull erodes it by one 0.42 mm line width to 24.16 x 0.249 mm and 0.249 < 0.84
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

// peg_fixture plus a second identical peg 3 mm to its +x: model x 6.0..6.5, the same y 8.1..8.6 and
// z 7.4..8.2. The two peg bottoms are separate overhang polygons on one object layer, their contacts
// sit 3.5 mm apart, and generate_contact_points' own per-layer hash grid is coarse enough that two
// pegs closer than this share a cell and only one of them is ever given a contact.
TriangleMesh pegs_fixture()
{
    TriangleMesh base = peg_fixture();
    TriangleMesh peg  = make_cube(0.5, 0.5, 0.8);
    peg.translate(6.0f, 8.1f, 7.4f);
    base.merge(peg);
    return base;
}

// lip_fixture plus a 4 x 0.5 x 1.4 mm bar under the lip's -x end: model x 2.5..6.5, y 8.1..8.6,
// z 6.8..8.2, so the bar's top 0.2 mm is inside the lip and its bottom hangs 1.2 mm (6 layers) under
// the lip's underside. It is peg_fixture's peg stretched along x and up in z: an island appearing in
// mid-air, which detection files both as an overhang and as a sharp tail, and long enough that the
// contact pass places more than one contact under it where the peg takes exactly one.
TriangleMesh tail_fixture()
{
    TriangleMesh base = lip_fixture();
    TriangleMesh bar  = make_cube(4.0, 0.5, 1.4);
    bar.translate(2.5f, 8.1f, 6.8f);
    base.merge(bar);
    return base;
}

// A 6 x 6 x 8 mm column carrying a 6 x 3 x 2 mm guard that starts 2 mm inside the column's top and
// reaches 4 mm past it, and off the guard's own +x face a 6 x 0.8 x 2 mm blade at the same height:
// model x 0..16, y 0..6, z 0..10. What juts past the column is one overhang polygon on the layer the
// guard and blade appear on, and the model under that polygon is 3 mm across at the guard end and
// 0.8 mm at the blade end. In the centred object frame the guard spans x -4..2, y -1.5..1.5, the
// blade x 2..8, y -0.4..0.4, and the overhang starts where the column's face is, at x -2.
TriangleMesh blade_fixture()
{
    TriangleMesh column = make_cube(6, 6, 8);
    TriangleMesh guard  = make_cube(6, 3, 2);
    guard.translate(4.f, 1.5f, 8.f);
    column.merge(guard);
    TriangleMesh blade = make_cube(6, 0.8, 2);
    blade.translate(10.f, 2.6f, 8.f);
    column.merge(blade);
    return column;
}

// A closed 24 x 24 x 14 mm box with 2 mm walls: a 20 x 20 x 10 mm void with a solid floor at z 2 and
// a ceiling at z 12. The ceiling's underside is an overhang the walls give no way out of, so a branch
// under it can only come down onto the floor, which is model. The termination is forced by the
// geometry; what the settings decide is whether it counts.
TriangleMesh box_fixture()
{
    TriangleMesh box  = make_cube(24, 24, 2);          // floor
    TriangleMesh wall = make_cube(24, 2, 14);
    box.merge(wall);                                    // y 0..2
    wall = make_cube(24, 2, 14);
    wall.translate(0.f, 22.f, 0.f);
    box.merge(wall);                                    // y 22..24
    wall = make_cube(2, 24, 14);
    box.merge(wall);                                    // x 0..2
    wall = make_cube(2, 24, 14);
    wall.translate(22.f, 0.f, 0.f);
    box.merge(wall);                                    // x 22..24
    TriangleMesh ceiling = make_cube(24, 24, 2);
    ceiling.translate(0.f, 0.f, 12.f);
    box.merge(ceiling);
    return box;
}

// box_fixture() with a 19 x 19 x 1 mm platform floating at z 6..7 inside its void, 0.5 mm clear of
// every wall. The ceiling's branches have no way past the platform, so every branch above z 7 rests on
// model material connected to nothing below it.
TriangleMesh caged_island_fixture()
{
    TriangleMesh box      = box_fixture();
    TriangleMesh platform = make_cube(19, 19, 1);
    platform.translate(2.5f, 2.5f, 6.f);
    box.merge(platform);
    return box;
}

// A 5 x 0.6 x 4 mm block at x -1..4 carrying a 20 x 0.6 x 2 mm slab rotated 10 degrees about Y,
// its underside rising from z 3.29 at x 0 to 6.76 at x 19.7 and crossing the block's top at
// x 4.03, so every band from the block's +x face up is an overhang of a face 10 degrees off
// horizontal. At 0.06 mm layers under a 20 degree threshold (21 effective) each layer sheds a
// 0.184 mm wide band stepping 0.34 mm in x, one region per layer, all one component; 0.6 mm
// across so every cell of a band lies within 1 mm of the contact one band down.
TriangleMesh wedge_fixture()
{
    TriangleMesh base = make_cube(5, 0.6, 4);
    base.translate(-1.f, 0.f, 0.f);
    TriangleMesh slab = make_cube(20, 0.6, 2);
    slab.rotate_y(float(-10. * M_PI / 180.));
    slab.translate(0.f, 0.f, 3.29f);
    base.merge(slab);
    return base;
}

// A 6.4 x 2 x 4 mm wall at x -0.9..5.5, y -2.3..-0.3 carrying, off its y -0.3 face, a 4.6 x 0.6 x 1 mm bar
// at x 0..4.6, z 2.2..3.2 and two 0.5 x 0.6 x 1.2 mm pads at x -0.6..-0.1 and x 4.7..5.2, z 2.0..3.2.
// Each pad's underside is one overhang region one layer under the bar's, 0.1 mm from its end, so the
// three regions are one component; the pads' kept contacts sit within 1.5 mm of the bar's corner
// contacts and suppress them, and the bar's middle cells lie over 2 mm from either pad.
TriangleMesh bar_fixture()
{
    TriangleMesh wall = make_cube(6.4, 2., 4.);
    wall.translate(-0.9f, -2.3f, 0.f);
    TriangleMesh bar = make_cube(4.6, 0.6, 1.);
    bar.translate(0.f, -0.3f, 2.2f);
    TriangleMesh left = make_cube(0.5, 0.6, 1.2);
    left.translate(-0.6f, -0.3f, 2.f);
    TriangleMesh right = make_cube(0.5, 0.6, 1.2);
    right.translate(4.7f, -0.3f, 2.f);
    wall.merge(bar);
    wall.merge(left);
    wall.merge(right);
    return wall;
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

// Measures one model both ways for the corpus harness (hidden, see TEST_CASE "Miniature contact
// decimation over a corpus"): runs `run` with the mode off and then on, prints one line per mode,
// and writes `<stem>.miniature.csv` beside the corpus when `corpus_dir` is set.
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

// One print processed with the legacy support analysis asked for. init_and_process_print() processes
// on the way in and the request has to reach the Print before process() runs, so this holds the two
// halves apart. The Model outlives the Print because the Print reads the Model's meshes.
struct AnalysisRun
{
    Slic3r::Model model;
    Slic3r::Print print;

    const PrintObject &object() const { return *print.objects().front(); }
    const SupportAnalysis::Report *report() const { return print.objects().front()->support_analysis().get(); }
};

void run_analysis(AnalysisRun &run, TriangleMesh &&shape, const DynamicPrintConfig &config, bool request = true,
                  const std::function<void(ModelVolume &)> &paint = {})
{
    std::vector<TriangleMesh> meshes;
    meshes.emplace_back(std::move(shape));
    init_print(std::move(meshes), run.print, run.model, config);
    if (paint) {
        // init_print has already handed the model to the Print, and Print::apply keeps a copy of its
        // own, so painting reaches the pipeline the way the GUI's painting does: paint the model,
        // then apply it again. The config is rebuilt the way init_print built it, so this second
        // apply differs from the first in the paint and in nothing else.
        paint(*run.model.objects.front()->volumes.front());
        DynamicPrintConfig full = DynamicPrintConfig::full_print_config();
        full.apply(config);
        full.set_key_value("gcode_comments", new ConfigOptionBool(true));
        REQUIRE(run.print.apply(run.model, full) != Print::APPLY_STATUS_UNCHANGED);
    }
    if (request)
        run.print.request_legacy_support_analysis();
    run.print.process();
}

// Paints every facet of `mv` that `pick` selects as a support enforcer, through the encoding 3MF
// loading uses: one hex nibble per original triangle, "4" being an unsplit leaf whose state is
// EnforcerBlockerType::ENFORCER (TriangleSelector::serialize writes a leaf as xxyy, xx the state and
// yy the number of split sides). Degenerate facets are skipped. Returns how many facets were painted.
size_t paint_enforcers(ModelVolume &mv, const std::function<bool(const Vec3f &, const Vec3f &, const Vec3f &)> &pick)
{
    const indexed_triangle_set &its     = mv.mesh().its;
    size_t                      painted = 0;
    mv.supported_facets.reset();
    for (int i = 0; i < int(its.indices.size()); ++ i) {
        const Vec3f &a = its.vertices[its.indices[i](0)];
        const Vec3f &b = its.vertices[its.indices[i](1)];
        const Vec3f &c = its.vertices[its.indices[i](2)];
        if (a == b || a == c || b == c || ! pick(a, b, c))
            continue;
        mv.supported_facets.set_triangle_from_string(i, "4");
        ++ painted;
    }
    mv.supported_facets.shrink_to_fit();
    return painted;
}

// Paints every vertical facet of `mv` as a support enforcer. Verticality is the test slice_mesh_slabs
// applies, a triangle whose projection onto the bed has no signed area, so this paint produces
// vertical enforcer points and nothing else: a vertical facet projects to nothing downwards, and
// detect_overhangs' enforced overhang branch intersects that downward projection with the layer's new
// material. The volume's transform is a translation, which moves the three vertices of a facet alike
// and so cannot turn a vertical facet into a sloped one.
size_t paint_vertical_enforcers(ModelVolume &mv)
{
    return paint_enforcers(mv, [](const Vec3f &a, const Vec3f &b, const Vec3f &c) {
        const double abx = double(b.x()) - double(a.x()), aby = double(b.y()) - double(a.y());
        const double bcx = double(c.x()) - double(b.x()), bcy = double(c.y()) - double(b.y());
        return abx * bcy - aby * bcx == 0.;
    });
}

// Every anchor id the report's regions carry, sorted; the problem's whole seed set.
std::vector<uint64_t> all_anchor_ids(const SupportAnalysis::Report &report)
{
    std::vector<uint64_t> ids;
    for (const SupportAnalysis::RegionCoverage &region : report.coverage)
        ids.insert(ids.end(), region.anchor_ids.begin(), region.anchor_ids.end());
    std::sort(ids.begin(), ids.end());
    return ids;
}

// The critical anchors of one region.
size_t critical_anchors_of(const SupportAnalysis::Report &report, const SupportAnalysis::RegionCoverage &region)
{
    size_t n = 0;
    for (uint64_t id : region.anchor_ids)
        if (std::binary_search(report.critical_anchor_ids.begin(), report.critical_anchor_ids.end(), id))
            ++ n;
    return n;
}

// An axis-aligned rectangle in mm, as an ExPolygon in scaled object coordinates.
ExPolygon rect_mm(double x0, double y0, double x1, double y1)
{
    ExPolygon ex;
    ex.contour.points = { Point(scale_(x0), scale_(y0)), Point(scale_(x1), scale_(y0)),
                          Point(scale_(x1), scale_(y1)), Point(scale_(x0), scale_(y1)) };
    return ex;
}

Point pt_mm(double x, double y) { return Point(scale_(x), scale_(y)); }

// One required region of a written-out problem, taking the id its insertion order gives it.
void add_region(MiniatureSupport::Problem &problem, const ExPolygon &polygon, size_t layer, double contact_z,
                double reach)
{
    MiniatureSupport::RequiredRegion region;
    region.id             = uint64_t(problem.regions.size());
    region.object_layer   = layer;
    region.polygon        = polygon;
    region.contact_z_mm   = contact_z;
    region.legal_reach_mm = reach;
    // Read off the polygon and the problem's width the way `build_required_regions` reads it, so a
    // hand-written sliver is non-printable here too without the leg having to say so.
    region.printable      = MiniatureSupport::holds_an_extrusion(ExPolygons{ polygon }, problem.extrusion_width_mm);
    // A written-out region is its own component, the same value as its id, unless the leg assigns one.
    region.component      = problem.regions.size();
    problem.regions.push_back(region);
}

// One contact seed of a written-out problem, likewise dense from zero in insertion order.
void add_seed(MiniatureSupport::Problem &problem, uint64_t region_id, double x, double y, bool pinned = false,
              bool critical = false, double radius_mm = 0.4)
{
    MiniatureSupport::ContactSeed seed;
    seed.id        = uint64_t(problem.seeds.size());
    seed.region_id = region_id;
    seed.position  = pt_mm(x, y);
    seed.radius_mm = radius_mm;
    seed.pinned    = pinned;
    seed.critical  = critical;
    problem.seeds.push_back(seed);
}

// Which of `region`'s witness cells the contacts at `positions` legally cover between them.
std::vector<bool> covered_union(const MiniatureSupport::RequiredRegion &region, double extrusion_width_mm,
                                const std::vector<Point> &positions)
{
    const MiniatureSupport::Witnesses lattice = MiniatureSupport::witness_cells(region.polygon, extrusion_width_mm);
    std::vector<bool>                 covered(lattice.size(), false);
    for (const Point &position : positions) {
        const std::vector<bool> one =
            MiniatureSupport::covered_by_contact(region.polygon, lattice, position, region.legal_reach_mm);
        for (size_t c = 0; c < covered.size() && c < one.size(); ++ c)
            covered[c] = covered[c] || one[c];
    }
    return covered;
}

// Where each seed of `problem` ended up: its own position when the selection dropped it, and the
// position the selection retained it at when it kept it.
std::vector<Point> retained_positions(const MiniatureSupport::Problem &problem, const MiniatureSupport::Selection &selection)
{
    std::vector<Point> positions;
    positions.reserve(problem.seeds.size());
    for (const MiniatureSupport::ContactSeed &seed : problem.seeds)
        positions.push_back(seed.position);
    for (const MiniatureSupport::ContactSeed &seed : selection.retained)
        if (seed.id < positions.size())
            positions[size_t(seed.id)] = seed.position;
    return positions;
}

// A prepared problem and an emitted record written out over one real required region, so a removal
// group's shape is the test's to arrange rather than a generator's to happen upon. `contacts` are the
// positions contacts are placed at and `footprints` the plan-view column each of them stands on; the
// columns are printed from the plate up to the tip slab that carries the contacts, and two columns
// are one piece of support exactly where their footprints meet.
struct Fabricated
{
    MiniatureSupport::Problem       problem;
    SupportAnalysis::EmittedSupport emitted;

    // The layer printed nearest `z`, for a caller adding material to it.
    SupportAnalysis::EmittedLayer &near(double z)
    {
        SupportAnalysis::EmittedLayer *best = &emitted.layers.front();
        for (SupportAnalysis::EmittedLayer &layer : emitted.layers)
            if (std::abs(layer.print_z - z) < std::abs(best->print_z - z))
                best = &layer;
        return *best;
    }
};

Fabricated fabricate(const SupportAnalysis::CoverageKey &key, const std::vector<Point> &contacts,
                     const std::vector<ExPolygon> &footprints)
{
    Fabricated out;
    out.problem.extrusion_width_mm = key.extrusion_width_mm;
    add_region(out.problem, key.witnesses.front(), key.region_layers.front(), key.region_contact_z.front(),
               MiniatureSupport::legal_reach(5., 10.));
    for (const Point &position : contacts) {
        MiniatureSupport::ContactSeed seed;
        seed.id        = uint64_t(out.problem.seeds.size());
        seed.region_id = 0;
        seed.position  = position;
        seed.radius_mm = 0.4;
        out.problem.seeds.push_back(seed);
    }

    const double slab  = 0.2;
    out.emitted.top_gap_mm          = slab;
    out.emitted.max_layer_height_mm = slab;
    const double tip_z = key.region_contact_z.front() - slab;
    const size_t count = size_t(std::max<long long>(1, std::llround(tip_z / slab)));
    Polygons all;
    for (const ExPolygon &footprint : footprints)
        append(all, to_polygons(footprint));
    const ExPolygons ground = union_ex(all);
    for (size_t i = 0; i < count; ++ i) {
        SupportAnalysis::EmittedLayer layer;
        layer.support_layer_index = i;
        layer.print_z             = tip_z - slab * double(count - 1 - i);
        layer.bottom_z            = layer.print_z - slab;
        layer.emitted             = ground;
        layer.emitted_available   = true;
        out.emitted.layers.push_back(layer);
    }
    // The tip each contact stands on, filed the way the router files one: its own attributed area,
    // carrying that contact alone.
    for (size_t i = 0; i < footprints.size() && i < contacts.size(); ++ i) {
        SupportAnalysis::AttributedArea area;
        area.area            = footprints[i];
        area.source_ids      = { uint64_t(i) };
        area.termination     = SupportAnalysis::Termination::Roof;
        area.min_diameter_mm = 1.;
        out.emitted.layers.back().areas.push_back(area);
    }
    return out;
}

// A `side` mm square of printed footprint centred on a contact.
ExPolygon column_at(const Point &centre, double side)
{
    const double x = unscale<double>(centre.x()), y = unscale<double>(centre.y());
    return rect_mm(x - 0.5 * side, y - 0.5 * side, x + 0.5 * side, y + 0.5 * side);
}

// A roof gap the router draws and never extrudes, over `where`: it carries its contacts like any
// other attributed area and no extrusion ever covers it.
SupportAnalysis::AttributedArea drawn_gap(const ExPolygon &where, std::vector<uint64_t> sources)
{
    SupportAnalysis::AttributedArea area;
    area.area        = where;
    area.source_ids  = std::move(sources);
    area.termination = SupportAnalysis::Termination::GapAbove;
    area.virtual_gap = true;
    return area;
}

} // namespace

TEST_CASE("Two contacts of one region whose tips print a layer apart inside the planned gap both anchor it", "[MiniatureContacts]")
{
    // One real required region off a processed print, and two written-out columns under it. The
    // second column's tip is filed one slab under the first's, which is still inside the region's
    // top-gap interval - the interval is one slab deep - the way a tip clipped against the model's xy
    // clearance prints its first material a layer under its neighbour's.
    AnalysisRun run;
    run_analysis(run, lip_fixture(),
                 fixture_config({ { "support_style", "tree_slim" }, { "support_top_z_distance", "0.2" },
                                  { "support_miniature_contacts", "1" }, { "support_contact_min_distance", "0" } }),
                 false);
    REQUIRE(run.report() != nullptr);
    const SupportAnalysis::CoverageKey key = run.report()->key;
    REQUIRE(key.witnesses.size() == 1);

    const BoundingBox band = get_extents(key.witnesses.front());
    const Point       left(band.min.x() + (band.max.x() - band.min.x()) / 4, band.center().y());
    const Point       right(band.min.x() + 3 * (band.max.x() - band.min.x()) / 4, band.center().y());
    const std::vector<ExPolygon> columns{ column_at(left, 1.), column_at(right, 1.) };

    // The tip area of the right column moved `down` slabs: the printed footprint stays, the record of
    // whose material it is moves with the tip.
    const auto staggered = [&](size_t down) {
        Fabricated out = fabricate(key, { left, right }, columns);
        REQUIRE(out.emitted.layers.size() > down);
        SupportAnalysis::EmittedLayer &tip = out.emitted.layers.back();
        REQUIRE(tip.areas.size() == 2);
        out.emitted.layers[out.emitted.layers.size() - 1 - down].areas.push_back(tip.areas.back());
        tip.areas.pop_back();
        return out;
    };

    const Fabricated              one   = staggered(1);
    const SupportAnalysis::Report inside = SupportAnalysis::measure(run.object(), one.problem, one.emitted);
    REQUIRE(inside.coverage_available);
    REQUIRE(inside.coverage.front().anchored);
    REQUIRE(inside.missing_anchor_ids.empty());
    REQUIRE_THAT(inside.coverage.front().tip_print_z, Catch::Matchers::WithinAbs(one.emitted.layers.back().print_z, 1e-9));

    // Two slabs down is under the interval: that tip anchors nothing, whatever printed there.
    const Fabricated              two    = staggered(2);
    const SupportAnalysis::Report below  = SupportAnalysis::measure(run.object(), two.problem, two.emitted);
    REQUIRE(below.coverage.front().anchored);
    REQUIRE(below.missing_anchor_ids == std::vector<uint64_t>{ 1 });
}

TEST_CASE("A region with no contact of its own is covered and has a path when a lower neighbour within the distance printed",
          "[MiniatureContacts]")
{
    // One real required region off a processed print with a written-out column under it, and a second
    // region of the same overhang component one layer and 0.2 mm higher, carrying no contact at all.
    AnalysisRun run;
    run_analysis(run, lip_fixture(),
                 fixture_config({ { "support_style", "tree_slim" }, { "support_top_z_distance", "0.2" },
                                  { "support_miniature_contacts", "1" }, { "support_contact_min_distance", "0" } }),
                 false);
    REQUIRE(run.report() != nullptr);
    const SupportAnalysis::CoverageKey key = run.report()->key;
    REQUIRE(key.witnesses.size() == 1);

    const BoundingBox band = get_extents(key.witnesses.front());
    const Point       left(band.min.x() + (band.max.x() - band.min.x()) / 4, band.center().y());

    Fabricated out = fabricate(key, { left }, { column_at(left, 1.) });
    add_region(out.problem, column_at(left, 0.6), key.region_layers.front() + 1, key.region_contact_z.front() + 0.2,
               MiniatureSupport::legal_reach(5., 10.));
    out.problem.regions[1].component      = out.problem.regions[0].component;
    out.problem.contact_min_distance_mm   = 1.;
    const SupportAnalysis::Report carried = SupportAnalysis::measure(run.object(), out.problem, out.emitted);
    REQUIRE(carried.coverage.size() == 2);
    REQUIRE(carried.coverage[0].anchored);
    REQUIRE(carried.coverage[0].emitted_path);
    REQUIRE_FALSE(carried.coverage[1].anchored);
    REQUIRE(carried.coverage[1].emitted_path);
    REQUIRE(carried.coverage[1].cell_count() ==
            MiniatureSupport::witness_cells(out.problem.regions[1].polygon, key.extrusion_width_mm).size());
    REQUIRE(carried.coverage[1].covered_count() == carried.coverage[1].cell_count());
    REQUIRE(carried.missing_anchor_ids.empty());
    REQUIRE(carried.stability.available);
    REQUIRE(carried.stability.unrooted_groups == 0);

    // The carried path is a coverage fact and the unrooted count a stability one: giving region 1 a source of
    // its own whose only attributed area is a roof gap the router draws and never extrudes leaves it routed
    // and unreached, which `measure_stability` counts, while the neighbour below still carries its cells.
    Fabricated open = out;
    add_seed(open.problem, 1, unscale<double>(left.x()), unscale<double>(left.y()));
    open.emitted.layers.back().areas.push_back(drawn_gap(column_at(left, 0.6), { 1 }));
    const SupportAnalysis::Report unrooted = SupportAnalysis::measure(run.object(), open.problem, open.emitted);
    REQUIRE(unrooted.stability.available);
    REQUIRE(unrooted.stability.unrooted_groups == 1);
    REQUIRE(unrooted.coverage[1].emitted_path);
    REQUIRE(unrooted.missing_anchor_ids == std::vector<uint64_t>{ 1 });

    // The control: at a distance of 0 nothing is carried, so the same lattice is sized and empty and the
    // region has no path of its own.
    out.problem.contact_min_distance_mm    = 0.;
    const SupportAnalysis::Report uncarried = SupportAnalysis::measure(run.object(), out.problem, out.emitted);
    REQUIRE(uncarried.coverage[1].cell_count() == carried.coverage[1].cell_count());
    REQUIRE(uncarried.coverage[1].covered_count() == 0);
    REQUIRE_FALSE(uncarried.coverage[1].emitted_path);
}

TEST_CASE("A removal group is what printed support connects, and a drawn roof gap divides nothing", "[MiniatureContacts]")
{
    // One real required region off a processed print: the band under the lip, on the object layer and
    // at the contact Z the detector filed it at. What stands under it is written out below, so the
    // grouping is measured against material whose connectivity is known rather than generated.
    AnalysisRun run;
    run_analysis(run, lip_fixture(),
                 fixture_config({ { "support_style", "tree_slim" }, { "support_top_z_distance", "0.2" },
                                  { "support_miniature_contacts", "1" }, { "support_contact_min_distance", "0" } }),
                 false);
    REQUIRE(run.report() != nullptr);
    const SupportAnalysis::CoverageKey key = run.report()->key;
    REQUIRE(key.witnesses.size() == 1);

    const BoundingBox band = get_extents(key.witnesses.front());
    const Point       left(band.min.x() + (band.max.x() - band.min.x()) / 4, band.center().y());
    const Point       right(band.min.x() + 3 * (band.max.x() - band.min.x()) / 4, band.center().y());
    const std::vector<ExPolygon> columns{ column_at(left, 1.), column_at(right, 1.) };
    Fabricated                   apart = fabricate(key, { left, right }, columns);

    const SupportAnalysis::Report separate = SupportAnalysis::measure(run.object(), apart.problem, apart.emitted);
    REQUIRE(separate.damage.available);
    REQUIRE_FALSE(separate.has_reason(SupportAnalysis::Reason::DamageUnavailable));
    REQUIRE(separate.damage.total_group_risk > 0.);
    // Two columns the material never joins are two removal groups: the worst of them carries less
    // than the two of them together.
    REQUIRE(separate.damage.max_group_risk < separate.damage.total_group_risk - 1e-9);

    SECTION("printed material that joins two columns makes one group of them")
    {
        // A bar across the gap at a third of the way up, touching no attributed area: the contacts and
        // the material each of them stands on are what they were, and the only thing that changed is
        // that one piece of support now holds both.
        Fabricated joined = apart;
        SupportAnalysis::EmittedLayer &bridge = joined.near(0.33 * key.region_contact_z.front());
        const BoundingBox              left_box = get_extents(columns.front()), right_box = get_extents(columns.back());
        bridge.emitted = union_ex(ExPolygons{ bridge.emitted.front(),
                                              rect_mm(unscale<double>(left_box.center().x()),
                                                      unscale<double>(left_box.min.y()),
                                                      unscale<double>(right_box.center().x()),
                                                      unscale<double>(left_box.max.y())),
                                              bridge.emitted.back() });
        const SupportAnalysis::Report one = SupportAnalysis::measure(run.object(), joined.problem, joined.emitted);
        REQUIRE(one.damage.available);
        // One group now, so the worst of them is all of them, and it carries what both of them
        // carried and the longer run through the bar they are joined by on top of that.
        CHECK_THAT(one.damage.max_group_risk, WithinRel(one.damage.total_group_risk, 1e-9));
        CHECK(one.damage.total_group_risk >= separate.damage.total_group_risk - 1e-9);
        CHECK(one.damage.max_group_risk > separate.damage.max_group_risk + 1e-9);
    }

    SECTION("a drawn roof gap neither joins two groups nor divides one")
    {
        // The gap the router draws over a tip is never extruded, so it says nothing about what is
        // connected to what: one laid across both columns joins nothing, and one laid over a column
        // divides nothing.
        Fabricated drawn = apart;
        const BoundingBox left_box = get_extents(columns.front()), right_box = get_extents(columns.back());
        drawn.emitted.layers.back().areas.push_back(
            drawn_gap(rect_mm(unscale<double>(left_box.min.x()), unscale<double>(left_box.min.y()),
                              unscale<double>(right_box.max.x()), unscale<double>(right_box.max.y())),
                      { 0, 1 }));
        drawn.near(0.33 * key.region_contact_z.front()).areas.push_back(drawn_gap(columns.front(), { 0 }));

        const SupportAnalysis::Report with_gaps = SupportAnalysis::measure(run.object(), drawn.problem, drawn.emitted);
        REQUIRE(with_gaps.damage.available);
        CHECK(with_gaps.damage.unknown_contacts == separate.damage.unknown_contacts);
        CHECK(with_gaps.damage.inaccessible_groups == separate.damage.inaccessible_groups);
        CHECK_THAT(with_gaps.damage.max_group_risk, WithinRel(separate.damage.max_group_risk, 1e-9));
        CHECK_THAT(with_gaps.damage.total_group_risk, WithinRel(separate.damage.total_group_risk, 1e-9));
    }
}

TEST_CASE("Group risk weighs contact area by model risk, lengthens with the run to the root and counts blocked access apart", "[MiniatureContacts]")
{
    AnalysisRun run;
    run_analysis(run, lip_fixture(),
                 fixture_config({ { "support_style", "tree_slim" }, { "support_top_z_distance", "0.2" },
                                  { "support_miniature_contacts", "1" }, { "support_contact_min_distance", "0" } }),
                 false);
    REQUIRE(run.report() != nullptr);
    const SupportAnalysis::CoverageKey key = run.report()->key;
    REQUIRE(key.witnesses.size() == 1);
    const BoundingBox band = get_extents(key.witnesses.front());
    const Point       left(band.min.x() + (band.max.x() - band.min.x()) / 4, band.center().y());
    const Point       right(band.min.x() + 3 * (band.max.x() - band.min.x()) / 4, band.center().y());
    const std::vector<ExPolygon> columns{ column_at(left, 1.), column_at(right, 1.) };

    SECTION("the same contacts on the same material weigh more the further they stand from their root")
    {
        const Fabricated straight = fabricate(key, { left, right }, columns);
        const SupportAnalysis::Report upright =
            SupportAnalysis::measure(run.object(), straight.problem, straight.emitted);
        REQUIRE(upright.damage.available);
        REQUIRE(upright.damage.total_group_risk > 0.);

        // The same two contacts standing on the same tips, with the left column leaning 0.3 mm per
        // layer as it goes down: nothing about the contacts, the material they touch or the ground
        // the column roots on changed, and the run between the two is half again as long.
        Fabricated leaning = straight;
        const size_t count = leaning.emitted.layers.size();
        for (size_t i = 0; i + 1 < count; ++ i) {
            ExPolygon shifted = columns.front();
            shifted.translate(0, coord_t(scale_(0.3 * double(count - 1 - i))));
            Polygons both;
            append(both, to_polygons(shifted));
            append(both, to_polygons(columns.back()));
            leaning.emitted.layers[i].emitted = union_ex(both);
        }
        const SupportAnalysis::Report leant = SupportAnalysis::measure(run.object(), leaning.problem, leaning.emitted);
        REQUIRE(leant.damage.available);
        CHECK(leant.damage.total_group_risk > upright.damage.total_group_risk + 1e-9);
        CHECK(leant.damage.max_group_risk > upright.damage.max_group_risk + 1e-9);
    }

    SECTION("material two contacts share is weighed once, at the worse of the two estimates")
    {
        // One column carrying one printed tip. Filed for the left contact alone, for the right one
        // alone, and for both: the same square millimetre of support in all three.
        const std::vector<ExPolygon> one{ column_at(left, 1.) };
        const std::vector<Point>     pair{ left, Point(left.x() + coord_t(scale_(0.3)), left.y()) };
        const auto weigh = [&](std::vector<uint64_t> sources) {
            Fabricated fabricated = fabricate(key, pair, one);
            fabricated.emitted.layers.back().areas.front().source_ids = std::move(sources);
            return SupportAnalysis::measure(run.object(), fabricated.problem, fabricated.emitted).damage;
        };
        const SupportAnalysis::Damage first = weigh({ 0 }), second = weigh({ 1 }), shared = weigh({ 0, 1 });
        REQUIRE(first.available);
        REQUIRE(first.total_group_risk > 0.);
        REQUIRE(second.total_group_risk > 0.);
        // Once, at the worse estimate: never the two of them added up, which is what counting the
        // area for each contact carrying it would come to.
        CHECK(shared.total_group_risk >= std::max(first.total_group_risk, second.total_group_risk) - 1e-9);
        CHECK(shared.total_group_risk < first.total_group_risk + second.total_group_risk - 1e-9);
    }

    SECTION("a group nothing can reach is counted as one, and still weighs what it weighs")
    {
        // The closed box: its ceiling's underside is an overhang inside a sealed void, so a contact
        // placed there has no way out at all, while the lip's band is open on every side.
        AnalysisRun sealed;
        run_analysis(sealed, box_fixture(),
                     fixture_config({ { "support_style", "tree_slim" }, { "support_top_z_distance", "0.2" },
                                      { "support_miniature_contacts", "1" }, { "support_contact_min_distance", "0" } }),
                     false);
        REQUIRE(sealed.report() != nullptr);
        const SupportAnalysis::CoverageKey inside = sealed.report()->key;
        REQUIRE(inside.witnesses.size() == 1);
        const Point       ceiling = get_extents(inside.witnesses.front()).center();
        const Fabricated  caged   = fabricate(inside, { ceiling }, { column_at(ceiling, 1.) });
        const SupportAnalysis::Report unreachable = SupportAnalysis::measure(sealed.object(), caged.problem, caged.emitted);
        REQUIRE(unreachable.damage.available);
        CHECK(unreachable.damage.inaccessible_groups == 1);
        // The count is recorded beside the weight rather than in place of it.
        CHECK(unreachable.damage.total_group_risk > 0.);

        const Fabricated              open = fabricate(key, { left }, { column_at(left, 1.) });
        const SupportAnalysis::Report reachable = SupportAnalysis::measure(run.object(), open.problem, open.emitted);
        REQUIRE(reachable.damage.available);
        CHECK(reachable.damage.inaccessible_groups == 0);
    }

    SECTION("a contact under a raft is probed against the model where the model prints")
    {
        // The raft lifts every contact by the object's first print_z: a probe started against the
        // model where it would stand without the raft starts inside the lip, which is open on every side.
        AnalysisRun rafted;
        run_analysis(rafted, lip_fixture(),
                     fixture_config({ { "support_style", "tree_slim" }, { "support_top_z_distance", "0.2" },
                                      { "support_miniature_contacts", "1" }, { "support_contact_min_distance", "0" },
                                      { "raft_layers", "3" } }),
                     false);
        REQUIRE(rafted.report() != nullptr);
        const SupportAnalysis::CoverageKey raft_key = rafted.report()->key;
        REQUIRE(raft_key.witnesses.size() == 1);
        const Point probe = get_extents(raft_key.witnesses.front()).center();
        Fabricated  open  = fabricate(raft_key, { probe }, { column_at(probe, 1.) });
        // fabricate prints the column from the plate up; under a raft it stands on the raft, so the
        // layers below the object's first print_z go, as the generator never prints them.
        const double object_bottom_z = rafted.object().slicing_parameters().object_print_z_min;
        std::vector<SupportAnalysis::EmittedLayer> &layers = open.emitted.layers;
        layers.erase(std::remove_if(layers.begin(), layers.end(),
                                    [&](const SupportAnalysis::EmittedLayer &layer) { return layer.bottom_z < object_bottom_z - 1e-6; }),
                     layers.end());
        REQUIRE(! layers.empty());
        const SupportAnalysis::Report report = SupportAnalysis::measure(rafted.object(), open.problem, open.emitted);
        REQUIRE(report.damage.available);
        CHECK(report.damage.inaccessible_groups == 0);
        CHECK(report.damage.unknown_contacts == 0);
    }
}

TEST_CASE("A measurement that requires nothing is complete and one that printed nothing is unresolved", "[MiniatureContacts]")
{
    AnalysisRun run;
    run_analysis(run, lip_fixture(),
                 fixture_config({ { "support_style", "tree_slim" }, { "support_top_z_distance", "0.2" },
                                  { "support_miniature_contacts", "1" }, { "support_contact_min_distance", "0" } }),
                 false);
    REQUIRE(run.report() != nullptr);

    // Nothing required and nothing emitted: there is nothing to cover, so coverage is complete by
    // construction, and stability reads the object standing on the plate by itself.
    const SupportAnalysis::Report nothing = SupportAnalysis::measure(run.object(), MiniatureSupport::Problem(), SupportAnalysis::EmittedSupport());
    CHECK(nothing.status == SupportAnalysis::Report::Status::Complete);
    CHECK(nothing.coverage_available);
    CHECK(nothing.stability.available);
    CHECK(nothing.damage.available);
    CHECK(nothing.has_reason(SupportAnalysis::Reason::NoProblem));
    CHECK_FALSE(nothing.has_reason(SupportAnalysis::Reason::EmittedMaterialMissing));

    // A region the lip requires, with a contact placed on it and nothing printed: the region is still
    // owed support, so its coverage stays unresolved rather than reading as measured.
    const SupportAnalysis::CoverageKey key = run.report()->key;
    REQUIRE(key.witnesses.size() == 1);
    const Point                   centre     = get_extents(key.witnesses.front()).center();
    const Fabricated              fabricated = fabricate(key, { centre }, { column_at(centre, 1.) });
    const SupportAnalysis::Report report     = SupportAnalysis::measure(run.object(), fabricated.problem, SupportAnalysis::EmittedSupport());
    CHECK(report.status == SupportAnalysis::Report::Status::UnresolvedCoverage);
    CHECK_FALSE(report.coverage_available);
    CHECK(report.has_reason(SupportAnalysis::Reason::EmittedMaterialMissing));
}

TEST_CASE("Miniature contacts leave a stock slice untouched when off", "[MiniatureContacts]")
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
}

TEST_CASE("Explicitly disabled miniature contacts preserve stock legacy output and shared reprocessing", "[MiniatureContacts]")
{
    // Seven runs a leg: the legacy tree generator's output is not run-to-run reproducible (AGENTS.md
    // "Testing"), so its support measures are compared as envelopes rather than as numbers, and seven
    // samples leave each leg a median one outlier cannot move.
    constexpr int repeats = 7;

    // The three legacy tree styles the mode acts on, and organic, which it must not reach: the mode
    // is a legacy-tree feature and the organic generator runs the same detect_overhangs().
    for (const char *style : { "tree_slim", "tree_strong", "tree_hybrid", "organic" }) {
        INFO("support_style " << style);
        // support_top_z_distance explicitly, because at 0 the tips print onto the object and the roof
        // areas the rest of this file measures come back empty. 0.2 is PrintConfig.cpp's default.
        const DynamicPrintConfig stock = fixture_config({ { "support_style", style }, { "support_top_z_distance", "0.2" } });
        DynamicPrintConfig       off   = stock;
        off.set_deserialize_strict({ { "support_miniature_contacts", "0" } });
        // set_deserialize_strict drops a key print_config_def does not carry instead of throwing, so
        // reading the value back is what proves this leg is more than a copy of the stock one.
        REQUIRE(off.option("support_miniature_contacts") != nullptr);
        REQUIRE(off.opt_bool("support_miniature_contacts") == false);

        DynamicPrintConfig                       resolved;
        std::map<int, std::pair<size_t, double>> roles;
        ContactClusters                          anchors;
        Annotations                              marks;
        std::vector<double>                      volume[2], regions[2];

        for (int run = 0; run < 2 * repeats; ++ run) {
            const int leg = run < repeats ? 0 : 1; // 0 stock, 1 the mode spelled out as off
            INFO("leg " << (leg == 0 ? "stock" : "off") << " run " << run % repeats);

            Slic3r::Print print;
            init_and_process_print({ fin_fixture() }, print, leg == 0 ? stock : off);
            const PrintObject &po = *print.objects().front();

            const std::map<int, std::pair<size_t, double>> run_roles   = role_totals(po);
            // The required anchors: a roof gap area is the room the generator leaves between a
            // contact tip and the model, so the set of them is the set of places this print has to
            // anchor support to, and contact_clusters() already reads it off SupportLayer.
            const ContactClusters                          run_anchors = contact_clusters(po);
            const SupportMetrics                           run_support = support_metrics(po);
            const Annotations                              run_marks   = annotations(po);
            // Whatever this pass generated, it belongs to this object and its annotations are whole.
            require_valid_support_layers(po, po);

            if (run == 0) {
                resolved = print.full_print_config();
                roles    = run_roles;
                anchors  = run_anchors;
                marks    = run_marks;
                REQUIRE(! roles.empty());
                // The organic generator files no roof gap areas at all, so it anchors nothing here
                // and its two legs meet on the empty set and on the support envelopes below.
                if (std::string(style) != "organic")
                    REQUIRE(anchors.count() > 0);
            } else {
                // support_miniature_contacts defaults to false, so spelling it out has to leave the
                // print on the same resolved configuration, key for key.
                REQUIRE(print.full_print_config() == resolved);
                REQUIRE(same_roles(run_roles, roles));
                REQUIRE(same_clusters(run_anchors, anchors));
                REQUIRE(same_annotations(run_marks, marks));
            }
            volume[leg].push_back(run_support.volume_mm3);
            regions[leg].push_back(run_support.footprint_regions);
        }

        const Envelope stock_volume  = envelope_of(volume[0]),  off_volume  = envelope_of(volume[1]);
        const Envelope stock_regions = envelope_of(regions[0]), off_regions = envelope_of(regions[1]);
        // What this host measured, for the record. The comparison below is scaled against these, so
        // they are a reading of the machine the suite ran on, never a range to carry to another one.
        std::cout << std::setprecision(9) << "style " << style
                  << " required_anchors=" << anchors.count()
                  << " sharp_tails=" << marks.sharp_tails << " cantilevers=" << marks.cantilevers
                  << " support_volume_mm3 stock=[" << stock_volume.min << "," << stock_volume.max << "] median=" << stock_volume.median
                  << " off=[" << off_volume.min << "," << off_volume.max << "] median=" << off_volume.median
                  << " footprint_regions stock=[" << stock_regions.min << "," << stock_regions.max << "] median=" << stock_regions.median
                  << " off=[" << off_regions.min << "," << off_regions.max << "] median=" << off_regions.median
                  << std::endl;
        require_same_envelope("support volume mm3", stock_volume, off_volume);
        require_same_envelope("support footprint regions", stock_regions, off_regions);
    }

    // The rest of the journey, under tree_slim, the style the mode acts on: an object sharing another
    // object's slice, a support setting changing under it, and an instance going away.
    struct Stage
    {
        ContactClusters owner, copy, after_setting, after_removal;
        Annotations     owner_marks, after_removal_marks;
    };
    const auto journey = [](const DynamicPrintConfig &config_in) {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.apply(config_in);

        // Two objects off one mesh. Print::process() shares a slice only between objects whose
        // volumes hold the same mesh pointer, which ModelVolume's copy keeps, and whose object
        // configs match, so the source carries the extruder Model::add_object gives the clone.
        Slic3r::Model model;
        ModelObject  *first = model.add_object();
        first->name         = "shared_object.stl";
        first->add_volume(fin_fixture());
        first->config.set_key_value("extruder", new ConfigOptionInt(1));
        first->add_instance()->set_offset(Vec3d(100., 100., 0.));
        ModelObject *clone = model.add_object(*first);
        clone->instances.front()->set_offset(Vec3d(140., 100., 0.));
        for (ModelObject *mo : model.objects)
            mo->ensure_on_bed();

        Slic3r::Print print;
        print.set_status_silent();
        print.apply(model, config);
        print.process();
        REQUIRE(print.objects().size() == 2);
        REQUIRE(print.objects()[1]->get_shared_object() == print.objects()[0]);

        Stage stage;
        stage.owner       = contact_clusters(*print.objects()[0]);
        stage.copy        = contact_clusters(*print.objects()[1]);
        stage.owner_marks = annotations(*print.objects()[0]);
        REQUIRE(stage.owner.count() > 0);
        // The pass belongs to the owner, and the shared object reads the owner's layers rather than
        // layers of its own: every support layer it lists is one of the owner's, in print_z order.
        require_valid_support_layers(*print.objects()[0], *print.objects()[0]);
        require_valid_support_layers(*print.objects()[1], *print.objects()[0]);
        REQUIRE(print.objects()[1]->support_layers().size() == print.objects()[0]->support_layers().size());
        REQUIRE(print.objects()[0]->support_layers().size() > 0);
        // It reads the owner's generator cache too, rather than one of its own or none at all.
        REQUIRE(print.objects()[0]->tree_support_preview_cache() != nullptr);
        REQUIRE(print.objects()[1]->tree_support_preview_cache() == print.objects()[0]->tree_support_preview_cache());

        // A support setting changes: applying it has to invalidate the support step on both objects,
        // so the reprocess below regenerates rather than serving what is already there.
        DynamicPrintConfig steeper = config;
        steeper.set_deserialize_strict({ { "support_threshold_angle", "20" } });
        print.apply(model, steeper);
        for (const PrintObject *po : print.objects()) {
            REQUIRE_FALSE(po->is_step_done(posSupportMaterial));
            // Neither the owner nor the object reading its layers is left holding a layer the owner
            // is about to delete, or a cache from a pass that no longer applies.
            REQUIRE(po->support_layers().empty());
            REQUIRE(po->tree_support_preview_cache() == nullptr);
        }
        print.process();
        for (const PrintObject *po : print.objects())
            REQUIRE(po->is_step_done(posSupportMaterial));
        stage.after_setting = contact_clusters(*print.objects()[0]);

        // The clone's only instance goes away, the original settings come back, and the print is
        // processed once more: one object is left, holding the anchors the first pass measured.
        clone->delete_last_instance();
        print.apply(model, config);
        print.process();
        REQUIRE(print.objects().size() == 1);
        stage.after_removal       = contact_clusters(*print.objects()[0]);
        stage.after_removal_marks = annotations(*print.objects()[0]);
        require_valid_support_layers(*print.objects()[0], *print.objects()[0]);
        REQUIRE(print.objects()[0]->support_layers().size() > 0);
        return stage;
    };

    const DynamicPrintConfig stock_slim = fixture_config({ { "support_style", "tree_slim" }, { "support_top_z_distance", "0.2" } });
    DynamicPrintConfig       off_slim   = stock_slim;
    off_slim.set_deserialize_strict({ { "support_miniature_contacts", "0" } });

    const Stage stock_journey = journey(stock_slim);
    const Stage off_journey   = journey(off_slim);

    // The shared object copies its owner's layers, so it reports the owner's anchors.
    REQUIRE(same_clusters(stock_journey.copy, stock_journey.owner));
    REQUIRE(same_clusters(off_journey.copy, off_journey.owner));
    // The changed support setting reached the generator, not just the step flags: a shallower
    // threshold leaves a different set of places to anchor to.
    REQUIRE_FALSE(same_clusters(stock_journey.after_setting, stock_journey.owner));
    // Restoring the settings restores the anchors the first pass measured, through the reprocess and
    // the instance removal both.
    REQUIRE(same_clusters(stock_journey.after_removal, stock_journey.owner));
    REQUIRE(same_clusters(off_journey.after_removal, off_journey.owner));
    // The annotations detection wrote come back with them, on the object that owns the pass.
    REQUIRE(same_annotations(stock_journey.after_removal_marks, stock_journey.owner_marks));
    REQUIRE(same_annotations(off_journey.after_removal_marks, off_journey.owner_marks));
    REQUIRE(same_annotations(off_journey.owner_marks, stock_journey.owner_marks));
    // And the explicitly disabled print walks that journey to the stock print's anchors at every
    // stage, the changed setting included.
    REQUIRE(same_clusters(off_journey.owner, stock_journey.owner));
    REQUIRE(same_clusters(off_journey.after_setting, stock_journey.after_setting));
    REQUIRE(same_clusters(off_journey.after_removal, stock_journey.after_removal));

    // What one generated pass leaves the object holding. peg_fixture()'s peg floats, so detection
    // files sharp tails on the object's layers, and a raft puts support layers under the object's own
    // first layer: the layers, the annotations, the generator's cache and the raft count are one
    // result, and the object has to report all of it after the pass that produced it.
    const DynamicPrintConfig raft_slim = fixture_config({ { "support_style", "tree_slim" },
                                                          { "support_top_z_distance", "0.2" },
                                                          { "raft_layers", "3" } });
    Slic3r::Print raft_print;
    Slic3r::Model raft_model;
    init_print({ peg_fixture() }, raft_print, raft_model, raft_slim);
    raft_print.process();
    const PrintObject &raft_object = *raft_print.objects().front();

    const Annotations raft_marks = annotations(raft_object);
    REQUIRE(raft_marks.sharp_tails > 0);
    REQUIRE(raft_marks.sharp_tails == raft_marks.sharp_tails_height);
    require_valid_support_layers(raft_object, raft_object);
    REQUIRE(raft_object.support_layers().size() > 0);
    REQUIRE(raft_object.tree_support_preview_cache() != nullptr);

    // Every support layer below the object's own first layer is raft, and the count the object
    // reports is exactly those.
    const double object_bottom_z = raft_object.slicing_parameters().object_print_z_min;
    size_t       raft_layers     = 0;
    for (const SupportLayer *sl : raft_object.support_layers())
        if (sl->print_z < object_bottom_z + EPSILON)
            ++ raft_layers;
    REQUIRE(raft_layers >= 3);
    REQUIRE(raft_object.support_raft_layers() == raft_layers);

    // A second pass over the same object starts from a fresh node pool, because clear_support_layers()
    // reset the cache when the support step was invalidated: the pool holds the contacts of this pass
    // alone. Legacy tree output
    // is not reproducible run to run (AGENTS.md "Testing"), so the pool is held to a bound rather than
    // to a number; a pass that inherited the previous pool would hold both passes at once.
    const ContactClusters first_anchors = contact_clusters(raft_object);
    const size_t          first_nodes   = raft_object.tree_support_preview_cache()->contact_nodes.size();
    REQUIRE(first_nodes > 0);

    // A support setting changes and changes back: the support step stays invalidated either way, so
    // the reprocess below generates a second pass under the settings the first one ran under.
    DynamicPrintConfig steeper_raft = raft_slim;
    steeper_raft.set_deserialize_strict({ { "support_threshold_angle", "20" } });
    raft_print.apply(raft_model, steeper_raft);
    // The pass goes with the step it belongs to: once the support step is invalidated the object
    // reports no support layer, no generator cache, no raft and none of the pass's annotations, so
    // nothing the next pass produces can be confused with what the last one left.
    REQUIRE(raft_object.support_layers().empty());
    REQUIRE(raft_object.tree_support_preview_cache() == nullptr);
    REQUIRE(raft_object.support_raft_layers() == 0);
    REQUIRE(annotations(raft_object).sharp_tails == 0);
    raft_print.apply(raft_model, raft_slim);
    REQUIRE_FALSE(raft_object.is_step_done(posSupportMaterial));
    raft_print.process();
    REQUIRE(raft_object.is_step_done(posSupportMaterial));

    const size_t second_nodes = raft_object.tree_support_preview_cache()->contact_nodes.size();
    std::cout << "raft pass contact nodes first=" << first_nodes << " second=" << second_nodes << std::endl;
    REQUIRE(2 * second_nodes < 3 * first_nodes);
    // And the second pass reproduces the first one's result, down to the raft it laid.
    REQUIRE(same_clusters(contact_clusters(raft_object), first_anchors));
    REQUIRE(same_annotations(annotations(raft_object), raft_marks));
    REQUIRE(raft_object.support_raft_layers() == raft_layers);
    require_valid_support_layers(raft_object, raft_object);
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

    // Leg 6, the band sliced and printed under a requested contact distance of 5 mm, twice the legal
    // reach the active 5 mm branch distance and 10 mm bridge limit resolve to. A distance-only pass
    // would have collapsed the 25 mm lip onto contacts 5 mm apart and left most of it unsupported;
    // asking for more space than a contact can anchor is a preference, so nothing the band needs may
    // go with it.
    const auto lip_config = [](const char *distance) {
        return fixture_config({ { "support_style", "tree_slim" },
                                { "support_top_z_distance", "0.2" },
                                { "support_miniature_contacts", "1" },
                                { "support_contact_min_distance", distance } });
    };
    AnalysisRun untouched, spaced;
    run_analysis(untouched, lip_fixture(), lip_config("0"), false);
    run_analysis(spaced,    lip_fixture(), lip_config("5"), false);
    REQUIRE(untouched.report() != nullptr);
    REQUIRE(spaced.report() != nullptr);
    // Both passes ran against one prepared problem, so the two name the same regions.
    REQUIRE(untouched.report()->key == spaced.report()->key);

    // And the geometry says the same thing. Coverage is measured off the material actually laid, so
    // the widest run of covered witness cells is how far along the fixture printed support reaches:
    // it spans the 25 mm lip with no thinning at all, and asking for 5 mm of space does not shorten it.
    const auto covered_span = [](const SupportAnalysis::Report &report) {
        double widest = 0.;
        for (const SupportAnalysis::RegionCoverage &region : report.coverage) {
            const MiniatureSupport::Witnesses lattice =
                MiniatureSupport::witness_cells(report.key.witnesses[region.region_id], report.key.extrusion_width_mm);
            double lo = 0., hi = 0.;
            bool   any = false;
            for (size_t c = 0; c < region.covered.size() && c < lattice.size(); ++ c)
                if (region.covered[c]) {
                    const double left  = unscale<double>(lattice.cells[c].bbox.min.x());
                    const double right = unscale<double>(lattice.cells[c].bbox.max.x());
                    lo  = any ? std::min(lo, left) : left;
                    hi  = any ? std::max(hi, right) : right;
                    any = true;
                }
            if (any)
                widest = std::max(widest, hi - lo);
        }
        return widest;
    };
    const double whole = covered_span(*untouched.report()), thinned = covered_span(*spaced.report());
    INFO("covered span " << thinned << " vs " << whole);
    REQUIRE(whole > 20.);
    REQUIRE(thinned >= whole - 1e-9);

    // Leg 7, what the model around each contact is worth carrying it. Every contact under the lip
    // sits in the 1.2 mm band, fused on its own layer to a base 8 mm deep in y, so a measurement that
    // read the surrounding model solid reports the band's width and not the base's.
    const SupportAnalysis::Report &measured = *untouched.report();
    REQUIRE(measured.contact_risk_available);
    size_t anchors = 0;
    for (const SupportAnalysis::RegionCoverage &region : measured.coverage)
        anchors += region.anchor_ids.size();
    REQUIRE(measured.contact_risk.size() == anchors);
    REQUIRE(anchors > 0);
    double widest = 0.;
    for (const SupportAnalysis::ContactRisk &risk : measured.contact_risk) {
        INFO("contact " << risk.seed_id << " t " << risk.sample.local_width_mm << " n " << risk.sample.neck_width_mm
                        << " L " << risk.sample.lever_mm);
        // The lip is model solid all the way to the bed, so nothing under it is unmeasured.
        CHECK(risk.sample.status != ModelSupportRisk::Sample::Status::Unknown);
        CHECK(std::isfinite(risk.sample.risk_per_mm2));
        CHECK(risk.sample.risk_per_mm2 > 0.);
        widest = std::max(widest, risk.sample.local_width_mm);
    }
    // The band's own 1.089 mm depth, read from both its sides, and nowhere near the 8 mm the base is
    // deep under it: the measurement is off the model solid around the contact, not off the object.
    CHECK(widest > 1.5);
    CHECK(widest < 3.);
    CHECK(measured.damage.unknown_contacts == 0);
    // The group half of Damage is measured off the same pass, so the domain is available and carries
    // no unavailable reason.
    CHECK(measured.damage.available);
    CHECK_FALSE(measured.has_reason(SupportAnalysis::Reason::DamageUnavailable));
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

    // Leg 3, two peg bottoms whose contacts sit 3.5 mm apart on one object layer, with the lip 0.6 mm
    // above them, under a requested contact distance of 4 mm that spans both. The two peg bottoms and
    // the lip band are three overhang components: the pegs stand 3 mm apart in plan and the lip is
    // three layers up, outside the two-layer window that links a band to the one below it. So each
    // keeps the lowest contact of its own component and none of them carries another's cells, however
    // close the requested distance says they sit.
    const auto two_pegs = [](const char *mode) {
        const DynamicPrintConfig config = fixture_config({ { "support_style", "tree_slim" },
                                                           { "support_top_z_distance", "0.2" },
                                                           { "support_remove_small_overhang", "0" },
                                                           { "support_miniature_contacts", mode },
                                                           { "support_contact_min_distance", "4" } });
        Slic3r::Print print;
        init_and_process_print({ pegs_fixture() }, print, config);
        return contact_clusters(*print.objects().front());
    };
    const ContactClusters pegs_off = two_pegs("0");
    REQUIRE(count_in_box(pegs_off, -12.8, -11.7, 3.2, 4.3, 7.4) == 1);
    REQUIRE(count_in_box(pegs_off, -9.3, -8.2, 3.2, 4.3, 7.4) == 1);
    REQUIRE(count_in_box(pegs_off, -13.0, 13.0, 3.3, 6.0, 8.0) >= 1);

    const ContactClusters pegs_on = two_pegs("1");
    REQUIRE(count_in_box(pegs_on, -12.8, -11.7, 3.2, 4.3, 7.4) == 1);
    REQUIRE(count_in_box(pegs_on, -9.3, -8.2, 3.2, 4.3, 7.4) == 1);
    REQUIRE(count_in_box(pegs_on, -13.0, 13.0, 3.3, 6.0, 8.0) >= 1);
}

TEST_CASE("A wedge sheds one band per layer and thins their contacts to a fraction under the contact distance", "[MiniatureContacts]")
{
    // One slice of the wedge at a requested contact distance, with the measurement it installed. The
    // top gap is stated rather than inherited, the way every other analysis case here states it, and
    // 0.2 is PrintConfig.cpp's own default for it.
    struct Sliced
    {
        ContactClusters         clusters;
        SupportAnalysis::Report report;
    };
    const auto clusters_for = [](const char *distance) {
        // `init_print` arranges against an infinite bed and leaves the instance at the origin, so the
        // stock 0..200 mm bed's border would clip every branch that walks across x 0, which the wedge's
        // thinned branches do.
        const DynamicPrintConfig config = fixture_config({ { "support_style", "tree_slim" },
                                                           { "printable_area", "-100x-100,100x-100,100x100,-100x100" },
                                                           { "support_top_z_distance", "0.2" },
                                                           { "layer_height", "0.06" },
                                                           { "support_threshold_angle", "20" },
                                                           { "support_remove_small_overhang", "0" },
                                                           { "support_miniature_contacts", "1" },
                                                           { "support_contact_min_distance", distance } });
        Slic3r::Print print;
        init_and_process_print({ wedge_fixture() }, print, config);
        // The miniature mode forces the measurement, so a regression there is a failure here rather
        // than a crash on the dereference below.
        REQUIRE(print.objects().front()->support_analysis() != nullptr);
        return Sliced{ contact_clusters(*print.objects().front()), *print.objects().front()->support_analysis() };
    };

    // The slab's plan footprint past the block (the block's +x face is at centred x -5.35, the slab's
    // high end at 10.35) and every tip between the block's top at z 4 and the slab's high underside at
    // z 6.76, less the 0.2 mm gap.
    // Counted in layers, not in clusters: a contact cluster is one roof-gap piece, and a thinned tip is
    // fatter than the one it replaced and breaks into several of them, so the pieces count the shape of a
    // tip rather than the contacts. The wedge sheds one band per layer and takes one contact per band, so
    // the layers holding a contact under the slab are its contacts.
    const auto count_under_slab = [](const ContactClusters &cl) {
        std::set<double> layers;
        for (const std::pair<Point, double> &c : cl.centroids) {
            const double x = unscale<double>(c.first.x());
            const double y = unscale<double>(c.first.y());
            if (x >= -5.5 && x <= 10.5 && y >= -0.8 && y <= 0.8 && c.second >= 3.5 && c.second <= 7.0)
                layers.insert(c.second);
        }
        return int(layers.size());
    };

    const Sliced off = clusters_for("0");
    REQUIRE(count_under_slab(off.clusters) >= 20);

    const Sliced on = clusters_for("1");
    REQUIRE(on.report.key == off.report.key);
    // The thinned run reaches the same outcome and leaves no support group standing on nothing: what a
    // requested distance buys is fewer contacts, never a worse measurement.
    REQUIRE(on.report.status == off.report.status);
    REQUIRE(on.report.stability.unrooted_groups == 0);
    // The wedge sheds a 0.184 mm band per layer and the support extrusion is 0.42 mm wide, so no band
    // holds one extrusion: every region the problem carries is a sliver with no witness lattice at all.
    // A measurement that carried no regions at all would pass the loop below without reading anything.
    REQUIRE(! on.report.coverage.empty());
    for (const SupportAnalysis::RegionCoverage &region : on.report.coverage) {
        REQUIRE(region.cell_count() == 0);
        // And the report says why they hold no cells rather than leaving a reader to infer it.
        REQUIRE(region.printable == false);
    }
    REQUIRE(2 * count_under_slab(on.clusters) <= count_under_slab(off.clusters));

    // The seed counts the measurement carries, which is where a reader learns what the thinning did
    // rather than re-deriving it from the toolpaths. Both runs prepare the same problem, so they name
    // the same candidates; a contact distance of 0 thins none of them; the thinned run keeps fewer;
    // and what it kept is what it retained less whatever was added back.
    REQUIRE(on.report.seeds_candidate == off.report.seeds_candidate);
    REQUIRE(off.report.seeds_retained == off.report.seeds_candidate);
    REQUIRE(on.report.seeds_retained < off.report.seeds_retained);
    REQUIRE(on.report.seeds_kept + on.report.seeds_restored == on.report.seeds_retained);

    // The slab is one continuous 0.6 mm feature, so its neck is 0.6 mm and no contact under it stands on
    // anything narrower than the 0.42 mm support extrusion: the critical set here is what
    // `build_contact_seeds` marks first of its component and nothing else, the same set both ways, and a
    // small fraction of the candidates. A rule that read the sample's local width instead of its neck
    // would mark every candidate, because the local width reads zero at a contact on the model's edge.
    REQUIRE(on.report.critical_anchor_ids == off.report.critical_anchor_ids);
    REQUIRE(on.report.critical_anchor_ids.size() * 4 < on.report.seeds_candidate);
}

TEST_CASE("A bar between two pads restores the one contact decimation took from its uncovered middle", "[MiniatureContacts]")
{
    // tree_support_branch_distance 12 and max_bridge_length 10 put the legal reach at 5 mm (half the
    // smaller of the two) and spread the contour walk far enough that each region takes one contact.
    // The printable area is centred because init_print leaves the instance on the origin.
    const auto bar_config = [](std::initializer_list<ConfigBase::SetDeserializeItem> extra) {
        DynamicPrintConfig config = fixture_config({ { "support_style", "tree_slim" },
                                                     { "printable_area", "-100x-100,100x-100,100x100,-100x100" },
                                                     { "support_top_z_distance", "0.2" },
                                                     { "support_remove_small_overhang", "0" },
                                                     { "support_miniature_contacts", "1" },
                                                     { "support_contact_min_distance", "1.5" },
                                                     { "tree_support_branch_distance", "12" },
                                                     { "max_bridge_length", "10" } });
        config.set_deserialize_strict(extra);
        return config;
    };

    // Leg 1, the add-back through the pipeline. Both of the bar's contacts stand within the requested
    // 1.5 mm of a pad contact and are decimated; the bar's middle cells are over 2 mm from either pad,
    // so the bar keeps one contact of its own and the region ends up fully covered with material under it.
    Slic3r::Print print;
    init_and_process_print({ bar_fixture() }, print, bar_config({}));
    REQUIRE(print.objects().front()->support_analysis() != nullptr);
    const SupportAnalysis::Report &report = *print.objects().front()->support_analysis();

    // The bar's own region, named by its contact z and by being the only wide one: the pads are half a
    // millimetre across and the bar is 4.6 mm long.
    size_t bar = report.key.region_ids.size();
    size_t wide_count = 0;
    for (size_t i = 0; i < report.key.region_ids.size(); ++ i)
        if (std::abs(report.key.region_contact_z[i] - 2.2) < 1e-6 &&
            get_extents(report.key.witnesses[i]).size().x() > scale_(4.)) {
            bar = i;
            ++ wide_count;
        }
    INFO("regions " << report.key.region_ids.size());
    REQUIRE(wide_count == 1);
    REQUIRE(report.coverage[bar].printable == true);
    REQUIRE(report.coverage[bar].cell_count() > 0);
    REQUIRE(report.coverage[bar].covered_count() == report.coverage[bar].cell_count());
    REQUIRE(report.coverage[bar].emitted_path == true);
    REQUIRE(report.seeds_restored == 1);
    REQUIRE(report.seeds_retained == report.seeds_kept + 1);
    REQUIRE(report.seeds_kept >= 2);

    // Leg 2, the same fixture printed through a 0.6 mm nozzle at a 0.7 mm extrusion. The bar's 0.6 mm
    // underside no longer holds one extrusion, so the detector files it as regions that are not
    // printable, and the neck the risk field measures under every contact of the bar and the pads is
    // narrower than that extrusion: criticality reaches the report for more than the one seed that is
    // first of the fixture's single overhang component.
    Slic3r::Print wide;
    init_and_process_print({ bar_fixture() }, wide,
                           bar_config({ { "nozzle_diameter", "0.6" }, { "line_width", "0.7" }, { "support_line_width", "0.7" } }));
    REQUIRE(wide.objects().front()->support_analysis() != nullptr);
    const SupportAnalysis::Report &wide_report = *wide.objects().front()->support_analysis();

    size_t bar_regions = 0;
    for (size_t i = 0; i < wide_report.key.region_ids.size(); ++ i)
        if (std::abs(wide_report.key.region_contact_z[i] - 2.2) < 1e-6) {
            ++ bar_regions;
            REQUIRE(wide_report.coverage[i].printable == false);
            REQUIRE(wide_report.coverage[i].critical == true);
        }
    INFO("wide regions " << wide_report.key.region_ids.size());
    REQUIRE(bar_regions >= 1);
    // First-of-component alone would leave exactly one critical seed on this one-component fixture.
    REQUIRE(wide_report.critical_anchor_ids.size() > 1);
    // Nothing is restored: no region here is printable, so there is no uncovered printable region to
    // take a contact back.
    REQUIRE(wide_report.seeds_restored == 0);

    // Leg 3, a plain cube: nothing overhangs, so the problem carries no region and no contact at all and
    // the measurement leaves nothing open.
    Slic3r::Print cube;
    init_and_process_print({ make_cube(10., 10., 10.) }, cube, bar_config({}));
    REQUIRE(cube.objects().front()->support_analysis() != nullptr);
    const SupportAnalysis::Report &cube_report = *cube.objects().front()->support_analysis();
    REQUIRE(cube_report.key.region_ids.empty());
    REQUIRE(cube_report.seeds_candidate == 0);
    REQUIRE(cube_report.seeds_retained == 0);
    REQUIRE(SupportAnalysis::support_unresolved(cube_report) == false);
}

TEST_CASE("A blade beside a broader guard is placed off the print's own geometry without giving up its coverage", "[MiniatureContacts]")
{
    // tree_support_branch_diameter sizes generate_contact_points' per-layer hash grid: the cell is
    // `radius_scaled + 1`, half the branch diameter, so at the 5 mm default one contact per 2.5 mm of
    // overhang is all this band ever gets and there is nothing for a placement to have room in. At
    // 1 mm the detector places the contacts it means to.
    const auto config = [](const char *mode) {
        return fixture_config({ { "support_style", "tree_slim" }, { "support_top_z_distance", "0.2" },
                                { "support_remove_small_overhang", "0" },
                                { "tree_support_branch_diameter", "1" },
                                { "support_miniature_contacts", mode }, { "support_contact_min_distance", "3" } });
    };
    AnalysisRun off, on;
    run_analysis(off, blade_fixture(), config("0"), true);
    run_analysis(on,  blade_fixture(), config("1"), true);
    REQUIRE(off.report() != nullptr);
    REQUIRE(on.report() != nullptr);

    // Summed over the contacts of one report: the weight the model carries at each of them, which is
    // the estimate this pass acts on before any material exists.
    const auto estimated_damage = [](const SupportAnalysis::Report &report) {
        double total = 0.;
        for (const SupportAnalysis::ContactRisk &contact : report.contact_risk)
            total += contact.sample.risk_per_mm2;
        return total;
    };

    SECTION("one prepared problem serves both modes and what ships gives up no cell, no anchor and no ground")
    {
        // The same regions, layers and witness polygons: the mode changed which contacts an attempt may
        // move, and nothing about the problem they are placed against.
        REQUIRE(on.report()->key == off.report()->key);
        REQUIRE(on.report()->contact_risk_available);
        REQUIRE(off.report()->contact_risk_available);
        CHECK(on.report()->measured_contact_mm2 > 0.);
        // The estimate can only improve: the mode off generates from the contacts the detector placed
        // and never relocates one, and the mode on only ever moves a contact onto ground the field
        // measures as no worse.
        CHECK(estimated_damage(*on.report()) <= estimated_damage(*off.report()) + 1e-9);
    }

    SECTION("contacts on the print's own overhang leave the model's edges for broader measured ground")
    {
        // The blade's overhang polygon as the slicer produced it, and the model field measured off the
        // object's own layers: the geometry this print ran on, not a written-out stand-in for it.
        const SupportAnalysis::CoverageKey &key = off.report()->key;
        REQUIRE(key.witnesses.size() == 1);
        std::vector<ModelSupportRisk::Slice> slices;
        for (const Layer *layer : off.object().layers()) {
            ModelSupportRisk::Slice slice;
            slice.bottom_z_mm = layer->bottom_z();
            slice.top_z_mm    = layer->print_z;
            slice.solids      = layer->lslices;
            slices.push_back(std::move(slice));
        }
        const ModelSupportRisk::Field field = ModelSupportRisk::build(slices, key.extrusion_width_mm, []() { return false; });
        REQUIRE(field.status == ModelSupportRisk::Field::Status::Complete);

        const size_t layer = key.region_layers.front();
        MiniatureSupport::Problem problem;
        problem.extrusion_width_mm = key.extrusion_width_mm;
        // legal_reach off the settings in force: the 5 mm branch distance and the 10 mm bridge length
        // fixture_config leaves at their defaults.
        add_region(problem, key.witnesses.front(), layer, key.region_contact_z.front(),
                   MiniatureSupport::legal_reach(5., 10.));
        // A contact on every vertex of that polygon, which is where the contact pass puts its corner
        // contacts: on the outline, where the model has no thickness left to measure.
        for (const Point &vertex : key.witnesses.front().contour.points) {
            MiniatureSupport::ContactSeed seed;
            seed.id        = uint64_t(problem.seeds.size());
            seed.position  = vertex;
            seed.radius_mm = 0.5;
            problem.seeds.push_back(seed);
        }
        REQUIRE(problem.seeds.size() > 8);

        const auto weight_of = [&](const std::vector<Point> &positions) {
            double total = 0.;
            for (const Point &position : positions)
                total += ModelSupportRisk::sample(field, layer, position).risk_per_mm2;
            return total;
        };
        const auto broad = [&](const std::vector<Point> &positions) {
            size_t n = 0;
            for (const Point &position : positions)
                if (ModelSupportRisk::sample(field, layer, position).local_width_mm >= 1.)
                    ++ n;
            return n;
        };

        const std::vector<Point>          before    = retained_positions(problem, MiniatureSupport::Selection());
        const MiniatureSupport::Selection selection = MiniatureSupport::select_contacts(problem, field);
        REQUIRE(selection.retained.size() == problem.seeds.size());
        const std::vector<Point> after = retained_positions(problem, selection);

        // Every contact ends on model the field can measure, on more material than it started on, and
        // carrying less weight for it.
        for (const Point &position : after)
            CHECK(ModelSupportRisk::sample(field, layer, position).status == ModelSupportRisk::Sample::Status::Known);
        CHECK(broad(after) > broad(before));
        CHECK(weight_of(after) < weight_of(before));

        // And the region gave up nothing to get there.
        const std::vector<bool> held = covered_union(problem.regions.front(), problem.extrusion_width_mm, before);
        const std::vector<bool> kept = covered_union(problem.regions.front(), problem.extrusion_width_mm, after);
        REQUIRE(held.size() == kept.size());
        for (size_t c = 0; c < held.size(); ++ c)
            if (held[c])
                CHECK(kept[c]);

        // The blade's far half is 0.8 mm of model with nothing broader inside a contact's reach of it.
        // A contact out there stays out there: the tip needs the support it has, and there is no
        // stronger place for it to be traded to.
        size_t on_the_tip = 0;
        for (size_t s = 0; s < before.size(); ++ s)
            if (unscale<double>(before[s].x()) > 4.) {
                ++ on_the_tip;
                CHECK(unscale<double>(after[s].x()) > 4.);
            }
        CHECK(on_the_tip > 0);
    }
}

TEST_CASE("A requested support analysis names required regions and contact seeds by value", "[MiniatureContacts]")
{
    // support_style, because the default resolves to organic, which the analysis does not run under;
    // support_top_z_distance explicitly at PrintConfig.cpp's own default, so the gap the tips are
    // planned against is a stated number rather than whatever the fixture happens to resolve.
    // support_remove_small_overhang = 0 keeps the lip's thin band as well, so the problem carries a
    // detected region beside the tail; sharp tail detection sits under its own gate and is untouched
    // by that key.
    const DynamicPrintConfig config = fixture_config({ { "support_style", "tree_slim" },
                                                       { "support_top_z_distance", "0.2" },
                                                       { "support_remove_small_overhang", "0" } });

    AnalysisRun run;
    run_analysis(run, tail_fixture(), config);
    const SupportAnalysis::Report *report = run.report();
    REQUIRE(report != nullptr);

    // The key names the problem the report was measured from: the region ids in order, the object
    // layer and contact Z each of them came from, the polygon each one is, and the width one support
    // extrusion resolves to. Ids are dense values counted from zero, so nothing here is an address.
    const SupportAnalysis::CoverageKey &key = report->key;
    REQUIRE(key.region_ids.size() > 1);
    REQUIRE(key.region_layers.size() == key.region_ids.size());
    REQUIRE(key.region_contact_z.size() == key.region_ids.size());
    REQUIRE(key.witnesses.size() == key.region_ids.size());
    REQUIRE(key.extrusion_width_mm > 0.);
    REQUIRE(report->coverage.size() == key.region_ids.size());
    for (size_t i = 0; i < key.region_ids.size(); ++ i) {
        INFO("region " << i);
        REQUIRE(key.region_ids[i] == uint64_t(i));
        REQUIRE(report->coverage[i].region_id == uint64_t(i));
        REQUIRE(key.witnesses[i].area() > 0);
        // A region is one same-layer connected overhang polygon, and its contact Z is the underside
        // of the object layer it was detected on.
        REQUIRE(key.region_layers[i] < size_t(run.object().layer_count()));
        REQUIRE_THAT(key.region_contact_z[i], WithinAbs(run.object().get_layer(int(key.region_layers[i]))->bottom_z(), 1e-9));
        if (i > 0)
            REQUIRE(key.region_layers[i - 1] <= key.region_layers[i]);
    }

    // Seed ids are dense values too, counted from zero over the whole problem in the deterministic
    // order (object layer, region, position, pin category), so every id belongs to exactly one region.
    const std::vector<uint64_t> anchors = all_anchor_ids(*report);
    REQUIRE(anchors.size() > key.region_ids.size());
    for (size_t i = 0; i < anchors.size(); ++ i)
        REQUIRE(anchors[i] == uint64_t(i));

    // The critical set is sorted, carries no id twice, and names only seeds this problem has.
    REQUIRE(! report->critical_anchor_ids.empty());
    REQUIRE(std::is_sorted(report->critical_anchor_ids.begin(), report->critical_anchor_ids.end()));
    REQUIRE(std::adjacent_find(report->critical_anchor_ids.begin(), report->critical_anchor_ids.end()) == report->critical_anchor_ids.end());
    REQUIRE(report->critical_anchor_ids.size() <= anchors.size());
    REQUIRE(report->critical_anchor_ids.back() < uint64_t(anchors.size()));
    // A region is critical exactly when it carries a critical seed.
    for (const SupportAnalysis::RegionCoverage &region : report->coverage) {
        INFO("region " << region.region_id);
        REQUIRE(region.critical == (critical_anchors_of(*report, region) > 0));
    }
    // Of the clauses `seed.critical = seed.pinned || first || critical_by_neck[i]` ORs, nothing here
    // is pinned and no neck reads narrower than an extrusion, so the marking that is left is `first`:
    // the first seed of each directed overhang component and no other. Every region keeps at most one
    // critical seed, and the problem still keeps some.
    REQUIRE(report->pinned_anchor_ids.empty());
    REQUIRE(report->critical_anchor_ids.size() < anchors.size());
    for (const SupportAnalysis::RegionCoverage &region : report->coverage) {
        INFO("region " << region.region_id);
        REQUIRE(critical_anchors_of(*report, region) <= 1);
    }

    // The order the ids are handed out in is the sorted order, not the order the parallel contact
    // pass inserted the nodes in: one worker reaches the same problem, key and critical set.
    AnalysisRun serial;
    {
        tbb::global_control gc(tbb::global_control::max_allowed_parallelism, 1);
        run_analysis(serial, tail_fixture(), config);
    }
    REQUIRE(serial.report() != nullptr);
    REQUIRE(serial.report()->key == report->key);
    REQUIRE(serial.report()->critical_anchor_ids == report->critical_anchor_ids);

    // And the key compares by value rather than by identity: a different overhang threshold leaves a
    // different set of regions, so the two reports cannot be mistaken for one problem.
    AnalysisRun steeper;
    DynamicPrintConfig steeper_config = config;
    steeper_config.set_deserialize_strict({ { "support_threshold_angle", "20" } });
    run_analysis(steeper, tail_fixture(), steeper_config);
    REQUIRE(steeper.report() != nullptr);
    REQUIRE(steeper.report()->key != report->key);
}

TEST_CASE("A painted vertical enforcer seeds no contact and keeps the contact holding its slot", "[MiniatureContacts]")
{
    const DynamicPrintConfig config = fixture_config({ { "support_style", "tree_slim" }, { "support_top_z_distance", "0.2" } });

    // fin_fixture, because its 40 deg fin leaves a detected overhang band on layer after layer while
    // the fin's two x-normal faces and the base's four walls stay vertical. Painting those puts
    // enforcer points on layers that already carry a region, which is where build_contact_seeds
    // matches contacts to regions.
    size_t      painted = 0;
    AnalysisRun run;
    run_analysis(run, fin_fixture(), config, true, [&painted](ModelVolume &mv) { painted = paint_vertical_enforcers(mv); });
    REQUIRE(painted > 0);
    const SupportAnalysis::Report *report = run.report();
    REQUIRE(report != nullptr);

    // What the paint put into the pipeline, read back through the seam the support generator reads it
    // through: vertical enforcer points, and not one enforced overhang.
    std::vector<Polygons>                enforcers;
    std::vector<std::pair<Vec3f, Vec3f>> vertical_points;
    run.object().project_and_append_custom_facets(false, EnforcerBlockerType::ENFORCER, enforcers, &vertical_points);
    REQUIRE(! vertical_points.empty());
    for (const Polygons &layer_enforcers : enforcers)
        REQUIRE(layer_enforcers.empty());

    // And they land where the nearest-region fallback could reach them: build_contact_seeds only
    // looks at contacts on an object layer that carries a region, so at least one enforcer point has
    // to sit on such a layer for this fixture to state anything.
    const std::set<size_t> region_layers(report->key.region_layers.begin(), report->key.region_layers.end());
    size_t                 points_on_region_layers = 0;
    for (const std::pair<Vec3f, Vec3f> &pt_and_normal : vertical_points)
        for (const Layer *layer : run.object().layers())
            // generate_contact_points files an enforcer point on the first layer whose top reaches it.
            if (float(layer->print_z) >= pt_and_normal.first.z()) {
                if (layer->id() > 0 && region_layers.count(size_t(layer->id())) > 0)
                    ++ points_on_region_layers;
                break;
            }
    REQUIRE(points_on_region_layers > 0);

    // The behavior: a vertical enforcer point is a contact placed for no overhang polygon of its own,
    // so it is a source of nothing, and the regions and the contact seeds are the ones the same model
    // produces with nothing painted at all. Where a detected contact already holds the slot a point
    // asked for, that contact stands at the paint and is pinned; a pinned seed is critical, so the
    // critical set is the unpainted one plus the pinned seeds.
    AnalysisRun plain;
    run_analysis(plain, fin_fixture(), config);
    REQUIRE(plain.report() != nullptr);
    REQUIRE(plain.report()->key == report->key);
    REQUIRE(all_anchor_ids(*plain.report()) == all_anchor_ids(*report));
    REQUIRE(plain.report()->pinned_anchor_ids.empty());
    REQUIRE(! report->pinned_anchor_ids.empty());
    std::vector<uint64_t> expected_critical;
    std::set_union(plain.report()->critical_anchor_ids.begin(), plain.report()->critical_anchor_ids.end(),
                   report->pinned_anchor_ids.begin(), report->pinned_anchor_ids.end(), std::back_inserter(expected_critical));
    REQUIRE(report->critical_anchor_ids == expected_critical);
    REQUIRE(plain.report()->coverage.size() == report->coverage.size());
    for (size_t i = 0; i < report->coverage.size(); ++ i) {
        INFO("region " << i);
        REQUIRE(report->coverage[i].anchor_ids == plain.report()->coverage[i].anchor_ids);
        const bool holds_pinned = std::any_of(report->coverage[i].anchor_ids.begin(), report->coverage[i].anchor_ids.end(),
            [report](uint64_t id) { return std::binary_search(report->pinned_anchor_ids.begin(), report->pinned_anchor_ids.end(), id); });
        REQUIRE(report->coverage[i].critical == (plain.report()->coverage[i].critical || holds_pinned));
    }
}

TEST_CASE("A painted overhang keeps every contact placed under the paint", "[MiniatureContacts]")
{
    // lip_fixture's underside is an overhang the detector finds on its own, so the detected band places
    // its contacts on that layer before the enforced band does, and the two claim the same slots.
    const DynamicPrintConfig config = fixture_config({ { "support_style", "tree_slim" }, { "support_top_z_distance", "0.2" },
                                                       { "support_miniature_contacts", "1" }, { "support_contact_min_distance", "5" } });
    // The lip's underside: the horizontal facets facing down, above the base's bottom on the bed.
    const auto paint_lip_underside = [](ModelVolume &mv) {
        float bed_z = std::numeric_limits<float>::max();
        for (const Vec3f &v : mv.mesh().its.vertices)
            bed_z = std::min(bed_z, v.z());
        REQUIRE(paint_enforcers(mv, [bed_z](const Vec3f &a, const Vec3f &b, const Vec3f &c) {
                    return a.z() == b.z() && a.z() == c.z() && a.z() > bed_z + 1.f && (b - a).cross(c - a).z() < 0.f;
                }) > 0);
    };

    AnalysisRun plain, painted;
    run_analysis(plain, lip_fixture(), config, false);
    run_analysis(painted, lip_fixture(), config, false, paint_lip_underside);
    REQUIRE(plain.report() != nullptr);
    REQUIRE(painted.report() != nullptr);

    // Unpainted, the 5 mm distance thins the lip's contacts, so keeping all of them is the paint's doing.
    INFO("plain retained " << plain.report()->seeds_retained << " of " << plain.report()->seeds_candidate);
    REQUIRE(plain.report()->seeds_retained < plain.report()->seeds_candidate);

    // Every contact on the lip's layer stands on the paint, whichever band placed it, so every one is
    // pinned and the thinning keeps them all.
    INFO("painted retained " << painted.report()->seeds_retained << " of " << painted.report()->seeds_candidate
                             << ", pinned " << painted.report()->pinned_anchor_ids.size());
    REQUIRE(painted.report()->pinned_anchor_ids.size() == painted.report()->seeds_candidate);
    REQUIRE(painted.report()->seeds_retained == painted.report()->seeds_candidate);
}

TEST_CASE("Merged support branches carry every contact source that reached them", "[MiniatureContacts]")
{
    const DynamicPrintConfig config = fixture_config({ { "support_style", "tree_slim" }, { "support_top_z_distance", "0.2" } });

    AnalysisRun run;
    run_analysis(run, fin_fixture(), config);
    const SupportAnalysis::Report *report = run.report();
    REQUIRE(report != nullptr);

    // The attributed areas the router recorded, one per routed node area, before draw_circles unions
    // the layer's areas together and there is no way left to tell whose material is whose.
    const std::shared_ptr<const SupportAnalysis::EmittedSupport> &emitted = run.object().emitted_support();
    REQUIRE(emitted != nullptr);
    REQUIRE(! emitted->layers.empty());

    const std::vector<uint64_t> anchors = all_anchor_ids(*report);
    REQUIRE(anchors.size() > 1);
    std::vector<uint64_t> region_of(anchors.size(), 0);
    for (const SupportAnalysis::RegionCoverage &region : report->coverage)
        for (uint64_t id : region.anchor_ids)
            region_of[id] = region.region_id;

    size_t             areas = 0, multi_region_areas = 0, max_regions_in_area = 0;
    std::set<uint64_t> reached;
    for (const SupportAnalysis::EmittedLayer &layer : emitted->layers) {
        INFO("support layer " << layer.support_layer_index);
        REQUIRE(layer.print_z > layer.bottom_z);
        for (const SupportAnalysis::AttributedArea &area : layer.areas) {
            ++ areas;
            // Sorted and unique: a union, not an append. A branch carrying one source twice would
            // count that source twice everywhere the report reads the list.
            REQUIRE(std::is_sorted(area.source_ids.begin(), area.source_ids.end()));
            REQUIRE(std::adjacent_find(area.source_ids.begin(), area.source_ids.end()) == area.source_ids.end());
            std::set<uint64_t> regions;
            for (uint64_t id : area.source_ids) {
                REQUIRE(id < uint64_t(anchors.size()));
                reached.insert(id);
                regions.insert(region_of[id]);
            }
            if (regions.size() > 1)
                ++ multi_region_areas;
            max_regions_in_area = std::max(max_regions_in_area, regions.size());
        }
    }
    REQUIRE(areas > 0);

    // Branches converge on the way down and a merge unions what the two sides carried rather than
    // picking a winner, so some routed area below the tips carries sources from two regions at once.
    REQUIRE(multi_region_areas > 0);
    REQUIRE(max_regions_in_area >= 2);

    // And no source is lost on the way: every contact this pass placed is carried by some routed area.
    REQUIRE(reached.size() == anchors.size());
}

TEST_CASE("Emitted contact is measured from printed support material, not from the planned gap", "[MiniatureContacts]")
{
    // Leg 1, a positive top gap: the tip sits a planned gap under the model and the layer drawn
    // inside that gap is never extruded, so only what the toolpaths cover can count as contact.
    const double gap = 0.2, layer_height = 0.2;
    AnalysisRun  gap_run;
    run_analysis(gap_run, fin_fixture(),
                 fixture_config({ { "support_style", "tree_slim" }, { "support_top_z_distance", "0.2" } }));
    const SupportAnalysis::Report *gap_report = gap_run.report();
    REQUIRE(gap_report != nullptr);
    REQUIRE(gap_report->coverage_available);
    REQUIRE(gap_report->measured_contact_mm2 > 0.);
    REQUIRE(gap_report->provenance.emitted_areas > 0);
    REQUIRE(gap_report->provenance.traced_regions > 0);
    REQUIRE(gap_report->status != SupportAnalysis::Report::Status::UnresolvedCoverage);

    size_t              anchored = 0;
    double              summed = 0.;
    std::set<long long> tip_layers;
    for (const SupportAnalysis::RegionCoverage &region : gap_report->coverage) {
        summed += region.covered_mm2;
        if (! region.anchored)
            continue;
        ++ anchored;
        INFO("region " << region.region_id);
        const double contact_z = gap_report->key.region_contact_z[region.region_id];
        // The tip that anchors a region sits inside that region's own planned gap interval: one
        // whole gap under it, and no more than one support slab below that.
        REQUIRE(region.tip_print_z <= contact_z - gap + 1e-6);
        REQUIRE(region.tip_print_z >= contact_z - gap - layer_height - 1e-6);
        REQUIRE(region.covered_mm2 > 0.);
        // Nothing here reads the witness lattice: the fin's overhang bands are narrower than one support
        // extrusion, so `build_required_regions` lays them none and this case measures contact by area.
        tip_layers.insert(std::llround(region.tip_print_z * 1000.));
    }
    REQUIRE(anchored > 1);
    // The fin's overhang bands sit on many object layers, and material printed under one of them
    // cannot stand in for another: vertically separate regions are anchored at their own tips.
    REQUIRE(tip_layers.size() > 1);
    // Material two regions share counts once globally and stays traceable to both, so the union of
    // every qualifying intersection is never larger than the per-region areas added up.
    REQUIRE(gap_report->measured_contact_mm2 <= summed + 1e-9);
    // And it is bounded by what the support extrusions actually cover: the measurement is an
    // intersection with printed material, not a reading of the drawn areas.
    double printed_mm2 = 0.;
    for (const SupportLayer *sl : gap_run.object().support_layers())
        for (const ExPolygon &poly : union_ex(sl->support_fills.polygons_covered_by_width(0.f)))
            printed_mm2 += poly.area() * SCALING_FACTOR * SCALING_FACTOR;
    REQUIRE(printed_mm2 > 0.);
    REQUIRE(gap_report->measured_contact_mm2 < printed_mm2);

    // Leg 2, a zero top gap: no gap layer is drawn at all, the tip prints onto the model, and the
    // contact measured for a region reaches that region's own contact Z.
    AnalysisRun zero_run;
    run_analysis(zero_run, fin_fixture(),
                 fixture_config({ { "support_style", "tree_slim" }, { "support_top_z_distance", "0" } }));
    const SupportAnalysis::Report *zero_report = zero_run.report();
    REQUIRE(zero_report != nullptr);
    REQUIRE(zero_report->coverage_available);
    REQUIRE(zero_report->measured_contact_mm2 > 0.);
    size_t zero_anchored = 0;
    for (const SupportAnalysis::RegionCoverage &region : zero_report->coverage) {
        if (! region.anchored)
            continue;
        ++ zero_anchored;
        INFO("region " << region.region_id);
        const double contact_z = zero_report->key.region_contact_z[region.region_id];
        REQUIRE(region.tip_print_z <= contact_z + 1e-6);
        REQUIRE(region.tip_print_z >= contact_z - layer_height - 1e-6);
    }
    REQUIRE(zero_anchored > 0);
}

TEST_CASE("Support and raft volume are summed from the emitted extrusions alone", "[MiniatureContacts]")
{
    // A raft under the object, so both halves of the material metric are non-zero. The report is taken
    // where the support is generated and posSimplifySupportPath rewrites those paths afterwards, so
    // this leg turns path simplification off (arc fitting off, resolution 0) and the two sums below
    // are then the same extrusions read twice.
    AnalysisRun run;
    run_analysis(run, fin_fixture(),
                 fixture_config({ { "support_style", "tree_slim" }, { "support_top_z_distance", "0.2" },
                                  { "raft_layers", "3" },
                                  { "enable_arc_fitting", "0" }, { "resolution", "0" } }));
    const SupportAnalysis::Report *report = run.report();
    REQUIRE(report != nullptr);

    const PrintObject &po = run.object();
    REQUIRE(po.support_raft_layers() >= 3);

    // The same two sums taken independently, straight off the extrusions the generator emitted: the
    // leading support layers are the raft the object stands on, the rest is its support.
    double expected_raft = 0., expected_support = 0.;
    size_t index = 0;
    for (const SupportLayer *sl : po.support_layers()) {
        (index < po.support_raft_layers() ? expected_raft : expected_support) += sl->support_fills.total_volume();
        ++ index;
    }
    REQUIRE(expected_raft > 0.);
    REQUIRE(expected_support > 0.);
    // 1e-5 relative: the report is taken where the support is generated and posSimplifySupportPath
    // rewrites those paths afterwards. With arc fitting off and the resolution at zero that pass
    // moves the sums by about 1e-6 of themselves on this fixture, and by about 1.6e-5 at the stock
    // resolution, so this bound separates "the same extrusions" from "the simplified ones".
    REQUIRE_THAT(report->raft_volume_mm3, WithinRel(expected_raft, 1e-5));
    REQUIRE_THAT(report->support_volume_mm3, WithinRel(expected_support, 1e-5));

    // The optimization metric is the two added up, and it holds nothing the object prints for itself.
    const double metric = report->support_volume_mm3 + report->raft_volume_mm3;
    double       model_volume = 0.;
    for (const std::pair<const int, std::pair<size_t, double>> &role : role_totals(po))
        model_volume += role.second.second;
    REQUIRE(model_volume > 0.);
    REQUIRE_THAT(metric, WithinRel(expected_raft + expected_support, 1e-5));
    REQUIRE(std::abs(metric - (expected_raft + expected_support + model_volume)) > 1e-6);

    // And it is read off the extrusions, not off an export: no G-code was produced and the print's
    // own statistics, which only an export fills in, are still empty.
    REQUIRE_FALSE(run.print.is_step_done(psGCodeExport));
    REQUIRE_THAT(run.print.print_statistics().total_extruded_volume, WithinAbs(0., 1e-12));

    // Without a raft the raft half is zero rather than absent, the support half still stands, and the
    // brim the object prints for itself stays outside both. PrintObject::has_brim() is false while a
    // raft is on, so the brim only exists on this leg.
    AnalysisRun no_raft;
    run_analysis(no_raft, fin_fixture(),
                 fixture_config({ { "support_style", "tree_slim" }, { "support_top_z_distance", "0.2" },
                                  { "brim_type", "outer_only" }, { "brim_width", "3" },
                                  { "enable_arc_fitting", "0" }, { "resolution", "0" } }));
    REQUIRE(no_raft.report() != nullptr);
    REQUIRE(no_raft.object().support_raft_layers() == 0);
    REQUIRE_THAT(no_raft.report()->raft_volume_mm3, WithinAbs(0., 1e-12));
    REQUIRE(no_raft.report()->support_volume_mm3 > 0.);

    double no_raft_support = 0.;
    for (const SupportLayer *sl : no_raft.object().support_layers())
        no_raft_support += sl->support_fills.total_volume();
    REQUIRE_THAT(no_raft.report()->support_volume_mm3, WithinRel(no_raft_support, 1e-5));

    double brim_volume = 0.;
    for (const std::pair<const ObjectID, ExtrusionEntityCollection> &brim : no_raft.print.get_brimMap())
        brim_volume += brim.second.total_volume();
    REQUIRE(brim_volume > 0.);
    REQUIRE(std::abs(no_raft.report()->support_volume_mm3 - (no_raft_support + brim_volume)) > 1e-6);
}

TEST_CASE("Support components come from printed slabs that touch, and material with no root is counted", "[MiniatureContacts]")
{
    AnalysisRun run;
    run_analysis(run, fin_fixture(),
                 fixture_config({ { "support_style", "tree_slim" }, { "support_top_z_distance", "0.2" } }));
    const SupportAnalysis::Report *report = run.report();
    REQUIRE(report != nullptr);

    // Stability is measured now, so it is available and carries no unavailable reason.
    REQUIRE(report->stability.available);
    REQUIRE_FALSE(report->has_reason(SupportAnalysis::Reason::StabilityUnavailable));
    // Every branch this fixture prints stands on the plate: nothing floats.
    REQUIRE(report->stability.unsupported_paths == 0);

    const std::shared_ptr<const SupportAnalysis::EmittedSupport> emitted = run.object().emitted_support();
    REQUIRE(emitted != nullptr);

    // The other failure count, recomputed from the emitted record the same way a reader would: a
    // required region whose contacts the router carried into some drawn area, none of which reached
    // printed material, has routed provenance and no emitted path.
    std::map<uint64_t, uint64_t> region_of_seed;
    for (const SupportAnalysis::RegionCoverage &region : report->coverage)
        for (uint64_t id : region.anchor_ids)
            region_of_seed[id] = region.region_id;
    std::set<uint64_t> routed, printed;
    for (const SupportAnalysis::EmittedLayer &layer : emitted->layers)
        for (const SupportAnalysis::AttributedArea &area : layer.areas) {
            const bool on_paper = ! area.virtual_gap && layer.emitted_available &&
                                  ! intersection_ex(ExPolygons{ area.area }, layer.emitted).empty();
            for (uint64_t id : area.source_ids) {
                const std::map<uint64_t, uint64_t>::const_iterator it = region_of_seed.find(id);
                if (it == region_of_seed.end())
                    continue;
                routed.insert(it->second);
                if (on_paper)
                    printed.insert(it->second);
            }
        }
    size_t expected_unrooted = 0;
    for (uint64_t id : routed)
        if (printed.count(id) == 0)
            ++ expected_unrooted;
    REQUIRE(report->stability.unrooted_groups == expected_unrooted);
    // And the per-region fact the count is made of is on the report itself, so a group cannot leave
    // a comparison by quietly losing its provenance.
    for (const SupportAnalysis::RegionCoverage &region : report->coverage)
        REQUIRE(region.emitted_path == (printed.count(region.region_id) > 0));
    // A region whose material was measured at its tip printed something by construction.
    for (const SupportAnalysis::RegionCoverage &region : report->coverage)
        if (region.anchored)
            REQUIRE(region.emitted_path);

    // The same emitted geometry with everything under 1 mm taken away: the branches that stood on
    // the plate no longer reach it, and floating material is counted rather than passed over.
    SupportAnalysis::EmittedSupport cut = *emitted;
    size_t                          severed = 0;
    for (SupportAnalysis::EmittedLayer &layer : cut.layers)
        if (layer.print_z < 1.) {
            layer.emitted.clear();
            layer.emitted_available = false;
            ++ severed;
        }
    REQUIRE(severed > 0);
    const SupportAnalysis::Report lifted = SupportAnalysis::measure(run.object(), MiniatureSupport::Problem(), cut);
    REQUIRE(lifted.stability.available);
    REQUIRE(lifted.stability.unsupported_paths > 0);

    // A band of printed material removed in the middle: the drawn masks still span the gap and the
    // routed provenance still runs through it, and neither joins what printed material no longer
    // joins. What is left above the gap is a separate component, and it has no root.
    SupportAnalysis::EmittedSupport gapped = *emitted;
    size_t                          blanked = 0;
    for (SupportAnalysis::EmittedLayer &layer : gapped.layers)
        if (layer.print_z > 5. && layer.print_z < 6.) {
            layer.emitted.clear();
            layer.emitted_available = false;
            ++ blanked;
        }
    REQUIRE(blanked > 0);
    const SupportAnalysis::Report split = SupportAnalysis::measure(run.object(), MiniatureSupport::Problem(), gapped);
    REQUIRE(split.stability.available);
    REQUIRE(split.stability.unsupported_paths > 0);

    // A 0.1 mm layer on the plate under 0.25 mm layers above it, so a slab standing on the first
    // layer is taller than twice the height it stands at. Only a bottom at zero is on the plate:
    // whatever the slab above it measures, it is a whole layer up in the air.
    AnalysisRun thin;
    run_analysis(thin, fin_fixture(),
                 fixture_config({ { "support_style", "tree_slim" }, { "support_top_z_distance", "0.2" },
                                  { "initial_layer_print_height", "0.1" }, { "layer_height", "0.25" } }));
    const SupportAnalysis::Report *thin_report = thin.report();
    REQUIRE(thin_report != nullptr);
    REQUIRE(thin_report->stability.available);
    REQUIRE(thin_report->stability.unsupported_paths == 0);

    const std::shared_ptr<const SupportAnalysis::EmittedSupport> thin_emitted = thin.object().emitted_support();
    REQUIRE(thin_emitted != nullptr);
    SupportAnalysis::EmittedSupport lifted_first = *thin_emitted;
    size_t                          plate_slabs = 0, first_layer_up = 0;
    for (SupportAnalysis::EmittedLayer &layer : lifted_first.layers) {
        if (! layer.emitted_available || layer.emitted.empty())
            continue;
        if (layer.bottom_z < 0.05) {
            layer.emitted.clear();          // the material that was standing on the plate
            layer.emitted_available = false;
            ++ plate_slabs;
        } else if (layer.bottom_z < 0.15 && layer.print_z - layer.bottom_z > 0.2) {
            ++ first_layer_up;              // 0.1 mm up, 0.25 mm tall: over twice its own height
        }
    }
    REQUIRE(plate_slabs > 0);
    REQUIRE(first_layer_up > 0);
    const SupportAnalysis::Report floating =
        SupportAnalysis::measure(thin.object(), MiniatureSupport::Problem(), lifted_first);
    REQUIRE(floating.stability.available);
    // The support that is left hangs a first layer above the plate, so it holds nothing up ...
    REQUIRE(floating.stability.unsupported_paths > 0);
    // ... and none of it is ground: what the object stands on is its own first layer alone.
    const double model_ground_mm2 =
        area(union_ex(thin.object().layers().front()->lslices)) * SCALING_FACTOR * SCALING_FACTOR;
    REQUIRE(model_ground_mm2 > 0.);
    REQUIRE_THAT(floating.stability.bed_footprint_mm2, WithinRel(model_ground_mm2, 1e-9));
}

TEST_CASE("Bed margin holds the sliced centroid inside the footprint the object stands on", "[MiniatureContacts]")
{
    // A skirt on this leg: it is printed material on the first layer that touches nothing the object
    // stands on, so it may not widen the footprint by a square millimetre.
    AnalysisRun run;
    run_analysis(run, fin_fixture(),
                 fixture_config({ { "support_style", "tree_slim" }, { "support_top_z_distance", "0.2" },
                                  { "skirt_loops", "3" }, { "skirt_distance", "6" } }));
    const SupportAnalysis::Report *report = run.report();
    REQUIRE(report != nullptr);
    REQUIRE(report->stability.available);

    const PrintObject &po = run.object();
    REQUIRE(! po.layers().empty());
    // The explanatory height the margin is normalized by is the sliced volume's own extent.
    const double expected_height = po.layers().back()->print_z - (po.layers().front()->print_z - po.layers().front()->height);
    REQUIRE(expected_height > 0.);
    REQUIRE_THAT(report->stability.object_height_mm, WithinRel(expected_height, 1e-9));

    // The ground the object stands on: its own first layer plus every support slab that reaches the
    // plate, taken off the emitted record the measurement reads.
    const std::shared_ptr<const SupportAnalysis::EmittedSupport> emitted = po.emitted_support();
    REQUIRE(emitted != nullptr);
    ExPolygons ground = po.layers().front()->lslices;
    size_t     bed_slabs = 0;
    for (const SupportAnalysis::EmittedLayer &layer : emitted->layers)
        if (layer.emitted_available && layer.bottom_z <= EPSILON) {
            append(ground, layer.emitted);
            ++ bed_slabs;
        }
    REQUIRE(bed_slabs > 0);
    const ExPolygons connected = union_ex(ground);
    const double     ground_mm2 = area(connected) * SCALING_FACTOR * SCALING_FACTOR;
    REQUIRE(ground_mm2 > 0.);
    REQUIRE_THAT(report->stability.bed_footprint_mm2, WithinRel(ground_mm2, 1e-9));

    // The skirt printed real ground of its own on that same first layer, and none of it counted.
    Polygons skirt_ground = run.print.skirt().polygons_covered_by_width(0.f);
    REQUIRE(! skirt_ground.empty());
    REQUIRE(area(union_ex(skirt_ground)) > 0.);
    ExPolygons with_skirt = connected;
    append(with_skirt, union_ex(skirt_ground));
    REQUIRE(area(union_ex(with_skirt)) * SCALING_FACTOR * SCALING_FACTOR > ground_mm2 + 1e-6);

    // The centroid of the sliced volume, in the object's own frame: area-weighted by layer height.
    double weight = 0.;
    Vec2d  moment(0., 0.);
    for (const Layer *layer : po.layers())
        for (const ExPolygon &slice : layer->lslices) {
            const double contour_area = slice.contour.area() * layer->height;
            weight += contour_area;
            moment += contour_area * slice.contour.centroid().cast<double>();
            for (const Polygon &hole : slice.holes) {
                const double hole_area = hole.area() * layer->height;
                weight += hole_area;
                moment += hole_area * hole.centroid().cast<double>();
            }
        }
    REQUIRE(weight > 0.);
    const Point centroid(Vec2d(moment / weight));

    // The fin leans clear of its column, so the column's own first layer does not hold the centroid
    // over it: what makes the margin positive is the support standing on the plate under the fin.
    REQUIRE_FALSE(Geometry::convex_hull(po.layers().front()->lslices).contains(centroid));
    const Polygon hull = Geometry::convex_hull(connected);
    REQUIRE(hull.contains(centroid));
    REQUIRE(report->stability.min_bed_margin > 0.);

    // And it is that distance, over the object's height: a length divided by a length.
    double nearest = std::numeric_limits<double>::max();
    for (size_t i = 0; i < hull.points.size(); ++ i)
        nearest = std::min(nearest, Line(hull.points[i], hull.points[(i + 1) % hull.points.size()]).distance_to(centroid));
    REQUIRE_THAT(report->stability.min_bed_margin, WithinRel(nearest * SCALING_FACTOR / expected_height, 1e-6));

    // Had the skirt counted, the hull would have reached out to it and the margin would have grown.
    // It did not: a detached loop of material improves no margin.
    const Polygon skirt_hull = Geometry::convex_hull(union_ex(with_skirt));
    REQUIRE(skirt_hull.contains(centroid));
    double skirt_nearest = std::numeric_limits<double>::max();
    for (size_t i = 0; i < skirt_hull.points.size(); ++ i)
        skirt_nearest = std::min(skirt_nearest,
            Line(skirt_hull.points[i], skirt_hull.points[(i + 1) % skirt_hull.points.size()]).distance_to(centroid));
    REQUIRE(skirt_nearest > nearest + 1e-6);
    REQUIRE(report->stability.min_bed_margin < skirt_nearest * SCALING_FACTOR / expected_height - 1e-6);

    // The same emitted material with its roots widened by a millimetre, which is what a connected
    // root does when it is made wider: the ground the object stands on grows, and a margin measured
    // against a footprint that grew cannot shrink.
    SupportAnalysis::EmittedSupport wider = *emitted;
    size_t                          widened_slabs = 0;
    for (SupportAnalysis::EmittedLayer &layer : wider.layers)
        if (layer.emitted_available && layer.bottom_z <= EPSILON) {
            layer.emitted = union_ex(offset_ex(layer.emitted, scale_(1.)));
            ++ widened_slabs;
        }
    REQUIRE(widened_slabs == bed_slabs);
    const SupportAnalysis::Report widened =
        SupportAnalysis::measure(po, MiniatureSupport::Problem(), wider);
    REQUIRE(widened.stability.available);
    REQUIRE(widened.stability.bed_footprint_mm2 > report->stability.bed_footprint_mm2 + 1e-6);
    REQUIRE(widened.stability.min_bed_margin >= report->stability.min_bed_margin - 1e-9);
}

TEST_CASE("Branch slenderness divides the longest unbraced run by the thinnest printed section", "[MiniatureContacts]")
{
    // The width of a section is the section's own, so a square measures its side and a circle its
    // diameter, while nothing that encloses no area measures anything at all.
    {
        ExPolygon square;
        square.contour.points = { Point(scale_(0.), scale_(0.)), Point(scale_(2.), scale_(0.)),
                                  Point(scale_(2.), scale_(2.)), Point(scale_(0.), scale_(2.)) };
        REQUIRE_THAT(SupportAnalysis::cross_section_width_mm(square), WithinRel(2., 1e-6));
        ExPolygon strip;
        strip.contour.points = { Point(scale_(0.), scale_(0.)), Point(scale_(9.), scale_(0.)),
                                 Point(scale_(9.), scale_(0.5)), Point(scale_(0.), scale_(0.5)) };
        REQUIRE_THAT(SupportAnalysis::cross_section_width_mm(strip), WithinRel(0.5, 1e-6));
        ExPolygon degenerate;
        degenerate.contour.points = { Point(scale_(0.), scale_(0.)), Point(scale_(1.), scale_(0.)) };
        REQUIRE_THAT(SupportAnalysis::cross_section_width_mm(degenerate), WithinAbs(0., 1e-12));
        REQUIRE_THAT(SupportAnalysis::cross_section_width_mm(ExPolygon()), WithinAbs(0., 1e-12));
    }

    const auto stem = [](const char *diameter) {
        return fixture_config({ { "support_style", "tree_slim" }, { "support_top_z_distance", "0.2" },
                                { "tree_support_branch_diameter", diameter } });
    };
    AnalysisRun thick, thin;
    run_analysis(thick, fin_fixture(), stem("5"));
    run_analysis(thin,  fin_fixture(), stem("2"));
    REQUIRE(thick.report() != nullptr);
    REQUIRE(thin.report() != nullptr);
    REQUIRE(thick.report()->stability.available);
    REQUIRE(thin.report()->stability.available);
    REQUIRE(thick.report()->stability.max_slenderness > 0.);

    // The same fixture at the same height, its stem narrowed: the runs are as long and the sections
    // they run over are thinner, so slenderness rises.
    REQUIRE_THAT(thin.report()->stability.object_height_mm,
                 WithinRel(thick.report()->stability.object_height_mm, 1e-9));
    REQUIRE(thin.report()->stability.max_slenderness > thick.report()->stability.max_slenderness);

    // The diameter a routed area carries is the width of the section that was drawn for it, not the
    // radius the router planned for its node.
    const std::shared_ptr<const SupportAnalysis::EmittedSupport> emitted = thin.object().emitted_support();
    REQUIRE(emitted != nullptr);
    size_t sections = 0;
    for (const SupportAnalysis::EmittedLayer &layer : emitted->layers)
        for (const SupportAnalysis::AttributedArea &area : layer.areas) {
            REQUIRE_THAT(area.min_diameter_mm, WithinRel(SupportAnalysis::cross_section_width_mm(area.area), 1e-9));
            if (area.min_diameter_mm > 0.)
                ++ sections;
        }
    REQUIRE(sections > 0);

    // In-generation the footprint is the object's own plus the roots the generator laid: under a raft
    // the object's first layer is up on the raft, and what stands on the plate is the raft.
    // The report is taken where the support is generated and posSimplifySupportPath rewrites those
    // paths afterwards, so this leg turns path simplification off and reads the same extrusions twice.
    AnalysisRun rafted;
    run_analysis(rafted, fin_fixture(),
                 fixture_config({ { "support_style", "tree_slim" }, { "support_top_z_distance", "0.2" },
                                  { "raft_layers", "3" }, { "enable_arc_fitting", "0" }, { "resolution", "0" } }));
    REQUIRE(rafted.report() != nullptr);
    REQUIRE(rafted.report()->stability.available);
    const PrintObject &rafted_object = rafted.object();
    REQUIRE(rafted_object.support_raft_layers() >= 3);
    const Layer *first = rafted_object.layers().front();
    REQUIRE(first->print_z - first->height > 0.5 * first->height);   // the object starts up on the raft
    ExPolygons raft_ground;
    size_t     raft_index = 0;
    for (const SupportLayer *sl : rafted_object.support_layers()) {
        if (raft_index ++ >= rafted_object.support_raft_layers())
            break;
        if (sl->print_z - sl->height <= 0.5 * sl->height)
            append(raft_ground, union_ex(sl->support_fills.polygons_covered_by_width(0.f)));
    }
    REQUIRE(! raft_ground.empty());
    // 1e-5 relative, the bound the volume leg uses for the same reason: with arc fitting off and the
    // resolution at zero, simplification still moves these paths by about a millionth of themselves.
    REQUIRE_THAT(rafted.report()->stability.bed_footprint_mm2,
                 WithinRel(area(union_ex(raft_ground)) * SCALING_FACTOR * SCALING_FACTOR, 1e-5));
}

TEST_CASE("The support warning names a printable critical region without material or a group with no root", "[MiniatureContacts]")
{
    // One measurement read on its own terms, with nothing to compare it against: what leaves a slice
    // unresolved is an overhang the detector filed as wide enough for one support extrusion, marked
    // critical, that no printed material reached, or a routed group that never printed at all. A
    // region narrower than an extrusion carries no witness lattice and is no need here either, which
    // the measurement states on the region as `printable` rather than leaving a reader to re-derive.
    const auto measured = [](bool printable, bool critical, bool emitted_path) {
        SupportAnalysis::RegionCoverage region;
        region.region_id    = 0;
        region.printable    = printable;
        region.critical     = critical;
        region.emitted_path = emitted_path;
        SupportAnalysis::Report report;
        report.key.region_ids     = { 0 };
        report.coverage           = { region };
        report.coverage_available = true;
        return report;
    };

    // A printable critical region no material reached: the overhang that needs support got none.
    REQUIRE(SupportAnalysis::support_unresolved(measured(true, true, false)));
    // The same region too narrow for one extrusion: a sliver the detector filed, carried by the wall
    // beside it whether or not a branch reached it.
    REQUIRE_FALSE(SupportAnalysis::support_unresolved(measured(false, true, false)));
    // Material reached every region and a routed group still never printed: support in mid-air.
    {
        SupportAnalysis::Report report    = measured(true, true, true);
        report.stability.unrooted_groups  = 1;
        REQUIRE(SupportAnalysis::support_unresolved(report));
    }
    // A measurement that required nothing leaves nothing open: an empty problem is not an unmet one.
    REQUIRE_FALSE(SupportAnalysis::support_unresolved(SupportAnalysis::Report()));
    // A region no material reached that the detector never marked critical is not a need either.
    REQUIRE_FALSE(SupportAnalysis::support_unresolved(measured(true, false, false)));
}

TEST_CASE("A branch roots on the model only where the settings allow it", "[MiniatureContacts]")
{
    const auto box = [](const char *plate_only) {
        return fixture_config({ { "support_style", "tree_slim" }, { "support_top_z_distance", "0.2" },
                                { "support_on_build_plate_only", plate_only } });
    };
    AnalysisRun anywhere, plate_only;
    run_analysis(anywhere,   box_fixture(), box("0"));
    run_analysis(plate_only, box_fixture(), box("1"));
    REQUIRE(anywhere.report() != nullptr);
    REQUIRE(anywhere.object().config().support_on_build_plate_only.value == false);
    REQUIRE(plate_only.object().config().support_on_build_plate_only.value == true);

    // Nothing the pass emitted inside the box reaches the plate: the walls give a branch no way out,
    // so the lowest material it laid is up on the floor at z 2, over the gap the settings leave.
    const std::shared_ptr<const SupportAnalysis::EmittedSupport> emitted = anywhere.object().emitted_support();
    REQUIRE(emitted != nullptr);
    double lowest = std::numeric_limits<double>::max();
    for (const SupportAnalysis::EmittedLayer &layer : emitted->layers)
        if (layer.emitted_available && ! layer.emitted.empty())
            lowest = std::min(lowest, layer.bottom_z);
    REQUIRE(lowest > 2.);

    // Where the settings permit resting on the object, and the model it rests on is connected to its
    // own first layer, which the plate or a raft carries, those branches are rooted.
    REQUIRE(anywhere.report()->stability.available);

    // The same emitted geometry read under settings that forbid resting on the model: not one
    // millimetre of it differs, and every branch the model was holding is floating. The count can
    // only go up, whatever stray fragment this run of the generator happened to leave behind.
    const SupportAnalysis::Report forbidden =
        SupportAnalysis::measure(plate_only.object(), MiniatureSupport::Problem(), *emitted);
    REQUIRE(forbidden.stability.available);
    REQUIRE(forbidden.stability.unsupported_paths > anywhere.report()->stability.unsupported_paths);

    // The support material a print laid from `z` up, read off the extrusions the measurement reads.
    const auto volume_above = [](const PrintObject &object, double z) {
        double volume = 0.;
        for (const SupportLayer *layer : object.support_layers())
            if (layer->print_z - layer->height >= z - EPSILON)
                volume += layer->support_fills.total_volume();
        return volume;
    };
    // The centred bed keeps draw_circles' machine-border clip off branches that cross x 0.
    const DynamicPrintConfig centred = fixture_config({ { "support_style", "tree_slim" }, { "support_top_z_distance", "0.2" },
                                                        { "support_on_build_plate_only", "0" },
                                                        { "printable_area", "-100x-100,100x-100,100x100,-100x100" } });

    // A platform caged in the box, connected to nothing below it. A stock slice keeps every branch the
    // generator laid on it; a pass that measures itself takes them out, because model material that is
    // not connected to the object's first layer holds nothing up.
    AnalysisRun island_stock, island_measured;
    run_analysis(island_stock, caged_island_fixture(), centred, false);
    run_analysis(island_measured, caged_island_fixture(), centred);
    REQUIRE(island_stock.report() == nullptr);
    REQUIRE(volume_above(island_stock.object(), 7.) > 0.);
    REQUIRE(island_measured.report() != nullptr);
    CHECK_THAT(volume_above(island_measured.object(), 7.), WithinAbs(0., 1e-9));

    // The box on a raft, measured so the floating pass runs: its floor is part of the object's first
    // layer, which the raft carries, so the branches resting on the floor stand and survive the pass.
    const DynamicPrintConfig rafted = fixture_config({ { "support_style", "tree_slim" }, { "support_top_z_distance", "0.2" },
                                                       { "support_on_build_plate_only", "0" }, { "raft_layers", "3" },
                                                       { "printable_area", "-100x-100,100x-100,100x100,-100x100" } });
    AnalysisRun measured;
    run_analysis(measured, box_fixture(), rafted);
    REQUIRE(measured.report() != nullptr);
    REQUIRE(measured.object().support_raft_layers() >= 3);
    REQUIRE(measured.report()->stability.available);
    // Everything from the object's first layer up, so the raft is left out.
    REQUIRE(volume_above(measured.object(), measured.object().slicing_parameters().object_print_z_min) > 0.);
}

TEST_CASE("A disabled print measures itself only when asked, and lets the measurement go with its slice", "[MiniatureContacts]")
{
    const DynamicPrintConfig config = fixture_config({ { "support_style", "tree_slim" }, { "support_top_z_distance", "0.2" } });

    // Leg 1, ordinary disabled slicing: nothing asked for a measurement, so the pass carried no
    // provenance, recorded no attributed area and left no report. It still generated its support.
    AnalysisRun plain;
    run_analysis(plain, fin_fixture(), config, false);
    REQUIRE(plain.object().support_layers().size() > 0);
    REQUIRE(plain.object().support_analysis() == nullptr);
    REQUIRE(plain.object().emitted_support() == nullptr);

    // Leg 2, the same disabled print with the analysis asked for: the same geometry, contact for
    // contact and extrusion for extrusion, with a measurement of it beside it.
    AnalysisRun asked;
    run_analysis(asked, fin_fixture(), config, true);
    REQUIRE(asked.report() != nullptr);
    REQUIRE(asked.object().support_layers().size() == plain.object().support_layers().size());
    REQUIRE(same_clusters(contact_clusters(asked.object()), contact_clusters(plain.object())));
    REQUIRE(same_roles(role_totals(asked.object()), role_totals(plain.object())));
    REQUIRE(same_annotations(annotations(asked.object()), annotations(plain.object())));

    // Stability and removal damage are both measured off this pass, so neither carries an unavailable
    // reason and neither is a zero standing in for a measurement that was never taken.
    REQUIRE(asked.report()->stability.available);
    REQUIRE_FALSE(asked.report()->has_reason(SupportAnalysis::Reason::StabilityUnavailable));
    REQUIRE(asked.report()->damage.available);
    REQUIRE_FALSE(asked.report()->has_reason(SupportAnalysis::Reason::DamageUnavailable));

    // Leg 3, organic: the analysis belongs to the legacy tree generator, and asking adds no work to
    // the organic one, which produces its support and no measurement.
    AnalysisRun organic;
    run_analysis(organic, fin_fixture(),
                 fixture_config({ { "support_style", "organic" }, { "support_top_z_distance", "0.2" } }), true);
    REQUIRE(organic.object().support_layers().size() > 0);
    REQUIRE(organic.object().support_analysis() == nullptr);
    REQUIRE(organic.object().emitted_support() == nullptr);

    // Leg 4, the shared-object journey. Two objects off one mesh: one owns the pass and the other
    // reads it, so it reads the owner's measurement of it too.
    DynamicPrintConfig full = DynamicPrintConfig::full_print_config();
    full.apply(config);

    Slic3r::Model model;
    ModelObject  *first = model.add_object();
    first->name         = "shared_object.stl";
    first->add_volume(fin_fixture());
    first->config.set_key_value("extruder", new ConfigOptionInt(1));
    first->add_instance()->set_offset(Vec3d(100., 100., 0.));
    ModelObject *clone = model.add_object(*first);
    clone->instances.front()->set_offset(Vec3d(140., 100., 0.));
    for (ModelObject *mo : model.objects)
        mo->ensure_on_bed();

    Slic3r::Print print;
    print.set_status_silent();
    print.apply(model, full);
    print.request_legacy_support_analysis();
    print.process();
    REQUIRE(print.objects().size() == 2);
    REQUIRE(print.objects()[1]->get_shared_object() == print.objects()[0]);
    REQUIRE(print.objects()[0]->support_analysis() != nullptr);
    REQUIRE(print.objects()[1]->support_analysis() == print.objects()[0]->support_analysis());

    // A slice invalidation takes the geometry the measurement was taken against, so it takes the
    // measurement with it. xy_contour_compensation invalidates posSlice alone, and the support step
    // is reached only through that cascade, which is the path a retained report has to travel.
    DynamicPrintConfig resliced = full;
    resliced.set_deserialize_strict({ { "xy_contour_compensation", "0.05" } });
    REQUIRE_THAT(resliced.opt_float("xy_contour_compensation"), WithinAbs(0.05, 1e-9));
    print.apply(model, resliced);
    for (const PrintObject *po : print.objects()) {
        REQUIRE_FALSE(po->is_step_done(posSupportMaterial));
        REQUIRE(po->support_analysis() == nullptr);
        REQUIRE(po->emitted_support() == nullptr);
        REQUIRE(po->support_layers().empty());
    }

    // And one request buys one generation: the pass that honoured it cleared it, so the reprocess
    // regenerates the support and measures nothing.
    print.process();
    for (const PrintObject *po : print.objects()) {
        REQUIRE(po->is_step_done(posSupportMaterial));
        REQUIRE(po->support_layers().size() > 0);
        REQUIRE(po->support_analysis() == nullptr);
    }
}



TEST_CASE("An overhang no branch can reach warns the user, and one every branch reaches does not", "[MiniatureContacts]")
{
    // The closed box's ceiling is an overhang its own walls give a branch no way out of, so under
    // settings that forbid resting on the model the generated pass places no support for it at all:
    // the requirement stays open, and the condition reaches the user as a non-critical warning on the
    // step that is still active.
    const auto box = [](const char *plate_only) {
        return fixture_config({ { "support_style", "tree_slim" }, { "support_top_z_distance", "0.2" },
                                { "support_on_build_plate_only", plate_only },
                                { "support_miniature_contacts", "1" }, { "support_contact_min_distance", "1" } });
    };
    AnalysisRun impossible;
    run_analysis(impossible, box_fixture(), box("1"), false);
    REQUIRE(impossible.report() != nullptr);
    REQUIRE(impossible.report()->key.region_ids.size() == 1);
    REQUIRE(impossible.object().support_layers().empty());
    REQUIRE(impossible.object().is_step_done(posSupportMaterial));

    const PrintStateBase::StateWithWarnings state = impossible.object().step_state_with_warnings(posSupportMaterial);
    REQUIRE(state.warnings.size() == 1);
    REQUIRE(state.warnings.front().level == PrintStateBase::WarningLevel::NON_CRITICAL);
    // The default notification id: the message identifies itself, and no unrelated id is borrowed for it.
    REQUIRE(state.warnings.front().message_id == int(PrintStateBase::SlicingDefaultNotification));
    REQUIRE_FALSE(state.warnings.front().message.empty());
    // And the predicate the warning is posted on, read on the very report the object holds: the
    // condition and the site that acts on it are pinned together rather than each on its own fixture.
    REQUIRE(SupportAnalysis::support_unresolved(*impossible.report()));

    // The same box under settings that let a branch rest on the model: the ceiling requirement the
    // plate-only run left open is met here, so what that run stopped on was the requirement and not
    // the analysis marking every box unresolved. The one required region is covered, its anchor is
    // reached, and this pass's own coverage is measured, which leaves the admissibility of the paths
    // it laid as the only condition that could still leave this run open.
    AnalysisRun reachable;
    run_analysis(reachable, box_fixture(), box("0"), false);
    REQUIRE(reachable.report() != nullptr);
    REQUIRE(reachable.object().support_layers().size() > 0);
    REQUIRE(reachable.report()->key.region_ids.size() == 1);
    REQUIRE(reachable.report()->coverage_available);
    REQUIRE(reachable.report()->coverage.size() == 1);
    REQUIRE(reachable.report()->coverage.front().critical);
    REQUIRE(reachable.report()->coverage.front().covered_count() > 0);
    REQUIRE(reachable.report()->coverage.front().anchored);
    REQUIRE(reachable.report()->missing_anchor_ids.empty());
    // The generator takes the extrusions resting on nothing out of its toolpaths after
    // generate_toolpaths (remove_floating_toolpaths), so the paths this pass laid stay rooted run
    // after run of a generator that is otherwise not reproducible (AGENTS.md "Testing").
    REQUIRE(reachable.report()->stability.available);
    REQUIRE(reachable.report()->stability.unsupported_paths == 0);
    REQUIRE(reachable.report()->stability.unrooted_groups == 0);
    REQUIRE(reachable.report()->stability.min_bed_margin >= 0.);
    REQUIRE_FALSE(SupportAnalysis::support_unresolved(*reachable.report()));
    REQUIRE(reachable.object().step_state_with_warnings(posSupportMaterial).warnings.empty());
}

TEST_CASE("The generated pass ships as one pass and a support blocker stays authoritative", "[MiniatureContacts]")
{
    // Leg 1, the widest contact distance this fixture allows. The lip band is one required region
    // 25 mm long, and the material that reaches it comes down a corridor between the base wall and the
    // band's own overhang. Placement moves the contacts onto broader model material and prints less
    // for them, and at this distance one non-critical contact then stands where another already
    // carries every cell it held, so it goes: that is the pass the object ends up holding.
    AnalysisRun retained;
    run_analysis(retained, lip_fixture(),
                 fixture_config({ { "support_style", "tree_slim" }, { "support_top_z_distance", "0.2" },
                                  { "support_miniature_contacts", "1" }, { "support_contact_min_distance", "5" } }),
                 false);
    REQUIRE(retained.report() != nullptr);

    // What shipped still reaches every critical anchor the problem named, a non-critical contact the
    // thinning takes being the whole point of the mode, and the band carries covered material of its own.
    const std::vector<uint64_t> &critical = retained.report()->critical_anchor_ids;
    for (uint64_t id : retained.report()->missing_anchor_ids) {
        INFO("anchor " << id << " was not reached");
        REQUIRE_FALSE(std::binary_search(critical.begin(), critical.end(), id));
    }
    REQUIRE(retained.report()->coverage.size() == 1);
    REQUIRE(retained.report()->coverage.front().covered_count() > 0);

    // And it shipped as one pass: the support layers on the object, the emitted record the report
    // was measured from and the report itself all describe that pass. The measurement is taken
    // before posSimplifySupportPath rewrites the paths, so the volumes agree to within that.
    require_valid_support_layers(retained.object(), retained.object());
    REQUIRE(retained.object().support_layers().size() > 0);
    REQUIRE(retained.object().emitted_support() != nullptr);
    REQUIRE_THAT(support_metrics(retained.object()).volume_mm3,
                 WithinRel(retained.report()->support_volume_mm3 + retained.report()->raft_volume_mm3, 0.01));

    // Leg 2, the same fixture with a support blocker over the lip band. A blocker is the user's own
    // instruction and the pass reads it from the one preparation, so no support is placed under the
    // band at all. Nothing was required, so nothing is left unresolved and the user is told nothing.
    const auto blocked_run = [](bool blocked) {
        DynamicPrintConfig full = DynamicPrintConfig::full_print_config();
        full.apply(fixture_config({ { "support_style", "tree_slim" }, { "support_top_z_distance", "0.2" },
                                    { "support_miniature_contacts", "1" }, { "support_contact_min_distance", "1" } }));

        Slic3r::Model model;
        ModelObject  *mo = model.add_object();
        mo->name         = "lip.stl";
        mo->add_volume(lip_fixture());
        if (blocked) {
            // A 40 x 6 x 3 mm slab spanning x -5..35, y 6..12, z 6.5..9.5 in the fixture's own frame:
            // it covers the band at z 8.0..8.2 and stops below the base's top face at z 10.
            TriangleMesh slab = make_cube(40., 6., 3.);
            slab.translate(-5.f, 6.f, 6.5f);
            mo->add_volume(std::move(slab), ModelVolumeType::SUPPORT_BLOCKER, false);
        }
        mo->config.set_key_value("extruder", new ConfigOptionInt(1));
        mo->add_instance()->set_offset(Vec3d(100., 100., 0.));
        mo->ensure_on_bed();

        auto print = std::make_unique<Slic3r::Print>();
        print->set_status_silent();
        print->apply(model, full);
        print->process();
        return print;
    };

    const std::unique_ptr<Slic3r::Print> open    = blocked_run(false);
    const std::unique_ptr<Slic3r::Print> blocked = blocked_run(true);
    const PrintObject                   &open_po    = *open->objects().front();
    const PrintObject                   &blocked_po = *blocked->objects().front();
    REQUIRE(open_po.support_analysis() != nullptr);
    REQUIRE(blocked_po.support_analysis() != nullptr);

    // Without the blocker the band is a required region and it is supported; with it, neither.
    REQUIRE(open_po.support_analysis()->key.region_ids.size() == 1);
    REQUIRE(support_metrics(open_po).volume_mm3 > 0.);
    REQUIRE(blocked_po.support_analysis()->key.region_ids.empty());
    REQUIRE_THAT(support_metrics(blocked_po).volume_mm3, WithinAbs(0., 1e-9));
    REQUIRE(blocked_po.step_state_with_warnings(posSupportMaterial).warnings.empty());
}

TEST_CASE("A corpus row reads every object the case selected rather than whichever came first", "[MiniatureContacts]")
{
    // A manifest case selects objects, plural: `selectors` is an array and `instance_ids` indexes a
    // whole file. A row read off print.objects().front() would hand the gate the first object's
    // numbers and drop what every other object measured, so a second object's open requirement, its
    // missing anchors and its risk would never reach the validator.
    const DynamicPrintConfig config =
        fixture_config({ { "support_style", "tree_slim" }, { "support_top_z_distance", "0.2" },
                         { "support_on_build_plate_only", "0" },
                         { "support_miniature_contacts", "1" }, { "support_contact_min_distance", "1" } });

    std::vector<TriangleMesh> meshes;
    meshes.emplace_back(box_fixture());
    meshes.emplace_back(lip_fixture());
    Slic3r::Model model;
    Slic3r::Print print;
    init_print(std::move(meshes), print, model, config);
    print.request_legacy_support_analysis();
    print.process();

    REQUIRE(print.objects().size() == 2);
    const SupportAnalysis::Report *first  = print.objects()[0]->support_analysis().get();
    const SupportAnalysis::Report *second = print.objects()[1]->support_analysis().get();
    REQUIRE(first != nullptr);
    REQUIRE(second != nullptr);
    // What makes the two objects worth reading separately: the second object leaves anchors of its
    // own unreached. Which of the two the row describes is the whole point.
    REQUIRE_FALSE(second->missing_anchor_ids.empty());

    SupportValidation::CaseResult row;
    SupportValidation::read_print_analyses(print, row);

    // The open requirement the second object carries is the row's outcome, not a detail the first
    // object's clean result hides.
    CHECK(row.status == SupportValidation::worst_outcome(SupportValidation::outcome_of(*first), SupportValidation::outcome_of(*second)));

    const SupportValidation::Metrics one = SupportValidation::metrics_of(*first);
    const SupportValidation::Metrics two = SupportValidation::metrics_of(*second);
    CHECK_THAT(row.metrics.support_volume_mm3, WithinAbs(one.support_volume_mm3 + two.support_volume_mm3, 1e-9));
    CHECK_THAT(row.metrics.raft_volume_mm3, WithinAbs(one.raft_volume_mm3 + two.raft_volume_mm3, 1e-9));
    CHECK(row.metrics.missing_critical_anchors == one.missing_critical_anchors + two.missing_critical_anchors);
    CHECK(row.metrics.invalid_paths == one.invalid_paths + two.invalid_paths);
    CHECK(row.metrics.unrooted_groups == one.unrooted_groups + two.unrooted_groups);
    CHECK(row.metrics.unknown_contacts == one.unknown_contacts + two.unknown_contacts);
    CHECK(row.metrics.inaccessible_groups == one.inaccessible_groups + two.inaccessible_groups);
    CHECK_THAT(row.metrics.total_group_risk, WithinAbs(one.total_group_risk + two.total_group_risk, 1e-9));
    CHECK_THAT(row.metrics.max_group_risk, WithinAbs(std::max(one.max_group_risk, two.max_group_risk), 1e-9));
    CHECK_THAT(row.metrics.max_slenderness, WithinAbs(std::max(one.max_slenderness, two.max_slenderness), 1e-9));
    CHECK_THAT(row.metrics.min_bed_margin, WithinAbs(std::min(one.min_bed_margin, two.min_bed_margin), 1e-9));
    // A domain one object never measured is unavailable for the row: an unmeasured object is not a
    // result of zero.
    CHECK(row.metrics.coverage_available == (one.coverage_available && two.coverage_available));
    CHECK(row.metrics.stability_available == (one.stability_available && two.stability_available));
    CHECK(row.metrics.damage_available == (one.damage_available && two.damage_available));

    for (SupportAnalysis::Reason reason : second->reasons)
        CHECK(std::find(row.reason_codes.begin(), row.reason_codes.end(), std::string(SupportValidation::reason_name(reason))) != row.reason_codes.end());
}

// Hidden ([.]): one corpus model costs two full Print::process() passes on a miniature at fine
// layers, minutes each, and the corpus it reads lives outside the repo under
// $ORCA_MINIATURE_CORPUS (docs/miniature_support_validation.md). It measures how far decimation thins the contact
// set, model by model; it does not pin it.
TEST_CASE("Miniature contact decimation over a corpus", "[MiniatureContacts][.]")
{
    SupportValidation::use_os_temporary_dir();

    const char       *env        = std::getenv("ORCA_MINIATURE_CORPUS");
    const std::string corpus_dir = env != nullptr ? std::string(env) : std::string();

    // A manifest turns the harness from a diagnostic into an acceptance run: every case it declares
    // is sliced under every style, feature mode and repeat it declares, and each of those writes one
    // row to $ORCA_MINIATURE_RESULTS for scripts/validate_miniature_supports.py to hold to the
    // manifest. Without one the harness still prints the fin, and that run is a diagnostic the
    // acceptance command rejects.
    const char *manifest_env = std::getenv("ORCA_MINIATURE_MANIFEST");
    if (manifest_env != nullptr && *manifest_env != '\0') {
        const SupportValidation::Manifest manifest = SupportValidation::load_manifest(manifest_env);
        const char                       *results  = std::getenv("ORCA_MINIATURE_RESULTS");
        REQUIRE(results != nullptr);
        std::ofstream rows(results);
        REQUIRE(rows.good());
        for (const SupportValidation::ManifestCase &entry : manifest.cases)
            for (const std::string &style : entry.styles)
                for (const std::string &mode : entry.feature_modes)
                    for (size_t repeat = 0; repeat < entry.repeats; ++ repeat) {
                        SupportValidation::CaseResult row = SupportValidation::measure_case(
                            manifest, entry, fixture_config({ { "support_top_z_distance", "0.2" } }), style, mode, repeat);
                        row.harness = "miniature_contacts";
                        SupportValidation::write_result(row, rows);
                        std::cout << "case " << entry.id << " " << style << "/" << mode << " repeat " << repeat
                                  << " status=" << SupportValidation::outcome_name(row.status)
                                  << " support_mm3=" << row.metrics.support_volume_mm3
                                  << " elapsed_s=" << row.elapsed_s << std::endl;
                    }
        rows.close();
        REQUIRE(rows.good());
        return;
    }

    // Model 0 is always the built-in fin fixture, under the two settings "A requested support analysis
    // names required regions and contact seeds by value" states: tree_slim, because the organic
    // generator never fills roof_gap_areas, and support_top_z_distance at its PrintConfig default of
    // 0.2, so the gap the tips are planned against is a stated number.
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

    SupportValidation::for_each_corpus_object(corpus_dir, fixture_config({ { "support_style", "tree_slim" },
        { "support_top_z_distance", "0.2" } }), 1,
        [&](size_t index, const SupportValidation::CorpusObject &object) {
            // roof_gap_areas is the gap the generator leaves between a contact tip and the object, so
            // a config that prints the tips straight onto the object files none of it: both modes
            // would measure an empty set, whatever the contact selection did.
            if (object.config.opt_float("support_top_z_distance") <= 0.) {
                std::cout << "model " << index << " " << object.stem
                          << " skipped: support_top_z_distance is 0, roof_gap_areas would be empty" << std::endl;
                return;
            }
            report(index, object.stem, corpus_dir, [&](bool on) {
                DynamicPrintConfig mode_config = object.config;
                mode_config.set_deserialize_strict({ { "support_miniature_contacts", on ? "1" : "0" } });
                Slic3r::Print print;
                print.set_status_silent();
                print.apply(object.model, mode_config);
                print.process();
                return contact_clusters(*print.objects().front());
            });
        });
}
