#include <unordered_set>
#include <unordered_map>
#include <random>
#include <numeric>
#include <cstdint>

#include "sla_test_utils.hpp"

#include <libslic3r/TriangleMeshSlicer.hpp>
#include <libslic3r/SLA/SupportTreeMesher.hpp>
#include <libslic3r/SLA/Concurrency.hpp>

namespace {

const char *const BELOW_PAD_TEST_OBJECTS[] = {
    "20mm_cube.obj",
    "V.obj",
};

const char *const AROUND_PAD_TEST_OBJECTS[] = {
    "20mm_cube.obj",
    "V.obj",
    "frog_legs.obj",
    "cube_with_concave_hole_enlarged.obj",
};

const char *const SUPPORT_TEST_MODELS[] = {
    "cube_with_concave_hole_enlarged_standing.obj",
    "A_upsidedown.obj",
    "extruder_idler.obj"
};

// A 40x40x4 block at x 0..40, y 0..40, z 0..4 with a 4x4x10 column at
// x 18..22, y 0..4, z 4..14 carrying a 4x40x2 lip at x 18..22, y 0..40,
// z 12..14. A point under the lip at (20, 20, 12) faces the block top and sits
// 20 mm from every block edge, beyond a 10 mm bridge, so its head reaches
// neither a pillar nor the ground.
indexed_triangle_set blocked_lip_mesh()
{
    auto box = [](double x, double y, double z, float dx, float dy, float dz) {
        TriangleMesh m = make_cube(x, y, z);
        m.translate(dx, dy, dz);
        return m;
    };

    indexed_triangle_set its = box(40., 40., 4., 0.f, 0.f, 0.f).its;
    its_merge(its, box(4., 4., 10., 18.f, 0.f, 4.f).its);
    its_merge(its, box(4., 40., 2., 18.f, 0.f, 12.f).its);

    return its;
}

sla::SupportTreeConfig blocked_lip_config()
{
    sla::SupportTreeConfig cfg;
    cfg.object_elevation_mm         = 0.;
    cfg.max_bridge_length_mm        = 10.;
    cfg.max_pillar_link_distance_mm = 10.;

    return cfg;
}

const sla::SupportPoints BLOCKED_LIP_POINTS = {sla::SupportPoint(Vec3f(20.f, 20.f, 12.f), 0.2f)};

// A 2x40x30 wall at x 0..2, y 0..40, z 0..30, a 0.2 mm ground plate at
// x 4.5..plate_x_end, y 0..40, and a 6x6x2 slab at x 2..8, y 17..23, z h..h+2
// off the wall's +x face. A point under the slab at (3, 20, h) walks toward +x
// to reach the ground, and the plate's footprint sets how far that walk must
// run before its floor point clears the pillar base gap, so the walk's length
// is decided by the corrector cap rather than by the wall.
indexed_triangle_set corrector_sweep_mesh(double h, double plate_x_end)
{
    auto box = [](double x, double y, double z, float dx, float dy, float dz) {
        TriangleMesh m = make_cube(x, y, z);
        m.translate(dx, dy, dz);
        return m;
    };

    indexed_triangle_set its = box(2., 40., 30., 0.f, 0.f, 0.f).its;
    its_merge(its, box(plate_x_end - 4.5, 40., 0.2, 4.5f, 0.f, 0.f).its);
    its_merge(its, box(6., 6., 2., 2.f, 17.f, float(h)).its);

    return its;
}

// A 10x10x1 plate at x 0..10, y 0..10, z 30..31, a 20x10x1 plate at
// x 30..50, y 0..10, z 30..31 and a 2x2x1 foot at x 60..62, y 0..2, z 0..1
// that pins the ground at z 0. TWO_PLATES_POINTS puts two points 5 mm apart
// under the small plate and two 16 mm apart under the long one.
indexed_triangle_set two_plates_mesh()
{
    auto box = [](double x, double y, double z, float dx, float dy, float dz) {
        TriangleMesh m = make_cube(x, y, z);
        m.translate(dx, dy, dz);
        return m;
    };

    indexed_triangle_set its = box(10., 10., 1., 0.f, 0.f, 30.f).its;
    its_merge(its, box(20., 10., 1., 30.f, 0.f, 30.f).its);
    its_merge(its, box(2., 2., 1., 60.f, 0.f, 0.f).its);

    return its;
}

sla::SupportTreeConfig two_plates_config(double slenderness)
{
    sla::SupportTreeConfig cfg;
    cfg.object_elevation_mm         = 0.;
    cfg.head_back_radius_mm         = 0.6;
    cfg.base_radius_mm              = 1.2;
    cfg.base_height_mm              = 0.5;
    cfg.max_bridge_length_mm        = 12.;
    cfg.max_pillar_link_distance_mm = 12.;
    cfg.pillar_connection_mode      = sla::PillarConnectionMode::zigzag;
    cfg.allow_model_anchors         = false;
    cfg.pillar_link_slenderness     = slenderness;

    return cfg;
}

const sla::SupportPoints TWO_PLATES_POINTS = {
    sla::SupportPoint(Vec3f(2.5f, 5.f, 30.f), 0.2f),
    sla::SupportPoint(Vec3f(7.5f, 5.f, 30.f), 0.2f),
    sla::SupportPoint(Vec3f(32.f, 5.f, 30.f), 0.2f),
    sla::SupportPoint(Vec3f(48.f, 5.f, 30.f), 0.2f),
};

} // namespace

TEST_CASE("Pillar pairhash should be unique", "[SLASupportGeneration]") {
    test_pairhash<int, int>();
    test_pairhash<int, long>();
    test_pairhash<unsigned, unsigned>();
    test_pairhash<unsigned, unsigned long>();
}

TEST_CASE("Support point generator should be deterministic if seeded", 
          "[SLASupportGeneration], [SLAPointGen]") {
    TriangleMesh mesh = load_model("A_upsidedown.obj");
    
    sla::IndexedMesh emesh{mesh};
    
    sla::SupportTreeConfig supportcfg;
    sla::SupportPointGenerator::Config autogencfg;
    autogencfg.head_diameter = float(2 * supportcfg.head_front_radius_mm);
    sla::SupportPointGenerator point_gen{emesh, autogencfg, [] {}, [](int) {}};
        
    auto   bb      = mesh.bounding_box();
    double zmin    = bb.min.z();
    double zmax    = bb.max.z();
    double gnd     = zmin - supportcfg.object_elevation_mm;
    auto   layer_h = 0.05f;
    
    auto slicegrid = grid(float(gnd), float(zmax), layer_h);
    std::vector<ExPolygons> slices = slice_mesh_ex(mesh.its, slicegrid, CLOSING_RADIUS);
    
    point_gen.seed(0);
    point_gen.execute(slices, slicegrid);
    
    auto get_chksum = [](const std::vector<sla::SupportPoint> &pts){
        int64_t chksum = 0;
        for (auto &pt : pts) {
            auto p = scaled(pt.pos);
            chksum += p.x() + p.y() + p.z();
        }
        
        return chksum;
    };
    
    int64_t checksum = get_chksum(point_gen.output());
    size_t ptnum = point_gen.output().size();
    REQUIRE(point_gen.output().size() > 0);
    
    for (int i = 0; i < 20; ++i) {
        point_gen.output().clear();
        point_gen.seed(0);
        point_gen.execute(slices, slicegrid);
        REQUIRE(point_gen.output().size() == ptnum);
        REQUIRE(checksum == get_chksum(point_gen.output()));
    }
}

TEST_CASE("Flat pad geometry is valid", "[SLASupportGeneration]") {
    sla::PadConfig padcfg;
    
    // Disable wings
    padcfg.wall_height_mm = .0;
    
    for (auto &fname : BELOW_PAD_TEST_OBJECTS) test_pad(fname, padcfg);
}

TEST_CASE("WingedPadGeometryIsValid", "[SLASupportGeneration]") {
    sla::PadConfig padcfg;
    
    // Add some wings to the pad to test the cavity
    padcfg.wall_height_mm = 1.;
    
    for (auto &fname : BELOW_PAD_TEST_OBJECTS) test_pad(fname, padcfg);
}

TEST_CASE("FlatPadAroundObjectIsValid", "[SLASupportGeneration]") {
    sla::PadConfig padcfg;
    
    // Add some wings to the pad to test the cavity
    padcfg.wall_height_mm = 0.;
    // padcfg.embed_object.stick_stride_mm = 0.;
    padcfg.embed_object.enabled = true;
    padcfg.embed_object.everywhere = true;
    
    for (auto &fname : AROUND_PAD_TEST_OBJECTS) test_pad(fname, padcfg);
}

TEST_CASE("WingedPadAroundObjectIsValid", "[SLASupportGeneration]") {
    sla::PadConfig padcfg;
    
    // Add some wings to the pad to test the cavity
    padcfg.wall_height_mm = 1.;
    padcfg.embed_object.enabled = true;
    padcfg.embed_object.everywhere = true;
    
    for (auto &fname : AROUND_PAD_TEST_OBJECTS) test_pad(fname, padcfg);
}

TEST_CASE("ElevatedSupportGeometryIsValid", "[SLASupportGeneration]") {
    sla::SupportTreeConfig supportcfg;
    supportcfg.object_elevation_mm = 10.;
    
    for (auto fname : SUPPORT_TEST_MODELS) test_supports(fname, supportcfg);
}

TEST_CASE("FloorSupportGeometryIsValid", "[SLASupportGeneration]") {
    sla::SupportTreeConfig supportcfg;
    supportcfg.object_elevation_mm = 0;
    
    for (auto &fname: SUPPORT_TEST_MODELS) test_supports(fname, supportcfg);
}

TEST_CASE("ElevatedSupportsDoNotPierceModel", "[SLASupportGeneration]") {
    
    sla::SupportTreeConfig supportcfg;
    
    for (auto fname : SUPPORT_TEST_MODELS)
        test_support_model_collision(fname, supportcfg);
}

TEST_CASE("FloorSupportsDoNotPierceModel", "[SLASupportGeneration]") {
    
    sla::SupportTreeConfig supportcfg;
    supportcfg.object_elevation_mm = 0;
    
    for (auto fname : SUPPORT_TEST_MODELS)
        test_support_model_collision(fname, supportcfg);
}

TEST_CASE("A model-facing head is dropped when model anchors are forbidden", "[SLASupportGeneration]") {
    indexed_triangle_set mesh = blocked_lip_mesh();

    sla::SupportTreeConfig forbidden = blocked_lip_config();
    forbidden.allow_model_anchors = false;
    sla::SupportTreeBuilder dropped;
    sla::SupportableMesh dropped_sm{mesh, BLOCKED_LIP_POINTS, forbidden};
    REQUIRE_FALSE(sla::SupportTreeBuildsteps::execute(dropped, dropped_sm));
    REQUIRE(dropped.heads().size() == 1);
    CHECK_FALSE(dropped.heads().front().is_valid());
    CHECK(dropped.pillars().empty());

    sla::SupportTreeConfig allowed = blocked_lip_config();
    allowed.allow_model_anchors = true;
    sla::SupportTreeBuilder anchored;
    sla::SupportableMesh anchored_sm{mesh, BLOCKED_LIP_POINTS, allowed};
    REQUIRE_FALSE(sla::SupportTreeBuildsteps::execute(anchored, anchored_sm));
    REQUIRE(anchored.heads().size() == 1);
    CHECK(anchored.heads().front().is_valid());
    REQUIRE(anchored.pillars().size() >= 1);
    for (const sla::Pillar &pillar : anchored.pillars()) {
        CHECK(pillar.endpt.z() >= 3.9);
        CHECK(pillar.endpt.z() <= 12.0);
    }
}

TEST_CASE("A stop condition halts the builder between steps", "[SLASupportGeneration]") {
    indexed_triangle_set mesh = blocked_lip_mesh();
    sla::SupportTreeConfig cfg = blocked_lip_config();
    cfg.allow_model_anchors = true;

    sla::SupportTreeBuilder stopped;
    sla::JobController ctl;
    ctl.stopcondition = [] { return true; };
    stopped.set_ctl(ctl);
    sla::SupportableMesh stopped_sm{mesh, BLOCKED_LIP_POINTS, cfg};
    CHECK(sla::SupportTreeBuildsteps::execute(stopped, stopped_sm));
    CHECK(stopped.pillars().empty());

    sla::SupportTreeBuilder completed;
    sla::SupportableMesh completed_sm{mesh, BLOCKED_LIP_POINTS, cfg};
    CHECK_FALSE(sla::SupportTreeBuildsteps::execute(completed, completed_sm));
    CHECK(completed.pillars().size() >= 1);
}

TEST_CASE("A zero-elevation corrector bridge walks no further than its drop allows", "[SLASupportGeneration]") {
    // A corrector bridge is the walk create_ground_pillar lays at polar PI - bridge_slope: its XY moves and
    // its descent is its length times cos(slope). Its length may pass the drop over cos(slope) by one walk
    // step (the loop tests t < tmax and then adds the radius), never more. At pi/3 the walk descends half its
    // length, so it may run sideways up to tan(pi/3) times its drop, and the sweep lays walks that run further
    // sideways than their drop plus one step; a cap by the sine of the slope would stop every walk within one
    // step of its drop. At pi/4 the two caps agree and no walk runs sideways past its drop by a step.
    const auto [slope, min_wide] = GENERATE(table<double, size_t>({{M_PI / 4, 0}, {M_PI / 3, 1}}));
    const double cs = std::cos(slope);

    size_t correctors = 0, wide = 0;
    for (int hi = 0; hi <= 8; ++hi) {
        const double h  = 6. + 1. * hi;
        const double zj = h - 1.3, jx = 3.5; // the junction under the point, read off the default head
        for (int xi = 0; xi <= 12; ++xi) {
            const double plate_x_end = jx + zj - 3. + 0.5 * xi;
            indexed_triangle_set mesh = corrector_sweep_mesh(h, plate_x_end);
            const sla::SupportPoints points = {sla::SupportPoint(Vec3f(3.f, 20.f, float(h)), 0.2f)};

            sla::SupportTreeConfig cfg;
            cfg.object_elevation_mm  = 0.;
            cfg.max_bridge_length_mm = 30.;
            cfg.allow_model_anchors  = false;
            cfg.bridge_slope         = slope;

            sla::SupportTreeBuilder builder;
            sla::SupportableMesh sm{mesh, points, cfg};
            sla::SupportTreeBuildsteps::execute(builder, sm);

            for (const sla::Bridge &b : builder.bridges()) {
                const double xy      = (b.endp.head<2>() - b.startp.head<2>()).norm();
                const double descent = b.startp.z() - b.endp.z();
                if (xy < 1e-6) continue;
                if (std::abs(descent - b.get_length() * cs) > 1e-6) continue;
                ++correctors;
                const double drop  = b.startp.z() - builder.ground_level;
                const double bound = drop / cs + b.r + 1e-6;
                INFO("slope " << slope << " h " << h << " plate_x_end " << plate_x_end << " length "
                     << b.get_length() << " bound " << bound);
                CHECK(b.get_length() <= bound);
                if (xy > drop + b.r)
                    ++wide;
            }
        }
    }
    INFO("slope " << slope << " correctors " << correctors);
    CHECK(correctors >= 1);
    CHECK(wide >= min_wide);
}

TEST_CASE("A pillar stands at least the safety distance clear of the model", "[SLASupportGeneration]") {
    // The point under the corrector sweep's slab walks out past the ground plate's edge in steps of the bridge
    // radius until the ray casts, widened by the safety distance, clear the plate. At the default 0.5 mm the walk
    // stops with the pillar under 1 mm from the plate; at 1 mm it walks on and keeps 1 mm.
    const auto pillar_clearance = [](const sla::SupportTreeConfig &cfg) {
        indexed_triangle_set mesh = corrector_sweep_mesh(11., 10.2);
        sla::IndexedMesh emesh(mesh);
        const sla::SupportPoints points = {sla::SupportPoint(Vec3f(3.f, 20.f, 11.f), 0.2f)};
        sla::SupportTreeBuilder builder;
        sla::SupportableMesh sm{mesh, points, cfg};
        sla::SupportTreeBuildsteps::execute(builder, sm);
        REQUIRE(builder.pillars().size() == 1);
        // Samples along the pillar's axis, both ends left out, less its radius.
        const sla::Pillar &pillar = builder.pillars().front();
        double clearance = std::numeric_limits<double>::max();
        for (int k = 1; k < 20; ++k) {
            const Vec3d p = pillar.endpt + (pillar.startpoint() - pillar.endpt) * (k / 20.);
            clearance = std::min(clearance, std::sqrt(emesh.squared_distance(p)) - pillar.r);
        }
        return clearance;
    };

    sla::SupportTreeConfig cfg;
    cfg.object_elevation_mm  = 0.;
    cfg.max_bridge_length_mm = 30.;
    cfg.allow_model_anchors  = false;
    const double at_default = pillar_clearance(cfg);
    cfg.safety_distance_mm = 1.;
    const double at_one = pillar_clearance(cfg);
    INFO("pillar clearance " << at_default << " mm at the default, " << at_one << " mm at 1 mm");
    CHECK(at_default >= 0.5);
    CHECK(at_default < 1.);
    CHECK(at_one >= 1.);
}

TEST_CASE("Slender pillars braced in one plane or none count as unbraced and get no helper pillar", "[SLASupportGeneration]") {
    indexed_triangle_set mesh = two_plates_mesh();

    // The pair 5 mm apart chains to itself, one plane, and the pair 16 mm
    // apart is past the 12 mm link distance, so all four stand unbraced.
    sla::SupportTreeBuilder braced;
    sla::SupportableMesh braced_sm{mesh, TWO_PLATES_POINTS, two_plates_config(15.)};
    REQUIRE_FALSE(sla::SupportTreeBuildsteps::execute(braced, braced_sm));
    CHECK(braced.pillars().size() == 4);
    CHECK(braced.crossbridges().size() >= 1);
    CHECK(braced.unbraced_pillars == 4);

    sla::SupportTreeBuilder cascaded;
    sla::SupportableMesh cascaded_sm{mesh, TWO_PLATES_POINTS, two_plates_config(0.)};
    REQUIRE_FALSE(sla::SupportTreeBuildsteps::execute(cascaded, cascaded_sm));
    CHECK(cascaded.pillars().size() >= 6);
    CHECK(cascaded.unbraced_pillars == 0);
}

TEST_CASE("Slender pillars with neighbours in two planes are braced from both", "[SLASupportGeneration]") {
    // Three points 6 mm apart in a triangle under the small plate: each
    // pillar's two neighbours lie 60 degrees apart, so each takes chains in
    // two planes and none stands unbraced.
    const sla::SupportPoints points = {
        sla::SupportPoint(Vec3f(2.f, 2.f, 30.f), 0.2f),
        sla::SupportPoint(Vec3f(8.f, 2.f, 30.f), 0.2f),
        sla::SupportPoint(Vec3f(5.f, 7.2f, 30.f), 0.2f),
    };
    indexed_triangle_set mesh = two_plates_mesh();

    sla::SupportTreeBuilder builder;
    sla::SupportableMesh sm{mesh, points, two_plates_config(15.)};
    REQUIRE_FALSE(sla::SupportTreeBuildsteps::execute(builder, sm));
    REQUIRE(builder.pillars().size() == 3);
    CHECK(builder.unbraced_pillars == 0);

    for (const sla::Pillar &pillar : builder.pillars()) {
        const Vec2d axis = pillar.endpt.head<2>();
        std::vector<Vec2d> dirs;
        for (const sla::Bridge &br : builder.crossbridges())
            for (const auto &[on, off] : {std::pair{br.startp, br.endp}, std::pair{br.endp, br.startp}})
                if ((on.head<2>() - axis).norm() <= pillar.r)
                    dirs.emplace_back((off - on).head<2>().normalized());

        bool two_planes = false;
        for (const Vec2d &a : dirs)
            for (const Vec2d &b : dirs)
                two_planes = two_planes || std::abs(cross2(a, b)) >= std::sqrt(0.5) - EPSILON;
        INFO("pillar at " << axis.x() << ", " << axis.y() << " with " << dirs.size() << " brace ends");
        CHECK(two_planes);
    }
}

TEST_CASE("InitializedRasterShouldBeNONEmpty", "[SLARasterOutput]") {
    // Default SL1 display parameters
    sla::Resolution res{2560, 1440};
    sla::PixelDim   pixdim{120. / res.width_px, 68. / res.height_px};
    
    sla::RasterGrayscaleAAGammaPower raster(res, pixdim, {}, 1.);
    REQUIRE(raster.resolution().width_px == res.width_px);
    REQUIRE(raster.resolution().height_px == res.height_px);
    REQUIRE(raster.pixel_dimensions().w_mm == Catch::Approx(pixdim.w_mm));
    REQUIRE(raster.pixel_dimensions().h_mm == Catch::Approx(pixdim.h_mm));
}

TEST_CASE("MirroringShouldBeCorrect", "[SLARasterOutput]") {
    sla::RasterBase::TMirroring mirrorings[] = {sla::RasterBase::NoMirror,
                                                sla::RasterBase::MirrorX,
                                                sla::RasterBase::MirrorY,
                                                sla::RasterBase::MirrorXY};

    sla::RasterBase::Orientation orientations[] =
        {sla::RasterBase::roLandscape, sla::RasterBase::roPortrait};
    
    for (auto orientation : orientations)
        for (auto &mirror : mirrorings)
            check_raster_transformations(orientation, mirror);
}


TEST_CASE("RasterizedPolygonAreaShouldMatch", "[SLARasterOutput]") {
    double disp_w = 120., disp_h = 68.;
    sla::Resolution res{2560, 1440};
    sla::PixelDim pixdim{disp_w / res.width_px, disp_h / res.height_px};
    
    double gamma = 1.;
    sla::RasterGrayscaleAAGammaPower raster(res, pixdim, {}, gamma);
    auto bb = BoundingBox({0, 0}, {scaled(disp_w), scaled(disp_h)});
    
    ExPolygon poly = square_with_hole(10.);
    poly.translate(bb.center().x(), bb.center().y());
    raster.draw(poly);
    
    double a = poly.area() / (scaled<double>(1.) * scaled(1.));
    double ra = raster_white_area(raster);
    double diff = std::abs(a - ra);
    
    REQUIRE(diff <= predict_error(poly, pixdim));
    
    raster.clear();
    poly = square_with_hole(60.);
    poly.translate(bb.center().x(), bb.center().y());
    raster.draw(poly);
    
    a = poly.area() / (scaled<double>(1.) * scaled(1.));
    ra = raster_white_area(raster);
    diff = std::abs(a - ra);
    
    REQUIRE(diff <= predict_error(poly, pixdim));
    
    sla::RasterGrayscaleAA raster0(res, pixdim, {}, [](double) { return 0.; });
    REQUIRE(raster_pxsum(raster0) == 0);
    
    raster0.draw(poly);
    ra = raster_white_area(raster);
    REQUIRE(raster_pxsum(raster0) == 0);
}


TEST_CASE("halfcone test", "[halfcone]") {
    sla::DiffBridge br{Vec3d{1., 1., 1.}, Vec3d{10., 10., 10.}, 0.25, 0.5};

    indexed_triangle_set m = sla::get_mesh(br, 45);

    its_merge_vertices(m);
    write_debug_obj("sla_print/Halfcone.obj", m);
}

TEST_CASE("Test concurrency")
{
    std::vector<double> vals = grid(0., 100., 10.);

    double ref = std::accumulate(vals.begin(), vals.end(), 0.);

    double s = execution::accumulate(ex_tbb, vals.begin(), vals.end(), 0.);

    REQUIRE(s == Catch::Approx(ref));
}
