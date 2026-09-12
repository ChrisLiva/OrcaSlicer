#include <catch2/catch_all.hpp>

#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Support/SupportAnalysis.hpp"
#include "libslic3r/libslic3r.h"

#include <initializer_list>
#include <vector>

using namespace Slic3r;

namespace {

// An axis-aligned rectangle in mm, as an ExPolygon in scaled coordinates.
ExPolygon rect_mm(double x0, double y0, double x1, double y1)
{
    ExPolygon ex;
    ex.contour.points = { Point(scale_(x0), scale_(y0)), Point(scale_(x1), scale_(y0)),
                          Point(scale_(x1), scale_(y1)), Point(scale_(x0), scale_(y1)) };
    return ex;
}

// One printed slab from `bottom` to `top` mm carrying `polygons`, in the order given.
SupportAnalysis::Slab slab_mm(double bottom, double top, std::initializer_list<ExPolygon> polygons)
{
    SupportAnalysis::Slab slab;
    slab.bottom_z = bottom;
    slab.print_z  = top;
    slab.polygons = polygons;
    return slab;
}

// A 10 x 10 mm block standing on the plate, five 0.2 mm slabs high.
std::vector<SupportAnalysis::Slab> block_model()
{
    std::vector<SupportAnalysis::Slab> model;
    for (int i = 0; i < 5; ++ i)
        model.push_back(slab_mm(0.2 * i, 0.2 * (i + 1), { rect_mm(0., 0., 10., 10.) }));
    return model;
}

} // namespace

TEST_CASE("Floating support is what no printed slab carries down to the plate", "[SupportAnalysis]")
{
    // Slab by slab, all 0.2 mm high and each starting where the one under it ends, except the slab
    // that is missing under the last column. Beside the block: a column standing on the plate, a piece
    // touching that column along one edge only, a piece with nothing under it at all, and a column
    // whose middle slab was never printed.
    const ExPolygon column = rect_mm(20., 0., 21., 1.);
    const ExPolygon edge   = rect_mm(21., 0., 22., 1.);   // shares the column's x = 21 edge, no area
    const ExPolygon alone  = rect_mm(30., 0., 31., 1.);
    const ExPolygon gapped = rect_mm(40., 0., 41., 1.);
    const std::vector<SupportAnalysis::Slab> support{
        slab_mm(0.0, 0.2, { column, gapped }),
        slab_mm(0.2, 0.4, { column, edge }),
        slab_mm(0.4, 0.6, { column, alone, gapped }),
    };
    const std::vector<std::vector<bool>> floating =
        SupportAnalysis::floating_pieces(support, block_model(), true, 0.);
    REQUIRE(floating.size() == 3);
    REQUIRE(floating[0] == std::vector<bool>{ false, false });
    REQUIRE(floating[1] == std::vector<bool>{ false, true });
    REQUIRE(floating[2] == std::vector<bool>{ false, true, true });

    // No model at all is the plate alone: the same answer, since nothing here rests on the model.
    REQUIRE(SupportAnalysis::floating_pieces(support, {}, false, 0.) == floating);
}

TEST_CASE("Support rests on the model only where the settings allow it and within the bottom gap", "[SupportAnalysis]")
{
    // Over the block's top at z = 1: a piece standing directly on it with a second piece over that one,
    // and a piece 0.4 mm above the top with nothing between.
    const ExPolygon on_top   = rect_mm(2., 2., 3., 3.);
    const ExPolygon over_it  = rect_mm(2.2, 2.2, 2.8, 2.8);
    const ExPolygon above    = rect_mm(6., 6., 7., 7.);
    const std::vector<SupportAnalysis::Slab> support{
        slab_mm(1.0, 1.2, { on_top }),
        slab_mm(1.2, 1.4, { over_it }),
        slab_mm(1.4, 1.6, { above }),
    };
    const std::vector<SupportAnalysis::Slab> model = block_model();

    // Plate only: nothing here reaches the plate, so all of it floats.
    REQUIRE(SupportAnalysis::floating_pieces(support, model, true, 0.) ==
            std::vector<std::vector<bool>>{ { true }, { true }, { true } });

    // Resting on the model with no bottom gap: the piece on the top roots, and the one over it roots
    // through it. The piece 0.4 mm up reaches no model within its own slab and floats.
    REQUIRE(SupportAnalysis::floating_pieces(support, model, false, 0.) ==
            std::vector<std::vector<bool>>{ { false }, { false }, { true } });

    // A 0.3 mm bottom gap: the piece 0.4 mm up now reaches the top within the gap plus its own slab.
    REQUIRE(SupportAnalysis::floating_pieces(support, model, false, 0.3) ==
            std::vector<std::vector<bool>>{ { false }, { false }, { false } });

    // The object on a raft: the same block lifted 0.2 mm off the plate. The block's first slab is the
    // object's ground whether the plate or a raft carries it, so the piece on its top roots.
    std::vector<SupportAnalysis::Slab> lifted = model;
    for (SupportAnalysis::Slab &slab : lifted) {
        slab.bottom_z += 0.2;
        slab.print_z  += 0.2;
    }
    const std::vector<SupportAnalysis::Slab> raised{ slab_mm(1.2, 1.4, { on_top }) };
    REQUIRE(SupportAnalysis::floating_pieces(raised, lifted, false, 0.) == std::vector<std::vector<bool>>{ { false } });

    // Model material not connected to the first slab holds nothing: an island beside the block from
    // z 0.6 to 1.0, with a piece standing on its top.
    std::vector<SupportAnalysis::Slab> island_model = block_model();
    island_model[3].polygons.push_back(rect_mm(20., 20., 22., 22.));
    island_model[4].polygons.push_back(rect_mm(20., 20., 22., 22.));
    const std::vector<SupportAnalysis::Slab> on_island{ slab_mm(1.0, 1.2, { rect_mm(20.5, 20.5, 21.5, 21.5) }) };
    REQUIRE(SupportAnalysis::floating_pieces(on_island, island_model, false, 0.) == std::vector<std::vector<bool>>{ { true } });
}
