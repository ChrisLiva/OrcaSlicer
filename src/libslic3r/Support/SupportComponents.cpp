#include "SupportComponents.hpp"

#include "DisjointSets.hpp"
#include "../ClipperUtils.hpp"
#include "../Layer.hpp"
#include "../Print.hpp"
#include "../libslic3r.h"

#include <algorithm>

namespace Slic3r {
namespace SupportAnalysis {

bool on_bed(double bottom_z)
{
    return bottom_z <= EPSILON;
}

Components build_components(const std::vector<Slab> &slabs, double ground_z)
{
    Components out;
    out.slab_range.resize(slabs.size());
    for (size_t s = 0; s < slabs.size(); ++ s) {
        out.slab_range[s].first = out.pieces.size();
        for (const ExPolygon &polygon : slabs[s].polygons) {
            Piece piece;
            piece.polygon  = polygon;
            piece.bottom_z = slabs[s].bottom_z;
            piece.print_z  = slabs[s].print_z;
            out.pieces.emplace_back(std::move(piece));
        }
        out.slab_range[s].second = out.pieces.size();
    }

    DisjointSets sets(out.pieces.size());

    // Two pieces whose boxes do not overlap intersect in nothing, so the clip runs on the pairs whose
    // boxes do: a few per piece on a layer of hundreds, where the pairwise clip is hundreds per piece.
    std::vector<BoundingBox> boxes;
    boxes.reserve(out.pieces.size());
    for (const Piece &piece : out.pieces)
        boxes.push_back(get_extents(piece.polygon));
    for (size_t s = 0; s + 1 < slabs.size(); ++ s) {
        if (slabs[s + 1].bottom_z > slabs[s].print_z + 1e-6)
            continue; // the two slabs do not touch: nothing printed between them
        for (size_t i = out.slab_range[s].first; i < out.slab_range[s].second; ++ i)
            for (size_t j = out.slab_range[s + 1].first; j < out.slab_range[s + 1].second; ++ j) {
                if (! boxes[i].overlap(boxes[j]))
                    continue;
                if (intersection_ex(ExPolygons{ out.pieces[i].polygon }, ExPolygons{ out.pieces[j].polygon }).empty())
                    continue;
                out.pieces[i].above.push_back(j);
                out.pieces[j].below.push_back(i);
                sets.join(j, i);
            }
    }

    std::vector<size_t> label(out.pieces.size(), size_t(-1));
    for (size_t i = 0; i < out.pieces.size(); ++ i) {
        const size_t root = sets.find(i);
        if (label[root] == size_t(-1))
            label[root] = out.count ++;
        out.pieces[i].component = label[root];
    }
    out.bed_rooted.assign(out.count, 0);
    for (const Piece &piece : out.pieces)
        if (piece.bottom_z <= ground_z + EPSILON)
            out.bed_rooted[piece.component] = 1;
    return out;
}

std::vector<char> rooted_components(const Components &support, const std::vector<Slab> &model_slabs, const Components &model,
                                    bool on_build_plate_only, double bottom_gap)
{
    std::vector<char> rooted = support.bed_rooted;
    if (on_build_plate_only)
        return rooted;
    std::vector<double> model_tops(model_slabs.size(), 0.);
    for (size_t s = 0; s < model_slabs.size(); ++ s)
        model_tops[s] = model_slabs[s].print_z;
    for (const Piece &piece : support.pieces) {
        if (rooted[piece.component])
            continue;
        // The object material this slab comes down onto: no further below its underside than the
        // gap the settings leave between a support bottom and the object, plus the slab itself,
        // because the surface it stands over is only known to the layer it was sliced at.
        const double reach = bottom_gap + (piece.print_z - piece.bottom_z);
        size_t       s     = size_t(std::lower_bound(model_tops.begin(), model_tops.end(),
                                                     piece.bottom_z - reach - 1e-6) - model_tops.begin());
        for (; s < model_slabs.size() && model_slabs[s].print_z <= piece.bottom_z + 1e-6; ++ s) {
            for (size_t m = model.slab_range[s].first; m < model.slab_range[s].second; ++ m) {
                // The object it rests on has to be standing up itself: material that is floating
                // holds nothing.
                if (! model.bed_rooted[model.pieces[m].component])
                    continue;
                if (intersection_ex(ExPolygons{ piece.polygon }, ExPolygons{ model.pieces[m].polygon }).empty())
                    continue;
                rooted[piece.component] = 1;
                break;
            }
            if (rooted[piece.component])
                break;
        }
    }
    return rooted;
}

Vec3d piece_middle(const Piece &piece)
{
    const Point point = piece.polygon.contour.centroid();
    return Vec3d(point.x() * SCALING_FACTOR, point.y() * SCALING_FACTOR, 0.5 * (piece.bottom_z + piece.print_z));
}

size_t piece_containing(const Components &support, size_t slab, const ExPolygon &part)
{
    if (part.contour.points.empty())
        return size_t(-1);
    for (size_t k = support.slab_range[slab].first; k < support.slab_range[slab].second; ++ k)
        if (support.pieces[k].polygon.contains(part.contour.points.front(), true))
            return k;
    return size_t(-1);
}

ExPolygons bed_ground(const std::vector<Slab> &model_slabs, const std::vector<Slab> &support_slabs, const Polygons &adhesion)
{
    ExPolygons ground;
    if (! model_slabs.empty() && on_bed(model_slabs.front().bottom_z))
        ground = model_slabs.front().polygons;
    for (const Slab &slab : support_slabs)
        if (on_bed(slab.bottom_z))
            append(ground, slab.polygons);
    append(ground, union_ex(adhesion));
    return union_ex(ground);
}

std::vector<Slab> model_slabs_of(const PrintObject &object)
{
    std::vector<Slab> slabs;
    slabs.reserve(object.layers().size());
    for (const Layer *layer : object.layers()) {
        Slab slab;
        slab.print_z  = layer->print_z;
        slab.bottom_z = layer->print_z - layer->height;
        slab.polygons = layer->lslices;
        slabs.emplace_back(std::move(slab));
    }
    return slabs;
}

std::vector<std::vector<bool>> floating_pieces(const std::vector<Slab> &support, const std::vector<Slab> &model,
                                               bool on_build_plate_only, double bottom_gap_mm)
{
    const Components        components = build_components(support, 0.);
    const std::vector<char> rooted     = model.empty() ? components.bed_rooted :
        rooted_components(components, model, build_components(model, model.front().bottom_z), on_build_plate_only,
                          std::max(0., bottom_gap_mm));
    std::vector<std::vector<bool>> floating(support.size());
    for (size_t s = 0; s < support.size(); ++ s) {
        floating[s].assign(support[s].polygons.size(), false);
        for (size_t k = components.slab_range[s].first; k < components.slab_range[s].second; ++ k)
            floating[s][k - components.slab_range[s].first] = ! rooted[components.pieces[k].component];
    }
    return floating;
}

} // namespace SupportAnalysis
} // namespace Slic3r
