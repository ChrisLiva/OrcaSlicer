#include "AutoTiltScorer.hpp"

#include <algorithm>
#include <limits>
#include <utility>

#include "ClipperUtils.hpp"
#include "Geometry.hpp"
#include "Layer.hpp"
#include "Support/TreeSupport.hpp"
#include "libslic3r.h"

namespace Slic3r { namespace AutoTilt {

ContactScorer::ContactScorer(const ModelObject &object, const DynamicPrintConfig &full_config, const Constants &k, MainThreadRunner run_on_main)
    : m_k(k), m_run_on_main(std::move(run_on_main)), m_config(full_config)
{
    // add_object() clones every instance; the search only ever poses one, so drop the rest.
    ModelObject *obj = m_model.add_object(object);
    while (obj->instances.size() > 1)
        obj->delete_last_instance();

    m_root     = obj->instances.front()->get_transformation().get_matrix();
    m_root_box = obj->instance_bounding_box(0); // the constructor never calls ensure_on_bed(), so this is the root pose's box
    m_pivot    = m_root_box.center();

    // The object's own layer height wins over the preset's, so read it before erasing it.
    const double print_h = obj->config.has("layer_height") ? obj->config.opt_float("layer_height") : full_config.opt_float("layer_height");
    m_h_search           = std::max(print_h, m_k.h_search_min_mm);

    // Every per-object route to a layer height has to go, not just one: update_layer_height_profile
    // prefers the profile over the ranges, so leaving either in place re-slices at the object's height.
    obj->config.erase("layer_height");
    obj->config.erase("support_remove_small_overhang");
    obj->layer_height_profile.clear();
    obj->layer_config_ranges.clear();

    m_config.set("layer_height", m_h_search);
    // Poses are compared on the contact the detector finds, so the small-overhang filter, which
    // drops contacts by area, must not vary the measurement between candidates.
    m_config.set("support_remove_small_overhang", false);
    m_print.set_status_silent();
}

Contact ContactScorer::score(const Pose &pose)
{
    ModelObject   *obj  = m_model.objects.front();
    ModelInstance *inst = obj->instances.front();
    inst->set_transformation(Geometry::Transformation(candidate_transform(m_root, m_pivot, pose)));
    obj->invalidate_bounding_box();
    obj->ensure_on_bed();
    const Transform3d posed = inst->get_transformation().get_matrix(); // read after ensure_on_bed() moved it

    // Print::apply rewrites the model tree and bumps ObjectIDs, so it belongs to the main thread;
    // slicing and overhang detection below run right here on the worker.
    m_run_on_main([this]() { m_print.apply(m_model, m_config); });

    PrintObject *po = m_print.objects_mutable().front();
    po->slice();

    TreeSupport ts(*po, po->slicing_parameters());
    ts.m_scoring_mode = true; // layer-height-stable sharp-tail bands, no wall-clock cutoff
    ts.detect_overhangs();

    // Serial, ascending, fixed order: two candidates must sum the same floats in the same order,
    // whatever the worker pool looks like.
    Contact         c;
    ExPolygons      shadow; // union of every lower layer's slices; only filled under shadow_correction
    const LayerPtrs &layers = po->layers();
    for (size_t i = 0; i < layers.size(); ++i) {
        const Layer *layer = layers[i];
        c.object_volume_mm3 += area(layer->lslices) * SCALING_FACTOR * SCALING_FACTOR * layer->height;
        // Layer 0 sits on the plate: whatever the detector calls an overhang there needs no support.
        if (i == 0)
            continue;
        if (this->shadow_correction)
            shadow = union_ex(shadow, layers[i - 1]->lslices);
        for (size_t j = 0; j < layer->loverhangs.size(); ++j) {
            // A reference, not a copy: overhang_types is keyed on the address of this very element.
            const ExPolygon &p = layer->loverhangs[j];

            const auto it         = ts.overhang_types.find(&p);
            const bool type_floor = (it != ts.overhang_types.end() && it->second == TreeSupport::SharpTail) ||
                                    (! layer->cantilevers.empty() && overlaps(layer->cantilevers, p));

            // Contact low on the object in its root pose is what the exclusion plane drops. A
            // degenerate contour needs no special case: centroid() returns an integer Point, so the
            // fraction is finite, and a zero-area polygon adds 0 to both sums either way.
            const Vec3d centroid_world =
                to_3d(unscale(p.contour.centroid()) + unscale(po->instances().front().shift), layer->print_z);
            const double fraction = root_height_fraction(m_root, posed, m_root_box, centroid_world);
            const bool   excluded = fraction < m_k.bottom_exclusion_fraction;

            // The row describes the whole polygon, excluded or not: the harness needs the rows the
            // plane drops to judge where to put it. Written before the `continue` for that reason.
            if (this->records != nullptr) {
                const double a = p.area() * SCALING_FACTOR * SCALING_FACTOR;
                double       perimeter_scaled = p.contour.length();
                for (const Polygon &hole : p.holes)
                    perimeter_scaled += hole.length();
                const double perimeter = unscale<double>(perimeter_scaled);
                this->records->push_back(PolygonRecord{ i, layer->print_z, a, perimeter,
                                                        perimeter > 0. ? 2. * a / perimeter : std::numeric_limits<double>::quiet_NaN(),
                                                        fragility_weight(a, perimeter, type_floor, m_k), fraction, type_floor, excluded });
            }

            if (excluded)
                continue;

            // One piece's contribution. The arithmetic and its order are the same whether `p` is
            // charged whole or in shadow-corrected pieces, so the plain path's sums are unchanged.
            const auto charge = [&](const ExPolygon &piece) {
                const double a = piece.area() * SCALING_FACTOR * SCALING_FACTOR;
                double       perimeter_scaled = piece.contour.length();
                for (const Polygon &hole : piece.holes)
                    perimeter_scaled += hole.length();
                const double perimeter = unscale<double>(perimeter_scaled);

                const double w = fragility_weight(a, perimeter, type_floor, m_k);
                c.volume_mm3 += a * m_k.h_ref_mm;
                c.score_mm3 += w * a * m_k.h_ref_mm;
            };

            if (this->shadow_correction)
                // The part of the overhang standing over solid object needs no support under it.
                for (const ExPolygon &piece : diff_ex(p, shadow))
                    charge(piece);
            else
                charge(p);
        }
    }
    return c;
}

}} // namespace Slic3r::AutoTilt
