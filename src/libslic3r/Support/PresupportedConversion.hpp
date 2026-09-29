#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>

namespace Slic3r {

class ModelObject;

// Turns a pre-supported model, a figure an artist ships with resin supports in the same mesh, into the bare figure and
// one enforced scaffold point per artist tip at the tip's contact site, along its approach axis and sized by its contact
// diameter. The Tree Scaffold builder routes its own pillars, braces and pad from the points; nothing else of the
// artist's supports carries over. docs/HLSD/scaffold-support.md, "Converting a pre-supported model".
namespace PresupportedConversion {

// NoSupports: the mesh splits into no artist support, as a plain model does and as one whose supports are welded to
// the figure does. NoArtistTips: the supports hold no tip. SeveralParts: the object is not one model part.
enum class Refusal : uint8_t { None, SeveralParts, NoSupports, NoArtistTips };

struct Summary
{
    Refusal refusal               = Refusal::None;
    size_t  tips_converted        = 0; // points written, one per distinct artist tip
    size_t  duplicates_removed    = 0; // tips another tip repeats at the same contact site and axis
    size_t  axes_clamped          = 0; // axes leaning past the builder's head tilt cap, moved onto the cap
    size_t  micro_struts_dropped  = 0; // supports off the plate with no tip and two or more ends on the figure
    size_t  tips_rooted_on_figure = 0; // converted tips whose support never reaches the plate
};

// Leaves `object` unchanged on a refusal. Otherwise replaces its model part's mesh with the figure, in place, so the
// figure keeps the artist's pose; turns auto_drop off on every instance, so no bed drop takes the artist's lift; and
// writes a UserModified list stamped with instance `instance_idx`'s pose and the figure's mesh box. The points come in
// the mesh's shell order, so a file converts to the same list every time. `before_change` runs once, just before the
// object changes, where the GUI takes its undo snapshot.
Summary convert(ModelObject &object, size_t instance_idx, const std::function<void()> &before_change = {});

} // namespace PresupportedConversion
} // namespace Slic3r
