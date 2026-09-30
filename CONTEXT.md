# Tree Scaffold support

Tree Scaffold is the FDM support style for miniatures: thin pillars on a pad, each ending in a head that fuses to the
model. This glossary covers the scaffold and the conversion of an artist's resin supports into it.

## Language

### The scaffold

**Scaffold point**:
A place where a scaffold tip meets the model, held in the object's list with a tip size and an approach axis.
_Avoid_: support point, contact point

**Tip size**:
The preset width of a scaffold tip's contact disc, Light or Heavy, counted in support lines rather than millimetres.
_Avoid_: tip diameter, head radius

**Approach axis**:
The direction along which a scaffold tip's neck meets the model at a scaffold point.
_Avoid_: normal, tilt

**Pad**:
The printed plate on the bed that every scaffold pillar stands on.
_Avoid_: raft, base

**Bare island**:
A part of the model that first prints in mid-air with no scaffold tip holding it.
_Avoid_: unsupported region

### Conversion

**Pre-supported model**:
A figure an artist ships with resin supports already modelled into the same file.
_Avoid_: supported STL

**Artist support**:
Any piece of a pre-supported model that is not the figure: tips, trunks, branches, braces and the artist raft.
_Avoid_: resin support, original support

**Artist tip**:
The tapered end of an artist support that touches the figure, a cone swept between a wide sphere and a narrow one.
_Avoid_: contact cone

**Contact site**:
The point on the figure where an artist tip's narrow end sits; it becomes a scaffold point's position.
_Avoid_: contact point, tip position

**Micro strut**:
A hair-thin artist support with both ends on the figure that ties fragile parts together; conversion drops it.
_Avoid_: micro support, reinforcement

**Artist raft**:
The pads and bars on the build plate that the artist supports stand on, discarded in favour of the pad.
_Avoid_: base, raft layers

**Conversion**:
Turning a pre-supported model into the bare figure plus one scaffold point per artist tip, at the artist tip's contact
site and approach axis, sized to the scaffold's tip sizes. The artist's trunks, braces and raft are not carried over.
_Avoid_: import, re-support
