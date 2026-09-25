# Scaffold support — High Level Design

## Purpose and scope

Tree Scaffold is a support style for miniatures and other fine models printed
with a small nozzle. Its body is the SLA support tree: thin pillars standing on
a pad, joined by bridges and braces, each pillar ending in a small head
that fuses to the model. A print peels off the pad in one piece and the heads
snap off the model at their necks, which leaves small marks rather than the
broad scars a tree roof leaves on a figure's underside.

The style reuses the legacy tree's front half: overhang detection, contact
seeding, the miniature contact selection, the model risk field and the planned
support layer heights. It replaces only the body. `ScaffoldSupport::draw` builds
the SLA tree on the object mesh with its ground on the bed, adds a pad and
slices both into the planned layers, and `TreeSupport` prints those areas through the
legacy tree's toolpath, floating-removal and measurement tail. The style
generates FFF support only; nothing in the SLA printing pipeline changes.

## Settings and eligibility

`support_style` gains the value `tree_scaffold`, labelled Tree Scaffold and
listed last in the style menu. Like the other tree styles it needs a tree
`support_type`: `SupportParameters` resolves the style to grid under a normal
support type.

Two settings belong to the style and show only under it:

- `scaffold_bridge_length`, 3 to 30 mm with a 12 mm default, bounds every
  bridge from a tip to its pillar and every brace between pillars.
- `scaffold_brace_slenderness`, 5 to 40 with a 15 default, is the unbraced
  height of a pillar, in pillar diameters, above which the builder braces it
  to its neighbours.

`tree_support_branch_diameter` sets the pillar diameter, as it sets the branch
diameter of the slim, strong and hybrid styles, and
`tree_support_branch_diameter_angle` sets how fast a pillar widens toward the
pad, as it sets how fast a tree branch widens toward its root. The settings
panel shows the angle under Scaffold as well as under the organic style.

Under Scaffold three settings the style depends on read fixed, and the
settings panel greys their fields out. The constructor of `TreeSupport` forces
two of them: `support_miniature_contacts` reads on, because the scaffold places
one tip per contact the miniature selection keeps, and `support_top_z_distance`
reads zero, because a tip fuses to the model rather than printing under a gap.
`remove_floating_toolpaths` applies the third: it roots support on the plate
alone under `m_scaffold || support_on_build_plate_only`, because a pillar never
stands on the model. Auto-tilt names the style as its own generator and refuses
to evaluate an object that uses it, with the message "Auto-tilt cannot verify
this object: Tree Scaffold support is not evaluated."

A project that names `tree_scaffold` in a build without the style loads with
the default style substituted, and the two scaffold keys are dropped as
unknown keys, so the style needs no migration code.

## The seam in TreeSupport::generate()

`TreeSupport::m_scaffold` is true when the resolved style is
`smsTreeScaffold`, and every scaffold branch in the generator reads it. The
front half runs as it does for the slim style with miniature contacts on:
`detect_overhangs`, the contact seeds, `select_contacts`, the risk field. When
the contact selection erases the contacts it did not keep, the scaffold keeps
those pointers in `m_dropped_contacts`, because the island hold floor draws on
them.

Before `plan_layer_heights`, under Scaffold, the generator fills a
`ScaffoldSupport::Params` from the object config (toolpath width, pillar
diameter, `m_ts_data->m_xy_distance`, the two scaffold keys,
`max_bridge_length`, the interface layer count and the object's print z
offset) and calls `ScaffoldSupport::choose_tips` on the kept and the dropped
contacts. `plan_layer_heights` then takes every chosen tip's z as a layer top
besides the contacts' own, so a tip the hold floor seeded or restored where no
contact stands prints its top ring on the layer right under it, not under an
air gap. Under every other style that list is empty.

After the plan the generator branches. Under Scaffold it sets the pad thickness
to 0.6 mm rounded up to the first planned layer at or above it and computes
each planned layer's clip once, a `ScaffoldSupport::LayerClip`: the model over
the layer's whole height, and that model grown by the xy distance together
with the bottom-gap trim regions. It calls `ScaffoldSupport::draw` with the
chosen tips, and the draw's neck check reads the same clips. `drop_nodes`,
`smooth_nodes` and `draw_circles` do not run.

For each planned layer `TreeSupport` then clips the returned areas.
`ScaffoldSupport::clip_base` keeps the base out of the grown band and adds back
the enforced tips' heads kept out of the model alone, the machine border clips
the result, and the interface areas stay out of the model alone, since the
rings fuse to it. The base areas become `BaseType` area groups with no infill
and the interface areas become `Roof1stLayer` groups; a group on one of the
pad's layers carries `pad = true`.
The shared members `finish_layer_areas`, `normalize_interface_ids` and
`erase_empty_support_layers`, which `draw_circles` also calls, finish the
layers, and the generator records one emitted layer per filled planned layer
for the measurement. From there `generate_toolpaths` and the rest of the tail
run for every style alike.

## The ScaffoldSupport module

`src/libslic3r/Support/ScaffoldSupport.{hpp,cpp}` holds two functions that
run seven steps between them. `ScaffoldSupport::choose_tips`, the selection,
runs steps 1 to 4 on the contact nodes, reads no planned layer and returns the
tips with the hold floor's count, which is why it can run before the layers are
planned. `ScaffoldSupport::draw` runs steps 5 to 7 on those tips and the
planned layers, turns them into per-layer areas and counts what it did with
them.

1. Tip selection. Every contact node becomes a tip, except an interior contact
   (`SupportNode::Placement::Interior`) whose overhang, shrunk by half of
   `max_bridge_length`, leaves nothing: under an overhang narrower than one
   bridge span the corner and contour tips on its rim already hold it.
2. The wall skip. The seam clips support inside the xy distance of the model,
   so a head whose neck stood in that band would lose its neck while its ring
   survived. The selection reads the object layer at the neck's bottom, one head
   width plus one toolpath width under the tip, and skips a tip whose centre
   lies within the xy distance of that layer's slices; the wall anchors that
   band as it does under the legacy tree. The layer just under the overhang is
   the wrong reference: on a sloped underside every contact sits a fraction of
   a layer past it, while a slope has receded by the neck's bottom and a wall
   has not. A skipped tip logs `scaffold tip skipped at (x, y, z): wall` at
   debug level and counts as neither placed nor dropped. The same test applies
   to the dropped contacts the hold floor may restore and to the tips it
   seeds. An enforced tip is exempt: a contact a support enforcer asked for,
   `SupportNode::is_pinned`, which under Scaffold marks every contact on an
   overhang an enforcer covers, whether painted facets or an enforcer
   modifier volume, and every vertical enforcer point painted facets place,
   becomes a `TipSite` with `enforced` set, and the skip keeps it however
   close the wall stands, so the tip fuses where the user asked for support
   and leaves its scar there.
3. The island hold floor. `SupportAnalysis::island_joins` maps every mid-air
   island of the model to the slab where it first meets the rooted body. An
   island needs one tip when its unjoined height is at most 1 mm, two up to
   5 mm and three above, counted greedily from the lowest tip and only where a
   tip stands a pillar diameter from every tip counted before it. A
   never-joining island's height runs to the top of the part it ends up in:
   where two such islands meet in mid-air the older owns the pieces above the
   merge, so a short leg of a taller floating part measures to that part's top
   and keeps its floor and its seed. Only a whole never-joining part at most
   1 mm tall is mesh debris, at any height: its islands get no floor, no tip
   and no count, and log
   `scaffold island skipped at z: debris` at debug level. An island with no tip
   and no dropped contact under it gets one tip seeded at the deepest point of
   its birth piece, at that piece's bottom, in the small grade. When the wall
   skip removes that seed and the island joins within 1 mm, the wall beside it
   holds it: it gets no tip, no count, and logs
   `scaffold island held at z: wall` at debug level. A taller island whose seed
   stands at a wall counts as under-held. Short of the floor, the selection
   restores dropped contacts under the island, lowest first and among equals
   the one furthest from the island's tips. An island counts as under-held only
   when it holds fewer tips than both its floor and the number its birth piece
   fits, the points of a hexagonal grid at the pillar diameter inside the piece
   shrunk by half a pillar diameter, never fewer than one. A tip belongs to the
   island that owns the model piece over it on the overhang's own layer, one
   above the node's layer.
4. The alias merge. The front half can hand the same overhang spot on two
   consecutive layers, and the builder keeps one point of each pair within
   `sla::D_SP`. The selection visits the tips lowest first, among equals by seed
   id, and merges a tip standing within `sla::D_SP` in 3-D of a tip already
   kept into it, so no two tips it hands the builder are aliases; a kept tip is
   enforced when any tip merged into it was. A merge logs
   `scaffold tip merged at (x, y, z)` at debug level and counts as neither
   placed nor dropped.
5. Grading. A tip's head fuses to the model with a disc two support lines
   wide, or four where the risk field reads the model under it as known and
   hanging off a neck at least eight lines wide; a seeded tip keeps two. The
   head's pin radius is half that width.
6. The build. The draw hands the tips and the object mesh, in the frame the
   object's slices use, to `sla::SupportTreeBuildsteps::execute` with the
   configuration `tree_config` writes: head front, penetration and fallback
   radius at one toolpath width, pillar radius at half the pillar diameter,
   no model anchors, bridge and pillar-link lengths at the
   scaffold bridge length, a 45 degree bridge slope, the object's xy distance
   as the safety distance and the scaffold brace slenderness. The draw sets
   the elevation to the mesh's lowest z: the builder grounds pillars at that z
   less the elevation, so the ground sits at the pad's top on the bed for an
   object standing on it and for one lifted off it with auto-drop off. A
   lifted object's positive elevation also turns off the zero-elevation walk
   that steers a pillar's foot out from under the model, so a pillar drops
   straight under the underside. A head the
   builder kept is a routed tip; a head it built and gave up on is an
   unrouted drop; a point that never got a head is a filtered drop.
   A head pointing straight down needs its whole length above the pad's top,
   the 1 mm head width plus twice each radius less the penetration, 2.62 mm
   at a 0.42 mm line and a 1.2 mm pillar, so a tip lower than that, such as
   the underside of an object lifted 1 mm, gets no head. After each run such a
   point stands on a post instead when it is no more than that length plus the
   0.5 mm pillar base above the pad's top and its disc stays out of the clip's
   band on every planned layer under its rings and on every pad layer, where
   the seam would cut the post or the pad under it: a pillar as wide as the
   tip's disc from the pad's top to the tip, added to the builder so the cage
   and the pad take it in. A post's rings are its disc on the layers a head's
   rings would take, and the neck check reads them with the heads' rings: a
   post whose rings float is cut as a `neck` drop and leaves the point set for
   the next run. A posted tip counts as routed. The band test matters beside
   a model standing on the bed: a tip in a crevice there has rings the model
   alone clips, over pad layers the band cuts away, and the neck check reads
   no pad layer.
   Every pillar the builder stood on the pad's top then widens toward it over
   its whole height: a cone from the pillar's radius at its top to that
   radius plus its height times the branch diameter angle, in radians, at
   the pad. A sideways push on the model, a nozzle catching an edge high up,
   bends each pillar most at its foot, and the more so the taller the pillar
   stands, while a straight pillar's foot is no stronger than its top. A foot
   stops half a toolpath width past the midpoint to its nearest pillar or
   post, so neighbouring feet fuse along a line. A foot that took its
   neighbour in would leave that neighbour nothing printed to stand on: the
   base walls follow the outline of the union, and where the feet part higher
   up the neighbour's wall would start over the union's empty inside. A pillar
   the builder gave no base, one whose foot stands too near the model, runs
   down into the pad and stays straight, and so do the posts. The cones go on
   after routing, so the taper moves no pillar and no bridge; the seam's band
   clips a foot that reaches toward the model as it clips any base.
   The neck check then reads what the seam would print of the build on each
   planned layer above the pad, as the outlines its lines cover, since the
   floating pass reads `polygons_covered_by_width`. The areas are the cage
   outside the clip's band with the enforced heads outside the model, through
   the `clip_base` the seam calls, and the rings outside the model, with the
   small holes `TreeSupport::fill_small_holes` fills filled, as
   `finish_layer_areas` fills them on every support layer, so an enforced head's
   neck beside a wall holds its rings and the head is not cut for it. The base
   goes through `Params::base_cover`, which under the default base pattern
   lays its walls
   through the same `tree_supports_generate_paths` call `generate_toolpaths`
   makes, so a wide area's inside and a sliver too short for a loop print
   nothing, and under a pattern with infill other than lightning reads the
   area opened by half a support line. A ring
   prints one interface loop on its area shrunk by half the interface
   spacing, laid through `extrusion_entities_append_loops` as
   `make_perimeter_and_infill` lays it, with its infill counted as solid, so a
   part of a ring narrower than one spacing prints nothing and a ring touching
   a neighbour only across such a neck prints apart from it.
   `SupportAnalysis::floating_pieces`, the rule the floating pass applies,
   finds the pieces with no chain of overlaps down to the pad's top, and a
   head one of whose rings lies in such a piece is cut: its tilted neck
   crosses the band under its rings, the base under them prints nothing, or
   the builder left it with no pillar and no bridge, which a side head whose
   ground pillar fails keeps. A floating piece that holds no ring and is the
   wall of a cage hole, a gap among fused necks 2 mm across or wider whose
   narrower stretches on the layers around it are filled, has its hole filled
   instead, which removes a wall that stands over the unprinted inside of the
   section below. An enforced head whose own neck under its rings lies in a
   floating piece that holds no ring and walls no hole is cut as well: the
   head runs through the model or narrows under a line there, so the neck the
   enforcer exempts from the band would print in mid-air. On a layer with an
   enforced head a piece counts as a hole's wall only when the base the seam
   prints after the fill borders no hole at it, since the band or the model
   can cut the filled hole out of that base again and leave the wall in
   mid-air. With no interface layers there is no ring to check, but the check
   still runs while any tip is enforced, since an enforced head prints its
   neck up to its tip. The cut heads' points
   leave the point set, in order, and the builder runs again, since a run
   re-routes the neighbours of what it lost and can strand another ring. It
   repeats until no head is cut, at most six runs (plate 3 of the corpus needs
   five), and the heads the sixth run still cuts are dropped without another
   run, their rings left out of the output. Each run logs
   `scaffold build <n>: <points> points, <cut> heads cut` and
   each drop `scaffold tip dropped at (x, y, z): <reason>` at debug level,
   the reason `filtered`, `unrouted` (also for a cut head with no pillar and
   no bridge) or `neck`. When anything routed, `SupportTreeBuilder::add_pad`
   adds a pad of the pad thickness with a 1.6 mm brim and no wall.
7. Slicing. Each run slices its cage at the middle of every planned layer,
   each routed head's own mesh at the tops of its top interface layer,
   the planned layer whose top is the tip's z, and of the layers under it up
   to the interface count, and an enforced tip's head at the middles of the
   layers it reaches under those. The last run's slices are the output: the
   pad, sliced on the layers it spans, joins the cage's sections as base
   areas, the rings and each post's disc on the layers a head's rings would
   take become the layer's interface areas and leave its base, and
   the enforced heads' slices go out apart for `clip_base`.

The returned `Output` carries one `LayerAreas` per planned layer (base,
interface and enforced heads), the number of
leading layers that are pad, the five tip and pillar counts, and the
milliseconds spent in the island map, in every build with the pad, and in the
slicing with the neck checks, which
`TreeSupport` writes into its profiler as `STAGE_ISLAND_JOINS`,
`STAGE_SCAFFOLD_BUILD` and `STAGE_SCAFFOLD_SLICE`.

## Builder changes

The SLA builder takes five additions. Each defaults to the behaviour SLA
printing had before, except the corrector cap, whose change the SLA suite
tolerates.

- Model anchors. `SupportTreeConfig::allow_model_anchors`, default true, gates
  the last-resort route of a head to the model body. The scaffold sets it
  false, so a head that reaches neither a pillar nor the ground is
  invalidated and counted as unrouted instead of standing on the figure.
- Runtime safety distance. `SupportTreeConfig::safety_distance_mm`, default
  0.5, is the clearance the head and pillar collision checks keep from the
  model; it used to be a compile-time constant. The scaffold passes the
  object's support xy distance.
- The cosine cap. The zero-elevation corrector bridge that walks a pillar out
  of the pad gap caps its length at the height it can descend divided by the
  cosine of the bridge slope, the length at which the walk reaches the
  ground.
- `set_ctl`. `SupportTreeBuilder::set_ctl` installs a `JobController` from
  outside `SupportTree::create`. The scaffold's controller stops on
  `Print::canceled()` and calls the print's cancel callback, so a cancelled
  slice throws `CanceledException` between build steps and leaves no support
  layer.
- Brace on demand. With `pillar_link_slenderness` above zero,
  `interconnect_pillars` measures each pillar's longest unbraced run between
  its ends and the ends of the pillar-to-pillar chains on its axis. A head's
  bridge into a pillar does not count: it ties the pillar to the model
  through a neck meant to snap. A pillar taller than the slenderness times
  its diameter takes chains to its nearest pillars within the link distance
  that `interconnect` can reach until its run fits and its chains lie in two
  vertical planes at least 45 degrees apart, since a zigzag chain stiffens a
  pillar only in the plane of the pair. A neighbour in a plane the pillar
  already has is taken only while the run is still too long. A pillar left
  with a run over the ratio or with its chains in one plane stays up and
  counts in `SupportTreeBuilder::unbraced_pillars`. The classic cascade and
  its helper pillars do not run under a slenderness.

## Emission and removal contract

The pad prints infill, not hollow walls. `remove_floating_toolpaths` and the
stability measurement read printed footprints, not drawn areas, so a pad with
an empty interior would leave every pillar foot standing on nothing printed
and the pass would strip the pillar. Every pad layer therefore takes the
sheath path the legacy tree uses on the bed layer. Layer 0 prints it
rectilinear at `raft_first_layer_density` with the first-layer flow, so the
bed holds the pad; the pad's top layer, `TreeSupport::m_pad_layers` layers up,
prints it at the same density with the support flow, so every foot lands on
printed material; the pad layers between only carry the top and print it at
half that density.

Above the pad, pillars, bridges and braces print as the legacy tree's base.
`generate_toolpaths` sets a base area's `need_infill` from `with_infill`, which
holds under every `support_base_pattern` but the default and none. Under those
two each area prints walls with no infill, laid by
`tree_supports_generate_paths`. Under a pattern with infill each area prints
walls and infill through `make_perimeter_and_infill`, or tree walls and
lightning infill under the lightning pattern, and `Params::base_cover` reads
such an area, lightning aside, as the area opened by half a support line. The
tip rings print as the top interface, in
the interface pattern and flow; with no interface layers configured the rings
stay in the base and print as base.

Base stays out of the model grown by the xy distance, except at an enforced
tip, one that painted enforcer facets or an enforcer modifier volume asked
for: the tip's own head under its rings stays out of the model alone. Without
that exception the band would cut the neck of a tip the user enforced beside a
wall and leave its rings over nothing. The exemption covers that head only;
pillars, bridges and braces keep the band, and the neck check cuts an enforced
head whose own neck would print in mid-air.

The floating pass runs on every pass that measures itself, and the scaffold
always does, since it forces miniature contacts on. Under Scaffold the pass
reads support as plate-only whatever `support_on_build_plate_only` says: a
printed piece belongs to the body only through a chain of overlapping printed
footprints down to the plate. The pass counts every piece it removes in
`m_floating_pieces_removed`.

The scaffold attributes no support area to a contact seed, so the coverage
`SupportAnalysis::measure` derives would mark every seed missing and read the
report Complete. `TreeSupport::generate()` overrides it under Scaffold: status
Unknown, `coverage_available` false, `coverage` and `missing_anchor_ids`
cleared. `support_unresolved` therefore stays quiet and no coverage warning
reaches the user; volume and stability stay measured from the printed
footprints.

## Measurement

`SupportAnalysis::Report` carries six counters: `tips_placed`, `tips_routed`,
`tips_dropped`, `islands_under_held` and `pillars_unbraced`, which the
scaffold fills from `ScaffoldSupport::Counts`, and
`floating_pieces_removed`, which the floating pass fills under every style
that runs it. They read zero under every other style except the last.

`PrintObject::_generate_support_material` logs one info line per object when
miniature contacts are on or the style is Tree Scaffold: `Support contact
layout for <object>`, followed by the critical regions without material, the
stability counts, the support volume, the seed counts and the six counters.
The profiler line `tree support time` names the three scaffold stages. Both
lines reach only the file the CLI's `--logfile` names; the per-tip drop lines
need `--debug 4`.

The test harness carries the counters into its rows:
`SupportValidation::Metrics` holds all six, `read_print_analyses` sums them
over a print's objects and `write_result` writes them under `metrics`. The
scaffold's corpus rows carry `harness = "scaffold_support"` and go to the file
`ORCA_SCAFFOLD_RESULTS` names, or to standard output when it is unset, since
`scripts/validate_miniature_supports.py` accepts only its own harness names.

## Implementation and verification

`tests/fff_print/test_scaffold_support.cpp` holds the `[ScaffoldSupport]`
cases. They cover the style and its keys in the config, the island map, the
result row, the pad's densities and the wall-only base on a shelf fixture with
nothing floating and no tip beside the column's wall, cancellation during the
build, rings printing as base without interface layers, a tip with no route
being dropped and counted, a head whose neck the band cuts on a slope beside
a wall being dropped with no ring left floating and no bare planned layer
left, interior tip thinning, the hold floor restoring contacts under tall
islands and capped by what a birth piece fits, a tip seeded under an unseeded
feature start with its ring on the layer its z tops and none under debris or
under a sliver the wall beside it holds, a sliver joining too high for its
wall counted under-held, braces on slender pillars, pillars widening toward
the pad by the branch diameter angle with the same tips routed, and a painted bar
underside and a painted column face beside the column's wall printing their
tips there at the corpus's widths and layer height, with two interface layers
and with none, where the unpainted bar prints none and no base but those
enforced heads enters the band. The SLA builder changes are covered in
`tests/sla_print/sla_print_tests.cpp`.

The hidden case "Scaffold support over corpus plate 3 in two poses"
(`[ScaffoldSupport][.]`) needs `ORCA_MINIATURE_CORPUS` pointing at the
directory holding `elf_test.3mf`. It slices plate 3's object under tree slim
and tree scaffold, first in the stored pose and then upright, times each
`Print::process()` and one `island_joins` call, writes one row per slice, and
requires of each scaffold slice: dropped tips at most a fifth of those placed,
no floating piece removed, a process time at most 1.75 times the tree-slim
slice's in the stored pose, whose painted enforcers add tips beside the walls,
and 1.5 times upright, `island_joins` within 2 s and a support volume at most
2.5 times the tree-slim slice's; the upright pose also requires no under-held
island.
