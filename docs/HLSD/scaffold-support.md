# Scaffold support — High Level Design

## Purpose and scope

Tree Scaffold is a support style for miniatures and other fine models printed
with a small nozzle. Its body is the SLA support tree: thin pillars standing on
a solid pad, joined by bridges and braces, each pillar ending in a small head
that fuses to the model. A print peels off the pad in one piece and the heads
snap off the model at their necks, which leaves small marks rather than the
broad scars a tree roof leaves on a figure's underside.

The style reuses the legacy tree's front half: overhang detection, contact
seeding, the miniature contact selection, the model risk field and the planned
support layer heights. It replaces only the body. `ScaffoldSupport::draw` builds
the SLA tree on the object mesh at zero elevation, adds a pad and slices both
into the planned layers, and `TreeSupport` prints those areas through the
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
  to a neighbour.

`tree_support_branch_diameter` sets the pillar diameter, as it sets the branch
diameter of the slim, strong and hybrid styles.

Under Scaffold the constructor of `TreeSupport` forces three settings the style
depends on, and the settings panel greys their fields out:
`support_miniature_contacts` reads on, because the scaffold places one tip per
contact the miniature selection keeps; `support_top_z_distance` reads zero,
because a tip fuses to the model rather than printing under a gap; and the
floating pass treats support as plate-only, because a pillar never stands on
the model. Auto-tilt names the style as its own generator and refuses to
evaluate an object that uses it, with the message "Auto-tilt cannot verify this
object: Tree Scaffold support is not evaluated."

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

After `plan_layer_heights` the generator branches. Under Scaffold it fills a
`ScaffoldSupport::Params` from the object config (toolpath width, pillar
diameter, `m_ts_data->m_xy_distance`, the two scaffold keys,
`max_bridge_length`, the interface layer count, the object's print z offset and
a pad thickness of 0.6 mm rounded up to the first planned layer at or above it)
and computes each planned layer's clip once, a `ScaffoldSupport::LayerClip`:
the model over the layer's whole height, and that model grown by the xy
distance together with the bottom-gap trim regions. It calls
`ScaffoldSupport::draw`, whose neck check reads the same clips. `drop_nodes`,
`smooth_nodes` and `draw_circles` do not run.

For each planned layer `TreeSupport` then clips the returned base areas against
the grown band and the machine border, and the interface areas against the
model alone, since the rings fuse to it. The base areas become
`BaseType` area groups with no infill and the interface areas become
`Roof1stLayer` groups; a group on one of the pad's layers carries `pad = true`.
The shared members `finish_layer_areas`, `normalize_interface_ids` and
`erase_empty_support_layers`, which `draw_circles` also calls, finish the
layers, and the generator records one emitted layer per filled planned layer
for the measurement. From there `generate_toolpaths` and the rest of the tail
run for every style alike.

## The ScaffoldSupport module

`src/libslic3r/Support/ScaffoldSupport.{hpp,cpp}` holds one function,
`ScaffoldSupport::draw`, that turns the contact nodes into per-layer areas and
counts what it did with them. It runs seven steps.

1. Tip selection. Every contact node becomes a tip, except an interior contact
   (`SupportNode::Placement::Interior`) whose overhang, shrunk by half of
   `max_bridge_length`, leaves nothing: under an overhang narrower than one
   bridge span the corner and contour tips on its rim already hold it.
2. The wall skip. The seam clips support inside the xy distance of the model,
   so a head whose neck stood in that band would lose its neck while its ring
   survived. The draw reads the object layer at the neck's bottom, one head
   width plus one toolpath width under the tip, and skips a tip whose centre
   lies within the xy distance of that layer's slices; the wall anchors that
   band as it does under the legacy tree. The layer just under the overhang is
   the wrong reference: on a sloped underside every contact sits a fraction of
   a layer past it, while a slope has receded by the neck's bottom and a wall
   has not. A skipped tip logs `scaffold tip skipped at (x, y, z): wall` at
   debug level and counts as neither placed nor dropped. The same test applies
   to the dropped contacts the hold floor may restore and to the tips it
   seeds.
3. The island hold floor. `SupportAnalysis::island_joins` maps every mid-air
   island of the model to the slab where it first meets the rooted body. An
   island needs one tip when its unjoined height is at most 1 mm, two up to
   5 mm and three above, counted greedily from the lowest tip and only where
   a tip stands a pillar diameter from every tip counted before it. A
   never-joining island's height runs to the object's top, and one whose
   height is at most 1 mm is mesh debris: it gets no floor, no tip and no
   count. An island with no tip and no dropped contact under it gets one tip
   seeded at the deepest point of its birth piece, at that piece's bottom, in
   the small grade. Short of the floor, the draw restores dropped contacts
   under the island, lowest first and among equals the one furthest from the
   island's tips. An island counts as under-held only when it holds fewer tips
   than both its floor and the number its birth piece fits, the points of a
   hexagonal grid at the pillar diameter inside the piece shrunk by half a
   pillar diameter, never fewer than one. A tip belongs to the island that owns
   the model piece over it on the overhang's own layer, one above the node's
   layer.
4. The alias merge. The front half can hand the same overhang spot on two
   consecutive layers, and the builder keeps one point of each pair within
   `sla::D_SP`. The draw visits the tips lowest first, among equals by seed
   id, and merges a tip standing within `sla::D_SP` in 3-D of a tip already
   kept into it, so no two tips it hands the builder are aliases. A merge logs
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
   zero elevation, no model anchors, bridge and pillar-link lengths at the
   scaffold bridge length, a 45 degree bridge slope, the object's xy distance
   as the safety distance and the scaffold brace slenderness. The mesh's
   ground level sits at the pad's top, so pillars end on it. A head the
   builder kept is a routed tip; a head it built and gave up on is an
   unrouted drop; a point that never got a head is a filtered drop.
   The neck check then reads what the seam would print of the build on each
   planned layer above the pad: the cage outside the clip's band, opened by
   half a support line since a sliver no line fits prints nothing, and the
   rings outside the model. `SupportAnalysis::floating_pieces`, the rule the
   floating pass applies, finds the pieces with no chain of overlaps down to
   the pad's top, and a head one of whose rings lies in such a piece is cut:
   its tilted neck crosses the band under its rings, or the builder left it
   with no pillar and no bridge, which a side head whose ground pillar fails
   keeps. The cut heads' points leave the point set, in order, and the
   builder runs again, since a run re-routes the neighbours of what it lost
   and can strand another ring. It repeats until no head is cut, at most six
   runs (plate 3 of the corpus needs five), and the heads the sixth run still
   cuts are dropped without another run, their rings left out of the output.
   Each run logs `scaffold build <n>: <points> points, <cut> heads cut` and
   each drop `scaffold tip dropped at (x, y, z): <reason>` at debug level,
   the reason `filtered`, `unrouted` (also for a cut head with no pillar and
   no bridge) or `neck`. When anything routed, `SupportTreeBuilder::add_pad`
   adds a pad of the pad thickness with a 1.6 mm brim and no wall.
7. Slicing. Each run slices its cage at the middle of every planned layer,
   and each routed head's own mesh at the tops of its top interface layer,
   the highest planned layer at or under the tip, and of the layers under it
   up to the interface count. The last run's slices are the output: the pad,
   sliced on the layers it spans, joins the cage's sections as base areas, and
   the rings become the layer's interface areas and leave its base.

The returned `Output` carries one `LayerAreas` per planned layer, the number of
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
  its ends and the bridge ends on its axis. A pillar whose run exceeds the
  slenderness times its diameter gets one chain to the nearest pillar within
  the link distance that `interconnect` can reach; a pillar no neighbour
  reaches stays up and counts in `SupportTreeBuilder::unbraced_pillars`. The
  classic cascade and its helper pillars do not run under a slenderness.

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

Above the pad, pillars, bridges and braces print exactly as the legacy tree's
base: walls around each area with no infill, laid by
`tree_supports_generate_paths`. The tip rings print as the top interface, in
the interface pattern and flow; with no interface layers configured the rings
stay in the base and print as base.

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
a wall being dropped with no ring left floating, interior tip thinning, the
hold floor restoring contacts under tall islands and capped by what a birth
piece fits, a tip seeded under an unseeded feature start with none under
debris, and braces on slender pillars. The SLA builder changes are covered in
`tests/sla_print/sla_print_tests.cpp`.

The hidden case "Scaffold support over corpus plate 3 in two poses"
(`[ScaffoldSupport][.]`) needs `ORCA_MINIATURE_CORPUS` pointing at the
directory holding `elf_test.3mf`. It slices plate 3's object under tree slim
and tree scaffold, first in the stored pose and then upright, times each
`Print::process()` and one `island_joins` call, writes one row per slice, and
requires of each scaffold slice: dropped tips at most a fifth of those placed,
no floating piece removed, a process time at most 1.5 times the tree-slim
slice's, `island_joins` within 2 s and a support volume at most 2.5 times the
tree-slim slice's; the upright pose also requires no under-held island.
