# Scaffold support — High Level Design

## Purpose and scope

Tree Scaffold is a support style for miniatures and other fine models printed
with a small nozzle. Its body is the SLA support tree: thin pillars standing on
a pad, joined by bridges and braces, each pillar ending in a small head
that fuses to the model. A print peels off the pad in one piece and the heads
snap off the model at their necks, which leaves small marks rather than the
broad scars a tree roof leaves on a figure's underside.

The style reuses parts of the legacy tree's front half: overhang detection,
the contacts enforcers ask for and the planned support layer heights. It places its tips with its own need planner, one where the
print needs a tip rather than one per overhang, and replaces the body. `ScaffoldSupport::draw` builds
the SLA tree on the object mesh with its ground on the bed, adds a pad and
slices both into the planned layers, and `TreeSupport` prints those areas through the
legacy tree's toolpath, floating-removal and measurement tail. The style
generates FFF support only; nothing in the SLA printing pipeline changes.

## Settings and eligibility

`support_style` gains the value `tree_scaffold`, labelled Tree Scaffold and
listed last in the style menu. Like the other tree styles it needs a tree
`support_type`: `SupportParameters` resolves the style to grid under a normal
support type.

Three settings belong to the style and show only under it:

- `scaffold_bridge_length`, 3 to 30 mm with a 12 mm default, bounds every
  bridge from a tip to its pillar and every brace between pillars.
- `scaffold_brace_slenderness`, 5 to 40 with a 15 default, is the unbraced
  height of a pillar, in pillar diameters, above which the builder braces it
  to its neighbours.
- `scaffold_brace_diameter`, 10 % to 100 % with a 60 % default, is a brace's
  diameter as a share of the pillar diameter, never under two support lines.

No setting sets how many tips the style places: the need planner, step 1 of
The ScaffoldSupport module, places one where the print needs it, and paint,
blockers and the Scaffold Points tool's hand edits override it.

Under Scaffold the slice reads no `support_contact_min_distance`, and the
settings panel greys the field under the style.

`tree_support_branch_diameter` sets the pillar diameter, as it sets the branch
diameter of the slim, strong and hybrid styles, and
`tree_support_branch_diameter_angle` sets how fast a pillar widens toward the
pad, as it sets how fast a tree branch widens toward its root. The settings
panel shows the angle under Scaffold as well as under the organic style.

Under Scaffold three settings the style depends on read fixed, and the
settings panel greys their fields out. The constructor of `TreeSupport` forces
two of them: `support_miniature_contacts` reads on, because miniature contacts
put the pass on the measured path, `m_analyze`, the only one that runs the
floating pass and the support analysis whose report carries the scaffold's
counters, and `support_top_z_distance` reads zero, because a tip fuses to the
model rather than printing under a gap.
`remove_floating_toolpaths` applies the third: it roots support on the plate
alone under `m_scaffold || support_on_build_plate_only`, because a pillar never
stands on the model. Auto-tilt names the style as its own generator and refuses
to evaluate an object that uses it, with the message "Auto-tilt cannot verify
this object: Tree Scaffold support is not evaluated."

A project that names `tree_scaffold` in a build without the style loads with
the default style substituted, and the scaffold keys are dropped as unknown
keys, so the style needs no migration code. A preset or project that carries a
`scaffold_density` key loads the same way, with that key dropped.

## The seam in TreeSupport::generate()

`TreeSupport::m_scaffold` is true when the resolved style is
`smsTreeScaffold`, and every scaffold branch in the generator reads it. Unless
the object holds a valid baked list, which Baked contact points describes, the
front half runs as it does for the slim style with miniature contacts on,
`detect_overhangs`, the contact seeds and the risk field, except that the
contact selection does not run: the scaffold sets
`m_problem.contact_min_distance_mm` to 0. The contacts only carry the
enforcers' requests to the planner. A slice from a valid baked list runs
`detect_overhangs` and none of the others.

Before `plan_layer_heights`, under Scaffold, the generator fills a
`ScaffoldSupport::Params` from the object config (toolpath width, pillar
diameter, `m_ts_data->m_xy_distance`, the three scaffold keys,
`max_bridge_length`, the interface layer count and the object's print z
offset). On a valid baked list it calls `ScaffoldSupport::baked_tips` with the
list, the parameters and the detector's threshold angle. Otherwise it gathers
the support blockers as `detect_overhangs` does, `slice_support_blockers` plus
painted blocker facets, and calls `ScaffoldSupport::place_tips` with the
contacts, the parameters, the threshold angle and the blockers.
`plan_layer_heights` then takes every chosen tip's z as a layer top
besides the contacts' own, so a tip the planner placed where no contact stands
prints its top ring on the layer right under it, not under an air gap. Under
every other style that list is empty.

After the plan the generator branches. Under Scaffold it sets the pad thickness
to 0.6 mm rounded up to the first planned layer at or above it and computes
each planned layer's clip once, a `ScaffoldSupport::LayerClip`: the model over
the layer's whole height, and that model grown by the xy distance together
with the bottom-gap trim regions. It calls `ScaffoldSupport::draw` with the
chosen tips, and the draw's neck check reads the same clips. `drop_nodes`,
`smooth_nodes` and `draw_circles` do not run.

For each planned layer `TreeSupport` then clips the returned areas.
`ScaffoldSupport::clip_base` keeps the base out of the grown band and adds back
the exempt heads, the enforced tips' heads and the heads the neck check
exempted, kept out of the model alone; the machine border clips
the result, and the interface areas stay out of the model alone, since the
rings fuse to it. The base areas become `BaseType` area groups with no infill
and the interface areas become `Roof1stLayer` groups; a group on one of the
pad's layers carries `pad = true`.
The shared members `finish_layer_areas`, `normalize_interface_ids` and
`erase_empty_support_layers`, which `draw_circles` also calls, finish the
layers, and the generator records one emitted layer per filled planned layer
for the measurement. From there `generate_toolpaths` and the rest of the tail
run for every style alike.

## Baked contact points

A Tree Scaffold object can hold a baked list of contact points, and a slice
builds the scaffold from that list in place of the contacts the front half
would place. The list fixes the tips from one slice to the next and lets the
user add, move, resize and remove them before printing.

### The list on ModelObject

`src/libslic3r/ScaffoldPoints.hpp` declares the shared types, and `ModelObject`
holds the list in four members: `scaffold_points`, `scaffold_points_status`,
`scaffold_points_pose` and `scaffold_points_mesh_box`. A `ScaffoldPoint` holds
a position in the frame of `ModelObject::raw_mesh()`, a head size and an
`enforced` flag. The size is a preset, not a radius: `Light` asks for a disc
two support lines wide and `Heavy` for one four lines wide, at the support
toolpath width of the slice that builds it, so a list survives a nozzle or
line width change. The status reads `NoPoints`, `AutoGenerated` or
`UserModified`. Under `NoPoints` the object holds no list whatever the vector
holds; under either other status the list is in use even when it is empty,
and an empty list builds no tip. The pose is the linear part of the matrix of
the instance the list was written under, and the mesh box is the object's
`raw_mesh_bounding_box()` at that moment.

`assign_copy`, the undo stack's serialization and `Selection::copy_to_clipboard`
carry all four members. `Print::apply` compares them through
`model_scaffold_points_data_changed` and, on a change, invalidates only the
support step of the object's PrintObjects, so a new list keeps the slices.
`Print::process` shares support layers between two objects only when their
lists match, since a shared layer would print one object's list under the
other.

### Validity

`TreeSupport::generate()` builds from the list when the resolved style is Tree
Scaffold, the status is not `NoPoints` and the list passes two tests for the
PrintObject being sliced:

- The pose. `ScaffoldSupport::baked_pose_valid` takes the change
  `M = L * pose.inverse()` from the stored pose to the linear part `L` of the
  matrix of the PrintObject's first model instance, and accepts it when `M`
  keeps lengths and keeps the Z axis, each to 1e-6. A turn about Z or a mirror
  in X or Y passes: the points turn with the mesh and every underside they sit
  on still faces down. A tilt, a mirror in Z or a scale fails, since each
  changes which faces overhang or how far apart the points stand.
- The mesh. The object's `raw_mesh_bounding_box()` must match the stamped mesh
  box within 1e-3 mm in every min and max coordinate. Reload from disk, Replace
  with STL and part-level transforms call no clearing hook, and the stamp
  catches those that move the box. An edit that leaves the box unchanged, such
  as moving an interior part, goes undetected.

The pose test reads the instance matrix, not `PrintObject::trafo()`:
`print_objects_from_model_object` builds that trafo with
`get_matrix_with_applied_shrinkage_compensation`, so under any filament
shrinkage other than 100 % it carries a scale and every list would read stale.
All instances of one PrintObject share one trafo, so the first instance speaks
for all of them, and two instances in different poses slice as two
PrintObjects that judge the list apart. A list that fails either test is
stale: that PrintObject slices as though the object held no list, and
`PrintObject::_generate_support_material` adds the warning "Scaffold points
are stale for this pose; auto contacts used." to its support step. Turning the
object back to a valid pose makes the next slice build from the list again.

### The baked pass

A baked slice skips every stage that places or thins contacts:
`build_required_regions`, the risk field, `generate_contact_points`,
`build_contact_seeds`, and `select_contacts` with its erase loop, so no
contact a painted enforcer asks for reaches the list, and `baked_tips` reads no
blocker. Painted enforcers and blockers therefore change nothing while a list
is in use, and the tool says so. `detect_overhangs`, the support layers, the
preview cache and the `Params` fill still run, and the generator calls
`ScaffoldSupport::baked_tips` in place of `place_tips`.

`baked_tips` maps each point through `trafo_centered()` into the frame the
builder uses and adds the object's print z offset. It snaps the point's z to
the bottom of the object layer holding it, the first layer whose top lies
above the point or the top layer when none does, and files it on the layer
under that one, where a contact under an overhang is filed. The list, in the
model and in the 3MF file, stores no head axis, so `baked_tips` builds the plan
input `place_tips` builds, with no blocker, and gives each point that is not
enforced the axis the planner's neck search reads at that spot, `neck_axis`: a
birth or stability tip Generate copied leans its neck as the auto slice leaned
it. The input reads no blocker because `Necks::clear` refuses every axis at a
blocked cell: a point under a blocker drawn after the list was written would
lose its lean, and the wall skip would take out a point whose neck clears only
leaning. It then runs steps 2 to 4 of the auto path on those sites:

- The wall skip calls the same `wall_skip` function `place_tips` calls, at
  each point's neck end along its axis. A point placed with the tool is
  enforced and never skipped; a point Generate copied carries the flag its tip
  had.
- The island count reads the islands by the planner's rule and gives none a
  tip: an island with a point on it is held, one the birth rule holds with no
  tip needs none, and any other is under-held and recorded as a bare island.
  It marks every point under a mid-air island as holding it, so the builder
  retries that point's head along leaning axes as it retries an auto slice's
  island holders, and a list Generate copied routes the island tips the auto
  slice routed.
- The alias merge runs unchanged, so a point within `sla::D_SP` of another
  merges into it.

Each site carries its point's index as `TipSite::source` and its size's disc
width as `TipSite::grade_mm`, which `tip_grades` keeps as it keeps the
planner's grade on an auto slice. From `plan_layer_heights` on, a baked slice runs as
an auto one: the tips' z values top planned layers, `draw` builds and checks
the heads, and the measurement reads the printed footprints with no seed and
the scaffold's coverage override.

### The record

Every scaffold slice installs a `ScaffoldRecord` on its PrintObject, read
through `PrintObject::scaffold_record()`; other styles install none.
`clear_support_layers` drops it with the support layers, and a PrintObject that
reads a shared owner's layers copies the owner's record. The record holds
`baked`, `stale`, the instance pose and toolpath width the slice used, the bare
islands and a list of tips, all in the raw-mesh frame: `generate()` takes the z
offset off each site and maps it back through the inverse of
`trafo_centered()`. The bare islands are `draw`'s `Output::bare_islands`, the
places where an island prints with no tip holding it. The tips and the bare
islands differ by pass:

- A baked record holds one tip per list point, in list order, with the point's
  size and flag. Its result is `draw`'s outcome (`Routed`, `Filtered`,
  `Unrouted` or `Neck`) for a point with a drawn site, `Wall` for a point the
  wall skip took out and `Merged` for any other point without a site. Its bare
  islands are the islands the island count reads under-held, one per island
  `islands_under_held` counts: at the tip the planner's rule would stand under
  each, or at the birth point of one no neck clears.
- An auto record holds one tip per site handed to `draw`, with `draw`'s
  outcome and a size read back from the tip's grade: `Heavy` above three
  support lines, `Light` otherwise. Its bare islands are the birth points of
  the islands the plan could not hold and of those whose holders all failed to
  route, one per island `islands_under_held` counts.

`scaffold_points_from` turns a record into a list by keeping its `Routed`
tips. A baked record with any point not `Routed` or any bare island adds the
warning "Scaffold points for <object>: <d> dropped, <b> islands left without a
tip." to the support step, and an auto record with any bare island adds
"Scaffold support for <object>: <b> islands print with no tip holding them."
The Scaffold Points tool draws either record's bare islands.

### The Scaffold Points tool

`GLGizmoScaffoldPoints` edits the list. It opens from the toolbar alone, with
no keyboard shortcut, on one full instance whose effective config enables
support with a tree support type and the Tree Scaffold style. Edits change only
the tool's cache, each under its own undo snapshot. A click adds an enforced
point of the preset size on the model, except on a face whose normal lies
within 30 degrees of straight up, the complement of the builder's 150 degree
`normal_cutoff_angle`, where the builder would filter the head. The tool writes
the model three ways:

- Apply writes the cache as a `UserModified` list with the selected instance's
  pose and a fresh mesh box stamp, makes the instance's plate current and
  reslices.
- Generate replaces the list with the routed tips of an auto slice as an
  `AutoGenerated` list, with the record's pose and a fresh mesh box stamp. It
  asks first when the list is user-modified and not empty. When the plate's
  finished slice of the object is auto and not stale, it copies from that
  slice at once. Otherwise it stashes the list, sets `NoPoints` so that the
  slice places contacts automatically, and reslices. It copies once the whole
  background process has finished, G-code export included, since the copy
  invalidates the support step, and only when the object still exists, its
  model-part meshes are the ones the Print sliced and the instance pose
  matches the record's within 1e-9. When a check fails, the slice did not
  finish or the tool closed first, it restores the stash and writes nothing.
- Revert to auto clears the list to `NoPoints` and invalidates the support
  step without reslicing.

After a slice from the list, the tool colours each point by its result when
the record matches the cache point for point, shows a point added since as
not yet sliced and draws each bare island as a flat disc. Slice builds from
the applied list while the tool holds unapplied edits, and closing the tool
with such edits asks whether to apply them.

### The 3MF file

The exporter writes `Metadata/scaffold_points.txt` when any object's status
is not `NoPoints`. Its first line is `scaffold_points_format_version=0`, and
each such object takes one line with its 1-based index in the model, its
status, its pose as nine doubles in row order, its mesh box as six doubles and
its points as groups of `x y z size enforced`, for example:

```
object_id=3|status=1|pose=1 0 0 0 1 0 0 0 1|box=-9.5 -8 0 9.5 8 31.2|1.25 -3.5 4.1 0 1 2 -3.5 4.1 1 0
```

Doubles go out with `%.17g` and point coordinates with `%.9g`, the widths
that bring each value back bit for bit. The importer parses every number
whole with `from_chars`. A missing header, an unknown version, a malformed or
repeated object line, or a status other than 1 or 2 loads no list for any
object, because half a list would build a scaffold the user never saw. A point
with a non-finite coordinate, or a size or flag other than 0 or 1, is dropped
alone with a warning. A build without the reader skips the file as an unknown
entry and loads every object with no list. When the loader zeroes the offset
of a single-volume object's only volume, it moves the brim ears, the list and
the mesh box by the same shift, because that shift moves the raw-mesh frame
they are stored in.

### Model edits

An edit that reshapes or reorients the mesh clears the list to `NoPoints`,
since the points no longer lie on the surface:

- `ModelObject::scale`, `mirror` and `scale_mesh_after_creation`, the last
  also reached by the rescale of a too-large object at load;
- both `ModelObject::rotate` overloads when the angle is not zero, which
  covers auto-orient; import rotates every object by `preferred_orientation`,
  zero by default, and that call keeps the list;
- the new objects `convert_units`, `clone_for_cut` and `merge_volumes` build;
  `ModelObject::split` builds its pieces with `Model::add_object()`, so they
  start with no list;
- `Plater::changed_mesh`, which repair, simplify and smoothing all call.

An edit that moves the mesh without reshaping it carries the list.
`ModelObject::translate`, and through it `center_around_origin`, moves every
point and the mesh box with the raw mesh. `ObjectList::merge` maps each
source's points into the merged object, marks the result `UserModified` when
any source held a list, and stamps the new instance's pose and the merged mesh
box. Copy and paste copies the list. An instance transform leaves the list
alone, and the validity test reads the new pose at the next slice. The undo
stack serializes the four members, so undoing an edit restores the list it
cleared.

## The ScaffoldSupport module

`src/libslic3r/Support/ScaffoldSupport.{hpp,cpp}` and
`src/libslic3r/Support/ScaffoldPlan.{hpp,cpp}` run seven steps between them.
`ScaffoldSupport::place_tips`, the selection, runs steps 1, 2 and 4, reads no
planned layer and returns the tips with what the planner could not meet and
the object mesh the planner read, which is why it can run before the layers
are planned.
`ScaffoldSupport::draw` runs steps 5 to 7 on those tips and the planned
layers, turns them into per-layer areas and counts what it did with them.
`ScaffoldSupport::baked_tips` stands in for `place_tips` on a baked list and
runs steps 2 to 4 on its points, as Baked contact points describes.

1. The need planner. FDM has no peel force, so area alone never calls for a
   tip. `ScaffoldSupport::plan_tips` places one only where the print needs
   it: where an island starts, where an underside hangs too far past its
   anchors, and along a stem that reaches farther from what holds it than its
   thickness allows. It reads
   a `PlanInput` that `prepare_plan` builds once per slice: the object's
   layers sampled on a lattice at half the support toolpath width, each cell
   labelled by the slab piece holding its centre (a point off the centre, as
   a corner on an outline is, reads its piece off the outlines, since its
   cell's label may name a neighbour), the pieces from `build_components`
   with the ground at the first slab, each layer's down-facing surface
   (`diff_ex` of its slices and the layer below's), each layer's
   self-support step `a`, the height of the layer below over the tangent of
   the detector's threshold angle (the threshold plus 1 degree, or 30 degrees
   when it reads 0), and the blockers rastered per layer. It also carries
   `max_bridge_length`, the longest line an underside bridges, the most a
   birth's or stability tip's neck may lean, `max_head_tilt_rad`, and the
   object mesh with its AABB tree, a `ScaffoldSupport::ObjectMesh` that
   `place_tips` builds once and hands to `draw`, off which the planner reads
   the normal the builder aims a head along. The reach `R`, how far an
   underside may hang past the step, is two support lines: the first line past
   a held edge bonds its side to a held line and the second to a line hanging
   by one, but a third would lie against a line hanging by two. The
   `PlanInput` lives only through `place_tips`, or `baked_tips` on a list.

   The planner walks the layers bottom up and tracks parts itself: a part is
   a connected set of pieces, and a part merges when a piece overlaps two
   sets below it. On a lifted object nothing is rooted, so plate 1's 65
   islands never join the ground, and every height below runs to a merge.
   - Enforced. Every contact an enforcer asked for becomes a tip and an
     anchor.
   - Birth. Each birth piece off the bed, nothing under it, is read in turn
     unless an enforced tip on it stands in for its tip:
     - A piece every point of which lies within the slab below's step `a` of
       that slab's material continues it as an overhang does, as `layer` reads
       a cell the step carries, and is no island: a rib stepping out less than
       `a` a layer would otherwise read as a birth on every layer.
     - Debris, a part that never merges and stands at most 1 mm, takes no tip.
     - A nub, a birth piece no wider than two support lines that merges having
       stood free less than 0.1 mm, prints its layer as it hangs and waits for
       its merge. The bound is the resin reference's, where every birth
       standing free 0.1 mm or longer carries a contact; one counted in layers
       would move with the layer height. A nub's cells hang from the nearest
       other material on their layer, which the merge bridges them from, so a
       nub standing far from it and wider than a small head's disc takes an
       underside head on its own layer. At the merge, a nub whose own part
       holds a tip is held by it. One lying wholly within the merge slab's `a`
       plus `R` of what the held parts, those a tip or the bed holds, have
       under the merge hangs from their tips: the merge layer bridges to it no
       farther than to any underside the planner leaves bare, and it prints one
       sagged layer. Any other nub takes its birth tip then, which holds its
       part for the nubs after it, so of two nubs meeting each other in
       mid-air the first takes a tip and the second hangs from it.
     - Any other island takes one birth tip where a neck clears, at the
       deepest point of its birth piece (`inscribed_point`) or at the cell of
       the piece nearest it whose neck clears, straight down or leaning as the
       neck search below finds, and one no neck clears counts as under-held.

     An island carries its part by the
     elder rule: where parts meet, the one born lowest carries on, a rooted
     part first, and every other part's eldest birth ends there; a birth that
     never ends carries to its part's top. The tip takes the heavy disc, four
     lines across, when its island carries its part more than 2 mm and the
     model within one toolpath width over the tip, the depth the pin reaches,
     fills at least twice as many lattice cells of the heavy disc as of the
     small one, and the small disc otherwise: on a thin section the heavy disc
     adds scar and no hold. An island left without a tip anchors what grows on
     it. The plan lists every island in `Plan::islands` with its birth point,
     the deepest point of its birth piece at the piece's bottom, and how it is
     held: `Tip`, by its own birth tip, the enforced tip on its birth piece or,
     for a nub, the underside head its own layer placed; `Hung`, from the
     parts it met, with every tip on the held parts within its hang, whatever
     need placed it, as its holders and `rooted` set when the bed held one of
     them; `NoNeck`, the under-held ones; or `Debris`.
   - Underside. Each cell carries a run, how far it hangs past what anchors
     it: 0 under a head, and otherwise the least, through the layer's
     material, of a neighbour's run plus the step between them, where a cell
     standing over the layer below starts at the run of the cell under it.
     The run on a cell is that least less `a`, so a slope steeper than the
     threshold accumulates nothing and a shallow one accumulates only its
     excess over `a`, layer by layer. A cell within the reach of a head on its
     own layer passes 0 up, as material bridging to that head holds what grows
     on it. The distance runs over the 16 moves of a king and a knight, which
     overstate a straight line by at most 3 %. A cell the layer below does not
     stand under is due where its run passes the reach plus half a cell, and
     each due cell is answered in turn:
     - The flap floor. The cells hanging past the step, connected on a layer
       and continued across layers where one lies within two cells of a
       hanging cell of the layer below, form a flap, whose cell count over all
       its layers is its projected hanging area. A due cell on a flap smaller
       than the small head's disc, pi w^2 or 0.15 mm2 at a 0.22 mm line,
       prints as it hangs, since the scar would outweigh the sag.
     - The bridge hold. Orca prints a bottom over tree support as a bridge
       along one direction it picks from the model's own layers, without the
       heads, and its perimeters follow the layer's contour. A due cell in the
       wall zone, within two lines of the layer's contour, takes no head where
       a straight line through it, among 64 directions 2.8 degrees apart, stays
       within one cell of the zone and ends on held cells on both sides within
       `max_bridge_length`. A due cell farther in than the zone takes none only
       where every direction ends so, on a held cell or on the contour once the
       zone is answered: no cell of it left due, and none hanging where no head
       could stand or across a gap from one, since the contour sags with such a
       cell. So a frontier between held ends, such as a shallow ramp between
       two walls, bridges, and a flat face born whole takes a ring of heads on
       its rim, which holds whichever direction Orca picks. `max_bridge_length`
       0 bridges nothing. A bridged cell passes its run up, as any cell out of
       a head's reach does.
     - Heads. The rest calls for heads, the wall zone's due cells first while
       any is left. A head covers the cells within the reach plus `a` plus one
       toolpath width in 3-D, and goes on a cell not stood under by the layer
       below that covers at least one due cell, the one covering the most due
       cells and pending cells of the layers under it: first on the rim,
       within two lines of a wall, with the small disc wholly on the layer's
       material; then on the rim; then with the disc on the material; then
       anywhere. A wall is a cell empty on the layer and on the layers above
       until their steps add up to two lines, since a boundary that advances
       slower than that is no frontier. Among equals the head goes nearest the
       due cell that hangs farthest, then lowest in y, then x. It stands only
       where it is eligible, not within the reach of a head already on the
       layer, and where the builder's normal stands within 60 degrees of
       straight down: the builder tilts a head at most 45 degrees, so such a
       head meets its face within 15 degrees of the normal, where a rim head
       beside a steep wall would be aimed into the wall. That normal is
       `sla::normals` read at the head's radius, one toolpath width: the
       normal of the face nearest the point, or, where that nearest point lies
       within the radius of a vertex or an edge, the average of the faces
       sharing it. Every underside head takes the small disc.
     - The termination guard. A due cell a head covered in 2-D and left due
       hangs across a gap from it, and hangs, so no later head stands for it
       on the strength of that cover.

     Due cells no head can answer hang. A cell that hangs stays pending while
     it lies within a head's cover under the layer walked, and one that falls
     out of that cover unanswered past one and a half reaches counts in
     `underside_unmet_mm2`. Each head keeps, as `PlannedTip::answered_mm2`,
     what past one and a half reaches it answered: the due cells it brought
     within the reach on its layer and the pending cells its cover took,
     which hang again if the head does not route.
   - Stability. Each piece of each slab is measured against its own window
     `W`, the larger of 3 mm and three times its stem's thickness `t`: the
     piece's own or, where thicker, that of a piece it grows out of 1 mm under
     its top, reached down through the pieces under it. A piece's thickness is
     the diameter of the largest circle inside it, which `inscribed_radius`
     finds by bisecting inward offsets to 0.01 mm, the search
     `inscribed_point` runs for a birth tip; the planner fills it for every
     piece in parallel before it walks, about 1.2 s on plate 1's figure, which
     a baked list, never planned, does not pay. A rod bends as F L^3 / (E
     t^4), and the nozzle pushes a section with a force that grows with the
     section it prints, so a fixed reach per thickness holds a fixed
     deflection; three thicknesses meet the 3 mm floor at a 1 mm stem. The
     piece's lever is how far the farthest point of its outline at its top
     stands from what holds its part: the part's tips in 3-D, the first slab's
     outline under a part the bed roots, and the first section under the stem,
     no deeper than the lever, that holds a piece twice as thick as the stem
     and so 16 times as stiff, which clamps it. A point stands from an outline
     by the hypotenuse of its distance to it in x and y, 0 inside, and its
     height over it. A part held by none of these reads a point's height over
     its highest anchor. A piece whose lever passes its window takes a small
     tip at a corner of its stem's down-facing surface above that clamping
     section and within `W` under its top, standing at least `W / 2` from what
     holds the part, since a tip beside an anchor shortens no lever, and
     within `W` of the farthest point, since a tip farther off cannot bring
     that point within the window. The corner whose face turns least from
     straight down goes first, then the one farthest from what holds the part.
     The face's turn is its slice angle, atan of the two slabs' rise over the
     corner's distance to the slab two under it, taken to the whole degree,
     since the slices read a flat face's angle only to float rounding. The
     corner must be eligible, and the builder's normal there must stand within
     60 degrees of down, as for an underside head. Where no candidate both
     clears straight down and has its builder's normal within the cap, the
     first in the same order whose builder's normal stands within the cap and
     whose neck clears leaning takes the least lean that clears, by the birth
     search's steps, cap and azimuth, and carries it as its axis. Every
     candidate is tried straight down before any leans, as in the birth
     search, so a steeper corner whose neck clears straight down takes the tip
     ahead of a flatter one whose neck clears only leaning. On a steep, bumpy
     surface, as plate 1's sword blade is
     between its widening point and its guard, no neck from the small faces
     within the cap clears the surface below straight down, while one leaning
     off the surface does. The cap stays on the builder's normal, not on the
     axis, since the head's pin still meets that face. Where a 45 degree face
     meets an upright side at a square edge that normal averages to exactly 60
     degrees, which counts as within. So legs, torso sides and head sides,
     which stand near upright, stay bare, while a raised spear, a fist, a
     forearm's underside or a hem takes holds, and a panel hanging from its
     corner takes them on its keel, not on its drafted sides. A square-edged
     keel more than 45 degrees from down reads past the cap at its corners,
     where the builder's normal averages it with the upright side. A branch
     with no such corner counts once in `islands_slender`, and the pieces
     growing out of it carry that count, so on a figure standing on a base
     each leg taller than three thicknesses counts once and the torso over
     them adds none. A piece with no corner, or still past its window with its
     new tip, is measured again 1 mm higher, and a piece takes the highest
     such height of the pieces under it. Rooted parts run the rule as floating
     ones do. A level member shares its slab pieces with the body it joins, so
     it reads the body's thickness, and the underside rule's bridge holds it.
     A plate reads as a rod as thick as the plate.

   A tip's neck runs one neck depth, a head width plus a toolpath width, from
   the tip along its axis. It clears when no blocker covers the tip's cell and
   its end, on the first slab whose top reaches it, stands outside the band.
   The lattice rejects first, where a cell of the model lies within the xy
   distance plus half a cell's diagonal, and `wall_band` then reads the band
   as the seam and the wall skip build it, the layer's slices grown by the xy
   distance with miter joins, which reach up to three times the distance out
   from a sharp corner. A tip on the model's edge, as every stability tip is,
   stands where the lattice's rounding decides, so the planner reads the exact
   band the wall skip reads, and the skip keeps the tips the planner counts
   on. The neck's shaft also
   crosses no material at the middle of each slab between its end and the
   tip, so a thin shelf the end has passed still turns the neck. Straight
   down, the run of material right under the tip is the tip's own face to the
   lattice's resolution and is not crossed: a tip on a face's edge, as every
   stability tip is, stands over the slab below's contour, whose cell reads
   material about half the time, and a steep face's column stays in its part
   for a few slabs. A leaning neck has no such allowance, so any material on
   its shaft is crossed: a birth's stands over no face of its own, and a
   stability tip's clears only where its shaft leaves its face's column at
   once. An underside tip stands only where its neck clears straight down. A
   stability tip leans only where no candidate both clears straight down and
   holds its head within the face cap, and then takes the least lean that
   clears at its corner, `Necks::leaning`.
   A birth's neck search, `Necks::search`, tries the deepest point and
   then the piece's cells nearest it first, straight down. Failing that it
   leans the neck in steps of asin(cell / neck depth), 5.2 degrees at a
   0.22 mm line, each moving the end one cell sideways, up to
   `max_head_tilt_rad`, 45 degrees, the bridge slope the builder saturates a
   head at: at the least lean at which any candidate clears, it takes the
   first such candidate and, among azimuths one cell apart at the end, the
   one whose end stands farthest from the model's material on the end's slab.
   A birth or stability tip carries its lean as `TipSite::axis`, which the
   wall skip reads its band along and the builder aims its head along. Every
   other tip carries a zero axis and keeps the builder's normal: a head on a
   side-on face already tilts to the cap along that normal and routes there.
   `neck_axis` runs the same search at one spot, which is how a baked list
   reads a point's axis again. The planner logs
   `scaffold plan: <n> birth, <n> underside, <n> stability, <n> enforced
   tips` at debug level. On plate 1's figure it runs in about 1.8 s, 1.2 s of
   it filling the piece thicknesses, after `prepare_plan` spends about 0.08 s
   once per slice.
2. The wall skip. The seam clips support inside the xy distance of the model,
   so a head whose neck stood in that band would lose its neck while its ring
   survived. The selection reads the neck's end, one head width plus one
   toolpath width from the tip along its axis, straight down where it has
   none, and skips a tip whose neck end lies within the xy distance of the
   slices of the object layer there; the wall anchors that band as it does
   under the legacy tree. The layer just under the overhang is the wrong
   reference: on a sloped underside every contact sits a fraction of a layer
   past it. The skip reads the planned neck's end, not the built neck: under a
   slope the head tilts along the underside's normal, and the neck check in
   step 6 exempts a cut head whose tilted neck bottoms outside the band. A
   birth or stability tip whose neck clears only leaning would stand in the
   band straight down, so without the axis the skip would take out the tip
   its lean placed. A skipped tip logs
   `scaffold tip skipped at (x, y, z): wall` at debug level and counts as
   neither placed nor dropped. The planner already keeps its tips off the
   band on its lattice, so the skip reads the band exactly for the few its
   lattice passed. An enforced tip is
   exempt: a contact a support enforcer asked for, `SupportNode::is_pinned`,
   which under Scaffold marks every contact on an overhang an enforcer
   covers, whether painted facets or an enforcer modifier volume, and every
   vertical enforcer point painted facets place, becomes a `TipSite` with
   `enforced` set, and the skip keeps it however close the wall stands, so
   the tip fuses where the user asked for support and leaves its scar there.
3. The island count, on a baked list only, where it gives no island a tip
   and names the islands the list leaves unheld, by the planner's island rule
   so that a list and an auto slice count an island alike.
   `SupportAnalysis::island_joins` maps every mid-air island of the model, a
   birth piece off the bed with nothing under it, to the pieces it owns; the
   map's pieces are the plan input's, index for index, since both come from
   `build_components` over the same slabs and ground. A point belongs to the
   island that owns the model piece over it on the overhang's own layer, one
   above the node's layer, and holds that island, as the planner's one birth
   tip holds it, however far the island stands free and however many more
   tips its birth piece has room for. An island with no point on it is read
   by the planner's birth rule, `read_births`. A birth piece continuing the
   slab below and debris need no tip. A nub needs none where its merge holds
   it as the planner's merge would, walked bottom up over the list's points:
   a point of the list stands on its own part, or it lies within the merge
   slab's `a` plus `R` of a part the bed or a point of the list holds. Any
   other nub, like any other birth, needs the tip the rule would stand under
   it, and a nub the rule tips holds its part for the nubs meeting it after
   it, as the planner's tip would. An island that needs a tip the list lacks
   is under-held, and so is one no neck clears, which the plan counts unheld
   too: each counts in `islands_under_held` and is named in the bare islands,
   at the tip the rule would stand under it, or, where no neck clears, at its
   birth point, the deepest point of its birth piece at the piece's bottom.
   Each logs
   `scaffold island at (x, y, z) unheld: <cause>` at debug level, the cause
   `no point` or `no neck`.
4. The alias merge. Two tips can stand on one spot on two consecutive layers,
   and the builder keeps one point of each pair within
   `sla::D_SP`. The selection visits the tips lowest first, among equals by seed
   id, and merges a tip standing within `sla::D_SP` in 3-D of a tip already
   kept into it, so no two tips it hands the builder are aliases; a kept tip is
   enforced when any tip merged into it was, holds an island when any did,
   and keeps its own axis. A merge
   logs
   `scaffold tip merged at (x, y, z)` at debug level and counts as neither
   placed nor dropped.
5. Grading. A tip's head fuses to the model with a disc of the width its
   grade names: the planner's for every tip it placed, four support lines on
   a birth whose island carries its part more than 2 mm on a section that
   fuses the wider disc, as step 1 states, and two on any other, the size's
   for a baked point, and two lines for an enforced tip, which asks for none.
   The head's pin radius is half that width.
6. The build. The draw hands the tips and the object mesh, in the frame the
   object's slices use, the one `place_tips` built or on a baked list one of
   its own, to `sla::SupportTreeBuildsteps::execute` with the
   configuration `tree_config` writes: head front, penetration and fallback
   radius at one toolpath width, pillar radius at half the pillar diameter,
   no model anchors, bridge and pillar-link lengths at the
   scaffold bridge length, a bridge slope of `max_head_tilt_rad`, 45 degrees,
   the object's xy distance as the safety distance, the scaffold brace
   slenderness, the brace radius at half the brace diameter, and the three
   routing retries of Builder changes on: heads facing the model route in the
   points' order, a head no route reaches retries thin, and one at a point
   starting an island retries along leaning axes. Each build
   hands the builder each point's tip axis in `SupportableMesh::head_axes`, in
   the order of the points the cuts left, and marks each point whose tip holds
   an island as starting one, `sla::SupportPoint::is_new_island`: on an auto
   slice `place_tips` marks every holder of an island the plan leaves
   unrooted, whatever need placed it, and on a baked list the island count
   marks every point under a mid-air island. The draw sets
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
   outside the clip's band with the exempt heads outside the model, through
   the `clip_base` the seam calls, and the rings outside the model, with the
   small holes `TreeSupport::fill_small_holes` fills filled, as
   `finish_layer_areas` fills them on every support layer, so an exempt head's
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
   section below. An exempt head whose own neck under its rings lies in a
   floating piece that holds no ring and walls no hole is cut as well: the
   head runs through the model or narrows under a line there, so the neck
   exempt from the band would print in mid-air. On a layer with an
   exempt head a piece counts as a hole's wall only when the base the seam
   prints after the fill borders no hole at it, since the band or the model
   can cut the filled hole out of that base again and leave the wall in
   mid-air. An enforced tip's head is exempt from the start. A head the check
   cuts for its neck, not for want of a pillar and a bridge, becomes exempt
   when its lowest neck slice clears the band, and the check reads the same
   build once more with that head's neck outside the model alone, which cuts
   it only where its rings or its own neck still float. A head whose lowest
   neck slice meets the band keeps no neck and stays cut, and a head the check
   never cuts keeps its neck clipped by the band.
   With no interface layers there is no ring to check, but the check
   still runs while any tip is enforced, since an enforced head prints its
   neck up to its tip. The cut heads' points
   leave the point set, in order, and the builder runs again, since a run
   re-routes the neighbours of what it lost and can strand another ring. It
   repeats until no head is cut, at most six runs (plate 3 of the corpus needs
   two, the first on 299 points cutting 12 heads and the second on 287 points
   cutting none), and the heads the sixth run still cuts are dropped without
   another run, their rings left out of the output. Each run logs
   `scaffold build <n>: <points> points, <cut> heads cut` and
   each drop `scaffold tip dropped at (x, y, z): <reason>` at debug level,
   the reason `filtered`, `unrouted` (also for a cut head with no pillar and
   no bridge) or `neck`. When anything routed, `SupportTreeBuilder::add_pad`
   adds a pad of the pad thickness with a 1.6 mm brim and no wall.
7. Slicing. Each run slices its cage at the middle of every planned layer,
   each routed head's own mesh at the tops of its top interface layer,
   the planned layer whose top is the tip's z, and of the layers under it up
   to the interface count, and every ringed or enforced head's own head, its
   neck, at the middles of the layers it reaches under those. A head that is
   not enforced keeps no neck when the lowest of those slices meets the band,
   so the neck check never exempts it. The cage leaves out each ringed or
   enforced head's own head over its rings, its pin, which the builder sinks
   into the model over the tip by the penetration: where the model is thinner
   than that reach, the pin would come out on the model's top face with
   nothing printed under it, a floating piece that holds no ring, walls no
   hole and is no neck. Inside the model or the band the clip removes that
   part anyway. The last run's slices are the output: the
   pad, sliced on the layers it spans, joins the cage's sections as base
   areas, the rings and each post's disc on the layers a head's rings would
   take become the layer's interface areas and leave its base, and
   the exempt heads' slices go out apart for `clip_base`.

Once the last run has settled each tip's outcome, `unheld_after_routing` reads
what routing left of the plan. It gives each planned tip the result of the
drawn site within `sla::D_SP` of it in 3-D, the alias merge's metric, so a
merged tip reads its keeper's result and a tip the wall skip took out, with no
drawn site, reads `Wall`. An island with holders that is not `rooted` and
whose holders all failed to route prints with no tip holding it, and a failed
Underside head's `answered_mm2` goes back into `underside_unmet_mm2`.
`islands_under_held` is the placement's count, the plan's `NoNeck` islands or
the island count's on a baked list, plus those islands, and `Output::bare_islands`
adds each one's birth point to the placement's list. A baked list carries no
plan, so an island whose points all fail to route adds nothing there: its
points count only in `tips_dropped` and in the baked warning's dropped count.
Each unheld island logs
`scaffold island at (x, y, z) unheld: <cause>` at debug level, with z its print
z and the cause `no neck`, a tipped island's own tip result (`unrouted`,
`filtered`, `neck` or `wall`) or, for a hung island, `holders unrouted`, so an
island whose own tip failed reads apart from one that hung from a failed
part. A planned slice with islands logs
`scaffold islands: <t> tipped, <h> hung, <u> unheld (<n> no neck, <m> not
routed)` at info level, debris left out. The planner does not re-plan:
corpus plate 1 reads no island unheld at plan time and four whose birth tips
the builder cannot route or cuts.

The returned `Output` carries one `LayerAreas` per planned layer (base,
interface and exempt heads), the number of
leading layers that are pad, the seven counts `tips_placed`, `tips_routed`,
`tips_dropped`, `islands_under_held`, `pillars_unbraced`, `islands_slender`
and `underside_unmet_mm2`, the bare islands, and the
milliseconds spent in the island map, in every build with the pad, and in the
slicing with the neck checks, which
`TreeSupport` writes into its profiler as `STAGE_ISLAND_JOINS`,
`STAGE_SCAFFOLD_BUILD` and `STAGE_SCAFFOLD_SLICE`.

## Builder changes

The SLA builder takes twelve additions. SLA printing runs each at its
default, where it leaves the output of the SLA path as it is, except the
corrector cap, which the SLA suite tolerates, and the isolated searches and the
face lookup, which run on every path and build what a search running alone and
a scan of every face would build.

- Face lookup. `sla::normals` averages, for a point within its `eps` of a
  mesh vertex or edge, the normals of the faces using that vertex or edge.
  `IndexedMesh::faces_of_vertex` finds them through the mesh's AABB tree, since
  every such face holds the vertex on its bounding box, and returns them in
  ascending order, the order the normals' unstable sort and sum take them in, so
  the normal reads as a scan of every face would read it. The planner reads that
  normal at each stability candidate, hundreds of times a slice, and a scan of
  a 1.67M-facet miniature takes about 4 ms a read.
- Head axes. `SupportableMesh::head_axes`, parallel to the points and empty
  by default, gives a point the direction its head is aimed along in place of
  the mesh normal, and a zero entry keeps the normal. The filter then
  saturates the axis at the bridge slope, checks the pinhead's clearance and
  searches a clear pose from it, as it does from a normal. The axes sit on the
  mesh, not on `sla::SupportPoint`, which SLA projects serialize and compare.

- Ordered routing. With `SupportTreeConfig::route_in_order`, default false,
  `routing_to_model` routes the heads facing the model one at a time in the
  order `classify` listed them, the points' order, followed by the cluster
  centroids whose ground pillar failed; the ray casts inside each route stay
  parallel. Routed at once, each head searches the pillars the others have
  stood by then and claims one of a pillar's three bridge slots first come,
  first served, so which heads route, and how, follows thread timing: on a
  comb of 30 heads under a ledge the same points built 10, 14 and 15 pillars
  in three runs. In order, one set of points builds one tree.
- Isolated searches. Each point's head search in `filter` and each route in
  `routing_to_model` runs under `tbb::this_task_arena::isolate`. The genetic
  searches reseed NLopt's thread-local generator, and a thread waiting on a
  search's parallel ray casts could otherwise take another point's search,
  which reseeds the generator mid-search: on one figure, three slices with
  identical routing found three different sets of head poses, and its
  support volume ranged from 1099.17 to 1115.22 mm3 over three more.
  Isolated, a search finds the pose it finds running alone, one an SLA slice
  could already produce, so the isolation runs on every path, and the three
  slices build one tree.
- Thin retry. With `retry_thin_head`, default false, a head that reached
  neither a pillar nor the ground retries at `head_fallback_radius_mm` along
  its direction, width zero, and drops a pillar from that thinner head's
  junction when the scan straight down from it clears the model, widening
  toward the pillar radius when it stands more than 20 radii tall, as any
  thin pillar does. The filter tries the thin
  head only when the full pinhead collides, never when a fitting head later
  fails to route, and a narrower scan ring passes where the full one meets a
  ledge. A retry that fails leaves the head as it was.
- Island axis retry. With `island_axis_retry`, default false, a head at a
  point marked `is_new_island` that every route so far failed tries fixed
  axes 15, 30 and 45 degrees from down, none past the bridge slope, at 24
  azimuths each, from the head's own azimuth outwards: for each axis whose
  pinhead clears the model by the filter's ring test at the full safety
  distance, the head is aimed along
  it and walks straight along it until the scan down clears, as
  `connect_to_ground`'s first attempt does, and the first axis that reaches
  the ground wins. The retry exists because the head's own route runs from a
  junction the mesh normal placed: under a lip over a shelf every walk from
  there meets the shelf, while a head leaning out starts its walk higher and
  further out. Each axis takes the full head first, as the filter does, and
  then the head's own size where the filter fell back to the thin one. A
  retry that fails leaves the head as it was and logs `Island support point
  <id> clears the model on <n> of <m> retry axes and routes along none` at
  debug level, an axis counted once whichever head size clears it. SLA points
  carry the flag from the support point generator, so the field, not the
  flag, gates the retry, and SLA printing, which leaves the field off, runs
  none.
  The retries' own tests keep the full safety distance from the model
  whatever the head's radius: the thin retry's scan down, and the axis
  retry's pinhead test, walk and scans down. The builder otherwise scales a
  thin element's clearance down by its radius, 0.18 mm for a 0.22 mm head at
  a 0.5 mm distance, and so does `create_ground_pillar`, which stands the
  pillar either retry routes to: a thin pillar up to 20 radii tall, 4.4 mm at
  0.22 mm, keeps no more than that scaled clearance on the corrector bridge it
  walks off the pad's gap, which the neck check reads afterwards like any
  other route. The seam clips base within
  the xy distance of the model on every layer, so a thin route passing nearer
  prints broken and the neck check cuts its head. On corpus plate 1's figure
  in its reference pose, an island born at z 35.2 clears retry axes with its
  head, none of them routes it at the full distance, and the slice names the
  island as bare.
- Model anchors. `SupportTreeConfig::allow_model_anchors`, default true, gates
  the last-resort route of a head to the model body. The scaffold sets it
  false, so a head that reaches neither a pillar nor the ground is
  invalidated and counted as unrouted instead of standing on the figure.
- Runtime safety distance. `SupportTreeConfig::safety_distance_mm`, default
  0.5, is the clearance the head and pillar collision checks keep from the
  model. The scaffold passes the object's support xy distance.
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
- Slim braces. `interconnect` builds a brace at `pillar_link_radius_mm`,
  capped at the pillar's radius, and at the pillar's radius when the field is
  zero, as SLA leaves it. A brace takes a pull or a push along its axis, so it
  needs far less section than the pillar it steadies, and every brace is a
  joint the user snaps to free the print: at the pillar's 1.2 mm a brace
  prints a 0.22 mm wall around a 1.2 by 1.7 mm slice, about 0.61 mm2 across
  its axis, and at 60 % about 0.31 mm2. `TreeSupport` floors the diameter at
  two support lines, since `tree_supports_generate_paths` closes a section by
  half a line before laying its wall and a narrower one prints nothing. A
  pillar-thick brace fuses the pillars it joins into one outline and a thin
  one leaves each pillar its own wall, so a densely braced plate prints more
  wall as its braces thin: plate 1 of the corpus, with about 1200
  braces, prints about 10 % more support path at 0.5 mm than at 1.2 mm.

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

Base stays out of the model grown by the xy distance, except at an exempt
head: an enforced tip's, one that painted enforcer facets or an enforcer
modifier volume asked for, and a head the neck check exempted because the
band stranded its rings while its lowest neck slice clears the band. The
head's own neck under its rings stays out of the model alone. Without that
exception the band would cut the neck of a tip the user enforced beside a
wall, or trim the neck of a head tilted under a sloped underside, and leave
its rings over nothing. The exemption covers that head only; pillars, bridges
and braces keep the band, and the neck check cuts an exempt head whose own
neck would print in mid-air.

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

`SupportAnalysis::Report` carries eight counters: `tips_placed`,
`tips_routed`, `tips_dropped`, `islands_under_held`, `pillars_unbraced`,
`islands_slender` and `underside_unmet_mm2`, which the scaffold fills from
`ScaffoldSupport::Counts`, and `floating_pieces_removed`, which the floating
pass fills under every style that runs it. They read zero under every other
style except the last. `islands_under_held` counts the islands that print
with no tip holding them once the build has routed, and `underside_unmet_mm2`
the underside left hanging past one and a half reaches, with what dropped
Underside heads answered; both read the plan and the build together. A baked
list has no plan, so its `islands_under_held` is the island count's alone.
`islands_slender` counts the branches the stability rule left past their
window with no corner a head can hold, once per branch, which reads the
placement alone: a Stability head the build drops counts only in
`tips_dropped`. It is routine on a figure standing on the bed, whose legs
stand taller than three thicknesses, and no warning reads it.

`PrintObject::_generate_support_material` logs one info line per object when
miniature contacts are on or the style is Tree Scaffold: `Support contact
layout for <object>`, followed by the critical regions without material, the
stability counts, the support volume, the seed counts and the eight counters.
The profiler line `tree support time` names the three scaffold stages. Both
lines, and the `scaffold islands` line, reach only the file the CLI's
`--logfile` names; the per-tip drop and per-island lines need `--debug 4`.

The test harness carries the counters into its rows:
`SupportValidation::Metrics` holds all eight, `read_print_analyses` sums them
over a print's objects and `write_result` writes them under `metrics`. The
scaffold's corpus rows carry `harness = "scaffold_support"` and go to the file
`ORCA_SCAFFOLD_RESULTS` names, or to standard output when it is unset, since
`scripts/validate_miniature_supports.py` accepts only its own harness names.

## Implementation and verification

`tests/fff_print/test_scaffold_support.cpp` holds the `[ScaffoldSupport]`
cases. They cover the style and its keys in the config, a project carrying
`scaffold_density` loading without it, the island map, the
result row, the pad's densities and the wall-only base on a shelf fixture with
nothing floating and no tip beside the column's wall, cancellation during the
build, rings printing as base without interface layers, a tip with no route
being dropped and counted, `unheld_after_routing` on hand-built plans, where
an island counts once its holders all fail to route, wall-skipped or merged
into a site that routed or did not, a site above the tip by more than
`sla::D_SP` matching nothing and a routed site among two in reach holding it,
and a dropped underside head hands its
answered underside back, a rod hanging into an open box whose birth tip has no
route listed as a bare island with the auto record's warning, a ledge in that
box whose dropped heads raise the underside unmet, a plank off a column's
face whose underside slopes
down away from it, where the automatic tips on a plank 2 mm long at 0.2 mm
layers and on one 4 mm long at 0.06 mm layers all keep their heads, and a
baked list on the 4 mm plank drops one or both of its points 0.6 and 1.3 mm
off the column face, whose necks bottom in the band, with no ring left
floating, no bare planned layer left and base in the band only under a ring,
a head under a sheet thinner than its pin leaving nothing floating over the
sheet, a rod hanging beside a column whose birth tip leans its neck away from
the column, stays through the wall skip and routes, in the auto slice, as a
baked point and as that baked point under a support blocker volume, which the
list ignores, `draw` called on a lifted slab's one tip with an axis leaning
45 degrees, whose head's slice 1.2 mm under the tip stands along the axis and
not straight under the tip, `draw` called on one tip under an island lip over
a shelf, dropped as a tip holding no island and routed along a leaning axis
as one holding the lip, a baked point there marked as holding it and
routed, two baked points across the lip's edge merged into the one outside
it, which holds the island for the other, and the need planner's own holder
of the lip marked by `place_tips` and routed, `baked_tips` on a block and a
rod hanging from a bar, where one point under the block holds it however far
it stands free, the rod, which no neck clears, reads under-held and is named
as the plan names it while a copied point under it is wall-skipped, and a point
placed there with the tool holds it, an empty list names the block at
the tip the rule would place and the rod, and a list copied from the auto
slice counts and names what the auto slice does, a tip placed under an unseeded
feature start with its ring on the layer its z tops and none under debris,
slivers beside a wall leaning their tips away from it and routing, braces on
slender pillars, pillars widening toward
the pad by the branch diameter angle with the same tips routed, the last two
over a baked 6 by 6 grid under the tall shelf's slab so that they read the
builder whatever the placement does, and a painted bar
underside and a painted column face beside the column's wall printing their
tips there at the corpus's widths and layer height, with two interface layers
and with none, where the unpainted bar prints none and no base but those
enforced heads enters the band. The SLA builder changes are covered in
`tests/sla_print/sla_print_tests.cpp`, the head axes by a head aimed along a
30 degree axis, one saturated at 45 degrees from a 60 degree axis, the normal
kept with no axis or a zero one, and a head whose axis runs into a wall
searching a clear pose instead, and the routing by a fin over a slot whose
full head is dropped while the thin retry drops a 0.22 mm pillar down the
slot, a comb of 30 points under a ledge that builds one tree in 20 runs
routed in order, and a point under a lip over a shelf whose head is dropped
unless the point starts an island, when it leans out along a retry axis and
stands a pillar past the shelf's edge.

`tests/fff_print/test_scaffold_plan.cpp` holds the `[ScaffoldPlan]` cases,
which call `prepare_plan` and `plan_tips` on fixtures sliced without support,
with the object mesh and the config's 10 mm `max_bridge_length`: the lattice's
area, a rod hanging under a slab taking one birth tip at its lowest point, a 1
mm ledge taking no tip and a 1.5 mm one taking heads with no unmet area, a fin
whose hanging area is under a head's disc taking none while a wider ledge
takes one, a cantilever taking its heads within two lines and a cell of its
far or side edges, a ramp between two walls bridging its middle 2 mm and
taking heads there with `max_bridge_length` 0, a lifted 9 mm disc born whole
taking a ring of heads within two lines and a cell of its rim and heads deeper
with no bridge hold, a lifted disc whose rim no head's neck clears taking
heads inside rather than bridging to that rim, a narrow ledge whose end leans
back taking its heads only where the builder's normal stands within 60 degrees
of down, a U-shaped ledge with no bridge hold whose near arm takes one head
while the cells a far arm head covers across the gap hang rather than call
their own, births carrying their parts taking the heavy disc only on sections
that fuse it to twice the small disc's area, a floating plate covered within
one and a half reaches with no bridge hold, a 10 degree flare taking more tips
than a 15 degree one and a 30 degree one none, a flare's runs restarting at a
solid column however the column is held, the stability rule: a blade whose
lower edge rises 40 degrees, steeper than the threshold, taking tips at three
heights or more; stability and underside tips taking the small disc on the
cantilever, a leaning rod and a hanging panel; a squat floating block taking
no stability tip beside a post a third as thick as it is tall; a blade
widening from its point taking tips up its spine within its 4.5 mm window,
three blade thicknesses, of each other and of the spine's end; a 1 mm rod
leaning 45 degrees out of a block on the bed taking at least three tips up its
underside, each at least half a window from the block; a panel hanging from
its corner taking tips only on its keel, not on its drafted sides; every
stability tip on a blade whose edge faces 50 degrees from down, on the rod
and on the panel standing where the builder's normal reads within 60 degrees
of down; a figure
standing on a base taking none and counting its two legs slender; a floating
cone taking none at its apex and counting nothing; a level spear held by
underside heads no more than `max_bridge_length` apart, the last covering its
end; a blade whose upright front edge carries teeth, each over the tooth below
so that no neck from the face under a tooth clears straight down, taking
stability tips on those faces whose necks lean off the blade's sides no more
than 45 degrees, with the builder's normal within 60 degrees of down and the
axis `neck_axis` reads again at each tip, and with no lean allowed none there
and one more slender count; a rod hanging from its flat end with a ledge on
its back whose underside rises 40 degrees and a tooth on its front whose
underside rises 25 degrees over a post, at the same height, taking its
stability tip on the ledge with no lean, and with no ledge taking it on the
tooth, leaning; and a tattered band hanging from two hem points taking its
stability tips on at least two of the four edges rising from those points,
none above the notch where its teeth meet, as a resin reference holds a
cape's hem. Then the birth rule: at 0.05 mm layers a nub within the merge
slab's step plus the reach of a held column hangs rooted from it while one
farther off and a taller one take birth tips, the same nub at 0.1 mm layers
taking a tip, of two nubs meeting in mid-air one taking the birth tip and the
other listing it as its holder, a fin within the step of the column beside it
taking no tip and counting no island, a far nub wide enough for a head taking
an underside head on its own layer and holding by it with no second tip, and a
rod over a shelf taking a birth tip whose neck leans the least that clears,
which `neck_axis` reads again at its spot and which no lean a step smaller
clears, and with no lean allowed no tip and an unheld island, a rod over a
shelf 0.3 mm thin whose straight neck ends clear under the shelf but crosses
it leaning past the shelf's edge, and with no lean allowed no tip, a lattice
built by hand where a one-cell plate in a neck's shaft makes it lean and the
lean takes the azimuth whose end stands farthest from a wall, and a baked
list's tipless nubs read by `read_births`: the near nub hanging from the
rooted column, the far and the taller ones tipped, the far nub held once the
list holds its underside head, and of two nubs meeting in mid-air one tipped
and the other hanging from it.

Two `[ScaffoldSupport]` cases cover the baked list on the shelf fixture. The
first bakes an auto slice's routed tips and slices from them with no contact
seed and at least 98 % of the points routed. It adds points by hand that read
`Routed`, `Filtered` on the top face, `Merged` beside an alias and `Wall`
inside the column, where no neck leaning up to 45 degrees clears the band, and
checks the dropped-points warning. It also checks that
paint places no tip on a baked slice, that another style records nothing,
that an empty list builds no tip, that a revert places what the first auto
slice placed, that a copy sharing the source's meshes prints its own list, and
that on the seeded-islands fixture the auto slice names as many bare islands as
it counts under-held while an empty list names each island the planner's rule
would tip.
The second slices two instances of one object as two PrintObjects and checks
that a quarter turn about Z keeps the list while a 30 degree tilt leaves it
stale with the warning, and that an instance scale and a part moved inside the
object make it stale. "Scaffold points survive a 3MF round trip and
model edits clear or carry them" (`[3mf][ScaffoldPoints]`, in
`tests/libslic3r/test_3mf.cpp`) checks the exact round trip, the reader's
rejection of a whole file and of a single point, the translation that carries
the list and the edits that clear it.

The hidden case "Scaffold support over corpus plate 3 in two poses"
(`[ScaffoldSupport][.]`) needs `ORCA_MINIATURE_CORPUS` pointing at the
directory holding `elf_test.3mf`. It slices plate 3's object under tree slim
and tree scaffold, first in the stored pose and then upright, times each
`Print::process()` and one `island_joins` call, writes one row per slice, and
requires of each scaffold slice: dropped tips at most a fifth of those placed,
no floating piece removed, a process time at most 1.5 times the tree-slim
slice's, `island_joins` within 2 s and a support volume at most 2.5 times the
tree-slim slice's; the upright pose also requires at most three under-held
islands, all born at z 15.4 with birth tips the builder cannot route, one of
them leaning its neck 15.5 degrees. The stored pose places 76 tips and routes
all 76, eight of its births leaning their necks 5 to 21 degrees, and the
upright pose places 220 and routes 195 on every run, since the builder routes
in order and isolates its searches.

The hidden case "Need-driven tips hold corpus plate 1's hand and sword with
few contacts" slices plate 1 and requires of the tips its record holds at most
10 on the hand over the raised knee and at most 117 in all, the resin
reference's count for the figure; of the tips the builder routes, a heavy one
at the sword's point, two under z 7.5 on the blade and no stretch of blade
under z 15 longer than 7 mm without one, the blade's stability window of three
2.25 mm thicknesses rounded up; and of its islands at most four under-held,
each named among the record's bare islands. Plate 1's record holds 97 tips, 7
on the hand, and on the blade routed tips at z 1.04 (the heavy birth tip),
1.16, 3.56, 8.24, 12.32 and 12.44: the widening edge takes a stability tip at
3.56, and above it the blade rises nearly upright to the guard, where the tip
at 12.32 drops its neck straight and those at 8.24 and 12.44 lean theirs 31
and 36 degrees off the blade, a largest gap of 4.68 mm. Of the record's 24
stability tips 13 lean, and 8 of those route. It has four bare islands: two
born at z 35.2 whose birth tips the builder cannot route, and two born at z
40.4 and 42.7 whose necks lean off a wall, one unrouted and one cut by the
neck check. The record holds five unrouted heads: the three unrouted births
and the leaning stability tips at z 38.12 and 39.44. All five hold islands, as
every stability tip in the record does, so the builder ran its routing retries
on each before dropping it.
