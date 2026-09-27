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

Four settings belong to the style and show only under it:

- `scaffold_bridge_length`, 3 to 30 mm with a 12 mm default, bounds every
  bridge from a tip to its pillar and every brace between pillars.
- `scaffold_brace_slenderness`, 5 to 40 with a 15 default, is the unbraced
  height of a pillar, in pillar diameters, above which the builder braces it
  to its neighbours.
- `scaffold_brace_diameter`, 10 % to 100 % with a 60 % default, is a brace's
  diameter as a share of the pillar diameter, never under two support lines.
- `scaffold_density`, Light, Medium or Heavy with Medium the default, sets how
  many tips an auto slice places. Medium keeps the tips the contact selection
  keeps. The selection thins contacts by `support_contact_min_distance` within
  one overhang component only, and a sculpted underside splits into many small
  components: plate 1 of the corpus reads 538, with 667 of its 819 kept
  contacts standing within 1 mm of a contact from another component. Heavy
  halves the selection's distance. Light keeps the distance between tips of
  different components too, in `choose_tips`. A baked list keeps its own points
  at any density. Plate 1 places 321, 401 and 575 tips at Light, Medium and
  Heavy.

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
`smsTreeScaffold`, and every scaffold branch in the generator reads it. Unless
the object holds a valid baked list, which Baked contact points describes, the
front half runs as it does for the slim style with miniature contacts on:
`detect_overhangs`, the contact seeds, `select_contacts`, the risk field. When
the contact selection erases the contacts it did not keep, the scaffold keeps
those pointers in `m_dropped_contacts`, because the island hold floor draws on
them. A slice from a valid baked list runs `detect_overhangs` and none of the
other three.

Before `plan_layer_heights`, under Scaffold, the generator fills a
`ScaffoldSupport::Params` from the object config (toolpath width, pillar
diameter, `m_ts_data->m_xy_distance`, the two scaffold keys,
`max_bridge_length`, the interface layer count and the object's print z
offset) and calls `ScaffoldSupport::choose_tips` on the kept and the dropped
contacts, or `ScaffoldSupport::baked_tips` on a valid baked list.
`plan_layer_heights` then takes every chosen tip's z as a layer top
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
`build_contact_seeds`, and `select_contacts` with its erase loop. Painted
enforcers and blockers therefore change nothing while a list is in use, and
the tool says so. `detect_overhangs`, the support layers, the preview cache
and the `Params` fill still run, and the generator calls
`ScaffoldSupport::baked_tips` in place of `choose_tips`.

`baked_tips` maps each point through `trafo_centered()` into the frame the
builder uses and adds the object's print z offset. It snaps the point's z to
the bottom of the object layer holding it, the first layer whose top lies
above the point or the top layer when none does, and files it on the layer
under that one, where a contact under an overhang is filed. It then runs
steps 2 to 4 of the auto path on those sites:

- The wall skip calls the same `wall_skip` function `choose_tips` calls. A
  point placed with the tool is enforced and never skipped; a point Generate
  copied carries the flag its tip had.
- The hold floor counts the islands the list leaves under-held but seeds and
  restores no tip: where it would have seeded one, it records a bare island.
- The alias merge runs unchanged, so a point within `sla::D_SP` of another
  merges into it.

Each site carries its point's index as `TipSite::source` and its size's disc
width as `TipSite::grade_mm`, which `tip_grades` keeps in place of grading the
tip against the risk field. From `plan_layer_heights` on, a baked slice runs as
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
`trafo_centered()`. The tips differ by pass:

- A baked record holds one tip per list point, in list order, with the point's
  size and flag. Its result is `draw`'s outcome (`Routed`, `Filtered`,
  `Unrouted` or `Neck`) for a point with a drawn site, `Wall` for a point the
  wall skip took out and `Merged` for any other point without a site.
- An auto record holds one tip per site handed to `draw`, with `draw`'s
  outcome and a size read back from the tip's grade: `Heavy` above three
  support lines, `Light` otherwise.

`scaffold_points_from` turns a record into a list by keeping its `Routed`
tips. A baked record with any point not `Routed` or any bare island adds the
warning "Scaffold points for <object>: <d> dropped, <b> islands left without a
tip." to the support step.

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

`src/libslic3r/Support/ScaffoldSupport.{hpp,cpp}` holds two functions that
run seven steps between them. `ScaffoldSupport::choose_tips`, the selection,
runs steps 1 to 4 on the contact nodes, reads no planned layer and returns the
tips with the hold floor's count, which is why it can run before the layers are
planned. `ScaffoldSupport::draw` runs steps 5 to 7 on those tips and the
planned layers, turns them into per-layer areas and counts what it did with
them. `ScaffoldSupport::baked_tips` stands in for `choose_tips` on a baked
list and runs steps 2 to 4 on its points, as Baked contact points describes.

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
   a layer past it. The skip reads the tip's centre at the depth of the neck's
   bottom, not the built neck: under a slope the head tilts along the
   underside's normal, and the neck check in step 6 exempts a cut head whose
   tilted neck bottoms outside the band. A skipped tip logs
   `scaffold tip skipped at (x, y, z): wall` at debug level and counts as
   neither placed nor dropped. The same test applies to the dropped contacts
   the hold floor may restore and to the tips it seeds. An enforced tip is
   exempt: a contact a support enforcer asked for, `SupportNode::is_pinned`,
   which under Scaffold marks every contact on an overhang an enforcer
   covers, whether painted facets or an enforcer modifier volume, and every
   vertical enforcer point painted facets place, becomes a `TipSite` with
   `enforced` set, and the skip keeps it however close the wall stands, so
   the tip fuses where the user asked for support and leaves its scar there. Under
   Light density the selection then visits the tips lowest first, among equals
   by seed id, and moves a tip standing within `support_contact_min_distance`
   in 3-D of a tip already kept, of any component, to the dropped contacts,
   where the hold floor can restore it; an enforced tip is kept and crowds no
   other.
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
   hanging off a neck at least eight lines wide; a seeded tip keeps two, and a
   baked point keeps the width its size asks for. The head's pin radius is half
   that width.
6. The build. The draw hands the tips and the object mesh, in the frame the
   object's slices use, to `sla::SupportTreeBuildsteps::execute` with the
   configuration `tree_config` writes: head front, penetration and fallback
   radius at one toolpath width, pillar radius at half the pillar diameter,
   no model anchors, bridge and pillar-link lengths at the
   scaffold bridge length, a 45 degree bridge slope, the object's xy distance
   as the safety distance, the scaffold brace slenderness, and the brace
   radius at half the brace diameter. The draw sets
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

The returned `Output` carries one `LayerAreas` per planned layer (base,
interface and exempt heads), the number of
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
being dropped and counted, a plank off a column's face whose underside slopes
down away from it, where a plank 2 mm long at 0.2 mm layers keeps every tip
and one 4 mm long at 0.06 mm layers drops the two tips 0.6 and 1.3 mm off the
column face, whose necks bottom in the band, with no ring left floating, no
bare planned layer left and base in the band only under a ring, a head under
a sheet thinner than its pin leaving nothing floating over the sheet,
interior tip thinning, the hold floor restoring contacts under tall islands
and capped by what a birth piece fits, a tip seeded under an unseeded
feature start with its ring on the layer its z tops and none under debris or
under a sliver the wall beside it holds, a sliver joining too high for its
wall counted under-held, braces on slender pillars, pillars widening toward
the pad by the branch diameter angle with the same tips routed, and a painted bar
underside and a painted column face beside the column's wall printing their
tips there at the corpus's widths and layer height, with two interface layers
and with none, where the unpainted bar prints none and no base but those
enforced heads enters the band. The SLA builder changes are covered in
`tests/sla_print/sla_print_tests.cpp`.

Two `[ScaffoldSupport]` cases cover the baked list on the shelf fixture. The
first bakes an auto slice's routed tips and slices from them with no contact
seed and at least 98 % of the points routed. It adds points by hand that read
`Routed`, `Filtered` on the top face, `Merged` beside an alias and `Wall` in
the column's band, and checks the dropped-points warning. It also checks that
paint places no tip on a baked slice, that another style records nothing,
that an empty list builds no tip, that a revert places what the first auto
slice placed and that a copy sharing the source's meshes prints its own list.
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
tree-slim slice's; the upright pose also requires no under-held island.
