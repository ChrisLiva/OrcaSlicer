# Spec: convert a pre-supported model into Tree Scaffold points

Labels: ready-for-agent

Glossary: `CONTEXT.md`. Decisions: `docs/adr/0001-convert-artist-tips-not-skeleton.md`,
`docs/adr/0002-scaffold-point-approach-axis.md`. Design of the style this builds on: `docs/HLSD/scaffold-support.md`.

## Problem Statement

Miniature artists ship pre-supported models: the figure and a full set of resin supports in one STL, tilted and lifted
off a raft, with every artist tip placed by hand for that pose. The artist's placement is the best contact layout
available for the figure, but it is built for resin. Printed on an FDM machine with a 0.2 mm nozzle, the resin trunks,
braces and micro struts are too thin or the wrong shape, and the artist raft is not the scaffold's pad. Today the only
way to use that placement with Tree Scaffold is to delete the supports in another tool and click every scaffold point
by hand at the spots the artist chose, 90 to 214 of them per figure, and even then each tool-placed point is aimed
along the mesh normal instead of along the artist's approach.

## Solution

The Scaffold Points tool gains a Convert button. On an object imported from a pre-supported STL, conversion removes
every artist support from the object's mesh, leaving the bare figure where the artist posed it, and writes one enforced
scaffold point per artist tip at the tip's contact site, with the tip's approach axis and a tip size read from the
artist's contact diameter. The figure keeps the artist's tilt and lift. The next slice builds the scaffold from that
list with the user's scaffold settings: pillar diameter, taper, bridge length, braces and pad. A notification says what
the conversion did: tips converted, duplicates removed, axes clamped, micro struts dropped and tips whose support
rooted on the figure.

## User Stories

1. As a miniature printer, I want to convert an artist's pre-supported STL into Tree Scaffold support, so that I print
   the artist's contact layout on my FDM machine.
2. As a miniature printer, I want the conversion to keep every artist tip's contact site exactly, so that the scaffold
   touches the figure where the artist decided it should.
3. As a miniature printer, I want each converted point to keep the artist tip's approach axis, so that a tip the artist
   brought in from the side still meets the figure from the side.
4. As a miniature printer, I want an artist axis leaning more than 45 degrees from vertical to be clamped to the nearest
   axis the builder can build, so that the point keeps as much of the artist's lean as the scaffold allows.
5. As a miniature printer, I want the artist's heaviest tips, 0.5 mm contact and wider, to become Heavy points and the
   rest Light, so that the artist's judgement of where the figure is loaded carries over.
6. As a miniature printer, I want the scaffold's pillars, braces, bridges and pad to follow my Tree Scaffold settings,
   so that the support prints and removes the way my other scaffold prints do.
7. As a miniature printer, I want the figure to keep the artist's tilt, so that the contact sites still sit on the
   overhangs the artist supported.
8. As a miniature printer, I want the figure to keep the artist's lift above the plate, so that the artist's lowest tips
   still have room for a full scaffold head.
9. As a miniature printer, I want the converted object to stay lifted when I move it or reopen the project, so that
   the lift is not lost to the bed drop.
10. As a miniature printer, I want the artist raft, trunks, branches and braces removed from the object, so that only
    the figure prints as model.
11. As a miniature printer, I want the artist's micro struts dropped, so that hair-thin struts that print nothing and
    would scar the figure do not become part of the model.
12. As a miniature printer, I want tips on supports that stood on the figure to be converted and routed from the pad,
    so that the artist's model-to-model contacts are not lost.
13. As a miniature printer, I want a tip the artist wrote twice to become one point, so that duplicated primitives do
    not stack two heads on one spot.
14. As a miniature printer, I want tips the scaffold cannot build, such as ones pointing down onto an up-facing face,
    to be converted anyway and reported by the slice as filtered, merged or unrouted, so that I see every artist
    contact and what the scaffold made of it.
15. As a miniature printer, I want the converted list to be the whole list, with no tips added by the need planner, so
    that the placement is the artist's and nothing else.
16. As a miniature printer, I want islands the artist left without a tip to show as bare islands in the tool, so that I
    can add a point by hand or run Generate if the FDM print needs one.
17. As a miniature printer, I want a notification that counts the tips converted, duplicates removed, axes clamped,
    micro struts dropped and tips rooted on the figure, so that I can judge the conversion at a glance.
18. As a miniature printer, I want converting an object that already holds a scaffold points list to ask before
    replacing it, so that I do not lose points I placed or edited.
19. As a miniature printer, I want one undo to restore the object with its artist supports and its earlier list, so that
    a conversion I did not want costs nothing.
20. As a miniature printer, I want a file whose supports share vertices with the figure to be refused with a message
    that the supports are welded to the figure and that Generate can place scaffold points instead, so that the
    conversion never guesses at a split it cannot see.
21. As a miniature printer, I want an object with no artist tips to be left unchanged with a message saying no artist
    tips were found, so that pressing Convert on an ordinary model is harmless.
22. As a miniature printer, I want every shell of the figure, such as a separate spear or shield, to stay part of the
    one object, so that one object comes in and one converted object comes out.
23. As a miniature printer, I want to split a file holding several figures into objects before converting each, so that
    each figure gets its own list.
24. As a miniature printer, I want converted points to appear in the tool like any other points, so that I can move,
    resize, erase or add points before slicing.
25. As a miniature printer, I want converted points to be exempt from the wall skip, so that the scaffold does not
    quietly drop an artist tip that sits beside a wall.
26. As a miniature printer, I want converted points coloured by their slice outcome after a slice, so that I can see
    which artist tips routed and which did not.
27. As a miniature printer, I want a project saved with converted points to reopen with the same points and axes, so
    that I can come back to a prepared print.
28. As a miniature printer, I want the converted list to go stale and fall back to automatic tips if I tilt the object
    afterwards, so that points planned for the artist's pose are never built in a pose they do not fit.
29. As a miniature printer, I want a Z rotation or a move of the converted object to keep the list, so that I can
    arrange the plate freely.
30. As a miniature printer, I want the conversion to work on pre-supported files from different support generators
    that build sphere-ended tips, so that the button does not stop working on the first pack from a new artist.
31. As a miniature printer, I want the conversion of an 80 MB, 1.7 million triangle STL to finish in seconds, so that
    it fits into preparing a plate.
32. As a user of an older build, I want a project that carries converted points to open without error, so that I can
    still print the figure, even though that build loads no points from it.
33. As a user of the Scaffold Points tool, I want points I click to behave exactly as before, aimed by the builder with
    no stored axis, so that the conversion changes nothing about hand placement.
34. As a user of Generate, I want points Generate copies to keep leaning as the automatic slice leaned them, so that
    the axis field changes nothing about Generate.
35. As a maintainer, I want the conversion to live in libslic3r behind one call, so that it is testable without the
    GUI and the button stays a thin wrapper.

## Implementation Decisions

- **Scope of what carries over.** Conversion keeps each artist tip's contact site, approach axis and a tip size, and
  nothing of the artist's skeleton. The builder routes pillars, bridges, braces and pad as it does for any baked list
  (ADR 0001).
- **One libslic3r conversion call.** A new module in the Support area takes a `ModelObject` and returns a summary. It
  replaces the object's model mesh with the figure, writes a `UserModified` scaffold points list stamped with the
  selected instance's pose and the new mesh box, and turns that instance's `auto_drop` off. The summary carries the
  counts the notification shows and a reason when it refused, either welded supports or no artist tips.
- **Separation.** The mesh splits into shells by connectivity, the same split `ModelObject::split` uses. The figure is
  the set of shells above the largest bounding-box volume gap, after setting aside flat shells on the plate and shells
  resting on them, as Resin2FDM's detection does. When the split yields a single shell, or the gap rule finds no
  support shells, the call refuses as welded.
- **Recognising an artist tip by shape, not by triangle count.** A tip is a shell with two rounded ends of different
  radii, each end fitted as a sphere. The narrow end's sphere centre must lie on the figure's surface within a
  tolerance. The contact site is that centre and the approach axis runs from the wide end's centre to the narrow end's.
  The contact diameter is the narrow sphere's diameter.
- **Micro struts** are shells whose ends both lie on the figure and whose diameter is under one support line. They are
  counted and dropped.
- **Tips rooted on the figure** are recognised by their support cluster never reaching the plate. Their tips convert
  like any other, and the summary counts them.
- **Duplicates.** Tips whose contact sites and axes match to a tolerance collapse to one point.
- **Axis clamp.** An axis leaning more than the builder's 45 degree head tilt cap from straight down is replaced by the
  axis on the cap's cone nearest to it, and counted.
- **Size cut.** A contact diameter of 0.45 mm or more becomes Heavy, anything narrower Light.
- **Frame and lift.** Points are written in the frame of the figure mesh after replacement. The figure keeps its world
  position. `auto_drop` is turned off before the mesh swap so the bed drop does not lower it. The builder already
  grounds a lifted object's pillars at the pad.
- **`ScaffoldPoint` gains an approach axis** (ADR 0002). A zero axis means the builder aims the head as today. A
  non-zero axis is handed to the builder through the per-point head axes the builder already accepts, and it takes
  precedence over the `neck_axis` reading the baked pass gives points that are not enforced. Enforced points keep their
  wall-skip exemption whatever their axis. The field joins the point's equality, the undo archive, copy and paste,
  merge and the translate that carries points with the mesh.
- **3MF format version 1.** `scaffold_points_format_version` becomes 1 and each point group gains three axis
  components after the existing five fields. The reader accepts version 0, reading a zero axis, and version 1. A build
  that knows only version 0 loads no list from a version-1 file, as it already does for any unknown version. A
  non-finite axis component drops that point alone with a warning, as a non-finite coordinate does.
- **Scaffold Points tool.** A Convert button runs the conversion on the tool's object under one undo snapshot, asks
  first when the object holds a non-empty `UserModified` list, as Generate does, then reloads the tool's cache from the
  new list and reslices. The tool shows the summary as a notification. Clicked points still get a zero axis.
- **Validity.** No change. The conversion stamps the pose, so the list is valid in the artist's pose. A later tilt makes
  it stale and a Z rotation keeps it, by the existing pose test.

## Testing Decisions

- **What a good test checks.** A good test states what a user would see: which points exist, where, aimed how and
  sized how; what the figure mesh is; what the slice built from them. It does not check how the separation walks the
  mesh or which intermediate structures it builds, so the tests survive a rewrite of the separation.
- **One new seam: the libslic3r conversion call.** Every conversion test goes through it.
  - A synthetic pre-supported fixture built in the test: a figure mesh; sphere-swept artist tips of two contact
    diameters, one leaning past 45 degrees; a tip written twice; a trunk and brace joining them to pad-and-bar raft
    pieces; one support rooted on the figure; one micro strut. Converting it yields the figure mesh alone, one point per
    distinct tip at its contact site, the clamped axis on the cap, Heavy and Light split at 0.45 mm, `auto_drop` off, the
    figure at its original world z, and a summary with the expected counts.
  - The same fixture, converted and then sliced with `init_print` under Tree Scaffold: the scaffold record reports a
    result per point, and a routed head is aimed along its point's axis. The existing "Draw aims each head along its
    tip's axis" case is the prior art for the axis assertion. "A baked scaffold list builds the tips it holds and reports
    each point's result" is the prior art for the record assertions.
  - A welded fixture, the figure and one support sharing vertices, and a plain figure with no supports each leave the
    object unchanged and return the matching refusal.
- **Existing seam: the 3MF round trip.** The scaffold points round-trip case gains a point with a non-zero axis, and a
  version-0 file that loads with zero axes.
- **Existing seam: the baked slice.** A stored axis overrides the planner's `neck_axis` for a point that is not
  enforced, checked the way the existing planner-axis case checks the wall skip's neck reading.
- **Hidden corpus case.** Tagged hidden, reading its files from `ORCA_MINIATURE_CORPUS`, it converts the three
  reference pairs and checks 117, 90 and 214 converted points, the distinct contact sites the mesh analysis measured,
  and checks the figure mesh against the unsupported file under a fitted rigid transform. It also checks each file's sha256 first,
  so a changed corpus file reads as a changed corpus rather than a regression.
- **Where they live.** Conversion and slice cases in the scaffold support test file in `fff_print_tests` under the
  `[ScaffoldSupport]` tag; the round trip in the 3MF test file in `libslic3r_tests`.
- **Not tested automatically.** The Convert button and the notification, which the in-app smoke check covers: import
  `STL_10_Dark Elves 1_Supported.stl`, convert, slice, and compare the points with the artist's tips.

## Out of Scope

- Rebuilding the artist's trunks, branches, braces or raft with scaffold diameters (ADR 0001).
- Editing a point's approach axis in the tool.
- Accepting the unsupported file as a second input to separate a welded file.
- Adding need-planner tips to the artist's list.
- Splitting a file of several figures into objects during conversion.
- An import-time prompt that detects a pre-supported file.
- Keeping the artist supports as a reference ghost after conversion.
- Converting the artist's tips for any support style other than Tree Scaffold.

## Further Notes

- Reference files are in `~/Downloads/resin_examples/`: three supported and unsupported pairs from one generator. The
  figure is one shell stored unchanged and rigidly re-posed; supports are closed 8-segment primitive shells that overlap
  it and never weld. Tips are sphere-swept cones whose contact sphere centre lies on the surface, median 0.001 mm off.
  Lift is 6.0, 6.0 and 5.0 mm. Ratmen repeats 661 of its 1,843 primitives.
- Contact diameters fall into three groups: 0.15 to 0.25 mm, 0.35 mm and 0.5 mm. Dark Elves 1 reads 16, 69 and 32 tips
  in those groups; Ratmen 133, 56 and 25.
- 19 of 117 Dark Elves 1 axes and 90 of 214 Ratmen axes lean past 40 degrees, so the clamp count will be large on some
  files.
- Resin2FDM, a GPL-3.0-or-later Blender add-on, informed the shell split and figure detection. No code is copied from
  it.
