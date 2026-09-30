# Scaffold points store an approach axis, format version 1

A scaffold point gains an optional approach axis, a zero axis meaning the builder aims the head as before, and
`Metadata/scaffold_points.txt` moves to `scaffold_points_format_version=1` with the axis in each point group. Converted
artist tips need it: without it every enforced point is aimed along the mesh normal, which throws away the side-on
approach the artist chose (median 15 to 19 degrees off the reversed normal on the reference files). A version-1 reader
still reads version 0, but a build that knows only version 0 loads no list from a version-1 file, so a project saved
with converted points opens without them in an older build.
