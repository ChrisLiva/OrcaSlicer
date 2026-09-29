# CLAUDE.md

OrcaSlicer — open-source C++17 3D slicer. wxWidgets GUI, CMake build system.

## Repository

Work against the fork `ChrisLiva/OrcaSlicer` (remote `origin`); its PRs, issues and branches live there. The clone also
has an `upstream` remote (`OrcaSlicer/OrcaSlicer`), and with no default set `gh` resolves to `upstream` before `origin`,
so an unqualified `gh pr view 2` read upstream's 2022 PR #2 instead of the fork's (2026-09-29). Pass
`-R ChrisLiva/OrcaSlicer` to every `gh` command, or check `gh repo set-default --view` names the fork first.

## Build Commands

```bash
# macOS first-time setup with Command Line Tools only (no Xcode): automake, libtool and texinfo
# are needed by deps/MPFR/MPFR.cmake's autoreconf and makeinfo steps, then build deps with Ninja
brew install cmake ninja automake libtool texinfo
./build_release_macos.sh -d -a arm64 -x

# macOS
cmake --build build/arm64 --config Release --target fff_print_tests -- -j6
cmake --build build/arm64 --config Release --target OrcaSlicer -- -j6

# Linux
cmake --build build --config Release --target all --

# Windows (replace %build_type% with Debug/Release)
cmake --build . --config %build_type% --target ALL_BUILD -- -m
```

Build `Release` to run the slicer. The `# Disable optimization for RelWithDebInfo` block in
`CMakeLists.txt` strips the optimizer out of `RelWithDebInfo`, rewriting `-O2` to `-O0` under
Clang and `/O2` to `/Od`, `/Ob1` to `/Ob0` under MSVC. Eigen's accessors and Clipper's point math
then stop inlining and go through call stubs, so a `RelWithDebInfo` build crawls through model
loading, slicing and the GUI alike. Reach for it only when you need a debugger, and expect the
slowdown.

`CMakeCache.txt` still reports `CMAKE_CXX_FLAGS_RELWITHDEBINFO:STRING=-O2 -g -DNDEBUG`, because
that rewrite assigns normal variables rather than cache entries. To read the flags a target
actually gets, grep the generated `build/<dir>/CMakeFiles/impl-<Config>.ninja` for its `.o` rule.

Cap Ninja's job count on a 16 GB Mac. `cmake --build` without `-j` runs 12 compilers on 10 cores
and each clang here peaks at 1.4 to 2.1 GB, so the machine pages: rebuilding the 22 `fff_print`
test objects took 395 s at `-j12` and 53 s at `-j5` (measured 2026-09-08). Since the precompiled headers instantiate
their templates, a compile's peak memory is 16 to 28 % lower, and a `libslic3r.h` edit rebuilding `fff_print_tests` took 190 s at
`-j5` and 161 s at `-j6` with no swapouts in `vm_stat` (2026-09-28). Pass `-- -j6` or set
`CMAKE_BUILD_PARALLEL_LEVEL=6` in the shell.

Build the target you are iterating on, not `all`: `all` is 929 objects, `OrcaSlicer` 716,
`fff_print_tests` 385 (106 of them Catch2, compiled once; counted 2026-09-08). Preview what a change will rebuild with
`ninja -C build/arm64 -f build-Release.ninja -n -d explain <target> | head`. The headers in
`src/libslic3r/pchheader.hpp` (`libslic3r.h`, `Point.hpp`, `Config.hpp`) reach every object in libslic3r and the GUI.
`PrintConfig.hpp` sits only in `src/slic3r/pchheader.hpp` and reaches every GUI object plus 119 of 240 libslic3r
objects, `Print.hpp` reaches 287 Release objects (200 of them GUI), and `Support/*.hpp` reach 7 or fewer
(`ninja -t deps`, 2026-09-28). `tests/fff_print/fff_print_pch.hpp` holds libslic3r's `pchheader.hpp`, Catch2 and six
libslic3r headers including `Model.hpp` and `Print.hpp`, so an edit to one of them rebuilds that header before the test
objects. Count a header's reach with its directory in the pattern, since a bare `Print.hpp` also matches `SLAPrint.hpp`:
`ninja -C build/arm64 -f build-Release.ninja -t deps | awk -v h=/libslic3r/Print.hpp '/^[^ ].*: #deps/{t=$1} index($0,h){n[t]=1} END{print length(n)}'`.
A `CMakeLists.txt` edit reconfigures but recompiles only what its flags change:
the `git_commit_hash_header` target in `src/slic3r/CMakeLists.txt` regenerates `git_commit_hash.h` on every build,
rewriting it only when the hash changes, and only `GUI/BuildCommit.cpp` (plus `BaseException.cpp` on Windows) and
`tests/fff_print/support_validation.cpp` include it, where the former global `add_definitions()` re-stamped every object
after each new commit (1 h 14 min for `--target all`, 2026-09-08).

Build the `OrcaSlicer` target after changing a type the undo/redo archive serializes: only the GUI instantiates those cereal saves, so a cereal ambiguity passes `fff_print_tests` and `libslic3r_tests` and fails the app. A `save(Archive&, const Matrix3d&)` in `ScaffoldPoints.hpp` also bound `Vec3f` and `Vec3d` through Eigen's converting constructor, and `GLGizmoBrimEars.cpp` failed with "cereal found more than one compatible output serialization function" three commits after the test targets had built it (2026-09-27).

Never start a second `cmake --build` in a build directory that already has one running: two Ninja
instances compile the same objects and starved each other to 0.03 s of CPU per compiler over 49 min
on a 16 GB Mac (2026-09-08). Check `pgrep -x ninja` first: under an agent harness `pgrep -fl 'ninja -f
build-Release.ninja'` matches the invoking shell's own command line and reports a build that is not running,
so a guard of the form `pgrep -fl ... || cmake --build ...` never lets a build start (2026-09-10).

Configure a build dir with `-DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DCMAKE_C_COMPILER_LAUNCHER=ccache` (`brew install ccache`).
Ninja rebuilds by mtime, so a checkout, reset or branch switch back to content already built recompiles every dependent,
about 40000 s of compile in the month to 2026-09-28; ccache hashes the content and returns the stored object. The
precompiled headers need `sloppiness = pch_defines,time_macros,include_file_mtime,include_file_ctime` in
`~/Library/Preferences/ccache/ccache.conf`, next to `depend_mode = true`, `compiler_check = %compiler% -v` and
`max_size = 20G`; `add_precompiled_header` already passes `-Xclang -fno-pch-timestamp`. Adding the launcher changes
every command line, so the first build after it recompiles everything. Time a build with `CCACHE_DISABLE=1`: a
`touch`ed file whose content did not change is a cache hit and reads as a fast compile.

## Testing

Catch2 framework. Tests in `tests/`; see [tests/AGENTS.md](tests/AGENTS.md) for where a new test belongs and the conventions to follow.

```bash
cd build && ctest -C Release --output-on-failure    # all tests
ctest --test-dir ./tests/libslic3r -C Release       # individual suite
ctest --test-dir ./tests/fff_print -C Release
```

A direct `fff_print_tests "[ScaffoldSupport]~[.]"` runs its cases one after another; `ctest --test-dir
build/arm64/tests/fff_print -C Release -j5 -L '^ScaffoldSupport$'` runs five at a time, longest first (26.8 s against
14.3 s, 2026-09-28). `-L` takes a regex, so anchor it. While iterating on libslic3r code, build and test only the
targets that skip `libslic3r_gui`, which a `PrintConfig.hpp` edit spares 435 GUI objects, and leave out the two cases
that set the gate's wall clock:

```bash
cmake --build build/arm64 --config Release --target fff_print_tests libslic3r_tests sla_print_tests libnest2d_tests filament_group_tests -- -j6
for d in fff_print libslic3r sla_print libnest2d filament_group; do
    ctest --test-dir build/arm64/tests/$d -C Release -j5 -E 'Surface centering survives|FilamentGroup property checks' || break
done
```

`slic3rutils_tests` is the only test target that links `libslic3r_gui`, so the loop skips its directory rather than
running its stale binary; an exclude list by test name would also drop fff_print's copy of `init_print functionality`,
which both suites compile from `test_helpers.cpp`. Run the full gate and build `OrcaSlicer` before a merge.

The legacy tree support's output is not run-to-run reproducible. Seven `--slice` runs of one Release
binary on one project (plate 2 of a 32 mm miniature, Tree Slim) spread the `Support` extrusion-move
count over 139166..141475 and the `Skirt` count over 88..102 while every other feature's count stayed
byte-identical (measured 2026-09-04). An oracle that expects byte-identical G-code or an exact
support-move count from `TreeSupport` therefore fails on an unchanged engine; compare the other features
exactly and the support count within a band (2 % held here).
The same generator lays branches that end in mid-air: 110 floating components of printed support on plate 3
of `elf_test.3mf` (Tree Slim, plate only, 0.5 mm xy distance), most of them one-layer slivers where a tip
circle was clipped against the model, and one branch of the closed-box fixture in roughly 1 run in 8.
`TreeSupport::remove_floating_toolpaths` takes those extrusions out after `generate_toolpaths` on a pass that
measures itself (`m_analyze`: analysis requested or miniature contacts on), by the connectivity rule
`SupportAnalysis::floating_pieces` shares with the stability measurement, so a report measured off generated
output has `unsupported_paths == 0` (plate 3: 110 -> 0, 2026-09-09); a report whose `EmittedSupport` a test
edited by hand still counts what the edit left floating. A stock slice runs no floating pass and keeps the
generator's output. The pass costs the union of every layer's footprints: `STAGE_GENERATE_TOOLPATHS` 0.7 s
-> 6.4 s per attempt on that plate.
Gate on Catch2 case counts, not assertion counts: `fff_print_tests "[MiniatureContacts]~[.]"` reported
102184, 103169, 104814, 106441 and 106975 assertions across five runs of one binary while its case count held
(2026-09-08/09).
A TBB task must never wait for another task of its own loop to start. When a task is spawned, oneTBB 2021.5 may miss
the wakeup of a sleeping worker, as the comment in `advertise_new_work` in its `src/tbb/arena.h` states, and then
relies on the spawning thread to run the task itself, which a thread blocked in such a wait never does.
`name_tbb_thread_pool_threads_set_locale` used a barrier of that kind over all `max_concurrency()` threads. On
2026-09-25 it froze the GUI on the first slice of a session, with 9 of 10 threads in the barrier and one worker
asleep, and held a lone `[ScaffoldSupport]` run for 5 min. A `tbb::task_scheduler_observer` has set up the workers
instead since 2026-09-25. The `ctest -j5` stalls of 2026-09-09 match that barrier's signature, with every thread in
`condition_variable::wait` in 2 of about 25 suite runs and five cases from four unrelated suites stalled in one run.
`orcaslicer_discover_tests` in `tests/CMakeLists.txt` sets `TIMEOUT 300` on every registered case so a stall fails
at 300 s instead of holding the run for ten minutes; a direct run of a binary has no timeout.
ClipperLib's output is not invariant under removing clip polygons that are provably disjoint from the subject: on
plate 3 of a 36 MB miniature project, clipping 61082 attributed support areas against a bbox-prefiltered clip
changed the result on 32894 of them by up to 7.4e-7 mm² and joined or split two pieces meeting at a one-unit
neck on 17, against the whole-layer clip (`intersection_ex`, measured 2026-09-09). An oracle that expects
byte-identical polygons from a Clipper call whose clip set changed fails on correct code; compare counts and
areas within an envelope instead.
The non-support features are not byte-identical either: three `--slice 3` runs of one Release binary on plate 3 of
`elf_test.3mf` read `Outer wall` extrusion moves 399740, 399740 and 399721, the 19 moves being the retraction wipes
and travel of one layer (z 1.82) while every wall extrusion vertex matched; an oracle for "support code left the
walls alone" compares wall vertices or excludes `WIPE_START..WIPE_END` and travel, never the move count (2026-09-10).
The CLI's `--debug 3` prints about five lines to stderr; the `tree support time` and `Support contact layout for` lines
land only in the file `--logfile <path>` names (2026-09-10). `--debug 3` is `info`, so it also filters out every
`BOOST_LOG_TRIVIAL(debug)` line: a throwaway probe logged at debug level and grepped out of a `--debug 3` run reads as
zero hits rather than as a probe that never fired. `--debug 4` is the first level that carries it (2026-09-18).
`init_print` in `tests/fff_print/test_helpers.cpp` arranges against `InfiniteBed{}` and leaves the instance at the
origin, so the stock 0..200 mm `m_machine_border` clips any support branch that walks across x 0 in
`TreeSupport::draw_circles` (`intersection_ex(base_areas, m_machine_border)`): a fixture centred on the origin whose
branches cross it loses them to `remove_floating_toolpaths` and reads `CoverageLost` for no router reason. Pass a
centred `printable_area` (`-100x-100,100x-100,100x100,-100x100`) in `fixture_config` for such a fixture (wedge
journey in `test_miniature_contacts.cpp`, 2026-09-09).
`Test::SupportValidation::contact_clusters` counts one cluster per roof-gap `ExPolygon` per support layer, so a thinned
attempt's fatter tips split into about three pieces each and the piece count barely moves (47 vs 49 on the wedge)
while the layers holding a contact and `total_mm2` halve (18 vs 50, 18.92 vs 40.88 mm2). An oracle for contact
thinning counts distinct contact `print_z` values or area, never pieces (2026-09-09).
`fff_print_tests "[AutoTilt]"` fails one case about 1 run in 12 with no stall (1 of 8 runs and 1 of 15 on 2026-09-09,
never captured, every re-run green); re-run once before reading a lone `[AutoTilt]` failure as a regression. One such
failure named its case: "A processed tree-support print
measures its emitted contact through the support analysis" failed once in the `ctest -j5` gate on 2026-09-10, passing
alone and on the re-run, assertion not captured. Capture the failing assertion before re-running.
`[MiniatureContacts]` "Support components come from printed slabs that touch and material with no root is counted"
failed its `split.stability.unsupported_paths > 0` leg once under `ctest -j5` (read `0 > 0`), then passed 12 of 12
runs alone, 30 of 30 runs five at a time and the full gate re-run (2026-09-10, no raft, analysis requested); re-run
once before reading a lone failure there as a regression.
`[ScaffoldSupport]` "A baked list goes stale under a tilt and stays valid under a Z rotation" fails its three
instance-0 checks (`baked` read false, `stale` read true and the stale warning fired) in about 1 run in 40, five
at a time: 2 of 80 runs with `branch_off_retry` off and 1 of 40 with it on, passing 5 of 5 lone re-runs
(2026-09-29). Its stale verdict reads no routing output; re-run once before reading a lone failure there as a
regression.
`sla_print_tests` reports a different assertion total on each run of one binary (13177, 13298 and 13300 over 35
passing cases, 2026-09-29), so gate that suite on its case count too.
The hidden `[ScaffoldSupport][.]` case "Scaffold support over corpus plate 3 in two poses" reads one result on every
run of one binary: `tree_config` sets `route_in_order`, so the SLA builder routes the heads in the points' order, and
`SupportTreeBuildsteps` runs each head search and each route under `tbb::this_task_arena::isolate`. Every run reads
stored `tips placed 76 / routed 76 / dropped 0`, 724.274 mm3, and upright `tips placed 220 / routed 200 / dropped 20`,
3 islands under-held, 1139.96 mm3, with `floating_pieces_removed` 0 in both poses (four runs over three Release builds
of the same placement code, 2026-09-28 and 2026-09-29; the stored volume, two runs since leaning necks must fit the
builder's full head, 2026-09-29; the upright counts and volume, two runs since a retry's branch bridge keeps the full
safety distance, 2026-09-29). On an unchanged corpus hash a non-zero
`floating_pieces_removed` or a changed placed, routed or dropped count is a regression; only the process times vary
between runs.
The corpus `elf_test.3mf` carries no painted enforcers since its intentional rewrite at 14:02 on 2026-09-25 (sha256
`201c5418…`). A second rewrite at 16:07 on 2026-09-29 (sha256 `84e5b41f…`) left every plate-3 number of the hidden case
unchanged. The three hidden `elf_test.3mf` cases pin that hash in their manifests but never check it, so a corpus
rewrite reads as a code regression. Hash the corpus with `shasum -a 256` before reading a hidden-case failure (2026-09-26).
The legacy tree's own floating pass is not idle on the corpus: a tree-slim slice of plate 3 strips 123 printed
pieces in the stored pose and 97 upright, every one a base shard of 0.002 to 0.59 mm2 (a temporary role log in
`remove_floating_toolpaths`, 2026-09-24), so an oracle expecting `floating_pieces_removed == 0` from a legacy style
fails on unchanged code; classify removed pieces by role before reading a count. The same case's tree-slim slices read
134 stored and 104 upright on each of three runs on 2026-09-28, unclassified.
`SupportAnalysis::Report::missing_anchor_ids` lists every seed whose region never reached printed material through
that seed, so the seeds `MiniatureSupport::select_contacts` decimates by design are in it: plate 3 of `elf_test.3mf`
reads 1638 `missing_critical_anchors` in the harness row while the slice log's `Support contact layout for` line
counts 2 critical printable regions without material. `validate_demonstrations`' `NO_WORSE_METRICS` still bounds
the metric; a gate that wants the region count reads `SupportAnalysis::support_unresolved` or the log line
(2026-09-10). `AutoTiltEvaluation` used to read a non-empty list as `required_region_unsupported`; since
`b0683c21f0` its `Status::Complete` arm calls `SupportAnalysis::support_unresolved`, so a pose whose only
complaint is a decimated seed reads `Complete` (2026-09-18).
Plate 3 baselines for a perf or density reading (`--debug 3 --slice 3`, Release, one slice at a time, 2026-09-10):
main `f3a07a0b37` 13.5 s wall, interface E 1.21 mm on 45 layers and 81 clusters, stable over three runs; the
miniature-contacts branch 43.1 s, 1.31 to 1.32 mm on 47 layers and 80 to 81 clusters, drifting
between runs of one binary. The branch's extra 29.6 s sits in `STAGE_RISK_FIELD` 9.3 s, `STAGE_MEASURE` 7.3 s,
`TreeSupport::build_contact_seeds` 4.7 s, `STAGE_SELECT_CONTACTS` 4.0 s and `remove_floating_toolpaths` 3.7 s.
That 4.7 s is the function's, not its region-merge loop's: a `steady_clock` probe around the merge on plate 3
measured dilate 4 ms plus union 11 ms over 2169 regions, 15 ms in total, inside an 8917 ms
`STAGE_BUILD_CONTACT_SEEDS`, so an optimization aimed at that loop has nothing to win. `build_contact_seeds` is
wrapped by its own profiler stage since `088312fe6f` and no longer has to be timed by hand;
`STAGE_BUILD_CONTACT_SEEDS` read 9.1 to 10.0 s across seven runs and `STAGE_SELECT_CONTACTS` fell from
5313/5356 ms to 4119/4053 ms at `b1492fe957` (2026-09-18).
Plate-3 contact counts are not run-to-run reproducible either: one Release binary reads either
`candidates 3737 / kept 2426 / retained 2429` with support 974.385 mm3, or
`candidates 3738 / kept 2427 / retained 2430` with 977.213 mm3, `restored 3` either way; four runs of one
unmodified binary read 3738, 3738, 3738, 3737. An oracle that pins a plate-3 contact count to one number fails
on unchanged code; compare against the two-value set (2026-09-18).
Auto-tilt runs only from the GUI (`Plater::auto_tilt()` is its one caller), so verifying it means relaunching the
app: a session already open holds the binary it started with, and a run driven in it exercises the pre-rebuild code
while `build/arm64/src/Release/OrcaSlicer.app` on disk is current. Which binary answered is readable off the log's
`auto-tilt root:` line, whose field order is `objectives_text`'s, and off `ps -eo lstart` against the executable's
mtime; one plate-4 run was read as a result before the start times were compared (2026-09-18).

## Documentation

- Docs live in `docs/`; the high-level design of a subsystem goes in `docs/HLSD/<subsystem>.md`.
- Describe the design as it stands — what the subsystem does, why it exists, and the constraints that shape it. Not the route that got there: no phases, task lists, status markers, or "before/after this PR" framing.
- Planning and investigation output (brainstorms, superpowers design and plan docs) stays in `docs/superpowers/`, which is gitignored. Never commit it.
- Write a doc only when the design is not evident from the code, and when a change invalidates an existing one, update it in the same PR.

## Code Style

- C++17, selective C++20. PascalCase classes, snake_case functions/variables
- `#pragma once` for headers. Smart pointers and RAII preferred
- Parallelization via TBB — be mindful of shared state
- Always use `SetSizerAndFit(sizer)` instead of `SetSizer(sizer)` on top level window. Unless `SetSizer` must be called before the full layout is built, call `sizer->SetSizeHints(window)` afterwards in this case.
- `ExPolygon::contains(point)` takes `border_result = true`, and `Slic3r::contains` returns that for Clipper's
  on-boundary `-1`, so a point exactly on a contour or a hole rim counts as inside. `closest_point`, inherited from
  `MultiPoint`, returns the nearest *vertex*, not the nearest point on the boundary: for an axis-aligned rectangle a
  query beside the middle of a long edge measures metres away. `ExPolygon::point_projection` is the nearest boundary
  point, holes included (2026-09-18).
- In code comments, cite another site by its symbol or by quoting the statement, never by `file:line`: every insertion above a cited line moves it, and a `TreeSupport.cpp:3530`-style citation pointed at unrelated code three commits after it was written.
- `.clang-format` sets `ColumnLimit: 140`, but no script under `scripts/` and no CMake target runs clang-format, and the tree ignores the limit: 568 lines under `tests/` and 695 under `src/libslic3r/Support/` already exceed 140 (`awk 'length>140'`, 2026-09-07). Match the width of the file you are editing rather than reformatting to the config's number.

## Key Entry Points

- App startup: `src/OrcaSlicer.cpp`
- Slicing pipeline: `src/libslic3r/Print.cpp`
- All print/printer/material settings: `src/libslic3r/PrintConfig.cpp`
- GUI: `src/slic3r/GUI/`
- Core algorithms: `src/libslic3r/` (GCode/, Fill/, Support/, Geometry/, Format/, Arachne/)
- Printer profiles: `resources/profiles/[manufacturer].json`

## Critical Constraints

- **Backward compatibility required** for .3mf project files and printer profiles
- **Cross-platform** — all changes must work on Windows, macOS, and Linux
- Profile/format changes need version migration handling
- Dependencies built separately in `deps/build/`, then linked to main app

## Code review focus areas

- Changes must not cause regressions in existing functionality, defaults, profiles, or project compatibility.
- Features gated by options must not affect existing behavior when those options are disabled.
- Changes should follow the existing code style and architecture. Architectural changes should be justified in code comments and the PR description.
- Add helper functions or utilities only when existing code cannot reasonably be reused. Avoid duplication.
- Keep code concise and clear. Manually simplify AI generated bloated codes before review.
- Include targeted tests or documented verification for behavior changes, especially in slicing logic, profiles, formats, and GUI defaults.
- For profile changes (`resources/profiles/<Vendor>/**`), check that `version` in the sibling `resources/profiles/<Vendor>.json` was bumped.
- For translation changes (`localization/i18n/**/*.po`), check that recurring terms match the [Localization glossary](https://github.com/OrcaSlicer/OrcaSlicer_WIKI/blob/main/developer_reference/localization_glossary.md) for that language.

## Localization & translations

Catalogs live in `localization/i18n/<lang>/OrcaSlicer_<lang>.po`; the template is `OrcaSlicer.pot`.
See the [Localization guide](https://github.com/OrcaSlicer/OrcaSlicer_WIKI/blob/main/developer_reference/localization_guide.md) for the human-facing version of these principles.

### Terminology

- Use the [Localization glossary](https://github.com/OrcaSlicer/OrcaSlicer_WIKI/blob/main/developer_reference/localization_glossary.md) as the source of truth for recurring terms, so the same English term is always rendered the same way within a language, and terms that must stay in English (brand/product names, acronyms, materials, file formats, G-code tokens, macros/variables/identifiers) are not translated.
- If a term's established translation changes, update both the affected `.po` files and the glossary (`localization_glossary.tsv`, then regenerate) so they stay in sync.
- Translate the *meaning*, not the words. Check what the string actually controls before translating it — English reuses one word for different things. `Flow ratio` (multiplier), `Flow Rate` (throughput) and `Flow Dynamics` (pressure compensation) are three different terms; `extruder` may mean the toolhead, the feeder motor, or the nozzle depending on the string.
- Reuse one template per recurring message shape (`Failed to connect to …`, `Are you sure you want to …?`), even where the English wording varies.

### Editing rules

- Only edit `msgstr` — **never** change `msgid`, and never "fix" wrong English in the translation alone. Report the source string instead.
- Preserve exactly: placeholders (`%s`, `%d`, `%1%`, `%zu`, `%%`), every `\n` (count *and* position, including leading/trailing), leading/trailing spaces, HTML tags, `℃`, and the file's encoding and line endings.
- **Never reorder positional arguments** in a `c-format` string. If the msgid is `%d` then `%s`, that order must hold — swapping them breaks at runtime.
- `msgctxt` separates homonyms — always read it. `Back`/`Camera View` is the rear view of the 3D navigator, while `Back`/`Navigation` is the go-back button; `Top` exists in the *Alignment*, *Layers* and *Camera View* senses.
- When a string needs disambiguating, add context in the source (`_L_CONTEXT`/`_u8L_CONTEXT`), don't work around it in the translation.
- A literal `%` inside a string xgettext flagged `possible-c-format` will fail `msgfmt`. Fix it with a `// xgettext:no-c-format, no-boost-format` comment above the string in the source — do not mangle the translation or use `%%` in text that is never passed through printf.
- Plural entries: read `nplurals` from the catalog's `Plural-Forms` header (it is **not** always 2 — ja/ko/zh/th/vi use 1, ru/cs/pl/lt use 3, uk uses 4). Each form must be genuinely inflected for its quantity; repeating one sentence across all forms is a bug in Slavic/Baltic languages, though it is correct for Turkish and Hungarian.
- An entry whose `msgstr` equals its `msgid` is untranslated even though it is not empty; a plural entry with any empty form is likewise incomplete.
- Mark machine-produced translations with an `# AI Translated` translator comment. Don't add it to a human translation you didn't actually rewrite.
- Don't reflow or re-wrap unrelated entries — keep the diff limited to the strings you changed.

### Verifying

- `scripts/run_gettext.bat --full` (Windows) regenerates the template, merges every catalog and compiles the `.mo` files. It must exit 0.
- Or check a single catalog with `msgfmt --check-format -o <out>.mo localization/i18n/<lang>/OrcaSlicer_<lang>.po`.
- Fuzzy entries are not shown to users. If you correct one, clear its `fuzzy` flag, otherwise the fix never ships.
