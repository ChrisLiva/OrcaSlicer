# CLAUDE.md

OrcaSlicer — open-source C++17 3D slicer. wxWidgets GUI, CMake build system.

## Build Commands

```bash
# macOS first-time setup with Command Line Tools only (no Xcode): automake, libtool and texinfo
# are needed by deps/MPFR/MPFR.cmake's autoreconf and makeinfo steps, then build deps with Ninja
brew install cmake ninja automake libtool texinfo
./build_release_macos.sh -d -a arm64 -x

# macOS
cmake --build build/arm64 --config Release --target fff_print_tests -- -j5
cmake --build build/arm64 --config Release --target OrcaSlicer -- -j5

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
test objects took 395 s at `-j12` and 53 s at `-j5` (measured 2026-09-08). Pass `-- -j5` or set
`CMAKE_BUILD_PARALLEL_LEVEL=5` in the shell.

Build the target you are iterating on, not `all`: `all` is 929 objects, `OrcaSlicer` 716,
`fff_print_tests` 385 (106 of them Catch2, compiled once; counted 2026-09-08). Preview what a change will rebuild with
`ninja -C build/arm64 -f build-Release.ninja -n -d explain <target> | head`. Headers in the
precompiled header set (`libslic3r.h`, `Point.hpp`, `PrintConfig.hpp`, `Config.hpp`) reach every
object in libslic3r and the GUI, `Print.hpp` reaches about 120 per config, `Support/*.hpp` reach 7
or fewer. A `CMakeLists.txt` edit reconfigures but recompiles only what its flags change:
`GIT_COMMIT_HASH` reaches the five sources that read it (`set_source_files_properties` in
`src/slic3r/CMakeLists.txt` and `tests/fff_print/CMakeLists.txt`), where the former global `add_definitions()` re-stamped every object
after each new commit (1 h 14 min for `--target all`, 2026-09-08).

Never start a second `cmake --build` in a build directory that already has one running: two Ninja
instances compile the same objects and starved each other to 0.03 s of CPU per compiler over 49 min
on a 16 GB Mac (2026-09-08). Check `pgrep -x ninja` first: under an agent harness `pgrep -fl 'ninja -f
build-Release.ninja'` matches the invoking shell's own command line and reports a build that is not running,
so a guard of the form `pgrep -fl ... || cmake --build ...` never lets a build start (2026-09-10).

## Testing

Catch2 framework. Tests in `tests/`; see [tests/AGENTS.md](tests/AGENTS.md) for where a new test belongs and the conventions to follow.

```bash
cd build && ctest --output-on-failure           # all tests
ctest --test-dir ./tests/libslic3r              # individual suite
ctest --test-dir ./tests/fff_print
```

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
Under `ctest -j5` a test process occasionally stalls with every thread in `condition_variable::wait`: 2 of
about 25 suite runs stalled one `fff_print_tests` case, and one run stalled five cases at once
from four unrelated suites (`SLASupportGeneration`, `MultiFilament`, `AutoTilt`, `MiniatureContacts`), each of
which passes alone in under 22 s (2026-09-09). `orcaslicer_discover_tests` in `tests/CMakeLists.txt` sets
`TIMEOUT 300` on every registered case so a stall fails at 300 s instead of holding the run for ten minutes;
re-run once before reading a lone timeout as a regression.
ClipperLib's output is not invariant under removing clip polygons that are provably disjoint from the subject: on
plate 3 of a 36 MB miniature project, clipping 61082 attributed support areas against a bbox-prefiltered clip
changed the result on 32894 of them by up to 7.4e-7 mm² and joined or split two pieces meeting at a one-unit
neck on 17, against the whole-layer clip (`intersection_ex`, measured 2026-09-09). An oracle that expects
byte-identical polygons from a Clipper call whose clip set changed fails on correct code; compare counts and
areas within an envelope instead.
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
`[MiniatureContacts]` "Support components come from printed slabs that touch, and material with no root is counted"
failed its `split.stability.unsupported_paths > 0` leg once under `ctest -j5` (read `0 > 0`), then passed 12 of 12
runs alone, 30 of 30 runs five at a time and the full gate re-run (2026-09-10, no raft, analysis requested); re-run
once before reading a lone failure there as a regression.
`SupportAnalysis::Report::missing_anchor_ids` lists every seed whose region never reached printed material through
that seed, so the seeds `MiniatureSupport::select_contacts` decimates by design are in it: plate 3 of `elf_test.3mf`
reads 1638 `missing_critical_anchors` in the harness row while the slice log's `Support contact layout for` line
counts 2 critical printable regions without material. `AutoTiltEvaluation` reads a non-empty list as
`required_region_unsupported`, and `validate_demonstrations`' `NO_WORSE_METRICS` still bounds the metric; a gate
that wants the region count reads `SupportAnalysis::support_unresolved` or the log line (2026-09-10).
Plate 3 baselines for a perf or density reading (`--debug 3 --slice 3`, Release, one slice at a time, 2026-09-10):
main `f3a07a0b37` 13.5 s wall, interface E 1.21 mm on 45 layers and 81 clusters, stable over three runs; the
miniature-contacts branch 43.1 s, 1.31 to 1.32 mm on 47 layers and 80 to 81 clusters, drifting
between runs of one binary. The branch's extra 29.6 s sits in `STAGE_RISK_FIELD` 9.3 s, `STAGE_MEASURE` 7.3 s,
the serial region-merge double loop in `TreeSupport::build_contact_seeds` 4.7 s (wrapped by no stage),
`STAGE_SELECT_CONTACTS` 4.0 s and `remove_floating_toolpaths` 3.7 s.

## Code Style

- C++17, selective C++20. PascalCase classes, snake_case functions/variables
- `#pragma once` for headers. Smart pointers and RAII preferred
- Parallelization via TBB — be mindful of shared state
- Always use `SetSizerAndFit(sizer)` instead of `SetSizer(sizer)` on top level window. Unless `SetSizer` must be called before the full layout is built, call `sizer->SetSizeHints(window)` afterwards in this case.
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
- For translation changes (`localization/i18n/**/*.po`), check that recurring terms match the [Localization glossary](https://github.com/OrcaSlicer/OrcaSlicer_WIKI/blob/main/guides/localization_glossary.md) for that language.

## Localization & translations

Catalogs live in `localization/i18n/<lang>/OrcaSlicer_<lang>.po`; the template is `OrcaSlicer.pot`.
See the [Localization guide](https://github.com/OrcaSlicer/OrcaSlicer_WIKI/blob/main/guides/localization_guide.md) for the human-facing version of these principles.

### Terminology

- Use the [Localization glossary](https://github.com/OrcaSlicer/OrcaSlicer_WIKI/blob/main/guides/localization_glossary.md) as the source of truth for recurring terms, so the same English term is always rendered the same way within a language, and terms that must stay in English (brand/product names, acronyms, materials, file formats, G-code tokens, macros/variables/identifiers) are not translated.
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
