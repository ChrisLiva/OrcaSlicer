# Miniature support validation

How the miniature-support work is held to a corpus: the manifest that declares the corpus, the two
hidden Catch2 harnesses that measure it, the rows they write, and the command that accepts or rejects
a run.

The automated gate prints nothing: it measures geometry the slicer emitted. Whether a support
actually comes off a printed miniature is a separate gate, held to prints, and it starts at
[The four-way physical print experiment](#the-four-way-physical-print-experiment).

## The pieces

| Piece | Where |
| --- | --- |
| Validator and runner | `scripts/validate_miniature_supports.py` (standard library only) |
| Shared measurement | `tests/fff_print/support_validation.hpp` / `.cpp` |
| Contact harness | `tests/fff_print/test_miniature_contacts.cpp`, case `Miniature contact decimation over a corpus` (`[.]`) |
| Auto-tilt harness | `tests/fff_print/test_auto_tilt.cpp`, case `Auto-tilt validation harness over a corpus` (`[.]`) |

Both harnesses carry the `[.]` tag, so `ctest` never discovers them: one corpus model costs hundreds
of full `Print::process()` passes. They are requested by name, and the runner does that for you.

## Commands

```bash
# The validator's own schema examples. Exits 0 only when every one of them behaves.
python3 scripts/validate_miniature_supports.py self-test

# Run both harnesses over a manifest, then accept or reject what they produced.
python3 scripts/validate_miniature_supports.py run \
    --manifest "$ORCA_MINIATURE_MANIFEST" \
    --test-binary build/arm64/tests/fff_print/Release/fff_print_tests.app/Contents/MacOS/fff_print_tests \
    --output-dir .crank/miniature-support-safety/validation

# Accept or reject rows a run already produced.
python3 scripts/validate_miniature_supports.py automated \
    --manifest "$ORCA_MINIATURE_MANIFEST" \
    --results out/miniature_contacts.jsonl --results out/auto_tilt.jsonl

# Accept or reject the prints an operator recorded. Nothing is sliced and no model file is read.
python3 scripts/validate_miniature_supports.py physical \
    --manifest "$ORCA_MINIATURE_MANIFEST" \
    --measurements "$ORCA_MINIATURE_PHYSICAL_CSV"
```

The `self-test` examples live in `scripts/test_validate_miniature_supports.py` and also run under
`python3 -m unittest discover -s scripts -p 'test_validate_miniature_supports.py'`.

`--test-binary` takes the path, so nothing assumes the macOS `.app` layout; derive it from your own
CMake output on Windows and Linux.

Exit codes: `0` for a complete accepted run, `1` for missing, invalid or failed data, `2` for
malformed command-line arguments.

### What `run` does, in order

1. Loads and validates the whole manifest, and hashes every model it names. Nothing is executed until
   both check out, so a bad hash never costs a slicing pass.
2. Creates the output directory and writes a `.gitignore` holding `*` into it, so a run inside a
   working tree leaves nothing to commit.
3. Invokes the test binary once with the single Catch2 filter argument
   `Miniature contact decimation over a corpus,Auto-tilt validation harness over a corpus`.
4. Hands it this environment:

   | Variable | Meaning |
   | --- | --- |
   | `ORCA_MINIATURE_MANIFEST` | the validated manifest; its presence is what puts either harness into acceptance mode |
   | `ORCA_MINIATURE_CORPUS` | the manifest's model root |
   | `ORCA_AUTOTILT_CORPUS` | the same root |
   | `ORCA_MINIATURE_RESULTS` | `<output-dir>/miniature_contacts.jsonl`, written by the contact harness |
   | `ORCA_AUTOTILT_RESULTS` | `<output-dir>/auto_tilt.jsonl`, written by the auto-tilt harness |

   `ORCA_AUTOTILT_COARSE` is removed from that environment, even when the calling shell set it: the
   exhaustive mode visits the whole grid or it is not the exhaustive mode.
5. Propagates a nonzero exit from the binary, then validates every row either harness wrote.

Without `ORCA_MINIATURE_MANIFEST` both harnesses still run as diagnostics and print the built-in fin
fixture alone. That run measures nothing the acceptance command accepts: `automated` rejects it,
because it satisfies no manifest.

## Manifest schema, version 1

```json
{
  "version": 1,
  "model_root": "models",
  "repeats": 7,
  "improvement_gain": { "support_volume_mm3": 0.5, "max_group_risk": 0.1 },
  "cases": [ … ]
}
```

| Field | Meaning |
| --- | --- |
| `version` | `1`. Anything else is rejected. |
| `model_root` | directory holding the models, relative to the manifest file |
| `repeats` | the suite's default repeat count |
| `improvement_gain` | how far a candidate has to clear the root before a saving counts, per metric |
| `cases` | a nonempty array; every case id is unique |

Each case:

| Field | Meaning |
| --- | --- |
| `id` | unique, and the name every row it produced carries |
| `model` | path under `model_root`; absolute paths and `..` are rejected |
| `sha256` | 64 lowercase hex digits, checked against the file before anything runs |
| `objects` | exactly one of `{"instance_ids": [...]}` (0-based indices into the file's flattened instance list) or `{"selectors": ["name:<object name>", "index:<object index>"]}`. An empty selection is an error, never a whole-file fallback. |
| `scale` | positive; multiplies each selected instance's own scaling factor |
| `config_digest` | 64 lowercase hex digits, declared by the manifest author (the harness copies it into each row and computes nothing); every row the case produced has to carry the same value |
| `category` | one of `thin_weapon`, `narrow_connection`, `cloth_edge`, `separate_island`, `stepped_island`, `constrained_routing` |
| `styles` | nonempty, from `tree_slim`, `tree_strong`, `tree_hybrid`, `organic` (the `support_style` values themselves) |
| `feature_modes` | nonempty, from `off`, `on` (`support_miniature_contacts`) |
| `nozzle_diameter_mm`, `extrusion_width_mm` | both positive, both stated: a nozzle case never inherits its width |
| `repeats` | at least 7 |
| `expected_outcome` | `complete`, `unresolved_coverage` or `organic_estimate` |
| `expected_reason` | required for `unresolved_coverage`, the exact reason code |
| `expected_outcome_by_mode`, `expected_reason_by_mode` | per feature mode, where the two modes differ |
| `quality_bounds` | `max_<metric>` keys, each a finite number |
| `config_overrides`, `override_reasons` | a documented override per key; an undocumented one is rejected |
| `demonstrates` | optional claims: `redundant_support_reduction`, `thin_feature_damage`, `weaker_neck_risk` |
| `physical_required`, `physical` | optional; `physical_required` defaults to false. See [The four-way physical print experiment](#the-four-way-physical-print-experiment) |

The suite as a whole has to carry all six categories, all three legacy styles, and the 0.25, 0.4 and
0.6 mm nozzles.

### Settings authority

The settings a 3MF carries stay authoritative. On top of them the harness applies, in order: the
style and feature mode the case is being measured under, the nozzle and extrusion width the case
states, and the case's own `config_overrides`. Every override has to name its reason in
`override_reasons`, so no unsupported style is quietly forced and then called equivalent.

### Per-mode outcomes

One generation runs in either feature mode, and its row reads the measurement: `complete` where the
measurement completed, `unresolved_coverage` where it did not. A case that behaves differently in
the two modes declares `expected_outcome_by_mode`; a case that declares only `expected_outcome` is
held to it in both.

## Result rows

One JSON object per line, written through the JSON library the tests already carry. Two row types.

A `pose` row is one measurement of one case, style, feature mode, pose and repeat, taken by one
harness. The harness is part of that identity: both harnesses measure the same case, style and mode,
and both write a row at the default pose, so a run that dropped it would read the two harnesses'
repeats as one series and reject its own correct output.

| Field | Meaning |
| --- | --- |
| `harness` | `miniature_contacts` or `auto_tilt` |
| `case_id`, `style`, `feature_mode`, `pose`, `repeat` | which measurement this is |
| `status` | `complete`, `unresolved_coverage`, `organic_estimate`, `invalid`, `unknown` |
| `measured` | false is a skipped measurement, which always fails |
| `printable`, `plate_contained` | `invalid` is accepted only where the evaluator refused the pose, through `plate_refusal` or `Print::validate` |
| `reason_codes` | the measurement's own reason codes |
| `metrics` | see below |
| `elapsed_s` | the wall clock the generation took |
| `peak_memory_bytes`, `peak_memory_available` | the process peak, or `null` under an explicit marker where the platform offers no reading (Windows has none in the test binaries) |
| `source_sha256`, `config_digest`, `build_revision` | the provenance of the row |
| `estimate_only`, `verified` | an Organic row is estimate-only and never verified |

`metrics` carries `support_volume_mm3`, `raft_volume_mm3`, `missing_critical_anchors`,
`invalid_paths`, `unrooted_groups`, `min_bed_margin`, `max_slenderness`, `unknown_contacts`,
`inaccessible_groups`, `max_group_risk` and `total_group_risk`, under three availability flags:
`coverage_available`, `stability_available` and `damage_available`. A domain nothing measured is not
a result of zero, so a number is only read where its flag says it was taken; a non-finite number
under a raised flag fails the run.

A `selection` row is what the auto-tilt exhaustive sweep found for one case, legacy style
(`tree_slim`, `tree_strong` or `tree_hybrid`) and feature mode. An `organic` style writes no auto-tilt
rows, because the evaluator refuses Organic before slicing; the contact harness carries that case as its
`organic_estimate` row. A selection row carries a `selection_summary`:

| Field | Meaning |
| --- | --- |
| `root_pose`, `selected_pose`, `best_pose` | the pose the object stands in, the pose the production search settled on, and the best pose the grid held |
| `grid_entries` | has to be 77, the entries `AutoTilt::grid()` returns unchanged |
| `evaluated_poses`, `invalid_poses` | they add up to `grid_entries` |
| `false_move` | the production comparator found the selected pose worse than the root |
| `discrete_worse` | the selected pose carries more contacts nothing could answer for, or more groups nothing can reach, than the best pose |
| `regret`, `regret_available` | `(selected - best) / max(abs(best), 1e-9)` on the first continuous objective of the ranking the two do not tie on |

## What a run has to clear

- Every declared style, feature mode and repeat produced exactly one row. A duplicate row and a
  missing repeat both fail.
- Every row's hashes match its manifest case, its build revision is stated, and its peak memory is a
  number or an explicit unavailable marker.
- A printable `complete` row measured coverage, stability and damage, and printed nothing in mid-air.
- An `unresolved_coverage` case is unresolved in every repeat and carries its declared reason. It
  never enters quality ranking, so its numbers are never held to a bound.
- An Organic row is estimate-only and never verified. `unknown` always fails, and `invalid` is
  accepted only where the evaluator refused the pose (`plate_refusal` or `Print::validate`).
- Every selection row visited all 77 grid entries, made no false move, carries no worse discrete
  classification than the best pose, and regrets at most 0.10.
- The suite demonstrated what it claims: at least one `redundant_support_reduction` case where some
  configuration's feature-on volume envelope clears its own feature-off envelope by the configured
  gain while no configuration of that case gives up coverage, stability or damage; at least one
  `weaker_neck_risk` case whose estimated removal risk improved by the configured gain in some
  configuration; and every `thin_feature_damage` case with damage that did not rise in any
  configuration.

### Why seven repeats

The legacy tree generator is not run-to-run reproducible (`AGENTS.md`, "Testing"): one project's
support-move count spread over 139166..141475 across seven runs of one binary. So every pose is
measured seven times and read as an envelope. A candidate counts as better only where its worst
reading clears the root's best reading by the configured gain; overlapping envelopes are
inconclusive, not a verified improvement. An envelope spans the repeats of one configuration and
nothing wider: the case, the harness, the style and the pose are all fixed, so the feature off and on
are the only things being compared. A pooled envelope over two styles or two poses reports a spread
no run ever produced. The 2 % support-move band `AGENTS.md` quotes came from one model on one host
and is not a universal volume tolerance.

## The four-way physical print experiment

Everything above measures geometry. None of it says a support came off a printed miniature without
taking a spear tip with it: that claim needs prints, and prints need a protocol that keeps one
treatment's advantage from being the day's filament or the operator's tenth attempt at the same cut.

### The four treatments

Every physical case is printed under four treatments, three repeats each, twelve prints per case:

| Treatment | `support_miniature_contacts` | Orientation |
| --- | --- | --- |
| `contacts_off_tilt_off` | off | the pose the model arrived in |
| `contacts_on_tilt_off` | on | the pose the model arrived in |
| `contacts_off_tilt_on` | off | the pose auto-tilt selected |
| `contacts_on_tilt_on` | on | the pose auto-tilt selected |

`contacts_off_tilt_off` is the baseline and `contacts_on_tilt_on` is the combined treatment. All four
are printed and all four are reported, so a saving stays attributable: a case where only the
combined treatment improves says the pair did it, and a case where `contacts_on_tilt_off` already
improves says the contacts did.

Held fixed across all twelve prints of a case: printer, nozzle, filament batch and its conditioning
(one spool, dried the same way), layer profile, temperatures, speeds, cooling, support gap and
interface settings, the removal tools and the operator. The two feature states and the orientation
the tilt state implies are the experiment's only variables. The validator holds `printer`,
`nozzle_mm`, `filament`, `layer_profile_digest`, `build_revision`, `operator` and `tool` constant
across a case and rejects a case where any of them moved.

Print order and removal order are each randomized over the twelve prints from one integer seed per
case, recorded on every row. Randomizing spreads the operator's practice effect and any drift of the
machine across the treatments instead of letting them follow one; recording the seed lets someone
else redraw the same two orders. Both orders are checked to be permutations of 1..12 within the case.

Three repeats bound nothing tightly. They catch a treatment that breaks a part on every print and
they cannot resolve a small difference, which is why acceptance asks for a direction and not for a
confidence interval.

### What a case declares

A case opts in with `physical_required: true` and then names what each treatment printed. The field
is optional and defaults to false, so a case that prints nothing declares nothing and validates
exactly as before.

```json
{
  "id": "thin_weapon_spear_slim_040",
  "physical_required": true,
  "physical": {
    "model_sha256": "…",
    "treatments": {
      "contacts_off_tilt_off": {
        "project": "prints/contacts_off_tilt_off.3mf",
        "project_sha256": "…",
        "config": "prints/contacts_off_tilt_off.ini",
        "config_sha256": "…"
      },
      "contacts_on_tilt_off":  { … },
      "contacts_off_tilt_on":  { … },
      "contacts_on_tilt_on":   { … }
    }
  }
}
```

All four treatments are required, each with a project and a config artifact and the 64-hex digest of
each. Artifact paths stay relative and under the manifest, like `model`. A print is therefore tied to
a sliced project and the config it was sliced under, never to a model name alone: "I printed the
spear" is not a record, and neither is a project nobody can hash.

### The measurement CSV

One row per print, twelve rows per case, read by header name. Every column below is required.

| Column | Meaning |
| --- | --- |
| `case_id` | a manifest case that declares `physical_required` |
| `treatment` | one of the four names above |
| `repeat` | 1, 2 or 3 |
| `run_order`, `removal_order` | this print's place in the two randomized orders, each a permutation of 1..12 per case |
| `order_seed` | the integer the orders were drawn from; one value per case |
| `model_sha256` | matches the case's `physical.model_sha256` |
| `project_sha256`, `config_sha256` | match this treatment's declared artifacts |
| `build_revision` | the slicer build that produced the project |
| `printer`, `nozzle_mm`, `filament`, `layer_profile_digest` | the fixed conditions; constant across the case |
| `completed` | `true` where the print finished |
| `detached_during_print` | `true` where the part came off the plate |
| `underside_defect_count` | contact scars, blemishes and pulled layers on the supported undersides |
| `broken_model_part_count` | model parts broken during removal |
| `damaged_feature_names` | which features; required to be nonempty where `broken_model_part_count` is above 0 |
| `removal_seconds` | the removal, timed |
| `support_mass_g` | the removed support, weighed |
| `operator`, `tool` | who removed it and with what; constant across the case |
| `observation_notes` | free text; photo links live here |

`completed` and `detached_during_print` take `true` or `false` and nothing else: `yes`, `mostly` and
an empty cell are all rejected rather than read as a `false`. Counts are nonnegative integers, and
`removal_seconds` and `support_mass_g` are finite nonnegative numbers, so `NaN`, `not measured` and
an empty cell each fail as an unmeasured removal rather than passing as a zero.

`observation_notes` is carried through to the report as written and is never parsed. A photo link in
it is a pointer for a human: the validator does not open it, does not check that it resolves, and
nothing about a passing case means a photograph was reviewed.

### What a physical case has to clear

- All twelve prints recorded, one row per `(case_id, treatment, repeat)`, every one `completed` and
  none `detached_during_print`.
- Every treatment carries three measured removals: three timings and three masses. An unmeasured
  removal, an omitted failure, a missing mass or a missing print makes the case incomplete, which is
  never a pass.
- The combined treatment broke no more model parts, and left no more underside defects, than the
  baseline.
- Of the median removal time and the median support mass, at least one fell against the baseline and
  neither rose. Medians are read over the three repeats, so one lucky cut does not carry a treatment.
- Hashes match the manifest artifacts, the fixed conditions are constant, and both orders are
  permutations of 1..12 under one recorded seed.

The command reports all four treatments, whether the case passed or not:

```text
case thin_weapon_spear_slim_040
  contacts_off_tilt_off completed 3/3  detached 0  broken 0  underside 1  median removal 71 s  median mass 1.2 g
  contacts_on_tilt_off  completed 3/3  detached 0  broken 0  underside 1  median removal 66 s  median mass 1.05 g
  contacts_off_tilt_on  completed 3/3  detached 0  broken 0  underside 0  median removal 63 s  median mass 1.14 g
  contacts_on_tilt_on   completed 3/3  detached 0  broken 0  underside 0  median removal 58 s  median mass 0.9 g
```

### Where the record lives

The CSV, the photos and the printed artifacts stay outside committed source, in a directory the
working tree ignores: point `ORCA_MINIATURE_PHYSICAL_CSV` at it and pass it to `--measurements`. The
validator only reads that file; it writes nothing and creates nothing, and it never touches the model
files, so a record can be validated on a machine that has neither the corpus nor the printer.

### Automated acceptance is not physical acceptance

The two gates answer different questions and neither substitutes for the other.

| Gate | What a pass means |
| --- | --- |
| `automated` | the emitted geometry preserved coverage and stability and improved what the manifest claims, over the corpus, in the slicer |
| `physical` | the recorded prints of that corpus are complete and the combined treatment measured no worse breakage and a lower removal time or support mass |

A `physical` pass is bounded by what was printed: three repeats of the cases the manifest declares,
on one printer, with one filament batch and one operator. That is evidence about those combinations
and not a proof that every miniature, material and machine is safe from breakage.

Release claims about reduced physical breakage stay pending until this comparison passes on measured
prints. Until then the feature's claim is the automated one: the geometry it emits preserves coverage
and stability and reduces the support it leaves in the places the corpus measures.

## Reading a failure

Every failure is one `FAIL <message>` line on stderr. An automated failure names the case, the style,
the feature mode and the repeat it came from; a physical failure names the case, the treatment and
the repeat. The exit code says which kind of failure it was; the lines say which measurement produced
it.

Runtime and memory are reported per row and nothing is promised about either: no latency budget is
claimed here, and none is raised automatically.
