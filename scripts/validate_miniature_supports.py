#!/usr/bin/env python3
"""Validate a miniature-support corpus manifest and the measurements a harness run produced.

Standard library only. See docs/miniature_support_validation.md for the manifest schema, the
result-row schema and how to read a failing run.

Exit codes: 0 for a complete accepted run, 1 for missing/invalid/failed data, 2 for malformed
command-line arguments.
"""

import argparse
import csv
import hashlib
import json
import math
import os
import statistics
import subprocess
import sys
import unittest

MANIFEST_VERSION = 1

# Every repeat of one pose the acceptance run demands. The legacy tree generator is not run-to-run
# reproducible (AGENTS.md "Testing"), so a single generation measures nothing about a pose.
ACCEPTANCE_REPEATS = 7

# The feature categories the suite has to carry before it says anything about miniatures.
REQUIRED_CATEGORIES = (
    "thin_weapon",
    "narrow_connection",
    "cloth_edge",
    "separate_island",
    "stepped_island",
    "constrained_routing",
)

# The legacy tree styles the suite has to exercise. Organic is not one of them: it is measured as an
# estimate and never as a generated result.
REQUIRED_STYLES = ("tree_slim", "tree_strong", "tree_hybrid")

# Organic is a style a case may name and the suite never has to carry: the evaluator does not answer
# for it, so an Organic case is measured as an estimate and never as a generated result. Spelled the
# way s_keys_map_SupportMaterialStyle spells it (PrintConfig.cpp), because the harness hands the
# string straight to set_deserialize_strict.
ALLOWED_STYLES = REQUIRED_STYLES + ("organic",)

# The nozzle diameters the suite has to cover, each with the extrusion width its case states.
REQUIRED_NOZZLE_DIAMETERS_MM = (0.25, 0.4, 0.6)

FEATURE_MODES = ("off", "on")

EXPECTED_OUTCOMES = ("complete", "unresolved_coverage", "organic_estimate")

# The outcomes that name a reason code of their own: a legacy pass that did not come out Complete
# has to say which condition it stopped at.
OUTCOMES_NEEDING_REASON = ("unresolved_coverage",)

ROW_TYPES = ("pose", "selection")

# The environment the runner hands the Catch2 binary.
ENV_MANIFEST = "ORCA_MINIATURE_MANIFEST"
ENV_MINIATURE_CORPUS = "ORCA_MINIATURE_CORPUS"
ENV_AUTOTILT_CORPUS = "ORCA_AUTOTILT_CORPUS"
ENV_MINIATURE_RESULTS = "ORCA_MINIATURE_RESULTS"
ENV_AUTOTILT_RESULTS = "ORCA_AUTOTILT_RESULTS"

# The two hidden Catch2 cases that own the corpus measurement, requested by name because Catch2
# never discovers a `[.]` case on its own.
CASE_NAMES = (
    "Miniature contact decimation over a corpus",
    "Auto-tilt validation harness over a corpus",
)

# The largest scalar regret a selected pose may carry against the best pose in the grid.
MAX_REGRET = 0.10

# The entries AutoTilt::grid() returns under its unchanged constants. The exhaustive validation mode
# visits every one of them; a sweep that visited fewer measured something else.
EXHAUSTIVE_GRID_ENTRIES = 77

# The four treatments every physical case is printed under. "contacts" is
# `support_miniature_contacts`; "tilt" is the auto-tilt pose against the pose the model arrived in.
TREATMENTS = (
    "contacts_off_tilt_off",
    "contacts_on_tilt_off",
    "contacts_off_tilt_on",
    "contacts_on_tilt_on",
)

# The two treatments acceptance compares. The rest are reported so a saving stays attributable to a
# feature rather than to the pair.
BASELINE_TREATMENT = TREATMENTS[0]
COMBINED_TREATMENT = TREATMENTS[3]

# Repeats per treatment, and the prints one physical case therefore costs.
PHYSICAL_REPEATS = 3
PHYSICAL_PRINTS_PER_CASE = len(TREATMENTS) * PHYSICAL_REPEATS

# Every column one printed repeat has to carry.
PHYSICAL_FIELDS = (
    "case_id",
    "treatment",
    "repeat",
    "run_order",
    "removal_order",
    "order_seed",
    "model_sha256",
    "project_sha256",
    "config_sha256",
    "build_revision",
    "printer",
    "nozzle_mm",
    "filament",
    "layer_profile_digest",
    "completed",
    "detached_during_print",
    "underside_defect_count",
    "broken_model_part_count",
    "damaged_feature_names",
    "removal_seconds",
    "support_mass_g",
    "operator",
    "tool",
    "observation_notes",
)

PHYSICAL_COUNT_FIELDS = ("underside_defect_count", "broken_model_part_count")
PHYSICAL_INT_FIELDS = ("repeat", "run_order", "removal_order", "order_seed") + PHYSICAL_COUNT_FIELDS
PHYSICAL_MEASUREMENT_FIELDS = ("removal_seconds", "support_mass_g")
PHYSICAL_BOOLEAN_FIELDS = ("completed", "detached_during_print")
PHYSICAL_TEXT_FIELDS = (
    "model_sha256",
    "project_sha256",
    "config_sha256",
    "build_revision",
    "printer",
    "filament",
    "layer_profile_digest",
    "damaged_feature_names",
    "operator",
    "tool",
)

# What the PHYSICAL_PRINTS_PER_CASE prints of one case hold fixed. Only the two feature states, and the orientation the
# tilt state implies, are the experiment's variables.
PHYSICAL_CONSTANT_FIELDS = (
    "printer",
    "nozzle_mm",
    "filament",
    "layer_profile_digest",
    "build_revision",
    "operator",
    "tool",
)

# The fields a row has to fill in with something, whatever it says.
PHYSICAL_NONEMPTY_FIELDS = ("build_revision", "printer", "filament", "layer_profile_digest", "operator", "tool")

HEX64 = set("0123456789abcdef")


def _is_hex64(value):
    return isinstance(value, str) and len(value) == 64 and set(value) <= HEX64


def _is_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _is_finite(value):
    return _is_number(value) and math.isfinite(value)


def sha256_of_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _validate_case(case, index, failures):
    where = "case %d" % index
    if not isinstance(case, dict):
        failures.append("%s: not an object" % where)
        return
    case_id = case.get("id")
    if not isinstance(case_id, str) or not case_id:
        failures.append("%s: id must be a nonempty string" % where)
    else:
        where = "case %s" % case_id

    model = case.get("model")
    if not isinstance(model, str) or not model:
        failures.append("%s: model must be a nonempty relative path" % where)
    elif os.path.isabs(model) or ".." in model.replace("\\", "/").split("/"):
        failures.append("%s: model path %r must stay under the model root" % (where, model))

    if not _is_hex64(case.get("sha256")):
        failures.append("%s: sha256 must be 64 lowercase hex digits" % where)
    if not _is_hex64(case.get("config_digest")):
        failures.append("%s: config_digest must be 64 lowercase hex digits" % where)

    objects = case.get("objects")
    if not isinstance(objects, dict):
        failures.append("%s: objects must name the instances or the selectors to print" % where)
    else:
        ids = objects.get("instance_ids")
        selectors = objects.get("selectors")
        chosen = [value for value in (ids, selectors) if value is not None]
        if len(chosen) != 1:
            failures.append("%s: objects takes exactly one of instance_ids or selectors" % where)
        elif not isinstance(chosen[0], list) or not chosen[0]:
            failures.append("%s: the object selection is empty" % where)

    if not _is_finite(case.get("scale")) or case.get("scale") <= 0.0:
        failures.append("%s: scale must be a positive finite number" % where)

    category = case.get("category")
    if category not in REQUIRED_CATEGORIES:
        failures.append("%s: category %r is not one of %s" % (where, category, ", ".join(REQUIRED_CATEGORIES)))

    styles = case.get("styles")
    if not isinstance(styles, list) or not styles:
        failures.append("%s: styles must be a nonempty list" % where)
    else:
        for style in styles:
            if style not in ALLOWED_STYLES:
                failures.append("%s: style %r is not a style this suite measures" % (where, style))

    modes = case.get("feature_modes")
    if not isinstance(modes, list) or not modes:
        failures.append("%s: feature_modes must be a nonempty list" % where)
    else:
        for mode in modes:
            if mode not in FEATURE_MODES:
                failures.append("%s: feature mode %r is not one of %s" % (where, mode, ", ".join(FEATURE_MODES)))

    if not _is_finite(case.get("nozzle_diameter_mm")) or case.get("nozzle_diameter_mm") <= 0.0:
        failures.append("%s: nozzle_diameter_mm must be a positive finite number" % where)
    if not _is_finite(case.get("extrusion_width_mm")) or case.get("extrusion_width_mm") <= 0.0:
        failures.append("%s: extrusion_width_mm must be stated explicitly beside the nozzle" % where)

    repeats = case.get("repeats")
    if not isinstance(repeats, int) or isinstance(repeats, bool) or repeats < ACCEPTANCE_REPEATS:
        failures.append("%s: repeats must be at least %d" % (where, ACCEPTANCE_REPEATS))

    outcome = case.get("expected_outcome")
    if outcome not in EXPECTED_OUTCOMES:
        failures.append("%s: expected_outcome %r is not one of %s" % (where, outcome, ", ".join(EXPECTED_OUTCOMES)))
    reason = case.get("expected_reason")
    if outcome in OUTCOMES_NEEDING_REASON:
        if not isinstance(reason, str) or not reason:
            failures.append("%s: expected_outcome %s needs an exact expected_reason code" % (where, outcome))
    elif reason and not case.get("expected_reason_by_mode"):
        failures.append("%s: expected_outcome %s names no reason code, but expected_reason is set" % (where, outcome))

    for field, allowed in (("expected_outcome_by_mode", EXPECTED_OUTCOMES), ("expected_reason_by_mode", None)):
        by_mode = case.get(field)
        if by_mode is None:
            continue
        if not isinstance(by_mode, dict) or not by_mode:
            failures.append("%s: %s must be a nonempty object keyed by feature mode" % (where, field))
            continue
        for mode, value in sorted(by_mode.items()):
            if mode not in FEATURE_MODES:
                failures.append("%s: %s names feature mode %r" % (where, field, mode))
            elif allowed is not None and value not in allowed:
                failures.append("%s: %s[%s] is %r, not one of %s" % (where, field, mode, value, ", ".join(allowed)))
            elif allowed is None and (not isinstance(value, str) or not value):
                failures.append("%s: %s[%s] is not a reason code" % (where, field, mode))
    # A mode whose declared outcome names a reason has to state one.
    for mode in case.get("feature_modes", []) or []:
        outcome_here, reason_here = _expected_for_mode(case, mode)
        if outcome_here in OUTCOMES_NEEDING_REASON and not (isinstance(reason_here, str) and reason_here):
            failures.append("%s: feature mode %s expects %s and names no reason code" % (where, mode, outcome_here))

    bounds = case.get("quality_bounds")
    if not isinstance(bounds, dict) or not bounds:
        failures.append("%s: quality_bounds must state the bounds this case is held to" % where)
    else:
        for key, value in sorted(bounds.items()):
            if not _is_finite(value):
                failures.append("%s: quality bound %s is not a finite number" % (where, key))

    _validate_physical_declaration(case, where, failures)

    # Loaded 3MF settings stay authoritative: a case may override one, and only where it says why.
    overrides = case.get("config_overrides", {})
    reasons = case.get("override_reasons", {})
    if not isinstance(overrides, dict):
        failures.append("%s: config_overrides must be an object" % where)
    elif not isinstance(reasons, dict):
        failures.append("%s: override_reasons must be an object" % where)
    else:
        for key in sorted(overrides):
            if not isinstance(reasons.get(key), str) or not reasons.get(key):
                failures.append("%s: config override %s is not documented in override_reasons" % (where, key))


def _relative_artifact(path):
    """Whether `path` is a nonempty relative path that stays under the manifest's directory."""
    return isinstance(path, str) and bool(path) and not os.path.isabs(path) \
        and ".." not in path.replace("\\", "/").split("/")


def _validate_physical_declaration(case, where, failures):
    """A case that opts into the print experiment names what every treatment printed.

    `physical_required` is optional and defaults to false, so a case that prints nothing declares
    nothing. Where it is true, the model hash and all four treatments' project and config artifacts
    are required: a row is matched to a sliced project, never to a model name alone.
    """
    required = case.get("physical_required", False)
    if not isinstance(required, bool):
        failures.append("%s: physical_required is %r, not true or false" % (where, required))
        return
    if not required:
        return
    physical = case.get("physical")
    if not isinstance(physical, dict):
        failures.append("%s: physical_required names no physical artifacts" % where)
        return
    if not _is_hex64(physical.get("model_sha256")):
        failures.append("%s: physical.model_sha256 must be 64 lowercase hex digits" % where)
    treatments = physical.get("treatments")
    if not isinstance(treatments, dict):
        failures.append("%s: physical.treatments must name all %d treatments" % (where, len(TREATMENTS)))
        return
    for name in sorted(treatments):
        if name not in TREATMENTS:
            failures.append("%s: physical treatment %r is not one of %s" % (where, name, ", ".join(TREATMENTS)))
    for treatment in TREATMENTS:
        entry = treatments.get(treatment)
        if not isinstance(entry, dict):
            failures.append("%s: physical treatment %s names no printed artifacts" % (where, treatment))
            continue
        for field in ("project_sha256", "config_sha256"):
            if not _is_hex64(entry.get(field)):
                failures.append("%s: physical %s %s must be 64 lowercase hex digits" % (where, treatment, field))
        for field in ("project", "config"):
            if not _relative_artifact(entry.get(field)):
                failures.append("%s: physical %s %s %r must be a relative path under the manifest"
                                % (where, treatment, field, entry.get(field)))


def validate_manifest(manifest):
    """Structural and suite-coverage failures of a manifest document, as a list of strings."""
    failures = []
    if not isinstance(manifest, dict):
        return ["manifest is not an object"]
    if manifest.get("version") != MANIFEST_VERSION:
        failures.append("manifest version %r is not %d" % (manifest.get("version"), MANIFEST_VERSION))

    cases = manifest.get("cases")
    if not isinstance(cases, list) or not cases:
        failures.append("manifest lists no cases")
        return failures

    seen_ids = set()
    for index, case in enumerate(cases):
        _validate_case(case, index, failures)
        if isinstance(case, dict):
            case_id = case.get("id")
            if case_id in seen_ids:
                failures.append("case id %r appears more than once" % case_id)
            seen_ids.add(case_id)

    categories = {case.get("category") for case in cases if isinstance(case, dict)}
    for category in REQUIRED_CATEGORIES:
        if category not in categories:
            failures.append("the suite carries no %s case" % category)

    styles = set()
    nozzles = set()
    for case in cases:
        if not isinstance(case, dict):
            continue
        for style in case.get("styles", []) or []:
            styles.add(style)
        if _is_finite(case.get("nozzle_diameter_mm")):
            nozzles.add(round(float(case["nozzle_diameter_mm"]), 6))
    for style in REQUIRED_STYLES:
        if style not in styles:
            failures.append("the suite exercises no %s case" % style)
    for nozzle in REQUIRED_NOZZLE_DIAMETERS_MM:
        if round(nozzle, 6) not in nozzles:
            failures.append("the suite carries no %.2f mm nozzle case" % nozzle)

    return failures


ROW_STATUSES = EXPECTED_OUTCOMES + ("unknown", "invalid")

# Which availability flag answers for which measurements. A domain that was never measured is not a
# result of zero, so its numbers are only read where its flag says they were taken.
METRIC_DOMAINS = {
    "coverage_available": ("support_volume_mm3", "raft_volume_mm3", "missing_critical_anchors"),
    "stability_available": ("invalid_paths", "unrooted_groups", "min_bed_margin", "max_slenderness"),
    "damage_available": ("unknown_contacts", "inaccessible_groups", "max_group_risk", "total_group_risk"),
}

# The statuses a printable legacy row may carry and still be held to the full measurement.
PRINTABLE_STATUSES = ("complete",)

# The two harnesses that write rows. The name is part of a row's identity, so a row that names neither
# belongs to no harness and would form a repeat group of its own.
HARNESSES = ("miniature_contacts", "auto_tilt")

ROW_REQUIRED_KEYS = (
    "case_id",
    "harness",
    "style",
    "feature_mode",
    "pose",
    "repeat",
    "status",
    "measured",
    "metrics",
    "elapsed_s",
    "source_sha256",
    "config_digest",
    "build_revision",
)


def load_rows(paths):
    """Every JSONL row of `paths`, and the failures reading them produced."""
    rows = []
    failures = []
    for path in paths:
        if not os.path.isfile(path):
            failures.append("results file %s does not exist" % path)
            continue
        seen = 0
        with open(path, "r", encoding="utf-8") as handle:
            for number, line in enumerate(handle, start=1):
                if not line.strip():
                    continue
                try:
                    rows.append(json.loads(line))
                except ValueError as error:
                    failures.append("%s line %d does not parse: %s" % (path, number, error))
                    continue
                seen += 1
        if seen == 0:
            failures.append("results file %s holds no rows" % path)
    return rows, failures


def validate_corpus(manifest, manifest_dir):
    """Failures reading the models a manifest names: a missing directory, a missing file, a bad hash."""
    failures = []
    root = os.path.join(manifest_dir, manifest.get("model_root", "."))
    if not os.path.isdir(root):
        return ["model root %s does not exist" % root]
    for case in manifest.get("cases", []):
        model = case.get("model")
        if not isinstance(model, str) or not model:
            continue
        path = os.path.join(root, model)
        if not os.path.isfile(path):
            failures.append("case %s: model %s does not exist" % (case.get("id"), path))
            continue
        digest = sha256_of_file(path)
        if digest != case.get("sha256"):
            failures.append("case %s: %s hashes %s, manifest says %s" % (case.get("id"), path, digest, case.get("sha256")))
    return failures


def _pose_key(pose):
    if not isinstance(pose, dict):
        return None
    return (round(float(pose.get("tilt_deg", 0.0)), 6), round(float(pose.get("lean_deg", 0.0)), 6))


def _validate_row_shape(row, index, cases, failures):
    """The case this row belongs to, or None when the row cannot be attributed to one."""
    where = "row %d" % index
    if not isinstance(row, dict):
        failures.append("%s: not an object" % where)
        return None
    for key in ROW_REQUIRED_KEYS:
        if key not in row:
            failures.append("%s: no %s" % (where, key))
            return None
    case = cases.get(row["case_id"])
    if case is None:
        failures.append("%s: names case %r, which the manifest does not list" % (where, row["case_id"]))
        return None
    where = "case %s %s/%s repeat %s" % (row["case_id"], row["style"], row["feature_mode"], row["repeat"])

    if row.get("row_type", "pose") not in ROW_TYPES:
        failures.append("%s: row_type %r is not one of %s" % (where, row.get("row_type"), ", ".join(ROW_TYPES)))
    if row["harness"] not in HARNESSES:
        failures.append("%s: harness %r is not one of %s" % (where, row["harness"], ", ".join(HARNESSES)))
    if row["style"] not in case.get("styles", []):
        failures.append("%s: style %r is not declared by its case" % (where, row["style"]))
    if row["feature_mode"] not in case.get("feature_modes", []):
        failures.append("%s: feature mode %r is not declared by its case" % (where, row["feature_mode"]))
    if not isinstance(row["repeat"], int) or isinstance(row["repeat"], bool) or not 0 <= row["repeat"] < case.get("repeats", 0):
        failures.append("%s: repeat index is outside the declared %s repeats" % (where, case.get("repeats")))
    if _pose_key(row["pose"]) is None:
        failures.append("%s: pose names no tilt and lean" % where)
    if row["status"] not in ROW_STATUSES:
        failures.append("%s: status %r is not one of %s" % (where, row["status"], ", ".join(ROW_STATUSES)))
    if row["measured"] is not True:
        failures.append("%s: the measurement was skipped" % where)
    if row["source_sha256"] != case.get("sha256"):
        failures.append("%s: source hash does not match the manifest case" % where)
    if row["config_digest"] != case.get("config_digest"):
        failures.append("%s: config digest does not match the manifest case" % where)
    if not isinstance(row["build_revision"], str) or not row["build_revision"]:
        failures.append("%s: no build revision" % where)
    elif row["build_revision"] == "0000000":
        # libslic3r_version.h's fallback: the harness was built without the commit stamped.
        failures.append("%s: build revision is the version header's 0000000 fallback" % where)
    if not _is_finite(row["elapsed_s"]) or row["elapsed_s"] < 0.0:
        failures.append("%s: elapsed_s is not a finite runtime" % where)

    # Peak memory is a value or an explicit unavailable marker, never a silent absence.
    available = row.get("peak_memory_available")
    memory = row.get("peak_memory_bytes")
    if available is True:
        if not _is_finite(memory) or memory < 0:
            failures.append("%s: peak memory is marked available and is not a number" % where)
    elif available is False:
        if memory is not None:
            failures.append("%s: peak memory is marked unavailable and still carries a value" % where)
    else:
        failures.append("%s: peak_memory_available says neither yes nor no" % where)

    metrics = row["metrics"]
    if not isinstance(metrics, dict):
        failures.append("%s: metrics is not an object" % where)
        return case
    for flag, keys in sorted(METRIC_DOMAINS.items()):
        if metrics.get(flag) is not True:
            continue
        for key in keys:
            if not _is_finite(metrics.get(key)):
                failures.append("%s: %s is available and %s is not a finite number" % (where, flag, key))
    return case


def _expected_for_mode(case, mode):
    """The outcome and reason this case declares for one feature mode.

    A case may declare a different outcome for each mode; one that declares only one outcome is held
    to it in both.
    """
    outcome = (case.get("expected_outcome_by_mode") or {}).get(mode, case.get("expected_outcome"))
    reason = (case.get("expected_reason_by_mode") or {}).get(mode, case.get("expected_reason"))
    return outcome, reason


def _validate_row_outcome(row, case, failures):
    where = "case %s %s/%s repeat %s" % (row["case_id"], row["style"], row["feature_mode"], row["repeat"])
    status = row["status"]
    expected, expected_reason = _expected_for_mode(case, row["feature_mode"])
    metrics = row["metrics"] if isinstance(row["metrics"], dict) else {}

    if status == "invalid":
        # The exhaustive sweep records Invalid where the evaluator refused the pose, through plate_refusal
        # or Print::validate, and for nothing else.
        if row.get("plate_contained") is not False:
            failures.append("%s: Invalid without a plate that refused the pose" % where)
        return
    if status == "unknown":
        failures.append("%s: Unknown is never an expected result" % where)
        return
    if status != expected:
        failures.append("%s: status %s is not the declared outcome %s" % (where, status, expected))
        return

    if status in OUTCOMES_NEEDING_REASON:
        if expected_reason not in (row.get("reason_codes") or []):
            failures.append("%s: %s does not carry its declared reason %r" % (where, status, expected_reason))
    if status == "organic_estimate":
        if row.get("estimate_only") is not True:
            failures.append("%s: an Organic row must be marked estimate-only" % where)
        if row.get("verified") is not False:
            failures.append("%s: an Organic row is never a verified result" % where)
    if status in PRINTABLE_STATUSES and row.get("printable", True):
        for flag in sorted(METRIC_DOMAINS):
            if metrics.get(flag) is not True:
                failures.append("%s: a printable %s row did not measure %s" % (where, status, flag))
        if metrics.get("invalid_paths") != 0:
            failures.append("%s: a printable %s row printed material in mid-air" % (where, status))


def _validate_selection_row(row, case, failures):
    """What the exhaustive sweep found, held to the gates the selection has to clear."""
    where = "case %s %s/%s selection" % (row["case_id"], row["style"], row["feature_mode"])
    summary = row.get("selection_summary")
    if not isinstance(summary, dict):
        failures.append("%s: no selection_summary" % where)
        return
    grid = summary.get("grid_entries")
    if grid != EXHAUSTIVE_GRID_ENTRIES:
        failures.append("%s: the sweep visited %r of the %d grid entries" % (where, grid, EXHAUSTIVE_GRID_ENTRIES))
    evaluated = summary.get("evaluated_poses")
    invalid = summary.get("invalid_poses")
    if not isinstance(evaluated, int) or not isinstance(invalid, int) or evaluated + invalid != grid:
        failures.append("%s: %r evaluated and %r invalid do not account for %r grid entries"
                        % (where, evaluated, invalid, grid))
    if summary.get("false_move") is not False:
        failures.append("%s: the search left the root for a pose the print measures as worse" % where)
    if summary.get("discrete_worse") is not False:
        failures.append("%s: the selected pose carries a worse safety classification than the best pose" % where)
    if summary.get("regret_available") is not True:
        failures.append("%s: the regret against the best pose was never measured" % where)
    elif not _is_finite(summary.get("regret")) or summary["regret"] > MAX_REGRET:
        failures.append("%s: regret %r is above %g" % (where, summary.get("regret"), MAX_REGRET))

    # The row is still a measurement of the pose the search settled on, held to the printable gates.
    _validate_row_outcome(row, case, failures)


def validate_results(manifest, rows):
    """Every failure the rows of one run carry against the manifest that declared them."""
    failures = []
    cases = {case["id"]: case for case in manifest.get("cases", []) if isinstance(case, dict) and "id" in case}
    if not rows:
        return ["the run produced no result rows"]

    pose_rows = []
    for index, row in enumerate(rows):
        case = _validate_row_shape(row, index, cases, failures)
        if case is None:
            continue
        if row.get("row_type", "pose") == "selection":
            _validate_selection_row(row, case, failures)
            continue
        pose_rows.append(row)
        _validate_row_outcome(row, case, failures)

    # Every declared style, feature mode and repeat produced exactly one row.
    groups = {}
    for row in pose_rows:
        # The harness belongs in the key: both harnesses measure the same case, style and mode, and
        # both write a row at the default pose, so a key without it merges two runs into one group.
        key = (row["case_id"], row["style"], row["feature_mode"], _pose_key(row["pose"]), row.get("harness"))
        groups.setdefault(key, []).append(row["repeat"])
    for case in manifest.get("cases", []):
        for style in case.get("styles", []):
            for mode in case.get("feature_modes", []):
                produced = [key for key in groups if key[:3] == (case["id"], style, mode)]
                if not produced:
                    failures.append("case %s %s/%s produced no rows" % (case["id"], style, mode))
                    continue
                for key in produced:
                    repeats = sorted(groups[key])
                    expected = list(range(case.get("repeats", 0)))
                    if repeats != expected:
                        failures.append("case %s %s/%s pose %s produced repeats %s, not %s"
                                        % (case["id"], style, mode, key[3], repeats, expected))

    failures.extend(_validate_quality_bounds(manifest, pose_rows))
    return failures


def _validate_quality_bounds(manifest, pose_rows):
    """Per-case quality bounds, over the cases quality ranking admits."""
    failures = []
    for case in manifest.get("cases", []):
        # An unresolved case never enters quality ranking: nothing about its numbers is a result.
        if case.get("expected_outcome") == "unresolved_coverage":
            continue
        ranked = [row for row in pose_rows if row["case_id"] == case["id"] and row["status"] in PRINTABLE_STATUSES]
        # One configuration per group, feature mode included: a bound is a statement about what a
        # style at a pose under a mode may reach, so the group that broke it is the one named.
        groups = _configurations(ranked, lambda row: (_envelope_key(row), row["feature_mode"]))
        for key, bound in sorted((case.get("quality_bounds") or {}).items()):
            if not key.startswith("max_"):
                failures.append("case %s: quality bound %s names no metric" % (case["id"], key))
                continue
            metric = key[len("max_"):]
            for group_key, group in groups:
                observed = [row["metrics"].get(metric) for row in group if isinstance(row.get("metrics"), dict)]
                observed = [value for value in observed if _is_finite(value)]
                if not observed:
                    continue
                if max(observed) > bound:
                    failures.append("case %s %s/%s: %s reached %g, above the bound %g"
                                    % (case["id"], _configuration_text(group_key[0]), group_key[1],
                                       metric, max(observed), bound))
    return failures


def envelope_of(values):
    """The spread one metric covered over its repeats, as (min, max). Empty measures nothing."""
    finite = [value for value in values if _is_finite(value)]
    if not finite:
        return None
    return (min(finite), max(finite))


def envelope_clears(root, candidate, gain):
    """Whether `candidate`'s worst reading sits at least `gain` below `root`'s best reading.

    Lower is better throughout. Overlapping spreads are inconclusive and answer False, and so does a
    gain nobody configured: a generator that is not run-to-run reproducible cannot be read for an
    improvement of nothing.
    """
    if root is None or candidate is None or not (gain > 0.0):
        return False
    return root[0] - candidate[1] >= gain


def _envelope_key(row):
    """The configuration whose repeats one envelope spans: everything a pose row varies but the
    feature mode and the repeat. Two styles, two poses and two harnesses measure different things, so
    an envelope pooling them reports a spread no configuration ever produced.
    """
    return (row["case_id"], row.get("harness"), row["style"], _pose_key(row["pose"]))


def _configuration_text(key):
    return "%s %s pose %s" % (key[1], key[2], key[3])


def _configurations(rows, key_of):
    """The rows of each configuration `key_of` names, as (key, rows), in first-seen order."""
    groups = []
    index = {}
    for row in rows:
        key = key_of(row)
        if key not in index:
            index[key] = len(groups)
            groups.append((key, []))
        groups[index[key]][1].append(row)
    return groups


def _case_configurations(rows, case_id):
    return _configurations([row for row in rows if row["case_id"] == case_id], _envelope_key)


def _mode_values(rows, mode, metric):
    """One metric over the repeats of one configuration in one feature mode."""
    return [row["metrics"].get(metric) for row in rows
            if row["feature_mode"] == mode and isinstance(row.get("metrics"), dict)]


# What a case must not give up to buy a saving: the counts and the weights, read on the worst reading
# each mode produced.
NO_WORSE_METRICS = (
    "missing_critical_anchors",
    "invalid_paths",
    "unrooted_groups",
    "unknown_contacts",
    "inaccessible_groups",
    "max_group_risk",
    "total_group_risk",
)


def _no_worse(rows, case_id, failures, where):
    """Whether the case gave nothing up, read inside each configuration.

    One style's readings are not repeats of another's, so a metric that rose under one style must not
    hide behind a second style's higher numbers: every configuration answers for itself.
    """
    groups = _case_configurations(rows, case_id)
    if not groups:
        failures.append("%s: no printable rows to compare" % where)
        return False
    ok = True
    for key, group in groups:
        text = _configuration_text(key)
        if not any(row["feature_mode"] == "off" for row in group) or not any(row["feature_mode"] == "on" for row in group):
            failures.append("%s %s: measured in one feature mode only, so nothing compares" % (where, text))
            ok = False
            continue
        for metric in NO_WORSE_METRICS:
            off = envelope_of(_mode_values(group, "off", metric))
            on = envelope_of(_mode_values(group, "on", metric))
            if off is None or on is None:
                failures.append("%s %s: %s was not measured in both modes" % (where, text, metric))
                ok = False
            elif on[1] > off[1]:
                failures.append("%s %s: %s rose from %g to %g with the feature on"
                                % (where, text, metric, off[1], on[1]))
                ok = False
    return ok


def _clears_in_any_configuration(rows, case_id, metric, gain):
    """Whether some configuration's own off and on envelopes clear `gain` on `metric`."""
    for _key, group in _case_configurations(rows, case_id):
        if envelope_clears(envelope_of(_mode_values(group, "off", metric)),
                           envelope_of(_mode_values(group, "on", metric)), gain):
            return True
    return False


def validate_demonstrations(manifest, rows):
    """Whether the suite demonstrated the improvements it claims, not merely preserved behavior."""
    failures = []
    gains = manifest.get("improvement_gain") or {}
    printable = [row for row in rows
                 if row.get("row_type", "pose") == "pose" and row.get("status") in PRINTABLE_STATUSES
                 and isinstance(row.get("metrics"), dict)]

    demonstrated = set()
    for case in manifest.get("cases", []):
        kinds = case.get("demonstrates") or []
        for kind in kinds:
            where = "case %s (%s)" % (case["id"], kind)
            if kind == "redundant_support_reduction":
                if not _clears_in_any_configuration(printable, case["id"], "support_volume_mm3",
                                                    gains.get("support_volume_mm3", 0.0)):
                    failures.append("%s: no configuration's generated volume envelopes clear the gain, "
                                    "so no saving is demonstrated" % where)
                    continue
                if not _no_worse(printable, case["id"], failures, where):
                    continue
                demonstrated.add(kind)
            elif kind == "thin_feature_damage":
                if _no_worse(printable, case["id"], failures, where):
                    demonstrated.add(kind)
            elif kind == "weaker_neck_risk":
                improved = any(_clears_in_any_configuration(printable, case["id"], metric, gains.get(metric, 0.0))
                               for metric in ("max_group_risk", "total_group_risk"))
                if not improved:
                    failures.append("%s: no estimated removal risk improved by the configured gain" % where)
                    continue
                if not _no_worse(printable, case["id"], failures, where):
                    continue
                demonstrated.add(kind)
            else:
                failures.append("%s: nothing validates this claim" % where)

    for kind in ("redundant_support_reduction", "weaker_neck_risk"):
        if kind not in demonstrated:
            failures.append("the suite demonstrates no %s: behavior may be preserved, but the requested "
                            "improvement is not shown" % kind)
    return failures


# ---------------------------------------------------------------------------------------------
# The four-way physical print experiment. Nothing here prints or measures anything: it reads what an
# operator recorded and says whether the record is complete enough to compare the four treatments.
# ---------------------------------------------------------------------------------------------


def load_measurements(path):
    """The recorded prints in the CSV at `path`, and the failures reading it produced."""
    if not os.path.isfile(path):
        return [], ["measurements file %s does not exist" % path]
    rows = []
    with open(path, "r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        header = reader.fieldnames or []
        missing = [field for field in PHYSICAL_FIELDS if field not in header]
        if missing:
            return [], ["measurements file %s has no %s column" % (path, field) for field in missing]
        for row in reader:
            # A blank separator line records no print.
            if not any(isinstance(value, str) and value.strip() for value in row.values()):
                continue
            rows.append(row)
    if not rows:
        return [], ["measurements file %s records no prints" % path]
    return rows, []


def _physical_text(row, field):
    value = row.get(field)
    return value.strip() if isinstance(value, str) else ""


def _physical_int(value):
    """The integer this cell names, or None where it names none."""
    if not isinstance(value, str) or not value.strip():
        return None
    try:
        return int(value.strip())
    except ValueError:
        return None


def _physical_float(value):
    """The finite number this cell names, or None. `NaN` and `not measured` both answer None."""
    if not isinstance(value, str) or not value.strip():
        return None
    try:
        number = float(value.strip())
    except ValueError:
        return None
    return number if math.isfinite(number) else None


def _physical_bool(value):
    """True, False, or None for anything that is not one of the two words."""
    text = value.strip().lower() if isinstance(value, str) else ""
    if text == "true":
        return True
    if text == "false":
        return False
    return None


def _parse_physical_row(row, number):
    """One CSV row's typed fields, with a None wherever a cell parses to nothing.

    Every field is read by its header name, so a reordered sheet still records the same prints.
    """
    record = {"row_number": number, "raw": row}
    record["case_id"] = _physical_text(row, "case_id")
    record["treatment"] = _physical_text(row, "treatment")
    for field in PHYSICAL_INT_FIELDS:
        record[field] = _physical_int(row.get(field))
    for field in PHYSICAL_MEASUREMENT_FIELDS + ("nozzle_mm",):
        record[field] = _physical_float(row.get(field))
    for field in PHYSICAL_BOOLEAN_FIELDS:
        record[field] = _physical_bool(row.get(field))
    for field in PHYSICAL_TEXT_FIELDS:
        record[field] = _physical_text(row, field)
    # The notes are carried through as written. Nothing here follows a link one holds.
    notes = row.get("observation_notes")
    record["observation_notes"] = notes if isinstance(notes, str) else ""
    return record


def _validate_physical_row(case, record, failures):
    """One print: its provenance, its two booleans, its counts and its two measurements."""
    where = "case %s %s repeat %d" % (case["id"], record["treatment"], record["repeat"])
    declared = case.get("physical") or {}
    if record["model_sha256"] != declared.get("model_sha256"):
        failures.append("%s: model hash %r is not the hash its manifest case declares"
                        % (where, record["model_sha256"]))
    entry = (declared.get("treatments") or {}).get(record["treatment"]) or {}
    for field in ("project_sha256", "config_sha256"):
        if record[field] != entry.get(field):
            failures.append("%s: %s %r is not the artifact this treatment declares" % (where, field, record[field]))
    for field in PHYSICAL_BOOLEAN_FIELDS:
        if record[field] is None:
            failures.append("%s: %s is %r, and takes true or false only" % (where, field, record["raw"].get(field)))
    for field in PHYSICAL_COUNT_FIELDS:
        if record[field] is None or record[field] < 0:
            failures.append("%s: %s is %r, not a count" % (where, field, record["raw"].get(field)))
    for field in PHYSICAL_MEASUREMENT_FIELDS:
        if record[field] is None or record[field] < 0.0:
            failures.append("%s: %s is %r, so this removal was never measured"
                            % (where, field, record["raw"].get(field)))
    if record["nozzle_mm"] is None or record["nozzle_mm"] <= 0.0:
        failures.append("%s: nozzle_mm is %r, not a diameter" % (where, record["raw"].get("nozzle_mm")))
    broken = record["broken_model_part_count"]
    if broken is not None and broken > 0 and not record["damaged_feature_names"]:
        failures.append("%s: %d model parts broke and damaged_feature_names is empty" % (where, broken))
    for field in PHYSICAL_NONEMPTY_FIELDS:
        if not record[field]:
            failures.append("%s: %s is empty" % (where, field))


def _validate_physical_constants(case_id, records):
    """The one seed a case draws, and the conditions its PHYSICAL_PRINTS_PER_CASE prints hold fixed."""
    failures = []
    if any(record["order_seed"] is None for record in records):
        failures.append("case %s: order_seed is not an integer on every row" % case_id)
    seeds = sorted({record["order_seed"] for record in records if record["order_seed"] is not None})
    if len(seeds) > 1:
        failures.append("case %s: the case records %d order seeds (%s) and draws one"
                        % (case_id, len(seeds), ", ".join(str(seed) for seed in seeds)))
    for field in PHYSICAL_CONSTANT_FIELDS:
        values = {record[field] for record in records}
        if len(values) > 1:
            failures.append("case %s: %s varies across the case (%s), and only the feature states may vary"
                            % (case_id, field, ", ".join(sorted(repr(value) for value in values))))
    return failures


def _validate_physical_orders(case_id, records):
    """Print order and removal order, each a permutation of 1..PHYSICAL_PRINTS_PER_CASE over the whole case."""
    failures = []
    expected = list(range(1, PHYSICAL_PRINTS_PER_CASE + 1))
    for field in ("run_order", "removal_order"):
        values = sorted(record[field] for record in records if record[field] is not None)
        if values != expected:
            failures.append("case %s: %s is %s, not a permutation of 1..%d"
                            % (case_id, field, values, PHYSICAL_PRINTS_PER_CASE))
    return failures


def _physical_totals(records):
    """What one treatment's repeats measured: the counts summed, the two measurements as medians.

    A domain one repeat left unmeasured makes the whole treatment's total None: three repeats minus
    the one nobody weighed is not a smaller total, it is an unknown one.
    """
    totals = {"prints": len(records)}
    for field in PHYSICAL_COUNT_FIELDS:
        values = [record[field] for record in records if record[field] is not None]
        totals[field] = sum(values) if len(values) == len(records) and records else None
    for field in PHYSICAL_MEASUREMENT_FIELDS:
        values = [record[field] for record in records if record[field] is not None]
        totals["median_" + field] = statistics.median(values) if len(values) == len(records) and records else None
    totals["completed"] = sum(1 for record in records if record["completed"] is True)
    totals["detached"] = sum(1 for record in records if record["detached_during_print"] is True)
    totals["notes"] = [(record["repeat"], record["observation_notes"]) for record in records
                       if record["observation_notes"].strip()]
    return totals


def physical_summary(rows):
    """Per case, what each of the four treatments measured, keyed by treatment.

    Every treatment is reported whether the case passed or not: an improvement is attributable to
    contacts or to tilt only where the two single-feature treatments are on the page beside the pair.
    """
    grouped = {}
    for number, row in enumerate(rows, start=1):
        record = _parse_physical_row(row, number)
        if record["treatment"] not in TREATMENTS:
            continue
        grouped.setdefault(record["case_id"], {}).setdefault(record["treatment"], []).append(record)
    return {case_id: {treatment: _physical_totals(sorted(records, key=lambda record: record["repeat"] or 0))
                      for treatment, records in treatments.items()}
            for case_id, treatments in grouped.items()}


def _physical_acceptance(case_id, seen):
    """The comparison a complete case has to clear: no failed print, and a saving that cost nothing.

    Lower is better in all four numbers. The combined treatment may not break more model parts or
    leave more underside defects than the baseline, and of its median removal time and median
    support mass at least one has to fall while neither rises.
    """
    failures = []
    totals = {}
    for treatment in TREATMENTS:
        records = [seen[(treatment, repeat)] for repeat in range(1, PHYSICAL_REPEATS + 1)]
        for record in records:
            where = "case %s %s repeat %d" % (case_id, treatment, record["repeat"])
            if record["completed"] is not True:
                failures.append("%s: the print did not complete" % where)
            if record["detached_during_print"] is not False:
                failures.append("%s: the print detached from the plate" % where)
        totals[treatment] = _physical_totals(records)

    baseline = totals[BASELINE_TREATMENT]
    combined = totals[COMBINED_TREATMENT]
    for field, label in (("broken_model_part_count", "broken model parts"),
                         ("underside_defect_count", "underside defects")):
        if baseline[field] is None or combined[field] is None:
            failures.append("case %s: %s were not counted in every repeat of both treatments" % (case_id, label))
        elif combined[field] > baseline[field]:
            failures.append("case %s: both features on left %d %s against the baseline's %d"
                            % (case_id, combined[field], label, baseline[field]))

    fell = False
    comparable = True
    for field, label in (("removal_seconds", "median removal time"), ("support_mass_g", "median support mass")):
        base = baseline["median_" + field]
        pair = combined["median_" + field]
        if base is None or pair is None:
            failures.append("case %s: %s was not measured in every repeat of both treatments" % (case_id, label))
            comparable = False
        elif pair > base:
            failures.append("case %s: %s rose from %g to %g with both features on" % (case_id, label, base, pair))
            comparable = False
        elif pair < base:
            fell = True
    if comparable and not fell:
        failures.append("case %s: neither the median removal time nor the median support mass fell, so the "
                        "pair of features bought nothing" % case_id)
    return failures


def _validate_physical_case(case, records):
    """Every failure one case's recorded prints carry against the protocol."""
    failures = []
    case_id = case["id"]
    seen = {}
    for record in records:
        where = "case %s row %d" % (case_id, record["row_number"])
        treatment = record["treatment"]
        if treatment not in TREATMENTS:
            failures.append("%s: treatment %r is not one of %s" % (where, treatment, ", ".join(TREATMENTS)))
            continue
        repeat = record["repeat"]
        if repeat is None or not 1 <= repeat <= PHYSICAL_REPEATS:
            failures.append("%s: repeat %r is outside 1..%d" % (where, record["raw"].get("repeat"), PHYSICAL_REPEATS))
            continue
        if (treatment, repeat) in seen:
            failures.append("case %s %s repeat %d is recorded more than once" % (case_id, treatment, repeat))
            continue
        seen[(treatment, repeat)] = record
        _validate_physical_row(case, record, failures)

    for treatment in TREATMENTS:
        for repeat in range(1, PHYSICAL_REPEATS + 1):
            if (treatment, repeat) not in seen:
                failures.append("case %s %s repeat %d was never printed or never recorded"
                                % (case_id, treatment, repeat))

    accepted = [seen[key] for key in sorted(seen)]
    failures.extend(_validate_physical_constants(case_id, accepted))
    # An incomplete case has already said so once per missing print; its orders cannot be a
    # permutation of PHYSICAL_PRINTS_PER_CASE and its treatments cannot be compared.
    if len(seen) == PHYSICAL_PRINTS_PER_CASE:
        failures.extend(_validate_physical_orders(case_id, accepted))
        failures.extend(_physical_acceptance(case_id, seen))
    return failures


def validate_physical(manifest, rows):
    """Every failure the recorded prints carry against the four-way protocol.

    An empty list means the record is complete and the combined treatment cleared the baseline. It
    does not mean the validator saw a print: every number here was typed by an operator.
    """
    failures = []
    cases = {case["id"]: case for case in manifest.get("cases", []) if isinstance(case, dict) and "id" in case}
    required = sorted(case_id for case_id, case in cases.items() if case.get("physical_required") is True)
    if not required:
        failures.append("no manifest case declares physical_required, so these prints answer for nothing")
    if not rows:
        failures.append("the record holds no prints")
        return failures

    by_case = {}
    for number, row in enumerate(rows, start=1):
        record = _parse_physical_row(row, number)
        case = cases.get(record["case_id"])
        if case is None:
            failures.append("row %d: names case %r, which the manifest does not list" % (number, record["case_id"]))
            continue
        if case.get("physical_required") is not True:
            failures.append("row %d: case %s does not declare physical_required" % (number, record["case_id"]))
            continue
        by_case.setdefault(record["case_id"], []).append(record)

    for case_id in required:
        records = by_case.get(case_id)
        if not records:
            failures.append("case %s declares physical_required and recorded no prints" % case_id)
            continue
        failures.extend(_validate_physical_case(cases[case_id], records))
    return failures


def main(argv):
    parser = argparse.ArgumentParser(prog="validate_miniature_supports.py", description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("self-test", help="run this script's own schema examples")
    automated = sub.add_parser("automated", help="validate a manifest and the rows a run produced")
    automated.add_argument("--manifest", required=True)
    automated.add_argument("--results", required=True, action="append")
    runner = sub.add_parser("run", help="run the corpus harnesses and validate what they produced")
    runner.add_argument("--manifest", required=True)
    runner.add_argument("--test-binary", required=True)
    runner.add_argument("--output-dir", required=True)
    physical = sub.add_parser("physical", help="validate the recorded four-way print experiment")
    physical.add_argument("--manifest", required=True)
    physical.add_argument("--measurements", required=True, help="the CSV an operator recorded, one row per print")
    args = parser.parse_args(argv)

    if args.command == "self-test":
        return run_self_test()
    if args.command == "automated":
        return cmd_automated(args.manifest, args.results)
    if args.command == "physical":
        return cmd_physical(args.manifest, args.measurements)
    return cmd_run(args.manifest, args.test_binary, args.output_dir)


def cmd_run(manifest_path, test_binary, output_dir):
    """Validate the manifest, run the two hidden corpus cases, then validate what they produced."""
    manifest, failures = load_manifest(manifest_path)
    if failures:
        return _report(failures)
    manifest_dir = os.path.dirname(os.path.abspath(manifest_path))
    failures = validate_manifest(manifest)
    failures.extend(validate_corpus(manifest, manifest_dir))
    if not os.path.isfile(test_binary):
        failures.append("test binary %s does not exist" % test_binary)
    if failures:
        # Nothing is run until the manifest and every model hash it declares check out.
        return _report(failures)

    os.makedirs(output_dir, exist_ok=True)
    # The output directory holds generated measurements, never sources: it ignores itself so a run
    # inside a working tree leaves nothing to commit.
    with open(os.path.join(output_dir, ".gitignore"), "w", encoding="utf-8") as handle:
        handle.write("*\n")

    model_root = os.path.join(manifest_dir, manifest.get("model_root", "."))
    result_paths = {
        ENV_MINIATURE_RESULTS: os.path.join(output_dir, "miniature_contacts.jsonl"),
        ENV_AUTOTILT_RESULTS: os.path.join(output_dir, "auto_tilt.jsonl"),
    }
    env = dict(os.environ)
    env[ENV_MANIFEST] = os.path.abspath(manifest_path)
    env[ENV_MINIATURE_CORPUS] = model_root
    env[ENV_AUTOTILT_CORPUS] = model_root
    env.update(result_paths)
    # The exhaustive mode visits the whole grid: no coarse-grid override reaches the binary, not even
    # one the caller's own shell had set.
    env.pop("ORCA_AUTOTILT_COARSE", None)

    # Catch2 never discovers a hidden case, so both are requested by name, as one filter argument.
    completed = subprocess.run([test_binary, ",".join(CASE_NAMES)], env=env)
    if completed.returncode != 0:
        failures.append("%s exited %d" % (test_binary, completed.returncode))

    rows, load_failures = load_rows(sorted(result_paths.values()))
    failures.extend(load_failures)
    if not load_failures:
        failures.extend(validate_results(manifest, rows))
        failures.extend(validate_demonstrations(manifest, rows))
    return _report(failures)


def _report(failures):
    for failure in failures:
        print("FAIL " + failure, file=sys.stderr)
    return 1 if failures else 0


def load_manifest(path):
    """The manifest document at `path`, and the failures reading it produced."""
    if not os.path.isfile(path):
        return None, ["manifest %s does not exist" % path]
    try:
        with open(path, "r", encoding="utf-8") as handle:
            return json.load(handle), []
    except ValueError as error:
        return None, ["manifest %s does not parse: %s" % (path, error)]


def cmd_automated(manifest_path, result_paths):
    manifest, failures = load_manifest(manifest_path)
    if failures:
        return _report(failures)
    failures = validate_manifest(manifest)
    failures.extend(validate_corpus(manifest, os.path.dirname(os.path.abspath(manifest_path))))
    if failures:
        return _report(failures)
    rows, load_failures = load_rows(result_paths)
    failures.extend(load_failures)
    if not failures:
        failures.extend(validate_results(manifest, rows))
        failures.extend(validate_demonstrations(manifest, rows))
    return _report(failures)


def _physical_number(value, unit=""):
    return "not measured" if value is None else "%g%s" % (value, unit)


def print_physical_summary(summary, stream=None):
    """Every treatment of every case, printed whether the case passed or not."""
    stream = stream or sys.stdout
    for case_id in sorted(summary):
        print("case %s" % case_id, file=stream)
        for treatment in TREATMENTS:
            totals = summary[case_id].get(treatment)
            if totals is None:
                print("  %-21s no prints recorded" % treatment, file=stream)
                continue
            print("  %-21s completed %d/%d  detached %d  broken %s  underside %s  median removal %s  median mass %s"
                  % (treatment, totals["completed"], PHYSICAL_REPEATS, totals["detached"],
                     _physical_number(totals["broken_model_part_count"]),
                     _physical_number(totals["underside_defect_count"]),
                     _physical_number(totals["median_removal_seconds"], " s"),
                     _physical_number(totals["median_support_mass_g"], " g")), file=stream)
            for repeat, note in totals["notes"]:
                # Notes are carried through as written. Nothing here opened a photo or followed a link.
                print("  %-21s repeat %s note (recorded, not inspected): %s" % ("", repeat, note), file=stream)


def cmd_physical(manifest_path, measurements_path):
    """Validate the manifest, then the recorded prints against it.

    No model file is read: a physical row is matched to the hashes its manifest case declares, so the
    record can be validated away from the machine that printed it.
    """
    manifest, failures = load_manifest(manifest_path)
    if failures:
        return _report(failures)
    failures = validate_manifest(manifest)
    if failures:
        return _report(failures)
    rows, load_failures = load_measurements(measurements_path)
    failures.extend(load_failures)
    if not load_failures:
        failures.extend(validate_physical(manifest, rows))
    print_physical_summary(physical_summary(rows))
    return _report(failures)


def run_self_test():
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import test_validate_miniature_supports as tests
    suite = unittest.TestLoader().loadTestsFromModule(tests)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
