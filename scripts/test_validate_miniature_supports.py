"""Self-test for validate_miniature_supports.py: finite valid and invalid schema examples."""

import contextlib
import csv
import hashlib
import io
import json
import os
import tempfile
import unittest

from validate_miniature_supports import (
    ACCEPTANCE_REPEATS,
    BASELINE_TREATMENT,
    CASE_NAMES,
    COMBINED_TREATMENT,
    ENV_AUTOTILT_CORPUS,
    ENV_AUTOTILT_RESULTS,
    ENV_MANIFEST,
    ENV_MINIATURE_CORPUS,
    ENV_MINIATURE_RESULTS,
    EXHAUSTIVE_GRID_ENTRIES,
    MANIFEST_VERSION,
    MAX_REGRET,
    PHYSICAL_FIELDS,
    PHYSICAL_PRINTS_PER_CASE,
    PHYSICAL_REPEATS,
    REQUIRED_CATEGORIES,
    TREATMENTS,
    cmd_physical,
    cmd_run,
    load_measurements,
    load_rows,
    main,
    physical_summary,
    sha256_of_file,
    validate_corpus,
    validate_demonstrations,
    validate_manifest,
    validate_physical,
    validate_results,
)


def _case(**overrides):
    case = {
        "id": "thin_weapon_spear_slim_040",
        "model": "spear.3mf",
        "sha256": "a" * 64,
        "objects": {"instance_ids": [1]},
        "scale": 1.0,
        "config_digest": "b" * 64,
        "category": "thin_weapon",
        "styles": ["tree_slim"],
        "feature_modes": ["off", "on"],
        "nozzle_diameter_mm": 0.4,
        "extrusion_width_mm": 0.42,
        "repeats": ACCEPTANCE_REPEATS,
        "expected_outcome": "complete",
        "quality_bounds": {"max_support_volume_mm3": 500.0, "max_unknown_contacts": 0},
    }
    case.update(overrides)
    return case


def _suite_cases():
    """One case per required category, together covering every required style and nozzle."""
    styles = ["tree_slim", "tree_strong", "tree_hybrid"]
    nozzles = [(0.25, 0.25), (0.4, 0.42), (0.6, 0.62)]
    cases = []
    for index, category in enumerate(REQUIRED_CATEGORIES):
        nozzle, width = nozzles[index % len(nozzles)]
        cases.append(
            _case(
                id="%s_%d" % (category, index),
                category=category,
                styles=[styles[index % len(styles)]],
                nozzle_diameter_mm=nozzle,
                extrusion_width_mm=width,
            )
        )
    return cases


def _manifest(**overrides):
    manifest = {
        "version": MANIFEST_VERSION,
        "model_root": "models",
        "repeats": ACCEPTANCE_REPEATS,
        "improvement_gain": {"support_volume_mm3": 0.5},
        "cases": _suite_cases(),
    }
    manifest.update(overrides)
    return manifest


class ManifestSchemaTest(unittest.TestCase):
    def test_a_complete_suite_passes(self):
        self.assertEqual(validate_manifest(_manifest()), [])

    def test_a_foreign_version_fails(self):
        self.assertTrue(validate_manifest(_manifest(version=2)))

    def test_an_empty_case_array_fails(self):
        self.assertTrue(validate_manifest(_manifest(cases=[])))

    def test_a_missing_case_array_fails(self):
        manifest = _manifest()
        del manifest["cases"]
        self.assertTrue(validate_manifest(manifest))

    def test_duplicate_case_ids_fail(self):
        cases = _suite_cases()
        cases[1]["id"] = cases[0]["id"]
        self.assertTrue(validate_manifest(_manifest(cases=cases)))

    def test_a_short_or_uppercase_sha256_fails(self):
        cases = _suite_cases()
        cases[0]["sha256"] = "A" * 64
        self.assertTrue(validate_manifest(_manifest(cases=cases)))
        cases[0]["sha256"] = "a" * 63
        self.assertTrue(validate_manifest(_manifest(cases=cases)))

    def test_an_absolute_or_escaping_model_path_fails(self):
        cases = _suite_cases()
        cases[0]["model"] = "/tmp/spear.3mf"
        self.assertTrue(validate_manifest(_manifest(cases=cases)))
        cases[0]["model"] = "../spear.3mf"
        self.assertTrue(validate_manifest(_manifest(cases=cases)))

    def test_an_empty_object_selection_fails(self):
        cases = _suite_cases()
        cases[0]["objects"] = {"instance_ids": []}
        self.assertTrue(validate_manifest(_manifest(cases=cases)))
        cases[0]["objects"] = {"selectors": []}
        self.assertTrue(validate_manifest(_manifest(cases=cases)))

    def test_explicit_object_selectors_are_an_accepted_selection(self):
        cases = _suite_cases()
        cases[0]["objects"] = {"selectors": ["name:Spear"]}
        self.assertEqual(validate_manifest(_manifest(cases=cases)), [])

    def test_a_nonpositive_scale_fails(self):
        cases = _suite_cases()
        cases[0]["scale"] = 0.0
        self.assertTrue(validate_manifest(_manifest(cases=cases)))

    def test_a_missing_config_digest_fails(self):
        cases = _suite_cases()
        del cases[0]["config_digest"]
        self.assertTrue(validate_manifest(_manifest(cases=cases)))

    def test_a_missing_required_category_fails(self):
        cases = [case for case in _suite_cases() if case["category"] != "constrained_routing"]
        self.assertTrue(validate_manifest(_manifest(cases=cases)))

    def test_a_suite_missing_a_required_style_fails(self):
        cases = _suite_cases()
        for case in cases:
            case["styles"] = ["tree_slim"]
        self.assertTrue(validate_manifest(_manifest(cases=cases)))

    def test_an_unknown_style_fails(self):
        cases = _suite_cases()
        cases[0]["styles"] = ["tree_bramble"]
        self.assertTrue(validate_manifest(_manifest(cases=cases)))

    def test_an_organic_case_names_organic_and_is_still_estimate_only(self):
        # The style string the harness hands to set_deserialize_strict, so it has to be the value
        # s_keys_map_SupportMaterialStyle carries: a spelling the enum does not know throws there and
        # lands every Organic row as Unknown, which no expected outcome accepts.
        cases = _suite_cases()
        cases.append(_case(id="organic_estimate_case", styles=["organic"], expected_outcome="organic_estimate"))
        self.assertEqual(validate_manifest(_manifest(cases=cases)), [])

    def test_a_suite_missing_a_required_nozzle_fails(self):
        cases = _suite_cases()
        for case in cases:
            case["nozzle_diameter_mm"] = 0.4
            case["extrusion_width_mm"] = 0.42
        self.assertTrue(validate_manifest(_manifest(cases=cases)))

    def test_a_nozzle_case_without_an_explicit_width_fails(self):
        cases = _suite_cases()
        del cases[0]["extrusion_width_mm"]
        self.assertTrue(validate_manifest(_manifest(cases=cases)))

    def test_an_empty_feature_mode_or_style_list_fails(self):
        cases = _suite_cases()
        cases[0]["feature_modes"] = []
        self.assertTrue(validate_manifest(_manifest(cases=cases)))
        cases = _suite_cases()
        cases[0]["styles"] = []
        self.assertTrue(validate_manifest(_manifest(cases=cases)))

    def test_fewer_repeats_than_the_acceptance_run_demands_fails(self):
        cases = _suite_cases()
        cases[0]["repeats"] = ACCEPTANCE_REPEATS - 1
        self.assertTrue(validate_manifest(_manifest(cases=cases)))

    def test_an_undocumented_config_override_fails(self):
        cases = _suite_cases()
        cases[0]["config_overrides"] = {"support_style": "tree_strong"}
        self.assertTrue(validate_manifest(_manifest(cases=cases)))

    def test_a_documented_config_override_passes(self):
        cases = _suite_cases()
        cases[0]["config_overrides"] = {"support_style": "tree_strong"}
        cases[0]["override_reasons"] = {"support_style": "the file was saved under an unsupported style"}
        self.assertEqual(validate_manifest(_manifest(cases=cases)), [])


def _metrics(**overrides):
    metrics = {
        "support_volume_mm3": 100.0,
        "raft_volume_mm3": 0.0,
        "missing_critical_anchors": 0,
        "invalid_paths": 0,
        "unrooted_groups": 0,
        "unknown_contacts": 0,
        "inaccessible_groups": 0,
        "max_group_risk": 1.0,
        "total_group_risk": 2.0,
        "min_bed_margin": 0.5,
        "max_slenderness": 3.0,
        "coverage_available": True,
        "stability_available": True,
        "damage_available": True,
    }
    metrics.update(overrides)
    return metrics


def _one_case_manifest(**overrides):
    return {
        "version": MANIFEST_VERSION,
        "model_root": "models",
        "repeats": ACCEPTANCE_REPEATS,
        "improvement_gain": {"support_volume_mm3": 0.5},
        "cases": [_case(**overrides)],
    }


def _pose_rows(case, **overrides):
    rows = []
    for style in case["styles"]:
        for mode in case["feature_modes"]:
            for repeat in range(case["repeats"]):
                row = {
                    "row_type": "pose",
                    "harness": "miniature_contacts",
                    "case_id": case["id"],
                    "style": style,
                    "feature_mode": mode,
                    "pose": {"tilt_deg": 0.0, "lean_deg": 0.0},
                    "repeat": repeat,
                    "status": "complete",
                    "measured": True,
                    "printable": True,
                    "plate_contained": True,
                    "reason_codes": [],
                    "metrics": _metrics(),
                    "elapsed_s": 1.0,
                    "peak_memory_bytes": 1024,
                    "peak_memory_available": True,
                    "source_sha256": case["sha256"],
                    "config_digest": case["config_digest"],
                    "build_revision": "deadbeef",
                }
                row.update(overrides)
                rows.append(dict(row))
    return rows


class ResultOutcomeTest(unittest.TestCase):
    def setUp(self):
        self.manifest = _one_case_manifest()
        self.case = self.manifest["cases"][0]

    def test_a_complete_result_set_passes(self):
        self.assertEqual(validate_results(self.manifest, _pose_rows(self.case)), [])

    def test_both_harnesses_measuring_the_same_case_each_keep_their_own_repeats(self):
        # Both harnesses iterate the same manifest. The contact harness writes its repeats at the
        # default pose, and the auto-tilt sweep writes its own repeats at grid entry 0, which is that
        # same default pose. A run that merged the two would read repeats [0, 0, 1, 1, ...] and
        # reject its own correct output.
        contacts = _pose_rows(self.case)
        tilt_root = _pose_rows(self.case, harness="auto_tilt")
        tilt_leaned = _pose_rows(self.case, harness="auto_tilt", pose={"tilt_deg": 6.0, "lean_deg": -3.0})
        self.assertEqual(validate_results(self.manifest, contacts + tilt_root + tilt_leaned), [])
        # A repeat one harness never wrote is still missing, whatever the other harness produced.
        self.assertTrue(validate_results(self.manifest, contacts + tilt_root[:-1]))

    def test_no_rows_at_all_fails(self):
        self.assertTrue(validate_results(self.manifest, []))

    def test_a_row_naming_an_unknown_case_fails(self):
        rows = _pose_rows(self.case)
        rows[0]["case_id"] = "no_such_case"
        self.assertTrue(validate_results(self.manifest, rows))

    def test_a_duplicate_repeat_fails(self):
        rows = _pose_rows(self.case)
        rows.append(dict(rows[0]))
        self.assertTrue(validate_results(self.manifest, rows))

    def test_a_missing_repeat_fails(self):
        rows = _pose_rows(self.case)[:-1]
        self.assertTrue(validate_results(self.manifest, rows))

    def test_a_declared_style_that_produced_nothing_fails(self):
        case = _case(styles=["tree_slim", "tree_strong"])
        manifest = _one_case_manifest(styles=["tree_slim", "tree_strong"])
        rows = [row for row in _pose_rows(case) if row["style"] == "tree_slim"]
        self.assertTrue(validate_results(manifest, rows))

    def test_a_declared_feature_mode_that_produced_nothing_fails(self):
        rows = [row for row in _pose_rows(self.case) if row["feature_mode"] == "off"]
        self.assertTrue(validate_results(self.manifest, rows))

    def test_a_nonfinite_available_metric_fails(self):
        rows = _pose_rows(self.case)
        rows[0]["metrics"] = _metrics(total_group_risk=float("nan"))
        self.assertTrue(validate_results(self.manifest, rows))
        rows[0]["metrics"] = _metrics(support_volume_mm3=float("inf"))
        self.assertTrue(validate_results(self.manifest, rows))

    def test_a_skipped_measurement_fails(self):
        rows = _pose_rows(self.case)
        rows[0]["measured"] = False
        self.assertTrue(validate_results(self.manifest, rows))

    def test_a_row_whose_hashes_do_not_match_its_case_fails(self):
        rows = _pose_rows(self.case)
        rows[0]["source_sha256"] = "c" * 64
        self.assertTrue(validate_results(self.manifest, rows))
        rows = _pose_rows(self.case)
        rows[0]["config_digest"] = "c" * 64
        self.assertTrue(validate_results(self.manifest, rows))

    def test_an_unavailable_peak_memory_needs_the_explicit_marker(self):
        rows = _pose_rows(self.case)
        rows[0]["peak_memory_bytes"] = None
        self.assertTrue(validate_results(self.manifest, rows))
        rows = _pose_rows(self.case, peak_memory_bytes=None, peak_memory_available=False)
        self.assertEqual(validate_results(self.manifest, rows), [])

    def test_a_printable_complete_row_with_an_invalid_path_fails(self):
        rows = _pose_rows(self.case)
        rows[0]["metrics"] = _metrics(invalid_paths=1)
        self.assertTrue(validate_results(self.manifest, rows))

    def test_a_printable_complete_row_with_an_unavailable_metric_fails(self):
        rows = _pose_rows(self.case)
        rows[0]["metrics"] = _metrics(damage_available=False)
        self.assertTrue(validate_results(self.manifest, rows))

    def test_an_outcome_declared_per_mode_holds_each_mode_to_its_own(self):
        # A case may declare a different outcome per feature mode: the measurement completes with the
        # feature off and stops short of a resolved coverage with it on.
        manifest = _one_case_manifest(
            expected_outcome="complete",
            expected_outcome_by_mode={"off": "complete", "on": "unresolved_coverage"},
            expected_reason_by_mode={"on": "MissingAnchor"},
        )
        case = manifest["cases"][0]
        rows = [row for row in _pose_rows(case) if row["feature_mode"] == "off"]
        for row in _pose_rows(case):
            if row["feature_mode"] != "on":
                continue
            row["status"] = "unresolved_coverage"
            row["reason_codes"] = ["MissingAnchor"]
            rows.append(row)
        self.assertEqual(validate_results(manifest, rows), [])

        # The same rows against a case declaring one outcome for both modes do not pass.
        self.assertTrue(validate_results(_one_case_manifest(), rows))

    def test_a_row_whose_status_is_not_the_declared_outcome_fails(self):
        rows = _pose_rows(self.case, status="unresolved_coverage", reason_codes=["MissingAnchor"])
        self.assertTrue(validate_results(self.manifest, rows))

    def test_an_unresolved_coverage_case_passes_only_when_every_repeat_is_unresolved(self):
        manifest = _one_case_manifest(expected_outcome="unresolved_coverage", expected_reason="MissingAnchor")
        case = manifest["cases"][0]
        rows = _pose_rows(
            case,
            status="unresolved_coverage",
            reason_codes=["MissingAnchor"],
            metrics=_metrics(missing_critical_anchors=1),
        )
        self.assertEqual(validate_results(manifest, rows), [])
        mixed = list(rows)
        mixed[0] = dict(mixed[0])
        mixed[0]["status"] = "complete"
        self.assertTrue(validate_results(manifest, mixed))

    def test_an_unresolved_coverage_case_never_enters_quality_ranking(self):
        manifest = _one_case_manifest(
            expected_outcome="unresolved_coverage",
            expected_reason="MissingAnchor",
            quality_bounds={"max_support_volume_mm3": 1.0},
        )
        case = manifest["cases"][0]
        rows = _pose_rows(
            case,
            status="unresolved_coverage",
            reason_codes=["MissingAnchor"],
            metrics=_metrics(support_volume_mm3=999.0, missing_critical_anchors=1),
        )
        self.assertEqual(validate_results(manifest, rows), [])

    def test_a_quality_bound_a_ranked_case_exceeds_fails(self):
        manifest = _one_case_manifest(quality_bounds={"max_support_volume_mm3": 1.0})
        rows = _pose_rows(manifest["cases"][0])
        self.assertTrue(validate_results(manifest, rows))

    def test_an_organic_row_is_estimate_only_and_never_complete(self):
        manifest = _one_case_manifest(expected_outcome="organic_estimate")
        case = manifest["cases"][0]
        rows = _pose_rows(case, status="organic_estimate", estimate_only=True, verified=False)
        self.assertEqual(validate_results(manifest, rows), [])
        claimed = _pose_rows(case, status="organic_estimate", estimate_only=False, verified=True)
        self.assertTrue(validate_results(manifest, claimed))

    def test_an_unexpected_unknown_status_fails(self):
        rows = _pose_rows(self.case, status="unknown")
        self.assertTrue(validate_results(self.manifest, rows))

    def test_an_unexpected_unresolved_status_fails(self):
        rows = _pose_rows(self.case, status="unresolved_coverage", reason_codes=["MissingAnchor"])
        self.assertTrue(validate_results(self.manifest, rows))

    def test_a_row_stamped_with_the_version_header_fallback_revision_fails(self):
        # libslic3r_version.h defines GIT_COMMIT_HASH "0000000" wherever the build did not stamp the
        # real commit, so a row carrying it names no build at all.
        rows = _pose_rows(self.case, build_revision="0000000")
        self.assertTrue(validate_results(self.manifest, rows))
        rows = _pose_rows(self.case, build_revision="")
        self.assertTrue(validate_results(self.manifest, rows))

    def test_an_invalid_pose_row_is_accepted_only_where_the_plate_cannot_hold_it(self):
        rows = _pose_rows(self.case, status="invalid", plate_contained=False, printable=False)
        self.assertEqual(validate_results(self.manifest, rows), [])
        rows = _pose_rows(self.case, status="invalid", plate_contained=True, printable=False)
        self.assertTrue(validate_results(self.manifest, rows))


def _demo_rows(case, off_volumes, on_volumes, on_metrics=None, on_status="complete", style=None, off_metrics=None):
    """One case's rows under one style: the feature off, then on, each repeat carrying its own volume."""
    rows = []
    for mode, volumes in (("off", off_volumes), ("on", on_volumes)):
        for repeat, volume in enumerate(volumes):
            metrics = _metrics(support_volume_mm3=volume)
            if mode == "on" and on_metrics:
                metrics.update(on_metrics)
            if mode == "off" and off_metrics:
                metrics.update(off_metrics)
            rows.append({
                "row_type": "pose",
                "harness": "miniature_contacts",
                "case_id": case["id"],
                "style": style or case["styles"][0],
                "feature_mode": mode,
                "pose": {"tilt_deg": 0.0, "lean_deg": 0.0},
                "repeat": repeat,
                "status": "complete" if mode == "off" else on_status,
                "measured": True,
                "printable": True,
                "plate_contained": True,
                "reason_codes": [] if mode == "off" else ["VolumeReduced"],
                "metrics": metrics,
                "elapsed_s": 1.0,
                "peak_memory_bytes": 1024,
                "peak_memory_available": True,
                "source_sha256": case["sha256"],
                "config_digest": case["config_digest"],
                "build_revision": "deadbeef",
            })
    return rows


def _selection_row(case, **overrides):
    row = {
        "row_type": "selection",
        "harness": "auto_tilt",
        "case_id": case["id"],
        "style": case["styles"][0],
        "feature_mode": case["feature_modes"][0],
        "pose": {"tilt_deg": 0.0, "lean_deg": 0.0},
        "repeat": 0,
        "status": "complete",
        "measured": True,
        "printable": True,
        "plate_contained": True,
        "reason_codes": [],
        "metrics": _metrics(),
        "elapsed_s": 1.0,
        "peak_memory_bytes": 1024,
        "peak_memory_available": True,
        "source_sha256": case["sha256"],
        "config_digest": case["config_digest"],
        "build_revision": "deadbeef",
        "selection_summary": {
            "root_pose": {"tilt_deg": 0.0, "lean_deg": 0.0},
            "selected_pose": {"tilt_deg": 0.0, "lean_deg": 0.0},
            "best_pose": {"tilt_deg": -4.0, "lean_deg": 0.0},
            "grid_entries": EXHAUSTIVE_GRID_ENTRIES,
            "evaluated_poses": EXHAUSTIVE_GRID_ENTRIES,
            "invalid_poses": 0,
            "false_move": False,
            "discrete_worse": False,
            "regret_available": True,
            "regret": 0.05,
        },
    }
    row["selection_summary"].update(overrides.pop("selection_summary", {}))
    row.update(overrides)
    return row


class SelectionRowTest(unittest.TestCase):
    def setUp(self):
        self.manifest = _one_case_manifest()
        self.case = self.manifest["cases"][0]
        self.rows = _pose_rows(self.case)

    def _run(self, **overrides):
        return validate_results(self.manifest, self.rows + [_selection_row(self.case, **overrides)])

    def test_a_selection_row_over_the_whole_grid_passes(self):
        self.assertEqual(self._run(), [])

    def test_a_sweep_that_visited_fewer_than_the_whole_grid_fails(self):
        self.assertTrue(self._run(selection_summary={"grid_entries": 9, "evaluated_poses": 9}))

    def test_a_false_move_fails(self):
        self.assertTrue(self._run(selection_summary={"false_move": True}))

    def test_regret_above_the_limit_fails_and_at_the_limit_passes(self):
        self.assertTrue(self._run(selection_summary={"regret": 0.11}))
        self.assertEqual(self._run(selection_summary={"regret": MAX_REGRET}), [])

    def test_a_regret_nobody_could_measure_fails(self):
        self.assertTrue(self._run(selection_summary={"regret_available": False, "regret": None}))

    def test_a_worse_discrete_classification_fails_whatever_the_regret(self):
        self.assertTrue(self._run(selection_summary={"discrete_worse": True, "regret": 0.0}))

    def test_the_evaluated_and_invalid_poses_have_to_add_up_to_the_grid(self):
        self.assertTrue(self._run(selection_summary={"evaluated_poses": 70, "invalid_poses": 0}))

    def test_a_printable_selection_row_that_printed_in_mid_air_fails(self):
        self.assertTrue(self._run(metrics=_metrics(invalid_paths=1)))

    def test_a_selection_row_with_no_summary_fails(self):
        row = _selection_row(self.case)
        del row["selection_summary"]
        self.assertTrue(validate_results(self.manifest, self.rows + [row]))


class DemonstrationTest(unittest.TestCase):
    def _suite(self, **case_overrides):
        redundant = _case(id="redundant", category="separate_island", demonstrates=["redundant_support_reduction"])
        neck = _case(id="neck", category="narrow_connection", demonstrates=["weaker_neck_risk"])
        thin = _case(id="thin", category="thin_weapon", demonstrates=["thin_feature_damage"])
        for case in (redundant, neck, thin):
            case.update(case_overrides)
        manifest = _manifest(cases=[redundant, neck, thin])
        manifest["improvement_gain"] = {"support_volume_mm3": 0.5, "max_group_risk": 0.1}
        return manifest

    def _rows(self, manifest, redundant_on=None, neck_risk_on=None, thin_risk_on=None):
        redundant, neck, thin = manifest["cases"]
        rows = _demo_rows(redundant, [10.0] * 7, redundant_on if redundant_on is not None else [9.0] * 7)
        rows += _demo_rows(neck, [10.0] * 7, [10.0] * 7,
                           on_metrics={"max_group_risk": neck_risk_on if neck_risk_on is not None else 0.5})
        rows += _demo_rows(thin, [10.0] * 7, [10.0] * 7,
                           on_metrics={"total_group_risk": thin_risk_on if thin_risk_on is not None else 2.0})
        return rows

    def test_a_suite_that_demonstrates_every_claim_passes(self):
        manifest = self._suite()
        self.assertEqual(validate_demonstrations(manifest, self._rows(manifest)), [])

    def test_a_suite_claiming_no_redundant_support_reduction_fails(self):
        manifest = self._suite()
        manifest["cases"][0]["demonstrates"] = []
        self.assertTrue(validate_demonstrations(manifest, self._rows(manifest)))

    def test_a_suite_whose_feature_on_rows_never_resolved_fails(self):
        manifest = self._suite()
        rows = self._rows(manifest)
        for row in rows:
            if row["feature_mode"] == "on":
                row["status"] = "unresolved_coverage"
                row["reason_codes"] = ["MissingAnchor"]
        self.assertTrue(validate_demonstrations(manifest, rows))

    def test_a_gain_below_the_configured_bound_does_not_demonstrate_a_reduction(self):
        manifest = self._suite()
        # Every feature-off reading is 9.9 and the feature-on readings span 9.4..9.8, so the two spreads
        # are disjoint: the case fails on the size of the gain (envelope_clears), not on overlap.
        rows = self._rows(manifest, redundant_on=[9.8, 9.4, 9.6, 9.5, 9.7, 9.6, 9.5])
        for row in rows:
            if row["case_id"] == "redundant" and row["feature_mode"] == "off":
                row["metrics"]["support_volume_mm3"] = 9.9
        self.assertTrue(validate_demonstrations(manifest, rows))

    def test_a_reduction_bought_with_worse_coverage_fails(self):
        manifest = self._suite()
        rows = self._rows(manifest)
        for row in rows:
            if row["case_id"] == "redundant" and row["feature_mode"] == "on":
                row["metrics"]["missing_critical_anchors"] = 1
        self.assertTrue(validate_demonstrations(manifest, rows))

    def test_a_reduction_bought_with_worse_damage_fails(self):
        manifest = self._suite()
        rows = self._rows(manifest)
        for row in rows:
            if row["case_id"] == "redundant" and row["feature_mode"] == "on":
                row["metrics"]["total_group_risk"] = 99.0
        self.assertTrue(validate_demonstrations(manifest, rows))

    def test_a_thin_feature_case_whose_damage_rose_fails(self):
        manifest = self._suite()
        self.assertTrue(validate_demonstrations(manifest, self._rows(manifest, thin_risk_on=9.0)))

    def test_a_weaker_neck_case_that_improved_no_risk_fails(self):
        manifest = self._suite()
        self.assertTrue(validate_demonstrations(manifest, self._rows(manifest, neck_risk_on=1.0)))

    def test_each_style_is_read_against_its_own_repeats(self):
        # Two styles of one case, each clearing the 0.5 mm3 gain inside its own spread: Slim lays
        # about ten times the material Strong does, so the two ranges are disjoint and an envelope
        # pooling them spans 9 to 100, which clears nothing.
        manifest = self._suite()
        redundant, neck, thin = manifest["cases"]
        redundant["styles"] = ["tree_slim", "tree_strong"]
        rows = _demo_rows(redundant, [100.0] * 7, [99.0] * 7, style="tree_slim")
        rows += _demo_rows(redundant, [10.0] * 7, [9.0] * 7, style="tree_strong")
        rows += _demo_rows(neck, [10.0] * 7, [10.0] * 7, on_metrics={"max_group_risk": 0.5})
        rows += _demo_rows(thin, [10.0] * 7, [10.0] * 7, on_metrics={"total_group_risk": 2.0})
        self.assertEqual(validate_demonstrations(manifest, rows), [])

    def test_a_style_whose_damage_rose_is_not_hidden_by_another_style(self):
        # Strong's risk rises from 10 to 95 with the feature on while Slim's falls. The worst reading either mode
        # produced across both styles hides that: 95 under Strong stays below Slim's 100 with the
        # feature off, so a pooled comparison passes a case that got worse under one style.
        manifest = self._suite()
        redundant, neck, thin = manifest["cases"]
        thin["styles"] = ["tree_slim", "tree_strong"]
        rows = self._rows(manifest)
        rows = [row for row in rows if row["case_id"] != thin["id"]]
        rows += _demo_rows(thin, [10.0] * 7, [10.0] * 7, style="tree_slim",
                           off_metrics={"total_group_risk": 100.0}, on_metrics={"total_group_risk": 90.0})
        rows += _demo_rows(thin, [10.0] * 7, [10.0] * 7, style="tree_strong",
                           off_metrics={"total_group_risk": 10.0}, on_metrics={"total_group_risk": 95.0})
        failures = validate_demonstrations(manifest, rows)
        self.assertTrue(any("tree_strong" in failure and "total_group_risk" in failure for failure in failures),
                        failures)


FAKE_BINARY = """#!/usr/bin/env python3
import json, os, sys
record = {"argv": sys.argv[1:], "env": dict(os.environ)}
with open(os.environ["FAKE_RECORD"], "w", encoding="utf-8") as handle:
    json.dump(record, handle)
source = os.environ.get("FAKE_ROWS")
rows = []
if source and os.path.isfile(source):
    rows = [json.loads(line) for line in open(source, encoding="utf-8") if line.strip()]
for variable, harness in (("ORCA_MINIATURE_RESULTS", "miniature_contacts"), ("ORCA_AUTOTILT_RESULTS", "auto_tilt")):
    target = os.environ.get(variable)
    if not target:
        continue
    with open(target, "w", encoding="utf-8") as handle:
        for row in rows:
            if row.get("harness") == harness:
                handle.write(json.dumps(row) + chr(10))
sys.exit(int(os.environ.get("FAKE_EXIT", "0")))
"""


class RunCommandTest(unittest.TestCase):
    def _corpus(self, root):
        """A manifest on disk whose models exist and hash as declared, with rows that accept."""
        models = os.path.join(root, "models")
        os.mkdir(models)
        for name in ("redundant.stl", "neck.stl", "thin.stl"):
            with open(os.path.join(models, name), "wb") as handle:
                handle.write(name.encode())
        cases = []
        plan = [
            ("redundant", "separate_island", "tree_slim", 0.4, 0.42, ["redundant_support_reduction"]),
            ("neck", "narrow_connection", "tree_strong", 0.25, 0.25, ["weaker_neck_risk"]),
            ("thin", "thin_weapon", "tree_hybrid", 0.6, 0.62, ["thin_feature_damage"]),
            ("cloth", "cloth_edge", "tree_slim", 0.4, 0.42, []),
            ("stepped", "stepped_island", "tree_slim", 0.4, 0.42, []),
            ("routed", "constrained_routing", "tree_slim", 0.4, 0.42, []),
        ]
        for name, category, style, nozzle, width, demos in plan:
            model = "%s.stl" % (name if name in ("redundant", "neck", "thin") else "thin")
            cases.append(_case(
                id=name, model=model, category=category, styles=[style],
                nozzle_diameter_mm=nozzle, extrusion_width_mm=width, demonstrates=demos,
                sha256=sha256_of_file(os.path.join(models, model)),
            ))
        manifest = _manifest(cases=cases)
        manifest["improvement_gain"] = {"support_volume_mm3": 0.5, "max_group_risk": 0.1}
        path = os.path.join(root, "manifest.json")
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(manifest, handle)

        rows = []
        for case in cases:
            on_metrics = {}
            if "weaker_neck_risk" in (case.get("demonstrates") or []):
                on_metrics["max_group_risk"] = 0.5
            on_volumes = [9.0] * 7 if "redundant_support_reduction" in (case.get("demonstrates") or []) else [10.0] * 7
            rows += _demo_rows(case, [10.0] * 7, on_volumes, on_metrics=on_metrics)
            rows.append(_selection_row(case))
        rows_path = os.path.join(root, "rows.jsonl")
        with open(rows_path, "w", encoding="utf-8") as handle:
            for row in rows:
                handle.write(json.dumps(row) + "\n")
        return path, rows_path

    def _binary(self, root):
        path = os.path.join(root, "fake_tests.py")
        with open(path, "w", encoding="utf-8") as handle:
            handle.write(FAKE_BINARY)
        os.chmod(path, 0o755)
        return path

    def _invoke(self, root, exit_code="0", rows_path=None):
        manifest_path, rows = self._corpus(root)
        binary = self._binary(root)
        record = os.path.join(root, "record.json")
        out_dir = os.path.join(root, "out")
        os.environ["FAKE_RECORD"] = record
        os.environ["FAKE_ROWS"] = rows_path if rows_path is not None else rows
        os.environ["FAKE_EXIT"] = exit_code
        os.environ["ORCA_AUTOTILT_COARSE"] = "1"
        try:
            code = cmd_run(manifest_path, binary, out_dir)
        finally:
            for key in ("FAKE_RECORD", "FAKE_ROWS", "FAKE_EXIT", "ORCA_AUTOTILT_COARSE"):
                os.environ.pop(key, None)
        return code, record, out_dir, manifest_path

    def test_a_complete_run_accepts_and_leaves_an_ignored_output_directory(self):
        with tempfile.TemporaryDirectory() as root:
            code, record, out_dir, manifest_path = self._invoke(root)
            self.assertEqual(code, 0)
            self.assertTrue(os.path.isdir(out_dir))
            with open(os.path.join(out_dir, ".gitignore"), "r", encoding="utf-8") as handle:
                self.assertIn("*", handle.read())

            with open(record, "r", encoding="utf-8") as handle:
                invocation = json.load(handle)
            # The two hidden cases are requested by name, as one Catch2 filter argument.
            self.assertEqual(invocation["argv"], [",".join(CASE_NAMES)])
            env = invocation["env"]
            self.assertEqual(env[ENV_MANIFEST], manifest_path)
            self.assertTrue(os.path.isdir(env[ENV_MINIATURE_CORPUS]))
            self.assertTrue(os.path.isdir(env[ENV_AUTOTILT_CORPUS]))
            self.assertTrue(env[ENV_MINIATURE_RESULTS].startswith(out_dir))
            self.assertTrue(env[ENV_AUTOTILT_RESULTS].startswith(out_dir))
            # No coarse-grid override reaches the binary, even when the caller's own shell set one.
            self.assertNotIn("ORCA_AUTOTILT_COARSE", env)

    def test_a_bad_hash_fails_before_the_binary_is_invoked(self):
        with tempfile.TemporaryDirectory() as root:
            manifest_path, rows = self._corpus(root)
            with open(manifest_path, "r", encoding="utf-8") as handle:
                manifest = json.load(handle)
            manifest["cases"][0]["sha256"] = "f" * 64
            with open(manifest_path, "w", encoding="utf-8") as handle:
                json.dump(manifest, handle)
            binary = self._binary(root)
            record = os.path.join(root, "record.json")
            os.environ["FAKE_RECORD"] = record
            try:
                code = cmd_run(manifest_path, binary, os.path.join(root, "out"))
            finally:
                os.environ.pop("FAKE_RECORD", None)
            self.assertEqual(code, 1)
            self.assertFalse(os.path.exists(record))

    def test_a_failing_process_propagates(self):
        with tempfile.TemporaryDirectory() as root:
            code, _, _, _ = self._invoke(root, exit_code="3")
            self.assertEqual(code, 1)

    def test_rows_that_do_not_satisfy_the_manifest_fail_the_run(self):
        with tempfile.TemporaryDirectory() as root:
            empty = os.path.join(root, "empty.jsonl")
            open(empty, "w", encoding="utf-8").close()
            code, _, _, _ = self._invoke(root, rows_path=empty)
            self.assertEqual(code, 1)

    def test_a_missing_test_binary_fails_without_running_anything(self):
        with tempfile.TemporaryDirectory() as root:
            manifest_path, _ = self._corpus(root)
            self.assertEqual(cmd_run(manifest_path, os.path.join(root, "absent"), os.path.join(root, "out")), 1)


class CommandLineTest(unittest.TestCase):
    def test_an_unknown_subcommand_is_a_usage_error(self):
        with self.assertRaises(SystemExit) as raised:
            main(["not-a-command"])
        self.assertEqual(raised.exception.code, 2)

    def test_a_missing_required_option_is_a_usage_error(self):
        with self.assertRaises(SystemExit) as raised:
            main(["automated", "--manifest", "x"])
        self.assertEqual(raised.exception.code, 2)

    def test_no_subcommand_at_all_is_a_usage_error(self):
        with self.assertRaises(SystemExit) as raised:
            main([])
        self.assertEqual(raised.exception.code, 2)


class CorpusFileTest(unittest.TestCase):
    def test_a_missing_model_directory_fails(self):
        manifest = _one_case_manifest()
        with tempfile.TemporaryDirectory() as root:
            self.assertTrue(validate_corpus(manifest, root))

    def test_a_missing_model_file_fails(self):
        manifest = _one_case_manifest()
        with tempfile.TemporaryDirectory() as root:
            os.mkdir(os.path.join(root, "models"))
            self.assertTrue(validate_corpus(manifest, root))

    def test_a_bad_hash_fails_and_the_declared_hash_passes(self):
        with tempfile.TemporaryDirectory() as root:
            os.mkdir(os.path.join(root, "models"))
            path = os.path.join(root, "models", "spear.3mf")
            with open(path, "wb") as handle:
                handle.write(b"not really a 3mf")
            manifest = _one_case_manifest()
            self.assertTrue(validate_corpus(manifest, root))
            manifest["cases"][0]["sha256"] = sha256_of_file(path)
            self.assertEqual(validate_corpus(manifest, root), [])


class ResultParseTest(unittest.TestCase):
    def test_a_malformed_jsonl_line_fails_the_load(self):
        with tempfile.TemporaryDirectory() as root:
            path = os.path.join(root, "rows.jsonl")
            with open(path, "w", encoding="utf-8") as handle:
                handle.write('{"case_id": "a"}\n{ not json\n')
            rows, failures = load_rows([path])
            self.assertTrue(failures)

    def test_a_missing_results_file_fails_the_load(self):
        with tempfile.TemporaryDirectory() as root:
            rows, failures = load_rows([os.path.join(root, "absent.jsonl")])
            self.assertTrue(failures)

    def test_an_empty_results_file_loads_no_rows_and_reports_it(self):
        with tempfile.TemporaryDirectory() as root:
            path = os.path.join(root, "rows.jsonl")
            open(path, "w", encoding="utf-8").close()
            rows, failures = load_rows([path])
            self.assertEqual(rows, [])
            self.assertTrue(failures)

    def test_a_row_survives_a_hostile_round_trip(self):
        # Sub-millisecond seconds, a unicode case id and a 64-bit memory reading, written the way a
        # harness writes a row and read back the way the validator reads one.
        row = {
            "case_id": "\u5c0f\u3055\u306a\u30df\u30cb \u2014 spear/\u00e9p\u00e9e",
            "elapsed_s": 0.000123456789,
            "peak_memory_bytes": 9007199254740993,
            "reason_codes": ["Missing\u00c4nchor"],
        }
        with tempfile.TemporaryDirectory() as root:
            path = os.path.join(root, "rows.jsonl")
            with open(path, "w", encoding="utf-8") as handle:
                handle.write(json.dumps(row) + "\n")
            back, failures = load_rows([path])
            self.assertEqual(failures, [])
            self.assertEqual(back, [row])


# ---------------------------------------------------------------------------------------------
# Self-test: the four-way physical print experiment. These examples verify the recorder, never the
# printer: a passing example says the record is complete and comparable, not that a print survived.
# ---------------------------------------------------------------------------------------------


def _hex(text):
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


# The first case _suite_cases() builds, and the one the physical examples print.
PHYSICAL_CASE_ID = "thin_weapon_0"

# One support mass per treatment, in grams. Only the combined treatment leaves measurably less
# behind; removal time is the same in all four, which is a decrease in one metric and a rise in
# neither.
_MASS_G = {
    "contacts_off_tilt_off": 1.20,
    "contacts_on_tilt_off": 1.15,
    "contacts_off_tilt_on": 1.14,
    "contacts_on_tilt_on": 0.90,
}


def _physical_declaration():
    """The manifest half of the experiment: the model, and the artifacts each treatment printed."""
    return {
        "physical_required": True,
        "physical": {
            "model_sha256": _hex("model"),
            "treatments": {
                treatment: {
                    "project": "prints/%s.3mf" % treatment,
                    "project_sha256": _hex("project:%s" % treatment),
                    "config": "prints/%s.ini" % treatment,
                    "config_sha256": _hex("config:%s" % treatment),
                }
                for treatment in TREATMENTS
            },
        },
    }


def _physical_manifest():
    cases = _suite_cases()
    cases[0].update(_physical_declaration())
    return _manifest(cases=cases)


def _physical_rows(case_id=PHYSICAL_CASE_ID):
    """One complete record: PHYSICAL_PRINTS_PER_CASE prints, every one finished, nothing broken, mass down."""
    rows = []
    for position, treatment in enumerate(TREATMENTS):
        for repeat in range(1, PHYSICAL_REPEATS + 1):
            index = position * PHYSICAL_REPEATS + (repeat - 1)
            rows.append({
                "case_id": case_id,
                "treatment": treatment,
                "repeat": str(repeat),
                # Two draws of the one seed, each a permutation of 1..PHYSICAL_PRINTS_PER_CASE; removal
                # order is the reverse of run order.
                "run_order": str(index + 1),
                "removal_order": str(PHYSICAL_PRINTS_PER_CASE - index),
                "order_seed": "20260908",
                "model_sha256": _hex("model"),
                "project_sha256": _hex("project:%s" % treatment),
                "config_sha256": _hex("config:%s" % treatment),
                "build_revision": "c7c24b83f3",
                "printer": "Bambu P1S",
                "nozzle_mm": "0.4",
                "filament": "PLA Basic lot 24B, dried 6 h",
                "layer_profile_digest": _hex("0.08 mm miniature"),
                "completed": "true",
                "detached_during_print": "false",
                "underside_defect_count": "0",
                "broken_model_part_count": "0",
                "damaged_feature_names": "",
                "removal_seconds": "%.1f" % (42.0 + repeat - 2),
                "support_mass_g": "%.2f" % (_MASS_G[treatment] + (repeat - 2) * 0.02),
                "operator": "chris",
                "tool": "flush cutters + tweezers",
                "observation_notes": "",
            })
    return rows


def _physical_row(rows, treatment, repeat):
    """The one row measuring that treatment and repeat, found by its own fields."""
    for row in rows:
        if row["treatment"] == treatment and row["repeat"] == str(repeat):
            return row
    raise AssertionError("no row for %s repeat %d" % (treatment, repeat))


def _write_measurements(path, rows):
    with open(path, "w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(PHYSICAL_FIELDS))
        writer.writeheader()
        writer.writerows(rows)


class PhysicalManifestTest(unittest.TestCase):
    def test_a_case_that_declares_no_physical_experiment_needs_no_artifacts(self):
        cases = _suite_cases()
        cases[0]["physical_required"] = False
        self.assertEqual(validate_manifest(_manifest(cases=cases)), [])

    def test_a_physical_case_declaring_all_four_treatments_passes(self):
        self.assertEqual(validate_manifest(_physical_manifest()), [])

    def test_a_physical_case_carrying_no_artifacts_at_all_fails(self):
        cases = _suite_cases()
        cases[0]["physical_required"] = True
        self.assertTrue(validate_manifest(_manifest(cases=cases)))

    def test_a_physical_case_missing_a_treatment_fails(self):
        manifest = _physical_manifest()
        del manifest["cases"][0]["physical"]["treatments"][COMBINED_TREATMENT]
        self.assertTrue(validate_manifest(manifest))

    def test_a_treatment_the_protocol_does_not_name_fails(self):
        manifest = _physical_manifest()
        manifest["cases"][0]["physical"]["treatments"]["contacts_on_tilt_maybe"] = {
            "project": "prints/maybe.3mf", "project_sha256": _hex("p"),
            "config": "prints/maybe.ini", "config_sha256": _hex("c"),
        }
        self.assertTrue(validate_manifest(manifest))

    def test_a_physical_case_without_the_expected_model_hash_fails(self):
        manifest = _physical_manifest()
        del manifest["cases"][0]["physical"]["model_sha256"]
        self.assertTrue(validate_manifest(manifest))

    def test_a_treatment_without_its_project_and_config_hashes_fails(self):
        for field in ("project_sha256", "config_sha256"):
            manifest = _physical_manifest()
            del manifest["cases"][0]["physical"]["treatments"][BASELINE_TREATMENT][field]
            self.assertTrue(validate_manifest(manifest), field)

    def test_an_artifact_path_that_escapes_the_manifest_fails(self):
        for path in ("/tmp/base.3mf", "../base.3mf"):
            manifest = _physical_manifest()
            manifest["cases"][0]["physical"]["treatments"][BASELINE_TREATMENT]["project"] = path
            self.assertTrue(validate_manifest(manifest), path)


class PhysicalMeasurementFileTest(unittest.TestCase):
    def test_a_missing_measurements_file_fails_the_load(self):
        with tempfile.TemporaryDirectory() as root:
            rows, failures = load_measurements(os.path.join(root, "absent.csv"))
            self.assertEqual(rows, [])
            self.assertTrue(failures)

    def test_an_empty_measurements_file_loads_no_rows_and_reports_it(self):
        with tempfile.TemporaryDirectory() as root:
            path = os.path.join(root, "prints.csv")
            open(path, "w", encoding="utf-8").close()
            rows, failures = load_measurements(path)
            self.assertEqual(rows, [])
            self.assertTrue(failures)

    def test_a_header_without_every_required_column_fails_the_load(self):
        with tempfile.TemporaryDirectory() as root:
            path = os.path.join(root, "prints.csv")
            with open(path, "w", encoding="utf-8", newline="") as handle:
                handle.write("case_id,treatment,repeat\nthin_weapon_0,contacts_off_tilt_off,1\n")
            rows, failures = load_measurements(path)
            self.assertEqual(rows, [])
            self.assertTrue(failures)

    def test_a_header_only_file_records_no_prints(self):
        with tempfile.TemporaryDirectory() as root:
            path = os.path.join(root, "prints.csv")
            _write_measurements(path, [])
            rows, failures = load_measurements(path)
            self.assertEqual(rows, [])
            self.assertTrue(failures)

    def test_a_row_survives_a_hostile_round_trip(self):
        # An operator name outside ASCII, a tool list carrying the delimiter, and a note holding a
        # quoted string and an embedded newline: written the way an operator's sheet exports and read
        # back the way the validator reads it.
        row = dict(_physical_rows()[0])
        row["operator"] = "Chloé — 小さな"
        row["tool"] = "flush cutters, 0.5 mm; tweezers"
        row["damaged_feature_names"] = 'spear tip, cloak "edge"'
        row["observation_notes"] = 'photo: https://example.invalid/p?a=1,b=2\nsecond line "quoted"'
        with tempfile.TemporaryDirectory() as root:
            path = os.path.join(root, "prints.csv")
            _write_measurements(path, [row])
            back, failures = load_measurements(path)
            self.assertEqual(failures, [])
            self.assertEqual(back, [row])


class PhysicalRecordTest(unittest.TestCase):
    def setUp(self):
        self.manifest = _physical_manifest()

    def test_a_complete_record_of_twelve_prints_passes(self):
        self.assertEqual(validate_physical(self.manifest, _physical_rows()), [])

    def test_a_manifest_that_declares_no_physical_case_fails(self):
        self.assertTrue(validate_physical(_manifest(), _physical_rows()))

    def test_a_row_naming_a_case_that_does_not_print_fails(self):
        rows = _physical_rows()
        rows[0] = dict(rows[0], case_id="cloth_edge_2")
        self.assertTrue(validate_physical(self.manifest, rows))

    def test_a_row_naming_a_case_the_manifest_never_lists_fails(self):
        rows = _physical_rows()
        rows[0] = dict(rows[0], case_id="no_such_case")
        self.assertTrue(validate_physical(self.manifest, rows))

    def test_a_missing_treatment_repeat_fails(self):
        rows = [row for row in _physical_rows()
                if not (row["treatment"] == COMBINED_TREATMENT and row["repeat"] == "2")]
        self.assertTrue(validate_physical(self.manifest, rows))

    def test_a_repeat_recorded_twice_fails(self):
        rows = _physical_rows()
        rows.append(dict(_physical_row(rows, BASELINE_TREATMENT, 1)))
        self.assertTrue(validate_physical(self.manifest, rows))

    def test_a_repeat_outside_one_to_three_fails(self):
        rows = _physical_rows()
        _physical_row(rows, BASELINE_TREATMENT, 3)["repeat"] = "4"
        self.assertTrue(validate_physical(self.manifest, rows))

    def test_a_treatment_the_protocol_does_not_name_fails(self):
        rows = _physical_rows()
        _physical_row(rows, BASELINE_TREATMENT, 1)["treatment"] = "contacts_on_tilt_maybe"
        self.assertTrue(validate_physical(self.manifest, rows))

    def test_a_run_or_removal_order_that_is_not_a_permutation_fails(self):
        for field in ("run_order", "removal_order"):
            rows = _physical_rows()
            _physical_row(rows, BASELINE_TREATMENT, 1)[field] = _physical_row(rows, BASELINE_TREATMENT, 2)[field]
            self.assertTrue(validate_physical(self.manifest, rows), field)

    def test_a_case_recording_more_than_one_seed_fails(self):
        rows = _physical_rows()
        _physical_row(rows, COMBINED_TREATMENT, 1)["order_seed"] = "20260909"
        self.assertTrue(validate_physical(self.manifest, rows))

    def test_a_seed_that_is_not_an_integer_fails(self):
        rows = _physical_rows()
        for row in rows:
            row["order_seed"] = "whatever I had open"
        self.assertTrue(validate_physical(self.manifest, rows))

    def test_a_hash_that_does_not_match_the_manifest_artifact_fails(self):
        for field in ("model_sha256", "project_sha256", "config_sha256"):
            rows = _physical_rows()
            _physical_row(rows, COMBINED_TREATMENT, 1)[field] = _hex("something else")
            self.assertTrue(validate_physical(self.manifest, rows), field)

    def test_a_treatment_printed_from_another_treatments_project_fails(self):
        rows = _physical_rows()
        row = _physical_row(rows, COMBINED_TREATMENT, 1)
        row["project_sha256"] = _hex("project:%s" % BASELINE_TREATMENT)
        self.assertTrue(validate_physical(self.manifest, rows))

    def test_a_fixed_condition_that_varied_inside_a_case_fails(self):
        for field, value in (("printer", "A1 mini"), ("nozzle_mm", "0.6"), ("filament", "PETG lot 3"),
                             ("layer_profile_digest", _hex("0.12 mm")), ("build_revision", "56e21864b6"),
                             ("operator", "someone else"), ("tool", "side cutters")):
            rows = _physical_rows()
            _physical_row(rows, COMBINED_TREATMENT, 1)[field] = value
            self.assertTrue(validate_physical(self.manifest, rows), field)

    def test_a_boolean_field_that_is_not_true_or_false_fails(self):
        for field in ("completed", "detached_during_print"):
            rows = _physical_rows()
            _physical_row(rows, BASELINE_TREATMENT, 1)[field] = "mostly"
            self.assertTrue(validate_physical(self.manifest, rows), field)

    def test_a_negative_or_unparsable_count_fails(self):
        for value in ("-1", "a few"):
            rows = _physical_rows()
            _physical_row(rows, BASELINE_TREATMENT, 1)["underside_defect_count"] = value
            self.assertTrue(validate_physical(self.manifest, rows), value)

    def test_a_negative_measurement_fails(self):
        rows = _physical_rows()
        _physical_row(rows, BASELINE_TREATMENT, 1)["removal_seconds"] = "-2.0"
        self.assertTrue(validate_physical(self.manifest, rows))

    def test_a_broken_part_with_no_named_feature_fails(self):
        rows = _physical_rows()
        row = _physical_row(rows, BASELINE_TREATMENT, 1)
        row["broken_model_part_count"] = "1"
        self.assertTrue(validate_physical(self.manifest, rows))
        row["damaged_feature_names"] = "spear tip"
        self.assertEqual(validate_physical(self.manifest, rows), [])

    def test_a_nozzle_that_is_not_a_diameter_fails(self):
        rows = _physical_rows()
        for row in rows:
            row["nozzle_mm"] = "0.4 mm"
        self.assertTrue(validate_physical(self.manifest, rows))

    def test_a_photo_link_in_the_notes_is_recorded_and_never_inspected(self):
        rows = _physical_rows()
        _physical_row(rows, COMBINED_TREATMENT, 2)["observation_notes"] = \
            "photo file:///nowhere/that/exists.jpg, https://example.invalid/never-fetched"
        self.assertEqual(validate_physical(self.manifest, rows), [])


class PhysicalAcceptanceTest(unittest.TestCase):
    def setUp(self):
        self.manifest = _physical_manifest()

    def test_a_case_that_meets_the_protocol_passes(self):
        # Twelve prints, none detached, nothing broken, and the combined treatment's median mass
        # below the baseline's while its median removal time matches.
        self.assertEqual(validate_physical(self.manifest, _physical_rows()), [])

    def test_a_print_that_detached_from_the_plate_fails(self):
        rows = _physical_rows()
        _physical_row(rows, COMBINED_TREATMENT, 3)["detached_during_print"] = "true"
        self.assertTrue(validate_physical(self.manifest, rows))

    def test_more_broken_parts_in_the_combined_treatment_than_the_baseline_fails(self):
        rows = _physical_rows()
        row = _physical_row(rows, COMBINED_TREATMENT, 1)
        row["broken_model_part_count"] = "1"
        row["damaged_feature_names"] = "spear tip"
        self.assertTrue(validate_physical(self.manifest, rows))

    def test_the_same_breakage_as_the_baseline_is_not_a_failure_of_the_comparison(self):
        rows = _physical_rows()
        for treatment in (BASELINE_TREATMENT, COMBINED_TREATMENT):
            row = _physical_row(rows, treatment, 1)
            row["broken_model_part_count"] = "1"
            row["damaged_feature_names"] = "spear tip"
        self.assertEqual(validate_physical(self.manifest, rows), [])

    def test_more_underside_defects_in_the_combined_treatment_fails(self):
        rows = _physical_rows()
        _physical_row(rows, COMBINED_TREATMENT, 2)["underside_defect_count"] = "2"
        self.assertTrue(validate_physical(self.manifest, rows))

    def test_a_case_where_neither_median_fell_fails(self):
        rows = _physical_rows()
        for repeat in range(1, PHYSICAL_REPEATS + 1):
            baseline = _physical_row(rows, BASELINE_TREATMENT, repeat)
            combined = _physical_row(rows, COMBINED_TREATMENT, repeat)
            combined["support_mass_g"] = baseline["support_mass_g"]
        self.assertTrue(validate_physical(self.manifest, rows))

    def test_a_median_that_rose_fails_even_where_the_other_fell(self):
        rows = _physical_rows()
        for repeat in range(1, PHYSICAL_REPEATS + 1):
            _physical_row(rows, COMBINED_TREATMENT, repeat)["removal_seconds"] = "90.0"
        self.assertTrue(validate_physical(self.manifest, rows))

    def test_the_median_is_read_over_the_repeats_rather_than_one_of_them(self):
        # One fast removal in the baseline does not make the baseline fast: with 1, 60 and 61
        # seconds its median is 60 and its mean is 40.7, so the combined treatment's 42 is a
        # decrease against the median and a rise against the mean.
        rows = _physical_rows()
        for repeat, seconds in ((1, "1.0"), (2, "60.0"), (3, "61.0")):
            _physical_row(rows, BASELINE_TREATMENT, repeat)["removal_seconds"] = seconds
        self.assertEqual(validate_physical(self.manifest, rows), [])

    def test_the_summary_reports_every_treatment_so_a_saving_stays_attributable(self):
        rows = _physical_rows()
        note = "photo https://example.invalid/combined-2.jpg"
        _physical_row(rows, COMBINED_TREATMENT, 2)["observation_notes"] = note
        summary = physical_summary(rows)[PHYSICAL_CASE_ID]
        self.assertEqual(sorted(summary), sorted(TREATMENTS))
        for treatment in TREATMENTS:
            self.assertEqual(summary[treatment]["prints"], PHYSICAL_REPEATS)
            self.assertEqual(summary[treatment]["completed"], PHYSICAL_REPEATS)
            self.assertEqual(summary[treatment]["detached"], 0)
            self.assertEqual(summary[treatment]["median_support_mass_g"], _MASS_G[treatment])
            self.assertEqual(summary[treatment]["median_removal_seconds"], 42.0)
        # Each of the two features is measured on its own, so a saving is not read off the pair.
        self.assertLess(summary["contacts_on_tilt_off"]["median_support_mass_g"],
                        summary[BASELINE_TREATMENT]["median_support_mass_g"])
        # The note is carried as written; nothing here fetched the link.
        self.assertEqual(summary[COMBINED_TREATMENT]["notes"], [(2, note)])


class PhysicalCommandTest(unittest.TestCase):
    """Run end to end over files on disk.

    Every example here verifies the recorder. None of them says a support came off a printed
    miniature: the numbers are typed, and a passing example means the record is complete and the
    comparison is legible.
    """

    def _invoke(self, root, rows):
        manifest_path = os.path.join(root, "manifest.json")
        with open(manifest_path, "w", encoding="utf-8") as handle:
            json.dump(_physical_manifest(), handle)
        measurements_path = os.path.join(root, "prints.csv")
        _write_measurements(measurements_path, rows)
        captured = io.StringIO()
        with contextlib.redirect_stdout(captured):
            code = cmd_physical(manifest_path, measurements_path)
        return code, captured.getvalue()

    def test_a_complete_record_exits_zero_and_reports_all_four_treatments(self):
        with tempfile.TemporaryDirectory() as root:
            code, output = self._invoke(root, _physical_rows())
            self.assertEqual(code, 0)
            for treatment in TREATMENTS:
                self.assertIn(treatment, output)

    def test_removing_one_row_exits_one(self):
        rows = [row for row in _physical_rows()
                if not (row["treatment"] == COMBINED_TREATMENT and row["repeat"] == "3")]
        with tempfile.TemporaryDirectory() as root:
            self.assertEqual(self._invoke(root, rows)[0], 1)

    def test_a_print_that_did_not_complete_exits_one(self):
        rows = _physical_rows()
        _physical_row(rows, "contacts_on_tilt_off", 2)["completed"] = "false"
        with tempfile.TemporaryDirectory() as root:
            self.assertEqual(self._invoke(root, rows)[0], 1)

    def test_a_broken_weapon_in_the_combined_treatment_exits_one(self):
        rows = _physical_rows()
        row = _physical_row(rows, COMBINED_TREATMENT, 2)
        row["broken_model_part_count"] = "1"
        row["damaged_feature_names"] = "spear haft"
        with tempfile.TemporaryDirectory() as root:
            self.assertEqual(self._invoke(root, rows)[0], 1)

    def test_a_measurement_recorded_as_nan_or_as_words_exits_one(self):
        for field, value in (("support_mass_g", "NaN"), ("support_mass_g", "not measured"),
                             ("removal_seconds", "not measured"), ("removal_seconds", "")):
            rows = _physical_rows()
            _physical_row(rows, COMBINED_TREATMENT, 1)[field] = value
            with tempfile.TemporaryDirectory() as root:
                self.assertEqual(self._invoke(root, rows)[0], 1, "%s=%r" % (field, value))

    def test_a_record_answering_no_manifest_case_exits_one(self):
        with tempfile.TemporaryDirectory() as root:
            manifest_path = os.path.join(root, "manifest.json")
            with open(manifest_path, "w", encoding="utf-8") as handle:
                json.dump(_manifest(), handle)
            measurements_path = os.path.join(root, "prints.csv")
            _write_measurements(measurements_path, _physical_rows())
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(cmd_physical(manifest_path, measurements_path), 1)

    def test_a_manifest_that_does_not_parse_exits_one(self):
        with tempfile.TemporaryDirectory() as root:
            manifest_path = os.path.join(root, "manifest.json")
            with open(manifest_path, "w", encoding="utf-8") as handle:
                handle.write("{ not json")
            measurements_path = os.path.join(root, "prints.csv")
            _write_measurements(measurements_path, _physical_rows())
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(cmd_physical(manifest_path, measurements_path), 1)

    def test_a_missing_measurements_file_exits_one(self):
        with tempfile.TemporaryDirectory() as root:
            manifest_path = os.path.join(root, "manifest.json")
            with open(manifest_path, "w", encoding="utf-8") as handle:
                json.dump(_physical_manifest(), handle)
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(cmd_physical(manifest_path, os.path.join(root, "absent.csv")), 1)

    def test_the_physical_subcommand_needs_both_paths(self):
        for argv in (["physical"], ["physical", "--manifest", "m.json"], ["physical", "--measurements", "p.csv"]):
            with self.assertRaises(SystemExit) as raised:
                main(argv)
            self.assertEqual(raised.exception.code, 2, argv)
