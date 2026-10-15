from __future__ import annotations

import json
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import contract  # noqa: E402
from calibrate_homography import build_report, synthetic_camera  # noqa: E402
import rules  # noqa: E402
from rules import CalibrationProfile, Observation, check_calibration, evaluate  # noqa: E402
from validate_twin_vectors import semantic_errors, validate  # noqa: E402
from test_contract import source_person, source_snapshot  # noqa: E402

SCHEMA = ROOT / "schemas" / "camera_aim_3d_vectors_v1.schema.json"


def demo_report(**overrides):
    spec = synthetic_camera(noise_px=0.5)
    spec.pop("project")
    spec.update(overrides)
    return build_report(spec)


def demo_profile(**overrides):
    return check_calibration(CalibrationProfile.from_report(demo_report(**overrides), "demo_camera_a"))


def obs(**overrides):
    base = dict(
        cameras=("A",), position={"x": 2.0, "y": 3.0, "z": 0.0}, quality="ok", reason="ok", metric_valid=True,
        room_metric_valid=True, visible=True, tracking_state="ACTIVE", last_seen_ago_s=0.1, capture_ts=100.0,
        frame_ts=100.0, inside_hull_flag=True, extrapolation_warning=False,
    )
    base.update(overrides)
    return Observation(**base)


class CalibrationRuleTests(unittest.TestCase):
    def test_good_undistorted_calibration_is_valid(self):
        profile = demo_profile()
        self.assertEqual(profile.status, "valid", profile.findings)

    def test_skipping_lens_undistortion_is_not_valid(self):
        profile = demo_profile(dist_coeffs=None)
        self.assertNotEqual(profile.status, "valid")
        self.assertTrue(any(f.startswith("C08_lens_undistorted") for f in profile.findings))

    def test_too_few_points_is_invalid(self):
        report = demo_report()
        report["points"] = 4
        profile = check_calibration(CalibrationProfile.from_report(report, "x"))
        self.assertEqual(profile.status, "invalid")

    def test_fingerprint_is_deterministic(self):
        self.assertEqual(demo_profile().fingerprint, demo_profile().fingerprint)
        self.assertRegex(demo_profile().fingerprint, r"^sha256:[0-9a-f]{16}$")

    def test_rule_constants_match_room_contract(self):
        self.assertEqual((rules.ROOM_WIDTH_M, rules.ROOM_LENGTH_M), (contract.ROOM_WIDTH_M, contract.ROOM_LENGTH_M))


class ObservationRuleTests(unittest.TestCase):
    def test_near_camera_is_measured(self):
        decision, trace = evaluate(obs(position={"x": 2.25, "y": 1.5, "z": 0.0}), demo_profile())
        self.assertEqual((decision.status, decision.trust, decision.rule_id), ("measured", "trusted", "R12_hull_uncertainty_trust"))
        self.assertEqual(trace[-1], decision.rule_id)

    def test_far_corner_is_not_trusted(self):
        decision, _ = evaluate(obs(position={"x": 4.4, "y": 9.9, "z": 0.0}), demo_profile())
        self.assertNotEqual(decision.trust, "trusted")

    def test_huge_uncertainty_is_rejected(self):
        tight = rules.Thresholds(max_radius_m=0.01)
        decision, _ = evaluate(obs(), demo_profile(), tight)
        self.assertEqual((decision.status, decision.rule_id), ("invalid", "R11_uncertainty_too_large"))
        self.assertFalse(decision.renderable)

    def test_invalid_calibration_blocks_all_positions(self):
        profile = demo_profile()
        profile.status = "invalid"
        decision, _ = evaluate(obs(), profile)
        self.assertEqual(decision.reason, "metric_invalid")

    def test_degraded_calibration_caps_trust(self):
        profile = demo_profile()
        profile.status = "degraded"
        decision, _ = evaluate(obs(position={"x": 2.25, "y": 1.5, "z": 0.0}), profile)
        self.assertEqual(decision.trust, "estimated")

    def test_capture_far_behind_frame_is_stale(self):
        decision, _ = evaluate(obs(capture_ts=80.0, frame_ts=100.0))
        self.assertEqual(decision.reason, "stale_track")

    def test_future_capture_timestamp_is_invalid(self):
        decision, _ = evaluate(obs(capture_ts=105.0, frame_ts=100.0))
        self.assertEqual((decision.status, decision.rule_id), ("invalid", "R03_future_timestamp"))

    def test_camera_b_wins_over_everything(self):
        decision, trace = evaluate(obs(cameras=("B",)), demo_profile())
        self.assertEqual(trace, ["R01_camera_b_door_only"])
        self.assertEqual(decision.status, "door_only")

    def test_same_input_same_output(self):
        profile = demo_profile()
        self.assertEqual(evaluate(obs(), profile), evaluate(obs(), profile))


class ExportWithCalibrationTests(unittest.TestCase):
    def test_export_records_calibration_provenance_and_validates(self):
        profile = demo_profile()
        payload = contract.adapt_twin_snapshot(source_snapshot(source_person()), profile)
        prov = payload["provenance"]
        self.assertEqual(prov["calibration_fingerprint"], profile.fingerprint)
        self.assertEqual(prov["rule_set_version"], rules.RULE_SET_VERSION)
        self.assertEqual(semantic_errors(payload), [])
        with self.subTest("schema"):
            import tempfile
            with tempfile.TemporaryDirectory() as tmp:
                path = Path(tmp) / "out.json"
                path.write_text(json.dumps(payload), encoding="utf-8")
                self.assertEqual(validate(path, SCHEMA), [])

    def test_validator_rejects_trusted_under_degraded_calibration(self):
        payload = contract.adapt_twin_snapshot(source_snapshot(source_person()))
        payload["provenance"]["calibration_status"] = "degraded"
        self.assertTrue(any("degraded" in e for e in semantic_errors(payload)))


if __name__ == "__main__":
    unittest.main()
