from __future__ import annotations

import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from contract import (  # noqa: E402
    GRID_COLS,
    GRID_ROWS,
    SCHEMA_VERSION,
    adapt_twin_snapshot,
    build_demo,
    world_to_grid,
    world_to_scene,
)
from validate_twin_vectors import load_json, semantic_errors, validate  # noqa: E402


SCHEMA = ROOT / "schemas" / "camera_aim_3d_vectors_v1.schema.json"
EXAMPLES = ROOT / "examples"


def source_person(**overrides):
    person = {
        "gid": 42,
        "name": "MUST_NOT_EXPORT",
        "identity_confidence": 0.99,
        "camera": "A",
        "cam": ["A"],
        "position_m": {"x": 1.5, "y": 3.0, "z": 0.0},
        "position_quality": "ok",
        "position_reason": "ok",
        "metric_valid": True,
        "grid_cell": {"col": 3, "row": 6, "cell_id": "C3_R6", "cell_size_m": 0.5},
        "visible": True,
        "tracking_state": "ACTIVE",
        "last_seen_ago_s": 0.2,
        "ts_capture": 1000.0,
        "inside_calibration_hull": True,
        "extrapolation_warning": False,
        "motion_available": True,
        "motion_reason": "ok",
        "vel": [0.1, 0.2],
        "speed": 0.2236,
        "heading": 1.107,
    }
    person.update(overrides)
    return person


def source_snapshot(person=None, *, room_metric_valid=True, trajectories=None):
    return {
        "online": True,
        "twin": {
            "schema_version": 2,
            "ts_capture": 1000.0,
            "generated_at": 1000.1,
            "room": {
                "width_m": 4.5,
                "height_m": 10.0,
                "ceiling_height_m": 3.0,
                "calibration_mode": "per_camera_validated",
                "metric_positions_valid": room_metric_valid,
            },
            "people": [person or source_person()],
            "trajectories": trajectories or {},
        },
    }


class RoomMathTests(unittest.TestCase):
    def test_room_grid_is_9_by_20(self):
        self.assertEqual((GRID_COLS, GRID_ROWS, GRID_COLS * GRID_ROWS), (9, 20, 180))

    def test_world_to_scene_mapping(self):
        self.assertEqual(world_to_scene(0, 0, 0), {"x": -2.25, "y": 0.0, "z": 5.0})
        self.assertEqual(world_to_scene(4.5, 10, 0), {"x": 2.25, "y": 0.0, "z": -5.0})
        self.assertEqual(world_to_scene(2.25, 5, 0), {"x": 0.0, "y": 0.0, "z": 0.0})

    def test_grid_boundaries_are_zero_based(self):
        self.assertEqual(world_to_grid(0, 0)["cell_id"], "C0_R0")
        self.assertEqual(world_to_grid(4.5, 10)["cell_id"], "C8_R19")
        self.assertEqual(world_to_grid(1.5, 3.0)["cell_id"], "C3_R6")
        self.assertIsNone(world_to_grid(-0.01, 1))
        self.assertIsNone(world_to_grid(1, 10.01))


class FixtureTests(unittest.TestCase):
    def test_all_contract_examples_validate(self):
        for name in (
            "demo_single_frame.json",
            "demo_multi_frame.json",
            "demo_invalid_and_outside_hull.json",
        ):
            with self.subTest(name=name):
                self.assertEqual(validate(EXAMPLES / name, SCHEMA), [])

    def test_demo_is_never_disguised_as_live(self):
        for kind in ("single", "multi", "invalid"):
            payload = build_demo(kind)
            self.assertEqual(payload["provenance"]["source"], "DEMO_SYNTHETIC")
            self.assertTrue(payload["provenance"]["synthetic"])
            for frame in payload["frames"]:
                self.assertTrue(all(person["id"].startswith("DEMO-") for person in frame["people"]))

    def test_multi_frame_identity_is_stable(self):
        frames = build_demo("multi")["frames"]
        self.assertEqual(len(frames), 5)
        ids_by_frame = [{person["id"] for person in frame["people"]} for frame in frames]
        self.assertTrue(all("DEMO-PERSON-ALPHA" in ids for ids in ids_by_frame))
        self.assertNotIn("DEMO-PERSON-BETA", ids_by_frame[3])
        self.assertIn("DEMO-PERSON-BETA", ids_by_frame[4])
        alpha_x = [frame["people"][0]["position_world_m"]["x"] for frame in frames]
        self.assertEqual(alpha_x, sorted(alpha_x))

    def test_invalid_fixture_never_fabricates_zero_position(self):
        people = build_demo("invalid")["frames"][0]["people"]
        by_id = {person["id"]: person for person in people}
        for key in ("DEMO-NO-FOOTPOINT", "DEMO-STALE", "DEMO-NO-WORLD-XY", "DEMO-CAMERA-B-DOOR"):
            self.assertFalse(by_id[key]["renderable"])
            self.assertIsNone(by_id[key]["position_world_m"])
            self.assertIsNone(by_id[key]["position_scene_m"])
            self.assertIsNone(by_id[key]["grid_cell"])
        outside = by_id["DEMO-OUTSIDE-HULL"]
        self.assertTrue(outside["renderable"])
        self.assertEqual(outside["trust_level"], "estimated")
        self.assertNotEqual(outside["position_world_m"], {"x": 0, "y": 0, "z": 0})


class AdapterTests(unittest.TestCase):
    def test_measured_camera_a_maps_to_scene_and_grid(self):
        exported = adapt_twin_snapshot(source_snapshot())
        person = exported["frames"][0]["people"][0]
        self.assertEqual(exported["schema_version"], SCHEMA_VERSION)
        self.assertEqual(person["id"], "G42")
        self.assertEqual(person["position_scene_m"], {"x": -0.75, "y": 0.0, "z": 2.0})
        self.assertEqual(person["grid_cell"]["cell_id"], "C3_R6")
        self.assertEqual(person["trust_level"], "trusted")
        self.assertNotIn("name", person)
        self.assertNotIn("identity_confidence", person)

    def test_camera_b_only_cannot_publish_world_xy(self):
        person = source_person(camera="B", cam=["B"])
        exported = adapt_twin_snapshot(source_snapshot(person))["frames"][0]["people"][0]
        self.assertEqual(exported["position_status"], "door_only")
        self.assertFalse(exported["renderable"])
        self.assertIsNone(exported["position_world_m"])

    def test_room_metric_invalid_strips_position(self):
        exported = adapt_twin_snapshot(source_snapshot(room_metric_valid=False))["frames"][0]["people"][0]
        self.assertEqual(exported["position_reason"], "metric_invalid")
        self.assertFalse(exported["metric_valid"])
        self.assertIsNone(exported["position_world_m"])

    def test_stale_track_strips_position(self):
        person = source_person(tracking_state="TEMP_LOST", visible=False, last_seen_ago_s=20)
        exported = adapt_twin_snapshot(source_snapshot(person))["frames"][0]["people"][0]
        self.assertEqual(exported["position_status"], "stale")
        self.assertIsNone(exported["position_world_m"])

    def test_invalid_footpoint_remains_null(self):
        person = source_person(position_m=None, grid_cell=None, position_quality="invalid", position_reason="no_footpoint", metric_valid=False)
        exported = adapt_twin_snapshot(source_snapshot(person))["frames"][0]["people"][0]
        self.assertEqual(exported["position_reason"], "no_footpoint")
        self.assertIsNone(exported["position_world_m"])

    def test_outside_hull_is_estimated_not_trusted(self):
        person = source_person(inside_calibration_hull=False, extrapolation_warning=True)
        exported = adapt_twin_snapshot(source_snapshot(person))["frames"][0]["people"][0]
        self.assertEqual(exported["position_status"], "estimated")
        self.assertEqual(exported["trust_level"], "estimated")
        self.assertTrue(exported["renderable"])

    def test_source_grid_disagreement_is_rejected(self):
        person = source_person(grid_cell={"col": 0, "row": 0, "cell_id": "C0_R0", "cell_size_m": 0.5})
        with self.assertRaisesRegex(ValueError, "disagrees"):
            adapt_twin_snapshot(source_snapshot(person))

    def test_legacy_trajectory_is_not_promoted(self):
        paths = {"G42": [[1.0, 2.0], [1.1, 2.1]]}
        person = adapt_twin_snapshot(source_snapshot(trajectories=paths))["frames"][0]["people"][0]
        self.assertEqual(person["trajectory"], [])

    def test_v1_trajectory_preserves_capture_time(self):
        paths = {"42": [
            {"x": 1.0, "y": 2.0, "capture_ts": 999.0, "metric_valid": True, "quality": "ok"},
            {"x": 1.2, "y": 2.2, "capture_ts": 1000.0, "metric_valid": True, "quality": "ok"},
        ]}
        person = adapt_twin_snapshot(source_snapshot(trajectories=paths))["frames"][0]["people"][0]
        self.assertEqual([point["capture_timestamp_s"] for point in person["trajectory"]], [999.0, 1000.0])


class ValidationSafetyTests(unittest.TestCase):
    def test_non_finite_json_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.json"
            path.write_text('{"x": NaN}', encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "non-finite"):
                load_json(path)

    def test_schema_rejects_pii_extension(self):
        payload = build_demo("single")
        payload["frames"][0]["people"][0]["employee_name"] = "PRIVATE"
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "pii.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            errors = validate(path, SCHEMA)
        self.assertTrue(any("Additional properties" in error for error in errors))

    def test_duplicate_id_in_frame_is_rejected_semantically(self):
        payload = build_demo("single")
        payload["frames"][0]["people"].append(copy.deepcopy(payload["frames"][0]["people"][0]))
        self.assertTrue(any("duplicate id" in error for error in semantic_errors(payload)))


if __name__ == "__main__":
    unittest.main()
