#!/usr/bin/env python3
"""Validate Camera AIM 3D vector JSON against schema and semantic invariants."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
from typing import Any

from jsonschema import Draft202012Validator, FormatChecker

from contract import (
    GRID_COLS,
    GRID_ROWS,
    ROOM_HEIGHT_M,
    ROOM_LENGTH_M,
    ROOM_WIDTH_M,
    SCHEMA_VERSION,
    world_to_grid,
    world_to_scene,
)


MAX_INPUT_BYTES = 20 * 1024 * 1024


def _reject_constant(value: str) -> None:
    raise ValueError(f"non-finite JSON number is not allowed: {value}")


def load_json(path: Path) -> Any:
    if path.stat().st_size > MAX_INPUT_BYTES:
        raise ValueError(f"file exceeds {MAX_INPUT_BYTES} bytes")
    return json.loads(path.read_text(encoding="utf-8"), parse_constant=_reject_constant)


def _same_vector(actual: dict[str, Any], expected: dict[str, float]) -> bool:
    return all(math.isclose(float(actual[key]), expected[key], abs_tol=1e-6) for key in ("x", "y", "z"))


def semantic_errors(payload: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    room = payload.get("room", {})
    expected_room = (ROOM_WIDTH_M, ROOM_LENGTH_M, ROOM_HEIGHT_M, GRID_COLS, GRID_ROWS, GRID_COLS * GRID_ROWS)
    actual_room = (
        room.get("width_m"), room.get("length_m"), room.get("height_m"),
        room.get("grid_cols"), room.get("grid_rows"), room.get("grid_total_cells"),
    )
    if actual_room != expected_room:
        errors.append(f"room contract mismatch: {actual_room!r} != {expected_room!r}")
    provenance = payload.get("provenance", {})
    is_demo = provenance.get("source") == "DEMO_SYNTHETIC"
    if bool(provenance.get("synthetic")) != is_demo:
        errors.append("provenance.synthetic must exactly match DEMO_SYNTHETIC source")
    for frame_index, frame in enumerate(payload.get("frames", [])):
        seen_ids: set[str] = set()
        for person_index, person in enumerate(frame.get("people", [])):
            path = f"frames[{frame_index}].people[{person_index}]"
            person_id = person.get("id")
            if person_id in seen_ids:
                errors.append(f"{path}: duplicate id {person_id!r} in one frame")
            seen_ids.add(person_id)
            if is_demo and not str(person_id).startswith("DEMO-"):
                errors.append(f"{path}: demo identity must start with DEMO-")
            if not is_demo and str(person_id).startswith("DEMO-"):
                errors.append(f"{path}: live export cannot use a DEMO identity")
            world = person.get("position_world_m")
            scene = person.get("position_scene_m")
            cell = person.get("grid_cell")
            renderable = person.get("renderable") is True
            if renderable:
                if not isinstance(world, dict) or not isinstance(scene, dict) or not isinstance(cell, dict):
                    errors.append(f"{path}: renderable vector requires world, scene and grid")
                    continue
                expected_scene = world_to_scene(world["x"], world["y"], world["z"])
                if not _same_vector(scene, expected_scene):
                    errors.append(f"{path}: scene vector does not match world-to-scene mapping")
                expected_cell = world_to_grid(world["x"], world["y"])
                if expected_cell != cell:
                    errors.append(f"{path}: grid cell does not match World XY")
                if person.get("position_status") in {"invalid", "stale", "unavailable", "door_only"}:
                    errors.append(f"{path}: non-placeable status cannot be renderable")
            elif any(value is not None for value in (world, scene, cell)):
                errors.append(f"{path}: non-renderable vector must not contain a fallback position")
            cameras = person.get("camera_sources", [])
            if "B" in cameras and "A" not in cameras and renderable:
                errors.append(f"{path}: Camera B door-only must never publish World XY")
            if person.get("trust_level") == "trusted" and person.get("position_status") != "measured":
                errors.append(f"{path}: only measured positions may be trusted")
            timestamps = [point.get("capture_timestamp_s") for point in person.get("trajectory", [])]
            if timestamps != sorted(timestamps):
                errors.append(f"{path}: trajectory capture timestamps must be monotonic")
    return errors


def validate(document_path: Path, schema_path: Path) -> list[str]:
    payload = load_json(document_path)
    schema = load_json(schema_path)
    if not isinstance(payload, dict) or not isinstance(schema, dict):
        return ["document and schema roots must be objects"]
    Draft202012Validator.check_schema(schema)
    validator = Draft202012Validator(schema, format_checker=FormatChecker())
    errors = [
        f"{'.'.join(str(part) for part in error.absolute_path) or '<root>'}: {error.message}"
        for error in sorted(validator.iter_errors(payload), key=lambda item: list(item.absolute_path))
    ]
    if payload.get("schema_version") != SCHEMA_VERSION:
        errors.append(f"schema_version must be {SCHEMA_VERSION}")
    errors.extend(semantic_errors(payload))
    return errors


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("document", type=Path)
    parser.add_argument("--schema", type=Path, default=root / "schemas" / "camera_aim_3d_vectors_v1.schema.json")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        errors = validate(args.document, args.schema)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"VALIDATION FAILED: {error}", file=sys.stderr)
        return 2
    if errors:
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        return 1
    print(f"VALID: {args.document}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
