"""Camera AIM 3D vector contract helpers (standard library only).

This module deliberately has no network, camera, database, or inference code.
It converts an already-exported Digital Twin JSON snapshot into a sanitized,
versioned handoff payload for 3D consumers.
"""

from __future__ import annotations

from datetime import datetime, timezone
import math
from typing import Any, Iterable, Mapping


SCHEMA_VERSION = "camera_aim_3d_vectors_v1"
ROOM_ID = "office_001"
ROOM_WIDTH_M = 4.5
ROOM_LENGTH_M = 10.0
ROOM_HEIGHT_M = 3.0
GRID_CELL_M = 0.5
GRID_COLS = 9
GRID_ROWS = 20
STALE_AFTER_S = 15.0


def _finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))


def _number_or_none(value: Any) -> float | None:
    return float(value) if _finite(value) else None


def world_to_scene(x: float, y: float, z: float = 0.0) -> dict[str, float]:
    """Map Camera AIM world coordinates to the current Three.js convention."""
    return {
        "x": round(float(x) - ROOM_WIDTH_M / 2.0, 6),
        "y": round(float(z), 6),
        "z": round(ROOM_LENGTH_M / 2.0 - float(y), 6),
    }


def world_to_grid(x: float, y: float) -> dict[str, Any] | None:
    """Return the authoritative 0-based grid cell; never clamp out-of-room XY."""
    if not (_finite(x) and _finite(y)):
        return None
    x_f, y_f = float(x), float(y)
    if x_f < 0 or x_f > ROOM_WIDTH_M or y_f < 0 or y_f > ROOM_LENGTH_M:
        return None
    col = min(math.floor(x_f / GRID_CELL_M), GRID_COLS - 1)
    row = min(math.floor(y_f / GRID_CELL_M), GRID_ROWS - 1)
    return {
        "col": col,
        "row": row,
        "cell_id": f"C{col}_R{row}",
        "cell_size_m": GRID_CELL_M,
    }


def room_metadata() -> dict[str, Any]:
    return {
        "id": ROOM_ID,
        "width_m": ROOM_WIDTH_M,
        "length_m": ROOM_LENGTH_M,
        "height_m": ROOM_HEIGHT_M,
        "grid_cell_m": GRID_CELL_M,
        "grid_cols": GRID_COLS,
        "grid_rows": GRID_ROWS,
        "grid_total_cells": GRID_COLS * GRID_ROWS,
    }


def coordinate_metadata() -> dict[str, Any]:
    return {
        "world": {
            "name": "camera_aim_world",
            "unit": "meter",
            "origin": [0.0, 0.0, 0.0],
            "x_positive": "left_to_right_looking_in",
            "y_positive": "door_to_window",
            "z_positive": "up",
            "ground_z": 0.0,
        },
        "scene": {
            "name": "camera_aim_threejs_y_up",
            "unit": "meter",
            "mapping": {
                "x": "world.x - room.width_m / 2",
                "y": "world.z",
                "z": "room.length_m / 2 - world.y",
            },
        },
    }


def _camera_sources(person: Mapping[str, Any]) -> list[str]:
    raw: list[Any] = [person.get("camera")]
    for key in ("cam", "cameras"):
        value = person.get(key)
        if isinstance(value, list):
            raw.extend(value)
    result: list[str] = []
    for value in raw:
        text = str(value or "").strip().upper().replace("CAMERA_", "").replace("CAM_", "")
        if text in {"A", "B"} and text not in result:
            result.append(text)
    return result


def _person_id(raw_gid: Any) -> str:
    text = str(raw_gid if raw_gid is not None else "").strip()
    if not text:
        raise ValueError("person.gid is required for a stable 3D vector identity")
    return text if text.upper().startswith("G") else f"G{text}"


def _is_stale(person: Mapping[str, Any]) -> bool:
    if person.get("visible") is False:
        return True
    state = str(person.get("tracking_state") or "").upper()
    if state and state not in {"ACTIVE", "TRACKED"}:
        return True
    age = _number_or_none(person.get("last_seen_ago_s"))
    return age is not None and age > STALE_AFTER_S


def _point(value: Any) -> dict[str, float] | None:
    if not isinstance(value, Mapping):
        return None
    x, y = _number_or_none(value.get("x")), _number_or_none(value.get("y"))
    z = _number_or_none(value.get("z"))
    if x is None or y is None:
        return None
    return {"x": x, "y": y, "z": 0.0 if z is None else z}


def _quality(value: Any) -> str:
    text = str(value or "unknown").lower()
    return text if text in {"ok", "clamped", "invalid", "unknown"} else "unknown"


def _status(
    person: Mapping[str, Any], room_metric_valid: bool, cameras: list[str], position: dict[str, float] | None,
) -> tuple[str, str, str, bool, bool | None]:
    b_only = "B" in cameras and "A" not in cameras
    if b_only:
        return "door_only", "camera_b_door_only", "untrusted", False, None
    if _is_stale(person):
        return "stale", "stale_track", "untrusted", False, None
    quality = _quality(person.get("position_quality"))
    reason = str(person.get("position_reason") or "").lower()
    if not room_metric_valid:
        return "invalid", "metric_invalid", "untrusted", False, None
    if quality in {"clamped"} or reason == "outside_valid_region":
        return "invalid", "outside_valid_region", "untrusted", False, True
    if quality == "invalid":
        return "invalid", "no_footpoint" if reason == "no_footpoint" or position is None else "invalid_position", "untrusted", False, False if position is None else None
    if position is None:
        return "unavailable", "missing_world_xy", "untrusted", False, None
    if quality != "ok":
        return "unavailable", "unknown_quality", "untrusted", False, None
    if person.get("metric_valid") is not True:
        return "invalid", "metric_invalid", "untrusted", False, True
    if world_to_grid(position["x"], position["y"]) is None or position["z"] != 0.0:
        return "invalid", "outside_room", "untrusted", False, True
    outside_hull = person.get("extrapolation_warning") is True or person.get("inside_calibration_hull") is False
    if outside_hull:
        return "estimated", "outside_calibration_hull", "estimated", True, True
    return "measured", "ok", "trusted", True, True


def _trajectory_points(raw: Any) -> list[dict[str, Any]]:
    if not isinstance(raw, list):
        return []
    output: list[dict[str, Any]] = []
    previous_ts: float | None = None
    previous_xy: tuple[float, float] | None = None
    for item in raw:
        # Legacy [x, y] points have no acquisition time/reliability and are
        # intentionally not promoted into the public animation contract.
        if not isinstance(item, Mapping):
            continue
        x = _number_or_none(item.get("x"))
        y = _number_or_none(item.get("y"))
        ts = _number_or_none(item.get("capture_ts"))
        if x is None or y is None or ts is None:
            continue
        if item.get("metric_valid") is False or str(item.get("quality") or "ok") != "ok":
            continue
        if world_to_grid(x, y) is None:
            continue
        if previous_ts is not None and (ts < previous_ts or ts - previous_ts > 5.0):
            previous_ts, previous_xy = None, None
        if previous_xy is not None and math.hypot(x - previous_xy[0], y - previous_xy[1]) > 2.0:
            previous_ts, previous_xy = None, None
        point_world = {"x": x, "y": y, "z": 0.0}
        output.append({
            "capture_timestamp_s": ts,
            "position_world_m": point_world,
            "position_scene_m": world_to_scene(x, y, 0.0),
            "quality": "ok",
        })
        previous_ts, previous_xy = ts, (x, y)
    return output


def _trajectory_for(trajectories: Mapping[str, Any], person_id: str, raw_gid: Any) -> Any:
    variants = {person_id, person_id.removeprefix("G"), str(raw_gid)}
    for key in variants:
        if key in trajectories:
            return trajectories[key]
    return []


def _motion(person: Mapping[str, Any], renderable: bool) -> dict[str, Any]:
    available = renderable and person.get("motion_available") is True
    vel = person.get("vel")
    vx = vy = None
    if available and isinstance(vel, (list, tuple)) and len(vel) >= 2:
        vx, vy = _number_or_none(vel[0]), _number_or_none(vel[1])
    speed = _number_or_none(person.get("speed")) if available else None
    heading = _number_or_none(person.get("heading")) if available else None
    if vx is None or vy is None:
        available = False
        vx = vy = speed = heading = None
    return {
        "available": available,
        "velocity_world_mps": {"x": vx, "y": vy} if available else None,
        "speed_mps": speed if available else None,
        "heading_rad": heading if available else None,
        "reason": "ok" if available else str(person.get("motion_reason") or "unavailable"),
    }


def _adapt_person(
    person: Mapping[str, Any], trajectories: Mapping[str, Any], frame_ts: float | None, room_metric_valid: bool,
) -> dict[str, Any]:
    person_id = _person_id(person.get("gid"))
    cameras = _camera_sources(person)
    position = _point(person.get("position_m"))
    status, reason, trust, renderable, footpoint_valid = _status(person, room_metric_valid, cameras, position)
    if not renderable:
        position = None
    scene = world_to_scene(position["x"], position["y"], position["z"]) if position else None
    grid = world_to_grid(position["x"], position["y"]) if position else None
    source_grid = person.get("grid_cell")
    if grid is not None and isinstance(source_grid, Mapping):
        expected = (grid["col"], grid["row"], grid["cell_id"])
        actual = (source_grid.get("col"), source_grid.get("row"), source_grid.get("cell_id"))
        if actual != expected:
            raise ValueError(f"{person_id}: source grid {actual!r} disagrees with World XY {expected!r}")
    capture_ts = _number_or_none(person.get("ts_capture"))
    last_seen = _number_or_none(person.get("last_seen_ago_s"))
    trajectory = _trajectory_points(_trajectory_for(trajectories, person_id, person.get("gid"))) if renderable else []
    return {
        "id": person_id,
        "camera_sources": cameras,
        "tracking_state": str(person.get("tracking_state") or "UNKNOWN").upper(),
        "visible": bool(person.get("visible", True)),
        "capture_timestamp_s": capture_ts if capture_ts is not None else frame_ts,
        "last_seen_ago_s": last_seen,
        "position_world_m": position,
        "position_scene_m": scene,
        "grid_cell": grid,
        "position_status": status,
        "position_reason": reason,
        "quality": _quality(person.get("position_quality")),
        "metric_valid": bool(room_metric_valid and person.get("metric_valid") is True and renderable),
        "footpoint_valid": footpoint_valid,
        "trust_level": trust,
        "renderable": renderable,
        "motion": _motion(person, renderable),
        "trajectory": trajectory,
    }


def _validate_source_room(room: Mapping[str, Any]) -> None:
    width = _number_or_none(room.get("width_m"))
    length = _number_or_none(room.get("height_m"))
    ceiling = _number_or_none(room.get("ceiling_height_m"))
    if width is None or length is None:
        raise ValueError("source twin.room must contain finite width_m and height_m")
    if abs(width - ROOM_WIDTH_M) > 1e-6 or abs(length - ROOM_LENGTH_M) > 1e-6:
        raise ValueError(f"source room {width}x{length} does not match office_001 {ROOM_WIDTH_M}x{ROOM_LENGTH_M}")
    if ceiling is not None and abs(ceiling - ROOM_HEIGHT_M) > 1e-6:
        raise ValueError(f"source ceiling {ceiling} does not match office_001 {ROOM_HEIGHT_M}")


def adapt_twin_snapshot(payload: Mapping[str, Any]) -> dict[str, Any]:
    """Convert a TwinLatest envelope or bare DigitalTwin snapshot."""
    twin_value = payload.get("twin") if isinstance(payload.get("twin"), Mapping) else payload
    if not isinstance(twin_value, Mapping):
        raise ValueError("input must be a TwinLatest envelope or DigitalTwin object")
    twin: Mapping[str, Any] = twin_value
    room = twin.get("room")
    if not isinstance(room, Mapping):
        raise ValueError("source twin.room is required")
    _validate_source_room(room)
    people = twin.get("people")
    if people is None:
        people = []
    if not isinstance(people, list):
        raise ValueError("source twin.people must be an array")
    trajectories = twin.get("trajectories") if isinstance(twin.get("trajectories"), Mapping) else {}
    frame_ts = _number_or_none(twin.get("ts_capture"))
    if frame_ts is None:
        frame_ts = _number_or_none(twin.get("generated_at"))
    room_metric_valid = room.get("metric_positions_valid") is True
    vectors = [_adapt_person(item, trajectories, frame_ts, room_metric_valid) for item in people if isinstance(item, Mapping)]
    return {
        "schema_version": SCHEMA_VERSION,
        "provenance": {
            "source": "CAMERA_AIM_TWIN_SNAPSHOT",
            "synthetic": False,
            "classification": "SANITIZED_EXPORT",
            "input_schema_version": twin.get("schema_version"),
            "contains_biometrics": False,
        },
        "generated_at_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "room": room_metadata(),
        "coordinate_system": coordinate_metadata(),
        "frames": [{
            "frame_id": "FRAME_0001",
            "capture_timestamp_s": frame_ts,
            "people": vectors,
        }],
    }


def _demo_person(
    person_id: str, x: float | None, y: float | None, *, status: str = "simulated",
    reason: str = "simulated", trust: str = "simulated", renderable: bool = True,
    cameras: Iterable[str] = ("A",), ts: float | None = None, trajectory: list[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    world = {"x": x, "y": y, "z": 0.0} if renderable and x is not None and y is not None else None
    return {
        "id": person_id,
        "camera_sources": list(cameras),
        "tracking_state": "SIMULATED" if status in {"simulated", "estimated"} else "UNKNOWN",
        "visible": status not in {"stale", "door_only"},
        "capture_timestamp_s": ts,
        "last_seen_ago_s": 20.0 if status == "stale" else 0.0,
        "position_world_m": world,
        "position_scene_m": world_to_scene(x, y, 0.0) if world else None,
        "grid_cell": world_to_grid(x, y) if world else None,
        "position_status": status,
        "position_reason": reason,
        "quality": "ok" if renderable else "invalid" if status == "invalid" else "unknown",
        "metric_valid": renderable,
        "footpoint_valid": True if renderable else False if reason == "no_footpoint" else None,
        "trust_level": trust,
        "renderable": renderable,
        "motion": {
            "available": False,
            "velocity_world_mps": None,
            "speed_mps": None,
            "heading_rad": None,
            "reason": "demo_no_measured_motion",
        },
        "trajectory": trajectory or [],
    }


def _demo_envelope(frames: list[dict[str, Any]], demo_kind: str) -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "provenance": {
            "source": "DEMO_SYNTHETIC",
            "synthetic": True,
            "classification": "PUBLIC_DEMO",
            "input_schema_version": None,
            "contains_biometrics": False,
        },
        "generated_at_utc": "2026-01-01T00:00:00Z",
        "room": room_metadata(),
        "coordinate_system": coordinate_metadata(),
        "demo_kind": demo_kind,
        "frames": frames,
    }


def build_demo(kind: str) -> dict[str, Any]:
    """Build deterministic synthetic fixtures; IDs can never collide with live GIDs."""
    if kind == "single":
        return _demo_envelope([{
            "frame_id": "DEMO_FRAME_0001",
            "capture_timestamp_s": 1000.0,
            "people": [_demo_person("DEMO-PERSON-ALPHA", 1.5, 3.0, ts=1000.0)],
        }], "single_frame")
    if kind == "multi":
        frames: list[dict[str, Any]] = []
        alpha_path = [(1.0, 2.0), (1.35, 2.35), (1.7, 2.7), (2.05, 3.05), (2.4, 3.4)]
        beta_path = [(3.5, 7.5), (3.3, 7.2), (3.1, 6.9), (2.9, 6.6), (2.7, 6.3)]
        alpha_traj: list[dict[str, Any]] = []
        beta_traj: list[dict[str, Any]] = []
        for index, ((ax, ay), (bx, by)) in enumerate(zip(alpha_path, beta_path)):
            ts = 2000.0 + index
            alpha_traj.append({"capture_timestamp_s": ts, "position_world_m": {"x": ax, "y": ay, "z": 0.0}, "position_scene_m": world_to_scene(ax, ay), "quality": "ok"})
            beta_traj.append({"capture_timestamp_s": ts, "position_world_m": {"x": bx, "y": by, "z": 0.0}, "position_scene_m": world_to_scene(bx, by), "quality": "ok"})
            frame_people = [
                _demo_person("DEMO-PERSON-ALPHA", ax, ay, ts=ts, trajectory=list(alpha_traj)),
            ]
            # One deliberately missing observation proves that consumers do
            # not accumulate old rows or fabricate a position for this frame.
            if index != 3:
                frame_people.append(
                    _demo_person("DEMO-PERSON-BETA", bx, by, ts=ts, trajectory=list(beta_traj))
                )
            frames.append({
                "frame_id": f"DEMO_FRAME_{index + 1:04d}",
                "capture_timestamp_s": ts,
                "people": frame_people,
            })
        return _demo_envelope(frames, "multi_frame_animation")
    if kind == "invalid":
        people = [
            _demo_person("DEMO-OUTSIDE-HULL", 3.9, 8.6, status="estimated", reason="outside_calibration_hull", trust="estimated", ts=3000.0),
            _demo_person("DEMO-NO-FOOTPOINT", None, None, status="invalid", reason="no_footpoint", trust="untrusted", renderable=False, ts=3000.0),
            _demo_person("DEMO-STALE", None, None, status="stale", reason="stale_track", trust="untrusted", renderable=False, ts=2980.0),
            _demo_person("DEMO-NO-WORLD-XY", None, None, status="unavailable", reason="missing_world_xy", trust="untrusted", renderable=False, ts=3000.0),
            _demo_person("DEMO-CAMERA-B-DOOR", None, None, status="door_only", reason="camera_b_door_only", trust="untrusted", renderable=False, cameras=("B",), ts=3000.0),
        ]
        return _demo_envelope([{"frame_id": "DEMO_FRAME_INVALID", "capture_timestamp_s": 3000.0, "people": people}], "invalid_and_outside_hull")
    raise ValueError("demo kind must be one of: single, multi, invalid")
