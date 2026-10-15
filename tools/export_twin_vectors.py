#!/usr/bin/env python3
"""Export sanitized Camera AIM vectors or deterministic offline demos."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any

from contract import adapt_twin_snapshot, build_demo
from rules import CalibrationProfile, check_calibration


MAX_INPUT_BYTES = 20 * 1024 * 1024


def _reject_constant(value: str) -> None:
    raise ValueError(f"non-finite JSON number is not allowed: {value}")


def read_json(path: Path) -> Any:
    size = path.stat().st_size
    if size > MAX_INPUT_BYTES:
        raise ValueError(f"input exceeds {MAX_INPUT_BYTES} bytes")
    return json.loads(path.read_text(encoding="utf-8"), parse_constant=_reject_constant)


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--input", type=Path, help="Local TwinLatest/DigitalTwin JSON snapshot")
    source.add_argument("--demo", choices=("single", "multi", "invalid"), help="Generate an offline synthetic fixture")
    parser.add_argument("--calibration", type=Path, help="Report from calibrate_homography.py (enables hull/uncertainty rules)")
    parser.add_argument("--output", type=Path, required=True, help="Output camera_aim_3d_vectors_v1 JSON")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        if args.demo:
            output = build_demo(args.demo)
        else:
            source = read_json(args.input)
            if not isinstance(source, dict):
                raise ValueError("input JSON root must be an object")
            calibration = None
            if args.calibration:
                calibration = check_calibration(
                    CalibrationProfile.from_report(read_json(args.calibration), args.calibration.stem)
                )
                print(f"CALIBRATION {calibration.status} {calibration.fingerprint} {calibration.findings}")
            output = adapt_twin_snapshot(source, calibration)
        write_json(args.output, output)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"EXPORT FAILED: {error}", file=sys.stderr)
        return 2
    print(f"WROTE {args.output} ({len(output['frames'])} frame(s))")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
