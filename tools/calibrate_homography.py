"""Fit and audit the Camera A image→floor homography from local correspondences.

Input JSON (keep real files under private/, which is git-ignored):

    {
      "camera_matrix": [[fx,0,cx],[0,fy,cy],[0,0,1]],   # optional
      "dist_coeffs": [k1,k2,p1,p2,k3],                   # optional
      "sigma_u_px": 2.0, "sigma_v_px": 4.0,              # optional footpoint noise
      "points": [{"u": 812.4, "v": 655.1, "x": 1.0, "y": 2.0}, ...]
    }

Prints held-out accuracy and writes a report with H, the calibration hull and a
95 % position-uncertainty radius per 0.5 m grid cell. ``--demo`` runs on a
synthetic camera so the tool can be tried without private data.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import random
import sys
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))

from contract import GRID_CELL_M, GRID_COLS, GRID_ROWS  # noqa: E402
from geometry import (  # noqa: E402
    apply_homography,
    calibration_report,
    convex_hull,
    distort_pixel,
    invert3,
    inside_hull,
    uncertainty_radius_m,
    undistort_pixel,
    world_covariance,
)


def synthetic_camera(noise_px: float = 0.0, seed: int = 7) -> dict[str, Any]:
    """DEMO_SYNTHETIC pinhole camera above the door, pitched 35° down, with lens distortion."""
    k = [[900.0, 0.0, 960.0], [0.0, 900.0, 540.0], [0.0, 0.0, 1.0]]
    dist = [-0.28, 0.09, 0.0005, -0.0003, 0.0]
    center = (2.25, -0.4, 2.7)
    pitch = math.radians(35.0)
    right = (1.0, 0.0, 0.0)
    forward = (0.0, math.cos(pitch), -math.sin(pitch))
    down = (0.0, -math.sin(pitch), -math.cos(pitch))
    rng = random.Random(seed)

    def project(x: float, y: float) -> tuple[float, float]:
        d = (x - center[0], y - center[1], 0.0 - center[2])
        xc = sum(r * q for r, q in zip(right, d))
        yc = sum(r * q for r, q in zip(down, d))
        zc = sum(r * q for r, q in zip(forward, d))
        u = k[0][0] * xc / zc + k[0][2]
        v = k[1][1] * yc / zc + k[1][2]
        return distort_pixel(u, v, k, dist)

    points = []
    for gx in (0.5, 1.5, 2.25, 3.0, 4.0):
        for gy in (1.0, 2.5, 4.0, 6.0, 8.0):
            u, v = project(gx, gy)
            points.append({"u": u + rng.gauss(0, noise_px), "v": v + rng.gauss(0, noise_px), "x": gx, "y": gy})
    return {"camera_matrix": k, "dist_coeffs": dist, "points": points, "project": project}


def build_report(spec: dict[str, Any]) -> dict[str, Any]:
    k, dist = spec.get("camera_matrix"), spec.get("dist_coeffs")
    sigma_u = float(spec.get("sigma_u_px", 2.0))
    sigma_v = float(spec.get("sigma_v_px", 4.0))
    image, world = [], []
    for p in spec["points"]:
        u, v = float(p["u"]), float(p["v"])
        if k and dist:
            u, v = undistort_pixel(u, v, k, dist)
        image.append((u, v))
        world.append((float(p["x"]), float(p["y"])))
    report = calibration_report(image, world)
    h = report["homography"]
    h_inv = invert3(h)
    hull = convex_hull(world)
    cells = []
    for row in range(GRID_ROWS):
        for col in range(GRID_COLS):
            cx, cy = (col + 0.5) * GRID_CELL_M, (row + 0.5) * GRID_CELL_M
            u, v = apply_homography(h_inv, cx, cy)
            cov = world_covariance(h, u, v, sigma_u, sigma_v)
            cells.append({
                "cell_id": f"C{col}_R{row}",
                "uncertainty_95_m": round(uncertainty_radius_m(cov), 4),
                "inside_calibration_hull": inside_hull(hull, cx, cy),
            })
    return {
        "undistorted": bool(k and dist),
        "points": report["points"],
        "train_rmse_m": report["train_rmse_m"],
        "train_max_m": report["train_max_m"],
        "loo_rmse_m": report["loo_rmse_m"],
        "loo_max_m": report["loo_max_m"],
        "homography_undistorted_px_to_world_m": h,
        "calibration_hull_world_m": [list(p) for p in hull],
        "sigma_px": {"u": sigma_u, "v": sigma_v},
        "cells": cells,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("input", nargs="?", type=Path, help="correspondence JSON")
    parser.add_argument("--demo", action="store_true", help="use a synthetic camera instead of input")
    parser.add_argument("--no-undistort", action="store_true", help="ignore lens distortion (for comparison)")
    parser.add_argument("--output", type=Path, help="write the full report JSON here")
    args = parser.parse_args()
    if args.demo:
        spec = synthetic_camera(noise_px=0.5)
        spec.pop("project")
    elif args.input:
        spec = json.loads(args.input.read_text(encoding="utf-8"))
    else:
        parser.error("give an input JSON or --demo")
    if args.no_undistort:
        spec = {**spec, "dist_coeffs": None}
    report = build_report(spec)
    worst = max(report["cells"], key=lambda c: c["uncertainty_95_m"])
    best = min(report["cells"], key=lambda c: c["uncertainty_95_m"])
    outside = sum(not c["inside_calibration_hull"] for c in report["cells"])
    print(f"points={report['points']} undistorted={report['undistorted']}")
    print(f"train RMSE={report['train_rmse_m'] * 100:.1f} cm (max {report['train_max_m'] * 100:.1f} cm)")
    print(f"held-out RMSE={report['loo_rmse_m'] * 100:.1f} cm (max {report['loo_max_m'] * 100:.1f} cm)")
    print(f"95% radius: best {best['cell_id']} {best['uncertainty_95_m'] * 100:.1f} cm, worst {worst['cell_id']} {worst['uncertainty_95_m'] * 100:.1f} cm")
    print(f"grid cells outside calibration hull: {outside}/{len(report['cells'])}")
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(f"report: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
