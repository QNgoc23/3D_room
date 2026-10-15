from __future__ import annotations

import math
from pathlib import Path
import random
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from calibrate_homography import build_report, synthetic_camera  # noqa: E402
from geometry import (  # noqa: E402
    apply_homography,
    convex_hull,
    distort_pixel,
    fit_homography,
    footpoint_from_bbox,
    inside_hull,
    uncertainty_radius_m,
    undistort_pixel,
    world_covariance,
)
from smoothing import TrackFilter  # noqa: E402


class GeometryTests(unittest.TestCase):
    def test_exact_homography_recovers_world_points(self):
        world = [(0.5, 1.0), (4.0, 1.0), (4.0, 9.0), (0.5, 9.0)]

        image = [(100.0, 900.0), (1800.0, 880.0), (1200.0, 300.0), (700.0, 310.0)]
        h = fit_homography(image, world)
        for (u, v), (x, y) in zip(image, world):
            wx, wy = apply_homography(h, u, v)
            self.assertAlmostEqual(wx, x, places=6)
            self.assertAlmostEqual(wy, y, places=6)

    def test_undistort_inverts_distort(self):
        k = [[900.0, 0.0, 960.0], [0.0, 900.0, 540.0], [0.0, 0.0, 1.0]]
        dist = [-0.28, 0.09, 0.0005, -0.0003, 0.0]
        for u, v in ((960.0, 540.0), (200.0, 900.0), (1700.0, 150.0)):
            du, dv = distort_pixel(u, v, k, dist)
            ru, rv = undistort_pixel(du, dv, k, dist)
            self.assertAlmostEqual(ru, u, delta=0.01)
            self.assertAlmostEqual(rv, v, delta=0.01)

    def test_undistortion_improves_held_out_accuracy(self):
        spec = synthetic_camera(noise_px=0.5)
        spec.pop("project")
        with_lens = build_report(spec)
        without = build_report({**spec, "dist_coeffs": None})
        self.assertLess(with_lens["loo_rmse_m"], 0.03)
        self.assertLess(with_lens["loo_rmse_m"], without["loo_rmse_m"] / 3)

    def test_uncertainty_grows_away_from_camera(self):
        spec = synthetic_camera()
        spec.pop("project")
        cells = {c["cell_id"]: c["uncertainty_95_m"] for c in build_report(spec)["cells"]}
        self.assertLess(cells["C4_R0"], cells["C4_R10"])
        self.assertLess(cells["C4_R10"], cells["C4_R19"])

    def test_covariance_is_symmetric_positive(self):
        h = fit_homography([(0, 0), (100, 0), (100, 100), (0, 100)], [(0, 0), (1, 0), (1, 1), (0, 1)])
        cov = world_covariance(h, 50, 50, 2.0, 4.0)
        self.assertEqual(cov[0][1], cov[1][0])
        self.assertGreater(cov[0][0], 0)
        self.assertGreater(uncertainty_radius_m(cov), 0)

    def test_footpoint_is_bbox_bottom_center(self):
        self.assertEqual(footpoint_from_bbox(100, 50, 200, 450), (150.0, 450))

    def test_hull_membership(self):
        hull = convex_hull([(0, 0), (2, 0), (2, 2), (0, 2), (1, 1)])
        self.assertEqual(len(hull), 4)
        self.assertTrue(inside_hull(hull, 1, 1))
        self.assertFalse(inside_hull(hull, 2.2, 1))
        self.assertTrue(inside_hull(hull, 2.2, 1, margin_m=0.3))

    def test_degenerate_points_rejected(self):
        with self.assertRaises(ValueError):
            fit_homography([(0, 0), (1, 1), (2, 2), (3, 3)], [(0, 0), (1, 1), (2, 2), (3, 3)])


class SmoothingTests(unittest.TestCase):
    def walk(self, noise_m, seed=3):
        rng = random.Random(seed)
        truth, meas = [], []
        for i in range(60):
            t = i * 0.2
            x, y = 1.0 + 0.4 * t, 2.0 + 0.3 * t
            truth.append((t, x, y))
            meas.append((t, x + rng.gauss(0, noise_m), y + rng.gauss(0, noise_m)))
        return truth, meas

    def test_filter_reduces_jitter_and_measures_velocity(self):
        truth, meas = self.walk(0.08)
        f = TrackFilter(default_meas_sigma_m=0.08, accel_sigma_mps2=0.3)
        raw_err, filt_err = [], []
        state = None
        for (t, tx, ty), (_, mx, my) in zip(truth, meas):
            state = f.update(t, mx, my)
            if t > 2.0:
                raw_err.append(math.hypot(mx - tx, my - ty))
                p = state["position_world_m"]
                filt_err.append(math.hypot(p["x"] - tx, p["y"] - ty))
        self.assertLess(sum(filt_err), 0.75 * sum(raw_err))
        self.assertAlmostEqual(state["speed_mps"], 0.5, delta=0.1)
        self.assertTrue(state["motion_converged"])

    def test_resets_on_gap_jump_and_time_reversal(self):
        f = TrackFilter()
        f.update(0.0, 1.0, 1.0)
        self.assertEqual(f.update(6.0, 1.0, 1.0)["event"], "reset")
        self.assertEqual(f.update(6.2, 3.5, 1.0)["event"], "reset")
        self.assertEqual(f.update(6.0, 3.5, 1.0)["event"], "reset")

    def test_single_outlier_is_gated(self):
        f = TrackFilter(default_meas_sigma_m=0.03)
        for i in range(10):
            f.update(i * 0.2, 1.0, 1.0)
        state = f.update(2.0, 2.4, 1.0)
        self.assertEqual(state["event"], "rejected_outlier")
        self.assertAlmostEqual(state["position_world_m"]["x"], 1.0, delta=0.05)

    def test_filter_never_clamps_into_room(self):
        f = TrackFilter()
        state = f.update(0.0, -0.3, 10.4)
        self.assertEqual(state["position_world_m"]["x"], -0.3)


if __name__ == "__main__":
    unittest.main()
