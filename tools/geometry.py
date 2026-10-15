"""Camera → World floor geometry reference (standard library only).

Reference implementation of the Camera A pipeline step
``footpoint → (undistort) → Homography P1 → World XY`` with honest error
reporting. It contains no camera connector, network or inference code and no
real calibration data; callers supply their own private correspondences.

Improvements over a bare ``H @ [u, v, 1]``:

1. Lens undistortion (OpenCV Brown–Conrady model) *before* the homography,
   because a homography is only exact for an ideal pinhole camera.
2. Hartley-normalized DLT fit, which is far better conditioned than fitting raw
   pixel coordinates.
3. Leave-one-out error, which measures accuracy on points the fit did not see
   (training RMSE alone is optimistic).
4. Per-point uncertainty by propagating pixel noise through the homography
   Jacobian: one pixel near the far wall is much more floor than one pixel near
   the camera, so far positions get a larger covariance.
5. Calibration hull computed from the correspondences instead of guessed.
"""

from __future__ import annotations

import math
from typing import Iterable, Sequence

Matrix3 = list[list[float]]
Point = tuple[float, float]
Cov2 = tuple[tuple[float, float], tuple[float, float]]

# chi-square quantile, 2 degrees of freedom, 95 %.
CHI2_2DOF_95 = 5.991


def _solve(a: list[list[float]], b: list[float]) -> list[float]:
    """Gaussian elimination with partial pivoting (small dense systems)."""
    n = len(b)
    m = [row[:] + [b[i]] for i, row in enumerate(a)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(m[r][col]))
        if abs(m[pivot][col]) < 1e-12:
            raise ValueError("degenerate correspondences (collinear or duplicated points)")
        m[col], m[pivot] = m[pivot], m[col]
        for r in range(col + 1, n):
            factor = m[r][col] / m[col][col]
            for c in range(col, n + 1):
                m[r][c] -= factor * m[col][c]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        x[r] = (m[r][n] - sum(m[r][c] * x[c] for c in range(r + 1, n))) / m[r][r]
    return x


def _matmul(a: Matrix3, b: Matrix3) -> Matrix3:
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _normalizer(points: Sequence[Point]) -> tuple[Matrix3, Matrix3]:
    """Hartley normalization: centroid at 0, mean distance sqrt(2)."""
    cx = sum(p[0] for p in points) / len(points)
    cy = sum(p[1] for p in points) / len(points)
    mean_dist = sum(math.hypot(p[0] - cx, p[1] - cy) for p in points) / len(points)
    if mean_dist < 1e-12:
        raise ValueError("degenerate correspondences (all points identical)")
    s = math.sqrt(2.0) / mean_dist
    t = [[s, 0.0, -s * cx], [0.0, s, -s * cy], [0.0, 0.0, 1.0]]
    t_inv = [[1.0 / s, 0.0, cx], [0.0, 1.0 / s, cy], [0.0, 0.0, 1.0]]
    return t, t_inv


def invert3(m: list[list[float]]) -> list[list[float]]:
    a, b, c = m[0]
    d, e, f = m[1]
    g, h, i = m[2]
    det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    if abs(det) < 1e-15:
        raise ValueError("homography is singular")
    return [
        [(e * i - f * h) / det, (c * h - b * i) / det, (b * f - c * e) / det],
        [(f * g - d * i) / det, (a * i - c * g) / det, (c * d - a * f) / det],
        [(d * h - e * g) / det, (b * g - a * h) / det, (a * e - b * d) / det],
    ]


def apply_homography(h: Matrix3, u: float, v: float) -> Point:
    w = h[2][0] * u + h[2][1] * v + h[2][2]
    if abs(w) < 1e-12:
        raise ValueError("point maps to infinity (on or above the horizon line)")
    return ((h[0][0] * u + h[0][1] * v + h[0][2]) / w, (h[1][0] * u + h[1][1] * v + h[1][2]) / w)


def fit_homography(image_pts: Sequence[Point], world_pts: Sequence[Point]) -> Matrix3:
    """Least-squares image→floor homography from >= 4 correspondences."""
    if len(image_pts) != len(world_pts) or len(image_pts) < 4:
        raise ValueError("need at least 4 matching image/world points")
    t_img, _ = _normalizer(image_pts)
    t_world, t_world_inv = _normalizer(world_pts)
    ata = [[0.0] * 8 for _ in range(8)]
    atb = [0.0] * 8
    for (u0, v0), (x0, y0) in zip(image_pts, world_pts):
        u, v = apply_homography(t_img, u0, v0)
        x, y = apply_homography(t_world, x0, y0)
        for row, rhs in (([u, v, 1.0, 0.0, 0.0, 0.0, -u * x, -v * x], x),
                         ([0.0, 0.0, 0.0, u, v, 1.0, -u * y, -v * y], y)):
            for i in range(8):
                atb[i] += row[i] * rhs
                for j in range(8):
                    ata[i][j] += row[i] * row[j]
    h = _solve(ata, atb) + [1.0]
    h_norm = [h[0:3], h[3:6], h[6:9]]
    result = _matmul(_matmul(t_world_inv, h_norm), t_img)
    scale = result[2][2]
    return [[value / scale for value in row] for row in result]


def undistort_pixel(
    u: float, v: float, camera_matrix: Matrix3, dist: Sequence[float], iterations: int = 20,
) -> Point:
    """Invert OpenCV's (k1, k2, p1, p2[, k3]) distortion; returns ideal pixels."""
    fx, fy = camera_matrix[0][0], camera_matrix[1][1]
    cx, cy = camera_matrix[0][2], camera_matrix[1][2]
    k1, k2, p1, p2, k3 = (list(dist) + [0.0] * 5)[:5]
    xd, yd = (u - cx) / fx, (v - cy) / fy
    x, y = xd, yd
    for _ in range(iterations):
        r2 = x * x + y * y
        radial = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
        dx = 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
        dy = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
        x, y = (xd - dx) / radial, (yd - dy) / radial
    return (x * fx + cx, y * fy + cy)


def distort_pixel(u: float, v: float, camera_matrix: Matrix3, dist: Sequence[float]) -> Point:
    """Forward OpenCV distortion (used for synthetic tests and round-trips)."""
    fx, fy = camera_matrix[0][0], camera_matrix[1][1]
    cx, cy = camera_matrix[0][2], camera_matrix[1][2]
    k1, k2, p1, p2, k3 = (list(dist) + [0.0] * 5)[:5]
    x, y = (u - cx) / fx, (v - cy) / fy
    r2 = x * x + y * y
    radial = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
    xd = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
    yd = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
    return (xd * fx + cx, yd * fy + cy)


def footpoint_from_bbox(x1: float, y1: float, x2: float, y2: float) -> Point:
    """Ground contact = bottom-center of the bbox (never the bbox center)."""
    return ((x1 + x2) / 2.0, max(y1, y2))


def homography_jacobian(h: Matrix3, u: float, v: float) -> tuple[tuple[float, float], tuple[float, float]]:
    """d(worldX, worldY) / d(u, v) at a pixel, in meters per pixel."""
    w = h[2][0] * u + h[2][1] * v + h[2][2]
    x, y = apply_homography(h, u, v)
    return (
        ((h[0][0] - x * h[2][0]) / w, (h[0][1] - x * h[2][1]) / w),
        ((h[1][0] - y * h[2][0]) / w, (h[1][1] - y * h[2][1]) / w),
    )


def world_covariance(h: Matrix3, u: float, v: float, sigma_u_px: float = 2.0, sigma_v_px: float = 4.0) -> Cov2:
    """World XY covariance (m²) from pixel noise: Σw = J · diag(σu², σv²) · Jᵀ.

    The vertical default is larger because the bbox bottom edge jitters more
    than its horizontal center (feet, shadows, partial occlusion).
    """
    (a, b), (c, d) = homography_jacobian(h, u, v)
    su2, sv2 = sigma_u_px ** 2, sigma_v_px ** 2
    sxx = a * a * su2 + b * b * sv2
    syy = c * c * su2 + d * d * sv2
    sxy = a * c * su2 + b * d * sv2
    return ((sxx, sxy), (sxy, syy))


def uncertainty_radius_m(cov: Cov2, chi2: float = CHI2_2DOF_95) -> float:
    """Semi-major axis of the 95 % error ellipse, in meters."""
    (sxx, sxy), (_, syy) = cov
    mean = (sxx + syy) / 2.0
    lam_max = mean + math.sqrt(max(((sxx - syy) / 2.0) ** 2 + sxy * sxy, 0.0))
    return math.sqrt(chi2 * lam_max)


def convex_hull(points: Iterable[Point]) -> list[Point]:
    """Andrew monotone chain; counter-clockwise, no repeated first point."""
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts

    def cross(o: Point, a: Point, b: Point) -> float:
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower: list[Point] = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper: list[Point] = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def inside_hull(hull: Sequence[Point], x: float, y: float, margin_m: float = 0.0) -> bool:
    """True if (x, y) is inside the CCW hull, allowing ``margin_m`` outside it."""
    if len(hull) < 3:
        return False
    for i, (ax, ay) in enumerate(hull):
        bx, by = hull[(i + 1) % len(hull)]
        edge = math.hypot(bx - ax, by - ay)
        if edge == 0:
            continue
        signed = ((bx - ax) * (y - ay) - (by - ay) * (x - ax)) / edge
        if signed < -margin_m:
            return False
    return True


def calibration_report(
    image_pts: Sequence[Point], world_pts: Sequence[Point],
) -> dict[str, float | int | Matrix3]:
    """Fit H and report training RMSE plus leave-one-out (held-out) error."""
    h = fit_homography(image_pts, world_pts)
    residuals = [
        math.dist(apply_homography(h, u, v), w) for (u, v), w in zip(image_pts, world_pts)
    ]
    loo: list[float] = []
    if len(image_pts) >= 6:
        for i in range(len(image_pts)):
            h_i = fit_homography(image_pts[:i] + image_pts[i + 1:], world_pts[:i] + world_pts[i + 1:])
            loo.append(math.dist(apply_homography(h_i, *image_pts[i]), world_pts[i]))
    return {
        "homography": h,
        "points": len(image_pts),
        "train_rmse_m": math.sqrt(sum(r * r for r in residuals) / len(residuals)),
        "train_max_m": max(residuals),
        "loo_rmse_m": math.sqrt(sum(r * r for r in loo) / len(loo)) if loo else float("nan"),
        "loo_max_m": max(loo) if loo else float("nan"),
    }
