"""Per-track constant-velocity Kalman filter for World XY (stdlib only).

Raw homography footpoints jitter by several centimeters per frame, and more
toward the far wall. This filter fuses each measurement with its own
covariance (see ``geometry.world_covariance``) so far, noisy points pull the
estimate less than near, precise ones, and it yields a measured velocity.

It follows the same continuity rules as the export contract: the track is
reset (never interpolated across) on time reversal, a gap above 5 s or a jump
above 2 m. Outliers are rejected with a Mahalanobis gate rather than averaged
in. Positions are never clamped into the room.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any

MAX_GAP_S = 5.0
MAX_JUMP_M = 2.0
# chi-square quantile, 2 degrees of freedom, 99.9 %.
GATE_CHI2 = 13.82


@dataclass
class TrackFilter:
    accel_sigma_mps2: float = 1.5
    default_meas_sigma_m: float = 0.10
    max_consecutive_rejects: int = 3
    x: list[float] = field(default_factory=list)  # [x, y, vx, vy]
    p: list[list[float]] = field(default_factory=list)
    ts: float | None = None
    rejects: int = 0

    def reset(self, ts: float, mx: float, my: float, r: list[list[float]]) -> None:
        vel_var = 1.0  # walking speed prior, (1 m/s)²
        self.x = [mx, my, 0.0, 0.0]
        self.p = [
            [r[0][0], r[0][1], 0.0, 0.0],
            [r[1][0], r[1][1], 0.0, 0.0],
            [0.0, 0.0, vel_var, 0.0],
            [0.0, 0.0, 0.0, vel_var],
        ]
        self.ts = ts
        self.rejects = 0

    def _predict(self, dt: float) -> None:
        x, y, vx, vy = self.x
        self.x = [x + vx * dt, y + vy * dt, vx, vy]
        f = [[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]]
        fp = [[sum(f[i][k] * self.p[k][j] for k in range(4)) for j in range(4)] for i in range(4)]
        p = [[sum(fp[i][k] * f[j][k] for k in range(4)) for j in range(4)] for i in range(4)]
        q = self.accel_sigma_mps2 ** 2
        dt2, dt3, dt4 = dt * dt, dt ** 3, dt ** 4
        for a, b in ((0, 2), (1, 3)):
            p[a][a] += q * dt4 / 4
            p[a][b] += q * dt3 / 2
            p[b][a] += q * dt3 / 2
            p[b][b] += q * dt2
        self.p = p

    def update(self, ts: float, mx: float, my: float, cov: Any = None) -> dict[str, Any]:
        """Feed one measurement; returns the filtered state and what happened."""
        r = [list(row) for row in cov] if cov else [[self.default_meas_sigma_m ** 2, 0.0], [0.0, self.default_meas_sigma_m ** 2]]
        event = "update"
        if self.ts is None or ts < self.ts or ts - self.ts > MAX_GAP_S:
            self.reset(ts, mx, my, r)
            return self._state("reset")
        if math.hypot(mx - self.x[0], my - self.x[1]) > MAX_JUMP_M:
            self.reset(ts, mx, my, r)
            return self._state("reset")
        dt = ts - self.ts
        if dt > 0:
            self._predict(dt)
        self.ts = ts
        # Innovation and its covariance S = H P Hᵀ + R with H = [I 0].
        iy = [mx - self.x[0], my - self.x[1]]
        s = [[self.p[0][0] + r[0][0], self.p[0][1] + r[0][1]], [self.p[1][0] + r[1][0], self.p[1][1] + r[1][1]]]
        det = s[0][0] * s[1][1] - s[0][1] * s[1][0]
        if det <= 0:
            self.reset(ts, mx, my, r)
            return self._state("reset")
        s_inv = [[s[1][1] / det, -s[0][1] / det], [-s[1][0] / det, s[0][0] / det]]
        d2 = sum(iy[i] * s_inv[i][j] * iy[j] for i in range(2) for j in range(2))
        if d2 > GATE_CHI2:
            self.rejects += 1
            if self.rejects >= self.max_consecutive_rejects:
                self.reset(ts, mx, my, r)
                return self._state("reset")
            return self._state("rejected_outlier")
        self.rejects = 0
        # K = P Hᵀ S⁻¹  (4×2)
        k = [[sum(self.p[i][m] * s_inv[m][j] for m in range(2)) for j in range(2)] for i in range(4)]
        self.x = [self.x[i] + k[i][0] * iy[0] + k[i][1] * iy[1] for i in range(4)]
        # Joseph-free form is fine here: P = (I - K H) P
        p_old = self.p
        self.p = [[p_old[i][j] - k[i][0] * p_old[0][j] - k[i][1] * p_old[1][j] for j in range(4)] for i in range(4)]
        return self._state(event)

    def _state(self, event: str) -> dict[str, Any]:
        x, y, vx, vy = self.x
        return {
            "event": event,
            "position_world_m": {"x": x, "y": y, "z": 0.0},
            "velocity_world_mps": {"x": vx, "y": vy},
            "speed_mps": math.hypot(vx, vy),
            "heading_rad": math.atan2(vy, vx),
            "position_sigma_m": math.sqrt(max(self.p[0][0], 0.0) + max(self.p[1][1], 0.0)),
            # Velocity is only meaningful after the filter has seen motion.
            "motion_converged": event == "update" and self.p[2][2] < 0.09 and self.p[3][3] < 0.09,
        }


class MultiTrackSmoother:
    """One ``TrackFilter`` per stable person id."""

    def __init__(self, **filter_kwargs: Any) -> None:
        self._kwargs = filter_kwargs
        self._tracks: dict[str, TrackFilter] = {}

    def update(self, person_id: str, ts: float, mx: float, my: float, cov: Any = None) -> dict[str, Any]:
        track = self._tracks.setdefault(person_id, TrackFilter(**self._kwargs))
        return track.update(ts, mx, my, cov)

    def drop(self, person_id: str) -> None:
        self._tracks.pop(person_id, None)
