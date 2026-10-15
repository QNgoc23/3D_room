"""Rule base for Camera A calibration and per-observation reliability (stdlib only).

Two ordered, deterministic rule sets replace the hand-written if-chain:

* Calibration rules (``C*``) decide whether a calibration profile is
  ``valid``, ``degraded`` (positions at most ``estimated``) or ``invalid``
  (no metric positions at all).
* Observation rules (``R*``) run first-match-wins on every person and yield
  ``(position_status, position_reason, trust_level, renderable, footpoint_valid)``
  plus a trace of the rule ids that were checked.

Exported reasons stay inside the v1 enum; the finer rule id lives in the trace.
Without a calibration profile the observation rules reproduce the legacy
behaviour exactly, so existing exports do not change.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import hashlib
import json
import math
from typing import Any, Callable, Mapping, Sequence

from geometry import apply_homography, inside_hull, invert3, uncertainty_radius_m, world_covariance

RULE_SET_VERSION = "camera_aim_rules_v2"

ROOM_WIDTH_M = 4.5
ROOM_LENGTH_M = 10.0
STALE_AFTER_S = 15.0
FUTURE_SKEW_S = 0.5


@dataclass(frozen=True)
class Thresholds:
    # Calibration acceptance.
    min_points: int = 6
    max_loo_rmse_m: float = 0.05
    max_loo_max_m: float = 0.15
    degraded_loo_rmse_m: float = 0.10
    min_hull_coverage: float = 0.35
    # Observation reliability (95 % radius from pixel-noise propagation).
    trusted_radius_m: float = 0.25
    max_radius_m: float = 0.60
    hull_margin_m: float = 0.25
    stale_after_s: float = STALE_AFTER_S
    future_skew_s: float = FUTURE_SKEW_S


DEFAULT_THRESHOLDS = Thresholds()


# --------------------------------------------------------------------------- calibration


@dataclass
class CalibrationProfile:
    calibration_id: str
    homography: list[list[float]]
    hull: list[tuple[float, float]]
    sigma_u_px: float = 2.0
    sigma_v_px: float = 4.0
    points: int = 0
    loo_rmse_m: float = math.nan
    loo_max_m: float = math.nan
    undistorted: bool = False
    status: str = "unchecked"
    findings: list[str] = field(default_factory=list)
    _h_inv: list[list[float]] | None = None

    @classmethod
    def from_report(cls, report: Mapping[str, Any], calibration_id: str) -> "CalibrationProfile":
        sigma = report.get("sigma_px") or {}
        return cls(
            calibration_id=calibration_id,
            homography=[list(map(float, row)) for row in report["homography_undistorted_px_to_world_m"]],
            hull=[(float(x), float(y)) for x, y in report.get("calibration_hull_world_m", [])],
            sigma_u_px=float(sigma.get("u", 2.0)),
            sigma_v_px=float(sigma.get("v", 4.0)),
            points=int(report.get("points", 0)),
            loo_rmse_m=float(report.get("loo_rmse_m", math.nan)),
            loo_max_m=float(report.get("loo_max_m", math.nan)),
            undistorted=bool(report.get("undistorted", False)),
        )

    @property
    def fingerprint(self) -> str:
        canonical = json.dumps({
            "h": [[round(v, 12) for v in row] for row in self.homography],
            "hull": [[round(x, 6), round(y, 6)] for x, y in self.hull],
            "sigma": [self.sigma_u_px, self.sigma_v_px],
            "undistorted": self.undistorted,
        }, sort_keys=True, separators=(",", ":"))
        return "sha256:" + hashlib.sha256(canonical.encode("utf-8")).hexdigest()[:16]

    def h_inv(self) -> list[list[float]]:
        if self._h_inv is None:
            self._h_inv = invert3(self.homography)
        return self._h_inv

    def radius_95_m(self, x: float, y: float) -> float:
        """95 % position radius at a world point (back-project, then propagate)."""
        u, v = apply_homography(self.h_inv(), x, y)
        return uncertainty_radius_m(world_covariance(self.homography, u, v, self.sigma_u_px, self.sigma_v_px))


def _hull_area(hull: Sequence[tuple[float, float]]) -> float:
    return abs(sum(x1 * y2 - x2 * y1 for (x1, y1), (x2, y2) in zip(hull, list(hull[1:]) + list(hull[:1])))) / 2.0


def _horizon_outside_room(profile: CalibrationProfile) -> bool:
    """All room corners must back-project in front of the camera (w > 0)."""
    h_inv = profile.h_inv()
    signs = set()
    for x, y in ((0, 0), (ROOM_WIDTH_M, 0), (ROOM_WIDTH_M, ROOM_LENGTH_M), (0, ROOM_LENGTH_M)):
        w = h_inv[2][0] * x + h_inv[2][1] * y + h_inv[2][2]
        signs.add(w > 0)
    return len(signs) == 1


CalibrationRule = tuple[str, str, Callable[[CalibrationProfile, Thresholds], bool]]

# (rule id, severity if violated, predicate that must hold)
CALIBRATION_RULES: list[CalibrationRule] = [
    ("C01_min_points", "invalid", lambda p, t: p.points >= t.min_points),
    ("C02_finite_homography", "invalid", lambda p, t: all(math.isfinite(v) for row in p.homography for v in row)),
    ("C03_horizon_outside_room", "invalid", lambda p, t: _horizon_outside_room(p)),
    ("C04_held_out_rmse", "invalid", lambda p, t: math.isfinite(p.loo_rmse_m) and p.loo_rmse_m <= t.degraded_loo_rmse_m),
    ("C05_held_out_rmse_trusted", "degraded", lambda p, t: p.loo_rmse_m <= t.max_loo_rmse_m),
    ("C06_held_out_max", "degraded", lambda p, t: math.isfinite(p.loo_max_m) and p.loo_max_m <= t.max_loo_max_m),
    ("C07_hull_coverage", "degraded", lambda p, t: len(p.hull) >= 3 and _hull_area(p.hull) / (ROOM_WIDTH_M * ROOM_LENGTH_M) >= t.min_hull_coverage),
    ("C08_lens_undistorted", "degraded", lambda p, t: p.undistorted),
]


def check_calibration(profile: CalibrationProfile, thresholds: Thresholds = DEFAULT_THRESHOLDS) -> CalibrationProfile:
    """Run every calibration rule; set ``profile.status`` and ``profile.findings``."""
    findings: list[str] = []
    status = "valid"
    for rule_id, severity, predicate in CALIBRATION_RULES:
        try:
            ok = predicate(profile, thresholds)
        except (ValueError, ZeroDivisionError):
            ok = False
        if not ok:
            findings.append(f"{rule_id}:{severity}")
            if severity == "invalid":
                status = "invalid"
            elif status == "valid":
                status = "degraded"
    profile.status, profile.findings = status, findings
    return profile


# --------------------------------------------------------------------------- observations


@dataclass(frozen=True)
class Observation:
    cameras: tuple[str, ...]
    position: Mapping[str, float] | None
    quality: str
    reason: str
    metric_valid: bool | None
    room_metric_valid: bool
    visible: bool | None
    tracking_state: str
    last_seen_ago_s: float | None
    capture_ts: float | None
    frame_ts: float | None
    inside_hull_flag: bool | None
    extrapolation_warning: bool | None


@dataclass(frozen=True)
class Decision:
    status: str
    reason: str
    trust: str
    renderable: bool
    footpoint_valid: bool | None
    rule_id: str = ""
    radius_95_m: float | None = None


@dataclass
class RuleContext:
    obs: Observation
    calibration: CalibrationProfile | None
    t: Thresholds


def _reject(status: str, reason: str, footpoint: bool | None = None) -> Decision:
    return Decision(status, reason, "untrusted", False, footpoint)


def _in_room(p: Mapping[str, float]) -> bool:
    return 0 <= p["x"] <= ROOM_WIDTH_M and 0 <= p["y"] <= ROOM_LENGTH_M and p["z"] == 0.0


def r_camera_b(c: RuleContext) -> Decision | None:
    o = c.obs
    if "B" in o.cameras and "A" not in o.cameras:
        return _reject("door_only", "camera_b_door_only")
    return None


def r_stale(c: RuleContext) -> Decision | None:
    o = c.obs
    if o.visible is False:
        return _reject("stale", "stale_track")
    if o.tracking_state and o.tracking_state not in {"ACTIVE", "TRACKED"}:
        return _reject("stale", "stale_track")
    if o.last_seen_ago_s is not None and o.last_seen_ago_s > c.t.stale_after_s:
        return _reject("stale", "stale_track")
    if o.capture_ts is not None and o.frame_ts is not None and o.frame_ts - o.capture_ts > c.t.stale_after_s:
        return _reject("stale", "stale_track")
    return None


def r_future_timestamp(c: RuleContext) -> Decision | None:
    o = c.obs
    if o.capture_ts is not None and o.frame_ts is not None and o.capture_ts - o.frame_ts > c.t.future_skew_s:
        return _reject("invalid", "invalid_position")
    return None


def r_room_metric(c: RuleContext) -> Decision | None:
    if not c.obs.room_metric_valid:
        return _reject("invalid", "metric_invalid")
    if c.calibration is not None and c.calibration.status == "invalid":
        return _reject("invalid", "metric_invalid")
    return None


def r_clamped(c: RuleContext) -> Decision | None:
    if c.obs.quality == "clamped" or c.obs.reason == "outside_valid_region":
        return _reject("invalid", "outside_valid_region", True)
    return None


def r_quality_invalid(c: RuleContext) -> Decision | None:
    o = c.obs
    if o.quality != "invalid":
        return None
    if o.reason == "no_footpoint" or o.position is None:
        return _reject("invalid", "no_footpoint", False if o.position is None else None)
    return _reject("invalid", "invalid_position")


def r_missing_xy(c: RuleContext) -> Decision | None:
    return _reject("unavailable", "missing_world_xy") if c.obs.position is None else None


def r_unknown_quality(c: RuleContext) -> Decision | None:
    return _reject("unavailable", "unknown_quality") if c.obs.quality != "ok" else None


def r_observation_metric(c: RuleContext) -> Decision | None:
    return _reject("invalid", "metric_invalid", True) if c.obs.metric_valid is not True else None


def r_outside_room(c: RuleContext) -> Decision | None:
    # Never clamp: a point outside the room is invalid, not snapped to the edge.
    return _reject("invalid", "outside_room", True) if not _in_room(c.obs.position) else None


def r_uncertainty_too_large(c: RuleContext) -> Decision | None:
    if c.calibration is None:
        return None
    p = c.obs.position
    radius = c.calibration.radius_95_m(p["x"], p["y"])
    if radius > c.t.max_radius_m:
        return Decision("invalid", "outside_valid_region", "untrusted", False, True, radius_95_m=radius)
    return None


def r_estimated(c: RuleContext) -> Decision | None:
    o = c.obs
    p = o.position
    radius = c.calibration.radius_95_m(p["x"], p["y"]) if c.calibration else None
    flagged = o.extrapolation_warning is True or o.inside_hull_flag is False
    if c.calibration is not None:
        flagged = flagged or not inside_hull(c.calibration.hull, p["x"], p["y"], c.t.hull_margin_m)
        flagged = flagged or (radius is not None and radius > c.t.trusted_radius_m)
        flagged = flagged or c.calibration.status == "degraded"
    if flagged:
        return Decision("estimated", "outside_calibration_hull", "estimated", True, True, radius_95_m=radius)
    return Decision("measured", "ok", "trusted", True, True, radius_95_m=radius)


OBSERVATION_RULES: list[tuple[str, Callable[[RuleContext], Decision | None]]] = [
    ("R01_camera_b_door_only", r_camera_b),
    ("R02_freshness", r_stale),
    ("R03_future_timestamp", r_future_timestamp),
    ("R04_room_metric_authority", r_room_metric),
    ("R05_clamped_or_outside_valid", r_clamped),
    ("R06_quality_invalid", r_quality_invalid),
    ("R07_missing_world_xy", r_missing_xy),
    ("R08_unknown_quality", r_unknown_quality),
    ("R09_observation_metric", r_observation_metric),
    ("R10_outside_room_no_clamp", r_outside_room),
    ("R11_uncertainty_too_large", r_uncertainty_too_large),
    ("R12_hull_uncertainty_trust", r_estimated),
]


def evaluate(
    obs: Observation, calibration: CalibrationProfile | None = None, thresholds: Thresholds = DEFAULT_THRESHOLDS,
) -> tuple[Decision, list[str]]:
    """First matching rule wins; returns the decision and the rule ids checked."""
    ctx = RuleContext(obs, calibration, thresholds)
    trace: list[str] = []
    for rule_id, rule in OBSERVATION_RULES:
        trace.append(rule_id)
        decision = rule(ctx)
        if decision is not None:
            return Decision(**{**decision.__dict__, "rule_id": rule_id}), trace
    raise AssertionError("R12 always decides")  # pragma: no cover
