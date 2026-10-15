# Rule base Calibration & Reliability (`camera_aim_rules_v2`)

Code: `tools/rules.py`. Hai tập luật có thứ tự, thuần hàm, deterministic. Cùng snapshot + cùng calibration luôn cho cùng kết quả.

```text
calibration points ─► calibrate_homography.py ─► report ─► C01..C08 ─► valid / degraded / invalid
                                                                         │
Twin snapshot ─► Observation ─► R01..R12 (first match wins) ◄────────────┘
                                   │
                                   └─► status, reason (enum v1), trust, renderable, rule_id, trace
```

## 1. Luật calibration (chạy một lần cho mỗi profile)

| Rule | Vi phạm → | Điều kiện phải đúng |
|---|---|---|
| C01_min_points | invalid | ≥ 6 cặp điểm pixel ↔ sàn |
| C02_finite_homography | invalid | H không có NaN/Inf |
| C03_horizon_outside_room | invalid | 4 góc phòng chiếu ngược nằm cùng phía trước camera |
| C04_held_out_rmse | invalid | Leave-one-out RMSE ≤ 10 cm |
| C05_held_out_rmse_trusted | degraded | Leave-one-out RMSE ≤ 5 cm |
| C06_held_out_max | degraded | Lỗi held-out lớn nhất ≤ 15 cm |
| C07_hull_coverage | degraded | Hull điểm calibration phủ ≥ 35 % diện tích sàn |
| C08_lens_undistorted | degraded | Footpoint được undistort trước homography |

- `invalid`: mọi vị trí → `metric_invalid`, không render.
- `degraded`: vẫn render, nhưng trust tối đa `estimated`.

## 2. Luật observation (mỗi người, first match wins)

| # | Rule | Kết quả | Ghi chú |
|---|---|---|---|
| R01 | camera_b_door_only | door_only | Camera B không bao giờ có World XY |
| R02 | freshness | stale | invisible, state ≠ ACTIVE/TRACKED, `last_seen > 15 s`, hoặc `frame_ts − capture_ts > 15 s` (**mới**) |
| R03 | future_timestamp | invalid / invalid_position | `capture_ts > frame_ts + 0,5 s`: lệch đồng hồ (**mới**) |
| R04 | room_metric_authority | invalid / metric_invalid | Room metric sai **hoặc calibration invalid** (**mới**) |
| R05 | clamped_or_outside_valid | invalid / outside_valid_region | Không dùng điểm đã bị clamp |
| R06 | quality_invalid | invalid / no_footpoint, invalid_position | |
| R07 | missing_world_xy | unavailable | |
| R08 | unknown_quality | unavailable | |
| R09 | observation_metric | invalid / metric_invalid | |
| R10 | outside_room_no_clamp | invalid / outside_room | Không snap về mép phòng |
| R11 | uncertainty_too_large | invalid / outside_valid_region | Bán kính 95 % > 60 cm (**mới**, cần calibration) |
| R12 | hull_uncertainty_trust | measured/trusted hoặc estimated | estimated nếu ngoài hull (+25 cm margin), bán kính 95 % > 25 cm, cờ từ backend, hoặc calibration degraded |

Bán kính 95 % lấy từ nhiễu pixel của footpoint (`σu=2px`, `σv=4px`) truyền qua Jacobian của H tại đúng điểm đó. Vì thế vùng xa camera tự động kém tin cậy hơn.

Khi không truyền calibration, R11 và phần hull/uncertainty của R12 bị tắt. Kết quả khi đó trùng với logic cũ, trừ hai luật timestamp mới ở R02/R03.

## 3. Tương thích contract v1

- `position_reason` vẫn nằm trong enum v1. Rule chi tiết nằm ở `rule_id`/trace nội bộ.
- `provenance` có thêm field **optional**: `rule_set_version`, `calibration_id`, `calibration_fingerprint` (sha256 của H + hull + σ), `calibration_status`.
- Validator thêm luật semantic: calibration `invalid` thì không được có vector renderable, `degraded` thì không được có `trusted`.

## 4. Chạy

```powershell
.\.venv\Scripts\python.exe tools\calibrate_homography.py private\camera_a_points.json --output private\camera_a.json
.\.venv\Scripts\python.exe tools\export_twin_vectors.py --input private\twin_snapshot.json --calibration private\camera_a.json --output exports\vectors.json
.\.venv\Scripts\python.exe tools\validate_twin_vectors.py exports\vectors.json
```

Ngưỡng nằm trong `rules.Thresholds`. Nên chỉnh theo kết quả đo thật của Camera A, không theo demo synthetic.
