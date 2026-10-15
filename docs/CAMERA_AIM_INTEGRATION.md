# Tích hợp Camera AIM

## Phạm vi

Gói này là boundary adapter ngoại tuyến giữa Camera AIM Digital Twin và ứng dụng 3D. Nó không sao chép backend production và không chứa camera connector.

```text
Camera A
  → YOLO person detection
  → ByteTrack / Global ID
  → ground footpoint (bbox bottom contact)
  → Homography P1
  → World XY (meter)
  → grid cell 0.5 m
  → Digital Twin schema v2
  → export_twin_vectors.py
  → camera_aim_3d_vectors_v1
  → 3D renderer

Camera B → door presence only → no World XY
```

Các invariant:

1. Bbox center chỉ phục vụ tracking/debug, không phải điểm chiếu sàn.
2. Camera A là nguồn World XY canonical.
3. Camera B không được tạo World XY.
4. Position invalid/stale không được chuyển thành `(0,0,0)`.
5. Outside calibration hull được gắn `estimated`, không nâng thành `trusted`.
6. Motion chỉ xuất khi backend nói `motion_available=true`.

## Source field → export field → render usage

| Digital Twin source | Export v1 | 3D usage |
|---|---|---|
| `schema_version` | `provenance.input_schema_version` | Theo dõi compatibility |
| `ts_capture` / `generated_at` | `frame.capture_timestamp_s` | Thứ tự frame; không dùng giờ export thay giờ capture |
| `people[].gid` | `people[].id` | Stable object key; chuẩn hóa thành `G…` |
| `camera`, `cam[]`, `cameras[]` | `camera_sources[]` | Gate Camera B door-only |
| `position_m.{x,y,z}` | `position_world_m` | Tọa độ đặt chân trên sàn |
| `position_quality` | `quality` | Chỉ `ok` mới có thể render |
| `position_reason` | `position_reason` | Giải thích no-footpoint/metric/outside |
| `metric_valid` + `room.metric_positions_valid` | `metric_valid`, `renderable` | Fail closed khi metric authority không hợp lệ |
| `grid_cell` | `grid_cell` | Kiểm tra với World XY; mismatch làm export fail |
| `inside_calibration_hull`, `extrapolation_warning` | `position_status`, `trust_level` | Render cảnh báo estimated |
| `visible`, `tracking_state`, `last_seen_ago_s` | trạng thái tương ứng | Stale không được đặt vào scene |
| `motion_available`, `vel`, `speed`, `heading` | `motion` | Hiện vector chuyển động khi có phép đo |
| `trajectories[gid][]` v1 | `trajectory[]` | Vẽ đường đi theo capture timestamp |

Không export: `name`, `display_name`, `identity_confidence`, ảnh, embedding, bbox, camera URL, token hoặc credential.

## Input compatibility

Exporter nhận:

- Envelope `TwinLatest`: `{ "online": true, "twin": { ... } }`.
- Bare DigitalTwin: `{ "schema_version": 2, "room": ..., "people": ... }`.

Trajectory object v1 `{x, y, capture_ts, metric_valid, quality}` được hỗ trợ. Legacy tuple `[x,y]` không có capture time/reliability nên bị bỏ khỏi public animation contract, thay vì đoán timestamp.

## Live integration về sau

Transport live không thuộc gói này. Khi triển khai:

1. Camera AIM owner cung cấp endpoint hoặc message bus đã được phê duyệt.
2. Adapter chạy tại boundary tin cậy, không đặt RTSP credential ở client 3D.
3. Mỗi payload phải qua JSON Schema + semantic validator.
4. Renderer chỉ dùng `renderable=true`.
5. Không log raw snapshot chứa dữ liệu định danh.

## Tối ưu tọa độ Camera A → World (reference, chưa nằm trong contract v1)

`tools/geometry.py`, `tools/smoothing.py` và `tools/calibrate_homography.py` là bản tham chiếu cho phía Camera AIM backend (stdlib, không có dữ liệu calibration thật):

1. **Undistort footpoint trước homography** (`undistort_pixel`, model OpenCV `k1,k2,p1,p2,k3`). Homography chỉ đúng với camera pinhole; trên camera synthetic, held-out RMSE giảm từ ~8 cm xuống ~1 cm.
2. **Fit H bằng DLT chuẩn hóa Hartley** với ≥ 6 điểm phân bố đều sàn, và đánh giá bằng **leave-one-out RMSE** thay vì RMSE trên chính điểm fit.
3. **Uncertainty từng điểm**: `world_covariance` truyền nhiễu pixel (`σu≈2px`, `σv≈4px` cho cạnh đáy bbox) qua Jacobian của H. Vùng xa camera có bán kính 95 % lớn hơn nhiều; nên đặt ngưỡng `trusted` theo bán kính này thay vì chỉ theo hull.
4. **Calibration hull tính từ điểm calibration** (`convex_hull`, `inside_hull`) để sinh `inside_calibration_hull` thay vì gán tay.
5. **Kalman constant-velocity theo từng `gid`** (`MultiTrackSmoother`): dùng covariance ở bước 3 làm nhiễu đo, gate outlier bằng Mahalanobis, reset khi gap > 5 s / jump > 2 m / timestamp đảo chiều (đúng quy tắc trajectory). Cho velocity đo được để bật `motion_available` khi `motion_converged=true`.

```powershell
.\.venv\Scripts\python.exe tools\calibrate_homography.py --demo
.\.venv\Scripts\python.exe tools\calibrate_homography.py private\camera_a_points.json --output private\camera_a_report.json
```

Muốn xuất `position_uncertainty_m` hoặc vị trí đã lọc ra JSON thì phải thêm field optional vào schema (thay đổi backward-compatible v1).
