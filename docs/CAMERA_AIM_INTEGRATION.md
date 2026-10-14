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
