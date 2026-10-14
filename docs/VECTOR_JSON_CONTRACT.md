# Camera AIM 3D Vector JSON Contract

Canonical artifact: `schemas/camera_aim_3d_vectors_v1.schema.json` (JSON Schema Draft 2020-12).

## Versioning

`schema_version` hiện là `camera_aim_3d_vectors_v1`.

- Thêm field optional: thay đổi backward-compatible trong v1.
- Đổi nghĩa field, enum, trục tọa độ hoặc nullability: tạo v2.
- Consumer phải từ chối major version không hỗ trợ.
- Prose không được ghi đè JSON Schema; Schema là nguồn quyền lực.

## Top-level

| Field | Required | Ý nghĩa |
|---|---|---|
| `schema_version` | Có | Contract version |
| `provenance` | Có | LIVE snapshot hay DEMO synthetic |
| `generated_at_utc` | Có | Thời điểm tạo file export, không phải capture time |
| `room` | Có | Room/Grid contract cố định |
| `coordinate_system` | Có | World và scene mapping |
| `frames[]` | Có | Một hoặc nhiều frame |

## Person vector

Các field quan trọng:

- `id`: opaque stable ID. Demo dùng prefix `DEMO-`; live export dùng `G…`.
- `camera_sources`: `A` và/hoặc `B`.
- `capture_timestamp_s`: acquisition timestamp từ nguồn; `null` nếu nguồn không có.
- `position_world_m`: `{x,y,z}` hoặc `null`.
- `position_scene_m`: Three.js adapter hoặc `null`.
- `grid_cell`: cell 0-based hoặc `null`.
- `position_status`: `measured`, `estimated`, `simulated`, `invalid`, `stale`, `unavailable`, `door_only`.
- `quality`: `ok`, `clamped`, `invalid`, `unknown`.
- `trust_level`: `trusted`, `estimated`, `simulated`, `untrusted`.
- `renderable`: gate duy nhất trước khi đặt avatar.
- `motion`: velocity/heading; field con là `null` khi unavailable.
- `trajectory`: các điểm v1 có capture timestamp và quality `ok`.

## Null policy

`null` mang nghĩa “nguồn không cung cấp phép đo hợp lệ”. Không thay bằng 0. `renderable=false` bắt buộc `position_world_m`, `position_scene_m`, `grid_cell` đều `null`.

## Timestamp

- `generated_at_utc`: giờ tạo file.
- `frame.capture_timestamp_s`: giờ capture của frame từ source.
- `person.capture_timestamp_s`: giờ capture của observation.
- `trajectory[].capture_timestamp_s`: giờ capture từng điểm.

Exporter không phát minh capture timestamp cho trajectory legacy `[x,y]`.
Các giá trị `*_timestamp_s` giữ nguyên clock domain của source; consumer chỉ so sánh chúng trong cùng payload/stream, không tự coi là Unix time nếu provider chưa công bố clock domain.
Một người vắng mặt trong một frame đơn giản là không có trong `people[]` của frame đó; consumer không được giữ lại position cũ như observation hiện tại.

## Validation

```powershell
py tools\validate_twin_vectors.py examples\demo_single_frame.json
```

Validator thực hiện hai lớp:

1. JSON Schema: required fields, type, enum, additional properties.
2. Semantic: World↔Scene, World↔Grid, Camera B, reliability, duplicate ID và demo provenance.
