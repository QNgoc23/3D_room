# Reliability và Grid

## Quyết định render

| Status | Render? | Trust | Ghi chú |
|---|---:|---|---|
| `measured` | Có | `trusted` | Camera A, metric authority hợp lệ, quality ok |
| `estimated` | Có | `estimated` | World XY ngoài calibration hull; cần cảnh báo |
| `simulated` | Có | `simulated` | Chỉ dùng demo/test |
| `invalid` | Không | `untrusted` | Footpoint/metric/out-of-room không hợp lệ |
| `stale` | Không | `untrusted` | Track mất hoặc quá TTL |
| `unavailable` | Không | `untrusted` | Không có World XY/quality xác định |
| `door_only` | Không | `untrusted` | Camera B không có homography phòng |

Pseudo-code consumer:

```text
if person.renderable is not true:
    show in "unverified" list using person.position_reason
else:
    place avatar at person.position_scene_m
    style by person.trust_level
```

## Lý do phổ biến

- `no_footpoint`: detection/track tồn tại nhưng không có điểm tiếp xúc sàn hợp lệ.
- `metric_invalid`: room metric authority hoặc observation không hợp lệ.
- `outside_valid_region`: clamped/outside vùng hợp lệ; không dùng làm chuẩn đo.
- `outside_calibration_hull`: trong room bounds nhưng là extrapolation estimate.
- `stale_track`: observation không còn tươi.
- `missing_world_xy`: không có tọa độ.
- `camera_b_door_only`: chỉ có sự kiện cửa.

## Grid không phải calibration coverage

Lưới 9×20 phủ toàn bộ hình học phòng. Calibration hull chỉ phủ vùng có quan sát hiệu chuẩn trực tiếp. Việc một vector có `grid_cell` không tự động làm nó thành trusted; consumer vẫn phải đọc `trust_level`.

## Motion

Không hiển thị speed/heading nếu `motion.available=false`. `speed_mps=null` khác hoàn toàn `speed_mps=0`: `null` là không có phép đo, còn `0` là phép đo vận tốc bằng không.

Không nội suy qua track mất, timestamp đảo chiều, gap trên 5 giây hoặc jump trên 2 mét. Exporter loại trajectory point không có capture time/metric quality thay vì đoán.
