# Hệ tọa độ World và Scene

## Room Contract `office_001`

| Thuộc tính | Giá trị |
|---|---:|
| Width theo World X | 4,5 m |
| Length theo World Y | 10,0 m |
| Height theo World Z | 3,0 m |
| Ground plane | Z = 0 |
| Grid cell | 0,5 × 0,5 m |
| Grid | 9 cột × 20 hàng = 180 ô |

Origin `(0,0,0)` nằm tại giao điểm sàn và mép cửa trái khi nhìn vào phòng. X tăng từ trái sang phải, Y tăng từ cửa đến cửa sổ, Z tăng lên trên.

## World → Three.js scene

Three.js dùng Y-up, vì vậy frontend Camera AIM đang áp dụng:

```text
sceneX = worldX - 4.5 / 2
sceneY = worldZ
sceneZ = 10 / 2 - worldY
```

Ví dụ:

| World `(X,Y,Z)` | Scene `(X,Y,Z)` |
|---|---|
| `(0,0,0)` | `(-2.25,0,5)` |
| `(4.5,10,0)` | `(2.25,0,-5)` |
| `(2.25,5,0)` | `(0,0,0)` |

`position_scene_m` trong JSON là kết quả adapter và validator kiểm tra lại từ `position_world_m`. Đơn vị luôn là mét; thay đổi góc camera 3D không thay đổi số đo.

## World → Grid

Chỉ số bắt đầu từ 0:

```text
col = floor(worldX / 0.5)
row = floor(worldY / 0.5)
cell_id = C{col}_R{row}
```

Điểm đúng biên ngoài `(4.5,10)` thuộc ô cuối `C8_R19`. Điểm ngoài `[0,4.5] × [0,10]` là invalid; không clamp về ô gần nhất.

Grid chỉ là partition để hiển thị, không phải tuyên bố độ chính xác. Avatar giữ tọa độ World chính xác, không snap vào tâm ô.

## Calibration hull

Toàn bộ phòng và vùng calibration đáng tin là hai khái niệm khác nhau. Một World XY có thể nằm trong phòng nhưng ngoài calibration hull. Trường hợp này:

- Có thể render nếu backend vẫn xuất `quality=ok`.
- `position_status=estimated`.
- `trust_level=estimated`.
- UI cần biểu diễn khác với `trusted`.

Không dùng outside-hull estimate làm ground truth đo lường mà không có xác nhận bổ sung.
