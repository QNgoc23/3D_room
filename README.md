# Camera AIM — Digital Twin 3D Vector Demo

Gói bàn giao độc lập để team 3D đọc vector người từ Camera AIM, kiểm tra độ tin cậy và dựng người trong phòng `office_001` mà **không cần camera thật, backend production, database hay GPU inference**.

> Các file trong `examples/` đều là `DEMO_SYNTHETIC`. Chúng không phải dữ liệu live và không chứa tên nhân viên, khuôn mặt, confidence sinh trắc hoặc thông tin camera riêng tư.

## Có gì trong repository

- JSON Schema quyền lực: `schemas/camera_aim_3d_vectors_v1.schema.json`.
- Demo một frame, nhiều frame và các trạng thái invalid/outside-hull.
- Exporter chuyển Twin snapshot đã có thành JSON công khai đã loại PII.
- Validator kiểm tra JSON Schema và các quy tắc World/Grid/Reliability.
- Viewer 3D offline bằng Canvas, không dùng CDN hoặc dependency frontend.
- Tests cho room contract, tọa độ, Camera B, stale, invalid và provenance.

## Chạy nhanh trên Windows

```powershell
git clone https://github.com/ONgoc23/3D_room.git
cd 3D_room
py -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements-dev.txt
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
.\.venv\Scripts\python.exe tools\validate_twin_vectors.py examples\demo_multi_frame.json
.\.venv\Scripts\python.exe -m http.server 8080
```

Mở `http://127.0.0.1:8080/viewer/`. Không mở trực tiếp `viewer/index.html` bằng `file://` vì trình duyệt chặn `fetch()` JSON cục bộ.

## Xuất JSON

Tạo demo cố định:

```powershell
py tools\export_twin_vectors.py --demo single --output exports\single.json
py tools\export_twin_vectors.py --demo multi --output exports\multi.json
py tools\export_twin_vectors.py --demo invalid --output exports\invalid.json
```

Chuyển một file TwinLatest hoặc DigitalTwin snapshot đã được cung cấp hợp lệ:

```powershell
py tools\export_twin_vectors.py --input .\private\twin_snapshot.json --output .\exports\vectors.json
py tools\validate_twin_vectors.py .\exports\vectors.json
```

Exporter chỉ đọc file local. Nó không gọi API, RTSP, YOLO, ByteTrack hoặc tự khởi động Edge. Thư mục `private/` và `exports/` được `.gitignore` để tránh commit snapshot thật ngoài ý muốn.

## Đọc vector người

```python
import json

payload = json.load(open("examples/demo_single_frame.json", encoding="utf-8"))
for person in payload["frames"][0]["people"]:
    if not person["renderable"]:
        print(person["id"], person["position_reason"])
        continue
    world = person["position_world_m"]
    scene = person["position_scene_m"]
    print(person["id"], world, scene, person["trust_level"])
```

Chỉ đặt người vào scene khi `renderable=true`. Không thay `null` bằng `(0,0,0)`.

## Hệ tọa độ

World dùng mét:

- `X`: trái → phải khi nhìn từ cửa vào, `[0, 4.5]`.
- `Y`: cửa → cửa sổ, `[0, 10]`.
- `Z`: hướng lên, mặt sàn `Z=0`, trần `Z=3`.

Adapter Three.js hiện tại:

```text
sceneX = worldX - roomWidth / 2
sceneY = worldZ
sceneZ = roomLength / 2 - worldY
```

Unity có thể dùng `(sceneX, sceneY, sceneZ)` trực tiếp nếu scene là Y-up. Blender mặc định Z-up nên thường dùng World `(X, Y, Z)` và tự dịch origin theo mô hình phòng.

## LIVE và DEMO

| Provenance | Ý nghĩa |
|---|---|
| `DEMO_SYNTHETIC` | Dữ liệu mô phỏng công khai, ID luôn bắt đầu `DEMO-` |
| `CAMERA_AIM_TWIN_SNAPSHOT` | File snapshot được operator cung cấp và exporter đã loại PII |

Gói này không tự kết nối live. Khi team 3D bổ sung live transport, payload nhận được vẫn phải validate qua cùng JSON Schema trước khi render.

## Tài liệu

- [Tích hợp Camera AIM](docs/CAMERA_AIM_INTEGRATION.md)
- [Hệ tọa độ World/Scene](docs/WORLD_COORDINATE_SYSTEM.md)
- [Vector JSON Contract](docs/VECTOR_JSON_CONTRACT.md)
- [Reliability và Grid](docs/RELIABILITY_AND_GRID.md)

## Lưu ý cấp phép

Repository chưa tuyên bố giấy phép nguồn mở. Team sở hữu repository cần chọn LICENSE trước khi cho phép bên thứ ba tái sử dụng ngoài phạm vi bàn giao nội bộ.
