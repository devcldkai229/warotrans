# WaroTrans

Robot AMR vận chuyển hàng trong kho. Đồ án tốt nghiệp — mã đề tài FA26SE019.

Repo này gộp **khung kỹ thuật** (rules, contract, quy trình, quality gate) và
**cây source ROS 2** vào một chỗ. Tách hai thứ đó ra hai folder sẽ sinh hai
`README.md` và hai tài liệu kiến trúc mâu thuẫn nhau ngay tuần đầu.

---

## Cấu trúc

```text
warotrans/
├── .cursor/rules/        guardrail cho AI: hardware truth, TF, safety, workflow
├── .cursorignore         chặn Cursor index build artifact và dữ liệu runtime
│
├── docs/                 NGUỒN SỰ THẬT — đọc trước khi code
│   ├── STATUS.md                     robot HIỆN đang làm được gì (evidence-based)
│   ├── interfaces.md                 TF / topic / serial / power contract
│   ├── calibration.md                log số đo: ngày, phương pháp, điều kiện
│   ├── architecture.md               ranh giới package, dependency direction
│   ├── MASTER_ENGINEERING_GUIDE.md   roadmap theo phase + Definition of Done
│   ├── PACKAGE_IMPLEMENTATION_ORDER.md
│   ├── CURSOR_STARTER_PROMPT.md      prompt mẫu cho task đầu tiên
│   ├── troubleshooting.md            debug theo pipeline, từ tầng thấp lên
│   ├── adr/                          quyết định kiến trúc có ngày tháng
│   └── hardware/                     ảnh, DXF, datasheet làm evidence
│
├── ros2_ws/src/          cây package ROS 2 (hiện là skeleton có chủ đích)
├── firmware/             ESP32-S3, chưa tạo — xem firmware/README.md
│
└── tools/
    ├── validate_structure.py   kiểm tra cấu trúc + TF contract, chạy offline
    ├── check.sh                quality gate trước khi deploy
    ├── deploy.sh / deploy.ps1  đồng bộ ros2_ws/src lên Pi, có backup
    ├── doctor.sh / doctor.ps1  health check trên Pi
    └── rollback_source.sh      quay lại bản backup gần nhất trên Pi
```

`ros2_ws/src` hiện chứa package metadata, contract và launch/config **rỗng có
chủ đích**. Code chạy được mà chứa hằng số bịa sẽ được tin và không ai kiểm tra
lại; file rỗng thì không đánh lừa được ai.

---

## Thứ tự nguồn sự thật

Khi hai chỗ nói khác nhau, tin theo thứ tự này:

1. Kết quả đo trên robot thật, ghi trong `docs/calibration.md`
2. `docs/STATUS.md` — trạng thái đã verify
3. `docs/interfaces.md` — contract giữa subsystem
4. `.cursor/rules/`
5. `docs/MASTER_ENGINEERING_GUIDE.md`
6. Code hiện tại

Giá trị phần cứng chưa xác nhận thì ghi `CHƯA XÁC NHẬN`, không điền số đoán.

---

## Bắt đầu

```bash
git init
git add .
git commit -m "chore: initial WaroTrans engineering baseline"
```

Mở **thư mục gốc `warotrans/`** trong Cursor, không mở riêng `ros2_ws/`. Rules
trong `.cursor/rules/` chỉ được nạp khi repo root là workspace root.

Trước khi giao task đầu tiên, cập nhật cho khớp robot thật:

```text
docs/STATUS.md        tick lại những gì đã thật sự chạy
docs/calibration.md   điền số đã đo, để trống thì thôi
docs/interfaces.md    sửa nếu contract khác thực tế
```

Ba file đó là thứ phân biệt "AI đoán về robot của bạn" với "AI biết robot của bạn".

Kiểm tra khung:

```bash
python3 tools/validate_structure.py
./tools/check.sh
```

---

## Vòng lặp mỗi task

```text
git status sạch
   ↓
AI đọc STATUS + interfaces + calibration + rule liên quan
   ↓
giao MỘT task, một mối quan tâm
   ↓
AI plan trước: Goal / Files / What will NOT change / How to test
   ↓
sửa nhỏ  →  đọc git diff
   ↓
./tools/check.sh
   ↓
./tools/deploy.sh --run <launch>
   ↓
test trên robot thật
   ↓
cập nhật STATUS.md / calibration.md
   ↓
commit
```

Build PASS không đồng nghĩa robot PASS. Definition of Done cho robotics phải có
bằng chứng runtime: `ros2 topic hz`, `tf2_echo`, phép đo trên sàn.

---

## Thứ tự triển khai

Chi tiết ở `docs/PACKAGE_IMPLEMENTATION_ORDER.md`. Tóm tắt:

```text
description → sensors → hardware → bringup → slam
    → localization → firmware PID → navigation
        → perception → fleet → simulation/multi-robot
```

Không nhảy cóc. Mỗi bước sai sẽ biểu hiện thành "SLAM chạy kém" ở cuối, và lúc đó
không biết lỗi nằm ở khâu nào.

---

## Nguyên tắc

**AI được phép sáng tạo implementation. AI không được sáng tạo sự thật vật lý.**
