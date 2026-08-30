# DevKit Changelog

## v2.2 — Đồng bộ với starter tree

- **Thêm `warotrans_sensors` vào package map.** Review starter tree phát hiện
  không package nào sở hữu driver LiDAR: grep toàn bộ source không ra một chữ
  "lidar". Config LiDAR sẽ trôi vào `bringup`, phá đúng ranh giới
  "bringup = orchestration only".
- **Thêm mục 3b Calibration ownership vào `architecture.md`:** tách rõ
  `docs/calibration.md` (log của người) với `calibration.yaml` (giá trị máy đọc),
  để URDF và node parameter không cầm hai bản sao của cùng một con số.
- **Thêm owner cho từng topic vào `interfaces.md`.**
- Bổ sung invariant: sensor driver và state estimation không cùng package.


## v2.1 — Gap fixes sau review

Các mục dưới đây là lỗ hổng phát hiện khi review v2.0, không phải thay đổi kiến trúc.
Kiến trúc, TF contract và source-of-truth model của v2.0 được giữ nguyên.

- **Thêm mục 6.1 vào Master Guide: USB current budget.** Khi Pi 5 không được cấp
  nguồn qua USB-C có đàm phán PD, tổng dòng cho các cổng USB bị giới hạn còn 600 mA.
  Đây là nguyên nhân trực tiếp làm LiDAR quay rồi dừng và `/scan` chập chờn.
  v2.0 thiếu hoàn toàn mục này.
- **Sửa `tools/check.sh`: TF contract guard chỉ quét YAML.** Lỗi `base_frame=base_link`
  thực tế hay nằm trong launch file Python (`'base_frame': 'base_link'`) chứ không
  nằm trong YAML. Guard cũ bỏ sót đúng chỗ dễ sai nhất. Nay quét cả YAML/Python/XML
  và thêm cảnh báo khi có nhiều file cùng publish TF.
- **Thêm nhánh 2b/2c vào troubleshooting:** LiDAR chập chờn do nguồn, và node chết
  do scan mode không tồn tại trên model đang dùng.
- **Thêm cảnh báo `scan_mode` vào Phase 2** của Master Guide.
- **Khôi phục thông tin camera đã biết trong `STATUS.md`.** v2.0 đánh dấu toàn bộ
  camera là "chưa xác nhận", làm mất thông tin từ quyết định mua. Nay tách rõ
  "đã biết nhưng chưa verify" và "chưa xác nhận", đồng thời thêm hai KNOWN RISKS
  có lead time (cáp CSI, driver camera) để kiểm tra sớm thay vì đợi tới Phase 13.
- **Thêm mục 4b Power & ground contract vào `interfaces.md`:** mass chung, cấm cấp
  hai nguồn cho ESP32, vòng lặp mass qua cáp USB, mức logic encoder.


## v2.0 — Professional engineering baseline

- Thêm `MASTER_ENGINEERING_GUIDE.md` end-to-end.
- Tách hardware thành CONFIRMED / CHƯA XÁC NHẬN.
- Bỏ việc mặc định Camera Module 3, battery cell/BMS/fuse nếu chưa verify.
- Bỏ VDA5050 khỏi vị trí "mặc định bắt buộc"; chuyển thành protocol adapter future.
- Chuẩn hóa TF owner và quy tắc một edge/một publisher.
- Thêm `STATUS.md`, `interfaces.md`, `architecture.md`, `troubleshooting.md`.
- Thêm ADR template và hardware evidence directory.
- Thêm rule vibe workflow/quality gates.
- Thêm `check.sh`.
- Deploy script không còn silent fallback nguy hiểm; hỗ trợ dry-run/cảnh báo.
- Doctor script chuyển sang best-effort diagnostics, không coi setting không bắt buộc là fatal.
- Các prompt ví dụ không còn biến số minh họa thành "measurement thật".
