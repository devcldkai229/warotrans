# firmware/

ESP32-S3 low-level controller.

Các sketch hiện tại:

- `firmware/encoder_test/encoder_test.ino`: kiểm tra encoder độc lập, không điều khiển motor.
- `firmware/warotrans_low_level/warotrans_low_level.ino`: Phase 4 open-loop
  controller cho MDD10A, có watchdog và chưa có PID.

`warotrans_low_level` hiện được giữ dưới dạng Arduino sketch để nạp và kiểm tra
trực tiếp trên ESP32-S3. Trước khi xem là production firmware, cần compile bằng
đúng ESP32 Arduino core đang dùng và hoàn thành runtime safety test trên robot.

Hướng đóng gói PlatformIO cho firmware production vẫn được giữ cho phase sau:

Lý do dùng PlatformIO cho firmware production:

- ghim phiên bản core và thư viện (`platformio.ini`), tránh việc toolchain tự cập
  nhật;
- tách logic thuần như giải mã quadrature và parse giao thức để test trên máy tính;
- build từ dòng lệnh để `tools/check.sh` kiểm tra được.

Xem `docs/MASTER_ENGINEERING_GUIDE.md` phần ESP32 và
`.cursor/rules/20-firmware-esp32.mdc` trước khi bắt đầu.
