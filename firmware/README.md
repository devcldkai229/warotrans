# firmware/

ESP32-S3 low-level controller.

**Chưa được tạo.** Sẽ là một project PlatformIO tại `firmware/esp32_low_level/`.

Không tạo thư mục này bằng cách copy một file `.ino` rời. Lý do dùng PlatformIO:

- ghim phiên bản core và thư viện (`platformio.ini`), tránh việc toolchain tự cập
  nhật rồi code ngừng biên dịch đúng lúc cần dùng;
- tách được logic thuần (giải mã quadrature, parse giao thức, chuyển đổi vận tốc)
  ra thư viện để test trên máy tính, không cần cắm board;
- build được từ dòng lệnh nên `tools/check.sh` kiểm tra được.

Xem `docs/MASTER_ENGINEERING_GUIDE.md` phần ESP32 và
`.cursor/rules/20-firmware-esp32.mdc` trước khi bắt đầu.
