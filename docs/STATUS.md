# WaroTrans — Current Robot Status

> Đây là **trạng thái thực tế**, không phải roadmap. Chỉ tick `[x]` sau khi có evidence trên robot thật.
> Mỗi session vibe coding nên đọc file này trước.

**Last reviewed:** 2026-08-30  
**Operator:** waro  
**Robot hostname:** `warotrans`

## VERIFIED

- [x] Raspberry Pi 5 boot được Ubuntu Server 24.04.
- [x] SSH vào Pi bằng user `waro`.
- [x] mDNS/hostname dùng được qua `warotrans.local`.
- [x] MDD10A đã được cấp nguồn và test bằng button trên board.
- [x] Các bánh/motor có thể quay trong bài test thủ công hiện tại.
- [x] Chassis hiện tại: 320 mm × 260 mm, mica 3 mm.
- [x] Wheel nominal diameter: Ø65 mm.

## IN PROGRESS / NEXT CHECKPOINT

- [ ] `usb_max_current_enable=1` trong `/boot/firmware/config.txt` (bắt buộc khi Pi
      chạy nguồn onboard; xem Master Guide 6.1). Chưa có mục này thì LiDAR sẽ chập chờn.
- [ ] ROS 2 Jazzy được cài và `ros2 --help` PASS.
- [ ] RPLIDAR được Pi nhận bằng USB.
- [ ] Stable device name `/dev/rplidar`.
- [ ] `/scan` publish ổn định.
- [ ] Laptop visualize LaserScan realtime.

## NOT YET VERIFIED

- [ ] ESP32 serial device `/dev/warotrans` ổn định.
- [ ] Encoder wiring được xác định bằng đo.
- [ ] Tick encoder đúng dấu.
- [ ] `/odom` publish ổn định.
- [ ] TF `odom → base_footprint → base_link → laser` PASS.
- [ ] `ticks_per_rev` calibrated.
- [ ] `wheel_radius` calibrated.
- [ ] `wheel_separation` calibrated.
- [ ] SLAM dựng map đạt quality gate.
- [ ] BNO055 / IMU integration.
- [ ] EKF localization fusion.
- [ ] Closed-loop motor PID.
- [ ] Static-map localization.
- [ ] Nav2 goal navigation.
- [ ] Camera model/interface được xác nhận và stream được.
- [ ] Fleet bridge end-to-end.
- [ ] Multi-robot.

## KNOWN UNKNOWNS

- Battery pack exact cell/BMS/fuse configuration: **không dùng DevKit này làm source of truth**.
- Encoder voltage/pin colors: **CHƯA XÁC NHẬN bằng measurement**.
- Physical calibration constants: xem `docs/calibration.md`; các ô trống là chưa đo.
- Camera pose trên robot: chưa đo (xem mục camera bên dưới cho phần đã biết).

## CAMERA — phân biệt đã biết và chưa xác minh

Đánh dấu "chưa xác nhận" toàn bộ camera là quá thận trọng và làm mất thông tin
đã có. Tách rõ hai loại:

**Đã biết (từ quyết định mua, chưa verify trên robot):**

- Camera dự kiến: Raspberry Pi Camera Module 3, bản có auto focus, ống kính thường
  (không phải bản wide).
- Interface: CSI.

**CHƯA XÁC NHẬN bằng measurement:**

- Cáp đi kèm có đúng loại cắm được vào Pi 5 hay không.
- Camera có được kernel/driver trên Ubuntu 24.04 nhận hay không.
- Pose camera trên robot.

## KNOWN RISKS — camera (kiểm tra sớm, đừng để tới Phase 13)

Hai rủi ro dưới đây có **lead time** hoặc **thời gian build dài**. Phát hiện muộn
sẽ chặn cả phase, nên xác minh ngay khi mở hộp chứ không đợi tới lúc làm camera.

**R-CAM-1 — cáp CSI có thể không cắm được vào Pi 5.**
Đầu CSI trên Pi 5 nhỏ hơn đầu trên các đời Pi trước. Cáp đi kèm module camera
thường là loại cho đời cũ.
*Kiểm tra ngay:* mở hộp, đếm số chân trên đầu cáp và so với cổng CAM/DISP trên Pi 5.
Nếu không khớp → đặt mua cáp chuyển đúng loại, và chọn chiều dài đủ cho đường đi
từ Pi lên vị trí gắn camera cộng bán kính uốn.
*Ghi kết quả vào:* `docs/calibration.md` mục 5.

**R-CAM-2 — driver camera trên Ubuntu có thể không nhận cảm biến.**
Không phải mọi cảm biến camera đều được hỗ trợ bởi bản libcamera phổ thông cài qua
apt. Triệu chứng điển hình là thông báo không tìm thấy camera, rất dễ bị hiểu nhầm
thành hỏng phần cứng hoặc hỏng cáp.
*Kiểm tra ngay:* sau khi có cáp đúng, thử liệt kê camera ở tầng thấp nhất trước khi
đụng tới bất kỳ package ROS nào. Chỉ khi tầng đó thấy cảm biến mới đi tiếp lên ROS.
*Đường lui:* chuẩn bị một thẻ microSD thứ hai cài Raspberry Pi OS. Trên đó camera
chạy với công cụ có sẵn, dùng để phân biệt "lỗi driver/OS" với "lỗi phần cứng"
trong vài phút thay vì vài giờ. Thẻ này đồng thời là backup nếu thẻ chính hỏng
filesystem.

Cả hai rủi ro chỉ được gỡ khỏi mục này khi có evidence thật, không phải khi
"chắc là ổn".

## KNOWN ISSUES

- Chưa ghi nhận issue runtime mới sau mốc SSH/network.

## Cách cập nhật

Khi một checkpoint PASS, ghi thêm evidence ngắn, ví dụ:

```text
- [x] `/scan` 7.1–7.4 Hz trong 60 s, 2026-09-01.
```

Không tick chỉ vì code build thành công.
