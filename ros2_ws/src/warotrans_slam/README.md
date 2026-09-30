# warotrans_slam

Owns SLAM Toolbox mapping configuration and launch.

Expected inputs:

```text
/scan
/odom
TF: odom -> base_footprint -> base_link -> laser
```

Expected outputs:

```text
/map
/map_metadata
TF: map -> odom
```

This package must not parse encoder serial data.

## Mode

`slam_toolbox` chạy ở **online asynchronous mapping**
(`async_slam_toolbox_node`, `mode: mapping`). Async được chọn vì Pi 5 phải chia
CPU với serial bridge, teleop và foxglove_bridge: khi quá tải, node bỏ scan cũ
thay vì tích luỹ độ trễ TF.

Node này là publisher **duy nhất** của `map -> odom`. Không thêm bất kỳ
static transform publisher nào cho `map -> base_link` hay `map -> base_footprint`.

## Build

Trên Raspberry Pi:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-slam-toolbox \
  ros-jazzy-nav2-map-server \
  ros-jazzy-foxglove-bridge \
  ros-jazzy-rplidar-ros

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y   # bỏ qua nếu DNS/rosdep lỗi
colcon build --symlink-install
source install/setup.bash
```

## Chạy mapping (Phase 8)

```bash
# Trên Pi — demo service phải OFF
sudo systemctl stop warotrans-demo   # nếu còn enabled/active

bash ~/bin/warotrans-mapping-start.sh
# hoặc: ros2 launch warotrans_bringup mapping.launch.py
```

Foxglove laptop: `ws://<Pi-IP>:8765`, Fixed frame **`map`**, topics `/map` + `/scan`.

Teleop: `http://<Pi-IP>:8080` — lái chậm, quét chu vi rồi vòng lại điểm xuất phát.

Lưu map (khi `/map` đang publish):

```bash
bash ~/tools/save_map.sh ~/maps/warotrans
```

Chỉ SLAM (khi base + LiDAR đã chạy sẵn ở terminal khác):

```bash
ros2 launch warotrans_slam slam.launch.py
```

Bật/tắt từng thành phần:

```bash
ros2 launch warotrans_bringup mapping.launch.py \
  start_teleop:=false start_foxglove:=false
```

## Foxglove trên laptop

Pi là Ubuntu Server, không cài desktop. `foxglove_bridge` mở websocket trên Pi,
laptop Windows chạy Foxglove và kết nối tới:

```text
ws://warotrans.local:8765
ws://10.42.0.1:8765     # khi Pi phát hotspot WaroTrans
```

Layout khuyến nghị — một panel **3D**:

| Mục | Giá trị |
|---|---|
| Fixed frame | `map` |
| Topic | `/map` |
| Topic | `/scan` |
| Hiển thị | TF (frames: `map`, `odom`, `base_footprint`, `base_link`, `laser`) |
| Hiển thị | RobotModel từ `/robot_description` nếu muốn |

Kết quả mong đợi là occupancy grid nhìn từ trên xuống:

```text
đen/xám đậm   vật cản, tường đã quan sát
trắng         vùng trống đã biết
xám           vùng chưa biết
```

cùng pose robot và scan LiDAR realtime nằm **đè khớp** lên tường.

## Verify trước khi tin map

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /map

ros2 topic echo /map --once

ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom laser

ros2 run tf2_tools view_frames
```

TF tree bắt buộc, không chấp nhận cây rời rạc:

```text
map
 └── odom
      └── base_footprint
           └── base_link
                └── laser
```

## Quy trình lái khi mapping

1. Bắt đầu ở vị trí dễ nhận diện, ghi nhớ điểm xuất phát.
2. Lái **chậm**; dùng mức speed thấp trên web teleop.
3. Đi thẳng mượt, vào cua bằng vòng cung rộng.
4. Tránh tăng/giảm tốc đột ngột và xoay tại chỗ không cần thiết (skid-steer
   trượt bánh khi xoay tại chỗ, đây là nguồn drift lớn nhất).
5. Giữ LiDAR luôn nhìn thấy tường/đặc trưng, không chạy giữa khoảng trống lớn.
6. Quét chu vi trước, sau đó mới vào các hành lang bên trong.
7. Đi lại vùng đã quét để scan matcher có dữ liệu chồng lặp.
8. Quay về gần điểm xuất phát và đi chậm để loop closure kịp kích hoạt.

## Khi thấy double wall — thứ tự debug

Double wall gần như luôn là lỗi input, **không** phải lỗi tham số SLAM.

```text
1. TF                     (tf2_echo, view_frames, cây có đứt không)
2. timestamp              (scan/odom có bị lệch thời gian không)
3. encoder sign/data      (tick có đúng dấu khi đi tới không)
4. wheel calibration      (ticks_per_rev, radius, separation)
5. LiDAR pose/yaw         (geometry.yaml so với số đo thật)
6. robot motion           (chạy quá nhanh, trượt bánh)
7. chỉ sau đó mới tune slam_toolbox
```

Không sửa `config/slam_toolbox.yaml` để che một trong 6 mục đầu.

## Lưu map

```bash
mkdir -p ~/maps

ros2 run nav2_map_server map_saver_cli \
  -f ~/maps/warotrans \
  --ros-args -p save_map_timeout:=10000.0
```

hoặc dùng wrapper (từ chối ghi đè im lặng):

```bash
bash tools/save_map.sh ~/maps/warotrans
```

Kết quả:

```text
~/maps/warotrans.pgm
~/maps/warotrans.yaml
```

## Ghi bag để debug

```bash
mkdir -p ~/bags && cd ~/bags

ros2 bag record \
  /scan \
  /odom \
  /tf \
  /tf_static \
  /cmd_vel \
  /map
```

Bag phải nằm ngoài cây source và không được commit (`.gitignore` đã chặn
`bags/`, `*.db3`, `*.mcap`).

## Map quality gate

Phase 8 chỉ PASS khi **tất cả** đúng:

- [ ] `/map` publish liên tục.
- [ ] `map -> odom` tồn tại và không nhảy bất thường.
- [ ] Tường chính một lớp, không double wall rõ rệt.
- [ ] Kích thước phòng gần số đo thật.
- [ ] Scan overlay bám vào tường trong map.
- [ ] Quay về điểm xuất phát cho loop closure hợp lý.
- [ ] Map lưu được và load lại được.

`/map` tồn tại **không** đồng nghĩa Phase 8 PASS.

## Cảnh báo calibration

`wheel.separation = 0.4535` (effective).
`ticks_per_rev` L/R = 2478.1; `wheel_radius` L/R = 0.03296 / 0.03263
(operator measure 2026-09-07 @ 15% — xem `docs/calibration.md`).

Triệu chứng rẽ sai: tường cong/double — sửa separation/radius, **không** tune
`slam_toolbox` trước.