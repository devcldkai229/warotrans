# Hai robot: trạng thái và cách bật

Viết cho người chạy lab. Mọi thứ dưới đây đã chạy trên backend thật với robot mô phỏng, **chưa chạy trên hai robot thật**.

**Tóm tắt:** hai robot chạy an toàn theo kiểu lần lượt: robot sau chạy tới trước xung đột rồi chờ. **Song song hai lane trên MAP-ONE-V004 hiện chưa đạt với plan Nav2 thật** (mục "Giới hạn đã đo"), dù harness với đường đi lý tưởng đạt.

## Hai chế độ

| Thiết lập backend | Mặc định | Tác dụng |
|---|---|---|
| `Traffic__SemanticPrefixGrant` | `true` (profile RealRobot) | Khi có xung đột, robot được cấp phần đường tới trước xung đột, chạy tới đó rồi chờ. Không dừng trong NO_STOP, không dừng trong vùng single chưa vào, không tiến vào phần đường robot kia còn cần |
| `Traffic__SemanticLaneHalfWidthMeters` | `0` (tắt) | Đoạn thẳng giữ chỗ bằng hình chữ nhật thay vì hình tròn, nên hai lane cạnh nhau không chung ô |

Chạy song song hai lane cần thêm cả hai dòng, **trên backend và không cần đổi gì ở robot** (hình học gửi kèm trong mỗi lần cấp quyền):

```
Traffic__SemanticLaneHalfWidthMeters=0.26
Traffic__SemanticCellSizeMeters=0.1
```

Khi đang chạy một robot, giữ tắt. Lý do ở mục "Rủi ro".

## Kiểm chứng

`tools/traffic_e2e/test_dual_lane_harness.py`, backend thật + Mosquitto thật + logic gateway thật, robot mô phỏng:

| Kịch bản | Kỳ vọng |
|---|---|
| Hai lane ngược chiều | Cùng được cấp quyền, không ai chờ ai, cách nhau ≥ 0,6 m |
| Vùng single | Robot sau chạy tới sát vùng rồi chờ, thân không vào vùng khi robot trước còn giữ |
| Nối đuôi trên một lane | Robot sau luôn cách robot trước ≥ 0,36 m |
| Cắt ngang | Chờ trước đường của robot kia |
| Đối đầu trên một lane | Không chạm nhau, không robot nào treo mãi mà không có kết quả |

Unit test: `SemanticPrefixGrantTests.cs`. C# và Python dùng chung `semantic_oriented_vectors.json`.

## Giới hạn đã đo

Đưa plan Nav2 thật của cả 90 chặng (lượt rig `all90c`) qua đúng hàm `collect_resources` của gateway, rồi so ô ở khúc giữa của từng cặp lane đối diện:

| Cặp lane | Tổ hợp plan chung ô |
|---|---|
| LTR-B và RTL-B | 256/256 |
| BTT-C và TTB-C | 441/441 |
| LTR-A và RTL-PARKING | 529/529 |

Kết quả không đổi khi thử ô 0,1 và 0,05 m với nửa bề ngang 0,22, 0,24 và 0,26 m, nên đổi thông số không giải quyết được. Nguyên nhân có khả năng nhất, chưa kiểm chứng riêng: lane V004 chỉ dài khoảng 0,8–1,9 m, còn vùng giữ chỗ hình tròn khi rẽ có bán kính khoảng 0,4 m nên phủ gần hết khúc giữa. Harness đạt vì đường đi của nó nằm ngay tim lane.

Thực tế nên chờ đợi khi bật hai dòng cấu hình trên: an toàn và robot không bị đứng vì gate (10/10 chặng trên rig, một robot), nhưng robot vẫn chờ nhau như chế độ lần lượt. Muốn song song thật cần thiết kế lại cách giữ chỗ ở đoạn rẽ, nên làm sau khi RBT-001 đã ổn.

Lane một chiều cũng khiến V004 không có đường vòng: thử chắn ngang giữa từng lane trong 7 lane chính, không cặp endpoint nào có đường hợp lệ dài hơn đáng kể (dài thêm trên 1 m). Cặp bị ảnh hưởng thì mất đường hẳn, nên robot chờ 60 giây rồi job PAUSED. Vật cản nhỏ ở rìa lane vẫn đi qua được.

## Rủi ro chỉ đo được tại lab

- **Gate chặn vì lệch ngang.** Với hình chữ nhật, thân robot chỉ được lệch khoảng 9 cm khỏi đường đã xin (0,26 − 0,15 − 0,02). Vượt là gate đóng và robot đứng. Đây là cơ chế an toàn nhưng AMCL dao động nhiều sẽ làm robot dừng lặp lại. Phải đo trên robot thật trước khi tin.
- **Đường đi lệch về phía lane đối diện.** Hai hình chữ nhật cách nhau 0,15 m khi cả hai đúng tim lane, ô lưới 0,1 m. Nếu tổng lệch vượt khoảng 0,05 m thì hai bên chung ô và một robot sẽ chờ. An toàn, nhưng khi đó không song song được.
- **Hai robot thấy nhau qua lidar.** Costmap sẽ coi robot kia là vật cản: cách tim lane đối diện 0,67 m, chưa đo ảnh hưởng tới RPP.

## Phương án lùi

Đặt lại `SemanticLaneHalfWidthMeters=0` và `SemanticCellSizeMeters=0.25`: hai robot chạy lần lượt, an toàn nhưng không song song.

## Khác

- RBT-002 đỗ ở `EP-OUTBOUND`: `PARKING-A` và `PARKING-B` xung đột khi đỗ cùng lúc.
- Khởi động: `ROBOT_CODE=RBT-002 ROS_DOMAIN_ID=31 BROKER=<ip> ~/warotrans_tools/pi_run_semantic.sh start`.
- Vẫn nên có E-stop phần cứng cho từng xe khi chạy hai robot.
