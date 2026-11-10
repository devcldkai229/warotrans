# Cảm biến điện áp pin (voltage-only SOC)

> **Trạng thái: CHƯA LẮP, CHƯA HIỆU CHUẨN.**
> - Phần mềm đã có đủ: firmware, ROS 2, gateway, backend và UI.
> - Đã chạy test giả lập, nhưng **chưa có số đo phần cứng nào**.
> - Robot thật hiện báo `UNAVAILABLE` (`batteryPercent: null`).
> - Không được ghi "VERIFIED" ở bất kỳ đâu cho tới khi làm xong V3–V5 bên dưới.

## 1. Phần cứng được dùng (và không dùng)

**Pack pin:**
- 6 × Samsung INR21700-50S ghép 3S2P, Li-ion.
- 12.6 V khi đầy, 10.8–11.1 V danh định, khoảng 10 Ah (~108 Wh).
- BMS 3S 30 A loại thường: chỉ bảo vệ, cân bằng cell, **không xuất dữ liệu**.
- Ngõ ra XT60, cổng sạc 12.6 V riêng (sạc 2–3 A).

**Cảm biến:** module "DC 0–25 V voltage sensor". Đây chỉ là một cầu chia áp 30 kΩ / 7.5 kΩ, tỉ lệ danh định ≈ 1/5.

**Không dùng:**
- Cảm biến dòng (INA226, shunt), Smart BMS, cảm biến nhiệt.
- Vì vậy `current`, `charge` và `temperature` luôn là NaN.

**Đây không phải mạch bảo vệ.**
- SOC chỉ là con số để hiển thị và để quyết định có giao Job hay không.
- Việc chống quá xả, quá dòng, ngắn mạch vẫn hoàn toàn do BMS đảm nhận.

## 2. Chân ESP32-S3: GPIO1 (ADC1_CH0)

**Bằng chứng:** đối chiếu với pinout chính thức của MakerEdu.
- Nguồn: repo `makereduvn/MKE-K01-ESP32-S3-DEV-KIT`, file `extras/MKE-K01_1.png`.
- Board được mô tả là "tương thích ESP32-S3 DevKitC-1", dùng module ESP32-S3-WROOM-1 N16R8.

| Kiểm tra | Kết quả |
|---|---|
| GPIO1 có ra header? | Có: header **phải**, ngay dưới `RX` (GPIO44), silkscreen ghi `1` |
| Chức năng | `GPIO1 · RTC · TOUCH1 · ADC1_0` |
| Có thuộc ADC1 không? | Có. ADC2 xung đột với Wi-Fi, ADC1 thì không |
| Chân strapping? | Không. Strapping là 0, 3, 45, 46 (BOOT = GPIO0) |
| Nối linh kiện onboard? | Không. LED RGB dùng GPIO48; RESET/BOOT không dùng GPIO1 |
| Flash/PSRAM? | Không. N16R8 dùng GPIO35/36/37 |
| USB? | Không. USB D+/D− là GPIO20/19; firmware dùng `CDCOnBoot=cdc` |
| Trùng chân firmware đang dùng? | Không. MDD10A dùng GPIO4–7, encoder dùng GPIO8–11 |

**Dự phòng:** GPIO2 (ADC1_1). Nếu đổi chân, sửa `battery_adc_pin` trong firmware.

**Trước khi hàn, kiểm tra lại trên board thật (V0):** silkscreen `1` nằm cạnh `RX`, và GPIO1 không bị dây nào khác chiếm. Ảnh pinout chỉ là tài liệu của nhà sản xuất, chưa phải board trên robot.

## 3. Đi dây và yêu cầu an toàn

```
Pack P+ ──(sau công tắc nguồn chính / cầu chì)── VCC ┐
                                                     │  module 0–25 V (30k/7.5k)
Pack P− ─────────────────────────────────────── GND ┘
                                     S ──[1 kΩ]──┬── GPIO1 (ADC1_CH0)
                                                 └─[100 nF]── GND ESP32 (sát chân)
                                     − (module) ───────────── GND ESP32
                                     + (module) ── KHÔNG NỐI
```

1. **Cắt nguồn và kê bánh khỏi mặt đất** trước khi đấu dây. Cầu chì và XT60 phải được tháo hoặc ngắt.
2. Đầu vào module lấy ở **P+/P− của pack, sau công tắc chính**:
   - khi tắt robot, module không rút dòng ≈ 0.34 mA (12.6 V / 37.5 kΩ);
   - module đo đúng điện áp mà tải nhìn thấy.
3. **Chung mass bắt buộc:**
   - `−` của module nối GND ESP32;
   - GND ESP32 phải cùng điểm với P− (qua buck của Pi hoặc dây riêng).
   - Nếu không chung mass, số đo vô nghĩa và có thể làm hỏng chân ADC.
4. Đặt **điện trở 1 kΩ nối tiếp** và **tụ 100 nF xuống GND sát chân GPIO1**: hạn dòng khi lỡ quá áp, đồng thời lọc nhiễu PWM motor.
5. Tuỳ chọn: diode Schottky kẹp từ GPIO1 lên 3V3.
6. **Biên điện áp:**

   | Pack | Điện áp ở chân |
   |---|---|
   | 12.6 V | ≈ 2.52 V |
   | 9.0 V | ≈ 1.80 V |
   | 16.5 V | 3.3 V (giới hạn tuyệt đối) |

   - Firmware dùng attenuation 11 dB (= 12 dB trong IDF 5), dải ≈ 0–3.1 V.
   - **Không bao giờ** nối module vào nguồn trên 15 V: bộ sạc lỗi, nguồn bench đặt sai…
7. Không đổi thứ tự 3 chân S/+/−. Dò bằng đồng hồ trước khi cắm.
8. Đi dây xa dây motor và dây MDD10A, dùng dây xoắn đôi S/GND nếu được.

## 4. Luồng dữ liệu

```
GPIO1 ─ firmware "B <pin_mV> <n> <millis>" (200 ms, BATTERY_ADC_ENABLED 1)
  → esp32_bridge_node → /battery/adc_sample (warotrans_msgs/BatteryAdcSample)
  → battery_monitor_node (warotrans_hardware/battery_soc.py, config/battery.yaml)
      → /battery_state (sensor_msgs/BatteryState): percentage 0–1 | NaN, current NaN
      → /battery/estimate (warotrans_msgs/BatteryEstimate): mode, confidence, charging_state, fault
  → robot_gateway → MQTT telemetry: batteryPercent | null, batteryVoltage, batteryEstimationMode,
                    batteryConfidence, batteryChargingState=UNKNOWN, batteryMeasuredAt
  → backend Fleet (Fleet:Battery) → Web / App
```

- **Firmware** chỉ cộng dồn `analogReadMilliVolts` (đã hiệu chuẩn eFuse) rồi lấy trung bình. Mọi bước quy đổi đều làm trên Pi, nên hiệu chuẩn lại không cần flash firmware.
- **Mặc định `BATTERY_ADC_ENABLED 0`:** chưa lắp module thì không gửi dòng `B` nào.
  - Lý do: chân ADC để trống (floating) sẽ ra số rác.
  - Khi đó node báo `UNAVAILABLE / no_samples`.

## 5. Thuật toán (voltage-only)

1. **Hiệu chuẩn:**
   - `pack_V = divider_gain · pin_V + offset_v`;
   - hoặc dùng bảng nhiều điểm `calibration_pin_volts` / `calibration_pack_volts` (nội suy theo khoảng).
2. **Kiểm tra lỗi** (khi lỗi thì trả UNAVAILABLE, không trả số):
   - chân < 100 mV: `adc_disconnected`;
   - chân > 3050 mV: `adc_saturated`;
   - pack ngoài 8.0–13.5 V: `out_of_range`;
   - không có mẫu quá 2 s: `stale`;
   - chưa đủ cửa sổ median: `warming_up`.
   - Sau một lỗi, bộ lọc được xoá và phải nạp lại từ đầu.
3. **Lọc:** median 5 mẫu (loại gai), sau đó EMA với τ = 5 s.
4. **Bù sụt áp:** `OCV ≈ V + idle_sag_v + sag_v_per_mps·|v| + sag_v_per_radps·|ω|`.
   - Tốc độ lấy từ `/cmd_vel` và `/odom`, là proxy tải duy nhất vì không có cảm biến dòng.
5. **Bảng OCV→SOC theo cell** (`pack / 3`):
   - Tra theo đường cong riêng của INR21700-50S, **không** nội suy tuyến tính cả dải 3.0–4.2 V.
   - Ví dụ: 3.60 V ≈ 26 %. Tuyến tính sẽ cho 50 %.
6. **Chống nhảy SOC:**
   - Robot đang chạy hoặc vừa dừng (< `rest_settle_s` = 120 s):
     - SOC **chỉ được giảm**, tối đa `max_drop_pct_per_min`;
     - SOC không bao giờ tăng, nên áp hồi lên khi motor dừng không làm % nhảy lên;
     - confidence `LOW`.
   - Robot đã đứng yên đủ lâu (RESTED):
     - SOC tiến dần về OCV theo cả hai chiều, giới hạn `rest_slew_pct_per_min`;
     - muốn tăng phải vượt `rest_hysteresis_pct`;
     - confidence `MEDIUM`.
7. **Trạng thái sạc:**
   - Luôn là `UNKNOWN`. **Không suy đoán** CHARGING từ điện áp tăng.
   - Không có tín hiệu độc lập nào xác nhận việc sạc, và trạng thái này không bao giờ đổi `Robot.Status`.
8. **Lưu trạng thái:**
   - Ghi atomic ra `~/.local/state/warotrans/battery_soc.json` mỗi 30 s, hoặc khi SOC đổi ≥ 1 %.
   - Lúc khởi động, chỉ khôi phục khi file chưa quá 12 h **và** lệch ≤ 15 % so với số đo đầu tiên.
   - Giá trị khôi phục mang confidence `LOW` cho tới khi robot RESTED.

Mọi tham số đều nằm trong `ros2_ws/src/warotrans_hardware/config/battery.yaml`. Giá trị hiện tại là **giá trị khởi đầu**.

## 6. Quy trình kiểm chứng phần cứng (làm sau khi mua và lắp)

**Quy tắc chung:**
- Người vận hành có mặt.
- Robot kê bánh khỏi mặt đất ở V1–V5.
- V6 chỉ do người vận hành lái.
- Mỗi bước ghi kết quả (ngày, lệnh, số đo) vào `docs/STATUS.md` hoặc `docs/calibration.md`.

| Bước | Làm gì | Đạt khi |
|---|---|---|
| **V0 — Pinout** | Chụp board thật, xác nhận GPIO1 (silkscreen `1` cạnh `RX`) không bị chiếm. Lưu ảnh vào `docs/hardware/`. | Có ảnh và ghi chú |
| **V1 — Tỉ lệ module** | Chưa nối ESP32. Cấp 3 mức điện áp vào module (pack thật hoặc nguồn bench ≤ 13 V), đo `S` bằng đồng hồ. | Tỉ lệ đo được (ví dụ 4.98–5.02) → ghi `divider_gain` |
| **V2 — Lắp và bật firmware** | Tắt nguồn, đấu dây theo §3. Sửa `BATTERY_ADC_ENABLED 1`, compile (`arduino-cli compile --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc`), flash. Đọc serial: các dòng `B <mV> <n> <ms>` mỗi 200 ms, `n` ≈ 40. | Dòng `B` đều đặn; dòng `O` vẫn 20 ms; teleop vẫn bình thường |
| **V3 — Hiệu chuẩn** | Đo pack tại XT60 bằng đồng hồ ở 3 mức (≈ 12.5 / 11.4 / 10.5 V hoặc nguồn bench), cùng lúc đọc `ros2 topic echo /battery/adc_sample`. Ghi `divider_gain`/`offset_v`, hoặc bảng `calibration_*`. | Sai số pack ≤ ±50 mV ở cả 3 mức |
| **V4 — Nhiễu** | Robot bật, đứng yên 10 phút. Ghi `/battery_state.voltage`. | Độ lệch chuẩn sau lọc < 10 mV; không có `fault` |
| **V5 — Đường OCV** | Sạc đầy, nghỉ 1 h, ghi áp. Xả từng nấc (chạy hoặc để tải), mỗi nấc nghỉ ≥ 30 phút rồi ghi áp, tới ≈ 10.5 V. So với bảng `ocv_*`, chỉnh YAML. | Có bảng OCV đo thật thay cho bảng khởi đầu |
| **V6 — Sụt áp** | (a) Đo pack khi robot tắt so với khi bật mà đứng yên → `idle_sag_v`. (b) Người vận hành teleop ở vài vận tốc cố định (v, ω), ghi độ sụt áp → `sag_v_per_mps`, `sag_v_per_radps`. (c) Đo thời gian áp hồi sau khi dừng → `rest_settle_s`. | SOC lúc chạy và SOC sau khi nghỉ lệch ≤ 5 % |
| **V7 — Lỗi cảm biến** | Rút dây `S` khi đang chạy. | Trong 2 s chuyển `UNAVAILABLE` (`adc_disconnected`); web/app hiện "Unknown"; dispatch bị chặn |
| **V8 — End-to-end** | `ros2 topic echo /battery/estimate` → MQTT `telemetry` có `batteryEstimationMode: VOLTAGE_ONLY` → web/app hiện %, V, confidence → Start Job được khi SOC ≥ `MinDispatchPercent`, bị chặn khi dưới ngưỡng. | Đủ cả chuỗi |

**Chỉ sau V3–V5** mới đổi `docs/STATUS.md` từ "CHƯA LẮP / CHƯA HIỆU CHUẨN" sang "đã hiệu chuẩn (ngày…)".

## 7. Giới hạn đã biết

- **Không đo dòng**, nên SOC lúc chạy chỉ là heuristic: bù sụt áp theo vận tốc và chặn tốc độ giảm. Độ tin cậy cao nhất (`MEDIUM`) chỉ có khi robot đã đứng yên đủ lâu.
- **Không phát hiện được việc sạc.**
  - Khi cắm sạc, áp đo cao hơn OCV, nên SOC có thể bị ước cao hơn thật.
  - Đó là lý do trạng thái sạc luôn `UNKNOWN`, và tốc độ tăng của SOC bị chặn.
- Không đo nhiệt và không đo từng cell, nên không phát hiện được cell lệch. BMS lo cân bằng cell.
- Bảng OCV và mọi hệ số sụt áp hiện **chưa hiệu chuẩn**.

## 8. Còn chặn (blockers)

1. Mua và lắp module, rồi làm V0–V8.
2. Bật `BATTERY_ADC_ENABLED 1` và flash firmware (chỉ làm ở bước V2, có người vận hành).
3. Trên backend production, `Fleet:Battery:AllowUnknownBatteryDispatch` = `false`, nên robot thật **không được tự giao Job** cho tới khi có số đo pin hợp lệ. Khi test có giám sát thì bật rõ ràng bằng biến môi trường:
   ```
   Fleet__Battery__AllowUnknownBatteryDispatch=true
   ```
