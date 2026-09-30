# WaroTrans Tools

Thư mục này **cố ý gọn**. Không thêm script one-off / probe / recover vào đây —
chạy lệnh trực tiếp trên Pi khi debug.

## Giữ lại

| File | Việc |
|---|---|
| `warotrans-teleop-start.sh` | Join Wi‑Fi + `mobile_base` (teleop `:8080`) |
| `warotrans-teleop.service` | systemd unit boot |
| `install_teleop_autostart.sh` | Cài/enable service trên Pi (chạy 1 lần) |
| `warotrans-wifi.conf.example` | Mẫu `/home/waro/warotrans-wifi.conf` |
| `warotrans-stop-all.sh` | Tắt teleop service + mọi stack ROS |
| `save_map.sh` | Lưu map SLAM |
| `check.sh` + `validate_structure.py` | Quality gate trước deploy |
| `deploy.sh` / `deploy.ps1` | Đồng bộ source lên Pi |
| `doctor.sh` / `doctor.ps1` | Thu thập evidence trên robot |
| `rollback_source.sh` / `rollback.ps1` | Rollback source sau deploy hỏng |

## Phone teleop khi Pi boot

Trên Pi (một lần):

```bash
# 1) Wi-Fi / hotspot phone
cp ~/warotrans/tools/warotrans-wifi.conf.example ~/warotrans-wifi.conf
nano ~/warotrans-wifi.conf   # WIFI_SSID / WIFI_PASSWORD

# 2) Cần sudo nmcli không hỏi mật khẩu (User=waro trong systemd)
#    (nếu chưa có) thêm sudoers NOPASSWD cho /usr/bin/nmcli

# 3) Cài service
cd ~/warotrans   # hoặc path repo trên Pi
bash tools/install_teleop_autostart.sh
```

Sau reboot:

- Service `warotrans-teleop` tự chạy.
- Điện thoại cùng Wi‑Fi/hotspot: `http://<IP_Pi>:8080`
- IP: trên Pi `hostname -I`

```bash
sudo systemctl status warotrans-teleop
journalctl -u warotrans-teleop -f
sudo systemctl stop warotrans-teleop    # trước khi chạy Nav2 / mapping
sudo systemctl restart warotrans-teleop # sau khi sửa wifi.conf
```

**Không** chạy Nav2 cùng teleop (`/cmd_vel` xung đột).

## Nav2 / mapping

Dùng lệnh ROS trực tiếp, không qua script phụ:

```bash
bash ~/tools/warotrans-stop-all.sh
ros2 launch warotrans_bringup navigation.launch.py map:=$HOME/maps/warotrans.yaml
# hoặc mapping:
ros2 launch warotrans_bringup mapping.launch.py
bash ~/tools/save_map.sh ~/maps/warotrans
```
