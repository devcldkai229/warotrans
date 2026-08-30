#!/usr/bin/env bash
# WaroTrans runtime diagnostics. Chay TREN Raspberry Pi.
# Usage: doctor.sh [idle|base|mapping|navigation]
set -u
MODE="${1:-idle}"

ok(){ echo "[OK ] $*"; }
warn(){ echo "[WARN] $*"; }
info(){ echo "[INFO] $*"; }
has(){ command -v "$1" >/dev/null 2>&1; }

echo "WaroTrans Doctor — mode=${MODE}"
echo "======================================"

echo
echo "[SYSTEM]"
printf "hostname: "; hostname 2>/dev/null || true
printf "kernel:   "; uname -r 2>/dev/null || true
printf "ip:       "; hostname -I 2>/dev/null || true
printf "uptime:   "; uptime -p 2>/dev/null || true

if [[ -r /sys/class/thermal/thermal_zone0/temp ]]; then
  awk '{printf "CPU temp: %.1f C\n", $1/1000}' /sys/class/thermal/thermal_zone0/temp
fi

if has vcgencmd; then
  info "$(vcgencmd get_throttled 2>/dev/null || true)"
else
  info "vcgencmd khong co tren image nay; bo qua Pi firmware health command"
fi

if grep -q 'usb_max_current_enable=1' /boot/firmware/config.txt 2>/dev/null; then
  info "usb_max_current_enable=1 present"
else
  info "usb_max_current_enable=1 khong thay; day KHONG phai fatal. Chi xem xet neu USB power co trieu chung."
fi

echo
echo "[DEVICES]"
id -nG | grep -qw dialout && ok "user thuoc dialout" || warn "user chua thuoc dialout"

for dev in /dev/rplidar /dev/warotrans; do
  if [[ -e "$dev" ]]; then
    ok "$dev -> $(readlink -f "$dev" 2>/dev/null || echo '?')"
  else
    warn "$dev khong ton tai"
  fi
done

info "USB devices:"
lsusb 2>/dev/null | sed 's/^/  /' || true

echo
echo "[ROS 2]"
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  warn "ROS 2 Jazzy chua cai"
  exit 0
fi
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
[[ -f "$HOME/ros2_ws/install/setup.bash" ]] && source "$HOME/ros2_ws/install/setup.bash" 2>/dev/null || true

for pkg in rplidar_ros slam_toolbox; do
  ros2 pkg prefix "$pkg" >/dev/null 2>&1 && ok "$pkg installed" || warn "$pkg missing"
done

if ros2 pkg prefix warotrans_bringup >/dev/null 2>&1; then ok "warotrans_bringup built"; else info "warotrans_bringup chua build/chua co"; fi

info "nodes:"
timeout 4 ros2 node list 2>/dev/null | sed 's/^/  /' || true
info "topics:"
timeout 4 ros2 topic list 2>/dev/null | sed 's/^/  /' || true

check_topic(){
  local topic="$1"
  if timeout 3 ros2 topic info "$topic" >/dev/null 2>&1; then ok "topic $topic exists"; else warn "topic $topic missing"; fi
}

case "$MODE" in
  idle)
    ;;
  base)
    check_topic /odom
    ;;
  mapping)
    check_topic /scan
    check_topic /odom
    check_topic /map
    echo "TF map -> base_footprint (best effort):"
    timeout 4 ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | head -8 || true
    ;;
  navigation)
    check_topic /scan
    check_topic /odom
    check_topic /map
    check_topic /cmd_vel
    echo "TF map -> base_footprint (best effort):"
    timeout 4 ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | head -8 || true
    ;;
  *)
    warn "Mode khong hop le. Dung idle|base|mapping|navigation"
    ;;
esac

echo
echo "Doctor chi thu thap evidence; khong tu ket luan calibration/hardware."
