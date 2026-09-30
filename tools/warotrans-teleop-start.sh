#!/usr/bin/env bash
# =============================================================================
# WaroTrans phone teleop — boot entrypoint
# =============================================================================
# Join Wi-Fi from /home/waro/warotrans-wifi.conf, wait for ESP32, then
# launch mobile_base (description + hardware + web teleop :8080).
#
# Phone (same Wi-Fi / hotspot): http://<pi-ip>:8080
# Do NOT run Nav2 / mapping at the same time (/cmd_vel conflict).
# =============================================================================
set -eo pipefail
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source /home/waro/ros2_ws/install/setup.bash
set -u

WIFI_CONF="${WAROTRANS_WIFI_CONF:-/home/waro/warotrans-wifi.conf}"

join_wifi_from_conf() {
  if [[ ! -f "${WIFI_CONF}" ]]; then
    echo "WARN: ${WIFI_CONF} missing — skip Wi-Fi join"
    return 0
  fi
  set +u
  # shellcheck source=/dev/null
  source "${WIFI_CONF}"
  set -u
  if [[ -z "${WIFI_SSID:-}" || -z "${WIFI_PASSWORD:-}" ]]; then
    echo "WARN: WIFI_SSID/WIFI_PASSWORD empty in ${WIFI_CONF}"
    return 0
  fi
  sudo nmcli radio wifi on >/dev/null 2>&1 || true
  sudo nmcli device set wlan0 managed yes >/dev/null 2>&1 || true
  cur="$(sudo nmcli -t -f ACTIVE,SSID device wifi | awk -F: '$1=="yes"{print $2; exit}')"
  if [[ "${cur}" == "${WIFI_SSID}" ]]; then
    echo "Wi-Fi already on ${WIFI_SSID}"
    return 0
  fi
  sudo nmcli device wifi rescan >/dev/null 2>&1 || true
  sleep 3
  if sudo nmcli -t -f NAME connection show | grep -Fxq "${WIFI_SSID}"; then
    sudo nmcli connection modify "${WIFI_SSID}" wifi-sec.psk "${WIFI_PASSWORD}" 2>/dev/null || true
    sudo nmcli connection up "${WIFI_SSID}" && return 0
  fi
  sudo nmcli device wifi connect "${WIFI_SSID}" password "${WIFI_PASSWORD}" || \
    echo "WARN: failed to join Wi-Fi '${WIFI_SSID}' — continue anyway"
}

join_wifi_from_conf

echo "Waiting for /dev/warotrans ..."
for _ in $(seq 1 45); do
  [[ -e /dev/warotrans ]] && break
  sleep 1
done
if [[ ! -e /dev/warotrans ]]; then
  echo "WARN: /dev/warotrans not found — teleop may have no motors"
fi

IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
echo "Teleop UI: http://${IP:-<pi-ip>}:8080"
echo "Also try:  http://warotrans.local:8080"

set +u
exec ros2 launch warotrans_bringup mobile_base.launch.py
