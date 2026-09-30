#!/usr/bin/env bash
# Install / enable WaroTrans phone-teleop boot service on the Raspberry Pi.
# Run once on Pi (from repo or after copying tools/):
#   bash tools/install_teleop_autostart.sh
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
START_SRC="${ROOT}/tools/warotrans-teleop-start.sh"
UNIT_SRC="${ROOT}/tools/warotrans-teleop.service"
SUDO_PASS="${SUDO_PASS:-12}"

if [[ ! -f "${START_SRC}" || ! -f "${UNIT_SRC}" ]]; then
  START_SRC="${HOME}/tools/warotrans-teleop-start.sh"
  UNIT_SRC="${HOME}/tools/warotrans-teleop.service"
fi

if [[ ! -f "${START_SRC}" || ! -f "${UNIT_SRC}" ]]; then
  echo "ERROR: missing teleop start script or unit file" >&2
  exit 1
fi

install -d "${HOME}/bin" "${HOME}/tools"
install -m 0755 "${START_SRC}" "${HOME}/bin/warotrans-teleop-start.sh"
install -m 0644 "${UNIT_SRC}" "${HOME}/tools/warotrans-teleop.service"
# Keep a copy under ~/tools for edits
install -m 0755 "${START_SRC}" "${HOME}/tools/warotrans-teleop-start.sh"

printf '%s\n' "${SUDO_PASS}" | sudo -S usermod -aG dialout waro || true

# Disable old demo boot unit if present (same /cmd_vel / serial conflict)
printf '%s\n' "${SUDO_PASS}" | sudo -S systemctl disable --now warotrans-demo.service 2>/dev/null || true

printf '%s\n' "${SUDO_PASS}" | sudo -S install -m 0644 \
  "${HOME}/tools/warotrans-teleop.service" \
  /etc/systemd/system/warotrans-teleop.service

printf '%s\n' "${SUDO_PASS}" | sudo -S systemctl daemon-reload
printf '%s\n' "${SUDO_PASS}" | sudo -S systemctl enable warotrans-teleop.service
printf '%s\n' "${SUDO_PASS}" | sudo -S systemctl restart warotrans-teleop.service

echo "Enabled warotrans-teleop.service"
systemctl --no-pager status warotrans-teleop.service | head -25 || true
echo
IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
echo "Phone UI: http://${IP:-<pi-ip>}:8080"
echo "Logs:     journalctl -u warotrans-teleop -f"
echo "Stop:     sudo systemctl stop warotrans-teleop   # before Nav2/mapping"
echo "Wifi:     edit /home/waro/warotrans-wifi.conf then: sudo systemctl restart warotrans-teleop"
