#!/usr/bin/env bash
# Keep the robot's wall clock on the backend laptop's time (Evidence: two robots' videos and paths on one UTC
# timeline; spec §6). The Pi has no RTC battery; without this its clock is whatever the last deploy set.
#
#   sudo tools/install_chrony.sh [SERVER]      SERVER: the laptop by NAME (default LAPTOP-Q0QAV6GM.local, or
#                                              WARO_BACKEND_HOST); chrony re-resolves it when the laptop's IP changes.
# Sources, in order: the laptop (same clock as the Recorder), the laptop on the ICS cable (192.168.137.1), and the
# public pool when the Wi-Fi has Internet. The laptop itself follows Internet time, so all of them agree on UTC.
#
# The laptop must answer NTP: run tools/laptop_lab_network.ps1 once as admin (W32Time server + firewall).
#
# Check afterwards:  chronyc tracking   (Leap status: Normal; "System time" is the offset)
set -euo pipefail
SERVER="${1:-${WARO_BACKEND_HOST:-LAPTOP-Q0QAV6GM.local}}"
if [[ $EUID -ne 0 ]]; then echo "run with sudo"; exit 2; fi
apt-get install -y chrony >/dev/null
cat > /etc/chrony/conf.d/warotrans.conf <<EOF
# WaroTrans: the backend laptop (same clock as the Evidence Recorder) and the public pool; chrony picks the source with
# the smallest error, so the pool when there is Internet (the laptop reports 10 s), the laptop alone otherwise.
server ${SERVER} iburst minpoll 4 maxpoll 6
server 192.168.137.1 iburst minpoll 4 maxpoll 6
pool ntp.ubuntu.com iburst maxsources 2
# Windows W32Time always announces a 10 s root dispersion (seen 2026-11-17, while within 2 ms of the pool); the default
# maxdistance (3 s) would reject the laptop, which is the only source in a room without Internet. With Internet the
# pool still wins (smaller distance); with the laptop alone the agent's error bound says honestly it is uncertain.
maxdistance 16.0
# Step the clock at boot if it is far off (no RTC), slew afterwards (never jump during a run).
makestep 1.0 3
EOF
systemctl disable --now systemd-timesyncd 2>/dev/null || true
systemctl enable --now chrony
systemctl restart chrony
sleep 3
chronyc -n sources || true
chronyc tracking || true
echo "chrony configured against ${SERVER}"
