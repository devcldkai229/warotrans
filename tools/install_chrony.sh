#!/usr/bin/env bash
# Keep the robot's wall clock on the backend laptop's time (Evidence: two robots' videos and paths on one UTC
# timeline; spec §6). The Pi has no RTC battery; without this its clock is whatever the last deploy set.
#
#   sudo tools/install_chrony.sh [SERVER]      SERVER defaults to the laptop on the ICS link, 192.168.137.1
#
# The laptop must answer NTP (Windows, admin PowerShell, once):
#   Set-ItemProperty HKLM:\SYSTEM\CurrentControlSet\Services\W32Time\TimeProviders\NtpServer -Name Enabled -Value 1
#   Set-ItemProperty HKLM:\SYSTEM\CurrentControlSet\Services\W32Time\Config -Name AnnounceFlags -Value 5
#   Restart-Service w32time; New-NetFirewallRule -DisplayName "NTP" -Direction Inbound -Protocol UDP -LocalPort 123 -Action Allow
#
# Check afterwards:  chronyc tracking   (Leap status: Normal; "System time" is the offset)
set -euo pipefail
SERVER="${1:-192.168.137.1}"
if [[ $EUID -ne 0 ]]; then echo "run with sudo"; exit 2; fi
apt-get install -y chrony >/dev/null
cat > /etc/chrony/conf.d/warotrans.conf <<EOF
# WaroTrans: the backend laptop first (same time as the Evidence Recorder), public pool when there is Internet.
server ${SERVER} iburst prefer minpoll 4 maxpoll 6
pool ntp.ubuntu.com iburst maxsources 2
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
