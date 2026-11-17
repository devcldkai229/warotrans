#!/usr/bin/env bash
# Robot network that survives changing IPs (run ON the Pi with sudo). Idempotent; run again to ADD another Wi-Fi.
#
#   sudo WIFI_PSK='<password>' tools/pi_network_setup.sh --wifi "<SSID>" [--eth-fallback 192.168.137.50/24]
#   sudo tools/pi_network_setup.sh --confirm        keep the change (otherwise it is rolled back after 180 s)
#   sudo tools/pi_network_setup.sh --packages       install avahi + libnss-mdns + chrony (needs Internet)
#
# What it does:
#   * Wi-Fi: one saved profile per SSID, autoconnect; every saved network stays, the Pi joins whichever is in range.
#     The password comes from the environment (never argv, never a repo) and is stored only in the root-only
#     NetworkManager keyfile or netplan file.
#   * Wi-Fi is the default route (metric 100); eth0 keeps DHCP (laptop ICS, metric 600) PLUS a fixed fallback address,
#     so `ssh waro@<fallback>` over the cable works even when ICS DHCP does not answer.
#   * names instead of addresses: avahi publishes <hostname>.local, libnss-mdns lets the robot resolve the laptop's
#     name (MQTT, Recorder, chrony).
#   * safety: the old configuration is backed up first and restored automatically after 180 s unless --confirm.
set -euo pipefail
[[ $EUID -eq 0 ]] || { echo "run with sudo"; exit 2; }

SSID=""; FALLBACK=""; CONFIRM=0; PACKAGES=0; ROLLBACK_S=180
while [[ $# -gt 0 ]]; do
  case "$1" in
    --wifi) SSID="$2"; shift 2 ;;
    --eth-fallback) FALLBACK="$2"; shift 2 ;;
    --confirm) CONFIRM=1; shift ;;
    --packages) PACKAGES=1; shift ;;
    --rollback-s) ROLLBACK_S="$2"; shift 2 ;;
    *) echo "unknown argument $1"; exit 2 ;;
  esac
done
UNIT=warotrans-net-rollback
BACKUP_DIR=/root/warotrans_network_backups

if [[ $CONFIRM == 1 ]]; then
  systemctl stop "$UNIT.timer" 2>/dev/null || true
  systemctl reset-failed "$UNIT.service" 2>/dev/null || true
  echo "network change kept (rollback cancelled)"
  exit 0
fi

if [[ $PACKAGES == 1 ]]; then
  apt-get update -qq
  DEBIAN_FRONTEND=noninteractive apt-get install -y -qq avahi-daemon libnss-mdns chrony netcat-openbsd >/dev/null
  systemctl enable --now avahi-daemon
  grep -q mdns4_minimal /etc/nsswitch.conf && echo "mDNS name resolution: on" || echo "WARN: mdns4_minimal missing in /etc/nsswitch.conf"
  exit 0
fi

[[ -n "$SSID" ]] || { echo "--wifi <SSID> is required"; exit 2; }
[[ -n "${WIFI_PSK:-}" ]] || { echo "WIFI_PSK must be set in the environment"; exit 2; }
[[ ${#WIFI_PSK} -ge 8 ]] || { echo "WIFI_PSK shorter than 8 characters"; exit 2; }

if systemctl is-active --quiet NetworkManager && nmcli -t -f DEVICE,STATE device 2>/dev/null | grep -q '^wlan0:' \
   && ! nmcli -t -f DEVICE,STATE device | grep -q '^wlan0:unmanaged'; then
  BACKEND=nm
else
  BACKEND=netplan
fi
echo "network backend: $BACKEND"

# --- backup + automatic rollback --------------------------------------------------------------------------------
STAMP=$(date +%Y%m%d-%H%M%S)
mkdir -p "$BACKUP_DIR"; chmod 700 "$BACKUP_DIR"
ARCHIVE="$BACKUP_DIR/network_$STAMP.tgz"
tar -czf "$ARCHIVE" -C / etc/netplan $( [[ -d /etc/NetworkManager/system-connections ]] && echo etc/NetworkManager/system-connections ) 2>/dev/null
chmod 600 "$ARCHIVE"
echo "backup: $ARCHIVE"
RESTORE="rm -f /etc/netplan/60-warotrans.yaml /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg /etc/NetworkManager/system-connections/warotrans-*.nmconnection; \
tar -xzf $ARCHIVE -C /; netplan generate; netplan apply; systemctl is-active --quiet NetworkManager && nmcli connection reload || true"
systemctl stop "$UNIT.timer" 2>/dev/null || true
systemctl reset-failed "$UNIT.service" 2>/dev/null || true
systemd-run --quiet --unit "$UNIT" --on-active="${ROLLBACK_S}s" /bin/bash -c "$RESTORE"
echo "rollback armed: in ${ROLLBACK_S}s unless: sudo $0 --confirm"

# --- apply -------------------------------------------------------------------------------------------------------
if [[ $BACKEND == nm ]]; then
  slug=$(printf '%s' "$SSID" | tr -c 'A-Za-z0-9_-' '_')
  file="/etc/NetworkManager/system-connections/warotrans-wifi-$slug.nmconnection"
  # a keyfile written directly: the password never shows up in a process list
  umask 077
  python3 - "$file" "$SSID" <<'PY'
import os, sys, uuid
path, ssid = sys.argv[1], sys.argv[2]
old_uuid = None
if os.path.exists(path):
    for line in open(path, encoding="utf-8"):
        if line.startswith("uuid="):
            old_uuid = line.split("=", 1)[1].strip()
with open(path, "w", encoding="utf-8") as f:
    f.write(f"""[connection]
id=warotrans-wifi-{ssid}
uuid={old_uuid or uuid.uuid4()}
type=wifi
interface-name=wlan0
autoconnect=true
autoconnect-priority=10

[wifi]
mode=infrastructure
ssid={ssid}

[wifi-security]
key-mgmt=wpa-psk
psk={os.environ['WIFI_PSK']}

[ipv4]
method=auto
route-metric=100

[ipv6]
method=auto
""")
os.chmod(path, 0o600)
PY
  nmcli connection reload
  if [[ -n "$FALLBACK" ]]; then
    eth=$(nmcli -g GENERAL.CONNECTION device show eth0 2>/dev/null || true)
    if [[ -z "$eth" ]]; then
      nmcli connection add type ethernet ifname eth0 con-name warotrans-eth0 >/dev/null
      eth=warotrans-eth0
    fi
    # DHCP stays (ICS), the fixed address is added next to it; IPv6 link-local is untouched (SSH by name keeps working)
    nmcli connection modify "$eth" ipv4.method auto ipv4.addresses "$FALLBACK" ipv4.route-metric 600 \
      ipv4.may-fail yes connection.autoconnect yes
    nmcli connection up "$eth" >/dev/null 2>&1 || true
  fi
  nmcli device wifi rescan >/dev/null 2>&1 || true
  sleep 3
  nmcli connection up "warotrans-wifi-$SSID" >/dev/null 2>&1 || echo "WARN: '$SSID' not in range now (saved, joins when seen)"
else
  python3 - "$SSID" "$FALLBACK" <<'PY'
import glob, os, sys, yaml
ssid, fallback = sys.argv[1], sys.argv[2]
ours = "/etc/netplan/60-warotrans.yaml"
cfg = yaml.safe_load(open(ours)) if os.path.exists(ours) else None
cfg = cfg or {"network": {"version": 2}}
net = cfg["network"]
# carry the Wi-Fi networks of the other netplan files over, so ours is complete on its own
aps = {}
for p in sorted(glob.glob("/etc/netplan/*.yaml")):
    if p == ours:
        continue
    other = (yaml.safe_load(open(p)) or {}).get("network", {})
    for _, w in (other.get("wifis") or {}).items():
        aps.update(w.get("access-points") or {})
wl = net.setdefault("wifis", {}).setdefault("wlan0", {})
aps.update(wl.get("access-points") or {})
aps[ssid] = {"password": os.environ["WIFI_PSK"]}
wl.update({"dhcp4": True, "optional": True, "dhcp4-overrides": {"route-metric": 100}, "access-points": aps})
if fallback:
    eth = net.setdefault("ethernets", {}).setdefault("eth0", {})
    eth.update({"dhcp4": True, "optional": True, "dhcp4-overrides": {"route-metric": 600}, "addresses": [fallback]})
old = os.umask(0o077)
with open(ours, "w") as f:
    yaml.safe_dump(cfg, f, sort_keys=False)
os.umask(old)
os.chmod(ours, 0o600)
PY
  # cloud-init must not re-render 50-cloud-init.yaml over this at a later boot
  if [[ -d /etc/cloud/cloud.cfg.d ]]; then
    echo 'network: {config: disabled}' > /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg
  fi
  netplan generate
  netplan apply
fi

sleep 8
echo "--- addresses"; ip -br -4 addr show | grep -v '^lo'
echo "--- default routes"; ip route show default
echo "--- Internet"; if ping -c1 -W3 1.1.1.1 >/dev/null 2>&1; then echo "ok"; else echo "none yet"; fi
echo "If you can still reach this Pi: sudo $0 --confirm"
