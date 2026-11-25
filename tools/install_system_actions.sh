#!/usr/bin/env bash
# Lets the robot gateway (user waro) reboot / power off this Pi and restart its stack service without a password,
# for SYSTEM_ACTION REBOOT / SHUTDOWN / RESTART_STACK from the console. Nothing else is allowed.
# Run once on the Pi:
#   bash ~/warotrans_tools/install_system_actions.sh                 (sudo asks for the password)
#   SERVICE=warotrans-rbt002.service bash install_system_actions.sh  (RBT-002: its stack is a systemd unit)
set -euo pipefail

USER_NAME="${USER_NAME:-waro}"
SERVICE="${SERVICE:-}"
RULE_FILE=/etc/sudoers.d/warotrans-system-actions
TMP="$(mktemp)"
trap 'rm -f "$TMP"' EXIT

REBOOT_BIN="$(command -v reboot || echo /usr/sbin/reboot)"
POWEROFF_BIN="$(command -v poweroff || echo /usr/sbin/poweroff)"
{
  echo "# WaroTrans SYSTEM_ACTION (tools/install_system_actions.sh): reboot / power off from the console."
  echo "$USER_NAME ALL=(root) NOPASSWD: $REBOOT_BIN, $POWEROFF_BIN"
  if [[ -n "$SERVICE" ]]; then
    SYSTEMCTL_BIN="$(command -v systemctl || echo /usr/bin/systemctl)"
    echo "$USER_NAME ALL=(root) NOPASSWD: $SYSTEMCTL_BIN restart $SERVICE"
  fi
} > "$TMP"

sudo visudo -cf "$TMP"
sudo install -m 0440 -o root -g root "$TMP" "$RULE_FILE"
echo "installed $RULE_FILE:"
sudo cat "$RULE_FILE"
sudo -n -l "$REBOOT_BIN" >/dev/null && echo "check: $USER_NAME may run $REBOOT_BIN without a password"
