#!/usr/bin/env bash
# Chay TREN PI. Restore backup ros2_ws/src gan nhat do deploy tao ra.
set -euo pipefail

BACKUP_DIR="$HOME/warotrans_backups"
LATEST="$(ls -1t "$BACKUP_DIR"/ros2_src_*.tgz 2>/dev/null | head -1 || true)"

if [[ -z "$LATEST" ]]; then
  echo "[FAIL] Khong co source backup trong $BACKUP_DIR"
  exit 1
fi

TS="$(date +%Y%m%d_%H%M%S)"
echo "Backup se restore: $LATEST"
echo "Current source se duoc doi thanh: ~/ros2_ws/src.failed_${TS}"

if [[ -d "$HOME/ros2_ws/src" ]]; then
  mv "$HOME/ros2_ws/src" "$HOME/ros2_ws/src.failed_${TS}"
fi

tar -xzf "$LATEST" -C "$HOME/ros2_ws"
echo "[OK] Source restored. Hay build/test lai truoc khi chay robot."
