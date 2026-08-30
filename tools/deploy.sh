#!/usr/bin/env bash
# WaroTrans deploy (Linux/macOS/WSL/Git Bash)
# Source laptop = source of truth. Chi dong bo ros2_ws/src, KHONG dong maps/bags/log runtime.
set -euo pipefail

PI="${PI:-waro@warotrans.local}"
RUN=""
BUILD=1
DRY=0
DELETE=1

usage(){
  cat <<USAGE
Usage: ./tools/deploy.sh [options]
  --pi USER@HOST        default: ${PI}
  --run NAME            run warotrans_bringup NAME.launch.py after build
  --no-build            sync only
  --dry-run             show rsync changes only; no backup/build/run
  --no-delete           do not delete stale files on remote source

Default deploy creates a timestamped remote source backup, then mirrors ros2_ws/src.
Runtime data outside ros2_ws/src is untouched.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --pi) PI="$2"; shift 2;;
    --run) RUN="$2"; shift 2;;
    --no-build) BUILD=0; shift;;
    --dry-run) DRY=1; shift;;
    --no-delete) DELETE=0; shift;;
    -h|--help) usage; exit 0;;
    *) echo "Unknown option: $1"; usage; exit 2;;
  esac
done

command -v rsync >/dev/null 2>&1 || { echo "[FAIL] rsync is required for deterministic deploy."; exit 1; }
[[ -d ros2_ws/src ]] || { echo "[FAIL] Run from repo root; ros2_ws/src missing."; exit 1; }

echo "==> Target: $PI"
echo "==> Scope : ros2_ws/src ONLY"

RSYNC_ARGS=(-az --itemize-changes --exclude '__pycache__' --exclude '*.pyc')
[[ $DELETE -eq 1 ]] && RSYNC_ARGS+=(--delete-after)
[[ $DRY -eq 1 ]] && RSYNC_ARGS+=(--dry-run)

if [[ $DRY -eq 0 ]]; then
  echo "==> Backup remote source (best effort)"
  ssh "$PI" 'mkdir -p ~/warotrans_backups; if [ -d ~/ros2_ws/src ]; then ts=$(date +%Y%m%d_%H%M%S); tar -czf ~/warotrans_backups/ros2_src_${ts}.tgz -C ~/ros2_ws src; ls -1t ~/warotrans_backups/ros2_src_*.tgz 2>/dev/null | tail -n +6 | xargs -r rm -f; fi'
  ssh "$PI" 'mkdir -p ~/ros2_ws/src ~/warotrans_tools'
fi

echo "==> Sync ROS source"
rsync "${RSYNC_ARGS[@]}" ros2_ws/src/ "${PI}:~/ros2_ws/src/"

if [[ $DRY -eq 1 ]]; then
  echo "==> Dry-run complete. Nothing changed."
  exit 0
fi

# Copy operational tools; these do not modify runtime data by themselves.
scp -q tools/doctor.sh tools/check.sh tools/rollback_source.sh "${PI}:~/warotrans_tools/"
ssh "$PI" 'chmod +x ~/warotrans_tools/doctor.sh ~/warotrans_tools/check.sh ~/warotrans_tools/rollback_source.sh'

if [[ $BUILD -eq 1 ]]; then
  echo "==> Build on Pi"
  if ! ssh "$PI" 'set -e; source /opt/ros/jazzy/setup.bash; cd ~/ros2_ws; colcon build --symlink-install'; then
    echo "[FAIL] Build failed. Remote source backup was kept in ~/warotrans_backups/."
    echo "       Roll back with: ssh -t $PI ~/warotrans_tools/rollback_source.sh"
    exit 1
  fi
fi

if [[ -n "$RUN" ]]; then
  echo "==> Run ${RUN}.launch.py"
  ssh -t "$PI" "source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 launch warotrans_bringup ${RUN}.launch.py"
fi

echo "==> Deploy complete"
