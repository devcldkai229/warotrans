#!/usr/bin/env bash
# Build the packages the rig needs into a private WSL workspace (never ~/ros2_ws, never the Pi).
#   wsl -e bash tools/nav_recovery_rig/build_ws.sh
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$(cd "$HERE/../../ros2_ws/src" && pwd)"
WS="${WT_RIG_WS:-$HOME/.cache/warotrans_rig_ws}"
mkdir -p "$WS/src"
for pkg in warotrans_msgs warotrans_lane_planner warotrans_navigation warotrans_fleet; do
  rm -rf "${WS:?}/src/$pkg"
  # Copied, not symlinked: the Windows drive has no usable symlinks/permissions for colcon.
  tar -C "$SRC" --exclude='__pycache__' --exclude='build' -cf - "$pkg" | tar -C "$WS/src" -xf -
done
set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "$WS"
colcon build --packages-up-to warotrans_navigation warotrans_lane_planner warotrans_fleet \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF --event-handlers console_direct- 2>&1 | tail -25
echo "ws=$WS"
