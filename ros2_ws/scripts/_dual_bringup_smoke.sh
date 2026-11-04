#!/usr/bin/env bash
# Dual-robot bringup smoke (readiness only — not full E2E traffic matrix).
set +e
source /opt/ros/jazzy/setup.bash
source /mnt/e/programming/my_project/warotrans/ros2_ws/install/setup.bash
ROOT=/mnt/e/programming/my_project/warotrans/ros2_ws
AUD=/mnt/e/programming/my_project/warotrans-system/docs/audits/autonomous_gazebo_acceptance
mkdir -p "$AUD/runtime"
bash "$ROOT/scripts/kill_sim.sh"
pkill -9 -f ros2-daemon 2>/dev/null || true
LOG=$AUD/runtime/dual_bringup.log
timeout 120 bash "$ROOT/scripts/launch_sim_map_one_dual.sh" gui:=false gateway:=true 2>&1 | tee "$LOG"
EC=${PIPESTATUS[0]}
echo "DUAL_LAUNCH_EXIT=$EC"
echo "---GREP---"
grep -E "multi_sim|SIM-01|SIM-02|LaneAware|Managed nodes are active|ERROR|Fatal|process has died|RBT-SIM" "$LOG" | tail -80
echo DUAL_SMOKE_DONE
