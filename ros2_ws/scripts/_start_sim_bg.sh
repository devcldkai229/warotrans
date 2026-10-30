#!/usr/bin/env bash
set -eo pipefail
LOGDIR=/mnt/e/programming/my_project/warotrans/docs/simulation/traffic-hardening/logs
mkdir -p "$LOGDIR"
pkill -f 'sim_bringup.launch.py' 2>/dev/null || true
pkill -f 'gz sim' 2>/dev/null || true
sleep 2
nohup bash /mnt/e/programming/my_project/warotrans/ros2_ws/scripts/_launch_sim_active_headless.sh \
  >"$LOGDIR/launcher.out" 2>&1 &
echo $! >"$LOGDIR/sim.pid"
echo "started pid=$(cat "$LOGDIR/sim.pid")"
