#!/usr/bin/env bash
set -eo pipefail
LOG=/mnt/e/programming/my_project/warotrans/docs/simulation/traffic-hardening/logs/sim_active.log
grep 'Managed nodes are active' "$LOG" || true
echo ---
ps -eo pid,cmd | grep -E 'sim_bringup|gz sim |robot_gateway|controller_server' | grep -v grep || true
echo ---
j1=$(grep -c 'jump back in time' "$LOG" || true)
echo "jumps_now=$j1"
sleep 8
j2=$(grep -c 'jump back in time' "$LOG" || true)
echo "jumps_after_8s=$j2"
echo "delta=$((j2-j1))"
grep -E 'Failed to bring up|MQTT disconnected' "$LOG" | tail -5 || true
