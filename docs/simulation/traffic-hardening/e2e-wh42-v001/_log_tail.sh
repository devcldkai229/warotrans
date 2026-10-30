#!/usr/bin/env bash
LOG=/mnt/e/programming/my_project/warotrans/docs/simulation/traffic-hardening/logs/sim_active.log
grep -E 'CommandResult|CONTROLLER|collision|jump back|segment|NAVIGATE' "$LOG" | tail -40
echo "JUMPS=$(grep -c 'jump back in time' "$LOG" || true)"
pgrep -af parameter_bridge | head -5 || true
