#!/usr/bin/env bash
# Headless Active sim launcher for traffic-hardening evidence runs.
set -eo pipefail
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
source /opt/ros/jazzy/setup.bash
source /mnt/e/programming/my_project/warotrans/ros2_ws/install/setup.bash
export PYTHONPATH="/home/ser/pydeps${PYTHONPATH:+:$PYTHONPATH}"
export GUI=false
mkdir -p /mnt/e/programming/my_project/warotrans/docs/simulation/traffic-hardening/logs
exec bash /mnt/e/programming/my_project/warotrans/ros2_ws/scripts/launch_sim_active.sh \
  > /mnt/e/programming/my_project/warotrans/docs/simulation/traffic-hardening/logs/sim_active.log 2>&1
