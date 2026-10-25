#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /ws/devel/setup.bash
set -u
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
roscore >/ws/roscore.log 2>&1 &
core_pid=$!
trap 'kill "$launch_pid" "$core_pid" 2>/dev/null || true' EXIT
for i in $(seq 1 50); do
  if rostopic list >/dev/null 2>&1; then break; fi
  sleep 0.2
done
roslaunch /ws/src/rmader_ugv/integration/rover_sim.launch >/ws/roslaunch.log 2>&1 &
launch_pid=$!
python3 /ws/src/rmader_ugv/integration/ros1_bridge.py >/ws/bridge.log 2>&1
