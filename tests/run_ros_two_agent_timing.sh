#!/usr/bin/env bash
set -euo pipefail

mode="${1:-closed-loop}"
test "$mode" = closed-loop || { echo "usage: $0 closed-loop" >&2; exit 2; }
test -f /ws/devel/setup.bash || { echo "Run tests/run_ros_build.sh first" >&2; exit 1; }

source /opt/ros/noetic/setup.bash
source /ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
export ROS_HOME=/tmp/rmader-ros-home
mkdir -p "$ROS_HOME" /results

roscore >/results/roscore.log 2>&1 &
core_pid=$!
launch_pid=
cleanup() {
  if test -n "$launch_pid"; then kill "$launch_pid" 2>/dev/null || true; fi
  kill "$core_pid" 2>/dev/null || true
  wait "$launch_pid" 2>/dev/null || true
  wait "$core_pid" 2>/dev/null || true
}
trap cleanup EXIT

for attempt in $(seq 1 30); do
  if rostopic list >/dev/null 2>&1; then break; fi
  sleep 1
done
rostopic list >/dev/null

roslaunch /ws/src/rmader_ugv/tests/two_agent.launch >/results/roslaunch.log 2>&1 &
launch_pid=$!
python3 /ws/src/rmader_ugv/tests/two_agent_timing.py \
  --duration "${TEST_DURATION:-90}" --report /results/two_agent_timing.json || {
    tail -n 100 /results/roslaunch.log >&2
    exit 1
  }