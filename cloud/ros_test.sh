#!/usr/bin/env bash
# Run a closed-loop ROS 1 test without Docker: start a private roscore,
# launch the planners, run the test driver, and tear everything down.
#
#   cloud/ros_test.sh <launch file> <results dir> <driver.py> [driver args...]
#
# ROS_LAUNCH_ARGS="name:=value ..." is passed on to roslaunch.
#
# Example (the two-agent timing test from UGV_NOTES.md):
#   cloud/ros_test.sh tests/two_agent.launch test-results \
#     tests/two_agent_timing.py --duration 90 \
#     --report test-results/two_agent_timing.json
set -euo pipefail

test "$#" -ge 3 || { sed -n '2,10p' "$0" >&2; exit 2; }
launch_file="$1"; results="$2"; driver="$3"; shift 3

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
set +u
# shellcheck disable=SC1091
source "$here/activate.sh"
set -u
test -f "$RMADER_WS/devel/setup.bash" || { echo "Build the workspace first: cloud/build.sh" >&2; exit 1; }

mkdir -p "$results"
export ROS_HOME="${ROS_HOME:-$results/ros-home}"
port="${ROS_TEST_PORT:-11311}"
export ROS_MASTER_URI="http://127.0.0.1:$port"
export ROS_IP=127.0.0.1

roscore -p "$port" >"$results/roscore.log" 2>&1 &
core_pid=$!
launch_pid=
cleanup() {
  if test -n "$launch_pid"; then kill "$launch_pid" 2>/dev/null || true; fi
  kill "$core_pid" 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup EXIT

for _ in $(seq 1 30); do
  if rostopic list >/dev/null 2>&1; then break; fi
  sleep 1
done
rostopic list >/dev/null

# shellcheck disable=SC2086
roslaunch "$launch_file" ${ROS_LAUNCH_ARGS:-} >"$results/roslaunch.log" 2>&1 &
launch_pid=$!
python3 "$driver" "$@" || {
  status=$?
  tail -n 60 "$results/roslaunch.log" >&2
  exit "$status"
}
