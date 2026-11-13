#!/usr/bin/env bash
# Run every check that works without Docker, Gazebo or a Gurobi license:
# the unit tests, then the closed-loop ROS tests (about 6 minutes).
#
#   cloud/run_tests.sh [results dir]      # default: test-results/cloud
#
# Each closed-loop test writes a JSON report; the summary at the end lists
# which passed. Exit status is non-zero if any did not.
set -uo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$(dirname "$here")"
cd "$repo"
out="${1:-test-results/cloud}"
mkdir -p "$out"
set +u
# shellcheck disable=SC1091
source "$here/activate.sh"
set -u

failed=()
run() {
  local name="$1"; shift
  echo "=== $name"
  if "$@" >"$out/$name.log" 2>&1; then echo "    passed"; else echo "    FAILED (see $out/$name.log)"; failed+=("$name"); fi
}
closed_loop() {
  local name="$1" launch="$2" launch_args="$3" driver="$4"; shift 4
  ROS_LAUNCH_ARGS="$launch_args" run "$name" "$here/ros_test.sh" "$launch" "$out/$name" "$driver" \
    --report "$out/$name/report.json" "$@"
}

run unit_dyn_obstacles rosrun rmader test_dyn_obstacles
run unit_python python3 -m pytest -q tests

obs=tests/observed_obstacles.launch
drv=tests/observed_obstacles_test.py
closed_loop crossing_reachable "$obs" "dyn_mode:=reachable" "$drv" --scenario crossing --duration 60
closed_loop crossing_predicted "$obs" "dyn_mode:=predicted" "$drv" --scenario crossing --duration 60
closed_loop crossing_control "$obs" "dyn_enabled:=false" "$drv" --scenario crossing --duration 60 --expect-collision
closed_loop hover_avoidance "$obs" "hover_avoidance:=true" "$drv" --scenario hover --duration 45
closed_loop hover_control "$obs" "hover_avoidance:=false" "$drv" --scenario hover --duration 45 --expect-collision
closed_loop two_agent tests/two_agent.launch "" tests/two_agent_timing.py --duration "${TEST_DURATION:-90}"

echo
if test "${#failed[@]}" -eq 0; then echo "All checks passed."; else echo "Failed: ${failed[*]}"; exit 1; fi
