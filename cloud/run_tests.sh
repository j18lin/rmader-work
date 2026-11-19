#!/usr/bin/env bash
# Run every check that works without Docker, Gazebo or a Gurobi license:
# the unit tests, then the closed-loop ROS tests (about 25 minutes).
# RUN_EXPERIMENTAL=1 adds the predicted-mode run.
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

# Aerial: one and two planners among three erratic obstacles, and the stopped-at-goal case.
closed_loop crossing_reachable "$obs" "escape:=true" "$drv" --scenario crossing --duration 60
# planner_bbox: 0.125 m of tracking allowance around the 0.35 m vehicle box, so that teammates stay apart too
closed_loop crossing_two_robots "$obs" "agents:=2 escape:=true planner_bbox:=0.6" "$drv" --scenario crossing2 --duration 60
closed_loop crossing_control "$obs" "dyn_enabled:=false" "$drv" --scenario crossing --duration 60 --expect-collision
closed_loop hover_avoidance "$obs" "hover_avoidance:=true" "$drv" --scenario hover --duration 45
closed_loop hover_control "$obs" "hover_avoidance:=false" "$drv" --scenario hover --duration 45 --expect-collision

# Acceleration-bounded boxes, on obstacles that really are sluggish (0.2 m/s^2) and report their velocity.
closed_loop crossing_accel_bounded "$obs" "escape:=true dyn_mode:=accel_bounded obst_max_acc:=0.2 obst_velocity_error:=0.1" \
  "$drv" --scenario crossing --duration 60 --obstacle-acceleration 0.2 --report-velocity

# Ground mode at rover limits (0.55 m/s) among obstacles at 0.25 m/s, with the settings rover_sim.launch uses.
rover="ground:=true obst_max_vel:=0.25 stop_dwell:=4.0 hover_avoidance:=true evade_trigger:=3.0 escape:=true"
closed_loop ground_rover "$obs" "$rover" "$drv" --scenario ground --duration 150 --min-legs 1
closed_loop ground_two_rovers "$obs" "$rover agents:=2" "$drv" --scenario ground2 --duration 180 --min-legs 1
closed_loop ground_control "$obs" "ground:=true dyn_enabled:=false" "$drv" --scenario ground --duration 90 --expect-collision

# The real rover launch file and bridge script, with this side playing the simulator over TCP.
closed_loop bridge integration/rover_sim.launch "dyn_obstacles:=true" tests/bridge_obstacles_test.py --duration 180
closed_loop bridge_control integration/rover_sim.launch "" tests/bridge_obstacles_test.py --duration 60 --expect-collision

# A teammate that stops talking: the link between two planners is cut for 15 s.
closed_loop peer_silence tests/peer_silence.launch "peer_silence:=true" tests/peer_silence_test.py
closed_loop peer_silence_control tests/peer_silence.launch "peer_silence:=false" tests/peer_silence_test.py --control

if test "${RUN_EXPERIMENTAL:-0}" = 1; then
  # predicted mode gives no guarantee and has collided in about one run in three
  closed_loop crossing_predicted "$obs" "dyn_mode:=predicted escape:=true" "$drv" --scenario crossing --duration 60
fi

closed_loop two_agent tests/two_agent.launch "" tests/two_agent_timing.py --duration "${TEST_DURATION:-90}"

echo
if test "${#failed[@]}" -eq 0; then echo "All checks passed."; else echo "Failed: ${failed[*]}"; exit 1; fi
