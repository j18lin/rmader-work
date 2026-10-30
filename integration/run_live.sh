#!/usr/bin/env bash
# Usage: integration/run_live.sh /path/to/rover_sim_dev [output-directory]
set -euo pipefail
if test $# -lt 1; then echo "pass the rover simulator checkout path" >&2; exit 2; fi
rmader_repo=$(cd "$(dirname "$0")/.." && pwd)
sim_repo=$(cd "$1" && pwd)
if test $# -ge 2; then
  mkdir -p "$2"
  out_dir=$(cd "$2" && pwd)
else
  mkdir -p "$rmader_repo/results"
  out_dir="$rmader_repo/results"
fi
duration=120
if test -n "$(printenv RMADER_DURATION_S || true)"; then duration=$RMADER_DURATION_S; fi
state_source=truth
if test -n "$(printenv RMADER_STATE_SOURCE || true)"; then state_source=$RMADER_STATE_SOURCE; fi
scenario=${RMADER_SCENARIO:-smoke}
if [[ "$scenario" != smoke && "$scenario" != crossing ]]; then
  echo "RMADER_SCENARIO must be smoke or crossing" >&2
  exit 2
fi
inject_jump_at=${RMADER_INJECT_JR81_DLIO_JUMP_AT:-0}
if ! [[ "$inject_jump_at" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "RMADER_INJECT_JR81_DLIO_JUMP_AT must be a nonnegative number" >&2
  exit 2
fi
cleanup() {
  docker rm -f sim-rmader >/dev/null 2>&1 || true
  (cd "$sim_repo" && python3 scripts/sim.py down) >/dev/null 2>&1 || true
}
trap cleanup EXIT
cd "$sim_repo"
python3 scripts/sim.py up "rmader_$scenario" --stacks zenoh,gazebo,drive,frame_align,dlio
if test "${RMADER_VIZ:-0}" = 1; then
  if test "$scenario" = crossing; then
    python3 scripts/sim.py viz --layers rmader,pose --overlay
  else
    python3 scripts/sim.py viz --layers rmader,pose
  fi
fi
docker run -d --name sim-rmader --network rover-sim \
  -v rmader_ugv_build:/ws -v "$rmader_repo:/ws/src/rmader_ugv:ro" \
  --entrypoint sleep rmader-ugv:noetic-build-deps infinity >/dev/null
docker exec -d sim-rmader bash /ws/src/rmader_ugv/integration/run_ros1.sh
docker exec sim-drive bash -lc \
  "source /opt/ros/humble/setup.bash && source /home/swarm/code/rover_ws/install/setup.bash && python3 /rover_sim/scripts/rover_rmader_bridge.py --duration $duration --state-source $state_source --scenario $scenario --inject-jr81-dlio-jump-at $inject_jump_at"
recording="/tmp/rover_sim-$(id -u)/run/rmader_recording.json"
cp "$recording" "$out_dir/rmader_recording.json"
python3 scripts/grade_rmader.py "$recording" --out "$out_dir/rmader_grade.json"
