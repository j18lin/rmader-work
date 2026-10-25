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
cleanup() {
  docker rm -f sim-rmader >/dev/null 2>&1 || true
  (cd "$sim_repo" && python3 scripts/sim.py down) >/dev/null 2>&1 || true
}
trap cleanup EXIT
cd "$sim_repo"
python3 scripts/sim.py up rmader_smoke --stacks zenoh,gazebo,drive,frame_align,dlio
docker run -d --name sim-rmader --network rover-sim \
  -v rmader_ugv_build:/ws -v "$rmader_repo:/ws/src/rmader_ugv:ro" \
  --entrypoint sleep rmader-ugv:noetic-build-deps infinity >/dev/null
docker exec -d sim-rmader bash /ws/src/rmader_ugv/integration/run_ros1.sh
docker exec sim-drive bash -lc \
  "source /opt/ros/humble/setup.bash && source /home/swarm/code/rover_ws/install/setup.bash && python3 /rover_sim/scripts/rover_rmader_bridge.py --duration $duration --state-source $state_source"
recording="/tmp/rover_sim-$(id -u)/run/rmader_recording.json"
cp "$recording" "$out_dir/rmader_recording.json"
python3 scripts/grade_rmader.py "$recording" --out "$out_dir/rmader_grade.json"
