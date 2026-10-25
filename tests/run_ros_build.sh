#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash
cd /ws

for path in \
  src/rmader_ugv/rmader/package.xml \
  src/rmader_ugv/rmader_msgs/package.xml \
  src/rmader_ugv/submodules/catkin_simple/package.xml \
  src/rmader_ugv/submodules/separator/package.xml \
  src/rmader_ugv/submodules/snapstack_msgs/package.xml; do
  test -f "$path" || { echo "Missing $path" >&2; exit 1; }
done

catkin init --no-extend >/dev/null 2>&1 || true
catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release -DUSE_GUROBI=OFF
catkin build rmader --force-cmake --no-status --jobs "${BUILD_JOBS:-2}" --summarize

source /ws/devel/setup.bash
test -x /ws/devel/lib/rmader/rmader_node
rospack find rmader