#!/usr/bin/env bash
# Build the catkin workspace with the NLopt backend (no Gurobi license needed).
#
#   cloud/build.sh [extra catkin build args]
#
# The workspace lives outside the repository ($RMADER_WS, default
# /opt/rmader_ws) and links the checkout in as src/rmader_ugv.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$(dirname "$here")"
set +u
# shellcheck disable=SC1091
source "$here/activate.sh"
set -u

mkdir -p "$RMADER_WS/src"
ln -sfn "$repo" "$RMADER_WS/src/rmader_ugv"
cd "$RMADER_WS"
catkin init >/dev/null
catkin config --extend "$CONDA_PREFIX" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DUSE_GUROBI=OFF >/dev/null
catkin build rmader --no-status --summarize --jobs "${BUILD_JOBS:-$(nproc)}" "$@"
test -x "$RMADER_WS/devel/lib/rmader/rmader_node"
