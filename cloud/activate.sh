# Source this file to use the RoboStack ROS Noetic toolchain installed by
# cloud/install_ros_env.py, and the catkin workspace if it has been built.
#
#   source cloud/activate.sh
#
# conda's own `activate` is not installed, so this replays the prefix's
# activation hooks directly.
export CONDA_PREFIX="${ROS_ENV_PREFIX:-/opt/ros-noetic-env}"
export RMADER_WS="${RMADER_WS:-/opt/rmader_ws}"
case ":$PATH:" in
  *":$CONDA_PREFIX/bin:"*) ;;
  *) export PATH="$CONDA_PREFIX/bin:$PATH" ;;
esac
export CMAKE_PREFIX_PATH="$CONDA_PREFIX"
for _hook in "$CONDA_PREFIX"/etc/conda/activate.d/*.sh; do
  # shellcheck disable=SC1090
  . "$_hook"
done
unset _hook
if [ -f "$RMADER_WS/devel/setup.bash" ]; then
  # shellcheck disable=SC1091
  . "$RMADER_WS/devel/setup.bash"
fi
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
export ROS_IP="${ROS_IP:-127.0.0.1}"
