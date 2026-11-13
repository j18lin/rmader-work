# Building and testing without Docker

`tests/Dockerfile.noetic-build` is the reference build (ROS Noetic on Ubuntu
20.04). Where Docker is not available, such as a Claude Code cloud session on
Ubuntu 24.04, the same toolchain comes from the
[RoboStack](https://robostack.github.io/) conda channel instead. The cloud
environment's setup script runs the first two steps for you.

~~~bash
python3 -m venv /opt/rattler-venv
/opt/rattler-venv/bin/pip install py-rattler
/opt/rattler-venv/bin/python cloud/install_ros_env.py /opt/ros-noetic-env   # ~1 min, 5 GB
git submodule update --init submodules/catkin_simple submodules/separator submodules/snapstack_msgs
cloud/build.sh                                                              # ~4 min on 4 cores
cloud/run_tests.sh                                                          # ~7 min
~~~

| File | Purpose |
| --- | --- |
| `install_ros_env.py` | Installs ROS Noetic, catkin tools, GCC 14, CGAL 5, NLopt and Eigen into a conda prefix. The pins and the reason for each are in the file. |
| `activate.sh` | `source` it to put that toolchain and the built workspace on your path. |
| `build.sh` | Builds the catkin workspace in `$RMADER_WS` (default `/opt/rmader_ws`) with the NLopt backend. |
| `ros_test.sh` | Starts a private `roscore`, a launch file and a test driver, then cleans up. |
| `run_tests.sh` | Unit tests plus every closed-loop test that needs neither Gazebo nor Gurobi. |

Network hosts the build needs beyond the package registries: `ftp.gnu.org`
(the `separator` submodule downloads GLPK 4.65 while configuring). The conda
packages come from `conda.anaconda.org` and `snapstack_msgs` from `gitlab.com`.

Not covered here: the live rover simulation (`integration/run_live.sh`) needs
Docker, Gazebo and the separate rover simulator checkout, and the Gurobi
backend needs a license.
