#!/usr/bin/env python3
"""Install a ROS Noetic toolchain into a conda prefix with py-rattler.

ROS 1 Noetic has no apt packages for Ubuntu 22.04+ and Docker is not
available in cloud sessions, so the toolchain comes from the RoboStack
conda channel instead. Usage:

    python cloud/install_ros_env.py /opt/ros-noetic-env
"""
import asyncio
import sys

from rattler import VirtualPackage, install, solve

CHANNELS = ["robostack-noetic", "conda-forge"]

SPECS = [
    # ROS packages rmader, rmader_msgs, snapstack_msgs, separator and
    # DecompROS need at build time, plus what the tests launch.
    "ros-noetic-ros-base",
    "ros-noetic-rviz",
    "ros-noetic-rviz-visual-tools",
    "ros-noetic-jsk-rviz-plugins",
    "ros-noetic-gazebo-msgs",
    "ros-noetic-tf",
    "ros-noetic-cmake-modules",
    "ros-noetic-message-generation",
    # Build tools.
    "catkin_tools",
    "empy <4",  # genmsg templates use the empy 3 API
    # GCC 14 exactly: the RoboStack binaries need its libstdc++ ABI
    # (CXXABI_1.3.15), while GCC 15 defaults to C23 (breaks the GLPK 4.65
    # that separator builds) and rejects the CGAL 5 headers.
    "gcc 14.*",
    "gxx 14.*",
    "cmake <4",
    "make",
    "pkg-config",
    # Planner libraries (NLopt backend; Gurobi is optional and licensed).
    "cgal-cpp 5.*",  # CGAL 6 dropped CGAL_USE_FILE, which rmader/CMakeLists.txt includes
    "nlopt",
    "eigen",
    "glpk",
    "numpy",
    "pytest",
]


async def main(prefix: str) -> None:
    records = await solve(
        sources=CHANNELS,
        specs=SPECS,
        virtual_packages=VirtualPackage.detect(),
    )
    print(f"solved {len(records)} packages; installing into {prefix}")
    await install(records, target_prefix=prefix, show_progress=False)
    print("done")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    asyncio.run(main(sys.argv[1]))
