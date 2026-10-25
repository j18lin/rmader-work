# R-MADER two-agent test

This checkout contains the upstream ROS 1 aerial R-MADER planner. The test below
builds its NLOPT path without a Gurobi license and drives two live planner nodes
with bounded simulated state feedback. It is a planner timing and separation
test; it does not exercise a UGV port or the ROS 2/Zenoh rover simulator.

## Run

From Ubuntu WSL, with Docker Desktop running:

~~~bash
cd ~/projects/rmader_ugv
git submodule update --init \
  submodules/catkin_simple submodules/separator submodules/snapstack_msgs

docker build -f tests/Dockerfile.noetic-build \
  -t rmader-ugv:noetic-build-deps .
docker volume create rmader_ugv_build
docker run --rm \
  -v rmader_ugv_build:/ws \
  -v "$PWD:/ws/src/rmader_ugv:ro" \
  rmader-ugv:noetic-build-deps \
  bash /ws/src/rmader_ugv/tests/run_ros_build.sh

mkdir -p test-results
docker run --rm --network none \
  -v rmader_ugv_build:/ws \
  -v "$PWD:/ws/src/rmader_ugv:ro" \
  -v "$PWD/test-results:/results" \
  rmader-ugv:noetic-build-deps \
  bash /ws/src/rmader_ugv/tests/run_ros_two_agent_timing.sh closed-loop
~~~

The report is written to test-results/two_agent_timing.json. A nonzero exit
code means a gate failed or a planner could not start.

## Scenario and gates

SQ01s and SQ02s repeatedly exchange opposite goals for 90 seconds. Each
planner uses a 0.35 m agent box, 2 m/s velocity limit, 4 m/s² acceleration
limit, and 50 ms artificial communication delay. The feedback model tracks
setpoints with those velocity and acceleration limits at 50 Hz. The test checks
that both agents finish at least two legs without a 30 second leg timeout,
their centers stay at least 0.5 m apart, the 95th percentile setpoint interval
stays at or below 0.25 seconds, and both publish committed trajectories while
their peer is subscribed. The JSON report includes individual leg times,
trajectory counts, setpoint intervals, and closest approach position and time.

Two 90 second runs on the 16-logical-CPU laptop passed: 10/9 legs with 0.518 m
minimum separation, and 10/10 legs with 0.595 m minimum separation. Earlier
diagnostic runs showed that mismatched planner/tracker motion limits and a
smaller 0.25 m agent box could fail the 0.5 m separation gate; those reports
are local in test-results/.

## Source compatibility

This upstream revision required small repairs for a Noetic NLOPT build:

- C++17 for the Ubuntu 20.04 CGAL headers.
- Preserve USE_GUROBI=OFF across CMake reconfiguration.
- Match the newer OctopusSearch::run and Rmader solver call signatures.
- Remove a numerical gradient diagnostic from every planner startup. The
  diagnostic constructed a separate solver with zero A* runtime and crashed.

The NLOPT compatibility adapter reports only optimization success; it does not
classify the three Gurobi-specific status flags. The live test checks observable
motion and separation, but it does not establish every internal safety branch.

This is an aerial planner test using a simplified tracker. It does not establish
UGV motion safety, rover hardware readiness, or parity with the separate
Mighty/ROS 2/Zenoh simulation. Those need the UGV-specific planner source and
an integration test against the rover stack.