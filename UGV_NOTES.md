# R-MADER rover integration and two-agent tests

This checkout contains the upstream ROS 1 R-MADER planner, a planar ground
configuration, a differential drive tracker, and a bridge to the ROS 2/Zenoh
rover simulator. The original aerial timing test remains below.

## Live rover simulation

Build the NLOPT image and catkin volume using the commands under Run below.
Clone the rover simulator as a sibling checkout and build its images as its own
README describes. Then run:

~~~bash
integration/run_live.sh ../rover_sim_dev ./test-results
~~~

To watch R-MADER live in the 2D viewer, run:

~~~bash
RMADER_VIZ=1 RMADER_STATE_SOURCE=dlio RMADER_DURATION_S=120 \
  integration/run_live.sh ../rover_sim_dev ./test-results/watch
~~~

The R-MADER layer shows each rover's cyan goal, amber candidate trajectory,
green committed trajectory, yellow current setpoint, and the DLIO pose disc.
The viewer starts before the R-MADER bridge so its subscriptions are present
when planning begins. This is a watch run: opening the viewer can affect timing
and localization, so use a separate run without the viewer for the graded
performance result. The test still stops automatically at the chosen duration.

To watch both rovers cross the arena centre in one shared world view, use the
crossing scenario. JR81 starts west of the centre and heads east; RR83 starts
south and heads north. Their direct routes intersect. Truth feedback isolates
the planner and controller behaviour from intermittent JR81 DLIO faults:

~~~bash
RMADER_SCENARIO=crossing RMADER_VIZ=1 RMADER_STATE_SOURCE=truth \
  RMADER_DURATION_S=100 \
  integration/run_live.sh ../rover_sim_dev ./test-results/crossing
~~~

The overlay shows both rovers and their R-MADER paths together. A short
committed trajectory away from the goal is marked `R-MADER HOLD`; in the
verified crossing it represented yielding while the peer crossed. The grade and full
recording are saved in the output directory. In the verified 100 s truth run,
JR81 held before crossing at 40.97 s, RR83 crossed at 27.17 s, both reached
three goals, and minimum chassis clearance was 1.358 m. These times are
specific to that run. The close-range controller stop was not triggered;
minimum centre distance was 1.76 m, above its 0.75 m threshold.

This runner cycles fixed goals. Gazebo can contain cube models and the
simulator's other planner supports manual RViz goals, but this R-MADER bridge
does not yet accept manual goals or ingest static cube geometry. Adding a cube
to the world alone will not make R-MADER plan around it.

The runner starts JR81 and RR83 in Gazebo, arms their safety nodes, starts two
R-MADER nodes in an isolated ROS 1 container, and connects them through a
private TCP bridge. By default it feeds simulator truth state to R-MADER, so
the test isolates planner and rover control integration from localization.
Set RMADER_STATE_SOURCE=dlio to feed world-aligned DLIO state instead.
After a 1.2 m forward drive, it tracks
R-MADER's ground-plane position and velocity setpoints with bounded
differential drive commands on each rover's cmd_vel_auto topic. It stops
commanding on stale state, missing planner data, a disconnected bridge, or
estimated center separation under 0.75 m. In DLIO mode, ground truth is
used only for the initial drive and independent grading.

The two rovers repeatedly visit opposite sides of the arena and return.
The evaluator saves rmader_recording.json and rmader_grade.json. It checks
at least two completed legs per rover in short trials or three in 120 s trials,
both agents' setpoints and committed
trajectories, planned yaw rate and vertical motion, direct peer subscriptions,
chassis separation and world contacts from Gazebo truth and URDF footprints,
DLIO error, and real time factor. The simulator uses the rover safety nodes and records real DLIO as
a diagnostic in truth mode. Its DLIO odometry gate is N/A in truth mode.
The fallback mapping stack is omitted because this integration does not
feed static obstacle geometry into R-MADER; the driven arena corridor is
clear.

The 180 s DLIO-controlled trial completed eight legs per rover and had
0.924 m minimum chassis clearance, but JR81's DLIO estimate diverged after
the eighth leg. A later 120 s DLIO-controlled trial triggered the bridge's
motion consistency stop at 95 s. Those runs fail the odometry gate. Truth
mode demonstrates the planner and differential drive connection while
that localization issue remains.

After the ground-planner change, separate 120 s and 240 s DLIO-controlled
trials passed all applicable gates. The 240 s run reached ten legs per rover;
maximum DLIO position error was 0.036 m for JR81 and 0.027 m for RR83,
minimum chassis clearance was 0.898 m, and no odometry fault was recorded.
These runs show DLIO control works in this arena under these conditions; the
older intermittent JR81 failure is not explained by them.

The bridge now latches a stop if consecutive DLIO poses imply translation or
heading faster than the rover can achieve. A test-only 2 m JR81 pose jump at
40 s triggered this check. The command trace reached zero within 0.06 s,
remained zero, and simulator truth showed only 0.028 m of path length in the
next 2 s. The injected run intentionally fails the normal odometry grade;
`scripts/audit_rmader_fault_stop.py` in the simulator checkout grades its
fail-stop response separately. The drive image's offline geofence end-to-end
suite passed 30/30 checks, including stale-limit and invalid-intent holds.
A further normal 120 s DLIO-controlled run with the plausibility guard enabled
passed all gates without a false trip; maximum DLIO errors were 0.033 m and
0.032 m for JR81 and RR83.

To repeat the fail-stop test, set `RMADER_STATE_SOURCE=dlio`,
`RMADER_DURATION_S=65`, and `RMADER_INJECT_JR81_DLIO_JUMP_AT=40` when calling
`integration/run_live.sh`. Its normal grade exits nonzero because the injected
fault must fail the odometry gate. Then run
`python3 ../rover_sim_dev/scripts/audit_rmader_fault_stop.py` on the saved
`rmader_recording.json` with `--out` to check stop latency and path length.

No physical rover was connected. The live integration does not engage the
drive-level geofence or feed static obstacle geometry, and it has no physical
sensor, wheel, terrain, or emergency-stop measurements. These simulation
results cannot establish hardware safety.

An earlier 120 s truth-state run passed its applicable gates: five legs per rover,
0.926 m minimum chassis clearance, no world box contact, peer subscriptions
in both directions, and minimum real time factor 0.998. Its DLIO diagnostic
still showed 9.108 m maximum JR81 error. The JSON reports retain the full
time series and the N/A odometry gate.

With ground constraints and new-goal spline reset, a 120 s truth-state run
passed the stricter three-leg gate: four legs per rover, 46 trajectories and
41 committed trajectories per planner, 0.914 m minimum chassis clearance,
no world box contact, and minimum real time factor 0.999. Both planners
reported zero yaw-rate violations and negligible vertical motion. The arena
legs were nearly straight, so this run does not stress tight curved paths.

Ground mode now adds planar forward-heading and yaw-rate constraints to the
NLopt spline optimizer. The initial spline tangent must lie within 0.5 rad of
the rover's heading. At samples throughout the path, planar curvature is
limited to a 0.8 rad/s yaw rate above near-zero speed, and dense validation
rejects unsampled violations and abrupt tangent reversals through a stop.
Setpoint yaw follows the path tangent.
The bridge rotates a rover in place before sending a goal that points behind
it. On a new ground goal, the planner clears the previous goal's spline and
starts from the next measured rover pose and heading. The aerial planner is
unchanged when ground mode is disabled. Ground mode
requires the NLopt backend; the Gurobi backend does not implement these
constraints.

These are sampled geometric constraints on a position spline, not a full
differential-drive trajectory optimization with an explicit heading state and
wheel dynamics. The tracker still handles acceleration, steering, and stops.
Successful simulator runs do not establish collision safety on arbitrary
terrain or hardware. DLIO divergence remains a separate localization issue.

## Aerial precursor

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

This aerial precursor uses a simplified tracker and does not establish rover
motion safety or hardware readiness. The live test above exercises the
two-rover simulator with a ground tracker, with the stated limitations.
