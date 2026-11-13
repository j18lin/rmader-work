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

The overlay shows both rovers and their R-MADER paths together. Each paired
goal is sent through one ROS 1 batch with the same timestamp, after both
rovers are ready. A short committed trajectory away from the goal is marked
`R-MADER HOLD`. The output directory contains `rmader_recording.json`,
`rmader_deconfliction.json`, and `rmader_grade.json`. The deconfliction report
compares the two committed trajectories at matching future timestamps. Their
geometric routes still cross; R-MADER can separate them in time by holding one
rover.

In a verified 100 s truth-feedback run, the first pair of committed plans
were created 0.0012 s apart and predicted only 0.009 m centre separation at
the intersection. RR83 committed a hold plan 0.27 s after the shared goal
batch; that plan and JR81's concurrent plan predicted 1.757 m minimum centre
separation. Both rovers completed two legs, the normal sim grade passed, and
actual minimum chassis clearance was 1.29 m. The controller's 0.75 m peer
stop was never used. Timing and which rover yields may vary in later runs.

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

## Observed dynamic obstacles

Upstream R-MADER avoids an obstacle only if something broadcasts that
obstacle's future trajectory as closed-form expressions of time. This checkout
adds a second way in, taken from SANDO (Kondo, Tordesillas, How,
[arXiv:2604.07599](https://arxiv.org/abs/2604.07599)): an obstacle only has to
be *observed*. Publish `rmader_msgs/ObstacleObservation` (stamp, id, position,
optional velocity, bounding box) on `/obstacle_observations` and set
`dyn_obstacles/enabled: true`. With it disabled, which is the default, the
planner takes none of the new paths.

What the planner does with an observation:

- **Tracks it.** One track per obstacle id: a velocity estimate when the
  observation carries none (position change over `velocity_window`, then
  low-pass filtered, so that observation noise is not amplified by the
  observation rate), a constant-velocity prediction, and a timeout after which
  an unseen obstacle is forgotten.
- **Bounds where it can be** (`mode: reachable`, the default). This is the
  paper's inflation, Section IV-B: for the piece of trajectory that ends at
  time `t`, the obstacle's box is grown on every side by
  `obst_max_vel * (t - t_observed) + obst_position_error`, plus the fixed
  `base_inflation` and half the agent's box. If every component of the
  obstacle's velocity stays within `obst_max_vel`, it cannot leave that box,
  whatever it does. The boxes go through the same convex-hull, A*,
  optimisation and delay-check path as every other obstacle, and R-MADER's
  per-interval hulls play the role of SANDO's time layers.
- **Plans with a little extra room** (`planning_margin`). Every new
  observation moves the boxes slightly. A trajectory planned flush against
  them would be rejected by the delay check that follows, so the hulls used
  for planning are that much larger than the ones used for checking.
- **Biases the A\* guess** with the paper's dynamic-obstacle heat map
  (Section V-B, parameters from its Tables II and III): heat around each
  obstacle's current position plus a time-discounted tube around its
  predicted path, maximum over obstacles, added to the edge cost as
  `length + w_heat * H`. NLopt backend only. It shapes the guess and has no
  part in the safety argument.
- **Keeps replanning near the goal** while any observed obstacle is tracked.
  Upstream stops once the committed trajectory reaches the goal.
- **Optionally steps aside** (`hover_avoidance/enabled`) when an obstacle
  approaches while the agent is stopped at its goal, and returns afterwards.

### How this relates to the paper

| | SANDO | Here |
| --- | --- | --- |
| Obstacle inflation per time layer | `r_n = v_max (n+1) dt + eps` around the estimate at planning time | Same formula per R-MADER interval, with the time measured from the observation, which also covers its age |
| Speed and error bounds | Per axis (Assumptions 1 and 2) | Per axis |
| Guarantee | Collision-free against tracked obstacles for the duration of the planned trajectory; no recursive feasibility | The same argument applies to `reachable` mode: each spline segment is separated by a plane from the box that contains the obstacle during that segment's interval. Not proven here beyond the unit test of the box property |
| When replanning fails | Keep executing the previous trajectory, then hover | Same (R-MADER's committed trajectory ends at rest) |
| Heat map | Static and dynamic terms on a voxel grid | Dynamic term only, evaluated at B-spline control points. There is no voxel map, so no static term |
| Global path | Voxel A* in which reachable sets are soft costs only | R-MADER's control-point A*, in which the inflated boxes are also hard constraints |
| Corridors and optimiser | Convex-decomposition polytopes per layer and path segment, MIQP with variable elimination, parallel time-allocation search | Not ported. R-MADER's separating planes and NLopt are used as they were |
| Unknown space | Unknown-region boundaries inflated by the same `r_n` | Not ported: R-MADER has no map of unknown space, so an obstacle that has not been observed is not protected against |
| Tracker | Point cloud, temporal occupancy grid, clustering, adaptive EKF | Observations arrive ready-made; windowed finite difference and low-pass filter |
| `predicted` mode | Named as future work | Extension: box follows the constant-velocity prediction. Tighter, no guarantee |
| Hover avoidance | In the released configuration, not in the paper | Own implementation |
| `planning_margin`, replanning at GOAL_SEEN | Not applicable | Added to fit R-MADER's delay check and status machine |

### Test

`tests/observed_obstacles_test.py` drives one planner in closed loop, with the
same bounded tracking model as the two-agent test. The obstacles never publish
a trajectory; the planner gets 20 Hz position observations with up to 0.03 m
of noise per axis and no velocity.

- `crossing`: the agent shuttles 8 m back and forth while three 0.6 m
  obstacles wander through its corridor, each heading for a new random
  waypoint at a new random speed up to 0.5 m/s whenever it arrives.
- `hover`: the agent is parked at its goal and an obstacle drives straight
  through that spot at 0.4 m/s.

Each scenario also has a control run with the feature off, which has to
collide for the scenario to mean anything.

~~~bash
cloud/run_tests.sh          # seed 7 of everything below, plus the unit tests
~~~

Results on the 4-CPU cloud container (NLopt backend, 2 m/s agent, 0.35 m
agent box, 60 s crossing runs). The gap is the smallest distance between the
agent's box and an obstacle's box; negative means they overlapped.

| Run | Obstacle seed | Minimum gap | Time overlapping | Outcome |
| --- | --- | --- | --- | --- |
| crossing, `reachable` | 7 | 1.63 m | 0 s | 3 legs |
| crossing, `reachable` | 11 | 1.33 m | 0 s | 4 legs |
| crossing, `reachable` | 23 | 1.33 m | 0 s | 5 legs |
| crossing, `predicted` | 7 | 0.80 m | 0 s | 5 legs |
| crossing, `predicted` | 11 | 0.64 m | 0 s | 6 legs |
| crossing, `predicted` | 23 | -0.47 m | 31 s | 2 legs; boxed in while stopped |
| crossing, feature off | 7 | -0.46 m | 11.4 s | 9 legs, straight through the obstacles |
| hover, avoidance on | n/a | 1.46 m | 0 s | moved 2.8 m aside, back within 0.01 m of the goal |
| hover, avoidance off | n/a | -0.48 m | 2.4 s | stayed put and was hit |

`reachable`, the paper's inflation, was clean in all three runs and slower:
it keeps the agent out of the corridor until there is room. Three runs are not
a collision rate.

`predicted` failed one run in three. With the tighter boxes the agent goes
into the corridor, and in the failing run it came to the end of its committed
trajectory there. From then on its own position was inside some obstacle's
inflated box almost all the time, so the A* search had no feasible start (97
failures), it sat still for about 40 s, and the wandering obstacles passed
over it. This is the failure the paper describes for planners whose start can
fall inside a blocked region, and the reason to treat `predicted` as
experimental.

An earlier build was worse in `predicted` mode: it estimated velocity by
differencing consecutive observations, so the boxes jittered and the delay
check rejected 120 of 160 candidates. `velocity_window` and `planning_margin`
came out of that; rejections then fell to between 18 and 56 per run.

The two-agent timing test above misses its 0.5 m separation gate on this
container with or without these changes: 0.385 m and 0.462 m (one leg
timeout) for the previous commit, and 0.644 m, 0.435 m, 0.459 m and 0.455 m
for this branch, all 90 s runs. The 16-CPU laptop results quoted below were
not reproduced here.

### Limits

- `reachable` holds only against obstacles that are being tracked, whose
  velocity components stay within `obst_max_vel`, and whose observed position
  is within `obst_position_error` per axis, and only up to the end of the
  committed trajectory, where the agent is at rest. As in the paper there is
  no recursive feasibility: a stopped agent can still be run into unless hover
  avoidance moves it. If a track is seen moving faster than the bound, that
  track's bound is raised to the measured speed.
- An obstacle that has not been observed yet is not protected against. SANDO
  handles that by inflating the boundary of unknown space, which needs a map
  this planner does not have.
- `predicted` is not a guarantee, and failed one of three test runs (above).
- Reachable boxes grow quickly: at 0.5 m/s an obstacle is a 3 m wider box
  three seconds into the plan. In clutter this can leave no path, and the
  agent then stops at the end of its committed trajectory.
- Hover avoidance only acts when the agent is stopped at its goal, not when it
  is stopped anywhere else.
- Nothing in this repository produces observations from sensors. The test
  publishes them from ground truth with noise; a detector or motion-capture
  relay has to do that on a real system. The rover bridge in `integration/`
  does not forward obstacles yet.
- The scenarios above use a single aerial agent. Several agents sharing
  observed obstacles, ground mode with observed obstacles, and the Gurobi
  backend were not run. The heat map is compiled out of the Gurobi build,
  which was not built here.
- While doing this, an upstream bug surfaced: `dynTraj2dynTrajCompiled` never
  copied `is_agent`, so the planner read an uninitialised flag. It is now
  copied and the struct is default-initialised.

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
