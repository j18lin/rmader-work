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
optional velocity, bounding box, optional `is_static`) on
`/obstacle_observations` and set `dyn_obstacles/enabled: true`. With it
disabled, which is the default, the planner takes none of the new paths.

### What the planner does with an observation

- **Tracks it.** One track per obstacle id: a velocity estimate when the
  observation carries none (position change over `velocity_window`, then
  low-pass filtered, so that observation noise is not amplified by the
  observation rate), a constant-velocity prediction, and a timeout after which
  an unseen obstacle is forgotten. An obstacle reported with `is_static` never
  grows and is never forgotten.
- **Bounds where it can be** (`mode: reachable`, the default). This is the
  paper's inflation, Section IV-B: for the piece of trajectory that ends at
  time `t`, the obstacle's box is grown on every side by
  `obst_max_vel * (t - t_observed) + obst_position_error`, plus the fixed
  `base_inflation` and half the agent's box. If every component of the
  obstacle's velocity stays within `obst_max_vel`, it cannot leave that box,
  whatever it does. The boxes go through the same convex-hull, A*,
  optimisation and delay-check path as every other obstacle, and R-MADER's
  per-interval hulls play the role of SANDO's time layers.
- **Chooses where to aim** (`global_path`). Upstream always aims at the point
  `Ra` along the straight line to the goal, so it cannot go around a large
  blocked region. Following the paper's Section V, a grid A* now runs from the
  plan's start to the goal: cells an obstacle occupies *now* are blocked,
  everything about its future is the soft heat map (`c = d + w_heat * H`).
  The planner aims at a point along that path.
- **Only ends a plan where it can afford to stop** (`stop_dwell`). Every plan
  ends at rest. The subgoal is walked back along the global path until no
  obstacle can reach it before `stop_dwell` seconds after the plan ends. If no
  point ahead qualifies, the planner keeps its committed plan and waits rather
  than creep forward.
- **Keeps plans short** (`max_horizon`, `max_time_factor`). Box growth is
  proportional to plan duration. Plans are limited to about `max_horizon`
  seconds while an obstacle is tracked, and upstream's 2.5 times stretch of
  plans near the goal is capped at 1.5.
- **Plans with a little extra room** (`planning_margin`). Every new
  observation moves the boxes slightly. A trajectory planned flush against
  them would be rejected by the delay check that follows, so the hulls used
  for planning are that much larger than the ones used for checking.
- **Keeps replanning near the goal** while any observed obstacle is tracked.
  Upstream stops once the committed trajectory reaches the goal.
- **Optionally steps aside when stopped** (`hover_avoidance/enabled`): at the
  goal, or wherever it ran out of plan, when an obstacle's predicted path
  comes within `trigger_dist`. At the goal it returns afterwards.
- **Optionally escapes when engulfed** (`escape_when_engulfed`). A stopped
  robot can end up inside an obstacle's inflated box because the obstacle
  came to it. No plan satisfies the constraints from there. With this on, it
  plans one move straight away against the obstacles' bodies only. No
  guarantee applies while that lasts; without it the robot stays put until
  the obstacle leaves, which is what SANDO does.

The heat map is also added to the cost of R-MADER's own control-point A*
(NLopt backend only). It shapes paths and has no part in the safety argument.

### Ground mode and the rover bridge

- `integration/rover_sim.launch` takes `dyn_obstacles:=true` (from
  `RMADER_DYN_OBSTACLES=1` when started through `integration/run_live.sh`)
  and `obst_max_vel`. Its other settings are the ones the rover-speed tests
  pass with: `stop_dwell` 4 s, stepping aside within 3 m, escape on.
- `integration/ros1_bridge.py` accepts obstacle rows from the simulator side
  and publishes them to both planners:

  ~~~json
  {"type": "obstacle", "id": 7, "p": [1.0, -2.0], "size": [0.6, 0.6],
   "v": [0.1, 0.0], "static": false, "age_s": 0.08}
  ~~~

  `v`, `static` and `age_s` are optional; `p` and `size` may have a third
  component. The two ends do not share a clock, so the observation is stamped
  on arrival minus `age_s`. The bridge's heartbeat reports `obstacle_rows`.
  Nothing on the simulator side sends these rows yet.
- A stopped rover cannot leave towards a subgoal outside its heading cone.
  The planner now asks for a turn on the spot: zero velocity and the wanted
  heading in the setpoint yaw. Whatever tracks the setpoints has to honour
  that; the test drivers do, the simulator's bridge has not been changed.
- The control-point A* never worked in ground mode: with `a_star_samp_z: 1`
  it divided by zero and every candidate became NaN, so the optimiser always
  started from "stay put". That went unnoticed while nothing was in the way.
  It is fixed, and the search now prints why it failed when it does.

### How this relates to the paper

| | SANDO | Here |
| --- | --- | --- |
| Obstacle inflation per time layer | `r_n = v_max (n+1) dt + eps` around the estimate at planning time | Same formula per R-MADER interval, with the time measured from the observation, which also covers its age |
| Speed and error bounds | Per axis (Assumptions 1 and 2) | Per axis |
| Guarantee | Collision-free against tracked obstacles for the duration of the planned trajectory; no recursive feasibility | The same argument applies to `reachable` mode: each spline segment is separated by a plane from the box that contains the obstacle during that segment's interval. Not proven here beyond the unit test of the box property |
| When replanning fails | Keep executing the previous trajectory, then hover | Same (R-MADER's committed trajectory ends at rest) |
| Global path | Voxel A* over a map; hard where static and dynamic obstacles are now, soft heat for the rest | Same rule on a grid built for each plan from the tracked obstacles only, since there is no map. It picks the subgoal; R-MADER's control-point A* then plans to it with the boxes as hard constraints |
| Safe subgoal | Walk back along the global path out of the inflated unknown region | Same walk-back, applied to tracked obstacles and extended by `stop_dwell` |
| Heat map | Static and dynamic terms | Dynamic term only. Its reach uses `max_horizon + stop_dwell` instead of the paper's prediction horizon, so that the path keeps as far away as a subgoal has to |
| Time allocation | Parallel search over factors from 1.0 to 2.5 | One factor, capped at `max_time_factor` |
| Corridors and optimiser | Convex-decomposition polytopes per layer and path segment, MIQP with variable elimination | Not ported. R-MADER's separating planes and NLopt are used as they were |
| Unknown space | Unknown-region boundaries inflated by the same `r_n` | Not ported: there is no map of unknown space, so an obstacle that has not been observed is not protected against |
| Tracker | Point cloud, temporal occupancy grid, clustering, adaptive EKF | Observations arrive ready-made; windowed finite difference and low-pass filter |
| `predicted` mode | Named as future work | Extension: box follows the constant-velocity prediction. Tighter, no guarantee |
| Stepping aside, escaping | Hover avoidance is in the released configuration, not in the paper | Own implementation |
| `planning_margin`, `stop_dwell`, replanning at GOAL_SEEN, turn on the spot | Not applicable | Added to fit R-MADER's delay check, status machine and ground mode |

### Test

`tests/observed_obstacles_test.py` drives the planners in closed loop. The
obstacles never publish a trajectory; the planners get 20 Hz position
observations with up to 0.03 m of noise per axis and no velocity. Aerial
robots are point masses with the two-agent test's limits; rovers are
unicycles at 0.55 m/s that turn on the spot when asked.

- `crossing`: one aerial robot shuttles 8 m while three 0.6 m obstacles wander
  through its corridor at up to 0.5 m/s, each heading for a new random
  waypoint at a new random speed whenever it arrives.
- `crossing2`: two aerial robots swap ends through that corridor.
- `hover`: an aerial robot is parked at its goal and an obstacle drives
  straight through that spot at 0.4 m/s.
- `ground`: one rover shuttles 10 m across a 15 m arena in which two obstacles
  wander at up to 0.25 m/s.
- `ground2`: two rovers cross at right angles through that arena.

`tests/bridge_obstacles_test.py` plays the simulator's side of the bridge
over TCP against the real `integration/rover_sim.launch` and
`integration/ros1_bridge.py`: two rovers crossing, a static crate on one
rover's line and a wandering obstacle.

Each scenario has a control run with the feature off, which has to collide
for the scenario to mean anything.

~~~bash
cloud/run_tests.sh          # seed 7 of everything below, plus the unit tests
~~~

Results on the 4-CPU cloud container (NLopt backend). The gap is the smallest
distance between a robot's box and an obstacle's box; negative means they
overlapped. Aerial runs last 60 s at 2 m/s among obstacles at up to 0.5 m/s;
rover runs last 150 s (one rover) or 180 s (two) at 0.55 m/s among obstacles
at up to 0.25 m/s.

| Run | Seed | Minimum gap | Robot to robot | Legs | Overlap |
| --- | --- | --- | --- | --- | --- |
| crossing | 7 | 1.08 m | n/a | 7 | none |
| crossing | 11 | 1.12 m | n/a | 6 | none |
| crossing | 23 | 0.92 m | n/a | 7 | none |
| crossing2 | 7 | 1.06 m | 0.20 m | 5 and 5 | none |
| crossing2 | 11 | 1.02 m | 0.01 m | 4 and 5 | none |
| hover | n/a | 1.51 m | n/a | stepped aside, back within 0.01 m | none |
| ground | 7 | 1.13 m | n/a | 4 | none |
| ground | 11 | 1.20 m | n/a | 2 | none |
| ground | 23 | 1.18 m | n/a | 3 | none |
| ground2 | 7 | 1.18 m | 0.16 m | 2 and 2 | none |
| ground2 | 11 | 1.14 m | 0.19 m | 3 and 3 | none |
| bridge | 7 | 0.41 m (crate) | 0.03 m | 2 and 2 | none |
| crossing, feature off | 7 | -0.46 m | n/a | 9 | 9.3 s |
| hover, avoidance off | n/a | -0.48 m | n/a | stayed put | 2.4 s |
| ground, feature off | 7 | -0.50 m | n/a | 3 | 11.0 s |
| bridge, feature off | 7 | -0.40 m | 0.89 m | 1 and 1 | 8.8 s |

Twelve runs with the feature on, no overlap with any obstacle; the four
controls all collide. That is evidence that the pieces work together in these
scenarios, not a collision rate. The bridge run delivered 3,364 obstacle rows.
Robot-to-robot gaps of a few centimetres are R-MADER's own inter-agent
spacing: it separates the two boxes and nothing more.

Rover legs are 10 m and take between 25 s and 90 s, against about 25 s with
nothing in the way: at barely twice the obstacles' speed the rover spends
much of its time waiting for a subgoal that is safe to stop at.

How it got here, because the intermediate failures are the useful part:

- Before the global path, a rover never finished a leg: it drove up to the
  obstacles' boxes and sat there, since R-MADER only aims straight ahead.
- With the global path but no `stop_dwell`, robots were hit while standing
  still in the middle of the obstacles (once for one rover, repeatedly for
  two). Stopping is the weak point of this whole family of planners.
- With `stop_dwell` but the paper's short heat horizon, the path hugged the
  obstacles, no subgoal ever qualified, and aerial robots mostly waited. The
  heat's reach now follows from `max_horizon + stop_dwell`.
- Upstream's 2.5 times stretch of plans near the goal made every box near
  the goal 2.5 times larger; one aerial run could not commit a single plan.
- `predicted` mode collided in about one run in three throughout, always
  with the robot stationary. It is out of the default test run.

The two-agent timing test above misses its 0.5 m separation gate on this
container with or without any of these changes: 0.385 m and 0.462 m (one leg
timeout) for the commit before this work, and between 0.435 m and 0.644 m
since, all 90 s runs. The 16-CPU laptop results quoted below were not
reproduced here.

### Limits

- `reachable` holds only against obstacles that are being tracked, whose
  velocity components stay within `obst_max_vel`, and whose observed position
  is within `obst_position_error` per axis, and only up to the end of the
  committed trajectory, where the robot is at rest. As in the paper there is
  no recursive feasibility. `stop_dwell` buys a fixed time after stopping,
  not a guarantee; after it a stopped robot can still be run into. If a track
  is seen moving faster than the bound, that track's bound is raised to the
  measured speed.
- The whole approach needs the robot to be clearly faster than the
  obstacles. The rover tests use obstacles at 0.25 m/s against a 0.55 m/s
  rover and are slow as a result; the paper flies at up to 5 m/s among
  obstacles at 0.5 m/s.
- An obstacle that has not been observed yet is not protected against. SANDO
  handles that by inflating the boundary of unknown space, which needs a map
  this planner does not have.
- `predicted` is not a guarantee and has collided in tests.
- Stepping aside and escaping act only when the robot is already stopped,
  which is outside what the guarantee covers. Escaping plans against the
  obstacles' bodies only.
- Nothing in this repository produces observations from sensors. The tests
  publish them from ground truth with noise. The bridge accepts obstacle
  rows, but the simulator side does not send any yet, and it has to honour
  the turn-on-the-spot request.
- The Gurobi backend was not built or run here; the heat cost in the
  control-point A* is compiled out of it.
- Two upstream bugs surfaced and are fixed: `dynTraj2dynTrajCompiled` never
  copied `is_agent`, so the planner read an uninitialised flag; and the
  ground-mode A* division by zero described above.

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
