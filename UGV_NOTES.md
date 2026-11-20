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
- **Or bounds it more tightly** (`mode: accel_bounded`). If each component of
  the obstacle's acceleration also stays within `obst_max_acc`, it can only
  get as far as accelerating flat out from its current velocity allows. The
  box is then centred on where the obstacle is heading and at first grows
  with time squared. It is never larger than the `reachable` box and comes
  with the same kind of guarantee under that one extra assumption, plus a
  bound on the velocity estimate's error (`obst_velocity_error`). A person
  can turn on the spot and breaks it; a vehicle with known limits does not.
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
| Heat map | Static and dynamic terms | Dynamic term; the static term too when there is a map. Its reach uses `max_horizon + stop_dwell` instead of the paper's prediction horizon, so that the path keeps as far away as a subgoal has to |
| Time allocation | Parallel search over factors from 1.0 to 2.5 | B-spline backend: one factor, capped at `max_time_factor`. Corridor backend: SANDO's window of factors, tried in order (see "SANDO's optimiser, map and tracker") |
| Corridors and optimiser | Convex-decomposition polytopes per layer and path segment, MIQP with variable elimination | Ported as a second backend, `corridor/enabled`, without Gurobi. The default backend is still R-MADER's separating planes and NLopt |
| Unknown space | Unknown-region boundaries inflated by the same `r_n` | Ported with a map built from range scans, `map/enabled`, for the corridor backend only |
| Tracker | Point cloud, temporal occupancy grid, clustering, adaptive EKF | Observations arrive ready-made; windowed finite difference and low-pass filter. Clustering and the adaptive filter are ported as an option, `tracker/enabled`, that turns scans into such observations |
| Tighter sets from prediction | Named as future work | `accel_bounded`: bounded acceleration from the estimated velocity, clipped to the reachable box. `predicted`: box follows the constant-velocity prediction, no guarantee |
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
- `crossing2`: two aerial robots swap ends through that corridor (planner
  box 0.6 m, see the results).
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
| crossing2 | 7 | 1.01 m | 0.13 m | 3 and 3 | none |
| crossing2 | 14 | 1.13 m | 0.06 m | 3 and 3 | none |
| crossing2 | 21 | 0.83 m | 0.69 m | 4 and 4 | none |
| crossing2 | 28 | 1.13 m | 0.48 m | 3 and 3 | none |
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

Fourteen runs with the feature on, no overlap with any obstacle; the four
controls all collide. That is evidence that the pieces work together in these
scenarios, not a collision rate. The bridge run delivered 3,364 obstacle rows.

Robot-to-robot spacing is R-MADER's own job and none of this changes it, but
the two-robot aerial test exposed how thin it is: R-MADER separates two
teammates' boxes exactly, with no allowance for a vehicle lagging its
setpoints. With the planner's box equal to the vehicle's 0.35 m, the two
robots passing head-on overlapped by 1 to 14 cm in five of ten runs, without
ever touching an obstacle. The `crossing2` rows above use a 0.6 m planner box
(0.125 m of tracking allowance per side) and scored against the 0.35 m
vehicle; with 0.5 m one run in four still overlapped by 1 cm. The rover runs
did not need this.

`accel_bounded` against `reachable`, on obstacles whose acceleration really
is limited to 0.2 m/s² per axis and which report their velocity
(`--obstacle-acceleration 0.2 --report-velocity`), one aerial robot, 60 s:

| Seed | `reachable` | `accel_bounded` |
| --- | --- | --- |
| 7 | 7 legs, 1.05 m | 8 legs, 0.74 m |
| 11 | 6 legs, 1.25 m | 7 legs, 0.94 m |
| 23 | 6 legs, 1.28 m | 7 legs, 0.84 m |

No overlap in any of the six. The tighter boxes buy about one leg in seven
here and far fewer waits; the gain is small because the heat map and the
safe-to-stop rule dominate, and it shrinks further as the velocity estimate
gets worse.

The whole suite (`cloud/run_tests.sh`, seed 7 of each scenario) was then run
once on the final build: every observed-obstacle, bridge and link-drop check
passed except `crossing2`, which at that point still used the 0.35 m planner
box and deadlocked nose to nose; it passed when rerun with the allowance.

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
- An obstacle that has not been observed yet is not protected against,
  unless the corridor backend is used with a map (next but one section).
- `predicted` is not a guarantee and has collided in tests.
- Stepping aside and escaping act only when the robot is already stopped,
  which is outside what the guarantee covers. Escaping plans against the
  obstacles' bodies only.
- By default nothing produces observations from sensors: the tests publish
  them from ground truth with noise. The optional tracker (`tracker/enabled`)
  finds moving obstacles in range scans and has been tried in one scenario. The bridge accepts obstacle
  rows, but the simulator side does not send any yet, and it has to honour
  the turn-on-the-spot request.
- The Gurobi backend was not built or run here; the heat cost in the
  control-point A* is compiled out of it.
- The planner's box has to include tracking error. R-MADER keeps teammates'
  boxes exactly apart and nothing more.
- `accel_bounded` depends on two more numbers than `reachable`, the
  acceleration bound and the velocity estimate's error bound, and is only as
  good as they are.
- Two upstream bugs surfaced and are fixed: `dynTraj2dynTrajCompiled` never
  copied `is_agent`, so the planner read an uninitialised flag; and the
  ground-mode A* division by zero described above.

## A teammate that stops talking

R-MADER's guarantee needs every plan to reach the other robots within the
delay-check time, and the paper says it "does not handle complete
communication drops". What upstream actually does when a link drops is keep
planning around the last trajectory it received from that teammate, for as
long as it stays in its local map, and forget the teammate altogether once
that trajectory stops mattering. `peer_silence/enabled: true` replaces that
with SANDO's idea applied to a teammate. It is not in either paper.

- Every planner sends a heartbeat on `/rmader_heartbeat` every
  `heartbeat_period`, so that a teammate with nothing new to say is not
  mistaken for a broken link. Teammates are remembered separately from the
  local map.
- **Hold.** Nothing from a teammate for `hold_after` (0.3 s): commit no new
  plan. The committed plan ends at rest and was checked against everything
  that teammate had published.
- **Silent.** Nothing for `silent_after` (1 s): stop trusting its last
  trajectory. From then on it is an obstacle that was on that trajectory when
  last heard and may have moved at up to `peer_max_vel` on each axis since.
  That is SANDO's reachable set with a bound that is known, because the
  teammate is one of ours.
- As soon as anything arrives from it, it is a teammate again.

What this gives and what it does not:

- It degrades to a stop, it does not keep the team working. A silent
  teammate's box grows at its top speed, so robots at 2 m/s are boxed in
  within a second or two and stay put until the link returns. Slow robots
  get more room for longer.
- Plans committed in the first `hold_after` after a link fails were made
  without knowing it had failed. That window is not covered.
- A teammate that never comes back blocks everything eventually.
  `forget_after` gives up on it after a set time; the default is never.
- There is no proof. The argument is that once both sides are silent to each
  other, each stays out of everywhere the other can be, and each is
  somewhere it can be.

`tests/peer_silence_test.py` swaps two aerial robots end for end, with the
geometry of the two-agent timing test, and makes one of them deaf, mute or
both for 15 s through a test-only topic (`peer_silence/fault_injection`).

| Run | Before the fault | While the link was down | After |
| --- | --- | --- | --- |
| deaf and mute, feature on | 0.29 m apart | both stopped (0.00 m/s), 3.3 m apart | 0.10 m, 2 more swaps |
| deaf and mute, feature on | 0.08 m | stopped (0.01 m/s), 1.97 m | 0.03 m, 2 more swaps |
| deaf only, feature on | 0.12 m | 0.11 m (the hearing robot kept going) | 0.10 m, 2 more swaps |
| mute only, feature on | 0.24 m | 3.57 m | 0.01 m, 3 more swaps |
| feature off | 0.06 m | kept flying at 0.8 m/s, 0.33 m | 0.17 m |
| feature off | 0.04 m | kept flying at 1.4 m/s, overlapped by 4 cm | overlapped by 30 cm |

Distances are box-to-box. With the feature on the robots stopped and stayed
apart in every run and carried on afterwards. Without it they always kept
flying while unable to hear each other. Whether they then touched depended on
small differences between their paths: one of these two runs, none of three
more on an earlier build, and one earlier run on a longer line. The small gaps
before and after the fault are plain R-MADER with a 0.35 m planner box, as
described above.

Not run: more than two robots, ground mode, and a silent teammate together
with observed obstacles. The test does not pass every time: see the end of
"Test" in the next section.


## SANDO's optimiser, map and tracker

The section on observed obstacles puts SANDO's obstacle model in front of
R-MADER's own search and B-spline optimiser. This section is the rest of
SANDO, ported from its source (`j18lin/sando`, BSD 3-Clause, copyright Kota
Kondo) as three options that are all off by default:

| Option | What it is | Needs |
| --- | --- | --- |
| `corridor/enabled` | SANDO's local planner as the backend: time-layered corridors and a trajectory optimised inside them | |
| `map/enabled` | A map of free, occupied and unknown space built from range scans; static heat; unknown-space inflation | `corridor/enabled` |
| `tracker/enabled` | Clustering and an adaptive Kalman filter that turn the same scans into obstacle observations | `map/enabled`, `dyn_obstacles/enabled` |

What stays R-MADER's in every case: the check after optimisation, the delay
check, the two-step publication, and the trajectory teammates receive (a
piecewise polynomial, as before). Planners on either backend can share an
arena, because what they exchange did not change. That mix was not run.

### The corridor backend

One replanning step, with the SANDO source it follows:

1. **Global path** (`graph_search.cpp`). The grid A* of the previous section,
   now also blocked where the map is occupied (grown by the robot's size) and
   where a teammate is at the start of the plan, and heated near obstacle
   surfaces (the paper's static heat) and along what each teammate has
   committed to. Unknown space is free here, as in SANDO. The grid path is
   straightened where that crosses nothing blocked and nothing hotter. The
   teammate terms are not from SANDO, which plans for one robot.
2. **Time allocation** (`getInitialDt`, the factor window in `sando.cpp`).
   `dt = factor * dt0`, with SANDO's `dt0` and SANDO's window of factors that
   follows the last one that worked. SANDO solves the window's factors in
   parallel threads and takes whichever finishes first; here they are tried
   in ascending order, so the shortest plan that works is the one used.
3. **End point** (`findSafeSubGoal`). For each factor, the furthest point of
   the path where the robot can stop and stay clear of every teammate and
   obstacle for `stop_dwell`, and that is outside occupied space and outside
   the unknown region grown by `v * T`. If the corridors to that point leave
   no trajectory, nearer end points are tried (0.6 and 0.36 of the path): a
   shorter plan gives everything less time to spread.
4. **Corridors** (`cvxEllipsoidDecompTimeLayered`). For each of the N pieces'
   time intervals and each of the P path segments, one polytope from
   DecompUtil's ellipsoid decomposition; 2D polygons extruded over a height
   band for ground robots, as in the fork's "2D changes". What the polytopes
   keep out, in layer `n`:
   - the convex hull R-MADER builds for interval `n` of every teammate's
     committed trajectory, every broadcast obstacle, every observed
     obstacle's reachable box and every silent teammate's box, as points
     sampled on the hull's surface;
   - occupied voxels, and the unknown region grown by
     `r_n = v_unknown * (time to the end of layer n)`.
   A polytope whose own path segment is inside one of those is dropped, and
   that layer cannot use that segment. SANDO fails the whole decomposition
   instead.
5. **Optimisation** (`gurobi_solver.cpp`). Same problem: N cubic pieces,
   position, velocity and acceleration continuous, start state given, end at
   rest, every piece inside one polytope of its layer, per-axis bounds on the
   velocity, acceleration and jerk control points, minimum squared jerk.
   Solved without Gurobi: the equalities are eliminated numerically, which
   leaves the paper's N - 3 free variables per axis, and the choice of
   polytopes is a branch and bound over small QPs solved by a dual active-set
   solver written for this (`dense_qp.cpp`). By default only choices that
   follow the path's order are tried; `corridor/exhaustive` tries all P^N,
   which is the optimum of SANDO's mixed-integer problem (checked against
   brute force in the unit test, where both settings gave the same optimum).
   For rovers the first piece also has to leave within the heading cone, and
   a result that turns faster than the rover can is rejected.
6. **Exact check.** The polytopes are built from sampled points, so before a
   trajectory is accepted every piece is checked against every hull of its
   interval with the separating-plane LP R-MADER uses everywhere else. This
   is what carries R-MADER's guarantee over to this backend. Against the map
   the check is weaker: the robot's box is tested against occupied voxels at
   points sampled along the trajectory. Unknown space is kept out by the
   polytopes alone, as in SANDO.

### The map

`voxel_map.cpp`. SANDO receives occupied and unknown voxels from a separate
mapper; here the planner builds the map itself from range scans
(`sensor_msgs/PointCloud2` on `~point_cloud`, world frame, taken from the
robot's position; a ray as long as `map/sensor_range` or longer found
nothing). A voxel is unknown until a ray has reached it. A ray that ends in a
voxel adds evidence and rays passing through take it away, so one stray
return does not build a wall and something that has left is forgotten after a
few scans. Whatever is tracked as a moving obstacle is kept out of the map.
Ground robots get one layer of voxels.

The map is used the way the paper says: occupied voxels block the global path
and unknown ones do not; the static heat is the paper's formula; in the
corridors unknown space counts as occupied and is grown, layer by layer, with
the paper's L-infinity ball. The speed of what may come out of unknown space
is `map/unknown_max_vel`, by default the obstacles' speed bound.

### The tracker

`cloud_tracker.cpp`, a port of `obstacle_tracker_node.cpp` without PCL: voxel
grid, Euclidean clustering, bounding boxes, nearest-track association, and
the adaptive Kalman filter with the same update of its noise covariances.
Two differences: the prediction step uses the time that really passed (SANDO
uses a fixed step), and only tracks that move faster than
`tracker/velocity_threshold` are passed on, as observations with a velocity,
to everything described under "Observed dynamic obstacles". What stands
still is left to the map.

### Test

Unit tests, all passing: `test_corridor_planner` (the QP solver on 2,000
random problems against the optimality conditions and an independent
feasibility test; the optimiser in free space, round a corner, with a passage
that closes over time, with the heading cone; corridor generation;
around a growing obstacle end to end), `test_voxel_map`, `test_cloud_tracker`.
One optimisation takes about 0.1 ms; a whole planning step took 2 to 30 ms.

Closed loop, corridor backend against the B-spline backend on the scenarios
of "Observed dynamic obstacles" (60 s aerial, 150 and 180 s ground):

| Scenario | B-spline backend | Corridor backend |
| --- | --- | --- |
| One aerial robot, three obstacles | 1.12 m to the nearest obstacle, 6 legs | 0.62 to 0.75 m, 2 to 3 legs (three runs) |
| Two aerial robots (planner box 0.6 m) | 0.85 m, robots 0.41 m apart, 5 legs each | 0.70 m, robots 0.46 m apart, 1 leg each |
| One rover, two obstacles | 1.38 m, 4 legs | 1.53 m, 1 leg |
| Two rovers | 1.25 m, rovers 0.12 m apart, 3 legs each | Two runs: 1.13 m with the rovers' boxes overlapping by 3.4 cm, 2 legs each; 0.87 m with the rovers 0.17 m apart, 2 legs each |

No robot touched an obstacle on either backend. The overlap of the two rovers
is between boxes the planner keeps exactly apart, with no allowance for how
well the rovers follow their setpoints (the planner's box is the vehicle's
box in the ground tests; the aerial two-robot test has such an allowance for
the same reason). With a 0.8 m planner box the two rovers stayed 0.25 m apart
but completed no leg in 180 s.

With a map, one rover (0.55 m/s) with a simulated planar range sensor
(4 m, 360 rays, 10 Hz), `tests/map_test.py`:

| Run | Result |
| --- | --- |
| Crates, unknown space grown at 0.1 m/s | 5 legs in 150 s, never closer than 0.53 m to a crate |
| Crates, unknown space not grown (`inflate_unknown:=false`) | 2 legs, 0.39 m |
| Crates, unknown space grown at 0.25 m/s | Stopped 0.8 m short of the first crate and stayed there |
| Crates, no map | Drove 0.65 m into a crate |
| Ambush, unknown space grown at 0.25 m/s, the hidden obstacle's speed | 5 legs, never closer than 0.95 m to the obstacle (two runs, the same to a millimetre) |
| Ambush, grown at 0.1 m/s | Hit by the obstacle while standing, boxes 0.25 m into each other |
| Ambush, not grown | Hit while standing, 0.45 m |
| Ambush, obstacle never reported, tracker on, grown at 0.25 m/s | 2 legs, 0.07 m |

The rover launch file and bridge (`integration/rover_sim.launch` with
`corridor:=true dyn_obstacles:=true`, this side playing the simulator) also
ran once on the corridor backend: 0.40 m to the crate that crosses, rovers
3.1 m apart, one leg each in 180 s (the B-spline backend did two).

In the ambush the rover drives along a wall and an obstacle comes round the
wall's far end across its way; the obstacle is reported to the planner only
while the rover has a line of sight to it. In the last row it is not reported
at all and the planner's own tracker has to find it in the scans.

What these show:

- The corridor backend kept every robot clear of every obstacle and teammate
  in every run, including while it was being debugged. It is slower than the
  B-spline backend in the air: with N = 5 there are two free variables per
  axis, the bounds are on control points, and the factor that works is
  usually near 2.
- The map does what it is for: without it the rover drives into a crate.
- Unknown-space inflation is the difference between passing and being hit in
  the ambush, but only when it assumes the speed the hidden obstacle really
  has. Without it, or with a lower speed, the rover commits to a plan that
  ends in the obstacle's way before it can see it, stops there, and is hit
  while standing.
- The same inflation at 0.25 m/s stops this rover in front of the first
  crate, for good: the unseen inside of a crate is unknown space too, and
  0.25 m/s for the several seconds a slow rover's plan lasts is more than a
  metre in every direction. SANDO has the same property and escapes it with
  short plans on a fast vehicle. Among the crates the test therefore grows
  unknown space at 0.1 m/s.

The whole suite (`cloud/run_tests.sh`, now 26 checks) was run once on this
build: 23 passed. The three that did not: two rovers on the corridor backend
(the 3.4 cm overlap above; it passed when rerun), upstream's `two_agent`
(as before), and `peer_silence`, described next.

`peer_silence` is the link-drop test of the previous section, on the default
backend with none of these options on. It turned out to fail in some runs:
after the link returns the two planners can keep rejecting each other's
trajectories in the delay check and never set off again, or pass each other
with their boxes touching by up to 4 cm. Run alternately, the previous commit
failed 1 of 4 and this build 2 of 4, the previous commit's failure having the
same log as this build's. Over everything run that day it was 1 of 7 against
8 of 13. No code path that differs with these options off was found, and the
difference is not explained. The results table of the previous section is
from runs that passed.

### Limits

- Everything in the list under "Observed dynamic obstacles" still applies.
- One run per configuration, apart from the aerial crossings.
- The map and the tracker were run closed-loop for a ground robot only. The
  3D map has unit tests and nothing else.
- The tracker passed its one closed-loop run with 7 cm to spare. It sees only
  the near faces of an obstacle, so its boxes are too small and off-centre;
  `tracker/bbox_padding` is a fixed allowance for that, not a bound. In the
  unit test the adaptive filter followed a turn worse than the plain one.
  The fork's tracker node clusters every return and so does this; neither
  separates moving from static points before clustering.
- The scans are taken to come from the robot's position in the world frame.
  There is no frame handling, and the rover bridge does not carry scans.
- With `map/enabled` the B-spline backend is refused, because it has no way
  to use the map.
- Not ported: solving the time factors in parallel, and Gurobi itself.
- The polytope choice is ordered by default. The unit test found the same
  optimum either way, on one problem.

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
