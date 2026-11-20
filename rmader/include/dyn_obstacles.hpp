/* ----------------------------------------------------------------------------
 * Observation-based dynamic obstacle handling for R-MADER.
 *
 * Upstream R-MADER only avoids obstacles that broadcast their own future
 * trajectory (a closed-form expression of time, see DynTraj.msg). This module
 * adds obstacles that are merely *observed* (position, optionally velocity, at
 * some timestamp) and whose future motion is unknown apart from a speed bound,
 * following SANDO (Kondo, Tordesillas, How, arXiv:2604.07599):
 *
 *  - a tracker that keeps one filtered track per observed obstacle and drops
 *    tracks that are no longer seen (the paper's Section VII uses an adaptive
 *    EKF on point-cloud clusters; here observations arrive ready-made),
 *  - per-interval reachable boxes, the paper's Section IV-B: for a time
 *    interval ending at t_b the obstacle's box is grown on every side by
 *    r = v_max * (t_b - t_observed) + epsilon,
 *  - the dynamic-obstacle heat map of Section V-B, used to bias the A* guess,
 *  - an evasion goal for an agent that is stopped while an obstacle closes in
 *    (in SANDO's released configuration, not in the paper).
 *
 * Nothing here depends on ROS, CGAL or the solvers, so it can be unit tested
 * on its own (see src/examples/test_dyn_obstacles.cpp).
 * -------------------------------------------------------------------------- */

#pragma once
#ifndef DYN_OBSTACLES_HPP
#define DYN_OBSTACLES_HPP

#include <Eigen/Dense>
#include <deque>
#include <functional>
#include <string>
#include <vector>

namespace dynobs
{
enum class Mode
{
  // SANDO's inflation (Section IV-B, Eq. 11). Only the speed bound is trusted: the box for [t_a, t_b] is
  // centred on the last observed position and grown by obst_max_vel * (t_b - t_observed) on every side.
  // If every component of the obstacle's velocity stays within the bound (Assumption 1), the obstacle
  // cannot leave the box. The paper measures the time from the planning instant; measuring it from the
  // observation also covers the age of the observation.
  REACHABLE = 0,
  // Not in the paper, which names it as future work ("inflating around the predicted future position").
  // The centre follows a constant-velocity prediction for prediction_horizon seconds and the box grows at
  // the slower prediction_growth rate. Tighter, but only as good as the prediction: no guarantee. The
  // result is clipped to the REACHABLE box, which it can never exceed.
  PREDICTED = 1,
  // Also beyond the paper, but with the same kind of guarantee as REACHABLE under one more assumption: each
  // component of the obstacle's acceleration stays within obst_max_acc. Starting from the estimated
  // velocity (known to within obst_velocity_error), the obstacle can then only get as far as accelerating
  // flat out allows: the set is centred on where it is heading and at first grows with time squared, not
  // linearly. It is never larger than the REACHABLE box. An obstacle that can turn on the spot (a person)
  // breaks the assumption; a vehicle with known limits does not.
  ACCEL_BOUNDED = 2
};

bool modeFromString(const std::string& name, Mode& mode);

struct Params
{
  bool enabled = false;
  Mode mode = Mode::REACHABLE;

  double obst_max_vel = 0.5;         // [m/s] v_max^obs: bound on each component of an obstacle's velocity
  double obst_position_error = 0.05; // [m] epsilon: bound on the per-axis error of an observed position
  double base_inflation = 0.1;       // [m] r_margin: constant extra margin around every observed obstacle
  double obst_max_acc = 1.0;         // [m/s^2] ACCEL_BOUNDED: bound on each component of an obstacle's acceleration
  double obst_velocity_error = 0.1;  // [m/s] ACCEL_BOUNDED: bound on the per-axis error of the velocity estimate
  double prediction_horizon = 1.0;   // [s] PREDICTED: how long the constant-velocity prediction is extrapolated
  double prediction_growth = 0.2;    // [m/s] PREDICTED: growth rate of the box along the prediction

  // Velocity is estimated by differencing positions velocity_window apart (differencing consecutive noisy
  // observations would amplify the noise by the observation rate) and then low-pass filtered.
  double velocity_window = 0.5;        // [s]
  double velocity_filter_alpha = 0.6;  // \in (0,1], weight of the newest velocity measurement
  double velocity_threshold = 0.05;    // [m/s] slower tracks are treated as static (no growth). 0 = never static
  double track_timeout = 5.0;          // [s] drop a track that has not been observed for this long
  double static_track_timeout = 0.0;   // [s] same for obstacles declared static. 0: keep them for good

  // Longest plan [s] while any observed obstacle is tracked (0 = no limit). The boxes grow with the plan's
  // duration, so a long plan is a wall of boxes; SANDO likewise plans a short horizon and replans. The goal
  // of each plan is pulled in to about v_max * max_horizon from its start.
  double max_horizon = 4.0;

  // Upper limit on the planner's time-allocation factor while an observed obstacle is tracked. Upstream
  // stretches plans to factor_alloc_close (2.5) times their minimum duration near the goal; box growth is
  // proportional to plan duration, so that makes every box near the goal 2.5 times larger. SANDO searches
  // factors from 1.0 upwards and takes the first that works.
  double max_time_factor = 1.5;

  // Every plan ends at rest. Its end point is chosen outside everything the obstacles can reach until this
  // long after the plan ends, so that a robot whose next replans fail is not hit the moment it stops.
  double stop_dwell = 1.5;  // [s]

  // Extra clearance used only while *planning*. Every new observation moves the boxes a little, so a
  // trajectory planned flush against them would fail the checks that follow (delay check, next replan).
  double planning_margin = 0.15;  // [m]

  // Dynamic-obstacle heat map for the A* guess (paper Section V-B/V-D, values from its Tables II and III).
  // Soft only: it shapes the guess and has no part in the safety argument.
  double heat_weight = 5.0;        // w_heat: edge cost = length + w_heat * H(q). 0 disables the heat map
  double heat_alpha_base = 1.0;    // alpha_0^d: scale of the heat around the current position
  double heat_alpha_tube = 2.0;    // alpha_1^d: scale of the tube around the predicted path
  double heat_power_base = 2.0;    // p^d
  double heat_power_tube = 2.0;    // q^d
  double heat_tube_growth = -1.0;  // gamma_k [m/s], growth of the tube radius. Negative: the speed bound
  double heat_tau_ratio = 0.5;     // tau_ratio^d: time weight exp(-t / (tau_ratio * prediction_horizon))
  int heat_num_samples = 15;       // M_tube: samples of the predicted path
  double heat_max = 50.0;          // H_max
  // Horizon used for the reach of the heat (R^d = R_0 + v_max * horizon) and the length of the tube.
  // 0: prediction_horizon. The planner sets it to max_horizon + stop_dwell when it is left at 0, so that the
  // global path keeps about as far from the obstacles as a plan's end point has to (see stop_dwell);
  // otherwise the path hugs them and no point along it ever qualifies as a subgoal.
  double heat_horizon = 0.0;

  // Global path (paper Section V-D): grid A* from the plan's start to the terminal goal. Only the obstacles'
  // *current* boxes are hard; everything about their future is the soft heat above. The planner aims at the
  // point max_horizon along this path instead of straight at the goal, so it can go around a blocked region.
  bool global_path_enabled = true;
  double global_resolution = 0.3;       // [m] grid size
  double global_window = 8.0;           // [m] half-size of the search region around the start (horizontal)
  double global_window_z = 2.0;         // [m] same, vertical (ignored for a planar search)
  int global_max_expansions = 40000;

  // Last resort, not in the paper. A robot that is standing still can end up inside an obstacle's inflated
  // box (the obstacle came to it). From there no plan satisfies the constraints, so SANDO-style it would sit
  // until hit or until the obstacle leaves. With this on, that one plan is made against the obstacles'
  // bodies only (no growth, no margins) and heads straight away from them. No guarantee while it lasts.
  bool escape_when_engulfed = false;
  double escape_dist = 1.5;  // [m] how far such a plan goes

  // Evasion while stopped: at the goal, or anywhere the robot ran out of plan.
  bool hover_avoidance_enabled = false;
  double hover_trigger_dist = 4.0;        // [m] evade when a moving obstacle is predicted to come this close
  double hover_evasion_dist = 3.0;        // [m] how far to move away
  bool hover_avoidance_2d = true;         // keep the evasion in the horizontal plane
  double hover_min_repulsion_norm = 0.01; // below this the repulsions cancel out and we side-step instead
};

struct Observation
{
  int id = 0;
  double stamp = 0.0;  // [s] time the obstacle was at pos
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  bool has_vel = false;  // false: the tracker estimates the velocity from consecutive positions
  Eigen::Vector3d bbox = Eigen::Vector3d::Zero();  // full size (hx, hy, hz) of the obstacle
  bool is_static = false;  // the source knows this obstacle does not move: no growth, no heat tube, no timeout
};

struct Track
{
  int id = 0;
  double stamp = 0.0;  // time of the last observation
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();  // filtered
  Eigen::Vector3d bbox = Eigen::Vector3d::Zero();
  double max_vel = 0.0;  // speed bound used for this track (>= Params::obst_max_vel, see Tracker::update)
  bool is_dynamic = false;
  bool declared_static = false;  // from Observation::is_static; max_vel is 0 then
  int num_observations = 0;
  // Recent (stamp, position) pairs, oldest first, spanning at most Params::velocity_window.
  std::deque<std::pair<double, Eigen::Vector3d>> history;
};

struct Box
{
  Eigen::Vector3d lo = Eigen::Vector3d::Zero();
  Eigen::Vector3d hi = Eigen::Vector3d::Zero();

  Eigen::Vector3d center() const
  {
    return 0.5 * (lo + hi);
  }
  bool contains(const Eigen::Vector3d& p, double tol = 0.0) const
  {
    return ((p.array() >= lo.array() - tol) && (p.array() <= hi.array() + tol)).all();
  }
  std::vector<Eigen::Vector3d> vertexes() const;
};

// Everywhere the *centre* of the obstacle can be during [t_a, t_b], for the given mode, grown by
// `margin` on every side. Pass half the obstacle size plus whatever clearance the caller needs as margin
// to get an occupancy box. Times before the observation are treated as the observation time.
Box centerBox(const Track& track, double t_a, double t_b, const Params& par,
              const Eigen::Vector3d& margin = Eigen::Vector3d::Zero());

// centerBox() grown by half the obstacle size, the observation error and the base inflation: a box that
// contains the obstacle's body during [t_a, t_b]. extra_margin is added on top (e.g. half the agent's size).
Box occupancyBox(const Track& track, double t_a, double t_b, const Params& par,
                 const Eigen::Vector3d& extra_margin = Eigen::Vector3d::Zero());

class Tracker
{
public:
  explicit Tracker(const Params& par = Params()) : par_(par)
  {
  }

  void setParams(const Params& par)
  {
    par_ = par;
  }
  const Params& params() const
  {
    return par_;
  }

  // Returns false (and changes nothing) if the observation is older than the one the track already holds.
  bool update(const Observation& obs);

  // Drops the tracks not observed for Params::track_timeout and returns their ids.
  std::vector<int> prune(double now);

  bool get(int id, Track& track) const;
  const std::vector<Track>& tracks() const
  {
    return tracks_;
  }
  void clear()
  {
    tracks_.clear();
  }

private:
  Params par_;
  std::vector<Track> tracks_;
};

// Dynamic-obstacle heat H^d(q) of the paper's Section V-B, capped at heat_max: per obstacle a base term
// around its current position plus a time-weighted tube around its predicted path, maximum over obstacles.
double heat(const Eigen::Vector3d& q, const std::vector<Track>& tracks, const Params& par);

struct GlobalPathRequest
{
  Eigen::Vector3d start = Eigen::Vector3d::Zero();
  Eigen::Vector3d goal = Eigen::Vector3d::Zero();
  // Region the path has to stay in (e.g. the arena limits). The search window is clipped to it.
  Eigen::Vector3d lower = Eigen::Vector3d::Constant(-1e9);
  Eigen::Vector3d upper = Eigen::Vector3d::Constant(1e9);
  Eigen::Vector3d agent_half_size = Eigen::Vector3d::Zero();  // half the agent's bounding box
  bool planar = false;                                        // true: search in the horizontal plane only
};

// What the global search needs to know beyond the observed obstacles: a static map, teammates. Both
// members may be left empty.
struct GlobalPathExtras
{
  std::function<bool(const Eigen::Vector3d&)> blocked;  // true where the agent's centre cannot be
  std::function<double(const Eigen::Vector3d&)> heat;   // combined with the dynamic heat by max, as in the paper
};

// Heat of a body whose future path is known, e.g. a teammate's committed trajectory: the tube term of the
// paper with the known path in place of the prediction and a tube that does not grow. `path` holds
// (time from now, position). This term is not in the paper, which considers a single agent.
double knownPathHeat(const Eigen::Vector3d& q, const std::vector<std::pair<double, Eigen::Vector3d>>& path,
                     double radius, const Params& par);

// Grid A* with the paper's edge cost c = d + heat_weight * H(q). A cell is blocked if the agent's box centred
// on it would overlap an obstacle's current box (grown by base_inflation and obst_position_error). The goal
// is clamped into the search window. If the start itself is blocked, the path may cross blocked cells until
// it first reaches a free one. Returns false if no path was found; `path` then is empty.
// On success `path` starts at request.start and ends at the cell nearest the (clamped) goal.
bool planGlobalPath(const GlobalPathRequest& request, const std::vector<Track>& tracks, const Params& par,
                    std::vector<Eigen::Vector3d>& path, const GlobalPathExtras* extras = nullptr);

// The point `arc_length` along a polyline from its first point (its last point if the polyline is shorter).
Eigen::Vector3d pointAlongPath(const std::vector<Eigen::Vector3d>& path, double arc_length);
double pathLength(const std::vector<Eigen::Vector3d>& path);

// True if an agent stopped at `pos` should move away. `goal` is then where to go.
bool hoverEvasionGoal(const Eigen::Vector3d& pos, double now, const std::vector<Track>& tracks, const Params& par,
                      Eigen::Vector3d& goal);

}  // namespace dynobs

#endif
