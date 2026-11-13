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
  PREDICTED = 1
};

bool modeFromString(const std::string& name, Mode& mode);

struct Params
{
  bool enabled = false;
  Mode mode = Mode::REACHABLE;

  double obst_max_vel = 0.5;         // [m/s] v_max^obs: bound on each component of an obstacle's velocity
  double obst_position_error = 0.05; // [m] epsilon: bound on the per-axis error of an observed position
  double base_inflation = 0.1;       // [m] r_margin: constant extra margin around every observed obstacle
  double prediction_horizon = 1.0;   // [s] PREDICTED: how long the constant-velocity prediction is extrapolated
  double prediction_growth = 0.2;    // [m/s] PREDICTED: growth rate of the box along the prediction

  // Velocity is estimated by differencing positions velocity_window apart (differencing consecutive noisy
  // observations would amplify the noise by the observation rate) and then low-pass filtered.
  double velocity_window = 0.5;        // [s]
  double velocity_filter_alpha = 0.6;  // \in (0,1], weight of the newest velocity measurement
  double velocity_threshold = 0.05;    // [m/s] slower tracks are treated as static (no growth). 0 = never static
  double track_timeout = 5.0;          // [s] drop a track that has not been observed for this long

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

  // Evasion while stopped at the goal.
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

// True if an agent stopped at `pos` should move away. `goal` is then where to go.
bool hoverEvasionGoal(const Eigen::Vector3d& pos, double now, const std::vector<Track>& tracks, const Params& par,
                      Eigen::Vector3d& goal);

}  // namespace dynobs

#endif
