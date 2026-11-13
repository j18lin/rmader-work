/* ----------------------------------------------------------------------------
 * Observation-based dynamic obstacle handling for R-MADER.
 *
 * Upstream R-MADER only avoids obstacles that broadcast their own future
 * trajectory (a closed-form expression of time, see DynTraj.msg). This module
 * adds the SANDO-style alternative for obstacles that are merely *observed*
 * (position, optionally velocity, at some timestamp) and whose future motion
 * is unknown apart from a speed bound:
 *
 *  - a tracker that keeps one filtered track per observed obstacle and drops
 *    tracks that are no longer seen,
 *  - time-varying occupancy boxes: for a time interval [t_a, t_b] the box
 *    bounds everywhere the obstacle can be, growing with the time elapsed
 *    since the last observation,
 *  - a "heat" cost used to bias the A* initial guess away from obstacles and
 *    from the path they are predicted to take,
 *  - an evasion goal for an agent that is stopped while an obstacle closes in.
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
  // Only the speed bound is trusted. The box for [t_a, t_b] is centred on the
  // last observed position and grows by obst_max_vel * (t_b - t_observed).
  // If the bound holds, the obstacle cannot leave the box.
  WORST_CASE = 0,
  // The centre follows a constant-velocity prediction for prediction_horizon
  // seconds and the box grows at the slower prediction_growth rate. Tighter
  // and less conservative, but only as good as the prediction. The result is
  // always clipped to the WORST_CASE box, which it can never exceed.
  PREDICTED = 1
};

bool modeFromString(const std::string& name, Mode& mode);

struct Params
{
  bool enabled = false;
  Mode mode = Mode::PREDICTED;

  double obst_max_vel = 0.5;         // [m/s] speed bound assumed for every observed obstacle
  double obst_position_error = 0.05; // [m] bound on the error of an observed position
  double base_inflation = 0.1;       // [m] constant extra margin around every observed obstacle
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

  // Heat cost for the A* guess (0 disables it).
  double heat_weight = 0.0;        // cost added per metre of path = heat_weight * heat
  double heat_alpha_reach = 0.2;   // weight of the reachability blob around the obstacle
  double heat_alpha_tube = 1.0;    // weight of the tube around the predicted path
  double heat_tube_radius = 0.5;   // [m] radius of that tube
  double heat_power = 2.0;         // falloff exponent of both terms
  double heat_max = 2.0;           // saturation of the summed heat

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

// Dimensionless cost in [0, heat_max] of being at p at time t.
double heat(const Eigen::Vector3d& p, double t, const std::vector<Track>& tracks, const Params& par);

// True if an agent stopped at `pos` should move away. `goal` is then where to go.
bool hoverEvasionGoal(const Eigen::Vector3d& pos, double now, const std::vector<Track>& tracks, const Params& par,
                      Eigen::Vector3d& goal);

}  // namespace dynobs

#endif
