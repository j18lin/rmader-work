/* ----------------------------------------------------------------------------
 * Copyright 2022, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include <Eigen/StdVector>
#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <limits>
#include <stdexcept>
#include <vector>
#include <stdlib.h>

#include "rmader.hpp"
#include "timer.hpp"
#include "termcolor.hpp"

using namespace termcolor;

// Uncomment the type of timer you want:
// typedef ROSTimer MyTimer;getNextGoal
// typedef ROSWallTimer MyTimer;
typedef RMADER_timers::Timer MyTimer;

Rmader::Rmader(mt::parameters par) : par_(par)
{
  // drone_status_ == DroneStatus::YAWING;
  drone_status_ == DroneStatus::TRAVELING;
  G_.pos << 0, 0, 0;
  G_term_.pos << 0, 0, 0;

  mtx_initial_cond.lock();
  stateA_.setZero();
  mtx_initial_cond.unlock();

  // Numerical gradient diagnostics belong in an explicit test, not planner startup.

  changeDroneStatus(DroneStatus::GOAL_REACHED);
  resetInitialization();

  ms::par_solver par_for_solver;

  par_for_solver.ground_robot = par_.ground_robot;
  par_for_solver.ground_max_yaw_rate = par_.ground_max_yaw_rate;
  par_for_solver.ground_heading_cone_rad = par_.ground_heading_cone_rad;

  par_for_solver.x_min = par_.x_min;
  par_for_solver.x_max = par_.x_max;

  par_for_solver.y_min = par_.y_min;
  par_for_solver.y_max = par_.y_max;

  par_for_solver.z_min = par_.z_min;
  par_for_solver.z_max = par_.z_max;

  par_for_solver.Ra = par_.Ra;
  par_for_solver.v_max = par_.v_max;
  par_for_solver.a_max = par_.a_max;
  par_for_solver.j_max = par_.j_max;
  par_for_solver.dc = par_.dc;
  par_for_solver.dist_to_use_straight_guess = par_.goal_radius;
  par_for_solver.a_star_samp_x = par_.a_star_samp_x;
  par_for_solver.a_star_samp_y = par_.a_star_samp_y;
  par_for_solver.a_star_samp_z = par_.a_star_samp_z;
  par_for_solver.a_star_fraction_voxel_size = par_.a_star_fraction_voxel_size;
  par_for_solver.num_pol = par_.num_pol;
  par_for_solver.deg_pol = par_.deg_pol;
  par_for_solver.weight = par_.weight;
  par_for_solver.epsilon_tol_constraints = par_.epsilon_tol_constraints;
  par_for_solver.xtol_rel = par_.xtol_rel;
  par_for_solver.ftol_rel = par_.ftol_rel;
  par_for_solver.solver = par_.solver;
  par_for_solver.basis = par_.basis;
  par_for_solver.a_star_bias = par_.a_star_bias;
  par_for_solver.allow_infeasible_guess = par_.allow_infeasible_guess;
  par_for_solver.alpha_shrink = par_.alpha_shrink;

  mt::basisConverter basis_converter;

  if (par.basis == "MINVO")
  {
    A_rest_pos_basis_ = basis_converter.getArestMinvo();
  }
  else if (par.basis == "BEZIER")
  {
    A_rest_pos_basis_ = basis_converter.getArestBezier();
  }
  else if (par.basis == "B_SPLINE")
  {
    A_rest_pos_basis_ = basis_converter.getArestBSpline();
  }
  else
  {
    // std::cout << red << "Basis " << par.basis << " not implemented yet" << reset << std::endl;
    // std::cout << red << "============================================" << reset << std::endl;
    abort();
  }

  A_rest_pos_basis_inverse_ = A_rest_pos_basis_.inverse();

#if USE_GUROBI_FLAG
  if (par_.ground_robot)
    throw std::runtime_error("ground_robot constraints require the NLopt solver");
  solver_ = std::unique_ptr<SolverGurobi>(new SolverGurobi(par_for_solver));
#else
  solver_ = std::unique_ptr<SolverNlopt>(new SolverNlopt(par_for_solver));
#endif

  // solver_ = new SolverNlopt(par_for_solver);
  // solver_ = new SolverGurobi(par_for_solver);

  separator_solver_ = new separator::Separator();

  if (par_.dyn.heat_horizon <= 0.0 && par_.dyn.max_horizon > 0.0)
  {
    par_.dyn.heat_horizon = std::max(par_.dyn.prediction_horizon, par_.dyn.max_horizon + par_.dyn.stop_dwell);
  }
  obs_tracker_.setParams(par_.dyn);
}

void Rmader::dynTraj2dynTrajCompiled(const mt::dynTraj& traj, mt::dynTrajCompiled& traj_compiled)
{
  mtx_t_.lock();
  for (auto function_i : traj.function)
  {
    typedef exprtk::symbol_table<double> symbol_table_t;
    typedef exprtk::expression<double> expression_t;
    typedef exprtk::parser<double> parser_t;

    symbol_table_t symbol_table;
    symbol_table.add_variable("t", t_);
    symbol_table.add_constants();
    expression_t expression;
    expression.register_symbol_table(symbol_table);

    parser_t parser;
    parser.compile(function_i, expression);

    traj_compiled.function.push_back(expression);
  }

  mtx_t_.unlock();

  traj_compiled.bbox = traj.bbox;
  traj_compiled.id = traj.id;
  // Upstream never copied is_agent, so every later `traj.is_agent` test read an uninitialized bool.
  traj_compiled.is_agent = traj.is_agent;
  traj_compiled.time_received = traj.time_received;  // ros::Time::now().toSec();
  traj_compiled.time_created = traj.time_created;
  traj_compiled.is_committed = traj.is_committed;
  traj_compiled.traj_id = traj.traj_id;

  traj_compiled.is_static =
      ((traj.is_agent == false) &&                           // is an obstacle and
       (traj.function[0].find("t") == std::string::npos) &&  // there is no dependence on t in the coordinate x
       (traj.function[1].find("t") == std::string::npos) &&  // there is no dependence on t in the coordinate y
       (traj.function[2].find("t") == std::string::npos))    // there is no dependence on t in the coordinate z
      ||                                                     // OR
      (traj.is_agent == true && fabs(traj.pwp.times.back() - traj.pwp.times.front()) < 1e-7);

  traj_compiled.pwp = traj.pwp;
}
// Note that we need to compile the trajectories inside mader.cpp because t_ is in mader.hpp

void Rmader::updateTrajObstacles_with_delaycheck(mt::dynTraj traj)
{
  // std::cout << "in updateTrajObstacles" << std::endl;

  MyTimer tmp_t(true);

  mt::dynTrajCompiled traj_compiled;
  dynTraj2dynTrajCompiled(traj, traj_compiled);

  // std::cout << "bef mtx_trajs_.lock() in updateTrajObstacles" << std::endl;
  mtx_trajs_.lock();

  if (traj.is_agent)
  {
    Peer& peer = peers_[traj.id];
    peer.last_heard = std::max(peer.last_heard, traj.time_received);
    if (traj.is_committed)
    {
      peer.last_committed = traj_compiled;
      peer.has_trajectory = true;
    }
  }
  // std::cout << "aft mtx_trajs_.lock() in updateTrajObstacles" << std::endl;

  // if (exists_in_local_map && traj.is_committed)
  if (traj.is_committed)
  {  // if that object already exists, substitute its trajectory
    // clean

    // std::cout << "clean!" << std::endl;

    trajs_.erase(std::remove_if(trajs_.begin(), trajs_.end(),
                                [&](mt::dynTrajCompiled const& traj) {
                                  return traj.id == traj_compiled.id && !traj.is_observed;
                                }),
                 trajs_.end());

    // std::cout << "traj_compiled.id is " << traj_compiled.id << std::endl;
    // for (auto traj : trajs_){std::cout << "traj.id is " << traj.id << std::endl;}

    trajs_.push_back(traj_compiled);
  }
  else
  {  // if it doesn't exist or traj.is_committed == false, then add it to the local map
    trajs_.push_back(traj_compiled);
    // std::cout << "traj_compiled.id is " << traj_compiled.id << std::endl;
    // for (auto traj : trajs_){std::cout << "traj.id is " << traj.id << std::endl;}
    // ROS_WARN_STREAM("Adding " << traj_compiled.id);
  }

  // and now let's delete those trajectories of the obs/agents whose current positions are outside the local map
  // Note that these positions are obtained with the trajectory stored in the past in the local map

  std::vector<mt::dynTrajCompiled> local_trajs;

  for (auto traj_compiled : trajs_)
  {
    Eigen::Vector3d center_obs = centerOfObstacle(traj_compiled, ros::Time::now().toSec());

    if (((traj_compiled.is_static == true) && (center_obs - state_.pos).norm() > 2 * par_.Ra) ||  ////
        ((traj_compiled.is_static == false) && (center_obs - state_.pos).norm() > 4 * par_.Ra))
    // #### Static Obstacle: 2*Ra because: traj_{k-1} is inside a sphere of Ra.
    // Then, in iteration k the point A (which I don't
    // know yet)  is taken along that trajectory, and
    // another trajectory of radius Ra will be obtained.
    // Therefore, I need to take 2*Ra to make sure the
    // extreme case (A taken at the end of traj_{k-1} is
    // covered).

    // #### Dynamic Agent: 4*Ra. Same reasoning as above, but with two agets
    // #### Dynamic Obstacle: 4*Ra, it's a heuristics.

    // ######REMEMBER######
    // Note that removeTrajsThatWillNotAffectMe will later
    // on take care of deleting the ones I don't need once
    // I know A
    {
      // if it's too far away then do nothing
    }
    else
    {
      local_trajs.push_back(traj_compiled);
    }
  }

  trajs_ = local_trajs;

  // print out trajs_
  // std::cout << "---------------------------------------------------------"<<std::endl;

  // for (int jj=1; jj<=10; jj++){
  //     std::cout << "Agent" << jj<<std::endl;
  //     for (auto traj_compiled : trajs_)
  //     {
  //       if(traj_compiled.id==jj){
  //       std::cout<< "     is_committed: " << traj_compiled.is_committed << std::endl;
  //    }
  //   }
  // }

  // std::cout << "bef mtx_trajs_.unlock() in updateTrajObstacles" << std::endl;
  mtx_trajs_.unlock();
  // std::cout << "aft mtx_trajs_.unlock() in updateTrajObstacles" << std::endl;
}

void Rmader::updateTrajObstacles(mt::dynTraj traj)
{
  MyTimer tmp_t(true);

  if (started_check_ == true && traj.is_agent == true)
  {
    have_received_trajectories_while_checking_ = true;
  }

  mtx_trajs_.lock();

  std::vector<mt::dynTrajCompiled>::iterator obs_ptr =
      std::find_if(trajs_.begin(), trajs_.end(),
                   [=](const mt::dynTrajCompiled& traj_compiled) {
                     return traj_compiled.id == traj.id && !traj_compiled.is_observed;
                   });

  bool exists_in_local_map = (obs_ptr != std::end(trajs_));

  mt::dynTrajCompiled traj_compiled;
  dynTraj2dynTrajCompiled(traj, traj_compiled);

  if (traj.is_agent)
  {
    Peer& peer = peers_[traj.id];
    peer.last_heard = std::max(peer.last_heard, traj.time_received);
    if (traj.is_committed)
    {
      peer.last_committed = traj_compiled;
      peer.has_trajectory = true;
    }
  }

  if (exists_in_local_map)
  {  // if that object already exists, substitute its trajectory
    *obs_ptr = traj_compiled;
  }
  else
  {  // if it doesn't exist, add it to the local map
    trajs_.push_back(traj_compiled);
    // ROS_WARN_STREAM("Adding " << traj_compiled.id);
  }

  // and now let's delete those trajectories of the obs/agents whose current positions are outside the local map
  // Note that these positions are obtained with the trajectory stored in the past in the local map
  std::vector<mt::dynTrajCompiled> local_trajs;

  for (auto traj_compiled : trajs_)
  {
    Eigen::Vector3d center_obs = centerOfObstacle(traj_compiled, ros::Time::now().toSec());

    if (((traj_compiled.is_static == true) && (center_obs - state_.pos).norm() > 2 * par_.Ra) ||  ////
        ((traj_compiled.is_static == false) && (center_obs - state_.pos).norm() > 4 * par_.Ra))
    // #### Static Obstacle: 2*Ra because: traj_{k-1} is inside a sphere of Ra.
    // Then, in iteration k the point A (which I don't
    // know yet)  is taken along that trajectory, and
    // another trajectory of radius Ra will be obtained.
    // Therefore, I need to take 2*Ra to make sure the
    // extreme case (A taken at the end of traj_{k-1} is
    // covered).

    // #### Dynamic Agent: 4*Ra. Same reasoning as above, but with two agets
    // #### Dynamic Obstacle: 4*Ra, it's a heuristics.

    // ######REMEMBER######
    // Note that removeTrajsThatWillNotAffectMe will later
    // on take care of deleting the ones I don't need once
    // I know A
    {
      // if it's too far away then do nothing
    }
    else
    {
      local_trajs.push_back(traj_compiled);
    }
  }

  trajs_ = local_trajs;

  mtx_trajs_.unlock();

  have_received_trajectories_while_checking_ = false;
  // std::cout << bold << blue << "updateTrajObstacles took " << tmp_t << reset << std::endl;
}

// ---------------------------------------------------------------------------
// Teammates that stop talking (see mt::peerSilence)
// ---------------------------------------------------------------------------

void Rmader::notePeerHeard(int id, double time)
{
  mtx_trajs_.lock();
  Peer& peer = peers_[id];
  peer.last_heard = std::max(peer.last_heard, time);
  mtx_trajs_.unlock();
}

bool Rmader::holdForQuietPeers()
{
  if (!par_.peer_silence.enabled)
  {
    return false;
  }
  const double now = ros::Time::now().toSec();
  bool hold = false;

  mtx_trajs_.lock();
  silent_peers_.clear();
  for (auto it = peers_.begin(); it != peers_.end();)
  {
    const int id = it->first;
    Peer& peer = it->second;
    const double age = now - peer.last_heard;

    if (par_.peer_silence.forget_after > 0.0 && age > par_.peer_silence.forget_after)
    {
      trajs_.erase(std::remove_if(trajs_.begin(), trajs_.end(),
                                  [&](const mt::dynTrajCompiled& traj) { return traj.is_agent && traj.id == id; }),
                   trajs_.end());
      it = peers_.erase(it);
      continue;
    }
    if (age > par_.peer_silence.silent_after)
    {
      silent_peers_.insert(id);
      // Its trajectory may have been dropped from the local map while it did not matter. It matters now.
      const bool in_local_map = std::any_of(trajs_.begin(), trajs_.end(), [&](const mt::dynTrajCompiled& traj) {
        return traj.is_agent && traj.id == id;
      });
      if (!in_local_map && peer.has_trajectory)
      {
        trajs_.push_back(peer.last_committed);
      }
    }
    else if (age > par_.peer_silence.hold_after)
    {
      hold = true;
    }
    ++it;
  }
  mtx_trajs_.unlock();
  return hold;
}

std::vector<int> Rmader::getSilentPeers()
{
  mtx_trajs_.lock();
  std::vector<int> ids(silent_peers_.begin(), silent_peers_.end());
  mtx_trajs_.unlock();
  return ids;
}

// Everywhere a silent teammate can be during [t_start, t_end]: it was on the trajectory we hold when we last
// heard from it, and since then it may have committed to anything its speed limit allows.
dynobs::Box Rmader::silentPeerBox(mt::dynTrajCompiled& traj, double t_start, double t_end)
{
  auto known = peers_.find(traj.id);
  const double t_heard = (known == peers_.end()) ? traj.time_received : known->second.last_heard;

  dynobs::Track track;
  track.id = traj.id;
  track.stamp = t_heard;
  track.pos = traj.pwp.eval(t_heard);
  track.bbox = traj.bbox;
  track.max_vel = (par_.peer_silence.peer_max_vel >= 0.0) ? par_.peer_silence.peer_max_vel : par_.v_max.maxCoeff();
  track.is_dynamic = true;

  dynobs::Params reachable = par_.dyn;
  reachable.mode = dynobs::Mode::REACHABLE;
  const Eigen::Vector3d margin =
      traj.bbox / 2.0 + par_.drone_bbox / 2.0 + (par_.beta + par_.alpha) * Eigen::Vector3d::Ones();
  return dynobs::centerBox(track, t_start, t_end, reachable, margin);
}

// ---------------------------------------------------------------------------
// Observation-based dynamic obstacles (see dyn_obstacles.hpp)
// ---------------------------------------------------------------------------

mt::dynTrajCompiled Rmader::compileObservedTrack(const dynobs::Track& track, double time_received)
{
  mt::dynTrajCompiled traj;
  traj.bbox = track.bbox;
  traj.id = track.id;
  traj.time_received = time_received;
  traj.time_created = track.stamp;
  traj.is_agent = false;
  traj.is_committed = true;
  traj.traj_id = track.num_observations;
  traj.is_observed = true;
  traj.track = track;
  // With only a speed bound to go by, a track that looks parked may start moving: it is never static.
  traj.is_static = track.declared_static || ((par_.dyn.mode == dynobs::Mode::PREDICTED) && !track.is_dynamic);
  return traj;
}

// Where the centre of an obstacle of the local map is (expected to be) at time t.
Eigen::Vector3d Rmader::centerOfObstacle(mt::dynTrajCompiled& traj, double t)
{
  if (traj.is_observed)
  {
    return dynobs::centerBox(traj.track, t, t, par_.dyn).center();
  }

  mtx_t_.lock();
  t_ = t;
  Eigen::Vector3d center(traj.function[0].value(), traj.function[1].value(), traj.function[2].value());
  mtx_t_.unlock();
  return center;
}

void Rmader::updateObservedObstacle(const dynobs::Observation& obs)
{
  if (!par_.dyn.enabled)
  {
    return;
  }

  mtx_trajs_.lock();

  if (obs_tracker_.update(obs))  // false for an out-of-order observation
  {
    dynobs::Track track;
    obs_tracker_.get(obs.id, track);
    mt::dynTrajCompiled traj_compiled = compileObservedTrack(track, ros::Time::now().toSec());

    auto it = std::find_if(trajs_.begin(), trajs_.end(), [&](const mt::dynTrajCompiled& traj) {
      return traj.is_observed && traj.id == traj_compiled.id;
    });
    if (it != trajs_.end())
    {
      *it = traj_compiled;
    }
    else
    {
      trajs_.push_back(traj_compiled);
    }
  }

  mtx_trajs_.unlock();
}

std::vector<dynobs::Track> Rmader::getObservedTracks()
{
  mtx_trajs_.lock();
  std::vector<dynobs::Track> tracks = obs_tracker_.tracks();
  mtx_trajs_.unlock();
  return tracks;
}

bool Rmader::hasObservedObstacles()
{
  mtx_trajs_.lock();
  bool result = !obs_tracker_.tracks().empty();
  mtx_trajs_.unlock();
  return result;
}

// Radius of the planning sphere while observed obstacles are tracked, see Params::max_horizon.
double Rmader::observedObstacleHorizon()
{
  if (!par_.dyn.enabled || par_.dyn.max_horizon <= 0.0 || !hasObservedObstacles())
  {
    return std::numeric_limits<double>::max();
  }
  return std::max(par_.v_max.maxCoeff() * par_.dyn.max_horizon, 3.0 * par_.goal_radius);
}

// Plan duration = this factor times the minimum time of a double integrator.
double Rmader::timeAllocationFactor(double dist_to_terminal_goal)
{
  double factor = (dist_to_terminal_goal > par_.dist_factor_alloc_close) ? par_.factor_alloc : par_.factor_alloc_close;
  if (par_.dyn.enabled && par_.dyn.max_time_factor > 0.0 && hasObservedObstacles())
  {
    factor = std::min(factor, std::max(1.0, par_.dyn.max_time_factor));
  }
  return factor;
}

// SANDO's global step (paper Section V): rather than aiming straight at the terminal goal, aim along a path
// that goes around where the observed obstacles are now and prefers to stay off where they may go. G is the
// point `ra` along that path, walked back until it is outside every box the obstacles can reach by the time
// the plan ends (the paper's safe-subgoal idea, Section IV-C, applied to tracked obstacles).
Rmader::Subgoal Rmader::subgoalAroundObservedObstacles(const mt::state& A, const mt::state& G_term, double ra,
                                                       double t_start, Eigen::Vector3d& G)
{
  if (!par_.dyn.enabled || !par_.dyn.global_path_enabled || ra <= 0.0)
  {
    return Subgoal::UNCHANGED;
  }
  const std::vector<dynobs::Track> tracks = getObservedTracks();
  if (tracks.empty())
  {
    return Subgoal::UNCHANGED;
  }

  dynobs::GlobalPathRequest request;
  request.start = A.pos;
  request.goal = G_term.pos;
  request.lower << par_.x_min, par_.y_min, par_.z_min;
  request.upper << par_.x_max, par_.y_max, par_.z_max;
  request.agent_half_size = par_.drone_bbox / 2.0;
  request.planar = par_.ground_robot;

  std::vector<Eigen::Vector3d> path;
  if (!dynobs::planGlobalPath(request, tracks, par_.dyn, path))
  {
    return Subgoal::UNCHANGED;
  }

  const double factor_alloc = timeAllocationFactor((G_term.pos - A.pos).norm());
  const Eigen::Vector3d clearance =
      par_.drone_bbox / 2.0 + (par_.beta + par_.alpha + par_.dyn.planning_margin) * Eigen::Vector3d::Ones();
  const double step = par_.dyn.global_resolution;

  for (double s = std::min(ra, dynobs::pathLength(path)); s > 0.5 * step; s -= step)
  {
    const Eigen::Vector3d candidate = dynobs::pointAlongPath(path, s);
    const double duration = factor_alloc * mu::getMinTimeDoubleIntegrator3D(A.pos, A.vel, candidate,
                                                                            Eigen::Vector3d::Zero(), par_.v_max,
                                                                            par_.a_max);
    const double t_end = t_start + duration;
    const double dwell = std::max(duration / par_.num_pol, par_.dyn.stop_dwell);
    bool is_free = true;
    for (const dynobs::Track& track : tracks)
    {
      if (dynobs::occupancyBox(track, t_end, t_end + dwell, par_.dyn, clearance).contains(candidate))
      {
        is_free = false;
        break;
      }
    }
    if (is_free)
    {
      G = candidate;
      return Subgoal::FOUND;
    }
  }
  // Every point ahead can be reached by some obstacle before the robot could have stopped there and stood
  // for stop_dwell. Creeping forward anyway is how a robot ends up parked among the obstacles.
  return Subgoal::NONE_SAFE;
}

// Stopped somewhere other than the goal (the committed plan ran out and replanning has not produced a new
// one) with an observed obstacle closing in: head out of its path first. Same rule as the evasion at the
// goal, applied to this plan's subgoal instead of the terminal goal.
bool Rmader::evasionSubgoal(const mt::state& A, double ra, Eigen::Vector3d& G)
{
  if (!par_.dyn.enabled || !par_.dyn.hover_avoidance_enabled)
  {
    return false;
  }
  if (A.vel.norm() > 0.05 || state_.vel.norm() > 0.05 || (A.pos - state_.pos).norm() > 0.2)
  {
    return false;  // still moving, or still has plan left to execute
  }
  Eigen::Vector3d away;
  if (!dynobs::hoverEvasionGoal(A.pos, ros::Time::now().toSec(), getObservedTracks(), par_.dyn, away))
  {
    return false;
  }
  // No further than one planning horizon, and inside the arena.
  const double reach = std::min(par_.dyn.hover_evasion_dist, std::min(par_.Ra, observedObstacleHorizon()));
  away = A.pos + reach * (away - A.pos).normalized();
  away.x() = std::min(std::max(away.x(), par_.x_min), par_.x_max);
  away.y() = std::min(std::max(away.y(), par_.y_min), par_.y_max);
  away.z() = std::min(std::max(away.z(), par_.z_min), par_.z_max);
  if ((away - A.pos).norm() < par_.goal_radius)
  {
    return false;  // cornered against the arena limits
  }
  G = away;
  ROS_INFO_STREAM_THROTTLE(1.0, "Observed obstacle closing in while stopped: stepping aside to " << G.transpose());
  return true;
}

// See Params::escape_when_engulfed. Returns true (and sets observed_escape_ and G) if the start of this plan
// is inside the box some observed obstacle gets for the plan's first time slice.
bool Rmader::escapeSubgoal(const mt::state& A, const Eigen::Vector3d& G_straight, double t_start, Eigen::Vector3d& G)
{
  observed_escape_ = false;
  if (!par_.dyn.enabled || !par_.dyn.escape_when_engulfed || A.vel.norm() > 0.05 || state_.vel.norm() > 0.05)
  {
    return false;
  }

  const double duration = mu::getMinTimeDoubleIntegrator3D(A.pos, A.vel, G_straight, Eigen::Vector3d::Zero(),
                                                          par_.v_max, par_.a_max);
  const Eigen::Vector3d clearance =
      par_.drone_bbox / 2.0 + (par_.beta + par_.alpha + par_.dyn.planning_margin) * Eigen::Vector3d::Ones();

  Eigen::Vector3d away = Eigen::Vector3d::Zero();
  Eigen::Vector3d sidestep = Eigen::Vector3d::UnitX();
  bool engulfed = false;
  for (const dynobs::Track& track : getObservedTracks())
  {
    if (!dynobs::occupancyBox(track, t_start, t_start + duration / par_.num_pol, par_.dyn, clearance).contains(A.pos))
    {
      continue;
    }
    if (!engulfed && track.vel.head<2>().norm() > 1e-6)
    {
      sidestep = Eigen::Vector3d(-track.vel.y(), track.vel.x(), 0.0).normalized();
    }
    engulfed = true;
    Eigen::Vector3d from_obstacle = A.pos - track.pos;
    if (par_.ground_robot || par_.dyn.hover_avoidance_2d)
    {
      from_obstacle.z() = 0.0;
    }
    if (from_obstacle.norm() > 1e-6)
    {
      away += from_obstacle.normalized() / std::max(from_obstacle.norm(), 0.1);
    }
  }
  if (!engulfed)
  {
    return false;
  }

  const Eigen::Vector3d direction = (away.norm() > 1e-3) ? away.normalized() : sidestep;
  Eigen::Vector3d target = A.pos + par_.dyn.escape_dist * direction;
  target.x() = std::min(std::max(target.x(), par_.x_min), par_.x_max);
  target.y() = std::min(std::max(target.y(), par_.y_min), par_.y_max);
  target.z() = std::min(std::max(target.z(), par_.z_min), par_.z_max);
  if ((target - A.pos).norm() < par_.goal_radius)
  {
    return false;  // cornered
  }
  G = target;
  observed_escape_ = true;
  ROS_WARN_STREAM_THROTTLE(1.0, "Standing inside an observed obstacle's box: escaping to " << G.transpose()
                                    << " against the obstacles' bodies only");
  return true;
}

// A stopped rover cannot leave towards a point outside its heading cone, and the optimiser would just keep
// failing. Ask whatever tracks the setpoints to turn it on the spot (setpoint yaw, zero velocity) and plan
// once it faces the right way. The simulator bridge does the same before sending a goal behind the rover;
// this covers subgoals the planner picks itself.
bool Rmader::groundTurnNeeded(const mt::state& A, const mt::state& G)
{
  if (!par_.ground_robot)
  {
    return false;
  }
  const Eigen::Vector2d to_goal = (G.pos - A.pos).head<2>();
  const bool stopped = A.vel.head<2>().norm() < 0.05 && state_.vel.head<2>().norm() < 0.05;
  if (!stopped || to_goal.norm() < 1e-3)
  {
    ground_turn_requested_ = false;
    return false;
  }
  const double wanted = std::atan2(to_goal.y(), to_goal.x());
  double error = wanted - state_.yaw;
  mu::angle_wrap(error);
  // Hysteresis: start turning when most of the cone is used up, stop once well inside it.
  const double limit = (ground_turn_requested_ ? 0.4 : 0.8) * par_.ground_heading_cone_rad;
  ground_turn_requested_ = std::fabs(error) > limit;
  ground_turn_yaw_ = wanted;
  return ground_turn_requested_;
}

bool Rmader::isEvading()
{
  return is_evading_;
}

// Forgets the obstacles that have not been observed for a while. A track that is still alive is put back in
// the local map if an earlier replan dropped it for being out of reach: it may have come closer since.
void Rmader::pruneObservedObstacles(double now)
{
  if (!par_.dyn.enabled)
  {
    return;
  }

  obs_tracker_.prune(now);

  trajs_.erase(std::remove_if(trajs_.begin(), trajs_.end(),
                              [&](const mt::dynTrajCompiled& traj) {
                                dynobs::Track track;
                                return traj.is_observed && !obs_tracker_.get(traj.id, track);
                              }),
               trajs_.end());

  for (const dynobs::Track& track : obs_tracker_.tracks())
  {
    bool in_local_map = std::any_of(trajs_.begin(), trajs_.end(), [&](const mt::dynTrajCompiled& traj) {
      return traj.is_observed && traj.id == track.id;
    });
    if (!in_local_map)
    {
      trajs_.push_back(compileObservedTrack(track, now));
    }
  }
}

// Biases the A* guess away from the observed obstacles and from where they are predicted to go.
void Rmader::setHeatCostForSolver()
{
#if !USE_GUROBI_FLAG
  const std::vector<dynobs::Track> tracks = obs_tracker_.tracks();
  if (!par_.dyn.enabled || par_.dyn.heat_weight <= 0.0 || tracks.empty())
  {
    solver_->setHeatCost(nullptr);
    return;
  }
  const dynobs::Params dyn = par_.dyn;
  solver_->setHeatCost(
      [tracks, dyn](const Eigen::Vector3d& q, double) { return dyn.heat_weight * dynobs::heat(q, tracks, dyn); });
#endif
}

// SANDO-style hover avoidance: an agent that is stopped at its goal gets out of the way of an observed
// obstacle that is about to come too close, and returns to the goal once the goal is clear again.
void Rmader::updateHoverAvoidance()
{
  if (!par_.dyn.enabled || !par_.dyn.hover_avoidance_enabled || initializedStateAndTermGoal() == false ||
      drone_status_ != DroneStatus::GOAL_REACHED)
  {
    return;
  }

  const double now = ros::Time::now().toSec();
  const std::vector<dynobs::Track> tracks = getObservedTracks();
  const Eigen::Vector3d here = state_.pos;

  mt::state target;
  bool move = false;

  if (dynobs::hoverEvasionGoal(here, now, tracks, par_.dyn, target.pos))
  {
    if (!is_evading_)
    {
      hover_home_ = G_term_;
    }
    target.pos.x() = std::min(std::max(target.pos.x(), par_.x_min), par_.x_max);
    target.pos.y() = std::min(std::max(target.pos.y(), par_.y_min), par_.y_max);
    target.pos.z() = std::min(std::max(target.pos.z(), par_.z_min), par_.z_max);
    is_evading_ = true;
    move = (target.pos - here).norm() > par_.goal_radius;
    std::cout << bold << "Observed obstacle too close while stopped: evading to " << target.pos.transpose() << reset
              << std::endl;
  }
  else if (is_evading_)
  {
    Eigen::Vector3d unused;
    if (!dynobs::hoverEvasionGoal(hover_home_.pos, now, tracks, par_.dyn, unused))
    {
      target = hover_home_;
      is_evading_ = false;
      move = true;
      std::cout << bold << "Goal is clear again: returning to " << target.pos.transpose() << reset << std::endl;
    }
  }

  if (move)
  {
    setTerminalGoalInternal(target);
  }
}

std::vector<Eigen::Vector3d> Rmader::vertexesOfInterval(mt::PieceWisePol& pwp, double t_start, double t_end,
                                                        const Eigen::Vector3d& delta)
{
  std::vector<Eigen::Vector3d> points;

  std::vector<double>::iterator low = std::lower_bound(pwp.times.begin(), pwp.times.end(), t_start);
  std::vector<double>::iterator up = std::upper_bound(pwp.times.begin(), pwp.times.end(), t_end);

  // // Example: times=[1 2 3 4 5 6 7]
  // // t_start=1.5;
  // // t_end=5.5
  // // then low points to "2" (low - pwp.times.begin() is 1)
  // // and up points to "6" (up - pwp.times.begin() is 5)

  int index_first_interval = low - pwp.times.begin() - 1;  // index of the interval [1,2]
  int index_last_interval = up - pwp.times.begin() - 1;    // index of the interval [5,6]

  mu::saturate(index_first_interval, 0, (int)(pwp.coeff_x.size() - 1));
  mu::saturate(index_last_interval, 0, (int)(pwp.coeff_x.size() - 1));

  // int index_first_interval = 0;
  // int index_last_interval = static_cast<int>(pwp.coeff_x.size() - 1);

  Eigen::Matrix<double, 3, 4> P;
  Eigen::Matrix<double, 3, 4> V;

  // push all the complete intervals
  for (int i = index_first_interval; i <= index_last_interval; i++)
  {
    P.row(0) = pwp.coeff_x[i];
    P.row(1) = pwp.coeff_y[i];
    P.row(2) = pwp.coeff_z[i];

    V = P * A_rest_pos_basis_inverse_;

    for (int j = 0; j < V.cols(); j++)
    {
      double x = V(0, j);
      double y = V(1, j);
      double z = V(2, j);  //[x,y,z] is the point

      if (delta.norm() < 1e-6)
      {  // no inflation
        points.push_back(Eigen::Vector3d(x, y, z));
      }
      else
      {
        // points.push_back(Eigen::Vector3d(V(1, j), V(2, j), V(3, j)));  // x,y,z

        if (j == V.cols() - 1)
        {
          points.push_back(Eigen::Vector3d(x + delta.x(), y + delta.y(), z + delta.z()));
          points.push_back(Eigen::Vector3d(x + delta.x(), y - delta.y(), z - delta.z()));
          points.push_back(Eigen::Vector3d(x + delta.x(), y + delta.y(), z - delta.z()));
          points.push_back(Eigen::Vector3d(x + delta.x(), y - delta.y(), z + delta.z()));
          points.push_back(Eigen::Vector3d(x - delta.x(), y - delta.y(), z - delta.z()));
          points.push_back(Eigen::Vector3d(x - delta.x(), y + delta.y(), z + delta.z()));
          points.push_back(Eigen::Vector3d(x - delta.x(), y + delta.y(), z - delta.z()));
          points.push_back(Eigen::Vector3d(x - delta.x(), y - delta.y(), z + delta.z()));
        }
        else
        {
          points.push_back(Eigen::Vector3d(x, y, z));
        }
      }
    }
  }

  return points;
}

// return a vector that contains all the vertexes of the polyhedral approx of an interval.
std::vector<Eigen::Vector3d> Rmader::vertexesOfInterval(mt::dynTrajCompiled& traj, double t_start, double t_end)
{
  Eigen::Vector3d delta = Eigen::Vector3d::Zero();
  Eigen::Vector3d drone_boundarybox = par_.drone_bbox;

  if (traj.is_observed && observed_escape_)
  {
    // Params::escape_when_engulfed: the obstacle's body as last seen, the agent's size and the observation
    // error. Nothing else.
    dynobs::Box body;
    Eigen::Vector3d half = traj.track.bbox / 2.0 + drone_boundarybox / 2.0 +
                           par_.dyn.obst_position_error * Eigen::Vector3d::Ones();
    body.lo = traj.track.pos - half;
    body.hi = traj.track.pos + half;
    return body.vertexes();
  }

  if (traj.is_observed)
  {
    // Everything the obstacle can occupy during the interval, grown by the time elapsed since it was last
    // seen, plus the same clearance the broadcast obstacles get below.
    Eigen::Vector3d clearance = drone_boundarybox / 2.0 +
                                (par_.beta + par_.alpha + observed_extra_margin_) * Eigen::Vector3d::Ones();
    return dynobs::occupancyBox(traj.track, t_start, t_end, par_.dyn, clearance).vertexes();
  }

  if (traj.is_agent == false)
  {
    std::vector<Eigen::Vector3d> points;
    // delta = traj.bbox / 2.0 + (par_.drone_radius + par_.beta + par_.alpha) *
    //                            Eigen::Vector3d::Ones();  // every side of the box will be increased by 2*delta
    //(+delta on one end, -delta on the other)

    // changeBBox(drone_boundarybox);

    delta = traj.bbox / 2.0 + drone_boundarybox / 2.0 + (par_.beta + par_.alpha) * Eigen::Vector3d::Ones();
    // std::cout << "boundary box size" << std::endl;
    // std::cout << drone_boundarybox[0] << std::endl;
    // std::cout << drone_boundarybox[1] << std::endl;
    // std::cout << drone_boundarybox[2] << std::endl;

    // Will always have a sample at the beginning of the interval, and another at the end.
    for (double t = t_start;                           /////////////
         (t < t_end) ||                                /////////////
         ((t > t_end) && ((t - t_end) < par_.gamma));  /////// This is to ensure we have a sample a the end
         t = t + par_.gamma)
    {
      mtx_t_.lock();
      t_ = std::min(t, t_end);  // this min only has effect on the last sample

      double x = traj.function[0].value();
      double y = traj.function[1].value();
      double z = traj.function[2].value();
      mtx_t_.unlock();

      //"Minkowski sum along the trajectory: box centered on the trajectory"
      points.push_back(Eigen::Vector3d(x + delta.x(), y + delta.y(), z + delta.z()));
      points.push_back(Eigen::Vector3d(x + delta.x(), y - delta.y(), z - delta.z()));
      points.push_back(Eigen::Vector3d(x + delta.x(), y + delta.y(), z - delta.z()));
      points.push_back(Eigen::Vector3d(x + delta.x(), y - delta.y(), z + delta.z()));
      points.push_back(Eigen::Vector3d(x - delta.x(), y - delta.y(), z - delta.z()));
      points.push_back(Eigen::Vector3d(x - delta.x(), y + delta.y(), z + delta.z()));
      points.push_back(Eigen::Vector3d(x - delta.x(), y + delta.y(), z - delta.z()));
      points.push_back(Eigen::Vector3d(x - delta.x(), y - delta.y(), z + delta.z()));
    }

    return points;
  }
  else if (silent_peers_.count(traj.id) > 0)
  {
    return silentPeerBox(traj, t_start, t_end).vertexes();
  }
  else
  {  // is an agent --> use the pwp field

    // delta = traj.bbox / 2.0 + (par_.drone_radius) * Eigen::Vector3d::Ones();
    // delta = traj.bbox / 2.0 + par_.drone_bbox / 2.0;  // instad of using drone_radius

    // changeBBox(drone_boundarybox);

    delta = traj.bbox / 2.0 + drone_boundarybox / 2.0 + (par_.beta + par_.alpha) * Eigen::Vector3d::Ones();
    // std::cout << "boundary box size" << std::endl;
    // std::cout << drone_boundarybox[0] << std::endl;
    // std::cout << drone_boundarybox[1] << std::endl;
    // std::cout << drone_boundarybox[2] << std::endl;

    // std::cout << "****traj.bbox = " << traj.bbox << std::endl;
    // std::cout << "****par_.drone_radius = " << par_.drone_radius << std::endl;
    // std::cout << "****Inflation by delta= " << delta.transpose() << std::endl;

    return vertexesOfInterval(traj.pwp, t_start, t_end, delta);
  }
}

void Rmader::changeBBox(Eigen::Vector3d& drone_boundarybox)
{
  Eigen::Vector3d v(1e-5, 1e-5, 1e-5);  // how much you make the box small?
  Eigen::Vector3d bbox_min_possible(par_.drone_bbox[0] - 0.02, par_.drone_bbox[1] - 0.02, par_.drone_bbox[2] - 0.02);
  // double bbox_change_time = 1; //seconds, used for timer
  double unstuck_dist = 1.0;            // meters
  Eigen::Vector3d dist(1e5, 1e5, 1e5);  // random big numer, need to initialized before used

  if (par_.is_stuck || if_bbox_change_ || is_A_star_failed_30_)
  {
    if (!if_bbox_change_)
    {
      // timer_bbox_.Reset(); // start timer
      stuck_state_for_bbox_ = state_;  // get the current position
      // measure the progress
    }

    dist = state_.pos - stuck_state_for_bbox_.pos;
    // std::cout << "boundary box change" << std::endl;
    // std::cout << "dist is " << dist << std::endl;
    // std::cout << "state_ is " << state_.pos << std::endl;
    // std::cout << "stuck_state_for_bbox_ is " << stuck_state_for_bbox_.pos << std::endl;

    // if (timer_bbox_.ElapsedMs() < bbox_change_time * 1000){ //using timer
    // if (!par_.is_stuck){ // TODO: measure the progress like position and if the drone moves certain distance, put the
    // bbox size back

    if (dist.norm() < unstuck_dist)
    {
      drone_boundarybox = par_.drone_bbox - (stuck_count_for_bbox_ + 1) * v;
      // std::cout << "drone_bbox[0] is " << drone_boundarybox[0] << std::endl;
      // std::cout << "drone_bbox[1] is " << drone_boundarybox[1] << std::endl;
      // std::cout << "drone_bbox[2] is " << drone_boundarybox[2] << std::endl;

      for (int i = 0; i < 3; i++)
      {
        drone_boundarybox[i] = std::max(drone_boundarybox[i], bbox_min_possible[i]);
      };
      // std::cout << "using smaller bbox" << std::endl;
      // std::cout << "stuck count is " << stuck_count_for_bbox_ << std::endl;
      // std::cout << "par_.is_stuck is " << par_.is_stuck << std::endl;
      stuck_count_for_bbox_ = stuck_count_for_bbox_ + 1;
      if_bbox_change_ = true;
    }
    else
    {
      stuck_count_for_bbox_ = 0;
      if_bbox_change_ = false;
    }
  }
  else
  {
    drone_boundarybox = par_.drone_bbox;
    stuck_count_for_bbox_ = 0;
  }
}

// See https://doc.cgal.org/Manual/3.7/examples/Convex_hull_3/quickhull_3.cpp
CGAL_Polyhedron_3 Rmader::convexHullOfInterval(mt::dynTrajCompiled& traj, double t_start, double t_end)
{
  // std::cout << "1.1.4" << std::endl;

  std::vector<Eigen::Vector3d> points = vertexesOfInterval(traj, t_start, t_end);

  // std::cout << "1.1.5" << std::endl;

  std::vector<Point_3> points_cgal;
  for (auto point_i : points)
  {
    points_cgal.push_back(Point_3(point_i.x(), point_i.y(), point_i.z()));
  }

  // std::cout << "1.1.6" << std::endl;

  return cu::convexHullOfPoints(points_cgal);
}

// trajs_ is already locked when calling this function
void Rmader::removeTrajsThatWillNotAffectMe(const mt::state& A, double t_start, double t_end)
{
  std::vector<int> ids_to_remove;

  std::vector<Eigen::Vector3d> pointsB;
  Eigen::Vector3d delta = par_.Ra * Eigen::Vector3d::Ones();

  pointsB.push_back(Eigen::Vector3d(A.pos.x() + delta.x(), A.pos.y() + delta.y(), A.pos.z() + delta.z()));
  pointsB.push_back(Eigen::Vector3d(A.pos.x() + delta.x(), A.pos.y() - delta.y(), A.pos.z() - delta.z()));
  pointsB.push_back(Eigen::Vector3d(A.pos.x() + delta.x(), A.pos.y() + delta.y(), A.pos.z() - delta.z()));
  pointsB.push_back(Eigen::Vector3d(A.pos.x() + delta.x(), A.pos.y() - delta.y(), A.pos.z() + delta.z()));
  pointsB.push_back(Eigen::Vector3d(A.pos.x() - delta.x(), A.pos.y() - delta.y(), A.pos.z() - delta.z()));
  pointsB.push_back(Eigen::Vector3d(A.pos.x() - delta.x(), A.pos.y() + delta.y(), A.pos.z() + delta.z()));
  pointsB.push_back(Eigen::Vector3d(A.pos.x() - delta.x(), A.pos.y() + delta.y(), A.pos.z() - delta.z()));
  pointsB.push_back(Eigen::Vector3d(A.pos.x() - delta.x(), A.pos.y() - delta.y(), A.pos.z() + delta.z()));

  std::vector<mt::dynTrajCompiled> local_trajs;

  for (auto traj : trajs_)
  {
    bool traj_affects_me = false;

    // A silent teammate has no trajectory to test: its box only grows, and nothing will re-add it later.
    if (traj.is_agent && silent_peers_.count(traj.id) > 0)
    {
      local_trajs.push_back(traj);
      continue;
    }

    // STATIC OBSTACLES/AGENTS
    if (traj.is_static == true)
    {
      mtx_t_.lock();
      t_ = t_start;  // which is constant along the trajectory

      Eigen::Vector3d center_obs;
      if (traj.is_observed)
      {
        center_obs = traj.track.pos;
      }
      else if (traj.is_agent == false)
      {
        center_obs << traj.function[0].value(), traj.function[1].value(), traj.function[2].value();
      }
      else
      {
        center_obs = traj.pwp.eval(t_);
      }

      mtx_t_.unlock();
      // mtx_t_.unlock();
      Eigen::Vector3d positive_half_diagonal;
      positive_half_diagonal << traj.bbox[0] / 2.0, traj.bbox[1] / 2.0, traj.bbox[2] / 2.0;

      Eigen::Vector3d c1 = center_obs - positive_half_diagonal;
      Eigen::Vector3d c2 = center_obs + positive_half_diagonal;
      traj_affects_me = mu::boxIntersectsSphere(A.pos, par_.Ra, c1, c2);
    }
    else
    {                                                            // DYNAMIC OBSTACLES/AGENTS
      double deltaT = (t_end - t_start) / (1.0 * par_.num_pol);  // num_pol is the number of intervals
      for (int i = 0; i <= par_.num_pol; i++)                    // for each interval
      {
        std::vector<Eigen::Vector3d> pointsA =
            vertexesOfInterval(traj, t_start + i * deltaT, t_start + (i + 1) * deltaT);
        // Now we check whether the bbox centered in A (and total_sizes=2*Ra) collides with the polyhedron whose
        // vertexes are pointsA

        // To do that, we solve a linear feasibility program to find a plane that separates that box and the
        // polyhedron

        Eigen::Vector3d n;
        double d;

        bool solved = separator_solver_->solveModel(n, d, pointsA, pointsB);

        if (solved == false)
        {
          traj_affects_me = true;
          goto exit;
        }
      }
    }

  exit:

    if (traj_affects_me == false)
    {
      // std::cout << red << bold << "Going to  delete traj " << trajs_[index_traj].id << reset << std::endl;
      // ids_to_remove.push_back(traj.id);
    }
    else
    {
      local_trajs.push_back(traj);
    }
  }

  trajs_ = local_trajs;

  /*  std::cout << "After deleting the trajectory, we have these ids= " << std::endl;

    for (auto traj : trajs_)
    {
      std::cout << traj.id << std::endl;
    }*/
}

bool Rmader::IsTranslating()
{
  return (drone_status_ == DroneStatus::GOAL_SEEN || drone_status_ == DroneStatus::TRAVELING);
}

ConvexHullsOfCurve Rmader::convexHullsOfCurve(mt::dynTrajCompiled& traj, double t_start, double t_end)
{
  ConvexHullsOfCurve convexHulls;
  double deltaT = (t_end - t_start) / (1.0 * par_.num_pol);  // num_pol is the number of intervals

  // std::cout << "1.1.3" << std::endl;

  for (int i = 0; i <= par_.num_pol; i++)
  {
    double t_b = t_start + (i + 1) * deltaT;
    if (traj.is_observed && i == par_.num_pol)
    {
      // The interval after the plan ends, when the robot is at rest at its end point: see Params::stop_dwell
      t_b = t_start + i * deltaT + std::max(deltaT, par_.dyn.stop_dwell);
    }
    convexHulls.push_back(convexHullOfInterval(traj, t_start + i * deltaT, t_b));
  }

  return convexHulls;
}

ConvexHullsOfCurves Rmader::convexHullsOfCurves(double t_start, double t_end)
{
  ConvexHullsOfCurves result;

  // std::cout << "1.1.2" << std::endl;

  for (auto traj : trajs_)
  {
    result.push_back(convexHullsOfCurve(traj, t_start, t_end));
  }

  return result;
}

void Rmader::setTerminalGoal(mt::state& term_goal)
{
  is_evading_ = false;  // a goal from outside overrides any evasion in progress
  setTerminalGoalInternal(term_goal);
}

void Rmader::setTerminalGoalInternal(mt::state& term_goal)
{
  mtx_G_term.lock();
  mtx_G.lock();
  mtx_state.lock();
  mtx_planner_status_.lock();

  G_term_.pos = term_goal.pos;
  Eigen::Vector3d temp = state_.pos;
  G_.pos = G_term_.pos;
  if (par_.ground_robot)
  {
    // A rover can rotate while the old spline still predicts translation.
    // Start a new goal from the next measured pose and heading instead of
    // carrying the previous goal's tangent into the new optimization.
    state_initialized_ = false;
  }
  if (drone_status_ == DroneStatus::GOAL_REACHED)
  {
    // changeDroneStatus(DroneStatus::YAWING);
    changeDroneStatus(DroneStatus::TRAVELING);  // skip yawing
  }
  if (drone_status_ == DroneStatus::GOAL_SEEN)
  {
    changeDroneStatus(DroneStatus::TRAVELING);
  }
  terminal_goal_initialized_ = true;

  // std::cout << bold << red << "[FA] Received Term Goal=" << G_term_.pos.transpose() << reset << std::endl;
  // std::cout << bold << red << "[FA] Received Proj Goal=" << G_.pos.transpose() << reset << std::endl;

  mtx_state.unlock();
  mtx_G.unlock();
  mtx_G_term.unlock();
  mtx_planner_status_.unlock();
}

mt::state Rmader::getGterm()
{
  return G_term_;
}

void Rmader::getG(mt::state& G)
{
  G = G_;
}

// in case drones are stuck we need a new G to detour
void Rmader::getDetourG(mt::state& G)
{
  Eigen::Vector2d v = G_when_stuck_.pos.head(2) - stuck_state_.pos.head(2);
  Eigen::Vector2d v2;
  // v2 << v[1], -v[0]; // rotate a vector negative 90 deg
  // v2 << -v[0], -v[1]; // rotate a vector 180 deg
  v2 = RotationMatrix(v, -135 * M_PI / 180);  // rotate a vector -135 deg
  v2.normalize();

  double v2_mag = 4.0 * par_.drone_bbox[0];
  // G.pos.head(2) = G.pos.head(2) + v2_mag * v2; // for -90deg rotatoin
  // G.pos.head(2) = stuck_state_.pos.head(2) + v2_mag * v2; // for 180 deg rotation
  detoured_G_.pos.head(2) = stuck_state_.pos.head(2) + v2_mag * v2;
  detoured_G_.pos[2] = G_when_stuck_.pos[2];

  // if new G is outside of highbay boundary, the project new G to highbay boundary
  if (detoured_G_.pos[0] > par_.x_max - par_.drone_bbox[0])
  {
    detoured_G_.pos[0] = par_.x_max - par_.drone_bbox[0];
  }
  if (detoured_G_.pos[0] < par_.x_min + par_.drone_bbox[0])
  {
    detoured_G_.pos[0] = par_.x_min + par_.drone_bbox[0];
  }

  if (detoured_G_.pos[1] > par_.y_max - par_.drone_bbox[1])
  {
    detoured_G_.pos[1] = par_.y_max - par_.drone_bbox[1];
  }
  if (detoured_G_.pos[1] < par_.y_min + par_.drone_bbox[1])
  {
    detoured_G_.pos[1] = par_.y_min + par_.drone_bbox[1];
  }

  G = detoured_G_;

  //
  //      new G
  //        ^
  //        |
  //        |
  //     v2 | magnitude is v2_mag
  //        |
  //        |
  //        |            v
  //        G <----------------------- A (drone's current initial planning position)
  //
  //
}

void Rmader::moveAtowardG(mt::state& A, mt::state& G)
{
  Eigen::Vector2d v = G.pos.head(2) - stuck_state_.pos.head(2);
  A.pos[0] = stuck_state_.pos[0] + 0.01 * v[0];
  A.pos[1] = stuck_state_.pos[1] + 0.01 * v[1];
}

Eigen::Vector2d Rmader::RotationMatrix(Eigen::Vector2d& vec, const double& angle)
{
  // angle is radian
  Eigen::Vector2d vec2;
  vec2[0] = cos(angle) * vec[0] - sin(angle) * vec[1];
  vec2[1] = sin(angle) * vec[0] + cos(angle) * vec[1];
  return vec2;
}

void Rmader::getState(mt::state& data)
{
  mtx_state.lock();
  data = state_;
  mtx_state.unlock();
}

void Rmader::updateState(mt::state data)
{
  state_ = data;

  if (!state_initialized_)
  {
    mt::state tmp;
    tmp.pos = data.pos;
    tmp.yaw = data.yaw;
    mtx_plan_.lock();

    plan_.erase(plan_.begin(), plan_.end());
    plan_.push_back(tmp);  // plan_ should be empty

    mtx_plan_.unlock();
    // std::cout << "in updateState function ";
    // plan_.print();
    // previous_yaw_ = tmp.yaw;
    initial_yaw_ = tmp.yaw;
    state_initialized_ = true;
  }

  // printDroneStatus();

  // if (drone_status_ == DroneStatus::TRAVELING)
  // {
  // }

  /* skip yawing process
  if (drone_status_ == DroneStatus::YAWING)
  {
    state_initialized_ = true;
  }
  */
  // state_initialized_ = true;

  // mt::state tmp;
  // tmp.pos = data.pos;
  // tmp.yaw = data.yaw;
  // plan_.pop_front();
  // plan_.push_back(tmp); // plan_ should be empty
  // std::cout << "plan size " << plan_.size() << std::endl;
  // std::cout << "in updateState function ";
  // plan_.print();
  // previous_yaw_ = tmp.yaw;

  // state_.print();
}

bool Rmader::initializedAllExceptPlanner()
{
  if (!state_initialized_ || !terminal_goal_initialized_)
  {
    /*    std::cout << "state_initialized_= " << state_initialized_ << std::endl;
        std::cout << "terminal_goal_initialized_= " << terminal_goal_initialized_ << std::endl;*/
    return false;
  }
  return true;
}

bool Rmader::initializedStateAndTermGoal()
{
  if (!state_initialized_ || !terminal_goal_initialized_)
  {
    if (!state_initialized_)
    {
      // std::cout << "not initialized yet" << std::endl;
    }
    else
    {
      // std::cout << "not terminal_goal_initialized yet" << std::endl;
    }
    return false;
  }
  return true;
}

bool Rmader::initialized()
{
  if (!state_initialized_ || !terminal_goal_initialized_ || !planner_initialized_)
  {
    /*    std::cout << "state_initialized_= " << state_initialized_ << std::endl;
        std::cout << "terminal_goal_initialized_= " << terminal_goal_initialized_ << std::endl;
        std::cout << "planner_initialized_= " << planner_initialized_ << std::endl;*/
    return false;
  }
  return true;
}

std::vector<mt::dynTrajCompiled> Rmader::getTrajs()
{
  std::vector<mt::dynTrajCompiled> trajs;

  mtx_trajs_.lock();
  trajs = trajs_;
  mtx_trajs_.unlock();
  return trajs;
}

// check wheter a mt::dynTrajCompiled and a pwp_optimized are in collision in the interval [t_start, t_end]
bool Rmader::trajsAndPwpAreInCollision_with_inflation(mt::dynTrajCompiled traj, mt::PieceWisePol pwp_optimized,
                                                      double t_start, double t_end, bool& is_q0_fail)
{
  Eigen::Vector3d n_i;
  double d_i;

  double deltaT = (t_end - t_start) / (1.0 * par_.num_pol);  // num_pol is the number of intervals
  for (int i = 0; i < par_.num_pol; i++)                     // for each interval
  {
    // This is my trajectory (no inflation)
    // std::vector<Eigen::Vector3d> pointsA =
    //     vertexesOfInterval(pwp_optimized, t_start + i * deltaT, t_start + (i + 1) * deltaT, Eigen::Vector3d::Zero());

    // This is my trajectory (with inflation)
    Eigen::Vector3d inflation;
    inflation << 0.01, 0.01, 0.01;
    std::vector<Eigen::Vector3d> pointsA =
        vertexesOfInterval(pwp_optimized, t_start + i * deltaT, t_start + (i + 1) * deltaT, inflation);

    // This is the trajectory of the other agent/obstacle
    std::vector<Eigen::Vector3d> pointsB = vertexesOfInterval(traj, t_start + i * deltaT, t_start + (i + 1) * deltaT);

    // std::cout << "Going to solve model with pointsA.size()= " << pointsA.size() << std::endl;
    // for (auto point_i : pointsA)
    // {
    //   std::cout << point_i.transpose() << std::endl;
    // }

    // std::cout << "Going to solve model with pointsB.size()= " << pointsB.size() << std::endl;
    // for (auto point_i : pointsB)
    // {
    //   std::cout << point_i.transpose() << std::endl;
    // }

    if (separator_solver_->solveModel(n_i, d_i, pointsA, pointsB) == false)
    {
      if (i == 0)
      {
        is_q0_fail = true;
      }
      return true;  // There is not a solution --> they collide
    }
  }

  // if reached this point, they don't collide
  return false;
}

bool Rmader::trajsAndPwpAreInCollision(mt::dynTrajCompiled traj, mt::PieceWisePol pwp_optimized, double t_start,
                                       double t_end)
{
  Eigen::Vector3d n_i;
  double d_i;

  double deltaT = (t_end - t_start) / (1.0 * par_.num_pol);  // num_pol is the number of intervals
  for (int i = 0; i < par_.num_pol; i++)                     // for each interval
  {
    // This is my trajectory (no inflation)
    std::vector<Eigen::Vector3d> pointsA =
        vertexesOfInterval(pwp_optimized, t_start + i * deltaT, t_start + (i + 1) * deltaT, Eigen::Vector3d::Zero());

    // This is the trajectory of the other agent/obstacle
    std::vector<Eigen::Vector3d> pointsB = vertexesOfInterval(traj, t_start + i * deltaT, t_start + (i + 1) * deltaT);

    // std::cout << "Going to solve model with pointsA.size()= " << pointsA.size() << std::endl;
    // for (auto point_i : pointsA)
    // {
    //   std::cout << point_i.transpose() << std::endl;
    // }

    // std::cout << "Going to solve model with pointsB.size()= " << pointsB.size() << std::endl;
    // for (auto point_i : pointsB)
    // {
    //   std::cout << point_i.transpose() << std::endl;
    // }

    if (separator_solver_->solveModel(n_i, d_i, pointsA, pointsB) == false)
    {
      return true;  // There is not a solution --> they collide
    }
  }

  // if reached this point, they don't collide
  return false;
}

// Checks that I have not received new trajectories that affect me while doing the optimization
// Check period and Recheck period is defined here
bool Rmader::safetyCheckAfterOptForRmader(mt::PieceWisePol pwp_optimized, bool& is_q0_fail)
{
  bool result = true;
  for (auto& traj : trajs_)
  {
    // need to include the trajs that came in the last delay check, and the obstacles observed meanwhile
    if (traj.is_agent == true || traj.is_observed)
    {
      if (trajsAndPwpAreInCollision_with_inflation(traj, pwp_optimized, pwp_optimized.times.front(),
                                                   pwp_optimized.times.back(), is_q0_fail))
      {
        ROS_ERROR_STREAM("Traj collides with " << traj.id);
        result = false;  // will have to redo the optimization
        break;
      }
    }
  }

  return result;
}

// Checks that I have not received new trajectories that affect me while doing the optimization
bool Rmader::safetyCheckAfterOpt(mt::PieceWisePol pwp_optimized)
{
  started_check_ = true;

  bool result = true;
  for (auto traj : trajs_)
  {
    if (traj.time_received > time_init_opt_ && (traj.is_agent == true || traj.is_observed))
    {
      if (trajsAndPwpAreInCollision(traj, pwp_optimized, pwp_optimized.times.front(), pwp_optimized.times.back()))
      {
        ROS_ERROR_STREAM("Traj collides with " << traj.id);
        result = false;  // will have to redo the optimization
        break;
      }
    }
  }

  // and now do another check in case I've received anything while I was checking. Note that mtx_trajs_ is locked!
  if (have_received_trajectories_while_checking_ == true)
  {
    ROS_ERROR_STREAM("Recvd traj while checking ");
    result = false;
  }

  started_check_ = false;

  return result;
}

/// Delay check
bool Rmader::delayCheck(mt::PieceWisePol pwp_now, const double& headsup_time)
{
  bool is_q0_fail = false;
  mtx_trajs_.lock();  // this function is called in mader_ros.cpp so need to lock in the function
  bool result = true;
  for (auto& traj_compiled : trajs_)
  {
    if (traj_compiled.is_agent == true)
    {
      if (par_.is_optimistic_dc)
      {
        if (!traj_compiled.is_committed)
        {
          // std::cout << "headsup_time " << headsup_time << std::endl;
          // std::cout << "time created " << traj_compiled.time_created << std::endl;
          if (traj_compiled.time_created - headsup_time > 1e-2)
          {
            // Do nothing. They will change their traj.
          }
          else if (headsup_time - traj_compiled.time_created > 1e-2)
          {
            if (trajsAndPwpAreInCollision_with_inflation(traj_compiled, pwp_now, pwp_now.times.front(),
                                                         pwp_now.times.back(), is_q0_fail))
            {
              ROS_ERROR_STREAM("In delay check traj_compiled collides with " << traj_compiled.id);
              result = false;  // will have to redo the optimization
            }
          }
          else if (abs(traj_compiled.time_created - headsup_time) < 1e-2)
          {
            if (trajsAndPwpAreInCollision_with_inflation(traj_compiled, pwp_now, pwp_now.times.front(),
                                                         pwp_now.times.back(), is_q0_fail))
            {
              // tie breaking: compare x, y, z and bigger one wins
              Eigen::Vector3d center_obs;
              center_obs << traj_compiled.function[0].value(), traj_compiled.function[1].value(),
                  traj_compiled.function[2].value();
              if (center_obs[0] > state_.pos[0])
              {
                result = false;
              }
              else if (center_obs[1] > state_.pos[1])
              {
                result = false;
              }
              else if (center_obs[2] > state_.pos[2])
              {
                result = false;
              }
              // center_obs[0] == state_.pos[0] &&  center_obs[1] == state_.pos[1] &&  center_obs[2] == state_.pos[2]
              // won't happen bc it's the same position and collision
            }
          }
        }
        else
        {
          if (trajsAndPwpAreInCollision_with_inflation(traj_compiled, pwp_now, pwp_now.times.front(),
                                                       pwp_now.times.back(), is_q0_fail))
          {
            ROS_ERROR_STREAM("In delay check traj_compiled collides with " << traj_compiled.id);
            result = false;
          }
        }
      }
      else
      {  // if traj_compiled.is_agent == false
        if (trajsAndPwpAreInCollision_with_inflation(traj_compiled, pwp_now, pwp_now.times.front(),
                                                     pwp_now.times.back(), is_q0_fail))
        {
          ROS_ERROR_STREAM("In delay check traj_compiled collides with " << traj_compiled.id);
          result = false;
        }
      }
    }
    else
    {
      if (trajsAndPwpAreInCollision_with_inflation(traj_compiled, pwp_now, pwp_now.times.front(), pwp_now.times.back(),
                                                   is_q0_fail))
      {
        ROS_ERROR_STREAM("In delay check traj_compiled collides with " << traj_compiled.id);
        result = false;
      }
    }
  }
  // std::cout << "bef mtx_trajs_.unlock() in delayCheck" << std::endl;
  mtx_trajs_.unlock();
  // std::cout << "aft mtx_trajs_.unlock() in delayCheck" << std::endl;

  return result;
}

// this is just Check in case A* failed
bool Rmader::safetyCheck_for_A_star_failure(mt::PieceWisePol pwp_prev)
{
  bool result = true;
  for (auto traj : trajs_)
  {
    if (traj.time_received > time_init_opt_ && traj.is_agent == true)
    {
      if (trajsAndPwpAreInCollision(traj, pwp_prev, pwp_prev.times.front(), pwp_prev.times.back()))
      {
        if (id_ < traj.id)
        {  // tie breaking mechanism
          ROS_ERROR_STREAM("my previous traj collides with "
                           << traj.id << " and they have higher number, so I need to change my traj.");
          result = false;  // will have to redo the optimization
        }
        break;
      }
    }
  }
  return result;
}

// this is just Check in case A* failed
bool Rmader::safetyCheck_for_A_star_failure_pwp_now(mt::PieceWisePol pwp_now)
{
  bool result = true;
  for (auto traj : trajs_)
  {
    if (traj.time_received > time_init_opt_ && traj.is_agent == true)
    {
      if (trajsAndPwpAreInCollision(traj, pwp_now, pwp_now.times.front(), pwp_now.times.back()))
      {
        if (id_ < traj.id)
        {  // tie breaking mechanism
          ROS_ERROR_STREAM("my pwp_now collides with " << traj.id
                                                       << " and they have higher number, so I need to change my traj.");
          result = false;  // will have to redo the optimization
        }
        break;
      }
    }
  }
  return result;
}

bool Rmader::isReplanningNeeded()
{
  if (initializedStateAndTermGoal() == false)
  {
    // std::cout << "here" << std::endl;
    return false;
  }

  //////////////////////////////////////////////////////////////////////////
  mtx_G_term.lock();

  mt::state G_term = G_term_;  // Local copy of the terminal terminal goal

  mtx_G_term.unlock();

  // Check if we have reached the goal
  mtx_plan_.lock();

  double dist_to_goal = (G_term.pos - plan_.front().pos).norm();
  // std::cout << "dist_to_goal= " << dist_to_goal << std::endl;
  if (dist_to_goal < par_.goal_radius)
  {
    changeDroneStatus(DroneStatus::GOAL_REACHED);
    exists_previous_pwp_ = false;
  }

  // Check if we have seen the goal in the last replan
  double dist_last_plan_to_goal = (G_term.pos - plan_.back().pos).norm();
  // std::cout << "dist_last_plan_to_goal= " << dist_last_plan_to_goal << std::endl;
  mtx_plan_.unlock();

  if (dist_last_plan_to_goal < par_.goal_radius && drone_status_ == DroneStatus::TRAVELING)
  {
    changeDroneStatus(DroneStatus::GOAL_SEEN);
    // std::cout << "Status changed to GOAL_SEEN!" << std::endl;
    exists_previous_pwp_ = false;
  }

  // Don't plan if drone is not traveling
  // A ground rover can see its terminal goal in a long initial spline before a peer's
  // trajectory arrives. Keep replanning until it actually reaches the goal so a later
  // peer trajectory can change the committed path at a crossing.
  // Likewise, observed obstacles keep moving in ways the committed trajectory did not account for, so
  // keep replanning (at the same throttled rate) while any of them is being tracked.
  const bool replan_at_goal_seen = par_.ground_robot || (par_.dyn.enabled && hasObservedObstacles());

  if (drone_status_ == DroneStatus::GOAL_REACHED || (drone_status_ == DroneStatus::YAWING) ||
      (drone_status_ == DroneStatus::GOAL_SEEN && !replan_at_goal_seen))
  {
    // std::cout << "No replanning needed because" << std::endl;
    // printDroneStatus();
    return false;
  }
  if (replan_at_goal_seen && drone_status_ == DroneStatus::GOAL_SEEN)
  {
    // Four checks per second are enough to incorporate peer plans without flooding the
    // ROS bridge and the localization stack with candidate trajectories.
    double now = ros::Time::now().toSec();
    if (now - last_ground_goal_seen_replan_time_ < 0.25)
    {
      return false;
    }
    last_ground_goal_seen_replan_time_ = now;
  }
  return true;
}

bool Rmader::isGoalSeen()
{
  if (drone_status_ == DroneStatus::GOAL_SEEN)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool Rmader::isGoalReached()
{
  if (drone_status_ == DroneStatus::GOAL_REACHED)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool Rmader::replan_with_delaycheck(mt::Edges& edges_obstacles_out, std::vector<mt::state>& headsup_plan,
                                    std::vector<Hyperplane3D>& planes, int& num_of_LPs_run, int& num_of_QCQPs_run,
                                    mt::PieceWisePol& pwp_now, double& headsup_time)
{
  updateHoverAvoidance();

  if (isReplanningNeeded() == false)
  {
    // std::cout << "replan is not needed" << std::endl;
    return false;
  }

  if (holdForQuietPeers())
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "A teammate has gone quiet: holding the committed plan");
    return false;
  }

  // Pop-up for demos

  // if (is_z_max_increased_ && !is_going_back_to_normal_z_max_){
  //   // following popped up trajectory and haven't yet reached to pop_up_last_state_in_plan_
  //   Eigen::Vector3d diff = state_.pos - pop_up_last_state_in_plan_.pos;
  //   pwp_out = mu::constPosition2pwp(pop_up_last_state_in_plan_.pos);
  //   if (diff.norm() > 0.1){
  //     return true;
  //   }
  //   std::cout << "reached pop_up_last_state_in_plan_" << "\n";
  //   is_going_back_to_normal_z_max_ = true;
  // }

  // if (is_z_max_increased_ && is_going_back_to_normal_z_max_){
  //   // once you reached pop_up_last_state_in_plan_, then you start planning new traj in extended z_max space
  //   // std::cout << "state_.pos[2] " << state_.pos[2] << "\n";
  //   // std::cout << "par_.z_max_ " << par_.z_max << "\n";
  //   // std::cout << "par_for_solver.z_max " << solver_->printZmax() << "\n";
  //   std::cout << "going back to the nominal space" << "\n";
  //   if (state_.pos[2] < par_.z_max - 0.3){ // if you go back to the nominal z_max, then put z_max back. 0.3 is a
  //   buffer
  //     std::cout << "got back to the nomial space!!" << "\n";
  //     solver_->changeZmax(par_.z_max); // put the z_max back to the nominal value
  //     is_z_max_increased_ = false;
  //     is_going_back_to_normal_z_max_ = false;
  //   }
  // }
  // std::cout << "par_for_solver.z_max " << solver_->printZmax() << "\n";

  MyTimer replanCB_t(true);

  std::cout << bold << on_white << "**********************IN REPLAN CB*******************" << reset << std::endl;

  //////////////////////////////////////////////////////////////////////////
  ///////////////////////// Select mt::state A /////////////////////////////////
  //////////////////////////////////////////////////////////////////////////

  mtx_G_term.lock();
  mt::state G_term = G_term_;  // Local copy of the terminal terminal goal
  mtx_G_term.unlock();

  mt::state A;
  int k_index;

  // If k_index_end_=0, then A = plan_.back() = plan_[plan_.size() - 1]

  mtx_plan_.lock();

  mu::saturate(deltaT_, par_.lower_bound_runtime_snlopt / par_.dc, par_.upper_bound_runtime_snlopt / par_.dc);

  k_index_end_ = std::max((int)(plan_.size() - deltaT_), 0);

  if (plan_.size() < 5)
  {
    k_index_end_ = 0;
  }

  k_index = plan_.size() - 1 - k_index_end_;
  A = plan_.get(k_index);

  // std::cout << "in replan_with_delaycheck before opt" << std::endl;
  // plan_.print();

  mtx_plan_.unlock();

  // std::cout << blue << "k_index:" << k_index << reset << std::endl;
  // std::cout << blue << "k_index_end_:" << k_index_end_ << reset << std::endl;
  // std::cout << blue << "plan_.size():" << plan_.size() << reset << std::endl;

  double runtime_snlopt;

  if (k_index_end_ != 0)
  {
    runtime_snlopt = k_index * par_.dc;  // std::min(, par_.upper_bound_runtime_snlopt);
  }
  else
  {
    runtime_snlopt = par_.upper_bound_runtime_snlopt;  // I'm stopped at the end of the trajectory --> take my
                                                       // time to replan
  }
  mu::saturate(runtime_snlopt, par_.lower_bound_runtime_snlopt, par_.upper_bound_runtime_snlopt);

  // std::cout << green << "Runtime snlopt= " << runtime_snlopt << reset << std::endl;

  //////////////////////////////////////////////////////////////////////////
  ///////////////////////// Get point G ////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////

  double distA2TermGoal = (G_term.pos - A.pos).norm();
  double ra = std::min((distA2TermGoal - 0.001), par_.Ra);  // radius of the sphere S
  ra = std::min(ra, observedObstacleHorizon());
  mt::state G;
  G.pos = A.pos + ra * (G_term.pos - A.pos).normalized();
  const Subgoal subgoal =
      subgoalAroundObservedObstacles(A, G_term, ra, k_index * par_.dc + ros::Time::now().toSec(), G.pos);
  const bool stepping_aside = evasionSubgoal(A, ra, G.pos);
  const bool escaping = escapeSubgoal(A, G.pos, k_index * par_.dc + ros::Time::now().toSec(), G.pos);
  if (subgoal == Subgoal::NONE_SAFE && !stepping_aside && !escaping)
  {
    ROS_INFO_STREAM_THROTTLE(1.0, "No point ahead is safe to stop at yet: keeping the committed plan");
    return false;
  }

  // detoured G

  // Eigen::Vector3d dist_prog(1e5, 1e5, 1e5);
  // Eigen::Vector3d dist_to_goal(1e5, 1e5, 1e5);
  // Eigen::Vector3d dist_from_state__to_goal(1e5, 1e5, 1e5);
  // double unstuck_dist = 2.0; //meters
  // double reached_goal_dist = par_.drone_bbox[0]+0.1; // avoid the detoured goal is within others bbox and cannot
  // move at all sitution. double how_much_to_detoured_G = 0.7; //0.1 means 10% double vel_stuck_detect = 0.1; // m/s

  // double detour_max_time = 5; //seconds, how long want to detour

  // if (par_.is_stuck || if_detour_ || is_A_star_failed_30_){

  //   if (par_.is_stuck && !if_detour_){
  //     // timer_detour_.Reset();
  //     stuck_state_ = state_;
  //     A_when_stuck_ = A;
  //     G_when_stuck_ = G;
  //   }

  //   dist_prog = state_.pos - stuck_state_.pos;
  //   dist_to_goal = detoured_G_.pos - stuck_state_.pos;
  //   dist_from_state__to_goal = detoured_G_.pos - state_.pos;

  //   // std::cout << "dist_prog is " << dist_prog << std::endl;
  //   // std::cout << "state_.pos is " << state_.pos << std::endl;
  //   // std::cout << "stuck_state_.pos is " << stuck_state_.pos << std::endl;

  //   //if (timer_detour_.ElapsedMs() < detour_max_time * 1000){
  //   if (state_.vel.norm() < vel_stuck_detect &&
  //    dist_from_state__to_goal.norm() > reached_goal_dist &&
  //    dist_prog.norm() < unstuck_dist &&
  //    dist_prog.norm() < how_much_to_detoured_G * dist_to_goal.norm()){

  //     getDetourG(G); // if stuck, make a new G for detour
  //     // if (!if_A_moveback_){
  //     //   moveAtowardG(A, G);
  //     //   if_A_moveback_ = true;
  //     // }
  //     std::cout << "using detoured G" << std::endl;
  //     stuck_count_for_detour_ = stuck_count_for_detour_ + 1;
  //     if_detour_ = true;
  //     if (stuck_count_for_detour_ > 10){
  //       if_detour_ = false; // maybe the detour G is also causing stuck, so go back to the original one
  //       if_A_moveback_ = false;
  //     }
  //   } else {
  //     stuck_count_for_detour_ = 0;
  //     if_detour_ = false;
  //     if_A_moveback_ = false;
  //   //   std::cout << "stop using detoured G" << std::endl;
  //   //   std::cout << "dist_prog.norm() is " << dist_prog.norm() << std::endl;
  //   }
  // }

  mt::state initial = A;
  if (par_.ground_robot)
  {
    // A can be a future sample of the previously committed path. Its tangent
    // is the heading that the new path must continue; at a stop use the rover.
    initial.yaw = initial.vel.head<2>().norm() > 0.05
                      ? std::atan2(initial.vel.y(), initial.vel.x()) : state_.yaw;
    if (groundTurnNeeded(A, G))
    {
      return false;  // turning on the spot first, see getDesiredYaw
    }
  }
  if (is_movingAoutOfBbox_)
  {
    initial = movedA_;
    A = movedA_;
    is_movingAoutOfBbox_ = false;
  }
  mt::state final = G;

  mtx_G.lock();
  G_ = G;
  mtx_G.unlock();

  //////////////////////////////////////////////////////////////////////////
  ///////////////////////// Solve optimization! ////////////////////////////
  //////////////////////////////////////////////////////////////////////////

  solver_->setMaxRuntimeKappaAndMu(runtime_snlopt, par_.kappa, par_.mu);

  //////////////////////
  time_now_ = ros::Time::now().toSec();  // TODO this ros dependency shouldn't be here

  double t_start = k_index * par_.dc + time_now_;

  double factor_alloc = timeAllocationFactor(distA2TermGoal);

  double time_allocated = factor_alloc * mu::getMinTimeDoubleIntegrator3D(initial.pos, initial.vel, final.pos,
                                                                          final.vel, par_.v_max, par_.a_max);

  std::cout << "time_allocated= " << time_allocated << std::endl;

  // std::cout << "initial is " << initial.pos.transpose() << std::endl;
  // std::cout << "Pos of A is" << A.pos.transpose() << std::endl;

  double t_final = t_start + time_allocated;

  bool correctInitialCond =
      solver_->setInitStateFinalStateInitTFinalT(initial, final, t_start,
                                                 t_final);  // note that here t_final may have been updated

  if (correctInitialCond == false)
  {
    std::cout << bold << red << "The solver cannot guarantee feasibility for v1" << reset << std::endl;
    return false;
  }

  ////////////////
  // std::cout << "bef mtx_trajs_.lock() in replan" << std::endl;
  mtx_trajs_.lock();
  // std::cout << "aft mtx_trajs_.lock() in replan" << std::endl;

  time_init_opt_ = ros::Time::now().toSec();
  pruneObservedObstacles(time_init_opt_);
  setHeatCostForSolver();
  removeTrajsThatWillNotAffectMe(A, t_start, t_final);

  // std::cout << "1.1" << std::endl;

  observed_extra_margin_ = par_.dyn.planning_margin;
  ConvexHullsOfCurves hulls = convexHullsOfCurves(t_start, t_final);
  observed_extra_margin_ = 0.0;

  // std::cout << "bef mtx_trajs_.unlock() in replan" << std::endl;
  mtx_trajs_.unlock();
  // std::cout << "aft mtx_trajs_.unlock() in replan" << std::endl;

  mt::ConvexHullsOfCurves_Std hulls_std = cu::vectorGCALPol2vectorStdEigen(hulls);
  // poly_safe_out = cu::vectorGCALPol2vectorJPSPol(hulls);
  // std::cout << "1.2" << std::endl;
  edges_obstacles_out = cu::vectorGCALPol2edges(hulls);

  // std::cout << "2" << std::endl;

  solver_->setHulls(hulls_std);

  //////////////////////
  // std::cout << on_cyan << bold << "Solved so far" << solutions_found_ << "/" << total_replannings_ << reset
  //           << std::endl;

  // std::cout << "[FA] Calling NL" << std::endl;

  bool is_stuck;
  bool is_A_star_failed;
  bool is_q0_fail = false;
  bool result = solver_->optimize(is_stuck, is_A_star_failed, is_q0_fail);  // calling the solver

  // if (!result){
  //   // std::cout << "q0_fail_count_ is " << q0_fail_count_ << std::endl;
  //   opt_fail_count_ += 1;
  //   if (opt_fail_count_ > 3 && state_.vel.norm() < 0.001){
  //     // in this case one agent is on the bbox constraint so need to move A away
  //     std::cout << "[opt fail] move A out of the bbox" << std::endl;
  //     movedA_ = moveAoutOfBbox(A);
  //     is_movingAoutOfBbox_ = true;
  //   }
  // } else {
  //   q0_fail_count_ = 0;
  // }

  // right after taking off, sometims drones cannot find a path
  // sometimes the very initial path search takes more than how_many_A_star_failure counts and fails
  // if (planner_initialized_){

  //   // check if A_star is failed and see if the previous plan is feasible
  //   if (is_A_star_failed && !is_pwp_prev_feasible_){
  //     A_star_fail_count_ += 1;
  //     is_A_star_failed_30_ = (A_star_fail_count_ > 30);
  //     // need to check if my previous traj collides with others. and if that's the case pop it up
  //     std::cout << "A_star is failing\n";
  //     std::cout << "A_star_fail_count_ is " << A_star_fail_count_ << "\n";
  //     if (is_A_star_failed_30_){
  //       if(!safetyCheck_for_A_star_failure(pwp_prev_)){
  //         std::cout << "previous pwp collide!" << "\n";
  //         // if previous pwp is not feasible pop up the drone
  //         // this only happens when two agents commit traj at the very same time (or in Recheck period)
  //         is_pop_up_ = true;
  //         A_star_fail_count_ = 0;
  //         return false; //abort mader
  //       } else {
  //         // std::cout << "previous pwp doesn't collide!\n";
  //         is_pwp_prev_feasible_ = true;
  //         is_pop_up_ = false;
  //         A_star_fail_count_ = 0;
  //       }
  //     }
  //   } else {
  //     is_pop_up_ = false;
  //     A_star_fail_count_ = 0;
  //   }

  // }

  // // check if drones are stuck or not
  // if (is_stuck){
  //   par_.is_stuck = true;
  // } else {
  //   par_.is_stuck = false;
  // }

  num_of_LPs_run = solver_->getNumOfLPsRun();
  num_of_QCQPs_run = solver_->getNumOfQCQPsRun();

  total_replannings_++;
  if (result == false)
  {
    int states_last_replan = ceil(replanCB_t.ElapsedMs() / (par_.dc * 1000) +
                                  par_.delay_check * par_.comm_delay_param / par_.dc);  // Number of states that
                                                                                        // would have been needed for
                                                                                        // the last replan
    deltaT_ = std::max(par_.factor_alpha * states_last_replan, 1.0);
    deltaT_ = std::min(1.0 * deltaT_, 2.0 / par_.dc);
    std::cout << "solver couldn't find optimal path" << std::endl;
    return false;
  }

  solver_->getPlanes(planes);

  solutions_found_++;

  // av_improvement_nlopt_ = ((solutions_found_ - 1) * av_improvement_nlopt_ + solver_->improvement_) /
  // solutions_found_;

  // std::cout << blue << "Average improvement so far" << std::setprecision(5) << av_improvement_nlopt_ << reset
  //          << std::endl;

  solver_->getSolution(pwp_now);

  // // check if the current position/plan is colliding
  // if (is_pop_up_initialized_){

  //   // check if A_star is failed and see if the previous plan is feasible
  //   if (is_A_star_failed_){
  //     // need to check if my previous traj collides with others. and if that's the case pop it up
  //     A_star_fail_count_pwp_now_ = A_star_fail_count_pwp_now_ + 1;
  //     std::cout << "A_star is failing\n";
  //     std::cout << "A_star_fail_count_pwp_now_ is " << A_star_fail_count_pwp_now_ << "\n";
  //     double how_many_A_star_failure_now = 30;
  //     if (A_star_fail_count_pwp_now_ > how_many_A_star_failure_now){
  //       if(!safetyCheck_for_A_star_failure_pwp_now(pwp_now)){
  //         std::cout << "pwp now collide!" << "\n";
  //         // if previous pwp is not feasible pop up the drone
  //         // this only happens when two agents commit traj at the very same time (or in Recheck period)
  //         is_pop_up_ = true;
  //         return false; //abort mader
  //       } else {
  //         is_pop_up_ = false;
  //         A_star_fail_count_pwp_now_ = 0;
  //       }
  //     }
  //   } else {
  //     is_pop_up_ = false;
  //     A_star_fail_count_pwp_now_ = 0;
  //   }

  // }

  MyTimer check_t(true);

  if (par_.is_check)
  {
    // std::cout << "bef mtx_trajs_.lock() in replan (safetyCheckAfterOpt)" << std::endl;
    mtx_trajs_.lock();
    // std::cout << "aft mtx_trajs_.lock() in replan (safetyCheckAfterOpt)" << std::endl;

    // first make sure none of the trajectory is checked in this optimization
    // for (auto &traj : trajs_){
    //   traj.is_checked = false;
    // }

    // check and recheck are done in safetyChechAfterOpt()

    bool is_safe_after_opt = safetyCheckAfterOptForRmader(pwp_now, is_q0_fail);

    // std::cout << "bef mtx_trajs_.unlock() in replan (safetyCheckAfterOpt)" << std::endl;
    mtx_trajs_.unlock();
    headsup_time = ros::Time::now().toSec();
    // std::cout << "aft mtx_trajs_.unlock() in replan (safetyCheckAfterOpt)" << std::endl;

    // if (!is_safe_after_opt){
    //   // std::cout << "q0_fail_count_ is " << q0_fail_count_ << std::endl;
    //   safetycheck_fail_count_ += 1;
    //   if (safetycheck_fail_count_ > 3 && state_.vel.norm() < 0.001){
    //     // in this case one agent is on the bbox constraint so need to move A away
    //     std::cout << "[safety check fail] move A out of the bbox" << std::endl;
    //     movedA_ = moveAoutOfBbox(A);
    //     is_movingAoutOfBbox_ = true;
    //   }
    // } else {
    //   q0_fail_count_ = 0;
    // }

    if (is_safe_after_opt == false)
    {
      // int states_last_replan = ceil(replanCB_t.ElapsedMs() / (par_.dc * 1000) +
      //                               par_.delay_check * par_.comm_delay_param / par_.dc);  // Number of states that
      //                                                                                     // would have been needed
      //                                                                                     for
      //                                                                                     // the last replan
      // deltaT_ = std::max(par_.factor_alpha * states_last_replan, 1.0);
      // deltaT_ = std::min(1.0 * deltaT_, 2.0 / par_.dc);
      ROS_ERROR_STREAM("safetyCheckAfterOpt is not satisfied, returning");
      return false;
    }
  }
  headsup_time = ros::Time::now().toSec();

  ///////////////////////////////////////////////////////////
  ///////////////       OTHER STUFF    //////////////////////
  //////////////////////////////////////////////////////////

  mtx_offsets.lock();

  int states_last_replan = ceil(replanCB_t.ElapsedMs() / (par_.dc * 1000) +
                                par_.delay_check * par_.comm_delay_param / par_.dc);  // Number of states that
                                                                                      // would have been needed
                                                                                      // for the last replan
  deltaT_ = std::max(par_.factor_alpha * states_last_replan, 1.0);
  deltaT_ = std::min(1.0 * deltaT_, 2.0 / par_.dc);
  mtx_offsets.unlock();

  // std::cout << "end of replan" << "\n";
  // std::cout << "k_index_end_ is " << k_index_end_ << "\n";
  // std::cout << "deltaT_ is " << deltaT_ << "\n";
  // std::cout << "plan_.size() " << plan_.size() << "\n";

  planner_initialized_ = true;

  mtx_plan_.lock();
  // headsup trajectory
  headsup_plan = plan_.toStdVector();
  mtx_plan_.unlock();

  int headsup_plan_size = headsup_plan.size();

  if ((headsup_plan_size - 1 - k_index_end_) < 0)
  {
    std::cout << bold << red << "Already published the point A" << reset << std::endl;
    std::cout << "headsup_plan_size= " << headsup_plan_size << std::endl;
    std::cout << "k_index_end_= " << k_index_end_ << std::endl;
    return false;
  }
  else
  {
    headsup_plan.erase(headsup_plan.end() - k_index_end_ - 1,
                       headsup_plan.end());                     // this deletes also the initial condition...
    for (int i = 0; i < (solver_->traj_solution_).size(); i++)  //... which is included in traj_solution_[0]
    {
      headsup_plan.push_back(solver_->traj_solution_[i]);
    }
  }

  return true;
}

bool Rmader::addTrajToPlan_with_delaycheck(mt::PieceWisePol& pwp)
{
  // std::cout << bold << "Check Timer=" << check_t << std::endl;

  mtx_G_term.lock();
  mt::state G_term = G_term_;  // Local copy of the terminal terminal goal
  mtx_G_term.unlock();

  M_ = G_term;

  //////////////////////////////////////////////////////////////////////////
  ///////////////////////// Append to plan /////////////////////////////////
  //////////////////////////////////////////////////////////////////////////

  mtx_plan_.lock();

  int plan_size = plan_.size();

  if ((plan_size - 1 - k_index_end_) < 0)
  {
    std::cout << bold << red << "Already published the point A" << reset << std::endl;
    std::cout << "plan_size= " << plan_size << std::endl;
    std::cout << "k_index_end_= " << k_index_end_ << std::endl;
    mtx_plan_.unlock();
    return false;
  }
  else
  {
    // std::cout << "Appending" << std::endl;
    // std::cout << "before, plan_size=" << plan_.size() << std::endl;
    plan_.erase(plan_.end() - k_index_end_ - 1, plan_.end());  // this deletes also the initial condition...
    // std::cout << "middle, plan_size=" << plan_.size() << " sol.size()=" << (solver_->traj_solution_).size()
    // << std::endl;
    for (int i = 0; i < (solver_->traj_solution_).size(); i++)  //... which is included in traj_solution_[0]
    {
      plan_.push_back(solver_->traj_solution_[i]);
    }
    // std::cout << "after, plan_size=" << plan_.size() << std::endl;
  }

  // std::cout << "in addTrajToPlan_with_delaycheck" << std::endl;
  // plan_.print();
  mtx_plan_.unlock();

  ////////////////////
  ////////////////////

  if (exists_previous_pwp_ == true)
  {
    pwp_prev_ = mu::composePieceWisePol(time_now_, par_.dc, pwp_prev_, pwp);
  }
  else
  {
    pwp_prev_ = pwp;
    exists_previous_pwp_ = true;
  }

  is_pwp_prev_feasible_ =
      false;  // we don't know for sure if this traj is feasible until you run opt in the next step and see A* fails

  return true;
}

mt::state Rmader::moveAoutOfBbox(const mt::state& A)
{
  mt::state newA = A;
  double tol = 0.001;
  double move_how_much = 0.002;

  mtx_trajs_.lock();

  // identify which agent we are stuck with
  for (auto traj : trajs_)
  {
    Eigen::Vector3d dist = state_.pos - traj.pwp.eval(ros::Time::now().toSec());

    if (dist[0] <= par_.drone_bbox[0] + tol)
    {
      std::cout << "stuck with agent " << traj.id << std::endl;
      std::cout << "plan_.size() is " << plan_.size() << std::endl;
      std::cout << "k_index_end_ is" << k_index_end_ << std::endl;
      newA.pos[0] = state_.pos[0] + move_how_much;
      break;
    }
    else if (-par_.drone_bbox[0] - tol <= dist[0])
    {
      std::cout << "stuck with agent " << traj.id << std::endl;
      std::cout << "plan_.size() is " << plan_.size() << std::endl;
      std::cout << "k_index_end_ is" << k_index_end_ << std::endl;
      newA.pos[0] = state_.pos[0] - move_how_much;
      break;
    }
    else if (dist[1] <= par_.drone_bbox[1] + tol)
    {
      std::cout << "stuck with agent " << traj.id << std::endl;
      std::cout << "plan_.size() is " << plan_.size() << std::endl;
      std::cout << "k_index_end_ is" << k_index_end_ << std::endl;
      newA.pos[1] = state_.pos[1] + move_how_much;
      break;
    }
    else if (-par_.drone_bbox[1] - tol <= dist[1])
    {
      std::cout << "stuck with agent " << traj.id << std::endl;
      std::cout << "plan_.size() is " << plan_.size() << std::endl;
      std::cout << "k_index_end_ is" << k_index_end_ << std::endl;
      newA.pos[1] = state_.pos[1] - move_how_much;
      break;
    }
    else if (dist[2] <= par_.drone_bbox[2] + tol)
    {
      std::cout << "stuck with agent " << traj.id << std::endl;
      std::cout << "plan_.size() is " << plan_.size() << std::endl;
      std::cout << "k_index_end_ is" << k_index_end_ << std::endl;
      newA.pos[2] = state_.pos[2] + move_how_much;
      break;
    }
    else if (-par_.drone_bbox[2] - tol <= dist[2])
    {
      std::cout << "stuck with agent " << traj.id << std::endl;
      std::cout << "plan_.size() is " << plan_.size() << std::endl;
      std::cout << "k_index_end_ is" << k_index_end_ << std::endl;
      newA.pos[2] = state_.pos[2] - move_how_much;
      break;
    }
  }

  mtx_trajs_.unlock();

  return newA;
}

bool Rmader::replan(mt::Edges& edges_obstacles_out, std::vector<mt::state>& X_safe_out,
                    std::vector<Hyperplane3D>& planes, int& num_of_LPs_run, int& num_of_QCQPs_run,
                    mt::PieceWisePol& pwp_out)
{
  updateHoverAvoidance();

  if (isReplanningNeeded() == false)
  {
    return false;
  }

  if (holdForQuietPeers())
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "A teammate has gone quiet: holding the committed plan");
    return false;
  }

  // if (is_z_max_increased_ && !is_going_back_to_normal_z_max_){
  //   // following popped up trajectory and haven't yet reached to pop_up_last_state_in_plan_
  //   Eigen::Vector3d diff = state_.pos - pop_up_last_state_in_plan_.pos;
  //   pwp_out = mu::constPosition2pwp(pop_up_last_state_in_plan_.pos);
  //   if (diff.norm() > 0.1){
  //     return true;
  //   }
  //   std::cout << "reached pop_up_last_state_in_plan_" << "\n";
  //   is_going_back_to_normal_z_max_ = true;
  // }

  // if (is_z_max_increased_ && is_going_back_to_normal_z_max_){
  //   // once you reached pop_up_last_state_in_plan_, then you start planning new traj in extended z_max space
  //   // std::cout << "state_.pos[2] " << state_.pos[2] << "\n";
  //   // std::cout << "par_.z_max_ " << par_.z_max << "\n";
  //   // std::cout << "par_for_solver.z_max " << solver_->printZmax() << "\n";
  //   std::cout << "going back to the nominal space" << "\n";
  //   if (state_.pos[2] < par_.z_max - 0.3){ // if you go back to the nominal z_max, then put z_max back. 0.3 is a
  //   buffer
  //     std::cout << "got back to the nomial space!!" << "\n";
  //     solver_->changeZmax(par_.z_max); // put the z_max back to the nominal value
  //     is_z_max_increased_ = false;
  //     is_going_back_to_normal_z_max_ = false;
  //   }
  // }
  // std::cout << "par_for_solver.z_max " << solver_->printZmax() << "\n";

  MyTimer replanCB_t(true);

  std::cout << bold << on_white << "**********************IN REPLAN CB*******************" << reset << std::endl;

  //////////////////////////////////////////////////////////////////////////
  ///////////////////////// Select mt::state A /////////////////////////////////
  //////////////////////////////////////////////////////////////////////////

  mtx_G_term.lock();
  mt::state G_term = G_term_;  // Local copy of the terminal terminal goal
  mtx_G_term.unlock();

  mt::state A;
  int k_index_end, k_index;

  // If k_index_end=0, then A = plan_.back() = plan_[plan_.size() - 1]

  mtx_plan_.lock();

  mu::saturate(deltaT_, par_.lower_bound_runtime_snlopt / par_.dc, par_.upper_bound_runtime_snlopt / par_.dc);

  k_index_end = std::max((int)(plan_.size() - deltaT_), 0);

  if (plan_.size() < 5)
  {
    k_index_end = 0;
  }

  k_index = plan_.size() - 1 - k_index_end;
  A = plan_.get(k_index);

  mtx_plan_.unlock();

  // std::cout << blue << "k_index:" << k_index << reset << std::endl;
  // std::cout << blue << "k_index_end:" << k_index_end << reset << std::endl;
  // std::cout << blue << "plan_.size():" << plan_.size() << reset << std::endl;

  double runtime_snlopt;

  if (k_index_end != 0)
  {
    runtime_snlopt = k_index * par_.dc;  // std::min(, par_.upper_bound_runtime_snlopt);
  }
  else
  {
    runtime_snlopt = par_.upper_bound_runtime_snlopt;  // I'm stopped at the end of the trajectory --> take my
                                                       // time to replan
  }
  mu::saturate(runtime_snlopt, par_.lower_bound_runtime_snlopt, par_.upper_bound_runtime_snlopt);

  // std::cout << green << "Runtime snlopt= " << runtime_snlopt << reset << std::endl;

  //////////////////////////////////////////////////////////////////////////
  ///////////////////////// Get point G ////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////

  double distA2TermGoal = (G_term.pos - A.pos).norm();
  double ra = std::min((distA2TermGoal - 0.001), par_.Ra);  // radius of the sphere S
  ra = std::min(ra, observedObstacleHorizon());
  mt::state G;
  G.pos = A.pos + ra * (G_term.pos - A.pos).normalized();
  const Subgoal subgoal =
      subgoalAroundObservedObstacles(A, G_term, ra, k_index * par_.dc + ros::Time::now().toSec(), G.pos);
  const bool stepping_aside = evasionSubgoal(A, ra, G.pos);
  const bool escaping = escapeSubgoal(A, G.pos, k_index * par_.dc + ros::Time::now().toSec(), G.pos);
  if (subgoal == Subgoal::NONE_SAFE && !stepping_aside && !escaping)
  {
    ROS_INFO_STREAM_THROTTLE(1.0, "No point ahead is safe to stop at yet: keeping the committed plan");
    return false;
  }

  // Eigen::Vector3d dist_prog(1e5, 1e5, 1e5);
  // Eigen::Vector3d dist_to_goal(1e5, 1e5, 1e5);
  // Eigen::Vector3d dist_from_state__to_goal(1e5, 1e5, 1e5);
  // double unstuck_dist = 2.0; //meters
  // double reached_goal_dist = par_.drone_bbox[0]+0.1; // avoid the detoured goal is within others bbox and cannot
  // move at all sitution. double how_much_to_detoured_G = 0.7; //0.1 means 10% double vel_stuck_detect = 0.1; // m/s

  // // double detour_max_time = 5; //seconds, how long want to detour

  // if (par_.is_stuck || if_detour_ || is_A_star_failed_30_){

  //   if (par_.is_stuck && !if_detour_){
  //     // timer_detour_.Reset();
  //     stuck_state_ = state_;
  //     A_when_stuck_ = A;
  //     G_when_stuck_ = G;
  //   }

  //   dist_prog = state_.pos - stuck_state_.pos;
  //   dist_to_goal = detoured_G_.pos - stuck_state_.pos;
  //   dist_from_state__to_goal = detoured_G_.pos - state_.pos;

  //   // std::cout << "dist_prog is " << dist_prog << std::endl;
  //   // std::cout << "state_.pos is " << state_.pos << std::endl;
  //   // std::cout << "stuck_state_.pos is " << stuck_state_.pos << std::endl;

  //   //if (timer_detour_.ElapsedMs() < detour_max_time * 1000){
  //   if (state_.vel.norm() < vel_stuck_detect &&
  //    dist_from_state__to_goal.norm() > reached_goal_dist &&
  //    dist_prog.norm() < unstuck_dist &&
  //    dist_prog.norm() < how_much_to_detoured_G * dist_to_goal.norm()){

  //     getDetourG(G); // if stuck, make a new G for detour
  //     // if (!if_A_moveback_){
  //     //   moveAtowardG(A, G);
  //     //   if_A_moveback_ = true;
  //     // }
  //     std::cout << "using detoured G" << std::endl;
  //     stuck_count_for_detour_ = stuck_count_for_detour_ + 1;
  //     if_detour_ = true;
  //     if (stuck_count_for_detour_ > 10){
  //       if_detour_ = false; // maybe the detour G is also causing stuck, so go back to the original one
  //       if_A_moveback_ = false;
  //     }
  //   } else {
  //     stuck_count_for_detour_ = 0;
  //     if_detour_ = false;
  //     if_A_moveback_ = false;
  //   //   std::cout << "stop using detoured G" << std::endl;
  //   //   std::cout << "dist_prog.norm() is " << dist_prog.norm() << std::endl;
  //   }
  // }

  mt::state initial = A;
  if (par_.ground_robot)
  {
    initial.yaw = initial.vel.head<2>().norm() > 0.05
                      ? std::atan2(initial.vel.y(), initial.vel.x()) : state_.yaw;
    if (groundTurnNeeded(A, G))
    {
      return false;  // turning on the spot first, see getDesiredYaw
    }
  }
  mt::state final = G;

  mtx_G.lock();
  G_ = G;
  mtx_G.unlock();

  //////////////////////////////////////////////////////////////////////////
  ///////////////////////// Solve optimization! ////////////////////////////
  //////////////////////////////////////////////////////////////////////////

  solver_->setMaxRuntimeKappaAndMu(runtime_snlopt, par_.kappa, par_.mu);

  //////////////////////
  double time_now = ros::Time::now().toSec();  // TODO this ros dependency shouldn't be here

  double t_start = k_index * par_.dc + time_now;

  double factor_alloc = timeAllocationFactor(distA2TermGoal);

  double time_allocated = factor_alloc * mu::getMinTimeDoubleIntegrator3D(initial.pos, initial.vel, final.pos,
                                                                          final.vel, par_.v_max, par_.a_max);

  std::cout << "time_allocated= " << time_allocated << std::endl;

  // std::cout << "initial is " << initial.pos.transpose() << std::endl;
  // std::cout << "Pos of A is" << A.pos.transpose() << std::endl;

  double t_final = t_start + time_allocated;

  bool correctInitialCond =
      solver_->setInitStateFinalStateInitTFinalT(initial, final, t_start,
                                                 t_final);  // note that here t_final may have been updated

  if (correctInitialCond == false)
  {
    std::cout << bold << red << "The solver cannot guarantee feasibility for v1" << reset << std::endl;
    return false;
  }

  ////////////////

  mtx_trajs_.lock();

  time_init_opt_ = ros::Time::now().toSec();
  pruneObservedObstacles(time_init_opt_);
  setHeatCostForSolver();
  removeTrajsThatWillNotAffectMe(A, t_start, t_final);

  observed_extra_margin_ = par_.dyn.planning_margin;
  ConvexHullsOfCurves hulls = convexHullsOfCurves(t_start, t_final);
  observed_extra_margin_ = 0.0;

  mtx_trajs_.unlock();

  mt::ConvexHullsOfCurves_Std hulls_std = cu::vectorGCALPol2vectorStdEigen(hulls);
  // poly_safe_out = cu::vectorGCALPol2vectorJPSPol(hulls);
  edges_obstacles_out = cu::vectorGCALPol2edges(hulls);

  solver_->setHulls(hulls_std);

  //////////////////////
  // std::cout << on_cyan << bold << "Solved so far" << solutions_found_ << "/" << total_replannings_ << reset
  //           << std::endl;

  // std::cout << "[FA] Calling NL" << std::endl;

  bool is_stuck;
  bool is_A_star_failed;
  bool is_q0_fail;

  bool result = solver_->optimize(is_stuck, is_A_star_failed, is_q0_fail);  // calling the solver

  // right after taking off, sometims drones cannot find a path
  // sometimes the very initial path search takes more than how_many_A_star_failure counts and fails
  // if (planner_initialized_){

  //   // check if A_star is failed and see if the previous plan is feasible
  //   if (is_A_star_failed && !is_pwp_prev_feasible_){
  //     A_star_fail_count_ += 1;
  //     is_A_star_failed_30_ = (A_star_fail_count_ > 30);
  //     // need to check if my previous traj collides with others. and if that's the case pop it up
  //     std::cout << "A_star is failing\n";
  //     std::cout << "A_star_fail_count_ is " << A_star_fail_count_ << "\n";
  //     if (is_A_star_failed_30_){
  //       if(!safetyCheck_for_A_star_failure(pwp_prev_)){
  //         std::cout << "previous pwp collide!" << "\n";
  //         // if previous pwp is not feasible pop up the drone
  //         // this only happens when two agents commit traj at the very same time (or in Recheck period)
  //         is_pop_up_ = true;
  //         A_star_fail_count_ = 0;
  //         return false; //abort mader
  //       } else {
  //         // std::cout << "previous pwp doesn't collide!\n";
  //         is_pwp_prev_feasible_ = true;
  //         is_pop_up_ = false;
  //         A_star_fail_count_ = 0;
  //       }
  //     }
  //   } else {
  //     is_pop_up_ = false;
  //     A_star_fail_count_ = 0;
  //   }

  // }

  // // check if drones are stuck or not
  // if (is_stuck){
  //   par_.is_stuck = true;
  // } else {
  //   par_.is_stuck = false;
  // }

  num_of_LPs_run = solver_->getNumOfLPsRun();
  num_of_QCQPs_run = solver_->getNumOfQCQPsRun();

  total_replannings_++;
  if (result == false)
  {
    int states_last_replan = ceil(replanCB_t.ElapsedMs() / (par_.dc * 1000));  // Number of states that
                                                                               // would have been needed for
                                                                               // the last replan
    deltaT_ = std::max(par_.factor_alpha * states_last_replan, 1.0);
    deltaT_ = std::min(1.0 * deltaT_, 2.0 / par_.dc);
    return false;
  }

  solver_->getPlanes(planes);

  solutions_found_++;

  // av_improvement_nlopt_ = ((solutions_found_ - 1) * av_improvement_nlopt_ + solver_->improvement_) /
  // solutions_found_;

  // std::cout << blue << "Average improvement so far" << std::setprecision(5) << av_improvement_nlopt_ << reset
  //          << std::endl;

  mt::PieceWisePol pwp_now;
  solver_->getSolution(pwp_now);

  // // check if the current position/plan is colliding
  // if (is_pop_up_initialized_){

  //   // check if A_star is failed and see if the previous plan is feasible
  //   if (is_A_star_failed_){
  //     // need to check if my previous traj collides with others. and if that's the case pop it up
  //     A_star_fail_count_pwp_now_ = A_star_fail_count_pwp_now_ + 1;
  //     std::cout << "A_star is failing\n";
  //     std::cout << "A_star_fail_count_pwp_now_ is " << A_star_fail_count_pwp_now_ << "\n";
  //     double how_many_A_star_failure_now = 30;
  //     if (A_star_fail_count_pwp_now_ > how_many_A_star_failure_now){
  //       if(!safetyCheck_for_A_star_failure_pwp_now(pwp_now)){
  //         std::cout << "pwp now collide!" << "\n";
  //         // if previous pwp is not feasible pop up the drone
  //         // this only happens when two agents commit traj at the very same time (or in Recheck period)
  //         is_pop_up_ = true;
  //         return false; //abort mader
  //       } else {
  //         is_pop_up_ = false;
  //         A_star_fail_count_pwp_now_ = 0;
  //       }
  //     }
  //   } else {
  //     is_pop_up_ = false;
  //     A_star_fail_count_pwp_now_ = 0;
  //   }

  // }

  MyTimer check_t(true);
  mtx_trajs_.lock();
  bool is_safe_after_opt = safetyCheckAfterOpt(pwp_now);
  mtx_trajs_.unlock();
  // std::cout << bold << "Check Timer=" << check_t << std::endl;

  if (is_safe_after_opt == false)
  {
    ROS_ERROR_STREAM("safetyCheckAfterOpt is not satisfied, returning");
    return false;
  }

  M_ = G_term;

  //////////////////////////////////////////////////////////////////////////
  ///////////////////////// Append to plan /////////////////////////////////
  //////////////////////////////////////////////////////////////////////////
  mtx_plan_.lock();

  int plan_size = plan_.size();

  if ((plan_size - 1 - k_index_end) < 0)
  {
    // std::cout << bold << red << "Already published the point A" << reset << std::endl;
    // std::cout << "plan_size= " << plan_size << std::endl;
    // std::cout << "k_index_end= " << k_index_end << std::endl;
    mtx_plan_.unlock();
    return false;
  }
  else
  {
    // std::cout << "Appending" << std::endl;
    // std::cout << "before, plan_size=" << plan_.size() << std::endl;
    plan_.erase(plan_.end() - k_index_end - 1, plan_.end());  // this deletes also the initial condition...
    // std::cout << "middle, plan_size=" << plan_.size() << " sol.size()=" << (solver_->traj_solution_).size()
    // << std::endl;
    for (int i = 0; i < (solver_->traj_solution_).size(); i++)  //... which is included in traj_solution_[0]
    {
      plan_.push_back(solver_->traj_solution_[i]);
    }
    // std::cout << "after, plan_size=" << plan_.size() << std::endl;
  }

  // plan_.print();
  mtx_plan_.unlock();

  ////////////////////
  ////////////////////

  if (exists_previous_pwp_ == true)
  {
    pwp_out = mu::composePieceWisePol(time_now, par_.dc, pwp_prev_, pwp_now);
    pwp_prev_ = pwp_out;
  }
  else
  {  //
    pwp_out = pwp_now;
    pwp_prev_ = pwp_now;
    exists_previous_pwp_ = true;
  }
  is_pwp_prev_feasible_ =
      false;  // we don't know for sure if this traj is feasible until you run opt in the next step and see A* fails

  mtx_plan_.lock();
  X_safe_out = plan_.toStdVector();

  ///////////////////////////////////////////////////////////
  ///////////////       OTHER STUFF    //////////////////////
  //////////////////////////////////////////////////////////

  // Check if we have planned until G_term
  mt::state F = plan_.back();  // Final point of the safe path (\equiv final point of the comitted path)
  mtx_plan_.unlock();
  double dist = (G_term_.pos - F.pos).norm();

  if (dist < par_.goal_radius)
  {
    changeDroneStatus(DroneStatus::GOAL_SEEN);
  }

  mtx_offsets.lock();

  int states_last_replan = ceil(replanCB_t.ElapsedMs() / (par_.dc * 1000));  // Number of states that
                                                                             // would have been needed for
                                                                             // the last replan
  deltaT_ = std::max(par_.factor_alpha * states_last_replan, 1.0);
  mtx_offsets.unlock();

  planner_initialized_ = true;

  return true;
}

void Rmader::resetInitialization()
{
  planner_initialized_ = false;
  state_initialized_ = false;

  terminal_goal_initialized_ = false;
}

void Rmader::yaw(double diff, mt::state& next_goal)
{
  // bang-buffer-bang control
  if (abs(diff) < 3 * M_PI / 180)
  {
    next_goal.dyaw = 0.0;
    next_goal.yaw = previous_yaw_;
  }
  else
  {
    mu::saturate(diff, -par_.dc * par_.w_max, par_.dc * par_.w_max);
    double dyaw_not_filtered;

    dyaw_not_filtered = copysign(1, diff) * par_.w_max;

    dyaw_filtered_ = (1 - par_.alpha_filter_dyaw) * dyaw_not_filtered + par_.alpha_filter_dyaw * dyaw_filtered_;
    next_goal.dyaw = dyaw_filtered_;
    next_goal.yaw = previous_yaw_ + dyaw_filtered_ * par_.dc;
  }
}

void Rmader::getDesiredYaw(mt::state& next_goal)
{
  if (par_.ground_robot)
  {
    next_goal.yaw = next_goal.vel.head<2>().norm() > 0.05 ? std::atan2(next_goal.vel.y(), next_goal.vel.x()) :
                                                             (ground_turn_requested_ ? ground_turn_yaw_ : state_.yaw);
    next_goal.dyaw = 0.0;
    return;
  }
  if (par_.is_camera_yawing)
  {
    // looking at the center of highbay
    double desired_yaw = atan2(state_.pos[1], state_.pos[0] - 5.5) -
                         M_PI / 2;  // - M_PI / 2 is because the camera is mounted pointing at y-axis
    double diff = desired_yaw - state_.yaw;
    mu::angle_wrap(diff);
    yaw(diff, next_goal);
  }
  else
  {
    next_goal.dyaw = 0.0;
    next_goal.yaw = initial_yaw_;
  }
  // next_goal.yaw = state_.yaw;
  /*
  switch (drone_status_)
  {
    case DroneStatus::YAWING:
      desired_yaw = atan2(G_term_.pos[1] - next_goal.pos[1], G_term_.pos[0] - next_goal.pos[0]);
      diff = desired_yaw - state_.yaw;
      break;
    case DroneStatus::TRAVELING:
    case DroneStatus::GOAL_SEEN:
      desired_yaw = atan2(M_.pos[1] - next_goal.pos[1], M_.pos[0] - next_goal.pos[0]);
      diff = desired_yaw - state_.yaw;
      break;
    case DroneStatus::GOAL_REACHED:
      next_goal.dyaw = 0.0;
      next_goal.yaw = previous_yaw_;
      return;
  }

  mu::angle_wrap(diff);
  if (fabs(diff) < 0.04 && drone_status_ == DroneStatus::YAWING)
  {
    changeDroneStatus(DroneStatus::TRAVELING);
  }
  yaw(diff, next_goal);
  */
}

bool Rmader::getNextGoal(mt::state& next_goal)
{
  if (initializedStateAndTermGoal() == false)  // || (drone_status_ == DroneStatus::GOAL_REACHED && plan_.size() ==
                                               // 1))
  {
    // std::cout << "Not publishing new goal!!" << std::endl;
    return false;
  }

  mtx_goals.lock();
  mtx_plan_.lock();

  next_goal.setZero();
  next_goal = plan_.front();

  // if (is_pop_up_ && !is_z_max_increased_){
  //   // we need to pop up the drone
  //   std::cout << "pop up!!!!" << std::endl;
  //   double pop_up_alt = par_.drone_bbox[2] + 0.1;
  //   pop_up_state_ = state_;
  //   std::cout << "plan_.size() is " << plan_.size() << "\n";
  //   if (plan_.size() != 0){
  //     for (int i = 0; i < plan_.size(); i++){
  //     // increment z axis value
  //     double pop_up_increment = pop_up_alt / plan_.size();
  //     plan_.content[i].pos[2] = plan_.content[i].pos[2] + i * pop_up_increment;
  //     }
  //   } else {
  //     double pop_up_increment = pop_up_alt / 400;
  //     for (int i = 0; i < 400; i++){
  //       mt::state temporaty_state = state_;
  //       temporaty_state.pos[2] += i * pop_up_increment;
  //       plan_.push_back(temporaty_state);
  //     }
  //   }

  //   is_pop_up_ = false;
  //   pop_up_last_state_in_plan_ = plan_.back();
  //   double new_zmax = par_.z_max + pop_up_alt;
  //   solver_->changeZmax(new_zmax);
  //   is_z_max_increased_ = true;
  // }

  if (plan_.size() > 1)
  {
    plan_.pop_front();
  }

  getDesiredYaw(next_goal);  // we don't need to control yaw
  previous_yaw_ = next_goal.yaw;

  mtx_goals.unlock();
  mtx_plan_.unlock();
  return true;
}

// Debugging functions
void Rmader::changeDroneStatus(int new_status)
{
  if (new_status == drone_status_)
  {
    return;
  }

  // std::cout << "Changing DroneStatus from ";
  switch (drone_status_)
  {
    case DroneStatus::YAWING:
      std::cout << bold << "YAWING" << reset;
      break;
    case DroneStatus::TRAVELING:
      std::cout << bold << "TRAVELING" << reset;
      break;
    case DroneStatus::GOAL_SEEN:
      std::cout << bold << "GOAL_SEEN" << reset;
      break;
    case DroneStatus::GOAL_REACHED:
      std::cout << bold << "GOAL_REACHED" << reset;
      break;
  }
  std::cout << " to ";

  switch (new_status)
  {
    case DroneStatus::YAWING:
      std::cout << bold << "YAWING" << reset;
      break;
    case DroneStatus::TRAVELING:
      std::cout << bold << "TRAVELING" << reset;
      break;
    case DroneStatus::GOAL_SEEN:
      std::cout << bold << "GOAL_SEEN" << reset;
      break;
    case DroneStatus::GOAL_REACHED:
      std::cout << bold << "GOAL_REACHED" << reset;
      break;
  }

  // std::cout << std::endl;

  drone_status_ = new_status;
}

void Rmader::printDroneStatus()
{
  switch (drone_status_)
  {
    case DroneStatus::YAWING:
      std::cout << bold << "status_=YAWING" << reset << std::endl;
      break;
    case DroneStatus::TRAVELING:
      std::cout << bold << "status_=TRAVELING" << reset << std::endl;
      break;
    case DroneStatus::GOAL_SEEN:
      std::cout << bold << "status_=GOAL_SEEN" << reset << std::endl;
      break;
    case DroneStatus::GOAL_REACHED:
      std::cout << bold << "status_=GOAL_REACHED" << reset << std::endl;
      break;
  }
}

void Rmader::getID(int& id)
{
  id_ = id;
}
