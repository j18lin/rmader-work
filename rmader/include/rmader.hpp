/* ----------------------------------------------------------------------------
 * Copyright 2022, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#pragma once
#ifndef RMADER_HPP
#define RMADER_HPP

#include <vector>
#include "cgal_utils.hpp"

#include <map>
#include <mutex>
#include <set>

#include "rmader_types.hpp"
#include "corridor_planner.hpp"
#include "stsfc.hpp"
#include "voxel_map.hpp"
#include "cloud_tracker.hpp"
#include <memory>

#if USE_GUROBI_FLAG
#include "solver_gurobi.hpp"
#else
#include "solver_nlopt.hpp"
#endif

// status_ : YAWING-->TRAVELING-->GOAL_SEEN-->GOAL_REACHED-->YAWING-->TRAVELING-->...

enum DroneStatus
{
  YAWING = 0,
  TRAVELING = 1,
  GOAL_SEEN = 2,
  GOAL_REACHED = 3
};

enum PlannerStatus
{
  FIRST_PLAN = 0,
  START_REPLANNING = 1,
  REPLANNED = 2
};

using namespace termcolor;

class Rmader
{
public:
  Rmader(mt::parameters par);
  bool replan(mt::Edges& edges_obstacles_out, std::vector<mt::state>& X_safe_out, std::vector<Hyperplane3D>& planes,
              int& num_of_LPs_run, int& num_of_QCQPs_run, mt::PieceWisePol& pwp_out);
  bool replan_with_delaycheck(mt::Edges& edges_obstacles_out, std::vector<mt::state>& headsup_plan,
                              std::vector<Hyperplane3D>& planes, int& num_of_LPs_run, int& num_of_QCQPs_run,
                              mt::PieceWisePol& pwp_now, double& headsup_time);
  bool addTrajToPlan_with_delaycheck(mt::PieceWisePol& pwp);
  bool delayCheck(mt::PieceWisePol pwp_optimized, const double& headsup_time);

  void updateState(mt::state data);

  bool getNextGoal(mt::state& next_goal);
  void getState(mt::state& data);
  void getG(mt::state& G);
  void getDetourG(mt::state& G);
  void moveAtowardG(mt::state& A, mt::state& G);
  void setTerminalGoal(mt::state& term_goal);
  void resetInitialization();

  bool IsTranslating();
  void updateTrajObstacles_with_delaycheck(mt::dynTraj traj);
  void updateTrajObstacles(mt::dynTraj traj);
  // Feeds one observation of an obstacle whose future motion is unknown (see dyn_obstacles.hpp).
  void updateObservedObstacle(const dynobs::Observation& obs);
  std::vector<dynobs::Track> getObservedTracks();
  // Teammates that have gone quiet (peer_silence). notePeerHeard: any message from that teammate arrived.
  // holdForQuietPeers: true while some teammate has been quiet for longer than hold_after but not yet
  // silent_after; also refreshes which teammates count as silent. Both take mtx_trajs_.
  void notePeerHeard(int id, double time);
  bool holdForQuietPeers();
  std::vector<int> getSilentPeers();
  bool isEvading();

  Eigen::Vector2d RotationMatrix(Eigen::Vector2d& vec, const double& angle);
  void getID(int& id);
  mt::state moveAoutOfBbox(const mt::state& A);
  int getMissedMsgsCnt();
  mt::state getGterm();
  bool isGoalSeen();
  bool isGoalReached();
  bool initializedAllExceptPlanner();

  std::vector<mt::dynTrajCompiled> getTrajs();

private:
  mt::state M_;
  mt::committedTrajectory plan_;

  double previous_yaw_ = 0.0;
  double initial_yaw_ = 0.0;

  bool isReplanningNeeded();

  void dynTraj2dynTrajCompiled(const mt::dynTraj& traj, mt::dynTrajCompiled& traj_compiled);

  bool initializedStateAndTermGoal();

  bool safetyCheckAfterOptForRmader(mt::PieceWisePol pwp_optimized, bool& is_q0_fail);
  bool safetyCheckAfterOpt(mt::PieceWisePol pwp_optimized);

  bool safetyCheck_for_A_star_failure(mt::PieceWisePol pwp_prev);
  bool safetyCheck_for_A_star_failure_pwp_now(mt::PieceWisePol pwp_now);

  bool trajsAndPwpAreInCollision_with_inflation(mt::dynTrajCompiled traj, mt::PieceWisePol pwp_optimized,
                                                double t_start, double t_end, bool& is_q0_fail);

  bool trajsAndPwpAreInCollision(mt::dynTrajCompiled traj, mt::PieceWisePol pwp_optimized, double t_start,
                                 double t_end);

  void removeTrajsThatWillNotAffectMe(const mt::state& A, double t_start, double t_end);

  // Observation-based dynamic obstacles. These expect mtx_trajs_ to be held by the caller...
  mt::dynTrajCompiled compileObservedTrack(const dynobs::Track& track, double time_received);
  Eigen::Vector3d centerOfObstacle(mt::dynTrajCompiled& traj, double t);
  void pruneObservedObstacles(double now);
  void setHeatCostForSolver();
  // ...and these take it themselves.
  bool hasObservedObstacles();
  double observedObstacleHorizon();
  double timeAllocationFactor(double dist_to_terminal_goal);
  enum class Subgoal
  {
    UNCHANGED,  // nothing observed (or no global path): G stays upstream's point on the straight line
    FOUND,      // G moved to a point along the global path that is safe to stop at
    NONE_SAFE   // no point along the path is safe to stop at right now: better not to plan a move at all
  };
  Subgoal subgoalAroundObservedObstacles(const mt::state& A, const mt::state& G_term, double ra, double t_start,
                                         Eigen::Vector3d& G);
  // --- SANDO's corridor optimiser as backend (par_.corridor) ---
  bool optimizeInCorridors(mt::state& A, const mt::state& initial, const mt::state& final, double t_start,
                           double max_runtime, mt::Edges& edges_obstacles_out, std::vector<Hyperplane3D>& planes);
  void getOptimizedPwp(mt::PieceWisePol& pwp);
  bool groundFeasible(const corridor::Solution& solution, double yaw);
  dynobs::GlobalPathExtras globalPathExtras(double t_start, double horizon);
  mt::state turnTarget(const mt::state& A, const mt::state& G);
  // --- Map of static and unknown space (par_.map), used by the corridor backend ---
public:
  // One range scan in the world frame: where the sensor was and where its rays ended.
  void updateMap(const Eigen::Vector3d& origin, const std::vector<Eigen::Vector3d>& points);
  std::shared_ptr<const vmap::VoxelMap> mapSnapshot();

private:
  double unknownSpeed() const;
  Eigen::Vector3d mapHalfSize() const;
  void trackMovingObstacles(const Eigen::Vector3d& origin, const std::vector<Eigen::Vector3d>& points);
  std::unique_ptr<ctrack::CloudTracker> cloud_tracker_;
  std::unique_ptr<vmap::VoxelMap> map_;
  std::mutex mtx_map_;
  std::shared_ptr<const vmap::VoxelMap> map_now_;  // the map this replanning step works with

  mt::PieceWisePol corridor_pwp_;
  corridor::FactorWindow corridor_factors_;
  std::vector<Eigen::Vector3d> corridor_path_;  // global path behind the current subgoal, if one was planned

  bool evasionSubgoal(const mt::state& A, double ra, Eigen::Vector3d& G);
  bool escapeSubgoal(const mt::state& A, const Eigen::Vector3d& G_straight, double t_start, Eigen::Vector3d& G);
  // Ground mode: true if the rover is stopped and has to turn on the spot before it can head for G.
  bool groundTurnNeeded(const mt::state& A, const mt::state& G);

  // Moves a stopped agent out of the way of an approaching observed obstacle, and back afterwards.
  void updateHoverAvoidance();
  void setTerminalGoalInternal(mt::state& term_goal);

  /*  vec_E<Polyhedron<3>> cu::vectorGCALPol2vectorJPSPol(ConvexHullsOfCurves& convex_hulls_of_curves);
    mt::ConvexHullsOfCurves_Std cu::vectorGCALPol2vectorStdEigen(ConvexHullsOfCurves& convexHulls);*/
  ConvexHullsOfCurves convexHullsOfCurves(double t_start, double t_end);
  ConvexHullsOfCurve convexHullsOfCurve(mt::dynTrajCompiled& traj, double t_start, double t_end);
  CGAL_Polyhedron_3 convexHullOfInterval(mt::dynTrajCompiled& traj, double t_start, double t_end);

  std::vector<Eigen::Vector3d> vertexesOfInterval(mt::PieceWisePol& pwp, double t_start, double t_end,
                                                  const Eigen::Vector3d& delta_inflation);
  std::vector<Eigen::Vector3d> vertexesOfInterval(mt::dynTrajCompiled& traj, double t_start, double t_end);
  void yaw(double diff, mt::state& next_goal);

  void changeBBox(Eigen::Vector3d& drone_boundarybox);

  void getDesiredYaw(mt::state& next_goal);

  void updateInitialCond(int i);

  void changeDroneStatus(int new_status);

  bool appendToPlan(int k_end_whole, const std::vector<mt::state>& whole, int k_safe,
                    const std::vector<mt::state>& safe);

  bool initialized();

  void printDroneStatus();

  mt::parameters par_;

  double t_;  // variable where the expressions of the trajs of the dyn obs are evaluated

  std::mutex mtx_trajs_;
  std::vector<mt::dynTrajCompiled> trajs_;

  bool state_initialized_ = false;
  bool planner_initialized_ = false;

  int deltaT_ = 75;

  int k_index_end_ = 0;

  double time_now_ = 0;
  double last_ground_goal_seen_replan_time_ = 0.0;

  bool terminal_goal_initialized_ = false;

  int drone_status_ = DroneStatus::TRAVELING;  // status_ can be TRAVELING, GOAL_SEEN, GOAL_REACHED
  int planner_status_ = PlannerStatus::FIRST_PLAN;

  double dyaw_filtered_ = 0;

  std::mutex mtx_goals;

  std::mutex mtx_k;

  std::mutex mtx_planner_status_;
  std::mutex mtx_initial_cond;
  std::mutex mtx_state;
  std::mutex mtx_offsets;
  std::mutex mtx_plan_;
  // std::mutex mtx_factors;

  std::mutex mtx_G;
  std::mutex mtx_G_term;
  std::mutex mtx_t_;

  std::mutex mtx_hrtwch_;

  mt::state stateA_;  // It's the initial condition for the solver

  mt::state state_;
  mt::state stuck_state_;
  mt::state stuck_state_for_bbox_;
  mt::state pop_up_state_;
  mt::state pop_up_last_state_in_plan_;
  mt::state G_;       // This goal is always inside of the map
  mt::state G_term_;  // This goal is the clicked goal
  mt::state detoured_G_;

  int solutions_found_ = 0;
  int total_replannings_ = 0;

  mt::PieceWisePol pwp_prev_;

  bool exists_previous_pwp_ = false;

  bool started_check_ = false;

  bool have_received_trajectories_while_checking_ = false;

  double time_init_opt_;

  bool if_detour_ = false;

  MyTimer timer_detour_;

  mt::state A_when_stuck_;
  mt::state G_when_stuck_;

  bool if_bbox_change_ = false;

  MyTimer timer_bbox_;

  int stuck_count_for_bbox_ = 0;
  int stuck_count_for_detour_ = 0;

  bool if_A_moveback_ = false;

  bool is_pwp_prev_feasible_ = false;

  bool is_pop_up_ = false;

  int id_;

  bool is_z_max_increased_ = false;

  bool is_going_back_to_normal_z_max_ = false;

  int A_star_fail_count_ = 0;

  int A_star_fail_count_pwp_now_ = 0;

  bool is_pop_up_initialized_ = false;

  bool is_A_star_failed_30_ = false;

  int q0_fail_count_ = 0;
  int opt_fail_count_ = 0;
  int safetycheck_fail_count_ = 0;

  bool is_movingAoutOfBbox_ = false;
  mt::state movedA_;

  // double av_improvement_nlopt_ = 0.0;

#if USE_GUROBI_FLAG
  std::unique_ptr<SolverGurobi> solver_;  // pointer to the optimization solver
#else
  std::unique_ptr<SolverNlopt> solver_;  // pointer to the optimization solver
#endif

  Eigen::Matrix<double, 4, 4> A_rest_pos_basis_;
  Eigen::Matrix<double, 4, 4> A_rest_pos_basis_inverse_;

  separator::Separator* separator_solver_;

  // Every teammate ever heard from. Kept apart from trajs_, which drops a teammate's trajectory as soon as it
  // stops mattering and relies on the teammate publishing again to get it back: a silent one never will.
  struct Peer
  {
    double last_heard = 0.0;
    bool has_trajectory = false;
    mt::dynTrajCompiled last_committed;
  };
  std::map<int, Peer> peers_;   // guarded by mtx_trajs_
  std::set<int> silent_peers_;  // guarded by mtx_trajs_, refreshed by holdForQuietPeers
  dynobs::Box silentPeerBox(mt::dynTrajCompiled& traj, double t_start, double t_end);

  dynobs::Tracker obs_tracker_;  // guarded by mtx_trajs_
  // Extra clearance around observed obstacles; non-zero only while building the hulls to plan against
  // (guarded by mtx_trajs_). The checks that follow use the boxes as they are.
  double observed_extra_margin_ = 0.0;
  // True for a plan made under Params::escape_when_engulfed: observed obstacles count as their bodies only,
  // in the hulls and in the checks of that plan. Set at the start of each replan.
  bool observed_escape_ = false;
  bool ground_turn_requested_ = false;  // a stopped rover is being turned on the spot towards ground_turn_yaw_
  double ground_turn_yaw_ = 0.0;
  bool is_evading_ = false;      // true while the terminal goal is an evasion goal
  mt::state hover_home_;         // terminal goal to return to after evading
};

#endif
