/* ----------------------------------------------------------------------------
 * Trajectory optimisation inside spatio-temporal safe flight corridors.
 *
 * Port of the local optimiser of SANDO (Kondo, Tordesillas, How; BSD 3-Clause,
 * Copyright (c) 2024 Kota Kondo), src/sando/gurobi_solver.cpp:
 *
 *   - N cubic pieces of equal duration dt, written with Bezier control points;
 *   - position, velocity and acceleration continuous at the knots;
 *   - the start state is given, the trajectory ends at rest at the goal;
 *   - piece n lies in ONE polytope chosen among the P polytopes of time layer
 *     n (all four control points inside it);
 *   - per-axis bounds on velocity, acceleration and jerk control points;
 *   - cost: sum of the squared jerk of the pieces.
 *
 * SANDO states this as a mixed-integer QP (one binary per piece and polytope)
 * and hands it to Gurobi. Here the equalities are eliminated first, which
 * leaves N - 3 free variables per axis as in the paper, and the choice of
 * polytopes is enumerated: every assignment gives a small convex QP, solved
 * exactly by dqp::solve. With `exhaustive` all P^N assignments are tried, so
 * the result is the optimum of the mixed-integer problem; by default only the
 * assignments that never go back to an earlier path segment are tried.
 *
 * This file does not depend on ROS.
 * SANDO's licence text is in rmader/LICENSE.SANDO.
 * -------------------------------------------------------------------------- */

#pragma once

#include <Eigen/Dense>
#include <limits>
#include <vector>

namespace corridor
{
struct Polytope  // { x in R^3 : A x <= b }
{
  Eigen::MatrixXd A;
  Eigen::VectorXd b;

  bool empty() const
  {
    return A.rows() == 0;
  }
  bool contains(const Eigen::Vector3d& point, double tolerance = 1e-6) const;
  // Move every face inwards by `distance` (rows of A must have unit norm).
  void shrink(double distance);
};

typedef std::vector<std::vector<Polytope>> Layers;  // [time layer n][path segment p]

struct State
{
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
};

struct Limits  // per axis, as in R-MADER and in SANDO's "Linf" mode
{
  Eigen::Vector3d v_max = Eigen::Vector3d::Constant(1.0);
  Eigen::Vector3d a_max = Eigen::Vector3d::Constant(1.0);
  Eigen::Vector3d j_max = Eigen::Vector3d::Constant(1.0);
};

struct Problem
{
  State start;
  State goal;          // the trajectory ends here; SANDO uses zero velocity and acceleration
  int num_pieces = 5;  // N
  double dt = 1.0;     // duration of one piece
  Layers corridors;    // [N][P]; an empty polytope cannot be chosen
  Limits limits;
  Eigen::Vector3d lower = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
  Eigen::Vector3d upper = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());

  // Ground robots: the velocity of the first piece stays in a cone around the
  // current heading (same constraint as the NLopt backend).
  bool heading_cone = false;
  double heading = 0.0;           // [rad]
  double heading_cone_tan = 0.0;  // tan of the half angle

  bool exhaustive = false;  // try all P^N assignments instead of the ordered ones
};

struct Solution
{
  std::vector<Eigen::Matrix<double, 3, 4>> control_points;  // Bezier points of each piece
  double dt = 0.0;
  std::vector<int> assignment;  // polytope chosen for each piece
  double cost = 0.0;            // sum over the pieces of |jerk|^2 * dt
  int assignments_tried = 0;
  int qps_solved = 0;

  double duration() const
  {
    return dt * control_points.size();
  }
  // t is measured from the start of the trajectory and is clamped to it.
  Eigen::Vector3d pos(double t) const;
  Eigen::Vector3d vel(double t) const;
  Eigen::Vector3d acc(double t) const;
  Eigen::Vector3d jerk(double t) const;
  // Coefficients [a b c d] of piece n so that p(u) = a u^3 + b u^2 + c u + d, u in [0, 1].
  Eigen::Matrix<double, 3, 4> polynomial(int piece) const;
};

// Solve for one value of dt. Returns false when no assignment is feasible.
bool solve(const Problem& problem, Solution& solution);

// Largest violation of the constraints of `problem` by `solution` (<= 0 when
// all hold). Used by the tests and as a last check before a trajectory is used.
double worstViolation(const Problem& problem, const Solution& solution);

// SANDO's first guess of the piece duration (getInitialDt): the longest of
// the per-axis times needed under the velocity, acceleration and jerk bounds
// taken one at a time, divided by the number of pieces.
double initialDt(const State& start, const State& goal, const Limits& limits, int num_pieces);

// SANDO's window of time-allocation factors (dt = factor * dt0). The window
// is centred on the last factor that worked, moves up after a failure, and
// goes back to the initial window once it reaches the upper end.
class FactorWindow
{
public:
  FactorWindow(double initial_mean = 1.5, double radius = 0.4, double step = 0.1, double lowest = 1.0,
               double highest = 2.5);
  const std::vector<double>& factors() const
  {
    return factors_;
  }
  // The factor to expect: the middle of the window (the last factor that worked, until a failure moves it).
  double expected() const;
  void succeeded(double factor);
  void failed();

private:
  void centre(double mean);
  double initial_mean_, radius_, step_, lowest_, highest_;
  int count_;
  std::vector<double> factors_;
};
}  // namespace corridor
