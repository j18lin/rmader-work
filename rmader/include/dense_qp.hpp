/* ----------------------------------------------------------------------------
 * Small dense convex QP solver (dual active set, Goldfarb and Idnani 1983).
 *
 *   minimise    0.5 x' G x + g' x
 *   subject to  A x <= b
 *
 * G must be symmetric positive definite. The solver is exact: it returns the
 * optimum with its multipliers, or reports that the constraints have no
 * common point. It is written for the corridor optimiser, where the number of
 * variables is tiny (a handful) and the number of constraints is a few
 * hundred, so the factorisation is redone from scratch at every step instead
 * of being updated.
 * -------------------------------------------------------------------------- */

#pragma once

#include <Eigen/Dense>

namespace dqp
{
enum class Status
{
  OPTIMAL,
  INFEASIBLE,
  ITERATION_LIMIT,
  NUMERICAL
};

struct Result
{
  Status status = Status::NUMERICAL;
  Eigen::VectorXd x;       // minimiser (valid when status == OPTIMAL)
  Eigen::VectorXd lambda;  // one multiplier per row of A, >= 0
  double cost = 0.0;       // 0.5 x' G x + g' x
  int iterations = 0;
};

struct Options
{
  double feasibility_tol = 1e-8;  // largest accepted violation of A x <= b
  int max_iterations = 0;         // 0: chosen from the problem size
};

Result solve(const Eigen::MatrixXd& G, const Eigen::VectorXd& g, const Eigen::MatrixXd& A,
             const Eigen::VectorXd& b, const Options& options = Options());

const char* statusName(Status status);
}  // namespace dqp
