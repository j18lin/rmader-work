/* ----------------------------------------------------------------------------
 * Small dense convex QP solver (dual active set). See dense_qp.hpp.
 * -------------------------------------------------------------------------- */

#include "dense_qp.hpp"

#include <algorithm>
#include <limits>
#include <vector>

namespace dqp
{
const char* statusName(Status status)
{
  switch (status)
  {
    case Status::OPTIMAL:
      return "optimal";
    case Status::INFEASIBLE:
      return "infeasible";
    case Status::ITERATION_LIMIT:
      return "iteration limit";
    default:
      return "numerical";
  }
}

Result solve(const Eigen::MatrixXd& G, const Eigen::VectorXd& g, const Eigen::MatrixXd& A,
             const Eigen::VectorXd& b, const Options& options)
{
  Result result;
  const int n = G.rows();
  const int m = A.rows();
  result.lambda = Eigen::VectorXd::Zero(m);

  Eigen::LLT<Eigen::MatrixXd> llt(G);
  if (n == 0 || llt.info() != Eigen::Success)
  {
    result.status = Status::NUMERICAL;
    return result;
  }
  const Eigen::MatrixXd Ginv = llt.solve(Eigen::MatrixXd::Identity(n, n));

  // The method works on constraints written as n_i' x >= c_i, so n_i = -a_i.
  // Rows are normalised only to rank the violations; the tolerance is applied
  // to the rows as given.
  Eigen::VectorXd row_norm(m);
  for (int i = 0; i < m; ++i)
  {
    row_norm[i] = std::max(A.row(i).norm(), 1e-12);
  }

  Eigen::VectorXd x = -(Ginv * g);  // unconstrained minimiser
  std::vector<int> active;          // rows of A in the working set
  std::vector<double> u;            // their multipliers

  const int max_iterations = options.max_iterations > 0 ? options.max_iterations : 20 * (n + m) + 50;
  constexpr double inf = std::numeric_limits<double>::infinity();

  while (result.iterations < max_iterations)
  {
    // Most violated constraint outside the working set.
    int p = -1;
    double worst = 0.0;
    const Eigen::VectorXd slack = b - A * x;
    for (int i = 0; i < m; ++i)
    {
      if (slack[i] >= -options.feasibility_tol)
      {
        continue;
      }
      if (std::find(active.begin(), active.end(), i) != active.end())
      {
        continue;
      }
      const double scaled = slack[i] / row_norm[i];
      if (scaled < worst)
      {
        worst = scaled;
        p = i;
      }
    }
    if (p < 0)
    {
      result.status = Status::OPTIMAL;
      result.x = x;
      for (size_t k = 0; k < active.size(); ++k)
      {
        result.lambda[active[k]] = u[k];
      }
      result.cost = 0.5 * x.dot(G * x) + g.dot(x);
      return result;
    }

    const Eigen::VectorXd np = -A.row(p).transpose();
    double up = 0.0;  // multiplier of the constraint being added

    // Bring constraint p into the working set, dropping others as needed.
    while (true)
    {
      result.iterations++;
      if (result.iterations > max_iterations)
      {
        result.status = Status::ITERATION_LIMIT;
        return result;
      }

      const int q = active.size();
      Eigen::VectorXd z;                             // step in x
      Eigen::VectorXd r = Eigen::VectorXd::Zero(q);  // change of the multipliers
      if (q > 0)
      {
        Eigen::MatrixXd N(n, q);
        for (int k = 0; k < q; ++k)
        {
          N.col(k) = -A.row(active[k]).transpose();
        }
        const Eigen::MatrixXd GiN = Ginv * N;
        const Eigen::MatrixXd M = N.transpose() * GiN;
        r = M.ldlt().solve(GiN.transpose() * np);
        z = Ginv * (np - N * r);
      }
      else
      {
        z = Ginv * np;
      }

      const double curvature = z.dot(np);  // >= 0, zero when np depends on the working set
      const bool no_primal_step = curvature <= 1e-11 * std::max(1.0, np.dot(Ginv * np));

      double t1 = inf;  // largest step that keeps the multipliers non-negative
      int drop = -1;
      for (int k = 0; k < q; ++k)
      {
        if (r[k] > 1e-12)
        {
          const double t = u[k] / r[k];
          if (t < t1)
          {
            t1 = t;
            drop = k;
          }
        }
      }

      const double violation = np.dot(x) + b[p];  // n_p' x - c_p with c_p = -b_p, negative
      const double t2 = no_primal_step ? inf : -violation / curvature;
      const double t = std::min(t1, t2);

      if (t == inf)
      {
        result.status = Status::INFEASIBLE;
        return result;
      }

      if (t2 == inf)
      {
        // Step in the multipliers only, then drop the blocking constraint.
        for (int k = 0; k < q; ++k)
        {
          u[k] -= t * r[k];
        }
        up += t;
        active.erase(active.begin() + drop);
        u.erase(u.begin() + drop);
        continue;
      }

      x += t * z;
      for (int k = 0; k < q; ++k)
      {
        u[k] -= t * r[k];
      }
      up += t;

      if (t2 <= t1)
      {
        active.push_back(p);  // full step: p is now satisfied with equality
        u.push_back(up);
        break;
      }
      active.erase(active.begin() + drop);  // partial step
      u.erase(u.begin() + drop);
    }
  }

  result.status = Status::ITERATION_LIMIT;
  return result;
}
}  // namespace dqp
