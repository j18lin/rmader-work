/* ----------------------------------------------------------------------------
 * Trajectory optimisation inside spatio-temporal safe flight corridors.
 * See corridor_planner.hpp for what is ported from SANDO and what differs.
 * -------------------------------------------------------------------------- */

#include "corridor_planner.hpp"
#include "dense_qp.hpp"

#include <unsupported/Eigen/Polynomials>

#include <algorithm>
#include <cmath>
#include <map>
#include <mutex>

namespace corridor
{
namespace
{
constexpr double kFixedTolerance = 1e-3;  // slack for constraints that the start or goal state decides alone

// ---------------------------------------------------------------------------
// Elimination of the equality constraints
// ---------------------------------------------------------------------------
// Per axis the unknowns are the 4N Bezier control points c. Start state, final
// state and continuity up to acceleration are 3N + 3 independent linear
// equations E c = d, where E depends only on N (the durations appear in d).
// Every solution is c = E^+ d + Z y with Z a basis of the null space of E,
// which has N - 3 columns: the free variables of the paper.
struct Basis
{
  int N = 0;
  int free = 0;            // N - 3
  Eigen::MatrixXd E_pinv;  // 4N x (3N + 3)
  Eigen::MatrixXd Z;       // 4N x free, orthonormal columns
  Eigen::MatrixXd D3;      // N x 4N, third difference of the control points of each piece
  Eigen::MatrixXd H;       // free x free, Z' D3' D3 Z
};

Basis makeBasis(int N)
{
  Basis basis;
  basis.N = N;
  basis.free = N - 3;
  const int cols = 4 * N;
  const int rows = 3 * N + 3;
  Eigen::MatrixXd E = Eigen::MatrixXd::Zero(rows, cols);

  E(0, 0) = 1.0;  // start position
  E(1, 1) = 1.0;  // start velocity * dt / 3
  E(1, 0) = -1.0;
  E(2, 2) = 1.0;  // start acceleration * dt^2 / 6
  E(2, 1) = -2.0;
  E(2, 0) = 1.0;
  E(3, cols - 1) = 1.0;  // final position
  E(4, cols - 1) = 1.0;  // final velocity * dt / 3
  E(4, cols - 2) = -1.0;
  E(5, cols - 1) = 1.0;  // final acceleration * dt^2 / 6
  E(5, cols - 2) = -2.0;
  E(5, cols - 3) = 1.0;
  for (int n = 0; n + 1 < N; ++n)
  {
    const int r = 6 + 3 * n;
    const int a = 4 * n;      // this piece
    const int b = 4 * n + 4;  // next piece
    E(r, a + 3) = 1.0;        // position
    E(r, b) = -1.0;
    E(r + 1, a + 3) = 1.0;  // velocity
    E(r + 1, a + 2) = -1.0;
    E(r + 1, b + 1) = -1.0;
    E(r + 1, b) = 1.0;
    E(r + 2, a + 3) = 1.0;  // acceleration
    E(r + 2, a + 2) = -2.0;
    E(r + 2, a + 1) = 1.0;
    E(r + 2, b + 2) = -1.0;
    E(r + 2, b + 1) = 2.0;
    E(r + 2, b) = -1.0;
  }

  Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod(E);
  basis.E_pinv = cod.pseudoInverse();

  Eigen::FullPivLU<Eigen::MatrixXd> lu(E);
  Eigen::MatrixXd kernel = lu.kernel();
  Eigen::HouseholderQR<Eigen::MatrixXd> qr(kernel);
  basis.Z = qr.householderQ() * Eigen::MatrixXd::Identity(cols, basis.free);

  basis.D3 = Eigen::MatrixXd::Zero(N, cols);
  for (int n = 0; n < N; ++n)
  {
    basis.D3(n, 4 * n) = -1.0;
    basis.D3(n, 4 * n + 1) = 3.0;
    basis.D3(n, 4 * n + 2) = -3.0;
    basis.D3(n, 4 * n + 3) = 1.0;
  }
  const Eigen::MatrixXd D3Z = basis.D3 * basis.Z;
  basis.H = D3Z.transpose() * D3Z;
  return basis;
}

const Basis& basisFor(int N)
{
  static std::mutex mutex;
  static std::map<int, Basis> cache;
  std::lock_guard<std::mutex> lock(mutex);
  auto it = cache.find(N);
  if (it == cache.end())
  {
    it = cache.emplace(N, makeBasis(N)).first;
  }
  return it->second;
}

// ---------------------------------------------------------------------------
// Constraints as rows over the free variables y = [y_x; y_y; y_z]
// ---------------------------------------------------------------------------
struct Rows
{
  std::vector<Eigen::RowVectorXd> a;
  std::vector<double> b;
  bool infeasible = false;  // a constraint without free variables is violated

  // coefficient * y + constant <= limit
  void add(const Eigen::RowVectorXd& coefficient, double constant, double limit)
  {
    if (coefficient.norm() < 1e-10)
    {
      if (constant > limit + kFixedTolerance)
      {
        infeasible = true;
      }
      return;
    }
    a.push_back(coefficient);
    b.push_back(limit - constant);
  }
};

struct Affine  // one scalar quantity: row * y + constant
{
  Eigen::RowVectorXd row;
  double constant = 0.0;
};

struct Model
{
  const Basis* basis = nullptr;
  int free = 0;                 // per axis
  Eigen::VectorXd particular[3];  // E^+ d of each axis

  // Weighted sum of control points of one axis.
  Affine combine(int axis, std::initializer_list<std::pair<int, double>> terms) const
  {
    Affine out;
    out.row = Eigen::RowVectorXd::Zero(3 * free);
    for (const auto& term : terms)
    {
      out.row.segment(axis * free, free) += term.second * basis->Z.row(term.first);
      out.constant += term.second * particular[axis][term.first];
    }
    return out;
  }
};

Affine operator*(double scale, const Affine& value)
{
  Affine out;
  out.row = scale * value.row;
  out.constant = scale * value.constant;
  return out;
}

Affine operator+(const Affine& lhs, const Affine& rhs)
{
  Affine out;
  out.row = lhs.row + rhs.row;
  out.constant = lhs.constant + rhs.constant;
  return out;
}

void addTwoSided(Rows& rows, const Affine& value, double limit)
{
  rows.add(value.row, value.constant, limit);
  rows.add(-value.row, -value.constant, limit);
}

Model makeModel(const Problem& problem)
{
  Model model;
  model.basis = &basisFor(problem.num_pieces);
  model.free = model.basis->free;
  const int N = problem.num_pieces;
  const double dt = problem.dt;
  for (int axis = 0; axis < 3; ++axis)
  {
    Eigen::VectorXd d = Eigen::VectorXd::Zero(3 * N + 3);
    d[0] = problem.start.pos[axis];
    d[1] = problem.start.vel[axis] * dt / 3.0;
    d[2] = problem.start.acc[axis] * dt * dt / 6.0;
    d[3] = problem.goal.pos[axis];
    d[4] = problem.goal.vel[axis] * dt / 3.0;
    d[5] = problem.goal.acc[axis] * dt * dt / 6.0;
    model.particular[axis] = model.basis->E_pinv * d;
  }
  return model;
}

// Bounds on velocity, acceleration, jerk and position, and the heading cone.
Rows commonRows(const Problem& problem, const Model& model)
{
  Rows rows;
  const int N = problem.num_pieces;
  const double dt = problem.dt;
  for (int axis = 0; axis < 3; ++axis)
  {
    for (int n = 0; n < N; ++n)
    {
      const int i = 4 * n;
      for (int j = 0; j < 3; ++j)
      {
        addTwoSided(rows, (3.0 / dt) * model.combine(axis, { { i + j + 1, 1.0 }, { i + j, -1.0 } }),
                    problem.limits.v_max[axis]);
      }
      for (int j = 0; j < 2; ++j)
      {
        addTwoSided(rows,
                    (6.0 / (dt * dt)) *
                        model.combine(axis, { { i + j + 2, 1.0 }, { i + j + 1, -2.0 }, { i + j, 1.0 } }),
                    problem.limits.a_max[axis]);
      }
      addTwoSided(rows,
                  (6.0 / (dt * dt * dt)) *
                      model.combine(axis, { { i + 3, 1.0 }, { i + 2, -3.0 }, { i + 1, 3.0 }, { i, -1.0 } }),
                  problem.limits.j_max[axis]);
      for (int j = 0; j < 4; ++j)
      {
        const Affine point = model.combine(axis, { { i + j, 1.0 } });
        if (std::isfinite(problem.upper[axis]))
        {
          rows.add(point.row, point.constant, problem.upper[axis]);
        }
        if (std::isfinite(problem.lower[axis]))
        {
          rows.add(-point.row, -point.constant, -problem.lower[axis]);
        }
      }
    }
  }

  if (problem.heading_cone)
  {
    const double hx = std::cos(problem.heading);
    const double hy = std::sin(problem.heading);
    const double cone = problem.heading_cone_tan;
    for (int j = 0; j < 3; ++j)  // velocity control points of the first piece
    {
      const Affine vx = (3.0 / dt) * model.combine(0, { { j + 1, 1.0 }, { j, -1.0 } });
      const Affine vy = (3.0 / dt) * model.combine(1, { { j + 1, 1.0 }, { j, -1.0 } });
      const Affine forward = hx * vx + hy * vy;
      const Affine lateral = (-hy) * vx + hx * vy;
      const Affine left = lateral + (-cone) * forward;
      const Affine right = (-1.0) * lateral + (-cone) * forward;
      // The fixed control points get the same 0.01 m/s slack as the NLopt backend.
      const double slack = forward.row.norm() < 1e-10 ? 0.01 - kFixedTolerance : 0.0;
      rows.add(-forward.row, -forward.constant, slack);
      rows.add(left.row, left.constant, slack);
      rows.add(right.row, right.constant, slack);
    }
  }
  return rows;
}

// The four control points of piece n inside one polytope.
Rows corridorRows(const Model& model, int n, const Polytope& polytope)
{
  Rows rows;
  for (int j = 0; j < 4; ++j)
  {
    Affine point[3];
    for (int axis = 0; axis < 3; ++axis)
    {
      point[axis] = model.combine(axis, { { 4 * n + j, 1.0 } });
    }
    for (int f = 0; f < polytope.A.rows(); ++f)
    {
      const Affine value =
          polytope.A(f, 0) * point[0] + polytope.A(f, 1) * point[1] + polytope.A(f, 2) * point[2];
      rows.add(value.row, value.constant, polytope.b[f]);
      if (rows.infeasible)
      {
        return rows;
      }
    }
  }
  return rows;
}

double bernstein(int i, double u)
{
  const double w = 1.0 - u;
  switch (i)
  {
    case 0:
      return w * w * w;
    case 1:
      return 3.0 * w * w * u;
    case 2:
      return 3.0 * w * u * u;
    default:
      return u * u * u;
  }
}

void locate(const Solution& solution, double t, int& piece, double& u)
{
  const int N = solution.control_points.size();
  const double clamped = std::min(std::max(t, 0.0), solution.dt * N);
  piece = std::min(static_cast<int>(clamped / solution.dt), N - 1);
  u = (clamped - piece * solution.dt) / solution.dt;
}

// Branch and bound over the polytope of each piece. A node fixes the polytopes
// of the first `depth` pieces; its QP (later pieces unconstrained by corridors)
// is a lower bound for every completion.
struct Search
{
  const Problem* problem;
  const Model* model;
  const Rows* common;
  std::vector<std::vector<Rows>> corridor;       // [n][p]
  std::vector<std::vector<int>> allowed;         // polytopes that piece n may use
  Eigen::MatrixXd G;
  Eigen::VectorXd g;
  double constant = 0.0;  // cost of the particular solution, so that costs compare across nodes

  std::vector<int> current;
  std::vector<int> best_assignment;
  Eigen::VectorXd best_y;
  double best_cost = std::numeric_limits<double>::infinity();
  int qps = 0;
  int leaves = 0;

  bool solveNode(int depth, Eigen::VectorXd& y, double& cost)
  {
    int count = common->a.size();
    for (int n = 0; n < depth; ++n)
    {
      count += corridor[n][current[n]].a.size();
    }
    Eigen::MatrixXd A(count, G.rows());
    Eigen::VectorXd b(count);
    int r = 0;
    auto append = [&](const Rows& rows) {
      for (size_t i = 0; i < rows.a.size(); ++i)
      {
        A.row(r) = rows.a[i];
        b[r] = rows.b[i];
        ++r;
      }
    };
    append(*common);
    for (int n = 0; n < depth; ++n)
    {
      append(corridor[n][current[n]]);
    }
    qps++;
    const dqp::Result result = dqp::solve(G, g, A, b);
    if (result.status != dqp::Status::OPTIMAL)
    {
      return false;
    }
    y = result.x;
    cost = result.cost + constant;
    return true;
  }

  void descend(int depth)
  {
    const int N = problem->num_pieces;
    Eigen::VectorXd y;
    double cost = 0.0;
    if (depth > 0)
    {
      if (!solveNode(depth, y, cost) || cost >= best_cost - 1e-12)
      {
        return;
      }
    }
    if (depth == N)
    {
      leaves++;
      best_cost = cost;
      best_y = y;
      best_assignment = current;
      return;
    }
    for (int p : allowed[depth])
    {
      if (!problem->exhaustive && depth > 0 && p < current[depth - 1])
      {
        continue;  // do not go back to an earlier path segment
      }
      current[depth] = p;
      descend(depth + 1);
    }
  }
};
}  // namespace

// ---------------------------------------------------------------------------
bool Polytope::contains(const Eigen::Vector3d& point, double tolerance) const
{
  if (empty())
  {
    return false;
  }
  return ((A * point - b).array() <= tolerance).all();
}

void Polytope::shrink(double distance)
{
  b.array() -= distance;
}

Eigen::Vector3d Solution::pos(double t) const
{
  int n;
  double u;
  locate(*this, t, n, u);
  const auto& c = control_points[n];
  return bernstein(0, u) * c.col(0) + bernstein(1, u) * c.col(1) + bernstein(2, u) * c.col(2) +
         bernstein(3, u) * c.col(3);
}

Eigen::Vector3d Solution::vel(double t) const
{
  int n;
  double u;
  locate(*this, t, n, u);
  const auto& c = control_points[n];
  const double w = 1.0 - u;
  return (3.0 / dt) * (w * w * (c.col(1) - c.col(0)) + 2.0 * w * u * (c.col(2) - c.col(1)) +
                       u * u * (c.col(3) - c.col(2)));
}

Eigen::Vector3d Solution::acc(double t) const
{
  int n;
  double u;
  locate(*this, t, n, u);
  const auto& c = control_points[n];
  return (6.0 / (dt * dt)) * ((1.0 - u) * (c.col(2) - 2.0 * c.col(1) + c.col(0)) +
                              u * (c.col(3) - 2.0 * c.col(2) + c.col(1)));
}

Eigen::Vector3d Solution::jerk(double t) const
{
  int n;
  double u;
  locate(*this, t, n, u);
  const auto& c = control_points[n];
  return (6.0 / (dt * dt * dt)) * (c.col(3) - 3.0 * c.col(2) + 3.0 * c.col(1) - c.col(0));
}

Eigen::Matrix<double, 3, 4> Solution::polynomial(int piece) const
{
  const auto& c = control_points[piece];
  Eigen::Matrix<double, 3, 4> out;
  out.col(0) = -c.col(0) + 3.0 * c.col(1) - 3.0 * c.col(2) + c.col(3);
  out.col(1) = 3.0 * c.col(0) - 6.0 * c.col(1) + 3.0 * c.col(2);
  out.col(2) = -3.0 * c.col(0) + 3.0 * c.col(1);
  out.col(3) = c.col(0);
  return out;
}

// ---------------------------------------------------------------------------
bool solve(const Problem& problem, Solution& solution)
{
  solution = Solution();
  const int N = problem.num_pieces;
  if (N < 4 || !(problem.dt > 0.0) || static_cast<int>(problem.corridors.size()) != N)
  {
    return false;
  }
  const Model model = makeModel(problem);
  const int free = model.free;

  const Rows common = commonRows(problem, model);
  if (common.infeasible)
  {
    return false;
  }

  Search search;
  search.problem = &problem;
  search.model = &model;
  search.common = &common;
  search.corridor.resize(N);
  search.allowed.resize(N);
  for (int n = 0; n < N; ++n)
  {
    const int P = problem.corridors[n].size();
    search.corridor[n].resize(P);
    for (int p = 0; p < P; ++p)
    {
      if (problem.corridors[n][p].empty())
      {
        continue;
      }
      search.corridor[n][p] = corridorRows(model, n, problem.corridors[n][p]);
      if (!search.corridor[n][p].infeasible)
      {
        search.allowed[n].push_back(p);
      }
    }
    if (search.allowed[n].empty())
    {
      return false;
    }
  }

  // Cost: sum of squared third differences (the factor 36 / dt^5 is added at the end).
  search.G = Eigen::MatrixXd::Zero(3 * free, 3 * free);
  search.g = Eigen::VectorXd::Zero(3 * free);
  const double regularisation = 1e-12 * std::max(1.0, model.basis->H.trace());
  for (int axis = 0; axis < 3; ++axis)
  {
    search.G.block(axis * free, axis * free, free, free) =
        model.basis->H + regularisation * Eigen::MatrixXd::Identity(free, free);
    const Eigen::VectorXd third = model.basis->D3 * model.particular[axis];
    search.g.segment(axis * free, free) = (model.basis->D3 * model.basis->Z).transpose() * third;
    search.constant += 0.5 * third.squaredNorm();
  }

  search.current.assign(N, 0);
  search.descend(0);
  solution.assignments_tried = search.leaves;
  solution.qps_solved = search.qps;
  if (search.best_assignment.empty())
  {
    return false;
  }

  solution.dt = problem.dt;
  solution.assignment = search.best_assignment;
  solution.control_points.resize(N);
  for (int axis = 0; axis < 3; ++axis)
  {
    const Eigen::VectorXd c =
        model.particular[axis] + model.basis->Z * search.best_y.segment(axis * free, free);
    for (int n = 0; n < N; ++n)
    {
      for (int j = 0; j < 4; ++j)
      {
        solution.control_points[n](axis, j) = c[4 * n + j];
      }
    }
  }
  solution.cost = 0.0;
  for (int n = 0; n < N; ++n)
  {
    solution.cost += solution.jerk((n + 0.5) * solution.dt).squaredNorm() * solution.dt;
  }
  return true;
}

// ---------------------------------------------------------------------------
double worstViolation(const Problem& problem, const Solution& solution)
{
  const int N = problem.num_pieces;
  if (static_cast<int>(solution.control_points.size()) != N || static_cast<int>(solution.assignment.size()) != N)
  {
    return std::numeric_limits<double>::infinity();
  }
  const double dt = solution.dt;
  const double T = N * dt;
  double worst = -std::numeric_limits<double>::infinity();
  auto equal = [&](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    worst = std::max(worst, (a - b).cwiseAbs().maxCoeff());
  };

  equal(solution.control_points.front().col(0), problem.start.pos);
  equal(solution.vel(0.0), problem.start.vel);
  equal(solution.acc(0.0), problem.start.acc);
  equal(solution.control_points.back().col(3), problem.goal.pos);
  equal(solution.vel(T), problem.goal.vel);
  equal(solution.acc(T), problem.goal.acc);

  for (int n = 0; n < N; ++n)
  {
    const auto& c = solution.control_points[n];
    if (n + 1 < N)
    {
      const auto& d = solution.control_points[n + 1];
      equal(c.col(3), d.col(0));
      equal((c.col(3) - c.col(2)) * (3.0 / dt), (d.col(1) - d.col(0)) * (3.0 / dt));
      equal((c.col(3) - 2.0 * c.col(2) + c.col(1)) * (6.0 / (dt * dt)),
            (d.col(2) - 2.0 * d.col(1) + d.col(0)) * (6.0 / (dt * dt)));
    }
    const Polytope& polytope = problem.corridors[n][solution.assignment[n]];
    for (int j = 0; j < 4; ++j)
    {
      worst = std::max(worst, (polytope.A * c.col(j) - polytope.b).maxCoeff());
      worst = std::max(worst, (c.col(j) - problem.upper).maxCoeff());
      worst = std::max(worst, (problem.lower - c.col(j)).maxCoeff());
    }
    for (int j = 0; j < 3; ++j)
    {
      const Eigen::Vector3d v = (c.col(j + 1) - c.col(j)) * (3.0 / dt);
      worst = std::max(worst, (v.cwiseAbs() - problem.limits.v_max).maxCoeff());
    }
    for (int j = 0; j < 2; ++j)
    {
      const Eigen::Vector3d a = (c.col(j + 2) - 2.0 * c.col(j + 1) + c.col(j)) * (6.0 / (dt * dt));
      worst = std::max(worst, (a.cwiseAbs() - problem.limits.a_max).maxCoeff());
    }
    const Eigen::Vector3d jerk = (c.col(3) - 3.0 * c.col(2) + 3.0 * c.col(1) - c.col(0)) * (6.0 / (dt * dt * dt));
    worst = std::max(worst, (jerk.cwiseAbs() - problem.limits.j_max).maxCoeff());
  }
  return worst;
}

// ---------------------------------------------------------------------------
namespace
{
double minPositiveRoot(const Eigen::VectorXd& coefficients)  // lowest order first
{
  // Drop leading zeros of the highest orders so that the solver gets a true polynomial.
  int degree = coefficients.size() - 1;
  while (degree > 0 && std::abs(coefficients[degree]) < 1e-12)
  {
    degree--;
  }
  if (degree == 0)
  {
    return 0.0;
  }
  Eigen::PolynomialSolver<double, Eigen::Dynamic> solver(coefficients.head(degree + 1));
  std::vector<double> roots;
  solver.realRoots(roots);
  double best = 0.0;
  for (double root : roots)
  {
    if (root > 0.0 && (best == 0.0 || root < best))
    {
      best = root;
    }
  }
  return best;
}
}  // namespace

double initialDt(const State& start, const State& goal, const Limits& limits, int num_pieces)
{
  double longest = 0.0;
  for (int axis = 0; axis < 3; ++axis)
  {
    const double delta = goal.pos[axis] - start.pos[axis];
    const double sign = std::copysign(1.0, delta);
    longest = std::max(longest, std::abs(delta) / limits.v_max[axis]);

    Eigen::VectorXd quadratic(3);
    quadratic << -delta, start.vel[axis], 0.5 * sign * limits.a_max[axis];
    longest = std::max(longest, minPositiveRoot(quadratic));

    Eigen::VectorXd cubic(4);
    cubic << -delta, start.vel[axis], 0.5 * start.acc[axis], sign * limits.j_max[axis] / 6.0;
    longest = std::max(longest, minPositiveRoot(cubic));
  }
  return longest / std::max(1, num_pieces);
}

// ---------------------------------------------------------------------------
FactorWindow::FactorWindow(double initial_mean, double radius, double step, double lowest, double highest)
  : initial_mean_(initial_mean), radius_(radius), step_(step), lowest_(lowest), highest_(highest)
{
  count_ = static_cast<int>(2.0 * radius_ / step_ + 1e-9) + 1;
  centre(initial_mean_);
}

void FactorWindow::centre(double mean)
{
  factors_.clear();
  for (int i = 0; i < count_; ++i)
  {
    const double factor = mean - radius_ + i * step_;
    if (factor >= lowest_ - 1e-9 && factor <= highest_ + 1e-9)
    {
      factors_.push_back(factor);
    }
  }
  if (factors_.empty())
  {
    factors_.push_back(std::min(std::max(mean, lowest_), highest_));
  }
}

double FactorWindow::expected() const
{
  double mean = 0.0;
  for (double factor : factors_)
  {
    mean += factor;
  }
  return mean / factors_.size();
}

void FactorWindow::succeeded(double factor)
{
  centre(factor);
}

void FactorWindow::failed()
{
  double mean = 0.0;
  for (double factor : factors_)
  {
    mean += factor;
  }
  mean /= factors_.size();
  if (mean + step_ > highest_)
  {
    centre(initial_mean_);
    return;
  }
  std::vector<double> shifted;
  for (double factor : factors_)
  {
    if (factor + step_ <= highest_ + 1e-9)
    {
      shifted.push_back(factor + step_);
    }
  }
  factors_ = shifted;
  if (factors_.empty())
  {
    centre(initial_mean_);
  }
}
}  // namespace corridor
