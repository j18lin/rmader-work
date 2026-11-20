/* ----------------------------------------------------------------------------
 * Tests of the dense QP solver and of the corridor optimiser (no ROS needed):
 *   rosrun rmader test_corridor_planner
 * -------------------------------------------------------------------------- */

#include "corridor_planner.hpp"
#include "dense_qp.hpp"
#include "stsfc.hpp"

#include <chrono>
#include <iostream>
#include <random>

namespace
{
int failures = 0;

void check(bool condition, const std::string& what)
{
  std::cout << (condition ? "  ok    " : "  FAIL  ") << what << std::endl;
  if (!condition)
  {
    failures++;
  }
}

corridor::Polytope box(const Eigen::Vector3d& lo, const Eigen::Vector3d& hi)
{
  corridor::Polytope out;
  out.A = Eigen::MatrixXd::Zero(6, 3);
  out.b = Eigen::VectorXd::Zero(6);
  for (int axis = 0; axis < 3; ++axis)
  {
    out.A(2 * axis, axis) = 1.0;
    out.b[2 * axis] = hi[axis];
    out.A(2 * axis + 1, axis) = -1.0;
    out.b[2 * axis + 1] = -lo[axis];
  }
  return out;
}

// Karush-Kuhn-Tucker residual of a QP solution: zero exactly at the optimum.
double kktResidual(const Eigen::MatrixXd& G, const Eigen::VectorXd& g, const Eigen::MatrixXd& A,
                   const Eigen::VectorXd& b, const dqp::Result& r)
{
  const Eigen::VectorXd slack = b - A * r.x;
  double worst = (G * r.x + g + A.transpose() * r.lambda).cwiseAbs().maxCoeff();
  worst = std::max(worst, -slack.minCoeff());
  worst = std::max(worst, -r.lambda.minCoeff());
  worst = std::max(worst, (slack.array() * r.lambda.array()).abs().maxCoeff());
  return worst;
}

void testQp()
{
  std::cout << "Dense QP" << std::endl;
  std::mt19937 gen(3);
  std::normal_distribution<double> normal(0.0, 1.0);
  double worst = 0.0;
  int optimal = 0, infeasible = 0, other = 0;
  for (int trial = 0; trial < 2000; ++trial)
  {
    const int n = 1 + trial % 8;
    const int m = 1 + (trial * 7) % 40;
    Eigen::MatrixXd R(n, n);
    Eigen::VectorXd g(n);
    Eigen::MatrixXd A(m, n);
    Eigen::VectorXd b(m);
    for (int i = 0; i < n; ++i)
    {
      g[i] = normal(gen);
      for (int j = 0; j < n; ++j)
      {
        R(i, j) = normal(gen);
      }
    }
    for (int i = 0; i < m; ++i)
    {
      b[i] = normal(gen) + (trial % 3 == 0 ? -0.5 : 0.8);
      for (int j = 0; j < n; ++j)
      {
        A(i, j) = normal(gen);
      }
    }
    if (trial % 5 == 0 && m > 2)
    {
      A.row(1) = A.row(0);  // duplicated and dependent rows
      b[1] = b[0];
      A.row(2) = -2.0 * A.row(0);
    }
    const Eigen::MatrixXd G = R.transpose() * R + 0.05 * Eigen::MatrixXd::Identity(n, n);
    const dqp::Result r = dqp::solve(G, g, A, b);
    if (r.status == dqp::Status::OPTIMAL)
    {
      optimal++;
      worst = std::max(worst, kktResidual(G, g, A, b, r));
    }
    else if (r.status == dqp::Status::INFEASIBLE)
    {
      infeasible++;
      // The claim is checked with a different method: many projection sweeps
      // onto the half-spaces must not find a common point either.
      Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
      for (int sweep = 0; sweep < 3000; ++sweep)
      {
        for (int i = 0; i < m; ++i)
        {
          const double excess = A.row(i).dot(x) - b[i];
          if (excess > 0.0)
          {
            x -= excess / A.row(i).squaredNorm() * A.row(i).transpose();
          }
        }
      }
      if ((A * x - b).maxCoeff() < 1e-9)
      {
        other++;
      }
    }
    else
    {
      other++;
    }
  }
  std::cout << "    " << optimal << " optimal, " << infeasible << " infeasible, worst KKT residual " << worst
            << std::endl;
  check(optimal > 500 && infeasible > 100, "random problems cover both outcomes");
  check(worst < 1e-6, "optimal solutions satisfy the KKT conditions");
  check(other == 0, "no wrong infeasibility claim and no solver failure");
}

corridor::Problem baseProblem()
{
  corridor::Problem problem;
  problem.num_pieces = 5;
  problem.dt = 1.0;
  problem.start.pos = Eigen::Vector3d(0, 0, 1);
  problem.goal.pos = Eigen::Vector3d(4, 0, 1);
  problem.limits.v_max = Eigen::Vector3d::Constant(2.0);
  problem.limits.a_max = Eigen::Vector3d::Constant(3.0);
  problem.limits.j_max = Eigen::Vector3d::Constant(10.0);
  return problem;
}

void testFreeSpace()
{
  std::cout << "Free space" << std::endl;
  corridor::Problem problem = baseProblem();
  problem.corridors.assign(5, { box(Eigen::Vector3d(-10, -10, -10), Eigen::Vector3d(10, 10, 10)) });
  corridor::Solution solution;
  check(corridor::solve(problem, solution), "solved");
  check(corridor::worstViolation(problem, solution) < 1e-6, "all constraints hold");
  check((solution.pos(solution.duration()) - problem.goal.pos).norm() < 1e-9, "ends at the goal");
  check(solution.vel(solution.duration()).norm() < 1e-9 && solution.acc(solution.duration()).norm() < 1e-9,
        "ends at rest");
  check(std::abs(solution.pos(2.5).y()) < 1e-8 && std::abs(solution.pos(2.5).z() - 1.0) < 1e-8,
        "straight line when nothing is in the way");
  check(std::abs(solution.pos(2.5).x() - 2.0) < 1e-6, "symmetric in time");

  // Moving start: the state is reproduced.
  problem.start.vel = Eigen::Vector3d(0.5, 0.2, 0.0);
  problem.start.acc = Eigen::Vector3d(0.1, -0.3, 0.0);
  check(corridor::solve(problem, solution), "solved from a moving start");
  check(corridor::worstViolation(problem, solution) < 1e-6, "all constraints hold from a moving start");

  // Polynomial form agrees with the Bezier form.
  double worst = 0.0;
  for (int n = 0; n < 5; ++n)
  {
    const Eigen::Matrix<double, 3, 4> poly = solution.polynomial(n);
    for (double u : { 0.0, 0.3, 0.9 })
    {
      const Eigen::Vector3d p = poly.col(0) * u * u * u + poly.col(1) * u * u + poly.col(2) * u + poly.col(3);
      worst = std::max(worst, (p - solution.pos((n + u) * solution.dt)).norm());
    }
  }
  check(worst < 1e-9, "polynomial coefficients match the control points");

  // Too little time for the limits.
  problem = baseProblem();
  problem.corridors.assign(5, { box(Eigen::Vector3d(-10, -10, -10), Eigen::Vector3d(10, 10, 10)) });
  problem.dt = 0.3;
  check(!corridor::solve(problem, solution), "infeasible when the time is too short");
}

// An L-shaped passage: along x, then along y.
corridor::Layers elbow(int layers)
{
  const std::vector<corridor::Polytope> segments = {
    box(Eigen::Vector3d(-0.5, -0.5, 0.5), Eigen::Vector3d(2.5, 0.5, 1.5)),
    box(Eigen::Vector3d(1.5, -0.5, 0.5), Eigen::Vector3d(4.5, 0.5, 1.5)),
    box(Eigen::Vector3d(3.5, -0.5, 0.5), Eigen::Vector3d(4.5, 3.5, 1.5)),
  };
  return corridor::Layers(layers, segments);
}

void testElbow()
{
  std::cout << "Corridor with a turn" << std::endl;
  corridor::Problem problem = baseProblem();
  problem.goal.pos = Eigen::Vector3d(4, 3, 1);
  problem.dt = 1.6;
  problem.corridors = elbow(5);
  corridor::Solution ordered, exhaustive;
  check(corridor::solve(problem, ordered), "solved with ordered assignments");
  if (ordered.assignment.empty())
  {
    return;
  }
  check(corridor::worstViolation(problem, ordered) < 1e-6, "all constraints hold");
  bool monotone = true;
  for (size_t n = 1; n < ordered.assignment.size(); ++n)
  {
    monotone = monotone && ordered.assignment[n] >= ordered.assignment[n - 1];
  }
  check(monotone && ordered.assignment.front() == 0 && ordered.assignment.back() == 2,
        "goes through the segments in order");

  // Samples of the trajectory stay inside the union of the boxes.
  bool inside = true;
  for (double t = 0.0; t <= ordered.duration(); t += 0.02)
  {
    bool any = false;
    for (const auto& polytope : problem.corridors[0])
    {
      any = any || polytope.contains(ordered.pos(t), 1e-6);
    }
    inside = inside && any;
  }
  check(inside, "the sampled trajectory stays inside the corridor");

  problem.exhaustive = true;
  check(corridor::solve(problem, exhaustive), "solved with all assignments");
  check(exhaustive.cost <= ordered.cost + 1e-9, "all assignments are never worse than the ordered ones");
  std::cout << "    cost ordered " << ordered.cost << " (" << ordered.qps_solved << " QPs), all "
            << exhaustive.cost << " (" << exhaustive.qps_solved << " QPs)" << std::endl;
  check(std::abs(exhaustive.cost - ordered.cost) < 1e-6 * std::max(1.0, ordered.cost),
        "here both give the same optimum");

  // Brute force over all 3^5 assignments agrees with branch and bound.
  double brute = std::numeric_limits<double>::infinity();
  for (int code = 0; code < 243; ++code)
  {
    corridor::Problem single = problem;
    int rest = code;
    for (int n = 0; n < 5; ++n)
    {
      single.corridors[n] = { problem.corridors[n][rest % 3] };
      rest /= 3;
    }
    corridor::Solution candidate;
    if (corridor::solve(single, candidate))
    {
      brute = std::min(brute, candidate.cost);
    }
  }
  check(std::abs(brute - exhaustive.cost) < 1e-6 * std::max(1.0, brute),
        "branch and bound finds the optimum of the mixed-integer problem");
}

void testTimeLayers()
{
  std::cout << "Time layers" << std::endl;
  // The first half of the passage is closed during the last three layers, as
  // if an obstacle's reachable set had grown over it: the robot must be past
  // it by then.
  corridor::Problem problem = baseProblem();
  problem.goal.pos = Eigen::Vector3d(6, 0, 1);
  problem.dt = 1.2;
  const corridor::Polytope whole = box(Eigen::Vector3d(-0.5, -0.5, 0.5), Eigen::Vector3d(6.5, 0.5, 1.5));
  const corridor::Polytope far_end = box(Eigen::Vector3d(2.4, -0.5, 0.5), Eigen::Vector3d(6.5, 0.5, 1.5));
  problem.corridors.assign(5, { whole, far_end });
  corridor::Solution free_run;
  check(corridor::solve(problem, free_run), "solved without the closure");
  for (int n = 2; n < 5; ++n)
  {
    problem.corridors[n] = { corridor::Polytope(), far_end };
  }
  corridor::Solution solution;
  check(corridor::solve(problem, solution), "solved with the closure");
  if (solution.assignment.empty())
  {
    return;
  }
  check(corridor::worstViolation(problem, solution) < 1e-6, "all constraints hold");
  check(solution.pos(2 * problem.dt).x() >= 2.4 - 1e-6, "past the closed part before it closes");
  check(free_run.pos(2 * problem.dt).x() < 2.4, "which the unconstrained solution was not");
  check(solution.cost > free_run.cost, "and it costs more");

  problem.limits.v_max = Eigen::Vector3d::Constant(1.0);
  check(!corridor::solve(problem, solution), "infeasible when the robot is too slow to get past in time");
}

void testHeadingCone()
{
  std::cout << "Heading cone" << std::endl;
  corridor::Problem problem = baseProblem();
  problem.goal.pos = Eigen::Vector3d(3, 3, 1);
  problem.dt = 1.5;
  problem.corridors.assign(5, { box(Eigen::Vector3d(-10, -10, -10), Eigen::Vector3d(10, 10, 10)) });
  corridor::Solution plain, coned;
  check(corridor::solve(problem, plain), "solved without the cone");
  problem.heading_cone = true;
  problem.heading = 0.0;  // facing +x, goal at 45 degrees
  problem.heading_cone_tan = std::tan(0.2);
  check(corridor::solve(problem, coned), "solved with the cone");
  bool in_cone = true;
  for (double t = 0.01; t < problem.dt; t += 0.01)
  {
    const Eigen::Vector3d v = coned.vel(t);
    in_cone = in_cone && v.x() >= -1e-6 && std::abs(v.y()) <= problem.heading_cone_tan * v.x() + 1e-6;
  }
  check(in_cone, "the first piece leaves within the cone");
  const Eigen::Vector3d v_plain = plain.vel(0.5 * problem.dt);
  check(std::abs(v_plain.y()) > problem.heading_cone_tan * v_plain.x(), "which the plain solution did not");
}

void testTimeAllocation()
{
  std::cout << "Time allocation" << std::endl;
  corridor::State start, goal;
  goal.pos = Eigen::Vector3d(10, 0, 0);
  corridor::Limits limits;
  limits.v_max = Eigen::Vector3d::Constant(2.0);
  limits.a_max = Eigen::Vector3d::Constant(100.0);
  limits.j_max = Eigen::Vector3d::Constant(1000.0);
  check(std::abs(corridor::initialDt(start, goal, limits, 5) - 1.0) < 1e-9, "velocity-limited: 10 m at 2 m/s in 5 pieces");
  limits.v_max = Eigen::Vector3d::Constant(100.0);
  limits.a_max = Eigen::Vector3d::Constant(5.0);
  check(std::abs(corridor::initialDt(start, goal, limits, 5) - 2.0 / 5.0) < 1e-9,
        "acceleration-limited: sqrt(2 * 10 / 5) s");
  limits.a_max = Eigen::Vector3d::Constant(1000.0);
  limits.j_max = Eigen::Vector3d::Constant(60.0);
  check(std::abs(corridor::initialDt(start, goal, limits, 5) - 1.0 / 5.0) < 1e-9, "jerk-limited: cbrt(6 * 10 / 60) s");

  corridor::FactorWindow window(1.5, 0.4, 0.1, 1.0, 2.5);
  check(window.factors().size() == 9 && std::abs(window.factors().front() - 1.1) < 1e-9 &&
            std::abs(window.factors().back() - 1.9) < 1e-9,
        "initial window 1.1 .. 1.9");
  window.succeeded(1.1);
  check(std::abs(window.factors().front() - 1.0) < 1e-9 && std::abs(window.factors().back() - 1.5) < 1e-9,
        "after a success at 1.1: 1.0 .. 1.5 (clipped below)");
  window.failed();
  check(std::abs(window.factors().front() - 1.1) < 1e-9 && std::abs(window.factors().back() - 1.6) < 1e-9,
        "after a failure the window moves up by one step");
  for (int i = 0; i < 30; ++i)
  {
    window.failed();
  }
  check(!window.factors().empty() && window.factors().back() <= 2.5 + 1e-9, "the window never leaves the range");

  // A search over the factors on a real problem.
  corridor::Problem problem = baseProblem();
  problem.goal.pos = Eigen::Vector3d(4, 3, 1);
  problem.corridors = elbow(5);
  const double dt0 = corridor::initialDt(problem.start, problem.goal, problem.limits, 5);
  double found = 0.0;
  int tried = 0;
  auto t0 = std::chrono::steady_clock::now();
  for (double factor = 1.0; factor <= 4.0 && found == 0.0; factor += 0.1)
  {
    problem.dt = factor * dt0;
    corridor::Solution solution;
    tried++;
    if (corridor::solve(problem, solution))
    {
      found = factor;
    }
  }
  const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  std::cout << "    first feasible factor " << found << " after " << tried << " factors, " << ms << " ms"
            << std::endl;
  check(found > 1.0, "the first guess is too short and a larger factor works");
}

// Distance from a point to a polytope's interior: > 0 when the point is outside.
double outsideBy(const corridor::Polytope& polytope, const Eigen::Vector3d& q)
{
  return (polytope.A * q - polytope.b).maxCoeff();
}

void testCorridors()
{
  std::cout << "Corridor generation" << std::endl;
  // A 1 m box right of a straight path, growing layer after layer as a reachable set does.
  const std::vector<Eigen::Vector3d> path = { { 0, 0, 1 }, { 2, 0, 1 }, { 4, 0, 1 }, { 6, 0, 1 } };
  const Eigen::Vector3d centre(3.0, 1.6, 1.0);
  std::vector<corridor::LayerPoints> layers(5);
  for (int n = 0; n < 5; ++n)
  {
    const Eigen::Vector3d half = Eigen::Vector3d::Constant(0.5 + 0.2 * n);
    corridor::sampleBox(centre - half, centre + half, 0.1, layers[n].inflated);
  }
  corridor::DecompOptions options;
  options.local_bbox = Eigen::Vector3d(2.0, 2.0, 1.0);

  for (bool planar : { false, true })
  {
    options.planar = planar;
    options.z_min = 0.5;
    options.z_max = 1.5;
    const corridor::Layers corridors = corridor::buildCorridors(path, layers, options);
    bool shape = corridors.size() == 5;
    bool contains = true, excludes = true, shrinks = true;
    for (int n = 0; n < 5 && shape; ++n)
    {
      shape = shape && corridors[n].size() == 3;
      for (int p = 0; p < 3 && shape; ++p)
      {
        const corridor::Polytope& polytope = corridors[n][p];
        if (polytope.empty())
        {
          contains = false;
          continue;
        }
        for (double s : { 0.0, 0.5, 1.0 })
        {
          contains = contains && polytope.contains(path[p] + s * (path[p + 1] - path[p]), 1e-6);
        }
        for (const Eigen::Vector3d& q : layers[n].inflated)
        {
          excludes = excludes && outsideBy(polytope, q) > -1e-6;
        }
      }
      // The polytope next to the box gives way as the box grows.
      if (n > 0 && shape && !corridors[n][1].empty() && !corridors[n - 1][1].empty())
      {
        const Eigen::Vector3d probe(3.0, 0.9 - 0.2 * (n - 1), 1.0);  // just below the previous layer's box
        shrinks = shrinks && (!corridors[n][1].contains(probe) || outsideBy(corridors[n - 1][1], probe) < 0);
      }
    }
    const std::string kind = planar ? " (planar)" : " (3D)";
    check(shape, "one polytope per layer and segment" + kind);
    check(contains, "every polytope contains its segment" + kind);
    check(excludes, "no polytope contains an obstacle point of its layer" + kind);
    check(shrinks, "later layers give way to the grown obstacle" + kind);
    check(corridors[0][1].contains(Eigen::Vector3d(3.0, 0.9, 1.0)) &&
              !corridors[4][1].contains(Eigen::Vector3d(3.0, 0.9, 1.0)),
          "a point free in the first layer is excluded in the last" + kind);
  }

  // Raw points (map voxels) keep the robot's clearance; inflated points stay excluded.
  options.planar = false;
  options.shrink = 0.3;
  std::vector<corridor::LayerPoints> mixed(1);
  corridor::sampleBox(centre - Eigen::Vector3d::Constant(0.5), centre + Eigen::Vector3d::Constant(0.5), 0.1,
                      mixed[0].inflated);
  for (double x = 0.0; x <= 6.0; x += 0.1)
  {
    for (double z = 0.5; z <= 1.5; z += 0.1)
    {
      mixed[0].raw.emplace_back(x, -1.0, z);  // a wall on the other side
    }
  }
  const corridor::Layers with_wall = corridor::buildCorridors(path, mixed, options);
  double nearest_raw = 1e9, nearest_inflated = 1e9;
  bool valid = true;
  for (const corridor::Polytope& polytope : with_wall[0])
  {
    valid = valid && !polytope.empty();
    if (polytope.empty())
    {
      continue;
    }
    for (const Eigen::Vector3d& q : mixed[0].raw)
    {
      nearest_raw = std::min(nearest_raw, outsideBy(polytope, q));
    }
    for (const Eigen::Vector3d& q : mixed[0].inflated)
    {
      nearest_inflated = std::min(nearest_inflated, outsideBy(polytope, q));
    }
  }
  std::cout << "    clearance from raw points " << nearest_raw << " m, from inflated points " << nearest_inflated
            << " m" << std::endl;
  check(valid, "polytopes exist next to a wall");
  check(nearest_raw >= 0.3 - 1e-6, "raw points are kept at the robot's clearance");
  check(nearest_inflated >= -1e-6, "inflated points are still excluded after the faces move in");

  // A segment that runs through an obstacle has no polytope.
  std::vector<corridor::LayerPoints> blocked(1);
  corridor::sampleBox(Eigen::Vector3d(2.5, -0.5, 0.5), Eigen::Vector3d(3.5, 0.5, 1.5), 0.1, blocked[0].inflated);
  options.shrink = 0.0;
  const corridor::Layers blocked_out = corridor::buildCorridors(path, blocked, options);
  check(blocked_out[0][1].empty() && !blocked_out[0][0].empty(), "a blocked segment gets no polytope");

  // Path shaping.
  const std::vector<Eigen::Vector3d> zigzag = { { 0, 0, 0 }, { 1, 0.1, 0 }, { 2, 0, 0 }, { 3, 0.1, 0 },
                                                { 4, 0, 0 },   { 4, 3, 0 } };
  auto clear_of_corner = [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    // Keep out of the square x < 3.5, y > 0.5.
    for (double s = 0.0; s <= 1.0; s += 0.02)
    {
      const Eigen::Vector3d q = a + s * (b - a);
      if (q.x() < 3.5 && q.y() > 0.5)
      {
        return false;
      }
    }
    return true;
  };
  const auto shaped = corridor::shapePath(zigzag, 3, 0.2, clear_of_corner);
  check(shaped.size() == 4 && (shaped[0] - zigzag[0]).norm() < 1e-9 && (shaped.back() - zigzag.back()).norm() < 1e-9,
        "a zigzag with one real corner becomes three segments from start to end");
  bool legal = true;
  for (size_t i = 0; i + 1 < shaped.size(); ++i)
  {
    legal = legal && clear_of_corner(shaped[i], shaped[i + 1]);
  }
  check(legal, "the shortcuts stay legal");
  const auto straight = corridor::shapePath({ { 0, 0, 0 }, { 6, 0, 0 } }, 3, 0.2, nullptr);
  check(straight.size() == 4 && std::abs((straight[1] - straight[0]).norm() - 2.0) < 1e-9 &&
            std::abs((straight[3] - straight[2]).norm() - 2.0) < 1e-9,
        "a straight line is cut into three equal segments");
  const auto many = corridor::shapePath({ { 0, 0, 0 }, { 1, 0, 0 }, { 1, 1, 0 }, { 2, 1, 0 }, { 2, 2, 0 } }, 3, 0.2,
                                        [](const Eigen::Vector3d&, const Eigen::Vector3d&) { return false; });
  check(many.size() == 4 && (many.back() - Eigen::Vector3d(2, 1, 0)).norm() < 1e-9,
        "more corners than segments: the first three segments are kept");
}

void testAroundAnObstacle()
{
  std::cout << "Around an obstacle, end to end" << std::endl;
  // A box sits on the straight line; the path goes around it; the box grows with time.
  const Eigen::Vector3d centre(2.5, 0.0, 1.0);
  const std::vector<Eigen::Vector3d> path = { { 0, 0, 1 }, { 1.2, 1.7, 1 }, { 3.8, 1.7, 1 }, { 5, 0, 1 } };
  corridor::Problem problem = baseProblem();
  problem.goal.pos = path.back();
  const double dt0 = corridor::initialDt(problem.start, problem.goal, problem.limits, 5);
  corridor::DecompOptions options;
  options.local_bbox = Eigen::Vector3d(2.0, 2.0, 1.0);
  double found = 0.0;
  corridor::Solution solution;
  std::vector<Eigen::Vector3d> half_by_layer;
  auto t0 = std::chrono::steady_clock::now();
  for (double factor = 1.0; factor <= 4.0 && found == 0.0; factor += 0.1)
  {
    problem.dt = factor * dt0;
    std::vector<corridor::LayerPoints> layers(5);
    half_by_layer.clear();
    for (int n = 0; n < 5; ++n)
    {
      const Eigen::Vector3d half = Eigen::Vector3d::Constant(0.6 + 0.1 * (n + 1) * problem.dt);  // 0.1 m/s
      half_by_layer.push_back(half);
      corridor::sampleBox(centre - half, centre + half, 0.1, layers[n].inflated);
    }
    problem.corridors = corridor::buildCorridors(path, layers, options);
    if (corridor::solve(problem, solution))
    {
      found = factor;
    }
  }
  const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  std::cout << "    factor " << found << ", " << ms << " ms in total, assignment ";
  for (int p : solution.assignment)
  {
    std::cout << p;
  }
  std::cout << std::endl;
  check(found > 0.0, "a trajectory is found");
  if (found == 0.0)
  {
    return;
  }
  bool clear = true;
  for (double t = 0.0; t <= solution.duration(); t += 0.01)
  {
    const int n = std::min(4, static_cast<int>(t / solution.dt));
    const Eigen::Vector3d offset = (solution.pos(t) - centre).cwiseAbs() - half_by_layer[n];
    clear = clear && offset.maxCoeff() > -1e-6;
  }
  check(clear, "the trajectory is outside the obstacle's box of every layer it passes through");
  check(corridor::worstViolation(problem, solution) < 1e-6, "all constraints hold");
}

void testSpeed()
{
  std::cout << "Speed" << std::endl;
  corridor::Problem problem = baseProblem();
  problem.goal.pos = Eigen::Vector3d(4, 3, 1);
  problem.dt = 1.6;
  problem.corridors = elbow(5);
  for (bool exhaustive : { false, true })
  {
    problem.exhaustive = exhaustive;
    corridor::Solution solution;
    auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < 50; ++i)
    {
      corridor::solve(problem, solution);
    }
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() / 50;
    std::cout << "    " << (exhaustive ? "all assignments: " : "ordered assignments: ") << ms << " ms per solve, "
              << solution.qps_solved << " QPs" << std::endl;
    check(ms < 50.0, exhaustive ? "all assignments in under 50 ms" : "ordered assignments in under 50 ms");
  }
}
}  // namespace

int main()
{
  testQp();
  testFreeSpace();
  testElbow();
  testTimeLayers();
  testHeadingCone();
  testTimeAllocation();
  testCorridors();
  testAroundAnObstacle();
  testSpeed();
  std::cout << (failures == 0 ? "ALL PASSED" : "FAILED: " + std::to_string(failures)) << std::endl;
  return failures == 0 ? 0 : 1;
}
