/* ----------------------------------------------------------------------------
 * Unit test for the observation-based dynamic obstacle module (no ROS needed).
 * Exits non-zero on the first failed check.
 *
 *   rosrun rmader test_dyn_obstacles
 * -------------------------------------------------------------------------- */

#include <cmath>
#include <iostream>
#include <random>

#include "dyn_obstacles.hpp"

namespace
{
int failures = 0;

void check(bool condition, const std::string& what)
{
  if (!condition)
  {
    failures++;
    std::cout << "FAILED: " << what << std::endl;
  }
}

dynobs::Observation observation(int id, double stamp, const Eigen::Vector3d& pos)
{
  dynobs::Observation obs;
  obs.id = id;
  obs.stamp = stamp;
  obs.pos = pos;
  obs.bbox = Eigen::Vector3d(0.4, 0.4, 0.4);
  return obs;
}

bool inside(const dynobs::Box& inner, const dynobs::Box& outer)
{
  return ((inner.lo.array() >= outer.lo.array() - 1e-9) && (inner.hi.array() <= outer.hi.array() + 1e-9)).all();
}

void testTracker()
{
  dynobs::Params par;
  par.velocity_filter_alpha = 1.0;
  par.track_timeout = 2.0;
  dynobs::Tracker tracker(par);

  check(tracker.update(observation(7, 10.0, Eigen::Vector3d(0, 0, 1))), "first observation is accepted");
  dynobs::Track track;
  check(tracker.get(7, track) && !track.is_dynamic, "a single observation has no velocity yet");

  check(tracker.update(observation(7, 10.5, Eigen::Vector3d(0.2, 0, 1))), "second observation is accepted");
  check(tracker.get(7, track), "track exists");
  check((track.vel - Eigen::Vector3d(0.4, 0, 0)).norm() < 1e-9, "velocity comes from finite differences");
  check(track.is_dynamic, "a moving track is dynamic");
  check(std::fabs(track.max_vel - par.obst_max_vel) < 1e-12, "speed bound is the configured one");

  check(!tracker.update(observation(7, 10.2, Eigen::Vector3d(5, 5, 5))), "out-of-order observation is rejected");
  check(tracker.get(7, track) && std::fabs(track.stamp - 10.5) < 1e-12, "rejected observation changes nothing");

  // Faster than the configured bound: the bound used for this track must follow the measurement.
  check(tracker.update(observation(7, 11.0, Eigen::Vector3d(1.2, 0, 1))), "third observation is accepted");
  check(tracker.get(7, track) && std::fabs(track.max_vel - 2.0) < 1e-9, "speed bound follows a faster obstacle");

  // An observation that carries its own velocity is used as is.
  dynobs::Observation with_vel = observation(8, 11.0, Eigen::Vector3d(3, 3, 1));
  with_vel.has_vel = true;
  with_vel.vel = Eigen::Vector3d(0, -0.3, 0);
  tracker.update(with_vel);
  check(tracker.get(8, track) && (track.vel - with_vel.vel).norm() < 1e-12, "reported velocity is used");

  check(tracker.prune(12.5).empty(), "fresh tracks survive pruning");
  const std::vector<int> removed = tracker.prune(13.5);
  check(removed.size() == 2 && tracker.tracks().empty(), "stale tracks are dropped");
}

// Differencing consecutive 20 Hz observations with 3 cm of noise would give velocity errors of the order of
// the obstacle's own speed. The windowed estimate has to stay close to the truth.
void testVelocityEstimateWithNoise()
{
  std::mt19937 rng(3);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);

  dynobs::Params par;  // defaults: 0.5 s window, alpha 0.6
  dynobs::Tracker tracker(par);
  const Eigen::Vector3d velocity(0.3, -0.2, 0.0);
  double worst = 0.0;
  for (int k = 0; k < 400; k++)
  {
    const double t = 50.0 + 0.05 * k;
    Eigen::Vector3d noise(unit(rng), unit(rng), unit(rng));
    noise *= 0.03 / std::sqrt(3.0);
    tracker.update(observation(1, t, velocity * (t - 50.0) + noise));
    dynobs::Track track;
    tracker.get(1, track);
    if (k >= 20)  // after one second of observations
    {
      worst = std::max(worst, (track.vel - velocity).norm());
    }
    check(track.history.size() <= 12, "history stays bounded by the velocity window");
  }
  check(worst < 0.1, "velocity estimate is robust to observation noise");
}

// The property the planner relies on: an obstacle that respects the speed bound never leaves the
// reachable box, whatever it does after it was observed.
void testReachableBoxContainsAnyBoundedMotion()
{
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  std::uniform_real_distribution<double> zero_one(0.0, 1.0);

  dynobs::Params par;
  par.mode = dynobs::Mode::REACHABLE;
  par.obst_max_vel = 0.8;
  par.obst_position_error = 0.05;

  int checked = 0;
  for (int trial = 0; trial < 2000; trial++)
  {
    dynobs::Tracker tracker(par);
    Eigen::Vector3d truth(unit(rng) * 5, unit(rng) * 5, unit(rng) * 2);
    double t = 100.0 + zero_one(rng);

    // Two noisy observations so that the tracker also has a velocity estimate.
    for (int k = 0; k < 2; k++)
    {
      Eigen::Vector3d noise(unit(rng), unit(rng), unit(rng));
      noise *= par.obst_position_error;  // per-axis error <= epsilon (Assumption 2)
      tracker.update(observation(1, t, truth + noise));
      if (k == 0)
      {
        Eigen::Vector3d v(unit(rng), unit(rng), unit(rng));
        v *= par.obst_max_vel;
        truth += v * 0.1;
        t += 0.1;
      }
    }
    dynobs::Track track;
    tracker.get(1, track);
    // Only the configured bound is being tested here; noisy differences may have raised the track's own.
    track.max_vel = par.obst_max_vel;
    const double t_observed = t;

    // Adversarial motion under the paper's Assumption 1 (each velocity component bounded by v_max): every
    // 50 ms, either full speed along every axis with random signs, or a random velocity inside the bound.
    const double dt = 0.05;
    for (int step = 0; step < 80; step++)
    {
      Eigen::Vector3d v(unit(rng), unit(rng), unit(rng));
      if (zero_one(rng) < 0.5)
      {
        v = v.array().sign().matrix();
      }
      truth += v * par.obst_max_vel * dt;
      t += dt;

      const double t_a = t_observed + zero_one(rng) * (t - t_observed);
      const dynobs::Box box =
          dynobs::centerBox(track, t_a, t, par, Eigen::Vector3d::Constant(par.obst_position_error));
      check(box.contains(truth, 1e-9), "bounded motion stays inside the reachable box");
      checked++;
    }
  }
  check(checked == 2000 * 80, "reachable-set property was exercised");
}

void testBoxes()
{
  dynobs::Params par;
  par.velocity_filter_alpha = 1.0;
  par.obst_max_vel = 1.0;
  par.prediction_horizon = 1.0;
  par.prediction_growth = 0.2;

  dynobs::Tracker tracker(par);
  tracker.update(observation(1, 0.0, Eigen::Vector3d(0, 0, 0)));
  tracker.update(observation(1, 1.0, Eigen::Vector3d(0.5, 0, 0)));  // 0.5 m/s along x
  dynobs::Track track;
  tracker.get(1, track);

  par.mode = dynobs::Mode::REACHABLE;
  const dynobs::Box early = dynobs::centerBox(track, 1.0, 1.5, par);
  const dynobs::Box late = dynobs::centerBox(track, 1.5, 3.0, par);
  check(std::fabs(early.hi.x() - 1.0) < 1e-9 && std::fabs(early.lo.x() - 0.0) < 1e-9,
        "reachable box grows by max_vel * elapsed");
  check(inside(early, late), "reachable boxes only grow with time");
  check(inside(dynobs::centerBox(track, 0.2, 0.9, par), dynobs::centerBox(track, 1.0, 1.0, par)),
        "times before the observation are clamped to it");

  par.mode = dynobs::Mode::PREDICTED;
  for (double t_a = 1.0; t_a < 4.0; t_a += 0.37)
  {
    for (double span = 0.0; span < 2.0; span += 0.21)
    {
      dynobs::Params worst_par = par;
      worst_par.mode = dynobs::Mode::REACHABLE;
      const dynobs::Box predicted = dynobs::centerBox(track, t_a, t_a + span, par);
      check(inside(predicted, dynobs::centerBox(track, t_a, t_a + span, worst_par)),
            "predicted box never exceeds the reachable box");
      // An obstacle that really keeps its velocity for the prediction horizon is inside the predicted box.
      for (double t = t_a; t <= t_a + span + 1e-9; t += 0.05)
      {
        const double tau = std::min(t - 1.0, par.prediction_horizon);
        check(predicted.contains(track.pos + track.vel * tau, 1e-9), "predicted box contains the prediction");
      }
    }
  }

  // The predicted box is strictly tighter than the worst case for a horizon that matters.
  dynobs::Params worst_par = par;
  worst_par.mode = dynobs::Mode::REACHABLE;
  const dynobs::Box predicted = dynobs::centerBox(track, 1.5, 2.0, par);
  const dynobs::Box worst = dynobs::centerBox(track, 1.5, 2.0, worst_par);
  check((predicted.hi - predicted.lo).prod() < 0.5 * (worst.hi - worst.lo).prod(), "prediction is less conservative");

  // Occupancy adds the body, the observation error, the base inflation and the caller's margin.
  par.obst_position_error = 0.05;
  par.base_inflation = 0.1;
  const dynobs::Box center = dynobs::centerBox(track, 1.0, 1.0, par);
  const dynobs::Box occupancy = dynobs::occupancyBox(track, 1.0, 1.0, par, Eigen::Vector3d(0.3, 0.3, 0.3));
  check(((occupancy.hi - center.hi) - Eigen::Vector3d::Constant(0.2 + 0.05 + 0.1 + 0.3)).norm() < 1e-9,
        "occupancy margin is body + error + inflation + extra");
  check(occupancy.vertexes().size() == 8, "a box has eight vertexes");

  // A track that is not moving does not grow in predicted mode, but does in reachable mode.
  dynobs::Tracker still(par);
  still.update(observation(2, 0.0, Eigen::Vector3d(2, 2, 0)));
  still.update(observation(2, 1.0, Eigen::Vector3d(2, 2, 0)));
  dynobs::Track parked;
  still.get(2, parked);
  check(!parked.is_dynamic, "a track that does not move is static");
  const dynobs::Box parked_predicted = dynobs::centerBox(parked, 1.0, 3.0, par);
  check((parked_predicted.hi - parked_predicted.lo).norm() < 1e-9, "static track does not grow when predicted");
  const dynobs::Box parked_worst = dynobs::centerBox(parked, 1.0, 3.0, worst_par);
  check(std::fabs(parked_worst.hi.x() - parked_worst.lo.x() - 4.0) < 1e-9, "reachable box grows even for a parked track");
}

// Section V-B of the paper, with its Table II values (the defaults).
void testHeat()
{
  dynobs::Params par;
  par.velocity_filter_alpha = 1.0;
  par.prediction_horizon = 1.0;
  dynobs::Tracker tracker(par);
  tracker.update(observation(1, 0.0, Eigen::Vector3d(0, 0, 0)));
  tracker.update(observation(1, 1.0, Eigen::Vector3d(0.5, 0, 0)));  // at (0.5, 0, 0), heading +x at 0.5 m/s
  const Eigen::Vector3d here(0.5, 0, 0);

  // At the obstacle: base = alpha_0 and the j = 0 tube sample contributes alpha_1 * 1.
  check(std::fabs(dynobs::heat(here, tracker.tracks(), par) - 3.0) < 1e-9, "heat at the obstacle is alpha_0 + alpha_1");

  // Base term alone, by hand: R_0 = 0.2 + 0.1, R^d = R_0 + 0.5 * 1 = 0.8; 0.6 m behind the obstacle the
  // tube contributes nothing (every sample is further than its radius), so H = (1 - 0.6 / 0.8)^2.
  const double behind = dynobs::heat(here + Eigen::Vector3d(-0.6, 0, 0), tracker.tracks(), par);
  check(std::fabs(behind - 0.0625) < 1e-9, "base heat matches the formula");

  // Same distance from the obstacle: on its predicted path is hotter than beside it or behind it.
  const double ahead = dynobs::heat(here + Eigen::Vector3d(0.6, 0, 0), tracker.tracks(), par);
  const double beside = dynobs::heat(here + Eigen::Vector3d(0, 0.6, 0), tracker.tracks(), par);
  check(ahead > beside && beside > behind, "heat is highest on the predicted path");

  check(dynobs::heat(Eigen::Vector3d(10, 10, 0), tracker.tracks(), par) == 0.0, "no heat far from every obstacle");

  // Max aggregation: a second obstacle in the same place does not add up.
  dynobs::Tracker two(par);
  for (int id = 1; id <= 2; id++)
  {
    two.update(observation(id, 0.0, Eigen::Vector3d(0, 0, 0)));
    two.update(observation(id, 1.0, Eigen::Vector3d(0.5, 0, 0)));
  }
  check(std::fabs(dynobs::heat(here, two.tracks(), par) - 3.0) < 1e-9, "obstacles combine by maximum, not sum");

  dynobs::Params capped = par;
  capped.heat_max = 1.5;
  check(std::fabs(dynobs::heat(here, tracker.tracks(), capped) - 1.5) < 1e-12, "heat saturates at H_max");

  // An obstacle that is not moving keeps its base heat; its tube collapses onto it.
  dynobs::Tracker still(par);
  still.update(observation(2, 0.0, Eigen::Vector3d(0, 0, 0)));
  still.update(observation(2, 1.0, Eigen::Vector3d(0, 0, 0)));
  check(dynobs::heat(Eigen::Vector3d(0.6, 0, 0), still.tracks(), par) > 0.0, "a parked obstacle still has heat");
}

void testHoverEvasion()
{
  dynobs::Params par;
  par.velocity_filter_alpha = 1.0;
  par.hover_trigger_dist = 2.0;
  par.hover_evasion_dist = 3.0;
  par.prediction_horizon = 2.0;

  // Obstacle 3 m west of the agent, heading straight at it at 0.5 m/s.
  dynobs::Tracker tracker(par);
  tracker.update(observation(1, 0.0, Eigen::Vector3d(-3.5, 0.05, 1.0)));
  tracker.update(observation(1, 1.0, Eigen::Vector3d(-3.0, 0.05, 1.0)));

  const Eigen::Vector3d agent(0, 0, 1.5);
  Eigen::Vector3d goal;
  check(dynobs::hoverEvasionGoal(agent, 1.0, tracker.tracks(), par, goal), "an approaching obstacle triggers evasion");
  check(std::fabs((goal - agent).norm() - par.hover_evasion_dist) < 1e-9, "evasion goal is hover_evasion_dist away");
  check(std::fabs(goal.z() - agent.z()) < 1e-9, "2D evasion keeps the height");
  check(std::fabs(goal.y()) > std::fabs(goal.x()), "evasion steps out of the obstacle's path");

  check(!dynobs::hoverEvasionGoal(Eigen::Vector3d(0, 8, 1.5), 1.0, tracker.tracks(), par, goal),
        "an obstacle that passes far away does not trigger evasion");

  dynobs::Tracker still(par);
  still.update(observation(2, 0.0, Eigen::Vector3d(0.5, 0, 1.5)));
  still.update(observation(2, 1.0, Eigen::Vector3d(0.5, 0, 1.5)));
  check(!dynobs::hoverEvasionGoal(agent, 1.0, still.tracks(), par, goal), "static obstacles never trigger evasion");
}

void testModeNames()
{
  dynobs::Mode mode = dynobs::Mode::PREDICTED;
  check(dynobs::modeFromString("reachable", mode) && mode == dynobs::Mode::REACHABLE, "reachable parses");
  check(dynobs::modeFromString("predicted", mode) && mode == dynobs::Mode::PREDICTED, "predicted parses");
  check(!dynobs::modeFromString("optimistic", mode), "unknown mode is rejected");
}
}  // namespace

int main()
{
  testTracker();
  testVelocityEstimateWithNoise();
  testReachableBoxContainsAnyBoundedMotion();
  testBoxes();
  testHeat();
  testHoverEvasion();
  testModeNames();

  if (failures > 0)
  {
    std::cout << failures << " check(s) failed" << std::endl;
    return 1;
  }
  std::cout << "test_dyn_obstacles: all checks passed" << std::endl;
  return 0;
}
