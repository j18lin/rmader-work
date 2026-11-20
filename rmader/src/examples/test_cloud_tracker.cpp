/* ----------------------------------------------------------------------------
 * Tests of the point-cloud obstacle tracker (no ROS needed):
 *   rosrun rmader test_cloud_tracker
 * -------------------------------------------------------------------------- */

#include "cloud_tracker.hpp"

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

// Points on the faces of a box that a sensor at `sensor` can see, with noise.
void seenFaces(const Eigen::Vector3d& center, const Eigen::Vector3d& size, const Eigen::Vector3d& sensor,
               std::mt19937& gen, double noise, std::vector<Eigen::Vector3d>& out, double spacing = 0.07)
{
  std::uniform_real_distribution<double> jitter(-noise, noise);
  for (int axis = 0; axis < 3; ++axis)
  {
    const double side = sensor[axis] > center[axis] ? 1.0 : -1.0;
    const int u = (axis + 1) % 3, v = (axis + 2) % 3;
    for (double a = -0.5 * size[u]; a <= 0.5 * size[u] + 1e-9; a += spacing)
    {
      for (double b = -0.5 * size[v]; b <= 0.5 * size[v] + 1e-9; b += spacing)
      {
        Eigen::Vector3d p = center;
        p[axis] += side * 0.5 * size[axis];
        p[u] += a;
        p[v] += b;
        out.push_back(p + Eigen::Vector3d(jitter(gen), jitter(gen), jitter(gen)));
      }
    }
  }
}

void testClustering()
{
  std::cout << "Clustering" << std::endl;
  std::mt19937 gen(1);
  const Eigen::Vector3d sensor(0, 0, 1);
  std::vector<Eigen::Vector3d> cloud;
  seenFaces(Eigen::Vector3d(3, 1, 1), Eigen::Vector3d(0.6, 0.6, 0.6), sensor, gen, 0.01, cloud);
  seenFaces(Eigen::Vector3d(4, -2, 1), Eigen::Vector3d(0.5, 0.8, 1.0), sensor, gen, 0.01, cloud);
  for (double y = -6.0; y <= 6.0; y += 0.07)  // a wall, 12 m long
  {
    for (double z = 0.0; z <= 2.0; z += 0.07)
    {
      cloud.emplace_back(8.0, y, z);
    }
  }
  cloud.emplace_back(-3.0, 0.0, 1.0);  // a stray point

  ctrack::Params par;
  const std::vector<Eigen::Vector3d> thinned = ctrack::downsample(cloud, par.leaf_size);
  check(thinned.size() < cloud.size() / 3, "the voxel grid thins the cloud");
  const std::vector<ctrack::Cluster> clusters = ctrack::extractClusters(thinned, par);
  check(clusters.size() == 2, "two boxes found; the wall and the stray point are dropped");
  bool placed = clusters.size() == 2;
  for (const ctrack::Cluster& cluster : clusters)
  {
    const bool first = (cluster.center - Eigen::Vector3d(3, 1, 1)).norm() < 0.35;
    const bool second = (cluster.center - Eigen::Vector3d(4, -2, 1)).norm() < 0.45;
    placed = placed && (first || second);
  }
  check(placed, "their centres are within half a box of the truth (only the near faces are seen)");
}

void testFilter()
{
  std::cout << "Adaptive Kalman filter" << std::endl;
  ctrack::Params par;
  std::mt19937 gen(2);
  std::normal_distribution<double> noise(0.0, 0.03);

  // Constant velocity, 10 Hz, 3 cm of measurement noise.
  ctrack::Track track;
  const Eigen::Vector3d velocity(0.5, -0.2, 0.0);
  track.x.head<3>() = Eigen::Vector3d(1, 2, 1);
  track.updates = 1;
  double worst_late = 0.0;
  for (int k = 1; k <= 80; ++k)
  {
    const double t = 0.1 * k;
    const Eigen::Vector3d truth = Eigen::Vector3d(1, 2, 1) + velocity * t;
    ctrack::predict(track, 0.1);
    ctrack::update(track, truth + Eigen::Vector3d(noise(gen), noise(gen), noise(gen)), Eigen::Vector3d::Ones(), t, par);
    if (k > 40)
    {
      worst_late = std::max(worst_late, (track.vel() - velocity).norm());
    }
  }
  std::cout << "    velocity error after 4 s: at most " << worst_late << " m/s" << std::endl;
  check(worst_late < 0.15, "velocity of a steadily moving obstacle is recovered from positions alone");
  check((track.pos() - (Eigen::Vector3d(1, 2, 1) + velocity * 8.0)).norm() < 0.08, "and its position follows");

  // A turn: the adaptive filter follows a change of velocity faster than the plain one with the same start.
  for (bool adaptive : { true, false })
  {
    par.adaptive = adaptive;
    ctrack::Track turning;
    turning.updates = 1;
    Eigen::Vector3d p(0, 0, 0);
    double error = 0.0;
    std::mt19937 gen2(3);
    for (int k = 1; k <= 120; ++k)
    {
      const Eigen::Vector3d v = k <= 60 ? Eigen::Vector3d(0.5, 0, 0) : Eigen::Vector3d(0, 0.5, 0);
      p += 0.1 * v;
      ctrack::predict(turning, 0.1);
      ctrack::update(turning, p + Eigen::Vector3d(noise(gen2), noise(gen2), noise(gen2)), Eigen::Vector3d::Ones(),
                     0.1 * k, par);
      if (k > 60 && k <= 80)
      {
        error += (turning.pos() - p).norm() / 20.0;
      }
    }
    std::cout << "    mean position error in the 2 s after a turn, " << (adaptive ? "adaptive: " : "plain: ") << error
              << " m" << std::endl;
    check(error < 0.15, adaptive ? "the adaptive filter follows a turn" : "the plain filter follows a turn");
  }
}

void testTracking()
{
  std::cout << "Tracking over time" << std::endl;
  ctrack::Params par;
  ctrack::CloudTracker tracker(par);
  std::mt19937 gen(4);
  const Eigen::Vector3d sensor(0, 0, 1);
  const Eigen::Vector3d size(0.6, 0.6, 0.6);
  int first_id = -1, second_id = -1;
  bool ids_stable = true;
  for (int k = 0; k <= 60; ++k)
  {
    const double t = 0.1 * k;
    // Two obstacles that pass each other 1.6 m apart.
    const Eigen::Vector3d a = Eigen::Vector3d(3.0, -2.0 + 0.6 * t, 1.0);
    const Eigen::Vector3d b = Eigen::Vector3d(4.6, 2.0 - 0.6 * t, 1.0);
    std::vector<Eigen::Vector3d> cloud;
    seenFaces(a, size, sensor, gen, 0.01, cloud);
    seenFaces(b, size, sensor, gen, 0.01, cloud);
    tracker.insert(cloud, t);
    if (k == 0)
    {
      check(tracker.tracks().size() == 2, "two tracks from the first cloud");
      for (const ctrack::Track& track : tracker.tracks())
      {
        ((track.pos() - a).norm() < 0.5 ? first_id : second_id) = track.id;
      }
    }
    else if (tracker.tracks().size() == 2 && first_id >= 0 && second_id >= 0)
    {
      const ctrack::Track* first = tracker.find(first_id);
      const ctrack::Track* second = tracker.find(second_id);
      ids_stable = ids_stable && first != nullptr && second != nullptr && (first->pos() - a).norm() < 0.6 &&
                   (second->pos() - b).norm() < 0.6;
    }
    else
    {
      ids_stable = false;
    }
  }
  check(ids_stable, "each track stays on its obstacle while the two pass each other");
  const ctrack::Track* first = tracker.find(first_id);
  check(first != nullptr && (first->vel() - Eigen::Vector3d(0, 0.6, 0)).norm() < 0.2,
        "the velocity estimate matches the motion (0.6 m/s)");
  check(first != nullptr && std::abs(first->bbox.y() - 0.6) < 0.15, "the size estimate matches the box");

  // The obstacles are gone: the tracks are deleted after the timeout.
  tracker.insert({}, 6.0 + par.time_to_delete - 0.5);
  check(tracker.tracks().size() == 2, "tracks are kept while they may still be near");
  tracker.insert({}, 6.0 + par.time_to_delete + 0.5);
  check(tracker.tracks().empty(), "and deleted once they have not been seen for the timeout");
}
}  // namespace

int main()
{
  testClustering();
  testFilter();
  testTracking();
  std::cout << (failures == 0 ? "ALL PASSED" : "FAILED: " + std::to_string(failures)) << std::endl;
  return failures == 0 ? 0 : 1;
}
