/* ----------------------------------------------------------------------------
 * Tests of the voxel map (no ROS needed):
 *   rosrun rmader test_voxel_map
 * -------------------------------------------------------------------------- */

#include "voxel_map.hpp"

#include <cmath>
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

struct Aabb
{
  Eigen::Vector3d lo, hi;
};

// Where a ray from `origin` along `direction` ends: on the nearest box, or at `range`.
Eigen::Vector3d castRay(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction, double range,
                        const std::vector<Aabb>& boxes)
{
  double nearest = range;
  for (const Aabb& box : boxes)
  {
    double t_in = 0.0, t_out = range;
    bool miss = false;
    for (int axis = 0; axis < 3 && !miss; ++axis)
    {
      if (std::abs(direction[axis]) < 1e-12)
      {
        miss = origin[axis] < box.lo[axis] || origin[axis] > box.hi[axis];
        continue;
      }
      double a = (box.lo[axis] - origin[axis]) / direction[axis];
      double b = (box.hi[axis] - origin[axis]) / direction[axis];
      if (a > b)
      {
        std::swap(a, b);
      }
      t_in = std::max(t_in, a);
      t_out = std::min(t_out, b);
      miss = t_in > t_out;
    }
    if (!miss)
    {
      nearest = std::min(nearest, t_in);
    }
  }
  return origin + direction * nearest;
}

std::vector<Eigen::Vector3d> planarScan(const Eigen::Vector3d& origin, double range, const std::vector<Aabb>& boxes,
                                        int rays = 720)
{
  std::vector<Eigen::Vector3d> points;
  for (int i = 0; i < rays; ++i)
  {
    const double angle = 2.0 * M_PI * i / rays;
    points.push_back(castRay(origin, Eigen::Vector3d(std::cos(angle), std::sin(angle), 0.0), range, boxes));
  }
  return points;
}

vmap::Params planarParams()
{
  vmap::Params par;
  par.resolution = 0.1;
  par.lower = Eigen::Vector3d(-6, -6, -0.2);
  par.upper = Eigen::Vector3d(6, 6, 0.6);
  par.planar = true;
  par.sensor_range = 4.0;
  par.heat_radius = 1.0;
  return par;
}

void testScan()
{
  std::cout << "One scan" << std::endl;
  vmap::VoxelMap map(planarParams());
  check(map.count(vmap::Cell::UNKNOWN) == 120 * 120, "everything is unknown before the first scan");

  const std::vector<Aabb> boxes = { { Eigen::Vector3d(1.5, -0.5, 0.0), Eigen::Vector3d(2.5, 0.5, 1.0) } };
  const Eigen::Vector3d origin(0.0, 0.0, 0.2);
  map.insertScan(origin, planarScan(origin, 5.0, boxes));

  check(map.at(Eigen::Vector3d(0.8, 0.0, 0.2)) == vmap::Cell::FREE, "free between the sensor and the box");
  check(map.at(Eigen::Vector3d(1.55, 0.0, 0.2)) == vmap::Cell::OCCUPIED, "the face of the box is occupied");
  check(map.at(Eigen::Vector3d(3.2, 0.0, 0.2)) == vmap::Cell::UNKNOWN, "unknown in the shadow behind the box");
  check(map.at(Eigen::Vector3d(-3.0, 0.0, 0.2)) == vmap::Cell::FREE, "free in the open, inside the sensor's range");
  check(map.at(Eigen::Vector3d(-4.5, 0.0, 0.2)) == vmap::Cell::UNKNOWN, "unknown beyond the sensor's range");
  check(map.at(Eigen::Vector3d(20.0, 0.0, 0.2)) == vmap::Cell::OUTSIDE, "outside the map is reported as such");

  check(map.collides(Eigen::Vector3d(1.2, 0.0, 0.2), Eigen::Vector3d(0.35, 0.35, 0.2)),
        "a robot-sized box next to the obstacle collides");
  check(!map.collides(Eigen::Vector3d(0.8, 0.0, 0.2), Eigen::Vector3d(0.35, 0.35, 0.2)), "further away it does not");

  // A ray that leaves the height band says nothing in planar mode.
  vmap::VoxelMap other(planarParams());
  other.insertScan(origin, { Eigen::Vector3d(2.0, 0.0, -1.0) });
  check(other.count(vmap::Cell::UNKNOWN) == 120 * 120, "a ray that ends below the band is not used");
}

void testEvidence()
{
  std::cout << "Evidence over several scans" << std::endl;
  vmap::VoxelMap map(planarParams());
  const Eigen::Vector3d origin(0.0, 0.0, 0.2);
  const std::vector<Aabb> crate = { { Eigen::Vector3d(1.5, -0.3, 0.0), Eigen::Vector3d(2.1, 0.3, 1.0) } };
  map.insertScan(origin, planarScan(origin, 5.0, crate));
  const Eigen::Vector3d face(1.55, 0.0, 0.2);
  check(map.at(face) == vmap::Cell::OCCUPIED, "seen once: occupied");
  // The crate is gone: rays now pass through where it was.
  map.insertScan(origin, planarScan(origin, 5.0, {}));
  check(map.at(face) == vmap::Cell::OCCUPIED, "one scan through it does not erase it");
  map.insertScan(origin, planarScan(origin, 5.0, {}));
  map.insertScan(origin, planarScan(origin, 5.0, {}));
  check(map.at(face) == vmap::Cell::FREE, "three scans through it do");
  check(map.at(Eigen::Vector3d(3.0, 0.0, 0.2)) == vmap::Cell::FREE, "and what was in its shadow is now seen");

  // A tracked obstacle is kept out of the static map.
  vmap::VoxelMap tracked(planarParams());
  vmap::IgnoreBox ignore;
  ignore.lo = Eigen::Vector3d(1.3, -0.5, -1.0);
  ignore.hi = Eigen::Vector3d(2.3, 0.5, 2.0);
  tracked.insertScan(origin, planarScan(origin, 5.0, crate), { ignore });
  check(tracked.count(vmap::Cell::OCCUPIED) == 0, "rays that end on a tracked obstacle do not mark the map");
  check(tracked.at(Eigen::Vector3d(0.8, 0.0, 0.2)) == vmap::Cell::FREE, "but still clear the space in front of it");

  // Seen once before it was tracked: its marks go as soon as it is.
  vmap::VoxelMap late(planarParams());
  late.insertScan(origin, planarScan(origin, 5.0, crate));
  check(late.count(vmap::Cell::OCCUPIED) > 0, "an obstacle seen before it is tracked marks the map");
  late.insertScan(origin, planarScan(origin, 5.0, crate), { ignore });
  check(late.count(vmap::Cell::OCCUPIED) == 0, "and the marks are removed once it is tracked");
}

void testHeat()
{
  std::cout << "Static heat" << std::endl;
  vmap::Params par = planarParams();
  vmap::VoxelMap map(par);
  map.fill(par.lower, par.upper, vmap::Cell::FREE);
  map.fill(Eigen::Vector3d(1.0, -1.0, 0.0), Eigen::Vector3d(2.0, 1.0, 0.5), vmap::Cell::OCCUPIED);
  check(std::abs(map.staticHeat(Eigen::Vector3d(1.05, 0.0, 0.2)) - par.heat_alpha) < 1e-9, "alpha on the obstacle");
  check(map.staticHeat(Eigen::Vector3d(-0.5, 0.0, 0.2)) == 0.0, "zero beyond the halo radius");
  const double near = map.staticHeat(Eigen::Vector3d(0.75, 0.0, 0.2));
  const double far = map.staticHeat(Eigen::Vector3d(0.35, 0.0, 0.2));
  check(near > far && far > 0.0, "falls with distance inside the halo");

  // Against the definition: alpha (1 - d / R)^p with d the distance to the nearest occupied voxel centre.
  std::mt19937 gen(5);
  std::uniform_real_distribution<double> uniform(-3.0, 4.0);
  double worst = 0.0;
  for (int trial = 0; trial < 400; ++trial)
  {
    const Eigen::Vector3d q(uniform(gen), uniform(gen), 0.2);
    int qx, qy, qz;
    map.index(q, qx, qy, qz);
    const Eigen::Vector3d q_centre = map.center(qx, qy, qz);
    double nearest = par.heat_radius;
    for (int ix = 0; ix < map.size().x(); ++ix)
    {
      for (int iy = 0; iy < map.size().y(); ++iy)
      {
        const Eigen::Vector3d c = map.center(ix, iy, 0);
        if (map.at(c) == vmap::Cell::OCCUPIED)
        {
          nearest = std::min(nearest, (c - q_centre).head<2>().norm());
        }
      }
    }
    const double expected =
        nearest >= par.heat_radius ? 0.0 : par.heat_alpha * std::pow(1.0 - nearest / par.heat_radius, par.heat_power);
    worst = std::max(worst, std::abs(expected - map.staticHeat(q)));
  }
  std::cout << "    largest difference from the definition " << worst << std::endl;
  check(worst < 0.05 * par.heat_alpha, "matches the paper's formula at random points");
}

void testUnknown()
{
  std::cout << "Unknown space and its inflation" << std::endl;
  vmap::Params par = planarParams();
  vmap::VoxelMap map(par);
  // Known: the half plane x < 2. Unknown: x >= 2.
  map.fill(par.lower, Eigen::Vector3d(2.0, 6.0, 1.0), vmap::Cell::FREE);
  check(map.nearUnknown(Eigen::Vector3d(1.45, 0.0, 0.2), 0.6), "0.55 m from the unknown: within 0.6 m");
  check(!map.nearUnknown(Eigen::Vector3d(1.0, 0.0, 0.2), 0.6), "1.0 m from the unknown: not within 0.6 m");
  check(!map.nearUnknown(Eigen::Vector3d(-2.0, 0.0, 0.2), 3.0), "4 m away: not within 3 m");

  // Against the definition, on a ragged unknown region: p is near if an unknown voxel overlaps the
  // L-infinity ball; the map may err by one voxel on the safe side only.
  vmap::VoxelMap ragged(par);
  const Eigen::Vector3d origin(0.0, 0.0, 0.2);
  const std::vector<Aabb> boxes = { { Eigen::Vector3d(1.5, -0.5, 0.0), Eigen::Vector3d(2.5, 0.5, 1.0) },
                                    { Eigen::Vector3d(-2.0, 1.0, 0.0), Eigen::Vector3d(-1.2, 1.6, 1.0) } };
  ragged.insertScan(origin, planarScan(origin, 5.0, boxes, 1440));
  std::mt19937 gen(9);
  std::uniform_real_distribution<double> uniform(-5.0, 5.0);
  int missed = 0, extra = 0, agreed = 0;
  for (int trial = 0; trial < 600; ++trial)
  {
    const Eigen::Vector3d q(uniform(gen), uniform(gen), 0.2);
    const double radius = 0.3 + 0.1 * (trial % 10);
    double nearest = 1e9;  // L-infinity distance from q to the nearest unknown voxel's cube
    for (int ix = 0; ix < ragged.size().x(); ++ix)
    {
      for (int iy = 0; iy < ragged.size().y(); ++iy)
      {
        const Eigen::Vector3d c = ragged.center(ix, iy, 0);
        if (ragged.at(c) == vmap::Cell::UNKNOWN)
        {
          const double d = std::max(std::abs(c.x() - q.x()), std::abs(c.y() - q.y())) - 0.5 * par.resolution;
          nearest = std::min(nearest, std::max(d, 0.0));
        }
      }
    }
    const bool truth = nearest <= radius;
    const bool answer = ragged.nearUnknown(q, radius);
    if (truth && !answer)
    {
      missed++;
    }
    else if (!truth && answer && nearest > radius + 2.0 * par.resolution)
    {
      extra++;
    }
    else
    {
      agreed++;
    }
  }
  std::cout << "    " << agreed << " agree, " << missed << " missed, " << extra << " more than two voxels too cautious"
            << std::endl;
  check(missed == 0, "never says clear when unknown space is within the radius");
  check(extra == 0, "and is never more than two voxels too cautious");

  // The grown region's outer voxels, used as obstacle points for the corridors.
  std::vector<Eigen::Vector3d> shell;
  map.unknownSurface(Eigen::Vector3d(-6, -1, -1), Eigen::Vector3d(6, 1, 1), 0.5, shell);
  bool placed = !shell.empty();
  for (const Eigen::Vector3d& q : shell)
  {
    placed = placed && std::abs(q.x() - 1.55) < 1e-6;  // five voxels out from the boundary at x = 2
  }
  check(placed, "unknown grown by 0.5 m: its outer voxels are 0.5 m out from the boundary");
  std::vector<Eigen::Vector3d> boundary;
  map.unknownSurface(Eigen::Vector3d(-6, -1, -1), Eigen::Vector3d(6, 1, 1), 0.0, boundary);
  bool on_boundary = !boundary.empty();
  for (const Eigen::Vector3d& q : boundary)
  {
    on_boundary = on_boundary && std::abs(q.x() - 2.05) < 1e-6;
  }
  check(on_boundary, "grown by nothing: the boundary voxels of the unknown region");
  std::vector<Eigen::Vector3d> larger;
  map.unknownSurface(Eigen::Vector3d(-6, -1, -1), Eigen::Vector3d(6, 1, 1), 1.2, larger);
  check(!larger.empty() && std::abs(larger.front().x() - 0.85) < 1e-6, "a later time layer reaches further");

  std::vector<Eigen::Vector3d> surface;
  ragged.occupiedSurface(par.lower, par.upper, surface);
  bool all_occupied = !surface.empty();
  for (const Eigen::Vector3d& q : surface)
  {
    all_occupied = all_occupied && ragged.at(q) == vmap::Cell::OCCUPIED;
  }
  check(all_occupied && static_cast<int>(surface.size()) == ragged.count(vmap::Cell::OCCUPIED),
        "the occupied surface is every voxel a scan can mark (it only sees surfaces)");
}

void testVolume()
{
  std::cout << "Three dimensions" << std::endl;
  vmap::Params par;
  par.resolution = 0.2;
  par.lower = Eigen::Vector3d(-4, -4, 0);
  par.upper = Eigen::Vector3d(4, 4, 3);
  par.sensor_range = 3.5;
  vmap::VoxelMap map(par);
  const std::vector<Aabb> boxes = { { Eigen::Vector3d(1.4, -0.6, 0.0), Eigen::Vector3d(2.2, 0.6, 2.0) } };
  const Eigen::Vector3d origin(0.0, 0.0, 1.0);
  std::vector<Eigen::Vector3d> points;
  for (int i = 0; i < 180; ++i)
  {
    for (int j = -20; j <= 20; ++j)
    {
      const double yaw = 2.0 * M_PI * i / 180, pitch = 0.04 * j;
      const Eigen::Vector3d d(std::cos(pitch) * std::cos(yaw), std::cos(pitch) * std::sin(yaw), std::sin(pitch));
      points.push_back(castRay(origin, d, 5.0, boxes));
    }
  }
  map.insertScan(origin, points);
  check(map.at(Eigen::Vector3d(0.7, 0.0, 1.0)) == vmap::Cell::FREE, "free in front of the box");
  check(map.at(Eigen::Vector3d(1.5, 0.0, 1.0)) == vmap::Cell::OCCUPIED, "occupied on its face");
  check(map.at(Eigen::Vector3d(3.0, 0.0, 1.0)) == vmap::Cell::UNKNOWN, "unknown behind it");
  check(map.at(Eigen::Vector3d(0.0, 0.0, 2.9)) == vmap::Cell::UNKNOWN, "unknown above the sensor's field of view");
  check(map.nearUnknown(Eigen::Vector3d(0.0, 0.0, 2.3), 0.5) && !map.nearUnknown(Eigen::Vector3d(0.9, 0.0, 1.0), 0.3),
        "the blind cone overhead is unknown, the swept space in front is not near any");
  check(map.staticHeat(Eigen::Vector3d(1.1, 0.0, 1.0)) > map.staticHeat(Eigen::Vector3d(0.7, 0.0, 1.0)),
        "heat rises towards the box");
}
}  // namespace

int main()
{
  testScan();
  testEvidence();
  testHeat();
  testUnknown();
  testVolume();
  std::cout << (failures == 0 ? "ALL PASSED" : "FAILED: " + std::to_string(failures)) << std::endl;
  return failures == 0 ? 0 : 1;
}
