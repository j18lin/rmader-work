/* ----------------------------------------------------------------------------
 * Voxel map of free, occupied and unknown space, built from range scans.
 *
 * What SANDO (Kondo, Tordesillas, How; arXiv 2604.07599) needs from its map,
 * Sections IV-C and V-A:
 *   - every voxel is free, occupied or unknown (never observed);
 *   - the global planner treats occupied voxels, grown by the robot's size, as
 *     blocked and unknown voxels as free, and pays a "static heat" near
 *     obstacle surfaces;
 *   - corridor generation treats unknown voxels as occupied and grows the
 *     unknown region by the distance an unseen obstacle could have covered,
 *     U + B_inf(r), with r different for every time layer.
 *
 * SANDO takes the occupied and unknown voxels from an external mapper; here
 * the map is built directly by casting the rays of each scan. This file does
 * not depend on ROS.
 * -------------------------------------------------------------------------- */

#pragma once

#include <Eigen/Dense>

#include <cstdint>
#include <utility>
#include <vector>

namespace vmap
{
enum class Cell : uint8_t
{
  UNKNOWN = 0,
  FREE = 1,
  OCCUPIED = 2,
  OUTSIDE = 3  // not covered by the map
};

struct Params
{
  double resolution = 0.15;  // [m] voxel side
  Eigen::Vector3d lower = Eigen::Vector3d(-10.0, -10.0, 0.0);
  Eigen::Vector3d upper = Eigen::Vector3d(10.0, 10.0, 3.0);
  // Ground robots: one layer of voxels; only rays that end inside [lower.z, upper.z] are used.
  bool planar = false;
  double sensor_range = 6.0;  // [m] a ray this long or longer ended on nothing
  // Evidence per voxel: a ray ending in it adds `hit`, a ray passing through takes `miss` away; the voxel is
  // occupied while its score is positive. One pass does not erase a wall, a few do erase a trail.
  int hit = 3;
  int miss = 1;
  int max_score = 6;
  int min_score = -3;
  // Static heat (paper Table II): alpha (1 - d / radius)^power within `radius` of an obstacle surface.
  double heat_alpha = 5.0;
  double heat_power = 2.0;
  double heat_radius = 1.0;  // [m] the paper uses three robot radii
  double heat_max = 50.0;
};

struct IgnoreBox  // rays ending in here say nothing about the static map (a tracked dynamic obstacle)
{
  Eigen::Vector3d lo;
  Eigen::Vector3d hi;
};

class VoxelMap
{
public:
  explicit VoxelMap(const Params& params = Params());

  const Params& params() const
  {
    return par_;
  }

  // One scan: where the sensor was and where each ray ended, in the map frame.
  void insertScan(const Eigen::Vector3d& origin, const std::vector<Eigen::Vector3d>& points,
                  const std::vector<IgnoreBox>& ignore = std::vector<IgnoreBox>());
  // Prior knowledge and tests.
  void fill(const Eigen::Vector3d& lo, const Eigen::Vector3d& hi, Cell value);

  Cell at(const Eigen::Vector3d& p) const;
  int count(Cell value) const;

  // True if a box with half sizes `half` centred on p touches an occupied voxel.
  bool collides(const Eigen::Vector3d& p, const Eigen::Vector3d& half) const;
  // True if an unknown voxel lies within `radius` of p along every axis (the L-infinity ball of the paper).
  bool nearUnknown(const Eigen::Vector3d& p, double radius) const;
  // H^s(q) of the paper: maximum over the obstacle surface voxels, which only depends on the nearest one.
  double staticHeat(const Eigen::Vector3d& p) const;
  double distanceToOccupied(const Eigen::Vector3d& p) const;  // [m], capped at heat_radius

  // Points for corridor generation, inside the box [lo, hi]:
  // centres of the occupied voxels that have a free or unknown neighbour,
  void occupiedSurface(const Eigen::Vector3d& lo, const Eigen::Vector3d& hi, std::vector<Eigen::Vector3d>& out) const;
  // and centres of the outermost voxels of the unknown region grown by `radius`.
  void unknownSurface(const Eigen::Vector3d& lo, const Eigen::Vector3d& hi, double radius,
                      std::vector<Eigen::Vector3d>& out) const;

  Eigen::Vector3i size() const
  {
    return Eigen::Vector3i(nx_, ny_, nz_);
  }
  Eigen::Vector3d center(int ix, int iy, int iz) const;
  bool index(const Eigen::Vector3d& p, int& ix, int& iy, int& iz) const;  // false outside the map

private:
  size_t linear(int ix, int iy, int iz) const
  {
    return static_cast<size_t>(ix) + static_cast<size_t>(nx_) * (iy + static_cast<size_t>(ny_) * iz);
  }
  bool inside(int ix, int iy, int iz) const
  {
    return ix >= 0 && iy >= 0 && iz >= 0 && ix < nx_ && iy < ny_ && iz < nz_;
  }
  bool occupied(size_t i) const
  {
    return observed_[i] && score_[i] > 0;
  }
  void refresh() const;  // distance fields, recomputed after the map changed

  Params par_;
  int nx_ = 0, ny_ = 0, nz_ = 0;
  std::vector<int8_t> score_;
  std::vector<uint8_t> observed_;

  mutable bool dirty_ = true;
  mutable std::vector<uint8_t> cells_to_unknown_;  // Chebyshev distance to the nearest unknown voxel, in voxels
  mutable std::vector<float> metres_to_occupied_;  // Euclidean distance to the nearest occupied voxel centre
  mutable int unknown_reach_ = 0;                  // cells_to_unknown_ is exact up to this value
};
}  // namespace vmap
