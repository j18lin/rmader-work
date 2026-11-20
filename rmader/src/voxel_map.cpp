/* ----------------------------------------------------------------------------
 * Voxel map of free, occupied and unknown space. See voxel_map.hpp.
 * -------------------------------------------------------------------------- */

#include "voxel_map.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>

namespace vmap
{
namespace
{
constexpr int kUnknownReach = 80;  // voxels; further than any inflation radius in use
}

VoxelMap::VoxelMap(const Params& params) : par_(params)
{
  const Eigen::Vector3d extent = (par_.upper - par_.lower).cwiseMax(par_.resolution);
  nx_ = std::max(1, static_cast<int>(std::ceil(extent.x() / par_.resolution - 1e-9)));
  ny_ = std::max(1, static_cast<int>(std::ceil(extent.y() / par_.resolution - 1e-9)));
  nz_ = par_.planar ? 1 : std::max(1, static_cast<int>(std::ceil(extent.z() / par_.resolution - 1e-9)));
  score_.assign(static_cast<size_t>(nx_) * ny_ * nz_, 0);
  observed_.assign(score_.size(), 0);
}

Eigen::Vector3d VoxelMap::center(int ix, int iy, int iz) const
{
  const double z = par_.planar ? 0.5 * (par_.lower.z() + par_.upper.z()) : par_.lower.z() + (iz + 0.5) * par_.resolution;
  return Eigen::Vector3d(par_.lower.x() + (ix + 0.5) * par_.resolution, par_.lower.y() + (iy + 0.5) * par_.resolution,
                         z);
}

bool VoxelMap::index(const Eigen::Vector3d& p, int& ix, int& iy, int& iz) const
{
  ix = static_cast<int>(std::floor((p.x() - par_.lower.x()) / par_.resolution));
  iy = static_cast<int>(std::floor((p.y() - par_.lower.y()) / par_.resolution));
  iz = par_.planar ? 0 : static_cast<int>(std::floor((p.z() - par_.lower.z()) / par_.resolution));
  return inside(ix, iy, iz);
}

Cell VoxelMap::at(const Eigen::Vector3d& p) const
{
  int ix, iy, iz;
  if (!index(p, ix, iy, iz))
  {
    return Cell::OUTSIDE;
  }
  const size_t i = linear(ix, iy, iz);
  if (!observed_[i])
  {
    return Cell::UNKNOWN;
  }
  return score_[i] > 0 ? Cell::OCCUPIED : Cell::FREE;
}

int VoxelMap::count(Cell value) const
{
  int total = 0;
  for (size_t i = 0; i < score_.size(); ++i)
  {
    const Cell cell = !observed_[i] ? Cell::UNKNOWN : (score_[i] > 0 ? Cell::OCCUPIED : Cell::FREE);
    total += cell == value;
  }
  return total;
}

void VoxelMap::fill(const Eigen::Vector3d& lo, const Eigen::Vector3d& hi, Cell value)
{
  for (int iz = 0; iz < nz_; ++iz)
  {
    for (int iy = 0; iy < ny_; ++iy)
    {
      for (int ix = 0; ix < nx_; ++ix)
      {
        const Eigen::Vector3d c = center(ix, iy, iz);
        const bool in_xy = c.x() >= lo.x() && c.x() <= hi.x() && c.y() >= lo.y() && c.y() <= hi.y();
        if (!in_xy || (!par_.planar && (c.z() < lo.z() || c.z() > hi.z())))
        {
          continue;
        }
        const size_t i = linear(ix, iy, iz);
        observed_[i] = value != Cell::UNKNOWN;
        score_[i] = value == Cell::OCCUPIED ? par_.max_score : (value == Cell::FREE ? par_.min_score : 0);
      }
    }
  }
  dirty_ = true;
}

void VoxelMap::insertScan(const Eigen::Vector3d& origin, const std::vector<Eigen::Vector3d>& points,
                          const std::vector<IgnoreBox>& ignore)
{
  // Every voxel is updated at most once per scan, and a ray that ends in a voxel outweighs rays that pass
  // through it, so that the density of the scan does not change how fast evidence builds up.
  std::vector<uint8_t> mark(score_.size(), 0);  // 1: passed through, 2: a ray ended here
  std::vector<size_t> touched;
  const double step = 0.5 * par_.resolution;

  auto ignored = [&](const Eigen::Vector3d& p) {
    for (const IgnoreBox& box : ignore)
    {
      if ((p.array() >= box.lo.array()).all() && (p.array() <= box.hi.array()).all())
      {
        return true;
      }
    }
    return false;
  };

  for (const Eigen::Vector3d& point : points)
  {
    if (!point.allFinite())
    {
      continue;
    }
    if (par_.planar && (point.z() < par_.lower.z() || point.z() > par_.upper.z()))
    {
      continue;  // the ray left the band the robot moves in before it ended
    }
    Eigen::Vector3d ray = point - origin;
    if (par_.planar)
    {
      ray.z() = 0.0;
    }
    const double range = ray.norm();
    if (range < 1e-6)
    {
      continue;
    }
    const bool hit = range < par_.sensor_range && !ignored(point);
    const double length = std::min(range, par_.sensor_range);
    const Eigen::Vector3d direction = ray / range;

    int ex, ey, ez;
    const bool end_inside = index(origin + direction * length, ex, ey, ez);
    const size_t end = end_inside ? linear(ex, ey, ez) : std::numeric_limits<size_t>::max();

    size_t last = std::numeric_limits<size_t>::max();
    for (double s = 0.0; s < length; s += step)
    {
      const Eigen::Vector3d q = origin + direction * s;
      int ix, iy, iz;
      if (!index(q, ix, iy, iz))
      {
        continue;
      }
      const size_t i = linear(ix, iy, iz);
      if (i == last || i == end)
      {
        continue;
      }
      last = i;
      if (!ignore.empty() && ignored(q))
      {
        continue;
      }
      if (mark[i] == 0)
      {
        mark[i] = 1;
        touched.push_back(i);
      }
    }
    if (end_inside)
    {
      if (hit)
      {
        if (mark[end] == 0)
        {
          touched.push_back(end);
        }
        mark[end] = 2;
      }
      else if (range >= par_.sensor_range && mark[end] == 0)
      {
        mark[end] = 1;  // nothing was found out to the sensor's range
        touched.push_back(end);
      }
    }
  }

  for (size_t i : touched)
  {
    const int delta = mark[i] == 2 ? par_.hit : -par_.miss;
    score_[i] = static_cast<int8_t>(std::min(par_.max_score, std::max(par_.min_score, score_[i] + delta)));
    observed_[i] = 1;
  }
  // A tracked obstacle is the tracker's business: whatever it left in the static map before it was tracked
  // is removed, or it would stand there as a static obstacle until the rays could see through it again.
  for (const IgnoreBox& box : ignore)
  {
    int lx, ly, lz, ux, uy, uz;
    index(box.lo, lx, ly, lz);
    index(box.hi, ux, uy, uz);
    for (int iz = std::max(lz, 0); iz <= std::min(uz, nz_ - 1); ++iz)
    {
      for (int iy = std::max(ly, 0); iy <= std::min(uy, ny_ - 1); ++iy)
      {
        for (int ix = std::max(lx, 0); ix <= std::min(ux, nx_ - 1); ++ix)
        {
          const size_t i = linear(ix, iy, iz);
          if (score_[i] > 0)
          {
            score_[i] = 0;
            dirty_ = true;
          }
        }
      }
    }
  }
  if (!touched.empty())
  {
    dirty_ = true;
  }
}

void VoxelMap::refresh() const
{
  if (!dirty_)
  {
    return;
  }
  dirty_ = false;
  const int dz_span = nz_ > 1 ? 1 : 0;

  // Chebyshev distance to the nearest unknown voxel: breadth-first over all neighbours, corners included.
  unknown_reach_ = std::min(kUnknownReach, 250);
  cells_to_unknown_.assign(score_.size(), static_cast<uint8_t>(unknown_reach_ + 1));
  std::deque<size_t> queue;
  for (size_t i = 0; i < score_.size(); ++i)
  {
    if (!observed_[i])
    {
      cells_to_unknown_[i] = 0;
      queue.push_back(i);
    }
  }
  while (!queue.empty())
  {
    const size_t i = queue.front();
    queue.pop_front();
    const int distance = cells_to_unknown_[i];
    if (distance >= unknown_reach_)
    {
      continue;
    }
    const int ix = i % nx_;
    const int iy = (i / nx_) % ny_;
    const int iz = i / (static_cast<size_t>(nx_) * ny_);
    for (int dz = -dz_span; dz <= dz_span; ++dz)
    {
      for (int dy = -1; dy <= 1; ++dy)
      {
        for (int dx = -1; dx <= 1; ++dx)
        {
          if (!inside(ix + dx, iy + dy, iz + dz))
          {
            continue;
          }
          const size_t j = linear(ix + dx, iy + dy, iz + dz);
          if (cells_to_unknown_[j] > distance + 1)
          {
            cells_to_unknown_[j] = static_cast<uint8_t>(distance + 1);
            queue.push_back(j);
          }
        }
      }
    }
  }

  // Euclidean distance to the nearest occupied voxel centre, up to heat_radius: each voxel remembers the
  // occupied voxel it is measured from and offers it to its neighbours.
  const float cap = static_cast<float>(par_.heat_radius);
  metres_to_occupied_.assign(score_.size(), cap);
  std::vector<int> source(score_.size(), -1);
  std::deque<size_t> open;
  for (size_t i = 0; i < score_.size(); ++i)
  {
    if (occupied(i))
    {
      metres_to_occupied_[i] = 0.0f;
      source[i] = static_cast<int>(i);
      open.push_back(i);
    }
  }
  while (!open.empty())
  {
    const size_t i = open.front();
    open.pop_front();
    const size_t s = source[i];
    const int sx = s % nx_;
    const int sy = (s / nx_) % ny_;
    const int sz = s / (static_cast<size_t>(nx_) * ny_);
    const int ix = i % nx_;
    const int iy = (i / nx_) % ny_;
    const int iz = i / (static_cast<size_t>(nx_) * ny_);
    for (int dz = -dz_span; dz <= dz_span; ++dz)
    {
      for (int dy = -1; dy <= 1; ++dy)
      {
        for (int dx = -1; dx <= 1; ++dx)
        {
          const int jx = ix + dx, jy = iy + dy, jz = iz + dz;
          if (!inside(jx, jy, jz))
          {
            continue;
          }
          const float distance = static_cast<float>(
              par_.resolution * std::sqrt(static_cast<double>((jx - sx) * (jx - sx) + (jy - sy) * (jy - sy) +
                                                              (jz - sz) * (jz - sz))));
          const size_t j = linear(jx, jy, jz);
          if (distance < metres_to_occupied_[j] - 1e-6f)
          {
            metres_to_occupied_[j] = distance;
            source[j] = static_cast<int>(s);
            open.push_back(j);
          }
        }
      }
    }
  }
}

bool VoxelMap::collides(const Eigen::Vector3d& p, const Eigen::Vector3d& half) const
{
  // A voxel is touched if its centre is within half + resolution / 2 of p along every axis.
  const Eigen::Vector3d reach = half + Eigen::Vector3d::Constant(0.5 * par_.resolution);
  int lx, ly, lz, ux, uy, uz;
  index(p - reach, lx, ly, lz);
  index(p + reach, ux, uy, uz);
  for (int iz = std::max(lz, 0); iz <= std::min(uz, nz_ - 1); ++iz)
  {
    for (int iy = std::max(ly, 0); iy <= std::min(uy, ny_ - 1); ++iy)
    {
      for (int ix = std::max(lx, 0); ix <= std::min(ux, nx_ - 1); ++ix)
      {
        if (!occupied(linear(ix, iy, iz)))
        {
          continue;
        }
        const Eigen::Vector3d offset = (center(ix, iy, iz) - p).cwiseAbs();
        if (offset.x() < reach.x() && offset.y() < reach.y() && (par_.planar || offset.z() < reach.z()))
        {
          return true;
        }
      }
    }
  }
  return false;
}

bool VoxelMap::nearUnknown(const Eigen::Vector3d& p, double radius) const
{
  int ix, iy, iz;
  if (!index(p, ix, iy, iz))
  {
    return false;
  }
  refresh();
  // A voxel c steps away is between (c - 1) and c voxel sides away, so one step is added to stay on the
  // safe side.
  const int steps = static_cast<int>(std::floor(std::max(radius, 0.0) / par_.resolution)) + 1;
  return cells_to_unknown_[linear(ix, iy, iz)] <= std::min(steps, unknown_reach_);
}

double VoxelMap::distanceToOccupied(const Eigen::Vector3d& p) const
{
  int ix, iy, iz;
  if (!index(p, ix, iy, iz))
  {
    return par_.heat_radius;
  }
  refresh();
  return metres_to_occupied_[linear(ix, iy, iz)];
}

double VoxelMap::staticHeat(const Eigen::Vector3d& p) const
{
  if (par_.heat_radius <= 0.0)
  {
    return 0.0;
  }
  const double distance = distanceToOccupied(p);
  if (distance >= par_.heat_radius)
  {
    return 0.0;
  }
  return std::min(par_.heat_alpha * std::pow(1.0 - distance / par_.heat_radius, par_.heat_power), par_.heat_max);
}

void VoxelMap::occupiedSurface(const Eigen::Vector3d& lo, const Eigen::Vector3d& hi,
                               std::vector<Eigen::Vector3d>& out) const
{
  int lx, ly, lz, ux, uy, uz;
  index(lo, lx, ly, lz);
  index(hi, ux, uy, uz);
  static const int neighbours[6][3] = { { 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 }, { 0, -1, 0 }, { 0, 0, 1 }, { 0, 0, -1 } };
  for (int iz = std::max(lz, 0); iz <= std::min(uz, nz_ - 1); ++iz)
  {
    for (int iy = std::max(ly, 0); iy <= std::min(uy, ny_ - 1); ++iy)
    {
      for (int ix = std::max(lx, 0); ix <= std::min(ux, nx_ - 1); ++ix)
      {
        if (!occupied(linear(ix, iy, iz)))
        {
          continue;
        }
        bool surface = false;
        for (int k = 0; k < (nz_ > 1 ? 6 : 4) && !surface; ++k)
        {
          const int jx = ix + neighbours[k][0], jy = iy + neighbours[k][1], jz = iz + neighbours[k][2];
          surface = !inside(jx, jy, jz) || !occupied(linear(jx, jy, jz));
        }
        if (surface)
        {
          out.push_back(center(ix, iy, iz));
        }
      }
    }
  }
}

void VoxelMap::unknownSurface(const Eigen::Vector3d& lo, const Eigen::Vector3d& hi, double radius,
                              std::vector<Eigen::Vector3d>& out) const
{
  refresh();
  const int steps = std::min(static_cast<int>(std::ceil(std::max(radius, 0.0) / par_.resolution - 1e-9)), unknown_reach_);
  int lx, ly, lz, ux, uy, uz;
  index(lo, lx, ly, lz);
  index(hi, ux, uy, uz);
  static const int neighbours[6][3] = { { 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 }, { 0, -1, 0 }, { 0, 0, 1 }, { 0, 0, -1 } };
  for (int iz = std::max(lz, 0); iz <= std::min(uz, nz_ - 1); ++iz)
  {
    for (int iy = std::max(ly, 0); iy <= std::min(uy, ny_ - 1); ++iy)
    {
      for (int ix = std::max(lx, 0); ix <= std::min(ux, nx_ - 1); ++ix)
      {
        const int distance = cells_to_unknown_[linear(ix, iy, iz)];
        if (distance > steps)
        {
          continue;
        }
        // The outermost voxels of the grown region: those with a neighbour outside it.
        bool surface = false;
        for (int k = 0; k < (nz_ > 1 ? 6 : 4) && !surface; ++k)
        {
          const int jx = ix + neighbours[k][0], jy = iy + neighbours[k][1], jz = iz + neighbours[k][2];
          surface = inside(jx, jy, jz) && cells_to_unknown_[linear(jx, jy, jz)] > steps;
        }
        if (surface)
        {
          out.push_back(center(ix, iy, iz));
        }
      }
    }
  }
}
}  // namespace vmap
