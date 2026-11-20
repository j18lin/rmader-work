/* ----------------------------------------------------------------------------
 * Moving obstacles from point clouds. See cloud_tracker.hpp.
 * -------------------------------------------------------------------------- */

#include "cloud_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

namespace ctrack
{
namespace
{
struct Key
{
  long x, y, z;
  bool operator==(const Key& other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct KeyHash
{
  size_t operator()(const Key& key) const
  {
    size_t h = static_cast<size_t>(key.x) * 73856093u;
    h ^= static_cast<size_t>(key.y) * 19349663u;
    h ^= static_cast<size_t>(key.z) * 83492791u;
    return h;
  }
};

Key keyOf(const Eigen::Vector3d& p, double size, bool planar = false)
{
  return Key{ static_cast<long>(std::floor(p.x() / size)), static_cast<long>(std::floor(p.y() / size)),
              planar ? 0 : static_cast<long>(std::floor(p.z() / size)) };
}
}  // namespace

std::vector<Eigen::Vector3d> downsample(const std::vector<Eigen::Vector3d>& points, double leaf_size)
{
  if (leaf_size <= 0.0)
  {
    return points;
  }
  // One point per voxel: the mean of the points in it, as pcl::VoxelGrid does.
  std::unordered_map<Key, std::pair<Eigen::Vector3d, int>, KeyHash> voxels;
  std::vector<Key> order;
  for (const Eigen::Vector3d& p : points)
  {
    if (!p.allFinite())
    {
      continue;
    }
    const Key key = keyOf(p, leaf_size);
    auto it = voxels.find(key);
    if (it == voxels.end())
    {
      voxels.emplace(key, std::make_pair(p, 1));
      order.push_back(key);
    }
    else
    {
      it->second.first += p;
      it->second.second++;
    }
  }
  std::vector<Eigen::Vector3d> out;
  out.reserve(order.size());
  for (const Key& key : order)
  {
    const auto& voxel = voxels[key];
    out.push_back(voxel.first / voxel.second);
  }
  return out;
}

std::vector<Cluster> extractClusters(const std::vector<Eigen::Vector3d>& points, const Params& params)
{
  // Buckets of side `tolerance`: the neighbours of a point are in its bucket or in the ones around it.
  const double tolerance = params.cluster_tolerance;
  std::unordered_map<Key, std::vector<int>, KeyHash> buckets;
  for (size_t i = 0; i < points.size(); ++i)
  {
    buckets[keyOf(points[i], tolerance, params.planar)].push_back(i);
  }
  auto distance2 = [&](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    Eigen::Vector3d d = a - b;
    if (params.planar)
    {
      d.z() = 0.0;
    }
    return d.squaredNorm();
  };

  std::vector<Cluster> clusters;
  std::vector<char> visited(points.size(), 0);
  for (size_t seed = 0; seed < points.size(); ++seed)
  {
    if (visited[seed])
    {
      continue;
    }
    std::vector<int> members = { static_cast<int>(seed) };
    visited[seed] = 1;
    for (size_t head = 0; head < members.size(); ++head)
    {
      const Eigen::Vector3d& p = points[members[head]];
      const Key key = keyOf(p, tolerance, params.planar);
      for (long dx = -1; dx <= 1; ++dx)
      {
        for (long dy = -1; dy <= 1; ++dy)
        {
          for (long dz = (params.planar ? 0 : -1); dz <= (params.planar ? 0 : 1); ++dz)
          {
            const auto it = buckets.find(Key{ key.x + dx, key.y + dy, key.z + dz });
            if (it == buckets.end())
            {
              continue;
            }
            for (int j : it->second)
            {
              if (!visited[j] && distance2(p, points[j]) <= tolerance * tolerance)
              {
                visited[j] = 1;
                members.push_back(j);
              }
            }
          }
        }
      }
    }
    if (static_cast<int>(members.size()) < params.min_cluster_size ||
        static_cast<int>(members.size()) > params.max_cluster_size)
    {
      continue;
    }
    Eigen::Vector3d lo = points[members.front()], hi = points[members.front()];
    for (int j : members)
    {
      lo = lo.cwiseMin(points[j]);
      hi = hi.cwiseMax(points[j]);
    }
    Cluster cluster;
    cluster.center = 0.5 * (lo + hi);
    cluster.bbox = hi - lo;
    cluster.points = members.size();
    if (cluster.bbox.norm() > params.bbox_cutoff)
    {
      continue;  // too large to be one of the obstacles this is for: probably a wall
    }
    clusters.push_back(cluster);
  }
  return clusters;
}

void predict(Track& track, double dt)
{
  Eigen::Matrix<double, 9, 9> F = Eigen::Matrix<double, 9, 9>::Identity();
  for (int axis = 0; axis < 3; ++axis)
  {
    F(axis, 3 + axis) = dt;
    F(axis, 6 + axis) = 0.5 * dt * dt;
    F(3 + axis, 6 + axis) = dt;
  }
  track.x = F * track.x;
  track.P = F * track.P * F.transpose() + track.Q;
}

void update(Track& track, const Eigen::Vector3d& measurement, const Eigen::Vector3d& bbox, double stamp,
            const Params& params)
{
  Eigen::Matrix<double, 3, 9> H = Eigen::Matrix<double, 3, 9>::Zero();
  H.leftCols<3>() = Eigen::Matrix3d::Identity();

  const Eigen::Vector3d innovation = measurement - H * track.x;
  const Eigen::Matrix3d S = H * track.P * H.transpose() + track.R;
  const Eigen::Matrix<double, 9, 3> K = track.P * H.transpose() * S.inverse();
  track.x += K * innovation;
  const Eigen::Vector3d residual = measurement - H * track.x;

  if (params.adaptive)
  {
    track.R = params.alpha * track.R +
              (1.0 - params.alpha) * (residual * residual.transpose() + H * track.P * H.transpose());
    track.Q = params.alpha * track.Q + (1.0 - params.alpha) * (K * innovation * innovation.transpose() * K.transpose());
  }
  else
  {
    track.R = 0.01 * Eigen::Matrix3d::Identity();
    track.Q = 0.01 * Eigen::Matrix<double, 9, 9>::Identity();
  }
  track.P = (Eigen::Matrix<double, 9, 9>::Identity() - K * H) * track.P;
  track.time_updated = stamp;
  track.bbox = track.updates == 0 ? bbox : Eigen::Vector3d(0.5 * track.bbox + 0.5 * bbox);
  track.updates++;
}

const Track* CloudTracker::find(int id) const
{
  for (const Track& track : tracks_)
  {
    if (track.id == id)
    {
      return &track;
    }
  }
  return nullptr;
}

std::vector<int> CloudTracker::insert(const std::vector<Eigen::Vector3d>& points, double stamp)
{
  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                               [&](const Track& track) { return stamp - track.time_updated > par_.time_to_delete; }),
                tracks_.end());

  const std::vector<Cluster> clusters = extractClusters(downsample(points, par_.leaf_size), par_);
  std::vector<int> updated;
  for (const Cluster& cluster : clusters)
  {
    // Nearest track within the association distance, as in SANDO; a track takes one cluster per cloud.
    int nearest = -1;
    double nearest_distance = par_.association_distance;
    for (size_t i = 0; i < tracks_.size(); ++i)
    {
      if (std::find(updated.begin(), updated.end(), tracks_[i].id) != updated.end())
      {
        continue;
      }
      const double distance = (cluster.center - tracks_[i].pos()).norm();
      if (distance < nearest_distance)
      {
        nearest_distance = distance;
        nearest = i;
      }
    }
    if (nearest >= 0)
    {
      predict(tracks_[nearest], std::max(stamp - tracks_[nearest].time_updated, 0.0));
      update(tracks_[nearest], cluster.center, cluster.bbox, stamp, par_);
      updated.push_back(tracks_[nearest].id);
      continue;
    }
    // A new track starts with the noise covariances the existing tracks have settled on.
    Track track;
    track.id = next_id_++;
    if (!tracks_.empty())
    {
      track.Q.setZero();
      track.R.setZero();
      for (const Track& other : tracks_)
      {
        track.Q += other.Q / tracks_.size();
        track.R += other.R / tracks_.size();
      }
    }
    track.x.head<3>() = cluster.center;
    track.bbox = cluster.bbox;
    track.time_created = stamp;
    track.time_updated = stamp;
    track.updates = 1;
    tracks_.push_back(track);
    updated.push_back(track.id);
  }
  return updated;
}
}  // namespace ctrack
