/* ----------------------------------------------------------------------------
 * Moving obstacles from point clouds: clustering and an adaptive Kalman filter.
 *
 * Port of SANDO's obstacle tracker (Kondo, Tordesillas, How; BSD 3-Clause,
 * Copyright (c) 2026 Kota Kondo), src/sando/obstacle_tracker_node.cpp, without
 * PCL or ROS:
 *   - the cloud is thinned with a voxel grid and split into Euclidean clusters;
 *   - a cluster is described by the centre and the size of its bounding box;
 *     clusters that are too large are dropped (walls);
 *   - each cluster is associated with the nearest track within a distance;
 *   - a track is a Kalman filter with state [position, velocity, acceleration]
 *     (constant acceleration model) that measures position. Its noise
 *     covariances adapt: R <- a R + (1 - a)(e e' + H P H'),
 *     Q <- a Q + (1 - a) K d d' K', with d the innovation, e the residual after
 *     the update and a the forgetting factor (0.9 in SANDO's configuration);
 *   - tracks that have not been updated for a while are deleted.
 *
 * Two differences: the prediction uses the time that actually passed since
 * the track's last update (SANDO uses a fixed step), and which points are
 * handed to the tracker is decided by the caller (see Rmader::updateMap: the
 * points the static map does not explain).
 * SANDO's licence text is in rmader/LICENSE.SANDO.
 * -------------------------------------------------------------------------- */

#pragma once

#include <Eigen/Dense>
#include <vector>

namespace ctrack
{
struct Params
{
  double leaf_size = 0.2;             // [m] voxel grid used to thin the cloud
  double cluster_tolerance = 0.5;     // [m] points closer than this are one cluster
  int min_cluster_size = 3;           // [points] after thinning
  int max_cluster_size = 2000;
  double bbox_cutoff = 2.0;           // [m] clusters with a longer bounding-box diagonal are dropped
  double association_distance = 1.0;  // [m] a cluster continues the nearest track within this distance
  bool adaptive = true;
  double alpha = 0.9;                 // forgetting factor of the adaptive filter; 1 = plain Kalman filter
  double time_to_delete = 5.0;        // [s]
  bool planar = false;                // ignore height when clustering (ground robots)
};

struct Cluster
{
  Eigen::Vector3d center = Eigen::Vector3d::Zero();  // of the bounding box
  Eigen::Vector3d bbox = Eigen::Vector3d::Zero();    // its size
  int points = 0;
};

struct Track
{
  int id = 0;
  Eigen::Matrix<double, 9, 1> x = Eigen::Matrix<double, 9, 1>::Zero();
  Eigen::Matrix<double, 9, 9> P = Eigen::Matrix<double, 9, 9>::Identity();
  Eigen::Matrix<double, 9, 9> Q = 0.01 * Eigen::Matrix<double, 9, 9>::Identity();
  Eigen::Matrix3d R = 0.01 * Eigen::Matrix3d::Identity();
  double time_created = 0.0;
  double time_updated = 0.0;
  Eigen::Vector3d bbox = Eigen::Vector3d::Zero();
  int updates = 0;

  Eigen::Vector3d pos() const
  {
    return x.head<3>();
  }
  Eigen::Vector3d vel() const
  {
    return x.segment<3>(3);
  }
  Eigen::Vector3d acc() const
  {
    return x.tail<3>();
  }
};

std::vector<Eigen::Vector3d> downsample(const std::vector<Eigen::Vector3d>& points, double leaf_size);
std::vector<Cluster> extractClusters(const std::vector<Eigen::Vector3d>& points, const Params& params);

void predict(Track& track, double dt);
void update(Track& track, const Eigen::Vector3d& measurement, const Eigen::Vector3d& bbox, double stamp,
            const Params& params);

class CloudTracker
{
public:
  explicit CloudTracker(const Params& params = Params()) : par_(params)
  {
  }
  // One cloud (already reduced to the points that may belong to moving obstacles). Returns the ids of the
  // tracks it updated or created.
  std::vector<int> insert(const std::vector<Eigen::Vector3d>& points, double stamp);
  const std::vector<Track>& tracks() const
  {
    return tracks_;
  }
  const Track* find(int id) const;

private:
  Params par_;
  std::vector<Track> tracks_;
  int next_id_ = 0;
};
}  // namespace ctrack
