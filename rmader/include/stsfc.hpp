/* ----------------------------------------------------------------------------
 * Spatio-temporal safe flight corridors (STSFC).
 *
 * Port of SANDO's cvxEllipsoidDecompTimeLayered (Kondo, Tordesillas, How; BSD
 * 3-Clause, Copyright (c) 2024 Kota Kondo), src/hgp/hgp_manager.cpp: for every
 * time layer n and every segment p of the global path, one convex polytope
 * that contains the segment and excludes the points that are occupied, or may
 * be occupied, during layer n. The decomposition is the ellipsoid method of
 * DecompUtil (Liu et al.), as in SANDO; ground robots use its 2D variant and
 * the polygon is extruded over a height band, as in the "2D changes" fork.
 *
 * Two kinds of points are accepted per layer:
 *   - `inflated`: already grown by the robot's size (R-MADER's hulls are);
 *   - `raw`: plain occupied or unknown points (map voxels). The polytopes are
 *     moved inwards by `shrink` to keep the robot clear of them.
 *
 * This file does not depend on ROS.
 * SANDO's licence text is in rmader/LICENSE.SANDO.
 * -------------------------------------------------------------------------- */

#pragma once

#include "corridor_planner.hpp"

#include <Eigen/Dense>
#include <functional>
#include <vector>

namespace corridor
{
struct LayerPoints
{
  std::vector<Eigen::Vector3d> inflated;
  std::vector<Eigen::Vector3d> raw;
};

struct DecompOptions
{
  // Box around each path segment, in the segment's frame (along, across, up), beyond which nothing is used.
  Eigen::Vector3d local_bbox = Eigen::Vector3d(2.0, 2.0, 1.0);
  double shrink = 0.0;  // [m] clearance kept from the raw points
  bool planar = false;  // 2D decomposition extruded over [z_min, z_max]
  double z_min = 0.0;
  double z_max = 0.0;
};

// Returns [layers.size()][path.size() - 1]. A polytope that no longer contains
// the middle of its segment (the segment is blocked in that layer) is empty.
Layers buildCorridors(const std::vector<Eigen::Vector3d>& path, const std::vector<LayerPoints>& layers,
                      const DecompOptions& options);

// Points on the surface of a box, at most `spacing` apart.
void sampleBox(const Eigen::Vector3d& lo, const Eigen::Vector3d& hi, double spacing,
               std::vector<Eigen::Vector3d>& out);

// Points on a triangle, at most `spacing` apart.
void sampleTriangle(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c, double spacing,
                    std::vector<Eigen::Vector3d>& out);

// Drop vertices of a polyline while `segment_free` accepts the straight line that replaces them.
std::vector<Eigen::Vector3d> shortcutPath(const std::vector<Eigen::Vector3d>& path,
                                          const std::function<bool(const Eigen::Vector3d&, const Eigen::Vector3d&)>&
                                              segment_free);

// Shape a polyline into the seed of a corridor with `max_segments` segments: drop vertices while
// `segment_free` accepts the shortcut, cut the longest segments evenly until there are `max_segments` (no
// piece shorter than `min_length`), and if the corners alone need more segments keep the first `max_segments`.
std::vector<Eigen::Vector3d> shapePath(const std::vector<Eigen::Vector3d>& path, int max_segments, double min_length,
                                       const std::function<bool(const Eigen::Vector3d&, const Eigen::Vector3d&)>&
                                           segment_free);
}  // namespace corridor
