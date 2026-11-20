/* ----------------------------------------------------------------------------
 * Spatio-temporal safe flight corridors. See stsfc.hpp.
 * -------------------------------------------------------------------------- */

#include "stsfc.hpp"

#include <decomp_util/ellipsoid_decomp.h>

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace corridor
{
namespace
{
Eigen::Vector3d closestOnSegment(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& q)
{
  const Eigen::Vector3d d = b - a;
  const double length2 = d.squaredNorm();
  if (length2 < 1e-12)
  {
    return a;
  }
  const double s = std::min(std::max((q - a).dot(d) / length2, 0.0), 1.0);
  return a + s * d;
}

// The points handed to the decomposition for one segment. Points that already
// include the robot's size are first moved away from the segment by `shrink`,
// so that moving the faces inwards by `shrink` afterwards leaves each of them
// on or outside the polytope: a face with normal n that excluded the moved
// point q + shrink * u (|u| = 1) ends up at most shrink * n.u <= shrink closer
// to the segment than q.
template <int Dim>
vec_Vecf<Dim> pointsForSegment(const LayerPoints& layer, const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                               const DecompOptions& options, double reach)
{
  vec_Vecf<Dim> out;
  std::unordered_set<uint64_t> seen;
  auto add = [&](const Eigen::Vector3d& q) {
    if (Dim == 2)
    {
      // Columns of points collapse to one point per xy cell of 1 cm.
      const uint64_t key = (static_cast<uint64_t>(static_cast<uint32_t>(std::llround(q.x() * 100.0))) << 32) |
                           static_cast<uint32_t>(std::llround(q.y() * 100.0));
      if (!seen.insert(key).second)
      {
        return;
      }
    }
    Vecf<Dim> point;
    for (int i = 0; i < Dim; ++i)
    {
      point[i] = q[i];
    }
    out.push_back(point);
  };
  auto relevant = [&](const Eigen::Vector3d& q) {
    if (options.planar && (q.z() < options.z_min || q.z() > options.z_max))
    {
      return false;
    }
    return (q - closestOnSegment(a, b, q)).norm() <= reach;
  };

  for (const Eigen::Vector3d& q : layer.raw)
  {
    if (relevant(q))
    {
      add(q);
    }
  }
  for (const Eigen::Vector3d& q : layer.inflated)
  {
    if (!relevant(q))
    {
      continue;
    }
    if (options.shrink <= 0.0)
    {
      add(q);
      continue;
    }
    Eigen::Vector3d away = q - closestOnSegment(a, b, q);
    if (options.planar)
    {
      away.z() = 0.0;
    }
    const double distance = away.norm();
    add(distance > 1e-6 ? Eigen::Vector3d(q + options.shrink * away / distance) : q);
  }
  return out;
}

template <int Dim>
Polytope toPolytope(const Polyhedron<Dim>& polyhedron, const DecompOptions& options)
{
  const auto faces = polyhedron.hyperplanes();
  const int extra = Dim == 2 ? 2 : 0;
  Polytope out;
  out.A = Eigen::MatrixXd::Zero(faces.size() + extra, 3);
  out.b = Eigen::VectorXd::Zero(faces.size() + extra);
  for (size_t f = 0; f < faces.size(); ++f)
  {
    const double norm = faces[f].n_.norm();
    for (int i = 0; i < Dim; ++i)
    {
      out.A(f, i) = faces[f].n_[i] / norm;
    }
    out.b[f] = faces[f].n_.dot(faces[f].p_) / norm;
  }
  if (Dim == 2)
  {
    out.A(faces.size(), 2) = 1.0;
    out.b[faces.size()] = options.z_max;
    out.A(faces.size() + 1, 2) = -1.0;
    out.b[faces.size() + 1] = -options.z_min;
  }
  return out;
}

template <int Dim>
Polytope decomposeSegment(const LayerPoints& layer, const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                          const DecompOptions& options)
{
  if ((b - a).head(Dim).norm() < 1e-6)
  {
    return Polytope();
  }
  const double reach = (b - a).norm() + options.local_bbox.norm() + options.shrink + 1.0;
  EllipsoidDecomp<Dim> decomp;
  decomp.set_obs(pointsForSegment<Dim>(layer, a, b, options, reach));
  Vecf<Dim> bbox;
  for (int i = 0; i < Dim; ++i)
  {
    bbox[i] = options.local_bbox[i];
  }
  decomp.set_local_bbox(bbox);
  vec_Vecf<Dim> seed(2);
  for (int i = 0; i < Dim; ++i)
  {
    seed[0][i] = a[i];
    seed[1][i] = b[i];
  }
  decomp.dilate(seed);
  if (options.shrink > 0.0)
  {
    decomp.shrink_polyhedrons(options.shrink);
  }
  const auto polyhedrons = decomp.get_polyhedrons();
  if (polyhedrons.size() != 1)
  {
    return Polytope();
  }
  Polytope out = toPolytope<Dim>(polyhedrons[0], options);
  if (out.A.hasNaN() || out.b.hasNaN())
  {
    return Polytope();
  }
  Eigen::Vector3d middle = 0.5 * (a + b);
  if (Dim == 2)
  {
    middle.z() = 0.5 * (options.z_min + options.z_max);
  }
  if (!out.contains(middle, 1e-9))
  {
    return Polytope();  // the segment itself is blocked in this layer
  }
  return out;
}
}  // namespace

Layers buildCorridors(const std::vector<Eigen::Vector3d>& path, const std::vector<LayerPoints>& layers,
                      const DecompOptions& options)
{
  Layers out(layers.size());
  if (path.size() < 2)
  {
    return out;
  }
  for (size_t n = 0; n < layers.size(); ++n)
  {
    out[n].resize(path.size() - 1);
    for (size_t p = 0; p + 1 < path.size(); ++p)
    {
      out[n][p] = options.planar ? decomposeSegment<2>(layers[n], path[p], path[p + 1], options) :
                                   decomposeSegment<3>(layers[n], path[p], path[p + 1], options);
    }
  }
  return out;
}

void sampleTriangle(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c, double spacing,
                    std::vector<Eigen::Vector3d>& out)
{
  const double longest = std::max({ (b - a).norm(), (c - a).norm(), (c - b).norm() });
  const int k = std::max(1, static_cast<int>(std::ceil(longest / spacing)));
  for (int i = 0; i <= k; ++i)
  {
    for (int j = 0; i + j <= k; ++j)
    {
      const double u = static_cast<double>(i) / k;
      const double v = static_cast<double>(j) / k;
      out.push_back(a + u * (b - a) + v * (c - a));
    }
  }
}

void sampleBox(const Eigen::Vector3d& lo, const Eigen::Vector3d& hi, double spacing,
               std::vector<Eigen::Vector3d>& out)
{
  int count[3];
  for (int axis = 0; axis < 3; ++axis)
  {
    count[axis] = std::max(1, static_cast<int>(std::ceil((hi[axis] - lo[axis]) / spacing)));
  }
  for (int i = 0; i <= count[0]; ++i)
  {
    for (int j = 0; j <= count[1]; ++j)
    {
      for (int k = 0; k <= count[2]; ++k)
      {
        if (i != 0 && i != count[0] && j != 0 && j != count[1] && k != 0 && k != count[2])
        {
          continue;  // inside
        }
        out.emplace_back(lo.x() + (hi.x() - lo.x()) * i / count[0], lo.y() + (hi.y() - lo.y()) * j / count[1],
                         lo.z() + (hi.z() - lo.z()) * k / count[2]);
      }
    }
  }
}

std::vector<Eigen::Vector3d> shortcutPath(const std::vector<Eigen::Vector3d>& path,
                                          const std::function<bool(const Eigen::Vector3d&, const Eigen::Vector3d&)>&
                                              segment_free)
{
  if (path.size() < 3)
  {
    return path;
  }
  // From each kept vertex, jump to the furthest vertex that can be reached in a straight line.
  std::vector<Eigen::Vector3d> shortened = { path.front() };
  size_t i = 0;
  while (i + 1 < path.size())
  {
    size_t next = i + 1;
    for (size_t j = path.size() - 1; j > i + 1; --j)
    {
      if (!segment_free || segment_free(path[i], path[j]))
      {
        next = j;
        break;
      }
    }
    shortened.push_back(path[next]);
    i = next;
  }
  return shortened;
}

std::vector<Eigen::Vector3d> shapePath(const std::vector<Eigen::Vector3d>& path, int max_segments, double min_length,
                                       const std::function<bool(const Eigen::Vector3d&, const Eigen::Vector3d&)>&
                                           segment_free)
{
  if (path.size() < 2 || max_segments < 1)
  {
    return path;
  }
  const std::vector<Eigen::Vector3d> shortened = shortcutPath(path, segment_free);

  // Remove repeated points.
  std::vector<Eigen::Vector3d> split = { shortened.front() };
  for (size_t k = 1; k < shortened.size(); ++k)
  {
    if ((shortened[k] - split.back()).norm() > 1e-6)
    {
      split.push_back(shortened[k]);
    }
  }
  // Give the longest segments more pieces until there are max_segments in total, then cut each evenly.
  std::vector<int> pieces(split.size() - 1, 1);
  int total = pieces.size();
  while (total < max_segments)
  {
    size_t longest = 0;
    double length = 0.0;
    for (size_t k = 0; k < pieces.size(); ++k)
    {
      const double piece_length = (split[k + 1] - split[k]).norm() / pieces[k];
      if (piece_length > length)
      {
        length = piece_length;
        longest = k;
      }
    }
    if (length < 2.0 * std::max(min_length, 1e-3))
    {
      break;
    }
    pieces[longest]++;
    total++;
  }
  std::vector<Eigen::Vector3d> corners;
  corners.swap(split);
  split.push_back(corners.front());
  for (size_t k = 0; k < pieces.size(); ++k)
  {
    for (int piece = 1; piece <= pieces[k]; ++piece)
    {
      split.push_back(corners[k] + (corners[k + 1] - corners[k]) * (static_cast<double>(piece) / pieces[k]));
    }
  }
  if (static_cast<int>(split.size()) > max_segments + 1)
  {
    split.resize(max_segments + 1);
  }
  return split;
}
}  // namespace corridor
