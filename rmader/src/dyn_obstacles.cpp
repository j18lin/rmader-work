/* ----------------------------------------------------------------------------
 * Observation-based dynamic obstacle handling for R-MADER.
 * See include/dyn_obstacles.hpp for the overview.
 * -------------------------------------------------------------------------- */

#include "dyn_obstacles.hpp"

#include <algorithm>
#include <cmath>

namespace dynobs
{
namespace
{
// Seconds elapsed since the track was last observed. The past is clamped to the observation.
double elapsed(const Track& track, double t)
{
  return std::max(0.0, t - track.stamp);
}

Box boxAround(const Eigen::Vector3d& center, double half, const Eigen::Vector3d& margin)
{
  Box box;
  box.lo = center - Eigen::Vector3d::Constant(half) - margin;
  box.hi = center + Eigen::Vector3d::Constant(half) + margin;
  return box;
}

void merge(Box& box, const Box& other)
{
  box.lo = box.lo.cwiseMin(other.lo);
  box.hi = box.hi.cwiseMax(other.hi);
}

Eigen::Vector3d predictedCenter(const Track& track, double tau, const Params& par)
{
  return track.pos + track.vel * std::min(tau, par.prediction_horizon);
}

Eigen::Vector3d closestPointOnSegment(const Eigen::Vector3d& p, const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
  const Eigen::Vector3d ab = b - a;
  const double length_sq = ab.squaredNorm();
  if (length_sq < 1e-12)
  {
    return a;
  }
  const double s = std::min(1.0, std::max(0.0, (p - a).dot(ab) / length_sq));
  return a + s * ab;
}

double falloff(double distance, double radius, double power)
{
  if (radius <= 0.0 || distance >= radius)
  {
    return 0.0;
  }
  return std::pow(1.0 - distance / radius, power);
}
}  // namespace

bool modeFromString(const std::string& name, Mode& mode)
{
  if (name == "reachable")
  {
    mode = Mode::REACHABLE;
    return true;
  }
  if (name == "predicted")
  {
    mode = Mode::PREDICTED;
    return true;
  }
  return false;
}

std::vector<Eigen::Vector3d> Box::vertexes() const
{
  std::vector<Eigen::Vector3d> points;
  points.reserve(8);
  for (int i = 0; i < 8; i++)
  {
    points.emplace_back((i & 1) ? hi.x() : lo.x(), (i & 2) ? hi.y() : lo.y(), (i & 4) ? hi.z() : lo.z());
  }
  return points;
}

Box centerBox(const Track& track, double t_a, double t_b, const Params& par, const Eigen::Vector3d& margin)
{
  const double tau_a = elapsed(track, t_a);
  const double tau_b = std::max(tau_a, elapsed(track, t_b));

  // SANDO Eq. (11): r = v_max * (time to the end of the interval) [+ epsilon, which is in the margin]. The
  // reachable set only grows with time, so the set at t_b covers the whole interval.
  const Box worst = boxAround(track.pos, track.max_vel * tau_b, margin);
  if (par.mode == Mode::REACHABLE)
  {
    return worst;
  }

  if (!track.is_dynamic)
  {
    return boxAround(track.pos, 0.0, margin);
  }

  // Centre and half-size are piecewise linear in time with a single kink where the prediction stops, so
  // the hull over the interval is the hull of the boxes at its ends and at that kink.
  Box box = boxAround(predictedCenter(track, tau_a, par), par.prediction_growth * tau_a, margin);
  merge(box, boxAround(predictedCenter(track, tau_b, par), par.prediction_growth * tau_b, margin));
  if (tau_a < par.prediction_horizon && par.prediction_horizon < tau_b)
  {
    merge(box, boxAround(predictedCenter(track, par.prediction_horizon, par),
                         par.prediction_growth * par.prediction_horizon, margin));
  }

  // A prediction can never be larger than what the speed bound allows.
  box.lo = box.lo.cwiseMax(worst.lo);
  box.hi = box.hi.cwiseMin(worst.hi);
  if ((box.lo.array() > box.hi.array()).any())
  {
    return worst;
  }
  return box;
}

Box occupancyBox(const Track& track, double t_a, double t_b, const Params& par, const Eigen::Vector3d& extra_margin)
{
  const Eigen::Vector3d margin = 0.5 * track.bbox +
                                 Eigen::Vector3d::Constant(par.obst_position_error + par.base_inflation) +
                                 extra_margin;
  return centerBox(track, t_a, t_b, par, margin);
}

bool Tracker::update(const Observation& obs)
{
  auto it = std::find_if(tracks_.begin(), tracks_.end(), [&](const Track& track) { return track.id == obs.id; });

  if (it == tracks_.end())
  {
    Track track;
    track.id = obs.id;
    track.vel = obs.has_vel ? obs.vel : Eigen::Vector3d::Zero();
    tracks_.push_back(track);
    it = tracks_.end() - 1;
  }
  else
  {
    if (obs.stamp < it->stamp)
    {
      return false;  // out of order
    }

    // Keep only the newest position that is at least velocity_window old, and everything after it.
    while (it->history.size() > 1 && obs.stamp - it->history[1].first >= par_.velocity_window)
    {
      it->history.pop_front();
    }

    if (obs.has_vel)
    {
      it->vel = par_.velocity_filter_alpha * obs.vel + (1.0 - par_.velocity_filter_alpha) * it->vel;
    }
    else
    {
      // Difference against the oldest position kept: the longer the baseline, the less the observation
      // noise matters.
      const double dt = obs.stamp - it->history.front().first;
      if (dt > 1e-4)
      {
        const Eigen::Vector3d measured = (obs.pos - it->history.front().second) / dt;
        // The first difference is the only information there is; later ones are low-pass filtered.
        it->vel = (it->num_observations == 1) ?
                      measured :
                      (par_.velocity_filter_alpha * measured + (1.0 - par_.velocity_filter_alpha) * it->vel);
      }
    }
  }

  it->history.emplace_back(obs.stamp, obs.pos);
  it->stamp = obs.stamp;
  it->pos = obs.pos;
  it->bbox = obs.bbox;
  it->num_observations++;
  // An obstacle seen moving faster than the configured bound shows the bound is wrong for it: use what was
  // measured instead, so that the boxes stay an over-approximation.
  // The bound is per axis (Assumption 1 of the paper).
  it->max_vel = std::max(par_.obst_max_vel, it->vel.lpNorm<Eigen::Infinity>());
  it->is_dynamic = (par_.velocity_threshold <= 0.0) || (it->vel.norm() > par_.velocity_threshold);
  return true;
}

std::vector<int> Tracker::prune(double now)
{
  std::vector<int> removed;
  for (const Track& track : tracks_)
  {
    if (now - track.stamp > par_.track_timeout)
    {
      removed.push_back(track.id);
    }
  }
  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                               [&](const Track& track) { return now - track.stamp > par_.track_timeout; }),
                tracks_.end());
  return removed;
}

bool Tracker::get(int id, Track& track) const
{
  auto it = std::find_if(tracks_.begin(), tracks_.end(), [&](const Track& item) { return item.id == id; });
  if (it == tracks_.end())
  {
    return false;
  }
  track = *it;
  return true;
}

double heat(const Eigen::Vector3d& q, const std::vector<Track>& tracks, const Params& par)
{
  const double horizon = par.prediction_horizon;        // T_h
  const double tau = par.heat_tau_ratio * horizon;      // tau^d
  const int samples = std::max(1, par.heat_num_samples);  // M_tube

  double hottest = 0.0;
  for (const Track& track : tracks)
  {
    // R_0 = max_i h_i + r_margin, R^d = R_0 + v_max * T_h
    const double base_radius = 0.5 * track.bbox.maxCoeff() + par.base_inflation;
    const double reach_radius = base_radius + track.max_vel * horizon;
    const double base =
        par.heat_alpha_base * falloff((q - track.pos).norm(), reach_radius, par.heat_power_base);

    // Tube: max over the sampled prediction of w(t_j) * [1 - |q - mu(t_j)| / (R_0 + gamma t_j)]^q
    const double growth = (par.heat_tube_growth < 0.0) ? track.max_vel : par.heat_tube_growth;
    double tube = 0.0;
    for (int j = 0; j < samples; j++)
    {
      const double t_j = (samples > 1) ? horizon * j / (samples - 1.0) : 0.0;
      const double weight = (tau > 0.0) ? std::exp(-t_j / tau) : (t_j > 0.0 ? 0.0 : 1.0);
      const Eigen::Vector3d center = track.pos + track.vel * t_j;  // constant-velocity prediction mu(t_j)
      tube = std::max(tube, weight * falloff((q - center).norm(), base_radius + growth * t_j, par.heat_power_tube));
    }

    hottest = std::max(hottest, base + par.heat_alpha_tube * tube);  // max over obstacles
  }
  return std::min(hottest, par.heat_max);
}

bool hoverEvasionGoal(const Eigen::Vector3d& pos, double now, const std::vector<Track>& tracks, const Params& par,
                      Eigen::Vector3d& goal)
{
  Eigen::Vector3d repulsion = Eigen::Vector3d::Zero();
  Eigen::Vector3d sidestep = Eigen::Vector3d::UnitX();
  bool triggered = false;

  for (const Track& track : tracks)
  {
    if (!track.is_dynamic)
    {
      continue;
    }
    const double tau = elapsed(track, now);
    const Eigen::Vector3d from = predictedCenter(track, tau, par);
    const Eigen::Vector3d to = from + track.vel * par.prediction_horizon;
    Eigen::Vector3d away = pos - closestPointOnSegment(pos, from, to);
    if (par.hover_avoidance_2d)
    {
      away.z() = 0.0;
    }
    const double distance = away.norm() - 0.5 * track.bbox.norm();
    if (distance > par.hover_trigger_dist)
    {
      continue;
    }

    Eigen::Vector3d heading = track.vel;
    if (par.hover_avoidance_2d)
    {
      heading.z() = 0.0;
    }
    Eigen::Vector3d perpendicular(-heading.y(), heading.x(), 0.0);
    perpendicular = (perpendicular.norm() > 1e-6) ? perpendicular.normalized() : Eigen::Vector3d::UnitX();
    if (!triggered)
    {
      sidestep = perpendicular;  // used when the repulsions cancel out
    }
    triggered = true;

    // Backing away along the path of an obstacle that is coming at us only delays the encounter, so for an
    // approaching obstacle keep only the part of the repulsion that leaves its line of motion.
    Eigen::Vector3d direction = away;
    if (heading.norm() > 1e-6 && heading.dot(pos - from) > 0.0)
    {
      const Eigen::Vector3d along = heading.normalized();
      const Eigen::Vector3d lateral = away - away.dot(along) * along;
      direction = (lateral.norm() > 1e-3) ? lateral : perpendicular;
    }
    if (direction.norm() > 1e-6)
    {
      repulsion += direction.normalized() / std::max(distance, 0.1);
    }
  }

  if (!triggered)
  {
    return false;
  }
  if (par.hover_avoidance_2d)
  {
    repulsion.z() = 0.0;
  }
  const Eigen::Vector3d direction =
      (repulsion.norm() < par.hover_min_repulsion_norm) ? sidestep : repulsion.normalized();
  goal = pos + par.hover_evasion_dist * direction;
  return true;
}

}  // namespace dynobs
