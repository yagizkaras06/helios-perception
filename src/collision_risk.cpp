#include "helios_obstacle_detection/collision_risk.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace helios_obstacle_detection
{

const char * toString(RiskLevel r) noexcept
{
  switch (r) {
    case RiskLevel::kNone: return "NONE";
    case RiskLevel::kLow: return "LOW";
    case RiskLevel::kMedium: return "MEDIUM";
    case RiskLevel::kHigh: return "HIGH";
    case RiskLevel::kCritical: return "CRITICAL";
  }
  return "?";
}

void RiskConfig::validate() const
{
  if (!(ego_radius >= 0.F && min_closing_speed > 0.F && sigma_factor >= 0.F && horizon_s > 0.F))
  {
    throw std::invalid_argument("risk: ego_radius >= 0, min_closing_speed/horizon > 0");
  }
  if (!(critical_ttc_s > 0.F && critical_ttc_s <= high_ttc_s && high_ttc_s <= medium_ttc_s &&
    medium_ttc_s <= horizon_s))
  {
    throw std::invalid_argument("risk: need 0 < critical <= high <= medium <= horizon (TTC)");
  }
  if (!(critical_distance >= 0.F && critical_distance <= high_distance &&
    high_distance <= medium_distance))
  {
    throw std::invalid_argument("risk: need 0 <= critical <= high <= medium (distance)");
  }
}

RiskAssessment assessRisk(const TrackOutput & t, const RiskConfig & cfg)
{
  constexpr float kInf = std::numeric_limits<float>::infinity();
  RiskAssessment r;
  const Eigen::Vector2f p(t.closest_point.x(), t.closest_point.y());
  const float range = p.norm();
  r.clearance = std::max(range - cfg.ego_radius, 0.F);
  r.ttc = r.ttc_pessimistic = kInf;
  r.cpa_time = kInf;
  r.cpa_distance = r.clearance;

  RiskLevel distance_level = RiskLevel::kNone;
  if (r.clearance < cfg.critical_distance) {
    distance_level = RiskLevel::kCritical;
  } else if (r.clearance < cfg.high_distance) {
    distance_level = RiskLevel::kHigh;
  } else if (r.clearance < cfg.medium_distance) {
    distance_level = RiskLevel::kMedium;
  }

  // Velocity of new / large / ego-unknown tracks is not trustworthy: distance-only grade.
  r.ttc_valid = t.velocity_reliable;
  if (!r.ttc_valid) {
    r.level = distance_level;
    return r;
  }

  const Eigen::Vector2f & v = t.relative_velocity;
  const Eigen::Matrix2f & S = t.relative_velocity_cov;
  const float k = cfg.sigma_factor;
  if (range > 1e-3F) {
    const Eigen::Vector2f u = p / range;
    r.closing_speed = -u.dot(v);
    r.closing_speed_sigma = std::sqrt(std::max(u.dot(S * u), 0.F));
    if (r.closing_speed > cfg.min_closing_speed) {
      r.ttc = r.clearance / r.closing_speed;
    }
    const float pess = r.closing_speed + k * r.closing_speed_sigma;
    if (pess > cfg.min_closing_speed) {
      r.ttc_pessimistic = r.clearance / pess;
    }
  }

  // closest point of approach of the box centre, constant relative velocity
  const Eigen::Vector2f c(t.center.x(), t.center.y());
  const float obj_radius = 0.5F * std::hypot(t.size.x(), t.size.y());
  const float v2 = v.squaredNorm();
  float lateral_sigma = 0.F;
  if (v2 > 1e-4F) {
    r.cpa_time = std::max(0.F, -c.dot(v) / v2);
    const Eigen::Vector2f at = c + v * r.cpa_time;
    r.cpa_distance = at.norm() - obj_radius - cfg.ego_radius;
    const Eigen::Vector2f vn = Eigen::Vector2f(-v.y(), v.x()) / std::sqrt(v2);
    lateral_sigma = std::sqrt(std::max(vn.dot(S * vn), 0.F)) * r.cpa_time;
  } else {
    r.cpa_time = 0.F;
    r.cpa_distance = c.norm() - obj_radius - cfg.ego_radius;
  }
  r.collision_course = r.cpa_time <= cfg.horizon_s &&
    (r.cpa_distance - k * lateral_sigma) <= 0.F && std::isfinite(r.ttc_pessimistic);

  RiskLevel ttc_level = RiskLevel::kNone;
  const float tp = r.ttc_pessimistic;
  if (r.collision_course) {
    if (tp < cfg.critical_ttc_s) {
      ttc_level = RiskLevel::kCritical;
    } else if (tp < cfg.high_ttc_s) {
      ttc_level = RiskLevel::kHigh;
    } else if (tp < cfg.medium_ttc_s) {
      ttc_level = RiskLevel::kMedium;
    } else {
      ttc_level = RiskLevel::kLow;
    }
  } else if (tp < cfg.high_ttc_s) {
    ttc_level = RiskLevel::kMedium;   // closing fast but predicted to pass by
  } else if (tp < cfg.horizon_s) {
    ttc_level = RiskLevel::kLow;
  }
  r.level = std::max(distance_level, ttc_level);
  return r;
}

}  // namespace helios_obstacle_detection
