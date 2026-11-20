// Time-to-collision / closest-point-of-approach and a perception-level risk grade.
//
// Everything is relative to a circle of radius `ego_radius` around the LiDAR origin (the vehicle
// footprint is unknown to this package). Inputs are the track's closest point and its velocity
// RELATIVE to the sensor (object velocity minus ego velocity, both from LiDAR only) with their
// covariance, so the uncertainty of the ego-motion estimate widens the bounds.
//
//   closing speed   c  = -u . v_rel        (u = unit vector sensor -> closest point)
//   sigma_c            = sqrt(u^T Sigma_rel u)
//   TTC nominal        = clearance / c                 (c > min_closing_speed)
//   TTC pessimistic    = clearance / (c + k sigma_c)   (worst case within k sigma)
//   CPA                = constant-velocity closest approach of the box centre; collision course
//                        if the k-sigma lateral band at t_cpa overlaps ego + object radius.
// The grade is NOT a control command; nothing in this package actuates the vehicle.
#pragma once

#include <cstdint>
#include <vector>

#include "helios_obstacle_detection/multi_object_tracker.hpp"

namespace helios_obstacle_detection
{

enum class RiskLevel : std::uint8_t {kNone = 0, kLow = 1, kMedium = 2, kHigh = 3, kCritical = 4};

const char * toString(RiskLevel r) noexcept;

struct RiskConfig
{
  float ego_radius{0.5F};           // [m] footprint approximated as a circle around the LiDAR
  float min_closing_speed{0.2F};    // [m/s]
  float sigma_factor{2.0F};
  float horizon_s{8.0F};
  float critical_ttc_s{1.5F};
  float high_ttc_s{3.0F};
  float medium_ttc_s{5.0F};
  float critical_distance{0.5F};    // [m] clearance (distance minus ego_radius)
  float high_distance{1.0F};
  float medium_distance{2.5F};

  void validate() const;
};

struct RiskAssessment
{
  bool ttc_valid{false};            // velocity of the track is usable
  float clearance{0.F};
  float closing_speed{0.F};
  float closing_speed_sigma{0.F};
  float ttc{0.F};                   // +inf when not closing
  float ttc_pessimistic{0.F};
  float cpa_time{0.F};
  float cpa_distance{0.F};          // signed gap at CPA (negative = overlap)
  bool collision_course{false};
  RiskLevel level{RiskLevel::kNone};
};

RiskAssessment assessRisk(const TrackOutput & track, const RiskConfig & config);

}  // namespace helios_obstacle_detection
