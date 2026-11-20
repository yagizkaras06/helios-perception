#include "helios_obstacle_detection/ground_segmenter.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <memory>
#include <sstream>
#include <stdexcept>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace helios_obstacle_detection
{

namespace
{

constexpr float kDegToRad = static_cast<float>(M_PI / 180.0);

/// z of `plane` at horizontal position (x, y). Requires plane normal z-component > 0.
inline float planeHeightAt(const Eigen::Vector4f & plane, float x, float y) noexcept
{
  return -(plane[0] * x + plane[1] * y + plane[3]) / plane[2];
}

inline float signedDistance(const Eigen::Vector4f & plane, const PointT & p) noexcept
{
  return plane[0] * p.x + plane[1] * p.y + plane[2] * p.z + plane[3];
}

}  // namespace

const char * toString(GroundStatus s) noexcept
{
  switch (s) {
    case GroundStatus::kReliable: return "RELIABLE";
    case GroundStatus::kDegraded: return "DEGRADED";
    case GroundStatus::kFailed: return "FAILED";
  }
  return "UNKNOWN";
}

void GroundConfig::validate()
{
  const float norm = expected_normal.norm();
  if (!(norm > 1e-6F)) {
    throw std::invalid_argument("ground.expected_normal must be non-zero");
  }
  expected_normal /= norm;
  if (expected_normal.z() < 0.7F) {
    throw std::invalid_argument(
            "ground.expected_normal must be within ~45 deg of sensor +Z (z component >= 0.7)");
  }
  if (!(max_normal_angle_deg > 0.F && max_normal_angle_deg < 45.F)) {
    throw std::invalid_argument("ground.max_normal_angle_deg must be in (0, 45)");
  }
  if (!(height_tolerance > 0.F && max_zone_height_jump > 0.F && candidate_above > 0.F &&
    candidate_below > 0.F && ransac_distance > 0.F && ground_threshold > 0.F &&
    below_ground_tolerance >= ground_threshold))
  {
    throw std::invalid_argument(
            "ground: tolerances must be > 0 and below_ground_tolerance >= ground_threshold");
  }
  if (!(max_obstacle_height > ground_threshold)) {
    throw std::invalid_argument("ground.max_obstacle_height must be > ground_threshold");
  }
  if (ransac_max_iterations < 10 || max_plane_attempts < 1 || min_zone_inliers < 3) {
    throw std::invalid_argument(
            "ground: ransac_max_iterations>=10, max_plane_attempts>=1, min_zone_inliers>=3");
  }
  if (zone_boundaries.size() < 2 || zone_boundaries.size() > 64) {
    throw std::invalid_argument("ground.zone_boundaries needs 2..64 values");
  }
  for (std::size_t i = 1; i < zone_boundaries.size(); ++i) {
    if (!(zone_boundaries[i] > zone_boundaries[i - 1])) {
      throw std::invalid_argument("ground.zone_boundaries must be strictly increasing");
    }
  }
  if (zone_boundaries.front() < 0.F) {
    throw std::invalid_argument("ground.zone_boundaries must be >= 0");
  }
}

GroundSegmenter::GroundSegmenter(const GroundConfig & config)
: config_(config)
{
  config_.validate();
  expected_plane_ << config_.expected_normal, -config_.expected_height;
}

int GroundSegmenter::zoneOf(float r) const noexcept
{
  const auto & b = config_.zone_boundaries;
  const int last_zone = static_cast<int>(b.size()) - 2;
  for (int i = 0; i <= last_zone; ++i) {
    if (r < b[static_cast<std::size_t>(i) + 1]) {
      return i;
    }
  }
  return last_zone;  // beyond last boundary: clamp to outermost zone
}

bool GroundSegmenter::fitZone(
  const CloudConstPtr & cloud, pcl::Indices candidates, const Eigen::Vector4f & reference,
  float reference_tolerance, ZonePlane & plane) const
{
  plane.candidates = candidates.size();
  plane.rejected_planes = 0;

  pcl::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setAxis(config_.expected_normal);
  seg.setEpsAngle(config_.max_normal_angle_deg * kDegToRad);
  seg.setDistanceThreshold(config_.ransac_distance);
  seg.setMaxIterations(config_.ransac_max_iterations);
  seg.setInputCloud(cloud);

  const float cos_max = std::cos(config_.max_normal_angle_deg * kDegToRad);
  const auto min_inliers = static_cast<std::size_t>(config_.min_zone_inliers);

  for (int attempt = 0; attempt < config_.max_plane_attempts; ++attempt) {
    if (candidates.size() < min_inliers) {
      break;
    }
    auto idx = std::make_shared<pcl::Indices>(candidates);
    seg.setIndices(idx);
    pcl::PointIndices inliers;
    pcl::ModelCoefficients coeffs;
    seg.segment(inliers, coeffs);
    if (inliers.indices.size() < min_inliers || coeffs.values.size() != 4) {
      break;
    }

    Eigen::Vector4f c(coeffs.values[0], coeffs.values[1], coeffs.values[2], coeffs.values[3]);
    const float n = c.head<3>().norm();
    if (n < 1e-6F) {
      break;
    }
    c /= n;
    if (c.head<3>().dot(config_.expected_normal) < 0.F) {
      c = -c;
    }

    // 1) orientation (re-checked: coefficient optimisation can drift beyond eps angle)
    bool valid = c.head<3>().dot(config_.expected_normal) >= cos_max;

    // 2) height consistency with reference plane, evaluated at the inlier centroid
    if (valid) {
      double sx = 0.0, sy = 0.0;
      for (const auto i : inliers.indices) {
        sx += (*cloud)[static_cast<std::size_t>(i)].x;
        sy += (*cloud)[static_cast<std::size_t>(i)].y;
      }
      const auto k = static_cast<double>(inliers.indices.size());
      const auto cx = static_cast<float>(sx / k);
      const auto cy = static_cast<float>(sy / k);
      valid = std::fabs(planeHeightAt(c, cx, cy) - planeHeightAt(reference, cx, cy)) <=
        reference_tolerance;
    }

    if (valid) {
      plane.coefficients = c;
      plane.state = ZonePlane::State::kValidated;
      plane.inliers = inliers.indices.size();
      return true;
    }

    // Rejected (e.g. table top / platform / wrong height): drop its support and retry.
    ++plane.rejected_planes;
    std::sort(inliers.indices.begin(), inliers.indices.end());
    pcl::Indices remaining;
    remaining.reserve(candidates.size());
    std::set_difference(
      candidates.begin(), candidates.end(), inliers.indices.begin(), inliers.indices.end(),
      std::back_inserter(remaining));
    candidates.swap(remaining);
  }
  return false;
}

void GroundSegmenter::segmentConst(const CloudConstPtr & cloud, GroundResult & result) const
{
  const std::size_t zone_count = config_.zone_boundaries.size() - 1;
  result = GroundResult{};
  result.zones.assign(zone_count, ZonePlane{});

  // --- collect per-zone candidates near the expected ground (indices stay sorted) ---
  std::vector<pcl::Indices> candidates(zone_count);
  std::vector<std::uint8_t> zone_of_point(cloud->size());
  for (std::size_t i = 0; i < cloud->size(); ++i) {
    const PointT & p = (*cloud)[i];
    const int z = zoneOf(std::hypot(p.x, p.y));
    zone_of_point[i] = static_cast<std::uint8_t>(z);
    const float s = signedDistance(expected_plane_, p);
    if (s >= -config_.candidate_below && s <= config_.candidate_above) {
      candidates[static_cast<std::size_t>(z)].push_back(static_cast<pcl::index_t>(i));
    }
  }

  // --- pass 1: validated fit per zone, inner to outer ---
  int last_valid = -1;
  for (std::size_t z = 0; z < zone_count; ++z) {
    const bool has_ref = last_valid >= 0;
    const Eigen::Vector4f & ref =
      has_ref ? result.zones[static_cast<std::size_t>(last_valid)].coefficients : expected_plane_;
    const float tol = has_ref ? config_.max_zone_height_jump : config_.height_tolerance;
    if (fitZone(cloud, std::move(candidates[z]), ref, tol, result.zones[z])) {
      last_valid = static_cast<int>(z);
    }
  }

  // --- pass 2: fill zones without a validated plane from nearest validated zone ---
  std::size_t validated = 0;
  bool degraded = false;
  for (std::size_t z = 0; z < zone_count; ++z) {
    ZonePlane & zp = result.zones[z];
    if (zp.state == ZonePlane::State::kValidated) {
      ++validated;
      continue;
    }
    if (zp.candidates >= static_cast<std::size_t>(config_.min_zone_inliers)) {
      degraded = true;  // there was ground-like data but no plane passed validation
    }
    int best = -1;
    for (std::size_t o = 0; o < zone_count; ++o) {
      if (result.zones[o].state == ZonePlane::State::kValidated &&
        (best < 0 || std::abs(static_cast<int>(o) - static_cast<int>(z)) <
        std::abs(best - static_cast<int>(z))))
      {
        best = static_cast<int>(o);
      }
    }
    if (best >= 0) {
      zp.coefficients = result.zones[static_cast<std::size_t>(best)].coefficients;
      zp.state = ZonePlane::State::kExtrapolated;
    } else {
      zp.coefficients = expected_plane_;
      zp.state = ZonePlane::State::kFallback;
    }
  }
  result.planes_valid = validated;
  result.planes_total = zone_count;
  if (validated == 0) {
    result.status = GroundStatus::kFailed;
  } else {
    result.status = degraded ? GroundStatus::kDegraded : GroundStatus::kReliable;
  }

  // --- classify every point against its zone plane ---
  result.ground.reserve(cloud->size());
  result.obstacle.reserve(cloud->size() / 2);
  for (std::size_t i = 0; i < cloud->size(); ++i) {
    const float h = signedDistance(result.zones[zone_of_point[i]].coefficients, (*cloud)[i]);
    if (h > config_.max_obstacle_height) {
      ++result.ignored_high;
    } else if (h < -config_.below_ground_tolerance) {
      ++result.ignored_low;
    } else if (h <= config_.ground_threshold) {
      result.ground.push_back(static_cast<pcl::index_t>(i));
    } else {
      result.obstacle.push_back(static_cast<pcl::index_t>(i));
    }
  }

  std::ostringstream os;
  os.setf(std::ios::fixed);
  os.precision(2);
  for (std::size_t z = 0; z < zone_count; ++z) {
    const ZonePlane & zp = result.zones[z];
    const char tag = zp.state == ZonePlane::State::kValidated ? 'V' :
      (zp.state == ZonePlane::State::kExtrapolated ? 'E' : 'F');
    os << "z" << z << ":" << tag << " h=" << planeHeightAt(zp.coefficients, 0.F, 0.F)
       << " in=" << zp.inliers << "/" << zp.candidates << " rej=" << zp.rejected_planes;
    if (z + 1 < zone_count) {
      os << " | ";
    }
  }
  result.detail = os.str();
}

}  // namespace helios_obstacle_detection
