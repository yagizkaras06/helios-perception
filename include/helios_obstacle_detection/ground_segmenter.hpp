// Zone-wise, geometrically validated RANSAC ground segmentation.
//
// The horizontal plane around the sensor is split into concentric range zones. In each zone
// candidate points close to the *expected* ground (mount height + nominal normal) are fed to a
// RANSAC plane fit constrained to be roughly perpendicular to the expected normal. Every plane
// found is validated (normal angle, height vs. reference plane, inlier support). Rejected planes
// (e.g. table tops, platforms) are removed from the candidate set and RANSAC is retried, so the
// largest plane is NOT automatically taken as ground. Zones without a valid plane borrow the
// nearest valid zone plane (DEGRADED). If no zone validates, status is FAILED and the configured
// expected plane is used as an explicit, reported fallback.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <pcl/types.h>

#include "helios_obstacle_detection/types.hpp"

namespace helios_obstacle_detection
{

enum class GroundStatus : std::uint8_t
{
  kReliable = 0,
  kDegraded = 1,
  kFailed = 2,
};

const char * toString(GroundStatus s) noexcept;

struct GroundConfig
{
  Eigen::Vector3f expected_normal{0.F, 0.F, 1.F};  // sensor frame, normalised in validate()
  float expected_height{-0.9F};      // [m] signed offset of ground along normal (z of ground)
  float height_tolerance{0.35F};     // [m] zone plane vs. expected plane (first validated zone)
  float max_zone_height_jump{0.30F}; // [m] zone plane vs. neighbouring validated zone plane
  float max_normal_angle_deg{15.F};  // [deg] plane normal vs. expected normal
  float candidate_above{0.40F};      // [m] candidate band above expected plane
  float candidate_below{0.80F};      // [m] candidate band below expected plane (down slopes)
  float ransac_distance{0.08F};      // [m] RANSAC inlier threshold
  int ransac_max_iterations{150};
  int max_plane_attempts{4};         // RANSAC retries per zone after plane rejection
  int min_zone_inliers{60};
  std::vector<float> zone_boundaries{0.F, 8.F, 16.F, 40.F};  // [m] horizontal range
  float ground_threshold{0.12F};     // [m] |height| <= threshold -> ground
  float max_obstacle_height{2.5F};   // [m] points higher above ground are ignored (ceilings)
  float below_ground_tolerance{0.30F};  // [m] points deeper than this below plane are ignored

  void validate();  // throws std::invalid_argument; normalises expected_normal
};

struct ZonePlane
{
  enum class State : std::uint8_t {kValidated, kExtrapolated, kFallback};
  Eigen::Vector4f coefficients{0.F, 0.F, 1.F, 0.F};  // n·p + d = 0, |n| = 1, n·expected > 0
  State state{State::kFallback};
  std::size_t inliers{0};
  std::size_t candidates{0};
  int rejected_planes{0};
};

struct GroundResult
{
  GroundStatus status{GroundStatus::kFailed};
  std::vector<ZonePlane> zones;      // RANSAC zones only (empty for CZM)
  std::size_t planes_valid{0};       // validated planes (zones or CZM bins)
  std::size_t planes_total{0};       // zones / populated bins considered
  pcl::Indices ground;
  pcl::Indices obstacle;
  std::size_t ignored_high{0};
  std::size_t ignored_low{0};
  std::string detail;  // compact per-zone summary for diagnostics
};

/// Common interface so the pipeline can switch between segmentation methods.
class GroundSegmenterBase
{
public:
  virtual ~GroundSegmenterBase() = default;
  /// `result` is fully overwritten. Not thread-safe (implementations may keep adaptive state).
  virtual void segment(const CloudConstPtr & cloud, GroundResult & result) = 0;
  /// Forget temporal / adaptive state (e.g. after a time jump in the input).
  virtual void reset() {}
  virtual const char * name() const noexcept = 0;
};

/// Zone-wise validated RANSAC (original method, kept as `ground.method: ransac_zones`).
class GroundSegmenter : public GroundSegmenterBase
{
public:
  explicit GroundSegmenter(const GroundConfig & config);

  void segment(const CloudConstPtr & cloud, GroundResult & result) override
  {
    segmentConst(cloud, result);
  }
  /// Stateless variant (thread-compatible).
  void segmentConst(const CloudConstPtr & cloud, GroundResult & result) const;
  const char * name() const noexcept override {return "ransac_zones";}

  const GroundConfig & config() const noexcept {return config_;}

private:
  int zoneOf(float horizontal_range) const noexcept;
  bool fitZone(
    const CloudConstPtr & cloud, pcl::Indices candidates, const Eigen::Vector4f & reference,
    float reference_tolerance, ZonePlane & plane) const;

  GroundConfig config_;
  Eigen::Vector4f expected_plane_;
};

}  // namespace helios_obstacle_detection
