// PointCloud2 -> PCL conversion fused with validity / range / ROI filtering,
// followed by optional voxel-grid down-sampling.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "helios_obstacle_detection/types.hpp"

namespace helios_obstacle_detection
{

struct AxisBox
{
  float min_x{0.F}, max_x{0.F};
  float min_y{0.F}, max_y{0.F};
  float min_z{0.F}, max_z{0.F};

  bool contains(float x, float y, float z) const noexcept
  {
    return x >= min_x && x <= max_x && y >= min_y && y <= max_y && z >= min_z && z <= max_z;
  }
};

struct CloudFilterConfig
{
  float min_range{0.5F};          // [m] 3D range from sensor origin
  float max_range{40.0F};         // [m]
  AxisBox roi{-40.F, 40.F, -40.F, 40.F, -3.F, 3.F};
  bool self_filter_enabled{false};
  AxisBox self_box{};             // points INSIDE are removed (vehicle body / mount)
  float voxel_leaf_size{0.10F};   // [m] <= 0 disables voxel filtering

  /// Throws std::invalid_argument describing the first inconsistent value.
  void validate() const;
};

struct FilterStats
{
  static constexpr int kAzimuthSectors = 36;    // 10 deg sectors for coverage analysis
  static constexpr std::size_t kMinPointsPerSector = 5;
  std::size_t input{0};
  std::size_t non_finite{0};
  std::size_t range_rejected{0};
  std::size_t roi_rejected{0};
  std::size_t kept{0};
  int azimuth_sectors_covered{0};  // sectors with >= kMinPointsPerSector in-range returns
};

/// Byte layout of the fields we need inside a PointCloud2.
struct FieldLayout
{
  std::uint32_t x{0}, y{0}, z{0};
  bool has_intensity{false};
  std::uint32_t intensity{0};
  std::uint8_t intensity_type{0};  // sensor_msgs::msg::PointField datatype
};

class CloudProcessor
{
public:
  explicit CloudProcessor(const CloudFilterConfig & config);

  /// Inspects the PointCloud2 fields. Returns false and fills `error` when the layout
  /// cannot be used (missing / non-float32 xyz, big-endian, truncated buffer ...).
  static bool resolveLayout(
    const sensor_msgs::msg::PointCloud2 & msg, FieldLayout & layout, std::string & error);

  /// Single pass over the raw buffer: drops NaN/Inf, applies range limits, ROI and
  /// self-filter, writing surviving points directly into `out` (no intermediate full cloud).
  bool convertAndFilter(
    const sensor_msgs::msg::PointCloud2 & msg, Cloud & out, FilterStats & stats,
    std::string & error) const;

  /// Voxel-grid down-sampling. When disabled, returns `in` itself (no copy).
  CloudConstPtr voxelDownsample(const CloudConstPtr & in) const;
  /// Same with an explicit leaf size (used by the load-adaptive controller); <= 0 disables.
  CloudConstPtr voxelDownsample(const CloudConstPtr & in, float leaf) const;

  const CloudFilterConfig & config() const noexcept {return config_;}

private:
  CloudFilterConfig config_;
  float min_range_sq_;
  float max_range_sq_;
};

}  // namespace helios_obstacle_detection
