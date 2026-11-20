#include "helios_obstacle_detection/cloud_processor.hpp"

#include <array>
#include <cmath>
#include <cstring>
#include <stdexcept>

#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/msg/point_field.hpp>

#include "helios_obstacle_detection/sensor_health.hpp"

namespace helios_obstacle_detection
{

using sensor_msgs::msg::PointField;

namespace
{

void checkBox(const AxisBox & b, const char * name)
{
  if (!(b.min_x < b.max_x && b.min_y < b.max_y && b.min_z < b.max_z)) {
    throw std::invalid_argument(std::string(name) + ": every min must be < max");
  }
}

const PointField * findField(const sensor_msgs::msg::PointCloud2 & msg, const char * name)
{
  for (const auto & f : msg.fields) {
    if (f.name == name) {
      return &f;
    }
  }
  return nullptr;
}

std::uint32_t datatypeSize(std::uint8_t datatype)
{
  switch (datatype) {
    case PointField::INT8:
    case PointField::UINT8: return 1;
    case PointField::INT16:
    case PointField::UINT16: return 2;
    case PointField::INT32:
    case PointField::UINT32:
    case PointField::FLOAT32: return 4;
    case PointField::FLOAT64: return 8;
    default: return 0;
  }
}

inline float readFloat(const std::uint8_t * p) noexcept
{
  float v;
  std::memcpy(&v, p, sizeof(float));
  return v;
}

inline float readIntensity(const std::uint8_t * p, std::uint8_t type) noexcept
{
  switch (type) {
    case PointField::FLOAT32: return readFloat(p);
    case PointField::UINT8: return static_cast<float>(*p);
    case PointField::UINT16: {
        std::uint16_t v;
        std::memcpy(&v, p, sizeof(v));
        return static_cast<float>(v);
      }
    case PointField::FLOAT64: {
        double v;
        std::memcpy(&v, p, sizeof(v));
        return static_cast<float>(v);
      }
    default: return 0.F;
  }
}

}  // namespace

void CloudFilterConfig::validate() const
{
  if (!(min_range >= 0.F && max_range > min_range)) {
    throw std::invalid_argument("range: require 0 <= min_range < max_range");
  }
  checkBox(roi, "roi");
  if (self_filter_enabled) {
    checkBox(self_box, "self_filter");
  }
  if (voxel_leaf_size > 0.F && voxel_leaf_size < 0.01F) {
    throw std::invalid_argument("voxel_leaf_size: must be 0 (disabled) or >= 0.01 m");
  }
  // pcl::VoxelGrid uses int32 voxel indices; guard against overflow for the ROI extent.
  if (voxel_leaf_size > 0.F) {
    const double nx = std::ceil((roi.max_x - roi.min_x) / voxel_leaf_size) + 1;
    const double ny = std::ceil((roi.max_y - roi.min_y) / voxel_leaf_size) + 1;
    const double nz = std::ceil((roi.max_z - roi.min_z) / voxel_leaf_size) + 1;
    if (nx * ny * nz > 2.0e9) {
      throw std::invalid_argument("voxel_leaf_size too small for the ROI (int32 overflow)");
    }
  }
}

CloudProcessor::CloudProcessor(const CloudFilterConfig & config)
: config_(config),
  min_range_sq_(config.min_range * config.min_range),
  max_range_sq_(config.max_range * config.max_range)
{
  config_.validate();
}

bool CloudProcessor::resolveLayout(
  const sensor_msgs::msg::PointCloud2 & msg, FieldLayout & layout, std::string & error)
{
  if (msg.is_bigendian) {
    error = "big-endian PointCloud2 not supported";
    return false;
  }
  const char * axes[3] = {"x", "y", "z"};
  std::uint32_t * offsets[3] = {&layout.x, &layout.y, &layout.z};
  for (int i = 0; i < 3; ++i) {
    const PointField * f = findField(msg, axes[i]);
    if (f == nullptr) {
      error = std::string("missing field '") + axes[i] + "'";
      return false;
    }
    if (f->datatype != PointField::FLOAT32 || f->count != 1) {
      error = std::string("field '") + axes[i] + "' must be FLOAT32 x1";
      return false;
    }
    if (f->offset + 4 > msg.point_step) {
      error = std::string("field '") + axes[i] + "' exceeds point_step";
      return false;
    }
    *offsets[i] = f->offset;
  }
  layout.has_intensity = false;
  if (const PointField * f = findField(msg, "intensity")) {
    const std::uint32_t sz = datatypeSize(f->datatype);
    const bool supported = f->datatype == PointField::FLOAT32 ||
      f->datatype == PointField::UINT8 || f->datatype == PointField::UINT16 ||
      f->datatype == PointField::FLOAT64;
    if (supported && f->offset + sz <= msg.point_step) {
      layout.has_intensity = true;
      layout.intensity = f->offset;
      layout.intensity_type = f->datatype;
    }
  }
  const std::size_t needed =
    static_cast<std::size_t>(msg.row_step) * static_cast<std::size_t>(msg.height);
  if (msg.point_step == 0 || msg.row_step < msg.point_step * msg.width ||
    msg.data.size() < needed)
  {
    error = "inconsistent point_step/row_step/data size";
    return false;
  }
  return true;
}

bool CloudProcessor::convertAndFilter(
  const sensor_msgs::msg::PointCloud2 & msg, Cloud & out, FilterStats & stats,
  std::string & error) const
{
  stats = FilterStats{};
  out.clear();
  FieldLayout layout;
  if (!resolveLayout(msg, layout, error)) {
    return false;
  }

  const std::size_t total = static_cast<std::size_t>(msg.width) * msg.height;
  stats.input = total;
  out.reserve(total);

  std::array<std::uint32_t, FilterStats::kAzimuthSectors> sector_count{};
  constexpr float kSectorScale =
    static_cast<float>(FilterStats::kAzimuthSectors) / (2.F * static_cast<float>(M_PI));
  const std::uint8_t * base = msg.data.data();
  for (std::uint32_t row = 0; row < msg.height; ++row) {
    const std::uint8_t * p = base + static_cast<std::size_t>(row) * msg.row_step;
    for (std::uint32_t col = 0; col < msg.width; ++col, p += msg.point_step) {
      const float x = readFloat(p + layout.x);
      const float y = readFloat(p + layout.y);
      const float z = readFloat(p + layout.z);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        ++stats.non_finite;
        continue;
      }
      const float r2 = x * x + y * y + z * z;
      if (r2 < min_range_sq_ || r2 > max_range_sq_) {
        ++stats.range_rejected;
        continue;
      }
      {
        int sec = static_cast<int>((fastAtan2(y, x) + static_cast<float>(M_PI)) * kSectorScale);
        sec = std::min(std::max(sec, 0), FilterStats::kAzimuthSectors - 1);
        ++sector_count[static_cast<std::size_t>(sec)];
      }
      if (!config_.roi.contains(x, y, z) ||
        (config_.self_filter_enabled && config_.self_box.contains(x, y, z)))
      {
        ++stats.roi_rejected;
        continue;
      }
      PointT pt;
      pt.x = x;
      pt.y = y;
      pt.z = z;
      pt.intensity = layout.has_intensity ? readIntensity(p + layout.intensity,
          layout.intensity_type) : 0.F;
      out.push_back(pt);  // updates width/height (unorganised cloud)
    }
  }
  out.is_dense = true;  // every non-finite point removed above
  stats.kept = out.size();
  for (const auto c : sector_count) {
    if (c >= FilterStats::kMinPointsPerSector) {
      ++stats.azimuth_sectors_covered;
    }
  }
  return true;
}

CloudConstPtr CloudProcessor::voxelDownsample(const CloudConstPtr & in) const
{
  return voxelDownsample(in, config_.voxel_leaf_size);
}

CloudConstPtr CloudProcessor::voxelDownsample(const CloudConstPtr & in, float leaf) const
{
  if (leaf <= 0.F || in->empty()) {
    return in;
  }
  auto out = std::make_shared<Cloud>();
  pcl::VoxelGrid<PointT> voxel;
  voxel.setInputCloud(in);
  voxel.setLeafSize(leaf, leaf, leaf);
  voxel.filter(*out);
  out->header = in->header;
  return out;
}

}  // namespace helios_obstacle_detection
