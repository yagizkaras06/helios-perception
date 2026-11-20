#include "helios_obstacle_detection/static_map.hpp"

#include <cmath>
#include <stdexcept>

namespace helios_obstacle_detection
{

void StaticMapConfig::validate() const
{
  if (!(voxel_size >= 0.05F && static_time_s > 0.F && min_occupancy > 0.F &&
    min_occupancy <= 1.F && forget_time_s > 0.F))
  {
    throw std::invalid_argument(
            "static_map: voxel_size >= 0.05, static_time_s > 0, min_occupancy in (0,1], "
            "forget_time_s > 0");
  }
}

StaticBackgroundMap::StaticBackgroundMap(const StaticMapConfig & config)
: config_(config)
{
  config_.validate();
  inv_voxel_ = 1.F / config_.voxel_size;
  voxels_.reserve(1 << 15);
}

void StaticBackgroundMap::reset()
{
  voxels_.clear();
  frame_ = 0;
  start_stamp_ = -1.0;
  static_count_ = 0;
}

std::uint64_t StaticBackgroundMap::key(int x, int y, int z) const noexcept
{
  constexpr std::uint64_t kMask = (1ULL << 21) - 1;
  const auto ux = static_cast<std::uint64_t>(x + (1 << 20)) & kMask;
  const auto uy = static_cast<std::uint64_t>(y + (1 << 20)) & kMask;
  const auto uz = static_cast<std::uint64_t>(z + (1 << 20)) & kMask;
  return (ux << 42) | (uy << 21) | uz;
}

bool StaticBackgroundMap::isStatic(const Voxel & v) const noexcept
{
  if (v.last_seen - v.first_seen < config_.static_time_s) {
    return false;
  }
  const double frames = static_cast<double>(v.last_frame - v.first_frame + 1);
  return static_cast<double>(v.hits) >= config_.min_occupancy * frames;
}

bool StaticBackgroundMap::staticAt(int x, int y, int z) const
{
  const auto it = voxels_.find(key(x, y, z));
  return it != voxels_.end() && isStatic(it->second);
}

void StaticBackgroundMap::processFrame(
  const Cloud & cloud, const Eigen::Vector3d & pose, double stamp, pcl::Indices & background,
  pcl::Indices & foreground)
{
  classify(cloud, pose, stamp, background, foreground);
  insert(stamp, nullptr);
}

void StaticBackgroundMap::classify(
  const Cloud & cloud, const Eigen::Vector3d & pose, double stamp, pcl::Indices & background,
  pcl::Indices & foreground)
{
  background.clear();
  foreground.clear();
  ++frame_;
  if (start_stamp_ < 0.0) {
    start_stamp_ = stamp;
  }
  const double c = std::cos(pose[2]), s = std::sin(pose[2]);
  keys_.resize(cloud.size());

  // classify against the map BEFORE inserting this frame
  for (std::size_t i = 0; i < cloud.size(); ++i) {
    const auto & p = cloud[i];
    const double xo = c * p.x - s * p.y + pose[0];
    const double yo = s * p.x + c * p.y + pose[1];
    const int vx = static_cast<int>(std::floor(xo * inv_voxel_));
    const int vy = static_cast<int>(std::floor(yo * inv_voxel_));
    const int vz = static_cast<int>(std::floor(p.z * inv_voxel_));
    keys_[i] = key(vx, vy, vz);
    bool bg = false;
    const auto it = voxels_.find(keys_[i]);
    if (it != voxels_.end() && isStatic(it->second)) {
      bg = true;
    } else if (config_.neighbor_tolerance) {
      for (int dx = -1; dx <= 1 && !bg; ++dx) {
        for (int dy = -1; dy <= 1 && !bg; ++dy) {
          for (int dz = -1; dz <= 1 && !bg; ++dz) {
            bg = (dx || dy || dz) && staticAt(vx + dx, vy + dy, vz + dz);
          }
        }
      }
    }
    (bg ? background : foreground).push_back(static_cast<pcl::index_t>(i));
  }
}

void StaticBackgroundMap::insert(double stamp, const std::vector<std::uint8_t> * dynamic_points)
{
  const bool guided = config_.track_guided && dynamic_points != nullptr &&
    dynamic_points->size() == keys_.size();
  // one hit per voxel and frame
  for (std::size_t i = 0; i < keys_.size(); ++i) {
    auto & v = voxels_[keys_[i]];
    if (guided && (*dynamic_points)[i] && v.hits > 0 && v.last_frame != frame_ && !isStatic(v)) {
      v.hits = 0;   // recently moving object: restart the static timer of this voxel
    }
    if (v.hits == 0) {
      v.first_seen = stamp;
      v.first_frame = frame_;
    }
    if (v.last_frame != frame_) {
      ++v.hits;
      v.last_frame = frame_;
      v.last_seen = stamp;
    }
  }
  keys_.clear();

  // forget voxels not seen for a while (amortised) and count static voxels
  if (frame_ % 10 == 0) {
    static_count_ = 0;
    for (auto it = voxels_.begin(); it != voxels_.end(); ) {
      if (stamp - it->second.last_seen > config_.forget_time_s) {
        it = voxels_.erase(it);
      } else {
        static_count_ += isStatic(it->second);
        ++it;
      }
    }
  }
}

}  // namespace helios_obstacle_detection
