// Static background map ("what stayed still for N seconds is background").
//
// Non-ground points are accumulated in a voxel hash map in the local odometry frame. A voxel is
// STATIC when it has been observed over at least `static_time_s` and was occupied in at least
// `min_occupancy` of the frames since it was first seen (a person pacing through an area does
// not qualify, a wall briefly occluded by a passer-by does). Voxels not seen for `forget_time_s`
// are dropped, so objects that leave stop being background.
//
// Each frame the non-ground cloud is split into background / foreground points BEFORE the map is
// updated with that frame; foreground points are then clustered separately, which also separates
// a person standing next to a wall from the wall.
//
// Track-guided update (v0.3): voxels hit by the points of objects that MOVED within the last
// static_time_s (the clusters their tracks were associated with) restart their static timer, so a person pacing around one spot is not
// absorbed into the background; once an object has been still for static_time_s it is no longer
// guided and becomes background after another static_time_s, i.e. "still for N seconds -> static"
// counts from the end of the motion. Voxels that are already static are never reset (a wall next
// to a person stays background).
//
// Limits: no ray casting (free space is not observed): a spot vacated by a static object stays
// background for up to `forget_time_s`. With a moving sensor the map lives in the LiDAR-only
// odometry frame and inherits its drift; when ego motion is UNKNOWN the map is not used.
#pragma once

#include <cstdint>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <pcl/types.h>

#include "helios_obstacle_detection/types.hpp"

namespace helios_obstacle_detection
{

struct StaticMapConfig
{
  bool enabled{true};
  float voxel_size{0.15F};      // [m]
  float static_time_s{10.0F};   // occupied for this long -> background
  float min_occupancy{0.5F};    // share of frames (since first seen) the voxel was occupied
  float forget_time_s{5.0F};    // not seen for this long -> removed
  bool neighbor_tolerance{true};  // a point is background if any of its 26 neighbours is static
  bool track_guided{true};        // recently moving objects do not accumulate static time

  void validate() const;
};

class StaticBackgroundMap
{
public:
  explicit StaticBackgroundMap(const StaticMapConfig & config);

  /// Splits `cloud` (sensor frame) into background / foreground indices using the map state
  /// before this frame, then inserts the frame. `pose` = (x, y, yaw) of the sensor in odom.
  void processFrame(
    const Cloud & cloud, const Eigen::Vector3d & pose, double stamp, pcl::Indices & background,
    pcl::Indices & foreground);

  /// Two-step form: classify() before clustering, insert() after tracking, so that the points
  /// of recently moving objects (`dynamic_points[i] != 0`, optional) do not accumulate static
  /// time. Both calls must use the same cloud.
  void classify(
    const Cloud & cloud, const Eigen::Vector3d & pose, double stamp, pcl::Indices & background,
    pcl::Indices & foreground);
  void insert(double stamp, const std::vector<std::uint8_t> * dynamic_points = nullptr);

  void reset();

  std::size_t voxelCount() const noexcept {return voxels_.size();}
  std::size_t staticVoxelCount() const noexcept {return static_count_;}
  /// Seconds since the map started (0 before the first frame): < static_time_s -> still learning.
  double learningAge(double stamp) const noexcept
  {
    return start_stamp_ < 0.0 ? 0.0 : stamp - start_stamp_;
  }
  const StaticMapConfig & config() const noexcept {return config_;}

private:
  struct Voxel
  {
    double first_seen{0.0};
    double last_seen{0.0};
    std::uint64_t first_frame{0};
    std::uint64_t last_frame{0};
    std::uint32_t hits{0};
  };

  std::uint64_t key(int x, int y, int z) const noexcept;
  bool isStatic(const Voxel & v) const noexcept;
  bool staticAt(int x, int y, int z) const;

  StaticMapConfig config_;
  float inv_voxel_;
  std::vector<std::uint64_t> keys_;     // voxel keys of the classified frame
  std::unordered_map<std::uint64_t, Voxel> voxels_;
  std::uint64_t frame_{0};
  double start_stamp_{-1.0};
  std::size_t static_count_{0};
};

}  // namespace helios_obstacle_detection
