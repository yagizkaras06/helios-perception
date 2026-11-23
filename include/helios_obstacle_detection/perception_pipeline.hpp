// ROS-free perception pipeline (one object per input stream). The ROS node, the offline bag
// benchmark and the tests all run exactly this code.
//
//   PointCloud2 --convertAndFilter--> ROI cloud --voxel (load adaptive)--> ground segmentation
//     --> non-ground cloud --+--> clustering --> AABB / OBB ---------------+
//                            +--> ego-motion registration (pool task) ----+--> tracker --> risk
// With the static background map enabled: ego-motion first, then the non-ground cloud is split
// into background (still for >= static_time_s) and foreground points, clustered separately.
//
// Not thread-safe: call process() from one thread. Internally uses a thread pool.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <pcl/PointIndices.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "helios_obstacle_detection/bounding_box.hpp"
#include "helios_obstacle_detection/cloud_processor.hpp"
#include "helios_obstacle_detection/collision_risk.hpp"
#include "helios_obstacle_detection/collision_warning.hpp"
#include "helios_obstacle_detection/czm_ground_segmenter.hpp"
#include "helios_obstacle_detection/ego_motion_estimator.hpp"
#include "helios_obstacle_detection/ground_segmenter.hpp"
#include "helios_obstacle_detection/multi_object_tracker.hpp"
#include "helios_obstacle_detection/obstacle_clusterer.hpp"
#include "helios_obstacle_detection/static_map.hpp"
#include "helios_obstacle_detection/thread_pool.hpp"

namespace helios_obstacle_detection
{

/// Load-adaptive voxel size: keeps the pipeline inside its time budget on slower machines or
/// very dense scenes by coarsening the voxel grid, and returns to nominal when there is margin.
struct AdaptiveConfig
{
  bool enabled{true};
  float target_ms{50.F};       // mean pipeline time above this -> coarser voxels
  float max_leaf{0.20F};       // [m]
  float leaf_step{0.02F};      // [m]
  int window{10};              // frames per decision

  void validate(float nominal_leaf) const;
};

enum class EgoMode : std::uint8_t
{
  kEstimate = 0,          // LiDAR-only registration (default)
  kAssumeStationary = 1,  // sensor known to be fixed (test bench): ego velocity = 0
  kExternal = 2,          // evaluation only: ego motion injected with setExternalEgo()
};

struct PipelineConfig
{
  CloudFilterConfig filter;
  std::string ground_method{"czm"};   // czm | ransac_zones
  GroundConfig ground;                // RANSAC method + shared thresholds
  CzmConfig czm;                      // czm.base is overwritten with `ground`
  ClusterConfig cluster;
  BoxFilterConfig boxes;
  EgoMode ego_mode{EgoMode::kEstimate};
  EgoMotionConfig ego;
  TrackerConfig tracker;
  StaticMapConfig static_map;         // background model ("still for N s -> static")
  RiskConfig risk;
  WarningConfig warning;              // operator collision warning (display only)
  AdaptiveConfig adaptive;
  int threads{2};                     // pool workers (0 = auto: cores - 1)

  void validate();
};

struct StageTimes
{
  double roi_ms{0}, voxel_ms{0}, ground_ms{0}, clustering_ms{0}, boxes_ms{0};
  double ego_ms{0}, tracking_ms{0}, total_ms{0};
};

/// Result of one frame. Non-copyable: `obstacles[i].indices` point into `clusters`.
struct FrameOutput
{
  FrameOutput() = default;
  FrameOutput(const FrameOutput &) = delete;
  FrameOutput & operator=(const FrameOutput &) = delete;
  FrameOutput(FrameOutput &&) = default;
  FrameOutput & operator=(FrameOutput &&) = default;

  double stamp{0.0};
  FilterStats filter;
  CloudPtr roi_cloud;
  CloudConstPtr voxel_cloud;
  GroundResult ground;
  CloudPtr obstacle_cloud;
  std::vector<pcl::PointIndices> clusters;
  std::vector<ObstacleCandidate> obstacles;
  EgoMotion ego;
  std::vector<TrackOutput> tracks;
  std::vector<RiskAssessment> risks;  // parallel to tracks
  RiskLevel max_risk{RiskLevel::kNone};
  int max_risk_track{-1};             // index into tracks
  CollisionWarning warning;           // operator collision warning (hysteresis applied)
  TrackerStats tracker_stats;
  std::size_t background_points{0};   // non-ground points classified as static background
  std::size_t static_voxels{0};
  bool background_learning{false};    // map younger than static_time_s
  StageTimes times;
  float voxel_leaf{0.F};
};

class PerceptionPipeline
{
public:
  explicit PerceptionPipeline(const PipelineConfig & config);

  /// Full pipeline from a driver message. Returns false (with `error`) for an unusable layout.
  bool process(
    const sensor_msgs::msg::PointCloud2 & msg, double stamp, FrameOutput & out,
    std::string & error);

  /// Pipeline from an already converted / filtered cloud (simulation, tests).
  void processFiltered(const CloudPtr & roi_cloud, double stamp, FrameOutput & out);

  /// Clears tracker, ego-motion reference and adaptive state (time jump, data gap).
  void reset();

  /// EgoMode::kExternal only (benchmarks with ground-truth ego motion).
  void setExternalEgo(const EgoMotion & ego) {external_ego_ = ego;}

  const PipelineConfig & config() const noexcept {return config_;}
  ThreadPool & pool() noexcept {return *pool_;}
  float currentVoxelLeaf() const noexcept {return voxel_leaf_;}
  const char * groundMethod() const noexcept {return ground_->name();}

private:
  void adaptLoad(double total_ms);

  PipelineConfig config_;
  std::unique_ptr<ThreadPool> pool_;
  std::unique_ptr<CloudProcessor> processor_;
  std::unique_ptr<GroundSegmenterBase> ground_;
  std::unique_ptr<ObstacleClusterer> clusterer_;
  std::unique_ptr<EgoMotionEstimator> ego_;
  std::unique_ptr<StaticBackgroundMap> static_map_;
  std::unique_ptr<MultiObjectTracker> tracker_;
  std::unique_ptr<CollisionWarningFilter> warning_;
  std::vector<ExclusionZone> exclusions_;   // moving tracks of the previous frame
  EgoMotion external_ego_;
  float voxel_leaf_{0.F};
  double load_sum_ms_{0.0};
  int load_frames_{0};
};

}  // namespace helios_obstacle_detection
