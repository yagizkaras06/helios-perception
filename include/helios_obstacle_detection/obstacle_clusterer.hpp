// Euclidean clustering of non-ground points.
//
// Two modes:
//   fixed    : pcl::EuclideanClusterExtraction with one tolerance (original behaviour)
//   adaptive : region growing whose neighbour radius grows with range,
//              tol(r) = clamp(max(tolerance, r * tan(tolerance_angle)), tolerance, max_tolerance)
//              because the gap between two Helios 32 beams grows linearly with range. A fixed
//              tolerance either merges nearby objects or splits far ones into beam stripes.
#pragma once

#include <vector>

#include <pcl/PointIndices.h>

#include "helios_obstacle_detection/types.hpp"

namespace helios_obstacle_detection
{

struct ClusterConfig
{
  float tolerance{0.45F};          // [m] neighbour distance (minimum in adaptive mode)
  int min_points{8};
  int max_points{25000};
  bool adaptive{true};
  float tolerance_angle_deg{1.5F}; // [deg] ~ vertical beam spacing of the Helios 32
  float max_tolerance{1.0F};       // [m]

  void validate() const;  // throws std::invalid_argument

  float toleranceAt(float range) const noexcept;
};

class ObstacleClusterer
{
public:
  explicit ObstacleClusterer(const ClusterConfig & config);

  /// Returns one PointIndices per cluster, indices refer to `cloud`.
  std::vector<pcl::PointIndices> extract(const CloudConstPtr & cloud) const;

  const ClusterConfig & config() const noexcept {return config_;}

  /// Reference implementation of the adaptive mode (k-d tree radius search); used for degenerate
  /// extents and to verify that the grid search gives identical clusters.
  std::vector<pcl::PointIndices> extractAdaptiveKdTree(const CloudConstPtr & cloud) const;

private:
  std::vector<pcl::PointIndices> extractFixed(const CloudConstPtr & cloud) const;
  std::vector<pcl::PointIndices> extractAdaptive(const CloudConstPtr & cloud) const;

  ClusterConfig config_;
  float tan_angle_{0.F};
};

}  // namespace helios_obstacle_detection
