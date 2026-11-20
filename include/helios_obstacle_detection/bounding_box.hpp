// Bounding boxes for clusters: axis-aligned (AABB) + yaw-oriented (OBB, L-shape fitting),
// distance estimation.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <pcl/PointIndices.h>

#include "helios_obstacle_detection/thread_pool.hpp"
#include "helios_obstacle_detection/types.hpp"

namespace helios_obstacle_detection
{

struct OrientedBox
{
  Eigen::Vector3f center{Eigen::Vector3f::Zero()};
  Eigen::Vector3f size{Eigen::Vector3f::Zero()};   // x = length along yaw (>= y), y = width
  float yaw{0.F};                                   // [rad] in [-pi/2, pi/2)
};

struct ObstacleCandidate
{
  std::uint32_t id{0};             // per-frame only, NOT a tracking id
  std::size_t point_count{0};
  Eigen::Vector3f min_pt{Eigen::Vector3f::Zero()};
  Eigen::Vector3f max_pt{Eigen::Vector3f::Zero()};
  Eigen::Vector3f center{Eigen::Vector3f::Zero()};   // (min+max)/2
  Eigen::Vector3f size{Eigen::Vector3f::Zero()};     // L, W, H = max-min
  Eigen::Vector3f closest_point{Eigen::Vector3f::Zero()};
  float closest_distance{0.F};    // [m] sensor origin -> closest cluster point
  float center_distance{0.F};     // [m] sensor origin -> AABB center
  bool large_structure{false};
  bool background{false};         // cluster of static-background points (static map)
  OrientedBox obb;                // yaw-oriented box (tracking / visualisation)
  const pcl::Indices * indices{nullptr};  // points of this cluster (owned by caller)
};

struct BoxFilterConfig
{
  float min_height{0.05F};             // [m] drop flatter clusters (ground residue)
  float large_structure_size{8.0F};    // [m] footprint extent that flags walls/buildings
  int max_obstacles{100};              // keep N nearest (bounded output / visualisation)
  bool oriented_boxes{true};           // L-shape fitting; false -> OBB = AABB (yaw 0)
  float obb_angle_step_deg{2.F};

  void validate() const;
};

/// Computes the AABB and both distances for a set of point indices. Precondition: non-empty.
ObstacleCandidate computeAabb(const Cloud & cloud, const pcl::Indices & indices);

/// Search-based rectangle fitting with the closeness criterion (Zhang et al., IV 2017).
/// Robust for L-shaped (partially visible) vehicles; yaw has a 90 deg ambiguity that is resolved
/// by making size.x the longer side.
OrientedBox fitOrientedBox(
  const Cloud & cloud, const pcl::Indices & indices, float angle_step_deg = 2.F);

/// Builds, filters and sorts (nearest first) obstacle candidates; assigns per-frame ids.
/// `pool` (optional) parallelises the per-cluster box fitting.
std::vector<ObstacleCandidate> buildObstacles(
  const Cloud & cloud, const std::vector<pcl::PointIndices> & clusters,
  const BoxFilterConfig & config, ThreadPool * pool = nullptr);

}  // namespace helios_obstacle_detection
