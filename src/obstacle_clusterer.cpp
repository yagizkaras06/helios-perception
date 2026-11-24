#include "helios_obstacle_detection/obstacle_clusterer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

namespace helios_obstacle_detection
{

void ClusterConfig::validate() const
{
  if (!(tolerance > 0.F)) {
    throw std::invalid_argument("clustering.tolerance must be > 0");
  }
  if (min_points < 1 || max_points < min_points) {
    throw std::invalid_argument("clustering: require 1 <= min_points <= max_points");
  }
  if (adaptive && !(tolerance_angle_deg > 0.F && tolerance_angle_deg < 10.F &&
    max_tolerance >= tolerance))
  {
    throw std::invalid_argument(
            "clustering: tolerance_angle_deg in (0,10), max_tolerance >= tolerance");
  }
}

float ClusterConfig::toleranceAt(float range) const noexcept
{
  if (!adaptive) {
    return tolerance;
  }
  const float t = range * std::tan(tolerance_angle_deg * static_cast<float>(M_PI) / 180.F);
  return std::clamp(t, tolerance, max_tolerance);
}

ObstacleClusterer::ObstacleClusterer(const ClusterConfig & config)
: config_(config)
{
  config_.validate();
  tan_angle_ = std::tan(config_.tolerance_angle_deg * static_cast<float>(M_PI) / 180.F);
}

std::vector<pcl::PointIndices> ObstacleClusterer::extract(const CloudConstPtr & cloud) const
{
  if (cloud->size() < static_cast<std::size_t>(config_.min_points)) {
    return {};
  }
  return config_.adaptive ? extractAdaptive(cloud) : extractFixed(cloud);
}

std::vector<pcl::PointIndices> ObstacleClusterer::extractFixed(const CloudConstPtr & cloud) const
{
  std::vector<pcl::PointIndices> clusters;
  // Unsorted radius-search results: ordering is irrelevant for region growing; measured on
  // recorded Helios data this cut clustering time from ~15.3 ms to ~10.5 ms (same clusters).
  auto tree = std::make_shared<pcl::search::KdTree<PointT>>(false);
  tree->setInputCloud(cloud);

  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(config_.tolerance);
  ec.setMinClusterSize(config_.min_points);
  ec.setMaxClusterSize(config_.max_points);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(clusters);
  return clusters;
}

std::vector<pcl::PointIndices> ObstacleClusterer::extractAdaptive(
  const CloudConstPtr & cloud) const
{
  // Region growing with a range-dependent radius. Neighbour candidates come from a uniform 3D
  // grid (cell = minimum tolerance) instead of a k-d tree: same neighbour definition
  // (squared distance < radius^2 of the expanding point, as FLANN's radius search), same seed
  // order -> identical clusters, but no tree traversal / result vectors and no distance test for
  // points that already belong to a cluster (dense near-field surfaces made this the hot spot).
  const std::size_t n = cloud->size();
  const float cell = config_.tolerance;
  const float inv = 1.F / cell;
  Eigen::Vector3f lo = Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
  Eigen::Vector3f hi = Eigen::Vector3f::Constant(std::numeric_limits<float>::lowest());
  for (const auto & p : *cloud) {
    lo = lo.cwiseMin(Eigen::Vector3f(p.x, p.y, p.z));
    hi = hi.cwiseMax(Eigen::Vector3f(p.x, p.y, p.z));
  }
  const Eigen::Vector3f span = (hi - lo) * inv;
  const auto nx = static_cast<std::int64_t>(span.x()) + 1;
  const auto ny = static_cast<std::int64_t>(span.y()) + 1;
  const auto nz = static_cast<std::int64_t>(span.z()) + 1;
  if (!(lo.allFinite() && hi.allFinite()) || nx * ny * nz > (std::int64_t{1} << 24)) {
    return extractAdaptiveKdTree(cloud);   // degenerate extent: fall back to the k-d tree
  }

  // counting sort of the points by cell
  std::vector<std::uint32_t> cell_of(n);
  std::vector<std::uint32_t> start(static_cast<std::size_t>(nx * ny * nz) + 1, 0);
  std::vector<float> radius2(n);
  std::vector<std::uint8_t> reach(n);   // cells to scan in each direction
  for (std::size_t i = 0; i < n; ++i) {
    const auto & p = (*cloud)[i];
    const auto cx = static_cast<std::int64_t>((p.x - lo.x()) * inv);
    const auto cy = static_cast<std::int64_t>((p.y - lo.y()) * inv);
    const auto cz = static_cast<std::int64_t>((p.z - lo.z()) * inv);
    cell_of[i] = static_cast<std::uint32_t>((cz * ny + cy) * nx + cx);
    ++start[cell_of[i] + 1];
    const float r = std::sqrt(p.x * p.x + p.y * p.y);
    const float rad = std::clamp(r * tan_angle_, config_.tolerance, config_.max_tolerance);
    radius2[i] = rad * rad;
    reach[i] = static_cast<std::uint8_t>(std::ceil(rad * inv));
  }
  for (std::size_t c = 1; c < start.size(); ++c) {
    start[c] += start[c - 1];
  }
  std::vector<std::uint32_t> order(n);
  {
    std::vector<std::uint32_t> fill(start.begin(), start.end() - 1);
    for (std::size_t i = 0; i < n; ++i) {
      order[fill[cell_of[i]]++] = static_cast<std::uint32_t>(i);
    }
  }

  std::vector<std::uint8_t> visited(n, 0);
  std::vector<pcl::PointIndices> clusters;
  pcl::Indices queue;
  queue.reserve(1024);
  const auto max_pts = static_cast<std::size_t>(config_.max_points);
  const auto min_pts = static_cast<std::size_t>(config_.min_points);

  for (std::size_t seed = 0; seed < n; ++seed) {
    if (visited[seed]) {
      continue;
    }
    queue.clear();
    queue.push_back(static_cast<pcl::index_t>(seed));
    visited[seed] = 1;
    for (std::size_t head = 0; head < queue.size(); ++head) {
      const auto q = static_cast<std::size_t>(queue[head]);
      const auto & pq = (*cloud)[q];
      const std::int64_t k = reach[q];
      const std::int64_t c = cell_of[q];
      const std::int64_t qx = c % nx, qy = (c / nx) % ny, qz = c / (nx * ny);
      const float r2 = radius2[q];
      for (std::int64_t z = std::max<std::int64_t>(qz - k, 0); z <= std::min(qz + k, nz - 1); ++z) {
        for (std::int64_t y = std::max<std::int64_t>(qy - k, 0); y <= std::min(qy + k, ny - 1); ++y) {
          const std::int64_t row = (z * ny + y) * nx;
          const auto x0 = static_cast<std::size_t>(row + std::max<std::int64_t>(qx - k, 0));
          const auto x1 = static_cast<std::size_t>(row + std::min(qx + k, nx - 1));
          // cells x0..x1 of this row are contiguous in `order`
          for (std::uint32_t o = start[x0]; o < start[x1 + 1]; ++o) {
            const std::uint32_t j = order[o];
            if (visited[j]) {
              continue;
            }
            const auto & pj = (*cloud)[j];
            const float dx = pj.x - pq.x, dy = pj.y - pq.y, dz = pj.z - pq.z;
            if (dx * dx + dy * dy + dz * dz < r2) {
              visited[j] = 1;
              queue.push_back(static_cast<pcl::index_t>(j));
            }
          }
        }
      }
    }
    // Same semantics as PCL: clusters outside [min, max] are discarded entirely.
    if (queue.size() >= min_pts && queue.size() <= max_pts) {
      pcl::PointIndices ci;
      ci.indices.assign(queue.begin(), queue.end());
      std::sort(ci.indices.begin(), ci.indices.end());
      clusters.push_back(std::move(ci));
    }
  }
  std::sort(
    clusters.begin(), clusters.end(), [](const pcl::PointIndices & a, const pcl::PointIndices & b) {
      return a.indices.size() > b.indices.size();
    });
  return clusters;
}

std::vector<pcl::PointIndices> ObstacleClusterer::extractAdaptiveKdTree(
  const CloudConstPtr & cloud) const
{
  const std::size_t n = cloud->size();
  pcl::KdTreeFLANN<PointT> tree(false);  // unsorted results
  tree.setInputCloud(cloud);

  std::vector<std::uint8_t> visited(n, 0);
  std::vector<float> radius(n);
  for (std::size_t i = 0; i < n; ++i) {
    const auto & p = (*cloud)[i];
    const float r = std::sqrt(p.x * p.x + p.y * p.y);
    radius[i] = std::clamp(r * tan_angle_, config_.tolerance, config_.max_tolerance);
  }

  std::vector<pcl::PointIndices> clusters;
  pcl::Indices nn;
  std::vector<float> nn_d;
  pcl::Indices queue;
  queue.reserve(1024);
  const auto max_pts = static_cast<std::size_t>(config_.max_points);
  const auto min_pts = static_cast<std::size_t>(config_.min_points);

  for (std::size_t seed = 0; seed < n; ++seed) {
    if (visited[seed]) {
      continue;
    }
    queue.clear();
    queue.push_back(static_cast<pcl::index_t>(seed));
    visited[seed] = 1;
    for (std::size_t head = 0; head < queue.size(); ++head) {
      const auto q = static_cast<std::size_t>(queue[head]);
      if (tree.radiusSearch((*cloud)[q], radius[q], nn, nn_d) <= 0) {
        continue;
      }
      for (const auto j : nn) {
        const auto ju = static_cast<std::size_t>(j);
        if (!visited[ju]) {
          visited[ju] = 1;
          queue.push_back(j);
        }
      }
    }
    // Same semantics as PCL: clusters outside [min, max] are discarded entirely.
    if (queue.size() >= min_pts && queue.size() <= max_pts) {
      pcl::PointIndices c;
      c.indices.assign(queue.begin(), queue.end());
      std::sort(c.indices.begin(), c.indices.end());
      clusters.push_back(std::move(c));
    }
  }
  std::sort(
    clusters.begin(), clusters.end(), [](const pcl::PointIndices & a, const pcl::PointIndices & b) {
      return a.indices.size() > b.indices.size();
    });
  return clusters;
}

}  // namespace helios_obstacle_detection
