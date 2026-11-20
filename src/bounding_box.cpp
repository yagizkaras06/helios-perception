#include "helios_obstacle_detection/bounding_box.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace helios_obstacle_detection
{

namespace
{
constexpr float kPi = static_cast<float>(M_PI);

float wrapHalfPi(float yaw) noexcept
{
  // to [-pi/2, pi/2)
  while (yaw >= kPi / 2.F) {
    yaw -= kPi;
  }
  while (yaw < -kPi / 2.F) {
    yaw += kPi;
  }
  return yaw;
}
}  // namespace

void BoxFilterConfig::validate() const
{
  if (min_height < 0.F || !(large_structure_size > 0.F) || max_obstacles < 1) {
    throw std::invalid_argument(
            "boxes: min_height >= 0, large_structure_size > 0, max_obstacles >= 1");
  }
  if (!(obb_angle_step_deg >= 0.25F && obb_angle_step_deg <= 15.F)) {
    throw std::invalid_argument("boxes.obb_angle_step_deg must be in [0.25, 15]");
  }
}

ObstacleCandidate computeAabb(const Cloud & cloud, const pcl::Indices & indices)
{
  if (indices.empty()) {
    throw std::invalid_argument("computeAabb: empty cluster");
  }
  constexpr float kInf = std::numeric_limits<float>::infinity();
  Eigen::Vector3f mn(kInf, kInf, kInf);
  Eigen::Vector3f mx(-kInf, -kInf, -kInf);
  float best_sq = kInf;
  Eigen::Vector3f closest = Eigen::Vector3f::Zero();

  for (const auto i : indices) {
    const PointT & p = cloud[static_cast<std::size_t>(i)];
    const Eigen::Vector3f v(p.x, p.y, p.z);
    mn = mn.cwiseMin(v);
    mx = mx.cwiseMax(v);
    const float d2 = v.squaredNorm();
    if (d2 < best_sq) {
      best_sq = d2;
      closest = v;
    }
  }

  ObstacleCandidate o;
  o.point_count = indices.size();
  o.min_pt = mn;
  o.max_pt = mx;
  o.center = 0.5F * (mn + mx);   // C = (min + max) / 2
  o.size = mx - mn;              // L, W, H = max - min
  o.closest_point = closest;
  o.closest_distance = std::sqrt(best_sq);
  o.center_distance = o.center.norm();
  o.indices = &indices;
  o.obb.center = o.center;
  o.obb.size = o.size;
  o.obb.yaw = 0.F;
  return o;
}

OrientedBox fitOrientedBox(
  const Cloud & cloud, const pcl::Indices & indices, float angle_step_deg)
{
  OrientedBox box;
  if (indices.empty()) {
    return box;
  }
  float zmin = std::numeric_limits<float>::infinity();
  float zmax = -zmin;
  for (const auto i : indices) {
    const float z = cloud[static_cast<std::size_t>(i)].z;
    zmin = std::min(zmin, z);
    zmax = std::max(zmax, z);
  }

  // Subsample for the angle search (extents are recomputed on all points afterwards).
  const std::size_t stride = std::max<std::size_t>(1, indices.size() / 300);
  thread_local std::vector<Eigen::Vector2f> pts;
  pts.clear();
  for (std::size_t k = 0; k < indices.size(); k += stride) {
    const auto & p = cloud[static_cast<std::size_t>(indices[k])];
    pts.emplace_back(p.x, p.y);
  }

  float best_theta = 0.F;
  if (pts.size() >= 4) {
    const float step = angle_step_deg * kPi / 180.F;
    float best_score = -1.F;
    constexpr float kD0 = 0.01F;
    thread_local std::vector<float> c1, c2;
    c1.resize(pts.size());
    c2.resize(pts.size());
    for (float th = 0.F; th < kPi / 2.F; th += step) {
      const float c = std::cos(th), s = std::sin(th);
      float mn1 = std::numeric_limits<float>::infinity(), mx1 = -mn1, mn2 = mn1, mx2 = -mn1;
      for (std::size_t k = 0; k < pts.size(); ++k) {
        c1[k] = pts[k].x() * c + pts[k].y() * s;
        c2[k] = -pts[k].x() * s + pts[k].y() * c;
        mn1 = std::min(mn1, c1[k]);
        mx1 = std::max(mx1, c1[k]);
        mn2 = std::min(mn2, c2[k]);
        mx2 = std::max(mx2, c2[k]);
      }
      float score = 0.F;
      for (std::size_t k = 0; k < pts.size(); ++k) {
        const float d1 = std::min(mx1 - c1[k], c1[k] - mn1);
        const float d2 = std::min(mx2 - c2[k], c2[k] - mn2);
        score += 1.F / std::max(std::min(d1, d2), kD0);
      }
      if (score > best_score) {
        best_score = score;
        best_theta = th;
      }
    }
  }

  const float c = std::cos(best_theta), s = std::sin(best_theta);
  float mn1 = std::numeric_limits<float>::infinity(), mx1 = -mn1, mn2 = mn1, mx2 = -mn1;
  for (const auto i : indices) {
    const auto & p = cloud[static_cast<std::size_t>(i)];
    const float a = p.x * c + p.y * s;
    const float b = -p.x * s + p.y * c;
    mn1 = std::min(mn1, a);
    mx1 = std::max(mx1, a);
    mn2 = std::min(mn2, b);
    mx2 = std::max(mx2, b);
  }
  const float m1 = 0.5F * (mn1 + mx1), m2 = 0.5F * (mn2 + mx2);
  box.center = Eigen::Vector3f(m1 * c - m2 * s, m1 * s + m2 * c, 0.5F * (zmin + zmax));
  float len = mx1 - mn1, wid = mx2 - mn2, yaw = best_theta;
  if (wid > len) {
    std::swap(len, wid);
    yaw += kPi / 2.F;
  }
  box.size = Eigen::Vector3f(len, wid, zmax - zmin);
  box.yaw = wrapHalfPi(yaw);
  return box;
}

std::vector<ObstacleCandidate> buildObstacles(
  const Cloud & cloud, const std::vector<pcl::PointIndices> & clusters,
  const BoxFilterConfig & config, ThreadPool * pool)
{
  std::vector<ObstacleCandidate> all(clusters.size());
  std::vector<std::uint8_t> keep(clusters.size(), 0);
  auto work = [&](std::size_t b, std::size_t e) {
      for (std::size_t k = b; k < e; ++k) {
        const auto & c = clusters[k];
        if (c.indices.empty()) {
          continue;
        }
        ObstacleCandidate o = computeAabb(cloud, c.indices);
        if (o.size.z() < config.min_height) {
          continue;  // flat residue (ground noise, road markings)
        }
        o.large_structure = std::max(o.size.x(), o.size.y()) >= config.large_structure_size;
        if (config.oriented_boxes) {
          o.obb = fitOrientedBox(cloud, c.indices, config.obb_angle_step_deg);
        }
        all[k] = o;
        keep[k] = 1;
      }
    };
  if (pool != nullptr) {
    pool->parallelFor(clusters.size(), work, 4);
  } else {
    work(0, clusters.size());
  }

  std::vector<ObstacleCandidate> out;
  out.reserve(clusters.size());
  for (std::size_t k = 0; k < all.size(); ++k) {
    if (keep[k]) {
      out.push_back(all[k]);
    }
  }
  std::sort(
    out.begin(), out.end(), [](const ObstacleCandidate & a, const ObstacleCandidate & b) {
      return a.closest_distance < b.closest_distance;
    });
  if (out.size() > static_cast<std::size_t>(config.max_obstacles)) {
    out.resize(static_cast<std::size_t>(config.max_obstacles));
  }
  for (std::size_t i = 0; i < out.size(); ++i) {
    out[i].id = static_cast<std::uint32_t>(i);
  }
  return out;
}

}  // namespace helios_obstacle_detection
