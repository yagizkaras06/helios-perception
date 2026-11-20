// Unit tests: concentric-zone (CZM) ground segmentation.
#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "helios_obstacle_detection/czm_ground_segmenter.hpp"

using namespace helios_obstacle_detection;

namespace
{

/// Polar ground pattern similar to a spinning LiDAR (rings get sparser with range).
void addRingGround(Cloud & c, float z0, float r_max, float slope_x = 0.F, unsigned seed = 1)
{
  std::mt19937 rng(seed);
  std::normal_distribution<float> n(0.F, 0.015F);
  for (float r = 1.0F; r <= r_max; r *= 1.06F) {
    for (float a = 0.F; a < 6.2831F; a += 0.0175F) {
      PointT p;
      p.x = r * std::cos(a);
      p.y = r * std::sin(a);
      p.z = z0 + slope_x * p.x + n(rng);
      c.push_back(p);
    }
  }
}

void addBox(Cloud & c, Eigen::Vector3f mn, Eigen::Vector3f mx, float step)
{
  for (float x = mn.x(); x <= mx.x() + 1e-4F; x += step) {
    for (float y = mn.y(); y <= mx.y() + 1e-4F; y += step) {
      for (float z = mn.z(); z <= mx.z() + 1e-4F; z += step) {
        PointT p;
        p.x = x;
        p.y = y;
        p.z = z;
        c.push_back(p);
      }
    }
  }
}

/// Surfaces a LiDAR at the origin would see of an axis-aligned box: near face + top.
void addVisibleBox(Cloud & c, Eigen::Vector3f mn, Eigen::Vector3f mx, float step)
{
  for (float y = mn.y(); y <= mx.y() + 1e-4F; y += step) {
    for (float z = mn.z(); z <= mx.z() + 1e-4F; z += step) {
      PointT p;
      p.x = mn.x();
      p.y = y;
      p.z = z;
      c.push_back(p);
    }
    for (float x = mn.x(); x <= mx.x() + 1e-4F; x += step) {
      PointT p;
      p.x = x;
      p.y = y;
      p.z = mx.z();
      c.push_back(p);
    }
  }
}

CzmConfig cfg()
{
  CzmConfig c;
  c.base.expected_height = -0.9F;
  return c;
}

}  // namespace

TEST(CzmGround, FlatGroundAndBox)
{
  auto cloud = std::make_shared<Cloud>();
  addRingGround(*cloud, -0.9F, 25.F);
  addVisibleBox(*cloud, {5.F, -0.4F, -0.85F}, {5.6F, 0.4F, 0.5F}, 0.1F);
  CzmGroundSegmenter seg(cfg());
  GroundResult r;
  seg.segment(cloud, r);
  EXPECT_EQ(r.status, GroundStatus::kReliable) << r.detail;
  std::size_t box_pts = 0;
  for (const auto i : r.obstacle) {
    const auto & p = (*cloud)[static_cast<std::size_t>(i)];
    EXPECT_GT(p.z, -0.9F + 0.10F) << r.detail;
    box_pts += p.x > 4.9F && p.x < 5.7F && std::fabs(p.y) < 0.5F;
  }
  EXPECT_GT(box_pts, 100u);
  // ground points really are ground (object base within the ground threshold may be ground)
  for (const auto i : r.ground) {
    EXPECT_LT((*cloud)[static_cast<std::size_t>(i)].z, -0.9F + 0.17F) << r.detail;
  }
}

TEST(CzmGround, FollowsSlope)
{
  auto cloud = std::make_shared<Cloud>();
  addRingGround(*cloud, -0.9F, 28.F, 0.06F);   // ~3.4 deg: +1.7 m at 28 m
  CzmGroundSegmenter seg(cfg());
  GroundResult r;
  seg.segment(cloud, r);
  ASSERT_NE(r.status, GroundStatus::kFailed) << r.detail;
  std::size_t wrong = 0;
  for (const auto i : r.obstacle) {
    wrong += (*cloud)[static_cast<std::size_t>(i)].x > 10.F;
  }
  EXPECT_LT(wrong, 20u) << r.detail;
}

TEST(CzmGround, ElevatedTableIsNotGround)
{
  // Dense, perfectly flat platform 0.75 m above the floor that hides the floor below it and
  // covers whole CZM bins (ring 2..4 m, sectors around +x): its plane fit is upright and flat,
  // so only the elevation / continuity checks can reject it.
  auto cloud = std::make_shared<Cloud>();
  Cloud ground;
  addRingGround(ground, -0.9F, 15.F);
  for (const auto & p : ground) {
    if (!(p.x >= 1.8F && p.x <= 4.2F && std::fabs(p.y) <= 1.8F)) {
      cloud->push_back(p);
    }
  }
  for (float x = 1.8F; x <= 4.2F; x += 0.05F) {
    for (float y = -1.8F; y <= 1.8F; y += 0.05F) {
      PointT p;
      p.x = x;
      p.y = y;
      p.z = -0.15F;
      cloud->push_back(p);
    }
  }
  CzmGroundSegmenter seg(cfg());
  GroundResult r;
  seg.segment(cloud, r);
  std::size_t table_as_ground = 0;
  for (const auto i : r.ground) {
    table_as_ground += std::fabs((*cloud)[static_cast<std::size_t>(i)].z + 0.15F) < 0.05F;
  }
  EXPECT_EQ(table_as_ground, 0u) << r.detail;
  EXPECT_GT(seg.lastStats().bins_by_state[static_cast<std::size_t>(
      BinState::kRejectedElevated)] + seg.lastStats().bins_by_state[static_cast<std::size_t>(
      BinState::kRejectedDiscontinuous)], 0u) << r.detail;
}

TEST(CzmGround, FailsWithoutGroundAndAdaptsThresholds)
{
  auto empty_ground = std::make_shared<Cloud>();
  addBox(*empty_ground, {3.F, -1.F, 0.5F}, {4.F, 1.F, 1.5F}, 0.1F);
  CzmGroundSegmenter seg(cfg());
  GroundResult r;
  seg.segment(empty_ground, r);
  EXPECT_EQ(r.status, GroundStatus::kFailed);

  auto cloud = std::make_shared<Cloud>();
  addRingGround(*cloud, -0.9F, 20.F);
  for (int k = 0; k < 30; ++k) {
    seg.segment(cloud, r);
  }
  const auto & thr = seg.lastStats().elevation_thresholds;
  ASSERT_FALSE(thr.empty());
  for (const float t : thr) {
    EXPECT_GE(t, cfg().elevation_min);
    EXPECT_LE(t, cfg().elevation_max);
  }
  // flat ground with little noise -> thresholds tighten from the initial 0.30 m
  EXPECT_LT(thr.front(), 0.30F);
}

TEST(CzmGround, ParallelMatchesSequential)
{
  auto cloud = std::make_shared<Cloud>();
  addRingGround(*cloud, -0.9F, 25.F, 0.02F, 3);
  addBox(*cloud, {7.F, 2.F, -0.85F}, {8.F, 3.F, 0.6F}, 0.08F);
  ThreadPool pool(4);
  CzmGroundSegmenter a(cfg()), b(cfg(), &pool);
  GroundResult ra, rb;
  a.segment(cloud, ra);
  b.segment(cloud, rb);
  EXPECT_EQ(ra.ground, rb.ground);
  EXPECT_EQ(ra.obstacle, rb.obstacle);
}
