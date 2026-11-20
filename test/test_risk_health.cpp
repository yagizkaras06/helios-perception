// Unit tests: TTC / risk, sensor health, thread pool, resource monitor, oriented boxes,
// adaptive clustering.
#include <gtest/gtest.h>

#include <atomic>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "helios_obstacle_detection/bounding_box.hpp"
#include "helios_obstacle_detection/collision_risk.hpp"
#include "helios_obstacle_detection/obstacle_clusterer.hpp"
#include "helios_obstacle_detection/resource_monitor.hpp"
#include "helios_obstacle_detection/sensor_health.hpp"
#include "helios_obstacle_detection/thread_pool.hpp"

using namespace helios_obstacle_detection;

namespace
{
TrackOutput track(float x, float y, float vx, float vy, float sigma = 0.05F)
{
  TrackOutput t;
  t.state = TrackState::kConfirmed;
  t.velocity_reliable = true;
  t.center = Eigen::Vector3f(x, y, 0.F);
  t.size = Eigen::Vector3f(0.6F, 0.6F, 1.7F);
  t.closest_point = Eigen::Vector3f(x - (x > 0 ? 0.3F : -0.3F), y, 0.F);
  t.closest_distance = t.closest_point.norm();
  t.relative_velocity = Eigen::Vector2f(vx, vy);
  t.relative_velocity_cov = Eigen::Matrix2f::Identity() * sigma * sigma;
  return t;
}
}  // namespace

TEST(Risk, HeadOnTtcAndCollisionCourse)
{
  RiskConfig cfg;
  const auto r = assessRisk(track(10.3F, 0.F, -2.F, 0.F), cfg);
  ASSERT_TRUE(r.ttc_valid);
  EXPECT_NEAR(r.closing_speed, 2.F, 1e-4);
  EXPECT_NEAR(r.ttc, (10.F - cfg.ego_radius) / 2.F, 1e-3);
  EXPECT_LT(r.ttc_pessimistic, r.ttc);
  EXPECT_TRUE(r.collision_course);
  EXPECT_EQ(r.level, RiskLevel::kMedium);   // ttc_pess ~4.5 s < 5 s
  const auto near = assessRisk(track(3.3F, 0.F, -2.F, 0.F), cfg);
  EXPECT_EQ(near.level, RiskLevel::kCritical);   // ~1.25 s
}

TEST(Risk, PassingAndRecedingObjects)
{
  RiskConfig cfg;
  const auto pass = assessRisk(track(10.F, 6.F, -2.F, 0.F), cfg);
  EXPECT_FALSE(pass.collision_course);
  EXPECT_GT(pass.cpa_distance, 0.F);
  const auto away = assessRisk(track(10.F, 0.F, 1.F, 0.F), cfg);
  EXPECT_TRUE(std::isinf(away.ttc));
  EXPECT_EQ(away.level, RiskLevel::kNone);
  // uncertain velocity widens the pessimistic bound, never the other way round
  const auto unsure = assessRisk(track(10.3F, 0.F, -0.1F, 0.F, 0.8F), cfg);
  EXPECT_TRUE(std::isinf(unsure.ttc));
  EXPECT_TRUE(std::isfinite(unsure.ttc_pessimistic));
}

TEST(Risk, UnreliableVelocityIsDistanceOnly)
{
  RiskConfig cfg;
  TrackOutput t = track(1.2F, 0.F, -5.F, 0.F);
  t.velocity_reliable = false;
  const auto r = assessRisk(t, cfg);
  EXPECT_FALSE(r.ttc_valid);
  EXPECT_EQ(r.level, RiskLevel::kCritical);   // clearance 0.4 m < 0.5 m
  t = track(8.F, 0.F, -5.F, 0.F);
  t.velocity_reliable = false;
  EXPECT_EQ(assessRisk(t, cfg).level, RiskLevel::kNone);
}

TEST(Health, GapsJumpsClockAndCoverage)
{
  SensorHealthMonitor h(SensorHealthConfig{});
  ScanQuality ok{57600, 40000, 36, 36};
  const double offset = 0.08;   // synchronised clocks, 80 ms stamp age
  for (int k = 0; k < 20; ++k) {
    const double t = 100.0 + 0.1 * k;
    const auto fh = h.onFrame(t, t + offset, ok);
    EXPECT_FALSE(fh.gap);
  }
  auto s = h.snapshot(102.0 + offset);
  EXPECT_NEAR(s.stamp_rate_hz, 10.0, 0.01);
  EXPECT_TRUE(s.clock_synced);
  EXPECT_NEAR(s.stamp_age_mean_ms, 80.0, 1.0);
  EXPECT_EQ(s.level, HealthLevel::kOk);

  const auto gap = h.onFrame(102.5, 102.5 + offset, ok);   // 0.6 s after 101.9
  EXPECT_TRUE(gap.gap);
  EXPECT_EQ(gap.missed_frames, 5);
  ScanQuality partial{57600, 30000, 36, 18};
  h.onFrame(102.6, 102.6 + offset, partial);
  s = h.snapshot(102.7);
  EXPECT_EQ(s.level, HealthLevel::kWarn);
  EXPECT_EQ(s.partial_scans, 1u);
  EXPECT_TRUE(h.onFrame(50.0, 102.8, ok).time_jump);

  // unsynchronised sensor clock (offset ~ 9.9 years like the real Helios here)
  SensorHealthMonitor u(SensorHealthConfig{});
  for (int k = 0; k < 20; ++k) {
    const double t = 1.483e9 + 0.1 * k;
    u.onFrame(t, t + 3.1188e8 + (k % 3) * 0.002, ok);
  }
  const auto us = u.snapshot(1.8e9);
  EXPECT_FALSE(us.clock_synced);
  EXPECT_NEAR(us.jitter_p95_ms, 4.0, 0.5);
}

TEST(Health, FastAtan2Accuracy)
{
  for (float a = -3.14F; a < 3.14F; a += 0.01F) {
    const float x = 7.F * std::cos(a), y = 7.F * std::sin(a);
    EXPECT_NEAR(fastAtan2(y, x), std::atan2(y, x), 0.006F);
  }
}

TEST(ThreadPool, ParallelForAndExceptions)
{
  ThreadPool pool(4);
  std::vector<int> v(10007, 0);
  pool.parallelFor(v.size(), [&](std::size_t b, std::size_t e) {
      for (std::size_t i = b; i < e; ++i) {
        v[i] = static_cast<int>(i % 7);
      }
    });
  long sum = 0;
  for (std::size_t i = 0; i < v.size(); ++i) {
    ASSERT_EQ(v[i], static_cast<int>(i % 7));
    sum += v[i];
  }
  EXPECT_GT(sum, 0);
  EXPECT_THROW(
    pool.parallelFor(100, [](std::size_t b, std::size_t) {
      if (b > 50) {
        throw std::runtime_error("x");
      }
    }), std::runtime_error);
  // parallelFor never waits for busy workers (no deadlock while all workers are blocked)
  std::atomic<bool> release{false};
  std::vector<std::future<void>> busy;
  for (int k = 0; k < 4; ++k) {
    busy.push_back(pool.submit([&] {while (!release) {std::this_thread::yield();}}));
  }
  std::atomic<int> count{0};
  pool.parallelFor(1000, [&](std::size_t b, std::size_t e) {count += static_cast<int>(e - b);});
  EXPECT_EQ(count.load(), 1000);
  release = true;
  for (auto & f : busy) {
    f.get();
  }
}

TEST(Resources, ReadsProc)
{
  ResourceMonitor m;
  volatile double x = 0;
  for (int i = 0; i < 3000000; ++i) {
    x = x + std::sqrt(static_cast<double>(i));
  }
  const auto s = m.sample();
  EXPECT_TRUE(s.valid);
  EXPECT_GT(s.rss_mb, 1.0);
  EXPECT_GE(s.threads, 1);
}

TEST(OrientedBox, RecoversYawOfRotatedRectangle)
{
  Cloud c;
  const float yaw = 0.5F, L = 4.F, W = 1.8F;
  // L-shape: only two visible sides, as seen by a LiDAR
  for (float s = -L / 2; s <= L / 2; s += 0.05F) {
    PointT p;
    p.x = 10.F + std::cos(yaw) * s - std::sin(yaw) * (-W / 2);
    p.y = 2.F + std::sin(yaw) * s + std::cos(yaw) * (-W / 2);
    p.z = 0.F;
    c.push_back(p);
  }
  for (float s = -W / 2; s <= W / 2; s += 0.05F) {
    PointT p;
    p.x = 10.F + std::cos(yaw) * (-L / 2) - std::sin(yaw) * s;
    p.y = 2.F + std::sin(yaw) * (-L / 2) + std::cos(yaw) * s;
    p.z = 1.F;
    c.push_back(p);
  }
  pcl::Indices idx(c.size());
  for (std::size_t i = 0; i < c.size(); ++i) {
    idx[i] = static_cast<pcl::index_t>(i);
  }
  const auto b = fitOrientedBox(c, idx, 1.F);
  EXPECT_NEAR(b.yaw, yaw, 0.03F);
  EXPECT_NEAR(b.size.x(), L, 0.1F);
  EXPECT_NEAR(b.size.y(), W, 0.1F);
  EXPECT_NEAR(b.center.x(), 10.F, 0.1F);
  EXPECT_NEAR(b.center.y(), 2.F, 0.1F);
}

TEST(Clustering, AdaptiveToleranceKeepsFarObjectTogether)
{
  // far object (25 m) sampled with 0.6 m vertical gaps (beam spacing) -> one cluster in
  // adaptive mode, several stripes with a fixed 0.45 m tolerance
  auto c = std::make_shared<Cloud>();
  for (float z = -0.8F; z <= 1.0F; z += 0.6F) {
    for (float y = -1.F; y <= 1.F; y += 0.1F) {
      PointT p;
      p.x = 25.F;
      p.y = y;
      p.z = z;
      c->push_back(p);
    }
  }
  ClusterConfig fixed;
  fixed.adaptive = false;
  fixed.min_points = 3;
  ClusterConfig adaptive = fixed;
  adaptive.adaptive = true;
  EXPECT_GT(ObstacleClusterer(fixed).extract(c).size(), 1u);
  EXPECT_EQ(ObstacleClusterer(adaptive).extract(c).size(), 1u);
  EXPECT_NEAR(adaptive.toleranceAt(25.F), 25.F * std::tan(1.5F * 3.14159F / 180.F), 1e-3);
  EXPECT_FLOAT_EQ(adaptive.toleranceAt(2.F), adaptive.tolerance);
}
