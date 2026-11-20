// Unit tests: LiDAR-only ego-motion estimation.
#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "helios_obstacle_detection/ego_motion_estimator.hpp"

using namespace helios_obstacle_detection;

namespace
{

void wall(Cloud & c, Eigen::Vector2f a, Eigen::Vector2f b, float step, std::mt19937 & rng)
{
  std::normal_distribution<float> n(0.F, 0.01F);
  const float len = (b - a).norm();
  for (float s = 0.F; s <= len; s += step) {
    const Eigen::Vector2f p = a + (b - a) * (s / len);
    for (float z = -0.7F; z <= 1.5F; z += step) {
      PointT q;
      q.x = p.x() + n(rng);
      q.y = p.y() + n(rng);
      q.z = z;
      c.push_back(q);
    }
  }
}

/// Room-like structure with walls in several directions plus pillars (world frame).
Cloud room(std::mt19937 & rng)
{
  Cloud c;
  wall(c, {-10.F, -8.F}, {15.F, -8.F}, 0.1F, rng);
  wall(c, {15.F, -8.F}, {15.F, 9.F}, 0.1F, rng);
  wall(c, {-10.F, 9.F}, {6.F, 9.F}, 0.1F, rng);
  wall(c, {-10.F, -8.F}, {-10.F, 9.F}, 0.1F, rng);
  for (const auto & p : {Eigen::Vector2f(4.F, 3.F), Eigen::Vector2f(-3.F, -4.F),
      Eigen::Vector2f(8.F, -2.F)})
  {
    wall(c, p, p + Eigen::Vector2f(0.6F, 0.F), 0.1F, rng);
    wall(c, p, p + Eigen::Vector2f(0.F, 0.6F), 0.1F, rng);
  }
  return c;
}

/// World cloud seen from a sensor at pose (x, y, yaw).
Cloud view(const Cloud & world, double x, double y, double yaw)
{
  Cloud out;
  const double c = std::cos(yaw), s = std::sin(yaw);
  for (const auto & p : world) {
    const double dx = p.x - x, dy = p.y - y;
    PointT q = p;
    q.x = static_cast<float>(c * dx + s * dy);
    q.y = static_cast<float>(-s * dx + c * dy);
    out.push_back(q);
  }
  return out;
}

}  // namespace

TEST(EgoMotion, RecoversPlanarMotion)
{
  std::mt19937 rng(5);
  const Cloud w = room(rng);
  EgoMotionEstimator est(EgoMotionConfig{});
  const double vx = 3.0, wz = 0.15, dt = 0.1;
  double x = 0.0, y = 0.0, yaw = 0.0;
  EgoMotion e;
  for (int k = 0; k < 8; ++k) {
    e = est.update(view(w, x, y, yaw), 50.0 + k * dt);
    x += vx * dt * std::cos(yaw);
    y += vx * dt * std::sin(yaw);
    yaw += wz * dt;
  }
  ASSERT_TRUE(e.measured) << e.reason;
  EXPECT_EQ(e.state, EgoState::kMoving);
  EXPECT_NEAR(e.velocity.x(), vx, 0.15) << e.reason;
  EXPECT_NEAR(e.velocity.y(), 0.0, 0.15);
  EXPECT_NEAR(e.yaw_rate, wz, 0.03);
  EXPECT_GT(e.degeneracy, 0.05F);
  // the reported uncertainty is small but non-zero
  EXPECT_GT(e.velocity_cov(0, 0), 0.0);
  EXPECT_LT(std::sqrt(e.velocity_cov(0, 0)), 0.2);
  // accumulated pose follows the trajectory (last update corresponds to the previous x/y/yaw)
  EXPECT_NEAR(e.pose[2], yaw - wz * dt, 0.03);
}

TEST(EgoMotion, StationarySensorIsStationary)
{
  std::mt19937 rng(6);
  const Cloud w = room(rng);
  EgoMotionEstimator est(EgoMotionConfig{});
  EgoMotion e;
  for (int k = 0; k < 6; ++k) {
    std::mt19937 r2(static_cast<unsigned>(k));
    e = est.update(view(room(r2), 0.0, 0.0, 0.0), 1.0 + 0.1 * k);
  }
  EXPECT_EQ(e.state, EgoState::kStationary) << e.reason;
  EXPECT_TRUE(e.valid);
  EXPECT_DOUBLE_EQ(e.pose[0], 0.0);   // zero-velocity update: no drift
}

TEST(EgoMotion, CorridorIsDegenerate)
{
  // two long parallel walls only: translation along the corridor is unobservable
  std::mt19937 rng(7);
  Cloud w;
  wall(w, {-30.F, -3.F}, {30.F, -3.F}, 0.1F, rng);
  wall(w, {-30.F, 3.F}, {30.F, 3.F}, 0.1F, rng);
  EgoMotionEstimator est(EgoMotionConfig{});
  est.update(view(w, 0.0, 0.0, 0.0), 1.0);
  const EgoMotion e = est.update(view(w, 0.3, 0.0, 0.0), 1.1);
  EXPECT_FALSE(e.measured);
  EXPECT_NE(e.reason.find("degenerate"), std::string::npos) << e.reason;
  EXPECT_NE(e.state, EgoState::kStationary);
  EXPECT_NE(e.state, EgoState::kMoving);
}

TEST(EgoMotion, GapResetsAndUnknownAfterPredictionTimeout)
{
  std::mt19937 rng(8);
  const Cloud w = room(rng);
  EgoMotionEstimator est(EgoMotionConfig{});
  est.update(view(w, 0, 0, 0), 1.0);
  EXPECT_TRUE(est.update(view(w, 0, 0, 0), 1.1).valid);
  const EgoMotion gap = est.update(view(w, 0, 0, 0), 3.0);   // 1.9 s gap
  EXPECT_FALSE(gap.measured);
  EXPECT_EQ(gap.state, EgoState::kUnknown);
  // empty clouds -> registration impossible -> short prediction, then UNKNOWN
  Cloud empty;
  EXPECT_TRUE(est.update(view(w, 0, 0, 0), 3.1).measured);
  EgoMotion e = est.update(empty, 3.2);
  EXPECT_EQ(e.state, EgoState::kUnreliable);
  EXPECT_TRUE(e.valid);
  for (int k = 0; k < 6; ++k) {
    e = est.update(empty, 3.3 + 0.1 * k);
  }
  EXPECT_EQ(e.state, EgoState::kUnknown);
  EXPECT_FALSE(e.valid);
}
