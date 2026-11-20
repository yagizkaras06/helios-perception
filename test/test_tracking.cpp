// Unit tests: Hungarian assignment, Kalman filter, multi-object tracker.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>

#include <Eigen/Eigenvalues>

#include "helios_obstacle_detection/hungarian.hpp"
#include "helios_obstacle_detection/kalman_filter.hpp"
#include "helios_obstacle_detection/multi_object_tracker.hpp"

using namespace helios_obstacle_detection;

namespace
{

ObstacleCandidate det(float x, float y, float l = 0.5F, float w = 0.5F, std::size_t pts = 60)
{
  ObstacleCandidate d;
  d.point_count = pts;
  d.center = Eigen::Vector3f(x, y, -0.05F);
  d.size = Eigen::Vector3f(l, w, 1.7F);
  d.min_pt = d.center - 0.5F * d.size;
  d.max_pt = d.center + 0.5F * d.size;
  d.obb.center = d.center;
  d.obb.size = d.size;
  d.obb.yaw = 0.F;
  d.closest_point = Eigen::Vector3f(x - 0.25F, y, -0.05F);
  d.closest_distance = d.closest_point.norm();
  d.center_distance = d.center.norm();
  return d;
}

EgoMotion stationaryEgo()
{
  EgoMotion e;
  e.state = EgoState::kStationary;
  e.valid = true;
  e.measured = true;
  e.velocity_cov = Eigen::Matrix3d::Identity() * 1e-4;
  e.delta_cov = Eigen::Matrix3d::Identity() * 1e-6;
  return e;
}

double bruteForce(const std::vector<double> & c, std::size_t r, std::size_t k, double inf,
  int & matched)
{
  // maximise number of feasible matches, then minimise cost (same objective as the solver)
  std::vector<int> cols(k);
  std::iota(cols.begin(), cols.end(), 0);
  double best = std::numeric_limits<double>::infinity();
  int best_m = -1;
  std::vector<int> perm(std::max(r, k));
  std::iota(perm.begin(), perm.end(), 0);
  do {
    double cost = 0.0;
    int m = 0;
    for (std::size_t i = 0; i < r; ++i) {
      const int j = perm[i];
      if (j < static_cast<int>(k) && c[i * k + static_cast<std::size_t>(j)] < inf) {
        cost += c[i * k + static_cast<std::size_t>(j)];
        ++m;
      }
    }
    if (m > best_m || (m == best_m && cost < best - 1e-9)) {
      best_m = m;
      best = cost;
    }
  } while (std::next_permutation(perm.begin(), perm.end()));
  matched = best_m;
  return best;
}

}  // namespace

TEST(Hungarian, MatchesBruteForceWithForbiddenPairs)
{
  std::mt19937 rng(3);
  std::uniform_real_distribution<double> u(0.0, 10.0);
  std::uniform_int_distribution<int> dim(1, 5);
  const double inf = 1e12;
  for (int trial = 0; trial < 300; ++trial) {
    const auto r = static_cast<std::size_t>(dim(rng));
    const auto k = static_cast<std::size_t>(dim(rng));
    std::vector<double> c(r * k);
    for (auto & v : c) {
      v = u(rng) < 2.5 ? inf : u(rng);
    }
    const auto a = solveAssignment(c, r, k, inf);
    double cost = 0.0;
    int m = 0;
    std::vector<int> used(k, 0);
    for (std::size_t i = 0; i < r; ++i) {
      if (a[i] >= 0) {
        ASSERT_LT(c[i * k + static_cast<std::size_t>(a[i])], inf);
        ASSERT_EQ(used[static_cast<std::size_t>(a[i])]++, 0);
        cost += c[i * k + static_cast<std::size_t>(a[i])];
        ++m;
      }
    }
    int bm = 0;
    const double bc = bruteForce(c, r, k, inf, bm);
    EXPECT_EQ(m, bm) << "trial " << trial;
    EXPECT_NEAR(cost, bc, 1e-6) << "trial " << trial;
  }
}

TEST(KalmanFilter, ConvergesToConstantVelocity)
{
  ConstantVelocityKF kf;
  kf.init(Eigen::Vector2d(0, 0), Eigen::Matrix2d::Identity() * 0.01, 3.0);
  std::mt19937 rng(1);
  std::normal_distribution<double> n(0.0, 0.05);
  const Eigen::Vector2d v(1.5, -0.7);
  for (int k = 1; k <= 50; ++k) {
    kf.predict(0.1, 1.0);
    const Eigen::Vector2d z = v * (0.1 * k) + Eigen::Vector2d(n(rng), n(rng));
    kf.update(z, Eigen::Matrix2d::Identity() * 0.0025);
  }
  EXPECT_NEAR(kf.velocity().x(), 1.5, 0.15);
  EXPECT_NEAR(kf.velocity().y(), -0.7, 0.15);
  // covariance stays symmetric positive definite
  const Eigen::Matrix4d P = kf.P();
  EXPECT_LT((P - P.transpose()).norm(), 1e-12);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es(P);
  EXPECT_GT(es.eigenvalues().minCoeff(), 0.0);
}

TEST(Tracker, ConfirmsAndKeepsIdsOfCrossingObjects)
{
  TrackerConfig cfg;
  MultiObjectTracker trk(cfg);
  std::vector<TrackOutput> out;
  const EgoMotion ego = stationaryEgo();
  // A moves +x, B moves -x, lateral separation 1.2 m: their paths pass each other.
  std::uint32_t id_a = 0, id_b = 0;
  for (int k = 0; k < 60; ++k) {
    const float t = 0.1F * static_cast<float>(k);
    std::vector<ObstacleCandidate> d = {det(2.F + 1.2F * t, 0.6F), det(9.F - 1.2F * t, -0.6F)};
    trk.update(100.0 + t, d, ego, out);
    for (const auto & o : out) {
      if (o.state == TrackState::kTentative || o.detection_index < 0) {
        continue;
      }
      if (o.detection_index == 0) {
        if (id_a == 0) {
          id_a = o.id;
        }
        EXPECT_EQ(o.id, id_a) << "A switched id at frame " << k;
      } else {
        if (id_b == 0) {
          id_b = o.id;
        }
        EXPECT_EQ(o.id, id_b) << "B switched id at frame " << k;
      }
    }
  }
  ASSERT_NE(id_a, 0u);
  ASSERT_NE(id_b, 0u);
  EXPECT_NE(id_a, id_b);
  for (const auto & o : out) {
    if (o.id == id_a) {
      EXPECT_NEAR(o.velocity.x(), 1.2F, 0.2F);
      EXPECT_EQ(o.motion, MotionState::kMoving);
      EXPECT_TRUE(o.heading_valid);
      EXPECT_NEAR(o.heading, 0.F, 0.2F);
    }
  }
}

TEST(Tracker, StaticObjectIsStaticAndUnknownWithoutEgo)
{
  MultiObjectTracker trk(TrackerConfig{});
  std::vector<TrackOutput> out;
  std::mt19937 rng(2);
  std::normal_distribution<float> n(0.F, 0.03F);
  for (int k = 0; k < 30; ++k) {
    trk.update(10.0 + 0.1 * k, {det(5.F + n(rng), 1.F + n(rng))}, stationaryEgo(), out);
  }
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].motion, MotionState::kStatic);
  EXPECT_LT(out[0].speed, 0.2F);
  EXPECT_TRUE(out[0].velocity_reliable);

  EgoMotion unknown;   // no ego-motion information
  trk.update(13.0, {det(5.F, 1.F)}, unknown, out);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].motion, MotionState::kUnknown);
  EXPECT_FALSE(out[0].velocity_reliable);
}

TEST(Tracker, StaticObjectSeenFromMovingSensorStaysStatic)
{
  // Sensor drives +x at 2 m/s and turns 0.1 rad/s; object fixed in the world at (12, 3).
  MultiObjectTracker trk(TrackerConfig{});
  std::vector<TrackOutput> out;
  Eigen::Vector3d pose = Eigen::Vector3d::Zero();
  const double v = 2.0, w = 0.1, dt = 0.1;
  for (int k = 0; k < 40; ++k) {
    EgoMotion e;
    e.state = EgoState::kMoving;
    e.valid = e.measured = true;
    if (k > 0) {
      e.delta = Eigen::Vector3d(v * dt * std::cos(0.5 * w * dt), v * dt * std::sin(0.5 * w * dt),
          w * dt);
      const double c = std::cos(pose[2]), s = std::sin(pose[2]);
      pose[0] += c * e.delta[0] - s * e.delta[1];
      pose[1] += s * e.delta[0] + c * e.delta[1];
      pose[2] += e.delta[2];
    }
    e.pose = pose;
    e.velocity = Eigen::Vector2d(v, 0.0);
    e.yaw_rate = w;
    e.velocity_cov = Eigen::Matrix3d::Identity() * 1e-3;
    e.delta_cov = Eigen::Matrix3d::Identity() * 1e-6;
    const double c = std::cos(pose[2]), s = std::sin(pose[2]);
    const Eigen::Vector2d d(12.0 - pose[0], 3.0 - pose[1]);
    const Eigen::Vector2d ps(c * d.x() + s * d.y(), -s * d.x() + c * d.y());
    trk.update(0.1 * k, {det(static_cast<float>(ps.x()), static_cast<float>(ps.y()))}, e, out);
  }
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].motion, MotionState::kStatic);
  EXPECT_LT(out[0].speed, 0.2F);
  // relative velocity must reflect the ego motion: object approaches at ~2 m/s
  EXPECT_LT(out[0].relative_velocity.x(), -1.5F);
}

TEST(Tracker, OccludedThenLostAndResetsOnTimeJump)
{
  TrackerConfig cfg;
  cfg.max_coast_time_s = 0.5F;
  MultiObjectTracker trk(cfg);
  std::vector<TrackOutput> out;
  for (int k = 0; k < 10; ++k) {
    trk.update(1.0 + 0.1 * k, {det(4.F, 0.F)}, stationaryEgo(), out);
  }
  ASSERT_EQ(out.size(), 1u);
  const auto id = out[0].id;
  trk.update(2.0, {}, stationaryEgo(), out);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].state, TrackState::kOccluded);
  EXPECT_TRUE(out[0].predicted);
  EXPECT_EQ(out[0].occlusion, OcclusionReason::kUnexplained);   // no cloud: free line of sight
  EXPECT_FALSE(out[0].updated);
  trk.update(2.1, {det(4.F, 0.F)}, stationaryEgo(), out);   // re-acquired: same id
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].id, id);
  for (int k = 1; k <= 7; ++k) {
    trk.update(2.1 + 0.1 * k, {}, stationaryEgo(), out);
  }
  EXPECT_TRUE(out.empty());
  trk.update(5.0, {det(4.F, 0.F)}, stationaryEgo(), out);   // 2.2 s gap > max_dt: reset
  const auto resets = trk.stats().resets;
  EXPECT_GE(resets, 1u);
  trk.update(1.0, {det(4.F, 0.F)}, stationaryEgo(), out);   // time jump backwards
  EXPECT_EQ(trk.stats().resets, resets + 1);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].state, TrackState::kTentative);
}
