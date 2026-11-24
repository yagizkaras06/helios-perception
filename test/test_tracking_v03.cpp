// v0.3 tracking tests: occlusion-aware lifecycle, re-identification, merge / split, motion onset,
// uncertainty handling, data interruptions and time jumps.
//   1 single-frame detection loss            6 standing object never MOVING
//   2 five-frame detection loss              7 ego-motion uncertainty
//   3 two objects merging and splitting      8 identity kept through occlusions
//   4 person walking along a wall            9 sensor data interruption
//   5 short walk (motion onset latency)     10 timestamp jump
#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <string>
#include <vector>

#include "helios_obstacle_detection/lidar_simulator.hpp"
#include "helios_obstacle_detection/multi_object_tracker.hpp"
#include "helios_obstacle_detection/perception_pipeline.hpp"
#include "helios_obstacle_detection/tracking_metrics.hpp"

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
  d.closest_point = Eigen::Vector3f(x - 0.5F * l, y, -0.05F);
  d.closest_distance = d.closest_point.norm();
  d.center_distance = d.center.norm();
  return d;
}

EgoMotion ego(EgoState state = EgoState::kStationary)
{
  EgoMotion e;
  e.state = state;
  e.valid = state != EgoState::kUnknown;
  e.measured = true;
  e.velocity_cov = Eigen::Matrix3d::Identity() * 1e-4;
  e.delta_cov = Eigen::Matrix3d::Identity() * 1e-6;
  return e;
}

/// Points of a wall segment at x = `x` covering y in [y0, y1] (an occluder in front of a track).
Cloud occluder(float x, float y0, float y1)
{
  Cloud c;
  for (float y = y0; y <= y1; y += 0.02F) {
    for (float z = -0.8F; z <= 0.8F; z += 0.2F) {
      PointT p;
      p.x = x;
      p.y = y;
      p.z = z;
      c.push_back(p);
    }
  }
  return c;
}

const TrackOutput * find(const std::vector<TrackOutput> & out, std::uint32_t id)
{
  for (const auto & t : out) {
    if (t.id == id) {
      return &t;
    }
  }
  return nullptr;
}

/// Walker along +y at x = 6 m (1 m/s), observed for `visible_s`; returns its track id.
std::uint32_t walk(MultiObjectTracker & trk, double t0, double visible_s, std::vector<TrackOutput> & out)
{
  std::uint32_t id = 0;
  for (double t = 0.0; t <= visible_s + 1e-6; t += 0.1) {
    trk.update(t0 + t, {det(6.F, static_cast<float>(-3.0 + t))}, ego(), out);
    if (!out.empty()) {
      id = out[0].id;
    }
  }
  return id;
}

PipelineConfig pipelineConfig()
{
  PipelineConfig pc;
  pc.filter.max_range = 30.F;
  pc.filter.roi = AxisBox{-30.F, 30.F, -30.F, 30.F, -2.5F, 2.5F};
  pc.ground.expected_height = -0.9F;
  pc.threads = 2;
  return pc;
}

sim::MotSummary runScenario(const std::string & name, double duration, unsigned seed = 1)
{
  PipelineConfig pc = pipelineConfig();
  PerceptionPipeline pipe(pc);
  sim::SimConfig sc;
  sc.seed = 42U + seed;
  sim::LidarSimulator lidar(sc);
  sim::Scene scene = sim::makeScenario(name, seed);
  sim::MotEvaluator eval(1.0, 10, pc.risk.horizon_s);
  const int frames = static_cast<int>(duration / 0.1);
  for (int k = 0; k < frames; ++k) {
    std::vector<int> hits;
    auto msg = lidar.scan(scene, hits);
    FrameOutput out;
    std::string err;
    EXPECT_TRUE(pipe.process(msg, 1000.0 + scene.time, out, err)) << err;
    if (k >= 5) {
      eval.addFrame(lidar.groundTruth(scene, hits, pc.risk.ego_radius, pc.risk.min_closing_speed),
        out.tracks, out.risks, scene.time);
    }
    scene.step(0.1);
  }
  return eval.summary();
}

}  // namespace

// 1 -------------------------------------------------------------------------------------------
TEST(TrackingV03, SingleFrameLossKeepsConfirmedTrack)
{
  MultiObjectTracker trk(TrackerConfig{});
  std::vector<TrackOutput> out;
  const auto id = walk(trk, 10.0, 1.0, out);
  ASSERT_NE(id, 0u);
  trk.update(11.1, {}, ego(), out);                       // one scan without the detection
  const auto * t = find(out, id);
  ASSERT_NE(t, nullptr);
  EXPECT_EQ(t->state, TrackState::kOccluded);
  EXPECT_TRUE(t->predicted);
  EXPECT_FALSE(t->updated);
  EXPECT_NE(t->velocity_status, VelocityStatus::kReliable);   // predictions are not trusted
  trk.update(11.2, {det(6.F, -1.8F)}, ego(), out);        // back
  t = find(out, id);
  ASSERT_NE(t, nullptr);
  EXPECT_EQ(t->state, TrackState::kConfirmed);
  EXPECT_FALSE(t->predicted);
}

// 2 -------------------------------------------------------------------------------------------
TEST(TrackingV03, FiveFrameLossKeepsIdentityAndFreezesMotion)
{
  MultiObjectTracker trk(TrackerConfig{});
  std::vector<TrackOutput> out;
  const auto id = walk(trk, 10.0, 2.0, out);
  ASSERT_EQ(find(out, id)->motion, MotionState::kMoving);
  const Cloud wall = occluder(3.F, -1.F, 1.F);            // nearer object covering the walker
  for (int k = 1; k <= 5; ++k) {
    trk.update(12.0 + 0.1 * k, {}, ego(), out, &wall);
    const auto * t = find(out, id);
    ASSERT_NE(t, nullptr) << "frame " << k;
    EXPECT_EQ(t->state, TrackState::kOccluded);
    EXPECT_EQ(t->occlusion, OcclusionReason::kOccluder);
    EXPECT_EQ(t->motion, MotionState::kMoving);           // frozen, not re-derived
    // the prediction follows the walk (y = -1 + 1 m/s * dt)
    EXPECT_NEAR(t->center.y(), -1.0 + 0.1 * k, 0.3);
  }
  trk.update(12.6, {det(6.F, -0.4F)}, ego(), out);
  ASSERT_NE(find(out, id), nullptr);
  EXPECT_EQ(find(out, id)->state, TrackState::kConfirmed);
}

TEST(TrackingV03, LongLossGoesLostAndIsReidentified)
{
  MultiObjectTracker trk(TrackerConfig{});
  std::vector<TrackOutput> out;
  const auto id = walk(trk, 10.0, 2.0, out);
  // 1.5 s without any detection and nothing in front: unexplained -> LOST (not published)
  bool seen_lost = false;
  for (int k = 1; k <= 15; ++k) {
    trk.update(12.0 + 0.1 * k, {}, ego(), out);
    seen_lost = seen_lost || find(out, id) == nullptr;
  }
  EXPECT_TRUE(seen_lost);
  EXPECT_GE(trk.stats().lost, 1u);
  // reappears where the walk leads: same identity (re-identification)
  trk.update(13.6, {det(6.F, 0.6F)}, ego(), out);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].id, id);
  EXPECT_GE(trk.stats().reidentified_total, 1u);
}

// 3 -------------------------------------------------------------------------------------------
TEST(TrackingV03, MergeAndSplitKeepsIdentities)
{
  const auto s = runScenario("merge_split", 20.0);
  EXPECT_GT(s.idf1, 0.88) << s.toMarkdown("merge_split");
  EXPECT_LE(s.idsw, 6u) << s.toMarkdown("merge_split");
  EXPECT_EQ(s.never_moved_as_moving, 0u);
}

// 4 -------------------------------------------------------------------------------------------
TEST(TrackingV03, PersonWalkingAlongWall)
{
  const auto s = runScenario("wall_walk", 25.0);
  EXPECT_EQ(s.idsw, 0u) << s.toMarkdown("wall_walk");
  EXPECT_EQ(s.fn, 0u);
  const double moving_share = static_cast<double>(s.moving_as_moving) /
    std::max<std::uint64_t>(s.moving_as_moving + s.moving_as_static + s.moving_as_unknown, 1);
  EXPECT_GT(moving_share, 0.85);
  EXPECT_EQ(s.moving_as_static, 0u);                      // the walker never becomes background
}

// 5 -------------------------------------------------------------------------------------------
TEST(TrackingV03, ShortWalksAreDetectedEarly)
{
  const auto s = runScenario("start_stop", 30.0);
  ASSERT_GE(s.onset_events, 20u);
  EXPECT_GE(s.onset_latency_s.size(), static_cast<std::size_t>(0.85 * s.onset_events));
  EXPECT_LE(sim::MotSummary::median(s.onset_latency_s), 0.35);
  EXPECT_EQ(s.never_moved_as_moving, 0u);
}

// 6 -------------------------------------------------------------------------------------------
TEST(TrackingV03, StandingObjectIsNeverMoving)
{
  // noisy box centre and partial views (shrinking box) of a static object, still sensor
  MultiObjectTracker trk(TrackerConfig{});
  std::vector<TrackOutput> out;
  std::mt19937 rng(3);
  std::normal_distribution<float> n(0.F, 0.04F);
  std::uniform_real_distribution<float> u(0.F, 1.F);
  for (int k = 0; k < 200; ++k) {
    const bool partial = u(rng) < 0.3F;
    const float l = partial ? 0.5F : 1.0F;
    const float x = 5.F + (partial ? -0.25F : 0.F) + n(rng);
    trk.update(20.0 + 0.1 * k, {det(x, 1.F + n(rng), l, 0.8F)}, ego(), out);
    for (const auto & t : out) {
      EXPECT_NE(t.motion, MotionState::kMoving) << "k=" << k;
    }
  }
}

// 7 -------------------------------------------------------------------------------------------
TEST(TrackingV03, EgoMotionUncertaintyIsReported)
{
  MultiObjectTracker trk(TrackerConfig{});
  std::vector<TrackOutput> out;
  // no ground reference: no absolute motion claims, relative motion only
  for (int k = 0; k < 15; ++k) {
    trk.update(30.0 + 0.1 * k, {det(6.F, -3.F + 0.1F * k)}, ego(EgoState::kUnknown), out);
  }
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].motion, MotionState::kUnknown);
  EXPECT_EQ(out[0].velocity_status, VelocityStatus::kUnknown);
  EXPECT_TRUE(out[0].relative_only);
  EXPECT_FALSE(out[0].velocity_reliable);
  // unreliable ego motion: velocity flagged, no new absolute decision
  MultiObjectTracker trk2(TrackerConfig{});
  for (int k = 0; k < 25; ++k) {
    trk2.update(40.0 + 0.1 * k, {det(6.F, -3.F + 0.1F * k)}, ego(EgoState::kUnreliable), out);
  }
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].velocity_status, VelocityStatus::kUnreliable);
  EXPECT_NE(out[0].motion, MotionState::kMoving);
  EXPECT_FALSE(out[0].heading_valid);
}

// 8 -------------------------------------------------------------------------------------------
TEST(TrackingV03, IdentityKeptThroughOcclusions)
{
  const auto s = runScenario("occlusion", 25.0);
  EXPECT_LE(s.idsw, 3u) << s.toMarkdown("occlusion");
  EXPECT_GT(s.idf1, 0.95) << s.toMarkdown("occlusion");
  EXPECT_EQ(s.never_moved_as_moving, 0u);
}

// 9 -------------------------------------------------------------------------------------------
TEST(TrackingV03, SensorDataInterruption)
{
  TrackerConfig cfg;
  MultiObjectTracker trk(cfg);
  std::vector<TrackOutput> out;
  const auto id = walk(trk, 50.0, 1.5, out);
  // 0.5 s without scans (shorter than max_dt): the track survives as a prediction
  trk.update(52.0, {det(6.F, -1.0F)}, ego(), out);
  ASSERT_NE(find(out, id), nullptr);
  EXPECT_EQ(trk.stats().resets, 0u);
  // a gap longer than max_dt resets the tracker (no stale predictions)
  trk.update(52.0 + cfg.max_dt + 0.5, {det(6.F, 0.F)}, ego(), out);
  EXPECT_EQ(trk.stats().resets, 1u);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].state, TrackState::kTentative);
  EXPECT_NE(out[0].id, id);
}

// 10 ------------------------------------------------------------------------------------------
TEST(TrackingV03, TimestampJumpResets)
{
  MultiObjectTracker trk(TrackerConfig{});
  std::vector<TrackOutput> out;
  const auto id = walk(trk, 60.0, 1.0, out);
  trk.update(10.0, {det(6.F, -2.F)}, ego(), out);         // jump backwards (bag loop)
  EXPECT_EQ(trk.stats().resets, 1u);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_NE(out[0].id, id);                               // ids are never reused
  EXPECT_EQ(out[0].state, TrackState::kTentative);
  EXPECT_EQ(out[0].motion, MotionState::kUnknown);
}

// Frame summary: a static box next to a still sensor (the desk case) keeps its per-track
// distance grade (CRITICAL) but does not drive max_risk while the ego is STATIONARY.
TEST(TrackingV03, StaticNeighbourDoesNotDriveSummaryWhileStationary)
{
  for (const bool skip : {true, false}) {
    PipelineConfig pc = pipelineConfig();
    pc.risk.summary_ignore_static_when_stationary = skip;
    PerceptionPipeline pipe(pc);
    sim::SimConfig sc;
    sim::LidarSimulator lidar(sc);
    sim::Scene scene;
    sim::SimBox desk;
    desk.position = Eigen::Vector2d(1.3, 0.0);   // near face at 0.8 m -> clearance 0.3 m
    desk.size = Eigen::Vector3d(1.0, 1.2, 1.0);
    desk.evaluate = false;
    scene.boxes.push_back(desk);
    // structure for the LiDAR ego estimate (walls + far static clutter, all outside 2.5 m)
    auto add = [&](double x, double y, double l, double w) {
        sim::SimBox b;
        b.position = Eigen::Vector2d(x, y);
        b.size = Eigen::Vector3d(l, w, 3.0);
        b.evaluate = false;
        scene.boxes.push_back(b);
      };
    add(5.0, 12.0, 40.0, 0.3);
    add(-12.0, 0.0, 0.3, 24.0);
    add(9.0, 4.0, 1.0, 1.0);
    add(-6.0, -5.0, 2.0, 2.0);
    add(4.0, -7.0, 0.4, 0.4);
    FrameOutput out;
    for (int k = 0; k < 160; ++k) {
      std::vector<int> hits;
      auto msg = lidar.scan(scene, hits);
      std::string err;
      out = FrameOutput{};
      ASSERT_TRUE(pipe.process(msg, 1000.0 + scene.time, out, err)) << err;
      scene.step(0.1);
    }
    ASSERT_EQ(out.ego.state, EgoState::kStationary);
    bool found = false;
    for (std::size_t i = 0; i < out.tracks.size(); ++i) {
      if (out.tracks[i].state != TrackState::kTentative &&
        out.tracks[i].motion == MotionState::kStatic && out.tracks[i].closest_distance < 1.F)
      {
        found = true;
        EXPECT_EQ(out.risks[i].level, RiskLevel::kCritical);   // per-track grade unchanged
      }
    }
    ASSERT_TRUE(found);
    if (skip) {
      EXPECT_EQ(out.max_risk, RiskLevel::kNone);
      EXPECT_EQ(out.max_risk_track, -1);
    } else {
      EXPECT_EQ(out.max_risk, RiskLevel::kCritical);
    }
  }
}
