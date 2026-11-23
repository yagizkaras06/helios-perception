// Collision warning: approaching objects on a collision course warn, passing / static / uncertain
// objects do not; hysteresis; end-to-end on the simulated "approach" scenario.
#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "helios_obstacle_detection/collision_warning.hpp"
#include "helios_obstacle_detection/lidar_simulator.hpp"
#include "helios_obstacle_detection/perception_pipeline.hpp"

using namespace helios_obstacle_detection;

namespace
{

/// Track at (x, y) with relative = ground velocity (still sensor), reliable unless stated.
TrackOutput track(float x, float y, float vx, float vy, bool reliable = true,
  MotionState motion = MotionState::kMoving)
{
  TrackOutput t;
  t.id = 7;
  t.state = TrackState::kConfirmed;
  t.motion = motion;
  t.ever_moved = motion == MotionState::kMoving;
  t.max_displacement = motion == MotionState::kMoving ? 2.F : 0.F;
  t.center = Eigen::Vector3f(x, y, 0.F);
  t.size = Eigen::Vector3f(0.5F, 0.5F, 1.7F);
  const Eigen::Vector2f c(x, y);
  const Eigen::Vector2f cp = c - 0.25F * c.normalized();
  t.closest_point = Eigen::Vector3f(cp.x(), cp.y(), 0.F);
  t.closest_distance = cp.norm();
  t.velocity = t.relative_velocity = Eigen::Vector2f(vx, vy);
  t.velocity_cov = t.relative_velocity_cov = 0.01F * Eigen::Matrix2f::Identity();
  t.speed = t.velocity.norm();
  t.speed_sigma = 0.1F;
  t.velocity_reliable = reliable;
  t.velocity_status = reliable ? VelocityStatus::kReliable : VelocityStatus::kUnreliable;
  return t;
}

CollisionWarning step(CollisionWarningFilter & f, double stamp, const TrackOutput & t)
{
  const RiskConfig rc;
  return f.update(stamp, {t}, {assessRisk(t, rc)});
}

CollisionWarningFilter makeFilter()
{
  return CollisionWarningFilter(WarningConfig{}, RiskConfig{});
}

}  // namespace

TEST(CollisionWarning, HeadOnApproachWarnsWithHysteresis)
{
  CollisionWarningFilter f = makeFilter();
  // 1.5 m/s straight at the sensor from 4 m: TTC ~2.2 s -> WARNING after 2 scans
  EXPECT_EQ(step(f, 0.0, track(4.F, 0.F, -1.5F, 0.F)).level, WarningLevel::kClear);
  const auto w = step(f, 0.1, track(3.85F, 0.F, -1.5F, 0.F));
  EXPECT_EQ(w.level, WarningLevel::kWarning);
  EXPECT_TRUE(w.collision_predicted);
  EXPECT_EQ(w.track_id, 7u);
  EXPECT_LT(w.ttc_pessimistic, w.ttc);
  EXPECT_NE(w.text.find("CARPISMA RISKI VAR"), std::string::npos);
  // object stops: the banner is held for hold_s, then clears
  const TrackOutput still = track(3.7F, 0.F, 0.F, 0.F, true, MotionState::kUnknown);
  EXPECT_EQ(step(f, 0.5, still).level, WarningLevel::kWarning);
  EXPECT_EQ(step(f, 1.3, still).level, WarningLevel::kClear);
}

TEST(CollisionWarning, FarApproachIsCaution)
{
  CollisionWarningFilter f = makeFilter();
  step(f, 0.0, track(8.F, 0.F, -1.5F, 0.F));    // TTC ~5 s
  EXPECT_EQ(step(f, 0.1, track(7.85F, 0.F, -1.5F, 0.F)).level, WarningLevel::kCaution);
}

TEST(CollisionWarning, PassingObjectDoesNotWarn)
{
  CollisionWarningFilter f = makeFilter();
  // walks across 4 m ahead, 3 m to the side: never reaches the ego circle
  for (int k = 0; k < 30; ++k) {
    const auto w = step(f, 0.1 * k, track(4.F, 3.F - 0.0F * k, -1.0F, 0.F));
    EXPECT_FALSE(w.collision_predicted) << k;
    EXPECT_NE(w.level, WarningLevel::kWarning) << k;
  }
}

TEST(CollisionWarning, StaticObjectNextToStillSensorDoesNotWarn)
{
  CollisionWarningFilter f = makeFilter();
  for (int k = 0; k < 20; ++k) {   // the desk next to the LiDAR (distance-only CRITICAL risk)
    EXPECT_EQ(step(f, 0.1 * k, track(0.6F, 0.3F, 0.F, 0.F, true, MotionState::kStatic)).level,
      WarningLevel::kClear);
  }
}

TEST(CollisionWarning, NoisyVelocityOfStillObjectDoesNotWarn)
{
  CollisionWarningFilter f = makeFilter();
  // UNKNOWN (not moving) object whose KF velocity is noise pointing at the sensor
  for (int k = 0; k < 20; ++k) {
    EXPECT_EQ(step(f, 0.1 * k, track(3.F, 0.F, -0.9F, 0.F, true, MotionState::kUnknown)).level,
      WarningLevel::kClear) << k;
  }
}

TEST(CollisionWarning, UnreliableVelocityNeverPredictsCollision)
{
  CollisionWarningFilter f = makeFilter();
  for (int k = 0; k < 10; ++k) {
    const auto w = step(f, 0.1 * k, track(3.F - 0.15F * k, 0.F, -1.5F, 0.F, false));
    EXPECT_EQ(w.level, WarningLevel::kClear);
    EXPECT_FALSE(w.collision_predicted);
  }
}

TEST(CollisionWarning, SimulatedApproachWarnsBeforeImpact)
{
  PipelineConfig pc;
  pc.filter.max_range = 30.F;
  pc.filter.roi = AxisBox{-30.F, 30.F, -30.F, 30.F, -2.5F, 2.5F};
  pc.ground.expected_height = -0.9F;
  pc.threads = 2;
  PerceptionPipeline pipe(pc);
  sim::LidarSimulator lidar(sim::SimConfig{});
  sim::Scene scene = sim::makeScenario("approach");   // object from 20 m at 2 m/s
  double first_warning = -1.0, first_caution = -1.0;
  for (int k = 0; k < 85; ++k) {
    std::vector<int> hits;
    auto msg = lidar.scan(scene, hits);
    FrameOutput out;
    std::string err;
    ASSERT_TRUE(pipe.process(msg, 100.0 + scene.time, out, err)) << err;
    if (out.warning.level >= WarningLevel::kCaution && first_caution < 0.0) {
      first_caution = scene.time;
    }
    if (out.warning.level == WarningLevel::kWarning && first_warning < 0.0) {
      first_warning = scene.time;
      EXPECT_TRUE(out.warning.collision_predicted);
    }
    scene.step(0.1);
  }
  // contact with the 0.5 m ego circle at ~ (20 - 0.5 - 0.5) / 2 = 9.5 s
  ASSERT_GE(first_caution, 0.0);
  ASSERT_GE(first_warning, 0.0);
  EXPECT_LT(first_caution, first_warning);
  EXPECT_LT(first_warning, 9.5 - 2.0);   // at least 2 s before contact
}

// RViz (Ogre MovableText) aborts on blank text: no TEXT marker may ever be empty / whitespace,
// in any warning state; line markers must be finite.
#include "helios_obstacle_detection/marker_builder.hpp"

TEST(CollisionWarning, MarkersNeverCarryBlankTextOrNaN)
{
  CollisionWarningFilter f = makeFilter();
  for (int level = 0; level < 3; ++level) {
    FrameOutput fr;
    fr.tracks = {track(level == 0 ? 6.F : 2.5F, 0.F, level == 0 ? 0.F : -1.5F, 0.F)};
    const RiskConfig rc;
    fr.risks = {assessRisk(fr.tracks[0], rc)};
    for (int k = 0; k < 3; ++k) {
      fr.warning = f.update(10.0 * level + 0.1 * k, fr.tracks, fr.risks);
    }
    const auto arr = buildTrackMarkers(std_msgs::msg::Header{}, fr, TrackVizConfig{},
        builtin_interfaces::msg::Duration{}, "");
    for (const auto & m : arr.markers) {
      if (m.type == visualization_msgs::msg::Marker::TEXT_VIEW_FACING && m.action == 0) {
        EXPECT_NE(m.text.find_first_not_of(" \t\n"), std::string::npos) << m.ns << " " << m.id;
      }
      for (const auto & p : m.points) {
        EXPECT_TRUE(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) << m.ns;
      }
    }
  }
}
