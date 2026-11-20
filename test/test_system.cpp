// System tests: full PerceptionPipeline on ray-cast Helios-32-like scans with ground truth.
#include <gtest/gtest.h>

#include <cmath>

#include "helios_obstacle_detection/lidar_simulator.hpp"
#include "helios_obstacle_detection/perception_pipeline.hpp"
#include "helios_obstacle_detection/tracking_metrics.hpp"

using namespace helios_obstacle_detection;

namespace
{

PipelineConfig config()
{
  PipelineConfig pc;
  pc.filter.max_range = 30.F;
  pc.filter.roi = AxisBox{-30.F, 30.F, -30.F, 30.F, -2.5F, 2.5F};
  pc.ground.expected_height = -0.9F;
  pc.ground.zone_boundaries = {0.F, 8.F, 16.F, 30.F};
  pc.threads = 4;
  // scenarios last 6-10 s: learn the background faster than the 10 s default
  pc.static_map.static_time_s = 2.0F;
  return pc;
}

struct RunResult
{
  sim::MotSummary mot;
  double ego_err_mean{0.0};
  int ego_valid{0};
  int frames{0};
  double max_total_ms{0.0};
};

RunResult run(
  const std::string & scenario, double duration, const PipelineConfig & pc,
  const std::vector<int> & drop_frames = {})
{
  PerceptionPipeline pipe(pc);
  sim::LidarSimulator lidar(sim::SimConfig{});
  sim::Scene scene = sim::makeScenario(scenario);
  sim::MotEvaluator eval(1.0, 10, pc.risk.horizon_s);
  RunResult rr;
  double err_sum = 0.0;
  const int frames = static_cast<int>(duration / 0.1);
  for (int k = 0; k < frames; ++k) {
    const bool dropped = std::find(drop_frames.begin(), drop_frames.end(), k) != drop_frames.end();
    std::vector<int> hits;
    auto msg = lidar.scan(scene, hits);
    if (!dropped) {
      FrameOutput out;
      std::string err;
      EXPECT_TRUE(pipe.process(msg, 500.0 + scene.time, out, err)) << err;
      rr.max_total_ms = std::max(rr.max_total_ms, out.times.total_ms);
      if (k >= 5) {
        eval.addFrame(lidar.groundTruth(scene, hits, pc.risk.ego_radius,
          pc.risk.min_closing_speed), out.tracks, out.risks);
        if (out.ego.valid) {
          err_sum += (out.ego.velocity - Eigen::Vector2d(scene.ego_speed, 0.0)).norm();
          ++rr.ego_valid;
        }
      }
      ++rr.frames;
    }
    scene.step(0.1);
  }
  rr.mot = eval.summary();
  rr.ego_err_mean = rr.ego_valid ? err_sum / rr.ego_valid : 0.0;
  return rr;
}

}  // namespace

TEST(System, CrossingStaticSensor)
{
  const auto r = run("crossing", 10.0, config());
  const auto & m = r.mot;
  SCOPED_TRACE(m.toMarkdown("crossing"));
  EXPECT_GT(m.mota, 0.85);
  EXPECT_LE(m.idsw, 5u);
  EXPECT_LT(m.motp_m, 0.35);
  EXPECT_LT(m.velocity_rmse, 0.5);
  EXPECT_EQ(m.static_as_moving, 0u);
  EXPECT_GT(static_cast<double>(m.moving_as_moving),
    0.7 * static_cast<double>(m.moving_as_moving + m.moving_as_static + m.moving_as_unknown));
  EXPECT_LT(m.heading_mae_deg, 15.0);
  EXPECT_LT(r.ego_err_mean, 0.05);
}

TEST(System, MovingSensorKeepsStaticWorldStatic)
{
  const auto r = run("ego_motion", 8.0, config());
  const auto & m = r.mot;
  SCOPED_TRACE(m.toMarkdown("ego_motion"));
  EXPECT_GT(r.ego_valid, 60);
  EXPECT_LT(r.ego_err_mean, 0.25);
  EXPECT_GT(m.mota, 0.6);
  const double static_total = static_cast<double>(m.static_as_static + m.static_as_moving +
    m.static_as_unknown);
  ASSERT_GT(static_total, 0.0);
  EXPECT_LT(static_cast<double>(m.static_as_moving) / static_total, 0.03);
  EXPECT_GT(static_cast<double>(m.static_as_static) / static_total, 0.7);
}

TEST(System, ApproachTtc)
{
  const auto r = run("approach", 6.0, config());
  const auto & m = r.mot;
  SCOPED_TRACE(m.toMarkdown("approach"));
  ASSERT_GT(m.ttc_samples, 20u);
  EXPECT_LT(m.ttc_mae_s, 0.5);
  // the pessimistic bound should be conservative most of the time
  EXPECT_LT(m.ttc_pessimistic_violations, 0.2);
}

TEST(System, SurvivesFrameDropsAndRansacMethod)
{
  PipelineConfig pc = config();
  pc.ground_method = "ransac_zones";
  // 3 single drops and one 0.5 s outage
  const auto r = run("crossing", 8.0, pc, {20, 21, 22, 23, 24, 40, 55, 70});
  SCOPED_TRACE(r.mot.toMarkdown("crossing+drops"));
  EXPECT_GT(r.mot.mota, 0.7);
  EXPECT_LE(r.mot.idsw, 6u);
}
