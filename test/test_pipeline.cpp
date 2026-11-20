// Synthetic-data unit tests for the algorithm library (no ROS graph needed).
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <random>

#include <pcl/common/io.h>
#include <sensor_msgs/msg/point_field.hpp>

#include "helios_obstacle_detection/bounding_box.hpp"
#include "helios_obstacle_detection/cloud_processor.hpp"
#include "helios_obstacle_detection/ground_segmenter.hpp"
#include "helios_obstacle_detection/obstacle_clusterer.hpp"
#include "helios_obstacle_detection/performance_monitor.hpp"

using namespace helios_obstacle_detection;

namespace
{

void addGround(Cloud & c, float z, float extent, float step, float slope_x = 0.F)
{
  for (float x = -extent; x <= extent; x += step) {
    for (float y = -extent; y <= extent; y += step) {
      PointT p;
      p.x = x;
      p.y = y;
      p.z = z + slope_x * x;
      p.intensity = 10.F;
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

GroundConfig defaultGround()
{
  GroundConfig g;
  g.zone_boundaries = {0.F, 8.F, 16.F, 30.F};
  return g;
}

}  // namespace

TEST(BoundingBox, AabbCenterSizeAndDistances)
{
  Cloud c;
  addBox(c, {2.F, -0.5F, -0.9F}, {3.F, 0.5F, 0.1F}, 0.1F);
  pcl::Indices idx(c.size());
  for (std::size_t i = 0; i < c.size(); ++i) {idx[i] = static_cast<pcl::index_t>(i);}
  const auto o = computeAabb(c, idx);
  EXPECT_NEAR(o.center.x(), 2.5F, 1e-4);
  EXPECT_NEAR(o.center.y(), 0.0F, 1e-4);
  EXPECT_NEAR(o.center.z(), -0.4F, 1e-4);
  EXPECT_NEAR(o.size.x(), 1.0F, 1e-4);
  EXPECT_NEAR(o.size.y(), 1.0F, 1e-4);
  EXPECT_NEAR(o.size.z(), 1.0F, 1e-4);
  // closest point is (2, 0, 0) or nearby on the near face
  EXPECT_NEAR(o.closest_distance, 2.0F, 0.01F);
  EXPECT_NEAR(o.center_distance, std::sqrt(2.5F * 2.5F + 0.4F * 0.4F), 1e-4);
  EXPECT_LT(o.closest_distance, o.center_distance);
}

TEST(CloudProcessor, FiltersNanRangeAndRoi)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.height = 1;
  msg.width = 5;
  msg.point_step = 16;
  msg.row_step = 80;
  msg.is_bigendian = false;
  const char * names[4] = {"x", "y", "z", "intensity"};
  for (int i = 0; i < 4; ++i) {
    sensor_msgs::msg::PointField f;
    f.name = names[i];
    f.offset = static_cast<std::uint32_t>(4 * i);
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.count = 1;
    msg.fields.push_back(f);
  }
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float pts[5][4] = {
    {1.F, 0.F, 0.F, 5.F},     // ok
    {nan, 0.F, 0.F, 5.F},     // NaN
    {0.1F, 0.F, 0.F, 5.F},    // too close
    {5.F, 0.F, 9.F, 5.F},     // outside ROI z
    {3.F, 1.F, -0.5F, 7.F}};  // ok
  msg.data.resize(80);
  std::memcpy(msg.data.data(), pts, 80);

  CloudFilterConfig cfg;
  cfg.min_range = 0.5F;
  cfg.max_range = 40.F;
  cfg.roi = {-40.F, 40.F, -40.F, 40.F, -3.F, 3.F};
  CloudProcessor proc(cfg);
  Cloud out;
  FilterStats st;
  std::string err;
  ASSERT_TRUE(proc.convertAndFilter(msg, out, st, err)) << err;
  EXPECT_EQ(out.size(), 2u);
  EXPECT_EQ(st.non_finite, 1u);
  EXPECT_EQ(st.range_rejected, 1u);
  EXPECT_EQ(st.roi_rejected, 1u);
  EXPECT_FLOAT_EQ(out[1].intensity, 7.F);

  msg.fields.erase(msg.fields.begin());  // drop "x"
  EXPECT_FALSE(proc.convertAndFilter(msg, out, st, err));
}

TEST(GroundSegmenter, SeparatesBoxFromFlatGround)
{
  auto cloud = std::make_shared<Cloud>();
  addGround(*cloud, -0.9F, 12.F, 0.2F);
  addBox(*cloud, {4.F, -0.4F, -0.85F}, {4.6F, 0.4F, 0.2F}, 0.1F);
  GroundSegmenter seg(defaultGround());
  GroundResult r;
  seg.segment(cloud, r);
  EXPECT_EQ(r.status, GroundStatus::kReliable) << r.detail;
  EXPECT_GT(r.obstacle.size(), 50u);
  for (auto i : r.obstacle) {
    EXPECT_GT((*cloud)[static_cast<std::size_t>(i)].z, -0.9F + 0.12F);
  }
}

TEST(GroundSegmenter, RejectsTableTopLargerThanGround)
{
  // Large dense horizontal surface at z=-0.2 (table / platform) plus sparser real ground.
  auto cloud = std::make_shared<Cloud>();
  addGround(*cloud, -0.9F, 7.5F, 0.35F);
  for (float x = 1.F; x <= 3.F; x += 0.04F) {
    for (float y = -1.F; y <= 1.F; y += 0.04F) {
      PointT p;
      p.x = x;
      p.y = y;
      p.z = -0.6F;
      cloud->push_back(p);
    }
  }
  GroundConfig g = defaultGround();
  g.height_tolerance = 0.2F;
  GroundSegmenter seg(g);
  GroundResult r;
  seg.segment(cloud, r);
  ASSERT_NE(r.status, GroundStatus::kFailed) << r.detail;
  const auto & c = r.zones[0].coefficients;
  EXPECT_NEAR(-c[3] / c[2], -0.9F, 0.05F) << r.detail;  // plane height at origin
  EXPECT_GE(r.zones[0].rejected_planes, 1) << r.detail;
}

TEST(GroundSegmenter, FollowsSlope)
{
  auto cloud = std::make_shared<Cloud>();
  addGround(*cloud, -0.9F, 20.F, 0.3F, 0.05F);  // ~2.9 deg uphill along x
  addBox(*cloud, {14.F, -0.4F, -0.9F + 0.7F}, {14.6F, 0.4F, 0.9F}, 0.1F);
  GroundSegmenter seg(defaultGround());
  GroundResult r;
  seg.segment(cloud, r);
  EXPECT_EQ(r.status, GroundStatus::kReliable) << r.detail;
  // at x=19 the ground is ~0.95 m above the level expectation: must still be ground
  std::size_t far_ground_as_obstacle = 0;
  for (auto i : r.obstacle) {
    const auto & p = (*cloud)[static_cast<std::size_t>(i)];
    if (p.x > 17.F && std::fabs(p.z - (-0.9F + 0.05F * p.x)) < 0.02F) {
      ++far_ground_as_obstacle;
    }
  }
  EXPECT_EQ(far_ground_as_obstacle, 0u) << r.detail;
}

TEST(GroundSegmenter, ReportsFailureWithoutGround)
{
  auto cloud = std::make_shared<Cloud>();
  addBox(*cloud, {3.F, -1.F, 0.5F}, {4.F, 1.F, 1.5F}, 0.1F);  // nothing near expected ground
  GroundSegmenter seg(defaultGround());
  GroundResult r;
  seg.segment(cloud, r);
  EXPECT_EQ(r.status, GroundStatus::kFailed);
}

TEST(Pipeline, TwoSeparatedObstacles)
{
  auto cloud = std::make_shared<Cloud>();
  addGround(*cloud, -0.9F, 10.F, 0.2F);
  addBox(*cloud, {3.F, 1.F, -0.85F}, {3.5F, 1.5F, 0.F}, 0.1F);
  addBox(*cloud, {6.F, -2.F, -0.85F}, {6.6F, -1.4F, 0.5F}, 0.1F);
  GroundSegmenter seg(defaultGround());
  GroundResult r;
  seg.segment(cloud, r);
  auto obst = std::make_shared<Cloud>();
  pcl::copyPointCloud(*cloud, r.obstacle, *obst);
  ObstacleClusterer cl(ClusterConfig{});
  const auto clusters = cl.extract(obst);
  const auto boxes = buildObstacles(*obst, clusters, BoxFilterConfig{});
  ASSERT_EQ(boxes.size(), 2u);
  EXPECT_EQ(boxes[0].id, 0u);
  EXPECT_LT(boxes[0].closest_distance, boxes[1].closest_distance);
  EXPECT_NEAR(boxes[0].center.x(), 3.25F, 0.06F);
}

TEST(Performance, Percentile)
{
  RollingStat s(100);
  for (int i = 1; i <= 100; ++i) {s.add(i);}
  EXPECT_DOUBLE_EQ(s.percentile(95), 95.0);
  EXPECT_DOUBLE_EQ(s.max(), 100.0);
  EXPECT_DOUBLE_EQ(s.mean(), 50.5);
  s.add(1000);  // ring buffer drops the oldest (1)
  EXPECT_DOUBLE_EQ(s.max(), 1000.0);
}
