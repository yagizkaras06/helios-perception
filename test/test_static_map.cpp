// Unit tests: static background map ("still for N seconds -> static").
#include <gtest/gtest.h>

#include <cmath>

#include "helios_obstacle_detection/static_map.hpp"

using namespace helios_obstacle_detection;

namespace
{
void addBlock(Cloud & c, float x0, float y0, float size, float step = 0.05F)
{
  for (float x = x0; x <= x0 + size + 1e-4F; x += step) {
    for (float y = y0; y <= y0 + size + 1e-4F; y += step) {
      for (float z = -0.8F; z <= 0.8F; z += 0.2F) {
        PointT p;
        p.x = x;
        p.y = y;
        p.z = z;
        c.push_back(p);
      }
    }
  }
}

StaticMapConfig cfg()
{
  StaticMapConfig c;
  c.static_time_s = 10.F;
  return c;
}
}  // namespace

TEST(StaticMap, StillObjectBecomesBackgroundAfterStaticTime)
{
  StaticBackgroundMap map(cfg());
  Cloud wall;
  addBlock(wall, 5.F, 0.F, 0.4F);
  pcl::Indices bg, fg;
  const Eigen::Vector3d pose = Eigen::Vector3d::Zero();
  for (int k = 0; k < 95; ++k) {          // 9.4 s: not yet static
    map.processFrame(wall, pose, 0.1 * k, bg, fg);
  }
  EXPECT_TRUE(bg.empty());
  EXPECT_EQ(fg.size(), wall.size());
  for (int k = 95; k < 115; ++k) {        // > 10 s (voxel statistics refresh every 10 frames)
    map.processFrame(wall, pose, 0.1 * k, bg, fg);
  }
  EXPECT_EQ(bg.size(), wall.size());
  EXPECT_TRUE(fg.empty());
  EXPECT_GT(map.staticVoxelCount(), 0u);
}

TEST(StaticMap, MovingObjectStaysForegroundAndIsSeparatedFromWall)
{
  StaticBackgroundMap map(cfg());
  Cloud wall;
  addBlock(wall, 5.F, 0.F, 0.4F);
  pcl::Indices bg, fg;
  const Eigen::Vector3d pose = Eigen::Vector3d::Zero();
  for (int k = 0; k < 120; ++k) {
    // person walking back and forth along y = 2 .. 4 m (never still for 10 s)
    Cloud c = wall;
    const float y = 2.F + 2.F * std::fabs(std::sin(0.3F * static_cast<float>(k)));
    addBlock(c, 3.F, y, 0.4F);
    map.processFrame(c, pose, 0.1 * k, bg, fg);
  }
  // wall is background, the person is foreground
  EXPECT_EQ(bg.size(), wall.size());
  EXPECT_GT(fg.size(), 100u);
  for (const auto i : fg) {
    EXPECT_LT(static_cast<std::size_t>(i), 2 * wall.size() + 1000);
  }
}

TEST(StaticMap, ObjectThatLeavesIsForgotten)
{
  StaticMapConfig c = cfg();
  c.forget_time_s = 2.F;
  StaticBackgroundMap map(c);
  Cloud box;
  addBlock(box, 4.F, 1.F, 0.4F);
  pcl::Indices bg, fg;
  for (int k = 0; k < 120; ++k) {
    map.processFrame(box, Eigen::Vector3d::Zero(), 0.1 * k, bg, fg);
  }
  EXPECT_EQ(bg.size(), box.size());
  Cloud empty;
  for (int k = 120; k < 160; ++k) {       // gone for 4 s
    map.processFrame(empty, Eigen::Vector3d::Zero(), 0.1 * k, bg, fg);
  }
  map.processFrame(box, Eigen::Vector3d::Zero(), 16.0, bg, fg);   // something new there
  EXPECT_TRUE(bg.empty());
}

TEST(StaticMap, OdometryFrameKeepsBackgroundWhileSensorMoves)
{
  StaticBackgroundMap map(cfg());
  Cloud world;
  addBlock(world, 8.F, 2.F, 0.6F);
  pcl::Indices bg, fg;
  for (int k = 0; k < 110; ++k) {
    const double x = 0.05 * k;           // sensor drives forward 0.5 m/s
    Cloud seen = world;
    for (auto & p : seen) {
      p.x -= static_cast<float>(x);
    }
    map.processFrame(seen, Eigen::Vector3d(x, 0.0, 0.0), 0.1 * k, bg, fg);
  }
  EXPECT_EQ(bg.size(), world.size());
}
