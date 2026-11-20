// Synthetic Helios-32-like LiDAR (ray casting) with ground truth, for tests and benchmarks.
//
// 32 beams (approximation of the Helios 32 vertical layout: dense near the horizon,
// +15 .. -55 deg), 1800 azimuth steps (0.2 deg), Gaussian range noise and random dropouts.
// Scene: flat ground plane + oriented boxes (pedestrians, cars, pillars, walls) moving with
// constant velocity / turn rate, sensor on a moving platform (planar). Output layout matches the
// rslidar_sdk driver (organised PointCloud2, width 32 x height 1800, XYZI float32, NaN = miss).
// Limitations: no intra-scan motion (all rays at the scan time), no occlusion-dependent
// intensity, perfect planar ground.
#pragma once

#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "helios_obstacle_detection/types.hpp"

namespace helios_obstacle_detection
{
namespace sim
{

struct SimBox
{
  int id{0};
  Eigen::Vector2d position{Eigen::Vector2d::Zero()};  // world, box centre (x, y)
  Eigen::Vector3d size{1.0, 1.0, 1.0};                // length (along yaw), width, height
  double yaw{0.0};
  Eigen::Vector2d velocity{Eigen::Vector2d::Zero()};  // world
  double yaw_rate{0.0};
  bool evaluate{true};    // false: walls / clutter (not part of the MOT ground truth)
  bool moving() const {return velocity.norm() > 0.05;}
};

struct SimConfig
{
  double sensor_height{0.9};       // [m] LiDAR above ground
  int azimuth_steps{1800};
  std::vector<double> elevations_deg;  // empty -> Helios-32-like default
  double range_noise{0.02};        // [m] 1 sigma
  double dropout{0.02};
  double min_range{0.2};
  double max_range{100.0};
  unsigned seed{42};
};

struct Scene
{
  double time{0.0};
  Eigen::Vector3d ego_pose{Eigen::Vector3d::Zero()};  // world x, y, yaw
  double ego_speed{0.0};                              // forward [m/s]
  double ego_yaw_rate{0.0};
  std::vector<SimBox> boxes;

  void step(double dt);
};

/// Ground truth of one evaluated object in the sensor frame at scan time.
struct GroundTruth
{
  int id{0};
  Eigen::Vector2d position{Eigen::Vector2d::Zero()};  // box centre, sensor frame
  Eigen::Vector3d size{Eigen::Vector3d::Zero()};
  double yaw{0.0};                                    // sensor frame
  Eigen::Vector2d velocity{Eigen::Vector2d::Zero()};  // ground-relative, sensor axes
  Eigen::Vector2d relative_velocity{Eigen::Vector2d::Zero()};
  Eigen::Vector2d closest_point{Eigen::Vector2d::Zero()};
  bool moving{false};
  bool evaluate{true};                                // false: wall / clutter, "don't care"
  int hits{0};                                        // LiDAR returns on the object
  double ttc{0.0};                                    // with the same definition as RiskConfig
};

class LidarSimulator
{
public:
  explicit LidarSimulator(const SimConfig & config);

  /// Ray-casts the scene. `hits` gets one entry per scene box.
  sensor_msgs::msg::PointCloud2 scan(const Scene & scene, std::vector<int> & hits);

  /// Ground truth of all boxes (sensor frame); non-evaluated ones are flagged.
  std::vector<GroundTruth> groundTruth(
    const Scene & scene, const std::vector<int> & hits, double ego_radius,
    double min_closing_speed) const;

  const std::vector<double> & elevations() const noexcept {return elev_;}

private:
  SimConfig config_;
  std::vector<double> elev_;
  std::vector<double> cos_az_, sin_az_;
  std::mt19937 rng_;
};

/// Named scenarios: "crossing" (static sensor, pedestrians + vehicles + static boxes),
/// "ego_motion" (sensor driving 3 m/s with a slight turn through static clutter + movers),
/// "approach" (static sensor, one object driving straight at it - TTC check).
Scene makeScenario(const std::string & name, unsigned seed = 7);

}  // namespace sim
}  // namespace helios_obstacle_detection
