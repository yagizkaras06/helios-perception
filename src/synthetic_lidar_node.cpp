// Synthetic Helios-32-like point cloud source for demos without a sensor.
//
// Publishes the ray-cast scans of one of the built-in simulator scenarios (the same simulator the
// tests and the benchmark use) as sensor_msgs/PointCloud2 at a fixed rate, in the rslidar_sdk
// layout. It is a data source only: it runs no perception code. The scene restarts every
// `loop_s` seconds (tracks of the previous pass are then dropped by the detector as usual).
//
//   ros2 run helios_obstacle_detection synthetic_lidar
//       --ros-args -p scenario:=crossing -p topic:=/synthetic/rslidar_points
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "helios_obstacle_detection/lidar_simulator.hpp"

namespace sim = helios_obstacle_detection::sim;

class SyntheticLidar : public rclcpp::Node
{
public:
  SyntheticLidar()
  : Node("synthetic_lidar")
  {
    scenario_ = declare_parameter<std::string>("scenario", "crossing");
    seed_ = static_cast<unsigned>(declare_parameter<int>("seed", 7));
    const double rate = declare_parameter<double>("rate_hz", 10.0);
    loop_s_ = declare_parameter<double>("loop_s", 30.0);
    frame_id_ = declare_parameter<std::string>("frame_id", "rslidar");
    const auto topic = declare_parameter<std::string>("topic", "/synthetic/rslidar_points");
    if (!(rate > 0.0 && rate <= 50.0)) {
      throw std::invalid_argument("rate_hz must be in (0, 50]");
    }
    dt_ = 1.0 / rate;
    sim::SimConfig sc;
    sc.seed = 42U + seed_;
    lidar_ = std::make_unique<sim::LidarSimulator>(sc);
    reset();   // throws for an unknown scenario name
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(topic, rclcpp::SensorDataQoS().reliable());
    timer_ = create_wall_timer(std::chrono::duration<double>(dt_), [this] {tick();});
    RCLCPP_INFO(get_logger(), "synthetic scenario '%s' (seed %u) on %s at %.1f Hz, loop %.0f s "
      "- simulated data, not a sensor", scenario_.c_str(), seed_, topic.c_str(), rate, loop_s_);
  }

private:
  void reset()
  {
    scene_ = sim::makeScenario(scenario_, seed_);
  }

  void tick()
  {
    if (loop_s_ > 0.0 && scene_.time >= loop_s_) {
      reset();
    }
    std::vector<int> hits;
    auto msg = lidar_->scan(scene_, hits);
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    pub_->publish(msg);
    scene_.step(dt_);
  }

  std::string scenario_;
  unsigned seed_{7};
  double dt_{0.1};
  double loop_s_{30.0};
  std::string frame_id_;
  std::unique_ptr<sim::LidarSimulator> lidar_;
  sim::Scene scene_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SyntheticLidar>());
  rclcpp::shutdown();
  return 0;
}
