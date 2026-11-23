// perception_node: obstacle_detector + event_recorder as two nodes in ONE process.
// Both share one DDS participant, so the large LiDAR clouds reach this process once and the
// recorder can subscribe reliably without adding transport load or throttling the driver.
// Node names stay obstacle_detector / event_recorder; namespace and parameter files apply to both.
#include <exception>

#include <rclcpp/rclcpp.hpp>

#include "helios_obstacle_detection/node_factories.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int rc = 0;
  try {
    rclcpp::NodeOptions opt;
    auto detector = helios_obstacle_detection::makeDetectorNode(opt);
    auto recorder = helios_obstacle_detection::makeEventRecorderNode(opt);
    // each node keeps its default (mutually exclusive) callback group: the recorder's callbacks
    // are serialised, and never delay the detector's input callback
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
    exec.add_node(detector);
    exec.add_node(recorder);
    exec.spin();
    exec.remove_node(recorder);
    exec.remove_node(detector);
    recorder.reset();   // closes a running event (shutdown) and flushes the writer
    detector.reset();   // joins worker + viz threads
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("perception_node"), "Fatal: %s", e.what());
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
