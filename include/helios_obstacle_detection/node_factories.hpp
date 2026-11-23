// Factories so that the detector and the event recorder can run as two nodes in one process
// (perception_node): one DDS participant receives the ~1 MB LiDAR clouds once for both, so the
// recorder adds no transport load and cannot throttle the driver.
#pragma once

#include <memory>

#include <rclcpp/node.hpp>

namespace helios_obstacle_detection
{
std::shared_ptr<rclcpp::Node> makeDetectorNode(const rclcpp::NodeOptions & options);
std::shared_ptr<rclcpp::Node> makeEventRecorderNode(const rclcpp::NodeOptions & options);
}  // namespace helios_obstacle_detection
