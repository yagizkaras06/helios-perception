// RViz2 MarkerArray construction (boxes, centers, distance labels).
#pragma once

#include <vector>

#include <builtin_interfaces/msg/duration.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>

#include "helios_obstacle_detection/bounding_box.hpp"
#include "helios_obstacle_detection/ground_segmenter.hpp"
#include "helios_obstacle_detection/perception_pipeline.hpp"

namespace helios_obstacle_detection
{

struct Rgb
{
  float r, g, b;
};

/// Distinct colour per per-frame obstacle id.
Rgb colorForId(std::uint32_t id) noexcept;

/// Every call starts with a DELETEALL marker, so stale boxes never survive a frame; markers
/// also carry `lifetime` so they vanish if the pipeline stops publishing.
visualization_msgs::msg::MarkerArray buildObstacleMarkers(
  const std_msgs::msg::Header & header, const std::vector<ObstacleCandidate> & obstacles,
  GroundStatus ground_status, const builtin_interfaces::msg::Duration & lifetime);

struct TrackVizConfig
{
  bool show_tentative{false};
  bool show_static{false};    // draw STATIC tracks (they stay in /tracks and in the risk anyway)
  bool show_unknown{true};    // draw UNKNOWN tracks (dynamic but not moving now / no evidence)
  // While the static background map is learning (first static_time_s) everything is drawn; after
  // that only dynamic objects (MOVING now or moved at some point). Visualisation only: all tracks
  // stay in /tracks and in the risk assessment.
  bool only_dynamic_after_learning{true};
  bool show_trails{true};
  bool show_uncertainty{true};
  bool show_prediction{true};
  float arrow_time_s{1.0F};          // velocity arrow length = speed * this
  float prediction_horizon_s{3.0F};  // relative constant-velocity prediction
  float prediction_step_s{0.25F};
  float ego_radius{0.5F};
  float min_arrow_speed{0.2F};       // [m/s]
};

Rgb colorForRisk(RiskLevel level) noexcept;

/// Track boxes coloured by risk, labels (id, motion, speed, heading, distance, TTC, risk),
/// velocity arrows, predicted relative paths, 2-sigma position ellipses, trails, ego footprint /
/// velocity and a status HUD. Starts with DELETEALL.
visualization_msgs::msg::MarkerArray buildTrackMarkers(
  const std_msgs::msg::Header & header, const FrameOutput & frame, const TrackVizConfig & cfg,
  const builtin_interfaces::msg::Duration & lifetime, const std::string & status_line);

/// Only a DELETEALL marker (used when input data stops).
visualization_msgs::msg::MarkerArray buildClearMarkers(const std_msgs::msg::Header & header);

}  // namespace helios_obstacle_detection
