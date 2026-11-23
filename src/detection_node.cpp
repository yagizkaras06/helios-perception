// helios_obstacle_detection: perception-only node (detection, tracking, prediction, risk).
// It NEVER publishes actuation commands (throttle / brake / steering / cmd_vel) and does not talk
// to any vehicle controller. Outputs are point clouds, obstacles, tracks, markers, statistics
// and diagnostics.
//
// Threading model (bounded latency, no backlog)
//   * Executor thread : subscription callback stores the newest message in a single-slot mailbox
//                       (O(1)); 1 Hz stats/watchdog timer; diagnostic updater.
//   * Worker thread   : runs PerceptionPipeline on the newest message (internally parallel via a
//                       thread pool) and publishes the primary outputs (obstacles, tracks).
//   * Viz thread      : builds markers / debug clouds from the newest finished frame. Slow RViz
//                       subscribers can therefore never delay the primary outputs.

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <pcl/common/io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "helios_obstacle_detection/marker_builder.hpp"
#include "helios_obstacle_detection/msg/obstacle_array.hpp"
#include "helios_obstacle_detection/msg/processing_stats.hpp"
#include "helios_obstacle_detection/msg/tracked_object_array.hpp"
#include "helios_obstacle_detection/msg/collision_warning.hpp"
#include "helios_obstacle_detection/perception_pipeline.hpp"
#include "helios_obstacle_detection/performance_monitor.hpp"
#include "helios_obstacle_detection/resource_monitor.hpp"
#include "helios_obstacle_detection/sensor_health.hpp"

namespace helios_obstacle_detection
{

using sensor_msgs::msg::PointCloud2;
using StatsMsg = helios_obstacle_detection::msg::ProcessingStats;
using ObstacleArrayMsg = helios_obstacle_detection::msg::ObstacleArray;
using TrackArrayMsg = helios_obstacle_detection::msg::TrackedObjectArray;
using TrackMsg = helios_obstacle_detection::msg::TrackedObject;
using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

namespace
{
AxisBox boxFromVector(const std::vector<double> & v, const char * name)
{
  if (v.size() != 6) {
    throw std::invalid_argument(std::string(name) + " must have 6 values [min_x,max_x,min_y,"
            "max_y,min_z,max_z]");
  }
  return AxisBox{
    static_cast<float>(v[0]), static_cast<float>(v[1]), static_cast<float>(v[2]),
    static_cast<float>(v[3]), static_cast<float>(v[4]), static_cast<float>(v[5])};
}

geometry_msgs::msg::Point toMsgPoint(const Eigen::Vector3f & v)
{
  geometry_msgs::msg::Point p;
  p.x = v.x();
  p.y = v.y();
  p.z = v.z();
  return p;
}

geometry_msgs::msg::Vector3 toMsgVec(float x, float y, float z)
{
  geometry_msgs::msg::Vector3 v;
  v.x = x;
  v.y = y;
  v.z = z;
  return v;
}

double wallNow()
{
  return std::chrono::duration<double>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

std::string fmt(const char * f, double v)
{
  char b[64];
  std::snprintf(b, sizeof(b), f, v);
  return b;
}
}  // namespace

class ObstacleDetectionNode : public rclcpp::Node
{
public:
  explicit ObstacleDetectionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("obstacle_detector", options), diagnostics_(this, 1.0)
  {
    loadParameters();  // throws on invalid configuration

    const auto out_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    pub_filtered_ = create_publisher<PointCloud2>("filtered_points", out_qos);
    pub_ground_ = create_publisher<PointCloud2>("ground_points", out_qos);
    pub_obstacle_pts_ = create_publisher<PointCloud2>("obstacle_points", out_qos);
    pub_clusters_ = create_publisher<PointCloud2>("obstacle_clusters", out_qos);
    pub_markers_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_markers", out_qos);
    pub_track_markers_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("track_markers", out_qos);
    pub_obstacles_ = create_publisher<ObstacleArrayMsg>("obstacles", rclcpp::QoS(5).reliable());
    pub_tracks_ = create_publisher<TrackArrayMsg>("tracks", rclcpp::QoS(5).reliable());
    pub_warning_ = create_publisher<helios_obstacle_detection::msg::CollisionWarning>(
      "collision_warning", rclcpp::QoS(5).reliable());
    pub_stats_ = create_publisher<StatsMsg>("processing_stats", rclcpp::QoS(10).reliable());

    // KeepLast(depth=1): the DDS reader keeps only the newest cloud -> never queues stale data.
    auto in_qos = rclcpp::QoS(rclcpp::KeepLast(input_depth_));
    if (input_reliable_) {
      in_qos.reliable();
    } else {
      in_qos.best_effort();
    }
    sub_ = create_subscription<PointCloud2>(
      input_topic_, in_qos,
      [this](PointCloud2::ConstSharedPtr msg) {onCloud(std::move(msg));});

    setupDiagnostics();

    start_time_ = SteadyClock::now();
    worker_ = std::thread([this] {workerLoop();});
    viz_thread_ = std::thread([this] {vizLoop();});
    stats_timer_ = create_wall_timer(
      std::chrono::duration<double>(stats_period_s_), [this] {onStatsTimer();});

    RCLCPP_INFO(
      get_logger(), "Listening on '%s' (%s, depth %d). Outputs under '%s'. Ground: %s, "
      "ego-motion: %s, tracking: %s, pool threads: %zu. Perception only - no actuation outputs.",
      input_topic_.c_str(), input_reliable_ ? "reliable" : "best_effort", input_depth_,
      get_effective_namespace().c_str(), pipeline_->groundMethod(),
      pipeline_->config().ego_mode == EgoMode::kEstimate ? "lidar_registration" :
      "assume_stationary", pipeline_->config().tracker.enabled ? "on" : "off",
      pipeline_->pool().size());
  }

  ~ObstacleDetectionNode() override
  {
    {
      std::lock_guard<std::mutex> lock(slot_mutex_);
      stop_ = true;
    }
    slot_cv_.notify_all();
    {
      std::lock_guard<std::mutex> lock(viz_mutex_);
      viz_stop_ = true;
    }
    viz_cv_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
    if (viz_thread_.joinable()) {
      viz_thread_.join();
    }
  }

  ObstacleDetectionNode(const ObstacleDetectionNode &) = delete;
  ObstacleDetectionNode & operator=(const ObstacleDetectionNode &) = delete;

private:
  // ------------------------------------------------------------------ parameters
  float f(const char * name, double def)
  {
    return static_cast<float>(declare_parameter<double>(name, def));
  }
  int i(const char * name, int def)
  {
    return static_cast<int>(declare_parameter<int>(name, def));
  }
  bool b(const char * name, bool def) {return declare_parameter<bool>(name, def);}
  std::vector<float> fv(const char * name, const std::vector<double> & def)
  {
    std::vector<float> out;
    for (const double v : declare_parameter<std::vector<double>>(name, def)) {
      out.push_back(static_cast<float>(v));
    }
    return out;
  }
  std::vector<int> iv(const char * name, const std::vector<int64_t> & def)
  {
    std::vector<int> out;
    for (const auto v : declare_parameter<std::vector<int64_t>>(name, def)) {
      out.push_back(static_cast<int>(v));
    }
    return out;
  }

  void loadParameters()
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/rslidar_points");
    input_depth_ = i("input_qos_depth", 1);
    const auto reliability = declare_parameter<std::string>("input_reliability", "reliable");
    if (reliability != "reliable" && reliability != "best_effort") {
      throw std::invalid_argument("input_reliability must be 'reliable' or 'best_effort'");
    }
    input_reliable_ = reliability == "reliable";
    publish_debug_clouds_ = b("publish_debug_clouds", true);
    marker_lifetime_s_ = declare_parameter<double>("marker_lifetime_s", 0.3);
    input_timeout_s_ = declare_parameter<double>("input_timeout_s", 0.5);
    max_processing_ms_ = declare_parameter<double>("max_processing_time_ms", 80.0);
    stats_period_s_ = declare_parameter<double>("stats_period_s", 1.0);
    log_period_s_ = declare_parameter<double>("log_period_s", 5.0);
    const auto window = declare_parameter<int>("stats_window", 300);
    if (input_topic_.empty()) {
      throw std::invalid_argument("input_topic must not be empty");
    }
    if (input_depth_ < 1 || input_depth_ > 10) {
      throw std::invalid_argument("input_qos_depth must be in [1,10]");
    }
    if (!(marker_lifetime_s_ > 0.0 && input_timeout_s_ > 0.0 && max_processing_ms_ > 0.0 &&
      stats_period_s_ >= 0.05 && log_period_s_ >= 0.0 && window >= 10))
    {
      throw std::invalid_argument("timing parameters out of range");
    }
    monitor_ = std::make_unique<PerformanceMonitor>(static_cast<std::size_t>(window));

    PipelineConfig pc;
    pc.threads = i("threads", 2);
    // pre-filter
    pc.filter.min_range = f("filter.min_range", 0.5);
    pc.filter.max_range = f("filter.max_range", 40.0);
    pc.filter.roi = boxFromVector(
      declare_parameter<std::vector<double>>(
        "filter.roi", {-40.0, 40.0, -40.0, 40.0, -3.0, 3.0}), "filter.roi");
    pc.filter.self_filter_enabled = b("filter.self_filter.enabled", false);
    pc.filter.self_box = boxFromVector(
      declare_parameter<std::vector<double>>(
        "filter.self_filter.box", {-0.5, 0.5, -0.5, 0.5, -2.0, 0.5}), "filter.self_filter.box");
    pc.filter.voxel_leaf_size = f("filter.voxel_leaf_size", 0.1);
    pc.adaptive.enabled = b("adaptive.enabled", true);
    pc.adaptive.target_ms = f("adaptive.target_ms", 50.0);
    pc.adaptive.max_leaf = f("adaptive.max_leaf", 0.20);
    pc.adaptive.leaf_step = f("adaptive.leaf_step", 0.02);
    pc.adaptive.window = i("adaptive.window", 10);

    // ground (shared + RANSAC)
    pc.ground_method = declare_parameter<std::string>("ground.method", "czm");
    GroundConfig & gc = pc.ground;
    const auto n = fv("ground.expected_normal", {0.0, 0.0, 1.0});
    if (n.size() != 3) {
      throw std::invalid_argument("ground.expected_normal must have 3 values");
    }
    gc.expected_normal = Eigen::Vector3f(n[0], n[1], n[2]);
    gc.expected_height = f("ground.expected_height", -0.9);
    gc.height_tolerance = f("ground.height_tolerance", 0.35);
    gc.max_zone_height_jump = f("ground.max_zone_height_jump", 0.30);
    gc.max_normal_angle_deg = f("ground.max_normal_angle_deg", 15.0);
    gc.candidate_above = f("ground.candidate_above", 0.40);
    gc.candidate_below = f("ground.candidate_below", 0.80);
    gc.ransac_distance = f("ground.ransac_distance", 0.08);
    gc.ransac_max_iterations = i("ground.ransac_max_iterations", 150);
    gc.max_plane_attempts = i("ground.max_plane_attempts", 4);
    gc.min_zone_inliers = i("ground.min_zone_inliers", 60);
    gc.zone_boundaries = fv("ground.zone_boundaries", {0.0, 8.0, 16.0, 40.0});
    gc.ground_threshold = f("ground.ground_threshold", 0.12);
    gc.max_obstacle_height = f("ground.max_obstacle_height", 2.5);
    gc.below_ground_tolerance = f("ground.below_ground_tolerance", 0.30);
    // CZM
    CzmConfig & cz = pc.czm;
    cz.zone_boundaries = fv("ground.czm.zone_boundaries", {0.0, 4.0, 10.0, 20.0, 30.0});
    cz.rings_per_zone = iv("ground.czm.rings_per_zone", {2, 3, 3, 2});
    cz.sectors_per_zone = iv("ground.czm.sectors_per_zone", {16, 32, 48, 32});
    cz.min_bin_points = i("ground.czm.min_bin_points", 8);
    cz.seed_count = i("ground.czm.seed_count", 20);
    cz.seed_height = f("ground.czm.seed_height", 0.15);
    cz.plane_distance = f("ground.czm.plane_distance", 0.10);
    cz.fit_iterations = i("ground.czm.fit_iterations", 3);
    cz.min_planarity_ratio = f("ground.czm.min_planarity_ratio", 4.0);
    cz.min_extent = f("ground.czm.min_extent", 0.10);
    cz.elevation_rings = i("ground.czm.elevation_rings", 4);
    cz.elevation_threshold = f("ground.czm.elevation_threshold", 0.30);
    cz.elevation_min = f("ground.czm.elevation_min", 0.15);
    cz.elevation_max = f("ground.czm.elevation_max", 0.50);
    cz.flatness_threshold = f("ground.czm.flatness_threshold", 0.004);
    cz.flatness_min = f("ground.czm.flatness_min", 0.001);
    cz.flatness_max = f("ground.czm.flatness_max", 0.010);
    cz.max_bin_height_jump = f("ground.czm.max_bin_height_jump", 0.25);
    cz.max_corner_height_jump = f("ground.czm.max_corner_height_jump", 0.40);
    cz.continuity_rings = i("ground.czm.continuity_rings", 2);
    cz.far_requires_reference = b("ground.czm.far_requires_reference", true);
    cz.far_height_tolerance = f("ground.czm.far_height_tolerance", 0.5);
    cz.adaptive = b("ground.czm.adaptive", true);
    cz.adaptation_rate = f("ground.czm.adaptation_rate", 0.05);
    cz.adaptive_sigma = f("ground.czm.adaptive_sigma", 3.0);
    cz.degraded_ratio = f("ground.czm.degraded_ratio", 0.5);

    // clustering + boxes
    pc.cluster.tolerance = f("clustering.tolerance", 0.45);
    pc.cluster.min_points = i("clustering.min_points", 8);
    pc.cluster.max_points = i("clustering.max_points", 25000);
    pc.cluster.adaptive = b("clustering.adaptive", true);
    pc.cluster.tolerance_angle_deg = f("clustering.tolerance_angle_deg", 1.5);
    pc.cluster.max_tolerance = f("clustering.max_tolerance", 1.0);
    pc.boxes.min_height = f("boxes.min_height", 0.05);
    pc.boxes.large_structure_size = f("boxes.large_structure_size", 8.0);
    pc.boxes.max_obstacles = i("boxes.max_obstacles", 100);
    pc.boxes.oriented_boxes = b("boxes.oriented_boxes", true);
    pc.boxes.obb_angle_step_deg = f("boxes.obb_angle_step_deg", 2.0);

    // ego motion
    const auto mode = declare_parameter<std::string>("ego_motion.mode", "estimate");
    if (mode == "estimate") {
      pc.ego_mode = EgoMode::kEstimate;
    } else if (mode == "assume_stationary") {
      pc.ego_mode = EgoMode::kAssumeStationary;
    } else {
      throw std::invalid_argument("ego_motion.mode must be 'estimate' or 'assume_stationary'");
    }
    EgoMotionConfig & ec = pc.ego;
    ec.voxel_size = f("ego_motion.voxel_size", 0.30);
    ec.min_range = f("ego_motion.min_range", 1.0);
    ec.max_range = f("ego_motion.max_range", 25.0);
    ec.max_points = i("ego_motion.max_points", 1500);
    ec.normal_neighbors = i("ego_motion.normal_neighbors", 8);
    ec.max_correspondence = f("ego_motion.max_correspondence", 1.0);
    ec.max_iterations = i("ego_motion.max_iterations", 15);
    ec.robust_kernel_scale = f("ego_motion.robust_kernel_scale", 0.07);
    ec.consensus_threshold = f("ego_motion.consensus_threshold", 0.06);
    ec.consensus_margin = f("ego_motion.consensus_margin", 0.02);
    ec.min_correspondences = i("ego_motion.min_correspondences", 80);
    ec.min_inlier_ratio = f("ego_motion.min_inlier_ratio", 0.30);
    ec.min_degeneracy = f("ego_motion.min_degeneracy", 0.05);
    ec.max_velocity_sigma = f("ego_motion.max_velocity_sigma", 0.30);
    ec.max_speed = f("ego_motion.max_speed", 20.0);
    ec.max_yaw_rate = f("ego_motion.max_yaw_rate", 1.5);
    ec.stationary_speed = f("ego_motion.stationary_speed", 0.08);
    ec.stationary_yaw_rate = f("ego_motion.stationary_yaw_rate", 0.03);
    ec.state_hysteresis_frames = i("ego_motion.state_hysteresis_frames", 3);
    ec.max_prediction_time = f("ego_motion.max_prediction_time", 0.5);
    ec.max_dt = f("ego_motion.max_dt", 0.5);

    // tracking
    TrackerConfig & tc = pc.tracker;
    tc.enabled = b("tracking.enabled", true);
    tc.process_accel_sigma = f("tracking.process_accel_sigma", 2.0);
    tc.meas_sigma_base = f("tracking.meas_sigma_base", 0.08);
    tc.meas_sigma_per_meter = f("tracking.meas_sigma_per_meter", 0.005);
    tc.meas_sigma_extent_factor = f("tracking.meas_sigma_extent_factor", 0.10);
    tc.sparse_points = i("tracking.sparse_points", 15);
    tc.init_velocity_sigma = f("tracking.init_velocity_sigma", 3.0);
    tc.gate_chi2 = f("tracking.gate_chi2", 13.8);
    tc.max_gate_distance = f("tracking.max_gate_distance", 3.0);
    tc.confirm_hits = i("tracking.confirm_hits", 3);
    tc.tentative_max_misses = i("tracking.tentative_max_misses", 2);
    tc.max_coast_time_s = f("tracking.max_coast_time_s", 0.8);
    tc.occluded_max_time_s = f("tracking.occluded_max_time_s", 2.5);
    tc.lost_max_time_s = f("tracking.lost_max_time_s", 2.0);
    tc.occlusion_bin_deg = f("tracking.occlusion_bin_deg", 1.0);
    tc.occlusion_margin = f("tracking.occlusion_margin", 0.3);
    tc.reid_max_distance = f("tracking.reid_max_distance", 3.0);
    tc.reid_speed = f("tracking.reid_speed", 1.0);
    tc.size_cost_weight = f("tracking.size_cost_weight", 1.0);
    tc.iou_cost_weight = f("tracking.iou_cost_weight", 1.0);
    tc.background_cost = f("tracking.background_cost", 4.0);
    tc.max_velocity_sigma = f("tracking.max_velocity_sigma", 0.6);
    tc.partial_view_model = b("tracking.partial_view_model", true);
    tc.merge_split = b("tracking.merge_split", true);
    tc.merge_max_tracks = i("tracking.merge_max_tracks", 3);
    tc.merge_max_time_s = f("tracking.merge_max_time_s", 4.0);
    tc.motion_evidence_timeout_s = f("tracking.motion_evidence_timeout_s", 2.0);
    tc.onset_detection = b("tracking.onset_detection", true);
    tc.onset_sigma_factor = f("tracking.onset_sigma_factor", 5.0);
    tc.onset_min_displacement = f("tracking.onset_min_displacement", 0.20);
    tc.onset_min_samples = i("tracking.onset_min_samples", 3);
    tc.max_position_sigma = f("tracking.max_position_sigma", 2.0);
    tc.max_tracks = i("tracking.max_tracks", 256);
    tc.max_dt = f("tracking.max_dt", 1.0);
    tc.shape_smoothing = f("tracking.shape_smoothing", 0.3);
    tc.moving_speed = f("tracking.moving_speed", 0.30);
    tc.static_speed = f("tracking.static_speed", 0.20);
    tc.motion_sigma_factor = f("tracking.motion_sigma_factor", 3.0);
    tc.min_motion_samples = i("tracking.min_motion_samples", 5);
    tc.shape_change_tolerance = f("tracking.shape_change_tolerance", 0.25);
    tc.shape_reset_frames = i("tracking.shape_reset_frames", 5);
    tc.inconsistent_sample_weight = f("tracking.inconsistent_sample_weight", 0.3);
    tc.point_evidence_age_s = f("tracking.point_evidence_age_s", 0.5);
    tc.point_evidence_override_speed = f("tracking.point_evidence_override_speed", 1.5);
    tc.snapshot_points = i("tracking.snapshot_points", 48);
    tc.min_age_s = f("tracking.min_age_s", 0.5);
    tc.displacement_window_s = f("tracking.displacement_window_s", 0.8);
    tc.moving_min_displacement = f("tracking.moving_min_displacement", 0.30);
    tc.motion_hysteresis_frames = i("tracking.motion_hysteresis_frames", 2);
    tc.history_length = i("tracking.history_length", 30);

    // static background map
    pc.static_map.enabled = b("static_map.enabled", true);
    pc.static_map.voxel_size = f("static_map.voxel_size", 0.15);
    pc.static_map.static_time_s = f("static_map.static_time_s", 10.0);
    pc.static_map.min_occupancy = f("static_map.min_occupancy", 0.5);
    pc.static_map.forget_time_s = f("static_map.forget_time_s", 5.0);
    pc.static_map.neighbor_tolerance = b("static_map.neighbor_tolerance", true);
    pc.static_map.track_guided = b("static_map.track_guided", true);

    // risk
    RiskConfig & rc = pc.risk;
    rc.ego_radius = f("risk.ego_radius", 0.5);
    rc.min_closing_speed = f("risk.min_closing_speed", 0.2);
    rc.sigma_factor = f("risk.sigma_factor", 2.0);
    rc.horizon_s = f("risk.horizon_s", 8.0);
    rc.critical_ttc_s = f("risk.critical_ttc_s", 1.5);
    rc.high_ttc_s = f("risk.high_ttc_s", 3.0);
    rc.medium_ttc_s = f("risk.medium_ttc_s", 5.0);
    rc.critical_distance = f("risk.critical_distance", 0.5);
    rc.high_distance = f("risk.high_distance", 1.0);
    rc.medium_distance = f("risk.medium_distance", 2.5);
    // collision warning (display only)
    WarningConfig & wc = pc.warning;
    wc.enabled = b("warning.enabled", true);
    wc.warning_ttc_s = f("warning.warning_ttc_s", 3.0);
    wc.caution_ttc_s = f("warning.caution_ttc_s", 6.0);
    wc.near_distance = f("warning.near_distance", 1.0);
    wc.min_closing_speed = f("warning.min_closing_speed", 0.3);
    wc.min_object_displacement = f("warning.min_object_displacement", 0.5);
    wc.raise_frames = i("warning.raise_frames", 2);
    wc.hold_s = f("warning.hold_s", 1.0);

    pipeline_ = std::make_unique<PerceptionPipeline>(pc);  // validates everything

    // health
    SensorHealthConfig hc;
    hc.expected_rate_hz = declare_parameter<double>("health.expected_rate_hz", 10.0);
    hc.gap_factor = declare_parameter<double>("health.gap_factor", 1.5);
    hc.min_valid_ratio = declare_parameter<double>("health.min_valid_ratio", 0.30);
    hc.min_azimuth_coverage = declare_parameter<double>("health.min_azimuth_coverage", 0.70);
    hc.point_drop_ratio = declare_parameter<double>("health.point_drop_ratio", 0.50);
    hc.clock_sync_tolerance_s = declare_parameter<double>("health.clock_sync_tolerance_s", 1.0);
    hc.event_memory_s = declare_parameter<double>("health.event_memory_s", 10.0);
    health_ = std::make_unique<SensorHealthMonitor>(hc);
    expected_rate_hz_ = hc.expected_rate_hz;

    // visualisation
    viz_.show_tentative = b("visualization.show_tentative", false);
    viz_.show_static = b("visualization.show_static", false);
    viz_.show_unknown = b("visualization.show_unknown", true);
    viz_.only_dynamic_after_learning = b("visualization.only_dynamic_after_learning", true);
    viz_.min_moving_displacement = f("visualization.min_moving_displacement", 0.5);
    viz_.min_dynamic_displacement = f("visualization.min_dynamic_displacement", 1.0);
    viz_.show_trails = b("visualization.show_trails", true);
    viz_.show_uncertainty = b("visualization.show_uncertainty", true);
    viz_.show_prediction = b("visualization.show_prediction", true);
    viz_.arrow_time_s = f("visualization.arrow_time_s", 1.0);
    viz_.prediction_horizon_s = f("visualization.prediction_horizon_s", 3.0);
    viz_.ego_radius = rc.ego_radius;
    viz_.min_arrow_speed = tc.static_speed;

    marker_lifetime_ = rclcpp::Duration::from_seconds(marker_lifetime_s_);
  }

  // ------------------------------------------------------------------ input
  void onCloud(PointCloud2::ConstSharedPtr msg)
  {
    const auto now = SteadyClock::now();
    const double wall = wallNow();
    monitor_->recordInput(now);
    last_rx_ns_.store(now.time_since_epoch().count(), std::memory_order_relaxed);
    bool replaced = false;
    {
      std::lock_guard<std::mutex> lock(slot_mutex_);
      replaced = static_cast<bool>(pending_);
      pending_ = std::move(msg);
      pending_rx_ = now;
      pending_rx_wall_ = wall;
    }
    if (replaced) {
      monitor_->recordDropped();
    }
    slot_cv_.notify_one();
  }

  void workerLoop()
  {
    while (true) {
      PointCloud2::ConstSharedPtr msg;
      SteadyClock::time_point rx;
      double rx_wall = 0.0;
      {
        std::unique_lock<std::mutex> lock(slot_mutex_);
        slot_cv_.wait(lock, [this] {return stop_ || pending_;});
        if (stop_) {
          return;
        }
        msg = std::move(pending_);
        pending_.reset();
        rx = pending_rx_;
        rx_wall = pending_rx_wall_;
      }
      const double wait_s = std::chrono::duration<double>(SteadyClock::now() - rx).count();
      if (wait_s > input_timeout_s_) {
        monitor_->recordDropped();  // too old to be meaningful
        continue;
      }
      monitor_->recordStage(Stage::kQueueWait, wait_s * 1000.0);
      try {
        processFrame(msg, rx, rx_wall);
      } catch (const std::exception & e) {
        if (rclcpp::ok()) {
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 2000, "Frame processing failed: %s", e.what());
        }
      }
    }
  }

  // ------------------------------------------------------------------ pipeline
  void processFrame(
    const PointCloud2::ConstSharedPtr & msg, SteadyClock::time_point rx, double rx_wall)
  {
    const double stamp = static_cast<double>(msg->header.stamp.sec) +
      1e-9 * static_cast<double>(msg->header.stamp.nanosec);

    auto frame = std::make_shared<FrameOutput>();
    std::string error;
    const auto t0 = SteadyClock::now();
    const bool ok = pipeline_->process(*msg, stamp, *frame, error);
    if (!ok) {
      invalid_input_.store(true);
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "Unusable PointCloud2 on '%s': %s",
        input_topic_.c_str(), error.c_str());
      pub_markers_->publish(buildClearMarkers(msg->header));
      pub_track_markers_->publish(buildClearMarkers(msg->header));
      return;
    }
    invalid_input_.store(false);

    // sensor health (gaps, time jumps, scan quality). A time jump invalidates every temporal
    // state, so the frame is re-run after a reset.
    ScanQuality q;
    q.total_points = frame->filter.input;
    q.finite_points = frame->filter.input - frame->filter.non_finite;
    q.azimuth_sectors_total = FilterStats::kAzimuthSectors;
    q.azimuth_sectors_covered = frame->filter.azimuth_sectors_covered;
    const FrameHealth fh = health_->onFrame(stamp, rx_wall, q);
    if (fh.time_jump) {
      RCLCPP_WARN(get_logger(), "Header stamp jumped backwards (bag loop / driver restart): "
        "resetting tracker and ego-motion.");
      pipeline_->reset();
      frame = std::make_shared<FrameOutput>();
      pipeline_->process(*msg, stamp, *frame, error);
    } else if (fh.gap) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "LiDAR frame gap: ~%d scan(s) missing.", fh.missed_frames);
    }
    const double pipeline_ms = std::chrono::duration<double, std::milli>(
      SteadyClock::now() - t0).count();

    // primary outputs first
    pub_obstacles_->publish(toObstacleMsg(msg->header, *frame));
    pub_tracks_->publish(toTrackMsg(msg->header, *frame));
    {
      const CollisionWarning & w = frame->warning;
      helios_obstacle_detection::msg::CollisionWarning wm;
      wm.header = msg->header;
      wm.level = static_cast<std::uint8_t>(w.level);
      wm.collision_predicted = w.collision_predicted;
      wm.track_id = w.track_id;
      wm.distance = w.distance;
      wm.clearance = w.clearance;
      wm.closing_speed = w.closing_speed;
      wm.closing_speed_sigma = w.closing_speed_sigma;
      wm.ttc = w.ttc;
      wm.ttc_pessimistic = w.ttc_pessimistic;
      wm.cpa_time = w.cpa_time;
      wm.cpa_distance = w.cpa_distance;
      wm.text = w.text;
      pub_warning_->publish(wm);
      if (w.level != last_warning_level_) {
        if (w.level == WarningLevel::kClear) {
          RCLCPP_INFO(get_logger(), "collision warning cleared");
        } else {
          RCLCPP_WARN(get_logger(), "%s", w.text.c_str());
        }
        last_warning_level_ = w.level;
      }
    }
    const auto published = SteadyClock::now();

    const StageTimes & st = frame->times;
    monitor_->recordStage(Stage::kRoiFilter, st.roi_ms);
    monitor_->recordStage(Stage::kVoxel, st.voxel_ms);
    monitor_->recordStage(Stage::kGround, st.ground_ms);
    monitor_->recordStage(Stage::kClustering, st.clustering_ms);
    monitor_->recordStage(Stage::kBoundingBox, st.boxes_ms);
    monitor_->recordStage(Stage::kEgoMotion, st.ego_ms);
    monitor_->recordStage(Stage::kTracking, st.tracking_ms);
    monitor_->recordStage(Stage::kTotal, pipeline_ms);
    monitor_->recordStage(Stage::kRxToPublish,
      std::chrono::duration<double, std::milli>(published - rx).count());
    const bool overload = pipeline_ms > max_processing_ms_;
    monitor_->recordProcessed(published, overload);

    {
      std::lock_guard<std::mutex> lock(frame_info_mutex_);
      FrameInfo & fi = frame_info_;
      const FilterStats & fs = frame->filter;
      fi.points_input = static_cast<std::uint32_t>(fs.input);
      fi.points_valid = static_cast<std::uint32_t>(fs.input - fs.non_finite - fs.range_rejected);
      fi.points_roi = static_cast<std::uint32_t>(fs.kept);
      fi.points_voxel = static_cast<std::uint32_t>(frame->voxel_cloud->size());
      fi.points_ground = static_cast<std::uint32_t>(frame->ground.ground.size());
      fi.points_obstacle = static_cast<std::uint32_t>(frame->obstacle_cloud->size());
      fi.clusters = static_cast<std::uint32_t>(frame->obstacles.size());
      fi.ground_status = static_cast<std::uint8_t>(frame->ground.status);
      fi.ground_detail = frame->ground.detail;
      fi.frame_id = msg->header.frame_id;
      fi.last_total_ms = pipeline_ms;
      fi.voxel_leaf = frame->voxel_leaf;
      fi.ego = frame->ego;
      fi.tracker = frame->tracker_stats;
      fi.max_risk = frame->max_risk;
      fi.background_points = static_cast<std::uint32_t>(frame->background_points);
      fi.static_voxels = static_cast<std::uint32_t>(frame->static_voxels);
      fi.background_learning = frame->background_learning;
      fi.has_frame = true;
    }

    // hand over to the visualisation thread (newest frame wins)
    {
      std::lock_guard<std::mutex> lock(viz_mutex_);
      viz_header_ = msg->header;
      viz_frame_ = std::move(frame);
    }
    viz_cv_.notify_one();
  }

  ObstacleArrayMsg toObstacleMsg(
    const std_msgs::msg::Header & header, const FrameOutput & fr) const
  {
    ObstacleArrayMsg out;
    out.header = header;
    out.ground_status = static_cast<std::uint8_t>(fr.ground.status);
    out.obstacles.reserve(fr.obstacles.size());
    for (const auto & o : fr.obstacles) {
      helios_obstacle_detection::msg::Obstacle m;
      m.id = o.id;
      m.point_count = static_cast<std::uint32_t>(o.point_count);
      m.center = toMsgPoint(o.center);
      m.size = toMsgVec(o.size.x(), o.size.y(), o.size.z());
      m.min = toMsgPoint(o.min_pt);
      m.max = toMsgPoint(o.max_pt);
      m.closest_point = toMsgPoint(o.closest_point);
      m.closest_distance = o.closest_distance;
      m.center_distance = o.center_distance;
      m.large_structure = o.large_structure;
      m.obb_center = toMsgPoint(o.obb.center);
      m.obb_size = toMsgVec(o.obb.size.x(), o.obb.size.y(), o.obb.size.z());
      m.obb_yaw = o.obb.yaw;
      m.background = o.background;
      out.obstacles.push_back(std::move(m));
    }
    return out;
  }

  TrackArrayMsg toTrackMsg(const std_msgs::msg::Header & header, const FrameOutput & fr) const
  {
    TrackArrayMsg out;
    out.header = header;
    const EgoMotion & e = fr.ego;
    out.ego_state = static_cast<std::uint8_t>(e.state);
    out.ego_motion_measured = e.measured;
    out.ego_velocity = toMsgVec(static_cast<float>(e.velocity.x()),
        static_cast<float>(e.velocity.y()), 0.F);
    out.ego_yaw_rate = static_cast<float>(e.yaw_rate);
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        out.ego_velocity_covariance[static_cast<std::size_t>(r * 3 + c)] =
          static_cast<float>(e.velocity_cov(r, c));
      }
    }
    out.ego_registration_inlier_ratio = e.inlier_ratio;
    out.ego_registration_rmse = e.rmse;
    out.ego_degeneracy = e.degeneracy;
    out.ego_status_text = std::string(toString(e.state)) +
      (e.reason.empty() ? "" : ": " + e.reason);
    out.ground_status = static_cast<std::uint8_t>(fr.ground.status);
    out.max_risk_level = static_cast<std::uint8_t>(fr.max_risk);
    out.max_risk_track_id = fr.max_risk_track >= 0 ?
      static_cast<std::int64_t>(fr.tracks[static_cast<std::size_t>(fr.max_risk_track)].id) : -1;
    out.objects.reserve(fr.tracks.size());
    for (std::size_t k = 0; k < fr.tracks.size(); ++k) {
      const TrackOutput & t = fr.tracks[k];
      const RiskAssessment & r = fr.risks[k];
      TrackMsg m;
      m.id = t.id;
      m.track_state = static_cast<std::uint8_t>(t.state);
      m.motion_state = static_cast<std::uint8_t>(t.motion);
      m.occlusion_reason = static_cast<std::uint8_t>(t.occlusion);
      m.velocity_status = static_cast<std::uint8_t>(t.velocity_status);
      m.predicted = t.predicted;
      m.merged = t.merged;
      m.relative_motion_only = t.relative_only;
      m.track_confidence = t.confidence;
      m.motion_confidence = t.motion_confidence;
      m.time_since_moving_s = t.time_since_moving_s;
      m.max_displacement = t.max_displacement;
      m.ever_moved = t.ever_moved;
      m.velocity_reliable = t.velocity_reliable;
      m.large_structure = t.large_structure;
      m.in_static_background = t.background;
      m.age_s = t.age_s;
      m.hits = t.hits;
      m.consecutive_misses = t.misses;
      m.time_since_update_s = t.time_since_update_s;
      m.point_count = t.point_count;
      m.detection_id = t.detection_index >= 0 && static_cast<std::size_t>(t.detection_index) <
        fr.obstacles.size() ?
        static_cast<std::int32_t>(fr.obstacles[static_cast<std::size_t>(t.detection_index)].id) :
        -1;
      m.center = toMsgPoint(t.center);
      m.size = toMsgVec(t.size.x(), t.size.y(), t.size.z());
      m.yaw = t.yaw;
      for (int a = 0; a < 2; ++a) {
        for (int c = 0; c < 2; ++c) {
          const auto idx = static_cast<std::size_t>(a * 2 + c);
          m.position_covariance[idx] = t.position_cov(a, c);
          m.velocity_covariance[idx] = t.velocity_cov(a, c);
          m.relative_velocity_covariance[idx] = t.relative_velocity_cov(a, c);
        }
      }
      m.velocity = toMsgVec(t.velocity.x(), t.velocity.y(), 0.F);
      m.speed = t.speed;
      m.speed_sigma = t.speed_sigma;
      m.heading = t.heading;
      m.heading_sigma = t.heading_sigma;
      m.heading_valid = t.heading_valid;
      m.window_speed = t.window_speed;
      m.window_speed_sigma = std::isfinite(t.window_speed_sigma) ? t.window_speed_sigma : -1.F;
      m.surface_speed = t.surface_speed;
      m.relative_velocity = toMsgVec(t.relative_velocity.x(), t.relative_velocity.y(), 0.F);
      m.closest_point = toMsgPoint(t.closest_point);
      m.closest_distance = t.closest_distance;
      m.ttc_valid = r.ttc_valid;
      m.clearance = r.clearance;
      m.closing_speed = r.closing_speed;
      m.closing_speed_sigma = r.closing_speed_sigma;
      m.ttc = r.ttc;
      m.ttc_pessimistic = r.ttc_pessimistic;
      m.cpa_time = r.cpa_time;
      m.cpa_distance = r.cpa_distance;
      m.collision_course = r.collision_course;
      m.risk_level = static_cast<std::uint8_t>(r.level);
      out.objects.push_back(std::move(m));
    }
    return out;
  }

  // ------------------------------------------------------------------ visualisation thread
  template<typename PointCloudT>
  void publishCloud(
    const rclcpp::Publisher<PointCloud2>::SharedPtr & pub, const PointCloudT & cloud,
    const std_msgs::msg::Header & header)
  {
    auto out = std::make_unique<PointCloud2>();
    pcl::toROSMsg(cloud, *out);
    out->header = header;  // exact ns stamp + frame (PCL header only keeps microseconds)
    pub->publish(std::move(out));
  }

  void vizLoop()
  {
    while (true) {
      std::shared_ptr<FrameOutput> frame;
      std_msgs::msg::Header header;
      {
        std::unique_lock<std::mutex> lock(viz_mutex_);
        viz_cv_.wait(lock, [this] {return viz_stop_ || viz_frame_;});
        if (viz_stop_) {
          return;
        }
        frame = std::move(viz_frame_);
        viz_frame_.reset();
        header = viz_header_;
      }
      try {
        ScopedStageTimer t(*monitor_, Stage::kVisualization);
        publishVisualization(header, *frame);
      } catch (const std::exception & e) {
        if (rclcpp::ok()) {
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 5000, "Visualisation failed: %s", e.what());
        }
      }
    }
  }

  void publishVisualization(const std_msgs::msg::Header & header, const FrameOutput & fr)
  {
    if (pub_markers_->get_subscription_count() > 0) {
      pub_markers_->publish(
        buildObstacleMarkers(header, fr.obstacles, fr.ground.status, marker_lifetime_));
    }
    if (pub_track_markers_->get_subscription_count() > 0) {
      std::string status_line;
      {
        std::lock_guard<std::mutex> lock(status_mutex_);
        status_line = hud_status_;
      }
      pub_track_markers_->publish(
        buildTrackMarkers(header, fr, viz_, marker_lifetime_, status_line));
    }
    if (!publish_debug_clouds_) {
      return;
    }
    if (pub_filtered_->get_subscription_count() > 0) {
      publishCloud(pub_filtered_, *fr.voxel_cloud, header);
    }
    if (pub_ground_->get_subscription_count() > 0) {
      Cloud g;
      pcl::copyPointCloud(*fr.voxel_cloud, fr.ground.ground, g);
      publishCloud(pub_ground_, g, header);
    }
    if (pub_obstacle_pts_->get_subscription_count() > 0) {
      publishCloud(pub_obstacle_pts_, *fr.obstacle_cloud, header);
    }
    if (pub_clusters_->get_subscription_count() > 0) {
      // colour clusters by their track's risk when tracked, else by per-frame id
      std::vector<int> det_to_track(fr.obstacles.size(), -1);
      for (std::size_t k = 0; k < fr.tracks.size(); ++k) {
        const int d = fr.tracks[k].detection_index;
        if (d >= 0 && static_cast<std::size_t>(d) < det_to_track.size()) {
          det_to_track[static_cast<std::size_t>(d)] = static_cast<int>(k);
        }
      }
      pcl::PointCloud<pcl::PointXYZRGB> colored;
      for (std::size_t j = 0; j < fr.obstacles.size(); ++j) {
        const auto & o = fr.obstacles[j];
        const int tk = det_to_track[j];
        const Rgb c = tk >= 0 && fr.tracks[static_cast<std::size_t>(tk)].state !=
          TrackState::kTentative ? colorForRisk(fr.risks[static_cast<std::size_t>(tk)].level) :
          colorForId(o.id);
        for (const auto idx : *o.indices) {
          const PointT & p = (*fr.obstacle_cloud)[static_cast<std::size_t>(idx)];
          pcl::PointXYZRGB q(
            static_cast<std::uint8_t>(c.r * 255.F), static_cast<std::uint8_t>(c.g * 255.F),
            static_cast<std::uint8_t>(c.b * 255.F));
          q.x = p.x;
          q.y = p.y;
          q.z = p.z;
          colored.push_back(q);
        }
      }
      publishCloud(pub_clusters_, colored, header);
    }
  }

  // ------------------------------------------------------------------ diagnostics
  void setupDiagnostics()
  {
    diagnostics_.setHardwareID("robosense_helios32");
    diagnostics_.add("LiDAR input", this, &ObstacleDetectionNode::diagInput);
    diagnostics_.add("Scan quality", this, &ObstacleDetectionNode::diagScan);
    diagnostics_.add("Time synchronization", this, &ObstacleDetectionNode::diagTime);
    diagnostics_.add("Processing", this, &ObstacleDetectionNode::diagProcessing);
    diagnostics_.add("Ground segmentation", this, &ObstacleDetectionNode::diagGround);
    diagnostics_.add("Ego motion", this, &ObstacleDetectionNode::diagEgo);
    diagnostics_.add("Tracking", this, &ObstacleDetectionNode::diagTracking);
  }

  struct FrameInfo
  {
    std::uint32_t points_input{0}, points_valid{0}, points_roi{0}, points_voxel{0};
    std::uint32_t points_ground{0}, points_obstacle{0}, clusters{0};
    std::uint8_t ground_status{static_cast<std::uint8_t>(GroundStatus::kFailed)};
    std::string ground_detail;
    std::string frame_id;
    double last_total_ms{0.0};
    float voxel_leaf{0.F};
    EgoMotion ego;
    TrackerStats tracker;
    RiskLevel max_risk{RiskLevel::kNone};
    std::uint32_t background_points{0};
    std::uint32_t static_voxels{0};
    bool background_learning{false};
    bool has_frame{false};
  };

  struct Snapshot
  {
    PerformanceSnapshot perf;
    HealthSnapshot health;
    ResourceSample res;
    double input_age{0.0};
    bool no_data{false};
  };

  Snapshot latest() const
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return last_snapshot_;
  }

  FrameInfo frameInfo() const
  {
    std::lock_guard<std::mutex> lock(frame_info_mutex_);
    return frame_info_;
  }

  void diagInput(diagnostic_updater::DiagnosticStatusWrapper & d)
  {
    const Snapshot s = latest();
    d.add("topic", input_topic_);
    d.add("input_rate_hz", fmt("%.2f", s.perf.input_fps));
    d.add("sensor_stamp_rate_hz", fmt("%.2f", s.health.stamp_rate_hz));
    d.add("expected_rate_hz", fmt("%.1f", expected_rate_hz_));
    d.add("last_input_age_s", fmt("%.2f", s.input_age));
    d.add("frames_received", std::to_string(s.perf.frames_received));
    d.add("frames_dropped_superseded", std::to_string(s.perf.frames_dropped));
    d.add("frame_gaps", std::to_string(s.health.gaps));
    d.add("missed_frames_estimate", std::to_string(s.health.missed_frames));
    d.add("time_jumps", std::to_string(s.health.time_jumps));
    if (s.no_data) {
      d.summary(DiagStatus::ERROR, "NO DATA for " + fmt("%.1f", s.input_age) + " s");
    } else if (s.perf.input_fps < 0.8 * expected_rate_hz_) {
      d.summary(DiagStatus::WARN, "input rate low: " + fmt("%.1f", s.perf.input_fps) + " Hz");
    } else {
      d.summary(DiagStatus::OK, "receiving " + fmt("%.1f", s.perf.input_fps) + " Hz");
    }
  }

  void diagScan(diagnostic_updater::DiagnosticStatusWrapper & d)
  {
    const Snapshot s = latest();
    d.add("valid_return_ratio", fmt("%.3f", s.health.valid_ratio));
    d.add("azimuth_coverage", fmt("%.3f", s.health.azimuth_coverage));
    d.add("finite_points", std::to_string(s.health.points));
    d.add("partial_scans", std::to_string(s.health.partial_scans));
    d.add("low_valid_scans", std::to_string(s.health.low_valid_scans));
    d.add("point_drops", std::to_string(s.health.point_drops));
    const auto level = s.health.level == HealthLevel::kError ? DiagStatus::ERROR :
      (s.health.level == HealthLevel::kWarn ? DiagStatus::WARN : DiagStatus::OK);
    d.summary(level, s.health.message);
  }

  void diagTime(diagnostic_updater::DiagnosticStatusWrapper & d)
  {
    const Snapshot s = latest();
    d.add("clock_synced", s.health.clock_synced ? "true" : "false");
    d.add("clock_offset_s", fmt("%.3f", s.health.clock_offset_s));
    d.add("delivery_jitter_p95_ms", fmt("%.2f", s.health.jitter_p95_ms));
    d.add("stamp_age_mean_ms", s.health.clock_synced ? fmt("%.1f", s.health.stamp_age_mean_ms) :
      "unobservable");
    if (s.health.clock_synced) {
      d.summary(DiagStatus::OK, "sensor clock synchronised, stamp age " +
        fmt("%.1f", s.health.stamp_age_mean_ms) + " ms");
    } else {
      d.summary(DiagStatus::WARN, "sensor clock NOT synchronised with host (offset " +
        fmt("%.0f", s.health.clock_offset_s) + " s): absolute sensor->host latency is "
        "unobservable, only jitter and host latency are reported");
    }
  }

  void diagProcessing(diagnostic_updater::DiagnosticStatusWrapper & d)
  {
    const Snapshot s = latest();
    const FrameInfo fi = frameInfo();
    for (const auto & st : s.perf.stages) {
      d.add(st.name + "_mean_p95_max_ms",
        fmt("%.2f", st.mean_ms) + " / " + fmt("%.2f", st.p95_ms) + " / " + fmt("%.2f", st.max_ms));
    }
    d.add("output_rate_hz", fmt("%.2f", s.perf.output_fps));
    d.add("overload_frames", std::to_string(s.perf.overload_frames));
    d.add("cpu_percent_one_core", fmt("%.1f", s.res.cpu_percent));
    d.add("cpu_percent_all_cores", fmt("%.1f", s.res.cpu_percent_total));
    d.add("rss_mb", fmt("%.1f", s.res.rss_mb));
    d.add("threads", std::to_string(s.res.threads));
    d.add("voxel_leaf_size", fmt("%.3f", fi.voxel_leaf));
    const bool coarse = fi.voxel_leaf > pipeline_->config().filter.voxel_leaf_size + 1e-4F;
    if (s.perf.overload_frames > last_diag_overload_) {
      d.summary(DiagStatus::WARN, "processing above " + fmt("%.0f", max_processing_ms_) + " ms");
    } else if (coarse) {
      d.summary(DiagStatus::WARN, "load-adaptive voxel grid coarsened to " +
        fmt("%.2f", fi.voxel_leaf) + " m");
    } else {
      d.summary(DiagStatus::OK, "OK");
    }
    last_diag_overload_ = s.perf.overload_frames;
  }

  void diagGround(diagnostic_updater::DiagnosticStatusWrapper & d)
  {
    const FrameInfo fi = frameInfo();
    d.add("method", pipeline_->groundMethod());
    d.add("detail", fi.ground_detail);
    d.add("ground_points", std::to_string(fi.points_ground));
    d.add("obstacle_points", std::to_string(fi.points_obstacle));
    const auto gs = static_cast<GroundStatus>(fi.ground_status);
    const auto level = gs == GroundStatus::kReliable ? DiagStatus::OK :
      (gs == GroundStatus::kDegraded ? DiagStatus::WARN : DiagStatus::ERROR);
    d.summary(fi.has_frame ? level : DiagStatus::STALE, toString(gs));
  }

  void diagEgo(diagnostic_updater::DiagnosticStatusWrapper & d)
  {
    const FrameInfo fi = frameInfo();
    const EgoMotion & e = fi.ego;
    d.add("state", toString(e.state));
    d.add("measured", e.measured ? "true" : "false");
    d.add("velocity_xy_mps", fmt("%.3f", e.velocity.x()) + ", " + fmt("%.3f", e.velocity.y()));
    d.add("yaw_rate_rps", fmt("%.4f", e.yaw_rate));
    d.add("velocity_sigma_mps", fmt("%.3f", std::sqrt(std::max(e.velocity_cov(0, 0),
      e.velocity_cov(1, 1)))));
    d.add("correspondences", std::to_string(e.correspondences));
    d.add("inlier_ratio", fmt("%.3f", e.inlier_ratio));
    d.add("rmse_m", fmt("%.3f", e.rmse));
    d.add("degeneracy", fmt("%.3f", e.degeneracy));
    d.add("reason", e.reason);
    if (!fi.has_frame) {
      d.summary(DiagStatus::STALE, "no frame yet");
    } else if (e.state == EgoState::kStationary || e.state == EgoState::kMoving) {
      d.summary(DiagStatus::OK, toString(e.state));
    } else {
      d.summary(DiagStatus::WARN, std::string(toString(e.state)) + ": " + e.reason +
        " -> object velocities not ground-relative / motion state UNKNOWN");
    }
  }

  void diagTracking(diagnostic_updater::DiagnosticStatusWrapper & d)
  {
    const FrameInfo fi = frameInfo();
    const TrackerStats & t = fi.tracker;
    d.add("tracks", std::to_string(t.tracks));
    d.add("confirmed", std::to_string(t.confirmed));
    d.add("moving", std::to_string(t.moving));
    d.add("static", std::to_string(t.static_tracks));
    d.add("created_total", std::to_string(t.created_total));
    d.add("resets", std::to_string(t.resets));
    d.add("mean_nis", fmt("%.2f", t.mean_nis));
    d.add("max_risk", toString(fi.max_risk));
    d.summary(fi.has_frame ? DiagStatus::OK : DiagStatus::STALE,
      std::to_string(t.confirmed) + " confirmed, " + std::to_string(t.moving) + " moving, max risk " +
      toString(fi.max_risk));
  }

  // ------------------------------------------------------------------ watchdog + stats
  void onStatsTimer()
  {
    const auto now = SteadyClock::now();
    const auto last_ns = last_rx_ns_.load(std::memory_order_relaxed);
    const auto last_rx = last_ns == 0 ? start_time_ :
      SteadyClock::time_point(SteadyClock::duration(last_ns));
    const double age = std::chrono::duration<double>(now - last_rx).count();
    const bool no_data = age > input_timeout_s_;
    const FrameInfo fi = frameInfo();

    if (no_data && !data_lost_) {
      data_lost_ = true;
      RCLCPP_WARN(
        get_logger(), "No LiDAR data on '%s' for %.2f s - obstacles/tracks cleared, NOT "
        "republished.", input_topic_.c_str(), age);
      std_msgs::msg::Header h;
      h.frame_id = fi.frame_id.empty() ? "rslidar" : fi.frame_id;
      h.stamp = get_clock()->now();
      pub_markers_->publish(buildClearMarkers(h));
      pub_track_markers_->publish(buildClearMarkers(h));
    } else if (!no_data && data_lost_) {
      data_lost_ = false;
      RCLCPP_INFO(get_logger(), "LiDAR data resumed on '%s'.", input_topic_.c_str());
    }

    Snapshot snap;
    snap.perf = monitor_->snapshot();
    snap.health = health_->snapshot(wallNow());
    snap.res = resources_.sample();
    snap.input_age = age;
    snap.no_data = no_data;
    const bool overload_now = snap.perf.overload_frames > last_overload_count_;
    last_overload_count_ = snap.perf.overload_frames;

    StatsMsg s;
    s.header.stamp = get_clock()->now();
    s.header.frame_id = fi.frame_id;
    const bool ego_unknown = fi.has_frame && pipeline_->config().tracker.enabled &&
      fi.ego.state == EgoState::kUnknown;
    if (no_data) {
      s.status = StatsMsg::STATUS_NO_DATA;
      s.status_text = "NO_DATA: no input for " + fmt("%.2f", age) + " s";
    } else if (invalid_input_.load()) {
      s.status = StatsMsg::STATUS_INVALID_INPUT;
      s.status_text = "INVALID_INPUT: PointCloud2 layout unusable";
    } else if (overload_now) {
      s.status = StatsMsg::STATUS_OVERLOAD;
      s.status_text = "OVERLOAD: processing exceeded " + fmt("%.1f", max_processing_ms_) + " ms";
    } else if (snap.health.level != HealthLevel::kOk) {
      s.status = StatsMsg::STATUS_SENSOR_DEGRADED;
      s.status_text = "SENSOR_DEGRADED: " + snap.health.message;
    } else if (fi.ground_status == static_cast<std::uint8_t>(GroundStatus::kFailed)) {
      s.status = StatsMsg::STATUS_GROUND_FAILED;
      s.status_text = "GROUND_FAILED: using configured fallback plane";
    } else if (fi.ground_status == static_cast<std::uint8_t>(GroundStatus::kDegraded)) {
      s.status = StatsMsg::STATUS_GROUND_DEGRADED;
      s.status_text = "GROUND_DEGRADED: large share of points on fallback plane";
    } else if (ego_unknown) {
      s.status = StatsMsg::STATUS_EGO_UNKNOWN;
      s.status_text = "EGO_UNKNOWN: " + fi.ego.reason;
    } else {
      s.status = StatsMsg::STATUS_OK;
      s.status_text = "OK";
    }
    s.input_fps = static_cast<float>(snap.perf.input_fps);
    s.output_fps = static_cast<float>(snap.perf.output_fps);
    s.sensor_rate_hz = static_cast<float>(snap.health.stamp_rate_hz);
    s.frames_received = snap.perf.frames_received;
    s.frames_processed = snap.perf.frames_processed;
    s.frames_dropped = snap.perf.frames_dropped;
    s.overload_frames = snap.perf.overload_frames;
    s.last_input_age_s = static_cast<float>(age);
    s.frame_gaps = snap.health.gaps;
    s.missed_frames_estimate = snap.health.missed_frames;
    s.time_jumps = snap.health.time_jumps;
    s.partial_scans = snap.health.partial_scans;
    s.low_valid_scans = snap.health.low_valid_scans;
    s.valid_return_ratio = static_cast<float>(snap.health.valid_ratio);
    s.azimuth_coverage = static_cast<float>(snap.health.azimuth_coverage);
    s.sensor_health_text = snap.health.message;
    s.clock_synced = snap.health.clock_synced;
    s.clock_offset_s = snap.health.clock_offset_s;
    s.delivery_jitter_p95_ms = static_cast<float>(snap.health.jitter_p95_ms);
    s.stamp_age_mean_ms = static_cast<float>(snap.health.stamp_age_mean_ms);
    s.stamp_age_p95_ms = static_cast<float>(snap.health.stamp_age_p95_ms);
    for (const auto & st : snap.perf.stages) {
      if (st.name == toString(Stage::kRxToPublish)) {
        s.latency_rx_to_publish_mean_ms = static_cast<float>(st.mean_ms);
        s.latency_rx_to_publish_p95_ms = static_cast<float>(st.p95_ms);
        s.latency_rx_to_publish_max_ms = static_cast<float>(st.max_ms);
      }
      helios_obstacle_detection::msg::StageTiming t;
      t.name = st.name;
      t.last_ms = static_cast<float>(st.last_ms);
      t.mean_ms = static_cast<float>(st.mean_ms);
      t.p95_ms = static_cast<float>(st.p95_ms);
      t.max_ms = static_cast<float>(st.max_ms);
      s.timings.push_back(t);
    }
    s.cpu_percent = static_cast<float>(snap.res.cpu_percent);
    s.cpu_percent_total = static_cast<float>(snap.res.cpu_percent_total);
    s.rss_mb = static_cast<float>(snap.res.rss_mb);
    s.threads = snap.res.threads;
    s.points_input = fi.points_input;
    s.points_valid = fi.points_valid;
    s.points_roi = fi.points_roi;
    s.points_voxel = fi.points_voxel;
    s.points_ground = fi.points_ground;
    s.points_obstacle = fi.points_obstacle;
    s.clusters = fi.clusters;
    s.voxel_leaf_size = fi.voxel_leaf;
    s.ground_status = fi.ground_status;
    s.ground_method = pipeline_->groundMethod();
    s.ground_detail = fi.ground_detail;
    s.ego_state = static_cast<std::uint8_t>(fi.ego.state);
    s.ego_status_text = std::string(toString(fi.ego.state)) +
      (fi.ego.reason.empty() ? "" : ": " + fi.ego.reason);
    s.tracks_total = static_cast<std::uint32_t>(fi.tracker.tracks);
    s.tracks_confirmed = static_cast<std::uint32_t>(fi.tracker.confirmed);
    s.tracks_moving = static_cast<std::uint32_t>(fi.tracker.moving);
    s.tracks_static = static_cast<std::uint32_t>(fi.tracker.static_tracks);
    s.tracks_created_total = fi.tracker.created_total;
    s.tracks_occluded = static_cast<std::uint32_t>(fi.tracker.occluded);
    s.tracks_lost = static_cast<std::uint32_t>(fi.tracker.lost);
    s.tracks_reidentified_total = fi.tracker.reidentified_total;
    s.merge_splits_total = fi.tracker.merge_splits_total;
    s.tracker_predict_ms = static_cast<float>(fi.tracker.predict_ms);
    s.tracker_association_ms = static_cast<float>(fi.tracker.association_ms);
    s.tracker_occlusion_ms = static_cast<float>(fi.tracker.occlusion_ms);
    s.tracker_resets = fi.tracker.resets;
    s.tracker_mean_nis = static_cast<float>(fi.tracker.mean_nis);
    s.max_risk_level = static_cast<std::uint8_t>(fi.max_risk);
    s.background_points = fi.background_points;
    s.static_voxels = fi.static_voxels;
    s.background_learning = fi.background_learning;
    pub_stats_->publish(s);

    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      last_snapshot_ = snap;
      hud_status_ = s.status == StatsMsg::STATUS_OK ? std::string() : s.status_text;
    }

    if (overload_now) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Processing time above %.1f ms (last %.1f ms)",
        max_processing_ms_, fi.last_total_ms);
    }
    if (log_period_s_ > 0.0 &&
      std::chrono::duration<double>(now - last_log_).count() >= log_period_s_)
    {
      last_log_ = now;
      logReport(s, snap.perf);
    }
  }

  void logReport(const StatsMsg & s, const PerformanceSnapshot & snap) const
  {
    std::string timing;
    char buf[160];
    for (const auto & st : snap.stages) {
      std::snprintf(
        buf, sizeof(buf), "\n    %-20s mean %7.2f  p95 %7.2f  max %7.2f ms", st.name.c_str(),
        st.mean_ms, st.p95_ms, st.max_ms);
      timing += buf;
    }
    RCLCPP_INFO(
      get_logger(),
      "[%s] in %.1f Hz | out %.1f Hz | rx %lu proc %lu drop %lu ovl %lu gaps %lu | CPU %.0f%% "
      "(1 core) RSS %.0f MB | pts in %u roi %u voxel %u ground %u obst %u | clusters %u | "
      "tracks %u conf %u moving %u | ego %s | risk %s | ground %s%s",
      s.status_text.c_str(), s.input_fps, s.output_fps,
      static_cast<unsigned long>(s.frames_received),
      static_cast<unsigned long>(s.frames_processed),
      static_cast<unsigned long>(s.frames_dropped),
      static_cast<unsigned long>(s.overload_frames),
      static_cast<unsigned long>(s.frame_gaps), s.cpu_percent, s.rss_mb, s.points_input,
      s.points_roi, s.points_voxel, s.points_ground, s.points_obstacle, s.clusters,
      s.tracks_total, s.tracks_confirmed, s.tracks_moving, s.ego_status_text.c_str(),
      toString(static_cast<RiskLevel>(s.max_risk_level)), s.ground_detail.c_str(),
      timing.c_str());
  }


  // parameters
  std::string input_topic_;
  int input_depth_{1};
  bool input_reliable_{true};
  bool publish_debug_clouds_{true};
  double marker_lifetime_s_{0.3};
  double input_timeout_s_{0.5};
  double max_processing_ms_{80.0};
  double stats_period_s_{1.0};
  double log_period_s_{5.0};
  double expected_rate_hz_{10.0};
  rclcpp::Duration marker_lifetime_{0, 0};
  TrackVizConfig viz_;

  // processing
  std::unique_ptr<PerformanceMonitor> monitor_;
  std::unique_ptr<PerceptionPipeline> pipeline_;
  std::unique_ptr<SensorHealthMonitor> health_;
  ResourceMonitor resources_;
  diagnostic_updater::Updater diagnostics_;

  // ROS interfaces
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_filtered_, pub_ground_, pub_obstacle_pts_,
    pub_clusters_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_,
    pub_track_markers_;
  rclcpp::Publisher<ObstacleArrayMsg>::SharedPtr pub_obstacles_;
  rclcpp::Publisher<TrackArrayMsg>::SharedPtr pub_tracks_;
  rclcpp::Publisher<helios_obstacle_detection::msg::CollisionWarning>::SharedPtr pub_warning_;
  WarningLevel last_warning_level_{WarningLevel::kClear};
  rclcpp::Publisher<StatsMsg>::SharedPtr pub_stats_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  // single-slot input mailbox
  std::mutex slot_mutex_;
  std::condition_variable slot_cv_;
  PointCloud2::ConstSharedPtr pending_;
  SteadyClock::time_point pending_rx_;
  double pending_rx_wall_{0.0};
  bool stop_{false};
  std::thread worker_;

  // single-slot visualisation mailbox
  std::mutex viz_mutex_;
  std::condition_variable viz_cv_;
  std::shared_ptr<FrameOutput> viz_frame_;
  std_msgs::msg::Header viz_header_;
  bool viz_stop_{false};
  std::thread viz_thread_;

  // state
  std::atomic<SteadyClock::rep> last_rx_ns_{0};
  std::atomic<bool> invalid_input_{false};
  SteadyClock::time_point start_time_;
  SteadyClock::time_point last_log_{};
  std::uint64_t last_overload_count_{0};
  std::uint64_t last_diag_overload_{0};
  bool data_lost_{false};
  mutable std::mutex frame_info_mutex_;
  FrameInfo frame_info_;
  mutable std::mutex status_mutex_;
  Snapshot last_snapshot_;
  std::string hud_status_;
};

}  // namespace helios_obstacle_detection

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int rc = 0;
  try {
    auto node = std::make_shared<helios_obstacle_detection::ObstacleDetectionNode>();
    rclcpp::spin(node);
    node.reset();  // joins worker + viz threads before context teardown
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("obstacle_detector"), "Fatal: %s", e.what());
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
