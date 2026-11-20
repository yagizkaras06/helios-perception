// Multi-object tracker for LiDAR clusters.
//
//  * State per track: constant-velocity Kalman filter in a local odometry frame (ego-motion
//    compensated with the LiDAR-only estimate), plus smoothed box shape.
//  * Measurement noise grows with range, sparse clusters and object extent (the box centre of a
//    partially visible object moves when its visible part changes), and includes the
//    uncertainty of this frame's ego-motion increment (yaw error -> lateral error ~ range).
//  * Association: gated (chi-square + metric gate) global optimal assignment (Hungarian) on
//    cost = NIS + ln|S|.
//  * Lifecycle: TENTATIVE -> CONFIRMED after `confirm_hits`; COASTING while unobserved; deleted
//    after `max_coast_time_s` (confirmed) or `tentative_max_misses` (tentative).
//  * Motion classification (UNKNOWN / STATIC / MOVING) combines two independent kinds of evidence
//      - box-centre regression over a time window (shape-consistent measurements only), and
//      - point-surface consistency: median 2D nearest-neighbour distance between the track's
//        current points and its points ~0.5 s earlier, both in the odometry frame. Surfaces of a
//        static object coincide whichever faces are visible, so this is immune to the
//        viewpoint-induced box-centre drift that a moving sensor causes.
//    It also needs a usable ego-motion estimate and persistence for N frames (hysteresis).
//    Otherwise it stays UNKNOWN - the system never claims "moving" or "static" without evidence.
#pragma once

#include <cstdint>
#include <deque>
#include <vector>

#include <Eigen/Core>

#include "helios_obstacle_detection/bounding_box.hpp"
#include "helios_obstacle_detection/ego_motion_estimator.hpp"
#include "helios_obstacle_detection/kalman_filter.hpp"

namespace helios_obstacle_detection
{

enum class TrackState : std::uint8_t {kTentative = 0, kConfirmed = 1, kCoasting = 2};
enum class MotionState : std::uint8_t {kUnknown = 0, kStatic = 1, kMoving = 2};

const char * toString(TrackState s) noexcept;
const char * toString(MotionState s) noexcept;

struct TrackerConfig
{
  bool enabled{true};
  float process_accel_sigma{2.0F};      // [m/s^2] white-noise acceleration
  float meas_sigma_base{0.08F};         // [m]
  float meas_sigma_per_meter{0.005F};   // [m/m]
  float meas_sigma_extent_factor{0.10F};// * max(L, W)
  int sparse_points{15};                // fewer points -> noise inflated by sqrt(sparse/n)
  float init_velocity_sigma{3.0F};      // [m/s] unknown initial velocity
  float gate_chi2{13.8F};               // 2 dof, 99.9 %
  float max_gate_distance{3.0F};        // [m]
  int confirm_hits{3};
  int tentative_max_misses{2};
  float max_coast_time_s{0.8F};
  float max_position_sigma{2.0F};       // [m] tracks more uncertain than this are dropped
  int max_tracks{256};
  float max_dt{1.0F};                   // [s] larger gaps / negative dt -> reset
  float shape_smoothing{0.3F};          // EMA factor for size / yaw
  // motion classification
  float moving_speed{0.30F};            // [m/s] (slow walking indoors)
  float static_speed{0.20F};            // [m/s]
  float min_age_s{0.5F};
  float displacement_window_s{0.8F};
  float moving_min_displacement{0.30F}; // [m] over the window
  float motion_sigma_factor{3.0F};      // window-regression speed must exceed k * its sigma
  int min_motion_samples{5};            // shape-consistent measurements in the window
  float shape_change_tolerance{0.25F};  // [m] box shrink/grow beyond this -> partial view
  int shape_reset_frames{5};            // that many inconsistent updates in a row -> new shape
  // Weight of shape-changed boxes in the motion regression (walking legs, partial views). The
  // static background map and the surface evidence guard static objects against centre jumps.
  float inconsistent_sample_weight{0.3F};
  float point_evidence_age_s{0.5F};     // reference point snapshot age for surface consistency
  float point_evidence_override_speed{1.5F};  // [m/s] above: centre regression alone decides
  int snapshot_points{48};
  int motion_hysteresis_frames{2};
  // Set by the pipeline when the static background map is enabled: STATIC then means "part of
  // the background (still for >= static_time_s)"; still objects that are not background yet are
  // UNKNOWN (= dynamic, not moving right now).
  bool static_requires_background{false};
  int history_length{30};               // trail samples (also used for displacement)

  void validate() const;
};

/// Output of one track in the SENSOR frame of the current scan.
struct TrackOutput
{
  std::uint32_t id{0};
  TrackState state{TrackState::kTentative};
  MotionState motion{MotionState::kUnknown};
  bool ever_moved{false};
  bool velocity_reliable{false};
  bool large_structure{false};
  bool background{false};              // belongs to the learned static background
  bool updated{false};                 // associated with a detection in this frame
  float age_s{0.F};
  std::uint32_t hits{0};
  std::uint32_t misses{0};
  float time_since_update_s{0.F};
  std::uint32_t point_count{0};

  Eigen::Vector3f center{Eigen::Vector3f::Zero()};
  Eigen::Vector3f size{Eigen::Vector3f::Zero()};
  float yaw{0.F};
  Eigen::Matrix2f position_cov{Eigen::Matrix2f::Identity()};

  Eigen::Vector2f velocity{Eigen::Vector2f::Zero()};     // ground-relative, sensor axes
  Eigen::Matrix2f velocity_cov{Eigen::Matrix2f::Identity()};
  float speed{0.F};
  float speed_sigma{0.F};
  float window_speed{0.F};             // regression speed over the motion window (classification)
  float window_speed_sigma{0.F};
  float surface_speed{-1.F};           // point-surface displacement / dt, -1 if not available
  float heading{0.F};
  float heading_sigma{0.F};
  bool heading_valid{false};
  Eigen::Vector2f relative_velocity{Eigen::Vector2f::Zero()};   // vs. moving sensor
  Eigen::Matrix2f relative_velocity_cov{Eigen::Matrix2f::Identity()};

  Eigen::Vector3f closest_point{Eigen::Vector3f::Zero()};
  float closest_distance{0.F};
  int detection_index{-1};             // index into this frame's obstacle list, -1 if coasting

  std::vector<Eigen::Vector2f> trail;  // past filtered positions (sensor frame, oldest first)
};

struct TrackerStats
{
  std::size_t tracks{0};
  std::size_t confirmed{0};
  std::size_t moving{0};
  std::size_t static_tracks{0};
  std::uint64_t created_total{0};
  std::uint64_t deleted_total{0};
  std::uint64_t resets{0};
  std::size_t associated{0};
  double mean_nis{0.0};                // filter consistency (expected ~2 for 2 dof)
};

class MultiObjectTracker
{
public:
  explicit MultiObjectTracker(const TrackerConfig & config);

  /// Runs predict / associate / update / manage for one scan. `cloud` (the cloud the detection
  /// indices refer to) enables the point-surface motion evidence; nullptr disables it.
  void update(
    double stamp, const std::vector<ObstacleCandidate> & detections, const EgoMotion & ego,
    std::vector<TrackOutput> & out, const Cloud * cloud = nullptr);
  void reset();

  const TrackerStats & stats() const noexcept {return stats_;}
  const TrackerConfig & config() const noexcept {return config_;}

private:
  struct Track
  {
    std::uint32_t id{0};
    TrackState state{TrackState::kTentative};
    MotionState motion{MotionState::kUnknown};
    MotionState pending_motion{MotionState::kUnknown};
    int pending_count{0};
    bool ever_moved{false};
    ConstantVelocityKF kf;
    double first_stamp{0.0};
    double last_update{0.0};
    std::uint32_t hits{0};
    std::uint32_t misses{0};
    float z_center{0.F};
    Eigen::Vector3f size{Eigen::Vector3f::Zero()};
    float yaw{0.F};                     // odom frame
    bool large_structure{false};
    std::uint32_t point_count{0};
    Eigen::Vector2d closest_offset{Eigen::Vector2d::Zero()};  // closest - centre (odom)
    float closest_z{0.F};
    std::deque<std::pair<double, Eigen::Vector2d>> history;  // (stamp, filtered position)
    // measured positions (odom) for the motion test; weight < 1 for shape-changed boxes
    struct MotionSample
    {
      double t;
      Eigen::Vector2d p;
      double w;
    };
    std::deque<MotionSample> motion_samples;
    int detection_index{-1};
    float window_speed{0.F};
    float window_speed_sigma{0.F};
    bool window_valid{false};
    Eigen::Vector2d window_velocity{Eigen::Vector2d::Zero()};   // odom, regression
    bool background{false};             // last associated detection was static background
    int shape_streak{0};                // consecutive shape-inconsistent updates
    // odom-frame point snapshots of the object surface (subsampled)
    std::deque<std::pair<double, std::vector<Eigen::Vector2f>>> snapshots;
    float surface_speed{-1.F};
  };

  Eigen::Matrix2d measurementCov(
    const ObstacleCandidate & d, const EgoMotion & ego, const Eigen::Matrix2d & rot) const;
  /// Extra measurement std-dev [m] when the observed box shrank / grew vs. the track shape
  /// (partial occlusion, merge / split): the box centre is then biased.
  double shapeInconsistency(const Track & t, const ObstacleCandidate & d, double yaw_odom) const;
  void classifyMotion(Track & t, double stamp, bool ego_usable, bool ego_stationary) const;
  void updateShape(
    Track & t, const ObstacleCandidate & d, double yaw_odom, bool reset = false) const;
  void updateSurfaceEvidence(
    Track & t, const ObstacleCandidate & d, const Cloud & cloud, double stamp,
    const Eigen::Matrix2d & rot, const Eigen::Vector2d & trans) const;

  TrackerConfig config_;
  std::vector<Track> tracks_;
  std::uint32_t next_id_{1};
  double last_stamp_{-1.0};
  TrackerStats stats_;
};

}  // namespace helios_obstacle_detection
