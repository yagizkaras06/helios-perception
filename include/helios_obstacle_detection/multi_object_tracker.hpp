// Multi-object tracker for LiDAR clusters.
//
//  * State per track: constant-velocity Kalman filter in a local odometry frame (ego-motion
//    compensated with the LiDAR-only estimate), plus smoothed box shape.
//  * Measurement noise grows with range, sparse clusters and object extent (the box centre of a
//    partially visible object moves when its visible part changes), and includes the
//    uncertainty of this frame's ego-motion increment (yaw error -> lateral error ~ range).
//  * Association: gated (chi-square + metric gate) global optimal assignment (Hungarian) on
//    cost = NIS + ln|S|.
//  * Lifecycle (v0.3, occlusion aware):
//      TENTATIVE --confirm_hits--> CONFIRMED --miss--> OCCLUDED --hit--> CONFIRMED
//      OCCLUDED --(unexplained for max_coast_time_s | occluded for occluded_max_time_s |
//                  position sigma > max_position_sigma)--> LOST
//      LOST --re-identified--> CONFIRMED,  LOST --lost_max_time_s--> DELETED
//      TENTATIVE --more than tentative_max_misses misses--> DELETED
//    An occlusion is "explained" when something nearer covers the predicted azimuth span, the
//    predicted box lies inside another object's cluster (merge) or it is out of range; only then
//    is a track kept for the long occlusion time. OCCLUDED tracks are output as predictions
//    (flagged, growing covariance); LOST tracks are kept internally for re-identification only.
//  * Partial-view measurement model: the visible points of an object lie inside its full box, so
//    a shrunken detection only constrains the centre to an interval (per box axis) instead of
//    dragging it (and the velocity) towards the visible part.
//  * Merge handling: a cluster that contains the predicted boxes of several established tracks is
//    split between them by their predicted boxes (at most `merge_max_tracks`), so two people
//    walking past each other keep their identities.
//  * Motion classification (UNKNOWN / STATIC / MOVING) combines two independent kinds of evidence
//      - box-centre regression over a time window (shape-consistent measurements only), and
//      - point-surface consistency: median 2D nearest-neighbour distance between the track's
//        current points and its points ~0.5 s earlier, both in the odometry frame. Surfaces of a
//        static object coincide whichever faces are visible, so this is immune to the
//        viewpoint-induced box-centre drift that a moving sensor causes.
//    It also needs a usable ego-motion estimate and persistence for N frames (hysteresis).
//    Otherwise it stays UNKNOWN - the system never claims "moving" or "static" without evidence.
//    Lifecycle and motion state are independent: without a measurement (OCCLUDED / LOST) the
//    motion state is frozen, never re-derived from the prediction.
#pragma once

#include <cstdint>
#include <deque>
#include <limits>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "helios_obstacle_detection/bounding_box.hpp"
#include "helios_obstacle_detection/ego_motion_estimator.hpp"
#include "helios_obstacle_detection/kalman_filter.hpp"

namespace helios_obstacle_detection
{

enum class TrackState : std::uint8_t {kTentative = 0, kConfirmed = 1, kOccluded = 2, kLost = 3};
enum class MotionState : std::uint8_t {kUnknown = 0, kStatic = 1, kMoving = 2};
/// Why an unobserved track is still kept (OCCLUDED / LOST).
enum class OcclusionReason : std::uint8_t
{
  kNone = 0,          // observed in this scan
  kOccluder = 1,      // something nearer covers the predicted azimuth span
  kMerged = 2,        // predicted box inside another object's cluster
  kOutOfRange = 3,    // beyond the usable range
  kUnexplained = 4    // line of sight looks free: object vanished / missed by segmentation
};
/// Trust in the ground-relative velocity / heading / TTC of a track.
enum class VelocityStatus : std::uint8_t
{
  kUnknown = 0,       // no ground reference (ego motion unknown) or no estimate yet
  kUnreliable = 1,    // estimate exists but is predicted / too uncertain / ego unreliable
  kReliable = 2
};

const char * toString(TrackState s) noexcept;
const char * toString(MotionState s) noexcept;
const char * toString(OcclusionReason s) noexcept;
const char * toString(VelocityStatus s) noexcept;

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
  float max_coast_time_s{0.8F};         // [s] unexplained disappearance -> LOST
  float occluded_max_time_s{2.5F};      // [s] explained occlusion (occluder / merge) -> LOST
  float lost_max_time_s{2.0F};          // [s] LOST tracks kept for re-identification
  float max_position_sigma{2.0F};       // [m] OCCLUDED tracks more uncertain -> LOST
  float occlusion_bin_deg{1.0F};        // azimuth resolution of the occlusion test
  float occlusion_margin{0.3F};         // [m] an occluder must be this much nearer
  float reid_max_distance{3.0F};        // [m] re-identification gate of LOST tracks
  float reid_speed{1.0F};               // [m/s] gate growth with the lost time (unmodelled motion)
  // association cost terms (dimensionless, added to NIS + ln|S|)
  float size_cost_weight{1.0F};         // log-size mismatch / ln(1.5), squared
  float iou_cost_weight{1.0F};          // (1 - BEV IoU) of predicted track box and detection
  float background_cost{4.0F};          // dynamic track (ever moved) <-> static-background cluster
  float max_velocity_sigma{0.6F};       // [m/s] speed sigma above -> velocity UNRELIABLE
  bool partial_view_model{true};
  bool merge_split{true};
  float max_range{30.F};                // [m] usable range (set from the pre-filter)
  int merge_max_tracks{3};              // hypotheses per merged cluster (bounded)
  float merge_max_time_s{4.0F};         // [s] tracks merged longer than this are not split
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
  float motion_evidence_timeout_s{2.0F};  // [s] MOVING without any motion evidence -> UNKNOWN
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
  OcclusionReason occlusion{OcclusionReason::kNone};
  VelocityStatus velocity_status{VelocityStatus::kUnknown};
  bool predicted{false};               // no measurement in this scan: position is a prediction
  bool merged{false};                  // measured from a split merged cluster (partial evidence)
  bool relative_only{false};           // ego motion unknown: only sensor-relative motion observable
  float confidence{0.F};               // track existence confidence [0, 1] (heuristic, see docs)
  float motion_confidence{0.F};        // evidence strength of the motion state [0, 1] (heuristic)
  bool ever_moved{false};
  float time_since_moving_s{std::numeric_limits<float>::infinity()};  // since last MOVING
  bool velocity_reliable{false};       // == (velocity_status == kReliable)
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
  std::size_t occluded{0};
  std::size_t lost{0};
  std::uint64_t created_total{0};
  std::uint64_t deleted_total{0};
  std::uint64_t resets{0};
  std::uint64_t reidentified_total{0};  // LOST tracks recovered
  std::uint64_t merge_splits_total{0};  // merged clusters split between tracks
  std::size_t associated{0};
  // timing of the last update [ms]
  double predict_ms{0.0};
  double association_ms{0.0};
  double occlusion_ms{0.0};             // occlusion test + merge split + re-identification
  double update_ms{0.0};
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
    // occlusion handling
    OcclusionReason occlusion{OcclusionReason::kNone};
    double unexplained_s{0.0};          // accumulated unexplained-miss time since last update
    double lost_since{-1.0};
    double merged_since{-1.0};          // first frame of the current merge, -1 if not merged
    bool merged{false};
    double last_meas_sigma{0.0};
    double last_moving{-1.0};           // stamp of the last frame classified MOVING
    double last_fit{-1.0};              // stamp of the last valid window regression        // largest axis sigma of the last measurement
  };

  /// Measurement of a track from (part of) a cluster.
  struct Measurement
  {
    Eigen::Vector2d z{Eigen::Vector2d::Zero()};
    Eigen::Matrix2d R{Eigen::Matrix2d::Identity()};
    bool partial{false};                // centre constrained by the partial-view model
    double max_sigma{0.0};
  };
  /// Points of a cluster (odom frame, 2D) handed to a track by the merge split.
  struct SubCluster
  {
    int detection{-1};
    pcl::Indices indices;
  };

  Eigen::Matrix2d measurementCov(
    const ObstacleCandidate & d, const EgoMotion & ego, const Eigen::Matrix2d & rot) const;
  /// Extra measurement std-dev [m] when the observed box shrank / grew vs. the track shape
  /// (partial occlusion, merge / split): the box centre is then biased.
  double shapeInconsistency(const Track & t, const ObstacleCandidate & d, double yaw_odom) const;
  void classifyMotion(
    Track & t, double stamp, bool ego_usable, bool ego_stationary, bool ego_unreliable) const;
  void updateShape(
    Track & t, const ObstacleCandidate & d, double yaw_odom, bool reset = false) const;
  void updateSurfaceEvidence(
    Track & t, const ObstacleCandidate & d, const Cloud & cloud, double stamp,
    const Eigen::Matrix2d & rot, const Eigen::Vector2d & trans) const;
  /// Partial-view measurement from the extents [a_min, a_max] x [b_min, b_max] of the visible
  /// points along the track's box axes (relative to its predicted centre).
  Measurement partialView(
    const Track & t, double a_min, double a_max, double b_min, double b_max,
    const Eigen::Matrix2d & R_extra) const;
  /// Measurement of track t from detection d (partial-view model when d is smaller than t).
  Measurement measure(
    const Track & t, const ObstacleCandidate & d, const Eigen::Vector2d & z_raw,
    const Eigen::Matrix2d & R_raw, double yaw_odom) const;
  double sizeCost(const Track & t, const ObstacleCandidate & d, double yaw_odom) const;
  double iouCost(const Track & t, const ObstacleCandidate & d, const Eigen::Vector2d & c_odom,
    double yaw_odom) const;
  void computeOcclusionHorizon(const Cloud & cloud);
  /// Share of the azimuth span of a box (sensor frame) covered by something nearer than it.
  double occludedFraction(const Eigen::Vector2d & pos_s, const Eigen::Vector3f & size,
    double yaw_s) const;
  /// Dynamic tracks (moved at some point) get the long occlusion time; others only the short one.
  static bool isDynamic(const Track & t)
  {
    return t.ever_moved || t.motion == MotionState::kMoving;
  }
  OcclusionReason explainMiss(
    const Track & t, const Eigen::Vector2d & pos_s, const std::vector<ObstacleCandidate> & dets,
    const std::vector<Eigen::Vector2d> & det_centres_s) const;
  float trackConfidence(const Track & t, double stamp) const;

  TrackerConfig config_;
  std::vector<Track> tracks_;
  std::vector<float> horizon_;          // min obstacle range per azimuth bin (this scan)
  float max_range_seen_{0.F};
  double last_yaw_{0.0};                // ego yaw of the current scan (odom -> sensor)
  std::uint32_t next_id_{1};
  double last_stamp_{-1.0};
  TrackerStats stats_;
};

}  // namespace helios_obstacle_detection
