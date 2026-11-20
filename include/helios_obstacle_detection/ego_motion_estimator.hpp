// LiDAR-only ego-motion estimation (no IMU / GPS / wheel odometry available).
//
// Consecutive non-ground clouds are registered with a planar (x, y, yaw) point-to-plane ICP
// with a Cauchy kernel. Dynamic objects are handled three ways: (1) regions of tracks that were
// MOVING in the previous frame are excluded, (2) the redescending kernel down-weights the rest,
// (3) hypothesis consensus: the ICP optimum is compared with the zero-motion and the
// constant-velocity hypotheses by counting geometrically consistent points; a simpler
// hypothesis with (almost) the same support wins. This prevents a static sensor from being
// "dragged" by a crowd of moving objects. From the Gauss-Newton Hessian H we get
//   * covariance  Cov = sigma^2 * H^-1   (sigma^2 = weighted residual variance)
//   * degeneracy  smallest eigenvalue of the translational block of H / N
//                 (long corridor, single wall, open field -> translation unobservable)
// The estimate is used to express object tracks in a local odometry frame so that static objects
// stay static while the sensor moves, and its covariance is propagated into every track
// measurement. When registration is unreliable the motion is predicted (constant velocity) with
// growing covariance for a short time and then declared UNKNOWN: downstream code must not claim
// ground-relative velocities in that case.
//
// Assumptions / limits (reported, not hidden):
//   * planar motion (roll / pitch / z ignored) - valid for a ground vehicle on moderate slopes
//   * no per-point timestamps in the driver output -> no intra-scan de-skewing; at 10 Hz and
//     v = 10 m/s a scan is smeared by up to 1 m which degrades the estimate at higher speed.
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <pcl/kdtree/kdtree_flann.h>

#include "helios_obstacle_detection/types.hpp"

namespace helios_obstacle_detection
{

enum class EgoState : std::uint8_t
{
  kUnknown = 0,      // no usable estimate: velocities are only sensor-relative
  kStationary = 1,
  kMoving = 2,
  kUnreliable = 3,   // registration failed recently; short-term constant-velocity prediction
};

const char * toString(EgoState s) noexcept;

struct EgoMotionConfig
{
  bool enabled{true};
  float voxel_size{0.30F};          // [m] registration down-sampling
  float min_range{1.0F};            // [m] horizontal
  float max_range{25.0F};
  int max_points{1500};
  int normal_neighbors{8};
  float max_correspondence{1.0F};   // [m]
  int max_iterations{15};
  float convergence_translation{1e-3F};  // [m]
  float convergence_rotation{1e-4F};     // [rad]
  float robust_kernel_scale{0.07F}; // [m] Cauchy kernel scale
  float consensus_threshold{0.06F}; // [m] |point-to-plane residual| counted as support
  float consensus_margin{0.02F};    // simpler hypothesis wins if support >= (1-margin) * ICP
  float min_sigma{0.01F};           // [m] residual floor (sensor noise)
  int min_correspondences{80};
  float min_inlier_ratio{0.30F};
  float min_degeneracy{0.05F};      // smallest translational eigenvalue of H / N
  float max_velocity_sigma{0.30F};  // [m/s] 1-sigma; above -> unreliable
  float max_speed{20.F};            // [m/s] plausibility
  float max_yaw_rate{1.5F};         // [rad/s]
  float stationary_speed{0.08F};    // [m/s]
  float stationary_yaw_rate{0.03F}; // [rad/s]
  int state_hysteresis_frames{3};
  float max_prediction_time{0.5F};  // [s] unreliable longer than this -> UNKNOWN
  float prediction_accel_sigma{2.0F};  // [m/s^2] covariance growth while predicting
  float max_dt{0.5F};               // [s] larger gaps reset the reference scan
  float max_vertical_normal{0.8F};  // |n_z| above -> correspondence ignored (floor/ceiling)

  void validate() const;
};

/// Circle in the current sensor frame whose points are excluded from registration.
struct ExclusionZone
{
  Eigen::Vector2f center{Eigen::Vector2f::Zero()};
  float radius{0.F};
};

struct EgoMotion
{
  EgoState state{EgoState::kUnknown};
  bool valid{false};       // velocity usable (measured or short-term predicted)
  bool measured{false};    // registration succeeded in this frame
  double dt{0.0};
  // Pose of the current sensor frame in the previous one: p_prev = R(dyaw) p_cur + (dx, dy).
  Eigen::Vector3d delta{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d delta_cov{Eigen::Matrix3d::Identity()};
  // Sensor velocity over ground, expressed in the CURRENT sensor axes.
  Eigen::Vector2d velocity{Eigen::Vector2d::Zero()};
  double yaw_rate{0.0};
  Eigen::Matrix3d velocity_cov{Eigen::Matrix3d::Identity()};  // (vx, vy, wz)
  // Accumulated local odometry pose of the current sensor frame (x, y, yaw). Drifts; only
  // relative consistency over a few seconds matters for tracking.
  Eigen::Vector3d pose{Eigen::Vector3d::Zero()};
  int correspondences{0};
  int iterations{0};
  int points{0};
  int excluded_points{0};  // inside exclusion zones (moving tracks)
  int support_icp{0};      // consensus counts of the hypotheses
  int support_zero{0};
  std::string hypothesis;  // "icp", "zero", "constant_velocity"
  float inlier_ratio{0.F};
  float rmse{0.F};
  float degeneracy{0.F};
  std::string reason;      // why not measured / not valid
};

class EgoMotionEstimator
{
public:
  explicit EgoMotionEstimator(const EgoMotionConfig & config);

  /// `cloud`: non-ground points of the current frame (sensor frame). `stamp` in seconds.
  /// `exclude`: regions of moving objects (predicted from the previous frame's tracks).
  EgoMotion update(
    const Cloud & cloud, double stamp, const std::vector<ExclusionZone> & exclude = {});
  void reset();

  const EgoMotionConfig & config() const noexcept {return config_;}
  const EgoMotion & last() const noexcept {return last_;}

private:
  struct Frame
  {
    CloudPtr cloud;
    std::vector<Eigen::Vector3f> normals;
    std::vector<std::uint8_t> normal_ok;
    std::shared_ptr<pcl::KdTreeFLANN<PointT>> tree;
    double stamp{0.0};
  };

  std::shared_ptr<Frame> prepare(
    const Cloud & cloud, double stamp, const std::vector<ExclusionZone> & exclude,
    int & excluded) const;
  bool registerFrames(
    const Frame & source, const Frame & target, Eigen::Vector3d & x, EgoMotion & out) const;
  int support(const Frame & source, const Frame & target, const Eigen::Vector3d & x) const;
  static Eigen::Vector3d predictedDelta(
    const Eigen::Vector2d & v_current_axes, double yaw_rate, double dt);
  void finalize(EgoMotion & out, double dt);

  EgoMotionConfig config_;
  std::shared_ptr<Frame> prev_;
  EgoMotion last_;
  double last_valid_stamp_{-1.0};
  EgoState pending_state_{EgoState::kUnknown};
  int pending_count_{0};
};

}  // namespace helios_obstacle_detection
