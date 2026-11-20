#include "helios_obstacle_detection/ego_motion_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <pcl/filters/voxel_grid.h>

namespace helios_obstacle_detection
{

namespace
{
double wrapPi(double a)
{
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}
}  // namespace

const char * toString(EgoState s) noexcept
{
  switch (s) {
    case EgoState::kUnknown: return "UNKNOWN";
    case EgoState::kStationary: return "STATIONARY";
    case EgoState::kMoving: return "MOVING";
    case EgoState::kUnreliable: return "UNRELIABLE";
  }
  return "?";
}

void EgoMotionConfig::validate() const
{
  if (!(voxel_size >= 0.05F && min_range >= 0.F && max_range > min_range && max_points >= 50 &&
    normal_neighbors >= 5 && max_correspondence > 0.F && max_iterations >= 1 &&
    robust_kernel_scale > 0.F && consensus_threshold > 0.F && consensus_margin >= 0.F &&
    consensus_margin < 0.5F && min_sigma > 0.F && min_correspondences >= 10))
  {
    throw std::invalid_argument("ego_motion: registration parameters out of range");
  }
  if (!(min_inlier_ratio > 0.F && min_inlier_ratio <= 1.F && min_degeneracy >= 0.F &&
    max_velocity_sigma > 0.F && max_speed > 0.F && max_yaw_rate > 0.F &&
    stationary_speed > 0.F && stationary_yaw_rate > 0.F && state_hysteresis_frames >= 1 &&
    max_prediction_time >= 0.F && prediction_accel_sigma > 0.F && max_dt > 0.F &&
    max_vertical_normal > 0.F && max_vertical_normal <= 1.F))
  {
    throw std::invalid_argument("ego_motion: validity / state parameters out of range");
  }
}

EgoMotionEstimator::EgoMotionEstimator(const EgoMotionConfig & config)
: config_(config)
{
  config_.validate();
}

void EgoMotionEstimator::reset()
{
  prev_.reset();
  last_ = EgoMotion{};
  last_valid_stamp_ = -1.0;
  pending_state_ = EgoState::kUnknown;
  pending_count_ = 0;
}

std::shared_ptr<EgoMotionEstimator::Frame> EgoMotionEstimator::prepare(
  const Cloud & cloud, double stamp, const std::vector<ExclusionZone> & exclude,
  int & excluded) const
{
  auto ranged = std::make_shared<Cloud>();
  ranged->reserve(cloud.size());
  const float r0 = config_.min_range * config_.min_range;
  const float r1 = config_.max_range * config_.max_range;
  excluded = 0;
  for (const auto & p : cloud) {
    const float r2 = p.x * p.x + p.y * p.y;
    if (r2 < r0 || r2 > r1) {
      continue;
    }
    bool skip = false;
    for (const auto & z : exclude) {
      const float dx = p.x - z.center.x(), dy = p.y - z.center.y();
      if (dx * dx + dy * dy <= z.radius * z.radius) {
        skip = true;
        break;
      }
    }
    if (skip) {
      ++excluded;
      continue;
    }
    ranged->push_back(p);
  }
  auto f = std::make_shared<Frame>();
  f->stamp = stamp;
  f->cloud = std::make_shared<Cloud>();
  if (!ranged->empty()) {
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(ranged);
    vg.setLeafSize(config_.voxel_size, config_.voxel_size, config_.voxel_size);
    Cloud tmp;
    vg.filter(tmp);
    const auto max_pts = static_cast<std::size_t>(config_.max_points);
    if (tmp.size() > max_pts) {
      // deterministic uniform subsampling
      const double step = static_cast<double>(tmp.size()) / static_cast<double>(max_pts);
      f->cloud->reserve(max_pts);
      for (std::size_t k = 0; k < max_pts; ++k) {
        f->cloud->push_back(tmp[static_cast<std::size_t>(static_cast<double>(k) * step)]);
      }
    } else {
      *f->cloud = std::move(tmp);
    }
  }
  const std::size_t n = f->cloud->size();
  f->normals.assign(n, Eigen::Vector3f::UnitZ());
  f->normal_ok.assign(n, 0);
  if (n < static_cast<std::size_t>(config_.normal_neighbors)) {
    return f;
  }
  f->tree = std::make_shared<pcl::KdTreeFLANN<PointT>>(false);
  f->tree->setInputCloud(f->cloud);
  pcl::Indices nn;
  std::vector<float> d2;
  // neighbours farther than 4 voxels -> isolated point, unreliable normal
  const float max_nn_d2 = 16.F * config_.voxel_size * config_.voxel_size;
  for (std::size_t i = 0; i < n; ++i) {
    if (f->tree->nearestKSearch((*f->cloud)[i], config_.normal_neighbors, nn, d2) <
      config_.normal_neighbors)
    {
      continue;
    }
    if (*std::max_element(d2.begin(), d2.end()) > max_nn_d2) {
      continue;
    }
    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    for (const auto j : nn) {
      mean += (*f->cloud)[static_cast<std::size_t>(j)].getVector3fMap();
    }
    mean /= static_cast<float>(nn.size());
    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for (const auto j : nn) {
      const Eigen::Vector3f d = (*f->cloud)[static_cast<std::size_t>(j)].getVector3fMap() - mean;
      cov.noalias() += d * d.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov);
    const auto & ev = es.eigenvalues();
    if (!(ev[1] > 3.F * ev[0])) {
      continue;  // not planar enough
    }
    const Eigen::Vector3f nrm = es.eigenvectors().col(0);
    if (std::fabs(nrm.z()) > config_.max_vertical_normal) {
      continue;  // horizontal surface: no information on x, y, yaw
    }
    f->normals[i] = nrm;
    f->normal_ok[i] = 1;
  }
  return f;
}

bool EgoMotionEstimator::registerFrames(
  const Frame & src, const Frame & tgt, Eigen::Vector3d & x, EgoMotion & out) const
{
  const std::size_t n = src.cloud->size();
  out.points = static_cast<int>(n);
  if (n < static_cast<std::size_t>(config_.min_correspondences) || !tgt.tree) {
    out.reason = "too few registration points";
    return false;
  }
  const float max_d2 = config_.max_correspondence * config_.max_correspondence;
  pcl::Indices nn(1);
  std::vector<float> d2(1);
  Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
  double wrr = 0.0;
  int corr = 0;
  bool converged = false;

  for (int it = 0; it < config_.max_iterations; ++it) {
    const double c = std::cos(x[2]), s = std::sin(x[2]);
    H.setZero();
    Eigen::Vector3d g = Eigen::Vector3d::Zero();
    wrr = 0.0;
    corr = 0;
    for (std::size_t i = 0; i < n; ++i) {
      const auto & p = (*src.cloud)[i];
      PointT q;
      q.x = static_cast<float>(c * p.x - s * p.y + x[0]);
      q.y = static_cast<float>(s * p.x + c * p.y + x[1]);
      q.z = p.z;
      if (tgt.tree->nearestKSearch(q, 1, nn, d2) < 1 || d2[0] > max_d2) {
        continue;
      }
      const auto j = static_cast<std::size_t>(nn[0]);
      if (!tgt.normal_ok[j]) {
        continue;
      }
      const Eigen::Vector3f & nr = tgt.normals[j];
      const auto & t = (*tgt.cloud)[j];
      const double r = nr.x() * (q.x - t.x) + nr.y() * (q.y - t.y) + nr.z() * (q.z - t.z);
      const Eigen::Vector3d J(
        nr.x(), nr.y(), nr.x() * -(q.y - x[1]) + nr.y() * (q.x - x[0]));
      const double rc = r / config_.robust_kernel_scale;
      const double w = 1.0 / (1.0 + rc * rc);   // Cauchy (redescending influence)
      H.noalias() += w * J * J.transpose();
      g.noalias() += w * J * r;
      wrr += w * r * r;
      ++corr;
    }
    out.iterations = it + 1;
    if (corr < config_.min_correspondences) {
      out.correspondences = corr;
      out.reason = "too few correspondences";
      return false;
    }
    const Eigen::Vector3d dx = H.ldlt().solve(-g);
    if (!dx.allFinite()) {
      out.reason = "singular normal equations";
      return false;
    }
    x += dx;
    x[2] = wrapPi(x[2]);
    if (dx.head<2>().norm() < config_.convergence_translation &&
      std::fabs(dx[2]) < config_.convergence_rotation)
    {
      converged = true;
      break;
    }
  }

  out.correspondences = corr;
  out.inlier_ratio = static_cast<float>(corr) / static_cast<float>(n);
  const double sigma2 = std::max(
    wrr / std::max(corr - 3, 1), static_cast<double>(config_.min_sigma * config_.min_sigma));
  out.rmse = static_cast<float>(std::sqrt(wrr / std::max(corr, 1)));
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(H.topLeftCorner<2, 2>());
  out.degeneracy = static_cast<float>(es.eigenvalues()[0] / std::max(corr, 1));
  out.delta_cov = sigma2 * H.inverse();
  if (!converged) {
    out.reason = "ICP not converged";
    return false;
  }
  if (out.inlier_ratio < config_.min_inlier_ratio) {
    out.reason = "low overlap / inlier ratio";
    return false;
  }
  if (out.degeneracy < config_.min_degeneracy) {
    out.reason = "degenerate geometry (translation unobservable)";
    return false;
  }
  if (!out.delta_cov.allFinite()) {
    out.reason = "invalid covariance";
    return false;
  }
  return true;
}

Eigen::Vector3d EgoMotionEstimator::predictedDelta(
  const Eigen::Vector2d & v_current_axes, double yaw_rate, double dt)
{
  // velocity is expressed in the axes of the frame it was estimated in (end of the interval);
  // the increment's translation lives in the previous frame: t = R(dyaw) * v * dt
  const double th = yaw_rate * dt;
  const double c = std::cos(th), s = std::sin(th);
  return Eigen::Vector3d(
    (c * v_current_axes.x() - s * v_current_axes.y()) * dt,
    (s * v_current_axes.x() + c * v_current_axes.y()) * dt, th);
}

int EgoMotionEstimator::support(
  const Frame & src, const Frame & tgt, const Eigen::Vector3d & x) const
{
  if (!tgt.tree) {
    return 0;
  }
  const double c = std::cos(x[2]), s = std::sin(x[2]);
  const float max_d2 = config_.max_correspondence * config_.max_correspondence;
  pcl::Indices nn(1);
  std::vector<float> d2(1);
  int count = 0;
  for (const auto & p : *src.cloud) {
    PointT q;
    q.x = static_cast<float>(c * p.x - s * p.y + x[0]);
    q.y = static_cast<float>(s * p.x + c * p.y + x[1]);
    q.z = p.z;
    if (tgt.tree->nearestKSearch(q, 1, nn, d2) < 1 || d2[0] > max_d2) {
      continue;
    }
    const auto j = static_cast<std::size_t>(nn[0]);
    const auto & t = (*tgt.cloud)[j];
    const float r = tgt.normal_ok[j] ?
      std::fabs(tgt.normals[j].dot(q.getVector3fMap() - t.getVector3fMap())) : std::sqrt(d2[0]);
    count += r < config_.consensus_threshold;
  }
  return count;
}

EgoMotion EgoMotionEstimator::update(
  const Cloud & cloud, double stamp, const std::vector<ExclusionZone> & exclude)
{
  EgoMotion out;
  out.pose = last_.pose;
  if (!config_.enabled) {
    out.reason = "disabled";
    last_ = out;
    return out;
  }
  int excluded = 0;
  auto cur = prepare(cloud, stamp, exclude, excluded);
  out.excluded_points = excluded;
  const double dt = prev_ ? stamp - prev_->stamp : 0.0;
  out.dt = dt;

  bool measured = false;
  Eigen::Vector3d x = Eigen::Vector3d::Zero();
  if (!prev_) {
    out.reason = "first frame";
  } else if (!(dt > 1e-3 && dt <= config_.max_dt)) {
    out.reason = "time gap / jump - reference reset";
    last_valid_stamp_ = -1.0;
  } else {
    // constant-velocity initial guess, expressed as a pose increment
    if (last_.valid) {
      x = predictedDelta(last_.velocity, last_.yaw_rate, dt);
    }
    const Eigen::Vector3d x_pred = x;
    measured = registerFrames(*cur, *prev_, x, out);
    if (measured) {
      // hypothesis consensus: prefer zero motion / constant velocity when they explain the scene
      // (almost) as well as the ICP optimum
      out.support_icp = support(*cur, *prev_, x);
      out.support_zero = support(*cur, *prev_, Eigen::Vector3d::Zero());
      const double need = (1.0 - config_.consensus_margin) * out.support_icp;
      out.hypothesis = "icp";
      if (out.support_zero >= need) {
        x.setZero();
        out.hypothesis = "zero";
      } else if (x_pred.norm() > 0.0 && support(*cur, *prev_, x_pred) >= need) {
        x = x_pred;
        out.hypothesis = "constant_velocity";
      }
    }
  }

  if (measured) {
    const double c = std::cos(x[2]), s = std::sin(x[2]);
    out.delta = x;
    out.measured = true;
    out.valid = true;
    // velocity of the sensor over ground in the current axes: R^T * t / dt
    out.velocity = Eigen::Vector2d(c * x[0] + s * x[1], -s * x[0] + c * x[1]) / dt;
    out.yaw_rate = x[2] / dt;
    Eigen::Matrix3d J = Eigen::Matrix3d::Zero();
    J(0, 0) = c / dt;
    J(0, 1) = s / dt;
    J(1, 0) = -s / dt;
    J(1, 1) = c / dt;
    J(2, 2) = 1.0 / dt;
    out.velocity_cov = J * out.delta_cov * J.transpose();
    const double sv = std::sqrt(std::max(out.velocity_cov(0, 0), out.velocity_cov(1, 1)));
    if (out.velocity.norm() > config_.max_speed || std::fabs(out.yaw_rate) > config_.max_yaw_rate)
    {
      out.measured = out.valid = false;
      out.reason = "implausible motion";
    } else if (sv > config_.max_velocity_sigma) {
      out.measured = out.valid = false;
      out.reason = "velocity uncertainty too high";
    }
  }
  finalize(out, dt);
  prev_ = std::move(cur);
  last_ = out;
  return out;
}

void EgoMotionEstimator::finalize(EgoMotion & out, double dt)
{
  if (out.measured) {
    last_valid_stamp_ = prev_ ? prev_->stamp + dt : -1.0;
    const bool still = out.velocity.norm() < config_.stationary_speed &&
      std::fabs(out.yaw_rate) < config_.stationary_yaw_rate;
    const EgoState cand = still ? EgoState::kStationary : EgoState::kMoving;
    if (last_.state != EgoState::kStationary && last_.state != EgoState::kMoving) {
      out.state = cand;  // no previous decision: take it directly
      pending_count_ = 0;
    } else if (cand == last_.state) {
      out.state = cand;
      pending_count_ = 0;
    } else {
      pending_count_ = (pending_state_ == cand) ? pending_count_ + 1 : 1;
      pending_state_ = cand;
      out.state = pending_count_ >= config_.state_hysteresis_frames ? cand : last_.state;
      if (out.state == cand) {
        pending_count_ = 0;
      }
    }
    if (out.state == EgoState::kStationary) {
      // zero-velocity update: do not integrate registration noise while standing still
      out.delta.setZero();
      out.velocity.setZero();
      out.yaw_rate = 0.0;
    }
  } else {
    // prediction with growing uncertainty, then UNKNOWN
    const double since_valid =
      (last_valid_stamp_ >= 0.0 && prev_) ? (prev_->stamp + dt - last_valid_stamp_) : 1e9;
    if (last_.valid && dt > 0.0 && since_valid <= config_.max_prediction_time) {
      out.valid = true;
      out.state = EgoState::kUnreliable;
      out.velocity = last_.velocity;
      out.yaw_rate = last_.yaw_rate;
      const double qa = config_.prediction_accel_sigma * dt;
      out.velocity_cov = last_.velocity_cov;
      out.velocity_cov(0, 0) += qa * qa;
      out.velocity_cov(1, 1) += qa * qa;
      out.velocity_cov(2, 2) += 0.25 * qa * qa;
      out.delta = predictedDelta(out.velocity, out.yaw_rate, dt);
      out.delta_cov = out.velocity_cov * dt * dt;
    } else {
      out.valid = false;
      out.state = EgoState::kUnknown;
      out.delta.setZero();   // best guess: no motion, but flagged as unknown
      out.velocity.setZero();
      out.yaw_rate = 0.0;
      out.velocity_cov = Eigen::Matrix3d::Identity() * 4.0;  // (2 m/s)^2
      out.delta_cov = out.velocity_cov * std::max(dt * dt, 0.01);
    }
  }
  // integrate pose: T_odom_cur = T_odom_prev * delta
  const double c = std::cos(out.pose[2]), s = std::sin(out.pose[2]);
  out.pose[0] += c * out.delta[0] - s * out.delta[1];
  out.pose[1] += s * out.delta[0] + c * out.delta[1];
  out.pose[2] = wrapPi(out.pose[2] + out.delta[2]);
}

}  // namespace helios_obstacle_detection
