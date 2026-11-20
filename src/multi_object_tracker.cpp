#include "helios_obstacle_detection/multi_object_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "helios_obstacle_detection/hungarian.hpp"

namespace helios_obstacle_detection
{

namespace
{
constexpr double kInfeasible = 1e12;

double wrapHalfPi(double a)
{
  while (a >= M_PI / 2.0) {
    a -= M_PI;
  }
  while (a < -M_PI / 2.0) {
    a += M_PI;
  }
  return a;
}

Eigen::Matrix2d rotation(double yaw)
{
  const double c = std::cos(yaw), s = std::sin(yaw);
  Eigen::Matrix2d r;
  r << c, -s, s, c;
  return r;
}
}  // namespace

const char * toString(TrackState s) noexcept
{
  switch (s) {
    case TrackState::kTentative: return "TENTATIVE";
    case TrackState::kConfirmed: return "CONFIRMED";
    case TrackState::kCoasting: return "COASTING";
  }
  return "?";
}

const char * toString(MotionState s) noexcept
{
  switch (s) {
    case MotionState::kUnknown: return "UNKNOWN";
    case MotionState::kStatic: return "STATIC";
    case MotionState::kMoving: return "MOVING";
  }
  return "?";
}

void TrackerConfig::validate() const
{
  if (!(process_accel_sigma > 0.F && meas_sigma_base > 0.F && meas_sigma_per_meter >= 0.F &&
    meas_sigma_extent_factor >= 0.F && sparse_points >= 1 && init_velocity_sigma > 0.F &&
    gate_chi2 > 0.F && max_gate_distance > 0.F))
  {
    throw std::invalid_argument("tracking: noise / gating parameters must be > 0");
  }
  if (confirm_hits < 1 || tentative_max_misses < 0 || !(max_coast_time_s > 0.F) ||
    !(max_position_sigma > 0.F) || max_tracks < 1 || !(max_dt > 0.F) ||
    !(shape_smoothing > 0.F && shape_smoothing <= 1.F))
  {
    throw std::invalid_argument("tracking: lifecycle parameters out of range");
  }
  if (!(moving_speed > static_speed && static_speed > 0.F &&
    min_age_s >= 0.F && displacement_window_s > 0.F && moving_min_displacement >= 0.F &&
    motion_hysteresis_frames >= 1 && history_length >= 2 && motion_sigma_factor > 0.F &&
    shape_reset_frames >= 1 && inconsistent_sample_weight >= 0.F &&
    inconsistent_sample_weight <= 1.F &&
    min_motion_samples >= 3 && shape_change_tolerance > 0.F && point_evidence_age_s > 0.F &&
    point_evidence_override_speed > moving_speed && snapshot_points >= 8))
  {
    throw std::invalid_argument(
            "tracking: need moving_speed > static_speed > 0 and positive windows");
  }
}

MultiObjectTracker::MultiObjectTracker(const TrackerConfig & config)
: config_(config)
{
  config_.validate();
}

void MultiObjectTracker::reset()
{
  stats_.deleted_total += tracks_.size();
  tracks_.clear();
  last_stamp_ = -1.0;
  ++stats_.resets;
}

Eigen::Matrix2d MultiObjectTracker::measurementCov(
  const ObstacleCandidate & d, const EgoMotion & ego, const Eigen::Matrix2d & rot) const
{
  const Eigen::Vector2d c(d.obb.center.x(), d.obb.center.y());
  double sigma = config_.meas_sigma_base + config_.meas_sigma_per_meter * c.norm();
  if (d.point_count < static_cast<std::size_t>(config_.sparse_points)) {
    sigma *= std::sqrt(static_cast<double>(config_.sparse_points) /
      static_cast<double>(std::max<std::size_t>(d.point_count, 1)));
  }
  const double ext = config_.meas_sigma_extent_factor *
    std::max(d.obb.size.x(), d.obb.size.y());
  Eigen::Matrix2d R = (sigma * sigma + ext * ext) * Eigen::Matrix2d::Identity();
  // Uncertainty of this frame's ego-motion increment: translation + yaw lever arm.
  R += rot * ego.delta_cov.topLeftCorner<2, 2>() * rot.transpose();
  const Eigen::Vector2d lever = rot * Eigen::Vector2d(-c.y(), c.x());
  R += ego.delta_cov(2, 2) * lever * lever.transpose();
  return R;
}

double MultiObjectTracker::shapeInconsistency(
  const Track & t, const ObstacleCandidate & d, double yaw_odom) const
{
  if (t.hits < 3) {
    return 0.0;
  }
  // measured extents expressed along the track's box axes (90 deg ambiguity resolved)
  Eigen::Vector2f m(d.obb.size.x(), d.obb.size.y());
  if (std::fabs(wrapHalfPi(yaw_odom + M_PI / 2.0 - t.yaw)) <
    std::fabs(wrapHalfPi(yaw_odom - t.yaw)))
  {
    std::swap(m.x(), m.y());
  }
  const double dl = std::fabs(m.x() - t.size.x());
  const double dw = std::fabs(m.y() - t.size.y());
  const double change = std::hypot(dl, dw);
  return change > config_.shape_change_tolerance ? 0.5 * change : 0.0;
}

void MultiObjectTracker::updateSurfaceEvidence(
  Track & t, const ObstacleCandidate & d, const Cloud & cloud, double stamp,
  const Eigen::Matrix2d & rot, const Eigen::Vector2d & trans) const
{
  t.surface_speed = -1.F;
  if (d.indices == nullptr || d.indices->empty()) {
    return;
  }
  const auto & idx = *d.indices;
  const std::size_t want = static_cast<std::size_t>(config_.snapshot_points);
  const std::size_t stride = std::max<std::size_t>(1, idx.size() / want);
  std::vector<Eigen::Vector2f> cur;
  cur.reserve(want + 1);
  const Eigen::Matrix2f r = rot.cast<float>();
  const Eigen::Vector2f tr = trans.cast<float>();
  for (std::size_t k = 0; k < idx.size(); k += stride) {
    const auto & p = cloud[static_cast<std::size_t>(idx[k])];
    cur.push_back(r * Eigen::Vector2f(p.x, p.y) + tr);
  }
  // reference: newest snapshot at least `point_evidence_age_s` old
  const std::vector<Eigen::Vector2f> * ref = nullptr;
  double ref_age = 0.0;
  for (auto it = t.snapshots.rbegin(); it != t.snapshots.rend(); ++it) {
    const double age = stamp - it->first;
    if (age >= config_.point_evidence_age_s) {
      ref = &it->second;
      ref_age = age;
      break;
    }
  }
  if (ref != nullptr && !ref->empty()) {
    thread_local std::vector<float> nn;
    nn.clear();
    for (const auto & c : cur) {
      float best = std::numeric_limits<float>::infinity();
      for (const auto & q : *ref) {
        best = std::min(best, (c - q).squaredNorm());
      }
      nn.push_back(best);
    }
    std::nth_element(nn.begin(), nn.begin() + static_cast<std::ptrdiff_t>(nn.size() / 2), nn.end());
    t.surface_speed = static_cast<float>(std::sqrt(nn[nn.size() / 2]) / ref_age);
  }
  t.snapshots.emplace_back(stamp, std::move(cur));
  while (!t.snapshots.empty() &&
    stamp - t.snapshots.front().first > 2.0 * config_.point_evidence_age_s + 0.3)
  {
    t.snapshots.pop_front();
  }
}

void MultiObjectTracker::updateShape(
  Track & t, const ObstacleCandidate & d, double yaw_odom, bool reset) const
{
  const float a = config_.shape_smoothing;
  // resolve the 90 deg ambiguity of the rectangle fit against the track's yaw
  double y0 = yaw_odom;
  Eigen::Vector3f s0 = d.obb.size;
  double y1 = yaw_odom + M_PI / 2.0;
  Eigen::Vector3f s1(d.obb.size.y(), d.obb.size.x(), d.obb.size.z());
  const bool first = t.hits <= 1 || reset;
  if (!first && std::fabs(wrapHalfPi(y1 - t.yaw)) < std::fabs(wrapHalfPi(y0 - t.yaw))) {
    std::swap(y0, y1);
    std::swap(s0, s1);
  }
  if (first) {
    t.yaw = static_cast<float>(wrapHalfPi(y0));
    t.size = s0;
    t.z_center = d.obb.center.z();
  } else {
    t.yaw = static_cast<float>(wrapHalfPi(t.yaw + a * wrapHalfPi(y0 - t.yaw)));
    t.size = (1.F - a) * t.size + a * s0;
    t.z_center = (1.F - a) * t.z_center + a * d.obb.center.z();
  }
}

void MultiObjectTracker::classifyMotion(
  Track & t, double stamp, bool ego_usable, bool ego_stationary) const
{
  // Speed from a least-squares line fit over the shape-consistent measurements of the window:
  // independent of the KF tuning and immune to occlusion-induced centre jumps (excluded).
  while (!t.motion_samples.empty() &&
    stamp - t.motion_samples.front().t > config_.displacement_window_s)
  {
    t.motion_samples.pop_front();
  }
  t.window_speed = 0.F;
  t.window_speed_sigma = std::numeric_limits<float>::infinity();
  t.window_valid = false;
  const std::size_t n = t.motion_samples.size();
  bool have_fit = false;
  double disp = 0.0;
  if (n >= static_cast<std::size_t>(config_.min_motion_samples)) {
    const double span = t.motion_samples.back().t - t.motion_samples.front().t;
    if (span >= 0.5 * config_.displacement_window_s) {
      // weighted least-squares line fit p(t) = pm + v (t - tm)
      double W = 0.0, tm = 0.0;
      Eigen::Vector2d pm = Eigen::Vector2d::Zero();
      for (const auto & s : t.motion_samples) {
        W += s.w;
        tm += s.w * s.t;
        pm += s.w * s.p;
      }
      tm /= W;
      pm /= W;
      double stt = 0.0;
      Eigen::Vector2d stp = Eigen::Vector2d::Zero();
      for (const auto & s : t.motion_samples) {
        stt += s.w * (s.t - tm) * (s.t - tm);
        stp += s.w * (s.t - tm) * (s.p - pm);
      }
      const Eigen::Vector2d v = stp / stt;
      double rss = 0.0;
      for (const auto & s : t.motion_samples) {
        rss += s.w * (s.p - (pm + v * (s.t - tm))).squaredNorm();
      }
      // per-axis residual variance (2 axes, 2 parameters each), floored by the sensor noise
      const double nn = static_cast<double>(n);
      const double sr2 = std::max(rss / W * nn / (2.0 * (nn - 2.0)),
          static_cast<double>(config_.meas_sigma_base * config_.meas_sigma_base) * 0.25);
      t.window_speed = static_cast<float>(v.norm());
      t.window_velocity = v;
      t.window_valid = true;
      t.window_speed_sigma = static_cast<float>(std::sqrt(sr2 / stt));
      disp = v.norm() * span;
      have_fit = true;
    }
  }

  if (!ego_usable) {
    t.motion = MotionState::kUnknown;  // immediately: no ground reference
    t.pending_count = 0;
    return;
  }
  MotionState cand = t.motion;   // ambiguous band: keep the current decision
  const double age = stamp - t.first_stamp;
  const bool have_surface = t.surface_speed >= 0.F;
  const bool surface_static = have_surface && t.surface_speed < config_.static_speed;
  const bool surface_moving = have_surface && t.surface_speed > config_.moving_speed;
  if (t.large_structure || t.state == TrackState::kTentative || age < config_.min_age_s) {
    cand = MotionState::kUnknown;
  } else if (have_fit) {
    const double v = t.window_speed, sv = t.window_speed_sigma;
    const bool reg_moving = v > config_.moving_speed && v > config_.motion_sigma_factor * sv &&
      disp > config_.moving_min_displacement;
    const bool reg_static = v < config_.static_speed &&
      v + config_.motion_sigma_factor * sv < config_.moving_speed;
    const bool fast = v > config_.point_evidence_override_speed;
    // Surface evidence may veto motion only when it says "static": for a slowly walking person
    // the surfaces overlap partially, so the surface rate underestimates the speed.
    if (reg_moving && (!surface_static || fast)) {
      cand = MotionState::kMoving;
    } else if (surface_static && !fast) {
      cand = MotionState::kStatic;   // surfaces coincide: centre drift is a viewpoint effect
    } else if (reg_static && !surface_moving) {
      cand = MotionState::kStatic;
    } else if (t.motion == MotionState::kUnknown) {
      cand = MotionState::kUnknown;
    }
  } else if (surface_static) {
    cand = MotionState::kStatic;   // shape keeps changing (no consistent centres), surfaces fixed
  } else if (n < 2) {
    // no consistent measurements at all (long occlusion): no evidence either way
    cand = t.motion == MotionState::kMoving ? MotionState::kMoving : MotionState::kUnknown;
  }
  // "Still for N seconds -> static" applies to a still sensor; while driving the evidence-based
  // decision is used (objects are seen too briefly and from changing viewpoints).
  if (config_.static_requires_background && ego_stationary) {
    if (t.background) {
      cand = MotionState::kStatic;     // in the background map: still for >= static_time_s
    } else if (cand == MotionState::kStatic) {
      cand = MotionState::kUnknown;    // still, but not (yet) part of the background
    }
  }
  if (cand == t.motion) {
    t.pending_count = 0;
    return;
  }
  t.pending_count = (t.pending_motion == cand) ? t.pending_count + 1 : 1;
  t.pending_motion = cand;
  if (t.pending_count >= config_.motion_hysteresis_frames) {
    t.motion = cand;
    t.pending_count = 0;
    if (cand == MotionState::kMoving) {
      t.ever_moved = true;
    }
  }
}

void MultiObjectTracker::update(
  double stamp, const std::vector<ObstacleCandidate> & dets, const EgoMotion & ego,
  std::vector<TrackOutput> & out, const Cloud * cloud)
{
  out.clear();
  double dt = last_stamp_ < 0.0 ? 0.0 : stamp - last_stamp_;
  if (last_stamp_ >= 0.0 && (dt <= 0.0 || dt > config_.max_dt)) {
    reset();  // time jump (bag loop, sensor restart) or long data gap
    dt = 0.0;
  }
  last_stamp_ = stamp;

  const Eigen::Matrix2d rot = rotation(ego.pose[2]);
  const Eigen::Vector2d trans(ego.pose[0], ego.pose[1]);
  const bool ego_usable = ego.valid && ego.state != EgoState::kUnknown;

  // ---- predict ------------------------------------------------------------------------------
  for (auto & t : tracks_) {
    t.kf.predict(dt, config_.process_accel_sigma);
    t.detection_index = -1;
  }

  // ---- measurements in the odometry frame ----------------------------------------------------
  const std::size_t nd = dets.size();
  std::vector<Eigen::Vector2d> z(nd);
  std::vector<Eigen::Matrix2d> R(nd);
  for (std::size_t j = 0; j < nd; ++j) {
    z[j] = rot * Eigen::Vector2d(dets[j].obb.center.x(), dets[j].obb.center.y()) + trans;
    R[j] = measurementCov(dets[j], ego, rot);
  }

  // ---- gated global association -------------------------------------------------------------
  const std::size_t nt = tracks_.size();
  std::vector<double> cost(nt * nd, kInfeasible);
  const double gate_d2 = static_cast<double>(config_.max_gate_distance) * config_.max_gate_distance;
  for (std::size_t i = 0; i < nt; ++i) {
    for (std::size_t j = 0; j < nd; ++j) {
      Eigen::Vector2d y;
      Eigen::Matrix2d S;
      tracks_[i].kf.innovation(z[j], R[j], y, S);
      if (y.squaredNorm() > gate_d2) {
        continue;
      }
      const double d2 = y.dot(S.ldlt().solve(y));
      if (d2 <= config_.gate_chi2) {
        cost[i * nd + j] = d2 + std::log(std::max(S.determinant(), 1e-12));
      }
    }
  }
  const std::vector<int> assign = solveAssignment(cost, nt, nd, kInfeasible);

  // ---- update -------------------------------------------------------------------------------
  std::vector<char> det_used(nd, 0);
  double nis_sum = 0.0;
  std::size_t associated = 0;
  for (std::size_t i = 0; i < nt; ++i) {
    Track & t = tracks_[i];
    const int j = assign[i];
    if (j >= 0) {
      const auto ju = static_cast<std::size_t>(j);
      const auto & d = dets[ju];
      const double meas_yaw = d.obb.yaw + ego.pose[2];
      double shape_sigma = shapeInconsistency(t, d, meas_yaw);
      t.shape_streak = shape_sigma > 0.0 ? t.shape_streak + 1 : 0;
      const bool shape_reset = t.shape_streak >= config_.shape_reset_frames;
      if (shape_reset) {
        // the object really looks different now (merge ended, new aspect): adopt the new shape
        // instead of treating every following measurement as a partial view
        shape_sigma = 0.0;
        t.shape_streak = 0;
        t.motion_samples.clear();   // centres before / after the change are not comparable
      }
      const Eigen::Matrix2d Rj = R[ju] + shape_sigma * shape_sigma * Eigen::Matrix2d::Identity();
      nis_sum += t.kf.mahalanobis2(z[ju], Rj);
      ++associated;
      t.kf.update(z[ju], Rj);
      // shape-changed boxes only count (down-weighted) with a still sensor; while driving the
      // changing viewpoint alone changes boxes of static objects
      const double w_bad = ego.state == EgoState::kStationary ?
        config_.inconsistent_sample_weight : 0.0;
      if (shape_sigma <= 0.0 || w_bad > 0.0) {
        t.motion_samples.push_back({stamp, z[ju], shape_sigma <= 0.0 ? 1.0 : w_bad});
      }
      if (cloud != nullptr) {
        updateSurfaceEvidence(t, d, *cloud, stamp, rot, trans);
      }
      det_used[ju] = 1;
      ++t.hits;
      t.misses = 0;
      t.last_update = stamp;
      t.detection_index = j;
      t.large_structure = d.large_structure;
      t.background = d.background;
      t.point_count = static_cast<std::uint32_t>(d.point_count);
      if (shape_reset) {
        updateShape(t, d, meas_yaw, true);
      } else if (shape_sigma <= 0.0) {
        updateShape(t, d, meas_yaw);   // keep the shape while partially occluded / merged
      }
      const Eigen::Vector2d cp =
        rot * Eigen::Vector2d(d.closest_point.x(), d.closest_point.y()) + trans;
      t.closest_offset = cp - t.kf.position();
      t.closest_z = d.closest_point.z();
      if (t.state == TrackState::kTentative &&
        t.hits >= static_cast<std::uint32_t>(config_.confirm_hits))
      {
        t.state = TrackState::kConfirmed;
      } else if (t.state == TrackState::kCoasting) {
        t.state = TrackState::kConfirmed;
      }
    } else {
      ++t.misses;
      t.surface_speed = -1.F;
      if (t.state == TrackState::kConfirmed) {
        t.state = TrackState::kCoasting;
      }
    }
  }
  stats_.associated = associated;
  stats_.mean_nis = associated ? nis_sum / static_cast<double>(associated) : 0.0;

  // ---- delete -------------------------------------------------------------------------------
  const double max_var = static_cast<double>(config_.max_position_sigma) *
    config_.max_position_sigma;
  const auto before = tracks_.size();
  tracks_.erase(
    std::remove_if(
      tracks_.begin(), tracks_.end(), [&](const Track & t) {
        if (t.state == TrackState::kTentative) {
          return t.misses > static_cast<std::uint32_t>(config_.tentative_max_misses);
        }
        const auto P = t.kf.positionCov();
        return stamp - t.last_update > config_.max_coast_time_s || P(0, 0) > max_var ||
        P(1, 1) > max_var;
      }), tracks_.end());
  stats_.deleted_total += before - tracks_.size();

  // ---- create (nearest unassociated detections first; dets are sorted by distance) ------------
  for (std::size_t j = 0; j < nd; ++j) {
    if (det_used[j] || tracks_.size() >= static_cast<std::size_t>(config_.max_tracks)) {
      continue;
    }
    Track t;
    t.id = next_id_++;
    if (next_id_ == 0) {
      next_id_ = 1;
    }
    t.kf.init(z[j], R[j], config_.init_velocity_sigma);
    t.first_stamp = stamp;
    t.last_update = stamp;
    t.hits = 1;
    t.detection_index = static_cast<int>(j);
    t.large_structure = dets[j].large_structure;
    t.background = dets[j].background;
    t.point_count = static_cast<std::uint32_t>(dets[j].point_count);
    updateShape(t, dets[j], dets[j].obb.yaw + ego.pose[2]);
    const Eigen::Vector2d cp =
      rot * Eigen::Vector2d(dets[j].closest_point.x(), dets[j].closest_point.y()) + trans;
    t.closest_offset = cp - z[j];
    t.closest_z = dets[j].closest_point.z();
    t.motion_samples.push_back({stamp, z[j], 1.0});
    if (cloud != nullptr) {
      updateSurfaceEvidence(t, dets[j], *cloud, stamp, rot, trans);
    }
    if (config_.confirm_hits <= 1) {
      t.state = TrackState::kConfirmed;
    }
    tracks_.push_back(std::move(t));
    ++stats_.created_total;
  }

  // ---- history, motion classification, output -------------------------------------------------
  const Eigen::Matrix2d rot_t = rot.transpose();
  const Eigen::Vector2d ego_v = ego.velocity;
  const Eigen::Matrix2d ego_vcov = ego.velocity_cov.topLeftCorner<2, 2>();
  const double ego_w = ego.yaw_rate;
  const double ego_w_var = ego.velocity_cov(2, 2);
  stats_.tracks = tracks_.size();
  stats_.confirmed = stats_.moving = stats_.static_tracks = 0;
  out.reserve(tracks_.size());

  for (auto & t : tracks_) {
    t.history.emplace_back(stamp, t.kf.position());
    while (t.history.size() > static_cast<std::size_t>(config_.history_length)) {
      t.history.pop_front();
    }
    classifyMotion(t, stamp, ego_usable, ego.state == EgoState::kStationary);

    TrackOutput o;
    o.id = t.id;
    o.state = t.state;
    o.motion = t.motion;
    o.ever_moved = t.ever_moved;
    o.large_structure = t.large_structure;
    o.updated = t.detection_index >= 0;
    o.detection_index = t.detection_index;
    o.age_s = static_cast<float>(stamp - t.first_stamp);
    o.hits = t.hits;
    o.misses = t.misses;
    o.time_since_update_s = static_cast<float>(stamp - t.last_update);
    o.point_count = t.point_count;
    o.velocity_reliable = ego_usable && !t.large_structure &&
      t.state != TrackState::kTentative && o.age_s >= config_.min_age_s;

    const Eigen::Vector2d pos_s = rot_t * (t.kf.position() - trans);
    o.center = Eigen::Vector3f(static_cast<float>(pos_s.x()), static_cast<float>(pos_s.y()),
        t.z_center);
    o.size = t.size;
    o.yaw = static_cast<float>(wrapHalfPi(t.yaw - ego.pose[2]));
    o.position_cov = (rot_t * t.kf.positionCov() * rot).cast<float>();

    // STATIC tracks report the static model (v = 0 with a small residual sigma): their KF
    // velocity only contains measurement artefacts (viewpoint / occlusion centre drift).
    Eigen::Vector2d v_s = rot_t * t.kf.velocity();
    Eigen::Matrix2d Pv_s = rot_t * t.kf.velocityCov() * rot;
    if (t.motion == MotionState::kStatic) {
      // residual sigma consistent with the STATIC evidence (window speed < static_speed)
      const double sv = 0.25 * config_.static_speed;
      v_s.setZero();
      Pv_s = sv * sv * Eigen::Matrix2d::Identity();
    }
    o.velocity = v_s.cast<float>();
    o.velocity_cov = Pv_s.cast<float>();
    o.speed = static_cast<float>(v_s.norm());
    const Eigen::Vector2d u = o.speed > 1e-6F ? Eigen::Vector2d(v_s / v_s.norm()) :
      Eigen::Vector2d(1.0, 0.0);
    const Eigen::Vector2d un(-u.y(), u.x());
    o.speed_sigma = static_cast<float>(std::sqrt(std::max(u.dot(Pv_s * u), 0.0)));
    o.window_speed = t.window_speed;
    o.window_speed_sigma = t.window_speed_sigma;
    o.surface_speed = t.surface_speed;
    o.background = t.background;
    o.heading = static_cast<float>(std::atan2(v_s.y(), v_s.x()));
    o.heading_sigma = o.speed > 1e-3F ?
      static_cast<float>(std::sqrt(std::max(un.dot(Pv_s * un), 0.0)) / o.speed) :
      static_cast<float>(M_PI);
    o.heading_valid = o.velocity_reliable && t.motion == MotionState::kMoving;
    // Heading of MOVING tracks from the 1 s regression of shape-consistent box centres: much
    // steadier than the instantaneous KF velocity (leg swing, partial views).
    if (o.heading_valid && t.window_valid && t.window_speed > config_.static_speed) {
      const Eigen::Vector2d wv = rot_t * t.window_velocity;
      o.heading = static_cast<float>(std::atan2(wv.y(), wv.x()));
      o.heading_sigma = std::min(static_cast<float>(M_PI),
          t.window_speed_sigma / std::max(t.window_speed, 1e-3F));
    }

    // Relative velocity of the object as seen from the (moving, rotating) sensor. For tracks
    // classified STATIC the object velocity is modelled as zero (small residual sigma), so the
    // relative motion comes from the ego-motion estimate only and KF velocity noise of
    // stationary objects does not create phantom closing speeds.
    const Eigen::Vector2d & v_obj = v_s;
    const Eigen::Matrix2d & Pv_obj = Pv_s;
    const Eigen::Vector2d w_x_p(-ego_w * pos_s.y(), ego_w * pos_s.x());
    o.relative_velocity = (v_obj - ego_v - w_x_p).cast<float>();
    Eigen::Matrix2d lever;
    lever << pos_s.y() * pos_s.y(), -pos_s.x() * pos_s.y(), -pos_s.x() * pos_s.y(),
      pos_s.x() * pos_s.x();
    o.relative_velocity_cov = (Pv_obj + ego_vcov + ego_w_var * lever).cast<float>();

    const Eigen::Vector2d cp_s = rot_t * (t.kf.position() + t.closest_offset - trans);
    o.closest_point = Eigen::Vector3f(static_cast<float>(cp_s.x()),
        static_cast<float>(cp_s.y()), t.closest_z);
    o.closest_distance = o.closest_point.norm();

    o.trail.reserve(t.history.size());
    for (const auto & h : t.history) {
      o.trail.push_back((rot_t * (h.second - trans)).cast<float>());
    }

    if (t.state != TrackState::kTentative) {
      ++stats_.confirmed;
    }
    if (t.motion == MotionState::kMoving) {
      ++stats_.moving;
    } else if (t.motion == MotionState::kStatic) {
      ++stats_.static_tracks;
    }
    out.push_back(std::move(o));
  }
}

}  // namespace helios_obstacle_detection
