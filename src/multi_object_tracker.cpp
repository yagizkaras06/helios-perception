#include "helios_obstacle_detection/multi_object_tracker.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <Eigen/Eigenvalues>

#include "helios_obstacle_detection/hungarian.hpp"

namespace helios_obstacle_detection
{

namespace
{
constexpr double kInfeasible = 1e12;
constexpr double kMaxPartialInterval = 1.0;   // [m] widest centre interval of a partial view

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
    case TrackState::kOccluded: return "OCCLUDED";
    case TrackState::kLost: return "LOST";
  }
  return "?";
}

const char * toString(OcclusionReason s) noexcept
{
  switch (s) {
    case OcclusionReason::kNone: return "NONE";
    case OcclusionReason::kOccluder: return "OCCLUDER";
    case OcclusionReason::kMerged: return "MERGED";
    case OcclusionReason::kOutOfRange: return "OUT_OF_RANGE";
    case OcclusionReason::kUnexplained: return "UNEXPLAINED";
  }
  return "?";
}

const char * toString(VelocityStatus s) noexcept
{
  switch (s) {
    case VelocityStatus::kUnknown: return "UNKNOWN";
    case VelocityStatus::kUnreliable: return "UNRELIABLE";
    case VelocityStatus::kReliable: return "RELIABLE";
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
  if (!(occluded_max_time_s >= max_coast_time_s && lost_max_time_s >= 0.F &&
    occlusion_bin_deg > 0.F && occlusion_bin_deg <= 10.F && occlusion_margin >= 0.F &&
    reid_max_distance > 0.F && reid_speed >= 0.F && size_cost_weight >= 0.F &&
    iou_cost_weight >= 0.F && background_cost >= 0.F && max_velocity_sigma > 0.F &&
    merge_max_tracks >= 2 && merge_max_time_s > 0.F && max_range > 0.F))
  {
    throw std::invalid_argument(
            "tracking: occlusion / re-identification / association parameters out of range "
            "(occluded_max_time_s >= max_coast_time_s, merge_max_tracks >= 2, ...)");
  }
  if (confirm_hits < 1 || tentative_max_misses < 0 || !(max_coast_time_s > 0.F) ||
    !(max_position_sigma > 0.F) || max_tracks < 1 || !(max_dt > 0.F) ||
    !(shape_smoothing > 0.F && shape_smoothing <= 1.F))
  {
    throw std::invalid_argument("tracking: lifecycle parameters out of range");
  }
  if (!(moving_speed > static_speed && static_speed > 0.F &&
    min_age_s >= 0.F && displacement_window_s > 0.F && moving_min_displacement >= 0.F &&
    motion_hysteresis_frames >= 1 && motion_evidence_timeout_s > 0.F &&
    onset_sigma_factor > 0.F && onset_min_displacement >= 0.F && onset_min_samples >= 2 &&
    history_length >= 2 && motion_sigma_factor > 0.F &&
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
  Track & t, double stamp, bool ego_usable, bool ego_stationary, bool ego_unreliable) const
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
      t.last_fit = stamp;
    }
  }

  // Motion onset (change-point) test: "still, then moving with constant velocity" fitted over the
  // window with the onset time searched among the samples. Reacts as soon as the moving part is
  // significant instead of waiting until the still samples have left the window. The search over
  // onset times is a multiple test, hence the stricter significance factor, the displacement
  // and the direction-consistency requirements.
  t.onset_speed = 0.F;
  bool onset_moving = false;
  if (config_.onset_detection && n >= static_cast<std::size_t>(config_.onset_min_samples) + 2) {
    std::vector<const Track::MotionSample *> smp;
    smp.reserve(n);
    for (const auto & q : t.motion_samples) {
      if (q.w > 0.0) {
        smp.push_back(&q);   // weighted like the long-window regression
      }
    }
    const std::size_t m = smp.size();
    const auto min_after = static_cast<std::size_t>(config_.onset_min_samples);
    double best_rss = std::numeric_limits<double>::infinity();
    Eigen::Vector2d best_v = Eigen::Vector2d::Zero();
    double best_sv = std::numeric_limits<double>::infinity(), best_dur = 0.0;
    for (std::size_t j = 0; m >= min_after + 2 && j + min_after < m; ++j) {
      const double tau = smp[j]->t;
      // p_i = a + v * max(0, t_i - tau)
      double W = 0.0, xm = 0.0;
      Eigen::Vector2d pm = Eigen::Vector2d::Zero();
      for (const auto * q : smp) {
        const double x = std::max(0.0, q->t - tau);
        W += q->w;
        xm += q->w * x;
        pm += q->w * q->p;
      }
      xm /= W;
      pm /= W;
      double sxx = 0.0;
      Eigen::Vector2d sxp = Eigen::Vector2d::Zero();
      for (const auto * q : smp) {
        const double dx = std::max(0.0, q->t - tau) - xm;
        sxx += q->w * dx * dx;
        sxp += q->w * dx * (q->p - pm);
      }
      if (sxx < 1e-9) {
        continue;
      }
      const Eigen::Vector2d v = sxp / sxx;
      double rss = 0.0;
      for (const auto * q : smp) {
        rss += q->w * (q->p - (pm + v * (std::max(0.0, q->t - tau) - xm))).squaredNorm();
      }
      if (rss < best_rss) {
        best_rss = rss;
        best_v = v;
        const double mm = static_cast<double>(m);
        const double sr2 = std::max(rss / W * mm / (2.0 * (mm - 3.0)),
            static_cast<double>(config_.meas_sigma_base * config_.meas_sigma_base) * 0.25);
        best_sv = std::sqrt(sr2 / sxx);
        best_dur = smp.back()->t - tau;
      }
    }
    if (std::isfinite(best_rss)) {
      const double v = best_v.norm();
      t.onset_speed = static_cast<float>(v);
      bool consistent_dir = v > 1e-6;
      if (consistent_dir) {
        // the last steps must advance along the fitted direction
        const Eigen::Vector2d u = best_v / v;
        for (std::size_t k = m - min_after + 1; k < m; ++k) {
          consistent_dir = consistent_dir && u.dot(smp[k]->p - smp[k - 1]->p) > 0.0;
        }
      }
      onset_moving = consistent_dir && v > config_.moving_speed &&
        v > config_.onset_sigma_factor * best_sv &&
        v * best_dur > config_.onset_min_displacement;
    }
  }

  if (!ego_usable) {
    t.motion = MotionState::kUnknown;  // immediately: no ground reference
    t.pending_count = 0;
    return;
  }
  if (ego_unreliable) {
    // ground reference too uncertain for a new absolute decision: keep the current state
    // (reported with velocity status UNRELIABLE)
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
    if ((reg_moving || onset_moving) && (!surface_static || fast)) {
      cand = MotionState::kMoving;
    } else if (surface_static && !fast) {
      cand = MotionState::kStatic;   // surfaces coincide: centre drift is a viewpoint effect
    } else if (reg_static && !surface_moving) {
      cand = MotionState::kStatic;
    } else if (t.motion == MotionState::kUnknown) {
      cand = MotionState::kUnknown;
    }
  } else if (onset_moving && !surface_static) {
    cand = MotionState::kMoving;
  } else if (surface_static) {
    cand = MotionState::kStatic;   // shape keeps changing (no consistent centres), surfaces fixed
  } else if (n < 2) {
    // no consistent measurements: keep MOVING only briefly (shape change while walking), never
    // indefinitely - a track that keeps being updated without motion evidence is UNKNOWN
    const bool recent = t.last_fit >= 0.0 && stamp - t.last_fit <= config_.motion_evidence_timeout_s;
    cand = (t.motion == MotionState::kMoving && (recent || surface_moving)) ?
      MotionState::kMoving : MotionState::kUnknown;
  } else if (t.motion == MotionState::kMoving && !surface_moving &&
    !(t.last_fit >= 0.0 && stamp - t.last_fit <= config_.motion_evidence_timeout_s))
  {
    cand = MotionState::kUnknown;
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

MultiObjectTracker::Measurement MultiObjectTracker::partialView(
  const Track & t, double a_min, double a_max, double b_min, double b_max,
  const Eigen::Matrix2d & R_extra) const
{
  // The visible points lie inside the full box [o - L/2, o + L/2] (per axis), hence the centre
  // offset o is confined to [vis_max - L/2, vis_min + L/2]. Fully visible axes give the visible
  // centre; partially visible axes give the predicted centre clamped into that interval with the
  // interval's uniform variance (no pull towards the visible part).
  const double tol = 0.5 * config_.shape_change_tolerance;
  Measurement m;
  double off[2], var[2];
  const double lo_v[2] = {a_min, b_min}, hi_v[2] = {a_max, b_max};
  const double ext[2] = {t.size.x(), t.size.y()};
  for (int k = 0; k < 2; ++k) {
    const double vis = hi_v[k] - lo_v[k];
    if (vis >= ext[k] - tol) {
      off[k] = 0.5 * (lo_v[k] + hi_v[k]);
      var[k] = 0.0;
    } else {
      const double lo = hi_v[k] - 0.5 * ext[k], hi = lo_v[k] + 0.5 * ext[k];
      off[k] = std::clamp(0.0, lo, hi);
      var[k] = (hi - lo) * (hi - lo) / 12.0;
      m.partial = true;
    }
  }
  const Eigen::Matrix2d U = rotation(t.yaw);
  m.z = t.kf.position() + U * Eigen::Vector2d(off[0], off[1]);
  m.R = R_extra + U * Eigen::Vector2d(var[0], var[1]).asDiagonal() * U.transpose();
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(m.R);
  m.max_sigma = std::sqrt(std::max(es.eigenvalues().maxCoeff(), 0.0));
  return m;
}

MultiObjectTracker::Measurement MultiObjectTracker::measure(
  const Track & t, const ObstacleCandidate & d, const Eigen::Vector2d & z_raw,
  const Eigen::Matrix2d & R_raw, double yaw_odom) const
{
  Measurement raw;
  raw.z = z_raw;
  raw.R = R_raw;
  raw.max_sigma = std::sqrt(std::max(R_raw(0, 0), R_raw(1, 1)));
  if (t.hits < 3 || t.large_structure || d.large_structure) {
    return raw;
  }
  // detection box corners projected on the track's box axes (relative to the predicted centre)
  const Eigen::Matrix2d U = rotation(t.yaw);
  const Eigen::Matrix2d D = rotation(yaw_odom);
  const Eigen::Vector2d rel = U.transpose() * (z_raw - t.kf.position());
  const Eigen::Matrix2d DU = U.transpose() * D;
  const double hx = 0.5 * d.obb.size.x(), hy = 0.5 * d.obb.size.y();
  const double ea = std::fabs(DU(0, 0)) * hx + std::fabs(DU(0, 1)) * hy;
  const double eb = std::fabs(DU(1, 0)) * hx + std::fabs(DU(1, 1)) * hy;
  const double tol = 0.5 * config_.shape_change_tolerance;
  if (2.0 * ea >= t.size.x() - tol && 2.0 * eb >= t.size.y() - tol) {
    return raw;   // fully visible (or larger: merge / growth -> shape logic)
  }
  raw.shrunk = true;
  // Partial-view model only for moving objects (their occlusion matters; static objects are
  // tracked against the background anyway) and only while the track box is plausible for the
  // visible part: a much larger box (e.g. adopted during a merge) leaves the centre free.
  if (!config_.partial_view_model || !isDynamic(t) ||
    t.size.x() - 2.0 * ea > kMaxPartialInterval || t.size.y() - 2.0 * eb > kMaxPartialInterval)
  {
    return raw;
  }
  // noise of a fully visible centre without the extent term (handled by the interval)
  Eigen::Matrix2d R_extra = R_raw;
  const double ext = config_.meas_sigma_extent_factor * std::max(d.obb.size.x(), d.obb.size.y());
  R_extra -= ext * ext * Eigen::Matrix2d::Identity();
  Measurement m = partialView(t, rel.x() - ea, rel.x() + ea, rel.y() - eb, rel.y() + eb, R_extra);
  m.shrunk = true;
  return m;
}

double MultiObjectTracker::sizeCost(
  const Track & t, const ObstacleCandidate & d, double /*yaw_odom*/) const
{
  if (config_.size_cost_weight <= 0.F || t.hits < 3) {
    return 0.0;
  }
  const double m1 = std::max(d.obb.size.x(), d.obb.size.y());
  const double m2 = std::min(d.obb.size.x(), d.obb.size.y());
  const double t1 = std::max(t.size.x(), t.size.y());
  const double t2 = std::min(t.size.x(), t.size.y());
  const double s0 = std::log(1.5);
  double e = 0.0;
  const double mm[2] = {m1, m2}, tt[2] = {t1, t2};
  for (int k = 0; k < 2; ++k) {
    const double r = std::log((mm[k] + 0.1) / (tt[k] + 0.1));
    // growth (merge, different object) costs more than shrinking (partial view / occlusion)
    e += r > 0.0 ? r * r : 0.25 * r * r;
  }
  return config_.size_cost_weight * std::min(e / (s0 * s0), 25.0);
}

double MultiObjectTracker::iouCost(
  const Track & t, const ObstacleCandidate & d, const Eigen::Vector2d & c_odom,
  double yaw_odom) const
{
  if (config_.iou_cost_weight <= 0.F || t.hits < 3) {
    return 0.0;
  }
  const Eigen::Matrix2d U = rotation(t.yaw);
  const Eigen::Vector2d rel = U.transpose() * (c_odom - t.kf.position());
  const Eigen::Matrix2d DU = U.transpose() * rotation(yaw_odom);
  const double hx = 0.5 * d.obb.size.x(), hy = 0.5 * d.obb.size.y();
  const double ea = std::fabs(DU(0, 0)) * hx + std::fabs(DU(0, 1)) * hy;
  const double eb = std::fabs(DU(1, 0)) * hx + std::fabs(DU(1, 1)) * hy;
  const double L = std::max(static_cast<double>(t.size.x()), 0.05);
  const double W = std::max(static_cast<double>(t.size.y()), 0.05);
  const double ix = std::max(0.0, std::min(rel.x() + ea, 0.5 * L) - std::max(rel.x() - ea, -0.5 * L));
  const double iy = std::max(0.0, std::min(rel.y() + eb, 0.5 * W) - std::max(rel.y() - eb, -0.5 * W));
  const double inter = ix * iy;
  const double area_d = std::max(4.0 * ea * eb, 1e-4);   // axis-aligned in the track frame
  const double iou = inter / std::max(L * W + area_d - inter, 1e-6);
  return config_.iou_cost_weight * (1.0 - std::clamp(iou, 0.0, 1.0));
}

void MultiObjectTracker::computeOcclusionHorizon(const Cloud & cloud)
{
  const auto bins = static_cast<std::size_t>(std::ceil(360.0 / config_.occlusion_bin_deg));
  horizon_.assign(bins, std::numeric_limits<float>::infinity());
  max_range_seen_ = 0.F;
  const float scale = static_cast<float>(bins) / (2.F * static_cast<float>(M_PI));
  for (const auto & p : cloud) {
    const float r = std::hypot(p.x, p.y);
    if (!std::isfinite(r)) {
      continue;
    }
    float a = std::atan2(p.y, p.x);
    if (a < 0.F) {
      a += 2.F * static_cast<float>(M_PI);
    }
    const auto b = std::min(static_cast<std::size_t>(a * scale), bins - 1);
    horizon_[b] = std::min(horizon_[b], r);
    max_range_seen_ = std::max(max_range_seen_, r);
  }
}

double MultiObjectTracker::occludedFraction(
  const Eigen::Vector2d & pos_s, const Eigen::Vector3f & size, double yaw_s) const
{
  const double r = pos_s.norm();
  if (horizon_.empty() || r < 1e-3) {
    return 0.0;
  }
  const auto bins = static_cast<int>(horizon_.size());
  const double bin = 2.0 * M_PI / bins;
  double a = std::atan2(pos_s.y(), pos_s.x());
  // half extents across / along the line of sight
  const double phi = yaw_s - a;
  const double hx = 0.5 * size.x(), hy = 0.5 * size.y();
  const double along = std::fabs(std::cos(phi)) * hx + std::fabs(std::sin(phi)) * hy;
  const double across = std::fabs(std::sin(phi)) * hx + std::fabs(std::cos(phi)) * hy;
  if (a < 0.0) {
    a += 2.0 * M_PI;
  }
  const double hw = std::atan2(across, r);
  const int b0 = static_cast<int>(std::floor((a - hw) / bin));
  const int b1 = static_cast<int>(std::floor((a + hw) / bin));
  // an occluder lies in front of the object's own near face (by the margin)
  const double near = r - along - config_.occlusion_margin;
  int total = 0, occluded = 0;
  for (int b = b0; b <= b1; ++b) {
    const int k = ((b % bins) + bins) % bins;
    ++total;
    occluded += horizon_[static_cast<std::size_t>(k)] < near;
  }
  return total > 0 ? static_cast<double>(occluded) / total : 0.0;
}

OcclusionReason MultiObjectTracker::explainMiss(
  const Track & t, const Eigen::Vector2d & pos_s, const std::vector<ObstacleCandidate> & dets,
  const std::vector<Eigen::Vector2d> & /*det_centres_s*/) const
{
  const double r = pos_s.norm();
  const double half = 0.5 * std::max(t.size.x(), t.size.y()) + 0.1;
  if (r > config_.max_range - half) {
    return OcclusionReason::kOutOfRange;
  }
  // predicted box inside another object's cluster (merged with it)
  for (const auto & d : dets) {
    const Eigen::Vector2d c(d.obb.center.x(), d.obb.center.y());
    const Eigen::Vector2d l = rotation(d.obb.yaw).transpose() * (pos_s - c);
    if (std::fabs(l.x()) <= 0.5 * d.obb.size.x() + 0.2 && std::fabs(l.y()) <= 0.5 * d.obb.size.y() + 0.2) {
      return OcclusionReason::kMerged;
    }
  }
  if (occludedFraction(pos_s, t.size, t.yaw - last_yaw_) >= 0.5) {
    return OcclusionReason::kOccluder;
  }
  return OcclusionReason::kUnexplained;
}

float MultiObjectTracker::trackConfidence(const Track & t, double stamp) const
{
  // Heuristic existence confidence: evidence (hits), recency of a measurement and position
  // uncertainty. Not a calibrated probability.
  if (t.state == TrackState::kLost) {
    return 0.F;
  }
  const double c_hits = 1.0 - std::exp(-static_cast<double>(t.hits) / 3.0);
  const double c_gap = std::exp(-(stamp - t.last_update) / 1.0);
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(t.kf.positionCov());
  const double sigma = std::sqrt(std::max(es.eigenvalues().maxCoeff(), 0.0));
  const double c_sigma = std::min(1.0, 0.3 / std::max(sigma, 1e-3));
  const double c_state = t.state == TrackState::kTentative ? 0.5 : 1.0;
  return static_cast<float>(std::clamp(c_hits * c_gap * std::sqrt(c_sigma) * c_state, 0.0, 1.0));
}

void MultiObjectTracker::update(
  double stamp, const std::vector<ObstacleCandidate> & dets, const EgoMotion & ego,
  std::vector<TrackOutput> & out, const Cloud * cloud)
{
  using Clock = std::chrono::steady_clock;
  auto ms = [](Clock::time_point a, Clock::time_point b) {
      return std::chrono::duration<double, std::milli>(b - a).count();
    };
  const auto c0 = Clock::now();
  out.clear();
  double dt = last_stamp_ < 0.0 ? 0.0 : stamp - last_stamp_;
  if (last_stamp_ >= 0.0 && (dt <= 0.0 || dt > config_.max_dt)) {
    reset();  // time jump (bag loop, sensor restart) or long data gap
    dt = 0.0;
  }
  last_stamp_ = stamp;

  const Eigen::Matrix2d rot = rotation(ego.pose[2]);
  const Eigen::Matrix2d rot_t = rot.transpose();
  const Eigen::Vector2d trans(ego.pose[0], ego.pose[1]);
  const bool ego_usable = ego.valid && ego.state != EgoState::kUnknown;

  // ---- predict (all tracks, LOST ones too: re-identification gate) ---------------------------
  for (auto & t : tracks_) {
    if ((t.state == TrackState::kOccluded || t.state == TrackState::kLost) && !isDynamic(t)) {
      // static motion model while unobserved: the KF velocity of a static / never-moving object
      // only holds measurement artefacts and must not carry the prediction away
      t.kf.resetVelocity(Eigen::Vector2d::Zero(), 0.1);
    }
    t.kf.predict(dt, config_.process_accel_sigma);
    t.detection_index = -1;
    t.merged = false;
  }
  last_yaw_ = ego.pose[2];
  const auto c1 = Clock::now();

  // ---- measurements in the odometry frame ----------------------------------------------------
  const std::size_t nd = dets.size();
  std::vector<Eigen::Vector2d> z(nd), c_s(nd);
  std::vector<Eigen::Matrix2d> R(nd);
  std::vector<double> yaw_odom(nd);
  for (std::size_t j = 0; j < nd; ++j) {
    c_s[j] = Eigen::Vector2d(dets[j].obb.center.x(), dets[j].obb.center.y());
    z[j] = rot * c_s[j] + trans;
    R[j] = measurementCov(dets[j], ego, rot);
    yaw_odom[j] = dets[j].obb.yaw + ego.pose[2];
  }

  const std::size_t nt = tracks_.size();
  std::vector<int> assign(nt, -1);                 // detection index per track
  std::vector<Measurement> meas(nt);
  std::vector<char> det_used(nd, 0);               // assigned or consumed by a merge split
  std::vector<char> from_split(nt, 0);

  // ---- gated global association of the active tracks -----------------------------------------
  const auto c2 = Clock::now();
  std::vector<std::size_t> rows;
  for (std::size_t i = 0; i < nt; ++i) {
    if (assign[i] < 0 && tracks_[i].state != TrackState::kLost) {
      rows.push_back(i);
    }
  }
  std::vector<std::size_t> cols;
  for (std::size_t j = 0; j < nd; ++j) {
    if (!det_used[j]) {
      cols.push_back(j);
    }
  }
  const std::size_t nr = rows.size(), nc = cols.size();
  std::vector<double> cost(nr * nc, kInfeasible);
  const double gate_d2 = static_cast<double>(config_.max_gate_distance) * config_.max_gate_distance;
  for (std::size_t r = 0; r < nr; ++r) {
    const Track & t = tracks_[rows[r]];
    for (std::size_t c = 0; c < nc; ++c) {
      const std::size_t j = cols[c];
      if ((z[j] - t.kf.position()).squaredNorm() >
        (config_.max_gate_distance + 0.5 * t.size.head<2>().norm()) *
        (config_.max_gate_distance + 0.5 * t.size.head<2>().norm()))
      {
        continue;
      }
      const Measurement m = measure(t, dets[j], z[j], R[j], yaw_odom[j]);
      Eigen::Vector2d y;
      Eigen::Matrix2d S;
      t.kf.innovation(m.z, m.R, y, S);
      if (y.squaredNorm() > gate_d2) {
        continue;
      }
      const double d2 = y.dot(S.ldlt().solve(y));
      if (d2 > config_.gate_chi2) {
        continue;
      }
      double cst = d2 + std::log(std::max(S.determinant(), 1e-12));
      cst += sizeCost(t, dets[j], yaw_odom[j]);
      cst += iouCost(t, dets[j], z[j], yaw_odom[j]);
      if (dets[j].background && t.ever_moved) {
        cst += config_.background_cost;   // a mover does not become part of the background
      }
      cost[r * nc + c] = cst;
    }
  }
  const std::vector<int> a_main = solveAssignment(cost, nr, nc, kInfeasible);
  for (std::size_t r = 0; r < nr; ++r) {
    if (a_main[r] >= 0) {
      const std::size_t i = rows[r], j = cols[static_cast<std::size_t>(a_main[r])];
      assign[i] = static_cast<int>(j);
      det_used[j] = 1;
      meas[i] = measure(tracks_[i], dets[j], z[j], R[j], yaw_odom[j]);
    }
  }
  const auto c3 = Clock::now();

  // ---- merge split (only where needed): an established track left without a detection whose
  // predicted box lies inside a cluster that another track took (or nobody took) -> that cluster
  // contains several objects; its points are split between the tracks by their predicted boxes.
  std::size_t merge_count = 0;
  if (config_.merge_split && cloud != nullptr) {
    std::vector<int> owner(nd, -1);
    for (std::size_t i = 0; i < nt; ++i) {
      if (assign[i] >= 0) {
        owner[static_cast<std::size_t>(assign[i])] = static_cast<int>(i);
      }
    }
    auto inside = [&](const Track & t, std::size_t j, double margin) {
        const Eigen::Vector2d ps = rot_t * (t.kf.position() - trans);
        const Eigen::Vector2d l = rotation(dets[j].obb.yaw).transpose() * (ps - c_s[j]);
        return std::fabs(l.x()) <= 0.5 * dets[j].obb.size.x() + margin &&
               std::fabs(l.y()) <= 0.5 * dets[j].obb.size.y() + margin;
      };
    auto larger = [&](const Track & t, std::size_t j) {
        const double d_area = static_cast<double>(dets[j].obb.size.x()) * dets[j].obb.size.y();
        const double t_area = static_cast<double>(t.size.x()) * t.size.y();
        return d_area > 1.3 * t_area ||
               std::max(dets[j].obb.size.x(), dets[j].obb.size.y()) >
               std::max(t.size.x(), t.size.y()) + 0.2;
      };
    auto established = [&](const Track & t) {
        return !t.large_structure &&
               (t.state == TrackState::kConfirmed || t.state == TrackState::kOccluded) &&
               !(t.merged_since >= 0.0 && stamp - t.merged_since > config_.merge_max_time_s);
      };
    for (std::size_t j = 0; j < nd; ++j) {
      const auto & d = dets[j];
      if (d.large_structure || d.indices == nullptr || d.indices->empty()) {
        continue;
      }
      // unassigned established tracks whose predicted box lies inside this cluster
      std::vector<std::pair<double, std::size_t>> cands;
      for (std::size_t i = 0; i < nt; ++i) {
        const Track & t = tracks_[i];
        if (assign[i] >= 0 || !established(t) || !inside(t, j, 0.25) || !larger(t, j)) {
          continue;
        }
        cands.emplace_back((rot_t * (t.kf.position() - trans) - c_s[j]).squaredNorm(), i);
      }
      if (cands.empty()) {
        continue;
      }
      std::sort(cands.begin(), cands.end());
      std::vector<std::size_t> sel;
      if (owner[j] >= 0) {
        const auto k = static_cast<std::size_t>(owner[j]);
        if (!established(tracks_[k]) || !inside(tracks_[k], j, 0.25)) {
          continue;
        }
        sel.push_back(k);
      }
      for (const auto & c : cands) {
        bool distinct = true;
        for (const auto k : sel) {
          distinct = distinct &&
            (tracks_[k].kf.position() - tracks_[c.second].kf.position()).norm() >= 0.35;
        }
        if (distinct) {
          sel.push_back(c.second);
        }
        if (static_cast<int>(sel.size()) >= config_.merge_max_tracks) {
          break;
        }
      }
      bool any_dynamic = false;
      for (const auto k : sel) {
        any_dynamic = any_dynamic || isDynamic(tracks_[k]);
      }
      if (sel.size() < 2 || !any_dynamic) {
        continue;
      }
      // partition the points by the nearest predicted box
      const std::size_t ns = sel.size();
      std::vector<double> a0(ns, 1e9), a1(ns, -1e9), b0(ns, 1e9), b1(ns, -1e9);
      std::vector<int> cnt(ns, 0), in_box(ns, 0);
      std::vector<Eigen::Matrix2d> Ut(ns);
      for (std::size_t q = 0; q < ns; ++q) {
        Ut[q] = rotation(tracks_[sel[q]].yaw).transpose();
      }
      for (const auto idx : *d.indices) {
        const auto & p = (*cloud)[static_cast<std::size_t>(idx)];
        const Eigen::Vector2d po = rot * Eigen::Vector2d(p.x, p.y) + trans;
        std::size_t best = 0;
        double bd = std::numeric_limits<double>::infinity();
        Eigen::Vector2d bl = Eigen::Vector2d::Zero();
        for (std::size_t q = 0; q < ns; ++q) {
          const Track & t = tracks_[sel[q]];
          const Eigen::Vector2d l = Ut[q] * (po - t.kf.position());
          const double ox = std::max(std::fabs(l.x()) - 0.5 * t.size.x(), 0.0);
          const double oy = std::max(std::fabs(l.y()) - 0.5 * t.size.y(), 0.0);
          const double dist = std::hypot(ox, oy) + 0.05 * l.norm();
          if (dist < bd) {
            bd = dist;
            best = q;
            bl = l;
          }
        }
        a0[best] = std::min(a0[best], bl.x());
        a1[best] = std::max(a1[best], bl.x());
        b0[best] = std::min(b0[best], bl.y());
        b1[best] = std::max(b1[best], bl.y());
        ++cnt[best];
        in_box[best] += bd - 0.05 * bl.norm() <= 0.1;   // inside its own predicted box
      }
      // a merge needs at least two objects with points where they are predicted; a track hidden
      // BEHIND the cluster (no points in its box) is occluded, not merged
      int present = 0;
      for (std::size_t q = 0; q < ns; ++q) {
        present += in_box[q] >= 3;
      }
      if (present < 2) {
        continue;
      }
      const double sigma = config_.meas_sigma_base + config_.meas_sigma_per_meter * c_s[j].norm();
      Eigen::Matrix2d R_extra = (sigma * sigma + 0.1 * 0.1) * Eigen::Matrix2d::Identity();
      R_extra += rot * ego.delta_cov.topLeftCorner<2, 2>() * rot.transpose();
      for (std::size_t q = 0; q < ns; ++q) {
        const auto i = sel[q];
        Track & t = tracks_[i];
        t.merged = true;
        if (t.merged_since < 0.0) {
          t.merged_since = stamp;
        }
        if (cnt[q] < 3 || in_box[q] < 3) {
          assign[i] = -1;   // fully hidden inside / behind the other object: no measurement
          from_split[i] = 0;
          continue;
        }
        meas[i] = partialView(t, a0[q], a1[q], b0[q], b1[q], R_extra);
        meas[i].partial = true;
        assign[i] = static_cast<int>(j);
        from_split[i] = 1;
      }
      det_used[j] = 1;
      ++merge_count;
    }
  }
  for (auto & t : tracks_) {
    if (!t.merged) {
      t.merged_since = -1.0;
    }
  }

  // ---- re-identification of LOST tracks with the remaining detections ------------------------
  std::size_t reid = 0;
  {
    std::vector<std::size_t> lost_rows;
    for (std::size_t i = 0; i < nt; ++i) {
      if (tracks_[i].state == TrackState::kLost) {
        lost_rows.push_back(i);
      }
    }
    std::vector<std::size_t> free_cols;
    for (std::size_t j = 0; j < nd; ++j) {
      if (!det_used[j] && !dets[j].large_structure) {
        free_cols.push_back(j);
      }
    }
    if (!lost_rows.empty() && !free_cols.empty()) {
      const std::size_t lr = lost_rows.size(), lc = free_cols.size();
      std::vector<double> rc(lr * lc, kInfeasible);
      for (std::size_t r = 0; r < lr; ++r) {
        const Track & t = tracks_[lost_rows[r]];
        const double gap = stamp - t.last_update;
        const double gate = std::min(static_cast<double>(config_.reid_max_distance),
            0.75 + config_.reid_speed * gap);
        for (std::size_t c = 0; c < lc; ++c) {
          const std::size_t j = free_cols[c];
          const double dist = (z[j] - t.kf.position()).norm();
          if (dist > gate || (dets[j].background && t.ever_moved)) {
            continue;
          }
          const double sc = sizeCost(t, dets[j], yaw_odom[j]);
          if (sc > 9.0 * std::max(config_.size_cost_weight, 1e-3F)) {
            continue;
          }
          rc[r * lc + c] = 4.0 * (dist / gate) * (dist / gate) + sc;
        }
      }
      const std::vector<int> a_reid = solveAssignment(rc, lr, lc, kInfeasible);
      for (std::size_t r = 0; r < lr; ++r) {
        if (a_reid[r] >= 0) {
          const std::size_t i = lost_rows[r], j = free_cols[static_cast<std::size_t>(a_reid[r])];
          assign[i] = static_cast<int>(j);
          det_used[j] = 1;
          meas[i] = measure(tracks_[i], dets[j], z[j], R[j], yaw_odom[j]);
          ++reid;
        }
      }
    }
  }
  const auto c4 = Clock::now();

  // ---- update / lifecycle -------------------------------------------------------------------
  if (cloud != nullptr) {
    computeOcclusionHorizon(*cloud);
  } else {
    horizon_.clear();
  }
  double nis_sum = 0.0;
  std::size_t associated = 0;
  for (std::size_t i = 0; i < nt; ++i) {
    Track & t = tracks_[i];
    const int j = assign[i];
    if (j >= 0) {
      const auto ju = static_cast<std::size_t>(j);
      const auto & d = dets[ju];
      const double meas_yaw = yaw_odom[ju];
      Measurement & m = meas[i];
      const bool was_lost = t.state == TrackState::kLost;
      double shape_sigma = from_split[i] ? 0.0 : shapeInconsistency(t, d, meas_yaw);
      // a box that shrank behind a nearer object is a partial view, not a new shape
      const bool occluded_view = (m.partial || m.shrunk) &&
        occludedFraction(rot_t * (t.kf.position() - trans), t.size, t.yaw - ego.pose[2]) >= 0.2;
      if (from_split[i] || occluded_view) {
        // keep the streak: neither confirms nor refutes the current shape
      } else {
        t.shape_streak = shape_sigma > 0.0 ? t.shape_streak + 1 : 0;
      }
      const bool shape_reset = t.shape_streak >= config_.shape_reset_frames;
      if (shape_reset) {
        // the object really looks different now (merge ended, new aspect): adopt the new shape
        // instead of treating every following measurement as a partial view
        shape_sigma = 0.0;
        t.shape_streak = 0;
        t.motion_samples.clear();   // centres before / after the change are not comparable
        m.z = z[ju];
        m.R = R[ju];
        m.partial = false;
      }
      // the partial-view model already accounts for shrunken boxes; growth (merge / new aspect)
      // still inflates the noise of the box centre
      Eigen::Matrix2d Rj = m.R;
      if (!m.partial) {
        Rj += shape_sigma * shape_sigma * Eigen::Matrix2d::Identity();
      }
      nis_sum += t.kf.mahalanobis2(m.z, Rj);
      ++associated;
      t.kf.update(m.z, Rj);
      t.last_meas_sigma = m.max_sigma;
      t.max_displacement = std::max(t.max_displacement, (t.kf.position() - t.origin).norm());
      // motion evidence: consistent boxes count fully; partial / shape-changed ones only with a
      // still sensor (while driving the changing viewpoint alone changes the boxes)
      // partial views count as consistent only with a still sensor (viewpoint changes otherwise)
      const bool consistent = shape_sigma <= 0.0 &&
        (!m.partial || (m.max_sigma < 0.15 && ego.state == EgoState::kStationary));
      const double w_bad = ego.state == EgoState::kStationary ?
        config_.inconsistent_sample_weight : 0.0;
      // a raw centre of a box shrunk behind an occluder is biased: no motion evidence
      const bool biased = occluded_view && !m.partial;
      if (consistent || (w_bad > 0.0 && !biased)) {
        t.motion_samples.push_back({stamp, m.partial ? m.z : z[ju], consistent ? 1.0 : w_bad});
      }
      if (cloud != nullptr && !from_split[i]) {
        updateSurfaceEvidence(t, d, *cloud, stamp, rot, trans);
      } else {
        t.surface_speed = -1.F;
      }
      det_used[ju] = 1;
      ++t.hits;
      t.misses = 0;
      t.last_update = stamp;
      t.detection_index = j;
      if (!from_split[i]) {
        // a shared (merged) cluster says nothing about this track's own background membership
        t.large_structure = d.large_structure;
        t.background = d.background;
        t.point_count = static_cast<std::uint32_t>(d.point_count);
      }
      t.occlusion = OcclusionReason::kNone;
      t.unexplained_s = 0.0;
      t.lost_since = -1.0;
      if (shape_reset) {
        updateShape(t, d, meas_yaw, true);
      } else if (shape_sigma <= 0.0 && !m.partial && !from_split[i]) {
        updateShape(t, d, meas_yaw);   // keep the shape while partially occluded / merged
      }
      if (!from_split[i]) {
        const Eigen::Vector2d cp =
          rot * Eigen::Vector2d(d.closest_point.x(), d.closest_point.y()) + trans;
        t.closest_offset = cp - t.kf.position();
        t.closest_z = d.closest_point.z();
      }
      if (t.state == TrackState::kTentative &&
        t.hits >= static_cast<std::uint32_t>(config_.confirm_hits))
      {
        t.state = TrackState::kConfirmed;
      } else if (t.state == TrackState::kOccluded || was_lost) {
        t.state = TrackState::kConfirmed;
      }
    } else {
      ++t.misses;
      t.surface_speed = -1.F;
      if (t.state == TrackState::kConfirmed || t.state == TrackState::kOccluded) {
        t.state = TrackState::kOccluded;
        const Eigen::Vector2d ps = rot_t * (t.kf.position() - trans);
        t.occlusion = explainMiss(t, ps, dets, c_s);
        if (t.occlusion == OcclusionReason::kUnexplained) {
          t.unexplained_s += dt;
        }
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(t.kf.positionCov());
        const double sigma = std::sqrt(std::max(es.eigenvalues().maxCoeff(), 0.0));
        const double max_occ = isDynamic(t) ? config_.occluded_max_time_s :
          config_.max_coast_time_s;
        if (t.unexplained_s > config_.max_coast_time_s || stamp - t.last_update > max_occ ||
          sigma > config_.max_position_sigma)
        {
          t.state = TrackState::kLost;
          t.lost_since = stamp;
        }
      }
    }
  }
  stats_.associated = associated;
  stats_.mean_nis = associated ? nis_sum / static_cast<double>(associated) : 0.0;
  stats_.reidentified_total += reid;
  stats_.merge_splits_total += merge_count;

  // ---- delete -------------------------------------------------------------------------------
  const auto before = tracks_.size();
  tracks_.erase(
    std::remove_if(
      tracks_.begin(), tracks_.end(), [&](const Track & t) {
        if (t.state == TrackState::kTentative) {
          return t.misses > static_cast<std::uint32_t>(config_.tentative_max_misses);
        }
        if (t.state == TrackState::kLost) {
          return stamp - t.lost_since > config_.lost_max_time_s;
        }
        return !t.kf.x().allFinite();
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
    t.origin = z[j];
    t.first_stamp = stamp;
    t.last_update = stamp;
    t.hits = 1;
    t.detection_index = static_cast<int>(j);
    t.large_structure = dets[j].large_structure;
    t.background = dets[j].background;
    t.point_count = static_cast<std::uint32_t>(dets[j].point_count);
    updateShape(t, dets[j], yaw_odom[j]);
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
  const auto c5 = Clock::now();

  // ---- history, motion classification, output -------------------------------------------------
  const Eigen::Vector2d ego_v = ego.velocity;
  const Eigen::Matrix2d ego_vcov = ego.velocity_cov.topLeftCorner<2, 2>();
  const double ego_w = ego.yaw_rate;
  const double ego_w_var = ego.velocity_cov(2, 2);
  stats_.tracks = tracks_.size();
  stats_.confirmed = stats_.moving = stats_.static_tracks = stats_.occluded = stats_.lost = 0;
  out.reserve(tracks_.size());

  for (auto & t : tracks_) {
    if (t.state == TrackState::kLost) {
      ++stats_.lost;
      continue;   // kept for re-identification only, not an output object
    }
    t.history.emplace_back(stamp, t.kf.position());
    while (t.history.size() > static_cast<std::size_t>(config_.history_length)) {
      t.history.pop_front();
    }
    // lifecycle and motion state are independent: no measurement -> no new motion decision
    if (t.detection_index >= 0 || !ego_usable) {
      classifyMotion(t, stamp, ego_usable, ego.state == EgoState::kStationary,
        ego.state == EgoState::kUnreliable);
    }

    if (t.motion == MotionState::kMoving) {
      t.last_moving = stamp;
    }
    TrackOutput o;
    o.id = t.id;
    o.state = t.state;
    o.motion = t.motion;
    o.occlusion = t.occlusion;
    o.predicted = t.detection_index < 0;
    o.merged = t.merged;
    o.relative_only = !ego_usable;
    o.ever_moved = t.ever_moved;
    if (t.last_moving >= 0.0) {
      o.time_since_moving_s = static_cast<float>(stamp - t.last_moving);
    }
    o.max_displacement = static_cast<float>(t.max_displacement);
    o.large_structure = t.large_structure;
    o.updated = t.detection_index >= 0;
    o.detection_index = t.detection_index;
    o.age_s = static_cast<float>(stamp - t.first_stamp);
    o.hits = t.hits;
    o.misses = t.misses;
    o.time_since_update_s = static_cast<float>(stamp - t.last_update);
    o.point_count = t.point_count;
    o.confidence = trackConfidence(t, stamp);

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

    // velocity trust: UNKNOWN without a ground reference or estimate, UNRELIABLE when predicted,
    // merged, too uncertain or the ego-motion estimate itself is unreliable
    if (!ego_usable || t.large_structure || t.state == TrackState::kTentative ||
      o.age_s < config_.min_age_s)
    {
      o.velocity_status = VelocityStatus::kUnknown;
    } else if (o.predicted || t.merged || ego.state == EgoState::kUnreliable ||
      o.speed_sigma > config_.max_velocity_sigma)
    {
      o.velocity_status = VelocityStatus::kUnreliable;
    } else {
      o.velocity_status = VelocityStatus::kReliable;
    }
    o.velocity_reliable = o.velocity_status == VelocityStatus::kReliable;

    o.window_speed = t.window_speed;
    o.window_speed_sigma = t.window_speed_sigma;
    o.surface_speed = t.surface_speed;
    o.background = t.background;
    o.heading = static_cast<float>(std::atan2(v_s.y(), v_s.x()));
    o.heading_sigma = o.speed > 1e-3F ?
      static_cast<float>(std::sqrt(std::max(un.dot(Pv_s * un), 0.0)) / o.speed) :
      static_cast<float>(M_PI);
    o.heading_valid = o.velocity_reliable && t.motion == MotionState::kMoving;
    // Heading of MOVING tracks from the regression of the motion window: much steadier than the
    // instantaneous KF velocity (leg swing, partial views).
    if (o.heading_valid && t.window_valid && t.window_speed > config_.static_speed) {
      const Eigen::Vector2d wv = rot_t * t.window_velocity;
      o.heading = static_cast<float>(std::atan2(wv.y(), wv.x()));
      o.heading_sigma = std::min(static_cast<float>(M_PI),
          t.window_speed_sigma / std::max(t.window_speed, 1e-3F));
    }
    // motion evidence strength (heuristic): significance of the window regression
    {
      const double zsig = t.window_valid && t.window_speed_sigma > 0.F ?
        t.window_speed / t.window_speed_sigma : 0.0;
      double mc = 0.0;
      if (t.motion == MotionState::kMoving) {
        mc = std::clamp(zsig / (2.0 * config_.motion_sigma_factor), 0.0, 1.0);
      } else if (t.motion == MotionState::kStatic) {
        mc = t.background ? 1.0 :
          1.0 - std::clamp(static_cast<double>(t.window_speed) / config_.moving_speed, 0.0, 1.0);
      }
      if (o.predicted) {
        mc *= std::exp(-(stamp - t.last_update));
      }
      o.motion_confidence = static_cast<float>(mc);
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
    if (t.state == TrackState::kOccluded) {
      ++stats_.occluded;
    }
    if (t.motion == MotionState::kMoving) {
      ++stats_.moving;
    } else if (t.motion == MotionState::kStatic) {
      ++stats_.static_tracks;
    }
    out.push_back(std::move(o));
  }
  const auto c6 = Clock::now();
  stats_.predict_ms = ms(c0, c1);
  stats_.association_ms = ms(c2, c3);
  stats_.occlusion_ms = ms(c1, c2) + ms(c3, c4);
  stats_.update_ms = ms(c4, c5) + ms(c5, c6);
}

}  // namespace helios_obstacle_detection
