#include "helios_obstacle_detection/collision_warning.hpp"

#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace helios_obstacle_detection
{

const char * toString(WarningLevel w) noexcept
{
  switch (w) {
    case WarningLevel::kClear: return "CLEAR";
    case WarningLevel::kCaution: return "CAUTION";
    case WarningLevel::kWarning: return "WARNING";
  }
  return "?";
}

void WarningConfig::validate() const
{
  if (!(warning_ttc_s > 0.F && caution_ttc_s >= warning_ttc_s && near_distance >= 0.F &&
    min_object_displacement >= 0.F &&
    min_closing_speed >= 0.F && raise_frames >= 1 && hold_s >= 0.F))
  {
    throw std::invalid_argument(
            "warning: need 0 < warning_ttc_s <= caution_ttc_s, raise_frames >= 1, hold_s >= 0");
  }
}

CollisionWarningFilter::CollisionWarningFilter(const WarningConfig & config, const RiskConfig & risk)
: config_(config), risk_(risk)
{
  config_.validate();
  risk_.validate();
}

void CollisionWarningFilter::reset()
{
  shown_ = pending_ = WarningLevel::kClear;
  pending_count_ = 0;
  last_raise_stamp_ = -1.0;
  last_threat_ = CollisionWarning{};
}

WarningLevel CollisionWarningFilter::classify(
  const TrackOutput & t, const RiskAssessment & r, bool & collision, RiskAssessment & used) const
{
  collision = false;
  used = r;
  if (t.state == TrackState::kTentative || t.state == TrackState::kLost || !r.ttc_valid ||
    !t.velocity_reliable)
  {
    return WarningLevel::kClear;   // no trustworthy motion: no prediction claims
  }
  if (t.motion == MotionState::kMoving) {
    if (t.max_displacement < config_.min_object_displacement) {
      return WarningLevel::kClear;   // has not really gone anywhere yet
    }
  } else if (t.speed > 1e-3F || t.velocity.norm() > 1e-3F) {
    // not MOVING: own velocity = 0, only the ego motion remains in the relative velocity
    TrackOutput still = t;
    still.relative_velocity = t.relative_velocity - t.velocity;
    still.relative_velocity_cov = t.relative_velocity_cov - t.velocity_cov +
      0.05F * 0.05F * Eigen::Matrix2f::Identity();
    still.velocity.setZero();
    used = assessRisk(still, risk_);
  }
  if (!(used.closing_speed > config_.min_closing_speed)) {
    return WarningLevel::kClear;   // not coming towards the sensor
  }
  collision = used.collision_course && std::isfinite(used.ttc_pessimistic);
  if (collision && std::isfinite(used.ttc) && used.ttc <= config_.warning_ttc_s) {
    return WarningLevel::kWarning;   // expected to hit within warning_ttc_s
  }
  if (collision && (used.ttc_pessimistic <= config_.warning_ttc_s ||
    (std::isfinite(used.ttc) && used.ttc <= config_.caution_ttc_s)))
  {
    return WarningLevel::kCaution;   // possible collision
  }
  const bool dynamic = t.motion == MotionState::kMoving || t.ever_moved;
  if (dynamic && used.clearance <= config_.near_distance) {
    return WarningLevel::kCaution;
  }
  return WarningLevel::kClear;
}

CollisionWarning CollisionWarningFilter::update(
  double stamp, const std::vector<TrackOutput> & tracks, const std::vector<RiskAssessment> & risks)
{
  // most severe instantaneous threat (ties: shortest pessimistic TTC, then nearest)
  CollisionWarning best;
  for (std::size_t i = 0; i < tracks.size() && i < risks.size(); ++i) {
    bool collision = false;
    RiskAssessment r;
    const WarningLevel lvl = classify(tracks[i], risks[i], collision, r);
    if (lvl == WarningLevel::kClear) {
      continue;
    }
    const bool better = best.track_index < 0 || lvl > best.level ||
      (lvl == best.level && (r.ttc_pessimistic < best.ttc_pessimistic ||
      (r.ttc_pessimistic == best.ttc_pessimistic && r.clearance < best.clearance)));
    if (better) {
      best.level = lvl;
      best.collision_predicted = collision;
      best.track_index = static_cast<int>(i);
      best.track_id = tracks[i].id;
      best.distance = tracks[i].closest_distance;
      best.clearance = r.clearance;
      best.closing_speed = r.closing_speed;
      best.closing_speed_sigma = r.closing_speed_sigma;
      best.ttc = r.ttc;
      best.ttc_pessimistic = r.ttc_pessimistic;
      best.cpa_time = r.cpa_time;
      best.cpa_distance = r.cpa_distance;
    }
  }
  if (!config_.enabled) {
    best = CollisionWarning{};
  }

  // hysteresis: raise after raise_frames consecutive scans, hold for hold_s
  const WarningLevel inst = best.level;
  if (inst > shown_) {
    pending_count_ = (pending_ == inst) ? pending_count_ + 1 : 1;
    pending_ = inst;
    if (pending_count_ >= config_.raise_frames) {
      shown_ = inst;
      pending_count_ = 0;
    }
  } else {
    pending_count_ = 0;
    pending_ = inst;
  }
  if (inst >= shown_ && inst != WarningLevel::kClear) {
    last_raise_stamp_ = stamp;
  } else if (shown_ != WarningLevel::kClear &&
    (last_raise_stamp_ < 0.0 || stamp - last_raise_stamp_ > config_.hold_s))
  {
    shown_ = inst;   // condition gone for hold_s: step down
    if (inst != WarningLevel::kClear) {
      last_raise_stamp_ = stamp;
    }
  }

  CollisionWarning out = best;
  if (shown_ != WarningLevel::kClear && best.level < shown_) {
    // held level: report the threat that raised it (positions may be stale; flagged in text)
    out = last_threat_;
    out.track_index = -1;
    for (std::size_t i = 0; i < tracks.size(); ++i) {
      if (tracks[i].id == out.track_id) {
        out.track_index = static_cast<int>(i);
      }
    }
  } else if (best.level == shown_ && shown_ != WarningLevel::kClear) {
    last_threat_ = best;
  }
  out.level = shown_;

  char buf[200];
  if (out.level == WarningLevel::kClear) {
    std::snprintf(buf, sizeof(buf), "CARPISMA YOK");
  } else {
    const char * head = out.level == WarningLevel::kWarning ? "!! CARPISMA RISKI VAR !!" :
      "DIKKAT: YAKLASAN NESNE";
    if (std::isfinite(out.ttc) && std::isfinite(out.ttc_pessimistic)) {
      std::snprintf(buf, sizeof(buf), "%s  ID%u  TTC %.1fs (en kotu %.1fs)  %.1fm  %.1fm/s%s",
        head, out.track_id, out.ttc, out.ttc_pessimistic, out.distance, out.closing_speed,
        out.collision_predicted ? "  rota: CARPISMA" : "  rota: yakin gecis");
    } else {
      std::snprintf(buf, sizeof(buf), "%s  ID%u  %.1fm  %.1fm/s", head, out.track_id,
        out.distance, out.closing_speed);
    }
  }
  out.text = buf;
  return out;
}

}  // namespace helios_obstacle_detection
