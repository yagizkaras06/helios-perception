#include "helios_obstacle_detection/sensor_health.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace helios_obstacle_detection
{

float fastAtan2(float y, float x) noexcept
{
  constexpr float kPi = 3.14159265358979F;
  constexpr float kPi2 = kPi / 2.F;
  const float ax = std::fabs(x), ay = std::fabs(y);
  if (ax < 1e-12F && ay < 1e-12F) {
    return 0.F;
  }
  const float a = std::min(ax, ay) / std::max(ax, ay);
  const float s = a * a;
  float r = ((-0.0464964749F * s + 0.15931422F) * s - 0.327622764F) * s * a + a;
  if (ay > ax) {
    r = kPi2 - r;
  }
  if (x < 0.F) {
    r = kPi - r;
  }
  if (y < 0.F) {
    r = -r;
  }
  return r;
}

void SensorHealthConfig::validate() const
{
  if (!(expected_rate_hz > 0.0 && gap_factor > 1.0 && min_valid_ratio >= 0.0 &&
    min_valid_ratio <= 1.0 && min_azimuth_coverage >= 0.0 && min_azimuth_coverage <= 1.0 &&
    point_drop_ratio >= 0.0 && point_drop_ratio < 1.0 && clock_sync_tolerance_s > 0.0 &&
    event_memory_s >= 0.0 && window >= 10))
  {
    throw std::invalid_argument("health: parameters out of range");
  }
}

SensorHealthMonitor::SensorHealthMonitor(const SensorHealthConfig & config)
: config_(config)
{
  config_.validate();
}

void SensorHealthMonitor::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  counters_ = HealthSnapshot{};
  stamps_.clear();
  offsets_.clear();
  last_stamp_ = -1.0;
  mean_points_ = 0.0;
  last_event_wall_ = -1e18;
  last_event_.clear();
}

FrameHealth SensorHealthMonitor::onFrame(double stamp, double rx_wall, const ScanQuality & q)
{
  std::lock_guard<std::mutex> lock(mutex_);
  FrameHealth fh;
  ++counters_.frames;
  const double period = 1.0 / config_.expected_rate_hz;
  auto event = [&](const std::string & what) {
      last_event_wall_ = rx_wall;
      last_event_ = what;
    };

  if (last_stamp_ >= 0.0) {
    const double dt = stamp - last_stamp_;
    if (dt < 0.0) {
      fh.time_jump = true;
      ++counters_.time_jumps;
      stamps_.clear();
      offsets_.clear();
      event("time jump backwards");
    } else if (dt == 0.0) {
      ++counters_.duplicate_stamps;
      event("duplicate stamp");
    } else if (dt > config_.gap_factor * period) {
      fh.gap = true;
      fh.missed_frames = std::max(1, static_cast<int>(std::lround(dt / period)) - 1);
      ++counters_.gaps;
      counters_.missed_frames += static_cast<std::uint64_t>(fh.missed_frames);
      std::ostringstream os;
      os << "frame gap " << static_cast<int>(dt * 1000.0) << " ms (~" << fh.missed_frames
         << " scans lost)";
      event(os.str());
    }
  }
  last_stamp_ = stamp;

  stamps_.push_back(stamp);
  offsets_.push_back(rx_wall - stamp);
  while (stamps_.size() > config_.window) {
    stamps_.pop_front();
  }
  while (offsets_.size() > config_.window) {
    offsets_.pop_front();
  }

  counters_.points = q.finite_points;
  counters_.valid_ratio = q.total_points ?
    static_cast<double>(q.finite_points) / static_cast<double>(q.total_points) : 0.0;
  counters_.azimuth_coverage = q.azimuth_sectors_total ?
    static_cast<double>(q.azimuth_sectors_covered) / q.azimuth_sectors_total : 0.0;
  if (counters_.valid_ratio < config_.min_valid_ratio) {
    ++counters_.low_valid_scans;
    event("low valid-return ratio (blocked / dirty window?)");
  }
  if (counters_.azimuth_coverage < config_.min_azimuth_coverage) {
    ++counters_.partial_scans;
    event("partial scan (azimuth coverage low: packet loss / occlusion)");
  }
  if (mean_points_ > 0.0 &&
    static_cast<double>(q.finite_points) < config_.point_drop_ratio * mean_points_)
  {
    ++counters_.point_drops;
    event("sudden drop of returns");
  }
  mean_points_ = mean_points_ <= 0.0 ? static_cast<double>(q.finite_points) :
    0.95 * mean_points_ + 0.05 * static_cast<double>(q.finite_points);
  return fh;
}

HealthSnapshot SensorHealthMonitor::snapshot(double now_wall) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  HealthSnapshot s = counters_;
  if (stamps_.size() >= 2) {
    const double span = stamps_.back() - stamps_.front();
    s.stamp_rate_hz = span > 0.0 ? static_cast<double>(stamps_.size() - 1) / span : 0.0;
  }
  if (!offsets_.empty()) {
    std::vector<double> o(offsets_.begin(), offsets_.end());
    std::sort(o.begin(), o.end());
    const double median = o[o.size() / 2];
    const double mn = o.front();
    const double p95 = o[std::min(o.size() - 1, static_cast<std::size_t>(0.95 * o.size()))];
    s.clock_offset_s = median;
    s.jitter_p95_ms = (p95 - mn) * 1000.0;
    s.clock_synced = std::fabs(median) < config_.clock_sync_tolerance_s && median > -0.05;
    if (s.clock_synced) {
      double sum = 0.0;
      for (const double v : o) {
        sum += v;
      }
      s.stamp_age_mean_ms = sum / static_cast<double>(o.size()) * 1000.0;
      s.stamp_age_p95_ms = p95 * 1000.0;
    }
  }
  const bool recent_event = now_wall - last_event_wall_ <= config_.event_memory_s;
  if (recent_event) {
    s.level = HealthLevel::kWarn;
    s.message = "recent: " + last_event_;
  } else {
    s.level = HealthLevel::kOk;
    s.message = "OK";
  }
  if (counters_.valid_ratio < 0.5 * config_.min_valid_ratio && counters_.frames > 0) {
    s.level = HealthLevel::kError;
    s.message = "very low valid-return ratio";
  }
  return s;
}

}  // namespace helios_obstacle_detection
