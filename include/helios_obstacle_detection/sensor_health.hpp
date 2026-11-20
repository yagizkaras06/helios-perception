// Sensor data-loss / quality / timing analysis (ROS-free, thread-safe).
//
//   * frame gaps from header stamps (sensor clock): missed scans are estimated from dt / period
//   * time jumps (non-monotonic stamps: bag loop, driver restart) -> pipeline reset
//   * scan quality: finite-return ratio (blockage, dirt, rain), azimuth coverage (packet loss,
//     occlusion of the sensor window), sudden drop of the number of returns
//   * clock analysis: offset = receive wall time - header stamp. With an unsynchronised sensor
//     clock (no PTP / GPS) the absolute sensor->host latency is NOT observable; then only the
//     delivery jitter (offset - minimum offset in the window) is reported.
#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>

namespace helios_obstacle_detection
{

enum class HealthLevel : std::uint8_t {kOk = 0, kWarn = 1, kError = 2};

struct ScanQuality
{
  std::size_t total_points{0};
  std::size_t finite_points{0};
  int azimuth_sectors_total{0};
  int azimuth_sectors_covered{0};
};

struct SensorHealthConfig
{
  double expected_rate_hz{10.0};
  double gap_factor{1.5};              // dt > factor * period -> gap
  double min_valid_ratio{0.30};
  double min_azimuth_coverage{0.70};
  double point_drop_ratio{0.50};       // finite points < ratio * running mean -> drop event
  double clock_sync_tolerance_s{1.0};  // |median offset| below -> clocks considered synced
  double event_memory_s{10.0};         // events keep WARN level this long
  std::size_t window{100};

  void validate() const;
};

struct FrameHealth
{
  bool time_jump{false};
  bool gap{false};
  int missed_frames{0};
};

struct HealthSnapshot
{
  HealthLevel level{HealthLevel::kOk};
  std::string message;
  std::uint64_t frames{0};
  std::uint64_t gaps{0};
  std::uint64_t missed_frames{0};
  std::uint64_t time_jumps{0};
  std::uint64_t duplicate_stamps{0};
  std::uint64_t partial_scans{0};
  std::uint64_t low_valid_scans{0};
  std::uint64_t point_drops{0};
  double stamp_rate_hz{0.0};
  double valid_ratio{0.0};
  double azimuth_coverage{0.0};
  std::size_t points{0};
  bool clock_synced{false};
  double clock_offset_s{0.0};          // median (receive wall time - header stamp)
  double jitter_p95_ms{0.0};           // delivery jitter (offset - min offset)
  double stamp_age_mean_ms{0.0};       // only meaningful when clock_synced
  double stamp_age_p95_ms{0.0};
};

class SensorHealthMonitor
{
public:
  explicit SensorHealthMonitor(const SensorHealthConfig & config);

  /// stamp_s: header stamp (sensor clock); rx_wall_s: host wall-clock receive time.
  FrameHealth onFrame(double stamp_s, double rx_wall_s, const ScanQuality & q);
  /// now_wall_s: host wall-clock time, used to age out events.
  HealthSnapshot snapshot(double now_wall_s) const;
  void reset();

private:
  SensorHealthConfig config_;
  mutable std::mutex mutex_;
  HealthSnapshot counters_;
  std::deque<double> stamps_;
  std::deque<double> offsets_;
  double last_stamp_{-1.0};
  double mean_points_{0.0};
  double last_event_wall_{-1e18};
  std::string last_event_;
};

/// Approximate atan2 (max error ~0.005 rad), used for azimuth coverage binning.
float fastAtan2(float y, float x) noexcept;

}  // namespace helios_obstacle_detection
