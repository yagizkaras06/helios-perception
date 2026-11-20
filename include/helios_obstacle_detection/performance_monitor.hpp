// Thread-safe timing / rate statistics measured with std::chrono::steady_clock.
#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

namespace helios_obstacle_detection
{

using SteadyClock = std::chrono::steady_clock;

/// Fixed-size ring buffer of samples with mean / max / percentile queries.
class RollingStat
{
public:
  explicit RollingStat(std::size_t window = 300);
  void add(double value);
  bool empty() const noexcept {return count_ == 0;}
  double last() const noexcept {return last_;}
  double mean() const;
  double max() const;
  double percentile(double p) const;  // p in [0,100], nearest-rank

private:
  std::vector<double> buffer_;
  std::size_t next_{0};
  std::size_t count_{0};
  double last_{0.0};
};

/// Event-rate estimate over a sliding time window.
class RateMeter
{
public:
  explicit RateMeter(std::chrono::duration<double> window = std::chrono::seconds(5));
  void tick(SteadyClock::time_point t);
  double rate(SteadyClock::time_point now);

private:
  std::chrono::duration<double> window_;
  std::deque<SteadyClock::time_point> events_;
};

enum class Stage : std::size_t
{
  kRoiFilter = 0,   // PointCloud2 -> PCL + NaN/range/ROI
  kVoxel,
  kGround,
  kClustering,      // clustering (runs concurrently with ego-motion)
  kBoundingBox,     // AABB + oriented box fitting
  kEgoMotion,       // LiDAR-only registration (concurrent task, own wall time)
  kTracking,        // association + Kalman update + risk
  kTotal,           // pipeline wall time (conversion .. risk)
  kQueueWait,       // driver message received -> worker starts processing
  kRxToPublish,     // driver message received -> primary outputs published (host latency)
  kVisualization,   // markers + debug clouds (separate thread, outside kTotal)
  kCount
};

const char * toString(Stage s) noexcept;

struct StageSummary
{
  std::string name;
  double last_ms{0}, mean_ms{0}, p95_ms{0}, max_ms{0};
  bool valid{false};
};

struct PerformanceSnapshot
{
  double input_fps{0};
  double output_fps{0};
  std::uint64_t frames_received{0};
  std::uint64_t frames_processed{0};
  std::uint64_t frames_dropped{0};
  std::uint64_t overload_frames{0};
  std::vector<StageSummary> stages;
};

class PerformanceMonitor
{
public:
  explicit PerformanceMonitor(std::size_t window = 300);

  void recordInput(SteadyClock::time_point t);
  void recordDropped();
  void recordProcessed(SteadyClock::time_point t, bool overload);
  void recordStage(Stage stage, double milliseconds);

  PerformanceSnapshot snapshot();

private:
  std::mutex mutex_;
  std::array<RollingStat, static_cast<std::size_t>(Stage::kCount)> stages_;
  RateMeter input_rate_;
  RateMeter output_rate_;
  std::uint64_t received_{0}, processed_{0}, dropped_{0}, overload_{0};
};

/// RAII timer: records elapsed milliseconds for a stage on destruction (or on stop()).
class ScopedStageTimer
{
public:
  ScopedStageTimer(PerformanceMonitor & monitor, Stage stage)
  : monitor_(monitor), stage_(stage), start_(SteadyClock::now()) {}
  ScopedStageTimer(const ScopedStageTimer &) = delete;
  ScopedStageTimer & operator=(const ScopedStageTimer &) = delete;
  ~ScopedStageTimer() {stop();}

  double stop()
  {
    if (!stopped_) {
      elapsed_ms_ =
        std::chrono::duration<double, std::milli>(SteadyClock::now() - start_).count();
      monitor_.recordStage(stage_, elapsed_ms_);
      stopped_ = true;
    }
    return elapsed_ms_;
  }

private:
  PerformanceMonitor & monitor_;
  Stage stage_;
  SteadyClock::time_point start_;
  double elapsed_ms_{0.0};
  bool stopped_{false};
};

}  // namespace helios_obstacle_detection
