#include "helios_obstacle_detection/performance_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace helios_obstacle_detection
{

RollingStat::RollingStat(std::size_t window)
: buffer_(std::max<std::size_t>(window, 1), 0.0) {}

void RollingStat::add(double value)
{
  buffer_[next_] = value;
  next_ = (next_ + 1) % buffer_.size();
  count_ = std::min(count_ + 1, buffer_.size());
  last_ = value;
}

double RollingStat::mean() const
{
  if (count_ == 0) {
    return 0.0;
  }
  return std::accumulate(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(count_),
           0.0) / static_cast<double>(count_);
}

double RollingStat::max() const
{
  if (count_ == 0) {
    return 0.0;
  }
  return *std::max_element(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(count_));
}

double RollingStat::percentile(double p) const
{
  if (count_ == 0) {
    return 0.0;
  }
  std::vector<double> v(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(count_));
  const double clamped = std::clamp(p, 0.0, 100.0);
  // nearest-rank method
  auto rank = static_cast<std::size_t>(std::ceil(clamped / 100.0 * static_cast<double>(count_)));
  rank = std::clamp<std::size_t>(rank, 1, count_);
  std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(rank - 1), v.end());
  return v[rank - 1];
}

RateMeter::RateMeter(std::chrono::duration<double> window)
: window_(window) {}

void RateMeter::tick(SteadyClock::time_point t)
{
  events_.push_back(t);
  while (!events_.empty() && t - events_.front() > window_) {
    events_.pop_front();
  }
}

double RateMeter::rate(SteadyClock::time_point now)
{
  while (!events_.empty() && now - events_.front() > window_) {
    events_.pop_front();
  }
  if (events_.size() < 2) {
    return 0.0;
  }
  const double span =
    std::chrono::duration<double>(events_.back() - events_.front()).count();
  // If the stream stopped, measure up to `now` so the rate decays instead of freezing.
  const double since_last = std::chrono::duration<double>(now - events_.back()).count();
  const double period = span / static_cast<double>(events_.size() - 1);
  const double effective_span = since_last > 2.0 * period ? span + since_last : span;
  return effective_span > 0.0 ?
         static_cast<double>(events_.size() - 1) / effective_span : 0.0;
}

const char * toString(Stage s) noexcept
{
  switch (s) {
    case Stage::kRoiFilter: return "roi_filter";
    case Stage::kVoxel: return "voxel_grid";
    case Stage::kGround: return "ground_segmentation";
    case Stage::kClustering: return "clustering";
    case Stage::kBoundingBox: return "bounding_box";
    case Stage::kEgoMotion: return "ego_motion";
    case Stage::kTracking: return "tracking_risk";
    case Stage::kTotal: return "total";
    case Stage::kQueueWait: return "queue_wait";
    case Stage::kRxToPublish: return "rx_to_publish";
    case Stage::kVisualization: return "visualization";
    case Stage::kCount: break;
  }
  return "unknown";
}

PerformanceMonitor::PerformanceMonitor(std::size_t window)
{
  for (auto & s : stages_) {
    s = RollingStat(window);
  }
}

void PerformanceMonitor::recordInput(SteadyClock::time_point t)
{
  std::lock_guard<std::mutex> lock(mutex_);
  input_rate_.tick(t);
  ++received_;
}

void PerformanceMonitor::recordDropped()
{
  std::lock_guard<std::mutex> lock(mutex_);
  ++dropped_;
}

void PerformanceMonitor::recordProcessed(SteadyClock::time_point t, bool overload)
{
  std::lock_guard<std::mutex> lock(mutex_);
  output_rate_.tick(t);
  ++processed_;
  if (overload) {
    ++overload_;
  }
}

void PerformanceMonitor::recordStage(Stage stage, double milliseconds)
{
  std::lock_guard<std::mutex> lock(mutex_);
  stages_[static_cast<std::size_t>(stage)].add(milliseconds);
}

PerformanceSnapshot PerformanceMonitor::snapshot()
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto now = SteadyClock::now();
  PerformanceSnapshot s;
  s.input_fps = input_rate_.rate(now);
  s.output_fps = output_rate_.rate(now);
  s.frames_received = received_;
  s.frames_processed = processed_;
  s.frames_dropped = dropped_;
  s.overload_frames = overload_;
  s.stages.reserve(stages_.size());
  for (std::size_t i = 0; i < stages_.size(); ++i) {
    const RollingStat & r = stages_[i];
    StageSummary ss;
    ss.name = toString(static_cast<Stage>(i));
    ss.valid = !r.empty();
    ss.last_ms = r.last();
    ss.mean_ms = r.mean();
    ss.p95_ms = r.percentile(95.0);
    ss.max_ms = r.max();
    s.stages.push_back(ss);
  }
  return s;
}

}  // namespace helios_obstacle_detection
