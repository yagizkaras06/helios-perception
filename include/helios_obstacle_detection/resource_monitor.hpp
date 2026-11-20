// Process CPU / memory usage from /proc (Linux).
#pragma once

#include <chrono>
#include <cstdint>

namespace helios_obstacle_detection
{

struct ResourceSample
{
  double cpu_percent{0.0};        // 100 % = one fully used core
  double cpu_percent_total{0.0};  // normalised by the number of cores (0..100)
  double rss_mb{0.0};
  int threads{0};
  bool valid{false};
};

class ResourceMonitor
{
public:
  ResourceMonitor();
  /// CPU usage since the previous call (first call: since construction).
  ResourceSample sample();

private:
  bool readProc(std::uint64_t & cpu_ticks, double & rss_mb, int & threads) const;

  std::uint64_t last_ticks_{0};
  std::chrono::steady_clock::time_point last_time_;
  long ticks_per_s_{100};
  long page_size_{4096};
  unsigned cores_{1};
};

}  // namespace helios_obstacle_detection
