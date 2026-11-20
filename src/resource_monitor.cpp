#include "helios_obstacle_detection/resource_monitor.hpp"

#include <unistd.h>

#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>

namespace helios_obstacle_detection
{

ResourceMonitor::ResourceMonitor()
{
  ticks_per_s_ = std::max(1L, sysconf(_SC_CLK_TCK));
  page_size_ = std::max(1L, sysconf(_SC_PAGESIZE));
  cores_ = std::max(1U, std::thread::hardware_concurrency());
  double rss = 0.0;
  int threads = 0;
  readProc(last_ticks_, rss, threads);
  last_time_ = std::chrono::steady_clock::now();
}

bool ResourceMonitor::readProc(std::uint64_t & cpu_ticks, double & rss_mb, int & threads) const
{
  std::ifstream f("/proc/self/stat");
  std::string line;
  if (!f || !std::getline(f, line)) {
    return false;
  }
  const auto pos = line.rfind(')');   // comm may contain spaces
  if (pos == std::string::npos) {
    return false;
  }
  std::istringstream is(line.substr(pos + 2));
  // fields after comm start at field 3 (state)
  std::string tok;
  std::uint64_t utime = 0, stime = 0;
  long num_threads = 0, rss_pages = 0;
  for (int field = 3; field <= 24 && (is >> tok); ++field) {
    if (field == 14) {
      utime = std::stoull(tok);
    } else if (field == 15) {
      stime = std::stoull(tok);
    } else if (field == 20) {
      num_threads = std::stol(tok);
    } else if (field == 24) {
      rss_pages = std::stol(tok);
    }
  }
  cpu_ticks = utime + stime;
  rss_mb = static_cast<double>(rss_pages) * static_cast<double>(page_size_) / (1024.0 * 1024.0);
  threads = static_cast<int>(num_threads);
  return true;
}

ResourceSample ResourceMonitor::sample()
{
  ResourceSample s;
  std::uint64_t ticks = 0;
  if (!readProc(ticks, s.rss_mb, s.threads)) {
    return s;
  }
  const auto now = std::chrono::steady_clock::now();
  const double wall = std::chrono::duration<double>(now - last_time_).count();
  if (wall > 0.0) {
    const double cpu_s = static_cast<double>(ticks - last_ticks_) /
      static_cast<double>(ticks_per_s_);
    s.cpu_percent = 100.0 * cpu_s / wall;
    s.cpu_percent_total = s.cpu_percent / static_cast<double>(cores_);
    s.valid = true;
  }
  last_ticks_ = ticks;
  last_time_ = now;
  return s;
}

}  // namespace helios_obstacle_detection
