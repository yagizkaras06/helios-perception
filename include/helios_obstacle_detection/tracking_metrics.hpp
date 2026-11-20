// CLEAR-MOT style evaluation of tracker output against simulator ground truth.
#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "helios_obstacle_detection/collision_risk.hpp"
#include "helios_obstacle_detection/lidar_simulator.hpp"
#include "helios_obstacle_detection/multi_object_tracker.hpp"

namespace helios_obstacle_detection
{
namespace sim
{

struct MotSummary
{
  std::uint64_t frames{0};
  std::uint64_t gt_visible{0};
  std::uint64_t tp{0}, fp{0}, fn{0}, idsw{0};
  double mota{0.0};
  double motp_m{0.0};              // mean centre distance of matches
  double precision{0.0}, recall{0.0};
  // velocity (matches with velocity_reliable)
  std::uint64_t velocity_samples{0};
  double velocity_rmse{0.0};       // vector error [m/s]
  double speed_mae{0.0};
  std::uint64_t heading_samples{0};
  double heading_mae_deg{0.0};
  // motion classification on matched, velocity_reliable tracks
  std::uint64_t moving_as_moving{0}, moving_as_static{0}, moving_as_unknown{0};
  std::uint64_t static_as_static{0}, static_as_moving{0}, static_as_unknown{0};
  // TTC (GT TTC finite and below the horizon, estimate valid)
  std::uint64_t ttc_samples{0};
  double ttc_mae_s{0.0};
  double ttc_pessimistic_violations{0.0};  // share of samples where pessimistic TTC > true TTC
  std::map<int, int> static_as_moving_by_gt;   // gt id -> frames
  std::map<int, int> moving_as_static_by_gt;
  std::map<int, int> idsw_by_gt;
  std::map<int, int> fn_by_gt;
  // identity metrics
  std::uint64_t idtp{0};
  double idf1{0.0};                // global bipartite GT id <-> track id assignment
  std::uint64_t fragmentations{0}; // tracked -> interrupted -> tracked again
  // objects that never moved: share of matched frames shown MOVING
  std::uint64_t never_moved_frames{0}, never_moved_as_moving{0};
  double false_moving_rate{0.0};
  // motion onset: GT starts moving after >= 1 s standing; latency until the matched track is
  // MOVING (missed if not within 3 s or before the object stops again)
  std::vector<double> onset_latency_s;
  std::uint64_t onset_events{0}, onset_missed{0};
  // motion stop: latency until the matched track is no longer MOVING
  std::vector<double> stop_latency_s;
  std::uint64_t stop_events{0}, stop_missed{0};

  static double median(std::vector<double> v);
  std::string toMarkdown(const std::string & title) const;
  /// one line "key=value ..." for scripts
  std::string toKeyValues() const;
};

class MotEvaluator
{
public:
  /// `match_threshold` in metres (plus a quarter of the GT footprint diagonal).
  explicit MotEvaluator(double match_threshold = 1.0, int min_hits = 10, double ttc_horizon = 8.0);

  /// `time` [s] of the frame (scene time); negative -> frame index * 0.1 s
  void addFrame(
    const std::vector<GroundTruth> & gt, const std::vector<TrackOutput> & tracks,
    const std::vector<RiskAssessment> & risks, double time = -1.0);

  MotSummary summary() const;

private:
  double threshold_;
  int min_hits_;
  double ttc_horizon_;
  MotSummary s_;
  double motp_sum_{0.0};
  double vel_sq_sum_{0.0};
  double speed_abs_sum_{0.0};
  double heading_abs_sum_{0.0};
  double ttc_abs_sum_{0.0};
  std::uint64_t ttc_violations_{0};
  std::map<int, std::uint32_t> last_match_;  // gt id -> track id
  std::map<std::pair<int, std::uint32_t>, std::uint64_t> co_;  // (gt, track) -> TP frames
  struct GtHistory
  {
    bool initialised{false};
    bool moving{false};
    bool ever_moved{false};
    double since{0.0};             // time of the last moving / standing change
    double onset{-1.0};            // open onset event
    double stop{-1.0};             // open stop event
    bool tracked_once{false};
    bool interrupted{false};
  };
  std::map<int, GtHistory> hist_;
};

}  // namespace sim
}  // namespace helios_obstacle_detection
