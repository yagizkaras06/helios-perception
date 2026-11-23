// Collision warning for the operator display: "is something coming at me, and do the predictions
// say it will hit?" Built on the per-track TTC / CPA of collision_risk.hpp.
//
//   WARNING  an object approaches with a RELIABLE velocity, its predicted path (constant relative
//            velocity, k-sigma band) reaches the ego circle and the EXPECTED (nominal) TTC is
//            <= warning_ttc_s: a collision is predicted
//   CAUTION  on a collision course and either only the pessimistic TTC is <= warning_ttc_s or the
//            nominal TTC is <= caution_ttc_s (possible collision), or a dynamic object
//            approaching inside `near_distance`
//   CLEAR    nothing of the above
//
// Objects that are not MOVING are evaluated with zero own velocity (their KF velocity is noise):
// only the sensor's own motion can make them approach - a still sensor never warns about the
// desk next to it, a sensor driving at a wall does. MOVING objects must have travelled
// `min_object_displacement` (same rule as the display) so a briefly mis-classified static
// object does not raise an alarm. Tracks without a reliable
// velocity (predicted / occluded, ego motion unknown) never produce a "collision predicted".
// Hysteresis: a level needs `raise_frames` consecutive scans, and is held for `hold_s`.
// Display / information only - nothing here is connected to vehicle control.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "helios_obstacle_detection/collision_risk.hpp"
#include "helios_obstacle_detection/multi_object_tracker.hpp"

namespace helios_obstacle_detection
{

enum class WarningLevel : std::uint8_t {kClear = 0, kCaution = 1, kWarning = 2};

const char * toString(WarningLevel w) noexcept;

struct WarningConfig
{
  bool enabled{true};
  float warning_ttc_s{3.0F};        // expected TTC on a collision course -> WARNING
  float caution_ttc_s{6.0F};        // expected TTC on a collision course -> CAUTION
  float near_distance{1.0F};        // [m] clearance of an approaching dynamic object -> CAUTION
  float min_closing_speed{0.3F};    // [m/s] slower objects are not "coming at me"
  float min_object_displacement{0.5F};  // [m] MOVING objects must have travelled this far
  int raise_frames{2};
  float hold_s{1.0F};

  void validate() const;
};

struct CollisionWarning
{
  WarningLevel level{WarningLevel::kClear};
  bool collision_predicted{false};  // the threat's predicted path reaches the ego circle
  int track_index{-1};              // index into the frame's tracks (-1: none)
  std::uint32_t track_id{0};
  float distance{0.F};              // [m] closest-point distance from the LiDAR
  float clearance{0.F};             // [m] distance minus ego radius
  float closing_speed{0.F};
  float closing_speed_sigma{0.F};
  float ttc{0.F};                   // [s] nominal
  float ttc_pessimistic{0.F};       // [s] with closing speed + k sigma
  float cpa_time{0.F};
  float cpa_distance{0.F};
  std::string text;                 // ASCII one-liner (RViz fonts lack Turkish glyphs)
};

class CollisionWarningFilter
{
public:
  CollisionWarningFilter(const WarningConfig & config, const RiskConfig & risk);

  /// Evaluates one scan (tracks / risks parallel) and applies the hysteresis.
  CollisionWarning update(
    double stamp, const std::vector<TrackOutput> & tracks, const std::vector<RiskAssessment> & risks);
  void reset();

  /// Instantaneous level of one track (no hysteresis). `used` receives the risk the decision was
  /// based on (re-evaluated with zero own velocity for objects that are not MOVING).
  WarningLevel classify(
    const TrackOutput & t, const RiskAssessment & r, bool & collision, RiskAssessment & used) const;

private:
  WarningConfig config_;
  RiskConfig risk_;
  WarningLevel shown_{WarningLevel::kClear};
  WarningLevel pending_{WarningLevel::kClear};
  int pending_count_{0};
  double last_raise_stamp_{-1.0};   // last scan whose instantaneous level was >= shown_
  CollisionWarning last_threat_;    // threat kept while a level is held
};

}  // namespace helios_obstacle_detection
