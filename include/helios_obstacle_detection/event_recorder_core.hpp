// Event recorder core (ROS-light, unit-testable): pre-event RAM buffer, trigger monitor, event
// session (pre / post windows, merging, max duration) and a bounded background rosbag2 writer.
//
// Time base: all windows use the MONOTONIC (steady) clock of the host. Sensor header stamps are
// never used for windowing (the Helios clock is not synchronised with the host). Messages are
// stored serialized and written byte-identical, so header stamps are preserved. Bag timestamps
// are derived monotonically: bag_ns = wall_at_event_start + (steady - steady_at_event_start), so
// a wall-clock jump cannot reorder a recording; such jumps are listed in metadata.json.
//
// Every buffered message gets a strictly increasing sequence number. While an event is active
// every new message is queued for the writer under the same lock that hands the pre-event
// snapshot over, so the hand-over can neither lose nor duplicate messages; the writer verifies
// the continuity of the sequence numbers (gaps = queue-overflow drops, reported).
#pragma once

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/qos.hpp>
#include <rclcpp/serialized_message.hpp>

namespace helios_obstacle_detection
{
namespace events
{

struct TopicInfo
{
  std::string name;               // fully qualified
  std::string type;               // e.g. sensor_msgs/msg/PointCloud2
  rclcpp::QoS qos{10};
};

struct RecordedMessage
{
  std::uint64_t seq{0};
  int topic{-1};                  // index into the topic table
  std::shared_ptr<const rclcpp::SerializedMessage> data;
  std::size_t bytes{0};
  std::int64_t steady_ns{0};      // receive time, monotonic
  std::int64_t system_ns{0};      // receive time, wall clock (informational)
};

// ------------------------------------------------------------------------------------ buffer
class PreEventBuffer
{
public:
  PreEventBuffer(double keep_s, std::size_t max_bytes);

  /// Appends (seq must increase) and evicts by age (relative to this message) and by bytes.
  void push(RecordedMessage m);
  /// Messages received at or after `t_ns` (oldest first).
  std::vector<RecordedMessage> since(std::int64_t t_ns) const;

  std::size_t bytes() const noexcept {return bytes_;}
  std::size_t size() const noexcept {return q_.size();}
  std::int64_t oldestNs() const noexcept {return q_.empty() ? 0 : q_.front().steady_ns;}
  std::uint64_t evictedByBytes() const noexcept {return evicted_bytes_;}
  /// Steady time of the newest message evicted because of the byte limit (0: never).
  std::int64_t lastByteEvictionNs() const noexcept {return last_byte_evict_ns_;}

private:
  std::int64_t keep_ns_;
  std::size_t max_bytes_;
  std::deque<RecordedMessage> q_;
  std::size_t bytes_{0};
  std::uint64_t evicted_bytes_{0};
  std::int64_t last_byte_evict_ns_{0};
};

// ---------------------------------------------------------------------------------- triggers
enum class TriggerType : std::uint8_t {kCriticalRisk = 0, kNoData = 1, kEgoUnknown = 2};
const char * toString(TriggerType t) noexcept;

struct Trigger
{
  TriggerType type{TriggerType::kCriticalRisk};
  std::int64_t steady_ns{0};
  std::int64_t system_ns{0};
  std::int64_t track_id{-1};
  std::string detail;
};

struct TriggerConfig
{
  bool critical_risk{true};
  bool no_data{true};
  bool ego_unknown{true};
  /// A condition must have been absent (risk) / present (data, valid ego) this long before it
  /// can trigger again: no event per message, no event storms when a state flickers.
  double rearm_s{1.0};
};

/// Edge-triggered conditions. Start-up is normal: no data yet / no valid ego yet never trigger;
/// a risk that is already CRITICAL in the very first message only becomes a trigger after it
/// has cleared for rearm_s. After a data loss the ego estimate restarts, so the ego condition
/// is re-armed only after a valid estimate has been seen again.
class TriggerMonitor
{
public:
  explicit TriggerMonitor(const TriggerConfig & config);

  /// One tracks message: ego state (TrackedObjectArray.EGO_*), risk level (RISK_*, already
  /// filtered by the caller) and the track carrying it (-1: none).
  std::vector<Trigger> onTracks(
    std::int64_t steady_ns, std::int64_t system_ns, std::uint8_t ego_state, std::uint8_t risk_level,
    std::int64_t risk_track_id);
  /// One processing_stats message (ProcessingStats.STATUS_*).
  std::vector<Trigger> onStatus(std::int64_t steady_ns, std::int64_t system_ns, std::uint8_t status);

  static constexpr std::uint8_t kEgoUnknown = 0;     // TrackedObjectArray.EGO_UNKNOWN
  static constexpr std::uint8_t kRiskCritical = 4;   // TrackedObject.RISK_CRITICAL
  static constexpr std::uint8_t kStatusNoData = 1;   // ProcessingStats.STATUS_NO_DATA

private:
  void dataFlowing(std::int64_t steady_ns);

  TriggerConfig config_;
  std::int64_t rearm_ns_;
  // risk
  bool tracks_seen_{false};
  bool risk_critical_{false};
  std::int64_t risk_clear_since_{0};
  // data
  bool data_flowing_{false};
  std::int64_t data_since_{0};
  // ego
  bool ego_valid_{false};
  std::int64_t ego_valid_since_{0};
};

// ------------------------------------------------------------------------------------ writer
struct JumpRecord
{
  std::string kind;               // "wall_clock" | "sensor_stamp"
  std::int64_t steady_ns{0};
  double delta_s{0.0};            // wall: change of (wall - steady); sensor: stamp step
};

struct EventInfo
{
  std::string id;
  std::string dir;                // event folder; bag in <dir>/bag, metadata in <dir>/metadata.json
  std::int64_t start_steady_ns{0};
  std::int64_t start_system_ns{0};
  std::int64_t end_steady_ns{0};
  std::vector<Trigger> triggers;
  double pre_requested_s{0.0};
  double pre_covered_s{0.0};
  std::string pre_limited_by;     // "" | "buffer_bytes" | "history" (recorder / data started later)
  double post_requested_s{0.0};
  double max_event_s{0.0};
  std::string end_reason;         // post_window | max_duration | shutdown
  std::vector<JumpRecord> jumps;
  std::string detector_parameters_json{"{}"};
  std::string recorder_parameters_json{"{}"};
  std::string software_version;
  std::uint64_t disk_budget_bytes{0};
};

struct WriterConfig
{
  std::size_t queue_max_bytes{256u << 20};
  std::string storage_id{"mcap"};
  /// Topic index whose header stamps are checked for missing frames (-1: none) and its nominal
  /// rate; the stamp is read from the CDR bytes (std_msgs/Header first), no deserialisation.
  int stamp_check_topic{-1};
  double stamp_check_rate_hz{10.0};
};

struct WriterStatus
{
  std::uint64_t events_written{0};
  std::uint64_t write_errors{0};
  std::uint64_t dropped_queue_total{0};  // messages dropped because the queue was full
  std::uint64_t truncated_events{0};     // disk budget reached during an event
  std::size_t queue_bytes{0};
  bool writing{false};
  std::string last_error;
  std::string last_event_dir;
};

/// Background rosbag2 writer fed through a byte-bounded queue. Message items are dropped (and
/// counted) when the queue is full; open / close items are always accepted.
class EventWriter
{
public:
  EventWriter(const WriterConfig & config, std::vector<TopicInfo> topics);
  ~EventWriter();
  EventWriter(const EventWriter &) = delete;
  EventWriter & operator=(const EventWriter &) = delete;

  void open(std::shared_ptr<EventInfo> info);
  bool push(const RecordedMessage & m);
  void close(std::shared_ptr<EventInfo> info);
  /// Blocks until the queue is empty and the current item is finished (tests / shutdown).
  void flush();
  WriterStatus status() const;

private:
  struct Item
  {
    int kind{0};                  // 0 open, 1 message, 2 close
    std::shared_ptr<EventInfo> info;
    RecordedMessage msg;
  };
  void loop();
  void doOpen(const std::shared_ptr<EventInfo> & info);
  void doWrite(const RecordedMessage & m);
  void doClose(const std::shared_ptr<EventInfo> & info);

  WriterConfig config_;
  std::vector<TopicInfo> topics_;
  mutable std::mutex mtx_;
  std::condition_variable cv_, idle_cv_;
  std::deque<Item> q_;
  std::size_t q_bytes_{0};
  bool busy_{false};
  bool stop_{false};
  WriterStatus status_;
  std::thread thread_;

  // state of the event being written (writer thread only)
  struct Current;
  std::unique_ptr<Current> cur_;
};

// ----------------------------------------------------------------------------------- session
struct SessionConfig
{
  double pre_event_s{5.0};
  double post_event_s{5.0};
  double max_event_s{60.0};
  double buffer_keep_margin_s{1.0};   // buffer holds pre_event_s + margin
  std::size_t buffer_max_bytes{192u << 20};
};

/// Ties buffer, triggers and writer together. NOT thread-safe: the caller serialises calls
/// (the ROS node runs a single-threaded executor).
class EventRecorderCore
{
public:
  EventRecorderCore(const SessionConfig & config, EventWriter & writer);

  /// Buffers one message (assigns its sequence number) and forwards it while an event runs.
  void onMessage(int topic, std::shared_ptr<const rclcpp::SerializedMessage> data,
    std::int64_t steady_ns, std::int64_t system_ns);
  /// Starts an event (returns its info) or merges into the running one (returns nullptr and
  /// extends the post window up to max_event_s). `make_info` fills id / dir / parameters.
  std::shared_ptr<EventInfo> onTrigger(const Trigger & t, std::int64_t now_steady_ns,
    std::shared_ptr<EventInfo> make_info);
  /// Ends the event when its post window / max duration is over; returns the closed info.
  std::shared_ptr<EventInfo> tick(std::int64_t now_steady_ns);
  /// Ends a running event immediately (shutdown).
  std::shared_ptr<EventInfo> finish(std::int64_t now_steady_ns, const std::string & reason);

  void noteJump(const JumpRecord & j);

  bool active() const noexcept {return active_ != nullptr;}
  const std::shared_ptr<EventInfo> & current() const noexcept {return active_;}
  const PreEventBuffer & buffer() const noexcept {return buffer_;}
  std::uint64_t eventsStarted() const noexcept {return started_;}
  std::uint64_t triggersMerged() const noexcept {return merged_;}

private:
  std::shared_ptr<EventInfo> close(std::int64_t now_steady_ns, const std::string & reason);

  SessionConfig config_;
  EventWriter & writer_;
  PreEventBuffer buffer_;
  std::uint64_t next_seq_{1};
  std::shared_ptr<EventInfo> active_;
  std::int64_t deadline_ns_{0};
  std::deque<JumpRecord> jumps_;
  std::uint64_t started_{0};
  std::uint64_t merged_{0};
};

}  // namespace events
}  // namespace helios_obstacle_detection
