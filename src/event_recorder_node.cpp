// event_recorder: records the 5 s before and after a critical event (rosbag2 + metadata.json).
//
// Runs as its own node in the detector's namespace and reads the detector's own settings from the
// same parameter file (input_topic, input_reliability), so topic names and the input QoS match.
// Recorded: raw LiDAR input, <ns>/tracks, <ns>/obstacles, <ns>/processing_stats (+ optional
// <ns>/track_markers, <ns>/collision_warning) and /diagnostics messages of the detector node.
// Triggers: risk -> CRITICAL, an established LiDAR stream -> STATUS_NO_DATA, a usable ego-motion
// estimate -> EGO_UNKNOWN (start-up without data / ego is normal and never triggers).
// Display / diagnosis only: the detector keeps running whatever happens here.
#include <algorithm>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <std_msgs/msg/string.hpp>

#include "helios_obstacle_detection/event_recorder_core.hpp"
#include "helios_obstacle_detection/msg/processing_stats.hpp"
#include "helios_obstacle_detection/msg/tracked_object.hpp"
#include "helios_obstacle_detection/msg/tracked_object_array.hpp"
#include "helios_obstacle_detection/node_factories.hpp"

#ifndef HELIOS_VERSION
#define HELIOS_VERSION "unknown"
#endif

namespace helios_obstacle_detection
{
namespace events
{

namespace
{
using SteadyClock = std::chrono::steady_clock;
using SystemClock = std::chrono::system_clock;

std::int64_t steadyNs()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    SteadyClock::now().time_since_epoch()).count();
}

std::int64_t systemNs()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    SystemClock::now().time_since_epoch()).count();
}

std::string expandHome(const std::string & p)
{
  if (!p.empty() && p[0] == '~') {
    const char * home = std::getenv("HOME");
    return std::string(home ? home : "") + p.substr(1);
  }
  return p;
}

nlohmann::json paramToJson(const rclcpp::Parameter & p)
{
  using T = rclcpp::ParameterType;
  switch (p.get_type()) {
    case T::PARAMETER_BOOL: return p.as_bool();
    case T::PARAMETER_INTEGER: return p.as_int();
    case T::PARAMETER_DOUBLE: return p.as_double();
    case T::PARAMETER_STRING: return p.as_string();
    case T::PARAMETER_BOOL_ARRAY: return p.as_bool_array();
    case T::PARAMETER_INTEGER_ARRAY: return p.as_integer_array();
    case T::PARAMETER_DOUBLE_ARRAY: return p.as_double_array();
    case T::PARAMETER_STRING_ARRAY: return p.as_string_array();
    default: return p.value_to_string();
  }
}

std::uint64_t directorySize(const std::filesystem::path & dir)
{
  std::uint64_t total = 0;
  std::error_code ec;
  if (!std::filesystem::exists(dir, ec)) {
    return 0;
  }
  for (auto it = std::filesystem::recursive_directory_iterator(dir,
    std::filesystem::directory_options::skip_permission_denied, ec);
    !ec && it != std::filesystem::recursive_directory_iterator(); it.increment(ec))
  {
    std::error_code e2;
    if (it->is_regular_file(e2)) {
      total += it->file_size(e2);
    }
  }
  return total;
}
}  // namespace

class EventRecorderNode : public rclcpp::Node
{
public:
  explicit EventRecorderNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("event_recorder", options), diagnostics_(this)
  {
    // ---- settings shared with the detector (same parameter file) ---------------------------
    input_topic_ = declare_parameter<std::string>("input_topic", "/rslidar_points");
    const std::string reliability = declare_parameter<std::string>("input_reliability", "reliable");
    // ---- recorder settings -------------------------------------------------------------------
    detector_node_ = declare_parameter<std::string>("events.detector_node", "obstacle_detector");
    out_dir_ = expandHome(declare_parameter<std::string>(
        "events.output_dir", "~/helios_ws/logs/events"));
    SessionConfig sc;
    sc.pre_event_s = declare_parameter<double>("events.pre_event_s", 5.0);
    sc.post_event_s = declare_parameter<double>("events.post_event_s", 5.0);
    sc.max_event_s = declare_parameter<double>("events.max_event_s", 60.0);
    sc.buffer_max_bytes = static_cast<std::size_t>(
      declare_parameter<double>("events.buffer_max_mb", 192.0) * 1024.0 * 1024.0);
    WriterConfig wc;
    wc.queue_max_bytes = static_cast<std::size_t>(
      declare_parameter<double>("events.queue_max_mb", 256.0) * 1024.0 * 1024.0);
    wc.storage_id = declare_parameter<std::string>("events.storage_id", "mcap");
    max_total_disk_ = static_cast<std::uint64_t>(
      declare_parameter<double>("events.max_total_disk_mb", 4096.0) * 1024.0 * 1024.0);
    max_event_disk_ = static_cast<std::uint64_t>(
      declare_parameter<double>("events.max_event_disk_mb", 1024.0) * 1024.0 * 1024.0);
    min_free_disk_ = static_cast<std::uint64_t>(
      declare_parameter<double>("events.min_free_disk_mb", 1024.0) * 1024.0 * 1024.0);
    TriggerConfig tc;
    tc.critical_risk = declare_parameter<bool>("events.trigger_critical_risk", true);
    tc.no_data = declare_parameter<bool>("events.trigger_no_data", true);
    tc.ego_unknown = declare_parameter<bool>("events.trigger_ego_unknown", true);
    tc.rearm_s = declare_parameter<double>("events.rearm_s", 1.0);
    cooldown_ns_ = static_cast<std::int64_t>(
      declare_parameter<double>("events.cooldown_s", 30.0) * 1e9);
    risk_ignore_static_ = declare_parameter<bool>("events.risk_ignore_static", true);
    const bool rec_markers = declare_parameter<bool>("events.record_track_markers", true);
    const bool rec_warning = declare_parameter<bool>("events.record_collision_warning", true);
    clock_jump_tolerance_s_ = declare_parameter<double>("events.clock_jump_tolerance_s", 0.5);
    sensor_jump_s_ = declare_parameter<double>("events.sensor_stamp_jump_s", 2.0);

    // ---- topic table (names resolved in this node's namespace, like the detector's) ---------
    auto resolve = [this](const std::string & n) {
        return get_node_topics_interface()->resolve_topic_name(n);
      };
    // Raw input: best effort by default. A best-effort reader is compatible with reliable and
    // best-effort writers and can never throttle the driver (a reliable reader of ~1 MB clouds can
    // stall the publisher through DDS flow control and starve the detector). "match_detector"
    // uses the detector's input_reliability instead.
    const std::string raw_rel = declare_parameter<std::string>("events.raw_reliability", "best_effort");
    rclcpp::QoS raw_qos = rclcpp::QoS(rclcpp::KeepLast(5));
    if (raw_rel == "best_effort" || (raw_rel == "match_detector" && reliability == "best_effort")) {
      raw_qos.best_effort();
    } else {
      raw_qos.reliable();
    }
    const rclcpp::QoS out_qos = rclcpp::QoS(rclcpp::KeepLast(20)).reliable();
    topics_.push_back({resolve(input_topic_), "sensor_msgs/msg/PointCloud2", raw_qos});
    idx_tracks_ = add({resolve("tracks"), "helios_obstacle_detection/msg/TrackedObjectArray", out_qos});
    add({resolve("obstacles"), "helios_obstacle_detection/msg/ObstacleArray", out_qos});
    idx_stats_ = add({resolve("processing_stats"), "helios_obstacle_detection/msg/ProcessingStats",
        out_qos});
    idx_diag_ = add({"/diagnostics", "diagnostic_msgs/msg/DiagnosticArray",
        rclcpp::QoS(rclcpp::KeepLast(50)).reliable()});
    if (rec_markers) {
      add({resolve("track_markers"), "visualization_msgs/msg/MarkerArray", out_qos});
    }
    if (rec_warning) {
      add({resolve("collision_warning"), "helios_obstacle_detection/msg/CollisionWarning", out_qos});
    }

    wc.stamp_check_topic = 0;   // raw input
    wc.stamp_check_rate_hz = declare_parameter<double>("health.expected_rate_hz", 10.0);
    writer_ = std::make_unique<EventWriter>(wc, topics_);
    core_ = std::make_unique<EventRecorderCore>(sc, *writer_);
    monitor_ = std::make_unique<TriggerMonitor>(tc);

    for (std::size_t i = 0; i < topics_.size(); ++i) {
      const int idx = static_cast<int>(i);
      subs_.push_back(create_generic_subscription(topics_[i].name, topics_[i].type, topics_[i].qos,
        [this, idx](std::shared_ptr<const rclcpp::SerializedMessage> msg) {onMessage(idx, msg);}));
    }
    pub_events_ = create_publisher<std_msgs::msg::String>("recorder_events",
        rclcpp::QoS(10).reliable().transient_local());
    tick_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] {onTick();});

    diagnostics_.setHardwareID("helios_event_recorder");
    diagnostics_.add("Event recorder", this, &EventRecorderNode::diag);

    params_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this,
        get_effective_namespace() == "/" ? "/" + detector_node_ :
        get_effective_namespace() + "/" + detector_node_);
    recorder_params_json_ = ownParameters();
    refreshDetectorParameters();

    std::error_code ec;
    std::filesystem::create_directories(out_dir_, ec);
    std::string tl;
    for (const auto & t : topics_) {
      tl += (tl.empty() ? "" : ", ") + t.name;
    }
    RCLCPP_INFO(get_logger(), "Event recorder: %.1f s before / %.1f s after (max %.0f s) -> %s; "
      "topics: %s", sc.pre_event_s, sc.post_event_s, sc.max_event_s, out_dir_.c_str(), tl.c_str());
  }

  ~EventRecorderNode() override
  {
    if (core_) {
      auto info = core_->finish(steadyNs(), "shutdown");
      (void)info;
    }
    if (writer_) {
      writer_->flush();
    }
  }

private:
  int add(TopicInfo t)
  {
    topics_.push_back(std::move(t));
    return static_cast<int>(topics_.size() - 1);
  }

  void onMessage(int idx, const std::shared_ptr<const rclcpp::SerializedMessage> & msg)
  {
    const std::int64_t st = steadyNs(), sy = systemNs();
    // wall clock jump: change of (wall - monotonic)
    const std::int64_t offset = sy - st;
    if (have_offset_ && std::llabs(offset - last_offset_) >
      static_cast<std::int64_t>(clock_jump_tolerance_s_ * 1e9))
    {
      core_->noteJump({"wall_clock", st, static_cast<double>(offset - last_offset_) * 1e-9});
      ++clock_jumps_;
      RCLCPP_WARN(get_logger(), "wall clock jumped by %.3f s",
        static_cast<double>(offset - last_offset_) * 1e-9);
    }
    last_offset_ = offset;
    have_offset_ = true;

    if (idx == idx_diag_) {
      // keep only the detector's diagnostics (original bytes are recorded unchanged)
      diagnostic_msgs::msg::DiagnosticArray d;
      try {
        diag_ser_.deserialize_message(msg.get(), &d);
      } catch (...) {
        return;
      }
      const std::string prefix = detector_node_ + ":";
      const bool mine = std::any_of(d.status.begin(), d.status.end(),
          [&](const auto & s) {return s.name.rfind(prefix, 0) == 0;});
      if (!mine) {
        return;
      }
    }
    core_->onMessage(idx, msg, st, sy);

    std::vector<Trigger> trig;
    if (idx == idx_tracks_) {
      msg::TrackedObjectArray t;
      tracks_ser_.deserialize_message(msg.get(), &t);
      std::uint8_t level = 0;
      std::int64_t id = -1;
      for (const auto & o : t.objects) {
        if (o.track_state == msg::TrackedObject::TRACK_TENTATIVE ||
          (risk_ignore_static_ && o.motion_state == msg::TrackedObject::MOTION_STATIC))
        {
          continue;   // static background (e.g. the desk at the sensor) is not an incident
        }
        if (risk_ignore_static_ && background_learning_ && !o.ever_moved) {
          continue;   // background still being learned: static objects are not STATIC yet
        }
        if (o.risk_level > level) {
          level = o.risk_level;
          id = static_cast<std::int64_t>(o.id);
        }
      }
      // sensor clock jumps (informational; the windows never use sensor time)
      const double stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9;
      if (last_stamp_ > 0.0 && (stamp < last_stamp_ - 0.05 || stamp - last_stamp_ > sensor_jump_s_)) {
        core_->noteJump({"sensor_stamp", st, stamp - last_stamp_});
      }
      last_stamp_ = stamp;
      trig = monitor_->onTracks(st, sy, t.ego_state, level, id);
    } else if (idx == idx_stats_) {
      msg::ProcessingStats s;
      stats_ser_.deserialize_message(msg.get(), &s);
      if (s.status != msg::ProcessingStats::STATUS_NO_DATA && s.frames_processed > 0) {
        background_learning_ = s.background_learning;   // meaningful only with processed frames
      }
      trig = monitor_->onStatus(st, sy, s.status);
    }
    for (const auto & t : trig) {
      handleTrigger(t, st);
    }
  }

  void handleTrigger(const Trigger & t, std::int64_t now)
  {
    if (core_->active()) {
      core_->onTrigger(t, now, nullptr);
      RCLCPP_INFO(get_logger(), "event %s: merged trigger %s", core_->current()->id.c_str(),
        toString(t.type));
      publishEvent("merged", core_->current(), &t);
      return;
    }
    // cooldown: a trigger of a type that ended an event less than cooldown_s ago is the same
    // situation repeating (e.g. a person next to the sensor) - counted, not recorded again
    const auto ti = static_cast<std::size_t>(t.type);
    if (last_end_ns_[ti] > 0 && now - last_end_ns_[ti] < cooldown_ns_) {
      ++suppressed_cooldown_;
      RCLCPP_DEBUG(get_logger(), "%s within cooldown: not a new event", toString(t.type));
      return;
    }
    // disk limits: quota of the events folder and free space on its file system
    std::error_code ec;
    std::filesystem::create_directories(out_dir_, ec);
    const std::uint64_t used = directorySize(out_dir_);
    const auto sp = std::filesystem::space(out_dir_, ec);
    const std::uint64_t free_b = ec ? 0 : static_cast<std::uint64_t>(sp.available);
    if (used >= max_total_disk_ || free_b < min_free_disk_) {
      ++skipped_disk_;
      disk_error_ = used >= max_total_disk_ ?
        "events folder quota reached (" + std::to_string(used >> 20) + " MB)" :
        "free disk space below limit (" + std::to_string(free_b >> 20) + " MB)";
      RCLCPP_ERROR(get_logger(), "event %s NOT recorded: %s", toString(t.type), disk_error_.c_str());
      return;
    }
    disk_error_.clear();
    auto info = std::make_shared<EventInfo>();
    info->id = makeId(t);
    info->dir = (std::filesystem::path(out_dir_) / info->id).string();
    info->software_version = std::string(HELIOS_VERSION) + " (built " + __DATE__ + " " + __TIME__ + ")";
    info->detector_parameters_json = detector_params_json_;
    info->recorder_parameters_json = recorder_params_json_;
    info->disk_budget_bytes = std::min<std::uint64_t>(max_event_disk_,
        std::min<std::uint64_t>(max_total_disk_ - used, free_b - min_free_disk_));
    core_->onTrigger(t, now, info);
    refreshDetectorParameters();   // fresh values for this event's metadata
    RCLCPP_WARN(get_logger(), "EVENT %s started (%s%s): %s", info->id.c_str(), toString(t.type),
      t.track_id >= 0 ? (", track " + std::to_string(t.track_id)).c_str() : "", t.detail.c_str());
    publishEvent("started", info, &t);
  }

  void onTick()
  {
    const std::int64_t now = steadyNs();
    auto closed = core_->tick(now);
    if (closed) {
      for (const auto & tr : closed->triggers) {
        last_end_ns_[static_cast<std::size_t>(tr.type)] = now;
      }
      RCLCPP_INFO(get_logger(), "event %s closed (%s, %.1f s pre covered, %zu trigger(s)) -> %s",
        closed->id.c_str(), closed->end_reason.c_str(), closed->pre_covered_s,
        closed->triggers.size(), closed->dir.c_str());
      publishEvent("closed", closed, nullptr);
    }
    if (++tick_count_ % 20 == 0) {
      diagnostics_.force_update();
    }
  }

  std::string makeId(const Trigger & t)
  {
    const std::time_t s = static_cast<std::time_t>(t.system_ns / 1000000000LL);
    std::tm tm{};
    localtime_r(&s, &tm);
    std::ostringstream os;
    os << std::put_time(&tm, "%Y%m%d_%H%M%S") << '_' << std::setw(3) << std::setfill('0') <<
      (t.system_ns / 1000000LL) % 1000 << '_' << toString(t.type);
    std::string id = os.str();
    for (int k = 2; std::filesystem::exists(std::filesystem::path(out_dir_) / id); ++k) {
      id = os.str() + "_" + std::to_string(k);
    }
    return id;
  }

  void publishEvent(const char * state, const std::shared_ptr<EventInfo> & info, const Trigger * t)
  {
    nlohmann::json j;
    j["state"] = state;
    j["id"] = info->id;
    j["dir"] = info->dir;
    if (t) {
      j["trigger"] = toString(t->type);
      j["track_id"] = t->track_id;
    }
    j["triggers"] = info->triggers.size();
    std_msgs::msg::String m;
    m.data = j.dump();
    pub_events_->publish(m);
  }

  std::string ownParameters()
  {
    nlohmann::json j = nlohmann::json::object();
    const auto names = list_parameters({}, 0).names;
    for (const auto & p : get_parameters(names)) {
      j[p.get_name()] = paramToJson(p);
    }
    return j.dump();
  }

  void refreshDetectorParameters()
  {
    if (!params_client_->service_is_ready()) {
      return;
    }
    params_client_->list_parameters({}, 0,
      [this](std::shared_future<rcl_interfaces::msg::ListParametersResult> f) {
        const auto names = f.get().names;
        params_client_->get_parameters(names,
        [this](std::shared_future<std::vector<rclcpp::Parameter>> g) {
          nlohmann::json j = nlohmann::json::object();
          for (const auto & p : g.get()) {
            j[p.get_name()] = paramToJson(p);
          }
          detector_params_json_ = j.dump();
          if (core_->active()) {
            core_->current()->detector_parameters_json = detector_params_json_;
          }
        });
      });
  }

  void diag(diagnostic_updater::DiagnosticStatusWrapper & d)
  {
    const WriterStatus ws = writer_->status();
    const auto & buf = core_->buffer();
    const double covered = buf.size() ?
      static_cast<double>(steadyNs() - buf.oldestNs()) * 1e-9 : 0.0;
    std::error_code ec;
    const auto sp = std::filesystem::space(out_dir_, ec);
    if (ws.write_errors > 0 && !ws.last_error.empty()) {
      d.summaryf(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "write error: %s",
        ws.last_error.c_str());
    } else if (!disk_error_.empty()) {
      d.summaryf(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "events not recorded: %s",
        disk_error_.c_str());
    } else if (ws.dropped_queue_total > 0 || ws.truncated_events > 0) {
      d.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "messages dropped (write queue full) or event truncated (disk budget)");
    } else {
      d.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
        core_->active() ? "recording event" : "armed");
    }
    d.add("output_dir", out_dir_);
    d.add("active_event", core_->active() ? core_->current()->id : std::string("-"));
    d.add("events_started", std::to_string(core_->eventsStarted()));
    d.add("events_written", std::to_string(ws.events_written));
    d.add("triggers_merged", std::to_string(core_->triggersMerged()));
    d.add("events_skipped_disk", std::to_string(skipped_disk_));
    d.add("triggers_in_cooldown", std::to_string(suppressed_cooldown_));
    d.add("buffer_seconds", std::to_string(covered));
    d.add("buffer_mb", std::to_string(static_cast<double>(buf.bytes()) / 1048576.0));
    d.add("buffer_evicted_by_bytes", std::to_string(buf.evictedByBytes()));
    d.add("queue_mb", std::to_string(static_cast<double>(ws.queue_bytes) / 1048576.0));
    d.add("queue_dropped_messages", std::to_string(ws.dropped_queue_total));
    d.add("truncated_events", std::to_string(ws.truncated_events));
    d.add("write_errors", std::to_string(ws.write_errors));
    d.add("last_error", ws.last_error.empty() ? std::string("-") : ws.last_error);
    d.add("free_disk_mb", ec ? std::string("?") : std::to_string(sp.available >> 20));
    d.add("wall_clock_jumps", std::to_string(clock_jumps_));
    d.add("last_event_dir", ws.last_event_dir.empty() ? std::string("-") : ws.last_event_dir);
  }

  // parameters / state
  std::string input_topic_, detector_node_, out_dir_;
  std::uint64_t max_total_disk_{0}, max_event_disk_{0}, min_free_disk_{0};
  bool risk_ignore_static_{true};
  bool background_learning_{true};   // from processing_stats (start: assume learning)
  double clock_jump_tolerance_s_{0.5}, sensor_jump_s_{2.0};
  std::vector<TopicInfo> topics_;
  int idx_tracks_{-1}, idx_stats_{-1}, idx_diag_{-1};
  std::unique_ptr<EventWriter> writer_;
  std::unique_ptr<EventRecorderCore> core_;
  std::unique_ptr<TriggerMonitor> monitor_;
  std::vector<rclcpp::GenericSubscription::SharedPtr> subs_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_events_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  diagnostic_updater::Updater diagnostics_;
  std::shared_ptr<rclcpp::AsyncParametersClient> params_client_;
  std::string detector_params_json_{"{}"}, recorder_params_json_{"{}"};
  rclcpp::Serialization<diagnostic_msgs::msg::DiagnosticArray> diag_ser_;
  rclcpp::Serialization<msg::TrackedObjectArray> tracks_ser_;
  rclcpp::Serialization<msg::ProcessingStats> stats_ser_;
  bool have_offset_{false};
  std::int64_t last_offset_{0};
  double last_stamp_{-1.0};
  std::uint64_t clock_jumps_{0}, skipped_disk_{0}, tick_count_{0};
  std::string disk_error_;
  std::int64_t cooldown_ns_{0};
  std::int64_t last_end_ns_[3]{0, 0, 0};   // per TriggerType: steady time the last event ended
  std::uint64_t suppressed_cooldown_{0};
};

}  // namespace events
}  // namespace helios_obstacle_detection

namespace helios_obstacle_detection
{
std::shared_ptr<rclcpp::Node> makeEventRecorderNode(const rclcpp::NodeOptions & options)
{
  return std::make_shared<events::EventRecorderNode>(options);
}
}  // namespace helios_obstacle_detection

#ifndef HELIOS_NO_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<helios_obstacle_detection::events::EventRecorderNode>();
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
#endif  // HELIOS_NO_MAIN
