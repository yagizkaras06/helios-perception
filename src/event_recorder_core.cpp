#include "helios_obstacle_detection/event_recorder_core.hpp"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include <nlohmann/json.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/topic_metadata.hpp>

namespace helios_obstacle_detection
{
namespace events
{

namespace
{
constexpr double kNs = 1e-9;

std::string isoTime(std::int64_t system_ns)
{
  const std::time_t s = static_cast<std::time_t>(system_ns / 1000000000LL);
  const int ms = static_cast<int>((system_ns / 1000000LL) % 1000);
  std::tm tm{};
  localtime_r(&s, &tm);
  std::ostringstream os;
  os << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S") << '.' << std::setw(3) << std::setfill('0') << ms <<
    std::put_time(&tm, "%z");
  return os.str();
}

nlohmann::json parseOr(const std::string & text)
{
  try {
    return nlohmann::json::parse(text);
  } catch (...) {
    return nlohmann::json(text);
  }
}
}  // namespace

const char * toString(TriggerType t) noexcept
{
  switch (t) {
    case TriggerType::kCriticalRisk: return "critical_risk";
    case TriggerType::kNoData: return "no_data";
    case TriggerType::kEgoUnknown: return "ego_unknown";
  }
  return "?";
}

// ===================================================================================== buffer
PreEventBuffer::PreEventBuffer(double keep_s, std::size_t max_bytes)
: keep_ns_(static_cast<std::int64_t>(keep_s * 1e9)), max_bytes_(max_bytes)
{
  if (!(keep_s > 0.0) || max_bytes == 0) {
    throw std::invalid_argument("event buffer: keep_s > 0 and max_bytes > 0 required");
  }
}

void PreEventBuffer::push(RecordedMessage m)
{
  if (!q_.empty() && m.seq <= q_.back().seq) {
    throw std::logic_error("event buffer: sequence numbers must increase");
  }
  const std::int64_t now = m.steady_ns;
  bytes_ += m.bytes;
  q_.push_back(std::move(m));
  while (!q_.empty() && q_.front().steady_ns < now - keep_ns_) {
    bytes_ -= q_.front().bytes;
    q_.pop_front();
  }
  while (bytes_ > max_bytes_ && q_.size() > 1) {
    last_byte_evict_ns_ = q_.front().steady_ns;
    bytes_ -= q_.front().bytes;
    q_.pop_front();
    ++evicted_bytes_;
  }
}

std::vector<RecordedMessage> PreEventBuffer::since(std::int64_t t_ns) const
{
  std::vector<RecordedMessage> out;
  for (const auto & m : q_) {
    if (m.steady_ns >= t_ns) {
      out.push_back(m);
    }
  }
  return out;
}

// =================================================================================== triggers
TriggerMonitor::TriggerMonitor(const TriggerConfig & config)
: config_(config), rearm_ns_(static_cast<std::int64_t>(config.rearm_s * 1e9))
{
  if (config.rearm_s < 0.0) {
    throw std::invalid_argument("event triggers: rearm_s >= 0 required");
  }
}

void TriggerMonitor::dataFlowing(std::int64_t steady_ns)
{
  if (!data_flowing_) {
    data_flowing_ = true;
    data_since_ = steady_ns;
  }
}

std::vector<Trigger> TriggerMonitor::onTracks(
  std::int64_t steady_ns, std::int64_t system_ns, std::uint8_t ego_state, std::uint8_t risk_level,
  std::int64_t risk_track_id)
{
  std::vector<Trigger> out;
  dataFlowing(steady_ns);   // tracks are only published for processed scans

  // ego motion: a usable estimate (seen for rearm_s) that turns into EGO_UNKNOWN
  if (ego_state != kEgoUnknown) {
    if (!ego_valid_) {
      ego_valid_ = true;
      ego_valid_since_ = steady_ns;
    }
  } else {
    if (ego_valid_ && steady_ns - ego_valid_since_ >= rearm_ns_ && config_.ego_unknown) {
      out.push_back({TriggerType::kEgoUnknown, steady_ns, system_ns, -1,
          "usable ego-motion estimate lost (EGO_UNKNOWN)"});
    }
    ego_valid_ = false;
  }

  // risk: transition into CRITICAL (the first message is only a baseline)
  const bool crit = risk_level >= kRiskCritical;
  if (!tracks_seen_) {
    tracks_seen_ = true;
    risk_critical_ = crit;
    risk_clear_since_ = steady_ns - rearm_ns_;   // armed unless already critical
    return out;
  }
  if (crit) {
    if (!risk_critical_ && steady_ns - risk_clear_since_ >= rearm_ns_ && config_.critical_risk) {
      out.push_back({TriggerType::kCriticalRisk, steady_ns, system_ns, risk_track_id,
          "risk level became CRITICAL"});
    }
    risk_critical_ = true;
  } else if (risk_critical_) {
    risk_critical_ = false;
    risk_clear_since_ = steady_ns;
  }
  return out;
}

std::vector<Trigger> TriggerMonitor::onStatus(
  std::int64_t steady_ns, std::int64_t system_ns, std::uint8_t status)
{
  std::vector<Trigger> out;
  if (status == kStatusNoData) {
    // only a stream that had been flowing can be interrupted (start-up is not an event)
    if (data_flowing_ && steady_ns - data_since_ >= rearm_ns_ && config_.no_data) {
      out.push_back({TriggerType::kNoData, steady_ns, system_ns, -1,
          "LiDAR data stream interrupted (STATUS_NO_DATA)"});
    }
    if (data_flowing_) {
      data_flowing_ = false;
      ego_valid_ = false;   // the ego estimate restarts after a gap: re-arm only when valid again
    }
  }
  return out;
}

// ===================================================================================== writer
struct EventWriter::Current
{
  std::shared_ptr<EventInfo> info;
  std::unique_ptr<rosbag2_cpp::Writer> bag;
  std::vector<std::uint64_t> count, bytes;
  std::uint64_t first_seq{0}, last_seq{0}, seq_gaps{0}, out_of_order{0};
  std::uint64_t written{0}, bytes_written{0}, skipped_truncated{0};
  std::int64_t first_steady{0}, last_steady{0};
  bool truncated{false};
  bool failed{false};
  std::string error;
};

EventWriter::EventWriter(const WriterConfig & config, std::vector<TopicInfo> topics)
: config_(config), topics_(std::move(topics))
{
  if (config_.queue_max_bytes == 0) {
    throw std::invalid_argument("event writer: queue_max_bytes > 0 required");
  }
  thread_ = std::thread([this] {loop();});
}

EventWriter::~EventWriter()
{
  {
    std::lock_guard<std::mutex> lk(mtx_);
    stop_ = true;
  }
  cv_.notify_all();
  if (thread_.joinable()) {
    thread_.join();
  }
}

void EventWriter::open(std::shared_ptr<EventInfo> info)
{
  {
    std::lock_guard<std::mutex> lk(mtx_);
    q_.push_back({0, std::move(info), {}});
  }
  cv_.notify_one();
}

bool EventWriter::push(const RecordedMessage & m)
{
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (q_bytes_ + m.bytes > config_.queue_max_bytes) {
      ++status_.dropped_queue_total;
      return false;
    }
    q_bytes_ += m.bytes;
    status_.queue_bytes = q_bytes_;
    q_.push_back({1, nullptr, m});
  }
  cv_.notify_one();
  return true;
}

void EventWriter::close(std::shared_ptr<EventInfo> info)
{
  {
    std::lock_guard<std::mutex> lk(mtx_);
    q_.push_back({2, std::move(info), {}});
  }
  cv_.notify_one();
}

void EventWriter::flush()
{
  std::unique_lock<std::mutex> lk(mtx_);
  idle_cv_.wait(lk, [this] {return q_.empty() && !busy_;});
}

WriterStatus EventWriter::status() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return status_;
}

void EventWriter::loop()
{
  for (;;) {
    Item it;
    {
      std::unique_lock<std::mutex> lk(mtx_);
      cv_.wait(lk, [this] {return stop_ || !q_.empty();});
      if (q_.empty()) {
        break;   // stop requested and everything written
      }
      it = std::move(q_.front());
      q_.pop_front();
      if (it.kind == 1) {
        q_bytes_ -= it.msg.bytes;
        status_.queue_bytes = q_bytes_;
      }
      busy_ = true;
    }
    try {
      if (it.kind == 0) {
        doOpen(it.info);
      } else if (it.kind == 1) {
        doWrite(it.msg);
      } else {
        doClose(it.info);
      }
    } catch (const std::exception & e) {
      std::lock_guard<std::mutex> lk(mtx_);
      ++status_.write_errors;
      status_.last_error = e.what();
      if (cur_) {
        cur_->failed = true;
        cur_->error = e.what();
      }
    }
    {
      std::lock_guard<std::mutex> lk(mtx_);
      busy_ = false;
    }
    idle_cv_.notify_all();
  }
  if (cur_) {   // shutdown in the middle of an event: still close the bag
    try {
      cur_->bag.reset();
    } catch (...) {
    }
  }
  idle_cv_.notify_all();
}

void EventWriter::doOpen(const std::shared_ptr<EventInfo> & info)
{
  cur_ = std::make_unique<Current>();
  cur_->info = info;
  cur_->count.assign(topics_.size(), 0);
  cur_->bytes.assign(topics_.size(), 0);
  {
    std::lock_guard<std::mutex> lk(mtx_);
    status_.writing = true;
  }
  std::filesystem::create_directories(info->dir);
  rosbag2_storage::StorageOptions so;
  so.uri = (std::filesystem::path(info->dir) / "bag").string();
  so.storage_id = config_.storage_id;
  cur_->bag = std::make_unique<rosbag2_cpp::Writer>();
  cur_->bag->open(so, rosbag2_cpp::ConverterOptions{"cdr", "cdr"});
  for (const auto & t : topics_) {
    rosbag2_storage::TopicMetadata md;
    md.name = t.name;
    md.type = t.type;
    md.serialization_format = "cdr";
    md.offered_qos_profiles = {t.qos};
    cur_->bag->create_topic(md);
  }
}

void EventWriter::doWrite(const RecordedMessage & m)
{
  if (!cur_ || cur_->failed) {
    return;
  }
  Current & c = *cur_;
  if (c.last_seq != 0) {
    if (m.seq <= c.last_seq) {
      ++c.out_of_order;   // never expected: sequence numbers are assigned under one lock
      return;
    }
    c.seq_gaps += m.seq - c.last_seq - 1;
  } else {
    c.first_seq = m.seq;
    c.first_steady = m.steady_ns;
  }
  c.last_seq = m.seq;
  c.last_steady = m.steady_ns;
  if (c.truncated || (c.info->disk_budget_bytes > 0 &&
    c.bytes_written + m.bytes > c.info->disk_budget_bytes))
  {
    if (!c.truncated) {
      c.truncated = true;
      std::lock_guard<std::mutex> lk(mtx_);
      ++status_.truncated_events;
    }
    ++c.skipped_truncated;
    return;
  }
  const auto & topic = topics_.at(static_cast<std::size_t>(m.topic));
  // monotonic bag time: event wall start + steady offset (immune to wall-clock jumps)
  const std::int64_t ts = c.info->start_system_ns + (m.steady_ns - c.info->start_steady_ns);
  c.bag->write(m.data, topic.name, topic.type, ts, ts);
  ++c.count[static_cast<std::size_t>(m.topic)];
  c.bytes[static_cast<std::size_t>(m.topic)] += m.bytes;
  ++c.written;
  c.bytes_written += m.bytes;
}

void EventWriter::doClose(const std::shared_ptr<EventInfo> & info)
{
  if (!cur_ || cur_->info != info) {
    return;
  }
  Current & c = *cur_;
  try {
    c.bag.reset();   // flush + close the rosbag2 storage
  } catch (const std::exception & e) {
    c.failed = true;
    c.error = e.what();
    std::lock_guard<std::mutex> lk(mtx_);
    ++status_.write_errors;
    status_.last_error = e.what();
  }
  const EventInfo & e = *info;
  const double t0 = static_cast<double>(e.start_steady_ns);
  nlohmann::json j;
  j["event_id"] = e.id;
  j["software_version"] = e.software_version;
  j["trigger_wall_time"] = isoTime(e.start_system_ns);
  j["trigger_monotonic_s"] = e.start_steady_ns * kNs;
  j["reasons"] = nlohmann::json::array();
  for (const auto & t : e.triggers) {
    nlohmann::json r;
    r["type"] = toString(t.type);
    r["wall_time"] = isoTime(t.system_ns);
    r["offset_s"] = (static_cast<double>(t.steady_ns) - t0) * kNs;
    r["track_id"] = t.track_id >= 0 ? nlohmann::json(t.track_id) : nlohmann::json(nullptr);
    r["detail"] = t.detail;
    j["reasons"].push_back(r);
  }
  nlohmann::json w;
  w["pre_requested_s"] = e.pre_requested_s;
  w["pre_covered_s"] = e.pre_covered_s;
  w["pre_limited_by"] = e.pre_limited_by.empty() ? nlohmann::json(nullptr) :
    nlohmann::json(e.pre_limited_by);
  w["post_requested_s"] = e.post_requested_s;
  w["max_event_s"] = e.max_event_s;
  w["recorded_from_s"] = c.written ? (static_cast<double>(c.first_steady) - t0) * kNs : 0.0;
  w["recorded_to_s"] = c.written ? (static_cast<double>(c.last_steady) - t0) * kNs : 0.0;
  w["event_end_s"] = (static_cast<double>(e.end_steady_ns) - t0) * kNs;
  w["end_reason"] = e.end_reason;
  w["time_base"] = "offsets relative to the first trigger, host monotonic clock";
  j["window"] = w;
  nlohmann::json b;
  b["path"] = "bag";
  b["storage_id"] = config_.storage_id;
  b["timestamps"] = "bag time = trigger wall time + monotonic offset; message headers unchanged";
  j["bag"] = b;
  j["topics"] = nlohmann::json::array();
  for (std::size_t i = 0; i < topics_.size(); ++i) {
    j["topics"].push_back({{"name", topics_[i].name}, {"type", topics_[i].type},
      {"messages", c.count[i]}, {"bytes", c.bytes[i]}});
  }
  nlohmann::json in;
  in["messages_written"] = c.written;
  in["bytes_written"] = c.bytes_written;
  in["first_seq"] = c.first_seq;
  in["last_seq"] = c.last_seq;
  in["sequence_gaps"] = c.seq_gaps;   // messages lost in the bounded queue
  in["out_of_order"] = c.out_of_order;
  in["truncated_by_disk_budget"] = c.truncated;
  in["messages_skipped_after_truncation"] = c.skipped_truncated;
  in["disk_budget_bytes"] = e.disk_budget_bytes;
  in["write_error"] = c.failed ? nlohmann::json(c.error) : nlohmann::json(nullptr);
  j["integrity"] = in;
  j["time_jumps"] = nlohmann::json::array();
  for (const auto & jr : e.jumps) {
    j["time_jumps"].push_back({{"kind", jr.kind},
      {"offset_s", (static_cast<double>(jr.steady_ns) - t0) * kNs}, {"delta_s", jr.delta_s}});
  }
  j["parameters"] = {{"detector", parseOr(e.detector_parameters_json)},
    {"recorder", parseOr(e.recorder_parameters_json)}};
  const std::string bag_dir = (std::filesystem::path(e.dir) / "bag").string();
  j["replay"] = {
    {"rerun_detection", "ros2 launch helios_obstacle_detection replay.launch.py bag:=" + bag_dir +
      " rviz:=true"},
    {"recorded_outputs", "ros2 launch helios_obstacle_detection replay.launch.py bag:=" + bag_dir +
      " mode:=recorded rviz:=true"}};
  {
    std::ofstream f(std::filesystem::path(e.dir) / "metadata.json");
    f << j.dump(2) << '\n';
    if (!f) {
      std::lock_guard<std::mutex> lk(mtx_);
      ++status_.write_errors;
      status_.last_error = "cannot write metadata.json in " + e.dir;
    }
  }
  std::lock_guard<std::mutex> lk(mtx_);
  if (!c.failed) {
    ++status_.events_written;
  }
  status_.last_event_dir = e.dir;
  status_.writing = false;
  cur_.reset();
}

// ==================================================================================== session
EventRecorderCore::EventRecorderCore(const SessionConfig & config, EventWriter & writer)
: config_(config), writer_(writer),
  buffer_(config.pre_event_s + config.buffer_keep_margin_s, config.buffer_max_bytes)
{
  if (!(config.pre_event_s >= 0.0 && config.post_event_s >= 0.0 &&
    config.max_event_s >= config.post_event_s && config.max_event_s > 0.0))
  {
    throw std::invalid_argument(
            "event session: pre/post >= 0 and max_event_s >= post_event_s > 0 required");
  }
}

void EventRecorderCore::onMessage(
  int topic, std::shared_ptr<const rclcpp::SerializedMessage> data, std::int64_t steady_ns,
  std::int64_t system_ns)
{
  RecordedMessage m;
  m.seq = next_seq_++;
  m.topic = topic;
  m.bytes = data ? data->size() : 0;
  m.data = std::move(data);
  m.steady_ns = steady_ns;
  m.system_ns = system_ns;
  if (active_) {
    writer_.push(m);
  }
  buffer_.push(std::move(m));
}

std::shared_ptr<EventInfo> EventRecorderCore::onTrigger(
  const Trigger & t, std::int64_t now_steady_ns, std::shared_ptr<EventInfo> make_info)
{
  const auto post_ns = static_cast<std::int64_t>(config_.post_event_s * 1e9);
  const auto max_ns = static_cast<std::int64_t>(config_.max_event_s * 1e9);
  if (active_) {
    // same incident: merge and extend the post window (bounded by max_event_s)
    active_->triggers.push_back(t);
    deadline_ns_ = std::max(deadline_ns_,
        std::min(now_steady_ns + post_ns, active_->start_steady_ns + max_ns));
    ++merged_;
    return nullptr;
  }
  auto info = make_info ? std::move(make_info) : std::make_shared<EventInfo>();
  info->start_steady_ns = t.steady_ns;
  info->start_system_ns = t.system_ns;
  info->triggers = {t};
  info->pre_requested_s = config_.pre_event_s;
  info->post_requested_s = config_.post_event_s;
  info->max_event_s = config_.max_event_s;
  const auto pre_ns = static_cast<std::int64_t>(config_.pre_event_s * 1e9);
  const auto snapshot = buffer_.since(t.steady_ns - pre_ns);
  info->pre_covered_s = snapshot.empty() ? 0.0 :
    std::max(0.0, static_cast<double>(t.steady_ns - snapshot.front().steady_ns) * kNs);
  if (info->pre_covered_s < config_.pre_event_s - 0.15) {
    const bool bytes_limited = buffer_.evictedByBytes() > 0 &&
      buffer_.lastByteEvictionNs() >= t.steady_ns - pre_ns;
    info->pre_limited_by = bytes_limited ? "buffer_bytes" : "history";
  }
  deadline_ns_ = std::min(t.steady_ns + post_ns, t.steady_ns + max_ns);
  writer_.open(info);
  for (const auto & m : snapshot) {
    writer_.push(m);
  }
  active_ = info;
  ++started_;
  return info;
}

std::shared_ptr<EventInfo> EventRecorderCore::tick(std::int64_t now_steady_ns)
{
  if (!active_ || now_steady_ns < deadline_ns_) {
    return nullptr;
  }
  const auto max_ns = static_cast<std::int64_t>(config_.max_event_s * 1e9);
  return close(now_steady_ns,
           deadline_ns_ >= active_->start_steady_ns + max_ns ? "max_duration" : "post_window");
}

std::shared_ptr<EventInfo> EventRecorderCore::finish(
  std::int64_t now_steady_ns, const std::string & reason)
{
  return active_ ? close(now_steady_ns, reason) : nullptr;
}

std::shared_ptr<EventInfo> EventRecorderCore::close(
  std::int64_t now_steady_ns, const std::string & reason)
{
  auto info = active_;
  info->end_steady_ns = now_steady_ns;
  info->end_reason = reason;
  const auto pre_ns = static_cast<std::int64_t>(config_.pre_event_s * 1e9);
  for (const auto & j : jumps_) {
    if (j.steady_ns >= info->start_steady_ns - pre_ns && j.steady_ns <= now_steady_ns) {
      info->jumps.push_back(j);
    }
  }
  writer_.close(info);
  active_.reset();
  return info;
}

void EventRecorderCore::noteJump(const JumpRecord & j)
{
  jumps_.push_back(j);
  while (jumps_.size() > 256) {
    jumps_.pop_front();
  }
}

}  // namespace events
}  // namespace helios_obstacle_detection
