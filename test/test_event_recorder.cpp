// Event recorder core: RAM buffer limits, trigger edges / start-up / re-arming, pre / post windows,
// merging, max duration, queue / disk limits, write errors and a rosbag2 round trip (topics,
// contents, header stamps, ordering, sequence continuity, metadata.json).
#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>

#include "helios_obstacle_detection/event_recorder_core.hpp"

using namespace helios_obstacle_detection::events;
namespace fs = std::filesystem;

namespace
{
constexpr std::int64_t kS = 1000000000LL;
constexpr std::int64_t kWall0 = 1700000000LL * kS;   // arbitrary wall time

std::shared_ptr<const rclcpp::SerializedMessage> headerMsg(std::int32_t sec, std::uint32_t nsec,
  const std::string & frame)
{
  std_msgs::msg::Header h;
  h.stamp.sec = sec;
  h.stamp.nanosec = nsec;
  h.frame_id = frame;
  auto out = std::make_shared<rclcpp::SerializedMessage>();
  rclcpp::Serialization<std_msgs::msg::Header>().serialize_message(&h, out.get());
  return out;
}

std::shared_ptr<const rclcpp::SerializedMessage> stringMsg(const std::string & s)
{
  std_msgs::msg::String m;
  m.data = s;
  auto out = std::make_shared<rclcpp::SerializedMessage>();
  rclcpp::Serialization<std_msgs::msg::String>().serialize_message(&m, out.get());
  return out;
}

std::vector<TopicInfo> topics()
{
  return {{"/raw", "std_msgs/msg/Header", rclcpp::QoS(10)},
    {"/status", "std_msgs/msg/String", rclcpp::QoS(10)}};
}

fs::path tmpDir(const std::string & name)
{
  auto d = fs::temp_directory_path() / ("helios_event_test_" + name + "_" +
    std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  fs::create_directories(d);
  return d;
}

std::shared_ptr<EventInfo> info(const fs::path & dir, const std::string & id)
{
  auto i = std::make_shared<EventInfo>();
  i->id = id;
  i->dir = (dir / id).string();
  i->software_version = "test";
  return i;
}

nlohmann::json readMeta(const std::string & dir)
{
  std::ifstream f(fs::path(dir) / "metadata.json");
  return nlohmann::json::parse(f);
}

struct BagMsg
{
  std::string topic;
  std::int64_t ts;
  std::shared_ptr<rclcpp::SerializedMessage> data;
};

std::vector<BagMsg> readBag(const std::string & dir)
{
  rosbag2_cpp::Reader r;
  rosbag2_storage::StorageOptions so;
  so.uri = (fs::path(dir) / "bag").string();
  so.storage_id = "mcap";
  r.open(so, rosbag2_cpp::ConverterOptions{"cdr", "cdr"});
  std::vector<BagMsg> out;
  while (r.has_next()) {
    auto m = r.read_next();
    auto ser = std::make_shared<rclcpp::SerializedMessage>(*m->serialized_data);
    out.push_back({m->topic_name, m->recv_timestamp, ser});
  }
  return out;
}

Trigger trig(TriggerType ty, std::int64_t steady, std::int64_t track = -1)
{
  return {ty, steady, kWall0 + steady, track, "test"};
}
}  // namespace

// ---------------------------------------------------------------------------- buffer limits
TEST(EventBuffer, EvictsByAgeAndBytes)
{
  PreEventBuffer b(2.0, 1000);
  std::uint64_t seq = 1;
  for (int k = 0; k <= 40; ++k) {   // 4 s at 10 Hz, 10 bytes each
    RecordedMessage m;
    m.seq = seq++;
    m.bytes = 10;
    m.steady_ns = k * kS / 10;
    b.push(m);
  }
  EXPECT_NEAR(static_cast<double>(4 * kS - b.oldestNs()) / kS, 2.0, 0.11);   // age limit
  EXPECT_EQ(b.evictedByBytes(), 0u);
  RecordedMessage big;
  big.seq = seq++;
  big.bytes = 950;
  big.steady_ns = 4 * kS + kS / 10;
  b.push(big);                                                               // byte limit
  EXPECT_LE(b.bytes(), 1000u);
  EXPECT_GT(b.evictedByBytes(), 0u);
  EXPECT_GT(b.lastByteEvictionNs(), 0);
  EXPECT_EQ(b.since(4 * kS).size(), 2u);        // frame at 4.0 s + the big one
  EXPECT_EQ(b.since(4 * kS + 1).size(), 1u);
}

// --------------------------------------------------------------------------------- triggers
TEST(EventTriggers, StartUpIsNotAnEvent)
{
  TriggerMonitor m(TriggerConfig{});
  // detector up, no data yet: NO_DATA status and (later) EGO_UNKNOWN / CRITICAL at first message
  EXPECT_TRUE(m.onStatus(0, kWall0, TriggerMonitor::kStatusNoData).empty());
  EXPECT_TRUE(m.onStatus(kS, kWall0 + kS, TriggerMonitor::kStatusNoData).empty());
  EXPECT_TRUE(m.onTracks(2 * kS, kWall0, 0 /*EGO_UNKNOWN*/, 4 /*CRITICAL*/, 7).empty());
  EXPECT_TRUE(m.onTracks(2 * kS + kS / 10, kWall0, 0, 4, 7).empty());
  EXPECT_TRUE(m.onTracks(2 * kS + 2 * kS / 10, kWall0, 1 /*STATIONARY*/, 4, 7).empty());
}

TEST(EventTriggers, NoDataAfterFlowingStreamOnce)
{
  TriggerMonitor m(TriggerConfig{});
  for (int k = 0; k < 20; ++k) {
    m.onTracks(k * kS / 10, kWall0, 1, 0, -1);
  }
  auto t = m.onStatus(3 * kS, kWall0, TriggerMonitor::kStatusNoData);
  ASSERT_EQ(t.size(), 1u);
  EXPECT_EQ(t[0].type, TriggerType::kNoData);
  EXPECT_TRUE(m.onStatus(4 * kS, kWall0, TriggerMonitor::kStatusNoData).empty());   // no repeat
  // resumes briefly (< rearm_s) and drops again: not a new event
  m.onTracks(5 * kS, kWall0, 1, 0, -1);
  EXPECT_TRUE(m.onStatus(5 * kS + kS / 2, kWall0, TriggerMonitor::kStatusNoData).empty());
  // flows for 2 s, then drops: new event
  for (int k = 0; k < 20; ++k) {
    m.onTracks(6 * kS + k * kS / 10, kWall0, 1, 0, -1);
  }
  EXPECT_EQ(m.onStatus(9 * kS, kWall0, TriggerMonitor::kStatusNoData).size(), 1u);
}

TEST(EventTriggers, EgoUnknownOnlyAfterValidEstimate)
{
  TriggerMonitor m(TriggerConfig{});
  m.onTracks(0, kWall0, 0, 0, -1);                          // start-up: unknown
  m.onTracks(kS / 10, kWall0, 1, 0, -1);                    // valid only briefly
  EXPECT_TRUE(m.onTracks(kS / 5, kWall0, 0, 0, -1).empty());   // flicker < rearm_s
  for (int k = 3; k < 25; ++k) {
    m.onTracks(k * kS / 10, kWall0, 2 /*MOVING*/, 0, -1);
  }
  auto t = m.onTracks(25 * kS / 10, kWall0, 0, 0, -1);
  ASSERT_EQ(t.size(), 1u);
  EXPECT_EQ(t[0].type, TriggerType::kEgoUnknown);
  EXPECT_TRUE(m.onTracks(26 * kS / 10, kWall0, 0, 0, -1).empty());   // stays unknown: no repeat
}

TEST(EventTriggers, EgoRestartAfterDataLossIsNotAnEvent)
{
  TriggerMonitor m(TriggerConfig{});
  for (int k = 0; k < 30; ++k) {
    m.onTracks(k * kS / 10, kWall0, 1, 0, -1);
  }
  EXPECT_EQ(m.onStatus(4 * kS, kWall0, TriggerMonitor::kStatusNoData).size(), 1u);
  // data back: the estimator starts UNKNOWN again - part of the recovery, not a new event
  EXPECT_TRUE(m.onTracks(6 * kS, kWall0, 0, 0, -1).empty());
}

TEST(EventTriggers, CriticalRiskEdgeWithRearm)
{
  TriggerMonitor m(TriggerConfig{});
  m.onTracks(0, kWall0, 1, 2, 3);                            // baseline MEDIUM
  auto t = m.onTracks(kS / 10, kWall0, 1, 4, 42);
  ASSERT_EQ(t.size(), 1u);
  EXPECT_EQ(t[0].type, TriggerType::kCriticalRisk);
  EXPECT_EQ(t[0].track_id, 42);
  EXPECT_TRUE(m.onTracks(2 * kS / 10, kWall0, 1, 4, 42).empty());    // still critical
  m.onTracks(3 * kS / 10, kWall0, 1, 3, 42);                          // HIGH for 0.2 s ...
  EXPECT_TRUE(m.onTracks(5 * kS / 10, kWall0, 1, 4, 42).empty());    // ... flicker: no event
  m.onTracks(6 * kS / 10, kWall0, 1, 1, -1);
  EXPECT_EQ(m.onTracks(2 * kS, kWall0, 1, 4, 9).size(), 1u);          // cleared >= 1 s: event
}

TEST(EventTriggers, AlreadyCriticalAtStartNeedsToClearFirst)
{
  TriggerMonitor m(TriggerConfig{});
  m.onTracks(0, kWall0, 1, 4, 1);
  EXPECT_TRUE(m.onTracks(kS, kWall0, 1, 4, 1).empty());
  m.onTracks(2 * kS, kWall0, 1, 0, -1);
  EXPECT_EQ(m.onTracks(4 * kS, kWall0, 1, 4, 5).size(), 1u);
}

// -------------------------------------------------------------- session + writer round trip
TEST(EventSession, PrePostWindowsMergeAndBagRoundTrip)
{
  const auto dir = tmpDir("roundtrip");
  EventWriter w(WriterConfig{}, topics());
  SessionConfig sc;
  sc.pre_event_s = 3.0;
  sc.post_event_s = 2.0;
  sc.max_event_s = 30.0;
  EventRecorderCore core(sc, w);
  std::shared_ptr<EventInfo> closed;
  int raw_sent = 0;
  for (int k = 0; k < 150; ++k) {    // 15 s at 10 Hz; header stamp != receive time (sensor clock)
    const std::int64_t st = k * kS / 10;
    core.onMessage(0, headerMsg(1000 + k / 10, static_cast<std::uint32_t>((k % 10) * 100000000),
      "f" + std::to_string(k)), st, kWall0 + st);
    if (k % 5 == 0) {
      core.onMessage(1, stringMsg("s" + std::to_string(k)), st, kWall0 + st);
    }
    ++raw_sent;
    if (k == 60) {
      ASSERT_NE(core.onTrigger(trig(TriggerType::kCriticalRisk, st, 42), st, info(dir, "ev")), nullptr);
    }
    if (k == 70) {   // second trigger while recording: merged, post window extended to 9 s
      EXPECT_EQ(core.onTrigger(trig(TriggerType::kNoData, st), st, nullptr), nullptr);
    }
    if (auto c = core.tick(st)) {
      closed = c;
    }
  }
  ASSERT_NE(closed, nullptr);
  w.flush();
  EXPECT_EQ(closed->end_reason, "post_window");
  EXPECT_EQ(closed->triggers.size(), 2u);
  EXPECT_NEAR(closed->pre_covered_s, 3.0, 1e-6);
  EXPECT_NEAR(static_cast<double>(closed->end_steady_ns - closed->start_steady_ns) / kS, 3.0, 0.11);

  const auto msgs = readBag(closed->dir);
  int raw = 0, status = 0;
  std::int64_t prev = 0;
  for (const auto & m : msgs) {
    EXPECT_GE(m.ts, prev);   // ordered
    prev = m.ts;
    const double off = static_cast<double>(m.ts - (kWall0 + 6 * kS)) / kS;
    EXPECT_GE(off, -3.0 - 1e-6);
    EXPECT_LE(off, 3.0 + 1e-6);
    if (m.topic == "/raw") {
      std_msgs::msg::Header h;
      rclcpp::Serialization<std_msgs::msg::Header>().deserialize_message(m.data.get(), &h);
      // receive offset -> original frame index; header stamp and content unchanged
      const int k = static_cast<int>(std::lround(off * 10.0)) + 60;
      EXPECT_EQ(h.frame_id, "f" + std::to_string(k));
      EXPECT_EQ(h.stamp.sec, 1000 + k / 10);
      EXPECT_EQ(h.stamp.nanosec, static_cast<std::uint32_t>((k % 10) * 100000000));
      ++raw;
    } else {
      ++status;
    }
  }
  EXPECT_EQ(raw, 61);       // frames 30 .. 90 (pre 3 s + post up to 9 s) - no loss, no duplicate
  EXPECT_EQ(status, 13);    // every 5th frame in 30 .. 90

  const auto meta = readMeta(closed->dir);
  EXPECT_EQ(meta["reasons"].size(), 2u);
  EXPECT_EQ(meta["reasons"][0]["type"], "critical_risk");
  EXPECT_EQ(meta["reasons"][0]["track_id"], 42);
  EXPECT_EQ(meta["reasons"][1]["type"], "no_data");
  EXPECT_NEAR(meta["reasons"][1]["offset_s"].get<double>(), 1.0, 1e-6);
  EXPECT_EQ(meta["integrity"]["sequence_gaps"], 0);
  EXPECT_EQ(meta["integrity"]["out_of_order"], 0);
  EXPECT_EQ(meta["integrity"]["messages_written"], 74);
  EXPECT_NEAR(meta["window"]["pre_covered_s"].get<double>(), 3.0, 1e-6);
  EXPECT_TRUE(meta["window"]["pre_limited_by"].is_null());
  EXPECT_EQ(meta["window"]["end_reason"], "post_window");
  fs::remove_all(dir);
}

TEST(EventSession, MaxDurationAndShortHistory)
{
  const auto dir = tmpDir("maxdur");
  EventWriter w(WriterConfig{}, topics());
  SessionConfig sc;
  sc.pre_event_s = 5.0;
  sc.post_event_s = 2.0;
  sc.max_event_s = 4.0;
  EventRecorderCore core(sc, w);
  std::shared_ptr<EventInfo> closed;
  for (int k = 0; k < 120 && !closed; ++k) {
    const std::int64_t st = k * kS / 10;
    core.onMessage(0, headerMsg(k, 0, "x"), st, kWall0 + st);
    if (k == 10) {   // only 1 s of history exists
      core.onTrigger(trig(TriggerType::kNoData, st), st, info(dir, "maxdur"));
    } else if (k > 10 && k % 10 == 0) {   // keeps re-triggering: bounded by max_event_s
      core.onTrigger(trig(TriggerType::kCriticalRisk, st), st, nullptr);
    }
    closed = core.tick(st);
  }
  ASSERT_NE(closed, nullptr);
  w.flush();
  EXPECT_EQ(closed->end_reason, "max_duration");
  EXPECT_NEAR(static_cast<double>(closed->end_steady_ns - closed->start_steady_ns) / kS, 4.0, 0.11);
  EXPECT_NEAR(closed->pre_covered_s, 1.0, 1e-6);
  EXPECT_EQ(closed->pre_limited_by, "history");
  const auto meta = readMeta(closed->dir);
  EXPECT_NEAR(meta["window"]["pre_covered_s"].get<double>(), 1.0, 1e-6);
  EXPECT_EQ(meta["window"]["pre_limited_by"], "history");
  fs::remove_all(dir);
}

TEST(EventSession, ByteLimitedBufferReportsCoverage)
{
  const auto dir = tmpDir("bytes");
  EventWriter w(WriterConfig{}, topics());
  SessionConfig sc;
  sc.pre_event_s = 5.0;
  sc.post_event_s = 0.5;
  // room for ~2 s of 10 Hz messages
  const std::size_t one = headerMsg(1, 0, std::string(200, 'p'))->size();
  sc.buffer_max_bytes = 20 * one;
  EventRecorderCore core(sc, w);
  std::shared_ptr<EventInfo> closed;
  for (int k = 0; k < 120 && !closed; ++k) {
    const std::int64_t st = k * kS / 10;
    core.onMessage(0, headerMsg(k, 0, std::string(200, 'p')), st, kWall0 + st);
    if (k == 80) {
      core.onTrigger(trig(TriggerType::kCriticalRisk, st), st, info(dir, "bytes"));
    }
    closed = core.tick(st);
  }
  ASSERT_NE(closed, nullptr);
  w.flush();
  EXPECT_LT(closed->pre_covered_s, 2.1);
  EXPECT_EQ(closed->pre_limited_by, "buffer_bytes");
  EXPECT_EQ(readMeta(closed->dir)["window"]["pre_limited_by"], "buffer_bytes");
  fs::remove_all(dir);
}

TEST(EventWriter, QueueLimitDiskBudgetAndWriteErrors)
{
  const auto dir = tmpDir("limits");
  {   // a message larger than the whole queue is dropped and shows up as a sequence gap
    WriterConfig wc;
    wc.queue_max_bytes = 4096;
    EventWriter w(wc, topics());
    SessionConfig sc;
    sc.pre_event_s = 1.0;
    sc.post_event_s = 1.0;
    EventRecorderCore core(sc, w);
    std::shared_ptr<EventInfo> closed;
    for (int k = 0; k < 40 && !closed; ++k) {
      const std::int64_t st = k * kS / 10;
      const bool huge = k == 15;
      core.onMessage(1, stringMsg(std::string(huge ? 10000 : 10, 'x')), st, kWall0 + st);
      if (k == 10) {
        core.onTrigger(trig(TriggerType::kNoData, st), st, info(dir, "queue"));
      }
      closed = core.tick(st);
    }
    ASSERT_NE(closed, nullptr);
    w.flush();
    EXPECT_EQ(w.status().dropped_queue_total, 1u);
    EXPECT_EQ(readMeta(closed->dir)["integrity"]["sequence_gaps"], 1);
  }
  {   // disk budget: the event is truncated, not the process
    EventWriter w(WriterConfig{}, topics());
    SessionConfig sc;
    sc.pre_event_s = 1.0;
    sc.post_event_s = 2.0;
    EventRecorderCore core(sc, w);
    std::shared_ptr<EventInfo> closed;
    for (int k = 0; k < 60 && !closed; ++k) {
      const std::int64_t st = k * kS / 10;
      core.onMessage(1, stringMsg(std::string(1000, 'y')), st, kWall0 + st);
      if (k == 20) {
        auto i = info(dir, "budget");
        i->disk_budget_bytes = 10 * 1000;
        core.onTrigger(trig(TriggerType::kCriticalRisk, st), st, i);
      }
      closed = core.tick(st);
    }
    ASSERT_NE(closed, nullptr);
    w.flush();
    const auto meta = readMeta(closed->dir);
    EXPECT_TRUE(meta["integrity"]["truncated_by_disk_budget"].get<bool>());
    EXPECT_GT(meta["integrity"]["messages_skipped_after_truncation"].get<int>(), 0);
    EXPECT_LE(meta["integrity"]["bytes_written"].get<std::uint64_t>(), 10000u);
    EXPECT_EQ(w.status().truncated_events, 1u);
  }
  {   // unwritable target: reported as a write error, no exception escapes
    EventWriter w(WriterConfig{}, topics());
    SessionConfig sc;
    sc.pre_event_s = 0.5;
    sc.post_event_s = 0.5;
    EventRecorderCore core(sc, w);
    auto i = std::make_shared<EventInfo>();
    i->id = "denied";
    i->dir = "/proc/helios_no_permission/denied";
    for (int k = 0; k < 20; ++k) {
      const std::int64_t st = k * kS / 10;
      core.onMessage(1, stringMsg("z"), st, kWall0 + st);
      if (k == 5) {
        core.onTrigger(trig(TriggerType::kNoData, st), st, i);
      }
      core.tick(st);
    }
    w.flush();
    EXPECT_GE(w.status().write_errors, 1u);
    EXPECT_FALSE(w.status().last_error.empty());
    EXPECT_EQ(w.status().events_written, 0u);
  }
  fs::remove_all(dir);
}
