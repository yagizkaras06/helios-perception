// Offline benchmark / evaluation tool (no ROS graph needed).
//
//   helios_benchmark synthetic [--scenario all|crossing|ego_motion|approach] [--duration 10]
//   helios_benchmark bag <bag_dir> [--topic /rslidar_points] [--csv frames.csv]
//   common: [--ground czm|ransac_zones] [--ego estimate|assume_stationary|oracle] [--threads N]
//           [--out report.md] [--repeat N]
//
// synthetic: ray-cast Helios-32-like scans with ground truth -> CLEAR-MOT metrics, velocity /
//            heading / TTC errors, moving-static confusion, ego-motion error, timings.
// bag      : runs the identical pipeline over a recording as fast as possible -> per-stage
//            timings, throughput, CPU time per frame, memory, ground / ego / tracking statistics.
// Parameters mirror config/detection.yaml defaults (see benchConfig()).

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "helios_obstacle_detection/lidar_simulator.hpp"
#include "helios_obstacle_detection/perception_pipeline.hpp"
#include "helios_obstacle_detection/resource_monitor.hpp"
#include "helios_obstacle_detection/tracking_metrics.hpp"

using namespace helios_obstacle_detection;

namespace
{

struct Args
{
  std::string mode;
  std::string bag;
  std::string topic{"/rslidar_points"};
  std::string scenario{"all"};
  std::string ground{"czm"};
  std::string ego{"estimate"};
  std::string out;
  std::string csv;
  std::string tracks_csv;
  double duration{10.0};
  int threads{2};
  int repeat{1};
  int trace_gt{-1};
  double static_time{10.0};
  double motion_window{-1.0};
  int hysteresis{-1};
  int min_samples{-1};
  double min_disp{-1.0};
  double moving_speed{-1.0};
  bool no_static_map{false};
  int dump_frame{-1};
  std::string dump_prefix;
  unsigned seed{7};
  double onset_sigma{-1.0};
  double onset_disp{-1.0};
  int onset_samples{-1};
  int no_onset{0};              // scenario layout / timing (start_stop) and sensor noise
};

/// Debug dump of one processed frame: points with labels + tracks (CSV, for plotting).
void dumpFrame(const FrameOutput & out, const std::string & prefix)
{
  std::ofstream pts(prefix + "_points.csv");
  pts << "x,y,z,label,cluster\n";   // label: 0 ground, 1 obstacle, 2 other (ignored high/low)
  std::vector<int> label(out.voxel_cloud->size(), 2);
  for (const auto i : out.ground.ground) {
    label[static_cast<std::size_t>(i)] = 0;
  }
  for (const auto i : out.ground.obstacle) {
    label[static_cast<std::size_t>(i)] = 1;
  }
  for (std::size_t i = 0; i < out.voxel_cloud->size(); ++i) {
    const auto & p = (*out.voxel_cloud)[i];
    pts << p.x << ',' << p.y << ',' << p.z << ',' << label[i] << ",-1\n";
  }
  for (std::size_t j = 0; j < out.obstacles.size(); ++j) {
    for (const auto i : *out.obstacles[j].indices) {
      const auto & p = (*out.obstacle_cloud)[static_cast<std::size_t>(i)];
      pts << p.x << ',' << p.y << ',' << p.z << ",3," << j << '\n';
    }
  }
  std::ofstream trk(prefix + "_tracks.csv");
  trk << "id,state,motion,cx,cy,cz,l,w,h,yaw,vx,vy,closest_x,closest_y,closest_d,risk,ttc,"
    "det\n";
  for (std::size_t k = 0; k < out.tracks.size(); ++k) {
    const auto & t = out.tracks[k];
    trk << t.id << ',' << toString(t.state) << ',' << toString(t.motion) << ',' << t.center.x() <<
      ',' << t.center.y() << ',' << t.center.z() << ',' << t.size.x() << ',' << t.size.y() << ',' <<
      t.size.z() << ',' << t.yaw << ',' << t.velocity.x() << ',' << t.velocity.y() << ',' <<
      t.closest_point.x() << ',' << t.closest_point.y() << ',' << t.closest_distance << ',' <<
      toString(out.risks[k].level) << ',' << out.risks[k].ttc << ',' << t.detection_index << '\n';
  }
}

PipelineConfig benchConfig(const Args & a)
{
  PipelineConfig pc;   // struct defaults == detection.yaml unless set here
  pc.filter.min_range = 0.5F;
  pc.filter.max_range = 30.F;
  pc.filter.roi = AxisBox{-30.F, 30.F, -30.F, 30.F, -2.5F, 2.5F};
  pc.filter.voxel_leaf_size = 0.10F;
  pc.ground.expected_height = -0.90F;
  pc.ground.zone_boundaries = {0.F, 8.F, 16.F, 30.F};
  pc.ground_method = a.ground;
  pc.ego_mode = a.ego == "assume_stationary" ? EgoMode::kAssumeStationary :
    (a.ego == "oracle" ? EgoMode::kExternal : EgoMode::kEstimate);
  pc.threads = a.threads;
  if (a.motion_window > 0) {
    pc.tracker.displacement_window_s = static_cast<float>(a.motion_window);
  }
  if (a.hysteresis > 0) {
    pc.tracker.motion_hysteresis_frames = a.hysteresis;
  }
  if (a.min_samples > 0) {
    pc.tracker.min_motion_samples = a.min_samples;
  }
  if (a.moving_speed > 0) {
    pc.tracker.moving_speed = static_cast<float>(a.moving_speed);
  }
  if (a.min_disp >= 0) {
    pc.tracker.moving_min_displacement = static_cast<float>(a.min_disp);
  }
  if (a.onset_sigma > 0) {
    pc.tracker.onset_sigma_factor = static_cast<float>(a.onset_sigma);
  }
  if (a.onset_disp >= 0) {
    pc.tracker.onset_min_displacement = static_cast<float>(a.onset_disp);
  }
  if (a.onset_samples > 0) {
    pc.tracker.onset_min_samples = a.onset_samples;
  }
  if (a.no_onset) {
    pc.tracker.onset_detection = false;
  }
  pc.static_map.enabled = !a.no_static_map;
  pc.static_map.static_time_s = static_cast<float>(a.static_time);
  return pc;
}

struct Stat
{
  std::vector<double> v;
  void add(double x) {v.push_back(x);}
  double mean() const
  {
    return v.empty() ? 0.0 : std::accumulate(v.begin(), v.end(), 0.0) / static_cast<double>(v.size());
  }
  double pct(double p) const
  {
    if (v.empty()) {
      return 0.0;
    }
    std::vector<double> s = v;
    std::sort(s.begin(), s.end());
    const auto k = std::min(s.size() - 1, static_cast<std::size_t>(std::ceil(p / 100.0 * s.size())) -
        (p > 0 ? 1 : 0));
    return s[k];
  }
  double max() const {return v.empty() ? 0.0 : *std::max_element(v.begin(), v.end());}
};

struct TimingTable
{
  std::map<std::string, Stat> st;
  std::vector<std::string> order{"roi_filter", "voxel_grid", "ground_segmentation", "clustering",
    "bounding_box", "ego_motion(concurrent)", "tracking_risk", "total"};
  void add(const StageTimes & t)
  {
    st["roi_filter"].add(t.roi_ms);
    st["voxel_grid"].add(t.voxel_ms);
    st["ground_segmentation"].add(t.ground_ms);
    st["clustering"].add(t.clustering_ms);
    st["bounding_box"].add(t.boxes_ms);
    st["ego_motion(concurrent)"].add(t.ego_ms);
    st["tracking_risk"].add(t.tracking_ms);
    st["total"].add(t.total_ms);
  }
  std::string markdown() const
  {
    std::ostringstream os;
    os << "| stage | mean ms | p95 ms | max ms |\n|---|---|---|---|\n";
    char b[160];
    for (const auto & n : order) {
      const auto it = st.find(n);
      if (it == st.end()) {
        continue;
      }
      std::snprintf(b, sizeof(b), "| %s | %.2f | %.2f | %.2f |\n", n.c_str(), it->second.mean(),
        it->second.pct(95), it->second.max());
      os << b;
    }
    return os.str();
  }
};

std::string runSynthetic(const Args & a, const std::string & name, bool & pass)
{
  PipelineConfig pc = benchConfig(a);
  PerceptionPipeline pipe(pc);
  sim::SimConfig sc;
  sc.seed = 42U + a.seed;
  sim::LidarSimulator lidar(sc);
  sim::Scene scene = sim::makeScenario(name, a.seed);
  sim::MotEvaluator eval(1.0, 10, pc.risk.horizon_s);
  TimingTable timing;
  Stat ego_err, ego_yaw_err;
  std::map<EgoState, int> ego_states;
  const double dt = 0.1;
  const int frames = static_cast<int>(a.duration / dt);
  const double t0 = 1000.0;
  Eigen::Vector3d prev_pose = scene.ego_pose;
  for (int k = 0; k < frames; ++k) {
    std::vector<int> hits;
    auto msg = lidar.scan(scene, hits);
    const double stamp = t0 + scene.time;
    if (pc.ego_mode == EgoMode::kExternal) {
      // ground-truth ego motion (upper bound for the tracker / classifier)
      EgoMotion e;
      const Eigen::Vector3d & p = scene.ego_pose;
      const double c = std::cos(prev_pose[2]), s = std::sin(prev_pose[2]);
      const Eigen::Vector2d d = p.head<2>() - prev_pose.head<2>();
      e.delta = Eigen::Vector3d(c * d.x() + s * d.y(), -s * d.x() + c * d.y(), p[2] - prev_pose[2]);
      e.pose = p;
      e.velocity = Eigen::Vector2d(scene.ego_speed, 0.0);
      e.yaw_rate = scene.ego_yaw_rate;
      e.valid = e.measured = true;
      e.state = scene.ego_speed > 0.05 ? EgoState::kMoving : EgoState::kStationary;
      e.velocity_cov = Eigen::Matrix3d::Identity() * 1e-4;
      e.delta_cov = Eigen::Matrix3d::Identity() * 1e-6;
      pipe.setExternalEgo(e);
      prev_pose = p;
    }
    FrameOutput out;
    std::string err;
    if (!pipe.process(msg, stamp, out, err)) {
      throw std::runtime_error(err);
    }
    timing.add(out.times);
    const auto gt = lidar.groundTruth(scene, hits, pc.risk.ego_radius, pc.risk.min_closing_speed);
    if (k + 1 == a.dump_frame) {
      dumpFrame(out, a.dump_prefix);
      std::ofstream g(a.dump_prefix + "_gt.csv");
      g << "id,x,y,l,w,yaw,hits\n";
      for (const auto & x : gt) {
        g << x.id << ',' << x.position.x() << ',' << x.position.y() << ',' << x.size.x() << ',' <<
          x.size.y() << ',' << x.yaw << ',' << x.hits << '\n';
      }
      std::cerr << "ground: " << out.ground.detail << "\n";
    }
    if (a.trace_gt >= 0) {
      for (const auto & g : gt) {
        if (g.id != a.trace_gt) {
          continue;
        }
        const TrackOutput * best = nullptr;
        double bd = 1e9;
        for (const auto & t : out.tracks) {
          const double d = (g.position - Eigen::Vector2d(t.center.x(), t.center.y())).norm();
          if (d < bd) {
            bd = d;
            best = &t;
          }
        }
        std::printf("t=%.1f gt%d pos=(%.2f,%.2f) v=(%.2f,%.2f) hits=%d", scene.time, g.id,
          g.position.x(), g.position.y(), g.velocity.x(), g.velocity.y(), g.hits);
        if (best) {
          std::printf(" | trk%u %s %s d=%.2f c=(%.2f,%.2f) sz=(%.2f,%.2f) yaw=%.2f v=(%.2f,%.2f)"
            "+-%.2f win=%.2f+-%.2f surf=%.2f pts=%u upd=%d", best->id, toString(best->state),
            toString(best->motion), bd, best->center.x(), best->center.y(), best->size.x(),
            best->size.y(), best->yaw, best->velocity.x(), best->velocity.y(), best->speed_sigma,
            best->window_speed, best->window_speed_sigma, best->surface_speed, best->point_count,
            best->updated ? 1 : 0);
        }
        std::printf(" | ego %s v=(%.2f,%.2f) w=%.3f inl=%.2f\n", toString(out.ego.state),
          out.ego.velocity.x(), out.ego.velocity.y(), out.ego.yaw_rate, out.ego.inlier_ratio);
      }
    }
    if (k >= 5) {   // allow the filters to initialise
      eval.addFrame(gt, out.tracks, out.risks, scene.time);
      ++ego_states[out.ego.state];
      if (out.ego.valid) {
        ego_err.add((out.ego.velocity - Eigen::Vector2d(scene.ego_speed, 0.0)).norm());
        ego_yaw_err.add(std::fabs(out.ego.yaw_rate - scene.ego_yaw_rate));
      }
    }
    scene.step(dt);
  }
  const auto s = eval.summary();
  std::ostringstream os;
  os << s.toMarkdown("Scenario `" + name + "` (" + std::to_string(frames) + " frames, ground " +
      pc.ground_method + ", ego " + a.ego + ")");
  char b[256];
  std::snprintf(b, sizeof(b),
    "| ego velocity error mean / p95 | %.3f / %.3f m/s |\n| ego yaw-rate error mean | %.4f rad/s |\n"
    "| ego states STATIONARY / MOVING / UNRELIABLE / UNKNOWN | %d / %d / %d / %d |\n\n",
    ego_err.mean(), ego_err.pct(95), ego_yaw_err.mean(), ego_states[EgoState::kStationary],
    ego_states[EgoState::kMoving], ego_states[EgoState::kUnreliable],
    ego_states[EgoState::kUnknown]);
  os << b << "Timing (synthetic scans, " << pipe.pool().size() + 1 << " threads):\n\n" <<
    timing.markdown() << "\n";
  pass = pass && s.mota > 0.6;
  std::printf("METRICS scenario=%s seed=%u %s\n", name.c_str(), a.seed, s.toKeyValues().c_str());
  return os.str();
}

std::string runBag(const Args & a)
{
  rosbag2_cpp::Reader reader;
  rosbag2_storage::StorageOptions so;
  so.uri = a.bag;
  reader.open(so);
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> ser;
  std::vector<sensor_msgs::msg::PointCloud2> msgs;
  while (reader.has_next()) {
    auto bm = reader.read_next();
    if (bm->topic_name != a.topic) {
      continue;
    }
    rclcpp::SerializedMessage sm(*bm->serialized_data);
    sensor_msgs::msg::PointCloud2 m;
    ser.deserialize_message(&sm, &m);
    msgs.push_back(std::move(m));
  }
  if (msgs.empty()) {
    throw std::runtime_error("no messages on " + a.topic);
  }

  PipelineConfig pc = benchConfig(a);
  PerceptionPipeline pipe(pc);
  TimingTable timing;
  Stat points_in, points_voxel, points_obst, clusters, tracks_conf, tracks_moving, ego_speed;
  Stat static_speed, ground_valid;
  std::map<GroundStatus, int> ground_states;
  std::map<EgoState, int> ego_states;
  std::map<RiskLevel, int> risk_counts;
  std::map<std::uint32_t, std::pair<double, double>> track_life;   // id -> first, last stamp
  std::map<std::uint32_t, double> moving_max_speed;
  std::map<std::uint32_t, double> moving_min_range;
  std::ofstream csv;
  if (!a.csv.empty()) {
    csv.open(a.csv);
    csv << "frame,stamp,total_ms,ground_ms,cluster_ms,ego_ms,points_voxel,obstacles,tracks,"
      "confirmed,moving,ground_status,ego_state,ego_vx,ego_vy,max_risk\n";
  }
  std::ofstream tcsv;
  if (!a.tracks_csv.empty()) {
    tcsv.open(a.tracks_csv);
    tcsv << "stamp,id,state,motion,bg,updated,x,y,l,w,vx,vy,speed,speed_sigma,win_speed,"
      "surface_speed,heading,heading_valid,points,ttc,risk\n";
  }
  ResourceMonitor res;
  res.sample();
  const auto wall0 = std::chrono::steady_clock::now();
  std::size_t frames = 0;
  double last_stamp = -1.0;
  for (int rep = 0; rep < a.repeat; ++rep) {
    if (rep > 0) {
      pipe.reset();
      last_stamp = -1.0;
    }
    for (const auto & m : msgs) {
      const double stamp = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec;
      if (last_stamp >= 0.0 && stamp <= last_stamp) {
        pipe.reset();
      }
      last_stamp = stamp;
      FrameOutput out;
      std::string err;
      if (!pipe.process(m, stamp, out, err)) {
        throw std::runtime_error(err);
      }
      ++frames;
      if (static_cast<int>(frames) == a.dump_frame) {
        dumpFrame(out, a.dump_prefix);
        std::cerr << "ground: " << out.ground.detail << "\nego: " << toString(out.ego.state) <<
          " " << out.ego.reason << " hyp=" << out.ego.hypothesis << " support icp/zero=" <<
          out.ego.support_icp << "/" << out.ego.support_zero << "\n";
      }
      timing.add(out.times);
      points_in.add(static_cast<double>(out.filter.input - out.filter.non_finite));
      points_voxel.add(static_cast<double>(out.voxel_cloud->size()));
      points_obst.add(static_cast<double>(out.obstacle_cloud->size()));
      clusters.add(static_cast<double>(out.obstacles.size()));
      tracks_conf.add(static_cast<double>(out.tracker_stats.confirmed));
      tracks_moving.add(static_cast<double>(out.tracker_stats.moving));
      ground_valid.add(static_cast<double>(out.ground.planes_valid));
      ++ground_states[out.ground.status];
      ++ego_states[out.ego.state];
      if (out.ego.measured) {
        ego_speed.add(out.ego.velocity.norm());
      }
      ++risk_counts[out.max_risk];
      if (tcsv) {
        for (std::size_t q = 0; q < out.tracks.size(); ++q) {
          const auto & t = out.tracks[q];
          tcsv << std::fixed << stamp << ',' << t.id << ',' << toString(t.state) << ',' <<
            toString(t.motion) << ',' << t.background << ',' << t.updated << ',' << t.center.x() <<
            ',' << t.center.y() << ',' << t.size.x() << ',' << t.size.y() << ',' <<
            t.velocity.x() << ',' << t.velocity.y() << ',' << t.speed << ',' << t.speed_sigma <<
            ',' << t.window_speed << ',' << t.surface_speed << ',' << t.heading << ',' <<
            t.heading_valid << ',' << t.point_count << ',' << out.risks[q].ttc << ',' <<
            toString(out.risks[q].level) << '\n';
        }
      }
      for (const auto & t : out.tracks) {
        if (t.state == TrackState::kTentative) {
          continue;
        }
        auto it = track_life.find(t.id);
        if (it == track_life.end()) {
          track_life[t.id] = {stamp - t.age_s, stamp};
        } else {
          it->second.second = stamp;
        }
        if (t.motion == MotionState::kStatic) {
          static_speed.add(t.speed);
        }
        if (t.motion == MotionState::kMoving) {
          moving_max_speed[t.id] = std::max(moving_max_speed[t.id], static_cast<double>(t.speed));
          const auto r = moving_min_range.find(t.id);
          moving_min_range[t.id] = r == moving_min_range.end() ?
            t.closest_distance : std::min(r->second, static_cast<double>(t.closest_distance));
        }
      }
      if (csv) {
        csv << frames << ',' << std::fixed << stamp << ',' << out.times.total_ms << ',' <<
          out.times.ground_ms << ',' << out.times.clustering_ms << ',' << out.times.ego_ms << ',' <<
          out.voxel_cloud->size() << ',' << out.obstacles.size() << ',' << out.tracks.size() <<
          ',' << out.tracker_stats.confirmed << ',' << out.tracker_stats.moving << ',' <<
          toString(out.ground.status) << ',' << toString(out.ego.state) << ',' <<
          out.ego.velocity.x() << ',' << out.ego.velocity.y() << ',' << toString(out.max_risk) <<
          '\n';
      }
    }
  }
  const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall0).count();
  const ResourceSample rs = res.sample();
  const double cpu_ms_per_frame = rs.cpu_percent / 100.0 * wall * 1000.0 / frames;

  Stat lifetimes;
  for (const auto & kv : track_life) {
    lifetimes.add(kv.second.second - kv.second.first);
  }
  std::ostringstream os;
  char b[512];
  os << "## Bag `" << a.bag << "`\n\n";
  std::snprintf(b, sizeof(b),
    "| item | value |\n|---|---|\n| frames processed | %zu (%zu msgs x %d) |\n"
    "| ground method / ego mode | %s / %s |\n| worker threads | %zu + caller |\n"
    "| offline throughput (max speed) | %.1f frames/s |\n"
    "| CPU time per frame | %.1f ms  (=> ~%.0f %% of one core at 10 Hz) |\n"
    "| peak-ish RSS | %.1f MB |\n",
    frames, msgs.size(), a.repeat, pc.ground_method.c_str(), a.ego.c_str(), pipe.pool().size(),
    frames / wall, cpu_ms_per_frame, cpu_ms_per_frame / 100.0 * 100.0, rs.rss_mb);
  os << b;
  std::snprintf(b, sizeof(b),
    "| points finite / voxel / non-ground (mean) | %.0f / %.0f / %.0f |\n"
    "| clusters per frame mean / max | %.1f / %.0f |\n"
    "| ground RELIABLE / DEGRADED / FAILED | %d / %d / %d |\n"
    "| accepted ground planes per frame (mean) | %.1f |\n",
    points_in.mean(), points_voxel.mean(), points_obst.mean(), clusters.mean(), clusters.max(),
    ground_states[GroundStatus::kReliable], ground_states[GroundStatus::kDegraded],
    ground_states[GroundStatus::kFailed], ground_valid.mean());
  os << b;
  std::snprintf(b, sizeof(b),
    "| ego STATIONARY / MOVING / UNRELIABLE / UNKNOWN | %d / %d / %d / %d |\n"
    "| measured ego speed mean / p95 / max | %.3f / %.3f / %.3f m/s |\n"
    "| confirmed tracks per frame mean / max | %.1f / %.0f |\n"
    "| moving tracks per frame mean / max | %.2f / %.0f |\n"
    "| confirmed track ids / median lifetime / p90 | %zu / %.1f s / %.1f s |\n"
    "| speed of STATIC tracks mean / p95 (noise floor) | %.3f / %.3f m/s |\n",
    ego_states[EgoState::kStationary], ego_states[EgoState::kMoving],
    ego_states[EgoState::kUnreliable], ego_states[EgoState::kUnknown], ego_speed.mean(),
    ego_speed.pct(95), ego_speed.max(), tracks_conf.mean(), tracks_conf.max(),
    tracks_moving.mean(), tracks_moving.max(), track_life.size(), lifetimes.pct(50),
    lifetimes.pct(90), static_speed.mean(), static_speed.pct(95));
  os << b;
  std::snprintf(b, sizeof(b),
    "| frames by max risk NONE/LOW/MEDIUM/HIGH/CRITICAL | %d / %d / %d / %d / %d |\n\n",
    risk_counts[RiskLevel::kNone], risk_counts[RiskLevel::kLow], risk_counts[RiskLevel::kMedium],
    risk_counts[RiskLevel::kHigh], risk_counts[RiskLevel::kCritical]);
  os << b;
  if (!moving_max_speed.empty()) {
    os << "Tracks classified MOVING (id: max speed, min distance):";
    for (const auto & kv : moving_max_speed) {
      std::snprintf(b, sizeof(b), " %u: %.2f m/s @%.1f m;", kv.first, kv.second,
        moving_min_range[kv.first]);
      os << b;
    }
    os << "\n\n";
  }
  os << "Timing (offline, as fast as possible):\n\n" << timing.markdown() << "\n";
  return os.str();
}

void usage()
{
  std::cerr <<
    "usage:\n  helios_benchmark synthetic [--scenario all|tracking|crossing|ego_motion|approach|"
    "occlusion|merge_split|wall_walk|start_stop] [--duration S] [--seed N] [--trace-gt ID]\n  helios_benchmark bag <bag_dir> [--topic T] [--csv file] "
    "[--repeat N] [--dump-frame N PREFIX]\n"
    "  common: [--ground czm|ransac_zones] [--ego estimate|assume_stationary|oracle] [--threads N] "
    "[--out report.md]\n          [--static-time S] [--no-static-map]\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  Args a;
  if (argc < 2) {
    usage();
    return 2;
  }
  a.mode = argv[1];
  int k = 2;
  if (a.mode == "bag") {
    if (argc < 3) {
      usage();
      return 2;
    }
    a.bag = argv[2];
    k = 3;
  }
  for (; k < argc; ++k) {
    const std::string o = argv[k];
    auto next = [&]() -> std::string {
        if (k + 1 >= argc) {
          throw std::invalid_argument("missing value for " + o);
        }
        return argv[++k];
      };
    if (o == "--topic") {
      a.topic = next();
    } else if (o == "--scenario") {
      a.scenario = next();
    } else if (o == "--ground") {
      a.ground = next();
    } else if (o == "--ego") {
      a.ego = next();
    } else if (o == "--out") {
      a.out = next();
    } else if (o == "--tracks-csv") {
      a.tracks_csv = next();
    } else if (o == "--csv") {
      a.csv = next();
    } else if (o == "--duration") {
      a.duration = std::stod(next());
    } else if (o == "--threads") {
      a.threads = std::stoi(next());
    } else if (o == "--dump-frame") {
      a.dump_frame = std::stoi(next());
      a.dump_prefix = next();
    } else if (o == "--motion-window") {
      a.motion_window = std::stod(next());
    } else if (o == "--hysteresis") {
      a.hysteresis = std::stoi(next());
    } else if (o == "--min-samples") {
      a.min_samples = std::stoi(next());
    } else if (o == "--moving-speed") {
      a.moving_speed = std::stod(next());
    } else if (o == "--min-disp") {
      a.min_disp = std::stod(next());
    } else if (o == "--static-time") {
      a.static_time = std::stod(next());
    } else if (o == "--no-static-map") {
      a.no_static_map = true;
    } else if (o == "--trace-gt") {
      a.trace_gt = std::stoi(next());
    } else if (o == "--onset-sigma") {
      a.onset_sigma = std::stod(next());
    } else if (o == "--onset-disp") {
      a.onset_disp = std::stod(next());
    } else if (o == "--onset-samples") {
      a.onset_samples = std::stoi(next());
    } else if (o == "--no-onset") {
      a.no_onset = 1;
    } else if (o == "--seed") {
      a.seed = static_cast<unsigned>(std::stoul(next()));
    } else if (o == "--repeat") {
      a.repeat = std::max(1, std::stoi(next()));
    } else {
      usage();
      return 2;
    }
  }
  try {
    std::string report;
    bool pass = true;
    if (a.mode == "synthetic") {
      std::vector<std::string> names{a.scenario};
      if (a.scenario == "all") {
        names = {"crossing", "ego_motion", "approach"};
      } else if (a.scenario == "tracking") {    // v0.3 occlusion / merge / onset scenarios
        names = {"occlusion", "merge_split", "wall_walk", "start_stop"};
      }
      report = "## Synthetic Helios-32 scenarios (ground truth)\n\n";
      for (const auto & n : names) {
        report += runSynthetic(a, n, pass);
      }
    } else if (a.mode == "bag") {
      report = runBag(a);
    } else {
      usage();
      return 2;
    }
    std::cout << report << std::flush;
    if (!a.out.empty()) {
      std::ofstream(a.out) << report;
    }
    return pass ? 0 : 1;
  } catch (const std::exception & e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
}
