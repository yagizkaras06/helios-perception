#include "helios_obstacle_detection/perception_pipeline.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <stdexcept>

#include <pcl/common/io.h>

namespace helios_obstacle_detection
{

namespace
{
using Clock = std::chrono::steady_clock;
double ms(Clock::time_point a, Clock::time_point b)
{
  return std::chrono::duration<double, std::milli>(b - a).count();
}
}  // namespace

void AdaptiveConfig::validate(float nominal_leaf) const
{
  if (!enabled) {
    return;
  }
  if (!(target_ms > 0.F && leaf_step > 0.F && window >= 1 && max_leaf >= nominal_leaf &&
    nominal_leaf > 0.F))
  {
    throw std::invalid_argument(
            "adaptive: target_ms, leaf_step > 0, window >= 1, max_leaf >= voxel_leaf_size > 0");
  }
}

void PipelineConfig::validate()
{
  filter.validate();
  ground.validate();
  czm.base = ground;
  czm.validate();
  cluster.validate();
  boxes.validate();
  ego.validate();
  // with a background map, STATIC means "part of the static background" only
  tracker.static_requires_background = static_map.enabled;
  tracker.validate();
  static_map.validate();
  risk.validate();
  adaptive.validate(filter.voxel_leaf_size);
  if (ground_method != "czm" && ground_method != "ransac_zones") {
    throw std::invalid_argument("ground.method must be 'czm' or 'ransac_zones'");
  }
  if (threads < 0 || threads > 64) {
    throw std::invalid_argument("threads must be in [0, 64] (0 = auto)");
  }
  const float max_leaf = adaptive.enabled ? adaptive.max_leaf : filter.voxel_leaf_size;
  if (cluster.tolerance < 1.5F * max_leaf) {
    throw std::invalid_argument(
            "clustering.tolerance must be >= 1.5 x the (maximum adaptive) voxel leaf size");
  }
}

PerceptionPipeline::PerceptionPipeline(const PipelineConfig & config)
: config_(config)
{
  config_.validate();
  pool_ = std::make_unique<ThreadPool>(static_cast<std::size_t>(config_.threads));
  processor_ = std::make_unique<CloudProcessor>(config_.filter);
  if (config_.ground_method == "czm") {
    ground_ = std::make_unique<CzmGroundSegmenter>(config_.czm, pool_.get());
  } else {
    ground_ = std::make_unique<GroundSegmenter>(config_.ground);
  }
  clusterer_ = std::make_unique<ObstacleClusterer>(config_.cluster);
  ego_ = std::make_unique<EgoMotionEstimator>(config_.ego);
  if (config_.static_map.enabled) {
    static_map_ = std::make_unique<StaticBackgroundMap>(config_.static_map);
  }
  tracker_ = std::make_unique<MultiObjectTracker>(config_.tracker);
  voxel_leaf_ = config_.filter.voxel_leaf_size;
}

void PerceptionPipeline::reset()
{
  exclusions_.clear();
  if (static_map_) {
    static_map_->reset();
  }
  ground_->reset();
  ego_->reset();
  tracker_->reset();
  load_sum_ms_ = 0.0;
  load_frames_ = 0;
}

bool PerceptionPipeline::process(
  const sensor_msgs::msg::PointCloud2 & msg, double stamp, FrameOutput & out, std::string & error)
{
  const auto t0 = Clock::now();
  auto roi = std::make_shared<Cloud>();
  FilterStats fstats;
  if (!processor_->convertAndFilter(msg, *roi, fstats, error)) {
    return false;
  }
  const auto t1 = Clock::now();
  processFiltered(roi, stamp, out);
  out.filter = fstats;
  out.times.roi_ms = ms(t0, t1);
  out.times.total_ms += out.times.roi_ms;
  return true;
}

void PerceptionPipeline::processFiltered(const CloudPtr & roi, double stamp, FrameOutput & out)
{
  const auto t0 = Clock::now();
  out = FrameOutput{};
  out.stamp = stamp;
  out.roi_cloud = roi;
  out.filter.kept = roi->size();

  // ---- voxel grid -----------------------------------------------------------------------------
  out.voxel_leaf = voxel_leaf_;
  out.voxel_cloud = processor_->voxelDownsample(roi, voxel_leaf_);
  const auto t1 = Clock::now();

  // ---- ground -----------------------------------------------------------------------------------
  ground_->segment(out.voxel_cloud, out.ground);
  out.obstacle_cloud = std::make_shared<Cloud>();
  pcl::copyPointCloud(*out.voxel_cloud, out.ground.obstacle, *out.obstacle_cloud);
  out.obstacle_cloud->header = out.voxel_cloud->header;
  const auto t2 = Clock::now();

  CloudConstPtr obstacle_cloud = out.obstacle_cloud;
  Clock::time_point t3, t4;
  auto fixedEgo = [&]() {
      if (config_.ego_mode == EgoMode::kExternal) {
        out.ego = external_ego_;
      } else {
        out.ego.state = EgoState::kStationary;
        out.ego.valid = true;
        out.ego.measured = false;
        out.ego.velocity_cov = Eigen::Matrix3d::Identity() * 1e-4;
        out.ego.delta_cov = Eigen::Matrix3d::Identity() * 1e-6;
        out.ego.reason = "assumed stationary (configuration)";
      }
    };
  const bool estimate_ego = config_.ego_mode == EgoMode::kEstimate;

  if (static_map_) {
    // ---- ego-motion first (the background map lives in the odometry frame) -------------------
    if (estimate_ego) {
      const auto a = Clock::now();
      out.ego = ego_->update(*obstacle_cloud, stamp, exclusions_);
      out.times.ego_ms = ms(a, Clock::now());
    } else {
      fixedEgo();
    }
    // ---- background / foreground split, separate clustering ----------------------------------
    pcl::Indices bg, fg;
    const bool usable = out.ego.valid && out.ego.state != EgoState::kUnknown;
    if (usable) {
      static_map_->processFrame(*obstacle_cloud, out.ego.pose, stamp, bg, fg);
    } else {
      fg.resize(obstacle_cloud->size());
      for (std::size_t i = 0; i < fg.size(); ++i) {
        fg[i] = static_cast<pcl::index_t>(i);
      }
    }
    out.background_points = bg.size();
    out.static_voxels = static_map_->staticVoxelCount();
    out.background_learning = usable &&
      static_map_->learningAge(stamp) < static_map_->config().static_time_s;
    auto clusterSubset = [this, &obstacle_cloud](const pcl::Indices & subset) {
        std::vector<pcl::PointIndices> res;
        if (subset.empty()) {
          return res;
        }
        auto sub = std::make_shared<Cloud>();
        pcl::copyPointCloud(*obstacle_cloud, subset, *sub);
        res = clusterer_->extract(sub);
        for (auto & c : res) {
          for (auto & i : c.indices) {
            i = subset[static_cast<std::size_t>(i)];   // back to obstacle-cloud indices
          }
        }
        return res;
      };
    // Separate clustering only with a still sensor: while driving, newly revealed faces of a
    // static object are not background yet and would split it into two objects.
    const bool split = out.ego.state == EgoState::kStationary;
    if (split) {
      auto bg_future = pool_->submit([&clusterSubset, &bg] {return clusterSubset(bg);});
      std::vector<pcl::PointIndices> fg_clusters;
      try {
        fg_clusters = clusterSubset(fg);
      } catch (...) {
        bg_future.wait();
        throw;
      }
      out.clusters = bg_future.get();
      out.clusters.insert(out.clusters.end(), std::make_move_iterator(fg_clusters.begin()),
        std::make_move_iterator(fg_clusters.end()));
    } else {
      out.clusters = clusterer_->extract(obstacle_cloud);
    }
    t3 = Clock::now();
    out.obstacles = buildObstacles(*obstacle_cloud, out.clusters, config_.boxes, pool_.get());
    std::vector<std::uint8_t> is_bg(obstacle_cloud->size(), 0);
    for (const auto i : bg) {
      is_bg[static_cast<std::size_t>(i)] = 1;
    }
    for (auto & o : out.obstacles) {
      std::size_t n_bg = 0;
      for (const auto i : *o.indices) {
        n_bg += is_bg[static_cast<std::size_t>(i)];
      }
      o.background = !o.indices->empty() && 2 * n_bg >= o.indices->size();   // majority
    }
    t4 = Clock::now();
  } else {
    // ---- ego-motion (concurrent) | clustering + boxes -----------------------------------------
    std::future<std::pair<EgoMotion, double>> ego_future;
    if (estimate_ego) {
      ego_future = pool_->submit(
        [this, obstacle_cloud, stamp, zones = exclusions_] {
          const auto a = Clock::now();
          EgoMotion e = ego_->update(*obstacle_cloud, stamp, zones);
          return std::make_pair(std::move(e), ms(a, Clock::now()));
        });
    }
    try {
      out.clusters = clusterer_->extract(obstacle_cloud);
      t3 = Clock::now();
      out.obstacles = buildObstacles(*obstacle_cloud, out.clusters, config_.boxes, pool_.get());
      t4 = Clock::now();
    } catch (...) {
      if (ego_future.valid()) {
        ego_future.wait();  // never leave the estimator running into the next frame
      }
      throw;
    }
    if (estimate_ego) {
      auto r = ego_future.get();
      out.ego = std::move(r.first);
      out.times.ego_ms = r.second;
    } else {
      fixedEgo();
    }
  }
  const auto t5 = Clock::now();

  // ---- tracking + risk ----------------------------------------------------------------------------
  if (config_.tracker.enabled) {
    tracker_->update(stamp, out.obstacles, out.ego, out.tracks, out.obstacle_cloud.get());
    out.tracker_stats = tracker_->stats();
    out.risks.reserve(out.tracks.size());
    for (std::size_t i = 0; i < out.tracks.size(); ++i) {
      out.risks.push_back(assessRisk(out.tracks[i], config_.risk));
      if (out.tracks[i].state != TrackState::kTentative &&
        (out.max_risk_track < 0 || out.risks[i].level > out.max_risk))
      {
        out.max_risk = out.risks[i].level;
        out.max_risk_track = static_cast<int>(i);
      }
    }
  }
  // exclusion zones for the next registration: where moving objects will be (constant
  // relative velocity over one nominal period)
  exclusions_.clear();
  for (const auto & t : out.tracks) {
    if (t.state == TrackState::kTentative ||
      !(t.motion == MotionState::kMoving || t.ever_moved))
    {
      continue;
    }
    constexpr float kDt = 0.1F;
    ExclusionZone z;
    z.center = t.center.head<2>() + t.relative_velocity * kDt;
    z.radius = 0.5F * t.size.head<2>().norm() + 0.3F + t.relative_velocity.norm() * kDt;
    exclusions_.push_back(z);
  }
  const auto t6 = Clock::now();

  out.times.voxel_ms = ms(t0, t1);
  out.times.ground_ms = ms(t1, t2);
  out.times.clustering_ms = ms(t2, t3);
  out.times.boxes_ms = ms(t3, t4);
  out.times.tracking_ms = ms(t5, t6);
  out.times.total_ms = ms(t0, t6);
  adaptLoad(out.times.total_ms);
}

void PerceptionPipeline::adaptLoad(double total_ms)
{
  if (!config_.adaptive.enabled) {
    return;
  }
  load_sum_ms_ += total_ms;
  if (++load_frames_ < config_.adaptive.window) {
    return;
  }
  const double mean = load_sum_ms_ / load_frames_;
  load_sum_ms_ = 0.0;
  load_frames_ = 0;
  const float nominal = config_.filter.voxel_leaf_size;
  if (mean > config_.adaptive.target_ms) {
    voxel_leaf_ = std::min(voxel_leaf_ + config_.adaptive.leaf_step, config_.adaptive.max_leaf);
  } else if (mean < 0.6 * config_.adaptive.target_ms && voxel_leaf_ > nominal) {
    voxel_leaf_ = std::max(voxel_leaf_ - config_.adaptive.leaf_step, nominal);
  }
}

}  // namespace helios_obstacle_detection
