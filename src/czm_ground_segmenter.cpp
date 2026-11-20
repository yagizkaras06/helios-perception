#include "helios_obstacle_detection/czm_ground_segmenter.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <sstream>
#include <stdexcept>

#include <Eigen/Eigenvalues>

namespace helios_obstacle_detection
{

namespace
{
constexpr float kPi = static_cast<float>(M_PI);
constexpr float kTwoPi = 2.F * kPi;
constexpr std::uint8_t kLabelGround = 0;
constexpr std::uint8_t kLabelObstacle = 1;
constexpr std::uint8_t kLabelHigh = 2;
constexpr std::uint8_t kLabelLow = 3;

inline float signedDistance(const Eigen::Vector4f & plane, float x, float y, float z) noexcept
{
  return plane[0] * x + plane[1] * y + plane[2] * z + plane[3];
}

/// PCA of a point subset: centroid, eigenvalues (ascending) and normal (smallest eigenvector).
bool pca(
  const Cloud & cloud, const std::vector<pcl::index_t> & idx, Eigen::Vector3f & centroid,
  Eigen::Vector3f & eigenvalues, Eigen::Vector3f & normal)
{
  if (idx.size() < 3) {
    return false;
  }
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  for (const auto i : idx) {
    const auto & p = cloud[static_cast<std::size_t>(i)];
    sum += Eigen::Vector3d(p.x, p.y, p.z);
  }
  const Eigen::Vector3d mean = sum / static_cast<double>(idx.size());
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  for (const auto i : idx) {
    const auto & p = cloud[static_cast<std::size_t>(i)];
    const Eigen::Vector3d d(p.x - mean.x(), p.y - mean.y(), p.z - mean.z());
    cov.noalias() += d * d.transpose();
  }
  cov /= static_cast<double>(idx.size());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
  if (es.info() != Eigen::Success) {
    return false;
  }
  centroid = mean.cast<float>();
  eigenvalues = es.eigenvalues().cast<float>();
  normal = es.eigenvectors().col(0).cast<float>();
  return true;
}
}  // namespace

const char * toString(BinState s) noexcept
{
  switch (s) {
    case BinState::kEmpty: return "empty";
    case BinState::kSparse: return "sparse";
    case BinState::kAccepted: return "accepted";
    case BinState::kRejectedTilt: return "tilt";
    case BinState::kRejectedRough: return "rough";
    case BinState::kRejectedElevated: return "elevated";
    case BinState::kRejectedDiscontinuous: return "discontinuous";
  }
  return "unknown";
}

void CzmConfig::validate()
{
  base.validate();
  const std::size_t zones = zone_boundaries.size();
  if (zones < 2 || zones > 16) {
    throw std::invalid_argument("czm.zone_boundaries needs 2..16 values");
  }
  if (zone_boundaries.front() < 0.F) {
    throw std::invalid_argument("czm.zone_boundaries must be >= 0");
  }
  for (std::size_t i = 1; i < zones; ++i) {
    if (!(zone_boundaries[i] > zone_boundaries[i - 1])) {
      throw std::invalid_argument("czm.zone_boundaries must be strictly increasing");
    }
  }
  if (rings_per_zone.size() != zones - 1 || sectors_per_zone.size() != zones - 1) {
    throw std::invalid_argument(
            "czm.rings_per_zone / sectors_per_zone need one value per zone (boundaries - 1)");
  }
  for (std::size_t i = 0; i + 1 < zones; ++i) {
    if (rings_per_zone[i] < 1 || rings_per_zone[i] > 32 || sectors_per_zone[i] < 1 ||
      sectors_per_zone[i] > 720)
    {
      throw std::invalid_argument("czm: rings in [1,32], sectors in [1,720] per zone");
    }
  }
  if (min_bin_points < 3 || seed_count < 3 || fit_iterations < 1 || fit_iterations > 10) {
    throw std::invalid_argument(
            "czm: min_bin_points>=3, seed_count>=3, fit_iterations in [1,10]");
  }
  if (!(seed_height > 0.F && plane_distance > 0.F && min_planarity_ratio >= 1.F &&
    min_extent >= 0.F && max_bin_height_jump > 0.F && far_height_tolerance > 0.F &&
    max_corner_height_jump > 0.F))
  {
    throw std::invalid_argument("czm: seed_height, plane_distance, jumps must be > 0");
  }
  if (!(elevation_min > 0.F && elevation_min <= elevation_threshold &&
    elevation_threshold <= elevation_max))
  {
    throw std::invalid_argument("czm: need 0 < elevation_min <= elevation_threshold <= max");
  }
  if (!(flatness_min > 0.F && flatness_min <= flatness_threshold &&
    flatness_threshold <= flatness_max))
  {
    throw std::invalid_argument("czm: need 0 < flatness_min <= flatness_threshold <= max");
  }
  if (!(adaptation_rate > 0.F && adaptation_rate <= 1.F && adaptive_sigma > 0.F)) {
    throw std::invalid_argument("czm: adaptation_rate in (0,1], adaptive_sigma > 0");
  }
  if (elevation_rings < 0 || continuity_rings < 1) {
    throw std::invalid_argument("czm: elevation_rings >= 0, continuity_rings >= 1");
  }
  if (!(degraded_ratio > 0.F && degraded_ratio <= 1.F)) {
    throw std::invalid_argument("czm.degraded_ratio must be in (0,1]");
  }
}

void CzmGroundSegmenter::RunningStat::add(double v, double alpha)
{
  if (!init) {
    mean = v;
    var = 0.0;
    init = true;
    return;
  }
  const double d = v - mean;
  mean += alpha * d;
  var = (1.0 - alpha) * (var + alpha * d * d);
}

CzmGroundSegmenter::CzmGroundSegmenter(const CzmConfig & config, ThreadPool * pool)
: config_(config), pool_(pool)
{
  config_.validate();
  expected_plane_ << config_.base.expected_normal, -config_.base.expected_height;
  cos_max_angle_ = std::cos(config_.base.max_normal_angle_deg * kPi / 180.F);

  const std::size_t zones = config_.zone_boundaries.size() - 1;
  std::size_t bins = 0;
  for (std::size_t z = 0; z < zones; ++z) {
    const float r0 = config_.zone_boundaries[z];
    const float r1 = config_.zone_boundaries[z + 1];
    const int rings = config_.rings_per_zone[z];
    for (int r = 0; r < rings; ++r) {
      ring_inner_.push_back(r0 + (r1 - r0) * static_cast<float>(r) / static_cast<float>(rings));
      ring_outer_.push_back(
        r0 + (r1 - r0) * static_cast<float>(r + 1) / static_cast<float>(rings));
      ring_sectors_.push_back(config_.sectors_per_zone[z]);
      ring_first_bin_.push_back(bins);
      bins += static_cast<std::size_t>(config_.sectors_per_zone[z]);
    }
  }
  total_rings_ = static_cast<int>(ring_inner_.size());
  bin_count_ = bins;
  config_.elevation_rings = std::min(config_.elevation_rings, total_rings_);
  reset();
}

void CzmGroundSegmenter::reset()
{
  const auto n = static_cast<std::size_t>(config_.elevation_rings);
  elev_stats_.assign(n, RunningStat{});
  flat_stats_.assign(n, RunningStat{});
  elev_thr_.assign(n, config_.elevation_threshold);
  flat_thr_.assign(n, config_.flatness_threshold);
}

int CzmGroundSegmenter::binOf(float x, float y, int * ring_out) const noexcept
{
  const float r = std::sqrt(x * x + y * y);
  int ring = total_rings_ - 1;
  for (int i = 0; i < total_rings_; ++i) {
    if (r < ring_outer_[static_cast<std::size_t>(i)]) {
      ring = i;
      break;
    }
  }
  const int sectors = ring_sectors_[static_cast<std::size_t>(ring)];
  const float a = std::atan2(y, x) + kPi;  // [0, 2pi]
  int s = static_cast<int>(a / kTwoPi * static_cast<float>(sectors));
  s = std::clamp(s, 0, sectors - 1);
  *ring_out = ring;
  return static_cast<int>(ring_first_bin_[static_cast<std::size_t>(ring)]) + s;
}

float CzmGroundSegmenter::cornerJump(
  const Eigen::Vector4f & a, const Eigen::Vector4f & b, int ring, int sector) const noexcept
{
  const auto rs = static_cast<std::size_t>(ring);
  const float sectors = static_cast<float>(ring_sectors_[rs]);
  float worst = 0.F;
  for (const float r : {std::max(ring_inner_[rs], 0.5F), ring_outer_[rs]}) {
    for (const float k : {0.F, 1.F}) {
      const float ang = (static_cast<float>(sector) + k) / sectors * kTwoPi - kPi;
      const float x = r * std::cos(ang), y = r * std::sin(ang);
      const float za = -(a[0] * x + a[1] * y + a[3]) / a[2];
      const float zb = -(b[0] * x + b[1] * y + b[3]) / b[2];
      worst = std::max(worst, std::fabs(za - zb));
    }
  }
  return worst;
}

void CzmGroundSegmenter::fitBin(
  const Cloud & cloud, std::size_t begin, std::size_t end, BinFit & fit) const
{
  fit = BinFit{};
  fit.count = static_cast<std::uint32_t>(end - begin);
  if (fit.count < static_cast<std::uint32_t>(config_.min_bin_points)) {
    return;
  }
  thread_local std::vector<std::pair<float, pcl::index_t>> heights;
  thread_local std::vector<pcl::index_t> current;
  heights.clear();
  for (std::size_t k = begin; k < end; ++k) {
    const auto i = sorted_[k];
    const auto & p = cloud[static_cast<std::size_t>(i)];
    const float s = signedDistance(expected_plane_, p.x, p.y, p.z);
    if (s >= -config_.base.candidate_below) {   // deep points: multipath / reflections
      heights.emplace_back(s, i);
    }
  }
  if (heights.size() < static_cast<std::size_t>(config_.min_bin_points)) {
    return;
  }
  // Lowest point representative (LPR): mean height of the lowest `seed_count` points.
  const std::size_t k = std::min<std::size_t>(heights.size(),
      static_cast<std::size_t>(config_.seed_count));
  std::nth_element(heights.begin(), heights.begin() + static_cast<std::ptrdiff_t>(k - 1),
    heights.end(), [](const auto & a, const auto & b) {return a.first < b.first;});
  float lpr = 0.F;
  for (std::size_t j = 0; j < k; ++j) {
    lpr += heights[j].first;
  }
  lpr /= static_cast<float>(k);

  current.clear();
  for (const auto & h : heights) {
    if (h.first < lpr + config_.seed_height) {
      current.push_back(h.second);
    }
  }

  Eigen::Vector3f centroid = Eigen::Vector3f::Zero(), ev = Eigen::Vector3f::Zero();
  Eigen::Vector3f normal = Eigen::Vector3f::UnitZ();
  for (int it = 0; it < config_.fit_iterations; ++it) {
    if (!pca(cloud, current, centroid, ev, normal)) {
      return;
    }
    if (normal.dot(config_.base.expected_normal) < 0.F) {
      normal = -normal;
    }
    const float d = -normal.dot(centroid);
    if (it + 1 == config_.fit_iterations) {
      fit.plane << normal, d;
      break;
    }
    current.clear();
    for (const auto & h : heights) {
      const auto & p = cloud[static_cast<std::size_t>(h.second)];
      const float dist = normal.x() * p.x + normal.y() * p.y + normal.z() * p.z + d;
      if (dist < config_.plane_distance && dist > -config_.base.below_ground_tolerance) {
        current.push_back(h.second);
      }
    }
    if (current.size() < 3) {
      return;
    }
  }
  fit.fitted = true;
  fit.centroid = centroid;
  fit.lambda_min = std::max(ev[0], 0.F);
  fit.lambda_mid = std::max(ev[1], 0.F);
  fit.upright = normal.dot(config_.base.expected_normal);
  fit.elevation = signedDistance(expected_plane_, centroid.x(), centroid.y(), centroid.z());
}

void CzmGroundSegmenter::segment(const CloudConstPtr & cloud_ptr, GroundResult & result)
{
  const Cloud & cloud = *cloud_ptr;
  const std::size_t n = cloud.size();
  result = GroundResult{};
  stats_ = CzmStats{};

  // ---- 1) counting sort of points into bins --------------------------------------------
  point_bin_.resize(n);
  bin_offsets_.assign(bin_count_ + 1, 0U);
  for (std::size_t i = 0; i < n; ++i) {
    int ring = 0;
    const int b = binOf(cloud[i].x, cloud[i].y, &ring);
    point_bin_[i] = static_cast<std::uint32_t>(b);
    ++bin_offsets_[static_cast<std::size_t>(b) + 1];
  }
  std::partial_sum(bin_offsets_.begin(), bin_offsets_.end(), bin_offsets_.begin());
  sorted_.resize(n);
  {
    std::vector<std::uint32_t> cursor(bin_offsets_.begin(), bin_offsets_.end() - 1);
    for (std::size_t i = 0; i < n; ++i) {
      sorted_[cursor[point_bin_[i]]++] = static_cast<pcl::index_t>(i);
    }
  }

  // ---- 2) region-wise plane fitting (parallel) -------------------------------------------
  fits_.resize(bin_count_);
  auto fit_range = [&](std::size_t b0, std::size_t b1) {
      for (std::size_t b = b0; b < b1; ++b) {
        fitBin(cloud, bin_offsets_[b], bin_offsets_[b + 1], fits_[b]);
      }
    };
  if (pool_ != nullptr) {
    pool_->parallelFor(bin_count_, fit_range, 8);
  } else {
    fit_range(0, bin_count_);
  }

  // ---- 3) ground likelihood estimation, inner -> outer ring per sector ---------------------
  states_.assign(bin_count_, BinState::kEmpty);
  const auto near_rings = config_.elevation_rings;
  for (int g = 0; g < total_rings_; ++g) {
    const auto gs = static_cast<std::size_t>(g);
    for (int s = 0; s < ring_sectors_[gs]; ++s) {
      const std::size_t b = ring_first_bin_[gs] + static_cast<std::size_t>(s);
      const BinFit & f = fits_[b];
      if (f.count == 0) {
        continue;
      }
      ++stats_.bins_populated;
      if (!f.fitted || f.lambda_mid < config_.min_planarity_ratio * f.lambda_min ||
        f.lambda_mid < config_.min_extent * config_.min_extent)
      {
        states_[b] = BinState::kSparse;
        continue;
      }
      if (f.upright < cos_max_angle_) {
        states_[b] = BinState::kRejectedTilt;
        continue;
      }
      const float flat_thr = g < near_rings ? flat_thr_[gs] : config_.flatness_max;
      if (f.lambda_min > flat_thr) {
        states_[b] = BinState::kRejectedRough;
        continue;
      }
      // nearest accepted inner bin of the same angular direction (sector counts may differ
      // between zones, so map by bin-centre azimuth)
      const float az = (static_cast<float>(s) + 0.5F) / static_cast<float>(ring_sectors_[gs]);
      int ref = -1;
      for (int gi = g - 1; gi >= 0 && gi >= g - config_.continuity_rings; --gi) {
        const auto gis = static_cast<std::size_t>(gi);
        const auto si = std::min(static_cast<std::size_t>(az * static_cast<float>(
            ring_sectors_[gis])), static_cast<std::size_t>(ring_sectors_[gis] - 1));
        const std::size_t bi = ring_first_bin_[gis] + si;
        if (states_[bi] == BinState::kAccepted) {
          ref = static_cast<int>(bi);
          break;
        }
      }
      bool ok = true;
      BinState fail = BinState::kRejectedDiscontinuous;
      if (g < near_rings && f.elevation > elev_thr_[gs]) {
        ok = false;
        fail = BinState::kRejectedElevated;
      }
      if (ok && ref >= 0) {
        const auto & rp = fits_[static_cast<std::size_t>(ref)].plane;
        const float jump = std::fabs(
          signedDistance(rp, f.centroid.x(), f.centroid.y(), f.centroid.z()));
        ok = jump <= config_.max_bin_height_jump &&
          cornerJump(f.plane, rp, g, s) <= config_.max_corner_height_jump;
      } else if (ok && g >= near_rings) {
        ok = !config_.far_requires_reference &&
          std::fabs(f.elevation) <= config_.far_height_tolerance &&
          cornerJump(f.plane, expected_plane_, g, s) <=
          config_.far_height_tolerance + config_.max_corner_height_jump;
      }
      states_[b] = ok ? BinState::kAccepted : fail;
    }
  }

  // ---- 4) reference plane for every non-accepted bin ---------------------------------------
  reference_bin_.assign(bin_count_, -1);
  for (int g = 0; g < total_rings_; ++g) {
    const auto gs = static_cast<std::size_t>(g);
    const int sectors = ring_sectors_[gs];
    for (int s = 0; s < sectors; ++s) {
      const std::size_t b = ring_first_bin_[gs] + static_cast<std::size_t>(s);
      if (states_[b] == BinState::kEmpty) {
        continue;
      }
      if (states_[b] == BinState::kAccepted) {
        reference_bin_[b] = static_cast<int>(b);
        continue;
      }
      const float az = (static_cast<float>(s) + 0.5F) / static_cast<float>(sectors);
      int best = -1;
      // same azimuth, nearest ring (inner first on ties)
      for (int dg = 1; dg < total_rings_ && best < 0; ++dg) {
        for (int gi : {g - dg, g + dg}) {
          if (gi < 0 || gi >= total_rings_) {
            continue;
          }
          const auto gis = static_cast<std::size_t>(gi);
          const auto si = std::min(static_cast<std::size_t>(az * static_cast<float>(
              ring_sectors_[gis])), static_cast<std::size_t>(ring_sectors_[gis] - 1));
          const std::size_t bi = ring_first_bin_[gis] + si;
          if (states_[bi] == BinState::kAccepted) {
            best = static_cast<int>(bi);
            break;
          }
        }
        if (dg >= 3) {
          break;  // too far along the ray -> try neighbouring sectors
        }
      }
      // same ring, neighbouring sectors
      for (int ds = 1; ds <= 2 && best < 0; ++ds) {
        for (int si : {s - ds, s + ds}) {
          const int wrapped = (si % sectors + sectors) % sectors;
          const std::size_t bi = ring_first_bin_[gs] + static_cast<std::size_t>(wrapped);
          if (states_[bi] == BinState::kAccepted) {
            best = static_cast<int>(bi);
            break;
          }
        }
      }
      reference_bin_[b] = best;
    }
  }

  // ---- 5) point classification (parallel, labels then ordered gather) ----------------------
  labels_.resize(n);
  const GroundConfig & gc = config_.base;
  auto classify = [&](std::size_t b0, std::size_t b1) {
      for (std::size_t b = b0; b < b1; ++b) {
        const int ref = reference_bin_[b];
        const Eigen::Vector4f & pl = ref >= 0 ? fits_[static_cast<std::size_t>(ref)].plane :
          expected_plane_;
        for (std::size_t k = bin_offsets_[b]; k < bin_offsets_[b + 1]; ++k) {
          const auto i = static_cast<std::size_t>(sorted_[k]);
          const auto & p = cloud[i];
          const float h = signedDistance(pl, p.x, p.y, p.z);
          labels_[i] = h > gc.max_obstacle_height ? kLabelHigh :
            (h < -gc.below_ground_tolerance ? kLabelLow :
            (h <= gc.ground_threshold ? kLabelGround : kLabelObstacle));
        }
      }
    };
  if (pool_ != nullptr) {
    pool_->parallelFor(bin_count_, classify, 16);
  } else {
    classify(0, bin_count_);
  }
  result.ground.reserve(n);
  result.obstacle.reserve(n / 2);
  for (std::size_t i = 0; i < n; ++i) {
    switch (labels_[i]) {
      case kLabelGround: result.ground.push_back(static_cast<pcl::index_t>(i)); break;
      case kLabelObstacle: result.obstacle.push_back(static_cast<pcl::index_t>(i)); break;
      case kLabelHigh: ++result.ignored_high; break;
      default: ++result.ignored_low; break;
    }
  }

  // ---- 6) statistics, status and adaptive thresholds ---------------------------------------
  std::size_t accepted = 0;
  for (std::size_t b = 0; b < bin_count_; ++b) {
    const auto st = states_[b];
    ++stats_.bins_by_state[static_cast<std::size_t>(st)];
    const std::size_t cnt = bin_offsets_[b + 1] - bin_offsets_[b];
    if (st == BinState::kAccepted) {
      ++accepted;
      stats_.points_own_plane += cnt;
    } else if (reference_bin_[b] >= 0) {
      stats_.points_reference_plane += cnt;
    } else {
      stats_.points_fallback_plane += cnt;
    }
  }
  stats_.elevation_thresholds = elev_thr_;
  stats_.flatness_thresholds = flat_thr_;
  result.planes_valid = accepted;
  result.planes_total = stats_.bins_populated;
  if (accepted == 0) {
    result.status = GroundStatus::kFailed;
  } else if (static_cast<float>(stats_.points_fallback_plane) >
    config_.degraded_ratio * static_cast<float>(std::max<std::size_t>(n, 1)))
  {
    result.status = GroundStatus::kDegraded;
  } else {
    result.status = GroundStatus::kReliable;
  }

  if (config_.adaptive && result.status != GroundStatus::kFailed) {
    for (int g = 0; g < near_rings; ++g) {
      const auto gs = static_cast<std::size_t>(g);
      for (int s = 0; s < ring_sectors_[gs]; ++s) {
        const std::size_t b = ring_first_bin_[gs] + static_cast<std::size_t>(s);
        if (states_[b] == BinState::kAccepted) {
          elev_stats_[gs].add(fits_[b].elevation, config_.adaptation_rate);
          flat_stats_[gs].add(fits_[b].lambda_min, config_.adaptation_rate);
        }
      }
      if (elev_stats_[gs].init) {
        elev_thr_[gs] = std::clamp(
          static_cast<float>(elev_stats_[gs].mean + config_.adaptive_sigma *
          std::sqrt(elev_stats_[gs].var)), config_.elevation_min, config_.elevation_max);
        flat_thr_[gs] = std::clamp(
          static_cast<float>(flat_stats_[gs].mean + config_.adaptive_sigma *
          std::sqrt(flat_stats_[gs].var)), config_.flatness_min, config_.flatness_max);
      }
    }
  }

  std::ostringstream os;
  os.setf(std::ios::fixed);
  os.precision(2);
  const auto & bs = stats_.bins_by_state;
  const double total_pts = static_cast<double>(std::max<std::size_t>(n, 1));
  os << "czm bins acc=" << bs[2] << "/" << stats_.bins_populated << " tilt=" << bs[3]
     << " rough=" << bs[4] << " elev=" << bs[5] << " disc=" << bs[6] << " sparse=" << bs[1]
     << " | pts own=" << 100.0 * static_cast<double>(stats_.points_own_plane) / total_pts
     << "% ref=" << 100.0 * static_cast<double>(stats_.points_reference_plane) / total_pts
     << "% fb=" << 100.0 * static_cast<double>(stats_.points_fallback_plane) / total_pts
     << "% | elev_thr=";
  for (std::size_t i = 0; i < stats_.elevation_thresholds.size(); ++i) {
    os << (i ? "," : "") << stats_.elevation_thresholds[i];
  }
  result.detail = os.str();
}

}  // namespace helios_obstacle_detection
