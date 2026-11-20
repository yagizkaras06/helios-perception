#include "helios_obstacle_detection/tracking_metrics.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <limits>
#include <numeric>
#include <sstream>

#include "helios_obstacle_detection/hungarian.hpp"

namespace helios_obstacle_detection
{
namespace sim
{

MotEvaluator::MotEvaluator(double match_threshold, int min_hits, double ttc_horizon)
: threshold_(match_threshold), min_hits_(min_hits), ttc_horizon_(ttc_horizon) {}

void MotEvaluator::addFrame(
  const std::vector<GroundTruth> & gt_all, const std::vector<TrackOutput> & tracks,
  const std::vector<RiskAssessment> & risks, double time)
{
  if (time < 0.0) {
    time = 0.1 * static_cast<double>(s_.frames);
  }
  ++s_.frames;
  // evaluated hypotheses: confirmed / coasting tracks
  std::vector<std::size_t> hyp;
  for (std::size_t k = 0; k < tracks.size(); ++k) {
    if (tracks[k].state != TrackState::kTentative) {
      hyp.push_back(k);
    }
  }
  std::vector<GroundTruth> gt;
  std::vector<GroundTruth> dont_care;
  for (const auto & g : gt_all) {
    (g.evaluate ? gt : dont_care).push_back(g);
  }
  const std::size_t ng = gt.size(), nh = hyp.size();
  constexpr double kInf = 1e12;
  std::vector<double> cost(ng * nh, kInf);
  for (std::size_t i = 0; i < ng; ++i) {
    const double thr = threshold_ + 0.25 * std::hypot(gt[i].size.x(), gt[i].size.y());
    for (std::size_t j = 0; j < nh; ++j) {
      const auto & t = tracks[hyp[j]];
      const double d = (gt[i].position - Eigen::Vector2d(t.center.x(), t.center.y())).norm();
      if (d <= thr) {
        // keep previous correspondences (CLEAR-MOT): strongly prefer the last matched track
        const auto it = last_match_.find(gt[i].id);
        const bool same = it != last_match_.end() && it->second == t.id;
        cost[i * nh + j] = same ? d - 1000.0 : d;
      }
    }
  }
  const std::vector<int> assign = solveAssignment(cost, ng, nh, kInf);
  std::vector<char> hyp_used(nh, 0);

  constexpr double kOnsetStill = 1.0;   // [s] standing before an onset event
  constexpr double kMinMove = 1.0;      // [s] moving before a stop event
  constexpr double kMaxLatency = 3.0;   // [s]
  for (std::size_t i = 0; i < ng; ++i) {
    const GroundTruth & g = gt[i];
    const bool visible = g.hits >= min_hits_;
    const int j = assign[i];
    const TrackOutput * mt = j >= 0 ? &tracks[hyp[static_cast<std::size_t>(j)]] : nullptr;
    // ---- motion onset / stop events --------------------------------------------------------
    GtHistory & h = hist_[g.id];
    if (!h.initialised) {
      h.initialised = true;
      h.moving = g.moving;
      h.ever_moved = g.moving;
      h.since = time;
    } else if (g.moving != h.moving) {
      if (g.moving && time - h.since >= kOnsetStill) {
        h.onset = time;
        ++s_.onset_events;
      }
      if (g.moving && h.stop >= 0.0) {          // moving again before the stop was seen
        ++s_.stop_missed;
        h.stop = -1.0;
      }
      if (!g.moving && h.onset >= 0.0) {        // stopped again before MOVING was reported
        ++s_.onset_missed;
        h.onset = -1.0;
      }
      if (!g.moving && time - h.since >= kMinMove) {
        h.stop = time;
        ++s_.stop_events;
      }
      h.moving = g.moving;
      h.ever_moved = h.ever_moved || g.moving;
      h.since = time;
    }
    if (h.onset >= 0.0) {
      if (mt != nullptr && mt->motion == MotionState::kMoving) {
        s_.onset_latency_s.push_back(time - h.onset);
        static const bool dbg = std::getenv("HELIOS_FP_DEBUG") != nullptr;
        if (dbg) {
          std::fprintf(stderr, "ONSET gt=%d t=%.1f latency=%.0f ms range=%.1f speed=%.2f\n", g.id,
            h.onset, 1000.0 * (time - h.onset), g.position.norm(), g.velocity.norm());
        }
        h.onset = -1.0;
      } else if (time - h.onset > kMaxLatency) {
        static const bool dbg2 = std::getenv("HELIOS_FP_DEBUG") != nullptr;
        if (dbg2) {
          std::fprintf(stderr, "ONSET gt=%d t=%.1f MISSED range=%.1f speed=%.2f\n", g.id, h.onset,
            g.position.norm(), g.velocity.norm());
        }
        ++s_.onset_missed;
        h.onset = -1.0;
      }
    }
    if (h.stop >= 0.0) {
      if (mt != nullptr && mt->motion != MotionState::kMoving) {
        s_.stop_latency_s.push_back(time - h.stop);
        h.stop = -1.0;
      } else if (time - h.stop > kMaxLatency) {
        ++s_.stop_missed;
        h.stop = -1.0;
      }
    }
    // ---- fragmentation -----------------------------------------------------------------------
    if (visible) {
      if (j >= 0) {
        if (h.interrupted) {
          ++s_.fragmentations;
          h.interrupted = false;
        }
        h.tracked_once = true;
      } else if (h.tracked_once) {
        h.interrupted = true;
      }
    }
    if (visible && mt != nullptr && !h.ever_moved) {
      ++s_.never_moved_frames;
      s_.never_moved_as_moving += mt->motion == MotionState::kMoving;
      static const bool dbg = std::getenv("HELIOS_FP_DEBUG") != nullptr;
      if (dbg && mt->motion == MotionState::kMoving) {
        std::fprintf(stderr, "FALSEMOV t=%.1f gt=%d trk=%u %s upd=%d d=%.2f win=%.2f+-%.2f surf=%.2f "
          "size=(%.2f,%.2f) gtpos=(%.2f,%.2f)\n", time, g.id, mt->id, toString(mt->state),
          mt->updated ? 1 : 0, (g.position - Eigen::Vector2d(mt->center.x(), mt->center.y())).norm(),
          mt->window_speed, mt->window_speed_sigma, mt->surface_speed, mt->size.x(), mt->size.y(),
          g.position.x(), g.position.y());
      }
    }
    if (j < 0) {
      if (visible) {
        ++s_.gt_visible;
        ++s_.fn;
        ++s_.fn_by_gt[g.id];
      }
      continue;
    }
    hyp_used[static_cast<std::size_t>(j)] = 1;
    if (!visible) {
      continue;  // matched to an invisible object: neither TP nor FP ("don't care")
    }
    ++s_.gt_visible;
    ++s_.tp;
    const std::size_t k = hyp[static_cast<std::size_t>(j)];
    const TrackOutput & t = tracks[k];
    const auto it = last_match_.find(g.id);
    if (it != last_match_.end() && it->second != t.id) {
      ++s_.idsw;
      ++s_.idsw_by_gt[g.id];
      static const bool dbg = std::getenv("HELIOS_FP_DEBUG") != nullptr;
      if (dbg) {
        std::fprintf(stderr, "IDSW t=%.1f gt=%d %u -> %u (%s %s) d=%.2f\n", time, g.id,
          it->second, t.id, toString(t.state), toString(t.motion),
          (g.position - Eigen::Vector2d(t.center.x(), t.center.y())).norm());
      }
    }
    last_match_[g.id] = t.id;
    ++co_[{g.id, t.id}];
    motp_sum_ += (g.position - Eigen::Vector2d(t.center.x(), t.center.y())).norm();

    if (t.velocity_reliable) {
      const Eigen::Vector2d v(t.velocity.x(), t.velocity.y());
      vel_sq_sum_ += (v - g.velocity).squaredNorm();
      speed_abs_sum_ += std::fabs(v.norm() - g.velocity.norm());
      ++s_.velocity_samples;
      if (g.moving && t.heading_valid && g.velocity.norm() > 0.3) {
        double dh = std::atan2(v.y(), v.x()) - std::atan2(g.velocity.y(), g.velocity.x());
        dh = std::atan2(std::sin(dh), std::cos(dh));
        heading_abs_sum_ += std::fabs(dh) * 180.0 / M_PI;
        ++s_.heading_samples;
      }
      if (g.moving) {
        if (t.motion == MotionState::kMoving) {
          ++s_.moving_as_moving;
        } else if (t.motion == MotionState::kStatic) {
          ++s_.moving_as_static;
          ++s_.moving_as_static_by_gt[g.id];
        } else {
          ++s_.moving_as_unknown;
        }
      } else {
        if (t.motion == MotionState::kStatic) {
          ++s_.static_as_static;
        } else if (t.motion == MotionState::kMoving) {
          ++s_.static_as_moving;
          ++s_.static_as_moving_by_gt[g.id];
        } else {
          ++s_.static_as_unknown;
        }
      }
      const RiskAssessment & r = risks[k];
      if (std::isfinite(g.ttc) && g.ttc < ttc_horizon_ && r.ttc_valid && std::isfinite(r.ttc)) {
        ttc_abs_sum_ += std::fabs(r.ttc - g.ttc);
        if (r.ttc_pessimistic > g.ttc) {
          ++ttc_violations_;
        }
        ++s_.ttc_samples;
      }
    }
  }
  // unmatched hypotheses inside a "don't care" object (walls, clutter) are not false positives
  auto inside = [](const GroundTruth & g, const Eigen::Vector2d & p, double margin) {
      const Eigen::Vector2d d = p - g.position;
      const double c = std::cos(g.yaw), s = std::sin(g.yaw);
      const double lx = c * d.x() + s * d.y(), ly = -s * d.x() + c * d.y();
      return std::fabs(lx) <= 0.5 * g.size.x() + margin && std::fabs(ly) <= 0.5 * g.size.y() + margin;
    };
  for (std::size_t j = 0; j < nh; ++j) {
    if (hyp_used[j]) {
      continue;
    }
    const auto & t = tracks[hyp[j]];
    const Eigen::Vector2d p(t.center.x(), t.center.y());
    // walls / buildings (footprint > large_structure_size) are never ground-truth objects
    bool ignore = t.large_structure;
    for (const auto & g : dont_care) {
      ignore = ignore || inside(g, p, 1.5);
    }
    if (!ignore) {
      ++s_.fp;
      static const bool dbg = std::getenv("HELIOS_FP_DEBUG") != nullptr;
      if (dbg) {
        std::fprintf(stderr, "FP t=%.1f id=%u state=%s motion=%s c=(%.2f,%.2f) size=(%.2f,%.2f) pts=%u upd=%d\n",
          time, t.id, toString(t.state), toString(t.motion), t.center.x(), t.center.y(),
          t.size.x(), t.size.y(), t.point_count, t.updated ? 1 : 0);
      }
    }
  }
}

MotSummary MotEvaluator::summary() const
{
  MotSummary s = s_;
  const auto gt = static_cast<double>(std::max<std::uint64_t>(s.gt_visible, 1));
  s.mota = 1.0 - static_cast<double>(s.fn + s.fp + s.idsw) / gt;
  s.motp_m = s.tp ? motp_sum_ / static_cast<double>(s.tp) : 0.0;
  s.precision = (s.tp + s.fp) ? static_cast<double>(s.tp) / static_cast<double>(s.tp + s.fp) : 0;
  s.recall = static_cast<double>(s.tp) / gt;
  // IDF1: optimal one-to-one GT id <-> track id assignment maximising co-occurring TP frames
  {
    std::vector<int> gids;
    std::vector<std::uint32_t> tids;
    for (const auto & kv : co_) {
      if (std::find(gids.begin(), gids.end(), kv.first.first) == gids.end()) {
        gids.push_back(kv.first.first);
      }
      if (std::find(tids.begin(), tids.end(), kv.first.second) == tids.end()) {
        tids.push_back(kv.first.second);
      }
    }
    constexpr double kInf = 1e12;
    std::vector<double> cost(gids.size() * tids.size(), kInf);
    for (const auto & kv : co_) {
      const auto gi = static_cast<std::size_t>(
        std::find(gids.begin(), gids.end(), kv.first.first) - gids.begin());
      const auto ti = static_cast<std::size_t>(
        std::find(tids.begin(), tids.end(), kv.first.second) - tids.begin());
      cost[gi * tids.size() + ti] = -static_cast<double>(kv.second);
    }
    const auto a = solveAssignment(cost, gids.size(), tids.size(), kInf);
    s.idtp = 0;
    for (std::size_t gi = 0; gi < gids.size(); ++gi) {
      if (a[gi] >= 0) {
        s.idtp += co_.at({gids[gi], tids[static_cast<std::size_t>(a[gi])]});
      }
    }
    const double denom = static_cast<double>(s.gt_visible + s.tp + s.fp);
    s.idf1 = denom > 0.0 ? 2.0 * static_cast<double>(s.idtp) / denom : 0.0;
  }
  s.false_moving_rate = s.never_moved_frames ?
    static_cast<double>(s.never_moved_as_moving) / static_cast<double>(s.never_moved_frames) : 0.0;
  if (s.velocity_samples) {
    s.velocity_rmse = std::sqrt(vel_sq_sum_ / static_cast<double>(s.velocity_samples));
    s.speed_mae = speed_abs_sum_ / static_cast<double>(s.velocity_samples);
  }
  if (s.heading_samples) {
    s.heading_mae_deg = heading_abs_sum_ / static_cast<double>(s.heading_samples);
  }
  if (s.ttc_samples) {
    s.ttc_mae_s = ttc_abs_sum_ / static_cast<double>(s.ttc_samples);
    s.ttc_pessimistic_violations =
      static_cast<double>(ttc_violations_) / static_cast<double>(s.ttc_samples);
  }
  return s;
}

double MotSummary::median(std::vector<double> v)
{
  if (v.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::sort(v.begin(), v.end());
  return v[v.size() / 2];
}

std::string MotSummary::toKeyValues() const
{
  char b[640];
  std::snprintf(b, sizeof(b),
    "frames=%lu gt=%lu tp=%lu fp=%lu fn=%lu idsw=%lu frag=%lu mota=%.4f idf1=%.4f "
    "precision=%.4f recall=%.4f heading_mae=%.2f never_moved_frames=%lu never_moved_as_moving=%lu "
    "onsets=%lu onset_detected=%zu onset_missed=%lu onset_median_ms=%.0f onset_mean_ms=%.0f "
    "stops=%lu stop_median_ms=%.0f mov_as_mov=%lu mov_as_static=%lu mov_as_unknown=%lu",
    static_cast<unsigned long>(frames), static_cast<unsigned long>(gt_visible),
    static_cast<unsigned long>(tp), static_cast<unsigned long>(fp), static_cast<unsigned long>(fn),
    static_cast<unsigned long>(idsw), static_cast<unsigned long>(fragmentations), mota, idf1,
    precision, recall, heading_mae_deg, static_cast<unsigned long>(never_moved_frames),
    static_cast<unsigned long>(never_moved_as_moving), static_cast<unsigned long>(onset_events),
    onset_latency_s.size(), static_cast<unsigned long>(onset_missed),
    1000.0 * median(onset_latency_s),
    onset_latency_s.empty() ? std::numeric_limits<double>::quiet_NaN() :
    1000.0 * std::accumulate(onset_latency_s.begin(), onset_latency_s.end(), 0.0) /
    static_cast<double>(onset_latency_s.size()),
    static_cast<unsigned long>(stop_events), 1000.0 * median(stop_latency_s),
    static_cast<unsigned long>(moving_as_moving), static_cast<unsigned long>(moving_as_static),
    static_cast<unsigned long>(moving_as_unknown));
  return b;
}

std::string MotSummary::toMarkdown(const std::string & title) const
{
  std::ostringstream os;
  char b[512];
  os << "### " << title << "\n\n| metric | value |\n|---|---|\n";
  std::snprintf(b, sizeof(b),
    "| frames | %lu |\n| visible GT object-frames | %lu |\n| TP / FP / FN / ID switches | "
    "%lu / %lu / %lu / %lu |\n| MOTA | %.3f |\n| MOTP (centre error) | %.3f m |\n"
    "| precision / recall | %.3f / %.3f |\n",
    static_cast<unsigned long>(frames), static_cast<unsigned long>(gt_visible),
    static_cast<unsigned long>(tp), static_cast<unsigned long>(fp),
    static_cast<unsigned long>(fn), static_cast<unsigned long>(idsw), mota, motp_m, precision,
    recall);
  os << b;
  std::snprintf(b, sizeof(b),
    "| velocity RMSE (vector) | %.3f m/s (%lu samples) |\n| speed MAE | %.3f m/s |\n"
    "| heading MAE (moving) | %.1f deg (%lu samples) |\n",
    velocity_rmse, static_cast<unsigned long>(velocity_samples), speed_mae, heading_mae_deg,
    static_cast<unsigned long>(heading_samples));
  os << b;
  std::snprintf(b, sizeof(b),
    "| moving GT -> MOVING / STATIC / UNKNOWN | %lu / %lu / %lu |\n"
    "| static GT -> STATIC / MOVING / UNKNOWN | %lu / %lu / %lu |\n"
    "| TTC MAE | %.3f s (%lu samples) |\n| pessimistic TTC > true TTC | %.1f %% |\n",
    static_cast<unsigned long>(moving_as_moving), static_cast<unsigned long>(moving_as_static),
    static_cast<unsigned long>(moving_as_unknown), static_cast<unsigned long>(static_as_static),
    static_cast<unsigned long>(static_as_moving), static_cast<unsigned long>(static_as_unknown),
    ttc_mae_s, static_cast<unsigned long>(ttc_samples), 100.0 * ttc_pessimistic_violations);
  os << b;
  auto dump = [&os](const char * name, const std::map<int, int> & m) {
      if (m.empty()) {
        return;
      }
      os << "| " << name << " (gt id: frames) |";
      for (const auto & kv : m) {
        os << " " << kv.first << ":" << kv.second;
      }
      os << " |\n";
    };
  std::snprintf(b, sizeof(b),
    "| IDF1 / fragmentations | %.3f / %lu |\n"
    "| never-moved objects shown MOVING | %lu / %lu frames (%.2f %%) |\n"
    "| motion onsets detected / missed (of) | %zu / %lu (%lu) |\n"
    "| onset latency median / mean | %.0f / %.0f ms |\n"
    "| stops: latency median (events) | %.0f ms (%lu) |\n",
    idf1, static_cast<unsigned long>(fragmentations),
    static_cast<unsigned long>(never_moved_as_moving), static_cast<unsigned long>(never_moved_frames),
    100.0 * false_moving_rate, onset_latency_s.size(), static_cast<unsigned long>(onset_missed),
    static_cast<unsigned long>(onset_events), 1000.0 * median(onset_latency_s),
    onset_latency_s.empty() ? 0.0 : 1000.0 * std::accumulate(onset_latency_s.begin(),
    onset_latency_s.end(), 0.0) / static_cast<double>(onset_latency_s.size()),
    1000.0 * median(stop_latency_s), static_cast<unsigned long>(stop_events));
  os << b;
  dump("static GT shown MOVING", static_as_moving_by_gt);
  dump("moving GT shown STATIC", moving_as_static_by_gt);
  dump("ID switches", idsw_by_gt);
  dump("missed (FN)", fn_by_gt);
  return os.str();
}

}  // namespace sim
}  // namespace helios_obstacle_detection
