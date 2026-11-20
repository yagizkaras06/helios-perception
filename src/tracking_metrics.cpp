#include "helios_obstacle_detection/tracking_metrics.hpp"

#include <cmath>
#include <cstdio>
#include <limits>
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
  const std::vector<RiskAssessment> & risks)
{
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

  for (std::size_t i = 0; i < ng; ++i) {
    const GroundTruth & g = gt[i];
    const bool visible = g.hits >= min_hits_;
    const int j = assign[i];
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
    }
    last_match_[g.id] = t.id;
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
    bool ignore = false;
    for (const auto & g : dont_care) {
      ignore = ignore || inside(g, p, 1.0);
    }
    if (!ignore) {
      ++s_.fp;
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
  dump("static GT shown MOVING", static_as_moving_by_gt);
  dump("moving GT shown STATIC", moving_as_static_by_gt);
  dump("ID switches", idsw_by_gt);
  dump("missed (FN)", fn_by_gt);
  return os.str();
}

}  // namespace sim
}  // namespace helios_obstacle_detection
