#include "helios_obstacle_detection/marker_builder.hpp"

#include <array>
#include <cmath>
#include <cstdio>

#include <Eigen/Eigenvalues>

#include <geometry_msgs/msg/point.hpp>

namespace helios_obstacle_detection
{

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

namespace
{

constexpr std::array<Rgb, 12> kPalette{{
  {0.90F, 0.10F, 0.10F}, {0.10F, 0.60F, 1.00F}, {0.20F, 0.85F, 0.20F}, {1.00F, 0.60F, 0.00F},
  {0.70F, 0.30F, 1.00F}, {0.00F, 0.90F, 0.90F}, {1.00F, 0.90F, 0.10F}, {1.00F, 0.40F, 0.70F},
  {0.55F, 0.80F, 0.30F}, {0.30F, 0.40F, 0.95F}, {0.95F, 0.55F, 0.40F}, {0.60F, 0.60F, 0.60F},
}};

geometry_msgs::msg::Point toPoint(float x, float y, float z)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

Marker makeMarker(
  const std_msgs::msg::Header & header, const char * ns, std::int32_t id, std::int32_t type,
  const builtin_interfaces::msg::Duration & lifetime)
{
  Marker m;
  m.header = header;
  m.ns = ns;
  m.id = id;
  m.type = type;
  m.action = Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.lifetime = lifetime;
  m.frame_locked = false;
  return m;
}

}  // namespace

Rgb colorForId(std::uint32_t id) noexcept
{
  return kPalette[id % kPalette.size()];
}

MarkerArray buildClearMarkers(const std_msgs::msg::Header & header)
{
  MarkerArray arr;
  Marker del;
  del.header = header;
  del.action = Marker::DELETEALL;
  arr.markers.push_back(del);
  return arr;
}

MarkerArray buildObstacleMarkers(
  const std_msgs::msg::Header & header, const std::vector<ObstacleCandidate> & obstacles,
  GroundStatus ground_status, const builtin_interfaces::msg::Duration & lifetime)
{
  MarkerArray arr = buildClearMarkers(header);
  arr.markers.reserve(1 + obstacles.size() * 3 + 1);

  for (const auto & o : obstacles) {
    const Rgb c = colorForId(o.id);
    const auto id = static_cast<std::int32_t>(o.id);

    // Wireframe AABB (12 edges)
    Marker box = makeMarker(header, "boxes", id, Marker::LINE_LIST, lifetime);
    box.scale.x = 0.03;
    box.color.r = c.r;
    box.color.g = c.g;
    box.color.b = c.b;
    box.color.a = 1.0F;
    const Eigen::Vector3f & a = o.min_pt;
    const Eigen::Vector3f & b = o.max_pt;
    const std::array<geometry_msgs::msg::Point, 8> v{{
      toPoint(a.x(), a.y(), a.z()), toPoint(b.x(), a.y(), a.z()),
      toPoint(b.x(), b.y(), a.z()), toPoint(a.x(), b.y(), a.z()),
      toPoint(a.x(), a.y(), b.z()), toPoint(b.x(), a.y(), b.z()),
      toPoint(b.x(), b.y(), b.z()), toPoint(a.x(), b.y(), b.z())}};
    constexpr std::array<std::array<int, 2>, 12> edges{{
      {0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4},
      {0, 4}, {1, 5}, {2, 6}, {3, 7}}};
    box.points.reserve(24);
    for (const auto & e : edges) {
      box.points.push_back(v[static_cast<std::size_t>(e[0])]);
      box.points.push_back(v[static_cast<std::size_t>(e[1])]);
    }
    arr.markers.push_back(std::move(box));

    // Center sphere
    Marker center = makeMarker(header, "centers", id, Marker::SPHERE, lifetime);
    center.pose.position = toPoint(o.center.x(), o.center.y(), o.center.z());
    center.scale.x = center.scale.y = center.scale.z = 0.12;
    center.color.r = c.r;
    center.color.g = c.g;
    center.color.b = c.b;
    center.color.a = 0.9F;
    arr.markers.push_back(std::move(center));

    // Distance label above the box (ASCII only: RViz font lacks Turkish glyphs)
    Marker text = makeMarker(header, "distances", id, Marker::TEXT_VIEW_FACING, lifetime);
    text.pose.position = toPoint(o.center.x(), o.center.y(), o.max_pt.z() + 0.45F);
    text.scale.z = 0.22;
    text.color.r = text.color.g = text.color.b = 1.0F;
    text.color.a = 1.0F;
    char buf[96];
    // Two short lines; RViz renders spaces very wide, so keep them to a minimum.
    // d = closest point distance, c = AABB center distance (both from the LiDAR origin).
    std::snprintf(
      buf, sizeof(buf), "#%u%s\nd=%.2fm\nc=%.2fm", o.id, o.large_structure ? "(L)" : "",
      o.closest_distance, o.center_distance);
    text.text = buf;
    arr.markers.push_back(std::move(text));
  }

  if (ground_status != GroundStatus::kReliable) {
    Marker warn = makeMarker(header, "status", 0, Marker::TEXT_VIEW_FACING, lifetime);
    warn.pose.position = toPoint(0.F, 0.F, 1.5F);
    warn.scale.z = 0.35;
    warn.color.r = 1.0F;
    warn.color.g = ground_status == GroundStatus::kFailed ? 0.1F : 0.7F;
    warn.color.b = 0.1F;
    warn.color.a = 1.0F;
    warn.text = std::string("GROUND ") + toString(ground_status);
    arr.markers.push_back(std::move(warn));
  }
  return arr;
}

}  // namespace helios_obstacle_detection

// ================================================================================ tracks
namespace helios_obstacle_detection
{

namespace
{
void setColor(Marker & m, const Rgb & c, float a)
{
  m.color.r = c.r;
  m.color.g = c.g;
  m.color.b = c.b;
  m.color.a = a;
}

void setYaw(Marker & m, float yaw)
{
  m.pose.orientation.x = 0.0;
  m.pose.orientation.y = 0.0;
  m.pose.orientation.z = std::sin(0.5 * yaw);
  m.pose.orientation.w = std::cos(0.5 * yaw);
}

void addOrientedBoxEdges(Marker & m, const Eigen::Vector3f & c, const Eigen::Vector3f & s, float yaw)
{
  const float cy = std::cos(yaw), sy = std::sin(yaw);
  const float hx = 0.5F * s.x(), hy = 0.5F * s.y();
  const float z0 = c.z() - 0.5F * s.z(), z1 = c.z() + 0.5F * s.z();
  std::array<geometry_msgs::msg::Point, 8> v;
  const std::array<std::array<float, 2>, 4> corners{{{hx, hy}, {-hx, hy}, {-hx, -hy}, {hx, -hy}}};
  for (std::size_t k = 0; k < 4; ++k) {
    const float x = c.x() + cy * corners[k][0] - sy * corners[k][1];
    const float y = c.y() + sy * corners[k][0] + cy * corners[k][1];
    v[k] = toPoint(x, y, z0);
    v[k + 4] = toPoint(x, y, z1);
  }
  constexpr std::array<std::array<int, 2>, 12> edges{{
    {0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4},
    {0, 4}, {1, 5}, {2, 6}, {3, 7}}};
  m.points.reserve(24 + 2);
  for (const auto & e : edges) {
    m.points.push_back(v[static_cast<std::size_t>(e[0])]);
    m.points.push_back(v[static_cast<std::size_t>(e[1])]);
  }
  // heading tick on the front face (+x of the box)
  const float fx = c.x() + cy * hx, fy = c.y() + sy * hx;
  m.points.push_back(toPoint(c.x(), c.y(), z1));
  m.points.push_back(toPoint(fx, fy, z1));
}

const char * motionTag(MotionState m) noexcept
{
  switch (m) {
    case MotionState::kMoving: return "MOV";
    case MotionState::kStatic: return "STAT";
    default: return "?";
  }
}
}  // namespace

Rgb colorForRisk(RiskLevel level) noexcept
{
  switch (level) {
    case RiskLevel::kCritical: return {1.0F, 0.0F, 0.0F};
    case RiskLevel::kHigh: return {1.0F, 0.45F, 0.0F};
    case RiskLevel::kMedium: return {1.0F, 0.85F, 0.0F};
    case RiskLevel::kLow: return {0.55F, 0.9F, 0.2F};
    default: return {0.2F, 0.8F, 1.0F};
  }
}

MarkerArray buildTrackMarkers(
  const std_msgs::msg::Header & header, const FrameOutput & frame, const TrackVizConfig & cfg,
  const builtin_interfaces::msg::Duration & lifetime, const std::string & status_line)
{
  MarkerArray arr = buildClearMarkers(header);
  arr.markers.reserve(1 + frame.tracks.size() * 7 + 4);

  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    const TrackOutput & t = frame.tracks[i];
    if ((t.motion == MotionState::kStatic && !cfg.show_static) ||
      (t.motion == MotionState::kUnknown && !cfg.show_unknown))
    {
      continue;   // visualisation filter only
    }
    if (t.state == TrackState::kTentative && !cfg.show_tentative) {
      continue;
    }
    const RiskAssessment & r = frame.risks[i];
    const bool tentative = t.state == TrackState::kTentative;
    const Rgb col = tentative ? Rgb{0.6F, 0.6F, 0.6F} : colorForRisk(r.level);
    const float alpha = t.state == TrackState::kConfirmed ? 1.0F : 0.5F;
    const auto id = static_cast<std::int32_t>(t.id & 0x7fffffff);
    const float z_bottom = t.center.z() - 0.5F * t.size.z();
    const float z_top = t.center.z() + 0.5F * t.size.z();

    Marker box = makeMarker(header, "track_boxes", id, Marker::LINE_LIST, lifetime);
    box.scale.x = r.level >= RiskLevel::kHigh ? 0.07 : 0.04;
    setColor(box, col, alpha);
    addOrientedBoxEdges(box, t.center, t.size, t.yaw);
    arr.markers.push_back(std::move(box));

    // label (ASCII only; RViz font lacks Turkish glyphs; keep lines short)
    Marker text = makeMarker(header, "track_labels", id, Marker::TEXT_VIEW_FACING, lifetime);
    text.pose.position = toPoint(t.center.x(), t.center.y(), z_top + 0.6F);
    text.scale.z = 0.22;
    setColor(text, Rgb{1.F, 1.F, 1.F}, 1.F);
    char line1[64], line2[64], line3[64];
    std::snprintf(line1, sizeof(line1), "ID%u %s%s%s", t.id, motionTag(t.motion),
      t.large_structure ? " L" : "", t.predicted ? " (pred)" : "");
    if (t.velocity_reliable) {
      if (t.heading_valid) {
        std::snprintf(line2, sizeof(line2), "%.1f+-%.1fm/s %.0fdeg", t.speed, t.speed_sigma,
          t.heading * 180.F / static_cast<float>(M_PI));
      } else {
        std::snprintf(line2, sizeof(line2), "%.1f+-%.1fm/s", t.speed, t.speed_sigma);
      }
    } else {
      std::snprintf(line2, sizeof(line2), "v:n/a");
    }
    if (r.ttc_valid && std::isfinite(r.ttc_pessimistic)) {
      if (std::isfinite(r.ttc)) {
        std::snprintf(line3, sizeof(line3), "d%.1fm TTC%.1fs[%.1f] %s", t.closest_distance,
          r.ttc, r.ttc_pessimistic, toString(r.level));
      } else {
        std::snprintf(line3, sizeof(line3), "d%.1fm TTC>[%.1f] %s", t.closest_distance,
          r.ttc_pessimistic, toString(r.level));
      }
    } else {
      std::snprintf(line3, sizeof(line3), "d%.1fm %s", t.closest_distance, toString(r.level));
    }
    text.text = std::string(line1) + "\n" + line2 + "\n" + line3;
    arr.markers.push_back(std::move(text));

    // ground-relative velocity arrow
    if (t.velocity_reliable && t.motion == MotionState::kMoving &&
      t.speed >= cfg.min_arrow_speed)
    {
      Marker arrow = makeMarker(header, "track_velocity", id, Marker::ARROW, lifetime);
      arrow.points.push_back(toPoint(t.center.x(), t.center.y(), z_top + 0.1F));
      arrow.points.push_back(toPoint(
          t.center.x() + t.velocity.x() * cfg.arrow_time_s,
          t.center.y() + t.velocity.y() * cfg.arrow_time_s, z_top + 0.1F));
      arrow.scale.x = 0.08;   // shaft diameter
      arrow.scale.y = 0.18;   // head diameter
      arrow.scale.z = 0.20;   // head length
      setColor(arrow, Rgb{0.2F, 1.F, 1.F}, 0.95F);
      arr.markers.push_back(std::move(arrow));
    }

    // predicted path relative to the sensor (constant relative velocity)
    if (cfg.show_prediction && r.ttc_valid && t.relative_velocity.norm() >= cfg.min_arrow_speed) {
      Marker path = makeMarker(header, "track_prediction", id, Marker::LINE_STRIP, lifetime);
      path.scale.x = 0.05;
      setColor(path, col, 0.6F);
      for (float s = 0.F; s <= cfg.prediction_horizon_s + 1e-3F; s += cfg.prediction_step_s) {
        path.points.push_back(toPoint(
            t.center.x() + t.relative_velocity.x() * s,
            t.center.y() + t.relative_velocity.y() * s, z_bottom + 0.05F));
      }
      arr.markers.push_back(std::move(path));
    }

    // 2-sigma position uncertainty ellipse
    if (cfg.show_uncertainty) {
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> es(t.position_cov);
      if (es.info() == Eigen::Success) {
        const Eigen::Vector2f ev = es.eigenvalues().cwiseMax(1e-6F);
        const Eigen::Vector2f major = es.eigenvectors().col(1);
        Marker ell = makeMarker(header, "track_uncertainty", id, Marker::CYLINDER, lifetime);
        ell.pose.position = toPoint(t.center.x(), t.center.y(), z_bottom + 0.02F);
        setYaw(ell, std::atan2(major.y(), major.x()));
        ell.scale.x = std::max(4.0 * std::sqrt(ev[1]), 0.05);   // 2 sigma diameter
        ell.scale.y = std::max(4.0 * std::sqrt(ev[0]), 0.05);
        ell.scale.z = 0.02;
        setColor(ell, col, 0.3F);
        arr.markers.push_back(std::move(ell));
      }
    }

    // trail of past filtered positions
    if (cfg.show_trails && t.trail.size() >= 2 && t.motion == MotionState::kMoving) {
      Marker trail = makeMarker(header, "track_trails", id, Marker::LINE_STRIP, lifetime);
      trail.scale.x = 0.04;
      setColor(trail, Rgb{0.8F, 0.8F, 0.8F}, 0.7F);
      for (const auto & p : t.trail) {
        trail.points.push_back(toPoint(p.x(), p.y(), z_bottom + 0.03F));
      }
      arr.markers.push_back(std::move(trail));
    }
  }

  // ego footprint + velocity
  const EgoMotion & ego = frame.ego;
  Rgb ego_col{0.3F, 0.9F, 0.3F};
  if (ego.state == EgoState::kUnknown) {
    ego_col = {1.F, 0.2F, 0.2F};
  } else if (ego.state == EgoState::kUnreliable) {
    ego_col = {1.F, 0.7F, 0.1F};
  }
  Marker foot = makeMarker(header, "ego", 0, Marker::CYLINDER, lifetime);
  foot.pose.position = toPoint(0.F, 0.F, -0.2F);
  foot.scale.x = foot.scale.y = 2.0 * std::max(cfg.ego_radius, 0.05F);
  foot.scale.z = 0.05;
  setColor(foot, ego_col, 0.35F);
  arr.markers.push_back(std::move(foot));
  if (ego.valid && ego.velocity.norm() >= cfg.min_arrow_speed) {
    Marker ev = makeMarker(header, "ego", 1, Marker::ARROW, lifetime);
    ev.points.push_back(toPoint(0.F, 0.F, 0.F));
    ev.points.push_back(toPoint(
        static_cast<float>(ego.velocity.x()) * cfg.arrow_time_s,
        static_cast<float>(ego.velocity.y()) * cfg.arrow_time_s, 0.F));
    ev.scale.x = 0.1;
    ev.scale.y = 0.22;
    ev.scale.z = 0.25;
    setColor(ev, ego_col, 0.9F);
    arr.markers.push_back(std::move(ev));
  }

  // HUD
  Marker hud = makeMarker(header, "status_hud", 0, Marker::TEXT_VIEW_FACING, lifetime);
  hud.pose.position = toPoint(0.F, 0.F, 2.2F);
  hud.scale.z = 0.3;
  const Rgb hud_col = frame.max_risk >= RiskLevel::kMedium ? colorForRisk(frame.max_risk) :
    (ego.state == EgoState::kUnknown || frame.ground.status == GroundStatus::kFailed ?
    Rgb{1.F, 0.5F, 0.2F} : Rgb{0.9F, 0.9F, 0.9F});
  setColor(hud, hud_col, 1.F);
  char buf[160];
  std::snprintf(buf, sizeof(buf), "EGO %s %.2fm/s | GROUND %s | RISK %s | moving %zu / tracks %zu",
    toString(ego.state), ego.velocity.norm(), toString(frame.ground.status),
    toString(frame.max_risk), frame.tracker_stats.moving, frame.tracker_stats.confirmed);
  std::string learn = frame.background_learning ? " | learning static background" : "";
  hud.text = std::string(buf) + learn + (status_line.empty() ? "" : "\n" + status_line);
  arr.markers.push_back(std::move(hud));
  return arr;
}

}  // namespace helios_obstacle_detection
