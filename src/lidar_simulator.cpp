#include "helios_obstacle_detection/lidar_simulator.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

#include <sensor_msgs/msg/point_field.hpp>

namespace helios_obstacle_detection
{
namespace sim
{

namespace
{
constexpr double kDeg = M_PI / 180.0;

Eigen::Matrix2d rot(double yaw)
{
  Eigen::Matrix2d r;
  r << std::cos(yaw), -std::sin(yaw), std::sin(yaw), std::cos(yaw);
  return r;
}

/// Ray / oriented box intersection (slab method in the box frame). Returns t or +inf.
double intersectBox(
  const Eigen::Vector3d & o, const Eigen::Vector3d & d, const SimBox & b, double sensor_h)
{
  const Eigen::Matrix2d rt = rot(b.yaw).transpose();
  const Eigen::Vector2d o2 = rt * (o.head<2>() - b.position);
  const Eigen::Vector2d d2 = rt * d.head<2>();
  const double oz = o.z() - (-sensor_h + 0.5 * b.size.z());  // box centre z in sensor-height frame
  const double lo[3] = {-0.5 * b.size.x(), -0.5 * b.size.y(), -0.5 * b.size.z()};
  const double hi[3] = {0.5 * b.size.x(), 0.5 * b.size.y(), 0.5 * b.size.z()};
  const double oo[3] = {o2.x(), o2.y(), oz};
  const double dd[3] = {d2.x(), d2.y(), d.z()};
  double tmin = 0.0, tmax = std::numeric_limits<double>::infinity();
  for (int k = 0; k < 3; ++k) {
    if (std::fabs(dd[k]) < 1e-12) {
      if (oo[k] < lo[k] || oo[k] > hi[k]) {
        return std::numeric_limits<double>::infinity();
      }
      continue;
    }
    double t1 = (lo[k] - oo[k]) / dd[k];
    double t2 = (hi[k] - oo[k]) / dd[k];
    if (t1 > t2) {
      std::swap(t1, t2);
    }
    tmin = std::max(tmin, t1);
    tmax = std::min(tmax, t2);
    if (tmin > tmax) {
      return std::numeric_limits<double>::infinity();
    }
  }
  return tmin > 1e-6 ? tmin : std::numeric_limits<double>::infinity();
}

Eigen::Vector2d closestPointOfBox(const Eigen::Vector2d & c, const Eigen::Vector3d & size, double yaw)
{
  // closest point of the rectangle to the origin
  const Eigen::Matrix2d r = rot(yaw);
  const Eigen::Vector2d o = r.transpose() * (-c);   // origin in the box frame
  const Eigen::Vector2d q(std::clamp(o.x(), -0.5 * size.x(), 0.5 * size.x()),
    std::clamp(o.y(), -0.5 * size.y(), 0.5 * size.y()));
  return c + r * q;
}
}  // namespace

void Scene::step(double dt)
{
  time += dt;
  for (auto & b : boxes) {
    if (std::fabs(b.yaw_rate) > 1e-9) {
      b.velocity = rot(b.yaw_rate * dt) * b.velocity;
      b.yaw += b.yaw_rate * dt;
    }
    b.position += b.velocity * dt;
  }
  const double yaw = ego_pose[2];
  ego_pose[0] += ego_speed * std::cos(yaw) * dt;
  ego_pose[1] += ego_speed * std::sin(yaw) * dt;
  ego_pose[2] += ego_yaw_rate * dt;
}

LidarSimulator::LidarSimulator(const SimConfig & config)
: config_(config), rng_(config.seed)
{
  elev_ = config_.elevations_deg;
  if (elev_.empty()) {
    // Helios-32-like: 24 beams +15 .. -15.6 deg (1.33 deg), 8 beams -19 .. -55 deg.
    for (int k = 0; k < 24; ++k) {
      elev_.push_back(15.0 - 1.33 * k);
    }
    for (int k = 0; k < 8; ++k) {
      elev_.push_back(-19.0 - (36.0 / 7.0) * k);
    }
  }
  if (config_.azimuth_steps < 8) {
    throw std::invalid_argument("sim: azimuth_steps too small");
  }
  cos_az_.resize(static_cast<std::size_t>(config_.azimuth_steps));
  sin_az_.resize(cos_az_.size());
  for (int k = 0; k < config_.azimuth_steps; ++k) {
    const double a = 2.0 * M_PI * k / config_.azimuth_steps;
    cos_az_[static_cast<std::size_t>(k)] = std::cos(a);
    sin_az_[static_cast<std::size_t>(k)] = std::sin(a);
  }
}

sensor_msgs::msg::PointCloud2 LidarSimulator::scan(const Scene & scene, std::vector<int> & hits)
{
  using sensor_msgs::msg::PointField;
  const auto rings = static_cast<std::uint32_t>(elev_.size());
  const auto cols = static_cast<std::uint32_t>(config_.azimuth_steps);
  sensor_msgs::msg::PointCloud2 msg;
  msg.height = cols;      // like rslidar_sdk: one row per firing
  msg.width = rings;
  msg.is_bigendian = false;
  msg.is_dense = false;
  msg.point_step = 16;
  msg.row_step = msg.point_step * msg.width;
  const char * names[4] = {"x", "y", "z", "intensity"};
  for (std::uint32_t k = 0; k < 4; ++k) {
    PointField f;
    f.name = names[k];
    f.offset = 4 * k;
    f.datatype = PointField::FLOAT32;
    f.count = 1;
    msg.fields.push_back(f);
  }
  msg.data.resize(static_cast<std::size_t>(msg.row_step) * msg.height);

  // boxes in the sensor frame
  std::vector<SimBox> local = scene.boxes;
  const Eigen::Matrix2d rt = rot(scene.ego_pose[2]).transpose();
  for (auto & b : local) {
    b.position = rt * (b.position - scene.ego_pose.head<2>());
    b.yaw -= scene.ego_pose[2];
  }
  hits.assign(scene.boxes.size(), 0);

  std::normal_distribution<double> noise(0.0, config_.range_noise);
  std::uniform_real_distribution<double> uni(0.0, 1.0);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const Eigen::Vector3d origin(0.0, 0.0, 0.0);
  const double h = config_.sensor_height;

  for (std::uint32_t c = 0; c < cols; ++c) {
    for (std::uint32_t r = 0; r < rings; ++r) {
      const double e = elev_[r] * kDeg;
      const Eigen::Vector3d d(std::cos(e) * cos_az_[c], std::cos(e) * sin_az_[c], std::sin(e));
      double best = std::numeric_limits<double>::infinity();
      int best_box = -1;
      if (d.z() < -1e-9) {
        best = h / -d.z();   // ground plane z = -h in the sensor frame
      }
      for (std::size_t k = 0; k < local.size(); ++k) {
        const double t = intersectBox(origin, d, local[k], h);
        if (t < best) {
          best = t;
          best_box = static_cast<int>(k);
        }
      }
      float out[4] = {nan, nan, nan, 0.F};
      if (std::isfinite(best) && best <= config_.max_range && best >= config_.min_range &&
        uni(rng_) >= config_.dropout)
      {
        const double t = best + noise(rng_);
        out[0] = static_cast<float>(d.x() * t);
        out[1] = static_cast<float>(d.y() * t);
        out[2] = static_cast<float>(d.z() * t);
        out[3] = best_box >= 0 ? 60.F : 10.F;
        if (best_box >= 0) {
          ++hits[static_cast<std::size_t>(best_box)];
        }
      }
      std::memcpy(&msg.data[static_cast<std::size_t>(c) * msg.row_step + r * msg.point_step],
        out, sizeof(out));
    }
  }
  return msg;
}

std::vector<GroundTruth> LidarSimulator::groundTruth(
  const Scene & scene, const std::vector<int> & hits, double ego_radius,
  double min_closing_speed) const
{
  std::vector<GroundTruth> out;
  const Eigen::Matrix2d rt = rot(scene.ego_pose[2]).transpose();
  // ego velocity in sensor axes: forward speed along +x
  const Eigen::Vector2d ego_v(scene.ego_speed, 0.0);
  for (std::size_t k = 0; k < scene.boxes.size(); ++k) {
    const SimBox & b = scene.boxes[k];
    GroundTruth g;
    g.evaluate = b.evaluate;
    g.id = b.id;
    g.position = rt * (b.position - scene.ego_pose.head<2>());
    g.size = b.size;
    g.yaw = b.yaw - scene.ego_pose[2];
    g.velocity = rt * b.velocity;
    const Eigen::Vector2d w_x_p(-scene.ego_yaw_rate * g.position.y(),
      scene.ego_yaw_rate * g.position.x());
    g.relative_velocity = g.velocity - ego_v - w_x_p;
    g.closest_point = closestPointOfBox(g.position, g.size, g.yaw);
    g.moving = b.moving();
    g.hits = k < hits.size() ? hits[k] : 0;
    const double range = g.closest_point.norm();
    const double closing = range > 1e-6 ? -g.closest_point.dot(g.relative_velocity) / range : 0.0;
    g.ttc = closing > min_closing_speed ? std::max(range - ego_radius, 0.0) / closing :
      std::numeric_limits<double>::infinity();
    out.push_back(g);
  }
  return out;
}

Scene makeScenario(const std::string & name, unsigned seed)
{
  Scene s;
  auto box = [&](int id, double x, double y, double l, double w, double hgt, double yaw,
    double vx, double vy, bool eval = true) {
      SimBox b;
      b.id = id;
      b.position = Eigen::Vector2d(x, y);
      b.size = Eigen::Vector3d(l, w, hgt);
      b.yaw = yaw;
      b.velocity = Eigen::Vector2d(vx, vy);
      b.evaluate = eval;
      s.boxes.push_back(b);
    };
  if (name == "crossing") {
    box(1, 6.0, -5.0, 0.5, 0.5, 1.7, 0.0, 0.0, 1.3);                 // pedestrian crossing
    box(2, 11.0, 5.0, 0.5, 0.5, 1.7, 0.0, -0.8, -0.6);               // pedestrian diagonal
    box(3, 28.0, 2.5, 4.5, 1.8, 1.5, 0.0, -5.0, 0.0);                // car, adjacent lane
    box(4, 8.0, -2.0, 1.0, 1.0, 1.0, 0.3, 0.0, 0.0);                 // static box
    box(5, 15.0, -6.0, 2.0, 1.0, 1.2, 0.0, 0.0, 0.0);                // static box
    box(6, 5.0, 6.0, 0.25, 0.25, 2.0, 0.0, 0.0, 0.0);                // pole
    box(7, -7.0, -3.0, 4.5, 1.8, 1.5, 0.2, 0.0, 0.0);                // parked car behind
    box(8, -12.0, 4.0, 0.6, 0.6, 1.7, 0.0, 1.0, 0.0);                // pedestrian behind
  } else if (name == "approach") {
    box(1, 20.0, 0.0, 1.0, 0.8, 1.5, 0.0, -2.0, 0.0);                // straight at the sensor
    box(2, 9.0, 4.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0);                  // static reference
    box(3, -6.0, -5.0, 2.0, 2.0, 1.2, 0.4, 0.0, 0.0);                // static clutter
    box(4, 4.0, -7.0, 0.4, 0.4, 2.0, 0.0, 0.0, 0.0);
    box(90, 5.0, 12.0, 40.0, 0.3, 3.0, 0.0, 0.0, 0.0, false);        // building walls
    box(91, -12.0, 0.0, 0.3, 24.0, 3.0, 0.0, 0.0, 0.0, false);
  } else if (name == "ego_motion") {
    s.ego_speed = 3.0;
    s.ego_yaw_rate = 0.05;
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> ux(-10.0, 45.0), uy(-14.0, 14.0), uyaw(0.0, M_PI),
      usz(0.4, 2.0);
    int id = 100;
    for (int k = 0; k < 40; ++k) {     // static clutter, kept off the driving corridor
      const double x = ux(rng), y = uy(rng);
      if (std::fabs(y) < 3.0) {
        continue;
      }
      box(id++, x, y, usz(rng), usz(rng), 0.8 + usz(rng) * 0.6, uyaw(rng), 0.0, 0.0);
    }
    box(1, 14.0, -6.0, 0.5, 0.5, 1.7, 0.0, 0.0, 1.2);                // pedestrian crossing
    box(2, 40.0, -2.5, 4.5, 1.8, 1.5, M_PI, -6.0, 0.0);              // oncoming car
    box(3, 20.0, 4.0, 0.6, 0.6, 1.7, 0.0, 0.9, 0.0);                 // pedestrian along road
    box(90, 20.0, 18.0, 60.0, 0.3, 3.0, 0.0, 0.0, 0.0, false);       // building walls
    box(91, 20.0, -18.0, 60.0, 0.3, 3.0, 0.0, 0.0, 0.0, false);
  } else {
    throw std::invalid_argument("unknown scenario '" + name + "'");
  }
  return s;
}

}  // namespace sim
}  // namespace helios_obstacle_detection
