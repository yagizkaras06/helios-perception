// Concentric-Zone-Model (CZM) ground segmentation, inspired by Patchwork / Patchwork++
// (Lim et al., RA-L 2021 / IROS 2022), adapted to a single Helios 32 without IMU.
//
//  1. Polar grid: range zones -> rings -> azimuth sectors (coarser bins far away, where the
//     point density of a 32-beam sensor is low). Points are bucketed with a counting sort
//     (no per-bin allocations).
//  2. Region-wise ground plane fit per bin (in parallel): lowest-point-representative seeds,
//     then iterative PCA plane fitting.
//  3. Ground likelihood estimation per bin, sequentially from inner to outer ring per sector:
//       uprightness  (plane normal vs. expected normal)
//       planarity    (reject line-like / degenerate supports)
//       flatness     (smallest eigenvalue, adaptive threshold)
//       elevation    (near rings: vs. the mounting-height prior, adaptive threshold)
//       continuity   (height jump vs. the nearest accepted inner bin of the same sector, at the
//                     bin centroid AND at the four bin corners: a plane tilted by an object's
//                     base in a sparse far bin is rejected even if it is right at the centroid)
//     Elevated flat surfaces (tables, platforms, car roofs) therefore are NOT taken as ground.
//  4. Points of rejected / sparse bins are classified against a reference plane: nearest
//     accepted bin of the same sector, then neighbouring sectors, then the configured expected
//     plane (counted as fallback and reported).
//  5. Adaptive thresholds: running mean / variance of the elevation and flatness of accepted
//     near bins (A-GLE) update the thresholds frame by frame, clamped to configured limits.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include "helios_obstacle_detection/ground_segmenter.hpp"
#include "helios_obstacle_detection/thread_pool.hpp"

namespace helios_obstacle_detection
{

struct CzmConfig
{
  // Shared with the RANSAC method: expected plane, thresholds for point classification.
  GroundConfig base{};

  std::vector<float> zone_boundaries{0.F, 4.F, 10.F, 20.F, 30.F};  // [m] horizontal range
  std::vector<int> rings_per_zone{2, 3, 3, 2};
  std::vector<int> sectors_per_zone{16, 32, 48, 32};
  int min_bin_points{8};
  int seed_count{20};             // lowest points used for the seed height
  float seed_height{0.15F};       // [m] seeds: below lowest-mean + this
  float plane_distance{0.10F};    // [m] plane inlier band during PCA iterations
  int fit_iterations{3};
  float min_planarity_ratio{4.F}; // middle / smallest eigenvalue; below -> line-like support
  float min_extent{0.10F};        // [m] sqrt(middle eigenvalue) of a plane support
  int elevation_rings{4};         // first N global rings are checked against the mount prior
  float elevation_threshold{0.30F};  // [m] initial threshold (adaptive afterwards)
  float elevation_min{0.15F};
  float elevation_max{0.50F};
  float flatness_threshold{0.004F};  // [m^2] smallest eigenvalue; initial threshold
  float flatness_min{0.001F};
  float flatness_max{0.010F};
  float max_bin_height_jump{0.25F};  // [m] vs. nearest accepted inner bin (same sector)
  float max_corner_height_jump{0.40F};  // [m] same, evaluated at the 4 bin corners (tilted fits)
  int continuity_rings{2};           // inner reference must be at most this many rings away
  // Far bins without an accepted inner bin (within continuity_rings): with a 32-beam sensor on
  // flat ground whole range rings receive no ground return at all (e.g. 15.4..25.6 m for the
  // Helios 32 at 0.9 m), so a plane fitted there usually is ONE beam ring on an object. By default
  // such planes are not accepted (missing an obstacle is worse than a fallback plane).
  bool far_requires_reference{true};
  float far_height_tolerance{0.5F};  // [m] only used when far_requires_reference is false
  bool adaptive{true};
  float adaptation_rate{0.05F};      // EMA factor of the running statistics
  float adaptive_sigma{3.F};         // threshold = mean + k * std
  float degraded_ratio{0.5F};        // > this share of points on fallback plane -> DEGRADED

  void validate();  // throws std::invalid_argument
};

enum class BinState : std::uint8_t
{
  kEmpty = 0,
  kSparse,        // too few points or degenerate support -> reference plane
  kAccepted,
  kRejectedTilt,
  kRejectedRough,
  kRejectedElevated,
  kRejectedDiscontinuous,
};

struct CzmStats
{
  std::size_t bins_populated{0};
  std::array<std::size_t, 7> bins_by_state{};
  std::size_t points_own_plane{0};
  std::size_t points_reference_plane{0};
  std::size_t points_fallback_plane{0};
  std::vector<float> elevation_thresholds;  // per near ring, used for this frame
  std::vector<float> flatness_thresholds;
};

class CzmGroundSegmenter : public GroundSegmenterBase
{
public:
  /// `pool` may be nullptr (sequential). The pool must outlive the segmenter.
  CzmGroundSegmenter(const CzmConfig & config, ThreadPool * pool = nullptr);

  void segment(const CloudConstPtr & cloud, GroundResult & result) override;
  void reset() override;
  const char * name() const noexcept override {return "czm";}

  const CzmStats & lastStats() const noexcept {return stats_;}
  const CzmConfig & config() const noexcept {return config_;}
  std::size_t binCount() const noexcept {return bin_count_;}

private:
  struct BinFit
  {
    std::uint32_t count{0};
    bool fitted{false};
    Eigen::Vector4f plane{0.F, 0.F, 1.F, 0.F};
    Eigen::Vector3f centroid{Eigen::Vector3f::Zero()};
    float lambda_min{0.F};
    float lambda_mid{0.F};
    float upright{0.F};
    float elevation{0.F};
  };

  struct RunningStat
  {
    double mean{0.0};
    double var{0.0};
    bool init{false};
    void add(double v, double alpha);
  };

  int binOf(float x, float y, int * ring_out) const noexcept;
  /// Largest |height difference| of two planes over the four corners of a bin.
  float cornerJump(
    const Eigen::Vector4f & a, const Eigen::Vector4f & b, int ring, int sector) const noexcept;
  void fitBin(const Cloud & cloud, std::size_t begin, std::size_t end, BinFit & fit) const;

  CzmConfig config_;
  ThreadPool * pool_;
  Eigen::Vector4f expected_plane_;
  float cos_max_angle_;

  // grid geometry
  int total_rings_{0};
  std::size_t bin_count_{0};
  std::vector<float> ring_inner_;       // per global ring
  std::vector<float> ring_outer_;
  std::vector<int> ring_sectors_;
  std::vector<std::size_t> ring_first_bin_;

  // adaptive state
  std::vector<RunningStat> elev_stats_;
  std::vector<RunningStat> flat_stats_;
  std::vector<float> elev_thr_;
  std::vector<float> flat_thr_;

  // reusable buffers (avoid per-frame allocations)
  std::vector<std::uint32_t> point_bin_;
  std::vector<std::uint32_t> bin_offsets_;
  std::vector<pcl::index_t> sorted_;
  std::vector<BinFit> fits_;
  std::vector<BinState> states_;
  std::vector<int> reference_bin_;      // -1 -> expected plane fallback
  std::vector<std::uint8_t> labels_;    // 0 ground, 1 obstacle, 2 high, 3 low

  CzmStats stats_;
};

const char * toString(BinState s) noexcept;

}  // namespace helios_obstacle_detection
