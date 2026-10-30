#pragma once

#include <Eigen/Geometry>
#include <cstddef>
#include <utility>
#include <vector>

#include "jp/benchmark/openlabel.hpp"

namespace jp::benchmark {

/// Rigid transform between the two LiDAR label frames restricted to yaw and translation
/// (both LiDARs are levelled, so roll and pitch differences are negligible).
struct PlanarPose {
  double yaw_rad = 0.0;
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();

  [[nodiscard]] Eigen::Isometry3d isometry() const;
  /// Maps a box into the target frame (centre and rotation).
  [[nodiscard]] Cuboid apply(const Cuboid& box) const;
};

struct RegistrationOptions {
  std::size_t hypothesis_frames = 12;  ///< Frames sampled to generate pose hypotheses.
  std::size_t scoring_frames = 60;     ///< Frames sampled to score the hypotheses.
  double min_heading_length_m = 3.0;   ///< Only boxes at least this long define a heading.
  double hypothesis_gate_m = 1.0;      ///< BEV distance counted as agreement when scoring.
  std::vector<double> refine_gates_m = {2.0, 1.0, 0.75};  ///< Matching gates while refining.
  double report_gate_m = 2.0;  ///< Gate for the reported agreement statistics.
};

struct RegistrationResult {
  PlanarPose south_from_north;
  std::size_t matched_pairs = 0;  ///< Box pairs within report_gate_m over all frames.
  double median_bev_error_m = 0.0;
  double p90_bev_error_m = 0.0;
  double median_height_error_m = 0.0;
};

/// Matched box pairs of one frame: indices into (south, north) within a BEV gate, one-to-one,
/// same category only. `north` must already be expressed in the south frame.
[[nodiscard]] std::vector<std::pair<std::size_t, std::size_t>> match_boxes(
    const std::vector<Cuboid>& south, const std::vector<Cuboid>& north, double gate_m);

/// Estimates the north-to-south LiDAR transform from the labels alone: pose hypotheses from
/// pairs of long boxes of the same category are scored by how many boxes they bring into
/// agreement, the best one is refined by least squares over all matched boxes of all frames.
/// `frames` holds (south boxes, north boxes) per frame, each in its own LiDAR frame.
[[nodiscard]] RegistrationResult register_label_sets(
    const std::vector<std::pair<std::vector<Cuboid>, std::vector<Cuboid>>>& frames,
    const RegistrationOptions& options = {});

}  // namespace jp::benchmark
