#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace jp::calibration {

/// Plane n·x + d = 0 with unit normal n pointing up (n.z > 0).
struct Plane {
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
  double offset = 0.0;

  /// Signed distance of a point above the plane (m).
  [[nodiscard]] double signed_distance(const Eigen::Vector3d& point) const;
  /// Orthogonal projection of a point onto the plane.
  [[nodiscard]] Eigen::Vector3d project(const Eigen::Vector3d& point) const;
};

struct PlaneFitOptions {
  double inlier_threshold_m = 0.15;
  int iterations = 1000;
  std::uint32_t seed = 42;
};

struct PlaneFit {
  Plane plane;
  std::size_t inliers = 0;
  double inlier_rms_m = 0.0;
};

/// Robust plane fit: RANSAC on point triples, then a least-squares refit on the inliers.
/// Throws std::invalid_argument for fewer than three points.
[[nodiscard]] PlaneFit fit_plane(const std::vector<Eigen::Vector3d>& points,
                                 const PlaneFitOptions& options = {});

}  // namespace jp::calibration
