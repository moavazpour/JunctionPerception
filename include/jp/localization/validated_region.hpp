#pragma once

#include <Eigen/Core>
#include <vector>

namespace jp::localization {

struct ValidatedRegionOptions {
  double margin_m = 10.0;            ///< Around the outline of the control points.
  double max_ground_m_per_px = 0.2;  ///< Coarsest ground sampling along the camera ray.
};

/// Where ground positions are good enough for safety events: inside the outline (convex hull)
/// of the calibration's control points plus a margin, and where one pixel of image motion moves
/// the ground point by at most `max_ground_m_per_px` along the camera ray (the direction in
/// which a camera resolves positions worst; it grows with the square of the range).
class ValidatedRegion {
 public:
  ValidatedRegion(const std::vector<Eigen::Vector2d>& control_points,
                  const Eigen::Matrix3d& image_to_ground, Eigen::Vector2d camera_position,
                  ValidatedRegionOptions options = {});

  [[nodiscard]] bool contains(const Eigen::Vector2d& point) const;

  /// Ground metres per pixel of image motion along the camera ray at `point` (worst image
  /// direction); infinite where the point is not in front of the camera.
  [[nodiscard]] double ground_m_per_px(const Eigen::Vector2d& point) const;

  /// Distance from `point` to the control point outline (0 inside).
  [[nodiscard]] double distance_to_outline(const Eigen::Vector2d& point) const;

  [[nodiscard]] const std::vector<Eigen::Vector2d>& outline() const { return outline_; }

 private:
  std::vector<Eigen::Vector2d> outline_;  // counter-clockwise
  Eigen::Matrix3d image_to_ground_;
  Eigen::Matrix3d ground_to_image_;
  Eigen::Vector2d camera_position_;
  double front_sign_ = 1.0;  // sign of the homogeneous image w for points in front of the camera
  ValidatedRegionOptions options_;
};

}  // namespace jp::localization
