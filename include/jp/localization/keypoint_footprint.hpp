#pragma once

#include <Eigen/Core>
#include <array>
#include <optional>
#include <vector>

#include "jp/detection/detection.hpp"
#include "jp/localization/box_fit.hpp"

namespace jp::localization {

/// Footprint of an object on the ground from the four bottom corners of its 3D box. The model
/// gives the corners in a fixed order: 0 and 1 at the front, 2 and 3 at the rear.
struct GroundFootprint {
  std::array<Eigen::Vector2d, 4> corners{};  ///< Ground corners in the model's order.
  Eigen::Vector2d center = Eigen::Vector2d::Zero();
  double heading_rad = 0.0;  ///< From the middle of the rear corners to that of the front ones.
  double length_m = 0.0;
  double width_m = 0.0;
  Eigen::Vector2d pixel_center = Eigen::Vector2d::Zero();  ///< Mean of the corner pixels.
};

struct KeypointFootprintOptions {
  float min_score = 0.5F;  ///< Every corner must be at least this visible.
  /// The measured length and width must lie within these factors of the class size, so that a
  /// corner on the wrong object or a projection near the horizon is not taken.
  double min_size_ratio = 0.4;
  double max_size_ratio = 2.5;
  double min_side_m = 0.2;  ///< Lower bound for the size check (pedestrians).
};

/// Ground footprint of a detection with four bottom-corner keypoints, through the image-to-ground
/// homography; empty without four visible corners, when a corner maps to infinity, or when the
/// shape is not a plausible box of the class size.
[[nodiscard]] std::optional<GroundFootprint> keypoint_footprint(
    const std::vector<detection::Keypoint>& keypoints, const Eigen::Matrix3d& image_to_ground,
    const ObjectSize& class_size, const KeypointFootprintOptions& options = {});

/// The direction of an axis (angle or angle + pi) closest to a reference direction.
[[nodiscard]] double orient_axis(double axis_rad, double reference_rad);

}  // namespace jp::localization
