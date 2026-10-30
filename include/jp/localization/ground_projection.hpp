#pragma once

#include <Eigen/Core>
#include <opencv2/core/types.hpp>
#include <optional>

namespace jp::localization {

/// Fixed footprint of an object class (m).
struct Footprint {
  double length_m = 0.0;
  double width_m = 0.0;
};

/// Ground point under the bottom centre of an image box (ground plane coordinates, m); empty if
/// the point maps to infinity (e.g. above the horizon).
[[nodiscard]] std::optional<Eigen::Vector2d> ground_contact(const cv::Rect2f& box,
                                                            const Eigen::Matrix3d& image_to_ground);

/// Distance from the visible ground contact (the side facing the camera) to the footprint centre
/// along the camera-to-object ground ray: half the length without a heading, otherwise
/// 0.5 * (|L cos t| + |W sin t|) with t the angle between heading and ray.
[[nodiscard]] double contact_to_centre_distance(const Footprint& footprint,
                                                const Eigen::Vector2d& ray_direction,
                                                std::optional<double> heading_rad);

/// Moves a ground contact away from the camera to the estimated footprint centre.
[[nodiscard]] Eigen::Vector2d correct_contact_bias(const Eigen::Vector2d& contact,
                                                   const Eigen::Vector2d& camera_position,
                                                   const Footprint& footprint,
                                                   std::optional<double> heading_rad);

}  // namespace jp::localization
