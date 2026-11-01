#include "jp/localization/keypoint_footprint.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <numbers>

namespace jp::localization {

namespace {

constexpr double kEpsilon = 1e-9;
constexpr std::size_t kCorners = 4;

double wrap(double angle) { return std::remainder(angle, 2.0 * std::numbers::pi); }

}  // namespace

double orient_axis(double axis_rad, double reference_rad) {
  return std::abs(wrap(axis_rad - reference_rad)) <= std::numbers::pi / 2.0
             ? wrap(axis_rad)
             : wrap(axis_rad + std::numbers::pi);
}

std::optional<GroundFootprint> keypoint_footprint(const std::vector<detection::Keypoint>& keypoints,
                                                  const Eigen::Matrix3d& image_to_ground,
                                                  const ObjectSize& class_size,
                                                  const KeypointFootprintOptions& options) {
  if (keypoints.size() != kCorners) {
    return std::nullopt;
  }
  GroundFootprint footprint;
  for (std::size_t i = 0; i < kCorners; ++i) {
    const auto& k = keypoints[i];
    if (k.score < options.min_score) {
      return std::nullopt;
    }
    const Eigen::Vector2d pixel(k.pixel.x, k.pixel.y);
    const Eigen::Vector3d ground = image_to_ground * pixel.homogeneous();
    if (std::abs(ground.z()) < kEpsilon) {
      return std::nullopt;
    }
    footprint.corners.at(i) = ground.hnormalized();
    footprint.center += footprint.corners.at(i) / static_cast<double>(kCorners);
    footprint.pixel_center += pixel / static_cast<double>(kCorners);
  }
  // Corners 0 and 1 are the front, 2 and 3 the rear (whichever side each is on).
  const auto& c = footprint.corners;
  const Eigen::Vector2d forward = ((c[0] + c[1]) - (c[2] + c[3])) / 2.0;
  footprint.length_m = forward.norm();
  footprint.width_m = ((c[0] - c[1]).norm() + (c[2] - c[3]).norm()) / 2.0;
  footprint.heading_rad = std::atan2(forward.y(), forward.x());

  const double length = std::max(class_size.length_m, options.min_side_m);
  const double width = std::max(class_size.width_m, options.min_side_m);
  const auto plausible = [&](double measured, double expected) {
    return measured >= options.min_size_ratio * expected &&
           measured <= options.max_size_ratio * expected;
  };
  if (!plausible(footprint.length_m, length) || !plausible(footprint.width_m, width)) {
    return std::nullopt;
  }
  return footprint;
}

}  // namespace jp::localization
