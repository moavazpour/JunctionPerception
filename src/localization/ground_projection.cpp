#include "jp/localization/ground_projection.hpp"

#include <Eigen/Geometry>
#include <cmath>

namespace jp::localization {

namespace {

constexpr double kEpsilon = 1e-9;

}  // namespace

std::optional<Eigen::Vector2d> ground_contact(const cv::Rect2f& box,
                                              const Eigen::Matrix3d& image_to_ground) {
  const Eigen::Vector3d pixel(box.x + (0.5 * box.width), box.y + box.height, 1.0);
  const Eigen::Vector3d ground = image_to_ground * pixel;
  if (std::abs(ground.z()) < kEpsilon) {
    return std::nullopt;
  }
  return ground.hnormalized();
}

double contact_to_centre_distance(const Footprint& footprint, const Eigen::Vector2d& ray_direction,
                                  std::optional<double> heading_rad) {
  if (!heading_rad) {
    return 0.5 * footprint.length_m;
  }
  const double angle = *heading_rad - std::atan2(ray_direction.y(), ray_direction.x());
  return 0.5 * ((std::abs(footprint.length_m * std::cos(angle))) +
                std::abs(footprint.width_m * std::sin(angle)));
}

Eigen::Vector2d correct_contact_bias(const Eigen::Vector2d& contact,
                                     const Eigen::Vector2d& camera_position,
                                     const Footprint& footprint,
                                     std::optional<double> heading_rad) {
  const Eigen::Vector2d ray = contact - camera_position;
  const double norm = ray.norm();
  if (norm < kEpsilon) {
    return contact;
  }
  const Eigen::Vector2d direction = ray / norm;
  return contact + (contact_to_centre_distance(footprint, direction, heading_rad) * direction);
}

}  // namespace jp::localization
