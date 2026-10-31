#include "jp/localization/ground_projection.hpp"

#include <Eigen/Geometry>
#include <cmath>

namespace jp::localization {

namespace {

constexpr double kEpsilon = 1e-9;

}  // namespace

Eigen::Vector2d contact_pixel(const cv::Rect2f& box) {
  return {box.x + (0.5 * box.width), box.y + box.height};
}

std::optional<Eigen::Vector2d> ground_contact(const cv::Rect2f& box,
                                              const Eigen::Matrix3d& image_to_ground) {
  const Eigen::Vector3d ground = image_to_ground * contact_pixel(box).homogeneous();
  if (std::abs(ground.z()) < kEpsilon) {
    return std::nullopt;
  }
  return ground.hnormalized();
}

std::optional<Eigen::Matrix2d> ground_jacobian(const Eigen::Vector2d& pixel,
                                               const Eigen::Matrix3d& image_to_ground) {
  const Eigen::Vector3d g = image_to_ground * pixel.homogeneous();
  if (std::abs(g.z()) < kEpsilon) {
    return std::nullopt;
  }
  // d(g.xy / g.z) / d(pixel) = (H.top-left - ground * H.row(2).left) / g.z
  const Eigen::Vector2d ground = g.hnormalized();
  return (image_to_ground.topLeftCorner<2, 2>() - (ground * image_to_ground.block<1, 2>(2, 0))) /
         g.z();
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
