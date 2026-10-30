#include "jp/calibration/ground_homography.hpp"

#include <cmath>
#include <stdexcept>

namespace jp::calibration {

namespace {

constexpr double kEpsilon = 1e-12;

}  // namespace

Eigen::Isometry3d sensor_from_ground_frame(const Plane& plane, const Eigen::Vector3d& origin_hint) {
  const Eigen::Vector3d z = plane.normal.normalized();
  Eigen::Vector3d x = Eigen::Vector3d::UnitX() - (Eigen::Vector3d::UnitX().dot(z) * z);
  if (x.norm() < kEpsilon) {
    throw std::invalid_argument("ground plane is perpendicular to the sensor x axis");
  }
  x.normalize();
  Eigen::Isometry3d frame = Eigen::Isometry3d::Identity();
  frame.linear().col(0) = x;
  frame.linear().col(1) = z.cross(x);
  frame.linear().col(2) = z;
  frame.translation() = plane.project(origin_hint);
  return frame;
}

Eigen::Matrix3d ground_to_image_homography(const Eigen::Matrix<double, 3, 4>& projection,
                                           const Eigen::Isometry3d& sensor_from_ground) {
  const Eigen::Matrix<double, 3, 4> m = projection * sensor_from_ground.matrix();
  Eigen::Matrix3d h;
  h.col(0) = m.col(0);
  h.col(1) = m.col(1);
  h.col(2) = m.col(3);
  return normalized_homography(h);
}

std::optional<Eigen::Vector2d> apply_homography(const Eigen::Matrix3d& h,
                                                const Eigen::Vector2d& point) {
  const Eigen::Vector3d p = h * point.homogeneous();
  if (std::abs(p.z()) < kEpsilon) {
    return std::nullopt;
  }
  return p.hnormalized();
}

Eigen::Matrix3d normalized_homography(const Eigen::Matrix3d& h) {
  if (std::abs(h(2, 2)) < kEpsilon) {
    throw std::invalid_argument("homography cannot be normalised (h22 = 0)");
  }
  return h / h(2, 2);
}

}  // namespace jp::calibration
