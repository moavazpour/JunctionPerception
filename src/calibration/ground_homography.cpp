#include "jp/calibration/ground_homography.hpp"

#include <Eigen/LU>
#include <Eigen/SVD>
#include <array>
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

std::optional<Eigen::Matrix<double, 3, 4>> projection_from_homography(
    const Eigen::Matrix3d& ground_to_image, int width_px, int height_px) {
  const double cx = 0.5 * width_px;
  const double cy = 0.5 * height_px;
  // Columns with the principal point removed: K^-1 h = (a.x / f, a.y / f, a.z).
  Eigen::Matrix3d a = ground_to_image;
  a.row(0) -= cx * ground_to_image.row(2);
  a.row(1) -= cy * ground_to_image.row(2);
  // Rotation columns r1, r2 must be orthogonal and of equal length. Both constraints are linear
  // in w = 1 / f^2: b * w + c = 0; solved together in the least-squares sense.
  const std::array<double, 2> b = {
      (a(0, 0) * a(0, 1)) + (a(1, 0) * a(1, 1)),
      (a(0, 0) * a(0, 0)) + (a(1, 0) * a(1, 0)) - (a(0, 1) * a(0, 1)) - (a(1, 1) * a(1, 1))};
  const std::array<double, 2> c = {a(2, 0) * a(2, 1), (a(2, 0) * a(2, 0)) - (a(2, 1) * a(2, 1))};
  const double normal = (b[0] * b[0]) + (b[1] * b[1]);
  if (normal < kEpsilon) {
    return std::nullopt;
  }
  const double w = -((b[0] * c[0]) + (b[1] * c[1])) / normal;
  if (w <= 0.0) {
    return std::nullopt;
  }
  const double f = 1.0 / std::sqrt(w);
  Eigen::Matrix3d k;
  k << f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0;
  Eigen::Matrix3d m = k.inverse() * ground_to_image;
  const double scale = 2.0 / (m.col(0).norm() + m.col(1).norm());
  m *= scale;
  // Ground points in front of the camera need positive depth: check the image centre's ray.
  const Eigen::Vector3d center_ground = ground_to_image.inverse() * Eigen::Vector3d(cx, cy, 1.0);
  if ((m * (center_ground / center_ground.z())).z() < 0.0) {
    m = -m;
  }
  Eigen::Matrix3d rotation;
  rotation.col(0) = m.col(0);
  rotation.col(1) = m.col(1);
  rotation.col(2) = m.col(0).cross(m.col(1));
  const Eigen::JacobiSVD<Eigen::Matrix3d> svd(rotation, Eigen::ComputeFullU | Eigen::ComputeFullV);
  rotation = svd.matrixU() * svd.matrixV().transpose();
  Eigen::Matrix<double, 3, 4> extrinsics;
  extrinsics.leftCols<3>() = rotation;
  extrinsics.col(3) = m.col(2);
  return k * extrinsics;
}

Eigen::Matrix3d normalized_homography(const Eigen::Matrix3d& h) {
  if (std::abs(h(2, 2)) < kEpsilon) {
    throw std::invalid_argument("homography cannot be normalised (h22 = 0)");
  }
  return h / h(2, 2);
}

}  // namespace jp::calibration
