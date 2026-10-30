#pragma once

#include <Eigen/Geometry>

namespace jp::core {

/// Inverse of a rigid transform (rotation + translation), computed as [R^T | -R^T t].
[[nodiscard]] inline Eigen::Isometry3d rigid_inverse(const Eigen::Isometry3d& transform) {
  Eigen::Isometry3d inverse = Eigen::Isometry3d::Identity();
  inverse.linear() = transform.linear().transpose();
  inverse.translation() = -(inverse.linear() * transform.translation());
  return inverse;
}

}  // namespace jp::core
