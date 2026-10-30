#include "jp/localization/box_fit.hpp"

#include <Eigen/LU>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>

namespace jp::localization {

namespace {

constexpr double kMinDepthM = 0.5;
constexpr double kJacobianStepM = 0.01;

// Residual between the projected and the detected box bottom centre (px).
std::optional<Eigen::Vector2d> bottom_center_residual(const CameraModel& camera,
                                                      const Eigen::Vector2d& center, double yaw,
                                                      const ObjectSize& size,
                                                      const cv::Rect2f& detection) {
  const auto box = project_object_box(camera, center, yaw, size);
  if (!box) {
    return std::nullopt;
  }
  return Eigen::Vector2d((box->x + (0.5 * box->width)) - (detection.x + (0.5 * detection.width)),
                         (box->y + box->height) - (detection.y + detection.height));
}

std::optional<Eigen::Vector2d> fit_with_heading(const CameraModel& camera,
                                                const cv::Rect2f& detection, const ObjectSize& size,
                                                double yaw, const Eigen::Vector2d& initial,
                                                const BoxFitOptions& options) {
  Eigen::Vector2d center = initial;
  for (int i = 0; i < options.iterations; ++i) {
    const auto r = bottom_center_residual(camera, center, yaw, size, detection);
    if (!r) {
      return std::nullopt;
    }
    Eigen::Matrix2d jacobian;
    for (int axis = 0; axis < 2; ++axis) {
      Eigen::Vector2d shifted = center;
      shifted(axis) += kJacobianStepM;
      const auto rs = bottom_center_residual(camera, shifted, yaw, size, detection);
      if (!rs) {
        return std::nullopt;
      }
      jacobian.col(axis) = (*rs - *r) / kJacobianStepM;
    }
    if (std::abs(jacobian.determinant()) < std::numeric_limits<double>::epsilon()) {
      return std::nullopt;
    }
    const Eigen::Vector2d step = jacobian.partialPivLu().solve(-*r);
    center += step;
    if ((center - initial).norm() > options.max_shift_m) {
      return std::nullopt;
    }
    if (step.norm() < options.tolerance_m) {
      return center;
    }
  }
  return center;
}

float iou(const cv::Rect2f& a, const cv::Rect2f& b) {
  const float intersection = (a & b).area();
  const float union_area = a.area() + b.area() - intersection;
  return union_area > 0.0F ? intersection / union_area : 0.0F;
}

bool touches_border(const CameraModel& camera, const cv::Rect2f& box, float margin) {
  return box.x <= margin || box.y <= margin ||
         box.x + box.width >= static_cast<float>(camera.width_px) - margin ||
         box.y + box.height >= static_cast<float>(camera.height_px) - margin;
}

}  // namespace

std::optional<cv::Rect2f> project_object_box(const CameraModel& camera,
                                             const Eigen::Vector2d& center, double yaw_rad,
                                             const ObjectSize& size) {
  const Eigen::Vector2d forward(std::cos(yaw_rad), std::sin(yaw_rad));
  const Eigen::Vector2d left(-forward.y(), forward.x());
  float min_u = std::numeric_limits<float>::max();
  float min_v = std::numeric_limits<float>::max();
  float max_u = std::numeric_limits<float>::lowest();
  float max_v = std::numeric_limits<float>::lowest();
  for (const double sx : {-0.5, 0.5}) {
    for (const double sy : {-0.5, 0.5}) {
      const Eigen::Vector2d xy =
          center + (sx * size.length_m * forward) + (sy * size.width_m * left);
      for (const double z : {0.0, size.height_m}) {
        const Eigen::Vector3d p = camera.projection * Eigen::Vector4d(xy.x(), xy.y(), z, 1.0);
        if (p.z() < kMinDepthM) {
          return std::nullopt;
        }
        const auto u = static_cast<float>(p.x() / p.z());
        const auto v = static_cast<float>(p.y() / p.z());
        min_u = std::min(min_u, u);
        max_u = std::max(max_u, u);
        min_v = std::min(min_v, v);
        max_v = std::max(max_v, v);
      }
    }
  }
  return cv::Rect2f(min_u, min_v, max_u - min_u, max_v - min_v);
}

std::optional<Eigen::Vector2d> fit_footprint_center(const CameraModel& camera,
                                                    const cv::Rect2f& detection,
                                                    const ObjectSize& size,
                                                    std::optional<double> heading_rad,
                                                    const Eigen::Vector2d& initial,
                                                    const BoxFitOptions& options) {
  if (touches_border(camera, detection, options.border_margin_px)) {
    return std::nullopt;
  }
  if (heading_rad) {
    return fit_with_heading(camera, detection, size, *heading_rad, initial, options);
  }
  std::optional<Eigen::Vector2d> best;
  float best_overlap = -1.0F;
  for (int k = 0; k < options.heading_candidates; ++k) {
    const double yaw = std::numbers::pi * k / options.heading_candidates;
    const auto center = fit_with_heading(camera, detection, size, yaw, initial, options);
    if (!center) {
      continue;
    }
    const auto box = project_object_box(camera, *center, yaw, size);
    const float overlap = box ? iou(*box, detection) : 0.0F;
    if (overlap > best_overlap) {
      best_overlap = overlap;
      best = center;
    }
  }
  return best;
}

}  // namespace jp::localization
