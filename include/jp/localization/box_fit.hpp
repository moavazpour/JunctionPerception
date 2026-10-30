#pragma once

#include <Eigen/Core>
#include <opencv2/core/types.hpp>
#include <optional>

namespace jp::localization {

/// Fixed object size of a class (m).
struct ObjectSize {
  double length_m = 0.0;
  double width_m = 0.0;
  double height_m = 0.0;
};

/// Pinhole camera seen from the ground frame.
struct CameraModel {
  Eigen::Matrix<double, 3, 4> projection;  ///< Ground frame (m) -> homogeneous pixels.
  int width_px = 0;
  int height_px = 0;
};

/// Image bounding box of a box of the given size standing on the ground at `center` with
/// heading `yaw_rad`; empty if any corner is behind or too close to the camera.
[[nodiscard]] std::optional<cv::Rect2f> project_object_box(const CameraModel& camera,
                                                           const Eigen::Vector2d& center,
                                                           double yaw_rad, const ObjectSize& size);

struct BoxFitOptions {
  int iterations = 15;
  double tolerance_m = 1e-3;
  double max_shift_m = 10.0;      ///< Fits moving further from the initial guess are rejected.
  int heading_candidates = 12;    ///< Headings tried over half a turn when the heading is unknown.
  float border_margin_px = 2.0F;  ///< Boxes touching the image border are not fitted.
};

/// Ground position of the footprint centre whose projected box has the same bottom centre as
/// the detected 2D box (Gauss-Newton from `initial`). With an unknown heading, headings over
/// half a turn are tried and the one whose projected box best overlaps the detection is kept.
/// Empty if the box touches the image border or the fit does not converge.
[[nodiscard]] std::optional<Eigen::Vector2d> fit_footprint_center(
    const CameraModel& camera, const cv::Rect2f& detection, const ObjectSize& size,
    std::optional<double> heading_rad, const Eigen::Vector2d& initial,
    const BoxFitOptions& options = {});

}  // namespace jp::localization
