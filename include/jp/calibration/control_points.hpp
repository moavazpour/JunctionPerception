#pragma once

#include <Eigen/Core>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "jp/calibration/ground_calibration.hpp"
#include "jp/core/geodesy.hpp"

namespace jp::calibration {

/// A point identified both in the camera image and on the map.
struct ControlPoint {
  std::string id;
  Eigen::Vector2d pixel = Eigen::Vector2d::Zero();  ///< Image position (px).
  Eigen::Vector2d world = Eigen::Vector2d::Zero();  ///< Map position in the world frame (m).
  bool check = false;  ///< Check points validate the fit and are not used to compute it.
};

struct PointResidual {
  std::string id;
  bool check = false;
  bool inlier = false;   ///< Fit points: kept by RANSAC.
  double error_m = 0.0;  ///< Distance between the mapped image point and its map position.
};

/// Image-to-ground homography fitted to control points, with its errors in metres.
struct ControlPointFit {
  Eigen::Matrix3d image_to_ground = Eigen::Matrix3d::Identity();
  std::vector<PointResidual> residuals;
  std::size_t fit_points = 0;
  std::size_t inliers = 0;
  double fit_rms_m = 0.0;              ///< Over the RANSAC inliers.
  std::optional<double> check_mean_m;  ///< Over the check points, if any.
  std::optional<double> check_max_m;
  /// Camera recovered from the homography (square pixels, centred principal point).
  std::optional<Eigen::Matrix<double, 3, 4>> ground_projection;
  std::optional<Eigen::Vector3d> camera_position;  ///< x, y on the ground and height (m).
};

/// Fits the homography to the fit points with RANSAC (threshold in metres) and evaluates every
/// point. Throws std::invalid_argument with fewer than four fit points.
[[nodiscard]] ControlPointFit fit_control_points(const std::vector<ControlPoint>& points,
                                                 int width_px, int height_px,
                                                 double ransac_threshold_m = 1.0);

/// Camera centre (x, y, z) of a projection matrix in its world frame.
[[nodiscard]] Eigen::Vector3d camera_center(const Eigen::Matrix<double, 3, 4>& projection);

/// Ground calibration of a custom scene from a fit. `camera_position` overrides the camera
/// ground position recovered from the homography (e.g. marked on the map); with
/// `camera_height_m` as well, the camera projection is derived from that centre instead of the
/// focal length estimate.
[[nodiscard]] GroundCalibration calibration_from_fit(
    const ControlPointFit& fit, const std::string& camera, int width_px, int height_px,
    const std::optional<Eigen::Vector2d>& camera_position = std::nullopt,
    std::optional<double> camera_height_m = std::nullopt);

/// Writes ground_homography.yaml of a custom scene including the fit statistics.
void write_ground_calibration(const std::filesystem::path& path,
                              const GroundCalibration& calibration, const ControlPointFit& fit);

/// control_points.yaml: image positions and WGS84 map positions with their role.
void write_control_points(const std::filesystem::path& path,
                          const std::vector<ControlPoint>& points, const core::GeoAnchor& anchor);
[[nodiscard]] std::vector<ControlPoint> load_control_points(const std::filesystem::path& path,
                                                            const core::GeoAnchor& anchor);

}  // namespace jp::calibration
