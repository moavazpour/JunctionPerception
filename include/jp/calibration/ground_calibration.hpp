#pragma once

#include <Eigen/Geometry>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

namespace jp::calibration {

/// Image <-> ground calibration of a scene camera, as stored in ground_homography.yaml.
struct GroundCalibration {
  std::string camera;
  int width_px = 0;
  int height_px = 0;
  Eigen::Matrix3d image_to_ground = Eigen::Matrix3d::Identity();
  Eigen::Vector2d camera_ground_position_m = Eigen::Vector2d::Zero();
  double camera_height_m = 0.0;
  /// Benchmark scenes: pose of the ground frame in the dataset's sensor frame.
  std::optional<Eigen::Isometry3d> sensor_from_ground;
};

/// Throws std::runtime_error on missing keys or malformed matrices.
[[nodiscard]] GroundCalibration parse_ground_calibration(std::string_view yaml_text);
[[nodiscard]] GroundCalibration load_ground_calibration(const std::filesystem::path& path);

}  // namespace jp::calibration
