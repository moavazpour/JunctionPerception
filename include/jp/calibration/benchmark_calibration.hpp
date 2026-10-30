#pragma once

#include <Eigen/Geometry>
#include <filesystem>
#include <opencv2/core/mat.hpp>
#include <string>
#include <vector>

#include "jp/benchmark/label_merge.hpp"
#include "jp/benchmark/openlabel.hpp"
#include "jp/calibration/ground_plane.hpp"

namespace jp::calibration {

/// Ground-plane localisation error of one range of camera distances.
struct DistanceBin {
  double from_m = 0.0;
  double to_m = 0.0;
  std::size_t objects = 0;
  double median_m = 0.0;
  double p90_m = 0.0;
};

/// Error of mapping ground-truth box bottom centres image -> ground with the homography:
/// each bottom centre is projected into the image with the full camera model and mapped back
/// onto the ground plane; the error is the BEV distance to the true bottom centre.
struct GroundCheck {
  std::size_t objects = 0;
  double mean_m = 0.0;
  double median_m = 0.0;
  double p90_m = 0.0;
  double max_m = 0.0;
  double median_height_above_plane_m = 0.0;
  std::vector<DistanceBin> by_distance;
};

/// Image <-> ground calibration of one benchmark camera. The sensor frame is the south LiDAR
/// frame of the TUMTraf labels; the ground frame lies on the fitted ground plane (z = 0).
struct BenchmarkCameraCalibration {
  std::string camera;
  int width_px = 0;
  int height_px = 0;
  Eigen::Isometry3d sensor_from_ground = Eigen::Isometry3d::Identity();
  Eigen::Matrix3d ground_to_image = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d image_to_ground = Eigen::Matrix3d::Identity();
  Eigen::Vector2d camera_ground_position_m = Eigen::Vector2d::Zero();
  double camera_height_m = 0.0;
  GroundCheck check;
};

/// Fits the ground plane to the bottom centres of all ground-truth boxes (south LiDAR frame).
[[nodiscard]] PlaneFit fit_ground_plane(
    const std::vector<std::vector<benchmark::GroundTruthObject>>& frames,
    const PlaneFitOptions& options = {});

/// Fits the ground plane to the bottom centres of the ground-truth boxes a camera sees, so that
/// the plane approximates the ground in that camera's view.
[[nodiscard]] PlaneFit fit_ground_plane_in_view(
    const std::vector<std::vector<benchmark::GroundTruthObject>>& frames,
    const benchmark::CameraCalibration& camera, const PlaneFitOptions& options = {});

/// Builds the ground homography of a camera from its projection matrix and the ground plane,
/// and evaluates it on the ground-truth boxes the camera sees.
[[nodiscard]] BenchmarkCameraCalibration calibrate_benchmark_camera(
    const benchmark::CameraCalibration& camera, const Plane& ground,
    const std::vector<std::vector<benchmark::GroundTruthObject>>& frames);

/// Draws ground-truth footprints (mapped through the ground homography), track IDs, classes and
/// a 10 m ground grid onto a camera image.
[[nodiscard]] cv::Mat draw_ground_truth_overlay(
    const cv::Mat& image, const BenchmarkCameraCalibration& calibration,
    const benchmark::CameraCalibration& camera, const Plane& ground,
    const std::vector<benchmark::GroundTruthObject>& objects);

/// Writes the calibration as ground_homography.yaml.
void write_ground_homography(const std::filesystem::path& path,
                             const BenchmarkCameraCalibration& calibration, const PlaneFit& ground,
                             const std::string& source_sequence);

}  // namespace jp::calibration
