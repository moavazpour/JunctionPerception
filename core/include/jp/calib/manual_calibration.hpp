#pragma once

#include <Eigen/Core>
#include <array>
#include <optional>
#include <string>
#include <vector>

#include "jp/geometry/camera.hpp"

namespace jp {

// One installer-clicked correspondence: a ground point seen in the image and the same point on a map.
struct GroundCorrespondence {
  Eigen::Vector2d px;  // undistorted frame pixel
  double lat = 0, lon = 0;
};

// A straight marking (lane line, kerb, stop line) traced in the image (two or more pixels anywhere along
// it) and the same marking on the map (two points on it). The map points need not match the traced pixels.
struct LineCorrespondence {
  std::vector<Eigen::Vector2d> px;
  std::array<Eigen::Vector2d, 2> geo;  // (lat, lon) x 2
};

// Vertical structure (pole, building edge) traced in the image: two pixels on it, no map counterpart.
using VerticalLine = std::array<Eigen::Vector2d, 2>;

struct ManualCalibrationInput {
  int width = 0, height = 0;
  std::vector<GroundCorrespondence> points;
  std::vector<LineCorrespondence> lines;
  std::vector<VerticalLine> verticals;
  GeoAnchor anchor;               // configured camera position (defines the local frame)
  double anchor_sigma_m = 30.0;   // how far the real camera may be from it
  std::optional<double> height_m;  // installer's estimate of the mounting height
  double height_sigma = 2.0;
  std::optional<double> focal_px;  // known intrinsics (skips the focal search)
  // Lens hint: "unknown", "phone_wide", "phone_2x", "phone_3x", "cctv". Sets the focal-length prior.
  std::string lens = "unknown";
  std::vector<double> dist;
  double click_sigma_px = 0;  // 0: derived from the image size (calibration view is 960 px wide)
  double map_sigma_m = 0.35;  // aerial image registration + clicking
};

struct ManualCalibrationResult {
  bool ok = false;
  bool plausible = false;  // a physically possible camera (may still fit the points poorly)
  CameraModel camera;
  double rms_px = 0;  // reprojection error of the point correspondences
  double max_px = 0;
  double ground_rms_m = 0, ground_max_m = 0;  // the same points measured on the ground
  std::string quality;                         // good | fair | poor
  std::string message;
  std::vector<std::string> warnings;
  struct Residual {
    std::string kind;  // point | line | vertical
    int index = 0;
    double px = 0;  // image error
    double m = 0;   // ground error (points and lines)
  };
  std::vector<Residual> residuals;
  // Expected ground-position error from the calibration alone, along the viewing direction.
  struct Accuracy {
    double range_m, sigma_m;
  };
  std::vector<Accuracy> accuracy;
  double focal_px = 0, fov_deg = 0;  // fov along the long image side
};

// Camera (focal length, orientation, position, height) from map correspondences on the road surface.
//
// Maximum a-posteriori estimate: reprojection of the points and lines (each weighted by click and map
// accuracy at its distance, with a robust loss) plus soft priors on the focal length (lens hint), the
// mounting height and the camera position. Multi-start over the focal length, Levenberg-Marquardt over
// all seven parameters. The configured anchor only defines the local ENU frame; the returned camera's
// anchor is the ground point below the solved camera centre.
ManualCalibrationResult calibrateManual(const ManualCalibrationInput& in);

// Points-only convenience (no lines, no height prior, unknown lens).
ManualCalibrationResult calibrateFromCorrespondences(int width, int height,
                                                     const std::vector<GroundCorrespondence>& pts,
                                                     const GeoAnchor& anchor,
                                                     std::optional<double> focal_px = std::nullopt,
                                                     const std::vector<double>& dist = {});

}  // namespace jp
