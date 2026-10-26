#pragma once

#include <Eigen/Core>
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

struct ManualCalibrationResult {
  bool ok = false;
  CameraModel camera;
  double rms_px = 0;           // reprojection error of the correspondences
  double max_px = 0;
  std::string message;
};

// Camera from >= 4 ground correspondences (map coordinates of points on the road surface).
// The focal length is estimated from the ground homography (principal point at the image centre) unless
// given, then focal length and pose are refined by minimising the reprojection error. The camera position
// is solved as well: the configured anchor only defines the local ENU frame; the returned camera's anchor
// is the ground point below the solved camera centre.
ManualCalibrationResult calibrateFromCorrespondences(int width, int height,
                                                     const std::vector<GroundCorrespondence>& pts,
                                                     const GeoAnchor& anchor,
                                                     std::optional<double> focal_px = std::nullopt,
                                                     const std::vector<double>& dist = {});

}  // namespace jp
