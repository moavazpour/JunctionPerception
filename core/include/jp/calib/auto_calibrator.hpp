#pragma once

#include <Eigen/Core>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "jp/common/types.hpp"
#include "jp/geometry/camera.hpp"

namespace jp {

// Self-calibration from traffic. Needs only the calibration-free network outputs, the image size and
// weak priors (mounting height, geo-position, road bearing from the map).
//
//  1. Image-space tracks of vehicle footprints give the road vanishing point (RANSAC over track lines).
//     Every ground plane compatible with that vanishing point is parameterised by focal length and one
//     rotation about the vanishing direction.
//  2. Focal length, that rotation and the camera height are found by minimising
//       - disagreement between lifted 3D boxes (re-projected) and the network's 2D boxes,
//       - speed variation along each track (perspective errors make speeds change with range),
//       - priors on height, roll and focal length,
//     with a coarse grid search followed by Nelder-Mead refinement.
//  3. The ground frame is rotated so that the vanishing direction matches the road bearing.
struct CalibrationPriors {
  double height_m = 7.0, height_sigma = 2.0;
  std::optional<double> focal_px;            // fix the focal length when known
  double focal_min_rel = 0.35, focal_max_rel = 8.0;  // relative to image width
  double roll_sigma_deg = 3.0;
  std::optional<double> road_bearing_deg;    // bearing of the dominant road (from map)
  double camera_bearing_deg = 0.0;           // rough viewing direction (used to resolve +-180 deg)
  GeoAnchor anchor;
};

struct CalibrationResult {
  bool ok = false;
  CameraModel camera;
  Eigen::Vector2d vanishing_point{0, 0};
  int tracks_used = 0, detections_used = 0;
  double box_residual = 0;    // median relative 2D box disagreement
  double speed_cv = 0;        // median coefficient of variation of speed along tracks
  double confidence = 0;
  std::string message;
};

class AutoCalibrator {
 public:
  struct Config {
    double min_duration_s = 45.0;
    int min_tracks = 12;
    int min_detections = 300;
    float min_score = 0.45f;
  };
  AutoCalibrator(int width, int height, CalibrationPriors priors, Config cfg);
  AutoCalibrator(int width, int height, CalibrationPriors priors) : AutoCalibrator(width, height, std::move(priors), Config{}) {}
  void add(double stamp, const std::vector<ImageDetection>& dets);
  bool ready() const;
  double progress() const;
  CalibrationResult solve() const;

  // Exposed for tests.
  struct ImgTrack {
    std::vector<double> t;
    std::vector<Eigen::Vector2d> kp;
    std::vector<ImageDetection> det;
  };
  static std::optional<Eigen::Vector2d> vanishingPoint(const std::vector<ImgTrack>& tracks, int width, int height);

 private:
  struct Params {
    double logf, psi, height;
  };
  CameraModel build(const Params& p, const Eigen::Vector2d& vp, double bearing = 0.0) const;
  double cost(const Params& p, const Eigen::Vector2d& vp, double* box_res = nullptr, double* speed_cv = nullptr) const;

  int w_, h_;
  CalibrationPriors pri_;
  Config cfg_;
  double t0_ = -1, t_last_ = 0;
  std::vector<ImgTrack> done_;
  std::vector<ImgTrack> live_;
  size_t n_dets_ = 0;
};

}  // namespace jp
