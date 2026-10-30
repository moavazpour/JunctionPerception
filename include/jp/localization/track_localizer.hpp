#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "jp/core/class_catalog.hpp"
#include "jp/localization/bev_kalman.hpp"
#include "jp/tracking/byte_tracker.hpp"

namespace jp::localization {

struct LocalizerOptions {
  bool bias_correction = true;
  double acceleration_std_mps2 = 2.0;
  double measurement_std_m = 0.3;       ///< Measurement std close to the camera.
  double measurement_std_per_m = 0.01;  ///< Added std per metre of distance from the camera.
  double heading_min_speed_mps = 0.5;   ///< Below this speed the last heading is held.
  double forget_after_s = 5.0;          ///< Filters of tracks unseen this long are dropped.
};

/// A tracked object on the ground plane.
struct LocalizedObject {
  std::int64_t id = 0;
  std::string twin_class;
  bool rider = false;
  float confidence = 0.0F;
  cv::Rect2f box;                                      ///< Image box of this frame.
  Eigen::Vector2d contact = Eigen::Vector2d::Zero();   ///< Box bottom centre on the ground.
  Eigen::Vector2d position = Eigen::Vector2d::Zero();  ///< Filtered footprint centre.
  Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
  double speed_mps = 0.0;
  std::optional<double> heading_rad;  ///< Empty until the object has moved fast enough.
};

/// Image tracks -> ground plane: box bottom centre through the image-to-ground homography,
/// optional shift from the visible contact to the footprint centre, a constant-velocity
/// Kalman filter per track, and heading from velocity (held while slow).
class TrackLocalizer {
 public:
  TrackLocalizer(Eigen::Matrix3d image_to_ground, Eigen::Vector2d camera_position,
                 core::ClassCatalog catalog, LocalizerOptions options = {});

  [[nodiscard]] std::vector<LocalizedObject> update(
      const std::vector<tracking::TrackedObject>& tracks, std::int64_t timestamp_ns);

 private:
  struct State {
    explicit State(BevKalman initial) : filter(std::move(initial)) {}

    BevKalman filter;
    std::optional<double> heading_rad;
  };

  Eigen::Matrix3d image_to_ground_;
  Eigen::Vector2d camera_position_;
  core::ClassCatalog catalog_;
  LocalizerOptions options_;
  std::map<std::int64_t, State> states_;
};

}  // namespace jp::localization
