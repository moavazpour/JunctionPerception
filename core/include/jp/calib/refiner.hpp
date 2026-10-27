#pragma once

#include <Eigen/Core>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "jp/geometry/camera.hpp"
#include "jp/scene/site_map.hpp"

namespace jp {

// Checks a manual calibration against the traffic and proposes a small correction (never applies it).
//
// Vehicles on a straight road drive parallel to each other and to the lanes. A slightly wrong pitch, roll
// or focal length bends those paths when they are projected onto the ground: far tracks fan out or
// converge. The refiner collects image paths (ground-contact keypoint) of moving vehicles, then searches
// a bounded correction of pitch, roll, yaw and focal length that makes the projected paths parallel to
// their dominant direction (and to the drawn lanes, if any), while the installer's correspondences must
// keep fitting. The proposal is reported with before/after numbers; the operator accepts or rejects it.
class CalibrationRefiner {
 public:
  struct Config {
    int min_tracks = 40;          // straight moving-vehicle paths needed
    double min_path_m = 12.0;     // ground length of a usable path
    double max_rot_deg = 2.5;     // bound of each rotation correction
    double max_focal_rel = 0.12;  // bound of the focal-length correction
    double min_gain = 0.2;        // required relative improvement of the path consistency
  };
  struct Proposal {
    CameraModel camera;
    double spread_before_deg = 0, spread_after_deg = 0;  // RMS deviation of paths from their direction
    double points_before_m = 0, points_after_m = 0;      // installer points, ground RMS
    double d_pitch_deg = 0, d_roll_deg = 0, d_yaw_deg = 0, d_focal_rel = 0;
    int tracks = 0;
  };

  CalibrationRefiner() = default;
  explicit CalibrationRefiner(Config cfg) : cfg_(cfg) {}
  // Starts collecting for a new calibration. Points: installer correspondences (pixel, local ENU).
  void reset(const CameraModel& cam, std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> points,
             std::vector<Lane> lanes = {});
  void setLanes(std::vector<Lane> lanes) { lanes_ = std::move(lanes); }
  void add(std::uint32_t track, double stamp, const Eigen::Vector2d& px);
  int usableTracks() const;
  double progress() const { return std::min(1.0, usableTracks() / static_cast<double>(cfg_.min_tracks)); }
  bool ready() const { return usableTracks() >= cfg_.min_tracks; }
  bool active() const { return cam_.valid(); }
  // Empty when the calibration is already consistent (or the correction would not help enough).
  std::optional<Proposal> solve() const;
  // RMS deviation (deg) of the projected paths from their direction under the current calibration.
  double currentSpread() const { return spread(cam_); }

 private:
  struct Path {
    std::vector<Eigen::Vector2d> px;
    double last = 0;
  };
  double spread(const CameraModel& cam, int* used = nullptr) const;
  double pointsRms(const CameraModel& cam) const;
  CameraModel apply(const Eigen::Vector4d& d) const;

  Config cfg_;
  CameraModel cam_;
  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> points_;
  std::vector<Lane> lanes_;
  std::map<std::uint32_t, Path> paths_;
};

}  // namespace jp
