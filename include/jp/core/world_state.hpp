#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace jp::core {

/// Version of the camera observation schema (docs/architecture.md). Version 2 dropped events,
/// zones and turning flags (no longer computed) and added camera_id and time_source.
inline constexpr int kWorldStateSchemaVersion = 2;

/// One road user in the world frame (metres, ground plane z = 0).
struct WorldObject {
  std::int64_t id = 0;
  std::string object_class;  ///< Twin class, e.g. "car", "pedestrian".
  bool vru = false;          ///< Vulnerable road user (class property).
  bool rider = false;        ///< Two-wheeler with rider.
  double x = 0.0;
  double y = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double speed_mps = 0.0;
  std::optional<double> speed_std_mps;  ///< Speed uncertainty; empty if not estimated.
  std::optional<double> heading_rad;    ///< Empty while the heading is unknown.
  /// Where the heading comes from: "motion" (direction of travel), "keypoints" (box corners),
  /// "held" (last heading kept while too slow to measure it) or "label" (ground truth); empty
  /// while the heading is unknown or for sources that do not say.
  std::optional<std::string> heading_source;
  /// Position covariance of the tracking filter (xx, xy, yy in m²); empty if not estimated.
  std::optional<std::array<double, 3>> position_cov_m2;
  double confidence = 0.0;
  /// Measured footprint (keypoint models), else empty: the twin uses the class size then.
  std::optional<double> length_m;
  std::optional<double> width_m;
  /// Inside the region where the calibration is validated (near its control points, fine ground
  /// sampling); positions outside are less trustworthy.
  bool validated = true;
};

/// State of a traffic light watched by a camera (a signal area drawn on its image).
struct SignalState {
  std::string id;
  std::string name;
  std::string state;  ///< "red", "yellow", "red_yellow", "green", "off" or "unknown".
  double confidence = 0.0;
  std::int64_t since_ns = 0;  ///< Timestamp of the frame the state began.
};

/// What one camera observes at one frame: its tracks in the site world frame.
struct WorldState {
  int schema_version = kWorldStateSchemaVersion;
  std::string scene_id;
  std::int64_t frame = 0;
  /// Time of the frame; what it counts is said by time_source.
  std::int64_t timestamp_ns = 0;
  /// "media": since the start of the recording (videos, image sequences without times);
  /// "capture": absolute capture time, Unix ns (sequences with timestamps, e.g. TUMTraf);
  /// "arrival": absolute time the frame was received, Unix ns (live streams).
  std::string time_source = "media";
  std::string camera_id;  ///< Camera folder of a site; empty for single-camera scenes.
  std::string source;     ///< "live", "oracle" or "replay".
  std::vector<WorldObject> objects;
  std::vector<SignalState> signals;  ///< Traffic lights of the camera (live sources).
};

}  // namespace jp::core
