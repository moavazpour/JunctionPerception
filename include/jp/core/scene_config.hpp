#pragma once

#include <filesystem>
#include <optional>
#include <string>

#include "jp/core/geodesy.hpp"

namespace jp::core {

/**
 * Settings of one camera of a scene that the pipeline needs. A scene is a site: either a
 * single-camera folder (scenes/<id>/scene.yaml with the calibration next to it) or an
 * intersection with up to four cameras, scenes/<id>/scene.yaml for the site (world frame, map,
 * zones) and scenes/<id>/cameras/<camera>/camera.yaml with each camera's image size next to its
 * calibration. All cameras of a site share its world frame and zones.
 */
struct SceneConfig {
  std::string scene_id;                  ///< Folder name of the scene (the site).
  std::string camera_id;                 ///< Camera folder name; empty for single-camera scenes.
  std::filesystem::path directory;       ///< Folder of the camera and its calibration.
  std::filesystem::path site_directory;  ///< Folder of the scene: scene.yaml and zones.
  std::string kind;                      ///< "custom" or "benchmark".
  int width_px = 0;                      ///< Camera image size.
  int height_px = 0;
  std::optional<GeoPoint> geo_anchor;  ///< Origin of the world frame; empty if not geo-referenced.
  double map_radius_m = 200.0;         ///< Area around the anchor covered by map layers.

  [[nodiscard]] std::filesystem::path calibration_path() const {
    return directory / "ground_homography.yaml";
  }
  [[nodiscard]] std::filesystem::path zones_path() const {
    return site_directory / "zones.geojson";
  }
  [[nodiscard]] std::filesystem::path control_points_path() const {
    return directory / "control_points.yaml";
  }
};

/**
 * Loads a single-camera scene folder or a camera folder of an intersection (one with
 * camera.yaml). Throws std::runtime_error if a file is missing or malformed.
 */
[[nodiscard]] SceneConfig load_scene_config(const std::filesystem::path& scene_dir);

}  // namespace jp::core
