#pragma once

#include <filesystem>
#include <optional>
#include <string>

#include "jp/core/geodesy.hpp"

namespace jp::core {

/// Settings of scenes/<id>/scene.yaml that the pipeline needs.
struct SceneConfig {
  std::string scene_id;              ///< Folder name of the scene.
  std::filesystem::path directory;   ///< Scene folder.
  std::string kind;                  ///< "custom" or "benchmark".
  int width_px = 0;                  ///< Camera image size.
  int height_px = 0;
  std::optional<GeoPoint> geo_anchor;  ///< Origin of the world frame; empty if not geo-referenced.
  double map_radius_m = 200.0;       ///< Area around the anchor covered by map layers.

  [[nodiscard]] std::filesystem::path calibration_path() const {
    return directory / "ground_homography.yaml";
  }
  [[nodiscard]] std::filesystem::path zones_path() const { return directory / "zones.geojson"; }
  [[nodiscard]] std::filesystem::path control_points_path() const {
    return directory / "control_points.yaml";
  }
};

/// Loads scene.yaml of a scene folder. Throws std::runtime_error if it is missing or malformed.
[[nodiscard]] SceneConfig load_scene_config(const std::filesystem::path& scene_dir);

}  // namespace jp::core
