#pragma once

#include <Eigen/Core>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace jp::vru {

/// A named area of the scene in world coordinates (metres).
struct Zone {
  std::string id;    ///< e.g. "crosswalk_n".
  std::string type;  ///< "crosswalk" or "intersection".
  std::vector<Eigen::Vector2d> polygon;

  /// Point-in-polygon test (even-odd rule); points on the boundary may go either way.
  [[nodiscard]] bool contains(const Eigen::Vector2d& point) const;
};

/// Parses a zones GeoJSON FeatureCollection of Polygon features with properties "id" and "type".
/// The foreign member "coordinate_frame" selects the coordinates: "scene_ground" (metres in the
/// scene's ground frame) is supported; WGS84 zones (the GeoJSON default) need the scene's geo
/// anchor and are rejected until it is configured. Throws std::runtime_error on invalid input.
[[nodiscard]] std::vector<Zone> parse_zones(std::string_view geojson);
[[nodiscard]] std::vector<Zone> load_zones(const std::filesystem::path& path);

}  // namespace jp::vru
