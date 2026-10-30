#pragma once

#include <Eigen/Core>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#include "jp/core/geodesy.hpp"
#include "jp/core/scene_config.hpp"

namespace jp::vru {

/// A named area of the scene in world coordinates (metres).
struct Zone {
  std::string id;    ///< e.g. "crosswalk_n".
  std::string type;  ///< "crosswalk" or "intersection".
  std::vector<Eigen::Vector2d> polygon;

  /// Point-in-polygon test (even-odd rule); points on the boundary may go either way.
  [[nodiscard]] bool contains(const Eigen::Vector2d& point) const;
};

/// Parses a zones GeoJSON FeatureCollection of Polygon features with properties "id" and "type"
/// into world coordinates. Positions are WGS84 longitude/latitude (the GeoJSON default) and are
/// converted with the scene's geo anchor; scenes without one may store metres in their ground
/// frame, declared by the foreign member "coordinate_frame": "scene_ground".
/// Throws std::runtime_error on invalid input or WGS84 zones without a geo anchor.
[[nodiscard]] std::vector<Zone> parse_zones(std::string_view geojson,
                                            const core::GeoAnchor* anchor = nullptr);
[[nodiscard]] std::vector<Zone> load_zones(const std::filesystem::path& path,
                                           const core::GeoAnchor* anchor = nullptr);

/// Zones of a scene (zones.geojson, converted with its geo anchor); empty if it has none.
[[nodiscard]] std::vector<Zone> load_scene_zones(const core::SceneConfig& scene);

/// Zones as GeoJSON: WGS84 with a geo anchor, otherwise metres in the scene ground frame.
[[nodiscard]] std::string zones_to_geojson(const std::vector<Zone>& zones,
                                           const core::GeoAnchor* anchor = nullptr);

}  // namespace jp::vru
