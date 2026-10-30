#include "jp/vru/zones.hpp"

#include <cmath>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <stdexcept>

namespace jp::vru {

namespace {

constexpr const char* kSceneGround = "scene_ground";
constexpr double kDegreeDecimals = 1e8;  // about 1 mm
constexpr double kMetreDecimals = 1e3;

double rounded(double value, double scale) { return std::round(value * scale) / scale; }

}  // namespace

bool Zone::contains(const Eigen::Vector2d& point) const {
  bool inside = false;
  const std::size_t n = polygon.size();
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
    const Eigen::Vector2d& a = polygon[i];
    const Eigen::Vector2d& b = polygon[j];
    if ((a.y() > point.y()) != (b.y() > point.y())) {
      const double x_cross = a.x() + ((point.y() - a.y()) * (b.x() - a.x()) / (b.y() - a.y()));
      if (point.x() < x_cross) {
        inside = !inside;
      }
    }
  }
  return inside;
}

std::vector<Zone> parse_zones(std::string_view geojson, const core::GeoAnchor* anchor) {
  std::vector<Zone> zones;
  try {
    const auto doc = nlohmann::json::parse(geojson);
    if (doc.at("type") != "FeatureCollection") {
      throw std::runtime_error("zones must be a GeoJSON FeatureCollection");
    }
    const bool scene_ground = doc.value("coordinate_frame", std::string("wgs84")) == kSceneGround;
    if (!scene_ground && anchor == nullptr) {
      throw std::runtime_error("WGS84 zones need the scene's geo anchor, which is not configured");
    }
    for (const auto& feature : doc.at("features")) {
      const auto& geometry = feature.at("geometry");
      if (geometry.at("type") != "Polygon") {
        throw std::runtime_error("zones must be Polygon features");
      }
      Zone zone;
      zone.id = feature.at("properties").at("id").get<std::string>();
      zone.type = feature.at("properties").at("type").get<std::string>();
      for (const auto& position : geometry.at("coordinates").at(0)) {
        const double a = position.at(0).get<double>();
        const double b = position.at(1).get<double>();
        zone.polygon.push_back(scene_ground ? Eigen::Vector2d(a, b) : anchor->to_world(b, a));
      }
      // GeoJSON rings repeat the first position at the end.
      if (zone.polygon.size() > 1 && zone.polygon.front().isApprox(zone.polygon.back())) {
        zone.polygon.pop_back();
      }
      if (zone.polygon.size() < 3) {
        throw std::runtime_error("zone " + zone.id + " needs at least three corners");
      }
      zones.push_back(std::move(zone));
    }
  } catch (const nlohmann::json::exception& error) {
    throw std::runtime_error(std::string("invalid zones: ") + error.what());
  }
  return zones;
}

std::vector<Zone> load_zones(const std::filesystem::path& path, const core::GeoAnchor* anchor) {
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("cannot read " + path.string());
  }
  std::ostringstream text;
  text << in.rdbuf();
  return parse_zones(text.str(), anchor);
}

std::vector<Zone> load_scene_zones(const core::SceneConfig& scene) {
  if (!std::filesystem::exists(scene.zones_path())) {
    return {};
  }
  if (!scene.geo_anchor) {
    return load_zones(scene.zones_path());
  }
  const core::GeoAnchor anchor(*scene.geo_anchor);
  return load_zones(scene.zones_path(), &anchor);
}

std::string zones_to_geojson(const std::vector<Zone>& zones, const core::GeoAnchor* anchor) {
  nlohmann::ordered_json doc = {{"type", "FeatureCollection"}};
  if (anchor == nullptr) {
    doc["coordinate_frame"] = kSceneGround;
  }
  nlohmann::ordered_json features = nlohmann::ordered_json::array();
  for (const auto& zone : zones) {
    nlohmann::ordered_json ring = nlohmann::ordered_json::array();
    auto corners = zone.polygon;
    if (!corners.empty()) {
      corners.push_back(corners.front());
    }
    for (const auto& corner : corners) {
      if (anchor != nullptr) {
        const auto geo = anchor->to_geo(corner);
        ring.push_back({rounded(geo.lon_deg, kDegreeDecimals), rounded(geo.lat_deg, kDegreeDecimals)});
      } else {
        ring.push_back({rounded(corner.x(), kMetreDecimals), rounded(corner.y(), kMetreDecimals)});
      }
    }
    features.push_back({{"type", "Feature"},
                        {"properties", {{"id", zone.id}, {"type", zone.type}}},
                        {"geometry", {{"type", "Polygon"}, {"coordinates", {ring}}}}});
  }
  doc["features"] = features;
  return doc.dump(2);
}

}  // namespace jp::vru
