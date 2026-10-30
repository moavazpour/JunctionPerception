#include "jp/vru/zones.hpp"

#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <stdexcept>

namespace jp::vru {

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

std::vector<Zone> parse_zones(std::string_view geojson) {
  std::vector<Zone> zones;
  try {
    const auto doc = nlohmann::json::parse(geojson);
    if (doc.at("type") != "FeatureCollection") {
      throw std::runtime_error("zones must be a GeoJSON FeatureCollection");
    }
    const std::string frame = doc.value("coordinate_frame", std::string("wgs84"));
    if (frame != "scene_ground") {
      throw std::runtime_error("zones in " + frame +
                               " need the scene's geo anchor, which is not configured");
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
        zone.polygon.emplace_back(position.at(0).get<double>(), position.at(1).get<double>());
      }
      // GeoJSON rings repeat the first position at the end.
      if (zone.polygon.size() > 1 && zone.polygon.front() == zone.polygon.back()) {
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

std::vector<Zone> load_zones(const std::filesystem::path& path) {
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("cannot read " + path.string());
  }
  std::ostringstream text;
  text << in.rdbuf();
  return parse_zones(text.str());
}

}  // namespace jp::vru
