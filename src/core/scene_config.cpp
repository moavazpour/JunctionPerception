#include "jp/core/scene_config.hpp"

#include <yaml-cpp/yaml.h>

#include <stdexcept>

namespace jp::core {

SceneConfig load_scene_config(const std::filesystem::path& scene_dir) {
  const auto path = scene_dir / "scene.yaml";
  SceneConfig config;
  config.directory = scene_dir;
  config.scene_id = scene_dir.filename().string();
  try {
    const YAML::Node root = YAML::LoadFile(path.string());
    config.kind = root["kind"].as<std::string>();
    config.width_px = root["image_size_px"][0].as<int>();
    config.height_px = root["image_size_px"][1].as<int>();
    if (const YAML::Node anchor = root["geo_anchor"]) {
      config.geo_anchor = GeoPoint{anchor["lat_deg"].as<double>(), anchor["lon_deg"].as<double>(),
                                   anchor["alt_m"].as<double>()};
    }
    if (const YAML::Node map = root["map"]) {
      config.map_radius_m = map["radius_m"].as<double>();
    }
  } catch (const YAML::Exception& error) {
    throw std::runtime_error("invalid scene config " + path.string() + ": " + error.what());
  }
  return config;
}

}  // namespace jp::core
