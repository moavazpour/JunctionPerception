#include "jp/core/scene_config.hpp"

#include <yaml-cpp/yaml.h>

#include <stdexcept>

namespace jp::core {

namespace {

YAML::Node load(const std::filesystem::path& path) {
  try {
    return YAML::LoadFile(path.string());
  } catch (const YAML::Exception& error) {
    throw std::runtime_error("invalid scene config " + path.string() + ": " + error.what());
  }
}

}  // namespace

SceneConfig load_scene_config(const std::filesystem::path& scene_dir) {
  SceneConfig config;
  config.directory = scene_dir;
  config.site_directory = scene_dir;
  const auto camera_path = scene_dir / "camera.yaml";
  YAML::Node camera;
  if (std::filesystem::exists(camera_path)) {
    // A camera of an intersection: scenes/<site>/cameras/<camera>/.
    camera = load(camera_path);
    config.camera_id = scene_dir.filename().string();
    config.site_directory = scene_dir.parent_path().parent_path();
  }
  config.scene_id = config.site_directory.filename().string();
  const auto path = config.site_directory / "scene.yaml";
  const YAML::Node root = load(path);
  try {
    config.kind = root["kind"].as<std::string>();
    const YAML::Node size =
        config.camera_id.empty() ? root["image_size_px"] : camera["image_size_px"];
    config.width_px = size[0].as<int>();
    config.height_px = size[1].as<int>();
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
