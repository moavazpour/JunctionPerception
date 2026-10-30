#include "jp/core/pipeline_config.hpp"

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace jp::core {

namespace {

YAML::Node section(const YAML::Node& root, const char* name) {
  YAML::Node node = root[name];
  if (!node) {
    throw std::runtime_error(std::string("pipeline config has no ") + name + " section");
  }
  return node;
}

}  // namespace

PipelineConfig parse_pipeline_config(std::string_view yaml_text,
                                     const std::filesystem::path& base_dir) {
  PipelineConfig config;
  try {
    const YAML::Node root = YAML::Load(std::string(yaml_text));
    const YAML::Node detection = root["detection"];
    if (!detection) {
      throw std::runtime_error("pipeline config has no detection section");
    }
    const std::filesystem::path model = detection["model"].as<std::string>();
    config.detection.model = model.is_absolute() ? model : base_dir / model;
    config.detection.confidence_threshold = detection["confidence_threshold"].as<float>();
    const auto provider = detection["execution_provider"].as<std::string>();
    if (provider != "cuda" && provider != "cpu") {
      throw std::runtime_error("execution_provider must be cuda or cpu, not " + provider);
    }
    config.detection.use_cuda = provider == "cuda";
    config.detection.cuda_device = detection["cuda_device"].as<int>();
    config.detection.class_source = detection["class_source"].as<std::string>();
    const YAML::Node riders = detection["rider_association"];
    if (!riders) {
      throw std::runtime_error("pipeline config has no detection.rider_association section");
    }
    config.riders.enabled = riders["enabled"].as<bool>();
    config.riders.min_person_overlap = riders["min_person_overlap"].as<float>();
    config.riders.min_feet_depth = riders["min_feet_depth"].as<float>();

    const YAML::Node tracking = section(root, "tracking");
    TrackingSettings& t = config.tracking;
    t.high_score = tracking["high_score"].as<float>();
    t.low_score = tracking["low_score"].as<float>();
    t.new_track_score = tracking["new_track_score"].as<float>();
    t.high_match_min_iou = tracking["high_match_min_iou"].as<float>();
    t.low_match_min_iou = tracking["low_match_min_iou"].as<float>();
    t.tentative_match_min_iou = tracking["tentative_match_min_iou"].as<float>();
    t.confirm_frames = tracking["confirm_frames"].as<int>();
    t.max_missed_frames = tracking["max_missed_frames"].as<int>();
    t.min_rider_votes = tracking["min_rider_votes"].as<float>();

    const YAML::Node localization = section(root, "localization");
    LocalizationSettings& l = config.localization;
    l.bias_correction = localization["bias_correction"].as<bool>();
    l.acceleration_std_mps2 = localization["acceleration_std_mps2"].as<double>();
    l.measurement_std_m = localization["measurement_std_m"].as<double>();
    l.measurement_std_per_m = localization["measurement_std_per_m"].as<double>();
    l.heading_min_speed_mps = localization["heading_min_speed_mps"].as<double>();
    l.forget_after_s = localization["forget_after_s"].as<double>();
  } catch (const YAML::Exception& error) {
    throw std::runtime_error(std::string("invalid pipeline config: ") + error.what());
  }
  if (config.detection.confidence_threshold < 0.0F ||
      config.detection.confidence_threshold > 1.0F) {
    throw std::runtime_error("confidence_threshold must be within [0, 1]");
  }
  return config;
}

PipelineConfig load_pipeline_config(const std::filesystem::path& path,
                                    const std::filesystem::path& base_dir) {
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("cannot read " + path.string());
  }
  std::ostringstream text;
  text << in.rdbuf();
  return parse_pipeline_config(text.str(), base_dir);
}

}  // namespace jp::core
