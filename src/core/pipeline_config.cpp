#include "jp/core/pipeline_config.hpp"

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace jp::core {

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
