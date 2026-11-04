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
    l.bias_correction = localization["bias_correction"].as<std::string>();
    if (l.bias_correction != "box_fit" && l.bias_correction != "ray_offset" &&
        l.bias_correction != "none") {
      throw std::runtime_error("bias_correction must be box_fit, ray_offset or none, not " +
                               l.bias_correction);
    }
    l.measurement_pixel_std = localization["measurement_pixel_std"].as<double>();
    l.measurement_floor_std_m = localization["measurement_floor_std_m"].as<double>();
    for (const auto& entry : localization["acceleration_std_mps2"]) {
      const auto name = entry.first.as<std::string>();
      if (name == "default") {
        l.acceleration_std_mps2 = entry.second.as<double>();
      } else {
        l.class_acceleration_std_mps2[name] = entry.second.as<double>();
      }
    }
    l.initial_speed_std_mps = localization["initial_speed_std_mps"].as<double>();
    l.innovation_gate = localization["innovation_gate"].as<double>();
    l.reanchor_after = localization["reanchor_after"].as<int>();
    l.heading_min_speed_mps = localization["heading_min_speed_mps"].as<double>();
    l.forget_after_s = localization["forget_after_s"].as<double>();
    if (const YAML::Node motion = localization["motion_heading"]) {
      l.heading_speed_sigma = motion["speed_sigma"].as<double>();
      l.heading_smoothing = motion["smoothing"].as<double>();
      l.heading_max_turn_rate_dps = motion["max_turn_rate_dps"].as<double>();
    }
    if (const YAML::Node keypoints = localization["keypoints"]) {
      l.keypoints_enabled = keypoints["enabled"].as<bool>();
      l.keypoint_min_score = keypoints["min_score"].as<double>();
      l.keypoint_min_size_ratio = keypoints["min_size_ratio"].as<double>();
      l.keypoint_max_size_ratio = keypoints["max_size_ratio"].as<double>();
      l.keypoint_floor_std_m = keypoints["floor_std_m"].as<double>();
      l.keypoint_smoothing = keypoints["smoothing"].as<double>();
      l.keypoint_heading_when_moving = keypoints["heading_when_moving"].as<bool>();
      l.keypoint_motion_min_speed_mps = keypoints["motion_min_speed_mps"].as<double>();
      l.keypoint_flip_frames = keypoints["flip_frames"].as<int>();
      l.keypoint_position = keypoints["position"].as<bool>();
      l.keypoint_position_weight = keypoints["position_weight"].as<double>();
    }

    const YAML::Node validation = section(root, "validation");
    config.validation.margin_m = validation["margin_m"].as<double>();
    config.validation.max_ground_m_per_px = validation["max_ground_m_per_px"].as<double>();

    if (const YAML::Node signals = root["signals"]) {
      SignalSettings& s = config.signals;
      s.min_value = signals["min_value"].as<int>();
      s.value_above_median = signals["value_above_median"].as<int>();
      s.min_saturation = signals["min_saturation"].as<int>();
      s.min_lit_fraction = signals["min_lit_fraction"].as<double>();
      s.min_lit_pixels = signals["min_lit_pixels"].as<int>();
      s.pair_ratio = signals["pair_ratio"].as<double>();
      s.confirm_frames = signals["confirm_frames"].as<int>();
      s.hold_frames = signals["hold_frames"].as<int>();
    }
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
