#pragma once

#include <filesystem>
#include <string>
#include <string_view>

namespace jp::core {

struct DetectionSettings {
  std::filesystem::path model;
  float confidence_threshold = 0.25F;
  bool use_cuda = true;
  int cuda_device = 0;
  std::string class_source = "coco";
};

/// Merging of persons with the bicycle or motorcycle they ride.
struct RiderSettings {
  bool enabled = true;
  float min_person_overlap = 0.2F;
  float min_feet_depth = 0.25F;
};

/// Settings of configs/pipeline.yaml.
struct PipelineConfig {
  DetectionSettings detection;
  RiderSettings riders;
};

/// Parses the YAML text; relative paths are resolved against `base_dir`.
/// Throws std::runtime_error on missing keys or invalid values.
[[nodiscard]] PipelineConfig parse_pipeline_config(std::string_view yaml_text,
                                                   const std::filesystem::path& base_dir);
[[nodiscard]] PipelineConfig load_pipeline_config(const std::filesystem::path& path,
                                                  const std::filesystem::path& base_dir);

}  // namespace jp::core
