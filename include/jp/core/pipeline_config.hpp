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

/// Settings of configs/pipeline.yaml.
struct PipelineConfig {
  DetectionSettings detection;
};

/// Parses the YAML text; relative paths are resolved against `base_dir`.
/// Throws std::runtime_error on missing keys or invalid values.
[[nodiscard]] PipelineConfig parse_pipeline_config(std::string_view yaml_text,
                                                   const std::filesystem::path& base_dir);
[[nodiscard]] PipelineConfig load_pipeline_config(const std::filesystem::path& path,
                                                  const std::filesystem::path& base_dir);

}  // namespace jp::core
