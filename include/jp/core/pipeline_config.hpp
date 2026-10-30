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

/// ByteTrack thresholds (see configs/pipeline.yaml).
struct TrackingSettings {
  float high_score = 0.5F;
  float low_score = 0.1F;
  float new_track_score = 0.6F;
  float high_match_min_iou = 0.2F;
  float low_match_min_iou = 0.5F;
  float tentative_match_min_iou = 0.3F;
  int confirm_frames = 3;
  int max_missed_frames = 15;
  float min_rider_votes = 1.0F;
};

/// Ground-plane localization (see configs/pipeline.yaml).
struct LocalizationSettings {
  bool bias_correction = true;
  double acceleration_std_mps2 = 2.0;
  double measurement_std_m = 0.3;
  double measurement_std_per_m = 0.01;
  double heading_min_speed_mps = 0.5;
  double forget_after_s = 5.0;
};

/// Settings of configs/pipeline.yaml.
struct PipelineConfig {
  DetectionSettings detection;
  RiderSettings riders;
  TrackingSettings tracking;
  LocalizationSettings localization;
};

/// Parses the YAML text; relative paths are resolved against `base_dir`.
/// Throws std::runtime_error on missing keys or invalid values.
[[nodiscard]] PipelineConfig parse_pipeline_config(std::string_view yaml_text,
                                                   const std::filesystem::path& base_dir);
[[nodiscard]] PipelineConfig load_pipeline_config(const std::filesystem::path& path,
                                                  const std::filesystem::path& base_dir);

}  // namespace jp::core
