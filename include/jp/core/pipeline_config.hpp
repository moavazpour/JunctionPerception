#pragma once

#include <filesystem>
#include <functional>
#include <map>
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
  std::string bias_correction = "box_fit";  ///< box_fit, ray_offset or none
  double measurement_pixel_std = 2.0;
  double measurement_floor_std_m = 0.5;
  double acceleration_std_mps2 = 2.0;  ///< Default; per-class values below.
  std::map<std::string, double, std::less<>> class_acceleration_std_mps2;
  double initial_speed_std_mps = 10.0;
  double innovation_gate = 18.4;
  int reanchor_after = 3;
  double heading_min_speed_mps = 0.5;
  /// Heading from motion (see configs/pipeline.yaml, localization.motion_heading).
  double heading_speed_sigma = 2.0;
  double heading_smoothing = 0.6;
  double heading_max_turn_rate_dps = 90.0;
  double heading_min_travel_m = 3.0;
  /// Measurements whose box bottom a nearer road user covers (localization.occlusion).
  double occlusion_noise_scale = 5.0;
  double occlusion_min_overlap = 0.2;
  double forget_after_s = 5.0;
  /// Keypoint models: footprint from the four bottom corners (see configs/pipeline.yaml).
  bool keypoints_enabled = true;
  double keypoint_min_score = 0.5;
  double keypoint_min_size_ratio = 0.4;
  double keypoint_max_size_ratio = 2.5;
  double keypoint_floor_std_m = 0.6;
  double keypoint_smoothing = 0.3;
  bool keypoint_heading_when_moving = true;
  double keypoint_motion_min_speed_mps = 2.0;
  int keypoint_flip_frames = 5;
  bool keypoint_position = false;
  double keypoint_position_weight = 1.0;
};

/// Region where camera positions are trusted (see configs/pipeline.yaml, validation).
struct ValidationSettings {
  double margin_m = 10.0;            ///< Around the outline of the calibration control points.
  double max_ground_m_per_px = 0.2;  ///< Coarsest ground sampling along the camera ray.
};

/// Settings of configs/pipeline.yaml.
/// Traffic-light colour from the signal areas of a camera (see configs/pipeline.yaml).
struct SignalSettings {
  int min_value = 150;             ///< A lit lamp pixel is at least this bright (HSV value) ...
  int value_above_median = 50;     ///< ... and this much brighter than the area median ...
  int min_saturation = 70;         ///< ... and this saturated.
  double min_lit_fraction = 0.01;  ///< Share of the area that must be lit for a colour.
  int min_lit_pixels = 6;
  double pair_ratio = 0.4;  ///< Red and yellow both lit (red-amber) above this ratio.
  int confirm_frames = 2;   ///< Frames a new colour must persist before it is reported.
  int hold_frames = 10;     ///< Frames an unlit reading keeps the last colour (LED flicker).
};

struct PipelineConfig {
  DetectionSettings detection;
  RiderSettings riders;
  TrackingSettings tracking;
  LocalizationSettings localization;
  ValidationSettings validation;
  SignalSettings signals;
};

/// Parses the YAML text; relative paths are resolved against `base_dir`.
/// Throws std::runtime_error on missing keys or invalid values.
[[nodiscard]] PipelineConfig parse_pipeline_config(std::string_view yaml_text,
                                                   const std::filesystem::path& base_dir);
[[nodiscard]] PipelineConfig load_pipeline_config(const std::filesystem::path& path,
                                                  const std::filesystem::path& base_dir);

}  // namespace jp::core
