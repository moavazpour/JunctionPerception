#include <gtest/gtest.h>

#include <stdexcept>

#include "jp/core/pipeline_config.hpp"
#include "jp/core/statistics.hpp"

namespace {

TEST(PipelineConfig, ParsesDetectionSection) {
  const auto config = jp::core::parse_pipeline_config(R"yaml(
detection:
  model: models/m.onnx
  confidence_threshold: 0.4
  execution_provider: cpu
  cuda_device: 1
  class_source: coco
  rider_association: {enabled: true, min_person_overlap: 0.3, min_feet_depth: 0.25}
tracking: {high_score: 0.5, low_score: 0.1, new_track_score: 0.6, high_match_min_iou: 0.2,
           low_match_min_iou: 0.5, tentative_match_min_iou: 0.3, confirm_frames: 3,
           max_missed_frames: 15, min_rider_votes: 1.0}
localization: {bias_correction: none, measurement_pixel_std: 2.0, measurement_floor_std_m: 0.2,
               acceleration_std_mps2: {default: 3.0, pedestrian: 1.0}, initial_speed_std_mps: 10.0,
               innovation_gate: 11.8, reanchor_after: 3, heading_min_speed_mps: 0.5,
               forget_after_s: 5.0}
events: {crosswalk_min_frames: 3, conflict_horizon_s: 3.0, conflict_step_s: 0.1,
         conflict_gap_m: 1.0, conflict_min_closing_speed_mps: 0.3, conflict_max_speed_std_mps: 0.5,
         stopped_speed_mps: 0.5, starting_window_s: 2.0, validated_margin_m: 10.0,
         validated_max_ground_m_per_px: 0.2, conflict_min_frames: 2, end_frames: 3,
         turning_window_s: 2.0, turning_threshold_deg: 20.0}
)yaml",
                                                      "/repo");
  EXPECT_EQ(config.detection.model, std::filesystem::path("/repo/models/m.onnx"));
  EXPECT_FLOAT_EQ(config.detection.confidence_threshold, 0.4F);
  EXPECT_FALSE(config.detection.use_cuda);
  EXPECT_EQ(config.detection.cuda_device, 1);
  EXPECT_TRUE(config.riders.enabled);
  EXPECT_FLOAT_EQ(config.riders.min_person_overlap, 0.3F);
  EXPECT_EQ(config.tracking.confirm_frames, 3);
  EXPECT_EQ(config.localization.bias_correction, "none");
  EXPECT_DOUBLE_EQ(config.localization.acceleration_std_mps2, 3.0);
  EXPECT_DOUBLE_EQ(config.localization.class_acceleration_std_mps2.at("pedestrian"), 1.0);
}

TEST(PipelineConfig, RejectsUnknownProvider) {
  EXPECT_THROW(static_cast<void>(jp::core::parse_pipeline_config(R"yaml(
detection:
  model: m.onnx
  confidence_threshold: 0.4
  execution_provider: tpu
  cuda_device: 0
  class_source: coco
  rider_association: {enabled: true, min_person_overlap: 0.3, min_feet_depth: 0.25}
tracking: {high_score: 0.5, low_score: 0.1, new_track_score: 0.6, high_match_min_iou: 0.2,
           low_match_min_iou: 0.5, tentative_match_min_iou: 0.3, confirm_frames: 3,
           max_missed_frames: 15, min_rider_votes: 1.0}
localization: {bias_correction: none, measurement_pixel_std: 2.0, measurement_floor_std_m: 0.2,
               acceleration_std_mps2: {default: 3.0, pedestrian: 1.0}, initial_speed_std_mps: 10.0,
               innovation_gate: 11.8, reanchor_after: 3, heading_min_speed_mps: 0.5,
               forget_after_s: 5.0}
events: {crosswalk_min_frames: 3, conflict_horizon_s: 3.0, conflict_step_s: 0.1,
         conflict_gap_m: 1.0, conflict_min_closing_speed_mps: 0.3, conflict_max_speed_std_mps: 0.5,
         stopped_speed_mps: 0.5, starting_window_s: 2.0, validated_margin_m: 10.0,
         validated_max_ground_m_per_px: 0.2, conflict_min_frames: 2, end_frames: 3,
         turning_window_s: 2.0, turning_threshold_deg: 20.0}
)yaml",
                                                                 "/repo")),
               std::runtime_error);
}

TEST(PipelineConfig, RepositoryConfigIsValid) {
  const auto config =
      jp::core::load_pipeline_config(JP_SOURCE_DIR "/configs/pipeline.yaml", JP_SOURCE_DIR);
  EXPECT_EQ(config.detection.class_source, "coco");
}

TEST(Statistics, Summarizes) {
  const auto s = jp::core::summarize({4.0, 1.0, 3.0, 2.0, 10.0});
  EXPECT_EQ(s.count, 5U);
  EXPECT_DOUBLE_EQ(s.mean, 4.0);
  EXPECT_DOUBLE_EQ(s.median, 3.0);
  EXPECT_DOUBLE_EQ(s.max, 10.0);
  EXPECT_EQ(jp::core::summarize({}).count, 0U);
}

}  // namespace
