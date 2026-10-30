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
)yaml",
                                                      "/repo");
  EXPECT_EQ(config.detection.model, std::filesystem::path("/repo/models/m.onnx"));
  EXPECT_FLOAT_EQ(config.detection.confidence_threshold, 0.4F);
  EXPECT_FALSE(config.detection.use_cuda);
  EXPECT_EQ(config.detection.cuda_device, 1);
  EXPECT_TRUE(config.riders.enabled);
  EXPECT_FLOAT_EQ(config.riders.min_person_overlap, 0.3F);
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
