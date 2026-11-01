#pragma once

#include <cstddef>
#include <opencv2/core/types.hpp>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "jp/detection/detection.hpp"

namespace jp::detection {

/// One row of the end-to-end YOLO26 output in network input pixels.
struct RawDetection {
  cv::Rect2f box;
  float score = 0.0F;
  int class_id = -1;
  std::vector<Keypoint> keypoints;  ///< Pose models only, in network input pixels.
};

/// Values per row of the end-to-end output of a detection model: x1, y1, x2, y2, score, class.
/// Pose models append x, y, visibility per keypoint.
inline constexpr std::size_t kEnd2EndRowSize = 6;
inline constexpr std::size_t kKeypointValues = 3;

/// Parses the NMS-free YOLO26 output of shape (1, N, row_size) and keeps rows with
/// score >= min_score. Rows are already final detections (no NMS is applied); rows longer than
/// six values carry (row_size - 6) / 3 keypoints. Throws std::invalid_argument if the buffer is
/// not a whole number of rows or the row size is not 6 + 3k.
[[nodiscard]] std::vector<RawDetection> parse_end2end_output(
    std::span<const float> output, float min_score, std::size_t row_size = kEnd2EndRowSize);

/// Parses the Ultralytics "names" metadata, e.g. "{0: 'person', 1: 'bicycle'}", into a list
/// indexed by class id. Throws std::invalid_argument on malformed text or non-contiguous ids.
[[nodiscard]] std::vector<std::string> parse_class_names(std::string_view metadata);

}  // namespace jp::detection
