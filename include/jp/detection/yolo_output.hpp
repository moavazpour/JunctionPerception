#pragma once

#include <cstddef>
#include <opencv2/core/types.hpp>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace jp::detection {

/// One row of the end-to-end YOLO26 output in network input pixels.
struct RawDetection {
  cv::Rect2f box;
  float score = 0.0F;
  int class_id = -1;
};

/// Number of values per row of the end-to-end output: x1, y1, x2, y2, score, class.
inline constexpr std::size_t kEnd2EndRowSize = 6;

/// Parses the NMS-free YOLO26 output of shape (1, N, 6) and keeps rows with score >= min_score.
/// Rows are already final detections (no NMS is applied). Throws std::invalid_argument if the
/// buffer size is not a multiple of 6.
[[nodiscard]] std::vector<RawDetection> parse_end2end_output(std::span<const float> output,
                                                             float min_score);

/// Parses the Ultralytics "names" metadata, e.g. "{0: 'person', 1: 'bicycle'}", into a list
/// indexed by class id. Throws std::invalid_argument on malformed text or non-contiguous ids.
[[nodiscard]] std::vector<std::string> parse_class_names(std::string_view metadata);

}  // namespace jp::detection
