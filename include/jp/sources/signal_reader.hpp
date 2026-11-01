#pragma once

#include <cstdint>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <string>
#include <vector>

#include "jp/core/pipeline_config.hpp"
#include "jp/core/world_state.hpp"

namespace jp::sources {

/// Colour of the lit lamp of a signal head in one image.
struct SignalReading {
  std::string state;  ///< "red", "yellow", "red_yellow", "green" or "off".
  double confidence = 0.0;
  double red = 0.0;  ///< Lit share of the area per colour.
  double yellow = 0.0;
  double green = 0.0;
};

/// Reads the colour of a signal head inside `polygon` (image pixels) of a BGR image: pixels that
/// are bright, clearly brighter than the rest of the area and saturated are lit; their hue tells
/// red, yellow (amber) and green apart. Red and yellow lit together is red-amber.
[[nodiscard]] SignalReading read_signal(const cv::Mat& bgr, const std::vector<cv::Point2f>& polygon,
                                        const core::SignalSettings& settings);

/// Steadies the readings of one signal: a new colour is reported once it persisted for a few
/// frames, and short unlit readings (LED flicker against the camera shutter, a passing truck)
/// keep the last colour.
class SignalFilter {
 public:
  explicit SignalFilter(core::SignalSettings settings) : settings_(settings) {}

  /// The state after this frame's reading.
  [[nodiscard]] core::SignalState update(const SignalReading& reading, std::int64_t timestamp_ns);

 private:
  core::SignalSettings settings_;
  std::string state_ = "unknown";
  double confidence_ = 0.0;
  std::int64_t since_ns_ = 0;
  std::string candidate_;
  int candidate_frames_ = 0;
  int unlit_frames_ = 0;
};

}  // namespace jp::sources
