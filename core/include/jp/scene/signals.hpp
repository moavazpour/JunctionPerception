#pragma once

#include <array>
#include <deque>
#include <map>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace jp {

// Traffic light head marked by the installer: a box around it in the camera image (frame pixels) and the stop
// line it controls (index into the drawn stop lines, -1 = none).
struct SignalHead {
  std::string id;
  std::array<double, 4> box{};  // u0, v0, u1, v1
  int stop_line = -1;
};

struct SignalState {
  std::string id;
  std::string state = "unknown";  // red | red_amber | amber | green | off | unknown
  double confidence = 0;
};

// Reads the lit colour of each head from the image: bright, saturated pixels by hue (red / amber / green),
// with the lamp position (red on top, green at the bottom) as a plausibility check, then a majority vote
// over the last half second so a single bad frame does not flip the state.
class SignalReader {
 public:
  std::vector<SignalState> update(const cv::Mat& frame_bgr, const std::vector<SignalHead>& heads);
  static SignalState classify(const cv::Mat& crop_bgr);

 private:
  std::map<std::string, std::deque<std::string>> history_;
};

}  // namespace jp
