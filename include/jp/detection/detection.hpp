#pragma once

#include <opencv2/core/types.hpp>
#include <string>

namespace jp::detection {

/// One 2D detection in image pixels.
struct Detection {
  cv::Rect2f box;            ///< Axis-aligned box in image pixels.
  float score = 0.0F;        ///< Detector confidence in [0, 1].
  int source_class = -1;     ///< Class index of the detector (e.g. COCO).
  std::string source_label;  ///< Class name of the detector, e.g. "person".
  std::string twin_class;    ///< Twin class after mapping, e.g. "pedestrian"; empty if unmapped.
};

}  // namespace jp::detection
