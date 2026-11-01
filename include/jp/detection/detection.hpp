#pragma once

#include <opencv2/core/types.hpp>
#include <string>
#include <vector>

namespace jp::detection {

/// A keypoint of a detection in image pixels with the detector's visibility score.
struct Keypoint {
  cv::Point2f pixel;
  float score = 0.0F;
};

/// One 2D detection in image pixels.
struct Detection {
  cv::Rect2f box;            ///< Axis-aligned box in image pixels.
  float score = 0.0F;        ///< Detector confidence in [0, 1].
  int source_class = -1;     ///< Class index of the detector (e.g. COCO).
  std::string source_label;  ///< Class name of the detector, e.g. "person".
  std::string twin_class;    ///< Twin class after mapping, e.g. "pedestrian"; empty if unmapped.
  bool rider = false;        ///< Two-wheeler merged with the person riding it.
  /// Keypoint models: the four bottom corners of the object's 3D box (its footprint on the
  /// ground); empty for box-only detectors.
  std::vector<Keypoint> keypoints;
};

}  // namespace jp::detection
