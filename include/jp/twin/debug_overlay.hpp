#pragma once

#include <filesystem>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <string>
#include <vector>

#include "jp/detection/detection.hpp"

namespace jp::twin {

/// Draws a box with a text label above it.
void draw_labeled_box(cv::Mat& canvas, const cv::Rect2f& box, const std::string& label,
                      const cv::Scalar& color);

/// Draws a status text line at the top left.
void draw_status_line(cv::Mat& canvas, const std::string& status_line);

/// Draws detections (box, twin class, score) and a text line at the top onto a copy of the image.
[[nodiscard]] cv::Mat draw_detections(const cv::Mat& image,
                                      const std::vector<detection::Detection>& detections,
                                      const std::string& status_line);

/// Writes annotated frames to an MP4 video (debug overlay sink).
class OverlayVideoWriter {
 public:
  /// Throws std::runtime_error if the file cannot be created.
  OverlayVideoWriter(const std::filesystem::path& path, double fps, const cv::Size& frame_size);
  ~OverlayVideoWriter();
  OverlayVideoWriter(const OverlayVideoWriter&) = delete;
  OverlayVideoWriter& operator=(const OverlayVideoWriter&) = delete;
  OverlayVideoWriter(OverlayVideoWriter&&) = delete;
  OverlayVideoWriter& operator=(OverlayVideoWriter&&) = delete;

  void write(const cv::Mat& frame);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace jp::twin
