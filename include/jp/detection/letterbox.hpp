#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

namespace jp::detection {

/// Aspect-preserving resize of an image into the network input with symmetric padding,
/// matching the Ultralytics letterbox (grey padding, centred image). The input may be square
/// (e.g. 1280 x 1280) or rectangular (e.g. 640 x 384).
struct Letterbox {
  int input_width = 0;  ///< Network input (px).
  int input_height = 0;
  float scale = 1.0F;  ///< Image px -> input px.
  int resized_width = 0;
  int resized_height = 0;
  int pad_left = 0;
  int pad_top = 0;

  [[nodiscard]] static Letterbox fit(const cv::Size& image_size, const cv::Size& input_size);
  [[nodiscard]] static Letterbox fit(const cv::Size& image_size, int input_size) {
    return fit(image_size, cv::Size(input_size, input_size));
  }

  /// Maps a box from network input pixels back to image pixels, clipped to the image.
  [[nodiscard]] cv::Rect2f to_image(const cv::Rect2f& input_box, const cv::Size& image_size) const;

  /// Maps a point from network input pixels back to image pixels (not clipped).
  [[nodiscard]] cv::Point2f to_image(const cv::Point2f& input_point) const;

  /// Resized and padded BGR image of input_width x input_height.
  [[nodiscard]] cv::Mat apply(const cv::Mat& image) const;
};

}  // namespace jp::detection
