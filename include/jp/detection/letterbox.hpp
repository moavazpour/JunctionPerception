#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

namespace jp::detection {

/// Aspect-preserving resize of an image into a square network input with symmetric padding,
/// matching the Ultralytics letterbox (grey padding, centred image).
struct Letterbox {
  int input_size = 0;  ///< Square network input (px).
  float scale = 1.0F;  ///< Image px -> input px.
  int resized_width = 0;
  int resized_height = 0;
  int pad_left = 0;
  int pad_top = 0;

  [[nodiscard]] static Letterbox fit(const cv::Size& image_size, int input_size);

  /// Maps a box from network input pixels back to image pixels, clipped to the image.
  [[nodiscard]] cv::Rect2f to_image(const cv::Rect2f& input_box, const cv::Size& image_size) const;

  /// Resized and padded BGR image of input_size x input_size.
  [[nodiscard]] cv::Mat apply(const cv::Mat& image) const;
};

}  // namespace jp::detection
