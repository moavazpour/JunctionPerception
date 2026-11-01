#include "jp/detection/letterbox.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>

namespace jp::detection {

namespace {

constexpr double kPadValue = 114.0;
// Ultralytics splits odd padding with round(pad / 2 -+ 0.1); this reproduces its left/top side.
constexpr double kPadBias = 0.1;

}  // namespace

Letterbox Letterbox::fit(const cv::Size& image_size, const cv::Size& input_size) {
  Letterbox lb;
  lb.input_width = input_size.width;
  lb.input_height = input_size.height;
  lb.scale =
      std::min(static_cast<float>(input_size.width) / static_cast<float>(image_size.width),
               static_cast<float>(input_size.height) / static_cast<float>(image_size.height));
  lb.resized_width = static_cast<int>(std::lround(static_cast<float>(image_size.width) * lb.scale));
  lb.resized_height =
      static_cast<int>(std::lround(static_cast<float>(image_size.height) * lb.scale));
  lb.pad_left = static_cast<int>(
      std::lround((static_cast<double>(input_size.width - lb.resized_width) / 2.0) - kPadBias));
  lb.pad_top = static_cast<int>(
      std::lround((static_cast<double>(input_size.height - lb.resized_height) / 2.0) - kPadBias));
  return lb;
}

cv::Rect2f Letterbox::to_image(const cv::Rect2f& input_box, const cv::Size& image_size) const {
  const auto unmap_x = [&](float x) {
    return std::clamp((x - static_cast<float>(pad_left)) / scale, 0.0F,
                      static_cast<float>(image_size.width));
  };
  const auto unmap_y = [&](float y) {
    return std::clamp((y - static_cast<float>(pad_top)) / scale, 0.0F,
                      static_cast<float>(image_size.height));
  };
  const float x1 = unmap_x(input_box.x);
  const float y1 = unmap_y(input_box.y);
  const float x2 = unmap_x(input_box.x + input_box.width);
  const float y2 = unmap_y(input_box.y + input_box.height);
  return {x1, y1, x2 - x1, y2 - y1};
}

cv::Point2f Letterbox::to_image(const cv::Point2f& input_point) const {
  return {(input_point.x - static_cast<float>(pad_left)) / scale,
          (input_point.y - static_cast<float>(pad_top)) / scale};
}

cv::Mat Letterbox::apply(const cv::Mat& image) const {
  cv::Mat canvas(input_height, input_width, image.type(), cv::Scalar::all(kPadValue));
  cv::Mat resized;
  cv::resize(image, resized, cv::Size(resized_width, resized_height), 0.0, 0.0, cv::INTER_LINEAR);
  resized.copyTo(canvas(cv::Rect(pad_left, pad_top, resized_width, resized_height)));
  return canvas;
}

}  // namespace jp::detection
