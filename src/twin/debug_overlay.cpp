#include "jp/twin/debug_overlay.hpp"

#include <algorithm>
#include <format>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <stdexcept>

#include "jp/twin/class_colors.hpp"

namespace jp::twin {

namespace {

constexpr double kFontScale = 0.55;
constexpr int kLabelHeight = 18;

}  // namespace

void draw_labeled_box(cv::Mat& canvas, const cv::Rect2f& box, const std::string& label,
                      const cv::Scalar& color) {
  const cv::Rect pixels(box);
  cv::rectangle(canvas, pixels, color, 2, cv::LINE_AA);
  const cv::Point anchor(pixels.x, std::max(pixels.y - 4, kLabelHeight));
  cv::putText(canvas, label, anchor, cv::FONT_HERSHEY_SIMPLEX, kFontScale, cv::Scalar(0, 0, 0), 3,
              cv::LINE_AA);
  cv::putText(canvas, label, anchor, cv::FONT_HERSHEY_SIMPLEX, kFontScale, color, 1, cv::LINE_AA);
}

void draw_status_line(cv::Mat& canvas, const std::string& status_line) {
  const cv::Point origin(12, 28);
  cv::putText(canvas, status_line, origin, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 4,
              cv::LINE_AA);
  cv::putText(canvas, status_line, origin, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255),
              2, cv::LINE_AA);
}

cv::Mat draw_detections(const cv::Mat& image, const std::vector<detection::Detection>& detections,
                        const std::string& status_line) {
  cv::Mat canvas = image.clone();
  for (const auto& d : detections) {
    const std::string label =
        std::format("{}{} {:.2f}", d.twin_class.empty() ? d.source_label : d.twin_class,
                    d.rider ? " +rider" : "", d.score);
    draw_labeled_box(canvas, d.box, label, class_color(d.twin_class));
  }
  draw_status_line(canvas, status_line);
  return canvas;
}

struct OverlayVideoWriter::Impl {
  cv::VideoWriter writer;
};

OverlayVideoWriter::OverlayVideoWriter(const std::filesystem::path& path, double fps,
                                       const cv::Size& frame_size)
    : impl_(std::make_unique<Impl>()) {
  if (path.has_parent_path()) {
    std::filesystem::create_directories(path.parent_path());
  }
  const int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
  if (!impl_->writer.open(path.string(), cv::CAP_FFMPEG, fourcc, fps, frame_size)) {
    throw std::runtime_error("cannot create video " + path.string());
  }
}

OverlayVideoWriter::~OverlayVideoWriter() = default;

void OverlayVideoWriter::write(const cv::Mat& frame) { impl_->writer.write(frame); }

}  // namespace jp::twin
