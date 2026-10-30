#include "jp/twin/debug_overlay.hpp"

#include <algorithm>
#include <format>
#include <map>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <stdexcept>

namespace jp::twin {

namespace {

constexpr double kFontScale = 0.55;
constexpr int kLabelHeight = 18;

cv::Scalar color_of(const std::string& twin_class) {
  static const std::map<std::string, cv::Scalar, std::less<>> colors = {
      {"car", {80, 200, 80}},
      {"van", {80, 220, 200}},
      {"truck", {60, 140, 255}},
      {"trailer", {40, 90, 200}},
      {"bus", {200, 120, 40}},
      {"pedestrian", {60, 60, 240}},
      {"bicycle", {240, 80, 220}},
      {"motorcycle", {180, 60, 180}},
      {"emergency_vehicle", {0, 0, 160}},
      {"other", {160, 160, 160}}};
  const auto it = colors.find(twin_class);
  return it == colors.end() ? cv::Scalar(255, 255, 255) : it->second;
}

}  // namespace

cv::Mat draw_detections(const cv::Mat& image, const std::vector<detection::Detection>& detections,
                        const std::string& status_line) {
  cv::Mat canvas = image.clone();
  for (const auto& d : detections) {
    const cv::Scalar color = color_of(d.twin_class);
    const cv::Rect box(d.box);
    cv::rectangle(canvas, box, color, 2, cv::LINE_AA);
    const std::string label =
        std::format("{}{} {:.2f}", d.twin_class.empty() ? d.source_label : d.twin_class,
                    d.rider ? " +rider" : "", d.score);
    const cv::Point anchor(box.x, std::max(box.y - 4, kLabelHeight));
    cv::putText(canvas, label, anchor, cv::FONT_HERSHEY_SIMPLEX, kFontScale, cv::Scalar(0, 0, 0), 3,
                cv::LINE_AA);
    cv::putText(canvas, label, anchor, cv::FONT_HERSHEY_SIMPLEX, kFontScale, color, 1, cv::LINE_AA);
  }
  const cv::Point origin(12, 28);
  cv::putText(canvas, status_line, origin, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 4,
              cv::LINE_AA);
  cv::putText(canvas, status_line, origin, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255),
              2, cv::LINE_AA);
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
