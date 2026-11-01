#include "jp/twin/debug_overlay.hpp"

#include <algorithm>
#include <cmath>
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

void draw_footprint(cv::Mat& canvas, const std::vector<detection::Keypoint>& corners,
                    const cv::Scalar& color, float min_score) {
  constexpr std::size_t kCorners = 4;
  constexpr double kFillOpacity = 0.25;
  if (corners.size() != kCorners ||
      std::ranges::any_of(corners, [&](const auto& k) { return k.score < min_score; })) {
    return;
  }
  // Ordered around their centre, whatever order the model gives them in.
  cv::Point2f centre(0.0F, 0.0F);
  for (const auto& k : corners) {
    centre += k.pixel * 0.25F;
  }
  std::vector<cv::Point2f> ordered;
  ordered.reserve(kCorners);
  for (const auto& k : corners) {
    ordered.push_back(k.pixel);
  }
  std::ranges::sort(ordered, {}, [&](const cv::Point2f& p) {
    return std::atan2(p.y - centre.y, p.x - centre.x);
  });
  std::vector<cv::Point> quad;
  quad.reserve(kCorners);
  for (const auto& p : ordered) {
    quad.emplace_back(cvRound(p.x), cvRound(p.y));
  }
  const cv::Rect area = cv::boundingRect(quad) & cv::Rect(0, 0, canvas.cols, canvas.rows);
  if (area.area() > 0) {
    cv::Mat patch = canvas(area);
    cv::Mat tinted = patch.clone();
    std::vector<cv::Point> local = quad;
    for (auto& p : local) {
      p -= area.tl();
    }
    cv::fillPoly(tinted, std::vector{local}, color, cv::LINE_AA);
    cv::addWeighted(tinted, kFillOpacity, patch, 1.0 - kFillOpacity, 0.0, patch);
  }
  cv::polylines(canvas, std::vector{quad}, true, color, 2, cv::LINE_AA);
  for (const auto& p : quad) {
    cv::circle(canvas, p, 3, color, cv::FILLED, cv::LINE_AA);
  }
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
    draw_footprint(canvas, d.keypoints, class_color(d.twin_class));
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
