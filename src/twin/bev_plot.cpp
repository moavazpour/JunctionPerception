#include "jp/twin/bev_plot.hpp"

#include <array>
#include <cmath>
#include <format>
#include <opencv2/imgproc.hpp>
#include <utility>

#include "jp/twin/class_colors.hpp"

namespace jp::twin {

namespace {

constexpr std::size_t kTrailLength = 40;
constexpr double kGridSpacingM = 10.0;
const cv::Scalar kBackground(245, 245, 245);
const cv::Scalar kGridColor(222, 222, 222);
const cv::Scalar kReferenceColor(120, 120, 120);

void put_label(cv::Mat& canvas, const std::string& text, const cv::Point& at, double scale,
               const cv::Scalar& color) {
  cv::putText(canvas, text, at, cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(255, 255, 255), 3,
              cv::LINE_AA);
  cv::putText(canvas, text, at, cv::FONT_HERSHEY_SIMPLEX, scale, color, 1, cv::LINE_AA);
}

}  // namespace

BevPlotWriter::BevPlotWriter(const std::filesystem::path& path, double fps, BevView view,
                             core::ClassCatalog catalog, Eigen::Vector2d camera_position)
    : view_(view),
      catalog_(std::move(catalog)),
      camera_position_(std::move(camera_position)),
      writer_(path, fps, cv::Size(view.size_px, view.size_px)) {}

cv::Point BevPlotWriter::to_pixel(const Eigen::Vector2d& ground) const {
  const double scale = view_.size_px / view_.span_m;
  const double half = 0.5 * view_.size_px;
  return {static_cast<int>(std::lround(half + ((ground.x() - view_.center.x()) * scale))),
          static_cast<int>(std::lround(half - ((ground.y() - view_.center.y()) * scale)))};
}

void BevPlotWriter::draw_footprint(cv::Mat& canvas, const Eigen::Vector2d& center, double yaw_rad,
                                   double length_m, double width_m, const cv::Scalar& color,
                                   int thickness) const {
  const Eigen::Vector2d forward(std::cos(yaw_rad), std::sin(yaw_rad));
  const Eigen::Vector2d left(-forward.y(), forward.x());
  std::array<cv::Point, 4> corners;
  const std::array<std::pair<double, double>, 4> signs = {
      std::pair{1.0, 1.0}, {-1.0, 1.0}, {-1.0, -1.0}, {1.0, -1.0}};
  for (std::size_t i = 0; i < corners.size(); ++i) {
    corners.at(i) = to_pixel(center + (0.5 * signs.at(i).first * length_m * forward) +
                             (0.5 * signs.at(i).second * width_m * left));
  }
  if (thickness < 0) {
    cv::fillConvexPoly(canvas, corners.data(), static_cast<int>(corners.size()), color,
                       cv::LINE_AA);
  } else {
    cv::polylines(canvas, std::vector<cv::Point>(corners.begin(), corners.end()), true, color,
                  thickness, cv::LINE_AA);
  }
}

void BevPlotWriter::write(const std::vector<localization::LocalizedObject>& objects,
                          const std::vector<ReferenceFootprint>& references,
                          const std::string& status_line) {
  cv::Mat canvas(view_.size_px, view_.size_px, CV_8UC3, kBackground);

  const double half_span = 0.5 * view_.span_m;
  const auto first_line = [&](double center) {
    return std::floor((center - half_span) / kGridSpacingM) * kGridSpacingM;
  };
  const int lines = static_cast<int>(std::ceil(view_.span_m / kGridSpacingM)) + 1;
  for (int i = 0; i <= lines; ++i) {
    const double x = first_line(view_.center.x()) + (i * kGridSpacingM);
    const double y = first_line(view_.center.y()) + (i * kGridSpacingM);
    cv::line(canvas, to_pixel({x, view_.center.y() - half_span}),
             to_pixel({x, view_.center.y() + half_span}), kGridColor, 1);
    cv::line(canvas, to_pixel({view_.center.x() - half_span, y}),
             to_pixel({view_.center.x() + half_span, y}), kGridColor, 1);
  }
  cv::drawMarker(canvas, to_pixel(camera_position_), cv::Scalar(0, 0, 0), cv::MARKER_TRIANGLE_UP,
                 14, 2);

  for (const auto& reference : references) {
    draw_footprint(canvas, reference.center, reference.yaw_rad, reference.length_m,
                   reference.width_m, kReferenceColor, 1);
  }

  for (const auto& object : objects) {
    auto& trail = trails_[object.id];
    trail.push_back(object.position);
    if (trail.size() > kTrailLength) {
      trail.pop_front();
    }
    const cv::Scalar color = class_color(object.twin_class);
    for (std::size_t i = 1; i < trail.size(); ++i) {
      cv::line(canvas, to_pixel(trail[i - 1]), to_pixel(trail[i]), color, 1, cv::LINE_AA);
    }
    const auto& info = catalog_.info(object.twin_class);
    draw_footprint(canvas, object.position, object.heading_rad.value_or(0.0), info.length_m,
                   info.width_m, color, -1);
    cv::circle(canvas, to_pixel(object.contact), 2, cv::Scalar(0, 0, 0), -1, cv::LINE_AA);
    put_label(canvas, std::format("{} {}", object.id, object.twin_class),
              to_pixel(object.position) + cv::Point(6, -6), 0.4, cv::Scalar(40, 40, 40));
  }

  put_label(canvas, status_line, cv::Point(10, 24), 0.6, cv::Scalar(0, 0, 0));
  put_label(canvas,
            std::format("grid {:.0f} m; filled: tracks, black dot: image contact point; "
                        "grey outline: reference",
                        kGridSpacingM),
            cv::Point(10, view_.size_px - 12), 0.45, cv::Scalar(60, 60, 60));
  writer_.write(canvas);
}

cv::Mat draw_tracks(const cv::Mat& image, const std::vector<localization::LocalizedObject>& objects,
                    const std::string& status_line) {
  cv::Mat canvas = image.clone();
  for (const auto& object : objects) {
    std::string label = std::format("#{} {}{} {:.0f} km/h", object.id, object.twin_class,
                                    object.rider ? " +rider" : "", object.speed_mps * 3.6);
    if (object.length_m && object.width_m) {
      // Size measured from the box corners (keypoint models).
      label += std::format(" {:.1f}x{:.1f} m", *object.length_m, *object.width_m);
    }
    draw_labeled_box(canvas, object.box, label, class_color(object.twin_class));
    draw_footprint(canvas, object.corners, class_color(object.twin_class));
  }
  draw_status_line(canvas, status_line);
  return canvas;
}

}  // namespace jp::twin
