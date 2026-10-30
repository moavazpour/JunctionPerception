#pragma once

#include <Eigen/Core>
#include <deque>
#include <filesystem>
#include <map>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <string>
#include <vector>

#include "jp/core/class_catalog.hpp"
#include "jp/localization/track_localizer.hpp"
#include "jp/twin/debug_overlay.hpp"

namespace jp::twin {

/// Area of the ground plane shown in a top-down plot.
struct BevView {
  Eigen::Vector2d center = Eigen::Vector2d::Zero();  ///< Ground point at the image centre (m).
  double span_m = 120.0;                             ///< Width and height shown (m).
  int size_px = 1000;
};

/// Reference footprint drawn in outline (e.g. benchmark ground truth), ground plane coordinates.
struct ReferenceFootprint {
  Eigen::Vector2d center = Eigen::Vector2d::Zero();
  double yaw_rad = 0.0;
  double length_m = 0.0;
  double width_m = 0.0;
};

/// Debug sink: top-down video of localized objects (class-coloured footprints oriented by
/// heading, track IDs, trails), the camera position, a 10 m grid and optional reference
/// footprints.
class BevPlotWriter {
 public:
  BevPlotWriter(const std::filesystem::path& path, double fps, BevView view,
                core::ClassCatalog catalog, Eigen::Vector2d camera_position);

  void write(const std::vector<localization::LocalizedObject>& objects,
             const std::vector<ReferenceFootprint>& references, const std::string& status_line);

 private:
  [[nodiscard]] cv::Point to_pixel(const Eigen::Vector2d& ground) const;
  void draw_footprint(cv::Mat& canvas, const Eigen::Vector2d& center, double yaw_rad,
                      double length_m, double width_m, const cv::Scalar& color,
                      int thickness) const;

  BevView view_;
  core::ClassCatalog catalog_;
  Eigen::Vector2d camera_position_;
  OverlayVideoWriter writer_;
  std::map<std::int64_t, std::deque<Eigen::Vector2d>> trails_;
};

/// Draws tracked boxes with track ID and class onto a copy of the image.
[[nodiscard]] cv::Mat draw_tracks(const cv::Mat& image,
                                  const std::vector<localization::LocalizedObject>& objects,
                                  const std::string& status_line);

}  // namespace jp::twin
