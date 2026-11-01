#pragma once

#include <Eigen/Core>
#include <filesystem>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <optional>
#include <string>
#include <vector>

#include "jp/detection/detection.hpp"

namespace jp::sources {

/// A detection area of a camera: a polygon in image pixels.
struct RegionArea {
  std::string id;
  std::string name;
  std::vector<cv::Point2f> points;
};

/// A line of a camera, drawn across (or along) the road in image pixels. Road users crossing it
/// are counted (site manager statistics). With a band, only road users within that ground
/// distance of a banded line are accepted at all.
struct RegionLine {
  std::string id;
  std::string name;
  std::vector<cv::Point2f> points;
  double band_m = 0.0;           ///< Half width of the accepting band; 0: the line only counts.
  double speed_limit_kmh = 0.0;  ///< For the statistics; 0: not set.
};

/// Where a camera's detections are accepted: inside one of its areas (if it has any) and within
/// the band of one of its banded lines (if it has any). Detections elsewhere are dropped before
/// tracking, so they are neither tracked nor shown. Stored per camera in regions.json.
struct DetectionRegions {
  std::vector<RegionArea> areas;
  std::vector<RegionLine> lines;

  [[nodiscard]] bool empty() const { return areas.empty() && lines.empty(); }

  /// Image point where a detection touches the ground: the centre of its bottom corners
  /// (keypoint models), else the bottom centre of its box.
  [[nodiscard]] static cv::Point2f contact(const detection::Detection& detection);

  /// Whether a detection lies inside the regions. Line bands need the image-to-ground
  /// homography; without a calibration they accept everything.
  [[nodiscard]] bool accepts(const detection::Detection& detection,
                             const std::optional<Eigen::Matrix3d>& image_to_ground) const;

  /// JSON text of regions.json; parse() throws std::runtime_error on malformed content.
  [[nodiscard]] std::string to_json() const;
  [[nodiscard]] static DetectionRegions parse(const std::string& text);
};

/// regions.json of a camera folder; empty regions (accept everything) if the file is missing.
[[nodiscard]] DetectionRegions load_regions(const std::filesystem::path& path);

/// Draws the areas and lines onto a camera image.
void draw_regions(cv::Mat& canvas, const DetectionRegions& regions);

}  // namespace jp::sources
