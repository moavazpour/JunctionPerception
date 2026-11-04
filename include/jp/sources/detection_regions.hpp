#pragma once

#include <Eigen/Core>
#include <filesystem>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <optional>
#include <string>
#include <vector>

#include "jp/core/world_state.hpp"
#include "jp/detection/detection.hpp"

namespace jp::sources {

/// A detection area of a camera: a polygon in image pixels.
struct RegionArea {
  std::string id;
  std::string name;
  std::vector<cv::Point2f> points;
};

/// A line of a camera in image pixels. With three or more points it outlines a lane, whose queue
/// and passing road users the site manager measures (manager/traffic.py) along the direction the
/// user set. With a band, only road users within that ground distance of a banded line are
/// accepted at all.
struct RegionLine {
  std::string id;
  std::string name;
  std::vector<cv::Point2f> points;
  double band_m = 0.0;  ///< Half width of the accepting band; 0: no band.
  /// Direction of travel set by the user: two image points, from tail to head; empty if unset.
  std::vector<cv::Point2f> direction;
};

/// A traffic light seen by a camera: the area of its signal head in image pixels (two points: a
/// rectangle), and where it stands in the site's world frame, placed in the twin.
struct RegionSignal {
  std::string id;
  std::string name;
  std::vector<cv::Point2f> points;
  std::optional<Eigen::Vector2d> position;  ///< World frame (m); empty until placed.
  double heading_deg = 0.0;  ///< Direction the lamps face, from +x, counter-clockwise.
  double height_m = 4.5;     ///< Height of the signal head above the road.

  /// The area as a polygon (a rectangle for two points).
  [[nodiscard]] std::vector<cv::Point2f> polygon() const;
};

/// Where a camera's detections are accepted: inside one of its areas (if it has any) and within
/// the band of one of its banded lines (if it has any). Detections elsewhere are dropped before
/// tracking, so they are neither tracked nor shown. Stored per camera in regions.json.
struct DetectionRegions {
  std::vector<RegionArea> areas;
  std::vector<RegionLine> lines;
  std::vector<RegionSignal> signals;  ///< Traffic lights; they do not limit detections.

  /// No areas and no lines: every detection is accepted.
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

/// Draws the signal areas in the colour each traffic light shows.
void draw_signals(cv::Mat& canvas, const DetectionRegions& regions,
                  const std::vector<core::SignalState>& states);

}  // namespace jp::sources
