#include "jp/sources/detection_regions.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <nlohmann/json.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <stdexcept>

namespace jp::sources {

namespace {

using json = nlohmann::json;

constexpr double kEpsilon = 1e-9;
constexpr std::size_t kCorners = 4;

std::vector<cv::Point2f> points_from(const json& j, std::size_t min_points,
                                     const std::string& what) {
  std::vector<cv::Point2f> points;
  for (const auto& p : j.at("points")) {
    points.emplace_back(p.at(0).get<float>(), p.at(1).get<float>());
  }
  if (points.size() < min_points) {
    throw std::runtime_error(what + " needs at least " + std::to_string(min_points) + " points");
  }
  return points;
}

json points_json(const std::vector<cv::Point2f>& points) {
  json list = json::array();
  for (const auto& p : points) {
    list.push_back({std::round(p.x * 10.0F) / 10.0F, std::round(p.y * 10.0F) / 10.0F});
  }
  return list;
}

std::optional<Eigen::Vector2d> to_ground(const cv::Point2f& pixel, const Eigen::Matrix3d& h) {
  const Eigen::Vector3d g = h * Eigen::Vector3d(pixel.x, pixel.y, 1.0);
  if (std::abs(g.z()) < kEpsilon) {
    return std::nullopt;
  }
  return g.hnormalized();
}

double segment_distance(const Eigen::Vector2d& p, const Eigen::Vector2d& a,
                        const Eigen::Vector2d& b) {
  const Eigen::Vector2d ab = b - a;
  const double length_sq = ab.squaredNorm();
  const double t = length_sq < kEpsilon ? 0.0 : std::clamp((p - a).dot(ab) / length_sq, 0.0, 1.0);
  return (a + (t * ab) - p).norm();
}

/// Ground distance from a point to a line drawn in the image; infinite where it maps to infinity.
double ground_distance(const Eigen::Vector2d& point, const RegionLine& line,
                       const Eigen::Matrix3d& h) {
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 1; i < line.points.size(); ++i) {
    const auto a = to_ground(line.points[i - 1], h);
    const auto b = to_ground(line.points[i], h);
    if (a && b) {
      best = std::min(best, segment_distance(point, *a, *b));
    }
  }
  return best;
}

std::vector<cv::Point> rounded(const std::vector<cv::Point2f>& points) {
  std::vector<cv::Point> out;
  out.reserve(points.size());
  for (const auto& p : points) {
    out.emplace_back(cvRound(p.x), cvRound(p.y));
  }
  return out;
}

}  // namespace

cv::Point2f DetectionRegions::contact(const detection::Detection& detection) {
  if (detection.keypoints.size() == kCorners) {
    cv::Point2f sum(0.0F, 0.0F);
    for (const auto& k : detection.keypoints) {
      sum += k.pixel;
    }
    return sum * (1.0F / static_cast<float>(kCorners));
  }
  const auto& box = detection.box;
  return {box.x + (box.width / 2.0F), box.y + box.height};
}

bool DetectionRegions::accepts(const detection::Detection& detection,
                               const std::optional<Eigen::Matrix3d>& image_to_ground) const {
  const cv::Point2f point = contact(detection);
  if (!areas.empty() && std::ranges::none_of(areas, [&](const RegionArea& area) {
        return cv::pointPolygonTest(area.points, point, false) >= 0.0;
      })) {
    return false;
  }
  const bool banded = std::ranges::any_of(lines, [](const auto& l) { return l.band_m > 0.0; });
  if (!banded || !image_to_ground) {
    return true;
  }
  const auto ground = to_ground(point, *image_to_ground);
  if (!ground) {
    return false;
  }
  return std::ranges::any_of(lines, [&](const RegionLine& line) {
    return line.band_m > 0.0 && ground_distance(*ground, line, *image_to_ground) <= line.band_m;
  });
}

std::string DetectionRegions::to_json() const {
  json out = {{"areas", json::array()}, {"lines", json::array()}};
  for (const auto& area : areas) {
    out["areas"].push_back(
        {{"id", area.id}, {"name", area.name}, {"points", points_json(area.points)}});
  }
  for (const auto& line : lines) {
    out["lines"].push_back({{"id", line.id},
                            {"name", line.name},
                            {"points", points_json(line.points)},
                            {"band_m", line.band_m},
                            {"speed_limit_kmh", line.speed_limit_kmh}});
  }
  return out.dump(2);
}

DetectionRegions DetectionRegions::parse(const std::string& text) {
  try {
    const json root = json::parse(text);
    DetectionRegions regions;
    for (const auto& a : root.value("areas", json::array())) {
      regions.areas.push_back(
          {a.at("id").get<std::string>(), a.value("name", ""), points_from(a, 3, "an area")});
    }
    for (const auto& l : root.value("lines", json::array())) {
      RegionLine line{l.at("id").get<std::string>(), l.value("name", ""),
                      points_from(l, 2, "a line"), l.value("band_m", 0.0),
                      l.value("speed_limit_kmh", 0.0)};
      if (line.band_m < 0.0 || line.speed_limit_kmh < 0.0) {
        throw std::runtime_error("band_m and speed_limit_kmh must not be negative");
      }
      regions.lines.push_back(std::move(line));
    }
    return regions;
  } catch (const json::exception& error) {
    throw std::runtime_error(std::string("invalid regions: ") + error.what());
  }
}

DetectionRegions load_regions(const std::filesystem::path& path) {
  if (!std::filesystem::exists(path)) {
    return {};
  }
  std::ifstream in(path);
  std::ostringstream text;
  text << in.rdbuf();
  return DetectionRegions::parse(text.str());
}

void draw_regions(cv::Mat& canvas, const DetectionRegions& regions) {
  const cv::Scalar area_color(240, 200, 60);  // BGR: light blue
  const cv::Scalar line_color(60, 190, 250);  // amber
  for (const auto& area : regions.areas) {
    cv::polylines(canvas, std::vector{rounded(area.points)}, true, area_color, 2, cv::LINE_AA);
  }
  for (const auto& line : regions.lines) {
    const auto points = rounded(line.points);
    cv::polylines(canvas, std::vector{points}, false, line_color, 3, cv::LINE_AA);
    if (!line.name.empty()) {
      cv::putText(canvas, line.name, points.front() + cv::Point(6, -8), cv::FONT_HERSHEY_SIMPLEX,
                  0.6, line_color, 2, cv::LINE_AA);
    }
  }
}

}  // namespace jp::sources
