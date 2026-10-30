#include "jp/calibration/benchmark_calibration.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <fstream>
#include <map>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <stdexcept>

#include "jp/benchmark/tumtraf_sequence.hpp"
#include "jp/calibration/ground_homography.hpp"
#include "jp/core/rigid_transform.hpp"
#include "jp/core/statistics.hpp"

namespace jp::calibration {

namespace {

constexpr double kMinDepthM = 1.0;
constexpr std::array kDistanceBinEdgesM = {0.0, 25.0, 50.0, 75.0, 100.0, 1000.0};
constexpr double kGridSpacingM = 10.0;
constexpr int kGridHalfLines = 12;
constexpr int kGridHalfSteps = 120;
constexpr double kGridStepM = 1.0;

// Pixel of a sensor-frame point if it lies in front of the camera and inside the image.
std::optional<Eigen::Vector2d> visible_pixel(const benchmark::CameraCalibration& camera,
                                             const Eigen::Vector3d& sensor_point) {
  const Eigen::Vector3d in_camera = camera.camera_from_lidar * sensor_point;
  if (in_camera.z() < kMinDepthM) {
    return std::nullopt;
  }
  const Eigen::Vector2d pixel = (camera.intrinsics * in_camera).hnormalized();
  if (pixel.x() < 0.0 || pixel.y() < 0.0 || pixel.x() >= camera.width_px ||
      pixel.y() >= camera.height_px) {
    return std::nullopt;
  }
  return pixel;
}

bool in_front(const benchmark::CameraCalibration& camera, const Eigen::Vector3d& sensor_point) {
  return (camera.camera_from_lidar * sensor_point).z() >= kMinDepthM;
}

cv::Scalar class_color(const std::string& category) {
  static const std::map<std::string, cv::Scalar, std::less<>> colors = {
      {"CAR", {80, 200, 80}},
      {"VAN", {80, 220, 200}},
      {"TRUCK", {60, 140, 255}},
      {"TRAILER", {40, 90, 200}},
      {"BUS", {200, 120, 40}},
      {"PEDESTRIAN", {60, 60, 240}},
      {"BICYCLE", {240, 80, 220}},
      {"MOTORCYCLE", {180, 60, 180}},
      {"EMERGENCY_VEHICLE", {0, 0, 160}},
      {"OTHER", {160, 160, 160}}};
  const auto it = colors.find(category);
  return it == colors.end() ? cv::Scalar(255, 255, 255) : it->second;
}

std::string lowercase(std::string text) {
  std::ranges::transform(text, text.begin(),
                         [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return text;
}

YAML::Node matrix_node(const Eigen::MatrixXd& m) {
  YAML::Node rows(YAML::NodeType::Sequence);
  for (Eigen::Index r = 0; r < m.rows(); ++r) {
    YAML::Node row(YAML::NodeType::Sequence);
    row.SetStyle(YAML::EmitterStyle::Flow);
    for (Eigen::Index c = 0; c < m.cols(); ++c) {
      row.push_back(m(r, c));
    }
    rows.push_back(row);
  }
  return rows;
}

// Draws ground-plane geometry into a camera image through the ground homography.
class GroundPainter {
 public:
  GroundPainter(const cv::Mat& image, const BenchmarkCameraCalibration& calibration,
                const benchmark::CameraCalibration& camera)
      : canvas_(image.clone()),
        calibration_(&calibration),
        camera_(&camera),
        ground_from_sensor_(core::rigid_inverse(calibration.sensor_from_ground)) {}

  void draw_grid() {
    const cv::Scalar color(200, 200, 200);
    for (int line = -kGridHalfLines; line <= kGridHalfLines; ++line) {
      const double fixed = line * kGridSpacingM;
      for (int step = -kGridHalfSteps; step < kGridHalfSteps; ++step) {
        const double t0 = step * kGridStepM;
        const double t1 = t0 + kGridStepM;
        draw_segment({t0, fixed}, {t1, fixed}, color);
        draw_segment({fixed, t0}, {fixed, t1}, color);
      }
    }
  }

  void draw_footprint(const benchmark::GroundTruthObject& object, const Plane& ground) {
    std::vector<cv::Point> polygon;
    for (const auto& corner : object.box.bottom_corners()) {
      const Eigen::Vector3d on_ground = ground_from_sensor_ * ground.project(corner);
      const auto pixel = to_pixel(on_ground.head<2>());
      if (!pixel) {
        return;
      }
      polygon.push_back(*pixel);
    }
    const cv::Scalar color = class_color(object.box.category);
    cv::polylines(canvas_, polygon, true, color, 2, cv::LINE_AA);
    const cv::Point anchor = *std::ranges::max_element(polygon, {}, [](const cv::Point& p) {
      return p.y;
    }) + cv::Point(4, 14);
    const std::string label =
        std::to_string(object.track_id) + " " + lowercase(object.box.category);
    cv::putText(canvas_, label, anchor, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 3,
                cv::LINE_AA);
    cv::putText(canvas_, label, anchor, cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_AA);
  }

  [[nodiscard]] const cv::Mat& canvas() const { return canvas_; }

 private:
  [[nodiscard]] std::optional<cv::Point> to_pixel(const Eigen::Vector2d& ground_xy) const {
    const Eigen::Vector3d sensor =
        calibration_->sensor_from_ground * Eigen::Vector3d(ground_xy.x(), ground_xy.y(), 0.0);
    if (!in_front(*camera_, sensor)) {
      return std::nullopt;
    }
    const auto pixel = apply_homography(calibration_->ground_to_image, ground_xy);
    if (!pixel) {
      return std::nullopt;
    }
    return cv::Point(static_cast<int>(std::lround(pixel->x())),
                     static_cast<int>(std::lround(pixel->y())));
  }

  void draw_segment(const Eigen::Vector2d& a, const Eigen::Vector2d& b, const cv::Scalar& color) {
    const auto pa = to_pixel(a);
    const auto pb = to_pixel(b);
    if (pa && pb) {
      cv::line(canvas_, *pa, *pb, color, 1, cv::LINE_AA);
    }
  }

  cv::Mat canvas_;
  const BenchmarkCameraCalibration* calibration_;
  const benchmark::CameraCalibration* camera_;
  Eigen::Isometry3d ground_from_sensor_;
};

}  // namespace

PlaneFit fit_ground_plane(const std::vector<std::vector<benchmark::GroundTruthObject>>& frames,
                          const PlaneFitOptions& options) {
  std::vector<Eigen::Vector3d> points;
  for (const auto& frame : frames) {
    for (const auto& object : frame) {
      points.push_back(object.box.bottom_center());
    }
  }
  return fit_plane(points, options);
}

PlaneFit fit_ground_plane_in_view(
    const std::vector<std::vector<benchmark::GroundTruthObject>>& frames,
    const benchmark::CameraCalibration& camera, const PlaneFitOptions& options) {
  std::vector<Eigen::Vector3d> points;
  for (const auto& frame : frames) {
    for (const auto& object : frame) {
      const Eigen::Vector3d bottom = object.box.bottom_center();
      if (visible_pixel(camera, bottom)) {
        points.push_back(bottom);
      }
    }
  }
  return fit_plane(points, options);
}

BenchmarkCameraCalibration calibrate_benchmark_camera(
    const benchmark::CameraCalibration& camera, const Plane& ground,
    const std::vector<std::vector<benchmark::GroundTruthObject>>& frames) {
  BenchmarkCameraCalibration result;
  result.camera = camera.name;
  result.width_px = camera.width_px;
  result.height_px = camera.height_px;
  result.sensor_from_ground = sensor_from_ground_frame(ground, Eigen::Vector3d::Zero());
  result.ground_to_image =
      ground_to_image_homography(camera.projection(), result.sensor_from_ground);
  result.image_to_ground = normalized_homography(result.ground_to_image.inverse());
  result.ground_projection = camera.projection() * result.sensor_from_ground.matrix();

  const Eigen::Isometry3d ground_from_sensor = core::rigid_inverse(result.sensor_from_ground);
  const Eigen::Vector3d camera_in_ground =
      ground_from_sensor * core::rigid_inverse(camera.camera_from_lidar).translation();
  result.camera_ground_position_m = camera_in_ground.head<2>();
  result.camera_height_m = camera_in_ground.z();

  std::vector<double> errors;
  std::vector<double> distances;
  std::vector<double> heights;
  for (const auto& frame : frames) {
    for (const auto& object : frame) {
      const Eigen::Vector3d bottom = object.box.bottom_center();
      const auto pixel = visible_pixel(camera, bottom);
      if (!pixel) {
        continue;
      }
      const auto mapped = apply_homography(result.image_to_ground, *pixel);
      if (!mapped) {
        continue;
      }
      const Eigen::Vector3d truth = ground_from_sensor * bottom;
      errors.push_back((*mapped - truth.head<2>()).norm());
      distances.push_back((truth.head<2>() - result.camera_ground_position_m).norm());
      heights.push_back(std::abs(truth.z()));
    }
  }

  GroundCheck& check = result.check;
  check.objects = errors.size();
  if (!errors.empty()) {
    double sum = 0.0;
    for (const double e : errors) {
      sum += e;
    }
    check.mean_m = sum / static_cast<double>(errors.size());
    check.median_m = core::percentile(errors, 0.5);
    check.p90_m = core::percentile(errors, 0.9);
    check.max_m = *std::ranges::max_element(errors);
    check.median_height_above_plane_m = core::percentile(heights, 0.5);
  }
  for (std::size_t b = 0; b + 1 < kDistanceBinEdgesM.size(); ++b) {
    DistanceBin bin{kDistanceBinEdgesM.at(b), kDistanceBinEdgesM.at(b + 1)};
    std::vector<double> in_bin;
    for (std::size_t i = 0; i < errors.size(); ++i) {
      if (distances[i] >= bin.from_m && distances[i] < bin.to_m) {
        in_bin.push_back(errors[i]);
      }
    }
    bin.objects = in_bin.size();
    bin.median_m = core::percentile(in_bin, 0.5);
    bin.p90_m = core::percentile(in_bin, 0.9);
    if (bin.objects > 0) {
      check.by_distance.push_back(bin);
    }
  }
  return result;
}

cv::Mat draw_ground_truth_overlay(const cv::Mat& image,
                                  const BenchmarkCameraCalibration& calibration,
                                  const benchmark::CameraCalibration& camera, const Plane& ground,
                                  const std::vector<benchmark::GroundTruthObject>& objects) {
  GroundPainter painter(image, calibration, camera);
  painter.draw_grid();
  for (const auto& object : objects) {
    painter.draw_footprint(object, ground);
  }
  return painter.canvas();
}

void write_ground_homography(const std::filesystem::path& path,
                             const BenchmarkCameraCalibration& calibration, const PlaneFit& ground,
                             const std::string& source_sequence) {
  YAML::Node root;
  root["camera"] = calibration.camera;
  root["image_size_px"].push_back(calibration.width_px);
  root["image_size_px"].push_back(calibration.height_px);
  root["image_size_px"].SetStyle(YAML::EmitterStyle::Flow);
  root["sensor_frame"] = std::string(benchmark::kSouthLidar);
  root["source_sequence"] = source_sequence;
  root["image_to_ground"] = matrix_node(calibration.image_to_ground);
  root["ground_to_image"] = matrix_node(calibration.ground_to_image);
  root["ground_projection"] = matrix_node(calibration.ground_projection);
  root["sensor_from_ground"] = matrix_node(calibration.sensor_from_ground.matrix());
  root["camera_ground_position_m"].push_back(calibration.camera_ground_position_m.x());
  root["camera_ground_position_m"].push_back(calibration.camera_ground_position_m.y());
  root["camera_ground_position_m"].SetStyle(YAML::EmitterStyle::Flow);
  root["camera_height_m"] = calibration.camera_height_m;

  YAML::Node plane;
  plane["normal"].push_back(ground.plane.normal.x());
  plane["normal"].push_back(ground.plane.normal.y());
  plane["normal"].push_back(ground.plane.normal.z());
  plane["normal"].SetStyle(YAML::EmitterStyle::Flow);
  plane["offset_m"] = ground.plane.offset;
  plane["inliers"] = ground.inliers;
  plane["inlier_rms_m"] = ground.inlier_rms_m;
  root["ground_plane"] = plane;

  const GroundCheck& c = calibration.check;
  YAML::Node check;
  check["objects"] = c.objects;
  check["mean_m"] = c.mean_m;
  check["median_m"] = c.median_m;
  check["p90_m"] = c.p90_m;
  check["max_m"] = c.max_m;
  check["median_height_above_plane_m"] = c.median_height_above_plane_m;
  for (const auto& bin : c.by_distance) {
    YAML::Node b;
    b["from_m"] = bin.from_m;
    b["to_m"] = bin.to_m;
    b["objects"] = bin.objects;
    b["median_m"] = bin.median_m;
    b["p90_m"] = bin.p90_m;
    check["by_distance"].push_back(b);
  }
  root["check"] = check;

  std::filesystem::create_directories(path.parent_path());
  std::ofstream out(path);
  if (!out) {
    throw std::runtime_error("cannot write " + path.string());
  }
  out << root << '\n';
}

}  // namespace jp::calibration
