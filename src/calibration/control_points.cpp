#include "jp/calibration/control_points.hpp"

#include <yaml-cpp/yaml.h>

#include <Eigen/LU>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <opencv2/calib3d.hpp>
#include <stdexcept>

#include "jp/calibration/ground_homography.hpp"

namespace jp::calibration {

namespace {

constexpr std::size_t kMinFitPoints = 4;

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

YAML::Node flow_pair(double a, double b) {
  YAML::Node node(YAML::NodeType::Sequence);
  node.SetStyle(YAML::EmitterStyle::Flow);
  node.push_back(a);
  node.push_back(b);
  return node;
}

void write_yaml(const std::filesystem::path& path, const YAML::Node& root) {
  if (path.has_parent_path()) {
    std::filesystem::create_directories(path.parent_path());
  }
  std::ofstream out(path);
  if (!out) {
    throw std::runtime_error("cannot write " + path.string());
  }
  out << root << '\n';
}

double error_of(const Eigen::Matrix3d& image_to_ground, const ControlPoint& point) {
  const auto mapped = apply_homography(image_to_ground, point.pixel);
  return mapped ? (*mapped - point.world).norm() : std::numeric_limits<double>::infinity();
}

}  // namespace

Eigen::Vector3d camera_center(const Eigen::Matrix<double, 3, 4>& projection) {
  const Eigen::Matrix3d m = projection.leftCols<3>();
  return -(m.inverse() * projection.col(3));
}

ControlPointFit fit_control_points(const std::vector<ControlPoint>& points, int width_px,
                                   int height_px, double ransac_threshold_m) {
  std::vector<cv::Point2d> pixels;
  std::vector<cv::Point2d> world;
  for (const auto& p : points) {
    if (!p.check) {
      pixels.emplace_back(p.pixel.x(), p.pixel.y());
      world.emplace_back(p.world.x(), p.world.y());
    }
  }
  if (pixels.size() < kMinFitPoints) {
    throw std::invalid_argument("at least four fit points are needed");
  }
  cv::Mat inlier_mask;
  const cv::Mat h = cv::findHomography(pixels, world, cv::RANSAC, ransac_threshold_m, inlier_mask);
  if (h.empty()) {
    throw std::invalid_argument("the control points do not determine a homography");
  }
  ControlPointFit fit;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      fit.image_to_ground(r, c) = h.at<double>(r, c);
    }
  }
  fit.image_to_ground = normalized_homography(fit.image_to_ground);
  fit.fit_points = pixels.size();

  double sum_sq = 0.0;
  double check_sum = 0.0;
  double check_max = 0.0;
  std::size_t checks = 0;
  int fit_index = 0;
  for (const auto& p : points) {
    PointResidual residual{p.id, p.check, false, error_of(fit.image_to_ground, p)};
    if (p.check) {
      ++checks;
      check_sum += residual.error_m;
      check_max = std::max(check_max, residual.error_m);
    } else {
      residual.inlier = inlier_mask.at<unsigned char>(fit_index++) != 0;
      if (residual.inlier) {
        ++fit.inliers;
        sum_sq += residual.error_m * residual.error_m;
      }
    }
    fit.residuals.push_back(residual);
  }
  fit.fit_rms_m = fit.inliers > 0 ? std::sqrt(sum_sq / static_cast<double>(fit.inliers)) : 0.0;
  if (checks > 0) {
    fit.check_mean_m = check_sum / static_cast<double>(checks);
    fit.check_max_m = check_max;
  }
  const auto projection =
      projection_from_homography(fit.image_to_ground.inverse(), width_px, height_px);
  if (projection) {
    fit.ground_projection = projection;
    fit.camera_position = camera_center(*projection);
  }
  return fit;
}

std::vector<Eigen::Vector2d> agreeing_points(const std::vector<ControlPoint>& points,
                                             const ControlPointFit& fit, double max_error_m) {
  std::vector<Eigen::Vector2d> agreeing;
  for (std::size_t i = 0; i < points.size() && i < fit.residuals.size(); ++i) {
    const auto& r = fit.residuals[i];
    if (r.check ? r.error_m <= max_error_m : r.inlier) {
      agreeing.push_back(points[i].world);
    }
  }
  return agreeing;
}

GroundCalibration calibration_from_fit(const ControlPointFit& fit, const std::string& camera,
                                       int width_px, int height_px,
                                       const std::optional<Eigen::Vector2d>& camera_position,
                                       std::optional<double> camera_height_m) {
  GroundCalibration calibration;
  calibration.camera = camera;
  calibration.width_px = width_px;
  calibration.height_px = height_px;
  calibration.image_to_ground = fit.image_to_ground;
  calibration.ground_projection = fit.ground_projection;
  calibration.ground_projection_estimated = fit.ground_projection.has_value();
  if (fit.camera_position) {
    calibration.camera_ground_position_m = fit.camera_position->head<2>();
    calibration.camera_height_m = fit.camera_position->z();
  }
  if (camera_position) {
    calibration.camera_ground_position_m = *camera_position;
    calibration.camera_position_entered = true;
  }
  if (camera_height_m) {
    calibration.camera_height_m = *camera_height_m;
    calibration.camera_height_entered = true;
  }
  // An entered position or height, completed by the estimate where only one was entered, fixes
  // the camera centre the projection is derived from.
  if ((camera_position || camera_height_m) && (camera_position || fit.camera_position) &&
      calibration.camera_height_m > 0.0) {
    const Eigen::Vector3d center(calibration.camera_ground_position_m.x(),
                                 calibration.camera_ground_position_m.y(),
                                 calibration.camera_height_m);
    calibration.ground_projection =
        projection_from_camera_center(fit.image_to_ground.inverse(), center, width_px, height_px);
    calibration.ground_projection_estimated = calibration.ground_projection.has_value();
  }
  return calibration;
}

void write_ground_calibration(const std::filesystem::path& path,
                              const GroundCalibration& calibration, const ControlPointFit& fit) {
  YAML::Node root;
  root["camera"] = calibration.camera;
  root["image_size_px"] = flow_pair(calibration.width_px, calibration.height_px);
  root["source"] = "control_points.yaml";
  root["image_to_ground"] = matrix_node(calibration.image_to_ground);
  root["ground_to_image"] =
      matrix_node(normalized_homography(calibration.image_to_ground.inverse()));
  if (calibration.ground_projection) {
    root["ground_projection"] = matrix_node(*calibration.ground_projection);
    root["ground_projection_estimated"] = calibration.ground_projection_estimated;
  }
  root["camera_ground_position_m"] =
      flow_pair(calibration.camera_ground_position_m.x(), calibration.camera_ground_position_m.y());
  root["camera_height_m"] = calibration.camera_height_m;
  root["camera_position_entered"] = calibration.camera_position_entered;
  root["camera_height_entered"] = calibration.camera_height_entered;
  YAML::Node check;
  check["fit_points"] = fit.fit_points;
  check["inliers"] = fit.inliers;
  check["fit_rms_m"] = fit.fit_rms_m;
  if (fit.check_mean_m && fit.check_max_m) {
    check["check_mean_m"] = *fit.check_mean_m;
    check["check_max_m"] = *fit.check_max_m;
  }
  root["check"] = check;
  write_yaml(path, root);
}

void write_control_points(const std::filesystem::path& path,
                          const std::vector<ControlPoint>& points, const core::GeoAnchor& anchor) {
  YAML::Node list(YAML::NodeType::Sequence);
  for (const auto& p : points) {
    const auto geo = anchor.to_geo(p.world);
    YAML::Node node;
    node.SetStyle(YAML::EmitterStyle::Flow);
    node["id"] = p.id;
    node["role"] = p.check ? "check" : "fit";
    node["u"] = p.pixel.x();
    node["v"] = p.pixel.y();
    node["lat"] = geo.lat_deg;
    node["lon"] = geo.lon_deg;
    list.push_back(node);
  }
  YAML::Node root;
  root["points"] = list;
  write_yaml(path, root);
}

std::vector<ControlPoint> load_control_points(const std::filesystem::path& path,
                                              const core::GeoAnchor& anchor) {
  std::vector<ControlPoint> points;
  try {
    const YAML::Node root = YAML::LoadFile(path.string());
    for (const auto& node : root["points"]) {
      ControlPoint p;
      p.id = node["id"].as<std::string>();
      p.check = node["role"].as<std::string>() == "check";
      p.pixel = {node["u"].as<double>(), node["v"].as<double>()};
      p.world = anchor.to_world(node["lat"].as<double>(), node["lon"].as<double>());
      points.push_back(std::move(p));
    }
  } catch (const YAML::Exception& error) {
    throw std::runtime_error("invalid control points " + path.string() + ": " + error.what());
  }
  return points;
}

}  // namespace jp::calibration
