#include "jp/calibration/ground_calibration.hpp"

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace jp::calibration {

namespace {

Eigen::MatrixXd matrix_of(const YAML::Node& node, Eigen::Index rows, Eigen::Index cols) {
  if (!node || !node.IsSequence() || static_cast<Eigen::Index>(node.size()) != rows) {
    throw std::runtime_error("matrix must have " + std::to_string(rows) + " rows");
  }
  Eigen::MatrixXd m(rows, cols);
  for (Eigen::Index r = 0; r < rows; ++r) {
    const YAML::Node row = node[static_cast<std::size_t>(r)];
    if (!row.IsSequence() || static_cast<Eigen::Index>(row.size()) != cols) {
      throw std::runtime_error("matrix row must have " + std::to_string(cols) + " values");
    }
    for (Eigen::Index c = 0; c < cols; ++c) {
      m(r, c) = row[static_cast<std::size_t>(c)].as<double>();
    }
  }
  return m;
}

}  // namespace

GroundCalibration parse_ground_calibration(std::string_view yaml_text) {
  GroundCalibration calibration;
  try {
    const YAML::Node root = YAML::Load(std::string(yaml_text));
    calibration.camera = root["camera"].as<std::string>();
    calibration.width_px = root["image_size_px"][0].as<int>();
    calibration.height_px = root["image_size_px"][1].as<int>();
    calibration.image_to_ground = matrix_of(root["image_to_ground"], 3, 3);
    const YAML::Node position = root["camera_ground_position_m"];
    calibration.camera_ground_position_m = {position[0].as<double>(), position[1].as<double>()};
    calibration.camera_height_m = root["camera_height_m"].as<double>();
    if (root["sensor_from_ground"]) {
      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      pose.matrix() = matrix_of(root["sensor_from_ground"], 4, 4);
      calibration.sensor_from_ground = pose;
    }
  } catch (const YAML::Exception& error) {
    throw std::runtime_error(std::string("invalid ground calibration: ") + error.what());
  }
  return calibration;
}

GroundCalibration load_ground_calibration(const std::filesystem::path& path) {
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("cannot read " + path.string());
  }
  std::ostringstream text;
  text << in.rdbuf();
  return parse_ground_calibration(text.str());
}

}  // namespace jp::calibration
