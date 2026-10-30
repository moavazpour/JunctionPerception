#include "jp/benchmark/openlabel.hpp"

#include <charconv>
#include <cmath>
#include <nlohmann/json.hpp>
#include <stdexcept>

namespace jp::benchmark {

namespace {

constexpr std::size_t kCuboidValues = 10;
constexpr std::size_t kNanosecondDigits = 9;
constexpr std::int64_t kNanosecondsPerSecond = 1'000'000'000;

Eigen::Isometry3d isometry_from_row_major(const nlohmann::json& values) {
  const auto v = values.get<std::vector<double>>();
  if (v.size() != 16) {
    throw std::runtime_error("matrix4x4 must have 16 values");
  }
  Eigen::Matrix4d m;
  for (Eigen::Index r = 0; r < 4; ++r) {
    for (Eigen::Index c = 0; c < 4; ++c) {
      m(r, c) = v[static_cast<std::size_t>((r * 4) + c)];
    }
  }
  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
  iso.linear() = m.topLeftCorner<3, 3>();
  iso.translation() = m.topRightCorner<3, 1>();
  return iso;
}

int num_points_of(const nlohmann::json& cuboid) {
  const auto attributes = cuboid.find("attributes");
  if (attributes == cuboid.end() || !attributes->contains("num")) {
    return 0;
  }
  for (const auto& attribute : attributes->at("num")) {
    if (attribute.at("name") == "num_points") {
      return attribute.at("val").get<int>();
    }
  }
  return 0;
}

Cuboid parse_cuboid(const std::string& id, const nlohmann::json& object_data) {
  const auto& cuboid = object_data.at("cuboid");
  const auto val = cuboid.at("val").get<std::vector<double>>();
  if (val.size() != kCuboidValues) {
    throw std::runtime_error("cuboid of object " + id + " must have 10 values");
  }
  Cuboid box;
  box.id = id;
  box.category = object_data.at("type").get<std::string>();
  box.center = {val[0], val[1], val[2]};
  box.rotation = Eigen::Quaterniond(val[6], val[3], val[4], val[5]).normalized();
  box.size = {val[7], val[8], val[9]};
  box.num_points = num_points_of(cuboid);
  return box;
}

CameraCalibration parse_camera(const std::string& name, const nlohmann::json& stream,
                               const nlohmann::json& coordinate_systems,
                               std::string_view lidar_name) {
  const auto& pinhole = stream.at("stream_properties").at("intrinsics_pinhole");
  const auto k = pinhole.at("camera_matrix_3x4").get<std::vector<std::vector<double>>>();
  if (k.size() != 3 || k[0].size() != 4) {
    throw std::runtime_error("camera_matrix_3x4 of " + name + " must be 3x4");
  }
  CameraCalibration camera;
  camera.name = name;
  camera.width_px = pinhole.at("width_px").get<int>();
  camera.height_px = pinhole.at("height_px").get<int>();
  for (Eigen::Index r = 0; r < 3; ++r) {
    for (Eigen::Index c = 0; c < 3; ++c) {
      camera.intrinsics(r, c) = k[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)];
    }
  }
  const auto& system = coordinate_systems.at(name);
  if (system.at("parent").get<std::string>() != lidar_name) {
    throw std::runtime_error("camera " + name + " is not given relative to " +
                             std::string(lidar_name));
  }
  // In TUMTraf R02 the camera's pose_wrt_parent maps LiDAR coordinates into the camera frame.
  camera.camera_from_lidar = isometry_from_row_major(system.at("pose_wrt_parent").at("matrix4x4"));
  return camera;
}

std::int64_t parse_integer(std::string_view text) {
  std::int64_t value = 0;
  const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), value);
  if (error != std::errc{} || end != text.data() + text.size()) {
    throw std::invalid_argument("not an integer: " + std::string(text));
  }
  return value;
}

}  // namespace

double Cuboid::yaw() const {
  const Eigen::Vector3d forward = rotation * Eigen::Vector3d::UnitX();
  return std::atan2(forward.y(), forward.x());
}

Eigen::Vector3d Cuboid::bottom_center() const {
  return center - (0.5 * size.z() * (rotation * Eigen::Vector3d::UnitZ()));
}

std::vector<Eigen::Vector3d> Cuboid::bottom_corners() const {
  const double hl = 0.5 * size.x();
  const double hw = 0.5 * size.y();
  const double hh = 0.5 * size.z();
  std::vector<Eigen::Vector3d> corners;
  corners.reserve(4);
  for (const auto& [sx, sy] : {std::pair{1.0, 1.0}, {-1.0, 1.0}, {-1.0, -1.0}, {1.0, -1.0}}) {
    corners.emplace_back(center + (rotation * Eigen::Vector3d(sx * hl, sy * hw, -hh)));
  }
  return corners;
}

Eigen::Matrix<double, 3, 4> CameraCalibration::projection() const {
  return intrinsics * camera_from_lidar.matrix().topRows<3>();
}

LabelFrame parse_openlabel_frame(std::string_view json_text, std::string_view lidar_name) {
  const auto doc = nlohmann::json::parse(json_text);
  const auto& openlabel = doc.at("openlabel");
  const auto& frames = openlabel.at("frames");
  if (frames.size() != 1) {
    throw std::runtime_error("expected exactly one frame per label file");
  }
  const auto& frame = frames.begin().value();
  const auto& properties = frame.at("frame_properties");

  LabelFrame result;
  result.image_file_names = properties.at("image_file_names").get<std::vector<std::string>>();
  result.point_cloud_file_name = properties.at("point_cloud_file_name").get<std::string>();
  for (const auto& [id, object] : frame.at("objects").items()) {
    result.objects.push_back(parse_cuboid(id, object.at("object_data")));
  }
  const auto& coordinate_systems = openlabel.at("coordinate_systems");
  for (const auto& [name, stream] : openlabel.at("streams").items()) {
    if (stream.at("type") == "camera") {
      result.cameras.emplace(name, parse_camera(name, stream, coordinate_systems, lidar_name));
    }
  }
  return result;
}

std::int64_t timestamp_from_file_name(std::string_view file_name) {
  const auto first = file_name.find('_');
  const auto second = first == std::string_view::npos ? first : file_name.find('_', first + 1);
  if (second == std::string_view::npos || second - first - 1 != kNanosecondDigits) {
    throw std::invalid_argument("not a TUMTraf file name: " + std::string(file_name));
  }
  const std::int64_t seconds = parse_integer(file_name.substr(0, first));
  const std::int64_t nanoseconds = parse_integer(file_name.substr(first + 1, kNanosecondDigits));
  return (seconds * kNanosecondsPerSecond) + nanoseconds;
}

std::string sensor_from_file_name(std::string_view file_name) {
  static_cast<void>(timestamp_from_file_name(file_name));
  const auto start = file_name.find('_', file_name.find('_') + 1) + 1;
  const auto dot = file_name.rfind('.');
  return std::string(file_name.substr(start, dot == std::string_view::npos ? dot : dot - start));
}

}  // namespace jp::benchmark
