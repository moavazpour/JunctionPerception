#pragma once

#include <Eigen/Geometry>
#include <cstdint>
#include <map>
#include <string>
#include <string_view>
#include <vector>

namespace jp::benchmark {

/// One labelled 3D box of a TUMTraf OpenLABEL frame, in the frame of the LiDAR it was labelled in.
struct Cuboid {
  std::string id;        ///< Object UUID; stable across the frames of one label set (track ID).
  std::string category;  ///< Dataset class, e.g. "CAR", "PEDESTRIAN".
  Eigen::Vector3d center = Eigen::Vector3d::Zero();  ///< Box centre (m).
  Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
  Eigen::Vector3d size = Eigen::Vector3d::Zero();  ///< Length (x), width (y), height (z) in m.
  int num_points = 0;                              ///< LiDAR points inside the box.

  /// Heading around the LiDAR z axis (rad).
  [[nodiscard]] double yaw() const;
  /// Centre of the bottom face (m).
  [[nodiscard]] Eigen::Vector3d bottom_center() const;
  /// The four bottom corners, counter-clockwise seen from above (m).
  [[nodiscard]] std::vector<Eigen::Vector3d> bottom_corners() const;
};

/// Pinhole camera calibration relative to a LiDAR frame.
struct CameraCalibration {
  std::string name;
  int width_px = 0;
  int height_px = 0;
  Eigen::Matrix3d intrinsics = Eigen::Matrix3d::Identity();
  Eigen::Isometry3d camera_from_lidar = Eigen::Isometry3d::Identity();

  /// 3x4 projection matrix from LiDAR coordinates to homogeneous pixels.
  [[nodiscard]] Eigen::Matrix<double, 3, 4> projection() const;
};

/// Content of one OpenLABEL label file (one frame, one LiDAR).
struct LabelFrame {
  std::vector<std::string> image_file_names;
  std::string point_cloud_file_name;
  std::vector<Cuboid> objects;
  std::map<std::string, CameraCalibration, std::less<>> cameras;
};

/// Parses one TUMTraf OpenLABEL label file. `lidar_name` is the coordinate system the boxes and
/// camera poses are expressed in (e.g. "s110_lidar_ouster_south").
/// Throws std::runtime_error on malformed content.
[[nodiscard]] LabelFrame parse_openlabel_frame(std::string_view json_text,
                                               std::string_view lidar_name);

/// Timestamp encoded in a TUMTraf file name "<seconds>_<nanoseconds>_<sensor>.<ext>" (ns).
/// Throws std::invalid_argument when the name does not follow this pattern.
[[nodiscard]] std::int64_t timestamp_from_file_name(std::string_view file_name);

/// Sensor name encoded in a TUMTraf file name "<seconds>_<nanoseconds>_<sensor>.<ext>".
[[nodiscard]] std::string sensor_from_file_name(std::string_view file_name);

}  // namespace jp::benchmark
