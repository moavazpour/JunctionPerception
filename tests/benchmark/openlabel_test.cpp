#include "jp/benchmark/openlabel.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <stdexcept>

namespace {

// Minimal TUMTraf R02 label file: one camera relative to the south LiDAR, one car.
constexpr std::string_view kLabelFile = R"json({
  "openlabel": {
    "metadata": {"schema_version": "1.0.0"},
    "coordinate_systems": {
      "s110_lidar_ouster_south": {"type": "sensor_cs", "parent": "s110_base", "children": []},
      "cam": {"type": "sensor_cs", "parent": "s110_lidar_ouster_south", "children": [],
              "pose_wrt_parent": {"matrix4x4": [1,0,0,1, 0,1,0,2, 0,0,1,3, 0,0,0,1]}}
    },
    "streams": {
      "cam": {"type": "camera", "uri": "", "description": "",
              "stream_properties": {"intrinsics_pinhole": {
                "width_px": 1920, "height_px": 1200,
                "camera_matrix_3x4": [[1000, 0, 960, 0], [0, 1000, 600, 0], [0, 0, 1, 0]]}}}
    },
    "frames": {"0": {
      "frame_properties": {
        "timestamp": 1651673050.0548077,
        "point_cloud_file_name": "1651673050_054807703_s110_lidar_ouster_south.pcd",
        "image_file_names": ["1651673049_964807393_cam.jpg"]
      },
      "objects": {
        "uuid-1": {"object_data": {"name": "CAR_1", "type": "CAR", "cuboid": {
          "name": "shape3D",
          "val": [10.0, 5.0, -6.0, 0.0, 0.0, 0.7071067811865476, 0.7071067811865476, 4.0, 2.0, 1.5],
          "attributes": {"num": [{"name": "num_points", "val": 134}]}}}}
      }
    }}
  }
})json";

TEST(OpenLabel, ParsesCuboidsAndCamera) {
  const auto frame = jp::benchmark::parse_openlabel_frame(kLabelFile, "s110_lidar_ouster_south");
  ASSERT_EQ(frame.objects.size(), 1U);
  const auto& car = frame.objects.front();
  EXPECT_EQ(car.id, "uuid-1");
  EXPECT_EQ(car.category, "CAR");
  EXPECT_EQ(car.num_points, 134);
  EXPECT_TRUE(car.center.isApprox(Eigen::Vector3d(10.0, 5.0, -6.0)));
  EXPECT_NEAR(car.yaw(), std::numbers::pi / 2, 1e-9);
  EXPECT_TRUE(car.bottom_center().isApprox(Eigen::Vector3d(10.0, 5.0, -6.75)));

  ASSERT_EQ(frame.cameras.count("cam"), 1U);
  const auto& camera = frame.cameras.at("cam");
  EXPECT_EQ(camera.width_px, 1920);
  EXPECT_DOUBLE_EQ(camera.intrinsics(0, 2), 960.0);
  EXPECT_TRUE(camera.camera_from_lidar.translation().isApprox(Eigen::Vector3d(1, 2, 3)));
  // A point 7 m ahead of the camera on its optical axis projects to the principal point.
  const Eigen::Vector3d lidar_point(-1.0, -2.0, 4.0);
  const Eigen::Vector2d pixel = (camera.projection() * lidar_point.homogeneous()).hnormalized();
  EXPECT_TRUE(pixel.isApprox(Eigen::Vector2d(960.0, 600.0)));
}

TEST(OpenLabel, RejectsCameraOfOtherLidar) {
  EXPECT_THROW(static_cast<void>(
                   jp::benchmark::parse_openlabel_frame(kLabelFile, "s110_lidar_ouster_north")),
               std::runtime_error);
}

TEST(OpenLabel, BottomCornersSpanFootprint) {
  jp::benchmark::Cuboid box;
  box.size = {4.0, 2.0, 1.0};
  const auto corners = box.bottom_corners();
  ASSERT_EQ(corners.size(), 4U);
  for (const auto& c : corners) {
    EXPECT_DOUBLE_EQ(std::abs(c.x()), 2.0);
    EXPECT_DOUBLE_EQ(std::abs(c.y()), 1.0);
    EXPECT_DOUBLE_EQ(c.z(), -0.5);
  }
}

TEST(OpenLabel, FileNameTimestampAndSensor) {
  constexpr std::string_view name = "1651673049_964807393_s110_camera_basler_south1_8mm.jpg";
  EXPECT_EQ(jp::benchmark::timestamp_from_file_name(name), 1651673049964807393);
  EXPECT_EQ(jp::benchmark::sensor_from_file_name(name), "s110_camera_basler_south1_8mm");
  EXPECT_THROW(static_cast<void>(jp::benchmark::timestamp_from_file_name("image.jpg")),
               std::invalid_argument);
}

}  // namespace
