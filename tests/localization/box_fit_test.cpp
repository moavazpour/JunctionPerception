#include "jp/localization/box_fit.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <numbers>

#include "jp/calibration/ground_homography.hpp"

namespace {

using jp::localization::CameraModel;
using jp::localization::ObjectSize;

// Camera 8.5 m above the ground at the origin, looking along +x and pitched down by 25 degrees,
// square pixels and the principal point at the image centre.
CameraModel test_camera() {
  const Eigen::Matrix3d axes = (Eigen::Matrix3d() << 0, -1, 0, 0, 0, -1, 1, 0, 0).finished();
  const Eigen::Matrix3d rotation =
      Eigen::AngleAxisd(25.0 * std::numbers::pi / 180.0, Eigen::Vector3d::UnitX())
          .toRotationMatrix() *
      axes;
  const Eigen::Vector3d camera_position(0.0, 0.0, 8.5);
  Eigen::Matrix<double, 3, 4> extrinsics;
  extrinsics.leftCols<3>() = rotation;
  extrinsics.col(3) = -(rotation * camera_position);
  Eigen::Matrix3d k;
  k << 1300, 0, 960, 0, 1300, 600, 0, 0, 1;
  return {k * extrinsics, 1920, 1200};
}

TEST(BoxFit, RecoversCenterWithKnownHeading) {
  const CameraModel camera = test_camera();
  const ObjectSize car{4.5, 1.8, 1.5};
  const Eigen::Vector2d truth(30.0, -4.0);
  const double yaw = 0.6;
  const auto detection = jp::localization::project_object_box(camera, truth, yaw, car);
  ASSERT_TRUE(detection.has_value());
  const auto fitted = jp::localization::fit_footprint_center(camera, *detection, car, yaw,
                                                             truth + Eigen::Vector2d(2.0, 1.0));
  ASSERT_TRUE(fitted.has_value());
  EXPECT_LT((*fitted - truth).norm(), 0.01);
}

TEST(BoxFit, SearchesHeadingWhenUnknown) {
  const CameraModel camera = test_camera();
  const ObjectSize car{4.5, 1.8, 1.5};
  const Eigen::Vector2d truth(25.0, 6.0);
  const double yaw = std::numbers::pi / 2;  // crossing the view
  const auto detection = jp::localization::project_object_box(camera, truth, yaw, car);
  ASSERT_TRUE(detection.has_value());
  const auto fitted = jp::localization::fit_footprint_center(camera, *detection, car, std::nullopt,
                                                             truth - Eigen::Vector2d(2.0, 0.0));
  ASSERT_TRUE(fitted.has_value());
  EXPECT_LT((*fitted - truth).norm(), 0.05);
}

TEST(BoxFit, SkipsBoxesTouchingTheImageBorder) {
  const CameraModel camera = test_camera();
  const auto fitted = jp::localization::fit_footprint_center(camera, cv::Rect2f(500, 900, 300, 300),
                                                             {4.5, 1.8, 1.5}, 0.0, {12.0, 0.0});
  EXPECT_FALSE(fitted.has_value());
}

TEST(ProjectionFromHomography, RecoversCameraWithHeight) {
  const CameraModel camera = test_camera();
  Eigen::Matrix3d ground_to_image;
  ground_to_image << camera.projection.col(0), camera.projection.col(1), camera.projection.col(3);
  const auto recovered = jp::calibration::projection_from_homography(
      ground_to_image, camera.width_px, camera.height_px);
  ASSERT_TRUE(recovered.has_value());
  for (const Eigen::Vector3d& point :
       {Eigen::Vector3d(20, 3, 0), Eigen::Vector3d(35, -8, 1.5), Eigen::Vector3d(60, 10, 3.2)}) {
    const Eigen::Vector2d expected = (camera.projection * point.homogeneous()).hnormalized();
    const Eigen::Vector2d actual = (*recovered * point.homogeneous()).hnormalized();
    EXPECT_LT((actual - expected).norm(), 1e-6);
  }
}

}  // namespace
