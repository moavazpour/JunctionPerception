#include <gtest/gtest.h>

#include <random>

#include "jp/calibration/ground_homography.hpp"
#include "jp/calibration/ground_plane.hpp"

namespace {

using jp::calibration::Plane;

TEST(GroundPlane, RobustFitIgnoresOutliers) {
  // Ground tilted slightly: z = 0.01 x - 0.02 y - 7.3, plus noise and elevated outliers.
  std::mt19937 rng(3);
  std::uniform_real_distribution<double> coord(-60.0, 60.0);
  std::normal_distribution<double> noise(0.0, 0.03);
  std::vector<Eigen::Vector3d> points;
  for (int i = 0; i < 400; ++i) {
    const double x = coord(rng);
    const double y = coord(rng);
    points.emplace_back(x, y, (0.01 * x) - (0.02 * y) - 7.3 + noise(rng));
  }
  for (int i = 0; i < 60; ++i) {
    points.emplace_back(coord(rng), coord(rng), -4.0);
  }
  const auto fit = jp::calibration::fit_plane(points);
  const Eigen::Vector3d expected_normal = Eigen::Vector3d(-0.01, 0.02, 1.0).normalized();
  EXPECT_GT(fit.plane.normal.dot(expected_normal), 0.99999);
  EXPECT_NEAR(fit.plane.signed_distance(Eigen::Vector3d(0, 0, -7.3)), 0.0, 0.02);
  EXPECT_GE(fit.inliers, 390U);
  EXPECT_LT(fit.inlier_rms_m, 0.05);
}

TEST(GroundPlane, ProjectLandsOnPlane) {
  const Plane plane{Eigen::Vector3d(0.0, 0.6, 0.8), 2.0};
  const Eigen::Vector3d projected = plane.project(Eigen::Vector3d(3.0, -1.0, 5.0));
  EXPECT_NEAR(plane.signed_distance(projected), 0.0, 1e-12);
}

TEST(GroundHomography, RoundTripMatchesFullProjection) {
  // Camera 8 m above a tilted ground, looking forward and down.
  const Plane ground{Eigen::Vector3d(0.02, -0.01, 1.0).normalized(), 7.5};
  const Eigen::Isometry3d sensor_from_ground =
      jp::calibration::sensor_from_ground_frame(ground, Eigen::Vector3d::Zero());

  Eigen::Isometry3d camera_from_sensor = Eigen::Isometry3d::Identity();
  // Camera axes: x right, y down, z forward (sensor x forward, z up), pitched down by 20 deg.
  const Eigen::Matrix3d axes = (Eigen::Matrix3d() << 0, -1, 0, 0, 0, -1, 1, 0, 0).finished();
  camera_from_sensor.linear() =
      Eigen::AngleAxisd(0.35, Eigen::Vector3d::UnitX()).toRotationMatrix() * axes;
  Eigen::Matrix3d k;
  k << 1300, 0, 960, 0, 1320, 600, 0, 0, 1;
  const Eigen::Matrix<double, 3, 4> projection = k * camera_from_sensor.matrix().topRows<3>();

  const Eigen::Matrix3d h =
      jp::calibration::ground_to_image_homography(projection, sensor_from_ground);
  const Eigen::Matrix3d h_inv = jp::calibration::normalized_homography(h.inverse());

  for (const auto& ground_xy :
       {Eigen::Vector2d(20, 3), Eigen::Vector2d(45, -12), Eigen::Vector2d(80, 25)}) {
    const Eigen::Vector3d sensor_point =
        sensor_from_ground * Eigen::Vector3d(ground_xy.x(), ground_xy.y(), 0.0);
    EXPECT_NEAR(ground.signed_distance(sensor_point), 0.0, 1e-9);
    const Eigen::Vector2d expected_pixel = (projection * sensor_point.homogeneous()).hnormalized();
    const auto pixel = jp::calibration::apply_homography(h, ground_xy);
    ASSERT_TRUE(pixel.has_value());
    EXPECT_TRUE(pixel->isApprox(expected_pixel, 1e-9));
    const auto back = jp::calibration::apply_homography(h_inv, *pixel);
    ASSERT_TRUE(back.has_value());
    EXPECT_TRUE(back->isApprox(ground_xy, 1e-9));
  }
}

}  // namespace
