#include "jp/calibration/control_points.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <format>
#include <numbers>
#include <stdexcept>
#include <string>

#include "jp/calibration/ground_calibration.hpp"
#include "jp/calibration/ground_homography.hpp"

namespace {

using jp::calibration::ControlPoint;

// Camera 9 m above (5, -20) looking north and down, square pixels, centred principal point.
Eigen::Matrix<double, 3, 4> test_projection() {
  const Eigen::Matrix3d axes = (Eigen::Matrix3d() << 1, 0, 0, 0, 0, -1, 0, 1, 0).finished();
  const Eigen::Matrix3d rotation =
      Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitX()).toRotationMatrix() * axes;
  const Eigen::Vector3d position(5.0, -20.0, 9.0);
  Eigen::Matrix<double, 3, 4> extrinsics;
  extrinsics.leftCols<3>() = rotation;
  extrinsics.col(3) = -(rotation * position);
  Eigen::Matrix3d k;
  k << 1400, 0, 960, 0, 1400, 540, 0, 0, 1;
  return k * extrinsics;
}

std::vector<ControlPoint> points_on_ground(const Eigen::Matrix<double, 3, 4>& p) {
  std::vector<ControlPoint> points;
  int i = 0;
  for (double x = -10; x <= 20; x += 7.5) {
    for (double y = 5; y <= 45; y += 10) {
      const Eigen::Vector2d pixel = (p * Eigen::Vector4d(x, y, 0, 1)).hnormalized();
      points.push_back({"p" + std::to_string(i), pixel, {x, y}, i % 4 == 3});
      ++i;
    }
  }
  return points;
}

TEST(ControlPointFit, RecoversHomographyAndCamera) {
  const auto projection = test_projection();
  const auto points = points_on_ground(projection);
  const auto fit = jp::calibration::fit_control_points(points, 1920, 1080);
  EXPECT_EQ(fit.inliers, fit.fit_points);
  EXPECT_LT(fit.fit_rms_m, 1e-6);
  ASSERT_TRUE(fit.check_max_m.has_value());
  EXPECT_LT(*fit.check_max_m, 1e-6);
  ASSERT_TRUE(fit.camera_position.has_value());
  EXPECT_LT((*fit.camera_position - Eigen::Vector3d(5.0, -20.0, 9.0)).norm(), 1e-3);
}

TEST(ControlPointFit, FlagsAnOutlierAndReportsItsError) {
  auto points = points_on_ground(test_projection());
  points[1].world += Eigen::Vector2d(4.0, 0.0);  // a mis-clicked fit point
  const auto fit = jp::calibration::fit_control_points(points, 1920, 1080);
  EXPECT_FALSE(fit.residuals[1].inlier);
  EXPECT_NEAR(fit.residuals[1].error_m, 4.0, 0.01);
  EXPECT_LT(fit.fit_rms_m, 1e-6);
  // The validated region is built from the other points only.
  const auto agreeing = jp::calibration::agreeing_points(points, fit);
  EXPECT_EQ(agreeing.size(), points.size() - 1);
  EXPECT_EQ(std::ranges::count(agreeing, points[1].world), 0);
}

TEST(ControlPointFit, NeedsFourFitPoints) {
  auto points = points_on_ground(test_projection());
  points.resize(4);
  points[3].check = true;
  EXPECT_THROW(static_cast<void>(jp::calibration::fit_control_points(points, 1920, 1080)),
               std::invalid_argument);
}

TEST(ControlPointFit, ProjectionFromKnownCameraCentreMatchesTheCamera) {
  const auto projection = test_projection();
  const auto fit = jp::calibration::fit_control_points(points_on_ground(projection), 1920, 1080);
  const auto calibration = jp::calibration::calibration_from_fit(fit, "test", 1920, 1080,
                                                                 Eigen::Vector2d(5.0, -20.0), 9.0);
  ASSERT_TRUE(calibration.ground_projection.has_value());
  EXPECT_DOUBLE_EQ(calibration.camera_height_m, 9.0);
  EXPECT_LT((jp::calibration::camera_center(*calibration.ground_projection) -
             Eigen::Vector3d(5.0, -20.0, 9.0))
                .norm(),
            1e-6);
  // Points above the ground (e.g. the top of a vehicle) land where the true camera sees them.
  for (const Eigen::Vector4d& point :
       {Eigen::Vector4d(0, 20, 1.5, 1), Eigen::Vector4d(12, 35, 3, 1)}) {
    const Eigen::Vector2d expected = (projection * point).hnormalized();
    const Eigen::Vector3d image = *calibration.ground_projection * point;
    // In metres: the third coordinate is the depth, which box fit gates on.
    EXPECT_NEAR(image.z(), (projection * point).z(), 1e-4);
    EXPECT_LT((image.hnormalized() - expected).norm(), 1e-3);
  }
}

TEST(GroundCalibration, StoredProjectionIsScaledToMetres) {
  // A site calibration written with the homography's scale (h33 = 1): loaded, its depths are in
  // metres again.
  const auto projection = test_projection();
  Eigen::Matrix3d ground_to_image;
  ground_to_image << projection.col(0), projection.col(1), projection.col(3);
  const Eigen::Matrix3d image_to_ground = ground_to_image.inverse();
  // Divided by a negative entry: the stored sign is also the wrong one.
  const Eigen::Matrix<double, 3, 4> stored = projection / -std::abs(projection(2, 3));
  const auto rows = [](const auto& m) {
    std::string text;
    for (Eigen::Index r = 0; r < m.rows(); ++r) {
      text += "  - [";
      for (Eigen::Index c = 0; c < m.cols(); ++c) {
        text += std::format("{:.17g}{}", m(r, c), c + 1 < m.cols() ? ", " : "]\n");
      }
    }
    return text;
  };
  std::string yaml = "camera: test\nimage_size_px: [1920, 1080]\nimage_to_ground:\n" +
                     rows(image_to_ground) +
                     "camera_ground_position_m: [5, -20]\ncamera_height_m: 9\n"
                     "ground_projection:\n" +
                     rows(stored);
  yaml += "ground_projection_estimated: true\n";
  const auto calibration = jp::calibration::parse_ground_calibration(yaml);
  ASSERT_TRUE(calibration.ground_projection.has_value());
  const Eigen::Vector4d point(0, 20, 1.5, 1);
  EXPECT_NEAR((*calibration.ground_projection * point).z(), (projection * point).z(), 1e-3);
}

TEST(ControlPoints, FilesRoundTrip) {
  const jp::core::GeoAnchor anchor({48.1760, 11.5556, 512.0});
  const auto projection = test_projection();
  const auto points = points_on_ground(projection);
  const auto dir = std::filesystem::temp_directory_path() / "jp_control_points_test";
  jp::calibration::write_control_points(dir / "control_points.yaml", points, anchor);
  const auto loaded = jp::calibration::load_control_points(dir / "control_points.yaml", anchor);
  ASSERT_EQ(loaded.size(), points.size());
  EXPECT_LT((loaded[3].world - points[3].world).norm(), 1e-3);
  EXPECT_TRUE(loaded[3].check);

  const auto fit = jp::calibration::fit_control_points(points, 1920, 1080);
  const auto calibration = jp::calibration::calibration_from_fit(fit, "test", 1920, 1080);
  jp::calibration::write_ground_calibration(dir / "ground_homography.yaml", calibration, fit);
  const auto read = jp::calibration::load_ground_calibration(dir / "ground_homography.yaml");
  EXPECT_TRUE(read.image_to_ground.isApprox(calibration.image_to_ground, 1e-9));
  EXPECT_TRUE(read.ground_projection_estimated);
  EXPECT_NEAR(read.camera_height_m, 9.0, 1e-3);
  std::filesystem::remove_all(dir);
}

}  // namespace
