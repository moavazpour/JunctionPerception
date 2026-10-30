#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

#include "jp/calibration/ground_calibration.hpp"
#include "jp/localization/bev_kalman.hpp"
#include "jp/localization/ground_projection.hpp"
#include "jp/localization/track_localizer.hpp"

namespace {

using jp::localization::Footprint;

constexpr std::int64_t kFrameNs = 100'000'000;  // 10 Hz

TEST(GroundProjection, ContactIsBoxBottomCentreThroughHomography) {
  // Image -> ground: 0.1 m per pixel, origin at pixel (0, 0).
  const Eigen::Matrix3d h = Eigen::Vector3d(0.1, 0.1, 1.0).asDiagonal();
  const auto contact = jp::localization::ground_contact(cv::Rect2f(100, 50, 40, 30), h);
  ASSERT_TRUE(contact.has_value());
  EXPECT_TRUE(contact->isApprox(Eigen::Vector2d(12.0, 8.0)));
}

TEST(GroundProjection, BiasCorrectionShiftsAwayFromCamera) {
  const Footprint car{4.5, 1.8};
  const Eigen::Vector2d camera(0.0, 0.0);
  const Eigen::Vector2d contact(20.0, 0.0);
  // No heading: half the length along the camera ray.
  EXPECT_TRUE(jp::localization::correct_contact_bias(contact, camera, car, std::nullopt)
                  .isApprox(Eigen::Vector2d(22.25, 0.0)));
  // Driving across the ray: half the width.
  EXPECT_TRUE(jp::localization::correct_contact_bias(contact, camera, car, std::numbers::pi / 2)
                  .isApprox(Eigen::Vector2d(20.9, 0.0)));
  // Driving along the ray (towards or away): half the length.
  EXPECT_NEAR(
      jp::localization::contact_to_centre_distance(car, Eigen::Vector2d(1, 0), std::numbers::pi),
      2.25, 1e-12);
  // 45 degrees: 0.5 * (L + W) / sqrt(2).
  EXPECT_NEAR(jp::localization::contact_to_centre_distance(car, Eigen::Vector2d(0, 1),
                                                           std::numbers::pi / 4),
              0.5 * (4.5 + 1.8) / std::numbers::sqrt2, 1e-12);
}

TEST(BevKalman, EstimatesConstantVelocity) {
  jp::localization::BevKalman filter(Eigen::Vector2d(0, 0), 0, 0.2, 1.0);
  for (int i = 1; i <= 50; ++i) {
    // 10 m/s along x, with irregular frame spacing.
    const std::int64_t t = (i * kFrameNs) + ((i % 3) * 20'000'000);
    filter.predict(t);
    filter.update(Eigen::Vector2d(10.0 * static_cast<double>(t) * 1e-9, 0.0), 0.2);
  }
  EXPECT_NEAR(filter.velocity().x(), 10.0, 0.3);
  EXPECT_NEAR(filter.velocity().y(), 0.0, 0.3);
}

TEST(TrackLocalizer, HoldsHeadingWhenStopped) {
  const auto catalog = jp::core::ClassCatalog::from_yaml(R"yaml(
classes:
  car: {length_m: 4.5, width_m: 1.8, height_m: 1.5, vru: false}
sources: {}
)yaml");
  const Eigen::Matrix3d h = Eigen::Vector3d(0.1, 0.1, 1.0).asDiagonal();
  jp::localization::LocalizerOptions options;
  options.contact_model = jp::localization::ContactModel::kNone;
  jp::localization::TrackLocalizer localizer(h, Eigen::Vector2d(0, -50), std::nullopt, catalog,
                                             options);

  jp::tracking::TrackedObject track;
  track.id = 7;
  track.twin_class = "car";
  std::vector<jp::localization::LocalizedObject> objects;
  for (int i = 0; i < 60; ++i) {
    // Moves along +y (pixel rows) at 5 m/s for 2 s, then stands still for 4 s.
    const float y = 100.0F + (5.0F * static_cast<float>(std::min(i, 20)));
    track.detection.box = cv::Rect2f(80, y - 20, 40, 20);
    objects = localizer.update({track}, i * kFrameNs);
  }
  ASSERT_EQ(objects.size(), 1U);
  EXPECT_LT(objects.front().speed_mps, 0.5);
  ASSERT_TRUE(objects.front().heading_rad.has_value());
  EXPECT_NEAR(*objects.front().heading_rad, std::numbers::pi / 2, 0.2);
}

TEST(GroundCalibration, LoadsRepositoryScene) {
  const auto calibration = jp::calibration::load_ground_calibration(
      JP_SOURCE_DIR "/scenes/tumtraf_s110_south1/ground_homography.yaml");
  EXPECT_EQ(calibration.camera, "s110_camera_basler_south1_8mm");
  EXPECT_EQ(calibration.width_px, 1920);
  EXPECT_GT(calibration.camera_height_m, 5.0);
  ASSERT_TRUE(calibration.sensor_from_ground.has_value());
  // The camera's own nadir maps near the camera ground position only at infinity, but the image
  // centre must land in front of the camera on the ground.
  const Eigen::Vector3d center = calibration.image_to_ground * Eigen::Vector3d(960.0, 600.0, 1.0);
  EXPECT_GT((center.hnormalized() - calibration.camera_ground_position_m).norm(), 10.0);
}

}  // namespace
