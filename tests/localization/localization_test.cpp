#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <numbers>

#include "jp/calibration/ground_calibration.hpp"
#include "jp/localization/bev_kalman.hpp"
#include "jp/localization/ground_projection.hpp"
#include "jp/localization/track_localizer.hpp"
#include "jp/localization/validated_region.hpp"

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
  const Eigen::Matrix2d cov = 0.04 * Eigen::Matrix2d::Identity();
  jp::localization::BevKalman filter(Eigen::Vector2d(0, 0), 0, cov, 1.0);
  for (int i = 1; i <= 50; ++i) {
    // 10 m/s along x, with irregular frame spacing.
    const std::int64_t t = (i * kFrameNs) + ((i % 3) * 20'000'000);
    filter.predict(t);
    filter.update(Eigen::Vector2d(10.0 * static_cast<double>(t) * 1e-9, 0.0), cov);
  }
  EXPECT_NEAR(filter.velocity().x(), 10.0, 0.3);
  EXPECT_NEAR(filter.velocity().y(), 0.0, 0.3);
  // Converged: the speed is known to well below 1 m/s, unlike a new track (10 m/s).
  EXPECT_LT(filter.speed_std(), 0.5);
  const jp::localization::BevKalman fresh(Eigen::Vector2d(0, 0), 0, cov, 1.0);
  EXPECT_NEAR(fresh.speed_std(), 10.0, 1e-9);
}

TEST(GroundProjection, JacobianMatchesFiniteDifferences) {
  // A perspective homography: ground resolution degrades towards the top of the image.
  const Eigen::Matrix3d h =
      (Eigen::Matrix3d() << 0.02, 0.001, -19.0, 0.0, -0.05, 60.0, 0.0, 0.001, 0.05).finished();
  const Eigen::Vector2d pixel(900.0, 500.0);
  const auto jacobian = jp::localization::ground_jacobian(pixel, h);
  ASSERT_TRUE(jacobian.has_value());
  const auto ground = [&](const Eigen::Vector2d& p) {
    return Eigen::Vector2d((h * p.homogeneous()).hnormalized());
  };
  for (int axis = 0; axis < 2; ++axis) {
    const Eigen::Vector2d step = 1e-3 * Eigen::Vector2d::Unit(axis);
    const Eigen::Vector2d numeric = (ground(pixel + step) - ground(pixel - step)) / 2e-3;
    EXPECT_LT((jacobian->col(axis) - numeric).norm(), 1e-6);
  }
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

TEST(TrackLocalizer, ReportsHeadingSourceAndPositionCovariance) {
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
  track.id = 3;
  track.twin_class = "car";
  const auto at = [&](int i, float y) {
    track.detection.box = cv::Rect2f(80, y - 20, 40, 20);
    return localizer.update({track}, i * kFrameNs).front();
  };
  // A new, standing track has no heading, so no source either.
  const auto first = at(0, 100.0F);
  EXPECT_FALSE(first.heading_rad.has_value());
  EXPECT_FALSE(first.heading_source.has_value());
  EXPECT_GT(first.position_cov.determinant(), 0.0);
  jp::localization::LocalizedObject object;
  for (int i = 1; i <= 20; ++i) {
    object = at(i, 100.0F + (5.0F * static_cast<float>(i)));
  }
  EXPECT_EQ(object.heading_source, "motion");
  for (int i = 21; i <= 60; ++i) {
    object = at(i, 200.0F);
  }
  EXPECT_EQ(object.heading_source, "held");
  ASSERT_TRUE(object.heading_rad.has_value());
}

TEST(TrackLocalizer, JitterOfAStandingCarDoesNotTurnIt) {
  const auto catalog = jp::core::ClassCatalog::from_yaml(R"yaml(
classes:
  car: {length_m: 4.5, width_m: 1.8, height_m: 1.5, vru: false}
sources: {}
)yaml");
  const Eigen::Matrix3d h = Eigen::Vector3d(0.1, 0.1, 1.0).asDiagonal();
  // Largest heading error while the car stands.
  const auto worst_error = [&](const jp::localization::LocalizerOptions& options) {
    jp::localization::TrackLocalizer localizer(h, Eigen::Vector2d(0, -50), std::nullopt, catalog,
                                               options);
    jp::tracking::TrackedObject track;
    track.id = 5;
    track.twin_class = "car";
    double worst = 0.0;
    for (int i = 0; i < 120; ++i) {
      // Drives along +y at 5 m/s for 2 s, then stands for 10 s with its box jumping 0.5 m
      // sideways every few frames (the box bottom of a far vehicle).
      const float y = 100.0F + (5.0F * static_cast<float>(std::min(i, 20)));
      const float x = (i > 20 && (i / 3) % 2 == 0) ? 85.0F : 80.0F;
      track.detection.box = cv::Rect2f(x, y - 20, 40, 20);
      const auto object = localizer.update({track}, i * kFrameNs).front();
      if (i > 20 && object.heading_rad) {
        worst =
            std::max(worst, std::abs(std::remainder(*object.heading_rad - (std::numbers::pi / 2),
                                                    2.0 * std::numbers::pi)));
      }
    }
    return worst;
  };
  jp::localization::LocalizerOptions options;
  options.contact_model = jp::localization::ContactModel::kNone;
  options.heading_min_speed_mps = 1.0;
  EXPECT_LT(worst_error(options), 0.2);
  // The raw motion direction above 0.5 m/s, as before the speed gate: the jitter turns the car.
  options.heading_min_speed_mps = 0.5;
  options.heading_speed_sigma = 0.0;
  options.heading_smoothing = 1.0;
  options.heading_max_turn_rate_dps = 0.0;
  EXPECT_GT(worst_error(options), 0.3);
}

TEST(TrackLocalizer, VehicleHeadingTurnsAtALimitedRate) {
  const auto catalog = jp::core::ClassCatalog::from_yaml(R"yaml(
classes:
  car: {length_m: 4.5, width_m: 1.8, height_m: 1.5, vru: false}
sources: {}
)yaml");
  const Eigen::Matrix3d h = Eigen::Vector3d(0.1, 0.1, 1.0).asDiagonal();
  jp::localization::LocalizerOptions options;
  options.contact_model = jp::localization::ContactModel::kNone;
  options.heading_smoothing = 1.0;
  options.heading_max_turn_rate_dps = 60.0;
  jp::localization::TrackLocalizer localizer(h, Eigen::Vector2d(0, -50), std::nullopt, catalog,
                                             options);
  jp::tracking::TrackedObject track;
  track.id = 9;
  track.twin_class = "car";
  constexpr int kTurn = 30;
  double heading = 0.0;
  for (int i = 0; i < 60; ++i) {
    // 8 m/s along +x for 3 s, then a sharp 90 degree turn to +y.
    const float x = 100.0F + (8.0F * static_cast<float>(std::min(i, kTurn)));
    const float y = 100.0F + (8.0F * static_cast<float>(std::max(i - kTurn, 0)));
    track.detection.box = cv::Rect2f(x - 20, y - 20, 40, 20);
    const auto object = localizer.update({track}, i * kFrameNs).front();
    if (!object.heading_rad) {
      continue;
    }
    heading = *object.heading_rad;
    if (i >= kTurn) {
      // At most 60 deg/s since the turn: 6 degrees per 0.1 s frame.
      EXPECT_LE(std::abs(heading), (6.0 * (i - kTurn) * std::numbers::pi / 180.0) + 1e-9)
          << "frame " << i;
    }
  }
  // It ends up following the new direction.
  EXPECT_NEAR(heading, std::numbers::pi / 2, 0.1);
}

TEST(TrackLocalizer, MeasurementNoiseGrowsAlongTheRay) {
  const auto catalog = jp::core::ClassCatalog::from_yaml(R"yaml(
classes:
  pedestrian: {length_m: 0.6, width_m: 0.6, height_m: 1.7, vru: true}
sources: {}
)yaml");
  // Camera 6 m high looking along +y with f = 1000 px: image row v sees ground y = 6000 / v.
  const Eigen::Matrix3d h =
      (Eigen::Matrix3d() << 6.0, 0.0, -5760.0, 0.0, 0.0, 6000.0, 0.0, 1.0, -540.0).finished();
  jp::localization::LocalizerOptions options;
  options.contact_model = jp::localization::ContactModel::kNone;
  options.measurement_floor_std_m = 0.0;
  jp::localization::TrackLocalizer localizer(h, Eigen::Vector2d(0, 0), std::nullopt, catalog,
                                             options);
  jp::tracking::TrackedObject track;
  track.twin_class = "pedestrian";
  // A pedestrian standing still at 10 m and one at 60 m, with the same 2 px box jitter.
  std::vector<double> spread;
  for (const float row : {1140.0F, 640.0F}) {
    track.id = static_cast<std::int64_t>(row);
    std::vector<double> ys;
    for (int i = 0; i < 40; ++i) {
      const float jitter = (i % 2 == 0) ? 2.0F : -2.0F;
      track.detection.box = cv::Rect2f(950, row - 40 + jitter, 20, 40);
      const auto objects = localizer.update({track}, i * kFrameNs);
      ys.push_back(objects.front().position.y());
    }
    spread.push_back(*std::ranges::max_element(ys) - *std::ranges::min_element(ys));
  }
  // The filter follows the 2 px noise at both ranges in proportion to the ground sampling:
  // much less positional jitter near the camera than far away.
  EXPECT_LT(spread[0], 0.2);
  EXPECT_GT(spread[1], 3.0 * spread[0]);
}

TEST(TrackLocalizer, RejectsSingleJumpsAndReanchorsPersistentOnes) {
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
  track.id = 3;
  track.twin_class = "car";
  const auto at = [&](float x, int frame) {
    track.detection.box = cv::Rect2f(x, 80, 40, 20);
    return localizer.update({track}, frame * kFrameNs).front();
  };
  int frame = 0;
  for (; frame < 30; ++frame) {
    static_cast<void>(at(100.0F, frame));  // parked at x = 12 m
  }
  // One frame 8 m away (e.g. a box briefly covering two objects): ignored.
  auto object = at(180.0F, frame++);
  EXPECT_NEAR(object.position.x(), 12.0, 0.1);
  EXPECT_LT(object.speed_mps, 0.5);
  object = at(100.0F, frame++);
  EXPECT_NEAR(object.position.x(), 12.0, 0.1);
  // The object stays at the new place: re-anchored there after three frames, speed unknown.
  for (int i = 0; i < 3; ++i) {
    object = at(180.0F, frame++);
  }
  EXPECT_NEAR(object.position.x(), 20.0, 0.1);
  EXPECT_GT(object.speed_std_mps, 5.0);
}

TEST(ValidatedRegion, OutlineMarginAndGroundResolution) {
  // Camera at the origin, 6 m high, looking along +y with f = 1000 px: one image row is
  // y^2 / 6000 m of ground, 0.2 m at y = 34.6 m.
  const Eigen::Matrix3d h =
      (Eigen::Matrix3d() << 6.0, 0.0, -5760.0, 0.0, 0.0, 6000.0, 0.0, 1.0, -540.0).finished();
  const std::vector<Eigen::Vector2d> points = {{-5, 10}, {5, 10}, {5, 60}, {-5, 60}, {0, 30}};
  const jp::localization::ValidatedRegion region(points, h, Eigen::Vector2d(0, 0),
                                                 {.margin_m = 10.0, .max_ground_m_per_px = 0.2});
  EXPECT_EQ(region.outline().size(), 4U);
  EXPECT_NEAR(region.ground_m_per_px({0, 30}), 30.0 * 30.0 / 6000.0, 1e-6);
  EXPECT_TRUE(region.contains({0, 30}));
  EXPECT_TRUE(region.contains({12, 30}));   // 7 m outside the outline, within the margin
  EXPECT_FALSE(region.contains({20, 30}));  // beyond the margin
  EXPECT_FALSE(region.contains({0, 50}));   // inside the outline, but 0.42 m per pixel
  EXPECT_FALSE(region.contains({0, -5}));   // behind the camera
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
