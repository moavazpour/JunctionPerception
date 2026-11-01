#include "jp/localization/keypoint_footprint.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <cmath>
#include <numbers>

#include "jp/localization/track_localizer.hpp"

namespace {

using jp::detection::Keypoint;
using jp::localization::ObjectSize;

constexpr std::int64_t kFrameNs = 100'000'000;  // 10 Hz

// Image -> ground: 0.1 m per pixel, origin at pixel (0, 0).
const Eigen::Matrix3d kTenthMetre = Eigen::Vector3d(0.1, 0.1, 1.0).asDiagonal();

// Corners of a box footprint facing `yaw` as keypoints (pixels of kTenthMetre), in the model's
// order: front left, front right, rear right, rear left.
std::vector<Keypoint> corners(const Eigen::Vector2d& center, double yaw, double length,
                              double width, float score = 0.9F) {
  const Eigen::Rotation2Dd r(yaw);
  std::vector<Keypoint> keypoints;
  for (const auto& [sx, sy] :
       {std::pair{1, 1}, std::pair{1, -1}, std::pair{-1, -1}, std::pair{-1, 1}}) {
    const Eigen::Vector2d g = center + (r * Eigen::Vector2d(sx * length / 2, sy * width / 2));
    keypoints.push_back(
        {cv::Point2f(static_cast<float>(g.x() * 10), static_cast<float>(g.y() * 10)), score});
  }
  return keypoints;
}

TEST(KeypointFootprint, RecoversCentreHeadingAndSize) {
  const ObjectSize car{4.5, 1.8, 1.5};
  for (const double yaw : {0.5, 0.5 + std::numbers::pi, -2.0}) {
    const auto footprint =
        jp::localization::keypoint_footprint(corners({20.0, 8.0}, yaw, 4.6, 1.9), kTenthMetre, car);
    ASSERT_TRUE(footprint.has_value());
    EXPECT_TRUE(footprint->center.isApprox(Eigen::Vector2d(20.0, 8.0), 1e-4));
    EXPECT_NEAR(footprint->length_m, 4.6, 1e-3);
    EXPECT_NEAR(footprint->width_m, 1.9, 1e-3);
    // The front corners give the heading with its direction.
    EXPECT_NEAR(std::remainder(footprint->heading_rad - yaw, 2.0 * std::numbers::pi), 0.0, 1e-4);
  }
}

TEST(KeypointFootprint, RejectsHiddenCornersAndImplausibleShapes) {
  const ObjectSize car{4.5, 1.8, 1.5};
  EXPECT_FALSE(
      jp::localization::keypoint_footprint(corners({20, 8}, 0, 4.5, 1.8, 0.3F), kTenthMetre, car)
          .has_value());
  // A "car" 20 m long: a corner on another object.
  EXPECT_FALSE(jp::localization::keypoint_footprint(corners({20, 8}, 0, 20, 1.8), kTenthMetre, car)
                   .has_value());
  EXPECT_FALSE(jp::localization::keypoint_footprint({}, kTenthMetre, car).has_value());
}

TEST(KeypointFootprint, OrientsTheAxisTowardsAReference) {
  EXPECT_NEAR(jp::localization::orient_axis(0.2, 0.0), 0.2, 1e-12);
  EXPECT_NEAR(jp::localization::orient_axis(0.2, std::numbers::pi), 0.2 - std::numbers::pi, 1e-12);
}

TEST(TrackLocalizer, KeypointsKeepHeadingAndSizeOfStoppedVehicles) {
  const auto catalog = jp::core::ClassCatalog::from_yaml(R"yaml(
classes:
  car: {length_m: 4.5, width_m: 1.8, height_m: 1.5, vru: false}
sources: {}
)yaml");
  jp::localization::LocalizerOptions options;
  options.keypoint_position = true;
  jp::localization::TrackLocalizer localizer(kTenthMetre, Eigen::Vector2d(0, -50), std::nullopt,
                                             catalog, options);
  jp::tracking::TrackedObject track;
  track.id = 3;
  track.twin_class = "car";
  track.detection.box = cv::Rect2f(150, 50, 100, 60);
  const double yaw = 1.0;
  const Eigen::Vector2d start(20.0, 8.0);
  const Eigen::Vector2d direction(std::cos(yaw), std::sin(yaw));
  std::vector<jp::localization::LocalizedObject> objects;
  const auto step = [&](int i, double travelled) {
    track.detection.keypoints = corners(start + (travelled * direction), yaw, 4.2, 1.7);
    objects = localizer.update({track}, i * kFrameNs);
  };
  // Standing from the start: the corners give the size and the heading.
  for (int i = 0; i < 5; ++i) {
    step(i, 0.0);
  }
  ASSERT_EQ(objects.size(), 1U);
  EXPECT_TRUE(objects.front().from_keypoints);
  ASSERT_TRUE(objects.front().heading_rad.has_value());
  EXPECT_NEAR(*objects.front().heading_rad, yaw, 1e-6);
  // Drives 2 s at 5 m/s along its axis, then stands for 3 s (e.g. at a red light).
  for (int i = 5; i < 25; ++i) {
    step(i, 0.5 * (i - 4));
  }
  for (int i = 25; i < 55; ++i) {
    step(i, 10.0);
  }
  ASSERT_EQ(objects.size(), 1U);
  const auto& car = objects.front();
  EXPECT_LT(car.speed_mps, 0.5);
  EXPECT_TRUE(car.position.isApprox(start + (10.0 * direction), 0.05));
  ASSERT_TRUE(car.heading_rad.has_value());
  EXPECT_NEAR(*car.heading_rad, yaw, 1e-2);
  ASSERT_TRUE(car.length_m.has_value());
  EXPECT_NEAR(*car.length_m, 4.2, 1e-3);
  EXPECT_NEAR(*car.width_m, 1.7, 1e-3);
}

TEST(TrackLocalizer, OverridesMomentaryReversedCorners) {
  const auto catalog = jp::core::ClassCatalog::from_yaml(R"yaml(
classes:
  car: {length_m: 4.5, width_m: 1.8, height_m: 1.5, vru: false}
sources: {}
)yaml");
  jp::localization::LocalizerOptions options;
  options.keypoint_position = true;
  options.keypoint_flip_frames = 5;
  jp::localization::TrackLocalizer localizer(kTenthMetre, Eigen::Vector2d(0, -50), std::nullopt,
                                             catalog, options);
  jp::tracking::TrackedObject track;
  track.id = 4;
  track.twin_class = "car";
  track.detection.box = cv::Rect2f(150, 50, 100, 60);
  const double yaw = 0.3;
  const Eigen::Vector2d direction(std::cos(yaw), std::sin(yaw));
  const auto heading = [&](int i, const Eigen::Vector2d& at, double corner_yaw) {
    track.detection.keypoints = corners(at, corner_yaw, 4.5, 1.8);
    const auto objects = localizer.update({track}, i * kFrameNs);
    return objects.front().heading_rad.value_or(99.0);
  };
  const auto off = [&](double h) {
    return std::abs(std::remainder(h - yaw, 2 * std::numbers::pi));
  };
  // Driving at 8 m/s: corners reversed for a few frames do not turn it.
  int i = 0;
  Eigen::Vector2d at(20.0, 8.0);
  for (; i < 30; ++i, at += 0.8 * direction) {
    EXPECT_LT(off(heading(i, at, i >= 20 && i < 24 ? yaw + std::numbers::pi : yaw)), 0.2) << i;
  }
  // Standing: two reversed frames are ignored, a lasting reversal turns it.
  for (int n = 0; n < 2; ++n, ++i) {
    EXPECT_LT(off(heading(i, at, yaw + std::numbers::pi)), 0.2);
  }
  for (int n = 0; n < 20; ++n, ++i) {
    heading(i, at, yaw);
  }
  double last = 0.0;
  for (int n = 0; n < 8; ++n, ++i) {
    last = heading(i, at, yaw + std::numbers::pi);
  }
  EXPECT_GT(off(last), std::numbers::pi - 0.2);
}

}  // namespace
