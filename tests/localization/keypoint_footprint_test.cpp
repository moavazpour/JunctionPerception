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

// Corners of a box footprint as keypoints (pixels of kTenthMetre), in a shuffled order.
std::vector<Keypoint> corners(const Eigen::Vector2d& center, double yaw, double length,
                              double width, float score = 0.9F) {
  const Eigen::Rotation2Dd r(yaw);
  std::vector<Keypoint> keypoints;
  for (const auto& [sx, sy] :
       {std::pair{1, 1}, std::pair{-1, -1}, std::pair{1, -1}, std::pair{-1, 1}}) {
    const Eigen::Vector2d g = center + (r * Eigen::Vector2d(sx * length / 2, sy * width / 2));
    keypoints.push_back(
        {cv::Point2f(static_cast<float>(g.x() * 10), static_cast<float>(g.y() * 10)), score});
  }
  return keypoints;
}

TEST(KeypointFootprint, RecoversCentreAxisAndSize) {
  const ObjectSize car{4.5, 1.8, 1.5};
  const auto footprint =
      jp::localization::keypoint_footprint(corners({20.0, 8.0}, 0.5, 4.6, 1.9), kTenthMetre, car);
  ASSERT_TRUE(footprint.has_value());
  EXPECT_TRUE(footprint->center.isApprox(Eigen::Vector2d(20.0, 8.0), 1e-4));
  EXPECT_NEAR(footprint->length_m, 4.6, 1e-3);
  EXPECT_NEAR(footprint->width_m, 1.9, 1e-3);
  // The axis is known up to a half turn.
  EXPECT_NEAR(std::abs(std::sin(footprint->axis_rad - 0.5)), 0.0, 1e-4);
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
  // Standing from the start: the corners give the size, but not which end is the front.
  for (int i = 0; i < 5; ++i) {
    step(i, 0.0);
  }
  ASSERT_EQ(objects.size(), 1U);
  EXPECT_TRUE(objects.front().from_keypoints);
  EXPECT_FALSE(objects.front().heading_rad.has_value());
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

}  // namespace
