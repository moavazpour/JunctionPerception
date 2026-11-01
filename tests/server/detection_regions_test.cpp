#include "jp/sources/detection_regions.hpp"

#include <gtest/gtest.h>

#include <stdexcept>

namespace {

using jp::detection::Detection;
using jp::sources::DetectionRegions;

// Image -> ground: 0.1 m per pixel, origin at pixel (0, 0).
const Eigen::Matrix3d kTenthMetre = Eigen::Vector3d(0.1, 0.1, 1.0).asDiagonal();

Detection box(float x, float y) {
  Detection d;
  d.box = cv::Rect2f(x - 20, y - 40, 40, 40);  // bottom centre at (x, y)
  return d;
}

TEST(DetectionRegions, AcceptsEverythingWithoutRegions) {
  EXPECT_TRUE(DetectionRegions{}.accepts(box(10, 10), std::nullopt));
}

TEST(DetectionRegions, KeepsDetectionsInsideTheAreas) {
  const auto regions = DetectionRegions::parse(R"json({
    "areas": [{"id": "near", "points": [[0, 500], [1000, 500], [1000, 1000], [0, 1000]]}]
  })json");
  EXPECT_TRUE(regions.accepts(box(300, 700), std::nullopt));
  EXPECT_FALSE(regions.accepts(box(300, 200), std::nullopt));
  // Keypoint models: the centre of the bottom corners decides, not the box.
  Detection corners = box(300, 200);
  for (const auto& p : {cv::Point2f(290, 690), cv::Point2f(310, 690), cv::Point2f(310, 710),
                        cv::Point2f(290, 710)}) {
    corners.keypoints.push_back({p, 0.9F});
  }
  EXPECT_TRUE(regions.accepts(corners, std::nullopt));
}

TEST(DetectionRegions, BandedLinesAcceptOnlyNearbyRoadUsers) {
  // A line along x at pixel row 600 (ground y = 60 m) with a 3 m band, and a counting line.
  const auto regions = DetectionRegions::parse(R"json({
    "lines": [{"id": "stop", "points": [[0, 600], [1000, 600]], "band_m": 3},
              {"id": "count", "points": [[0, 100], [1000, 100]]}]
  })json");
  EXPECT_TRUE(regions.accepts(box(500, 620), kTenthMetre));   // 2 m away
  EXPECT_FALSE(regions.accepts(box(500, 650), kTenthMetre));  // 5 m away
  EXPECT_FALSE(regions.accepts(box(500, 100), kTenthMetre));  // on the counting line only
  // Without a calibration the band cannot be measured: nothing is dropped for it.
  EXPECT_TRUE(regions.accepts(box(500, 650), std::nullopt));
}

TEST(DetectionRegions, KeepsSignalsWithTheirPlacement) {
  const auto regions = DetectionRegions::parse(R"json({
    "signals": [{"id": "s1", "name": "North", "points": [[10, 20], [30, 80]],
                 "position": [12.5, -3.25], "heading_deg": 90, "height_m": 5},
                {"id": "s2", "points": [[0, 0], [5, 0], [5, 9], [0, 9]]}]
  })json");
  ASSERT_EQ(regions.signals.size(), 2U);
  // Signals do not limit the detections.
  EXPECT_TRUE(regions.empty());
  // Two points are the corners of a rectangle.
  EXPECT_EQ(regions.signals[0].polygon().size(), 4U);
  const auto again = DetectionRegions::parse(regions.to_json());
  ASSERT_TRUE(again.signals[0].position.has_value());
  EXPECT_DOUBLE_EQ(again.signals[0].position->y(), -3.25);
  EXPECT_DOUBLE_EQ(again.signals[0].heading_deg, 90.0);
  EXPECT_DOUBLE_EQ(again.signals[0].height_m, 5.0);
  EXPECT_FALSE(again.signals[1].position.has_value());
  EXPECT_DOUBLE_EQ(again.signals[1].height_m, 4.5);
}

TEST(DetectionRegions, RoundTripsAndRejectsMalformedRegions) {
  const auto regions = DetectionRegions::parse(R"json({
    "areas": [{"id": "a", "name": "North", "points": [[0, 0], [10, 0], [10, 10]]}],
    "lines": [{"id": "l", "points": [[0, 5], [10, 5]], "band_m": 2.5, "speed_limit_kmh": 50}]
  })json");
  const auto again = DetectionRegions::parse(regions.to_json());
  ASSERT_EQ(again.areas.size(), 1U);
  EXPECT_EQ(again.areas[0].name, "North");
  ASSERT_EQ(again.lines.size(), 1U);
  EXPECT_DOUBLE_EQ(again.lines[0].band_m, 2.5);
  EXPECT_DOUBLE_EQ(again.lines[0].speed_limit_kmh, 50.0);
  EXPECT_THROW(static_cast<void>(DetectionRegions::parse(
                   R"json({"areas": [{"id": "a", "points": [[0, 0], [1, 1]]}]})json")),
               std::runtime_error);
  EXPECT_THROW(static_cast<void>(DetectionRegions::parse("not json")), std::runtime_error);
}

}  // namespace
