#include "jp/detection/rider_association.hpp"

#include <gtest/gtest.h>

#include <algorithm>

namespace {

using jp::detection::Detection;

Detection make(const std::string& twin_class, float x, float y, float w, float h, float score) {
  Detection d;
  d.twin_class = twin_class;
  d.box = cv::Rect2f(x, y, w, h);
  d.score = score;
  return d;
}

TEST(RiderAssociation, MergesPersonOnBicycle) {
  // Rider standing in the upper part of the bicycle box, feet near the pedals.
  const std::vector<Detection> detections = {make("pedestrian", 105, 40, 30, 80, 0.8F),
                                             make("bicycle", 95, 90, 50, 45, 0.6F),
                                             make("car", 300, 50, 100, 60, 0.9F)};
  const auto merged = jp::detection::associate_riders(detections);
  ASSERT_EQ(merged.size(), 2U);
  const auto& bike = merged[0];
  EXPECT_EQ(bike.twin_class, "bicycle");
  EXPECT_TRUE(bike.rider);
  EXPECT_FLOAT_EQ(bike.score, 0.8F);
  EXPECT_FLOAT_EQ(bike.box.y, 40.0F);                  // top of the rider
  EXPECT_FLOAT_EQ(bike.box.y + bike.box.height, 135);  // bottom of the wheels
  EXPECT_EQ(merged[1].twin_class, "car");
}

TEST(RiderAssociation, KeepsPedestrianNextToParkedBicycle) {
  // Person beside the bicycle: no horizontal containment, so both stay separate.
  const std::vector<Detection> detections = {make("pedestrian", 150, 30, 30, 90, 0.8F),
                                             make("bicycle", 95, 80, 50, 40, 0.6F)};
  const auto merged = jp::detection::associate_riders(detections);
  ASSERT_EQ(merged.size(), 2U);
  EXPECT_FALSE(merged[0].rider);
  EXPECT_FALSE(merged[1].rider);
}

TEST(RiderAssociation, PersonBehindVehicleIsNotARider) {
  // Feet above the two-wheeler: a person further away, not on it.
  const std::vector<Detection> detections = {make("pedestrian", 110, 0, 20, 60, 0.8F),
                                             make("motorcycle", 95, 50, 60, 60, 0.7F)};
  EXPECT_EQ(jp::detection::associate_riders(detections).size(), 2U);
}

TEST(RiderAssociation, OneRiderPerVehicle) {
  const std::vector<Detection> detections = {make("pedestrian", 100, 40, 30, 80, 0.8F),
                                             make("pedestrian", 104, 42, 30, 80, 0.7F),
                                             make("motorcycle", 90, 90, 60, 50, 0.6F)};
  const auto merged = jp::detection::associate_riders(detections);
  ASSERT_EQ(merged.size(), 2U);
  EXPECT_EQ(std::ranges::count_if(merged, [](const Detection& d) { return d.rider; }), 1);
  EXPECT_EQ(std::ranges::count_if(merged,
                                  [](const Detection& d) { return d.twin_class == "pedestrian"; }),
            1);
}

}  // namespace
