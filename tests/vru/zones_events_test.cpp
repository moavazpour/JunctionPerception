#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <stdexcept>

#include "jp/vru/event_detector.hpp"
#include "jp/vru/zones.hpp"

namespace {

using jp::core::WorldObject;

constexpr std::string_view kZones = R"json({
  "type": "FeatureCollection", "coordinate_frame": "scene_ground",
  "features": [{"type": "Feature", "properties": {"id": "crosswalk_n", "type": "crosswalk"},
                "geometry": {"type": "Polygon",
                             "coordinates": [[[0, 0], [10, 0], [10, 4], [0, 4], [0, 0]]]}}]
})json";

constexpr std::int64_t kFrameNs = 100'000'000;

WorldObject object(std::int64_t id, bool vru, double x, double y, double vx = 0.0,
                   double vy = 0.0) {
  WorldObject o;
  o.id = id;
  o.object_class = vru ? "pedestrian" : "car";
  o.vru = vru;
  o.x = x;
  o.y = y;
  o.vx = vx;
  o.vy = vy;
  o.speed_mps = std::hypot(vx, vy);
  if (o.speed_mps > 0.5) {
    o.heading_rad = std::atan2(vy, vx);
  }
  return o;
}

TEST(Zones, ParsesSceneGroundPolygons) {
  const auto zones = jp::vru::parse_zones(kZones);
  ASSERT_EQ(zones.size(), 1U);
  EXPECT_EQ(zones[0].polygon.size(), 4U);
  EXPECT_TRUE(zones[0].contains({5.0, 2.0}));
  EXPECT_FALSE(zones[0].contains({11.0, 2.0}));
  EXPECT_FALSE(zones[0].contains({5.0, -0.5}));
}

TEST(Zones, RejectsWgs84WithoutGeoAnchor) {
  constexpr std::string_view wgs84 = R"json({"type": "FeatureCollection", "features": []})json";
  EXPECT_THROW(static_cast<void>(jp::vru::parse_zones(wgs84)), std::runtime_error);
}

TEST(Conflict, PredictsClosestApproach) {
  // Car heading east at 10 m/s, pedestrian 20 m ahead crossing north at 1 m/s from y = -2.
  const auto approach = jp::vru::predict_closest_approach(object(1, false, 0, 0, 10, 0),
                                                          object(2, true, 20, -2, 0, 1), 3.0, 0.1);
  EXPECT_NEAR(approach.time_s, 2.0, 0.11);
  EXPECT_LT(approach.distance_m, 0.5);
}

TEST(EventDetector, CrosswalkEventIsDebounced) {
  jp::vru::EventDetector detector(jp::vru::parse_zones(kZones));
  std::int64_t t = 0;
  const auto step = [&](std::vector<WorldObject> objects) {
    t += kFrameNs;
    return detector.update(objects, t);
  };
  EXPECT_TRUE(step({object(5, true, 5, 2)}).empty());
  EXPECT_TRUE(step({object(5, true, 5, 2)}).empty());
  auto events = step({object(5, true, 5, 2)});
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].type, jp::vru::kVruOnCrosswalk);
  EXPECT_EQ(events[0].state, "active");
  EXPECT_EQ(events[0].zone, "crosswalk_n");
  const auto id = events[0].id;
  // Leaves the crosswalk: quiet while debouncing, then reported once as ended.
  EXPECT_TRUE(step({object(5, true, 5, 8)}).empty());
  EXPECT_TRUE(step({object(5, true, 5, 8)}).empty());
  events = step({object(5, true, 5, 8)});
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].id, id);
  EXPECT_EQ(events[0].state, "ended");
  EXPECT_TRUE(step({object(5, true, 5, 8)}).empty());
}

TEST(EventDetector, ConflictBetweenVruAndVehicle) {
  jp::vru::EventDetector detector({});
  std::vector<jp::core::WorldEvent> events;
  for (int i = 1; i <= 3; ++i) {
    std::vector<WorldObject> objects = {object(1, false, 0, 0, 10, 0),
                                        object(2, true, 20, -2, 0, 1), object(3, false, 0, 50)};
    events = detector.update(objects, i * kFrameNs);
  }
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].type, jp::vru::kVruVehicleConflict);
  EXPECT_EQ(events[0].object_ids, (std::vector<std::int64_t>{1, 2}));
  ASSERT_TRUE(events[0].time_to_closest_s.has_value());
  EXPECT_NEAR(*events[0].time_to_closest_s, 2.0, 0.11);
}

TEST(EventDetector, FlagsTurningVehicles) {
  jp::vru::EventDetector detector({});
  std::vector<WorldObject> objects;
  // Heading turns from east to north over 3 s at 10 Hz.
  for (int i = 0; i <= 30; ++i) {
    const double heading = (std::numbers::pi / 2) * i / 30.0;
    objects = {object(1, false, i, 0, 5 * std::cos(heading), 5 * std::sin(heading))};
    static_cast<void>(detector.update(objects, i * kFrameNs));
  }
  EXPECT_TRUE(objects[0].turning);
  // Driving straight on afterwards clears the flag once the window has passed.
  for (int i = 31; i <= 60; ++i) {
    objects = {object(1, false, 30, i, 0, 5)};
    static_cast<void>(detector.update(objects, i * kFrameNs));
  }
  EXPECT_FALSE(objects[0].turning);
}

}  // namespace

namespace {

TEST(Zones, Wgs84RoundTripWithGeoAnchor) {
  const jp::core::GeoAnchor anchor({48.1760, 11.5556, 512.0});
  const std::vector<jp::vru::Zone> zones = {
      {"crosswalk_e", "crosswalk", {{30, 10}, {42, 10}, {42, 14}, {30, 14}}}};
  const auto text = jp::vru::zones_to_geojson(zones, &anchor);
  EXPECT_EQ(text.find("coordinate_frame"), std::string::npos);  // WGS84 is the default
  const auto back = jp::vru::parse_zones(text, &anchor);
  ASSERT_EQ(back.size(), 1U);
  ASSERT_EQ(back[0].polygon.size(), 4U);
  EXPECT_LT((back[0].polygon[2] - Eigen::Vector2d(42, 14)).norm(), 0.01);
  EXPECT_TRUE(back[0].contains({36, 12}));
}

}  // namespace
