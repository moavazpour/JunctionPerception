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

TEST(Conflict, PredictsFootprintGap) {
  // Car (4.5 m x 1.8 m) heading east at 10 m/s, pedestrian 20 m ahead crossing north at 1 m/s
  // from y = -2: the pedestrian is inside the car footprint from t = 1.8 s.
  const jp::vru::EventOptions options;
  const auto approach = jp::vru::predict_closest_approach(object(1, false, 0, 0, 10, 0),
                                                          object(2, true, 20, -2, 0, 1), options);
  EXPECT_NEAR(approach.time_s, 1.8, 0.05);
  EXPECT_LT(approach.gap_m, 0.0);
  // Car 18.4 m short of the pedestrian, gap 20 - 2.25 - 0.3 along x and 2 m beside.
  EXPECT_GT(approach.initial_gap_m, 15.0);
}

TEST(Conflict, GapFollowsTheFootprint) {
  const jp::vru::EventOptions options;
  // Stopped car facing east; a standing pedestrian 2 m in front of the bumper and another 2 m
  // beside the door: same centre distance class, very different gaps.
  auto car = object(1, false, 0, 0);
  car.heading_rad = 0.0;
  const auto front = jp::vru::predict_closest_approach(car, object(2, true, 4.25, 0), options);
  const auto side = jp::vru::predict_closest_approach(car, object(3, true, 0, 2.9), options);
  EXPECT_NEAR(front.gap_m, 4.25 - 2.25 - 0.3, 1e-9);
  EXPECT_NEAR(side.gap_m, 2.9 - 0.9 - 0.3, 1e-9);
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
  EXPECT_NEAR(*events[0].time_to_closest_s, 1.8, 0.11);
  EXPECT_EQ(events[0].vehicle_state, "moving");
  ASSERT_TRUE(events[0].closing_speed_mps.has_value());
  EXPECT_GT(*events[0].closing_speed_mps, 5.0);
}

// Runs `frames` frames of the same objects through a detector; returns the last events.
std::vector<jp::core::WorldEvent> run(jp::vru::EventDetector& detector,
                                      const std::vector<WorldObject>& objects, int frames,
                                      int first_frame = 1) {
  std::vector<jp::core::WorldEvent> events;
  for (int i = first_frame; i < first_frame + frames; ++i) {
    std::vector<WorldObject> copy = objects;
    events = detector.update(copy, i * kFrameNs);
  }
  return events;
}

TEST(EventDetector, StandingCloseToAStoppedCarIsNoConflict) {
  jp::vru::EventDetector detector({});
  auto car = object(1, false, 0, 0);
  car.heading_rad = 0.0;
  // 0.3 m gap beside the car, nobody moves: close, but nothing is closing in.
  EXPECT_TRUE(run(detector, {car, object(2, true, 0, 1.5)}, 5).empty());
}

TEST(EventDetector, WalkingIntoAStoppedCarIsAConflictWithItsState) {
  jp::vru::EventDetector detector({});
  auto car = object(1, false, 0, 0);
  car.heading_rad = 0.0;
  // Walking at 1.4 m/s straight at the side of the car from 3 m away.
  const auto events = run(detector, {car, object(2, true, 0, 4.2, 0, -1.4)}, 3);
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].vehicle_state, "stopped");
}

TEST(EventDetector, UncertainVelocitiesAreNotExtrapolated) {
  jp::vru::EventDetector detector({});
  auto car = object(1, false, 0, 0);
  car.heading_rad = 0.0;
  // A new track "moving" at 5 m/s towards the car from 10 m: speed not yet known.
  auto pedestrian = object(2, true, 0, 10, 0, -5);
  pedestrian.speed_std_mps = 4.0;
  EXPECT_TRUE(run(detector, {car, pedestrian}, 5).empty());
  // Once the filter is sure of it, the same motion is a conflict.
  pedestrian.speed_std_mps = 0.2;
  EXPECT_EQ(run(detector, {car, pedestrian}, 3, 6).size(), 1U);
}

TEST(EventDetector, VehicleStartingAfterAStop) {
  jp::vru::EventDetector detector({});
  auto car = object(1, false, 0, 0);
  car.heading_rad = 0.0;
  static_cast<void>(run(detector, {car}, 5));
  // Pulls away towards a pedestrian crossing 6 m ahead: reported as starting.
  const auto events =
      run(detector, {object(1, false, 0, 0, 3, 0), object(2, true, 6, -1.5, 0, 1)}, 3, 6);
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].vehicle_state, "starting");
}

TEST(EventDetector, NoEventsOutsideTheValidatedRegion) {
  jp::vru::EventDetector detector(jp::vru::parse_zones(kZones));
  detector.set_validated_region([](const Eigen::Vector2d& p) { return p.x() < 3.0; });
  std::vector<WorldObject> objects = {object(5, true, 5, 2)};
  for (int i = 1; i <= 5; ++i) {
    objects = {object(5, true, 5, 2)};
    EXPECT_TRUE(detector.update(objects, i * kFrameNs).empty());
  }
  // Still tracked and placed in its zone, only flagged as not validated.
  EXPECT_FALSE(objects[0].validated);
  EXPECT_EQ(objects[0].zones, (std::vector<std::string>{"crosswalk_n"}));
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
