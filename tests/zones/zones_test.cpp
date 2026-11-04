#include "jp/zones/zones.hpp"

#include <gtest/gtest.h>

#include <stdexcept>

namespace {

constexpr std::string_view kZones = R"json({
  "type": "FeatureCollection", "coordinate_frame": "scene_ground",
  "features": [{"type": "Feature", "properties": {"id": "crosswalk_n", "type": "crosswalk"},
                "geometry": {"type": "Polygon",
                             "coordinates": [[[0, 0], [10, 0], [10, 4], [0, 4], [0, 0]]]}}]
})json";

TEST(Zones, ParsesSceneGroundPolygons) {
  const auto zones = jp::zones::parse_zones(kZones);
  ASSERT_EQ(zones.size(), 1U);
  EXPECT_EQ(zones[0].polygon.size(), 4U);
  EXPECT_TRUE(zones[0].contains({5.0, 2.0}));
  EXPECT_FALSE(zones[0].contains({11.0, 2.0}));
  EXPECT_FALSE(zones[0].contains({5.0, -0.5}));
}

TEST(Zones, RejectsWgs84WithoutGeoAnchor) {
  constexpr std::string_view wgs84 = R"json({"type": "FeatureCollection", "features": []})json";
  EXPECT_THROW(static_cast<void>(jp::zones::parse_zones(wgs84)), std::runtime_error);
}

TEST(Zones, Wgs84RoundTripWithGeoAnchor) {
  const jp::core::GeoAnchor anchor({48.1760, 11.5556, 512.0});
  const std::vector<jp::zones::Zone> zones = {
      {"crosswalk_e", "crosswalk", {{30, 10}, {42, 10}, {42, 14}, {30, 14}}}};
  const auto text = jp::zones::zones_to_geojson(zones, &anchor);
  EXPECT_EQ(text.find("coordinate_frame"), std::string::npos);  // WGS84 is the default
  const auto back = jp::zones::parse_zones(text, &anchor);
  ASSERT_EQ(back.size(), 1U);
  ASSERT_EQ(back[0].polygon.size(), 4U);
  EXPECT_LT((back[0].polygon[2] - Eigen::Vector2d(42, 14)).norm(), 0.01);
  EXPECT_TRUE(back[0].contains({36, 12}));
}

}  // namespace
