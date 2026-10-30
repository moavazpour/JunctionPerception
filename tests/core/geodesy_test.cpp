#include "jp/core/geodesy.hpp"

#include <gtest/gtest.h>

#include "jp/core/scene_config.hpp"

namespace {

constexpr double kMetresPerDegreeLatitude = 111'200.0;  // approximately, at 48 deg

TEST(GeoAnchor, AnchorIsTheOrigin) {
  const jp::core::GeoAnchor anchor({48.1760, 11.5556, 512.0});
  EXPECT_LT(anchor.to_world(48.1760, 11.5556).norm(), 1e-6);
}

TEST(GeoAnchor, EastAndNorthPointTheRightWay) {
  const jp::core::GeoAnchor anchor({48.1760, 11.5556, 512.0});
  const auto north = anchor.to_world(48.1770, 11.5556);
  EXPECT_NEAR(north.x(), 0.0, 0.01);
  EXPECT_NEAR(north.y(), 0.001 * kMetresPerDegreeLatitude, 1.0);
  const auto east = anchor.to_world(48.1760, 11.5576);
  EXPECT_GT(east.x(), 140.0);  // 0.002 deg longitude at 48 deg is about 149 m
  EXPECT_LT(east.x(), 160.0);
  EXPECT_NEAR(east.y(), 0.0, 0.01);
}

TEST(GeoAnchor, RoundTrip) {
  const jp::core::GeoAnchor anchor({48.1760, 11.5556, 512.0});
  for (const Eigen::Vector2d& world :
       {Eigen::Vector2d(120.5, -80.25), Eigen::Vector2d(-250, 300)}) {
    const auto geo = anchor.to_geo(world);
    EXPECT_LT((anchor.to_world(geo.lat_deg, geo.lon_deg) - world).norm(), 1e-4);
  }
}

TEST(SceneConfig, LoadsRepositoryScenes) {
  const auto custom = jp::core::load_scene_config(JP_SOURCE_DIR "/scenes/munich_5538");
  EXPECT_EQ(custom.scene_id, "munich_5538");
  EXPECT_EQ(custom.kind, "custom");
  EXPECT_EQ(custom.width_px, 1920);
  ASSERT_TRUE(custom.geo_anchor.has_value());
  EXPECT_NEAR(custom.geo_anchor->lat_deg, 48.176, 1e-9);
  const auto benchmark = jp::core::load_scene_config(JP_SOURCE_DIR "/scenes/tumtraf_s110_south1");
  EXPECT_FALSE(benchmark.geo_anchor.has_value());
}

}  // namespace
