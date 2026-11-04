#include <gtest/gtest.h>

#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>

#include "jp/calibration/ground_calibration.hpp"
#include "jp/core/class_catalog.hpp"
#include "jp/server/site_clock.hpp"
#include "jp/server/twin_server.hpp"
#include "jp/sources/world_source.hpp"
#include "jp/twin/world_state_json.hpp"
#include "jp/vru/zones.hpp"

namespace {

TEST(ServerConfig, RepositoryConfigIsValid) {
  const auto config = jp::server::load_server_config(JP_SOURCE_DIR "/configs/server.yaml");
  EXPECT_GT(config.server.port, 0);
  EXPECT_FALSE(config.startup.scene.empty());
  EXPECT_TRUE(config.startup.mode == "live" || config.startup.mode == "oracle" ||
              config.startup.mode == "replay");
}

TEST(SiteClock, PlaysPausesAndLoops) {
  const auto clock = jp::server::parse_site_clock(
      R"({"epoch_ns": 1000000000, "media_ns": 2000000000, "speed": 2.0, "playing": true,
          "period_ns": 10000000000})");
  ASSERT_TRUE(clock.has_value());
  // 1 s after the epoch at double speed: 2 s + 2 s into the recording.
  EXPECT_EQ(clock->position(2'000'000'000, 0), 4'000'000'000);
  // 5 s after: 12 s, wrapped into the 10 s loop.
  EXPECT_EQ(clock->position(6'000'000'000, clock->period_ns), 2'000'000'000);
  auto paused = *clock;
  paused.playing = false;
  EXPECT_EQ(paused.position(6'000'000'000, paused.period_ns), 2'000'000'000);
  EXPECT_FALSE(jp::server::parse_site_clock(R"({"epoch_ns": 1})").has_value());
  EXPECT_FALSE(jp::server::parse_site_clock("not json").has_value());
  EXPECT_FALSE(
      jp::server::parse_site_clock(R"({"epoch_ns": 0, "media_ns": 0, "speed": 0})").has_value());
}

TEST(ServerConfig, ReadsTheSiteClockPath) {
  const auto path = std::filesystem::temp_directory_path() / "jp_server_clock_test.yaml";
  {
    std::ifstream base(JP_SOURCE_DIR "/configs/server.yaml");
    std::ostringstream text;
    text << base.rdbuf();
    std::string yaml = text.str();
    yaml.replace(yaml.find("playback:"), std::string("playback:").size(),
                 "playback:\n  clock: /tmp/site/clock.json");
    std::ofstream(path) << yaml;
  }
  EXPECT_EQ(jp::server::load_server_config(path).server.clock,
            std::filesystem::path("/tmp/site/clock.json"));
  EXPECT_TRUE(
      jp::server::load_server_config(JP_SOURCE_DIR "/configs/server.yaml").server.clock.empty());
  std::filesystem::remove(path);
}

TEST(ServerConfig, SceneDescriptionCarriesCalibrationZonesAndClasses) {
  jp::server::ServedScene scene;
  scene.config = jp::core::load_scene_config(JP_SOURCE_DIR "/scenes/tumtraf_s110_south1");
  scene.ground = jp::calibration::load_ground_calibration(scene.config.calibration_path());
  scene.zones = jp::vru::load_scene_zones(scene.config);
  const auto catalog = jp::core::ClassCatalog::load(JP_SOURCE_DIR "/configs/classes.yaml");
  const auto description =
      nlohmann::json::parse(jp::server::scene_description(scene, catalog, false));
  EXPECT_EQ(description["scene_id"], "tumtraf_s110_south1");
  EXPECT_EQ(description["calibrated"], true);
  EXPECT_TRUE(description["geo_anchor"].is_null());
  EXPECT_TRUE(description["map"].is_null());
  EXPECT_EQ(description["zones"]["coordinate_frame"], "scene_ground");
  EXPECT_EQ(description["zones"]["features"].size(), 2U);
  EXPECT_EQ(description["classes"].size(), catalog.classes().size());
  ASSERT_EQ(description["image_to_ground"].size(), 3U);
  EXPECT_DOUBLE_EQ(description["image_to_ground"][2][2].get<double>(),
                   scene.ground->image_to_ground(2, 2));
}

TEST(ReplaySource, PlaysRecordedStatesWithoutZones) {
  const auto path = std::filesystem::temp_directory_path() / "jp_replay_test.jsonl";
  {
    std::ifstream in(JP_SOURCE_DIR "/tests/fixtures/world_state_v1.json");
    std::ostringstream text;
    text << in.rdbuf();
    auto state = jp::twin::world_state_from_json(text.str());
    jp::twin::JsonlWriter writer(path);
    for (int i = 0; i < 3; ++i) {
      state.frame = i;
      writer.write(state);
    }
  }
  const auto source = jp::sources::make_replay_source(path);
  EXPECT_EQ(source->frame_count(), 3);
  ASSERT_TRUE(source->seek(2));
  const auto frame = source->next(false);
  ASSERT_TRUE(frame.has_value());
  EXPECT_EQ(frame->index, 2);
  EXPECT_TRUE(frame->objects[1].zones.empty());
  EXPECT_FALSE(source->next(false).has_value());
  std::filesystem::remove(path);
}

}  // namespace
