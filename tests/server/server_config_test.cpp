#include <gtest/gtest.h>

#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>

#include "jp/calibration/ground_calibration.hpp"
#include "jp/core/class_catalog.hpp"
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
