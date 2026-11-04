#include "jp/twin/world_state_json.hpp"

#include <gtest/gtest.h>

#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <stdexcept>

namespace {

std::string read(const char* name) {
  std::ifstream in(std::string(JP_SOURCE_DIR "/tests/fixtures/") + name);
  std::ostringstream text;
  text << in.rdbuf();
  return text.str();
}

// The camera observation the manager's world model reads (manager/tests/test_world_model.py
// feeds the same file).
std::string fixture() { return read("camera_observation_v2.json"); }

TEST(WorldStateJson, ParsesSharedExample) {
  const auto state = jp::twin::world_state_from_json(fixture());
  EXPECT_EQ(state.schema_version, 2);
  EXPECT_EQ(state.scene_id, "tumtraf_s110");
  EXPECT_EQ(state.camera_id, "south1");
  EXPECT_EQ(state.time_source, "capture");
  ASSERT_EQ(state.objects.size(), 2U);
  EXPECT_EQ(state.objects[0].object_class, "car");
  EXPECT_FALSE(state.objects[1].heading_rad.has_value());
  EXPECT_DOUBLE_EQ(state.objects[0].speed_std_mps.value_or(0.0), 0.12);
  EXPECT_FALSE(state.objects[1].speed_std_mps.has_value());
  EXPECT_FALSE(state.objects[1].validated);
  EXPECT_EQ(state.objects[0].heading_source, "keypoints");
  ASSERT_TRUE(state.objects[0].position_cov_m2.has_value());
  EXPECT_DOUBLE_EQ((*state.objects[0].position_cov_m2)[2], 0.36);
  EXPECT_FALSE(state.objects[1].heading_source.has_value());
  EXPECT_FALSE(state.objects[1].position_cov_m2.has_value());
  ASSERT_EQ(state.signals.size(), 1U);
  EXPECT_EQ(state.signals[0].state, "red");
  EXPECT_EQ(state.signals[0].since_ns, 1651673058000000000);
}

TEST(WorldStateJson, WritesTheFixtureExactly) {
  // Field order and rounding are part of the contract: the serializer reproduces the fixture.
  const auto state = jp::twin::world_state_from_json(fixture());
  const auto expected = nlohmann::ordered_json::parse(fixture());
  EXPECT_EQ(nlohmann::ordered_json::parse(jp::twin::to_json(state)), expected);
}

TEST(WorldStateJson, ReadsVersion1Recordings) {
  // Recordings before the site world model: events, zones and turning flags are ignored, the
  // timestamps are media times.
  const auto state = jp::twin::world_state_from_json(read("world_state_v1.json"));
  EXPECT_EQ(state.schema_version, 2);
  EXPECT_EQ(state.time_source, "media");
  EXPECT_TRUE(state.camera_id.empty());
  ASSERT_EQ(state.objects.size(), 2U);
  EXPECT_EQ(state.objects[0].heading_source, "keypoints");
}

TEST(WorldStateJson, RejectsOtherSchemaVersion) {
  std::string text = fixture();
  const std::string version = "\"schema_version\": 2";
  text.replace(text.find(version), version.size(), "\"schema_version\": 3");
  EXPECT_THROW(static_cast<void>(jp::twin::world_state_from_json(text)), std::runtime_error);
}

TEST(WorldStateJson, JsonlWriterAndReader) {
  const auto path = std::filesystem::temp_directory_path() / "jp_world_state_test.jsonl";
  auto state = jp::twin::world_state_from_json(fixture());
  {
    jp::twin::JsonlWriter writer(path);
    writer.write(state);
    state.frame += 1;
    writer.write(state);
  }
  jp::twin::JsonlReader reader(path);
  EXPECT_EQ(reader.next()->frame, 1234);
  EXPECT_EQ(reader.next()->frame, 1235);
  EXPECT_FALSE(reader.next().has_value());
  std::filesystem::remove(path);
}

}  // namespace
