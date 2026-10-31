#include "jp/twin/world_state_json.hpp"

#include <gtest/gtest.h>

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace {

std::string fixture() {
  std::ifstream in(JP_SOURCE_DIR "/tests/fixtures/world_state_v1.json");
  std::ostringstream text;
  text << in.rdbuf();
  return text.str();
}

TEST(WorldStateJson, ParsesSharedExample) {
  const auto state = jp::twin::world_state_from_json(fixture());
  EXPECT_EQ(state.scene_id, "tumtraf_s110_south1");
  ASSERT_EQ(state.objects.size(), 2U);
  EXPECT_EQ(state.objects[0].object_class, "car");
  EXPECT_FALSE(state.objects[1].heading_rad.has_value());
  EXPECT_EQ(state.objects[1].zones, std::vector<std::string>{"crosswalk_n"});
  EXPECT_DOUBLE_EQ(state.objects[0].speed_std_mps.value_or(0.0), 0.12);
  EXPECT_FALSE(state.objects[1].speed_std_mps.has_value());
  EXPECT_FALSE(state.objects[1].validated);
  EXPECT_EQ(state.events[0].vehicle_state, "moving");
  ASSERT_EQ(state.events.size(), 2U);
  ASSERT_TRUE(state.events[0].min_distance_m.has_value());
  EXPECT_DOUBLE_EQ(*state.events[0].min_distance_m, 1.4);
  EXPECT_FALSE(state.events[1].time_to_closest_s.has_value());
}

TEST(WorldStateJson, RoundTripKeepsEveryField) {
  const auto state = jp::twin::world_state_from_json(fixture());
  const auto again = jp::twin::world_state_from_json(jp::twin::to_json(state));
  EXPECT_EQ(jp::twin::to_json(again), jp::twin::to_json(state));
  EXPECT_EQ(again.timestamp_ns, 1651673060123456789);
  ASSERT_TRUE(again.objects[0].heading_rad.has_value());
  EXPECT_NEAR(*again.objects[0].heading_rad, 1.5708, 1e-9);
}

TEST(WorldStateJson, RejectsOtherSchemaVersion) {
  std::string text = fixture();
  const std::string version = "\"schema_version\": 1";
  text.replace(text.find(version), version.size(), "\"schema_version\": 2");
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
