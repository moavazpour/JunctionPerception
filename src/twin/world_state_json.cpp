#include "jp/twin/world_state_json.hpp"

#include <array>
#include <cmath>
#include <nlohmann/json.hpp>
#include <stdexcept>

namespace jp::twin {

namespace {

using json = nlohmann::ordered_json;

// Rounds to a number of decimals for compact output (e.g. 3: millimetres).
double rounded(double value, int decimals) {
  const double scale = std::pow(10.0, decimals);
  return std::round(value * scale) / scale;
}

template <typename T>
json optional_value(const std::optional<T>& value) {
  return value ? json(*value) : json(nullptr);
}

template <typename T>
std::optional<T> optional_field(const json& j, const char* key) {
  const auto it = j.find(key);
  if (it == j.end() || it->is_null()) {
    return std::nullopt;
  }
  return it->get<T>();
}

json object_json(const core::WorldObject& o) {
  constexpr int kMetre = 3;
  constexpr int kRadian = 4;
  constexpr int kCovariance = 4;
  return {
      {"id", o.id},
      {"class", o.object_class},
      {"vru", o.vru},
      {"rider", o.rider},
      {"x", rounded(o.x, kMetre)},
      {"y", rounded(o.y, kMetre)},
      {"vx", rounded(o.vx, kMetre)},
      {"vy", rounded(o.vy, kMetre)},
      {"heading_rad", o.heading_rad ? json(rounded(*o.heading_rad, kRadian)) : json(nullptr)},
      {"speed_mps", rounded(o.speed_mps, kMetre)},
      {"speed_std_mps", o.speed_std_mps ? json(rounded(*o.speed_std_mps, kMetre)) : json(nullptr)},
      {"confidence", rounded(o.confidence, kMetre)},
      {"zones", o.zones},
      {"turning", o.turning},
      {"validated", o.validated},
      {"length_m", o.length_m ? json(rounded(*o.length_m, kMetre)) : json(nullptr)},
      {"width_m", o.width_m ? json(rounded(*o.width_m, kMetre)) : json(nullptr)},
      {"heading_source", optional_value(o.heading_source)},
      {"position_cov_m2", o.position_cov_m2
                              ? json::array({rounded((*o.position_cov_m2)[0], kCovariance),
                                             rounded((*o.position_cov_m2)[1], kCovariance),
                                             rounded((*o.position_cov_m2)[2], kCovariance)})
                              : json(nullptr)}};
}

json event_json(const core::WorldEvent& e) {
  return {{"id", e.id},
          {"type", e.type},
          {"state", e.state},
          {"object_ids", e.object_ids},
          {"min_distance_m", optional_value(e.min_distance_m)},
          {"time_to_closest_s", optional_value(e.time_to_closest_s)},
          {"closing_speed_mps", optional_value(e.closing_speed_mps)},
          {"vehicle_state", optional_value(e.vehicle_state)},
          {"zone", optional_value(e.zone)}};
}

core::WorldObject object_from(const json& j) {
  core::WorldObject o;
  o.id = j.at("id").get<std::int64_t>();
  o.object_class = j.at("class").get<std::string>();
  o.vru = j.at("vru").get<bool>();
  o.rider = j.at("rider").get<bool>();
  o.x = j.at("x").get<double>();
  o.y = j.at("y").get<double>();
  o.vx = j.at("vx").get<double>();
  o.vy = j.at("vy").get<double>();
  o.heading_rad = optional_field<double>(j, "heading_rad");
  o.speed_mps = j.at("speed_mps").get<double>();
  o.speed_std_mps = optional_field<double>(j, "speed_std_mps");
  o.confidence = j.at("confidence").get<double>();
  o.zones = j.at("zones").get<std::vector<std::string>>();
  o.turning = j.at("turning").get<bool>();
  o.validated = j.value("validated", true);
  o.length_m = optional_field<double>(j, "length_m");
  o.width_m = optional_field<double>(j, "width_m");
  o.heading_source = optional_field<std::string>(j, "heading_source");
  o.position_cov_m2 = optional_field<std::array<double, 3>>(j, "position_cov_m2");
  return o;
}

core::WorldEvent event_from(const json& j) {
  core::WorldEvent e;
  e.id = j.at("id").get<std::int64_t>();
  e.type = j.at("type").get<std::string>();
  e.state = j.at("state").get<std::string>();
  e.object_ids = j.at("object_ids").get<std::vector<std::int64_t>>();
  e.min_distance_m = optional_field<double>(j, "min_distance_m");
  e.time_to_closest_s = optional_field<double>(j, "time_to_closest_s");
  e.closing_speed_mps = optional_field<double>(j, "closing_speed_mps");
  e.vehicle_state = optional_field<std::string>(j, "vehicle_state");
  e.zone = optional_field<std::string>(j, "zone");
  return e;
}

}  // namespace

std::string to_json(const core::WorldState& state) {
  json objects = json::array();
  for (const auto& o : state.objects) {
    objects.push_back(object_json(o));
  }
  json events = json::array();
  for (const auto& e : state.events) {
    events.push_back(event_json(e));
  }
  json signals = json::array();
  for (const auto& s : state.signals) {
    signals.push_back({{"id", s.id},
                       {"name", s.name},
                       {"state", s.state},
                       {"confidence", rounded(s.confidence, 3)},
                       {"since_ns", s.since_ns}});
  }
  const json j = {{"schema_version", state.schema_version},
                  {"scene_id", state.scene_id},
                  {"frame", state.frame},
                  {"timestamp_ns", state.timestamp_ns},
                  {"source", state.source},
                  {"objects", objects},
                  {"events", events},
                  {"signals", signals}};
  return j.dump();
}

core::WorldState world_state_from_json(std::string_view text) {
  try {
    const json j = json::parse(text);
    core::WorldState state;
    state.schema_version = j.at("schema_version").get<int>();
    if (state.schema_version != core::kWorldStateSchemaVersion) {
      throw std::runtime_error("unsupported world-state schema version " +
                               std::to_string(state.schema_version));
    }
    state.scene_id = j.at("scene_id").get<std::string>();
    state.frame = j.at("frame").get<std::int64_t>();
    state.timestamp_ns = j.at("timestamp_ns").get<std::int64_t>();
    state.source = j.at("source").get<std::string>();
    for (const auto& o : j.at("objects")) {
      state.objects.push_back(object_from(o));
    }
    for (const auto& e : j.at("events")) {
      state.events.push_back(event_from(e));
    }
    // Added after version 1 was published: recordings made before have none.
    for (const auto& s : j.value("signals", json::array())) {
      state.signals.push_back({s.at("id").get<std::string>(), s.value("name", ""),
                               s.at("state").get<std::string>(), s.value("confidence", 0.0),
                               s.value("since_ns", std::int64_t{0})});
    }
    return state;
  } catch (const nlohmann::json::exception& error) {
    throw std::runtime_error(std::string("malformed world state: ") + error.what());
  }
}

JsonlWriter::JsonlWriter(const std::filesystem::path& path) {
  if (path.has_parent_path()) {
    std::filesystem::create_directories(path.parent_path());
  }
  out_.open(path);
  if (!out_) {
    throw std::runtime_error("cannot create " + path.string());
  }
}

void JsonlWriter::write(const core::WorldState& state) { out_ << to_json(state) << '\n'; }

JsonlReader::JsonlReader(const std::filesystem::path& path) : in_(path) {
  if (!in_) {
    throw std::runtime_error("cannot read " + path.string());
  }
}

std::optional<core::WorldState> JsonlReader::next() {
  std::string line;
  while (std::getline(in_, line)) {
    if (!line.empty()) {
      return world_state_from_json(line);
    }
  }
  return std::nullopt;
}

}  // namespace jp::twin
