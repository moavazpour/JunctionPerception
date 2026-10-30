#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace jp::core {

/// Version of the world-state schema (docs/architecture.md).
inline constexpr int kWorldStateSchemaVersion = 1;

/// One road user in the world frame (metres, ground plane z = 0).
struct WorldObject {
  std::int64_t id = 0;
  std::string object_class;  ///< Twin class, e.g. "car", "pedestrian".
  bool vru = false;          ///< Vulnerable road user (class property).
  bool rider = false;        ///< Two-wheeler with rider.
  double x = 0.0;
  double y = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double speed_mps = 0.0;
  std::optional<double> heading_rad;  ///< Empty while the heading is unknown.
  double confidence = 0.0;
  std::vector<std::string> zones;  ///< Zones containing the object.
  bool turning = false;            ///< Vehicle heading changed strongly over the last seconds.
};

/// A detected situation involving one or more objects.
struct WorldEvent {
  std::int64_t id = 0;
  std::string type;   ///< "VRU_ON_CROSSWALK" or "VRU_VEHICLE_CONFLICT".
  std::string state;  ///< "active" or "ended" (sent once when the event ends).
  std::vector<std::int64_t> object_ids;
  std::optional<double> min_distance_m;     ///< Conflicts: predicted closest distance.
  std::optional<double> time_to_closest_s;  ///< Conflicts: time until the closest approach.
  std::optional<std::string> zone;
};

/// Everything the twin knows at one frame.
struct WorldState {
  int schema_version = kWorldStateSchemaVersion;
  std::string scene_id;
  std::int64_t frame = 0;
  std::int64_t timestamp_ns = 0;
  std::string source;  ///< "live", "oracle" or "replay".
  std::vector<WorldObject> objects;
  std::vector<WorldEvent> events;
};

}  // namespace jp::core
