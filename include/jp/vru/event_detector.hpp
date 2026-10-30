#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "jp/core/pipeline_config.hpp"
#include "jp/core/world_state.hpp"
#include "jp/vru/zones.hpp"

namespace jp::vru {

inline constexpr const char* kVruOnCrosswalk = "VRU_ON_CROSSWALK";
inline constexpr const char* kVruVehicleConflict = "VRU_VEHICLE_CONFLICT";

struct EventOptions {
  int crosswalk_min_frames = 3;      ///< Frames a VRU must be on a crosswalk to start the event.
  double conflict_horizon_s = 3.0;   ///< Constant-velocity prediction horizon.
  double conflict_step_s = 0.1;      ///< Prediction step.
  double conflict_distance_m = 2.0;  ///< Predicted distance below which a pair is in conflict.
  int conflict_min_frames = 2;       ///< Frames in conflict before the event starts.
  int end_frames = 3;                ///< Frames without the condition before an event ends.
  double turning_window_s = 2.0;     ///< Heading change is measured over this time.
  double turning_threshold_deg = 20.0;
};

/// Event options from the events section of the pipeline config.
[[nodiscard]] EventOptions event_options(const core::EventSettings& settings);

/// Closest approach of two objects moving at constant velocity, sampled over the horizon.
struct ClosestApproach {
  double distance_m = 0.0;
  double time_s = 0.0;
};

[[nodiscard]] ClosestApproach predict_closest_approach(const core::WorldObject& a,
                                                       const core::WorldObject& b, double horizon_s,
                                                       double step_s);

/// Assigns zones and turning flags to the objects of each frame and detects VRU events with
/// debounced start and end. Events are reported every frame while active and once with state
/// "ended".
class EventDetector {
 public:
  EventDetector(std::vector<Zone> zones, EventOptions options = {});

  /// Updates `objects` in place (zones, turning) and returns the events of this frame.
  [[nodiscard]] std::vector<core::WorldEvent> update(std::vector<core::WorldObject>& objects,
                                                     std::int64_t timestamp_ns);

  /// Forgets all state (e.g. after seeking in a recording).
  void reset();
  /// Replaces the zones; running events end through the usual debouncing.
  void set_zones(std::vector<Zone> zones) { zones_ = std::move(zones); }

  [[nodiscard]] const std::vector<Zone>& zones() const { return zones_; }

 private:
  // Event identity: type and the involved object IDs (and zone for crosswalk events).
  using Key = std::tuple<std::string, std::int64_t, std::int64_t, std::string>;

  struct Candidate {
    int hits = 0;
    int misses = 0;
    bool seen_now = false;
    std::optional<std::int64_t> event_id;  // set once the event has started
    core::WorldEvent latest;
  };

  void assign_zones(core::WorldObject& object) const;
  void update_turning(core::WorldObject& object, std::int64_t timestamp_ns);
  void observe(const Key& key, core::WorldEvent event);
  void observe_crosswalks(const std::vector<core::WorldObject>& objects);
  void observe_conflicts(const std::vector<core::WorldObject>& objects);
  [[nodiscard]] std::vector<core::WorldEvent> collect_events();

  std::vector<Zone> zones_;
  EventOptions options_;
  std::map<Key, Candidate> candidates_;
  std::map<std::int64_t, std::deque<std::pair<std::int64_t, double>>> heading_history_;
  std::int64_t next_event_id_ = 1;
};

}  // namespace jp::vru
