#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "jp/core/class_catalog.hpp"
#include "jp/core/pipeline_config.hpp"
#include "jp/core/world_state.hpp"
#include "jp/vru/zones.hpp"

namespace jp::vru {

inline constexpr const char* kVruOnCrosswalk = "VRU_ON_CROSSWALK";
inline constexpr const char* kVruVehicleConflict = "VRU_VEHICLE_CONFLICT";

/// Ground footprint of a class (m).
struct ClassFootprint {
  double length_m = 0.0;
  double width_m = 0.0;
};

struct EventOptions {
  int crosswalk_min_frames = 3;     ///< Frames a VRU must be on a crosswalk to start the event.
  double conflict_horizon_s = 3.0;  ///< Constant-velocity prediction horizon.
  double conflict_step_s = 0.1;     ///< Prediction step.
  /// Predicted gap between the vehicle footprint and the VRU below which a pair is in conflict.
  double conflict_gap_m = 1.0;
  /// The gap must shrink at least this fast on average until the closest approach: objects that
  /// are merely close (standing, or passing in parallel) are not in conflict.
  double conflict_min_closing_speed_mps = 0.3;
  /// Velocities are extrapolated only once the filter knows them this well; until then the
  /// object is predicted to stay where it is (e.g. new tracks).
  double conflict_max_speed_std_mps = 0.5;
  double stopped_speed_mps = 0.5;  ///< Vehicles slower than this are stopped.
  double starting_window_s = 2.0;  ///< Moving vehicles stopped this recently are starting.
  int conflict_min_frames = 2;     ///< Frames in conflict before the event starts.
  int end_frames = 3;              ///< Frames without the condition before an event ends.
  double turning_window_s = 2.0;   ///< Heading change is measured over this time.
  double turning_threshold_deg = 20.0;
  /// Footprints by class; unknown classes use a car (vehicles) or a pedestrian (VRUs).
  std::map<std::string, ClassFootprint, std::less<>> footprints;
};

/// Event options from the events section of the pipeline config and the class footprints.
[[nodiscard]] EventOptions event_options(const core::EventSettings& settings,
                                         const core::ClassCatalog& catalog);

/// Closest predicted approach between a vehicle footprint and a VRU.
struct ClosestApproach {
  double gap_m = 0.0;   ///< Smallest gap between the footprints (negative: overlap).
  double time_s = 0.0;  ///< Time until that gap.
  double initial_gap_m = 0.0;
};

/// Predicts the gap between a vehicle (rectangle along its heading) and a VRU (discs along its
/// heading) over the horizon; each object moves at constant velocity if its velocity is known
/// well enough (`max_speed_std_mps`) and stays in place otherwise.
[[nodiscard]] ClosestApproach predict_closest_approach(const core::WorldObject& vehicle,
                                                       const core::WorldObject& vru,
                                                       const EventOptions& options);

/// Assigns zones, turning flags and the validated flag to the objects of each frame and detects
/// VRU events with debounced start and end. Events are reported every frame while active and
/// once with state "ended". Objects outside the validated region take part in no events.
class EventDetector {
 public:
  using RegionTest = std::function<bool(const Eigen::Vector2d&)>;

  EventDetector(std::vector<Zone> zones, EventOptions options = {});

  /// Updates `objects` in place (zones, turning, validated) and returns the events of this frame.
  [[nodiscard]] std::vector<core::WorldEvent> update(std::vector<core::WorldObject>& objects,
                                                     std::int64_t timestamp_ns);

  /// Forgets all state (e.g. after seeking in a recording).
  void reset();
  /// Replaces the zones; running events end through the usual debouncing.
  void set_zones(std::vector<Zone> zones) { zones_ = std::move(zones); }
  /// Restricts events to positions passing `test`; empty: no restriction.
  void set_validated_region(RegionTest test) { validated_ = std::move(test); }

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
  /// "moving", "stopped" or "starting" (moving after a stop within the starting window).
  [[nodiscard]] std::string update_vehicle_state(const core::WorldObject& vehicle,
                                                 std::int64_t timestamp_ns);
  void observe(const Key& key, core::WorldEvent event);
  void observe_crosswalks(const std::vector<core::WorldObject>& objects);
  void observe_conflicts(const std::vector<core::WorldObject>& objects, std::int64_t timestamp_ns);
  [[nodiscard]] std::vector<core::WorldEvent> collect_events();

  std::vector<Zone> zones_;
  EventOptions options_;
  RegionTest validated_;
  std::map<Key, Candidate> candidates_;
  std::map<std::int64_t, std::deque<std::pair<std::int64_t, double>>> heading_history_;
  std::map<std::int64_t, std::int64_t> last_stopped_ns_;
  std::int64_t next_event_id_ = 1;
};

}  // namespace jp::vru
