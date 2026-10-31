#include "jp/vru/event_detector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <utility>

namespace jp::vru {

namespace {

constexpr double kNanosecondsPerSecond = 1e9;

double wrapped_angle(double angle) { return std::remainder(angle, 2.0 * std::numbers::pi); }

int min_frames_for(const std::string& type, const EventOptions& options) {
  return type == kVruOnCrosswalk ? options.crosswalk_min_frames : options.conflict_min_frames;
}

}  // namespace

double rectangle_disc_gap(const Eigen::Vector2d& center, double heading_rad,
                          const ClassFootprint& size, const Eigen::Vector2d& disc, double radius) {
  const Eigen::Vector2d axis(std::cos(heading_rad), std::sin(heading_rad));
  const Eigen::Vector2d d = disc - center;
  const Eigen::Vector2d local(d.dot(axis), (axis.x() * d.y()) - (axis.y() * d.x()));
  const Eigen::Vector2d outside =
      (local.cwiseAbs() - Eigen::Vector2d(0.5 * size.length_m, 0.5 * size.width_m)).cwiseMax(0.0);
  return outside.norm() - radius;
}

EventOptions event_options(const core::EventSettings& settings, const core::ClassCatalog& catalog) {
  EventOptions o;
  o.crosswalk_min_frames = settings.crosswalk_min_frames;
  o.conflict_horizon_s = settings.conflict_horizon_s;
  o.conflict_step_s = settings.conflict_step_s;
  o.conflict_gap_m = settings.conflict_gap_m;
  o.conflict_min_closing_speed_mps = settings.conflict_min_closing_speed_mps;
  o.conflict_max_speed_std_mps = settings.conflict_max_speed_std_mps;
  o.stopped_speed_mps = settings.stopped_speed_mps;
  o.starting_window_s = settings.starting_window_s;
  o.conflict_min_frames = settings.conflict_min_frames;
  o.end_frames = settings.end_frames;
  o.turning_window_s = settings.turning_window_s;
  o.turning_threshold_deg = settings.turning_threshold_deg;
  for (const auto& info : catalog.classes()) {
    o.footprints[info.name] = {info.length_m, info.width_m};
  }
  return o;
}

namespace {

constexpr ClassFootprint kDefaultVehicle{4.5, 1.8};
constexpr ClassFootprint kDefaultVru{0.6, 0.6};

ClassFootprint footprint_of(const core::WorldObject& object, const EventOptions& options) {
  const auto it = options.footprints.find(object.object_class);
  if (it != options.footprints.end()) {
    return it->second;
  }
  return object.vru ? kDefaultVru : kDefaultVehicle;
}

bool velocity_known(const core::WorldObject& object, const EventOptions& options) {
  return !object.speed_std_mps || *object.speed_std_mps <= options.conflict_max_speed_std_mps;
}

Eigen::Vector2d predicted_velocity(const core::WorldObject& object, const EventOptions& options) {
  return velocity_known(object, options) ? Eigen::Vector2d(object.vx, object.vy)
                                         : Eigen::Vector2d::Zero();
}

// Footprint gap at one time. Vehicles without a heading are a disc of half their length (their
// orientation is unknown); VRUs are discs of half their width along their heading (a
// two-wheeler: a row of three), or one disc of half their length without a heading.
double footprint_gap(const Eigen::Vector2d& vehicle_center, const core::WorldObject& vehicle,
                     const ClassFootprint& vehicle_size, const Eigen::Vector2d& vru_center,
                     const core::WorldObject& vru, const ClassFootprint& vru_size) {
  std::vector<Eigen::Vector2d> discs;
  double radius = 0.5 * vru_size.length_m;
  if (vru.heading_rad) {
    radius = 0.5 * vru_size.width_m;
    const double half_span = 0.5 * std::max(vru_size.length_m - vru_size.width_m, 0.0);
    const Eigen::Vector2d axis(std::cos(*vru.heading_rad), std::sin(*vru.heading_rad));
    discs = {vru_center - (half_span * axis), vru_center, vru_center + (half_span * axis)};
  } else {
    discs = {vru_center};
  }
  double gap = std::numeric_limits<double>::infinity();
  for (const auto& disc : discs) {
    gap = std::min(
        gap,
        vehicle.heading_rad
            ? rectangle_disc_gap(vehicle_center, *vehicle.heading_rad, vehicle_size, disc, radius)
            : (disc - vehicle_center).norm() - (0.5 * vehicle_size.length_m) - radius);
  }
  return gap;
}

}  // namespace

ClosestApproach predict_closest_approach(const core::WorldObject& vehicle,
                                         const core::WorldObject& vru,
                                         const EventOptions& options) {
  const ClassFootprint vehicle_size = footprint_of(vehicle, options);
  const ClassFootprint vru_size = footprint_of(vru, options);
  const Eigen::Vector2d vehicle_velocity = predicted_velocity(vehicle, options);
  const Eigen::Vector2d vru_velocity = predicted_velocity(vru, options);
  const Eigen::Vector2d vehicle_start(vehicle.x, vehicle.y);
  const Eigen::Vector2d vru_start(vru.x, vru.y);
  const auto gap_at = [&](double t) {
    return footprint_gap(vehicle_start + (t * vehicle_velocity), vehicle, vehicle_size,
                         vru_start + (t * vru_velocity), vru, vru_size);
  };
  ClosestApproach best;
  best.initial_gap_m = gap_at(0.0);
  best.gap_m = best.initial_gap_m;
  const int steps =
      static_cast<int>(std::floor((options.conflict_horizon_s / options.conflict_step_s) + 1e-9));
  for (int i = 1; i <= steps; ++i) {
    const double t = i * options.conflict_step_s;
    const double gap = gap_at(t);
    if (gap < best.gap_m) {
      best.gap_m = gap;
      best.time_s = t;
    }
  }
  return best;
}

EventDetector::EventDetector(std::vector<Zone> zones, EventOptions options)
    : zones_(std::move(zones)), options_(std::move(options)) {}

void EventDetector::reset() {
  candidates_.clear();
  heading_history_.clear();
  last_stopped_ns_.clear();
}

void EventDetector::assign_zones(core::WorldObject& object) const {
  object.zones.clear();
  for (const auto& zone : zones_) {
    if (zone.contains({object.x, object.y})) {
      object.zones.push_back(zone.id);
    }
  }
}

void EventDetector::update_turning(core::WorldObject& object, std::int64_t timestamp_ns) {
  object.turning = false;
  if (object.vru || !object.heading_rad) {
    return;
  }
  auto& history = heading_history_[object.id];
  history.emplace_back(timestamp_ns, *object.heading_rad);
  const auto window_ns =
      static_cast<std::int64_t>(options_.turning_window_s * kNanosecondsPerSecond);
  // Keep the newest sample at least one window old as reference, drop older ones.
  while (history.size() > 1 && history[1].first <= timestamp_ns - window_ns) {
    history.pop_front();
  }
  const double change_deg = std::abs(wrapped_angle(*object.heading_rad - history.front().second)) *
                            180.0 / std::numbers::pi;
  object.turning = change_deg > options_.turning_threshold_deg;
}

std::string EventDetector::update_vehicle_state(const core::WorldObject& vehicle,
                                                std::int64_t timestamp_ns) {
  if (velocity_known(vehicle, options_) && vehicle.speed_mps < options_.stopped_speed_mps) {
    last_stopped_ns_[vehicle.id] = timestamp_ns;
    return "stopped";
  }
  const auto stopped = last_stopped_ns_.find(vehicle.id);
  const auto window_ns =
      static_cast<std::int64_t>(options_.starting_window_s * kNanosecondsPerSecond);
  if (stopped != last_stopped_ns_.end() && timestamp_ns - stopped->second <= window_ns) {
    return "starting";
  }
  return "moving";
}

void EventDetector::observe(const Key& key, core::WorldEvent event) {
  Candidate& candidate = candidates_[key];
  candidate.seen_now = true;
  ++candidate.hits;
  candidate.misses = 0;
  candidate.latest = std::move(event);
}

void EventDetector::observe_crosswalks(const std::vector<core::WorldObject>& objects) {
  for (const auto& object : objects) {
    if (!object.vru || !object.validated) {
      continue;
    }
    for (const auto& zone_id : object.zones) {
      const auto zone = std::ranges::find(zones_, zone_id, &Zone::id);
      if (zone == zones_.end() || zone->type != "crosswalk") {
        continue;
      }
      core::WorldEvent event;
      event.type = kVruOnCrosswalk;
      event.object_ids = {object.id};
      event.zone = zone_id;
      observe({kVruOnCrosswalk, object.id, 0, zone_id}, std::move(event));
    }
  }
}

void EventDetector::observe_conflicts(const std::vector<core::WorldObject>& objects,
                                      std::int64_t timestamp_ns) {
  for (const auto& vehicle : objects) {
    if (vehicle.vru) {
      continue;
    }
    // Kept for every vehicle (also outside the region) so "starting" survives a region border.
    const std::string state = update_vehicle_state(vehicle, timestamp_ns);
    if (!vehicle.validated) {
      continue;
    }
    for (const auto& vru : objects) {
      if (!vru.vru || !vru.validated) {
        continue;
      }
      const auto approach = predict_closest_approach(vehicle, vru, options_);
      if (approach.gap_m >= options_.conflict_gap_m || approach.time_s <= 0.0) {
        continue;
      }
      const double closing_speed = (approach.initial_gap_m - approach.gap_m) / approach.time_s;
      if (closing_speed < options_.conflict_min_closing_speed_mps) {
        continue;
      }
      core::WorldEvent event;
      event.type = kVruVehicleConflict;
      event.object_ids = {vehicle.id, vru.id};
      event.min_distance_m = approach.gap_m;
      event.time_to_closest_s = approach.time_s;
      event.closing_speed_mps = closing_speed;
      event.vehicle_state = state;
      if (!vru.zones.empty()) {
        event.zone = vru.zones.front();
      }
      observe({kVruVehicleConflict, vru.id, vehicle.id, ""}, std::move(event));
    }
  }
}

std::vector<core::WorldEvent> EventDetector::collect_events() {
  std::vector<core::WorldEvent> events;
  for (auto it = candidates_.begin(); it != candidates_.end();) {
    Candidate& c = it->second;
    if (!c.seen_now) {
      c.hits = 0;
      ++c.misses;
    }
    c.seen_now = false;
    const auto& type = std::get<0>(it->first);
    if (!c.event_id && c.hits >= min_frames_for(type, options_)) {
      c.event_id = next_event_id_++;
    }
    const bool ended = c.misses >= options_.end_frames;
    if (c.event_id && (c.misses == 0 || ended)) {
      core::WorldEvent event = c.latest;
      event.id = *c.event_id;
      event.state = ended ? "ended" : "active";
      events.push_back(std::move(event));
    }
    if (ended) {
      it = candidates_.erase(it);
    } else {
      ++it;
    }
  }
  return events;
}

std::vector<core::WorldEvent> EventDetector::update(std::vector<core::WorldObject>& objects,
                                                    std::int64_t timestamp_ns) {
  for (auto& object : objects) {
    assign_zones(object);
    update_turning(object, timestamp_ns);
    object.validated = !validated_ || validated_({object.x, object.y});
  }
  const auto gone = [&](const auto& entry) {
    return std::ranges::none_of(objects,
                                [&](const core::WorldObject& o) { return o.id == entry.first; });
  };
  std::erase_if(heading_history_, gone);
  std::erase_if(last_stopped_ns_, gone);
  observe_crosswalks(objects);
  observe_conflicts(objects, timestamp_ns);
  return collect_events();
}

}  // namespace jp::vru
