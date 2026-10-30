#include "jp/vru/event_detector.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace jp::vru {

namespace {

constexpr double kNanosecondsPerSecond = 1e9;

double wrapped_angle(double angle) { return std::remainder(angle, 2.0 * std::numbers::pi); }

int min_frames_for(const std::string& type, const EventOptions& options) {
  return type == kVruOnCrosswalk ? options.crosswalk_min_frames : options.conflict_min_frames;
}

}  // namespace

EventOptions event_options(const core::EventSettings& settings) {
  EventOptions o;
  o.crosswalk_min_frames = settings.crosswalk_min_frames;
  o.conflict_horizon_s = settings.conflict_horizon_s;
  o.conflict_step_s = settings.conflict_step_s;
  o.conflict_distance_m = settings.conflict_distance_m;
  o.conflict_min_frames = settings.conflict_min_frames;
  o.end_frames = settings.end_frames;
  o.turning_window_s = settings.turning_window_s;
  o.turning_threshold_deg = settings.turning_threshold_deg;
  return o;
}

ClosestApproach predict_closest_approach(const core::WorldObject& a, const core::WorldObject& b,
                                         double horizon_s, double step_s) {
  const Eigen::Vector2d dp(b.x - a.x, b.y - a.y);
  const Eigen::Vector2d dv(b.vx - a.vx, b.vy - a.vy);
  ClosestApproach best{dp.norm(), 0.0};
  const int steps = static_cast<int>(std::floor((horizon_s / step_s) + 1e-9));
  for (int i = 1; i <= steps; ++i) {
    const double t = i * step_s;
    const double d = (dp + (t * dv)).norm();
    if (d < best.distance_m) {
      best = {d, t};
    }
  }
  return best;
}

EventDetector::EventDetector(std::vector<Zone> zones, EventOptions options)
    : zones_(std::move(zones)), options_(options) {}

void EventDetector::reset() {
  candidates_.clear();
  heading_history_.clear();
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

void EventDetector::observe(const Key& key, core::WorldEvent event) {
  Candidate& candidate = candidates_[key];
  candidate.seen_now = true;
  ++candidate.hits;
  candidate.misses = 0;
  candidate.latest = std::move(event);
}

void EventDetector::observe_crosswalks(const std::vector<core::WorldObject>& objects) {
  for (const auto& object : objects) {
    if (!object.vru) {
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

void EventDetector::observe_conflicts(const std::vector<core::WorldObject>& objects) {
  for (const auto& vru : objects) {
    if (!vru.vru) {
      continue;
    }
    for (const auto& vehicle : objects) {
      if (vehicle.vru) {
        continue;
      }
      const auto approach = predict_closest_approach(vru, vehicle, options_.conflict_horizon_s,
                                                     options_.conflict_step_s);
      if (approach.distance_m >= options_.conflict_distance_m) {
        continue;
      }
      core::WorldEvent event;
      event.type = kVruVehicleConflict;
      event.object_ids = {vehicle.id, vru.id};
      event.min_distance_m = approach.distance_m;
      event.time_to_closest_s = approach.time_s;
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
  }
  std::erase_if(heading_history_, [&](const auto& entry) {
    return std::ranges::none_of(objects,
                                [&](const core::WorldObject& o) { return o.id == entry.first; });
  });
  observe_crosswalks(objects);
  observe_conflicts(objects);
  return collect_events();
}

}  // namespace jp::vru
