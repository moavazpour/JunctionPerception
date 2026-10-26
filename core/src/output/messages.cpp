#include "jp/output/messages.hpp"

#include <cmath>

namespace jp::msg {
namespace {
double r2(double v) { return std::round(v * 100.0) / 100.0; }
double r3(double v) { return std::round(v * 1000.0) / 1000.0; }
double r7(double v) { return std::round(v * 1e7) / 1e7; }
double deg(double rad) { return rad * 180.0 / M_PI; }
// Compass heading (deg, clockwise from north) of a ground-frame yaw (counter-clockwise from east).
double heading(double yaw) { return std::fmod(90.0 - deg(yaw) + 720.0, 360.0); }

std::string_view stateName(TrackState s) {
  switch (s) {
    case TrackState::Tentative: return "tentative";
    case TrackState::Confirmed: return "tracked";
    default: return "predicted";
  }
}
}  // namespace

nlohmann::json header(const std::string& type, const std::string& site, const std::string& camera, double stamp) {
  return {{"schema", "junction-perception/1.0"}, {"type", type}, {"site", site}, {"camera", camera}, {"ts", r3(stamp)}};
}

nlohmann::json roadUsers(const std::vector<RoadUser>& users, const SiteMap& map) {
  nlohmann::json a = nlohmann::json::array();
  for (const auto& u : users) {
    nlohmann::json o = {
        {"id", u.id},
        {"class", className(u.cls)},
        {"state", stateName(u.state)},
        {"x", r2(u.pos.x())},
        {"y", r2(u.pos.y())},
        {"lat", r7(u.lat)},
        {"lon", r7(u.lon)},
        {"heading", r2(heading(u.yaw))},
        {"yaw", r3(u.yaw)},
        {"speed", r2(u.speed)},
        {"speed_kmh", r2(u.speed * 3.6)},
        {"accel", r2(u.accel)},
        {"yaw_rate", r3(u.yaw_rate)},
        {"length", r2(u.dims[0])},
        {"width", r2(u.dims[1])},
        {"height", r2(u.dims[2])},
        {"sigma", {{"pos", r2(u.pos_sigma)}, {"speed", r2(u.speed_sigma)}, {"accel", r2(u.accel_sigma)}}},
        {"confidence", r2(u.confidence)},
        {"range", r2(u.range)},
        {"age", r2(u.age)},
        {"occluded", u.occluded},
    };
    if (!u.lane.empty()) {
      o["lane"] = u.lane;
      o["lane_s"] = r2(u.lane_s);
      o["lateral"] = r2(u.lateral);
      const Lane* l = map.lane(u.lane);
      if (l) o["approach"] = l->approach;
    }
    if (u.dist_to_stop) o["dist_to_stop"] = r2(*u.dist_to_stop);
    if (u.eta) o["eta"] = r2(*u.eta);
    if (u.eta_cv) o["eta_cv"] = r2(*u.eta_cv);
    if (u.eta) o["eta_sigma"] = r2(u.eta_sigma);
    if (u.stopping) o["stopping"] = true;
    if (u.queued) o["queued"] = true;
    a.push_back(o);
  }
  return a;
}

nlohmann::json lanes(const std::vector<LaneState>& lanes) {
  nlohmann::json a = nlohmann::json::array();
  for (const auto& l : lanes)
    a.push_back({{"id", l.id},
                 {"users", l.users},
                 {"queue_length", r2(l.queue_length)},
                 {"queue_count", l.queue_count},
                 {"crossings", l.crossings},
                 {"mean_speed", r2(l.mean_speed)}});
  return a;
}

nlohmann::json zones(const std::vector<ZoneState>& zones) {
  nlohmann::json a = nlohmann::json::array();
  for (const auto& z : zones)
    a.push_back({{"id", z.id},
                 {"type", z.type},
                 {"lane", z.lane},
                 {"occupied", z.occupied},
                 {"occupancy", r3(z.occupancy)},
                 {"count", z.count},
                 {"entered", z.entered_now},
                 {"mean_speed", r2(z.mean_speed)},
                 {"users", z.users}});
  return a;
}

nlohmann::json nearMiss(const NearMissEvent& e) {
  nlohmann::json j = {{"event_id", e.event_id},
                      {"kind", e.type},
                      {"severity", e.severity},
                      {"finished", e.finished},
                      {"start", r3(e.start)},
                      {"end", r3(e.end)},
                      {"duration", r2(e.end - e.start)},
                      {"a", {{"id", e.a}, {"class", className(e.cls_a)}, {"max_speed", r2(e.speed_a)}}},
                      {"x", r2(e.point.x())},
                      {"y", r2(e.point.y())},
                      {"lat", r7(e.lat)},
                      {"lon", r7(e.lon)}};
  if (e.type != "hard_braking") {
    j["b"] = {{"id", e.b}, {"class", className(e.cls_b)}, {"max_speed", r2(e.speed_b)}};
  }
  nlohmann::json m;
  if (e.min_ttc) m["ttc"] = r2(*e.min_ttc);
  if (e.min_mttc) m["mttc"] = r2(*e.min_mttc);
  if (e.min_pet) m["pet"] = r2(*e.min_pet);
  if (e.max_drac > 0) m["drac"] = r2(e.max_drac);
  if (e.type == "hard_braking") m["min_accel"] = r2(e.min_accel_a);
  j["metrics"] = m;
  return j;
}

nlohmann::json stopLineCrossing(const StopLineCrossing& c) {
  return {{"id", c.id}, {"lane", c.lane}, {"class", className(c.cls)}, {"t", r3(c.stamp)}, {"speed", r2(c.speed)},
          {"accel", r2(c.accel)}};
}

nlohmann::json health(const HealthReport& h) {
  nlohmann::json cov = nlohmann::json::array();
  for (const auto& z : h.coverage)
    cov.push_back({{"zone", z.zone},
                   {"visible", r3(z.visible)},
                   {"static_occluded", r3(z.static_occluded)},
                   {"dynamic_occluded", r3(z.dynamic_occluded)},
                   {"unobservable", r3(z.unobservable)}});
  return {
      {"level", levelName(h.level)},
      {"confidence", r3(h.confidence)},
      {"reasons", h.reasons},
      {"calibrated", h.calibrated},
      {"pipeline", {{"fps", r2(h.fps)}, {"latency_ms", r2(h.latency_ms)}, {"detector_ms", r2(h.detector_ms)}}},
      {"stability",
       {{"status", h.stability.status},
        {"jitter_px", r2(h.stability.jitter_px)},
        {"drift_deg", r3(h.stability.drift_deg)},
        {"track_ratio", r3(h.stability.track_ratio)},
        {"compensating", h.stability.compensating}}},
      {"visibility",
       {{"score", r3(h.visibility.score)},
        {"conditions", h.visibility.conditions},
        {"brightness", r2(h.visibility.brightness)},
        {"contrast", r3(h.visibility.contrast)},
        {"sharpness_ratio", r3(h.visibility.sharpness_ratio)},
        {"haze", r3(h.visibility.haze)},
        {"far_contrast_ratio", r3(h.visibility.far_contrast_ratio)},
        {"obstructed_fraction", r3(h.visibility.obstructed_fraction)},
        {"effective_range_m", r2(h.visibility.effective_range)},
        {"baseline_range_m", r2(h.visibility.baseline_range)},
        {"fragmentation", r3(h.visibility.fragmentation)}}},
      {"calibration",
       {{"status", h.calibration.status},
        {"score", r3(h.calibration.score)},
        {"size_ratio", r3(h.calibration.size_ratio)},
        {"size_slope", r3(h.calibration.size_slope)},
        {"heading_error_deg", r2(h.calibration.heading_error_deg)},
        {"speed_p95", r2(h.calibration.speed_p95)},
        {"samples", h.calibration.samples}}},
      {"coverage", cov},
  };
}

nlohmann::json occlusion(const OcclusionMap& occ) {
  // Run-length encoding of the grid: [value, count, value, count, ...] row-major from origin.
  nlohmann::json rle = nlohmann::json::array();
  const auto& g = occ.grid();
  for (size_t i = 0; i < g.size();) {
    size_t j = i;
    while (j < g.size() && g[j] == g[i]) ++j;
    rle.push_back(g[i]);
    rle.push_back(j - i);
    i = j;
  }
  nlohmann::json segs = nlohmann::json::array();
  for (const auto& s : occ.segments())
    segs.push_back({{"zone", s.zone}, {"s0", r2(s.s0)}, {"s1", r2(s.s1)}, {"cause", s.cause}});
  return {{"origin", {r2(occ.origin().x()), r2(occ.origin().y())}},
          {"cell", occ.cellSize()},
          {"cols", occ.cols()},
          {"rows", occ.rows()},
          {"legend", {"visible", "out_of_view", "static_occluded", "dynamic_occluded", "beyond_resolution"}},
          {"rle", rle},
          {"segments", segs}};
}

nlohmann::json calibration(const CameraModel& cam, const CalibrationResult* res, const std::string& source) {
  nlohmann::json j = cam.toJson();
  j["source"] = source;
  if (res) {
    j["method"] = res->method;
    if (res->method != "keypoints") j["vanishing_point"] = {r2(res->vanishing_point.x()), r2(res->vanishing_point.y())};
    j["tracks_used"] = res->tracks_used;
    j["detections_used"] = res->detections_used;
    j["box_residual"] = r3(res->box_residual);
    j["speed_drift"] = r3(res->speed_cv);
    j["confidence"] = r3(res->confidence);
  }
  return j;
}

nlohmann::json cpm(const std::vector<RoadUser>& users, const CameraModel& cam, std::uint32_t station_id, double stamp,
                   double perception_range) {
  // ETSI ITS timestamp: ms since 2004-01-01T00:00:00Z (TAI ignored; leap seconds not applied).
  constexpr double kItsEpoch = 1072915200.0;
  const auto its_ms = static_cast<std::int64_t>((stamp - kItsEpoch) * 1000.0);
  nlohmann::json objs = nlohmann::json::array();
  auto stationType = [](ObjectClass c) {
    // ETSI TrafficParticipantType
    switch (c) {
      case ObjectClass::Pedestrian: return 1;
      case ObjectClass::Bicycle: return 2;
      case ObjectClass::EScooter: return 2;  // light vehicle; refined by sub-class below
      case ObjectClass::Motorcycle: return 4;
      case ObjectClass::Car: return 5;
      case ObjectClass::Bus: return 6;
      case ObjectClass::Truck: return 8;
      case ObjectClass::Trailer: return 10;
      default: return 0;
    }
  };
  for (const auto& u : users) {
    if (u.state == TrackState::Tentative) continue;
    const double vx = u.speed * std::cos(u.yaw), vy = u.speed * std::sin(u.yaw);
    const double ax = u.accel * std::cos(u.yaw), ay = u.accel * std::sin(u.yaw);
    objs.push_back({
        {"objectId", u.id % 65536},
        {"measurementDeltaTime", 0},
        {"position", {{"xCoordinate", {{"value", std::lround(u.pos.x() * 100)}, {"confidence", std::min(4095L, std::lround(u.pos_sigma * 100 * 2))}}},
                      {"yCoordinate", {{"value", std::lround(u.pos.y() * 100)}, {"confidence", std::min(4095L, std::lround(u.pos_sigma * 100 * 2))}}}}},
        {"velocity", {{"cartesianVelocity", {{"xVelocity", {{"value", std::lround(vx * 100)}, {"confidence", std::min(127L, std::lround(u.speed_sigma * 100 * 2))}}},
                                              {"yVelocity", {{"value", std::lround(vy * 100)}, {"confidence", std::min(127L, std::lround(u.speed_sigma * 100 * 2))}}}}}}},
        {"acceleration", {{"cartesianAcceleration", {{"xAcceleration", {{"value", std::lround(ax * 10)}}},
                                                      {"yAcceleration", {{"value", std::lround(ay * 10)}}}}}}},
        {"angles", {{"zAngle", {{"value", std::lround(heading(u.yaw) * 10) % 3600}, {"confidence", 20}}}}},
        {"objectDimensionX", {{"value", std::lround(u.dims[0] * 10)}, {"confidence", 5}}},
        {"objectDimensionY", {{"value", std::lround(u.dims[1] * 10)}, {"confidence", 5}}},
        {"objectDimensionZ", {{"value", std::lround(u.dims[2] * 10)}, {"confidence", 5}}},
        {"objectAge", std::min(2047L, std::lround(u.age * 1000))},
        {"objectPerceptionQuality", std::clamp(static_cast<int>(u.confidence * 15), 0, 15)},
        {"classification", {{{"objectClass", {{"vruSubClass", u.cls == ObjectClass::Pedestrian ? "pedestrian" : u.cls == ObjectClass::Bicycle ? "bicyclistAndLightVruVehicle" : u.cls == ObjectClass::EScooter ? "bicyclistAndLightVruVehicle" : ""},
                                                {"vehicleSubClass", stationType(u.cls)}}},
                             {"confidence", std::clamp(static_cast<int>(u.confidence * 100), 0, 100)}}}},
    });
  }
  const auto [lat, lon] = enuToGeo(cam.anchor, Eigen::Vector2d::Zero());
  return {
      {"header", {{"protocolVersion", 2}, {"messageId", 14}, {"stationId", station_id}}},
      {"payload",
       {{"managementContainer",
         {{"referenceTime", its_ms},
          {"referencePosition", {{"latitude", std::lround(lat * 1e7)}, {"longitude", std::lround(lon * 1e7)}}}}},
        {"sensorInformationContainer",
         {{{"sensorId", 1}, {"sensorType", "monovideo"}, {"perceptionRegionShape", {{"radial", {{"range", std::lround(perception_range * 10)}}}}}}}},
        {"perceivedObjectContainer", {{"numberOfPerceivedObjects", objs.size()}, {"perceivedObjects", objs}}}}},
  };
}

}  // namespace jp::msg
