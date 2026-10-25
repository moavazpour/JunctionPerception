#pragma once

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "jp/calib/auto_calibrator.hpp"
#include "jp/health/monitors.hpp"
#include "jp/safety/near_miss.hpp"
#include "jp/scene/intersection.hpp"

namespace jp::msg {

// All messages carry: schema version, site id, camera id, stream timestamp (s, UNIX epoch).
nlohmann::json header(const std::string& type, const std::string& site, const std::string& camera, double stamp);

nlohmann::json roadUsers(const std::vector<RoadUser>& users, const SiteMap& map);
nlohmann::json lanes(const std::vector<LaneState>& lanes);
nlohmann::json zones(const std::vector<ZoneState>& zones);
nlohmann::json nearMiss(const NearMissEvent& e);
nlohmann::json stopLineCrossing(const StopLineCrossing& c);
nlohmann::json health(const HealthReport& h);
nlohmann::json occlusion(const OcclusionMap& occ);
nlohmann::json calibration(const CameraModel& cam, const CalibrationResult* res, const std::string& source);

// ETSI TS 103 324 Collective Perception Message (CPM) content as JSON: perceived object container with
// positions relative to the sensor reference point (here the camera ground point), WGS84 reference
// position, speeds, accelerations and object classification. Units follow the standard's base units
// (cm, cm/s, 0.1 deg) so an ASN.1 UPER encoder can map it 1:1.
nlohmann::json cpm(const std::vector<RoadUser>& users, const CameraModel& cam, std::uint32_t station_id, double stamp,
                   double perception_range);

}  // namespace jp::msg
