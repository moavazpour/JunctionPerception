#pragma once

#include <Eigen/Core>
#include <array>
#include <nlohmann/json_fwd.hpp>
#include <string>
#include <vector>

#include "jp/geometry/camera.hpp"
#include "jp/scene/signals.hpp"
#include "jp/scene/site_map.hpp"

namespace jp {

// Site layout drawn by the installer on the aerial map (WGS84, so it survives recalibration):
//   lanes       centreline clicked in the direction of travel + width
//   stop_lines  two points across the road; inbound lanes get their stop line from the one they cross
//   crosswalks  polygons
//   roi         polygon of the road area the system is responsible for (objects outside are ignored)
struct UserMap {
  struct LaneGeo {
    std::string id, name;
    std::vector<Eigen::Vector2d> pts;  // (lat, lon) in the direction of travel
    double width = 3.25;
  };
  std::vector<LaneGeo> lanes;
  std::vector<std::array<Eigen::Vector2d, 2>> stop_lines;
  std::vector<std::vector<Eigen::Vector2d>> crosswalks;
  std::vector<Eigen::Vector2d> roi;
  std::vector<SignalHead> signals;  // traffic lights (image boxes) and the stop line each controls

  bool empty() const { return lanes.empty() && stop_lines.empty() && crosswalks.empty() && roi.empty() && signals.empty(); }
  nlohmann::json toJson() const;
  static UserMap fromJson(const nlohmann::json& j);
  // Lanes, stop lines (stop_s), approaches, crosswalks and roi in the local frame of `anchor`.
  SiteMap build(const GeoAnchor& anchor, const std::string& name = "") const;
};

bool insidePolygon(const std::vector<Eigen::Vector2d>& poly, const Eigen::Vector2d& p);

}  // namespace jp
