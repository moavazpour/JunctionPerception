#pragma once

#include <deque>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "jp/geometry/camera.hpp"
#include "jp/scene/site_map.hpp"
#include "jp/tracking/tracker.hpp"

namespace jp {

struct RoadUser {
  std::uint32_t id = 0;
  ObjectClass cls = ObjectClass::Car;
  TrackState state = TrackState::Tentative;
  Eigen::Vector2d pos{0, 0};
  double lat = 0, lon = 0;
  double yaw = 0, speed = 0, accel = 0, yaw_rate = 0;
  double pos_sigma = 0, speed_sigma = 0, accel_sigma = 0;
  Eigen::Vector3d dims{0, 0, 0};
  double age = 0;
  float confidence = 0;
  double range = 0;
  // Lane relation (empty lane id when not on a known lane).
  std::string lane;
  double lane_s = 0, lateral = 0;
  std::optional<double> dist_to_stop;  // front bumper to stop line, positive before the line
  std::optional<double> eta;           // s, kinematic (uses acceleration)
  std::optional<double> eta_cv;        // s, constant velocity
  double eta_sigma = 0;
  bool stopping = false;               // current deceleration stops it before the line
  std::optional<double> stop_position_s;
  bool queued = false;
  bool occluded = false;
  // Whether speed and heading are trustworthy (enough track history, small uncertainty). Untrusted values
  // are still reported but must not drive decisions (safety events, ETAs).
  bool speed_valid = false;
  bool heading_valid = false;
  std::string heading_source;  // lane | motion | hold
};

struct ZoneState {
  std::string id, type, lane;
  bool occupied = false;
  double occupancy = 0;     // fraction of time occupied over the last window
  std::uint64_t count = 0;  // vehicles entered since start
  int entered_now = 0;
  double mean_speed = 0;
  std::vector<std::uint32_t> users;
};

struct LaneState {
  std::string id;
  double queue_length = 0;  // m from stop line to tail of the last queued vehicle
  int queue_count = 0;
  std::uint64_t crossings = 0;
  double mean_speed = 0;
  int users = 0;
};

struct StopLineCrossing {
  std::uint32_t id;
  std::string lane;
  double stamp, speed, accel;
  ObjectClass cls;
};

class IntersectionModel {
 public:
  IntersectionModel(SiteMap map, const CameraModel* cam) : map_(std::move(map)), cam_(cam) {}
  void setMap(SiteMap map) { map_ = std::move(map); }
  const SiteMap& map() const { return map_; }
  void setCamera(const CameraModel* cam) { cam_ = cam; }

  void update(double stamp, const std::vector<Track>& tracks,
              const std::function<bool(const Eigen::Vector2d&)>& is_occluded = {});

  const std::vector<RoadUser>& users() const { return users_; }
  const std::vector<ZoneState>& zones() const { return zones_; }
  const std::vector<LaneState>& lanes() const { return lane_states_; }
  const std::vector<StopLineCrossing>& crossings() const { return crossings_; }  // this frame

  // Kinematic time to cover distance d (m) with speed v and acceleration a. Empty if it stops first.
  static std::optional<double> timeToDistance(double d, double v, double a);
  // Distance covered along the lane in time t with speed v and acceleration a (a braking user stops, it does not
  // reverse). The inverse of timeToDistance; ETAs and lane-following prediction share it.
  static double distanceInTime(double t, double v, double a);

 private:
  RoadUser describe(const Track& t, double stamp) const;
  // Output layer between the filter and everything downstream: heading from the drawn lane direction or the
  // motion (never the box alone), no sudden reversals, a bounded turn rate, smoothed speed and acceleration.
  void stabilize(RoadUser& u, const Track& t, double stamp);
  struct Motion {
    double heading = 0, speed = 0, accel = 0, last = -1, reverse_since = -1;
    bool init = false;
  };
  std::map<std::uint32_t, Motion> motion_;
  void assignLane(RoadUser& u, const Track& t);

  SiteMap map_;
  const CameraModel* cam_;
  std::vector<RoadUser> users_;
  std::vector<ZoneState> zones_;
  std::vector<LaneState> lane_states_;
  std::vector<StopLineCrossing> crossings_;
  std::map<std::uint32_t, std::string> lane_memory_;
  std::map<std::uint32_t, double> prev_dist_;
  std::map<std::string, std::uint64_t> crossing_count_;
  std::map<std::string, std::uint64_t> zone_count_;
  std::map<std::string, std::vector<std::uint32_t>> zone_prev_;
  std::map<std::string, std::deque<std::pair<double, bool>>> zone_hist_;
};

}  // namespace jp
