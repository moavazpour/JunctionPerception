#pragma once

#include <Eigen/Core>
#include <nlohmann/json_fwd.hpp>
#include <optional>
#include <string>
#include <vector>

#include "jp/geometry/bev.hpp"
#include "jp/tracking/tracker.hpp"

namespace jp {

// Lane in the local ground frame. The centreline is ordered in the direction of travel.
struct Lane {
  std::string id;
  std::string approach;           // e.g. "west"
  bool to_junction = true;        // inbound lane (has a stop line) vs outbound
  double width = 3.25;
  Polyline centerline;
  std::optional<double> stop_s;   // arc length of the stop line along the centreline
  std::string source = "config";  // config | learned | osm
};

// Lane hint for the tracker: the lane a vehicle at p heading yaw drives in. Empty off the lanes, within
// end_margin of a lane end, and where lanes of clearly different direction overlap (the junction area).
std::optional<LaneHint> laneHintAt(const std::vector<Lane>& lanes, const Eigen::Vector2d& p, double yaw,
                                   double end_margin = 2.0);

struct Zone {
  std::string id;
  std::string type = "presence";  // presence | count | crosswalk | conflict
  std::string lane;               // optional owning lane
  std::vector<Eigen::Vector2d> polygon;
};

struct SiteMap {
  std::string name;
  std::vector<Lane> lanes;
  std::vector<Zone> zones;
  std::vector<std::vector<Eigen::Vector2d>> crosswalks;
  std::vector<Eigen::Vector2d> roi;  // area the system is responsible for (occlusion grid extent)

  const Lane* lane(const std::string& id) const;
  nlohmann::json toJson() const;
  static SiteMap fromJson(const nlohmann::json& j);
  // Default stop-bar and advance detectors for every inbound lane with a stop line.
  void addDefaultDetectors(double stopbar_len = 20.0, double advance_from = 50.0, double advance_len = 10.0);
};

// Learns lanes and stop lines from traffic when no map is given (or to refine one).
//
// Vehicles moving along a dominant road axis are binned by lateral offset; histogram peaks become
// lane centrelines (curved by per-segment medians). Stop lines come from where the first vehicle of
// a queue comes to rest.
class LaneLearner {
 public:
  struct Config {
    double min_speed = 2.0;        // m/s, only moving vehicles shape lanes
    double bin = 0.25;             // lateral histogram bin (m)
    double min_lane_sep = 2.6;     // m
    double seg_len = 10.0;         // m, longitudinal segment for curved centrelines
    int min_samples = 400;
    double max_range = 250.0;
  };
  LaneLearner() = default;
  explicit LaneLearner(Config cfg) : cfg_(cfg) {}
  // axis_yaw: dominant road direction in the ground frame; junction_ahead: junction lies at +axis.
  void setAxis(double axis_yaw, bool junction_ahead) {
    axis_yaw_ = axis_yaw;
    junction_ahead_ = junction_ahead;
  }
  void addMovingSample(const Eigen::Vector2d& p, double yaw, double speed);
  void addStoppedFront(const std::string& lane_id, double s_front);
  size_t samples() const { return samples_.size(); }
  // Builds lanes; existing stop-line knowledge is kept per lane id.
  std::vector<Lane> build() const;
  std::optional<double> learnedStopLine(const std::string& lane_id) const;

 private:
  struct Sample {
    double s, t;
    int dir;  // +1 along axis, -1 against
  };
  Config cfg_;
  double axis_yaw_ = 0;
  bool junction_ahead_ = true;
  std::vector<Sample> samples_;
  std::vector<std::pair<std::string, double>> stops_;
};

}  // namespace jp
