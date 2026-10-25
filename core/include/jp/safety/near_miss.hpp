#pragma once

#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "jp/scene/intersection.hpp"

namespace jp {

// Surrogate safety measures between two road users.
struct ConflictMetrics {
  std::optional<double> ttc;    // s, constant velocity, oriented footprints
  std::optional<double> mttc;   // s, with acceleration and turn rate (CTRA)
  std::optional<double> pet;    // s, post-encroachment time
  double drac = 0;              // m/s^2, deceleration rate to avoid the crash
  double gap = 0;               // m, current footprint distance
  double closing_speed = 0;     // m/s
  Eigen::Vector2d conflict_point{0, 0};
};

struct NearMissEvent {
  std::uint64_t event_id = 0;
  std::uint32_t a = 0, b = 0;
  ObjectClass cls_a{}, cls_b{};
  std::string type;      // rear_end | crossing | head_on | vru_vehicle | hard_braking
  std::string severity;  // critical | warning
  double start = 0, end = 0;
  std::optional<double> min_ttc, min_mttc, min_pet;
  double max_drac = 0;
  double speed_a = 0, speed_b = 0, min_accel_a = 0;
  Eigen::Vector2d point{0, 0};
  double lat = 0, lon = 0;
  bool finished = false;
};

struct NearMissConfig {
  double horizon = 5.0, step = 0.1;
  double ttc_critical = 1.5, ttc_warning = 3.0;
  double pet_critical = 1.0, pet_warning = 2.0;
  double drac_critical = 3.35;
  double hard_brake = -3.0;     // m/s^2
  double pair_range = 40.0;     // m
  double cell = 0.5;            // PET grid resolution (m)
  double footprint_shrink = 0.9;
};

class NearMissDetector {
 public:
  explicit NearMissDetector(NearMissConfig cfg = {}, const CameraModel* cam = nullptr) : cfg_(cfg), cam_(cam) {}
  void setCamera(const CameraModel* cam) { cam_ = cam; }
  // Returns events that started, escalated or finished in this frame.
  std::vector<NearMissEvent> update(double stamp, const std::vector<RoadUser>& users);
  const std::map<std::pair<std::uint32_t, std::uint32_t>, ConflictMetrics>& current() const { return current_; }

  ConflictMetrics evaluatePair(const RoadUser& a, const RoadUser& b) const;

 private:
  void updatePetGrid(double stamp, const std::vector<RoadUser>& users,
                     std::map<std::pair<std::uint32_t, std::uint32_t>, std::pair<double, Eigen::Vector2d>>& pets);
  NearMissConfig cfg_;
  const CameraModel* cam_;
  std::map<std::pair<std::uint32_t, std::uint32_t>, ConflictMetrics> current_;
  std::map<std::pair<std::uint32_t, std::uint32_t>, NearMissEvent> open_;
  std::map<std::uint32_t, NearMissEvent> braking_;
  struct Cell {
    std::uint32_t id = 0;
    double last = -1e9;
    double yaw = 0;
  };
  std::unordered_map<std::int64_t, Cell> grid_;
  std::uint64_t next_event_ = 1;
};

}  // namespace jp
