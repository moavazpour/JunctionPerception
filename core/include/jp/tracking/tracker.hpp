#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <vector>

#include "jp/common/types.hpp"

namespace jp {

// Constant turn rate and acceleration (CTRA) extended Kalman filter on the ground plane.
// State: [x, y, yaw, v, yaw_rate, a].
class CtraEkf {
 public:
  using Vec = Eigen::Matrix<double, 6, 1>;
  using Mat = Eigen::Matrix<double, 6, 6>;

  struct Noise {
    double jerk = 2.0;        // m/s^3 (longitudinal acceleration change)
    double yaw_acc = 0.6;     // rad/s^2
    double pos = 0.05;        // m/sqrt(s), unmodelled lateral motion
  };

  void init(const Eigen::Vector2d& p, double yaw, const Eigen::Matrix2d& pos_cov, double yaw_sigma);
  void predict(double dt, const Noise& q);
  // Measurement of position and (optionally) yaw. Yaw is folded by pi to the closest hypothesis.
  void update(const Eigen::Vector2d& p, const Eigen::Matrix2d& pos_cov, double yaw, double yaw_sigma, bool use_yaw);
  double mahalanobis2(const Eigen::Vector2d& p, const Eigen::Matrix2d& pos_cov) const;
  void normalize();

  static Vec f(const Vec& x, double dt);

  Vec x = Vec::Zero();
  Mat P = Mat::Identity();
};

enum class TrackState : std::uint8_t { Tentative, Confirmed, Coasting };

struct Track {
  std::uint32_t id = 0;
  ObjectClass cls = ObjectClass::Car;
  std::array<float, static_cast<size_t>(ObjectClass::Count)> class_votes{};
  CtraEkf ekf;
  Eigen::Vector3d dims{4.5, 1.9, 1.6};
  TrackState state = TrackState::Tentative;
  int hits = 0;
  int misses = 0;
  double first_seen = 0, last_update = 0;
  float score = 0;
  Eigen::Vector4d last_box2d = Eigen::Vector4d::Zero();
  double last_range = 0;

  Eigen::Vector2d position() const { return ekf.x.head<2>(); }
  double yaw() const { return ekf.x[2]; }
  double speed() const { return ekf.x[3]; }
  double yawRate() const { return ekf.x[4]; }
  double accel() const { return ekf.x[5]; }
  Eigen::Vector2d velocity() const { return speed() * Eigen::Vector2d(std::cos(yaw()), std::sin(yaw())); }
  double posSigma() const { return std::sqrt(std::max(ekf.P(0, 0), ekf.P(1, 1))); }
  double speedSigma() const { return std::sqrt(ekf.P(3, 3)); }
  double accelSigma() const { return std::sqrt(ekf.P(5, 5)); }
};

struct TrackerConfig {
  float high_score = 0.40f;
  float low_score = 0.15f;
  float new_track_score = 0.45f;
  int confirm_hits = 3;
  double max_coast_s = 1.2;
  double max_coast_occluded_s = 3.0;
  double gate_chi2 = 13.8;  // 99.9 % for 2 dof
  double max_jump_vehicle = 6.0, max_jump_vru = 2.5;
};

class Tracker {
 public:
  explicit Tracker(TrackerConfig cfg = {}) : cfg_(cfg) {}

  // is_occluded(position) lets tracks survive longer behind known occluders.
  const std::vector<Track>& update(double stamp, const std::vector<Detection3D>& dets,
                                   const std::function<bool(const Eigen::Vector2d&)>& is_occluded = {});
  const std::vector<Track>& tracks() const { return tracks_; }
  void reset() { tracks_.clear(); }

  static CtraEkf::Noise noiseFor(ObjectClass c);

 private:
  void associate(std::vector<int>& track_idx, const std::vector<int>& det_idx, const std::vector<Detection3D>& dets,
                 std::vector<int>& det_to_track);
  void applyMeasurement(Track& t, const Detection3D& d, double stamp);

  TrackerConfig cfg_;
  std::vector<Track> tracks_;
  std::uint32_t next_id_ = 1;
  double last_stamp_ = -1;
};

// Rectangular assignment (rows <= cols). Returns assignment per row (-1 if cost >= inf_cost).
std::vector<int> hungarian(const std::vector<std::vector<double>>& cost, double inf_cost);

}  // namespace jp
