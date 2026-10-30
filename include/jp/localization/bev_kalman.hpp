#pragma once

#include <Eigen/Core>
#include <cstdint>

namespace jp::localization {

/// Constant-velocity Kalman filter on the ground plane, state (x, y, vx, vy) in m and m/s.
/// Time steps follow the frame timestamps, so variable frame rates are handled.
class BevKalman {
 public:
  /// Starts at `position` with unknown velocity (std `initial_speed_std_mps`).
  BevKalman(const Eigen::Vector2d& position, std::int64_t timestamp_ns, double position_std_m,
            double acceleration_std_mps2, double initial_speed_std_mps = 10.0);

  /// Predicts to `timestamp_ns` (no-op for timestamps not after the current one).
  void predict(std::int64_t timestamp_ns);
  /// Fuses a position measurement with isotropic std `position_std_m`.
  void update(const Eigen::Vector2d& position, double position_std_m);

  [[nodiscard]] Eigen::Vector2d position() const { return state_.head<2>(); }
  [[nodiscard]] Eigen::Vector2d velocity() const { return state_.tail<2>(); }
  [[nodiscard]] std::int64_t timestamp_ns() const { return timestamp_ns_; }

 private:
  Eigen::Vector4d state_;
  Eigen::Matrix4d covariance_;
  std::int64_t timestamp_ns_;
  double acceleration_std_;
};

}  // namespace jp::localization
