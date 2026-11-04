#pragma once

#include <Eigen/Core>
#include <cstdint>

namespace jp::localization {

/// Constant-velocity Kalman filter on the ground plane, state (x, y, vx, vy) in m and m/s.
/// Time steps follow the frame timestamps, so variable frame rates are handled.
class BevKalman {
 public:
  /// Starts at `position` (covariance `position_cov`) with unknown velocity
  /// (std `initial_speed_std_mps` per axis).
  BevKalman(const Eigen::Vector2d& position, std::int64_t timestamp_ns,
            const Eigen::Matrix2d& position_cov, double acceleration_std_mps2,
            double initial_speed_std_mps = 10.0);

  /// Predicts to `timestamp_ns` (no-op for timestamps not after the current one).
  void predict(std::int64_t timestamp_ns);

  /// Squared Mahalanobis distance of a position measurement with covariance `cov` from the
  /// predicted position (chi-square distributed with 2 degrees of freedom for inliers).
  [[nodiscard]] double innovation_distance(const Eigen::Vector2d& position,
                                           const Eigen::Matrix2d& cov) const;

  /// Fuses a position measurement with covariance `cov` (Joseph form).
  void update(const Eigen::Vector2d& position, const Eigen::Matrix2d& cov);

  /// Restarts at a measured position (covariance `cov`), keeping the velocity estimate but with
  /// std `speed_std_mps` per axis: the motion is known again only after new measurements.
  void reanchor(const Eigen::Vector2d& position, const Eigen::Matrix2d& cov, double speed_std_mps);

  [[nodiscard]] Eigen::Vector2d position() const { return state_.head<2>(); }
  [[nodiscard]] Eigen::Vector2d velocity() const { return state_.tail<2>(); }
  [[nodiscard]] Eigen::Matrix2d position_cov() const { return covariance_.topLeftCorner<2, 2>(); }
  [[nodiscard]] Eigen::Matrix2d velocity_cov() const { return covariance_.block<2, 2>(2, 2); }
  /// Speed uncertainty: velocity std along the direction of motion, or the largest velocity
  /// std while the object is nearly at rest.
  [[nodiscard]] double speed_std() const;
  [[nodiscard]] std::int64_t timestamp_ns() const { return timestamp_ns_; }

 private:
  Eigen::Vector4d state_;
  Eigen::Matrix4d covariance_;
  std::int64_t timestamp_ns_;
  double acceleration_std_;
};

}  // namespace jp::localization
