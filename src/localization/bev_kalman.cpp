#include "jp/localization/bev_kalman.hpp"

#include <Eigen/Cholesky>

namespace jp::localization {

namespace {

constexpr double kNanosecondsPerSecond = 1e9;

}  // namespace

BevKalman::BevKalman(const Eigen::Vector2d& position, std::int64_t timestamp_ns,
                     double position_std_m, double acceleration_std_mps2,
                     double initial_speed_std_mps)
    : state_(Eigen::Vector4d::Zero()),
      covariance_(Eigen::Matrix4d::Zero()),
      timestamp_ns_(timestamp_ns),
      acceleration_std_(acceleration_std_mps2) {
  state_ << position, 0.0, 0.0;
  covariance_ = Eigen::Vector4d(position_std_m * position_std_m, position_std_m * position_std_m,
                                initial_speed_std_mps * initial_speed_std_mps,
                                initial_speed_std_mps * initial_speed_std_mps)
                    .asDiagonal();
}

void BevKalman::predict(std::int64_t timestamp_ns) {
  if (timestamp_ns <= timestamp_ns_) {
    return;
  }
  const double dt = static_cast<double>(timestamp_ns - timestamp_ns_) / kNanosecondsPerSecond;
  timestamp_ns_ = timestamp_ns;
  Eigen::Matrix4d f = Eigen::Matrix4d::Identity();
  f(0, 2) = dt;
  f(1, 3) = dt;
  // Piecewise white acceleration noise per axis.
  const double q = acceleration_std_ * acceleration_std_;
  const double dt2 = dt * dt;
  Eigen::Matrix4d noise = Eigen::Matrix4d::Zero();
  for (int axis = 0; axis < 2; ++axis) {
    noise(axis, axis) = q * dt2 * dt2 / 4.0;
    noise(axis, axis + 2) = q * dt2 * dt / 2.0;
    noise(axis + 2, axis) = q * dt2 * dt / 2.0;
    noise(axis + 2, axis + 2) = q * dt2;
  }
  state_ = f * state_;
  covariance_ = (f * covariance_ * f.transpose()) + noise;
}

void BevKalman::update(const Eigen::Vector2d& position, double position_std_m) {
  Eigen::Matrix<double, 2, 4> h = Eigen::Matrix<double, 2, 4>::Zero();
  h(0, 0) = 1.0;
  h(1, 1) = 1.0;
  const Eigen::Matrix2d innovation_cov =
      (h * covariance_ * h.transpose()) +
      (position_std_m * position_std_m * Eigen::Matrix2d::Identity());
  const Eigen::Matrix<double, 4, 2> gain =
      covariance_ * h.transpose() * innovation_cov.llt().solve(Eigen::Matrix2d::Identity());
  state_ += gain * (position - (h * state_));
  covariance_ = (Eigen::Matrix4d::Identity() - (gain * h)) * covariance_;
}

}  // namespace jp::localization
