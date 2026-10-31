#include "jp/localization/bev_kalman.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>

namespace jp::localization {

namespace {

constexpr double kNanosecondsPerSecond = 1e9;
constexpr double kAtRestSpeedMps = 0.1;

Eigen::Matrix<double, 2, 4> observation() {
  Eigen::Matrix<double, 2, 4> h = Eigen::Matrix<double, 2, 4>::Zero();
  h(0, 0) = 1.0;
  h(1, 1) = 1.0;
  return h;
}

}  // namespace

BevKalman::BevKalman(const Eigen::Vector2d& position, std::int64_t timestamp_ns,
                     const Eigen::Matrix2d& position_cov, double acceleration_std_mps2,
                     double initial_speed_std_mps)
    : state_(Eigen::Vector4d::Zero()),
      covariance_(Eigen::Matrix4d::Zero()),
      timestamp_ns_(timestamp_ns),
      acceleration_std_(acceleration_std_mps2) {
  state_ << position, 0.0, 0.0;
  covariance_.topLeftCorner<2, 2>() = position_cov;
  covariance_.bottomRightCorner<2, 2>() =
      initial_speed_std_mps * initial_speed_std_mps * Eigen::Matrix2d::Identity();
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

double BevKalman::innovation_distance(const Eigen::Vector2d& position,
                                      const Eigen::Matrix2d& cov) const {
  const Eigen::Matrix2d s = covariance_.topLeftCorner<2, 2>() + cov;
  const Eigen::Vector2d y = position - state_.head<2>();
  return y.dot(s.llt().solve(y));
}

void BevKalman::update(const Eigen::Vector2d& position, const Eigen::Matrix2d& cov) {
  const auto h = observation();
  const Eigen::Matrix2d innovation_cov = (h * covariance_ * h.transpose()) + cov;
  const Eigen::Matrix<double, 4, 2> gain =
      covariance_ * h.transpose() * innovation_cov.llt().solve(Eigen::Matrix2d::Identity());
  state_ += gain * (position - (h * state_));
  // Joseph form keeps the covariance symmetric and positive definite.
  const Eigen::Matrix4d a = Eigen::Matrix4d::Identity() - (gain * h);
  covariance_ = (a * covariance_ * a.transpose()) + (gain * cov * gain.transpose());
}

void BevKalman::reanchor(const Eigen::Vector2d& position, const Eigen::Matrix2d& cov,
                         double speed_std_mps) {
  state_.head<2>() = position;
  covariance_.setZero();
  covariance_.topLeftCorner<2, 2>() = cov;
  covariance_.bottomRightCorner<2, 2>() =
      speed_std_mps * speed_std_mps * Eigen::Matrix2d::Identity();
}

double BevKalman::speed_std() const {
  const Eigen::Matrix2d cov = velocity_cov();
  const Eigen::Vector2d v = velocity();
  const double speed = v.norm();
  if (speed > kAtRestSpeedMps) {
    const Eigen::Vector2d d = v / speed;
    return std::sqrt(std::max(d.dot(cov * d), 0.0));
  }
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigen(cov, Eigen::EigenvaluesOnly);
  return std::sqrt(std::max(eigen.eigenvalues().maxCoeff(), 0.0));
}

}  // namespace jp::localization
