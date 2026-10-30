#include "jp/tracking/box_kalman.hpp"

#include <Eigen/Cholesky>

namespace jp::tracking {

namespace {

constexpr double kPositionWeight = 1.0 / 20.0;
constexpr double kVelocityWeight = 1.0 / 160.0;
constexpr double kAspectStd = 1e-2;
constexpr double kAspectVelocityStd = 1e-5;
constexpr double kAspectMeasurementStd = 1e-1;

Eigen::Vector4d measurement_of(const cv::Rect2f& box) {
  return {box.x + (0.5 * box.width), box.y + (0.5 * box.height),
          box.height > 0.0F ? static_cast<double>(box.width) / box.height : 1.0, box.height};
}

}  // namespace

BoxKalman::BoxKalman(const cv::Rect2f& box) : mean_(Vector8::Zero()), covariance_(Matrix8::Zero()) {
  const Eigen::Vector4d z = measurement_of(box);
  mean_.setZero();
  mean_.head<4>() = z;
  const double h = z(3);
  Vector8 std;
  std << 2 * kPositionWeight * h, 2 * kPositionWeight * h, kAspectStd, 2 * kPositionWeight * h,
      10 * kVelocityWeight * h, 10 * kVelocityWeight * h, kAspectVelocityStd,
      10 * kVelocityWeight * h;
  covariance_ = std.array().square().matrix().asDiagonal();
}

void BoxKalman::predict() {
  const double h = mean_(3);
  Vector8 std;
  std << kPositionWeight * h, kPositionWeight * h, kAspectStd, kPositionWeight * h,
      kVelocityWeight * h, kVelocityWeight * h, kAspectVelocityStd, kVelocityWeight * h;
  Matrix8 f = Matrix8::Identity();
  f.topRightCorner<4, 4>().setIdentity();
  mean_ = f * mean_;
  covariance_ =
      (f * covariance_ * f.transpose()) + Matrix8(std.array().square().matrix().asDiagonal());
}

void BoxKalman::update(const cv::Rect2f& box) {
  const Eigen::Vector4d z = measurement_of(box);
  const double h = mean_(3);
  Eigen::Vector4d std(kPositionWeight * h, kPositionWeight * h, kAspectMeasurementStd,
                      kPositionWeight * h);
  Eigen::Matrix<double, 4, 8> projection = Eigen::Matrix<double, 4, 8>::Zero();
  projection.leftCols<4>().setIdentity();
  const Eigen::Matrix4d innovation_cov =
      (projection * covariance_ * projection.transpose()) +
      Eigen::Matrix4d(std.array().square().matrix().asDiagonal());
  const Eigen::Matrix<double, 8, 4> gain = covariance_ * projection.transpose() *
                                           innovation_cov.llt().solve(Eigen::Matrix4d::Identity());
  mean_ += gain * (z - (projection * mean_));
  covariance_ = (Matrix8::Identity() - (gain * projection)) * covariance_;
}

cv::Rect2f BoxKalman::box() const {
  const double h = mean_(3);
  const double w = mean_(2) * h;
  return {static_cast<float>(mean_(0) - (0.5 * w)), static_cast<float>(mean_(1) - (0.5 * h)),
          static_cast<float>(w), static_cast<float>(h)};
}

}  // namespace jp::tracking
