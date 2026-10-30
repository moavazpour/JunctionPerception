#pragma once

#include <Eigen/Core>
#include <opencv2/core/types.hpp>

namespace jp::tracking {

/// Constant-velocity Kalman filter on an image box (centre x, centre y, aspect ratio, height),
/// one step per frame, with noise proportional to the box height as in ByteTrack.
class BoxKalman {
 public:
  explicit BoxKalman(const cv::Rect2f& box);

  void predict();
  void update(const cv::Rect2f& box);
  [[nodiscard]] cv::Rect2f box() const;

 private:
  using Vector8 = Eigen::Matrix<double, 8, 1>;
  using Matrix8 = Eigen::Matrix<double, 8, 8>;
  Vector8 mean_;
  Matrix8 covariance_;
};

}  // namespace jp::tracking
