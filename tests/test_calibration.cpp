#include <gtest/gtest.h>

#include <random>

#include "jp/calib/auto_calibrator.hpp"

using namespace jp;

namespace {
// Network-like detection of a ground-truth box, with pixel noise.
std::optional<ImageDetection> observe(const CameraModel& cam, const Eigen::Vector3d& bottom, double yaw,
                                      const Eigen::Vector3d& dims, std::mt19937& rng) {
  std::normal_distribution<double> px(0, 1.0);
  auto kp = cam.groundToPixel(bottom);
  auto box = cam.projectBox(bottom, yaw, dims);
  if (!kp || !box) return std::nullopt;
  ImageDetection d;
  d.cls = ObjectClass::Car;
  d.score = 0.9f;
  d.kp = *kp + Eigen::Vector2d(px(rng), px(rng));
  d.dims = dims;
  const Eigen::Vector3d n = cam.planeCam().head<3>();
  Eigen::Vector3d r = cam.ray(*kp);
  r = (r - r.dot(n) * n).normalized();
  const Eigen::Vector3d s = n.cross(r);
  const Eigen::Vector3d h = cam.R_ground_cam().transpose() * Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0);
  d.alpha = std::atan2(h.dot(s), h.dot(r));
  double x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9;
  for (const auto& p : *box) {
    x0 = std::min(x0, p.x());
    y0 = std::min(y0, p.y());
    x1 = std::max(x1, p.x());
    y1 = std::max(y1, p.y());
  }
  d.box2d = {x0 + px(rng), y0 + px(rng), x1 + px(rng), y1 + px(rng)};
  if (x0 < 5 || y0 < 5 || x1 > cam.width() - 5 || y1 > cam.height() - 5) return std::nullopt;
  return d;
}
}  // namespace

TEST(AutoCalibration, RecoversCameraFromTraffic) {
  // A camera on a footbridge looking along a road (bearing 100 deg), like the test video.
  const double f = 2200, pitch = 9.0 * M_PI / 180, roll = 0.8 * M_PI / 180, height = 7.2, bearing = 100.0;
  const auto truth = CameraModel::fromPose(1920, 1080, f, pitch, roll, height, bearing);
  CalibrationPriors pri;
  pri.height_m = 7.0;
  pri.height_sigma = 1.5;
  pri.road_bearing_deg = 100.0;
  pri.camera_bearing_deg = 90.0;
  AutoCalibrator::Config cfg;
  cfg.min_duration_s = 30;
  cfg.min_tracks = 10;
  cfg.min_detections = 200;
  AutoCalibrator cal(1920, 1080, pri, cfg);
  std::mt19937 rng(3);
  // Road axis = camera bearing direction; vehicles in 4 lanes, both directions, 8-16 m/s.
  const double b = bearing * M_PI / 180;
  const Eigen::Vector2d ax(std::sin(b), std::cos(b)), lat(-ax.y(), ax.x());
  struct Car {
    double s0, t0, v, off;
    int dir;
  };
  std::vector<Car> cars;
  std::uniform_real_distribution<double> U(0, 1);
  for (int i = 0; i < 40; ++i)
    cars.push_back({0, i * 1.5, 8 + 8 * U(rng), (i % 4 - 1.5) * 3.4, i % 4 < 2 ? 1 : -1});
  for (double t = 0; t < 65; t += 1.0 / 10) {
    std::vector<ImageDetection> dets;
    for (const auto& c : cars) {
      if (t < c.t0) continue;
      double s = c.dir > 0 ? 20 + c.v * (t - c.t0) : 160 - c.v * (t - c.t0);
      if (s < 15 || s > 170) continue;
      const Eigen::Vector2d p = ax * s + lat * c.off;
      const double yaw = std::atan2(ax.y() * c.dir, ax.x() * c.dir);
      auto d = observe(truth, {p.x(), p.y(), 0}, yaw, {4.5, 1.85, 1.5}, rng);
      if (d) dets.push_back(*d);
    }
    cal.add(t, dets);
  }
  ASSERT_TRUE(cal.ready());
  const auto res = cal.solve();
  ASSERT_TRUE(res.ok) << res.message;
  EXPECT_NEAR(res.camera.K()(0, 0) / f, 1.0, 0.08);
  EXPECT_NEAR(res.camera.pitch() * 180 / M_PI, 9.0, 0.6);
  EXPECT_NEAR(res.camera.roll() * 180 / M_PI, 0.8, 0.6);
  EXPECT_NEAR(res.camera.cameraHeight(), height, 0.6);
  EXPECT_NEAR(std::remainder(res.camera.bearingDeg() - bearing, 360.0), 0.0, 1.0);
  // Positions agree with the truth at 80 m.
  const Eigen::Vector3d p(ax.x() * 80, ax.y() * 80, 0);
  auto px = truth.groundToPixel(p);
  auto g = res.camera.pixelToGround(*px);
  ASSERT_TRUE(g);
  EXPECT_LT((*g - p).norm(), 4.0);
}
