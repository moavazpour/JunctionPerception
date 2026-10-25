#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include "jp/geometry/bev.hpp"
#include "jp/geometry/camera.hpp"

using namespace jp;

namespace {
CameraModel testCamera() { return CameraModel::fromPose(1920, 1080, 1400, 12 * M_PI / 180, 1 * M_PI / 180, 7.5, 95.0); }

// Ground-truth image detection of a box seen by cam (bottom centre kp, alpha from heading).
ImageDetection synth(const CameraModel& cam, const Eigen::Vector3d& bottom, double yaw, const Eigen::Vector3d& dims) {
  ImageDetection d;
  d.kp = *cam.groundToPixel(bottom);
  d.dims = dims;
  d.score = 1;
  const Eigen::Vector3d n = cam.planeCam().head<3>();
  Eigen::Vector3d r = cam.ray(d.kp);
  r = (r - r.dot(n) * n).normalized();
  const Eigen::Vector3d s = n.cross(r);
  const Eigen::Vector3d h = cam.R_ground_cam().transpose() * Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0);
  d.alpha = std::atan2(h.dot(s), h.dot(r));
  auto px = cam.projectBox(bottom, yaw, dims);
  double x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9;
  for (const auto& p : *px) {
    x0 = std::min(x0, p.x());
    y0 = std::min(y0, p.y());
    x1 = std::max(x1, p.x());
    y1 = std::max(y1, p.y());
  }
  d.box2d = {x0, y0, x1, y1};
  return d;
}
}  // namespace

TEST(Camera, PoseRoundTrip) {
  const auto cam = testCamera();
  EXPECT_NEAR(cam.pitch(), 12 * M_PI / 180, 1e-9);
  EXPECT_NEAR(cam.roll(), 1 * M_PI / 180, 1e-9);
  EXPECT_NEAR(cam.cameraHeight(), 7.5, 1e-9);
  // Camera forward direction points along the bearing.
  const Eigen::Vector3d f = cam.cameraToGround(Eigen::Vector3d(0, 0, 10)) - cam.cameraPositionGround();
  EXPECT_NEAR(std::atan2(f.x(), f.y()) * 180 / M_PI, 95.0, 1e-6);
}

TEST(Camera, PixelGroundRoundTrip) {
  const auto cam = testCamera();
  const Eigen::Vector3d p(30, -5, 0);
  auto px = cam.groundToPixel(p);
  ASSERT_TRUE(px);
  auto g = cam.pixelToGround(*px);
  ASSERT_TRUE(g);
  EXPECT_NEAR((*g - p).norm(), 0, 1e-6);
  EXPECT_FALSE(cam.pixelToGround({960, 0}));  // above the horizon
}

TEST(Camera, LiftRecoversBox) {
  const auto cam = testCamera();
  const Eigen::Vector3d bottom(45, 3, 0), dims(4.5, 1.9, 1.5);
  const double yaw = 0.4;
  const auto d = synth(cam, bottom, yaw, dims);
  auto x = cam.lift(d);
  ASSERT_TRUE(x);
  EXPECT_NEAR((x->bottom - bottom).norm(), 0, 1e-6);
  EXPECT_NEAR(wrapAngle(x->yaw - yaw), 0, 1e-6);
  EXPECT_GT(x->pos_cov.trace(), 0);
}

TEST(Camera, JsonRoundTrip) {
  auto cam = testCamera();
  cam.anchor = {48.17, 11.55, 0, true};
  const auto j = cam.toJson();
  const auto c2 = CameraModel::fromJson(j);
  EXPECT_NEAR(c2.pitch(), cam.pitch(), 1e-9);
  EXPECT_NEAR(c2.bearingDeg(), cam.bearingDeg(), 1e-9);
  EXPECT_TRUE(c2.anchor.valid);
}

TEST(Camera, RotatedIsConsistent) {
  const auto cam = testCamera();
  const Eigen::Matrix3d R = Eigen::AngleAxisd(0.5 * M_PI / 180, Eigen::Vector3d::UnitX()).toRotationMatrix();
  const auto cam2 = cam.rotated(R);
  // A ground point seen by the rotated camera at the rotated pixel maps back to the same point.
  const Eigen::Vector3d p(40, 2, 0);
  const Eigen::Vector3d pc = cam.groundToCamera(p);
  const Eigen::Vector3d pc2 = R * pc;
  const Eigen::Vector3d uv = cam2.K() * pc2;
  auto g = cam2.pixelToGround(uv.head<2>() / uv.z());
  ASSERT_TRUE(g);
  EXPECT_NEAR((*g - p).norm(), 0, 1e-6);
}

TEST(Geo, EnuRoundTrip) {
  GeoAnchor a{48.175995, 11.555590, 0, true};
  const auto en = geoToEnu(a, 48.176500, 11.558000);
  const auto [lat, lon] = enuToGeo(a, en);
  EXPECT_NEAR(lat, 48.1765, 1e-9);
  EXPECT_NEAR(lon, 11.558, 1e-9);
  EXPECT_NEAR(en.y(), 56.2, 0.5);
}

TEST(Bev, OverlapAndDistance) {
  OrientedBox a{{0, 0}, 0, 4, 2}, b{{3, 0}, 0.3, 4, 2}, c{{10, 0}, 0, 4, 2};
  EXPECT_TRUE(overlaps(a, b));
  EXPECT_FALSE(overlaps(a, c));
  EXPECT_NEAR(distance(a, c), 6.0, 1e-9);
}

TEST(Bev, PolylineProjection) {
  Polyline l{{{0, 0}, {10, 0}, {10, 10}}};
  auto p = l.project({12, 5});
  EXPECT_NEAR(p.s, 15, 1e-9);
  EXPECT_NEAR(p.lateral, -2, 1e-9);
  EXPECT_NEAR(l.length(), 20, 1e-9);
}
