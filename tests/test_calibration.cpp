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

namespace {
// Keypoint-detector-like observation: bottom centre and the 8 cuboid corners, with pixel noise.
std::optional<ImageDetection> observeKp(const CameraModel& cam, const Eigen::Vector3d& bottom, double yaw,
                                        const Eigen::Vector3d& dims, double noise, std::mt19937& rng) {
  std::normal_distribution<double> px(0, noise);
  auto kp = cam.groundToPixel(bottom);
  auto box = cam.projectBox(bottom, yaw, dims);
  if (!kp || !box) return std::nullopt;
  ImageDetection d;
  d.cls = ObjectClass::Car;
  d.score = 0.9f;
  d.has_kpts = true;
  d.kpts[0] = {kp->x() + px(rng), kp->y() + px(rng), 0.95};
  double x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9;
  for (int i = 0; i < 8; ++i) {
    const auto& p = (*box)[i];
    d.kpts[1 + i] = {p.x() + px(rng), p.y() + px(rng), 0.9};
    x0 = std::min(x0, p.x());
    y0 = std::min(y0, p.y());
    x1 = std::max(x1, p.x());
    y1 = std::max(y1, p.y());
  }
  d.kp = d.kpts[0].head<2>();
  d.dims = classPriorDims(ObjectClass::Car);
  d.box2d = {x0, y0, x1, y1};
  if (x0 < 5 || y0 < 5 || x1 > cam.width() - 5 || y1 > cam.height() - 5) return std::nullopt;
  return d;
}

struct KpCase {
  int w, h;
  double f, pitch_deg, roll_deg, height, bearing, noise, pos_tol, pitch_tol;
};

void runKeypointCase(const KpCase& k) {
  const auto truth = CameraModel::fromPose(k.w, k.h, k.f, k.pitch_deg * M_PI / 180, k.roll_deg * M_PI / 180, k.height,
                                           k.bearing);
  CalibrationPriors pri;
  pri.height_m = k.height + 1.0;  // the installer's guess is off by a metre
  pri.height_sigma = 1.5;
  pri.road_bearing_deg = k.bearing + 5.0;
  pri.camera_bearing_deg = k.bearing - 10.0;
  AutoCalibrator::Config cfg;
  cfg.min_duration_s = 30;
  cfg.min_tracks = 10;
  cfg.min_detections = 200;
  AutoCalibrator cal(k.w, k.h, pri, cfg);
  std::mt19937 rng(11);
  std::uniform_real_distribution<double> U(0, 1);
  // Road at 5 deg to the viewing direction, 4 lanes, both directions; car sizes vary like real traffic.
  const double b = (k.bearing + 5.0) * M_PI / 180;
  const Eigen::Vector2d ax(std::sin(b), std::cos(b)), lat(-ax.y(), ax.x());
  struct Car {
    double t0, v, off;
    int dir;
    Eigen::Vector3d dims;
  };
  std::vector<Car> cars;
  for (int i = 0; i < 50; ++i)
    cars.push_back({i * 1.2, 8 + 8 * U(rng), (i % 4 - 1.5) * 3.4, i % 4 < 2 ? 1 : -1,
                    {4.1 + 0.8 * U(rng), 1.75 + 0.2 * U(rng), 1.45 + 0.25 * U(rng)}});
  for (double t = 0; t < 70; t += 1.0 / 10) {
    std::vector<ImageDetection> dets;
    for (const auto& c : cars) {
      if (t < c.t0) continue;
      const double s = c.dir > 0 ? 12 + c.v * (t - c.t0) : 120 - c.v * (t - c.t0);
      if (s < 10 || s > 125) continue;
      const Eigen::Vector2d p = ax * s + lat * c.off;
      const double yaw = std::atan2(ax.y() * c.dir, ax.x() * c.dir);
      auto d = observeKp(truth, {p.x(), p.y(), 0}, yaw, c.dims, k.noise, rng);
      if (d) dets.push_back(*d);
    }
    cal.add(t, dets);
  }
  ASSERT_TRUE(cal.ready());
  const auto res = cal.solve();
  ASSERT_TRUE(res.ok) << res.message;
  EXPECT_EQ(res.method, "keypoints");
  EXPECT_NEAR(res.camera.K()(0, 0) / k.f, 1.0, 0.06);
  EXPECT_NEAR(res.camera.pitch() * 180 / M_PI, k.pitch_deg, k.pitch_tol);
  EXPECT_NEAR(res.camera.roll() * 180 / M_PI, k.roll_deg, 0.6);
  EXPECT_NEAR(res.camera.cameraHeight() / k.height, 1.0, 0.06);
  EXPECT_NEAR(std::remainder(res.camera.bearingDeg() - k.bearing, 360.0), 0.0, 1.5);
  EXPECT_GT(res.confidence, 0.6);
  // Positions agree with the truth along the road.
  for (double s : {30.0, 60.0, 100.0}) {
    const Eigen::Vector3d p(ax.x() * s, ax.y() * s, 0);
    auto px = truth.groundToPixel(p);
    ASSERT_TRUE(px);
    auto g = res.camera.pixelToGround(*px);
    ASSERT_TRUE(g);
    EXPECT_LT((*g - p).norm(), k.pos_tol * s) << "at " << s << " m";
  }
}
}  // namespace

TEST(AutoCalibration, KeypointsFootbridgePhone) { runKeypointCase({1920, 1080, 1650, 10.0, 0.8, 7.0, 95.0, 1.5, 0.04, 0.6}); }

TEST(AutoCalibration, KeypointsGantryCamera) { runKeypointCase({1920, 1200, 1350, 18.0, -1.2, 8.6, 200.0, 2.0, 0.04, 1.0}); }

TEST(AutoCalibration, KeypointsWideAngleLowMount) { runKeypointCase({1280, 720, 700, 25.0, 2.0, 5.0, 10.0, 1.0, 0.08, 0.6}); }

#include "jp/calib/manual_calibration.hpp"

TEST(ManualCalibration, RecoversCameraAndPositionFromMapPoints) {
  const GeoAnchor configured{48.175995, 11.555590, 0, true};
  // The real camera stands 3 m east, 2 m south of the configured position.
  const auto [lat0, lon0] = enuToGeo(configured, Eigen::Vector2d(3.0, -2.0));
  auto truth = CameraModel::fromPose(1920, 1080, 1700, 11.0 * M_PI / 180, -0.7 * M_PI / 180, 7.3, 92.0);
  truth.anchor = {lat0, lon0, 0, true};
  std::vector<GroundCorrespondence> pts;
  std::mt19937 rng(5);
  std::normal_distribution<double> px(0, 1.0), m(0, 0.15);  // click noise, map-reading noise
  for (const Eigen::Vector2d g : {Eigen::Vector2d(25, 4), Eigen::Vector2d(30, -5), Eigen::Vector2d(55, 6),
                                  Eigen::Vector2d(60, -7), Eigen::Vector2d(90, 3), Eigen::Vector2d(110, -4)}) {
    // Ground points along the viewing direction (bearing 92 deg), in the camera's ENU frame.
    const double b = 92.0 * M_PI / 180;
    const Eigen::Vector2d e = g.x() * Eigen::Vector2d(std::sin(b), std::cos(b)) + g.y() * Eigen::Vector2d(-std::cos(b), std::sin(b));
    auto uv = truth.groundToPixel({e.x(), e.y(), 0});
    ASSERT_TRUE(uv);
    const auto [la, lo] = enuToGeo(truth.anchor, e + Eigen::Vector2d(m(rng), m(rng)));
    pts.push_back({*uv + Eigen::Vector2d(px(rng), px(rng)), la, lo});
  }
  const auto r = calibrateFromCorrespondences(1920, 1080, pts, configured);
  ASSERT_TRUE(r.ok) << r.message;
  EXPECT_NEAR(r.camera.K()(0, 0) / 1700, 1.0, 0.08);
  EXPECT_NEAR(r.camera.pitch() * 180 / M_PI, 11.0, 0.5);
  EXPECT_NEAR(r.camera.roll() * 180 / M_PI, -0.7, 0.5);
  EXPECT_NEAR(r.camera.cameraHeight(), 7.3, 0.5);
  EXPECT_NEAR(std::remainder(r.camera.bearingDeg() - 92.0, 360.0), 0.0, 1.0);
  EXPECT_LT((geoToEnu(configured, r.camera.anchor.lat, r.camera.anchor.lon) - Eigen::Vector2d(3.0, -2.0)).norm(), 1.5);
  EXPECT_LT(r.rms_px, 4.0);
}

TEST(ManualCalibration, RejectsTooFewPoints) {
  const auto r = calibrateFromCorrespondences(1920, 1080, {{{100, 500}, 48.1, 11.5}}, {48.1, 11.5, 0, true});
  EXPECT_FALSE(r.ok);
}

namespace {
// Ground point at distance `fwd` along the camera bearing and `lat` to its left, in the camera's ENU frame.
Eigen::Vector2d alongBearing(double bearing_deg, double fwd, double lat) {
  const double b = bearing_deg * M_PI / 180;
  return fwd * Eigen::Vector2d(std::sin(b), std::cos(b)) + lat * Eigen::Vector2d(-std::cos(b), std::sin(b));
}

// Median ground error (m) of the solved camera vs the truth over pixels that see the road 15-90 m away.
double groundMappingError(const CameraModel& truth, const CameraModel& solved, const GeoAnchor& configured) {
  const Eigen::Vector2d off = geoToEnu(configured, truth.anchor.lat, truth.anchor.lon) -
                              geoToEnu(configured, solved.anchor.lat, solved.anchor.lon);
  std::vector<double> e;
  for (int v = 0; v < truth.height(); v += 40)
    for (int u = 0; u < truth.width(); u += 80) {
      const auto g = truth.pixelToGround({u, v});
      if (!g || g->head<2>().norm() < 15 || g->head<2>().norm() > 90) continue;
      const auto s = solved.pixelToGround({u, v});
      if (!s) {
        e.push_back(99);
        continue;
      }
      e.push_back((g->head<2>() + off - s->head<2>()).norm());
    }
  std::nth_element(e.begin(), e.begin() + e.size() / 2, e.end());
  return e.empty() ? 99 : e[e.size() / 2];
}
}  // namespace

// The case that failed on a real site: two points near, two far (behind the stop line), unknown lens.
TEST(ManualCalibration, FourPointsNearAndFarWithHeightPrior) {
  const GeoAnchor configured{48.175995, 11.555590, 0, true};
  auto truth = CameraModel::fromPose(1920, 1080, 1500, 9.0 * M_PI / 180, 0.6 * M_PI / 180, 8.0, 90.0);
  truth.anchor = configured;
  std::mt19937 rng(11);
  std::normal_distribution<double> px(0, 1.5), m(0, 0.3);
  ManualCalibrationInput in;
  in.width = 1920;
  in.height = 1080;
  in.anchor = configured;
  in.height_m = 7.0;
  in.height_sigma = 1.5;
  for (const auto& g : {alongBearing(90, 22, 4), alongBearing(90, 24, -3), alongBearing(90, 68, 5), alongBearing(90, 71, -4)}) {
    auto uv = truth.groundToPixel({g.x(), g.y(), 0});
    ASSERT_TRUE(uv);
    const auto [la, lo] = enuToGeo(configured, g + Eigen::Vector2d(m(rng), m(rng)));
    in.points.push_back({*uv + Eigen::Vector2d(px(rng), px(rng)), la, lo});
  }
  const auto r = calibrateManual(in);
  ASSERT_TRUE(r.plausible) << r.message;
  EXPECT_NEAR(r.focal_px / 1500, 1.0, 0.3) << r.message;
  EXPECT_LT(groundMappingError(truth, r.camera, configured), 1.5) << r.message;
  EXPECT_FALSE(r.accuracy.empty());
}

TEST(ManualCalibration, TracedLanesTightenTheFocalLength) {
  const GeoAnchor configured{48.175995, 11.555590, 0, true};
  auto truth = CameraModel::fromPose(1920, 1080, 1500, 9.0 * M_PI / 180, 0.6 * M_PI / 180, 8.0, 90.0);
  truth.anchor = configured;
  std::mt19937 rng(12);
  std::normal_distribution<double> px(0, 1.5), m(0, 0.3);
  ManualCalibrationInput in;
  in.width = 1920;
  in.height = 1080;
  in.anchor = configured;
  in.height_m = 7.0;
  in.height_sigma = 1.5;
  for (const auto& g : {alongBearing(90, 22, 4), alongBearing(90, 24, -3), alongBearing(90, 68, 5), alongBearing(90, 71, -4)}) {
    auto uv = truth.groundToPixel({g.x(), g.y(), 0});
    const auto [la, lo] = enuToGeo(configured, g + Eigen::Vector2d(m(rng), m(rng)));
    in.points.push_back({*uv + Eigen::Vector2d(px(rng), px(rng)), la, lo});
  }
  // Two lane lines traced from 15 to 110 m; on the map two points each (anywhere along the line).
  for (double lat : {-1.6, 1.6}) {
    LineCorrespondence l;
    for (double f = 15; f <= 110; f += 12) l.px.push_back(*truth.groundToPixel({alongBearing(90, f, lat).x(), alongBearing(90, f, lat).y(), 0}) +
                                                         Eigen::Vector2d(px(rng), px(rng)));
    const auto [a1, o1] = enuToGeo(configured, alongBearing(90, 30, lat) + Eigen::Vector2d(m(rng), m(rng)));
    const auto [a2, o2] = enuToGeo(configured, alongBearing(90, 95, lat) + Eigen::Vector2d(m(rng), m(rng)));
    l.geo = {Eigen::Vector2d(a1, o1), Eigen::Vector2d(a2, o2)};
    in.lines.push_back(l);
  }
  const auto r = calibrateManual(in);
  ASSERT_TRUE(r.ok) << r.message;
  EXPECT_NEAR(r.focal_px / 1500, 1.0, 0.12) << r.message;
  EXPECT_NEAR(r.camera.pitch() * 180 / M_PI, 9.0, 0.6);
  EXPECT_LT(groundMappingError(truth, r.camera, configured), 1.2) << r.message;
}

TEST(ManualCalibration, FlagsAWrongPair) {
  const GeoAnchor configured{48.175995, 11.555590, 0, true};
  auto truth = CameraModel::fromPose(1920, 1080, 1500, 10.0 * M_PI / 180, 0, 7.0, 0.0);
  truth.anchor = configured;
  ManualCalibrationInput in;
  in.width = 1920;
  in.height = 1080;
  in.anchor = configured;
  in.height_m = 7.0;
  int k = 0;
  for (const auto& g : {alongBearing(0, 20, 4), alongBearing(0, 25, -4), alongBearing(0, 40, 6), alongBearing(0, 45, -5),
                        alongBearing(0, 70, 3), alongBearing(0, 80, -6), alongBearing(0, 60, 0)}) {
    auto uv = truth.groundToPixel({g.x(), g.y(), 0});
    // The last pair is clicked 6 m off on the map.
    const auto [la, lo] = enuToGeo(configured, g + (++k == 7 ? Eigen::Vector2d(6, 0) : Eigen::Vector2d(0, 0)));
    in.points.push_back({*uv, la, lo});
  }
  const auto r = calibrateManual(in);
  ASSERT_TRUE(r.plausible) << r.message;
  bool flagged = false;
  for (const auto& w : r.warnings) flagged |= w.find("point 7") != std::string::npos;
  EXPECT_TRUE(flagged) << r.message;
}

#include <nlohmann/json.hpp>

#include "jp/scene/user_map.hpp"

TEST(UserMap, InboundLaneGetsStopLineAndApproach) {
  const GeoAnchor a{48.1, 11.5, 0, true};
  UserMap um;
  auto geo = [&](double x, double y) {
    const auto [la, lo] = enuToGeo(a, {x, y});
    return Eigen::Vector2d(la, lo);
  };
  um.lanes.push_back({"in1", "", {geo(-60, 0), geo(-20, 0), geo(-5, 0)}, 3.3});
  um.lanes.push_back({"out1", "", {geo(5, 3), geo(60, 3)}, 3.3});
  um.stop_lines.push_back({geo(-8, -2), geo(-8, 2)});
  const SiteMap m = um.build(a);
  ASSERT_EQ(m.lanes.size(), 2u);
  EXPECT_TRUE(m.lanes[0].to_junction);
  ASSERT_TRUE(m.lanes[0].stop_s);
  EXPECT_NEAR(*m.lanes[0].stop_s, 52.0, 0.2);
  EXPECT_EQ(m.lanes[0].approach, "west");
  EXPECT_FALSE(m.lanes[1].to_junction);
  const auto j = um.toJson();
  EXPECT_EQ(UserMap::fromJson(j).lanes.size(), 2u);
}

#include "jp/calib/refiner.hpp"

TEST(CalibrationRefiner, ProposesTheMissingPitch) {
  auto truth = CameraModel::fromPose(1920, 1080, 1500, 9.0 * M_PI / 180, 0.0, 8.0, 90.0);
  truth.anchor = {48.1, 11.5, 0, true};
  // The calibration is 1.2 deg off in pitch.
  auto wrong = CameraModel::fromPose(1920, 1080, 1500, 10.2 * M_PI / 180, 0.0, 8.0, 90.0);
  wrong.anchor = truth.anchor;
  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> pts;
  for (const auto& g : {alongBearing(90, 20, 4), alongBearing(90, 30, -4), alongBearing(90, 60, 5), alongBearing(90, 70, -5)})
    pts.emplace_back(*truth.groundToPixel({g.x(), g.y(), 0}), g);
  CalibrationRefiner ref;
  ref.reset(wrong, pts);
  std::uint32_t id = 1;
  for (double lat : {-7.0, -3.5, 0.0, 3.5, 7.0})
    for (int k = 0; k < 10; ++k, ++id)
      for (double f = 15 + k; f < 110; f += 2.5) {
        const auto g = alongBearing(90, f, lat + 0.3 * std::sin(k));
        ref.add(id, f, *truth.groundToPixel({g.x(), g.y(), 0}));
      }
  ASSERT_TRUE(ref.ready());
  const auto p = ref.solve();
  ASSERT_TRUE(p.has_value()) << "spread " << ref.currentSpread() << " tracks " << ref.usableTracks();
  EXPECT_LT(p->spread_after_deg, 0.5 * p->spread_before_deg);
  EXPECT_NEAR(p->camera.pitch() * 180 / M_PI, 9.0, 0.5);
}

#include <opencv2/imgproc.hpp>

#include "jp/scene/signals.hpp"

TEST(Signals, ReadsTheLitLamp) {
  auto head = [](cv::Scalar bgr, int lamp) {
    cv::Mat m(60, 24, CV_8UC3, cv::Scalar(25, 25, 25));
    cv::circle(m, cv::Point(12, 10 + lamp * 20), 7, bgr, cv::FILLED);
    return m;
  };
  EXPECT_EQ(SignalReader::classify(head({40, 40, 255}, 0)).state, "red");
  EXPECT_EQ(SignalReader::classify(head({0, 190, 255}, 1)).state, "amber");
  EXPECT_EQ(SignalReader::classify(head({120, 230, 40}, 2)).state, "green");
  EXPECT_EQ(SignalReader::classify(cv::Mat(60, 24, CV_8UC3, cv::Scalar(20, 20, 20))).state, "off");
}
