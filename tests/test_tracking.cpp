#include <gtest/gtest.h>

#include <random>

#include "jp/scene/intersection.hpp"
#include "jp/tracking/tracker.hpp"

using namespace jp;

namespace {
Detection3D det(double x, double y, double yaw, float score = 0.9f, ObjectClass c = ObjectClass::Car) {
  Detection3D d;
  d.img.score = score;
  d.img.cls = c;
  d.img.box2d = {0, 0, 80, 60};
  d.bottom = {x, y, 0};
  d.yaw = yaw;
  d.dims = {4.5, 1.9, 1.5};
  d.range = std::hypot(x, y);
  d.pos_cov = Eigen::Matrix2d::Identity() * 0.09;
  d.img.cls = c;
  return d;
}
}  // namespace

TEST(Hungarian, Basic) {
  std::vector<std::vector<double>> c = {{4, 1, 3}, {2, 0, 5}, {3, 2, 2}};
  auto a = hungarian(c, 1e9);
  EXPECT_EQ(a[0], 1);
  EXPECT_EQ(a[1], 0);
  EXPECT_EQ(a[2], 2);
}

TEST(Tracker, EstimatesSpeedAndAcceleration) {
  Tracker tr;
  std::mt19937 rng(1);
  std::normal_distribution<double> noise(0, 0.25);
  const double v0 = 12.0, a = -2.5, dt = 1.0 / 15;
  for (int k = 0; k < 60; ++k) {
    const double t = k * dt;
    const double x = v0 * t + 0.5 * a * t * t;
    tr.update(t, {det(20 + x + noise(rng), 5 + noise(rng), 0.0)});
  }
  ASSERT_EQ(tr.tracks().size(), 1u);
  const auto& t = tr.tracks()[0];
  EXPECT_EQ(t.state, TrackState::Confirmed);
  const double tend = 59 * dt;
  EXPECT_NEAR(t.speed(), v0 + a * tend, 1.0);
  EXPECT_NEAR(t.accel(), a, 1.0);
}

TEST(Tracker, KeepsIdentitiesForCrossingTargets) {
  Tracker tr;
  const double dt = 0.1;
  for (int k = 0; k < 40; ++k) {
    const double t = k * dt;
    tr.update(t, {det(10 + 10 * t, 0, 0), det(30, -20 + 10 * t, M_PI / 2)});
  }
  EXPECT_EQ(tr.tracks().size(), 2u);
  for (const auto& t : tr.tracks()) EXPECT_LT(t.id, 3u);
}

TEST(Scene, EtaWithAcceleration) {
  // 50 m at 10 m/s accelerating 1 m/s^2: 0.5 t^2 + 10 t - 50 = 0 -> t = 4.142 s.
  auto t = IntersectionModel::timeToDistance(50, 10, 1);
  ASSERT_TRUE(t);
  EXPECT_NEAR(*t, 4.1421, 1e-3);
  // Braking at -2 m/s^2 from 10 m/s stops after 25 m: never reaches 50 m.
  EXPECT_FALSE(IntersectionModel::timeToDistance(50, 10, -2));
}

TEST(Scene, LaneAssignmentAndQueue) {
  SiteMap map;
  Lane l;
  l.id = "in1";
  l.centerline.pts = {{0, 0}, {100, 0}};
  l.stop_s = 90;
  map.lanes.push_back(l);
  map.addDefaultDetectors();
  IntersectionModel scene(map, nullptr);
  Tracker tr;
  for (int k = 0; k < 10; ++k) tr.update(k * 0.1, {det(86, 0.2, 0), det(78, -0.3, 0)});
  scene.update(1.0, tr.tracks());
  ASSERT_EQ(scene.users().size(), 2u);
  for (const auto& u : scene.users()) {
    EXPECT_EQ(u.lane, "in1");
    EXPECT_TRUE(u.queued);
  }
  EXPECT_EQ(scene.lanes()[0].queue_count, 2);
  bool stopbar_occupied = false;
  for (const auto& z : scene.zones())
    if (z.id == "in1_stopbar") stopbar_occupied = z.occupied;
  EXPECT_TRUE(stopbar_occupied);
}
