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

namespace {
Lane straightLane(const std::string& id, Eigen::Vector2d a, Eigen::Vector2d b) {
  Lane l;
  l.id = id;
  l.centerline.pts = {a, b};
  l.width = 3.5;
  return l;
}
}  // namespace

TEST(Tracker, LaneConstraintSettlesNoisyHeading) {
  // Car driving along a straight lane at 0.3 rad with very noisy box orientation.
  const double th = 0.3;
  const Eigen::Vector2d u(std::cos(th), std::sin(th));
  const std::vector<Lane> lanes = {straightLane("l1", {0, 0}, 200 * u)};
  auto run = [&](bool hint) {
    Tracker tr;
    if (hint) tr.setLaneHint([&](const Eigen::Vector2d& p, double yaw) { return laneHintAt(lanes, p, yaw); });
    std::mt19937 rng(7);
    std::normal_distribution<double> pos(0, 0.3), yaw(0, 0.35);
    double err = 0;
    int n = 0;
    for (int k = 0; k < 60; ++k) {
      const double t = k * 0.1;
      const Eigen::Vector2d p = (20 + 12 * t) * u + Eigen::Vector2d(pos(rng), pos(rng));
      tr.update(t, {det(p.x(), p.y(), th + yaw(rng))});
      if (k >= 40) {
        EXPECT_EQ(tr.tracks().size(), 1u);
        err += std::abs(wrapAngle(tr.tracks()[0].yaw() - th));
        ++n;
      }
    }
    return err / n;
  };
  const double with_lane = run(true), without = run(false);
  EXPECT_LT(with_lane, 1.5 * M_PI / 180);
  EXPECT_LT(with_lane, without);
}

TEST(Tracker, LaneConstraintStillFollowsLaneChange) {
  // Two lanes heading east, 3.5 m apart. The car changes from the right lane to the left one over 4 s.
  const std::vector<Lane> lanes = {straightLane("r", {0, 0}, {300, 0}), straightLane("l", {0, 3.5}, {300, 3.5})};
  Tracker tr;
  tr.setLaneHint([&](const Eigen::Vector2d& p, double yaw) { return laneHintAt(lanes, p, yaw); });
  std::mt19937 rng(3);
  std::normal_distribution<double> pos(0, 0.2), yaw(0, 0.15);
  std::uint32_t id = 0;
  double max_err = 0;
  for (int k = 0; k < 120; ++k) {
    const double t = k * 0.1;
    const double x = 20 + 12 * t;
    const double ph = std::clamp((t - 3.0) / 4.0, 0.0, 1.0);
    const double y = 1.75 * (1 - std::cos(M_PI * ph));
    const double vy = (t > 3.0 && t < 7.0) ? 1.75 * M_PI / 4 * std::sin(M_PI * ph) : 0.0;
    tr.update(t, {det(x + pos(rng), y + pos(rng), std::atan2(vy, 12.0) + yaw(rng))});
    ASSERT_EQ(tr.tracks().size(), 1u) << "t=" << t;
    if (k == 0) id = tr.tracks()[0].id;
    EXPECT_EQ(tr.tracks()[0].id, id);
    if (t > 1.0) max_err = std::max(max_err, std::abs(tr.tracks()[0].position().y() - y));
  }
  const Track& t = tr.tracks()[0];
  EXPECT_LT(max_err, 1.0);
  EXPECT_NEAR(t.position().y(), 3.5, 0.3);
  EXPECT_LT(std::abs(wrapAngle(t.yaw())), 3 * M_PI / 180);
}

TEST(SiteMap, LaneHintSkipsEndsAndJunctions) {
  // Eastbound lane, plus a northbound lane crossing it at x = 50 (junction area).
  const std::vector<Lane> lanes = {straightLane("e", {0, 0}, {100, 0}), straightLane("n", {50, -50}, {50, 50})};
  auto h = laneHintAt(lanes, {20, 0.4}, 0.1);
  ASSERT_TRUE(h);
  EXPECT_NEAR(h->heading, 0, 1e-9);
  EXPECT_NEAR(h->lateral, 0.4, 1e-9);
  EXPECT_FALSE(laneHintAt(lanes, {20, 0.4}, M_PI));   // driving against the lane
  EXPECT_FALSE(laneHintAt(lanes, {99, 0}, 0));        // lane end
  EXPECT_FALSE(laneHintAt(lanes, {50, 0}, M_PI / 4));  // both lanes match: ambiguous
  EXPECT_FALSE(laneHintAt(lanes, {20, 3}, 0));        // outside the lane
}
