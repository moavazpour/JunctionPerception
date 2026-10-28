#include <gtest/gtest.h>

#include "jp/safety/near_miss.hpp"

using namespace jp;

namespace {
RoadUser user(std::uint32_t id, Eigen::Vector2d p, double yaw, double v, ObjectClass c = ObjectClass::Car) {
  RoadUser u;
  u.id = id;
  u.cls = c;
  u.state = TrackState::Confirmed;
  u.pos = p;
  u.yaw = yaw;
  u.speed = v;
  u.dims = c == ObjectClass::Pedestrian ? Eigen::Vector3d(0.6, 0.6, 1.7) : Eigen::Vector3d(4.5, 1.9, 1.5);
  return u;
}
}  // namespace

TEST(NearMiss, RearEndTtc) {
  NearMissDetector nm;
  // Follower 20 m behind at 15 m/s, leader at 5 m/s: gap 20 - 4.5 = 15.5 m at 10 m/s closing.
  const auto m = nm.evaluatePair(user(1, {0, 0}, 0, 15), user(2, {20, 0}, 0, 5));
  ASSERT_TRUE(m.ttc);
  EXPECT_NEAR(*m.ttc, 15.5 / 10.0 * 1.0, 0.25);
  EXPECT_GT(m.drac, 2.5);
}

TEST(NearMiss, CrossingPedestrianEvent) {
  NearMissDetector nm;
  std::vector<NearMissEvent> all;
  // Car heading east, pedestrian crossing north into its path; they meet near (20, 0) at t = 2 s.
  for (int k = 0; k <= 25; ++k) {
    const double t = k * 0.1;
    std::vector<RoadUser> us = {user(1, {10 * t, 0}, 0, 10), user(2, {20, -3 + 1.5 * t}, M_PI / 2, 1.5, ObjectClass::Pedestrian)};
    auto ev = nm.update(t, us);
    all.insert(all.end(), ev.begin(), ev.end());
  }
  ASSERT_FALSE(all.empty());
  EXPECT_EQ(all.front().type, "vru_vehicle");
  bool critical = false;
  for (const auto& e : all) critical |= e.severity == "critical";
  EXPECT_TRUE(critical);
}

TEST(NearMiss, ParallelTrafficIsQuiet) {
  NearMissDetector nm;
  std::vector<NearMissEvent> all;
  for (int k = 0; k < 30; ++k) {
    const double t = k * 0.1;
    auto ev = nm.update(t, {user(1, {10 * t, 0}, 0, 10), user(2, {10 * t + 2, 3.5}, 0, 10)});
    all.insert(all.end(), ev.begin(), ev.end());
  }
  EXPECT_TRUE(all.empty());
}

TEST(NearMiss, OppositeLanesOfDividedRoadAreQuiet) {
  // Divided road: eastbound lane at y = 0, westbound lane at y = 5. Both cars carry a heading error of about 9
  // degrees toward the other lane; straight-line prediction makes them meet head on, lane following does not.
  SiteMap map;
  Lane e, w;
  e.id = "east";
  e.centerline.pts = {{0, 0}, {200, 0}};
  w.id = "west";
  w.centerline.pts = {{200, 5}, {0, 5}};
  map.lanes = {e, w};
  auto run = [&](bool lanes) {
    NearMissDetector nm;
    if (lanes) nm.setMap(&map);
    std::vector<NearMissEvent> all;
    for (int k = 0; k <= 50; ++k) {
      const double t = k * 0.1;
      RoadUser a = user(1, {60 + 15 * t, 0}, 0.15, 15), b = user(2, {140 - 15 * t, 5}, M_PI + 0.15, 15);
      a.lane = "east";
      a.lane_s = a.pos.x();
      b.lane = "west";
      b.lane_s = 200 - b.pos.x();
      auto ev = nm.update(t, {a, b});
      all.insert(all.end(), ev.begin(), ev.end());
    }
    return all;
  };
  const auto straight = run(false);
  ASSERT_FALSE(straight.empty());
  EXPECT_EQ(straight.front().type, "head_on");
  EXPECT_TRUE(run(true).empty());
}

TEST(NearMiss, LaneFollowingStillSeesRearEnd) {
  // Same lane, fast follower behind a slow leader: lane-following prediction keeps the rear-end conflict.
  SiteMap map;
  Lane l;
  l.id = "l";
  l.centerline.pts = {{0, 0}, {200, 0}};
  map.lanes = {l};
  NearMissDetector nm;
  nm.setMap(&map);
  RoadUser a = user(1, {0, 0}, 0.05, 15), b = user(2, {20, 0}, 0, 5);
  a.lane = b.lane = "l";
  a.lane_s = 0;
  b.lane_s = 20;
  const auto m = nm.evaluatePair(a, b);
  ASSERT_TRUE(m.ttc);
  EXPECT_NEAR(*m.ttc, 1.55, 0.25);
}
