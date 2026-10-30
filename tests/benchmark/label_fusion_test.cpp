#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <random>

#include "jp/benchmark/label_merge.hpp"
#include "jp/benchmark/label_registration.hpp"

namespace {

using jp::benchmark::Cuboid;
using jp::benchmark::PlanarPose;
using FramePairs = std::vector<std::pair<std::vector<Cuboid>, std::vector<Cuboid>>>;

Cuboid make_box(const std::string& id, const std::string& category, double x, double y, double yaw,
                double length, int points) {
  Cuboid box;
  box.id = id;
  box.category = category;
  box.center = {x, y, -6.5};
  box.rotation = Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  box.size = {length, 1.9, 1.5};
  box.num_points = points;
  return box;
}

// Vehicles driving through a scene seen by both label sets, plus objects only one set has.
// North boxes are the south boxes expressed in a frame rotated and shifted by `truth`.
FramePairs synthetic_frames(const PlanarPose& truth, double noise_m) {
  std::mt19937 rng(7);
  std::normal_distribution<double> noise(0.0, noise_m);
  const PlanarPose north_from_south{
      -truth.yaw_rad,
      -(Eigen::AngleAxisd(-truth.yaw_rad, Eigen::Vector3d::UnitZ()) * truth.translation)};
  FramePairs frames;
  for (int f = 0; f < 40; ++f) {
    std::vector<Cuboid> south;
    std::vector<Cuboid> north;
    for (int v = 0; v < 6; ++v) {
      const double x = -40.0 + (12.0 * v) + (0.8 * f);
      const double y = (v % 2 == 0 ? -4.0 : 4.0) + (0.3 * v);
      const std::string category = v == 5 ? "TRUCK" : "CAR";
      const double length = category == "TRUCK" ? 9.0 : 4.5;
      const Cuboid s = make_box("s" + std::to_string(v), category, x, y, 0.1 * v, length, 100);
      south.push_back(s);
      Cuboid n = north_from_south.apply(s);
      n.id = "n" + std::to_string(v);
      n.center.x() += noise(rng);
      n.center.y() += noise(rng);
      n.num_points = v == 0 ? 500 : 50;
      north.push_back(n);
    }
    south.push_back(make_box("s-only", "PEDESTRIAN", 5.0, 20.0, 0.0, 0.6, 20));
    north.push_back(make_box("n-only", "BICYCLE", 30.0, 30.0, 0.0, 1.8, 20));
    frames.emplace_back(std::move(south), std::move(north));
  }
  return frames;
}

TEST(LabelRegistration, RecoversPlanarTransform) {
  const PlanarPose truth{0.286, Eigen::Vector3d(1.35, -13.68, 0.29)};
  const auto result = jp::benchmark::register_label_sets(synthetic_frames(truth, 0.05));
  EXPECT_NEAR(result.south_from_north.yaw_rad, truth.yaw_rad, 2e-3);
  EXPECT_TRUE(result.south_from_north.translation.isApprox(truth.translation, 1e-2));
  EXPECT_EQ(result.matched_pairs, 40U * 6U);
  EXPECT_LT(result.median_bev_error_m, 0.1);
}

TEST(LabelRegistration, MatchBoxesRespectsCategoryAndGate) {
  const std::vector<Cuboid> south = {make_box("a", "CAR", 0, 0, 0, 4.5, 1),
                                     make_box("b", "PEDESTRIAN", 10, 0, 0, 0.5, 1)};
  const std::vector<Cuboid> north = {make_box("c", "TRUCK", 0.2, 0, 0, 9, 1),
                                     make_box("d", "PEDESTRIAN", 10.5, 0, 0, 0.5, 1),
                                     make_box("e", "CAR", 5, 0, 0, 4.5, 1)};
  const auto matches = jp::benchmark::match_boxes(south, north, 2.0);
  ASSERT_EQ(matches.size(), 1U);
  EXPECT_EQ(matches.front(), (std::pair<std::size_t, std::size_t>{1, 1}));
}

TEST(LabelMerge, KeepsBothSetsAndUnifiesTracks) {
  const PlanarPose truth{0.286, Eigen::Vector3d(1.35, -13.68, 0.29)};
  const auto frames = synthetic_frames(truth, 0.0);
  const auto merged = jp::benchmark::merge_label_sets(frames, truth);

  // 6 shared vehicles + 1 south-only pedestrian + 1 north-only bicycle per frame.
  EXPECT_EQ(merged.statistics.merged_boxes, 40U * 8U);
  EXPECT_EQ(merged.statistics.matched_boxes, 40U * 6U);
  EXPECT_EQ(merged.statistics.tracks, 8U);
  for (const auto& frame : merged.frames) {
    ASSERT_EQ(frame.size(), 8U);
    for (const auto& object : frame) {
      if (object.box.category == "CAR" && object.box.num_points == 500) {
        // The north box observed vehicle 0 with more points, so it is kept unchanged.
        EXPECT_EQ(object.source, jp::benchmark::LabelSource::kBoth);
        EXPECT_EQ(object.box.id, "n0");
      }
    }
  }
  // A track keeps its ID whichever label set supplies the box.
  EXPECT_EQ(merged.frames.front().front().track_id, merged.frames.back().front().track_id);
}

}  // namespace
