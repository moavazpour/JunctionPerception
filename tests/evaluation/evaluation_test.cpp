#include "jp/evaluation/evaluation.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

namespace {

using jp::evaluation::Frame;
using jp::evaluation::PredictedObject;
using jp::evaluation::Sequence;
using jp::evaluation::TruthObject;

constexpr std::int64_t kFrameNs = 100'000'000;  // 10 Hz

TruthObject truth(std::int64_t track, bool vru, double x, double y, bool visible = true) {
  TruthObject t;
  t.track_id = track;
  t.twin_class = vru ? "pedestrian" : "car";
  t.vru = vru;
  t.position = {x, y};
  t.length_m = vru ? 0.6 : 4.5;
  t.width_m = vru ? 0.6 : 1.8;
  t.visible = visible;
  return t;
}

PredictedObject predicted(std::int64_t id, bool vru, double x, double y) {
  PredictedObject p;
  p.id = id;
  p.twin_class = vru ? "pedestrian" : "car";
  p.vru = vru;
  p.position = {x, y};
  return p;
}

Sequence one_frame(std::vector<TruthObject> labels, std::vector<PredictedObject> predictions) {
  Sequence s;
  s.frames.push_back({0, 0, std::move(labels), std::move(predictions)});
  return s;
}

TEST(Evaluation, MatchesWithinClassGatesAndCountsDetections) {
  // Car matched 0.5 m off; the pedestrian prediction is 1.5 m off (beyond the 1 m VRU gate);
  // one prediction where nothing is labelled.
  const auto s = one_frame(
      {truth(1, false, 0, 0), truth(2, true, 10, 0)},
      {predicted(7, false, 0.5, 0), predicted(8, true, 10, 1.5), predicted(9, false, 50, 0)});
  const auto m = jp::evaluation::evaluate({&s}, {});
  EXPECT_EQ(m.truth, 2U);
  EXPECT_EQ(m.matched, 1U);
  EXPECT_EQ(m.false_positives, 2U);
  EXPECT_NEAR(m.recall(), 0.5, 1e-12);
  EXPECT_NEAR(m.precision(), 1.0 / 3.0, 1e-12);
  EXPECT_NEAR(m.position_m.median, 0.5, 1e-12);
  EXPECT_EQ(m.by_class.at("pedestrian").matched, 0U);
  EXPECT_EQ(m.confusion.at("car").at("car"), 1U);
}

TEST(Evaluation, DisplacedObjectsAreSeparatedFromMissesAndFalsePositives) {
  // A car placed 3 m off (beyond the 2 m gate, within 5 m) and one 20 m from any label.
  const auto s =
      one_frame({truth(1, false, 0, 0)}, {predicted(7, false, 3, 0), predicted(8, false, 20, 0)});
  const auto m = jp::evaluation::evaluate({&s}, {});
  EXPECT_EQ(m.matched, 0U);
  EXPECT_EQ(m.false_positives, 2U);
  EXPECT_EQ(m.displaced, 1U);
  EXPECT_NEAR(m.displaced_m.median, 3.0, 1e-12);
}

TEST(Evaluation, LabelsOutsideTheImageAreDontCare) {
  const auto s = one_frame({truth(1, false, 0, 0, false)}, {predicted(7, false, 0.2, 0)});
  const auto m = jp::evaluation::evaluate({&s}, {});
  EXPECT_EQ(m.truth, 0U);
  EXPECT_EQ(m.false_positives, 0U);
}

TEST(Evaluation, IdentitySwitchesMotaAndIdf1) {
  Sequence s;
  // One labelled car, predicted as track 7 twice, then as track 8.
  for (int f = 0; f < 3; ++f) {
    s.frames.push_back(
        {f, f * kFrameNs, {truth(1, false, 0, 0)}, {predicted(f < 2 ? 7 : 8, false, 0, 0)}});
  }
  const auto m = jp::evaluation::evaluate({&s}, {});
  EXPECT_EQ(m.tracking.id_switches, 1U);
  EXPECT_NEAR(m.tracking.mota, 1.0 - (1.0 / 3.0), 1e-12);
  // Best one-to-one identity assignment: label 1 <-> track 7 for 2 of 3 frames.
  EXPECT_NEAR(m.tracking.idf1, 2.0 * 2.0 / (3.0 + 3.0), 1e-12);
}

TEST(Evaluation, HeadingAndSpeedAgainstLabelMotion) {
  Sequence s;
  // Labelled car driving along +x at 10 m/s; predicted at 9 m/s, heading off by 90 degrees.
  for (int f = 0; f < 21; ++f) {
    auto p = predicted(7, false, f, 0);
    p.speed_mps = 9.0;
    p.heading_rad = std::numbers::pi / 2;
    s.frames.push_back({f, f * kFrameNs, {truth(1, false, f, 0)}, {p}});
  }
  const auto m = jp::evaluation::evaluate({&s}, {});
  ASSERT_GT(m.speed_mps.count, 0U);
  EXPECT_NEAR(m.speed_mps.median, 1.0, 1e-9);
  EXPECT_NEAR(m.heading_deg.median, 90.0, 1e-9);
}

TEST(Evaluation, UnlabelledRoadUsersCanBeIgnored) {
  // A prediction 3 m from a missed label (displaced) still counts; one 20 m from every label,
  // even one outside the image, is an unlabelled road user.
  const auto s = one_frame(
      {truth(1, false, 0, 0), truth(2, false, 40, 0, false)},
      {predicted(7, false, 3, 0), predicted(8, false, 20, 0), predicted(9, false, 43, 0)});
  jp::evaluation::Options options;
  options.ignore_unlabelled = true;
  const auto m = jp::evaluation::evaluate({&s}, options);
  EXPECT_EQ(m.unlabelled, 1U);
  EXPECT_EQ(m.false_positives, 2U);
  EXPECT_EQ(m.displaced, 1U);
}

TEST(Evaluation, FootprintHeadingSizeAndIou) {
  Sequence s;
  // A standing car (label 4.5 x 1.8 along +x) predicted with the right size facing backwards,
  // 0.5 m off along its length; a second one without a heading.
  for (int f = 0; f < 21; ++f) {
    auto p = predicted(7, false, 0.5, 0);
    p.heading_rad = std::numbers::pi;
    p.size_m = {4.5, 1.8};
    p.measured_size = true;
    auto q = predicted(8, false, 0, 10);
    q.size_m = {4.0, 1.8};
    s.frames.push_back({f, f * kFrameNs, {truth(1, false, 0, 0), truth(2, false, 0, 10)}, {p, q}});
  }
  const auto m = jp::evaluation::evaluate({&s}, {});
  const auto& fp = m.footprint;
  EXPECT_EQ(fp.vehicles, 42U);
  EXPECT_EQ(fp.with_heading, 21U);
  EXPECT_EQ(fp.stationary, 42U);
  EXPECT_EQ(fp.stationary_with_heading, 21U);
  EXPECT_EQ(fp.front_correct, 0U);
  EXPECT_EQ(fp.measured_size, 21U);
  EXPECT_NEAR(fp.axis_deg.median, 0.0, 1e-9);
  EXPECT_NEAR(fp.length_m.p90, 0.5, 1e-9);
  EXPECT_NEAR(fp.length_bias_m.mean, -0.25, 1e-9);
  // The first car is placed 0.5 m farther from the camera (at the origin).
  EXPECT_NEAR(m.range_bias_m.mean, 0.25, 1e-9);
  // Overlap 4.0 x 1.8 of two 4.5 x 1.8 rectangles.
  EXPECT_NEAR(fp.iou.median, 4.0 / 5.0, 1e-9);
  EXPECT_EQ(m.footprint_by_class.at("car").vehicles, 42U);
}

TEST(Evaluation, TractorAndTrailerBehindItAreOneTruck) {
  const auto vehicle = [](std::int64_t id, const char* cls, double x, double y, double yaw,
                          double length) {
    TruthObject t = truth(id, false, x, y);
    t.twin_class = cls;
    t.yaw_rad = yaw;
    t.length_m = length;
    t.width_m = 2.5;
    return t;
  };
  // Tractor 6 m long heading +x with its rear at x = 7; its trailer (13 m) starts 0.5 m behind.
  // Another trailer stands across the road, a lone tractor further away.
  std::vector<TruthObject> frame = {vehicle(1, "truck", 10.0, 0.0, 0.0, 6.0),
                                    vehicle(2, "trailer", 0.0, 0.0, 0.05, 13.0),
                                    vehicle(3, "trailer", 5.0, 3.0, std::numbers::pi / 2, 13.0),
                                    vehicle(4, "truck", 40.0, 20.0, 0.0, 6.0)};
  jp::evaluation::merge_articulated(frame, {});
  ASSERT_EQ(frame.size(), 3U);
  const TruthObject& merged = frame[0];
  EXPECT_EQ(merged.track_id, 1);
  EXPECT_EQ(merged.twin_class, "truck");
  // From the trailer's rear (about x = -6.5) to the tractor's front (x = 13).
  EXPECT_NEAR(merged.length_m, 19.5, 0.1);
  EXPECT_NEAR(merged.position.x(), 3.25, 0.1);
  EXPECT_DOUBLE_EQ(merged.yaw_rad, 0.0);
  EXPECT_EQ(frame[1].track_id, 3);
  EXPECT_EQ(frame[2].track_id, 4);
}

TEST(Evaluation, ErrorsByDistanceFromTheCamera) {
  const auto s =
      one_frame({truth(1, false, 10, 0), truth(2, false, 70, 0)}, {predicted(7, false, 10.2, 0)});
  const auto m = jp::evaluation::evaluate({&s}, {});
  ASSERT_EQ(m.by_range.size(), 5U);
  EXPECT_EQ(m.by_range[0].truth, 1U);
  EXPECT_EQ(m.by_range[0].matched, 1U);
  EXPECT_EQ(m.by_range[3].truth, 1U);
  EXPECT_EQ(m.by_range[3].matched, 0U);
}

}  // namespace
