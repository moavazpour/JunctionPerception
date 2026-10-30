#include "jp/tracking/byte_tracker.hpp"

#include <gtest/gtest.h>

namespace {

using jp::detection::Detection;
using jp::tracking::ByteTracker;

Detection det(float x, float y, float score, const std::string& twin_class = "car",
              bool rider = false) {
  Detection d;
  d.box = cv::Rect2f(x, y, 40, 30);
  d.score = score;
  d.twin_class = twin_class;
  d.rider = rider;
  return d;
}

TEST(ByteTracker, ConfirmsAfterConsecutiveFramesAndKeepsId) {
  ByteTracker tracker;
  EXPECT_TRUE(tracker.update({det(100, 100, 0.9F)}).empty());
  EXPECT_TRUE(tracker.update({det(104, 100, 0.9F)}).empty());
  const auto third = tracker.update({det(108, 100, 0.9F)});
  ASSERT_EQ(third.size(), 1U);
  const auto id = third.front().id;
  EXPECT_EQ(id, 1);
  for (int i = 3; i < 20; ++i) {
    const auto tracks = tracker.update({det(100.0F + (4.0F * i), 100, 0.9F)});
    ASSERT_EQ(tracks.size(), 1U);
    EXPECT_EQ(tracks.front().id, id);
  }
}

TEST(ByteTracker, LowScoreDetectionsContinueTracks) {
  ByteTracker tracker;
  for (int i = 0; i < 3; ++i) {
    static_cast<void>(tracker.update({det(100.0F + (4.0F * i), 100, 0.9F)}));
  }
  // Partially occluded: score drops below the high threshold but the track continues.
  const auto tracks = tracker.update({det(112, 100, 0.3F)});
  ASSERT_EQ(tracks.size(), 1U);
  EXPECT_EQ(tracks.front().id, 1);
  // A low-score detection elsewhere never starts a track.
  EXPECT_EQ(tracker.update({det(116, 100, 0.9F), det(500, 400, 0.3F)}).size(), 1U);
}

TEST(ByteTracker, RecoversAfterShortGapAndDropsAfterLongGap) {
  jp::tracking::TrackerOptions options;
  options.max_missed_frames = 5;
  ByteTracker tracker(options);
  for (int i = 0; i < 3; ++i) {
    static_cast<void>(tracker.update({det(100, 100, 0.9F)}));
  }
  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(tracker.update({}).empty());
  }
  const auto recovered = tracker.update({det(100, 100, 0.9F)});
  ASSERT_EQ(recovered.size(), 1U);
  EXPECT_EQ(recovered.front().id, 1);
  for (int i = 0; i < 6; ++i) {
    static_cast<void>(tracker.update({}));
  }
  EXPECT_EQ(tracker.active_tracks(), 0U);
}

TEST(ByteTracker, RiderEvidenceKeepsTwoWheelerClass) {
  ByteTracker tracker;
  for (int i = 0; i < 3; ++i) {
    static_cast<void>(tracker.update({det(100, 100, 0.8F, "bicycle", true)}));
  }
  // The bicycle is missed for several frames; only the rider is detected.
  std::vector<jp::tracking::TrackedObject> tracks;
  for (int i = 0; i < 6; ++i) {
    tracks = tracker.update({det(100, 100, 0.9F, "pedestrian")});
  }
  ASSERT_EQ(tracks.size(), 1U);
  EXPECT_EQ(tracks.front().twin_class, "bicycle");
  EXPECT_TRUE(tracks.front().rider);
}

TEST(ByteTracker, ClassFollowsMajorityVote) {
  ByteTracker tracker;
  std::vector<jp::tracking::TrackedObject> tracks;
  for (int i = 0; i < 5; ++i) {
    tracks = tracker.update({det(100, 100, 0.9F, i == 4 ? "truck" : "car")});
  }
  ASSERT_EQ(tracks.size(), 1U);
  EXPECT_EQ(tracks.front().twin_class, "car");
}

}  // namespace

namespace {

TEST(ByteTracker, RiderFlagOnlyForRiddenClass) {
  ByteTracker tracker;
  std::vector<jp::tracking::TrackedObject> tracks;
  // Two ridden bicycle detections, then a long run as bus: the track is a bus without rider.
  for (int i = 0; i < 12; ++i) {
    tracks = tracker.update({det(100, 100, 0.9F, i < 2 ? "bicycle" : "bus", i < 2)});
  }
  ASSERT_EQ(tracks.size(), 1U);
  EXPECT_EQ(tracks.front().twin_class, "bus");
  EXPECT_FALSE(tracks.front().rider);
}

}  // namespace
