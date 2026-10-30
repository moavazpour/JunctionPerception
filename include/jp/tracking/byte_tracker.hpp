#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "jp/detection/detection.hpp"
#include "jp/tracking/box_kalman.hpp"

namespace jp::tracking {

struct TrackerOptions {
  float high_score = 0.5F;       ///< Detections at or above this score are matched first.
  float low_score = 0.1F;        ///< Detections below this score are ignored.
  float new_track_score = 0.6F;  ///< Unmatched detections at or above this start a track.
  float high_match_min_iou = 0.2F;
  float low_match_min_iou = 0.5F;
  float tentative_match_min_iou = 0.3F;
  int confirm_frames = 3;      ///< Consecutive matched frames before a track gets an ID.
  int max_missed_frames = 15;  ///< Frames without a match before a confirmed track is dropped.
  std::string person_class = "pedestrian";
  float min_rider_votes = 1.0F;  ///< Summed scores of ridden detections that make a track a
                                 ///< two-wheeler with rider.
};

/// A confirmed track matched to a detection in the current frame.
struct TrackedObject {
  std::int64_t id = 0;
  detection::Detection detection;  ///< The matched detection of this frame.
  std::string twin_class;          ///< Class by score-weighted vote over the track's history.
  bool rider = false;              ///< The track has been seen as a ridden two-wheeler.
  int hits = 0;
};

/// ByteTrack multi-object tracker in image space: high-score detections are matched to all
/// tracks by IoU, then low-score detections to the remaining tracks that were matched in the
/// previous frame; new tracks are confirmed after consecutive matches before they get an ID.
class ByteTracker {
 public:
  explicit ByteTracker(TrackerOptions options = {});

  /// Advances all tracks by one frame and returns the confirmed tracks matched in this frame.
  [[nodiscard]] std::vector<TrackedObject> update(
      const std::vector<detection::Detection>& detections);

  [[nodiscard]] std::size_t active_tracks() const { return tracks_.size(); }

 private:
  struct Track {
    explicit Track(const cv::Rect2f& box) : filter(box) {}

    BoxKalman filter;
    std::int64_t id = 0;  // 0 while tentative
    int hits = 0;
    int missed = 0;
    bool matched_now = false;
    detection::Detection last;
    std::map<std::string, float, std::less<>> class_votes;
    std::map<std::string, float, std::less<>> rider_votes;  // ridden detections per class
  };

  void match(std::vector<std::size_t>& track_pool, std::vector<std::size_t>& detection_pool,
             const std::vector<detection::Detection>& detections, float min_iou);
  static void apply(Track& track, const detection::Detection& detection);
  [[nodiscard]] TrackedObject output(const Track& track) const;

  TrackerOptions options_;
  std::vector<Track> tracks_;
  std::int64_t next_id_ = 1;
};

}  // namespace jp::tracking
