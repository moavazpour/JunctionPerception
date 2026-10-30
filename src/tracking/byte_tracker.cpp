#include "jp/tracking/byte_tracker.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <utility>

#include "jp/core/assignment.hpp"

namespace jp::tracking {

namespace {

constexpr double kForbiddenCost = 1e9;

float iou(const cv::Rect2f& a, const cv::Rect2f& b) {
  const float intersection = (a & b).area();
  const float union_area = a.area() + b.area() - intersection;
  return union_area > 0.0F ? intersection / union_area : 0.0F;
}

void erase_indices(std::vector<std::size_t>& pool, const std::vector<std::size_t>& used) {
  std::erase_if(pool, [&](std::size_t i) { return std::ranges::find(used, i) != used.end(); });
}

}  // namespace

ByteTracker::ByteTracker(TrackerOptions options) : options_(std::move(options)) {}

void ByteTracker::match(std::vector<std::size_t>& track_pool,
                        std::vector<std::size_t>& detection_pool,
                        const std::vector<detection::Detection>& detections, float min_iou) {
  if (track_pool.empty() || detection_pool.empty()) {
    return;
  }
  Eigen::MatrixXd cost(static_cast<Eigen::Index>(track_pool.size()),
                       static_cast<Eigen::Index>(detection_pool.size()));
  for (std::size_t t = 0; t < track_pool.size(); ++t) {
    const cv::Rect2f predicted = tracks_[track_pool[t]].filter.box();
    for (std::size_t d = 0; d < detection_pool.size(); ++d) {
      const float overlap = iou(predicted, detections[detection_pool[d]].box);
      cost(static_cast<Eigen::Index>(t), static_cast<Eigen::Index>(d)) =
          overlap >= min_iou ? 1.0 - overlap : kForbiddenCost;
    }
  }
  std::vector<std::size_t> used_tracks;
  std::vector<std::size_t> used_detections;
  const auto assignment = core::solve_assignment(cost);
  for (std::size_t t = 0; t < assignment.size(); ++t) {
    const Eigen::Index d = assignment[t];
    if (d < 0 || cost(static_cast<Eigen::Index>(t), d) >= kForbiddenCost) {
      continue;
    }
    const std::size_t detection_index = detection_pool[static_cast<std::size_t>(d)];
    apply(tracks_[track_pool[t]], detections[detection_index]);
    used_tracks.push_back(track_pool[t]);
    used_detections.push_back(detection_index);
  }
  erase_indices(track_pool, used_tracks);
  erase_indices(detection_pool, used_detections);
}

void ByteTracker::apply(Track& track, const detection::Detection& detection) {
  track.filter.update(detection.box);
  ++track.hits;
  track.missed = 0;
  track.matched_now = true;
  track.last = detection;
  track.class_votes[detection.twin_class] += detection.score;
  if (detection.rider) {
    track.rider_votes[detection.twin_class] += detection.score;
  }
}

TrackedObject ByteTracker::output(const Track& track) const {
  TrackedObject object;
  object.id = track.id;
  object.detection = track.last;
  object.hits = track.hits;
  // With enough evidence of a rider, frames where only the person was detected count for the
  // two-wheeler, so the person class takes no part in the vote.
  float rider_total = 0.0F;
  for (const auto& [twin_class, votes] : track.rider_votes) {
    rider_total += votes;
  }
  const bool rider_evidence = rider_total >= options_.min_rider_votes;
  float best = -1.0F;
  for (const auto& [twin_class, votes] : track.class_votes) {
    if (rider_evidence && twin_class == options_.person_class) {
      continue;
    }
    if (votes > best) {
      best = votes;
      object.twin_class = twin_class;
    }
  }
  object.rider = rider_evidence && track.rider_votes.contains(object.twin_class);
  return object;
}

std::vector<TrackedObject> ByteTracker::update(
    const std::vector<detection::Detection>& detections) {
  for (auto& track : tracks_) {
    track.filter.predict();
    track.matched_now = false;
  }

  std::vector<std::size_t> high;
  std::vector<std::size_t> low;
  for (std::size_t i = 0; i < detections.size(); ++i) {
    if (detections[i].score >= options_.high_score) {
      high.push_back(i);
    } else if (detections[i].score >= options_.low_score) {
      low.push_back(i);
    }
  }
  std::vector<std::size_t> confirmed;
  std::vector<std::size_t> tentative;
  for (std::size_t i = 0; i < tracks_.size(); ++i) {
    (tracks_[i].id != 0 ? confirmed : tentative).push_back(i);
  }

  // 1. High-score detections against all confirmed tracks (tracked and lost).
  match(confirmed, high, detections, options_.high_match_min_iou);
  // 2. Low-score detections against the remaining tracks that were matched in the last frame.
  std::vector<std::size_t> recent;
  std::ranges::copy_if(confirmed, std::back_inserter(recent),
                       [&](std::size_t i) { return tracks_[i].missed == 0; });
  match(recent, low, detections, options_.low_match_min_iou);
  // 3. Remaining high-score detections against tentative tracks.
  match(tentative, high, detections, options_.tentative_match_min_iou);

  for (auto& track : tracks_) {
    if (!track.matched_now) {
      ++track.missed;
    }
  }
  std::erase_if(tracks_, [&](const Track& track) {
    return track.id == 0 ? !track.matched_now : track.missed > options_.max_missed_frames;
  });
  for (const std::size_t i : high) {
    if (detections[i].score >= options_.new_track_score) {
      Track track(detections[i].box);
      apply(track, detections[i]);
      tracks_.push_back(std::move(track));
    }
  }
  for (auto& track : tracks_) {
    if (track.id == 0 && track.hits >= options_.confirm_frames) {
      track.id = next_id_++;
    }
  }

  std::vector<TrackedObject> result;
  for (const auto& track : tracks_) {
    if (track.id != 0 && track.matched_now) {
      result.push_back(output(track));
    }
  }
  return result;
}

}  // namespace jp::tracking
