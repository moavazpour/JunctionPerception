#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace jp::evaluation {

/// A labelled road user of one frame, in the scene's ground frame.
struct TruthObject {
  std::int64_t track_id = 0;
  std::string twin_class;
  bool vru = false;
  Eigen::Vector2d position = Eigen::Vector2d::Zero();  ///< Footprint centre (m).
  double yaw_rad = 0.0;
  double length_m = 0.0;
  double width_m = 0.0;
  /// Seen by the camera (box centre projects into the image). Others are "don't care": a
  /// prediction matched to them is neither a true nor a false positive.
  bool visible = true;
};

/// A road user of the evaluated world state.
struct PredictedObject {
  std::int64_t id = 0;
  std::string twin_class;
  bool vru = false;
  Eigen::Vector2d position = Eigen::Vector2d::Zero();
  std::optional<double> heading_rad;
  double speed_mps = 0.0;
};

/// A VRU-vehicle conflict reported by the system, at the frame it started.
struct PredictedConflict {
  std::int64_t vehicle_id = 0;
  std::int64_t vru_id = 0;
};

struct Frame {
  std::int64_t index = 0;
  std::int64_t timestamp_ns = 0;
  std::vector<TruthObject> truth;
  std::vector<PredictedObject> predicted;
  std::vector<PredictedConflict> conflicts_started;
};

/// One evaluated recording (e.g. one sequence seen by one camera).
struct Sequence {
  std::string label;
  std::string condition;  ///< e.g. "day", "night"
  Eigen::Vector2d camera_position = Eigen::Vector2d::Zero();
  std::vector<Frame> frames;
};

struct Options {
  double vehicle_gate_m = 2.0;  ///< Matching gate for labelled vehicles.
  double vru_gate_m = 1.0;      ///< Matching gate for labelled VRUs.
  /// Second pass: unmatched labels and predictions within this distance are "displaced"
  /// (detected, but localized beyond the class gate) rather than a miss and a false positive.
  double displaced_gate_m = 5.0;
  /// Reference speed and direction of motion from label positions over this window (centred).
  double velocity_window_s = 1.0;
  double moving_speed_mps = 1.0;  ///< Heading errors are evaluated above this label speed.
  std::vector<double> range_bins_m = {0.0, 20.0, 40.0, 60.0, 80.0, 120.0};
  /// Label conflicts: smallest gap between the vehicle footprint and the VRU within the horizon
  /// below `conflict_gap_m`, shrinking on average by at least `conflict_min_closing_speed_mps`.
  double conflict_gap_m = 1.0;
  double conflict_horizon_s = 3.0;
  double conflict_min_closing_speed_mps = 0.3;
};

/// Count, mean, median and 90th percentile of a set of errors.
struct ErrorStats {
  std::size_t count = 0;
  double mean = 0.0;
  double median = 0.0;
  double p90 = 0.0;
};

[[nodiscard]] ErrorStats error_stats(const std::vector<double>& values);

struct ClassMetrics {
  std::size_t truth = 0;      ///< Visible labels of this class.
  std::size_t matched = 0;    ///< ... matched by a prediction of any class.
  std::size_t predicted = 0;  ///< Predictions of this class, without "don't care" matches.
  std::size_t false_positives = 0;
  ErrorStats position_m;  ///< By label class.
};

struct RangeMetrics {
  double min_m = 0.0;
  double max_m = 0.0;
  std::size_t truth = 0;
  std::size_t matched = 0;
  ErrorStats position_m;
};

struct TrackingMetrics {
  std::size_t id_switches = 0;
  double mota = 0.0;
  double idf1 = 0.0;
};

struct ConflictMetrics {
  std::size_t truth_episodes = 0;  ///< Label pairs in conflict (consecutive frames: one).
  std::size_t truth_detected = 0;  ///< ... of which the system reported.
  std::size_t predicted = 0;       ///< Conflicts reported by the system.
  std::size_t confirmed = 0;       ///< ... that match a label conflict.
  /// Smallest label gap ever seen between a vehicle and a VRU (both visible); empty if none.
  std::optional<double> closest_truth_gap_m;
};

struct Metrics {
  std::size_t frames = 0;
  std::size_t truth = 0;
  std::size_t matched = 0;
  std::size_t false_positives = 0;
  ErrorStats position_m;
  ErrorStats heading_deg;  ///< Moving labelled vehicles with a predicted heading.
  ErrorStats speed_mps;
  std::size_t moving_without_heading = 0;
  std::size_t displaced = 0;  ///< Pairs of a missed label and a false positive within 5 m.
  ErrorStats displaced_m;
  std::map<std::string, ClassMetrics> by_class;
  std::map<std::string, std::map<std::string, std::size_t>> confusion;  ///< label -> predicted
  std::vector<RangeMetrics> by_range;
  TrackingMetrics tracking;
  ConflictMetrics conflicts;

  [[nodiscard]] double precision() const;
  [[nodiscard]] double recall() const;
};

/// Matching per frame (Hungarian on ground distance, gated by the label's class group), then
/// detection, localization, motion, tracking (CLEAR MOT, IDF1) and conflict metrics over all
/// frames of all sequences.
[[nodiscard]] Metrics evaluate(const std::vector<const Sequence*>& sequences,
                               const Options& options);

}  // namespace jp::evaluation
