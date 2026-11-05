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
  double height_m = 0.0;
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
  /// Footprint length and width as the twin shows it (zero: unknown, no footprint metrics).
  Eigen::Vector2d size_m = Eigen::Vector2d::Zero();
  bool measured_size = false;  ///< Size measured by the detector (else the class size).
  double height_m = 0.0;       ///< Box height the twin assumes (the class height).
};

struct Frame {
  std::int64_t index = 0;
  std::int64_t timestamp_ns = 0;
  std::vector<TruthObject> truth;
  std::vector<PredictedObject> predicted;
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
  /// Predictions matched to no label and farther than `displaced_gate_m` from every label (also
  /// labels outside the image) are road users the labels miss: neither true nor false positives.
  bool ignore_unlabelled = false;
};

/// Tractor and trailer that the labels box separately (TUMTraf TRUCK + TRAILER), one behind the
/// other on one axis: the camera detector sees one vehicle.
struct ArticulatedMerge {
  double max_gap_m = 2.0;       ///< From the tractor's rear to the trailer's front.
  double max_angle_deg = 30.0;  ///< Between their yaws.
};

/// Replaces each tractor ("truck") and the trailer ("trailer") behind it by one "truck" label
/// from the trailer's rear to the tractor's front, with the tractor's track ID and yaw.
void merge_articulated(std::vector<TruthObject>& frame, const ArticulatedMerge& merge);

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

/// Orientation, size and footprint of matched labelled vehicles, against the label box.
struct FootprintMetrics {
  std::size_t vehicles = 0;      ///< Matched labelled vehicles.
  std::size_t with_heading = 0;  ///< ... with a predicted heading.
  std::size_t stationary = 0;    ///< ... standing (label slower than the moving speed).
  std::size_t stationary_with_heading = 0;
  std::size_t front_correct = 0;  ///< Heading within 90 degrees of the label's front.
  std::size_t measured_size = 0;  ///< Size measured by the detector.
  ErrorStats axis_deg;            ///< Heading against the label yaw, modulo 180 (0-90).
  ErrorStats stationary_axis_deg;
  ErrorStats length_m;  ///< Absolute error of the size the twin shows (measured or class size).
  ErrorStats width_m;
  ErrorStats length_bias_m;  ///< Signed: predicted minus label.
  ErrorStats width_bias_m;
  ErrorStats iou;  ///< Bird's-eye-view IoU of the footprints (predictions with a heading).
  /// 3D IoU of the boxes standing on the same ground: the twin places road users on the ground
  /// plane, so only the footprint and the height (label against class height) differ.
  ErrorStats iou_3d;
};

struct Metrics {
  std::size_t frames = 0;
  std::size_t truth = 0;
  std::size_t matched = 0;
  std::size_t false_positives = 0;
  std::size_t unlabelled = 0;  ///< Predictions ignored as unlabelled road users.
  ErrorStats position_m;
  ErrorStats range_bias_m;  ///< Signed: predicted minus label distance from the camera.
  ErrorStats heading_deg;   ///< Moving labelled vehicles with a predicted heading.
  ErrorStats speed_mps;
  std::size_t moving_without_heading = 0;
  std::size_t displaced = 0;  ///< Pairs of a missed label and a false positive within 5 m.
  ErrorStats displaced_m;
  std::map<std::string, ClassMetrics> by_class;
  std::map<std::string, std::map<std::string, std::size_t>> confusion;  ///< label -> predicted
  std::vector<RangeMetrics> by_range;
  FootprintMetrics footprint;
  std::map<std::string, FootprintMetrics> footprint_by_class;  ///< By label class.
  TrackingMetrics tracking;

  [[nodiscard]] double precision() const;
  [[nodiscard]] double recall() const;
};

/// Matching per frame (Hungarian on ground distance, gated by the label's class group), then
/// detection, localization, motion and tracking (CLEAR MOT, IDF1) metrics over all frames of all
/// sequences.
[[nodiscard]] Metrics evaluate(const std::vector<const Sequence*>& sequences,
                               const Options& options);

}  // namespace jp::evaluation
