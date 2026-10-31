#include "jp/evaluation/evaluation.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <numeric>
#include <set>
#include <tuple>
#include <unordered_map>
#include <utility>

#include "jp/core/assignment.hpp"
#include "jp/core/statistics.hpp"
#include "jp/vru/event_detector.hpp"

namespace jp::evaluation {

namespace {

constexpr double kNanosecondsPerSecond = 1e9;
constexpr double kMedian = 0.5;
constexpr double kP90 = 0.9;
constexpr double kNoMatch = 1e9;
constexpr std::size_t kNone = static_cast<std::size_t>(-1);

double gate_of(const TruthObject& truth, const Options& options) {
  return truth.vru ? options.vru_gate_m : options.vehicle_gate_m;
}

double seconds(std::int64_t ns) { return static_cast<double>(ns) / kNanosecondsPerSecond; }

double angle_error_deg(double a, double b) {
  return std::abs(std::remainder(a - b, 2.0 * std::numbers::pi)) * 180.0 / std::numbers::pi;
}

// Gap between a labelled vehicle's footprint and a labelled VRU (disc).
double truth_gap(const TruthObject& vehicle, const TruthObject& vru) {
  return vru::rectangle_disc_gap(vehicle.position, vehicle.yaw_rad,
                                 {vehicle.length_m, vehicle.width_m}, vru.position,
                                 0.5 * std::min(vru.length_m, vru.width_m));
}

// Label positions of every track over the frames of a sequence.
class TruthTracks {
 public:
  explicit TruthTracks(const Sequence& sequence) : sequence_(&sequence) {
    by_frame_.resize(sequence.frames.size());
    for (std::size_t f = 0; f < sequence.frames.size(); ++f) {
      for (const auto& object : sequence.frames[f].truth) {
        by_frame_[f][object.track_id] = &object;
        frames_of_[object.track_id].push_back(f);
      }
    }
  }

  [[nodiscard]] const TruthObject* at(std::size_t frame, std::int64_t track) const {
    const auto it = by_frame_[frame].find(track);
    return it == by_frame_[frame].end() ? nullptr : it->second;
  }

  /// Velocity from the label positions over a centred window; empty near track ends.
  [[nodiscard]] std::optional<Eigen::Vector2d> velocity(std::size_t frame, std::int64_t track,
                                                        double window_s) const {
    const auto& frames = frames_of_.at(track);
    const double t = seconds(sequence_->frames[frame].timestamp_ns);
    std::optional<std::size_t> first;
    std::optional<std::size_t> last;
    for (const std::size_t f : frames) {
      const double dt = seconds(sequence_->frames[f].timestamp_ns) - t;
      if (dt >= -0.5 * window_s && !first) {
        first = f;
      }
      if (dt <= 0.5 * window_s) {
        last = f;
      }
    }
    if (!first || !last) {
      return std::nullopt;
    }
    const double span = seconds(sequence_->frames[*last].timestamp_ns) -
                        seconds(sequence_->frames[*first].timestamp_ns);
    if (span < 0.5 * window_s) {
      return std::nullopt;
    }
    return (at(*last, track)->position - at(*first, track)->position) / span;
  }

 private:
  const Sequence* sequence_;
  std::vector<std::unordered_map<std::int64_t, const TruthObject*>> by_frame_;
  std::unordered_map<std::int64_t, std::vector<std::size_t>> frames_of_;
};

// Hungarian matching of labels (rows) to predictions (columns) on ground distance, pairs
// allowed within gate(label): label row -> prediction column (kNone: unmatched).
template <typename Gate>
std::vector<std::size_t> match(const std::vector<const TruthObject*>& truth,
                               const std::vector<const PredictedObject*>& predicted,
                               const Gate& gate) {
  std::vector<std::size_t> matches(truth.size(), kNone);
  if (truth.empty() || predicted.empty()) {
    return matches;
  }
  Eigen::MatrixXd cost(static_cast<Eigen::Index>(truth.size()),
                       static_cast<Eigen::Index>(predicted.size()));
  for (std::size_t t = 0; t < truth.size(); ++t) {
    for (std::size_t p = 0; p < predicted.size(); ++p) {
      const double d = (truth[t]->position - predicted[p]->position).norm();
      cost(static_cast<Eigen::Index>(t), static_cast<Eigen::Index>(p)) =
          d <= gate(*truth[t]) ? d : kNoMatch;
    }
  }
  const auto assignment = core::solve_assignment(cost);
  for (std::size_t t = 0; t < assignment.size(); ++t) {
    const auto p = assignment[t];
    if (p >= 0 && cost(static_cast<Eigen::Index>(t), p) < kNoMatch) {
      matches[t] = static_cast<std::size_t>(p);
    }
  }
  return matches;
}

template <typename T>
std::vector<const T*> pointers(const std::vector<T>& items) {
  std::vector<const T*> out;
  out.reserve(items.size());
  for (const auto& item : items) {
    out.push_back(&item);
  }
  return out;
}

// Matching of one frame with the class gates: label index -> prediction index.
std::vector<std::size_t> match_frame(const Frame& frame, const Options& options) {
  return match(pointers(frame.truth), pointers(frame.predicted),
               [&](const TruthObject& t) { return gate_of(t, options); });
}

// Errors collected over all frames, turned into Metrics at the end.
struct Accumulator {
  std::vector<double> position;
  std::vector<double> heading;
  std::vector<double> displaced;
  std::vector<double> speed;
  std::map<std::string, std::vector<double>> class_position;
  std::vector<std::vector<double>> range_position;
  double id_true_positives = 0.0;
  double id_truth = 0.0;
  double id_predicted = 0.0;
};

class SequenceEvaluator {
 public:
  SequenceEvaluator(const Sequence& sequence, const Options& options, Metrics& metrics,
                    Accumulator& acc)
      : sequence_(&sequence),
        options_(&options),
        metrics_(&metrics),
        acc_(&acc),
        tracks_(sequence) {}

  void run() {
    for (std::size_t f = 0; f < sequence_->frames.size(); ++f) {
      evaluate_frame(f);
    }
    finish_identities();
    evaluate_conflicts();
  }

 private:
  void evaluate_frame(std::size_t f) {
    const Frame& frame = sequence_->frames[f];
    ++metrics_->frames;
    const auto matches = match_frame(frame, *options_);
    std::vector<bool> dont_care(frame.predicted.size(), false);
    std::vector<bool> matched_visible(frame.predicted.size(), false);
    auto& prediction_to_track = prediction_to_track_.emplace_back();
    for (std::size_t t = 0; t < frame.truth.size(); ++t) {
      const TruthObject& truth = frame.truth[t];
      const std::size_t p = matches[t];
      if (p != kNone) {
        prediction_to_track[frame.predicted[p].id] = truth.track_id;
        if (truth.visible) {
          matched_visible[p] = true;
        } else {
          dont_care[p] = true;
        }
      }
      if (truth.visible) {
        count_truth(f, truth, p == kNone ? nullptr : &frame.predicted[p]);
      }
    }
    for (std::size_t p = 0; p < frame.predicted.size(); ++p) {
      if (dont_care[p]) {
        continue;
      }
      auto& cls = metrics_->by_class[frame.predicted[p].twin_class];
      ++cls.predicted;
      acc_->id_predicted += 1.0;
      if (!matched_visible[p]) {
        ++metrics_->false_positives;
        ++cls.false_positives;
      }
    }
    count_displaced(frame, matches, dont_care, matched_visible);
    count_identity_pairs(frame, dont_care);
  }

  // Labels and predictions left unmatched by the class gates but within the wider localization
  // gate of each other: detected, but placed too far away.
  void count_displaced(const Frame& frame, const std::vector<std::size_t>& matches,
                       const std::vector<bool>& dont_care,
                       const std::vector<bool>& matched_visible) {
    std::vector<const TruthObject*> truth;
    for (std::size_t t = 0; t < frame.truth.size(); ++t) {
      if (frame.truth[t].visible && matches[t] == kNone) {
        truth.push_back(&frame.truth[t]);
      }
    }
    std::vector<const PredictedObject*> predicted;
    for (std::size_t p = 0; p < frame.predicted.size(); ++p) {
      if (!dont_care[p] && !matched_visible[p]) {
        predicted.push_back(&frame.predicted[p]);
      }
    }
    const auto pairs =
        match(truth, predicted, [&](const TruthObject&) { return options_->displaced_gate_m; });
    for (std::size_t t = 0; t < pairs.size(); ++t) {
      if (pairs[t] != kNone) {
        ++metrics_->displaced;
        acc_->displaced.push_back((truth[t]->position - predicted[pairs[t]]->position).norm());
      }
    }
  }

  void count_truth(std::size_t f, const TruthObject& truth, const PredictedObject* prediction) {
    ++metrics_->truth;
    acc_->id_truth += 1.0;
    auto& cls = metrics_->by_class[truth.twin_class];
    ++cls.truth;
    const double range = (truth.position - sequence_->camera_position).norm();
    const auto bin = range_bin(range);
    if (bin) {
      ++metrics_->by_range[*bin].truth;
    }
    if (prediction == nullptr) {
      return;
    }
    ++metrics_->matched;
    ++cls.matched;
    ++metrics_->confusion[truth.twin_class][prediction->twin_class];
    const double error = (truth.position - prediction->position).norm();
    acc_->position.push_back(error);
    acc_->class_position[truth.twin_class].push_back(error);
    if (bin) {
      ++metrics_->by_range[*bin].matched;
      acc_->range_position[*bin].push_back(error);
    }
    // CLEAR MOT identity switch: the label is now matched to a different prediction.
    const auto last = last_match_.find(truth.track_id);
    if (last != last_match_.end() && last->second != prediction->id) {
      ++metrics_->tracking.id_switches;
    }
    last_match_[truth.track_id] = prediction->id;
    const auto velocity = tracks_.velocity(f, truth.track_id, options_->velocity_window_s);
    if (!velocity) {
      return;
    }
    acc_->speed.push_back(std::abs(prediction->speed_mps - velocity->norm()));
    if (!truth.vru && velocity->norm() >= options_->moving_speed_mps) {
      if (prediction->heading_rad) {
        acc_->heading.push_back(
            angle_error_deg(*prediction->heading_rad, std::atan2(velocity->y(), velocity->x())));
      } else {
        ++metrics_->moving_without_heading;
      }
    }
  }

  [[nodiscard]] std::optional<std::size_t> range_bin(double range) const {
    for (std::size_t i = 0; i + 1 < options_->range_bins_m.size(); ++i) {
      if (range >= options_->range_bins_m[i] && range < options_->range_bins_m[i + 1]) {
        return i;
      }
    }
    return std::nullopt;
  }

  // IDF1: frames in which a label track and a prediction track are within the gate.
  void count_identity_pairs(const Frame& frame, const std::vector<bool>& dont_care) {
    for (const auto& truth : frame.truth) {
      if (!truth.visible) {
        continue;
      }
      for (std::size_t p = 0; p < frame.predicted.size(); ++p) {
        if (!dont_care[p] &&
            (truth.position - frame.predicted[p].position).norm() <= gate_of(truth, *options_)) {
          ++co_occurrence_[{truth.track_id, frame.predicted[p].id}];
        }
      }
    }
  }

  // One-to-one assignment of label tracks to prediction tracks maximizing shared frames.
  void finish_identities() {
    std::map<std::int64_t, Eigen::Index> rows;
    std::map<std::int64_t, Eigen::Index> cols;
    for (const auto& [pair, count] : co_occurrence_) {
      rows.try_emplace(pair.first, static_cast<Eigen::Index>(rows.size()));
      cols.try_emplace(pair.second, static_cast<Eigen::Index>(cols.size()));
    }
    if (rows.empty()) {
      return;
    }
    Eigen::MatrixXd cost = Eigen::MatrixXd::Zero(static_cast<Eigen::Index>(rows.size()),
                                                 static_cast<Eigen::Index>(cols.size()));
    for (const auto& [pair, count] : co_occurrence_) {
      cost(rows[pair.first], cols[pair.second]) = -static_cast<double>(count);
    }
    const auto assignment = core::solve_assignment(cost);
    for (Eigen::Index r = 0; r < static_cast<Eigen::Index>(assignment.size()); ++r) {
      if (assignment[r] >= 0) {
        acc_->id_true_positives -= cost(r, assignment[r]);
      }
    }
  }

  // Whether a label pair is in conflict at frame f: the gap observed over the horizon.
  [[nodiscard]] bool truth_conflict(std::size_t f, const TruthObject& vehicle,
                                    const TruthObject& vru) const {
    const double t0 = seconds(sequence_->frames[f].timestamp_ns);
    const double gap0 = truth_gap(vehicle, vru);
    double best = gap0;
    double best_t = 0.0;
    for (std::size_t g = f + 1; g < sequence_->frames.size(); ++g) {
      const double dt = seconds(sequence_->frames[g].timestamp_ns) - t0;
      if (dt > options_->conflict_horizon_s) {
        break;
      }
      const TruthObject* a = tracks_.at(g, vehicle.track_id);
      const TruthObject* b = tracks_.at(g, vru.track_id);
      if (a == nullptr || b == nullptr) {
        continue;
      }
      const double gap = truth_gap(*a, *b);
      if (gap < best) {
        best = gap;
        best_t = dt;
      }
    }
    return best < options_->conflict_gap_m && best_t > 0.0 &&
           (gap0 - best) / best_t >= options_->conflict_min_closing_speed_mps;
  }

  using Episodes = std::map<std::pair<std::int64_t, std::int64_t>,
                            std::vector<std::pair<std::size_t, std::size_t>>>;

  void evaluate_conflicts() {
    const Episodes episodes = truth_episodes();
    check_reported_conflicts(episodes);
  }

  // Label conflict episodes: runs of consecutive conflict frames per (vehicle, VRU) pair.
  Episodes truth_episodes() {
    Episodes episodes;
    auto& conflicts = metrics_->conflicts;
    for (std::size_t f = 0; f < sequence_->frames.size(); ++f) {
      for (const auto& vru : sequence_->frames[f].truth) {
        if (!vru.vru || !vru.visible) {
          continue;
        }
        for (const auto& vehicle : sequence_->frames[f].truth) {
          if (vehicle.vru || !vehicle.visible) {
            continue;
          }
          const double gap = truth_gap(vehicle, vru);
          conflicts.closest_truth_gap_m =
              std::min(conflicts.closest_truth_gap_m.value_or(gap), gap);
          if (!truth_conflict(f, vehicle, vru)) {
            continue;
          }
          auto& runs = episodes[{vehicle.track_id, vru.track_id}];
          if (!runs.empty() && runs.back().second + 1 == f) {
            runs.back().second = f;
          } else {
            runs.emplace_back(f, f);
            ++conflicts.truth_episodes;
          }
        }
      }
    }
    return episodes;
  }

  // Index of an episode within the horizon of time t, or kNone.
  [[nodiscard]] std::size_t overlapping_episode(
      const std::vector<std::pair<std::size_t, std::size_t>>& runs, double t) const {
    for (std::size_t e = 0; e < runs.size(); ++e) {
      const double start = seconds(sequence_->frames[runs[e].first].timestamp_ns);
      const double end = seconds(sequence_->frames[runs[e].second].timestamp_ns);
      if (t >= start - options_->conflict_horizon_s && t <= end + options_->conflict_horizon_s) {
        return e;
      }
    }
    return kNone;
  }

  // Reported conflicts: confirmed if both objects match the labels of an episode around then.
  void check_reported_conflicts(const Episodes& episodes) {
    auto& conflicts = metrics_->conflicts;
    std::set<std::tuple<std::int64_t, std::int64_t, std::size_t>> detected;
    for (std::size_t f = 0; f < sequence_->frames.size(); ++f) {
      for (const auto& conflict : sequence_->frames[f].conflicts_started) {
        ++conflicts.predicted;
        const auto& to_track = prediction_to_track_[f];
        const auto vehicle = to_track.find(conflict.vehicle_id);
        const auto vru = to_track.find(conflict.vru_id);
        if (vehicle == to_track.end() || vru == to_track.end()) {
          continue;
        }
        const auto it = episodes.find({vehicle->second, vru->second});
        if (it == episodes.end()) {
          continue;
        }
        const double t = seconds(sequence_->frames[f].timestamp_ns);
        const auto e = overlapping_episode(it->second, t);
        if (e != kNone) {
          ++conflicts.confirmed;
          detected.emplace(vehicle->second, vru->second, e);
        }
      }
    }
    conflicts.truth_detected += detected.size();
  }

  const Sequence* sequence_;
  const Options* options_;
  Metrics* metrics_;
  Accumulator* acc_;
  TruthTracks tracks_;
  std::map<std::int64_t, std::int64_t> last_match_;
  std::map<std::pair<std::int64_t, std::int64_t>, std::size_t> co_occurrence_;
  std::vector<std::unordered_map<std::int64_t, std::int64_t>> prediction_to_track_;
};

}  // namespace

ErrorStats error_stats(const std::vector<double>& values) {
  ErrorStats stats;
  stats.count = values.size();
  if (values.empty()) {
    return stats;
  }
  stats.mean =
      std::accumulate(values.begin(), values.end(), 0.0) / static_cast<double>(values.size());
  stats.median = core::percentile(values, kMedian);
  stats.p90 = core::percentile(values, kP90);
  return stats;
}

double Metrics::precision() const {
  const std::size_t positives = matched + false_positives;
  return positives == 0 ? 0.0 : static_cast<double>(matched) / static_cast<double>(positives);
}

double Metrics::recall() const {
  return truth == 0 ? 0.0 : static_cast<double>(matched) / static_cast<double>(truth);
}

Metrics evaluate(const std::vector<const Sequence*>& sequences, const Options& options) {
  Metrics metrics;
  Accumulator acc;
  for (std::size_t i = 0; i + 1 < options.range_bins_m.size(); ++i) {
    metrics.by_range.push_back({options.range_bins_m[i], options.range_bins_m[i + 1], 0, 0, {}});
  }
  acc.range_position.resize(metrics.by_range.size());
  for (const Sequence* sequence : sequences) {
    SequenceEvaluator(*sequence, options, metrics, acc).run();
  }
  metrics.position_m = error_stats(acc.position);
  metrics.heading_deg = error_stats(acc.heading);
  metrics.displaced_m = error_stats(acc.displaced);
  metrics.speed_mps = error_stats(acc.speed);
  for (auto& [name, cls] : metrics.by_class) {
    cls.position_m = error_stats(acc.class_position[name]);
  }
  for (std::size_t i = 0; i < metrics.by_range.size(); ++i) {
    metrics.by_range[i].position_m = error_stats(acc.range_position[i]);
  }
  if (metrics.truth > 0) {
    const auto misses = static_cast<double>(metrics.truth - metrics.matched);
    metrics.tracking.mota = 1.0 - ((misses + static_cast<double>(metrics.false_positives) +
                                    static_cast<double>(metrics.tracking.id_switches)) /
                                   static_cast<double>(metrics.truth));
  }
  if (acc.id_truth + acc.id_predicted > 0.0) {
    metrics.tracking.idf1 = 2.0 * acc.id_true_positives / (acc.id_truth + acc.id_predicted);
  }
  return metrics;
}

}  // namespace jp::evaluation
