#include "jp/evaluation/evaluation.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <numeric>
#include <unordered_map>
#include <utility>

#include "jp/core/assignment.hpp"
#include "jp/core/statistics.hpp"

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

using Polygon = std::vector<Eigen::Vector2d>;

// Footprint rectangle, corners counter-clockwise.
Polygon rectangle(const Eigen::Vector2d& centre, double yaw, double length, double width) {
  const Eigen::Vector2d front(std::cos(yaw), std::sin(yaw));
  const Eigen::Vector2d left(-front.y(), front.x());
  const Eigen::Vector2d f = 0.5 * length * front;
  const Eigen::Vector2d l = 0.5 * width * left;
  return {centre + f + l, centre - f + l, centre - f - l, centre + f - l};
}

double cross(const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
  return a.x() * b.y() - a.y() * b.x();
}

double area(const Polygon& polygon) {
  double twice = 0.0;
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    twice += cross(polygon[i], polygon[(i + 1) % polygon.size()]);
  }
  return 0.5 * std::abs(twice);
}

// Intersection of two convex counter-clockwise polygons (Sutherland-Hodgman).
Polygon intersection(Polygon subject, const Polygon& clip) {
  for (std::size_t i = 0; i < clip.size() && !subject.empty(); ++i) {
    const Eigen::Vector2d a = clip[i];
    const Eigen::Vector2d edge = clip[(i + 1) % clip.size()] - a;
    const Polygon input = std::move(subject);
    subject.clear();
    for (std::size_t j = 0; j < input.size(); ++j) {
      const Eigen::Vector2d& p = input[j];
      const Eigen::Vector2d& q = input[(j + 1) % input.size()];
      const double sp = cross(edge, p - a);
      const double sq = cross(edge, q - a);
      if (sp >= 0.0) {
        subject.push_back(p);
      }
      if ((sp >= 0.0) != (sq >= 0.0)) {
        subject.emplace_back(p + (q - p) * (sp / (sp - sq)));
      }
    }
  }
  return subject;
}

double footprint_iou(const Polygon& a, const Polygon& b) {
  const double overlap = area(intersection(a, b));
  const double united = area(a) + area(b) - overlap;
  return united > 0.0 ? overlap / united : 0.0;
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

// Footprint errors of matched labelled vehicles.
struct FootprintAccumulator {
  FootprintMetrics counts;
  std::vector<double> axis;
  std::vector<double> stationary_axis;
  std::vector<double> length;  // signed: predicted minus label
  std::vector<double> width;
  std::vector<double> iou;

  [[nodiscard]] FootprintMetrics finish() const {
    const auto absolute = [](std::vector<double> values) {
      for (double& v : values) {
        v = std::abs(v);
      }
      return error_stats(values);
    };
    FootprintMetrics m = counts;
    m.axis_deg = error_stats(axis);
    m.stationary_axis_deg = error_stats(stationary_axis);
    m.length_m = absolute(length);
    m.width_m = absolute(width);
    m.length_bias_m = error_stats(length);
    m.width_bias_m = error_stats(width);
    m.iou = error_stats(iou);
    return m;
  }
};

// Errors collected over all frames, turned into Metrics at the end.
struct Accumulator {
  FootprintAccumulator footprint;
  std::map<std::string, FootprintAccumulator> class_footprint;
  std::vector<double> position;
  std::vector<double> range_bias;
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
  }

 private:
  void evaluate_frame(std::size_t f) {
    const Frame& frame = sequence_->frames[f];
    ++metrics_->frames;
    const auto matches = match_frame(frame, *options_);
    std::vector<bool> dont_care(frame.predicted.size(), false);
    std::vector<bool> matched_visible(frame.predicted.size(), false);
    if (options_->ignore_unlabelled) {
      ignore_unlabelled(frame, matches, dont_care);
    }
    for (std::size_t t = 0; t < frame.truth.size(); ++t) {
      const TruthObject& truth = frame.truth[t];
      const std::size_t p = matches[t];
      if (p != kNone) {
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

  // Unmatched predictions with no label within the displaced gate become "don't care".
  void ignore_unlabelled(const Frame& frame, const std::vector<std::size_t>& matches,
                         std::vector<bool>& dont_care) {
    std::vector<bool> matched(frame.predicted.size(), false);
    for (const std::size_t p : matches) {
      if (p != kNone) {
        matched[p] = true;
      }
    }
    for (std::size_t p = 0; p < frame.predicted.size(); ++p) {
      const bool near_label =
          std::any_of(frame.truth.begin(), frame.truth.end(), [&](const TruthObject& t) {
            return (t.position - frame.predicted[p].position).norm() <= options_->displaced_gate_m;
          });
      if (!matched[p] && !near_label) {
        dont_care[p] = true;
        ++metrics_->unlabelled;
      }
    }
  }

  // Heading, size and footprint of a matched labelled vehicle; velocity of the label if known.
  void count_footprint(const TruthObject& truth, const PredictedObject& prediction,
                       const std::optional<Eigen::Vector2d>& velocity) {
    const bool stationary = velocity && velocity->norm() < options_->moving_speed_mps;
    for (FootprintAccumulator* acc : {&acc_->footprint, &acc_->class_footprint[truth.twin_class]}) {
      auto& c = acc->counts;
      ++c.vehicles;
      c.stationary += stationary ? 1 : 0;
      if (prediction.size_m.x() > 0.0) {
        c.measured_size += prediction.measured_size ? 1 : 0;
        acc->length.push_back(prediction.size_m.x() - truth.length_m);
        acc->width.push_back(prediction.size_m.y() - truth.width_m);
      }
      if (!prediction.heading_rad) {
        continue;
      }
      ++c.with_heading;
      const double error = angle_error_deg(*prediction.heading_rad, truth.yaw_rad);
      const double axis = std::min(error, 180.0 - error);
      c.front_correct += error < 90.0 ? 1 : 0;
      acc->axis.push_back(axis);
      if (stationary) {
        ++c.stationary_with_heading;
        acc->stationary_axis.push_back(axis);
      }
      if (prediction.size_m.x() > 0.0) {
        acc->iou.push_back(
            footprint_iou(rectangle(truth.position, truth.yaw_rad, truth.length_m, truth.width_m),
                          rectangle(prediction.position, *prediction.heading_rad,
                                    prediction.size_m.x(), prediction.size_m.y())));
      }
    }
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
    acc_->range_bias.push_back((prediction->position - sequence_->camera_position).norm() - range);
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
    if (!truth.vru) {
      count_footprint(truth, *prediction, velocity);
    }
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

  const Sequence* sequence_;
  const Options* options_;
  Metrics* metrics_;
  Accumulator* acc_;
  TruthTracks tracks_;
  std::map<std::int64_t, std::int64_t> last_match_;
  std::map<std::pair<std::int64_t, std::int64_t>, std::size_t> co_occurrence_;
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
  metrics.range_bias_m = error_stats(acc.range_bias);
  metrics.heading_deg = error_stats(acc.heading);
  metrics.displaced_m = error_stats(acc.displaced);
  metrics.speed_mps = error_stats(acc.speed);
  metrics.footprint = acc.footprint.finish();
  for (const auto& [name, footprint] : acc.class_footprint) {
    metrics.footprint_by_class[name] = footprint.finish();
  }
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
