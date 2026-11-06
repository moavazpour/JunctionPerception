#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include <CLI/CLI.hpp>
#include <Eigen/Geometry>
#include <filesystem>
#include <format>
#include <fstream>
#include <map>
#include <nlohmann/json.hpp>
#include <numbers>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "jp/benchmark/ground_truth.hpp"
#include "jp/benchmark/tumtraf_sequence.hpp"
#include "jp/calibration/ground_calibration.hpp"
#include "jp/core/class_catalog.hpp"
#include "jp/core/rigid_transform.hpp"
#include "jp/core/version.hpp"
#include "jp/evaluation/evaluation.hpp"
#include "jp/twin/world_state_json.hpp"

namespace {

using json = nlohmann::ordered_json;
using jp::evaluation::Metrics;

struct RunSpec {
  std::string label;
  std::string condition;
  std::filesystem::path world;
  std::filesystem::path sequence;
  std::filesystem::path scene;
};

RunSpec parse_run(const std::string& text) {
  std::vector<std::string> parts;
  std::stringstream stream(text);
  for (std::string part; std::getline(stream, part, ',');) {
    parts.push_back(part);
  }
  if (parts.size() != 5) {
    throw std::runtime_error("--run expects LABEL,CONDITION,WORLD_JSONL,SEQUENCE_DIR,SCENE_DIR: " +
                             text);
  }
  return {parts[0], parts[1], parts[2], parts[3], parts[4]};
}

jp::evaluation::Options load_options(const std::filesystem::path& path) {
  const YAML::Node root = YAML::LoadFile(path.string());
  jp::evaluation::Options o;
  o.vehicle_gate_m = root["vehicle_gate_m"].as<double>();
  o.vru_gate_m = root["vru_gate_m"].as<double>();
  o.displaced_gate_m = root["displaced_gate_m"].as<double>();
  o.velocity_window_s = root["velocity_window_s"].as<double>();
  o.moving_speed_mps = root["moving_speed_mps"].as<double>();
  o.range_bins_m = root["range_bins_m"].as<std::vector<double>>();
  return o;
}

jp::evaluation::ArticulatedMerge load_articulated(const std::filesystem::path& path) {
  const YAML::Node articulated = YAML::LoadFile(path.string())["articulated"];
  return {articulated["max_gap_m"].as<double>(), articulated["max_angle_deg"].as<double>()};
}

// Labels of every frame in the scene's ground frame, flagged visible if the box centre projects
// into the camera image.
std::vector<std::vector<jp::evaluation::TruthObject>> load_truth(
    const std::filesystem::path& sequence_dir, const jp::calibration::GroundCalibration& ground,
    const jp::core::ClassCatalog& catalog, jp::benchmark::GroundTruthLabels labels,
    const std::optional<jp::evaluation::ArticulatedMerge>& articulated) {
  if (!ground.sensor_from_ground || !ground.ground_projection) {
    throw std::runtime_error("the scene has no benchmark calibration (sensor_from_ground)");
  }
  const Eigen::Isometry3d ground_from_sensor = jp::core::rigid_inverse(*ground.sensor_from_ground);
  const auto sequence = jp::benchmark::TumtrafSequence::open(sequence_dir);
  const auto truth = jp::benchmark::load_ground_truth(sequence, labels);
  std::vector<std::vector<jp::evaluation::TruthObject>> frames;
  std::set<std::string> unmapped;
  for (const auto& objects : truth.frames) {
    auto& out = frames.emplace_back();
    for (const auto& object : objects) {
      const auto twin_class = catalog.map("tumtraf", object.box.category);
      if (!twin_class) {
        unmapped.insert(object.box.category);
        continue;
      }
      const Eigen::Vector3d bottom = ground_from_sensor * object.box.bottom_center();
      const Eigen::Vector3d forward =
          ground_from_sensor.linear() * (object.box.rotation * Eigen::Vector3d::UnitX());
      const Eigen::Vector3d image =
          *ground.ground_projection * (ground_from_sensor * object.box.center).homogeneous();
      const Eigen::Vector2d pixel = image.hnormalized();
      jp::evaluation::TruthObject t;
      t.track_id = object.track_id;
      t.twin_class = *twin_class;
      t.vru = catalog.info(*twin_class).vru;
      t.position = bottom.head<2>();
      t.yaw_rad = std::atan2(forward.y(), forward.x());
      t.length_m = object.box.size.x();
      t.width_m = object.box.size.y();
      t.height_m = object.box.size.z();
      t.visible = image.z() > 0.0 && pixel.x() >= 0.0 && pixel.y() >= 0.0 &&
                  pixel.x() < ground.width_px && pixel.y() < ground.height_px;
      out.push_back(std::move(t));
    }
    if (articulated) {
      jp::evaluation::merge_articulated(out, *articulated);
    }
  }
  for (const auto& category : unmapped) {
    spdlog::warn("label category {} has no twin class and is ignored", category);
  }
  return frames;
}

jp::evaluation::Sequence load_run(
    const RunSpec& run, const jp::core::ClassCatalog& catalog,
    jp::benchmark::GroundTruthLabels labels,
    const std::optional<jp::evaluation::ArticulatedMerge>& articulated) {
  const auto ground =
      jp::calibration::load_ground_calibration(run.scene / "ground_homography.yaml");
  const auto truth = load_truth(run.sequence, ground, catalog, labels, articulated);
  jp::evaluation::Sequence sequence;
  sequence.label = run.label;
  sequence.condition = run.condition;
  sequence.camera_position = ground.camera_ground_position_m;
  jp::twin::JsonlReader reader(run.world);
  while (const auto state = reader.next()) {
    const auto index = static_cast<std::size_t>(state->frame);
    if (state->frame < 0 || index >= truth.size()) {
      continue;
    }
    jp::evaluation::Frame frame;
    frame.index = state->frame;
    frame.timestamp_ns = state->timestamp_ns;
    frame.truth = truth[index];
    for (const auto& o : state->objects) {
      // The size the twin shows: measured by a keypoint model, else the class size.
      const auto& cls = catalog.info(o.object_class);
      const bool measured = o.length_m && o.width_m;
      const Eigen::Vector2d size = measured ? Eigen::Vector2d(*o.length_m, *o.width_m)
                                            : Eigen::Vector2d(cls.length_m, cls.width_m);
      frame.predicted.push_back({o.id,
                                 o.object_class,
                                 o.vru,
                                 {o.x, o.y},
                                 o.heading_rad,
                                 o.speed_mps,
                                 size,
                                 measured,
                                 cls.height_m});
    }
    sequence.frames.push_back(std::move(frame));
  }
  spdlog::info("{}: {} frames of {}, {} label frames", run.label, sequence.frames.size(),
               run.world.string(), truth.size());
  return sequence;
}

double ratio(std::size_t part, std::size_t whole) {
  return whole == 0 ? 0.0 : static_cast<double>(part) / static_cast<double>(whole);
}

json stats_json(const jp::evaluation::ErrorStats& s) {
  return {{"count", s.count}, {"mean", s.mean}, {"median", s.median}, {"p90", s.p90}};
}

json footprint_json(const jp::evaluation::FootprintMetrics& f) {
  return {{"vehicles", f.vehicles},
          {"with_heading", f.with_heading},
          {"stationary", f.stationary},
          {"stationary_with_heading", f.stationary_with_heading},
          {"front_correct", f.front_correct},
          {"measured_size", f.measured_size},
          {"axis_deg", stats_json(f.axis_deg)},
          {"stationary_axis_deg", stats_json(f.stationary_axis_deg)},
          {"length_m", stats_json(f.length_m)},
          {"width_m", stats_json(f.width_m)},
          {"length_bias_m", stats_json(f.length_bias_m)},
          {"width_bias_m", stats_json(f.width_bias_m)},
          {"iou", stats_json(f.iou)},
          {"iou_3d", stats_json(f.iou_3d)}};
}

json metrics_json(const Metrics& m) {
  json classes = json::object();
  for (const auto& [name, c] : m.by_class) {
    const double recall = ratio(c.matched, c.truth);
    const double precision = ratio(c.predicted - c.false_positives, c.predicted);
    classes[name] = {{"truth", c.truth},
                     {"matched", c.matched},
                     {"recall", recall},
                     {"predicted", c.predicted},
                     {"false_positives", c.false_positives},
                     {"precision", precision},
                     {"position_m", stats_json(c.position_m)}};
  }
  json ranges = json::array();
  for (const auto& r : m.by_range) {
    ranges.push_back({{"min_m", r.min_m},
                      {"max_m", r.max_m},
                      {"truth", r.truth},
                      {"matched", r.matched},
                      {"recall", ratio(r.matched, r.truth)},
                      {"position_m", stats_json(r.position_m)}});
  }
  json footprint_classes = json::object();
  for (const auto& [name, f] : m.footprint_by_class) {
    footprint_classes[name] = footprint_json(f);
  }
  return {{"frames", m.frames},
          {"truth", m.truth},
          {"matched", m.matched},
          {"false_positives", m.false_positives},
          {"unlabelled", m.unlabelled},
          {"precision", m.precision()},
          {"recall", m.recall()},
          {"position_m", stats_json(m.position_m)},
          {"range_bias_m", stats_json(m.range_bias_m)},
          {"heading_deg", stats_json(m.heading_deg)},
          {"moving_without_heading", m.moving_without_heading},
          {"speed_mps", stats_json(m.speed_mps)},
          {"displaced", m.displaced},
          {"displaced_m", stats_json(m.displaced_m)},
          {"recall_within_displaced_gate", ratio(m.matched + m.displaced, m.truth)},
          {"precision_within_displaced_gate",
           ratio(m.matched + m.displaced, m.matched + m.false_positives)},
          {"tracking",
           {{"mota", m.tracking.mota},
            {"idf1", m.tracking.idf1},
            {"id_switches", m.tracking.id_switches}}},
          {"footprint", footprint_json(m.footprint)},
          {"footprint_by_class", footprint_classes},
          {"by_class", classes},
          {"by_range", ranges},
          {"confusion", m.confusion}};
}

std::string summary_row(const std::string& name, const Metrics& m) {
  return std::format(
      "| {} | {} | {} | {:.3f} | {:.3f} | {:.2f} / {:.2f} / {:.2f} | {:.1f} / {:.1f} | "
      "{:.2f} / {:.2f} | {:.3f} | {:.3f} | {} |\n",
      name, m.frames, m.truth, m.precision(), m.recall(), m.position_m.mean, m.position_m.median,
      m.position_m.p90, m.heading_deg.median, m.heading_deg.p90, m.speed_mps.median,
      m.speed_mps.p90, m.tracking.mota, m.tracking.idf1, m.tracking.id_switches);
}

// Share of a count as "part (percent)".
std::string share(std::size_t part, std::size_t whole) {
  return std::format("{} ({:.0f} %)", part, 100.0 * ratio(part, whole));
}

std::string footprint_row(const std::string& name, const jp::evaluation::FootprintMetrics& f) {
  return std::format(
      "| {} | {} | {} | {} | {:.1f} / {:.1f} / {:.1f} | {:.1f} / {:.1f} / {:.1f} | {} | {} | "
      "{:.2f} / {:.2f} / {:.2f} | {:.2f} / {:.2f} / {:.2f} | {:.2f} / {:.2f} / {:.2f} | "
      "{:.2f} / {:.2f} / {:.2f} |\n",
      name, f.vehicles, share(f.with_heading, f.vehicles),
      share(f.stationary_with_heading, f.stationary), f.axis_deg.mean, f.axis_deg.median,
      f.axis_deg.p90, f.stationary_axis_deg.mean, f.stationary_axis_deg.median,
      f.stationary_axis_deg.p90, share(f.front_correct, f.with_heading),
      share(f.measured_size, f.vehicles), f.length_m.mean, f.length_m.median, f.length_m.p90,
      f.width_m.mean, f.width_m.median, f.width_m.p90, f.iou.mean, f.iou.median, f.iou.p90,
      f.iou_3d.mean, f.iou_3d.median, f.iou_3d.p90);
}

std::string report(const std::string& title, const jp::evaluation::Options& options,
                   const std::vector<std::pair<std::string, Metrics>>& groups,
                   const Metrics& overall) {
  std::string md = std::format("# Evaluation: {}\n\n", title);
  md += std::format(
      "Matching gates: {:.1f} m (vehicles), {:.1f} m (VRUs). Labels count if their box centre "
      "projects into the camera image. Heading error: labelled vehicles moving at {:.1f} m/s or "
      "more. Errors: mean / median / P90.\n\n",
      options.vehicle_gate_m, options.vru_gate_m, options.moving_speed_mps);
  if (options.ignore_unlabelled) {
    md += std::format(
        "Unlabelled road users ignored: {} predictions matched to no label and farther than "
        "{:.1f} m from every label are neither true nor false positives.\n\n",
        overall.unlabelled, options.displaced_gate_m);
  }
  md +=
      "| Group | Frames | Labels | Precision | Recall | Position error (m) | Heading error (deg) "
      "| Speed error (m/s) | MOTA | IDF1 | ID switches |\n";
  md += "|---|---|---|---|---|---|---|---|---|---|---|\n";
  for (const auto& [name, m] : groups) {
    md += summary_row(name, m);
  }
  md += std::format(
      "\n## Localization beyond the gate\n\nA missed label and a false positive within {:.1f} m "
      "of each other are counted as one displaced object: detected, but placed beyond the "
      "matching gate.\n\n| Group | Displaced | Displaced error (m) | Precision within {:.1f} m | "
      "Recall within {:.1f} m |\n|---|---|---|---|---|\n",
      options.displaced_gate_m, options.displaced_gate_m, options.displaced_gate_m);
  for (const auto& [name, m] : groups) {
    md += std::format("| {} | {} | {:.2f} / {:.2f} / {:.2f} | {:.3f} | {:.3f} |\n", name,
                      m.displaced, m.displaced_m.mean, m.displaced_m.median, m.displaced_m.p90,
                      ratio(m.matched + m.displaced, m.matched + m.false_positives),
                      ratio(m.matched + m.displaced, m.truth));
  }
  md +=
      "\n## Footprint of labelled vehicles\n\nMatched labelled vehicles. Heading against the "
      "label box yaw modulo 180 degrees (axis, 0-90); front correct: within 90 degrees of the "
      "label's front. Standing: label slower than the moving speed. Size: the footprint the twin "
      "shows (measured by the detector, else the class size). IoU: bird's-eye-view footprints of "
      "predictions with a heading; 3D IoU: the same boxes with the label height and the class "
      "height, both standing on the ground.\n\n| Group | Vehicles | With heading | Standing with "
      "heading | Axis error (deg) | Standing axis error (deg) | Front correct | Measured size | "
      "Length error (m) | Width error (m) | BEV IoU | 3D IoU |\n"
      "|---|---|---|---|---|---|---|---|---|---|---|---|\n";
  for (const auto& [name, m] : groups) {
    md += footprint_row(name, m.footprint);
  }
  md +=
      "\n### Bias\n\nSigned errors, mean / median: distance from the camera of matched objects, "
      "length and width of matched vehicles (predicted minus label).\n\n| Group | Range bias (m) "
      "| Length bias (m) | Width bias (m) |\n|---|---|---|---|\n";
  for (const auto& [name, m] : groups) {
    md += std::format("| {} | {:+.2f} / {:+.2f} | {:+.2f} / {:+.2f} | {:+.2f} / {:+.2f} |\n", name,
                      m.range_bias_m.mean, m.range_bias_m.median, m.footprint.length_bias_m.mean,
                      m.footprint.length_bias_m.median, m.footprint.width_bias_m.mean,
                      m.footprint.width_bias_m.median);
  }
  md +=
      "\n### By label class (all runs)\n\n| Class | Vehicles | With heading | Standing with "
      "heading | Axis error (deg) | Standing axis error (deg) | Front correct | Measured size | "
      "Length error (m) | Width error (m) | BEV IoU | 3D IoU |\n"
      "|---|---|---|---|---|---|---|---|---|---|---|---|\n";
  for (const auto& [name, f] : overall.footprint_by_class) {
    md += footprint_row(name, f);
  }
  md +=
      "\n## Classes (all runs)\n\n| Class | Labels | Recall | Predictions | Precision | "
      "Position error (m) |\n|---|---|---|---|---|---|\n";
  for (const auto& [name, c] : overall.by_class) {
    md += std::format("| {} | {} | {:.3f} | {} | {:.3f} | {:.2f} / {:.2f} / {:.2f} |\n", name,
                      c.truth, ratio(c.matched, c.truth), c.predicted,
                      ratio(c.predicted - c.false_positives, c.predicted), c.position_m.mean,
                      c.position_m.median, c.position_m.p90);
  }
  md +=
      "\n## Distance from the camera (all runs)\n\n| Range (m) | Labels | Recall | Position "
      "error (m) |\n|---|---|---|---|\n";
  for (const auto& r : overall.by_range) {
    md += std::format("| {:.0f}–{:.0f} | {} | {:.3f} | {:.2f} / {:.2f} / {:.2f} |\n", r.min_m,
                      r.max_m, r.truth, ratio(r.matched, r.truth), r.position_m.mean,
                      r.position_m.median, r.position_m.p90);
  }
  md +=
      "\n## Class confusion (all runs, matched labels)\n\n| Label \\ predicted | Count "
      "|\n|---|---|\n";
  for (const auto& [label, row] : overall.confusion) {
    for (const auto& [predicted, count] : row) {
      md += std::format("| {} → {} | {} |\n", label, predicted, count);
    }
  }
  return md;
}

const char* status_name(jp::evaluation::TwinObjectStatus status) {
  switch (status) {
    case jp::evaluation::TwinObjectStatus::kCorrect:
      return "correct";
    case jp::evaluation::TwinObjectStatus::kMisplaced:
      return "misplaced";
    case jp::evaluation::TwinObjectStatus::kNoLabel:
      return "no_label";
  }
  return "";
}

json twin_json(const jp::evaluation::TwinAccuracy& a) {
  return {{"objects", a.objects},
          {"correct", a.correct},
          {"misplaced", a.misplaced},
          {"no_label", a.no_label},
          {"correct_share", ratio(a.correct, a.objects)},
          {"correct_share_of_labelled", ratio(a.correct, a.correct + a.misplaced)},
          {"position_m", stats_json(a.position_m)},
          {"heading_deg", stats_json(a.heading_deg)},
          {"iou", stats_json(a.iou)},
          {"iou_3d", stats_json(a.iou_3d)},
          {"labels", a.labels},
          {"labels_found", a.labels_found},
          {"recall", ratio(a.labels_found, a.labels)}};
}

std::string twin_row(const std::string& name, const jp::evaluation::TwinAccuracy& a) {
  return std::format(
      "| {} | {} | {} | {} | {} | {:.2f} / {:.2f} / {:.2f} | {:.1f} / {:.1f} | {:.2f} / {:.2f} | "
      "{} / {} ({:.0f} %) |\n",
      name, a.objects, share(a.correct, a.objects), share(a.misplaced, a.objects),
      share(a.no_label, a.objects), a.position_m.mean, a.position_m.median, a.position_m.p90,
      a.heading_deg.median, a.heading_deg.p90, a.iou.median, a.iou_3d.median, a.labels_found,
      a.labels, 100.0 * ratio(a.labels_found, a.labels));
}

std::string twin_report(
    const jp::evaluation::Options& options,
    const std::vector<std::pair<std::string, jp::evaluation::TwinAccuracy>>& groups) {
  std::string md = std::format(
      "\n## Road users of the twin with a heading\n\nEach road user the twin shows with a heading, "
      "in each frame (objects.csv lists them). Correct: matched to a label within {:.1f} m "
      "(vehicles) or {:.1f} m (VRUs). Misplaced: an unmatched label within {:.1f} m. No label: "
      "nothing labelled near it, an unlabelled road user or a false detection. Road users without "
      "a heading and the labels they match are left out. Position error of the correct ones (m), "
      "heading against the label yaw (deg), IoU medians of correct vehicles. Labels found: visible "
      "labels matched by a road user with a heading.\n\n| Group | Objects | Correct | Misplaced | "
      "No label | Position error (m) | Heading error (deg) | BEV / 3D IoU | Labels found "
      "|\n|---|---|---|---|---|---|---|---|---|\n",
      options.vehicle_gate_m, options.vru_gate_m, options.displaced_gate_m);
  for (const auto& [name, a] : groups) {
    md += twin_row(name, a);
  }
  return md;
}

std::string optional_text(const std::optional<double>& value) {
  return value ? std::format("{:.3f}", *value) : std::string{};
}

std::string objects_csv(const jp::evaluation::TwinAccuracy& accuracy) {
  std::string csv =
      "run,frame,timestamp_ns,id,class,x_m,y_m,heading_deg,status,label_track_id,label_class,"
      "label_x_m,label_y_m,distance_m,heading_error_deg,iou,iou_3d\n";
  for (const auto& r : accuracy.records) {
    const bool labelled = r.label_track_id.has_value();
    csv += std::format("{},{},{},{},{},{:.3f},{:.3f},{:.1f},{},{},{},{},{},{},{},{},{}\n", r.run,
                       r.frame, r.timestamp_ns, r.id, r.twin_class, r.position.x(), r.position.y(),
                       r.heading_rad * 180.0 / std::numbers::pi, status_name(r.status),
                       labelled ? std::to_string(*r.label_track_id) : "", r.label_class,
                       labelled ? std::format("{:.3f}", r.label_position.x()) : "",
                       labelled ? std::format("{:.3f}", r.label_position.y()) : "",
                       labelled ? std::format("{:.3f}", r.distance_m) : "",
                       optional_text(r.heading_error_deg), optional_text(r.iou),
                       optional_text(r.iou_3d));
  }
  return csv;
}

void write_text(const std::filesystem::path& path, const std::string& text) {
  std::ofstream out(path);
  if (!out) {
    throw std::runtime_error("cannot write " + path.string());
  }
  out << text;
}

}  // namespace

int main(int argc, char** argv) {
  CLI::App app{"Evaluates recorded world states against benchmark ground truth", "jp_eval"};
  app.set_version_flag("--version", std::string{jp::core::version()});
  std::vector<std::string> runs;
  std::string classes = "configs/classes.yaml";
  std::string config = "configs/evaluation.yaml";
  std::string out_dir;
  std::string title = "benchmark";
  std::string labels = "south";
  app.add_option("--run", runs, "LABEL,CONDITION,WORLD_JSONL,SEQUENCE_DIR,SCENE_DIR (repeatable)")
      ->required();
  app.add_option("--classes", classes, "Class catalog")->capture_default_str();
  app.add_option("--config", config, "Evaluation settings")->capture_default_str();
  app.add_option("--out", out_dir, "Output folder for metrics.json, report.md and objects.csv")
      ->required();
  app.add_option("--title", title, "Report title")->capture_default_str();
  app.add_option("--labels", labels,
                 "Ground truth: south (south LiDAR labels) or merged (south and north)")
      ->capture_default_str();
  bool ignore_unlabelled = false;
  app.add_flag("--ignore-unlabelled", ignore_unlabelled,
               "Predictions farther than the displaced gate from every label are not counted");
  bool merge_trailers = false;
  app.add_flag("--merge-trailers", merge_trailers,
               "A labelled tractor and the trailer behind it are one truck (as detected)");
  CLI11_PARSE(app, argc, argv);

  try {
    const auto catalog = jp::core::ClassCatalog::load(classes);
    auto options = load_options(config);
    options.ignore_unlabelled = ignore_unlabelled;
    const auto label_set = jp::benchmark::parse_ground_truth_labels(labels);
    if (!label_set) {
      throw std::runtime_error("--labels must be south or merged, not " + labels);
    }
    std::vector<jp::evaluation::Sequence> sequences;
    sequences.reserve(runs.size());
    for (const auto& text : runs) {
      sequences.push_back(
          load_run(parse_run(text), catalog, *label_set,
                   merge_trailers ? std::optional(load_articulated(config)) : std::nullopt));
    }
    const auto select = [&](const auto& keep) {
      std::vector<const jp::evaluation::Sequence*> selected;
      for (const auto& s : sequences) {
        if (keep(s)) {
          selected.push_back(&s);
        }
      }
      return selected;
    };
    const Metrics overall =
        jp::evaluation::evaluate(select([](const auto&) { return true; }), options);
    std::vector<std::pair<std::string, Metrics>> groups = {{"all", overall}};
    json by_condition = json::object();
    std::set<std::string> conditions;
    for (const auto& s : sequences) {
      conditions.insert(s.condition);
    }
    for (const auto& condition : conditions) {
      const auto m = jp::evaluation::evaluate(
          select([&](const auto& s) { return s.condition == condition; }), options);
      by_condition[condition] = metrics_json(m);
      groups.emplace_back(condition, m);
    }
    json by_run = json::object();
    const auto twin =
        jp::evaluation::twin_accuracy(select([](const auto&) { return true; }), options);
    std::vector<std::pair<std::string, jp::evaluation::TwinAccuracy>> twin_groups = {{"all", twin}};
    json twin_by_run = json::object();
    for (const auto& s : sequences) {
      const auto m = jp::evaluation::evaluate({&s}, options);
      by_run[s.label] = metrics_json(m);
      groups.emplace_back(s.label, m);
      const auto a = jp::evaluation::twin_accuracy({&s}, options);
      twin_by_run[s.label] = twin_json(a);
      twin_groups.emplace_back(s.label, a);
    }
    const json metrics = {{"title", title},
                          {"options",
                           {{"vehicle_gate_m", options.vehicle_gate_m},
                            {"vru_gate_m", options.vru_gate_m},
                            {"moving_speed_mps", options.moving_speed_mps},
                            {"ignore_unlabelled", options.ignore_unlabelled},
                            {"merge_trailers", merge_trailers}}},
                          {"overall", metrics_json(overall)},
                          {"by_condition", by_condition},
                          {"by_run", by_run},
                          {"twin_objects", {{"all", twin_json(twin)}, {"by_run", twin_by_run}}}};
    std::filesystem::create_directories(out_dir);
    write_text(std::filesystem::path(out_dir) / "metrics.json", metrics.dump(2) + "\n");
    write_text(std::filesystem::path(out_dir) / "report.md",
               report(title, options, groups, overall) + twin_report(options, twin_groups));
    write_text(std::filesystem::path(out_dir) / "objects.csv", objects_csv(twin));
    spdlog::info("{}: precision {:.3f}, recall {:.3f}, position error median {:.2f} m; wrote {}",
                 title, overall.precision(), overall.recall(), overall.position_m.median, out_dir);
  } catch (const std::exception& error) {
    spdlog::error("{}", error.what());
    return 1;
  }
  return 0;
}
