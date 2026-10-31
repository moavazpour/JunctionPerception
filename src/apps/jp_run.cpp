#include <spdlog/spdlog.h>

#include <CLI/CLI.hpp>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "jp/benchmark/ground_truth.hpp"
#include "jp/benchmark/tumtraf_sequence.hpp"
#include "jp/calibration/ground_calibration.hpp"
#include "jp/core/class_catalog.hpp"
#include "jp/core/pipeline_config.hpp"
#include "jp/core/rigid_transform.hpp"
#include "jp/core/scene_config.hpp"
#include "jp/core/statistics.hpp"
#include "jp/core/version.hpp"
#include "jp/detection/detector.hpp"
#include "jp/detection/rider_association.hpp"
#include "jp/io/frame_source.hpp"
#include "jp/localization/track_localizer.hpp"
#include "jp/sources/perception_pipeline.hpp"
#include "jp/sources/world_source.hpp"
#include "jp/tracking/byte_tracker.hpp"
#include "jp/twin/bev_plot.hpp"
#include "jp/twin/debug_overlay.hpp"
#include "jp/twin/world_state_json.hpp"
#include "jp/vru/event_detector.hpp"
#include "jp/vru/zones.hpp"

namespace {

using References = std::vector<std::vector<jp::twin::ReferenceFootprint>>;

struct RunArgs {
  std::string source;
  double image_sequence_fps = 10.0;
  std::string config = "configs/pipeline.yaml";
  std::string classes = "configs/classes.yaml";
  std::string scene;
  std::string overlay;
  std::string bev;
  std::string reference_sequence;
  std::string tracks_csv;
  std::string reference_csv;
  std::string world_jsonl;
  std::string bias_correction;
  std::int64_t max_frames = 0;
  std::int64_t warmup_frames = 10;
};

// Benchmark ground truth of every frame as ground-plane footprints of the scene; optionally also
// written as CSV (frame, ground truth track, category, footprint) for offline evaluation.
References reference_footprints(const std::filesystem::path& sequence_dir,
                                const jp::calibration::GroundCalibration& ground,
                                const std::string& csv_path) {
  if (!ground.sensor_from_ground) {
    throw std::runtime_error("the scene calibration has no sensor_from_ground pose");
  }
  const Eigen::Isometry3d ground_from_sensor = jp::core::rigid_inverse(*ground.sensor_from_ground);
  const auto sequence = jp::benchmark::TumtrafSequence::open(sequence_dir);
  const auto truth =
      jp::benchmark::load_ground_truth(sequence, jp::benchmark::GroundTruthLabels::kSouth);
  References frames;
  frames.reserve(truth.frames.size());
  std::ofstream csv;
  if (!csv_path.empty()) {
    std::filesystem::create_directories(std::filesystem::path(csv_path).parent_path());
    csv.open(csv_path);
    csv << "frame,track_id,category,x,y,yaw_rad,length_m,width_m\n";
  }
  for (const auto& objects : truth.frames) {
    auto& footprints = frames.emplace_back();
    for (const auto& object : objects) {
      const Eigen::Vector3d center = ground_from_sensor * object.box.bottom_center();
      const Eigen::Vector3d forward =
          ground_from_sensor.linear() * (object.box.rotation * Eigen::Vector3d::UnitX());
      footprints.push_back({center.head<2>(), std::atan2(forward.y(), forward.x()),
                            object.box.size.x(), object.box.size.y()});
      if (csv.is_open()) {
        const auto& f = footprints.back();
        csv << std::format("{},{},{},{:.3f},{:.3f},{:.4f},{:.2f},{:.2f}\n", frames.size() - 1,
                           object.track_id, object.box.category, f.center.x(), f.center.y(),
                           f.yaw_rad, f.length_m, f.width_m);
      }
    }
  }
  return frames;
}

// Debug outputs: overlay video, top-down video, CSV of localized tracks.
class Sinks {
 public:
  Sinks(RunArgs args, jp::core::ClassCatalog catalog,
        const std::optional<jp::calibration::GroundCalibration>& ground, References references,
        double fps)
      : args_(std::move(args)),
        catalog_(std::move(catalog)),
        ground_(ground),
        references_(std::move(references)),
        fps_(fps) {
    if (!args_.tracks_csv.empty()) {
      std::filesystem::create_directories(std::filesystem::path(args_.tracks_csv).parent_path());
      csv_.open(args_.tracks_csv);
      csv_ << "frame,timestamp_ns,id,class,rider,x,y,contact_x,contact_y,speed_mps,heading_rad,"
              "confidence,speed_std_mps\n";
    }
  }

  void write(const jp::io::Frame& frame, const jp::sources::PerceptionPipeline::Result& result,
             bool localized, const std::string& status) {
    if (!args_.overlay.empty()) {
      if (!overlay_) {
        overlay_ =
            std::make_unique<jp::twin::OverlayVideoWriter>(args_.overlay, fps_, frame.image.size());
      }
      overlay_->write(localized
                          ? jp::twin::draw_tracks(frame.image, result.objects, status)
                          : jp::twin::draw_detections(frame.image, result.detections, status));
    }
    if (!args_.bev.empty()) {
      write_bev(frame, result.objects, status);
    }
    if (csv_.is_open()) {
      write_csv(frame, result.objects);
    }
  }

  void log_paths() const {
    for (const auto& path : {args_.overlay, args_.bev, args_.tracks_csv}) {
      if (!path.empty()) {
        spdlog::info("wrote {}", path);
      }
    }
  }

 private:
  void write_bev(const jp::io::Frame& frame,
                 const std::vector<jp::localization::LocalizedObject>& objects,
                 const std::string& status) {
    if (!ground_) {
      return;
    }
    if (!bev_) {
      jp::twin::BevView view;
      const Eigen::Vector3d center =
          ground_->image_to_ground *
          Eigen::Vector3d(0.5 * frame.image.cols, 0.5 * frame.image.rows, 1.0);
      view.center = center.hnormalized();
      bev_ = std::make_unique<jp::twin::BevPlotWriter>(args_.bev, fps_, view, catalog_,
                                                       ground_->camera_ground_position_m);
    }
    const auto index = static_cast<std::size_t>(frame.index);
    bev_->write(objects, index < references_.size() ? references_[index] : no_references_, status);
  }

  void write_csv(const jp::io::Frame& frame,
                 const std::vector<jp::localization::LocalizedObject>& objects) {
    for (const auto& o : objects) {
      csv_ << std::format("{},{},{},{},{},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{},{:.3f},{:.3f}\n",
                          frame.index, frame.timestamp_ns, o.id, o.twin_class, o.rider ? 1 : 0,
                          o.position.x(), o.position.y(), o.contact.x(), o.contact.y(), o.speed_mps,
                          o.heading_rad ? std::format("{:.4f}", *o.heading_rad) : std::string(),
                          o.confidence, o.speed_std_mps);
    }
  }

  RunArgs args_;
  jp::core::ClassCatalog catalog_;
  std::optional<jp::calibration::GroundCalibration> ground_;
  References references_;
  std::vector<jp::twin::ReferenceFootprint> no_references_;
  double fps_;
  std::unique_ptr<jp::twin::OverlayVideoWriter> overlay_;
  std::unique_ptr<jp::twin::BevPlotWriter> bev_;
  std::ofstream csv_;
};

// Timing and class statistics over a run.
class RunStatistics {
 public:
  explicit RunStatistics(std::int64_t warmup_frames) : warmup_frames_(warmup_frames) {}

  void record(const jp::io::Frame& frame, const jp::sources::PerceptionPipeline::Result& result,
              const jp::detection::DetectorTimings& timings) {
    ++frames_;
    if (frame.index >= warmup_frames_) {
      preprocess_.push_back(timings.preprocess_ms);
      inference_.push_back(timings.inference_ms);
      postprocess_.push_back(timings.postprocess_ms);
      total_.push_back(result.elapsed_ms);
    }
    for (const auto& d : result.detections) {
      ++detections_[label(d.twin_class, d.rider)];
    }
    for (const auto& t : result.tracks) {
      track_classes_[t.id] = label(t.twin_class, t.rider);
    }
  }

  void log(const std::string& backend) const {
    spdlog::info("{} frames, detector {} ({} warm-up frames excluded from timing):", frames_,
                 backend, warmup_frames_);
    log_summary("preprocess", preprocess_);
    log_summary("inference", inference_);
    log_summary("postprocess", postprocess_);
    log_summary("frame total", total_);
    const auto total = jp::core::summarize(total_);
    if (total.mean > 0.0) {
      spdlog::info("  throughput   {:.1f} fps (decoding excluded)", 1000.0 / total.mean);
    }
    for (const auto& [twin_class, count] : detections_) {
      spdlog::info("  {:<24} {:6d} detections", twin_class, count);
    }
    std::map<std::string, std::size_t> tracks_per_class;
    for (const auto& [id, twin_class] : track_classes_) {
      ++tracks_per_class[twin_class];
    }
    for (const auto& [twin_class, count] : tracks_per_class) {
      spdlog::info("  {:<24} {:6d} confirmed tracks", twin_class, count);
    }
  }

 private:
  static std::string label(const std::string& twin_class, bool rider) {
    return rider ? twin_class + " (with rider)" : twin_class;
  }

  static void log_summary(std::string_view stage, const std::vector<double>& values) {
    const auto s = jp::core::summarize(values);
    spdlog::info("  {:<12} mean {:7.2f} ms, median {:7.2f} ms, P95 {:7.2f} ms, max {:7.2f} ms",
                 stage, s.mean, s.median, s.p95, s.max);
  }

  std::int64_t warmup_frames_;
  std::int64_t frames_ = 0;
  std::vector<double> preprocess_;
  std::vector<double> inference_;
  std::vector<double> postprocess_;
  std::vector<double> total_;
  std::map<std::string, std::size_t> detections_;
  std::map<std::int64_t, std::string> track_classes_;
};

// World states with zones and events, one JSON line per frame (source "live").
class WorldOutput {
 public:
  WorldOutput(const std::filesystem::path& path, const std::filesystem::path& scene_dir,
              const jp::core::EventSettings& events, jp::core::ClassCatalog catalog,
              const jp::calibration::GroundCalibration& ground)
      : writer_(path),
        scene_id_(scene_dir.filename().string()),
        events_(jp::vru::load_scene_zones(jp::core::load_scene_config(scene_dir)),
                jp::vru::event_options(events, catalog)),
        catalog_(std::move(catalog)) {
    const auto scene = jp::core::load_scene_config(scene_dir);
    if (auto region = jp::sources::validated_region(scene, ground, events)) {
      events_.set_validated_region(
          [region = std::move(*region)](const Eigen::Vector2d& p) { return region.contains(p); });
    }
  }

  void write(const jp::io::Frame& frame,
             const std::vector<jp::localization::LocalizedObject>& objects) {
    jp::core::WorldState state;
    state.scene_id = scene_id_;
    state.frame = frame.index;
    state.timestamp_ns = frame.timestamp_ns;
    state.source = "live";
    for (const auto& object : objects) {
      state.objects.push_back(jp::sources::to_world_object(object, catalog_));
    }
    state.events = events_.update(state.objects, frame.timestamp_ns);
    for (const auto& event : state.events) {
      if (event.state == "ended") {
        ++events_per_type_[event.type];
      }
    }
    writer_.write(state);
  }

  void log() const {
    spdlog::info("  {} zones; completed events{}", events_.zones().size(),
                 events_per_type_.empty() ? ": none" : ":");
    for (const auto& [type, count] : events_per_type_) {
      spdlog::info("    {:<24} {:6d}", type, count);
    }
  }

 private:
  jp::twin::JsonlWriter writer_;
  std::string scene_id_;
  jp::vru::EventDetector events_;
  jp::core::ClassCatalog catalog_;
  std::map<std::string, std::size_t> events_per_type_;
};

void run(const RunArgs& args) {
  auto config = jp::core::load_pipeline_config(args.config, std::filesystem::current_path());
  if (!args.bias_correction.empty()) {
    config.localization.bias_correction = args.bias_correction;
  }
  const auto catalog = jp::core::ClassCatalog::load(args.classes);
  std::optional<jp::calibration::GroundCalibration> ground;
  if (!args.scene.empty()) {
    ground = jp::calibration::load_ground_calibration(std::filesystem::path(args.scene) /
                                                      "ground_homography.yaml");
    if (ground->ground_projection_estimated) {
      spdlog::info("camera model estimated from the ground homography");
    }
  }
  if ((!args.bev.empty() || !args.tracks_csv.empty() || !args.reference_sequence.empty() ||
       !args.world_jsonl.empty()) &&
      !ground) {
    throw std::runtime_error("--bev, --tracks-csv, --world-jsonl and --reference need --scene");
  }
  std::optional<WorldOutput> world;
  if (!args.world_jsonl.empty() && ground) {
    const std::filesystem::path scene_dir(args.scene);
    world.emplace(args.world_jsonl, scene_dir, config.events, catalog, *ground);
  }

  jp::sources::PerceptionPipeline pipeline(config, catalog, ground);
  const auto source = jp::io::open_frame_source(args.source, args.image_sequence_fps);
  spdlog::info("model {} on {}; source {} ({:.2f} fps, {} frames); bias correction {}",
               config.detection.model.string(), pipeline.detector().backend(), args.source,
               source->fps(), source->frame_count().value_or(-1),
               config.localization.bias_correction);
  References references;
  if (!args.reference_sequence.empty() && ground) {
    references = reference_footprints(args.reference_sequence, *ground, args.reference_csv);
  }
  Sinks sinks(args, catalog, ground, std::move(references), source->fps());
  RunStatistics statistics(args.warmup_frames);

  std::int64_t frames = 0;
  while (auto frame = source->next()) {
    if (args.max_frames > 0 && frames++ >= args.max_frames) {
      break;
    }
    const auto result = pipeline.process(*frame);
    statistics.record(*frame, result, pipeline.detector().last_timings());
    const std::string status =
        std::format("frame {}  t={:.3f} s  {} detections  {} tracks  {:.1f} ms ({})", frame->index,
                    static_cast<double>(frame->timestamp_ns) * 1e-9, result.detections.size(),
                    result.tracks.size(), result.elapsed_ms, pipeline.detector().backend());
    sinks.write(*frame, result, pipeline.localizes(), status);
    if (world) {
      world->write(*frame, result.objects);
    }
  }
  statistics.log(pipeline.detector().backend());
  if (world) {
    world->log();
    spdlog::info("wrote {}", args.world_jsonl);
  }
  sinks.log_paths();
}

}  // namespace

int main(int argc, char** argv) {
  CLI::App app{"Runs the perception pipeline on a video source and writes the world state",
               "jp_run"};
  app.set_version_flag("--version", std::string{jp::core::version()});

  RunArgs args;
  app.add_option("--source", args.source, "Video file or folder of images")->required();
  app.add_option("--fps", args.image_sequence_fps, "Frame rate of an image folder")
      ->capture_default_str();
  app.add_option("--config", args.config, "Pipeline config")->capture_default_str();
  app.add_option("--classes", args.classes, "Class catalog")->capture_default_str();
  app.add_option("--scene", args.scene, "Scene folder with ground_homography.yaml");
  app.add_option("--overlay", args.overlay, "Write a debug overlay video (MP4)");
  app.add_option("--bev", args.bev, "Write a top-down debug video (MP4)");
  app.add_option("--reference", args.reference_sequence,
                 "TUMTraf sequence folder whose labels are drawn in the top-down video");
  app.add_option("--tracks-csv", args.tracks_csv, "Write localized tracks per frame (CSV)");
  app.add_option("--reference-csv", args.reference_csv,
                 "Write the --reference labels per frame as ground-plane footprints (CSV)");
  app.add_option("--world-jsonl", args.world_jsonl,
                 "Write world states with zones and events per frame (JSONL)");
  app.add_option("--bias-correction", args.bias_correction,
                 "Override the contact model of the config: box_fit, ray_offset or none")
      ->check(CLI::IsMember({"box_fit", "ray_offset", "none"}));
  app.add_option("--max-frames", args.max_frames, "Stop after this many frames (0: all)")
      ->capture_default_str();
  app.add_option("--warmup", args.warmup_frames, "Frames excluded from the timing statistics")
      ->capture_default_str();

  CLI11_PARSE(app, argc, argv);

  try {
    run(args);
  } catch (const std::exception& error) {
    spdlog::error("{}", error.what());
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
