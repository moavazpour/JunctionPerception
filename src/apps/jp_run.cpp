#include <spdlog/spdlog.h>

#include <CLI/CLI.hpp>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "jp/core/class_catalog.hpp"
#include "jp/core/pipeline_config.hpp"
#include "jp/core/statistics.hpp"
#include "jp/core/version.hpp"
#include "jp/detection/detector.hpp"
#include "jp/detection/rider_association.hpp"
#include "jp/io/frame_source.hpp"
#include "jp/twin/debug_overlay.hpp"

namespace {

struct RunArgs {
  std::string source;
  double image_sequence_fps = 10.0;
  std::string config = "configs/pipeline.yaml";
  std::string classes = "configs/classes.yaml";
  std::string overlay;
  std::int64_t max_frames = 0;
  std::int64_t warmup_frames = 10;
};

struct Timings {
  std::vector<double> preprocess;
  std::vector<double> inference;
  std::vector<double> postprocess;
  std::vector<double> frame;
};

void log_summary(std::string_view stage, const std::vector<double>& values) {
  const auto s = jp::core::summarize(values);
  spdlog::info("  {:<12} mean {:7.2f} ms, median {:7.2f} ms, P95 {:7.2f} ms, max {:7.2f} ms", stage,
               s.mean, s.median, s.p95, s.max);
}

void run(const RunArgs& args) {
  const auto config = jp::core::load_pipeline_config(args.config, std::filesystem::current_path());
  const auto catalog = jp::core::ClassCatalog::load(args.classes);
  const auto detector = jp::detection::make_onnx_detector(
      {config.detection.model, config.detection.confidence_threshold, config.detection.use_cuda,
       config.detection.cuda_device});
  jp::detection::RiderAssociationOptions rider_options;
  rider_options.min_person_overlap = config.riders.min_person_overlap;
  rider_options.min_feet_depth = config.riders.min_feet_depth;
  const auto source = jp::io::open_frame_source(args.source, args.image_sequence_fps);
  spdlog::info("model {} on {}; source {} ({:.2f} fps, {} frames)", config.detection.model.string(),
               detector->backend(), args.source, source->fps(), source->frame_count().value_or(-1));

  std::unique_ptr<jp::twin::OverlayVideoWriter> overlay;
  Timings timings;
  std::map<std::string, std::size_t> per_class;
  std::int64_t frames = 0;
  while (auto frame = source->next()) {
    if (args.max_frames > 0 && frames >= args.max_frames) {
      break;
    }
    const auto start = std::chrono::steady_clock::now();
    auto detections = jp::detection::map_to_twin_classes(detector->detect(frame->image), catalog,
                                                         config.detection.class_source);
    if (config.riders.enabled) {
      detections = jp::detection::associate_riders(std::move(detections), rider_options);
    }
    const double frame_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    ++frames;

    if (frame->index >= args.warmup_frames) {
      const auto t = detector->last_timings();
      timings.preprocess.push_back(t.preprocess_ms);
      timings.inference.push_back(t.inference_ms);
      timings.postprocess.push_back(t.postprocess_ms);
      timings.frame.push_back(frame_ms);
    }
    for (const auto& d : detections) {
      ++per_class[d.rider ? d.twin_class + " (with rider)" : d.twin_class];
    }

    if (!args.overlay.empty()) {
      if (!overlay) {
        overlay = std::make_unique<jp::twin::OverlayVideoWriter>(args.overlay, source->fps(),
                                                                 frame->image.size());
      }
      const std::string status =
          std::format("frame {}  t={:.3f} s  {} detections  {:.1f} ms ({})", frame->index,
                      static_cast<double>(frame->timestamp_ns) * 1e-9, detections.size(), frame_ms,
                      detector->backend());
      overlay->write(jp::twin::draw_detections(frame->image, detections, status));
    }
  }

  const auto total = jp::core::summarize(timings.frame);
  spdlog::info("{} frames, detector {} ({} warm-up frames excluded from timing):", frames,
               detector->backend(), args.warmup_frames);
  log_summary("preprocess", timings.preprocess);
  log_summary("inference", timings.inference);
  log_summary("postprocess", timings.postprocess);
  log_summary("detect total", timings.frame);
  if (total.mean > 0.0) {
    spdlog::info("  throughput   {:.1f} fps (detection only, decoding excluded)",
                 1000.0 / total.mean);
  }
  for (const auto& [twin_class, count] : per_class) {
    spdlog::info("  {:<18} {:6d} detections", twin_class, count);
  }
  if (!args.overlay.empty()) {
    spdlog::info("wrote {}", args.overlay);
  }
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
  app.add_option("--overlay", args.overlay, "Write a debug overlay video (MP4)");
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
