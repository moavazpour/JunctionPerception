#include <spdlog/spdlog.h>

#include <CLI/CLI.hpp>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <numbers>
#include <opencv2/imgcodecs.hpp>
#include <string>

#include "jp/benchmark/label_merge.hpp"
#include "jp/benchmark/tumtraf_sequence.hpp"
#include "jp/calibration/benchmark_calibration.hpp"
#include "jp/core/version.hpp"

namespace {

struct BenchmarkArgs {
  std::string sequence;
  std::string data_root;
  std::string out_dir = "outputs/calibration";
  std::size_t overlay_frame = 0;
};

void run_benchmark(const BenchmarkArgs& args) {
  namespace bm = jp::benchmark;
  namespace cal = jp::calibration;

  if (args.data_root.empty()) {
    throw std::runtime_error("dataset root not set (use --data-root or JP_DATA_ROOT)");
  }
  const std::filesystem::path sequence_dir = std::filesystem::path(args.data_root) / args.sequence;
  const auto sequence = bm::TumtrafSequence::open(sequence_dir);
  spdlog::info("{}: {} frames", sequence.name(), sequence.size());

  const auto truth = bm::merged_ground_truth(sequence);
  const auto& reg = truth.registration;
  spdlog::info(
      "north->south labels: yaw {:.3f} deg, t ({:.3f}, {:.3f}, {:.3f}) m; {} matched boxes, "
      "BEV error median {:.3f} m, P90 {:.3f} m, height error median {:.3f} m",
      reg.south_from_north.yaw_rad * 180.0 / std::numbers::pi, reg.south_from_north.translation.x(),
      reg.south_from_north.translation.y(), reg.south_from_north.translation.z(), reg.matched_pairs,
      reg.median_bev_error_m, reg.p90_bev_error_m, reg.median_height_error_m);
  const auto& stats = truth.merged.statistics;
  spdlog::info("merged: {} south + {} north boxes -> {} boxes ({} in both), {} tracks",
               stats.south_boxes, stats.north_boxes, stats.merged_boxes, stats.matched_boxes,
               stats.tracks);

  const auto log_plane = [](std::string_view label, const cal::PlaneFit& fit) {
    spdlog::info(
        "{} ground plane: normal ({:.4f}, {:.4f}, {:.4f}), offset {:.3f} m, {} inliers, "
        "RMS {:.3f} m",
        label, fit.plane.normal.x(), fit.plane.normal.y(), fit.plane.normal.z(), fit.plane.offset,
        fit.inliers, fit.inlier_rms_m);
  };
  log_plane("scene", cal::fit_ground_plane(truth.merged.frames));

  const std::size_t overlay_index = std::min(args.overlay_frame, sequence.size() - 1);
  for (const auto& camera_name : sequence.camera_names()) {
    const auto& camera = sequence.camera(camera_name);
    const auto ground = cal::fit_ground_plane_in_view(truth.merged.frames, camera);
    log_plane(camera_name, ground);
    const auto calibration =
        cal::calibrate_benchmark_camera(camera, ground.plane, truth.merged.frames);
    const auto& c = calibration.check;
    spdlog::info(
        "{}: camera at ({:.2f}, {:.2f}) m, height {:.2f} m; check on {} boxes: mean "
        "{:.3f} m, median {:.3f} m, P90 {:.3f} m, max {:.3f} m",
        camera_name, calibration.camera_ground_position_m.x(),
        calibration.camera_ground_position_m.y(), calibration.camera_height_m, c.objects, c.mean_m,
        c.median_m, c.p90_m, c.max_m);
    for (const auto& bin : c.by_distance) {
      spdlog::info("  {:>4.0f}-{:<4.0f} m: {:5d} boxes, median {:.3f} m, P90 {:.3f} m", bin.from_m,
                   bin.to_m, bin.objects, bin.median_m, bin.p90_m);
    }

    const std::filesystem::path out_dir = std::filesystem::path(args.out_dir) / camera_name;
    cal::write_ground_homography(out_dir / "ground_homography.yaml", calibration, ground,
                                 sequence.name());

    const auto& image_path = sequence.frame(overlay_index).images.at(camera_name).path;
    const cv::Mat image = cv::imread(image_path.string(), cv::IMREAD_COLOR);
    if (image.empty()) {
      throw std::runtime_error("cannot read " + image_path.string());
    }
    const cv::Mat overlay = cal::draw_ground_truth_overlay(image, calibration, camera, ground.plane,
                                                           truth.merged.frames.at(overlay_index));
    const auto overlay_path = out_dir / std::format("overlay_{:04d}.jpg", overlay_index);
    cv::imwrite(overlay_path.string(), overlay);
    spdlog::info("wrote {} and {}", (out_dir / "ground_homography.yaml").string(),
                 overlay_path.string());
  }
}

}  // namespace

int main(int argc, char** argv) {
  CLI::App app{"Fits and validates the image-to-ground homography of a scene", "jp_calibrate"};
  app.set_version_flag("--version", std::string{jp::core::version()});
  app.require_subcommand(1);

  BenchmarkArgs bench_args;
  if (const char* root = std::getenv("JP_DATA_ROOT")) {
    bench_args.data_root = root;
  }
  auto* bench = app.add_subcommand(
      "benchmark", "Ground homography of the TUMTraf cameras from the dataset calibration");
  bench->add_option("--sequence", bench_args.sequence, "Sequence folder, e.g. a9_dataset_r02_s03")
      ->required();
  bench->add_option("--data-root", bench_args.data_root, "Dataset root (default: $JP_DATA_ROOT)");
  bench->add_option("--out", bench_args.out_dir, "Output folder")->capture_default_str();
  bench->add_option("--overlay-frame", bench_args.overlay_frame, "Frame drawn in the overlay")
      ->capture_default_str();

  CLI11_PARSE(app, argc, argv);

  try {
    if (bench->parsed()) {
      run_benchmark(bench_args);
    }
  } catch (const std::exception& error) {
    spdlog::error("{}", error.what());
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
