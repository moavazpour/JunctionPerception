#include <spdlog/spdlog.h>

#include <CLI/CLI.hpp>
#include <cstdlib>
#include <filesystem>
#include <optional>
#include <string>

#include "jp/calibration/ground_calibration.hpp"
#include "jp/core/class_catalog.hpp"
#include "jp/core/pipeline_config.hpp"
#include "jp/core/scene_config.hpp"
#include "jp/core/version.hpp"
#include "jp/server/twin_server.hpp"
#include "jp/sources/world_source.hpp"
#include "jp/vru/event_detector.hpp"
#include "jp/vru/zones.hpp"

namespace {

struct ServerArgs {
  std::string config = "configs/server.yaml";
  std::string pipeline = "configs/pipeline.yaml";
  std::string classes = "configs/classes.yaml";
  std::string data_root;
  std::string scene;
  std::string mode;
  std::string input;
};

std::unique_ptr<jp::sources::WorldSource> make_source(
    const std::string& mode, const std::filesystem::path& input,
    const jp::core::PipelineConfig& pipeline, const jp::core::ClassCatalog& catalog,
    const std::optional<jp::calibration::GroundCalibration>& ground) {
  if (mode == "live") {
    return jp::sources::make_live_source(input, pipeline, catalog, ground);
  }
  if (mode == "oracle") {
    if (!ground) {
      throw std::runtime_error("oracle mode needs a calibrated benchmark scene");
    }
    return jp::sources::make_oracle_source(input, catalog, *ground);
  }
  return jp::sources::make_replay_source(input);
}

void run(const ServerArgs& args) {
  auto config = jp::server::load_server_config(args.config);
  if (!args.scene.empty()) {
    config.startup.scene = args.scene;
  }
  if (!args.mode.empty()) {
    config.startup.mode = args.mode;
  }
  if (!args.input.empty()) {
    config.startup.input = args.input;
  }
  const auto& startup = config.startup;
  if (startup.mode != "live" && startup.mode != "oracle" && startup.mode != "replay") {
    throw std::runtime_error("mode must be live, oracle or replay, not " + startup.mode);
  }
  // Recordings (live video, oracle sequence) live under the data root, replays in the repository.
  const std::filesystem::path input = startup.mode == "replay"
                                          ? std::filesystem::path(startup.input)
                                          : std::filesystem::path(args.data_root) / startup.input;

  const auto pipeline =
      jp::core::load_pipeline_config(args.pipeline, std::filesystem::current_path());
  const auto catalog = jp::core::ClassCatalog::load(args.classes);
  jp::server::ServedScene scene;
  scene.config = jp::core::load_scene_config(startup.scene);
  if (std::filesystem::exists(scene.config.calibration_path())) {
    scene.ground = jp::calibration::load_ground_calibration(scene.config.calibration_path());
  }
  scene.zones = jp::vru::load_scene_zones(scene.config);
  spdlog::info("scene {}: {}, {} zones; source {} from {}", scene.config.scene_id,
               scene.ground ? "calibrated" : "not calibrated", scene.zones.size(), startup.mode,
               input.string());

  auto source = make_source(startup.mode, input, pipeline, catalog, scene.ground);
  jp::vru::EventDetector events(scene.zones, jp::vru::event_options(pipeline.events, catalog));
  if (scene.ground) {
    if (auto region = jp::sources::validated_region(scene.config, *scene.ground, pipeline.events)) {
      spdlog::info("events limited to the validated region ({} control points outline)",
                   region->outline().size());
      events.set_validated_region(
          [region = std::move(*region)](const Eigen::Vector2d& p) { return region.contains(p); });
    }
  }
  jp::server::TwinServer server(config.server, std::move(scene), catalog, std::move(source),
                                std::move(events));
  server.run();
}

}  // namespace

int main(int argc, char** argv) {
  CLI::App app{"Serves the digital twin web UI, the REST API and the world-state stream",
               "jp_server"};
  app.set_version_flag("--version", std::string{jp::core::version()});

  ServerArgs args;
  if (const char* root = std::getenv("JP_DATA_ROOT")) {
    args.data_root = root;
  }
  app.add_option("--config", args.config, "Server config")->capture_default_str();
  app.add_option("--pipeline", args.pipeline, "Pipeline config")->capture_default_str();
  app.add_option("--classes", args.classes, "Class catalog")->capture_default_str();
  app.add_option("--data-root", args.data_root, "Dataset root (default: $JP_DATA_ROOT)");
  app.add_option("--scene", args.scene, "Scene folder (overrides the config)");
  app.add_option("--mode", args.mode, "live, oracle or replay (overrides the config)")
      ->check(CLI::IsMember({"live", "oracle", "replay"}));
  app.add_option("--input", args.input,
                 "Video, TUMTraf sequence folder (under the data root) or JSONL file");

  CLI11_PARSE(app, argc, argv);

  try {
    run(args);
  } catch (const std::exception& error) {
    spdlog::error("{}", error.what());
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
