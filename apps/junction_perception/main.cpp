// JunctionPerception runtime: one process per camera.
//
//   junction_perception --config /config/site.yaml [--max-frames N] [--no-realtime] [--recalibrate]
#include <spdlog/spdlog.h>

#include <csignal>
#include <cstring>
#include <iostream>

#include "jp/pipeline.hpp"

namespace {
jp::Pipeline* g_pipeline = nullptr;
void onSignal(int) {
  if (g_pipeline) g_pipeline->stop();
}
}  // namespace

int main(int argc, char** argv) {
  std::string config;
  std::uint64_t max_frames = 0;
  bool no_realtime = false, recalibrate = false;
  for (int i = 1; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--config") && i + 1 < argc) {
      config = argv[++i];
    } else if (!std::strcmp(argv[i], "--max-frames") && i + 1 < argc) {
      max_frames = std::stoull(argv[++i]);
    } else if (!std::strcmp(argv[i], "--no-realtime")) {
      no_realtime = true;
    } else if (!std::strcmp(argv[i], "--recalibrate")) {
      recalibrate = true;
    } else {
      std::cerr << "usage: junction_perception --config site.yaml [--max-frames N] [--no-realtime] [--recalibrate]\n";
      return 2;
    }
  }
  if (config.empty()) {
    std::cerr << "--config is required\n";
    return 2;
  }
  spdlog::set_pattern("%Y-%m-%d %H:%M:%S.%e %^%l%$ %v");
  try {
    auto cfg = jp::PipelineConfig::fromYaml(YAML::LoadFile(config));
    if (no_realtime) cfg.source.realtime = false;
    if (recalibrate) cfg.force_recalibration = true;
    jp::Pipeline p(cfg);
    g_pipeline = &p;
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    p.run(max_frames);
  } catch (const std::exception& e) {
    spdlog::critical("{}", e.what());
    return 1;
  }
  return 0;
}
