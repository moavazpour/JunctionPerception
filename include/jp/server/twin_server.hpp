#pragma once

#include <filesystem>
#include <memory>
#include <optional>
#include <string>

#include "jp/calibration/ground_calibration.hpp"
#include "jp/core/class_catalog.hpp"
#include "jp/core/scene_config.hpp"
#include "jp/sources/world_source.hpp"
#include "jp/vru/event_detector.hpp"

namespace jp::server {

struct ServerOptions {
  std::string bind_address = "0.0.0.0";
  int port = 8080;
  std::filesystem::path web_root;   ///< Built frontend (web/dist); empty: no static files.
  std::filesystem::path map_cache;  ///< Map layers per scene (tools/prepare_map_data.py).
  double video_max_fps = 5.0;       ///< Rate limit of the camera image channel.
  int video_width_px = 960;         ///< Camera images are scaled to this width.
  int jpeg_quality = 70;
  bool loop = true;    ///< Restart file sources at the end.
  double speed = 1.0;  ///< Playback speed relative to the recording's timestamps.
  /// Site clock file (written by the site manager) that recordings play by; empty: the server
  /// paces its recording by itself. See SiteClock.
  std::filesystem::path clock;
};

/// Startup selection of the scene and the world source.
struct StartupSettings {
  std::filesystem::path scene;
  std::string mode;   ///< "live", "oracle" or "replay".
  std::string input;  ///< Video, TUMTraf sequence folder or JSONL (relative to the data root).
};

struct ServerConfig {
  ServerOptions server;
  StartupSettings startup;
};

/// Parses configs/server.yaml; throws std::runtime_error on missing keys or invalid values.
[[nodiscard]] ServerConfig load_server_config(const std::filesystem::path& path);

/// The scene a server shows: its configuration and, once calibrated, its ground calibration.
struct ServedScene {
  core::SceneConfig config;
  std::optional<calibration::GroundCalibration> ground;
  std::vector<vru::Zone> zones;  ///< World frame.
};

/// JSON description of a scene for the frontend: image size, calibration summary, geo anchor,
/// map manifest location, zones (GeoJSON in the world frame) and the class catalog.
[[nodiscard]] std::string scene_description(const ServedScene& scene,
                                            const core::ClassCatalog& catalog, bool has_map);

/// Plays a world source in real time and publishes it:
///   GET  /api/scene                scene description
///   GET  /api/status               playback, frame rate and latency
///   POST /api/playback             {"action": "play" | "pause" | "seek", "frame": n}
///   WS   /ws/world                 one world state (JSON) per frame
///   WS   /ws/video                 camera images: 8-byte little-endian frame index + JPEG
///   GET  /api/frame                latest camera image without overlays (JPEG)
///   GET  /api/calibration          stored control points (world frame)
///   POST /api/calibration/fit      fit a homography to control points, errors in metres
///   POST /api/calibration/save     store control points and the ground calibration
///   POST /api/zones                store zones (GeoJSON in the world frame) and apply them
///   POST /api/restart              exit, so that the service restarts with the new calibration
///   GET  /map/...                  map layers of the scene
///   GET  /...                      static files of the built frontend
class TwinServer {
 public:
  TwinServer(ServerOptions options, ServedScene scene, core::ClassCatalog catalog,
             std::unique_ptr<sources::WorldSource> source, vru::EventDetector events);
  ~TwinServer();
  TwinServer(const TwinServer&) = delete;
  TwinServer& operator=(const TwinServer&) = delete;
  TwinServer(TwinServer&&) = delete;
  TwinServer& operator=(TwinServer&&) = delete;

  /// Serves and plays until the process receives SIGINT or SIGTERM or a restart is requested.
  void run();

  struct Impl;

 private:
  std::unique_ptr<Impl> impl_;
};

}  // namespace jp::server
