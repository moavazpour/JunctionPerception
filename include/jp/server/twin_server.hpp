#pragma once

#include <filesystem>
#include <memory>
#include <string>

#include "jp/calibration/ground_calibration.hpp"
#include "jp/core/class_catalog.hpp"
#include "jp/sources/world_source.hpp"
#include "jp/vru/event_detector.hpp"

namespace jp::server {

struct ServerOptions {
  std::string bind_address = "0.0.0.0";
  int port = 8080;
  std::filesystem::path web_root;  ///< Built frontend (web/dist); empty: no static files.
  double video_max_fps = 5.0;      ///< Rate limit of the camera image channel.
  int video_width_px = 960;        ///< Camera images are scaled to this width.
  int jpeg_quality = 70;
  bool loop = true;    ///< Restart file sources at the end.
  double speed = 1.0;  ///< Playback speed relative to the recording's timestamps.
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

/// JSON description of a scene for the frontend: ground calibration summary, zones (GeoJSON as
/// stored) and the class catalog.
[[nodiscard]] std::string scene_description(const std::string& scene_id,
                                            const calibration::GroundCalibration& ground,
                                            const std::string& zones_geojson,
                                            const core::ClassCatalog& catalog);

/// Plays a world source in real time and publishes it:
///   GET  /api/scene      scene description
///   GET  /api/status     playback, frame rate and latency
///   POST /api/playback   {"action": "play" | "pause" | "seek", "frame": n}
///   WS   /ws/world       one world state (JSON) per frame
///   WS   /ws/video       camera images: 8-byte little-endian frame index + JPEG, rate-limited
///   GET  /...            static files of the built frontend
class TwinServer {
 public:
  TwinServer(ServerOptions options, std::string scene_id, std::string scene_json,
             std::unique_ptr<sources::WorldSource> source, vru::EventDetector events);
  ~TwinServer();
  TwinServer(const TwinServer&) = delete;
  TwinServer& operator=(const TwinServer&) = delete;
  TwinServer(TwinServer&&) = delete;
  TwinServer& operator=(TwinServer&&) = delete;

  /// Serves and plays until the process receives SIGINT or SIGTERM.
  void run();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace jp::server
