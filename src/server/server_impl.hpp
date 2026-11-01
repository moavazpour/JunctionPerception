#pragma once

#include <crow.h>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <future>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include "jp/server/twin_server.hpp"

namespace jp::server {

using Clock = std::chrono::steady_clock;

/// WebSocket clients of one channel.
class Channel {
 public:
  void add(crow::websocket::connection* connection);
  void remove(crow::websocket::connection* connection);
  [[nodiscard]] std::size_t size();
  void send_text(const std::string& message);
  void send_binary(const std::string& message);

 private:
  std::mutex mutex_;
  std::unordered_set<crow::websocket::connection*> connections_;
};

/// Serves a file below `root`; 404 for missing files and paths leaving the root.
void serve_file(const std::filesystem::path& root, const std::string& path,
                crow::response& response);

/// JSON response with the given status code.
[[nodiscard]] crow::response json_response(const nlohmann::json& body, int code = 200);

struct TwinServer::Impl {
  Impl(ServerOptions server_options, ServedScene served_scene, core::ClassCatalog class_catalog,
       std::unique_ptr<sources::WorldSource> world_source, vru::EventDetector detector);

  ServerOptions options;
  core::ClassCatalog catalog;
  std::unique_ptr<sources::WorldSource> source;
  vru::EventDetector events;
  std::filesystem::path map_dir;

  crow::SimpleApp app;
  Channel world_clients;
  Channel video_clients;

  // Shared with the HTTP handlers; guarded by scene_mutex.
  std::mutex scene_mutex;
  ServedScene scene;
  std::string latest_raw_jpeg;

  // Requests from the HTTP handlers to the playback loop; guarded by control_mutex.
  std::mutex control_mutex;
  bool playing = true;
  std::optional<std::int64_t> pending_seek;
  std::optional<std::vector<vru::Zone>> pending_zones;

  std::mutex status_mutex;
  nlohmann::json status = nlohmann::json::object();

  // Messages of one frame, built before the frame is due.
  struct Prepared {
    std::string world;
    std::string video;  // empty if no image was requested
    std::int64_t frame = 0;
    std::int64_t timestamp_ns = 0;
    std::size_t objects = 0;
    std::size_t active_events = 0;
    double processing_ms = 0.0;
  };

  std::optional<std::pair<Clock::time_point, std::int64_t>> pace_origin;
  Clock::time_point last_video;

  void register_playback_routes();
  void register_scene_routes();
  void play_loop(const std::future<void>& server);
  // Applies pending seeks and zone changes; returns whether playback is running.
  bool apply_controls();
  [[nodiscard]] Prepared prepare(sources::SourceFrame frame, Clock::time_point started);
  void send(const Prepared& prepared);
  void wait_until_due(std::int64_t timestamp_ns);
};

}  // namespace jp::server
