#include "jp/server/twin_server.hpp"

#include <crow.h>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <bit>
#include <chrono>
#include <cstring>
#include <fstream>
#include <map>
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <set>
#include <sstream>
#include <thread>
#include <unordered_set>
#include <utility>

#include "jp/twin/world_state_json.hpp"

namespace jp::server {

namespace {

using Clock = std::chrono::steady_clock;
using nlohmann::json;

constexpr auto kIdleWait = std::chrono::milliseconds(20);
constexpr auto kMaxLag = std::chrono::seconds(1);
constexpr double kRateSmoothing = 0.1;

// WebSocket clients of one channel.
class Channel {
 public:
  void add(crow::websocket::connection* connection) {
    const std::scoped_lock lock(mutex_);
    connections_.insert(connection);
  }
  void remove(crow::websocket::connection* connection) {
    const std::scoped_lock lock(mutex_);
    connections_.erase(connection);
  }
  [[nodiscard]] std::size_t size() {
    const std::scoped_lock lock(mutex_);
    return connections_.size();
  }
  void send_text(const std::string& message) {
    const std::scoped_lock lock(mutex_);
    for (auto* connection : connections_) {
      connection->send_text(message);
    }
  }
  void send_binary(const std::string& message) {
    const std::scoped_lock lock(mutex_);
    for (auto* connection : connections_) {
      connection->send_binary(message);
    }
  }

 private:
  std::mutex mutex_;
  std::unordered_set<crow::websocket::connection*> connections_;
};

std::string video_message(std::int64_t frame, const cv::Mat& image, int width, int quality) {
  cv::Mat scaled;
  const double scale = static_cast<double>(width) / image.cols;
  cv::resize(image, scaled, cv::Size(), scale, scale, cv::INTER_AREA);
  std::vector<unsigned char> jpeg;
  cv::imencode(".jpg", scaled, jpeg, {cv::IMWRITE_JPEG_QUALITY, quality});
  std::string message(sizeof(std::int64_t) + jpeg.size(), '\0');
  auto index = static_cast<std::uint64_t>(frame);
  if constexpr (std::endian::native == std::endian::big) {
    index = std::byteswap(index);
  }
  std::memcpy(message.data(), &index, sizeof(index));
  std::ranges::copy(jpeg, message.begin() + sizeof(index));
  return message;
}

std::string mime_type(const std::filesystem::path& path) {
  static const std::map<std::string, std::string, std::less<>> types = {
      {".html", "text/html"},        {".js", "text/javascript"}, {".css", "text/css"},
      {".json", "application/json"}, {".svg", "image/svg+xml"},  {".png", "image/png"},
      {".glb", "model/gltf-binary"}, {".ico", "image/x-icon"}};
  const auto it = types.find(path.extension().string());
  return it == types.end() ? "application/octet-stream" : it->second;
}

}  // namespace

struct TwinServer::Impl {
  Impl(ServerOptions server_options, std::string id, std::string description,
       std::unique_ptr<sources::WorldSource> world_source, vru::EventDetector detector)
      : options(std::move(server_options)),
        scene_id(std::move(id)),
        scene_json(std::move(description)),
        source(std::move(world_source)),
        events(std::move(detector)) {}

  ServerOptions options;
  std::string scene_id;
  std::string scene_json;
  std::unique_ptr<sources::WorldSource> source;
  vru::EventDetector events;

  crow::SimpleApp app;
  Channel world_clients;
  Channel video_clients;

  std::mutex control_mutex;
  bool playing = true;
  std::optional<std::int64_t> pending_seek;

  std::mutex status_mutex;
  json status = json::object();

  void register_routes();
  void serve_static(const crow::request& request, crow::response& response,
                    const std::string& path) const;
  // Messages of one frame, built before the frame is due.
  struct Prepared {
    std::string world;
    std::string video;  // empty if no image was requested
    std::int64_t frame = 0;
    std::int64_t timestamp_ns = 0;
    std::size_t objects = 0;
    double processing_ms = 0.0;
  };

  void play_loop(const std::future<void>& server);
  // Applies a pending seek; returns whether playback is running.
  bool apply_controls();
  [[nodiscard]] Prepared prepare(sources::SourceFrame frame, Clock::time_point started);
  void send(const Prepared& prepared);
  void wait_until_due(std::int64_t timestamp_ns);

  std::optional<std::pair<Clock::time_point, std::int64_t>> pace_origin;
  Clock::time_point last_video;
};

void TwinServer::Impl::register_routes() {
  CROW_ROUTE(app, "/api/scene")
  ([this] {
    crow::response response(scene_json);
    response.set_header("Content-Type", "application/json");
    return response;
  });
  CROW_ROUTE(app, "/api/status")
  ([this] {
    const std::scoped_lock lock(status_mutex);
    crow::response response(status.dump());
    response.set_header("Content-Type", "application/json");
    return response;
  });
  CROW_ROUTE(app, "/api/playback").methods(crow::HTTPMethod::POST)([this](const crow::request& r) {
    const auto body = json::parse(r.body, nullptr, false);
    if (body.is_discarded() || !body.contains("action")) {
      return crow::response(400, R"(expected {"action": "play" | "pause" | "seek"})");
    }
    const auto action = body["action"].get<std::string>();
    const std::scoped_lock lock(control_mutex);
    if (action == "play" || action == "pause") {
      playing = action == "play";
    } else if (action == "seek" && body.contains("frame")) {
      pending_seek = body["frame"].get<std::int64_t>();
    } else {
      return crow::response(400, "unknown playback action");
    }
    return crow::response(204);
  });
  CROW_WEBSOCKET_ROUTE(app, "/ws/world")
      .onopen([this](crow::websocket::connection& c) { world_clients.add(&c); })
      .onclose([this](crow::websocket::connection& c, const std::string& /*reason*/,
                      std::uint16_t /*code*/) { world_clients.remove(&c); });
  CROW_WEBSOCKET_ROUTE(app, "/ws/video")
      .onopen([this](crow::websocket::connection& c) { video_clients.add(&c); })
      .onclose([this](crow::websocket::connection& c, const std::string& /*reason*/,
                      std::uint16_t /*code*/) { video_clients.remove(&c); });
  CROW_ROUTE(app, "/")
  ([this](const crow::request& r, crow::response& response) {
    serve_static(r, response, "index.html");
  });
  CROW_ROUTE(app, "/<path>")
  ([this](const crow::request& r, crow::response& response, const std::string& path) {
    serve_static(r, response, path);
  });
}

void TwinServer::Impl::serve_static(const crow::request& /*request*/, crow::response& response,
                                    const std::string& path) const {
  std::error_code error;
  const auto root = std::filesystem::weakly_canonical(options.web_root, error);
  const auto file = std::filesystem::weakly_canonical(options.web_root / path, error);
  const bool inside = !error && file.string().starts_with(root.string());
  if (options.web_root.empty() || !inside || !std::filesystem::is_regular_file(file)) {
    response.code = 404;
    response.end();
    return;
  }
  std::ifstream in(file, std::ios::binary);
  std::ostringstream content;
  content << in.rdbuf();
  response.set_header("Content-Type", mime_type(file));
  response.write(content.str());
  response.end();
}

TwinServer::Impl::Prepared TwinServer::Impl::prepare(sources::SourceFrame frame,
                                                     Clock::time_point started) {
  core::WorldState state;
  state.scene_id = scene_id;
  state.frame = frame.index;
  state.timestamp_ns = frame.timestamp_ns;
  state.source = source->kind();
  state.objects = std::move(frame.objects);
  state.events = events.update(state.objects, frame.timestamp_ns);
  Prepared prepared;
  prepared.world = twin::to_json(state);
  if (!frame.image.empty()) {
    prepared.video =
        video_message(frame.index, frame.image, options.video_width_px, options.jpeg_quality);
    last_video = started;
  }
  prepared.frame = state.frame;
  prepared.timestamp_ns = state.timestamp_ns;
  prepared.objects = state.objects.size();
  prepared.processing_ms =
      std::chrono::duration<double, std::milli>(Clock::now() - started).count();
  return prepared;
}

void TwinServer::Impl::send(const Prepared& prepared) {
  world_clients.send_text(prepared.world);
  if (!prepared.video.empty()) {
    video_clients.send_binary(prepared.video);
  }
  const std::scoped_lock lock(status_mutex);
  status["frame"] = prepared.frame;
  status["timestamp_ns"] = prepared.timestamp_ns;
  status["objects"] = prepared.objects;
  status["latency_ms"] = prepared.processing_ms;
}

// Real-time pacing on the recording's timestamps.
void TwinServer::Impl::wait_until_due(std::int64_t timestamp_ns) {
  if (!pace_origin) {
    pace_origin.emplace(Clock::now(), timestamp_ns);
  }
  const auto due =
      pace_origin->first +
      std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(
          static_cast<double>(timestamp_ns - pace_origin->second) * 1e-9 / options.speed));
  if (Clock::now() - due > kMaxLag) {
    pace_origin.emplace(Clock::now(), timestamp_ns);
  } else {
    std::this_thread::sleep_until(due);
  }
}

bool TwinServer::Impl::apply_controls() {
  std::optional<std::int64_t> seek;
  bool is_playing = false;
  {
    const std::scoped_lock lock(control_mutex);
    seek = std::exchange(pending_seek, std::nullopt);
    is_playing = playing;
  }
  if (seek) {
    source->seek(*seek);
    events.reset();
    pace_origin.reset();
    // While paused, show the frame at the new position.
    if (!is_playing) {
      if (auto frame = source->next(video_clients.size() > 0)) {
        send(prepare(std::move(*frame), Clock::now()));
      }
    }
  }
  if (!is_playing) {
    pace_origin.reset();
  }
  return is_playing;
}

void TwinServer::Impl::play_loop(const std::future<void>& server) {
  Clock::time_point last_frame;
  double fps = 0.0;
  while (server.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
    const bool is_playing = apply_controls();
    {
      const std::scoped_lock lock(status_mutex);
      status["playing"] = is_playing;
      status["fps"] = fps;
      status["clients"] = world_clients.size();
    }
    if (!is_playing) {
      std::this_thread::sleep_for(kIdleWait);
      continue;
    }

    const auto started = Clock::now();
    const bool want_video =
        video_clients.size() > 0 &&
        started - last_video >= std::chrono::duration<double>(1.0 / options.video_max_fps);
    auto frame = source->next(want_video);
    if (!frame) {
      const std::scoped_lock lock(control_mutex);
      if (options.loop) {
        pending_seek = 0;
      } else {
        playing = false;
      }
      continue;
    }
    const std::int64_t timestamp_ns = frame->timestamp_ns;
    const Prepared prepared = prepare(std::move(*frame), started);
    wait_until_due(timestamp_ns);
    send(prepared);
    const auto now = Clock::now();
    if (last_frame.time_since_epoch().count() != 0) {
      const double instant = 1.0 / std::chrono::duration<double>(now - last_frame).count();
      fps = fps == 0.0 ? instant : ((1.0 - kRateSmoothing) * fps) + (kRateSmoothing * instant);
    }
    last_frame = now;
  }
}

TwinServer::TwinServer(ServerOptions options, std::string scene_id, std::string scene_json,
                       std::unique_ptr<sources::WorldSource> source, vru::EventDetector events)
    : impl_(std::make_unique<Impl>(std::move(options), std::move(scene_id), std::move(scene_json),
                                   std::move(source), std::move(events))) {
  const std::scoped_lock lock(impl_->status_mutex);
  impl_->status = {{"scene_id", impl_->scene_id},
                   {"source", impl_->source->kind()},
                   {"frame_count", impl_->source->frame_count().value_or(-1)},
                   {"seekable", true}};
}

TwinServer::~TwinServer() = default;

void TwinServer::run() {
  impl_->app.loglevel(crow::LogLevel::Warning);
  impl_->register_routes();
  auto server = impl_->app.bindaddr(impl_->options.bind_address)
                    .port(static_cast<std::uint16_t>(impl_->options.port))
                    .multithreaded()
                    .run_async();
  impl_->app.wait_for_server_start();
  spdlog::info("serving scene {} ({}) on {}:{}", impl_->scene_id, impl_->source->kind(),
               impl_->options.bind_address, impl_->options.port);
  impl_->play_loop(server);
  spdlog::info("server stopped");
}

ServerConfig load_server_config(const std::filesystem::path& path) {
  ServerConfig config;
  try {
    const YAML::Node root = YAML::LoadFile(path.string());
    const YAML::Node server = root["server"];
    config.server.bind_address = server["bind_address"].as<std::string>();
    config.server.port = server["port"].as<int>();
    config.server.web_root = server["web_root"].as<std::string>();
    const YAML::Node video = root["video"];
    config.server.video_max_fps = video["max_fps"].as<double>();
    config.server.video_width_px = video["width_px"].as<int>();
    config.server.jpeg_quality = video["jpeg_quality"].as<int>();
    const YAML::Node playback = root["playback"];
    config.server.loop = playback["loop"].as<bool>();
    config.server.speed = playback["speed"].as<double>();
    const YAML::Node startup = root["startup"];
    config.startup.scene = startup["scene"].as<std::string>();
    config.startup.mode = startup["mode"].as<std::string>();
    config.startup.input = startup["input"].as<std::string>();
  } catch (const YAML::Exception& error) {
    throw std::runtime_error("invalid server config " + path.string() + ": " + error.what());
  }
  if (config.server.video_max_fps <= 0.0 || config.server.speed <= 0.0) {
    throw std::runtime_error("video.max_fps and playback.speed must be positive");
  }
  return config;
}

std::string scene_description(const std::string& scene_id,
                              const calibration::GroundCalibration& ground,
                              const std::string& zones_geojson, const core::ClassCatalog& catalog) {
  json classes = json::array();
  for (const auto& c : catalog.classes()) {
    classes.push_back({{"name", c.name},
                       {"length_m", c.length_m},
                       {"width_m", c.width_m},
                       {"height_m", c.height_m},
                       {"vru", c.vru}});
  }
  const json description = {
      {"scene_id", scene_id},
      {"camera", ground.camera},
      {"image_size_px", {ground.width_px, ground.height_px}},
      {"camera_ground_position_m",
       {ground.camera_ground_position_m.x(), ground.camera_ground_position_m.y()}},
      {"camera_height_m", ground.camera_height_m},
      {"zones", zones_geojson.empty() ? json(nullptr) : json::parse(zones_geojson)},
      {"classes", classes}};
  return description.dump();
}

}  // namespace jp::server
