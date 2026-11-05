#include "jp/server/twin_server.hpp"

#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <bit>
#include <cstring>
#include <fstream>
#include <map>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <thread>
#include <utility>

#include "jp/server/site_clock.hpp"
#include "jp/twin/world_state_json.hpp"
#include "server_impl.hpp"

namespace jp::server {

namespace {

using json = nlohmann::json;

constexpr auto kIdleWait = std::chrono::milliseconds(20);
constexpr auto kMaxLag = std::chrono::seconds(1);
// Playing by a site clock: farther behind it than kSkipLagNs, frames are passed unprocessed until
// within kCatchUpNs; ahead of it by more than kSyncToleranceNs (a seek back, the loop wrapping
// round), the recording continues at the clock's position.
constexpr std::int64_t kSkipLagNs = 300'000'000;
constexpr std::int64_t kCatchUpNs = 100'000'000;
constexpr std::int64_t kSyncToleranceNs = 500'000'000;
constexpr std::int64_t kMaxWaitNs = 2'000'000'000;
constexpr double kDefaultFps = 10.0;
constexpr double kRateSmoothing = 0.1;
constexpr int kRawJpegQuality = 92;

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
      {".html", "text/html"},        {".js", "text/javascript"},    {".css", "text/css"},
      {".json", "application/json"}, {".svg", "image/svg+xml"},     {".png", "image/png"},
      {".jpg", "image/jpeg"},        {".glb", "model/gltf-binary"}, {".ico", "image/x-icon"}};
  const auto it = types.find(path.extension().string());
  return it == types.end() ? "application/octet-stream" : it->second;
}

}  // namespace

void Channel::add(crow::websocket::connection* connection) {
  const std::scoped_lock lock(mutex_);
  connections_.insert(connection);
}

void Channel::remove(crow::websocket::connection* connection) {
  const std::scoped_lock lock(mutex_);
  connections_.erase(connection);
}

std::size_t Channel::size() {
  const std::scoped_lock lock(mutex_);
  return connections_.size();
}

void Channel::send_text(const std::string& message) {
  const std::scoped_lock lock(mutex_);
  for (auto* connection : connections_) {
    connection->send_text(message);
  }
}

void Channel::send_binary(const std::string& message) {
  const std::scoped_lock lock(mutex_);
  for (auto* connection : connections_) {
    connection->send_binary(message);
  }
}

void serve_file(const std::filesystem::path& root, const std::string& path,
                crow::response& response) {
  std::error_code error;
  const auto canonical_root = std::filesystem::weakly_canonical(root, error);
  const auto file = std::filesystem::weakly_canonical(root / path, error);
  const bool inside = !error && file.string().starts_with(canonical_root.string());
  if (root.empty() || !inside || !std::filesystem::is_regular_file(file)) {
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

crow::response json_response(const json& body, int code) {
  crow::response response(code, body.dump());
  response.set_header("Content-Type", "application/json");
  return response;
}

TwinServer::Impl::Impl(ServerOptions server_options, ServedScene served_scene,
                       core::ClassCatalog class_catalog,
                       std::unique_ptr<sources::WorldSource> world_source,
                       std::optional<localization::ValidatedRegion> validated_region)
    : options(std::move(server_options)),
      catalog(std::move(class_catalog)),
      source(std::move(world_source)),
      validated(std::move(validated_region)),
      map_dir(options.map_cache / served_scene.config.scene_id),
      scene(std::move(served_scene)) {}

void TwinServer::Impl::register_playback_routes() {
  CROW_ROUTE(app, "/api/status")
  ([this] {
    const std::scoped_lock lock(status_mutex);
    return json_response(status);
  });
  CROW_ROUTE(app, "/api/playback").methods(crow::HTTPMethod::POST)([this](const crow::request& r) {
    const auto body = json::parse(r.body, nullptr, false);
    if (body.is_discarded() || !body.contains("action")) {
      return crow::response(400, R"(expected {"action": "play" | "pause" | "seek"})");
    }
    if (synced()) {
      return crow::response(409, "playback follows the site clock: use the site's playback");
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
  ([this](const crow::request& /*request*/, crow::response& response) {
    serve_file(options.web_root, "index.html", response);
  });
  CROW_ROUTE(app, "/<path>")
  ([this](const crow::request& /*request*/, crow::response& response, const std::string& path) {
    serve_file(options.web_root, path, response);
  });
}

TwinServer::Impl::Prepared TwinServer::Impl::prepare(sources::SourceFrame frame,
                                                     Clock::time_point started) {
  core::WorldState state;
  state.scene_id = scene.config.scene_id;
  state.frame = frame.index;
  state.timestamp_ns = frame.timestamp_ns;
  state.time_source = frame.time_source;
  state.camera_id = scene.config.camera_id;
  state.source = source->kind();
  state.objects = std::move(frame.objects);
  sources::mark_validated(state.objects, validated);
  state.signals = std::move(frame.signals);
  Prepared prepared;
  prepared.world = twin::to_json(state);
  if (!frame.image.empty()) {
    last_video = started;
  }
  prepared.image = std::move(frame.image);
  prepared.raw_image = std::move(frame.raw_image);
  prepared.frame = state.frame;
  prepared.timestamp_ns = state.timestamp_ns;
  prepared.objects = state.objects.size();
  prepared.processing_ms =
      std::chrono::duration<double, std::milli>(Clock::now() - started).count();
  return prepared;
}

void TwinServer::Impl::send(const Prepared& prepared) {
  world_clients.send_text(prepared.world);
  // Handed over when the frame is due, so the video never runs ahead of the world state.
  if (!prepared.image.empty() || !prepared.raw_image.empty()) {
    video_encoder.submit(prepared.frame, prepared.image, prepared.raw_image);
  }
  const std::scoped_lock lock(status_mutex);
  status["frame"] = prepared.frame;
  status["timestamp_ns"] = prepared.timestamp_ns;
  status["objects"] = prepared.objects;
  status["latency_ms"] = prepared.processing_ms;
}

void TwinServer::Impl::send_video(std::int64_t frame, const cv::Mat& image, const cv::Mat& raw) {
  if (!image.empty()) {
    video_clients.send_binary(
        video_message(frame, image, options.video_width_px, options.jpeg_quality));
  }
  if (!raw.empty()) {
    std::vector<unsigned char> jpeg;
    cv::imencode(".jpg", raw, jpeg, {cv::IMWRITE_JPEG_QUALITY, kRawJpegQuality});
    const std::scoped_lock lock(scene_mutex);
    latest_raw_jpeg.assign(jpeg.begin(), jpeg.end());
  }
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
  std::optional<sources::DetectionRegions> regions;
  bool is_playing = false;
  {
    const std::scoped_lock lock(control_mutex);
    seek = std::exchange(pending_seek, std::nullopt);
    regions = std::exchange(pending_regions, std::nullopt);
    is_playing = playing;
  }
  if (regions) {
    source->set_regions(*regions);
  }
  if (seek) {
    source->seek(*seek);
    pace_origin.reset();
    // While paused, show the frame at the new position.
    if (!is_playing) {
      if (auto frame = source->next(true)) {
        send(prepare(std::move(*frame), Clock::now()));
      }
    }
  }
  if (!is_playing) {
    pace_origin.reset();
  }
  return is_playing;
}

bool TwinServer::Impl::synced() const {
  return !options.clock.empty() && source->frame_count().has_value();
}

void TwinServer::Impl::count_frame(Clock::time_point& last_frame, double& fps) {
  const auto now = Clock::now();
  if (last_frame.time_since_epoch().count() != 0) {
    const double instant = 1.0 / std::chrono::duration<double>(now - last_frame).count();
    fps = fps == 0.0 ? instant : ((1.0 - kRateSmoothing) * fps) + (kRateSmoothing * instant);
  }
  last_frame = now;
}

bool TwinServer::Impl::sync_catch_up(std::int64_t target) {
  while (!sync.have_last || sync.last_ns < target - kCatchUpNs) {
    const auto timestamp = source->skip();
    if (!timestamp) {
      sync.at_end = true;
      return false;
    }
    sync.have_last = true;
    sync.last_ns = *timestamp;
  }
  return true;
}

std::optional<TwinServer::Impl::Prepared> TwinServer::Impl::sync_read() {
  const auto started = Clock::now();
  const bool want_video =
      video_clients.size() > 0 &&
      started - last_video >= std::chrono::duration<double>(1.0 / options.video_max_fps);
  auto frame = source->next(want_video);
  if (!frame) {
    sync.at_end = true;
    return std::nullopt;
  }
  sync.have_last = true;
  sync.last_ns = frame->timestamp_ns;
  return prepare(std::move(*frame), started);
}

void TwinServer::Impl::sync_paused(std::int64_t target) {
  // The frame at the clock's position is shown once (after a seek, a start or the pause of a
  // camera that was behind), so all cameras stand at the same moment.
  const bool shown = sync.have_last && target - sync.last_ns <= kCatchUpNs;
  if (shown || sync.at_end || !sync_catch_up(target)) {
    std::this_thread::sleep_for(kIdleWait);
    return;
  }
  if (const auto prepared = sync_read()) {
    send(*prepared);
  }
}

bool TwinServer::Impl::sync_playing(const SiteClock& clock, std::int64_t period,
                                    std::int64_t target) {
  if (sync.at_end) {
    // Until the clock wraps round; the site's loop length may not be known yet.
    if (sync.own_period_ns == 0 && sync.have_last) {
      sync.own_period_ns = sync.last_ns + static_cast<std::int64_t>(1e9 / sync.rate);
    }
    std::this_thread::sleep_for(kIdleWait);
    return false;
  }
  const bool behind = !sync.have_last || target - sync.last_ns > kSkipLagNs;
  if (behind && !sync_catch_up(target)) {
    return false;
  }
  const auto prepared = sync_read();
  if (!prepared) {
    return false;
  }
  // Shown when the clock reaches the frame.
  const std::int64_t ahead = prepared->timestamp_ns - clock.position(wall_now_ns(), period);
  if (ahead > 0 && ahead < kMaxWaitNs) {
    std::this_thread::sleep_for(std::chrono::nanoseconds(
        static_cast<std::int64_t>(static_cast<double>(ahead) / clock.speed)));
  }
  send(*prepared);
  return true;
}

void TwinServer::Impl::play_synced(const std::future<void>& server) {
  SiteClockFile file(options.clock);
  // Until the manager's clock is read: playing from the start, now.
  SiteClock clock;
  clock.epoch_ns = wall_now_ns();
  clock.speed = options.speed;
  sync = {};
  sync.rate = source->fps().value_or(kDefaultFps);
  Clock::time_point last_frame;
  double fps = 0.0;
  while (server.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
    // Zones and detection regions; seeks and pauses come with the clock.
    static_cast<void>(apply_controls());
    if (auto fresh = file.poll()) {
      clock = *fresh;
    }
    const std::int64_t period = options.loop ? std::max(clock.period_ns, sync.own_period_ns) : 0;
    const std::int64_t target = clock.position(wall_now_ns(), period);
    {
      const std::scoped_lock lock(status_mutex);
      status["playing"] = clock.playing;
      status["fps"] = fps;
      status["clients"] = world_clients.size();
      status["clock_lag_ms"] =
          sync.have_last ? nlohmann::json(static_cast<double>(target - sync.last_ns) / 1e6)
                         : nlohmann::json(nullptr);
    }
    // Ahead of the clock (a seek back, the loop wrapping round): continue at its position.
    if (sync.have_last && target + kSyncToleranceNs < sync.last_ns) {
      source->seek(std::max<std::int64_t>(
          0, static_cast<std::int64_t>(static_cast<double>(target) * 1e-9 * sync.rate) - 1));
      sync.have_last = false;
      sync.at_end = false;
    }
    if (!clock.playing) {
      sync_paused(target);
    } else if (sync_playing(clock, period, target)) {
      count_frame(last_frame, fps);
    }
  }
}

void TwinServer::Impl::play_loop(const std::future<void>& server) {
  if (synced()) {
    spdlog::info("playback follows the site clock {}", options.clock.string());
    play_synced(server);
    return;
  }
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
    count_frame(last_frame, fps);
  }
}

TwinServer::TwinServer(ServerOptions options, ServedScene scene, core::ClassCatalog catalog,
                       std::unique_ptr<sources::WorldSource> source,
                       std::optional<localization::ValidatedRegion> validated)
    : impl_(std::make_unique<Impl>(std::move(options), std::move(scene), std::move(catalog),
                                   std::move(source), std::move(validated))) {
  const std::scoped_lock lock(impl_->status_mutex);
  // Camera streams have no frame count and cannot seek.
  const auto frame_count = impl_->source->frame_count();
  impl_->status = {{"scene_id", impl_->scene.config.scene_id},
                   {"camera_id", impl_->scene.config.camera_id},
                   {"source", impl_->source->kind()},
                   {"frame_count", frame_count.value_or(-1)},
                   {"seekable", frame_count.has_value()},
                   {"calibrated", impl_->scene.ground.has_value()},
                   {"synced", impl_->synced()}};
  // Recordings: their frame rate and length, for the site clock (seeks, loop length).
  if (const auto rate = impl_->source->fps(); rate && frame_count) {
    impl_->status["video_fps"] = *rate;
    impl_->status["duration_ns"] =
        static_cast<std::int64_t>(static_cast<double>(*frame_count) / *rate * 1e9);
  }
}

TwinServer::~TwinServer() = default;

void TwinServer::run() {
  impl_->app.loglevel(crow::LogLevel::Warning);
  impl_->register_scene_routes();
  impl_->register_region_routes();
  impl_->register_playback_routes();
  auto server = impl_->app.bindaddr(impl_->options.bind_address)
                    .port(static_cast<std::uint16_t>(impl_->options.port))
                    .multithreaded()
                    .run_async();
  impl_->app.wait_for_server_start();
  spdlog::info("serving scene {} ({}) on {}:{}", impl_->scene.config.scene_id,
               impl_->source->kind(), impl_->options.bind_address, impl_->options.port);
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
    config.server.map_cache = server["map_cache"].as<std::string>();
    const YAML::Node video = root["video"];
    config.server.video_max_fps = video["max_fps"].as<double>();
    config.server.video_width_px = video["width_px"].as<int>();
    config.server.jpeg_quality = video["jpeg_quality"].as<int>();
    const YAML::Node playback = root["playback"];
    config.server.loop = playback["loop"].as<bool>();
    config.server.speed = playback["speed"].as<double>();
    if (const YAML::Node clock = playback["clock"]) {
      config.server.clock = clock.as<std::string>();
    }
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

}  // namespace jp::server
