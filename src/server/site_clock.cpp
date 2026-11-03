#include "jp/server/site_clock.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>

namespace jp::server {

std::int64_t SiteClock::position(std::int64_t now_ns, std::int64_t period) const {
  std::int64_t at = media_ns;
  if (playing) {
    at += std::llround(static_cast<double>(now_ns - epoch_ns) * speed);
  }
  if (period > 0) {
    at %= period;
    if (at < 0) {
      at += period;
    }
  }
  return at;
}

std::optional<SiteClock> parse_site_clock(const std::string& text) {
  const auto json = nlohmann::json::parse(text, nullptr, false);
  if (json.is_discarded() || !json.is_object()) {
    return std::nullopt;
  }
  try {
    SiteClock clock;
    clock.epoch_ns = json.at("epoch_ns").get<std::int64_t>();
    clock.media_ns = json.at("media_ns").get<std::int64_t>();
    clock.speed = json.value("speed", 1.0);
    clock.playing = json.value("playing", true);
    clock.period_ns = json.value("period_ns", std::int64_t{0});
    if (clock.speed <= 0.0 || clock.period_ns < 0) {
      return std::nullopt;
    }
    return clock;
  } catch (const nlohmann::json::exception&) {
    return std::nullopt;
  }
}

std::int64_t wall_now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

std::optional<SiteClock> SiteClockFile::poll() {
  std::error_code error;
  const auto modified = std::filesystem::last_write_time(path_, error);
  if (error || modified_ == modified) {
    return std::nullopt;
  }
  std::ifstream in(path_);
  std::ostringstream text;
  text << in.rdbuf();
  auto clock = parse_site_clock(text.str());
  if (clock) {
    // A half-written file is read again on the next call (the manager replaces it atomically).
    modified_ = modified;
  }
  return clock;
}

}  // namespace jp::server
