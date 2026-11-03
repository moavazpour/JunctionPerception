#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>

namespace jp::server {

/// Playback clock shared by the cameras of a site (written by the site manager): every camera
/// plays its recording at the position this clock gives for the current wall time, so cameras
/// of different speed stay in step and seek, pause and loop together.
struct SiteClock {
  std::int64_t epoch_ns = 0;  ///< Wall time (Unix ns) at which the recording was at media_ns.
  std::int64_t media_ns = 0;
  double speed = 1.0;
  bool playing = true;
  std::int64_t period_ns = 0;  ///< Loop length of the site's recordings; 0: not known yet.

  /// Recording time due at wall time `now_ns`; wrapped into [0, period_ns) if `period_ns` > 0.
  [[nodiscard]] std::int64_t position(std::int64_t now_ns, std::int64_t period_ns) const;
};

/// Parses the manager's clock JSON; empty if it is malformed.
[[nodiscard]] std::optional<SiteClock> parse_site_clock(const std::string& text);

/// Wall time in Unix ns (shared by the processes of a host).
[[nodiscard]] std::int64_t wall_now_ns();

/// A clock file, re-read only when it changes.
class SiteClockFile {
 public:
  explicit SiteClockFile(std::filesystem::path path) : path_(std::move(path)) {}

  /// The clock if the file changed since the last call and is valid, else empty.
  [[nodiscard]] std::optional<SiteClock> poll();

 private:
  std::filesystem::path path_;
  std::optional<std::filesystem::file_time_type> modified_;
};

}  // namespace jp::server
