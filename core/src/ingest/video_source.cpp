#include "jp/ingest/video_source.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <thread>

namespace jp {
namespace {
double nowSeconds() {
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}
bool startsWith(const std::string& s, const std::string& p) { return s.rfind(p, 0) == 0; }
}  // namespace

VideoSource::VideoSource(VideoSourceConfig cfg) : cfg_(std::move(cfg)) {
  max_fps_ = cfg_.max_fps;
  if (!open()) throw std::runtime_error("cannot open video source " + cfg_.uri);
}

bool VideoSource::open() {
  const std::string& u = cfg_.uri;
  bool ok = false;
  is_file_ = false;
  if (startsWith(u, "gst:")) {
    ok = cap_.open(u.substr(4), cv::CAP_GSTREAMER);
  } else if (startsWith(u, "rtsp://") || startsWith(u, "http://") || startsWith(u, "https://")) {
    // Prefer FFmpeg (low latency TCP); fall back to GStreamer.
    ok = cap_.open(u, cv::CAP_FFMPEG) || cap_.open("rtspsrc location=" + u + " latency=100 ! decodebin ! "
                                                   "videoconvert ! video/x-raw,format=BGR ! appsink drop=1 sync=0",
                                                   cv::CAP_GSTREAMER);
  } else if (startsWith(u, "usb:")) {
    ok = cap_.open(std::stoi(u.substr(4)), cv::CAP_V4L2);
  } else if (startsWith(u, "v4l2://")) {
    ok = cap_.open(u.substr(7), cv::CAP_V4L2);
  } else {
    const std::string path = startsWith(u, "file://") ? u.substr(7) : u;
    ok = cap_.open(path, cv::CAP_FFMPEG);
    is_file_ = true;
  }
  if (!ok) return false;
  fps_ = cap_.get(cv::CAP_PROP_FPS);
  if (!(fps_ > 1 && fps_ < 241)) fps_ = 30;
  size_ = {static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH)), static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT))};
  file_t0_ = nowSeconds();
  frames_ = 0;
  paced_ = false;
  spdlog::info("Opened {} ({}x{} @ {:.2f} fps)", describe(), size_.width, size_.height, fps_);
  return true;
}

void VideoSource::startAt(double epoch) {
  if (!is_file_) return;
  epoch_ = epoch;
  double idx = std::max(0.0, (nowSeconds() - epoch) * fps_);
  const double n = cap_.get(cv::CAP_PROP_FRAME_COUNT);
  if (cfg_.loop && n > 1) {
    // Whole loops already played on the shared clock.
    const double k = std::floor(idx / n);
    epoch_ += k * n / fps_;
    idx -= k * n;
  }
  frames_ = static_cast<std::uint64_t>(idx);
  file_t0_ = epoch_;
  cap_.set(cv::CAP_PROP_POS_FRAMES, static_cast<double>(frames_));
  paced_ = true;
  last_emit_ = -1;
  spdlog::info("Playing on the shared clock from frame {}", frames_);
}

std::string VideoSource::describe() const {
  // Never log credentials embedded in RTSP URIs.
  std::string u = cfg_.uri;
  const auto at = u.find('@');
  const auto scheme = u.find("://");
  if (at != std::string::npos && scheme != std::string::npos && at > scheme) u = u.substr(0, scheme + 3) + "***" + u.substr(at);
  return u;
}

bool VideoSource::read(cv::Mat& frame, double& stamp) {
  for (;;) {
    if (!cap_.read(frame) || frame.empty()) {
      if (is_file_ && cfg_.loop) {
        cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
        file_t0_ += frames_ / fps_;
        if (epoch_ > 0) epoch_ += frames_ / fps_;
        wall0_ = std::chrono::steady_clock::now();
        frames_ = 0;
        loops_++;
        continue;
      }
      if (is_file_) return false;
      spdlog::warn("Stream {} lost, reconnecting", describe());
      std::this_thread::sleep_for(std::chrono::duration<double>(cfg_.reconnect_s));
      open();
      continue;
    }
    if (is_file_) {
      // File time base: frame index / fps, anchored to the wall clock at open.
      stamp = file_t0_ + frames_ / fps_;
      if (cfg_.realtime && epoch_ > 0) {
        // Shared clock: frame k is due at epoch + k / fps on every camera of the site.
        const auto due = std::chrono::system_clock::time_point(std::chrono::duration_cast<std::chrono::system_clock::duration>(
            std::chrono::duration<double>(epoch_ + frames_ / fps_)));
        std::this_thread::sleep_until(due);
      } else if (cfg_.realtime) {
        // Anchor pacing at the first frame actually read (engine builds may delay the first read).
        if (!paced_) {
          wall0_ = std::chrono::steady_clock::now() - std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(frames_ / fps_));
          paced_ = true;
        }
        const auto due = wall0_ + std::chrono::duration<double>(frames_ / fps_);
        std::this_thread::sleep_until(due);
      }
      last_index_ = frames_++;
    } else {
      stamp = nowSeconds();
      last_index_ = frames_++;
    }
    const double mf = max_fps_.load();
    if (mf > 0 && last_emit_ >= 0 && stamp - last_emit_ < 1.0 / mf - 1e-3) continue;
    last_emit_ = stamp;
    return true;
  }
}

}  // namespace jp
