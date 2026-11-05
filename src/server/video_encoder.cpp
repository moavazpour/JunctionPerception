#include "jp/server/video_encoder.hpp"

#include <utility>

namespace jp::server {

VideoEncoder::VideoEncoder(Sink sink)
    : sink_(std::move(sink)), thread_([this](const std::stop_token& stop) { run(stop); }) {}

void VideoEncoder::submit(std::int64_t frame, cv::Mat image, cv::Mat raw) {
  {
    const std::scoped_lock lock(mutex_);
    pending_ = Job{.frame = frame, .image = std::move(image), .raw = std::move(raw)};
    has_pending_ = true;
  }
  ready_.notify_one();
}

void VideoEncoder::run(const std::stop_token& stop) {
  while (true) {
    Job job;
    {
      std::unique_lock lock(mutex_);
      if (!ready_.wait(lock, stop, [this] { return has_pending_; })) {
        return;
      }
      job = std::exchange(pending_, Job{});
      has_pending_ = false;
    }
    sink_(job.frame, job.image, job.raw);
  }
}

}  // namespace jp::server
