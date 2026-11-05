#pragma once

#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <opencv2/core/mat.hpp>
#include <stop_token>
#include <thread>

namespace jp::server {

/// Compresses the console video off the processing thread. A frame submitted while the previous
/// one is still being compressed replaces the waiting one: when the host is short of time the
/// console shows fewer frames, the detection keeps its rate.
class VideoEncoder {
 public:
  /// Called on the encoder's thread, one frame at a time.
  using Sink = std::function<void(std::int64_t frame, const cv::Mat& image, const cv::Mat& raw)>;

  explicit VideoEncoder(Sink sink);
  /// Never waits for the sink.
  void submit(std::int64_t frame, cv::Mat image, cv::Mat raw);

 private:
  struct Job {
    std::int64_t frame = 0;
    cv::Mat image;
    cv::Mat raw;
  };
  void run(const std::stop_token& stop);

  Sink sink_;
  std::mutex mutex_;
  std::condition_variable_any ready_;
  Job pending_;
  bool has_pending_ = false;
  std::jthread thread_;  // last: stopped and joined before the members above are destroyed
};

}  // namespace jp::server
