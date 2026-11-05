#include "jp/server/video_encoder.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <vector>

namespace {

using namespace std::chrono_literals;

// A sink that is slow (blocked until released) must neither hold up submit nor receive every
// frame: frames that arrive meanwhile replace each other and only the latest is compressed.
TEST(VideoEncoder, BusySinkDropsWaitingFramesWithoutBlocking) {
  std::mutex mutex;
  std::condition_variable changed;
  std::vector<std::int64_t> seen;
  bool release = false;
  {
    jp::server::VideoEncoder encoder([&](std::int64_t frame, const cv::Mat& image, const cv::Mat&) {
      std::unique_lock lock(mutex);
      EXPECT_FALSE(image.empty());
      seen.push_back(frame);
      changed.notify_all();
      changed.wait(lock, [&] { return release; });
    });
    const cv::Mat image(4, 4, CV_8UC3, cv::Scalar::all(0));
    encoder.submit(1, image, {});
    {
      std::unique_lock lock(mutex);
      ASSERT_TRUE(changed.wait_for(lock, 5s, [&] { return seen.size() == 1; }));
    }
    const auto started = std::chrono::steady_clock::now();
    for (std::int64_t frame = 2; frame <= 5; ++frame) {
      encoder.submit(frame, image, {});
    }
    EXPECT_LT(std::chrono::steady_clock::now() - started, 1s);
    {
      const std::scoped_lock lock(mutex);
      release = true;
    }
    changed.notify_all();
    std::unique_lock lock(mutex);
    ASSERT_TRUE(changed.wait_for(lock, 5s, [&] { return seen.size() == 2; }));
  }
  EXPECT_EQ(seen, (std::vector<std::int64_t>{1, 5}));
}

}  // namespace
