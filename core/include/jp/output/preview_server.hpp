#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace jp {

// Minimal HTTP server for the live camera view of the web panel (behind the panel's reverse proxy):
//   GET /video.mjpg   multipart/x-mixed-replace JPEG stream
//   GET /frame.jpg    the latest frame
// Frames are only rendered while someone watches (wanted()).
class PreviewServer {
 public:
  explicit PreviewServer(int port, int max_clients = 8);
  ~PreviewServer();
  bool wanted() const;
  void push(std::vector<unsigned char> jpeg);
  int clients() const { return clients_.load(); }

 private:
  void acceptLoop();
  void serve(int fd);

  int port_, max_clients_, listen_fd_ = -1;
  std::atomic<bool> stop_{false};
  std::atomic<int> clients_{0};
  std::atomic<double> last_request_{0};
  std::thread acceptor_;
  mutable std::mutex m_;
  std::condition_variable cv_;
  std::vector<unsigned char> frame_;
  std::uint64_t seq_ = 0;
  std::atomic<int> active_{0};
};

}  // namespace jp
