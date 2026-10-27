#include "jp/output/preview_server.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <spdlog/spdlog.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstring>

namespace jp {
namespace {
double now() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }

bool sendAll(int fd, const void* data, size_t n) {
  const char* p = static_cast<const char*>(data);
  while (n > 0) {
    const ssize_t k = ::send(fd, p, n, MSG_NOSIGNAL);
    if (k <= 0) return false;
    p += k;
    n -= static_cast<size_t>(k);
  }
  return true;
}
bool sendStr(int fd, const std::string& s) { return sendAll(fd, s.data(), s.size()); }
}  // namespace

PreviewServer::PreviewServer(int port, int max_clients) : port_(port), max_clients_(max_clients) {
  listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  int one = 1;
  ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_addr.s_addr = htonl(INADDR_ANY);
  a.sin_port = htons(static_cast<uint16_t>(port_));
  if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&a), sizeof(a)) < 0 || ::listen(listen_fd_, 16) < 0) {
    spdlog::warn("Live view: cannot listen on port {} ({})", port_, std::strerror(errno));
    ::close(listen_fd_);
    listen_fd_ = -1;
    return;
  }
  acceptor_ = std::thread([this] { acceptLoop(); });
  spdlog::info("Live view on http://0.0.0.0:{}/video.mjpg", port_);
}

PreviewServer::~PreviewServer() {
  stop_ = true;
  cv_.notify_all();
  if (listen_fd_ >= 0) ::shutdown(listen_fd_, SHUT_RDWR);
  if (acceptor_.joinable()) acceptor_.join();
  if (listen_fd_ >= 0) ::close(listen_fd_);
  for (int i = 0; i < 100 && active_.load() > 0; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(20));
}

bool PreviewServer::wanted() const { return clients_.load() > 0 || now() - last_request_.load() < 3.0; }

void PreviewServer::push(std::vector<unsigned char> jpeg) {
  {
    std::lock_guard<std::mutex> lk(m_);
    frame_ = std::move(jpeg);
    ++seq_;
  }
  cv_.notify_all();
}

void PreviewServer::acceptLoop() {
  while (!stop_) {
    pollfd p{listen_fd_, POLLIN, 0};
    if (::poll(&p, 1, 500) <= 0) continue;
    const int fd = ::accept(listen_fd_, nullptr, nullptr);
    if (fd < 0) continue;
    if (clients_.load() >= max_clients_) {
      sendStr(fd, "HTTP/1.1 503 Service Unavailable\r\nContent-Length: 0\r\nConnection: close\r\n\r\n");
      ::close(fd);
      continue;
    }
    active_++;
    std::thread([this, fd] {
      serve(fd);
      active_--;
    }).detach();
  }
}

void PreviewServer::serve(int fd) {
  timeval tv{5, 0};
  ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
  char buf[2048];
  const ssize_t n = ::recv(fd, buf, sizeof(buf) - 1, 0);
  std::string req = n > 0 ? std::string(buf, static_cast<size_t>(n)) : "";
  const auto sp1 = req.find(' '), sp2 = req.find(' ', sp1 + 1);
  std::string path = sp1 != std::string::npos && sp2 != std::string::npos ? req.substr(sp1 + 1, sp2 - sp1 - 1) : "";
  if (const auto q = path.find('?'); q != std::string::npos) path = path.substr(0, q);
  last_request_ = now();
  const std::string common = "Cache-Control: no-cache, no-store\r\nAccess-Control-Allow-Origin: *\r\nConnection: close\r\n";
  // The proxy may keep its own prefix (/live/<site>/video.mjpg).
  if (path.ends_with("/video.mjpg")) path = "/video.mjpg";
  if (path.ends_with("/frame.jpg")) path = "/frame.jpg";
  if (path == "/frame.jpg") {
    std::vector<unsigned char> f;
    {
      std::unique_lock<std::mutex> lk(m_);
      // A first request starts rendering: wait briefly for a frame.
      cv_.wait_for(lk, std::chrono::seconds(2), [&] { return !frame_.empty() || stop_.load(); });
      f = frame_;
    }
    if (f.empty()) {
      sendStr(fd, "HTTP/1.1 503 Service Unavailable\r\nContent-Length: 0\r\n" + common + "\r\n");
    } else {
      sendStr(fd, "HTTP/1.1 200 OK\r\nContent-Type: image/jpeg\r\nContent-Length: " + std::to_string(f.size()) +
                      "\r\n" + common + "\r\n");
      sendAll(fd, f.data(), f.size());
    }
    ::close(fd);
    return;
  }
  if (path != "/video.mjpg") {
    sendStr(fd, "HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\n" + common + "\r\n");
    ::close(fd);
    return;
  }
  clients_++;
  bool ok = sendStr(fd, "HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=jpframe\r\n" + common + "\r\n");
  std::uint64_t seen = 0;
  while (ok && !stop_) {
    std::vector<unsigned char> f;
    {
      std::unique_lock<std::mutex> lk(m_);
      if (!cv_.wait_for(lk, std::chrono::seconds(10), [&] { return seq_ != seen || stop_.load(); })) continue;
      if (stop_) break;
      seen = seq_;
      f = frame_;
    }
    ok = sendStr(fd, "--jpframe\r\nContent-Type: image/jpeg\r\nContent-Length: " + std::to_string(f.size()) + "\r\n\r\n") &&
         sendAll(fd, f.data(), f.size()) && sendStr(fd, "\r\n");
  }
  clients_--;
  ::close(fd);
}

}  // namespace jp
