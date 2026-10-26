#pragma once

#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace jp {

// Where metadata goes. Implementations must never receive image data.
class Sink {
 public:
  virtual ~Sink() = default;
  virtual void publish(const std::string& topic, const std::string& payload, int qos = 0, bool retain = false) = 0;
  virtual bool connected() const { return true; }
  // Commands from operators (e.g. the web panel's manual calibration). Handlers run on the sink's thread.
  using Handler = std::function<void(const std::string& topic, const std::string& payload)>;
  virtual void subscribe(const std::string& /*topic*/, Handler /*cb*/) {}
};

struct MqttConfig {
  std::string uri = "tcp://localhost:1883";
  std::string client_id = "junction-perception";
  std::string username, password;
  int keepalive_s = 20;
};

// MQTT (paho C async client) with automatic reconnect and a bounded offline queue.
class MqttSink : public Sink {
 public:
  explicit MqttSink(const MqttConfig& cfg);
  ~MqttSink() override;
  void publish(const std::string& topic, const std::string& payload, int qos, bool retain) override;
  bool connected() const override;
  void subscribe(const std::string& topic, Handler cb) override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// JSON-lines file (for recording metadata and benchmarking).
class FileSink : public Sink {
 public:
  explicit FileSink(const std::string& path) : out_(path, std::ios::app) {}
  void publish(const std::string& topic, const std::string& payload, int, bool) override {
    std::lock_guard<std::mutex> lk(m_);
    out_ << "{\"topic\":\"" << topic << "\",\"msg\":" << payload << "}\n";
  }

 private:
  std::mutex m_;
  std::ofstream out_;
};

class MultiSink : public Sink {
 public:
  void add(std::unique_ptr<Sink> s) { sinks_.push_back(std::move(s)); }
  void publish(const std::string& topic, const std::string& payload, int qos, bool retain) override {
    for (auto& s : sinks_) s->publish(topic, payload, qos, retain);
  }
  bool connected() const override {
    for (const auto& s : sinks_)
      if (!s->connected()) return false;
    return true;
  }
  void subscribe(const std::string& topic, Handler cb) override {
    for (auto& s : sinks_) s->subscribe(topic, cb);
  }

 private:
  std::vector<std::unique_ptr<Sink>> sinks_;
};

}  // namespace jp
