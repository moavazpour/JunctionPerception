#include <spdlog/spdlog.h>

#include "jp/output/sink.hpp"

#ifdef JP_HAVE_MQTT
#include <MQTTAsync.h>

#include <atomic>
#include <mutex>
#include <stdexcept>
#include <utility>

namespace jp {

struct MqttSink::Impl {
  MQTTAsync client = nullptr;
  std::atomic<bool> up{false};
  MqttConfig cfg;
  std::mutex m;
  std::vector<std::pair<std::string, Handler>> subs;

  void subscribeAll() {
    std::lock_guard<std::mutex> lk(m);
    for (const auto& [topic, _] : subs) {
      MQTTAsync_responseOptions ro = MQTTAsync_responseOptions_initializer;
      MQTTAsync_subscribe(client, topic.c_str(), 1, &ro);
    }
  }
  static void onConnect(void* ctx, MQTTAsync_successData*) {
    auto* self = static_cast<Impl*>(ctx);
    self->up = true;
    spdlog::info("MQTT connected to {}", self->cfg.uri);
    self->subscribeAll();
  }
  static void onConnectFailure(void*, MQTTAsync_failureData* resp) {
    spdlog::warn("MQTT connect failed ({}), retrying", resp ? resp->code : -1);
  }
  static void onConnLost(void* ctx, char* cause) {
    static_cast<Impl*>(ctx)->up = false;
    spdlog::warn("MQTT connection lost: {}", cause ? cause : "unknown");
  }
  static int onMessage(void* ctx, char* topic, int topic_len, MQTTAsync_message* msg) {
    auto* self = static_cast<Impl*>(ctx);
    const std::string t = topic_len > 0 ? std::string(topic, topic_len) : std::string(topic);
    const std::string payload(static_cast<const char*>(msg->payload), msg->payloadlen);
    std::vector<Handler> hs;
    {
      std::lock_guard<std::mutex> lk(self->m);
      for (const auto& [st, h] : self->subs)
        if (st == t) hs.push_back(h);
    }
    for (const auto& h : hs) {
      try {
        h(t, payload);
      } catch (const std::exception& e) {
        spdlog::warn("MQTT handler for {} failed: {}", t, e.what());
      }
    }
    MQTTAsync_freeMessage(&msg);
    MQTTAsync_free(topic);
    return 1;
  }
  static void onConnected(void* ctx, char*) {  // also called after an automatic reconnect
    auto* self = static_cast<Impl*>(ctx);
    self->up = true;
    self->subscribeAll();
  }
};

MqttSink::MqttSink(const MqttConfig& cfg) : impl_(std::make_unique<Impl>()) {
  impl_->cfg = cfg;
  MQTTAsync_createOptions co = MQTTAsync_createOptions_initializer;
  co.sendWhileDisconnected = 1;
  co.maxBufferedMessages = 2000;
  if (MQTTAsync_createWithOptions(&impl_->client, cfg.uri.c_str(), cfg.client_id.c_str(), MQTTCLIENT_PERSISTENCE_NONE,
                                  nullptr, &co) != MQTTASYNC_SUCCESS)
    throw std::runtime_error("cannot create MQTT client");
  MQTTAsync_setCallbacks(impl_->client, impl_.get(), &Impl::onConnLost, &Impl::onMessage, nullptr);
  MQTTAsync_setConnected(impl_->client, impl_.get(), &Impl::onConnected);
  MQTTAsync_connectOptions opts = MQTTAsync_connectOptions_initializer;
  opts.keepAliveInterval = cfg.keepalive_s;
  opts.cleansession = 1;
  opts.automaticReconnect = 1;
  opts.minRetryInterval = 1;
  opts.maxRetryInterval = 10;
  opts.onSuccess = &Impl::onConnect;
  opts.onFailure = &Impl::onConnectFailure;
  opts.context = impl_.get();
  if (!cfg.username.empty()) {
    opts.username = cfg.username.c_str();
    opts.password = cfg.password.c_str();
  }
  MQTTAsync_connect(impl_->client, &opts);
}

MqttSink::~MqttSink() {
  if (impl_->client) {
    MQTTAsync_disconnectOptions d = MQTTAsync_disconnectOptions_initializer;
    d.timeout = 1000;
    MQTTAsync_disconnect(impl_->client, &d);
    MQTTAsync_destroy(&impl_->client);
  }
}

bool MqttSink::connected() const { return impl_->up; }

void MqttSink::publish(const std::string& topic, const std::string& payload, int qos, bool retain) {
  MQTTAsync_message m = MQTTAsync_message_initializer;
  m.payload = const_cast<char*>(payload.data());
  m.payloadlen = static_cast<int>(payload.size());
  m.qos = qos;
  m.retained = retain ? 1 : 0;
  MQTTAsync_responseOptions ro = MQTTAsync_responseOptions_initializer;
  MQTTAsync_sendMessage(impl_->client, topic.c_str(), &m, &ro);
}

void MqttSink::subscribe(const std::string& topic, Handler cb) {
  {
    std::lock_guard<std::mutex> lk(impl_->m);
    impl_->subs.emplace_back(topic, std::move(cb));
  }
  if (impl_->up) {
    MQTTAsync_responseOptions ro = MQTTAsync_responseOptions_initializer;
    MQTTAsync_subscribe(impl_->client, topic.c_str(), 1, &ro);
  }
}

}  // namespace jp

#else

namespace jp {
struct MqttSink::Impl {};
MqttSink::MqttSink(const MqttConfig&) { spdlog::warn("Built without MQTT support; MQTT output disabled"); }
MqttSink::~MqttSink() = default;
bool MqttSink::connected() const { return false; }
void MqttSink::publish(const std::string&, const std::string&, int, bool) {}
void MqttSink::subscribe(const std::string&, Handler) {}
}  // namespace jp

#endif
