#include <spdlog/spdlog.h>

#include "jp/output/sink.hpp"

#ifdef JP_HAVE_MQTT
#include <MQTTAsync.h>

#include <atomic>
#include <stdexcept>

namespace jp {

struct MqttSink::Impl {
  MQTTAsync client = nullptr;
  std::atomic<bool> up{false};
  MqttConfig cfg;

  static void onConnect(void* ctx, MQTTAsync_successData*) {
    static_cast<Impl*>(ctx)->up = true;
    spdlog::info("MQTT connected to {}", static_cast<Impl*>(ctx)->cfg.uri);
  }
  static void onConnectFailure(void* ctx, MQTTAsync_failureData* resp) {
    spdlog::warn("MQTT connect failed ({}), retrying", resp ? resp->code : -1);
  }
  static void onConnLost(void* ctx, char* cause) {
    static_cast<Impl*>(ctx)->up = false;
    spdlog::warn("MQTT connection lost: {}", cause ? cause : "unknown");
  }
  static int onMessage(void*, char* topic, int, MQTTAsync_message* m) {
    MQTTAsync_freeMessage(&m);
    MQTTAsync_free(topic);
    return 1;
  }
  static void onConnected(void* ctx, char*) { static_cast<Impl*>(ctx)->up = true; }
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

}  // namespace jp

#else

namespace jp {
struct MqttSink::Impl {};
MqttSink::MqttSink(const MqttConfig&) { spdlog::warn("Built without MQTT support; MQTT output disabled"); }
MqttSink::~MqttSink() = default;
bool MqttSink::connected() const { return false; }
void MqttSink::publish(const std::string&, const std::string&, int, bool) {}
}  // namespace jp

#endif
