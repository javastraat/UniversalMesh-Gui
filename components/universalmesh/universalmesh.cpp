#include "universalmesh.h"
#include "esphome/core/log.h"

#ifdef ESP8266
#include <ESP8266WiFi.h>
#elif defined(ESP32)
#include <esp_wifi.h>
#endif

namespace esphome {
namespace universalmesh {

static const char *TAG = "universalmesh";

static UniversalMeshComponent *instance_ = nullptr;

static void on_mesh_message(MeshPacket *packet, uint8_t *sender_mac) {
  if (instance_) instance_->on_message(packet, sender_mac);
}

void UniversalMeshComponent::setup() {
  instance_ = this;
#ifdef ESP8266
  WiFi.macAddress(my_mac_);
#else
  esp_wifi_get_mac(WIFI_IF_STA, my_mac_);
#endif
  ESP_LOGI(TAG, "MAC: %02X:%02X:%02X:%02X:%02X:%02X",
           my_mac_[0], my_mac_[1], my_mac_[2],
           my_mac_[3], my_mac_[4], my_mac_[5]);

  mesh_.begin(1);
  mesh_.onReceive(on_mesh_message);

#ifdef ESP32
  // Disconnect STA so WiFi doesn't hop channels during ESP-NOW scanning.
  // Mesh nodes don't need WiFi — the coordinator bridges to HA.
  esp_wifi_disconnect();
#endif

#ifdef ESP8266
  // CRITICAL: Disable WiFi STA scanning to prevent probe requests from disrupting ESP-NOW.
  // Disconnect saves credentials but drops any active connection; autoConnect keeps radio stable.
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect(true);  // true = save credentials but disconnect
    ESP_LOGI(TAG, "WiFi STA disabled — radio stable for ESP-NOW discovery");
    delay(200); // Let radio settle after WiFi disconnect (increased for ESP8266 stability)
  }
#endif

  last_retry_ = millis();
}

bool UniversalMeshComponent::connect_to_coordinator_() {
  mesh_channel_ = mesh_.findCoordinatorChannel(node_name_);
  if (mesh_channel_ == 0) return false;

  mesh_.getCoordinatorMac(coordinator_mac_);
  mesh_.begin(mesh_channel_);
  mesh_.setCoordinatorMac(coordinator_mac_);
  mesh_.onReceive(on_mesh_message);
#ifdef ESP8266
  WiFi.macAddress(my_mac_);
#else
  esp_wifi_get_mac(WIFI_IF_STA, my_mac_);
#endif

  mesh_.send(coordinator_mac_, MESH_TYPE_PING, 0x00,
             (const uint8_t *)node_name_, strlen(node_name_), 4);
  mesh_.send(coordinator_mac_, MESH_TYPE_DATA, 0x06,
             (const uint8_t *)node_name_, strlen(node_name_), 4);

#ifdef ESP8266
  // Start a hidden soft-AP on the coordinator's channel. On ESP8266 the AP
  // channel takes priority over STA scanning, so the radio is locked to this
  // channel and WiFi probes can no longer disrupt ESP-NOW traffic.
  if (WiFi.softAP(node_name_, "", mesh_channel_, true /* hidden */)) {
    ap_active_ = true;
    ESP_LOGI(TAG, "AP locked to ch%d — WiFi probes suppressed", mesh_channel_);
  }

  // Disconnect from any configured networks and disable auto-connect
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect(true);  // true = save credentials but disconnect
    ESP_LOGD(TAG, "Disconnected from WiFi STA mode");
  }
#endif

  connected_ = true;
  return true;
}

void UniversalMeshComponent::loop() {
  mesh_.update();

  if (!connected_) {
    if (millis() - last_retry_ > 30000) {
      last_retry_ = millis();
      ESP_LOGI(TAG, "Scanning for coordinator...");
#ifdef ESP8266
      if (ap_active_) {
        WiFi.softAPdisconnect(false);
        ap_active_ = false;
      }
#endif
      mesh_.begin(1);  // reinit ESP-NOW — WiFi adapter restarts deinit it
      mesh_.onReceive(on_mesh_message);
      if (connect_to_coordinator_()) {
        ESP_LOGI(TAG, "Coordinator found on channel %d", mesh_channel_);
        last_heartbeat_ = millis() - heartbeat_interval_ms_;
      }
    }
    return;
  }

  unsigned long now = millis();

  if (now - last_heartbeat_ >= heartbeat_interval_ms_) {
    last_heartbeat_ = now;
    send_heartbeat_();
  }

#ifdef ESP8266
  // Ensure WiFi STA stays disconnected after coordinator is found
  if (connected_ && ap_active_ && WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect(true);
  }
#endif
}

void UniversalMeshComponent::update() {
  if (connected_) send_sensors_();
}

void UniversalMeshComponent::send_heartbeat_() {
  uint8_t hb = 0x01;
  mesh_.send(coordinator_mac_, MESH_TYPE_DATA, 0x05, &hb, 1, 4);
  mesh_.send(coordinator_mac_, MESH_TYPE_DATA, 0x06,
             (const uint8_t *)node_name_, strlen(node_name_), 4);
  ESP_LOGD(TAG, "Heartbeat sent");
}

void UniversalMeshComponent::send_sensors_() {
  JsonDocument doc;
  doc["name"] = node_name_;
  bool has_value = false;

  for (auto &entry : sensors_) {
    if (entry.sensor->has_state()) {
      char buf[12];
      snprintf(buf, sizeof(buf), "%.1f", entry.sensor->state);
      doc[entry.key.c_str()] = serialized(String(buf));
      has_value = true;
    }
  }

  if (!has_value) return;

  String payload;
  serializeJson(doc, payload);

  if (mesh_.sendToCoordinator(0x01, payload)) {
    ESP_LOGI(TAG, "Sent: %s", payload.c_str());
  } else {
    ESP_LOGW(TAG, "Send failed");
  }
}

void UniversalMeshComponent::send_info_(uint8_t *dest_mac, uint8_t app_id) {
  char mac_str[18];
  snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
           my_mac_[0], my_mac_[1], my_mac_[2],
           my_mac_[3], my_mac_[4], my_mac_[5]);
  JsonDocument doc;
  doc["n"]    = node_name_;
  doc["mac"]  = mac_str;
  doc["up"]   = (unsigned long)(millis() / 1000UL);
  doc["heap"] = ESP.getFreeHeap();
  doc["ch"]   = mesh_channel_;
#ifdef ESP32
  doc["chip"] = ESP.getChipModel();
  doc["rev"]  = (int)ESP.getChipRevision();
#else
  doc["chip"] = "ESP8266";
#endif
  String out;
  serializeJson(doc, out);
  mesh_.send(dest_mac, MESH_TYPE_DATA, app_id, out);
}

void UniversalMeshComponent::on_message(MeshPacket *packet, uint8_t *sender_mac) {
  bool direct_to_me = (memcmp(packet->destMac, my_mac_, 6) == 0);
  if (packet->type != MESH_TYPE_DATA || !direct_to_me) return;

  char msg[201];
  uint8_t len = packet->payloadLen > 200 ? 200 : packet->payloadLen;
  memcpy(msg, packet->payload, len);
  msg[len] = '\0';

  if (len < 4 || strncmp(msg, "cmd:", 4) != 0) return;
  if (!connected_ || memcmp(sender_mac, coordinator_mac_, 6) != 0) return;

  const char *command = msg + 4;
  char ack[220];
  snprintf(ack, sizeof(ack), "command received:%s", command);
  mesh_.send(sender_mac, MESH_TYPE_DATA, packet->appId,
             (const uint8_t *)ack, strlen(ack), 4);

  if (strcmp(command, "info") == 0 || strcmp(command, "info:long") == 0) {
    send_info_(sender_mac, packet->appId);
  } else if (strcmp(command, "reboot") == 0) {
    ESP_LOGI(TAG, "Reboot command received");
    delay(100);
    ESP.restart();
  }
}

}  // namespace universalmesh
}  // namespace esphome
