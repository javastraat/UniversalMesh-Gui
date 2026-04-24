#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "lib/UniversalMesh.h"

#include <ArduinoJson.h>
#include <vector>
#include <string>

namespace esphome {
namespace universalmesh {

class UniversalMeshComponent : public Component {
 public:
  void set_node_name(const char *name) { node_name_ = name; }
  void set_update_interval_ms(uint32_t ms) { update_interval_ms_ = ms; }
  void set_heartbeat_interval_ms(uint32_t ms) { heartbeat_interval_ms_ = ms; }

  void register_sensor(const char *key, sensor::Sensor *s) {
    sensors_.push_back({std::string(key), s});
  }

  void setup() override;
  void loop() override;
  float get_setup_priority() const override { return -10.0f; }

  void on_message(MeshPacket *packet, uint8_t *sender_mac);

 protected:
  struct SensorEntry {
    std::string key;
    sensor::Sensor *sensor;
  };

  const char *node_name_{"mesh-node"};
  uint32_t update_interval_ms_{60000};
  uint32_t heartbeat_interval_ms_{120000};

  UniversalMesh mesh_;
  uint8_t my_mac_[6]{};
  uint8_t coordinator_mac_[6]{};
  uint8_t mesh_channel_{0};
  bool connected_{false};

  unsigned long last_sensor_{0};
  unsigned long last_heartbeat_{0};
  unsigned long last_retry_{0};

  std::vector<SensorEntry> sensors_;

  bool connect_to_coordinator_();
  void send_sensors_();
  void send_heartbeat_();
  void send_info_(uint8_t *dest_mac, uint8_t app_id);
};

}  // namespace universalmesh
}  // namespace esphome
