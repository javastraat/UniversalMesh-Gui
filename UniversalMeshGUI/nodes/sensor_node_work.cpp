#include <Arduino.h>
#include <ArduinoJson.h>
#if defined(ESP32)
  #include <esp_wifi.h>
#endif
#include "UniversalMesh.h"
#include "ota_update.h"
#ifdef HAS_PIR
#include "pir_sensor.h"
#endif

#ifndef NODE_NAME
  #define NODE_NAME "sensor-node"
#endif
#define HEARTBEAT_INTERVAL  60000
#define TEMP_INTERVAL       30000

UniversalMesh mesh;
uint8_t myMac[6]          = {0};
uint8_t coordinatorMac[6] = {0};
uint8_t meshChannel       = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastTemp      = 0;
volatile bool otaRequested  = false;

// Send all node info as a single JSON (200-byte payload fits everything)
static void sendInfo(uint8_t* destMac, uint8_t appId) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           myMac[0], myMac[1], myMac[2], myMac[3], myMac[4], myMac[5]);
  JsonDocument doc;
  doc["n"]    = NODE_NAME;
  doc["mac"]  = macStr;
  doc["up"]   = (unsigned long)(millis() / 1000UL);
  doc["heap"] = ESP.getFreeHeap();
  doc["rssi"] = WiFi.RSSI();
  doc["ch"]   = meshChannel;
  #if defined(ESP32)
    doc["chip"] = ESP.getChipModel();
    doc["rev"]  = (int)ESP.getChipRevision();
  #else
    doc["chip"] = "ESP8266";
  #endif
  String out;
  serializeJson(doc, out);
  mesh.send(destMac, MESH_TYPE_DATA, appId, out);
}

#ifdef HAS_PIR
void onPirTriggered() {
  if (!mesh.isCoordinatorFound()) return;
  JsonDocument doc;
  doc["name"] = NODE_NAME;
  doc["pir"]  = 1;
  String payload;
  serializeJson(doc, payload);
  mesh.sendToCoordinator(0x02, payload);
  Serial.printf("[PIR] Alert sent: %s\n", payload.c_str());
}
#endif

void onMeshMessage(MeshPacket* packet, uint8_t* senderMac) {
  bool directToMe = (memcmp(packet->destMac, myMac, 6) == 0);
  if (packet->type != MESH_TYPE_DATA || !directToMe) return;

  char msg[201];
  uint8_t len = packet->payloadLen > 200 ? 200 : packet->payloadLen;
  memcpy(msg, packet->payload, len);
  msg[len] = '\0';

  Serial.printf("[RX] From %02X:%02X:%02X:%02X:%02X:%02X | App 0x%02X | %s\n",
          packet->srcMac[0], packet->srcMac[1], packet->srcMac[2],
          packet->srcMac[3], packet->srcMac[4], packet->srcMac[5],
          packet->appId, msg);

  if (len < 4 || strncmp(msg, "cmd:", 4) != 0) return;

  bool fromCoordinator = mesh.isCoordinatorFound() &&
                         (memcmp(packet->srcMac, coordinatorMac, 6) == 0);
  if (!fromCoordinator) {
    Serial.println("[CMD] Ignored — not from coordinator");
    return;
  }

  const char* command = msg + 4;
  char ack[220];
  snprintf(ack, sizeof(ack), "command received:%s", command);
  mesh.send(packet->srcMac, MESH_TYPE_DATA, packet->appId, (const uint8_t*)ack, strlen(ack), 4);

  if (strcmp(command, "info") == 0 || strcmp(command, "info:long") == 0) {
    sendInfo(packet->srcMac, packet->appId);
  } else if (strcmp(command, "reboot") == 0) {
    Serial.println("[CMD] Reboot requested, restarting...");
    delay(100);
    ESP.restart();
  } else if (strcmp(command, "update") == 0) {
    otaRequested = true;
    Serial.println("[CMD] OTA requested");
  }
}

static bool connectToCoordinator() {
  meshChannel = mesh.findCoordinatorChannel(NODE_NAME);
  if (meshChannel == 0) return false;

  mesh.getCoordinatorMac(coordinatorMac);
  mesh.begin(meshChannel);
  mesh.setCoordinatorMac(coordinatorMac);
  mesh.onReceive(onMeshMessage);

  #if defined(ESP32)
    esp_wifi_get_mac(WIFI_IF_STA, myMac);
  #else
    WiFi.macAddress(myMac);
  #endif

  lastHeartbeat = millis() - HEARTBEAT_INTERVAL;
  lastTemp      = millis() - TEMP_INTERVAL;

  mesh.send(coordinatorMac, MESH_TYPE_PING, 0x00, (const uint8_t*)NODE_NAME, strlen(NODE_NAME), 4);
  mesh.send(coordinatorMac, MESH_TYPE_DATA, 0x06, (const uint8_t*)NODE_NAME, strlen(NODE_NAME), 4);
  return true;
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

#ifdef HAS_PIR
  pirSetup(onPirTriggered);
#endif

  Serial.println();
  Serial.println("========================================");
  Serial.println("       UniversalMesh  -  Sensor Node   ");
  Serial.println("========================================");
  Serial.println("  Scanning for Coordinator...");

  mesh.begin(1);
  mesh.onReceive(onMeshMessage);

  if (connectToCoordinator()) {
    Serial.printf("  Channel     : %d\n", meshChannel);
    Serial.printf("  MAC Address : %02X:%02X:%02X:%02X:%02X:%02X\n",
                  myMac[0], myMac[1], myMac[2], myMac[3], myMac[4], myMac[5]);
    Serial.println("  Coordinator : Found");
  } else {
    Serial.println("  Coordinator : Not found, retrying in loop...");
  }
  Serial.println("========================================");
}

void loop() {
  if (otaRequested) {
    static bool otaStarted = false;
    if (!otaStarted) {
      otaStarted = true;
      startOtaUpdate();
    }
    delay(20);
    return;
  }

  mesh.update();

#ifdef HAS_PIR
  pirLoop();
#endif

  if (!mesh.isCoordinatorFound()) {
    static unsigned long lastRetry = 0;
    if (millis() - lastRetry > 30000) {
      lastRetry = millis();
      Serial.println("[RETRY] Scanning for Coordinator...");
      if (connectToCoordinator()) {
        Serial.printf("[AUTO] Coordinator found on channel %d\n", meshChannel);
      }
    }
    return;
  }

  unsigned long now = millis();

  if (now - lastHeartbeat >= HEARTBEAT_INTERVAL) {
    lastHeartbeat = now;
    uint8_t hb = 0x01;
    mesh.send(coordinatorMac, MESH_TYPE_DATA, 0x05, &hb, 1, 4);
    mesh.send(coordinatorMac, MESH_TYPE_DATA, 0x06, (const uint8_t*)NODE_NAME, strlen(NODE_NAME), 4);
    Serial.printf("[TX] Heartbeat | %02X:%02X:%02X:%02X:%02X:%02X\n",
                  myMac[0], myMac[1], myMac[2], myMac[3], myMac[4], myMac[5]);
  }

  if (now - lastTemp >= TEMP_INTERVAL) {
    lastTemp = now;
    #if defined(ESP8266)
    float tempC = 18.0f + (rand() % 100) / 10.0f;
    #else
    float tempC = temperatureRead();
    #endif
    JsonDocument doc;
    doc["name"] = NODE_NAME;
    doc["temp"] = serialized(String(tempC, 1));
    String payload;
    serializeJson(doc, payload);
    if (mesh.sendToCoordinator(0x01, payload)) {
      Serial.printf("[TX] Sensor: %s\n", payload.c_str());
    } else {
      Serial.println("[TX] Send failed");
    }
  }
}
