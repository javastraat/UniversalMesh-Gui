#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <esp_wifi.h>
#include <ArduinoJson.h>
#include <Adafruit_NeoPixel.h>
#include "secrets.h"
#include "UniversalMesh.h"
#include "web/web.h"

#ifndef MESH_HOSTNAME
  #define MESH_HOSTNAME "universalmesh"
#endif
#ifndef MESH_NETWORK
  #define MESH_NETWORK "mesh"
#endif

// --- RGB LED ---
constexpr uint8_t LED_PIN  = 8;
Adafruit_NeoPixel rgbLed(1, LED_PIN, NEO_GRB + NEO_KHZ800);

struct RGB { uint8_t r, g, b; };
constexpr RGB COLOR_OFF    = {  0,   0,   0};
constexpr RGB COLOR_GREEN  = {  0, 255,   0};
constexpr RGB COLOR_RED    = {255,   0,   0};
constexpr RGB COLOR_BLUE   = {  0,   0, 255};
constexpr RGB COLOR_YELLOW = {255, 200,   0};

enum LedState { LED_CONNECTING, LED_CONNECTED, LED_NO_WIFI, LED_TX_BLINK, LED_RX_BLINK };
LedState ledState     = LED_CONNECTING;
LedState ledPrevState = LED_CONNECTING;
unsigned long ledTimer = 0;
bool ledToggle = false;
constexpr unsigned long BLINK_INTERVAL = 300;  // ms between connecting blinks
constexpr unsigned long FLASH_ON  = 250;        // ms LED on  per TX/RX blink
constexpr unsigned long FLASH_OFF = 250;         // ms LED off per TX/RX blink
constexpr uint8_t       FLASH_COUNT = 3;        // number of TX/RX blinks

int8_t ledFlashRemaining = 0;  // blinks left (counts down)

void setColor(const RGB& c, uint8_t brightness = 50) {
  uint16_t s = (uint16_t)brightness * 255 / 100;
  rgbLed.setPixelColor(0, rgbLed.Color((c.r * s) / 255, (c.g * s) / 255, (c.b * s) / 255));
  rgbLed.show();
}

void ledFlash(LedState flashState) {
  if (ledState == LED_TX_BLINK || ledState == LED_RX_BLINK) return; // don't interrupt ongoing flash
  ledPrevState = ledState;
  ledState = flashState;
  ledFlashRemaining = FLASH_COUNT;
  ledToggle = true;
  ledTimer = millis();
  setColor(flashState == LED_TX_BLINK ? COLOR_BLUE : COLOR_YELLOW);
}

void ledUpdate() {
  unsigned long now = millis();
  switch (ledState) {
    case LED_CONNECTING:
      if (now - ledTimer >= BLINK_INTERVAL) {
        ledToggle = !ledToggle;
        setColor(ledToggle ? COLOR_GREEN : COLOR_OFF);
        ledTimer = now;
      }
      break;
    case LED_CONNECTED:
    case LED_NO_WIFI:
      break; // steady — color already set on transition
    case LED_TX_BLINK:
    case LED_RX_BLINK: {
      unsigned long phase = ledToggle ? FLASH_ON : FLASH_OFF;
      if (now - ledTimer >= phase) {
        ledToggle = !ledToggle;
        if (ledToggle) {
          // starting a new on-phase = new blink
          ledFlashRemaining--;
          if (ledFlashRemaining <= 0) {
            ledState = ledPrevState;
            setColor(ledState == LED_CONNECTED ? COLOR_GREEN : COLOR_RED);
            break;
          }
          setColor(ledState == LED_TX_BLINK ? COLOR_BLUE : COLOR_YELLOW);
        } else {
          setColor(COLOR_OFF);
        }
        ledTimer = now;
      }
      break;
    }
  }
}



// --- ROUTING TABLE ---
#define MAX_NODES 20
struct KnownNode {
  uint8_t       mac[6];
  unsigned long lastSeen;
  bool          active;
  char          name[32];
};
KnownNode meshNodes[MAX_NODES];

// Function to register or update a node in the table
void updateNodeTable(uint8_t* mac) {
  // Check if we already know this node
  for (int i = 0; i < MAX_NODES; i++) {
    if (meshNodes[i].active && memcmp(meshNodes[i].mac, mac, 6) == 0) {
      meshNodes[i].lastSeen = millis(); // Update timestamp
      return;
    }
  }
  // If new, find an empty slot and add it
  for (int i = 0; i < MAX_NODES; i++) {
    if (!meshNodes[i].active) {
      memcpy(meshNodes[i].mac, mac, 6);
      meshNodes[i].lastSeen = millis();
      meshNodes[i].active = true;
      char discMsg[80];
      snprintf(discMsg, sizeof(discMsg), "[ROUTING] New node: %02X:%02X:%02X:%02X:%02X:%02X",
               mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
      Serial.println(discMsg);
      addSerialLog(discMsg);
      return;
    }
  }
}


#ifdef LILYGO_T_ETH_ELITE
// Called from onMeshMessage (ESP-NOW task) — no network I/O, just a memcpy into the queue.
// Must be called with _dataMutex held so meshNodes[] is safe to read.
static void mqttEnqueue(MeshPacket* packet) {
  if (packet->type != MESH_TYPE_DATA) return;
  if (packet->appId != 0x01 && packet->appId != 0x05 && packet->appId != 0x06) return;

  int next = (_mqttQHead + 1) % MQTT_QUEUE_SIZE;
  if (next == _mqttQTail) return; // queue full, drop

  // Resolve node identifier: name if known, compact MAC otherwise
  char nodeId[32] = {0};
  for (int i = 0; i < MAX_NODES; i++) {
    if (meshNodes[i].active && memcmp(meshNodes[i].mac, packet->srcMac, 6) == 0) {
      if (meshNodes[i].name[0]) strncpy(nodeId, meshNodes[i].name, sizeof(nodeId) - 1);
      break;
    }
  }
  if (!nodeId[0]) {
    snprintf(nodeId, sizeof(nodeId), "%02x%02x%02x%02x%02x%02x",
             packet->srcMac[0], packet->srcMac[1], packet->srcMac[2],
             packet->srcMac[3], packet->srcMac[4], packet->srcMac[5]);
  }

  MqttPub& msg = _mqttQueue[_mqttQHead];
  snprintf(msg.topic, sizeof(msg.topic), "%s/%s/nodes/%s/%02x",
           MESH_NETWORK, MESH_HOSTNAME, nodeId, packet->appId);
  if (packet->appId == 0x05) {
    strcpy(msg.payload, "1");
  } else {
    uint8_t len = packet->payloadLen < 200 ? packet->payloadLen : 200;
    memcpy(msg.payload, packet->payload, len);
    msg.payload[len] = '\0';
  }
  _mqttQHead = next;
}

// Called from loop() — safe context for network I/O.
static void mqttDrain() {
  lockMeshData();
  int head = _mqttQHead;
  unlockMeshData();
  while (_mqttQTail != head) {
    if (!_mqtt.publish(_mqttQueue[_mqttQTail].topic, _mqttQueue[_mqttQTail].payload)) break;
    Serial.printf("[MQTT] %s → %s\n", _mqttQueue[_mqttQTail].topic, _mqttQueue[_mqttQTail].payload);
    _mqttQTail = (_mqttQTail + 1) % MQTT_QUEUE_SIZE;
  }
}

// Publish coordinator runtime telemetry so MQTT consumers can monitor the bridge itself,
// not only data forwarded from mesh nodes.
static void mqttPublishCoordinatorData(bool force = false) {
  if (!_mqtt.connected()) return;
  unsigned long now = millis();
  if (!force && (now - _mqttLastCoordPub < MQTT_COORD_PUB_MS)) return;

  char topic[96];
  char payload[280];
  bool ethOk = isEthConnected();
  bool wifiOk = WiFi.status() == WL_CONNECTED;
  String ip = ethOk ? ETH.localIP().toString() : WiFi.localIP().toString();
  int rssi = wifiOk ? WiFi.RSSI() : 0;
  String ts = isNtpSynced() ? getNtpTimeStr() : String("unsynced");
  ts.replace(" ", "T");

  snprintf(topic, sizeof(topic), "%s/%s/coordinator/status", MESH_NETWORK, MESH_HOSTNAME);
  snprintf(payload, sizeof(payload),
           "{\"name\":\"%s\",\"ts\":\"%s\",\"uptime_s\":%lu,\"heap\":%u,\"mesh_ch\":%u,\"wifi_connected\":%s,\"eth_connected\":%s,\"rssi\":%d,\"ip\":\"%s\"}",
           MESH_HOSTNAME,
           ts.c_str(),
           now / 1000UL,
           ESP.getFreeHeap(),
           getMeshChannel(),
           wifiOk ? "true" : "false",
           ethOk ? "true" : "false",
           rssi,
           ip.c_str());

  if (_mqtt.publish(topic, payload, true)) {
    _mqttLastCoordPub = now;
    Serial.printf("[MQTT] %s -> %s\n", topic, payload);
  }
}

static void mqttConnect() {
  if (!isEthConnected()) return;
  if (_mqtt.connected()) return;
  unsigned long now = millis();
  if (now - _mqttLastAttempt < MQTT_RECONNECT_MS) return;
  _mqttLastAttempt = now;
  Serial.printf("[MQTT] Connecting to %s:%d...", MQTT_BROKER, MQTT_PORT);
  String clientId = "mesh-" + String(MESH_HOSTNAME);
  char availabilityTopic[96];
  snprintf(availabilityTopic, sizeof(availabilityTopic), "%s/%s/coordinator/availability", MESH_NETWORK, MESH_HOSTNAME);
  const char* user = strlen(MQTT_USER) ? MQTT_USER : nullptr;
  const char* pass = strlen(MQTT_PASS) ? MQTT_PASS : nullptr;
  if (_mqtt.connect(clientId.c_str(), user, pass, availabilityTopic, 1, true, "offline")) {
    Serial.println(" OK");
    _mqtt.publish(availabilityTopic, "online", true);
    mqttPublishCoordinatorData(true);
  } else {
    Serial.printf(" failed rc=%d\n", _mqtt.state());
  }
}
#endif // LILYGO_T_ETH_ELITE

AsyncWebServer server(80);
UniversalMesh mesh;

// --- RECEIVE CALLBACK ---
// Triggers when the mesh library catches a packet meant for us (or broadcast)
void onMeshMessage(MeshPacket* packet, uint8_t* senderMac) {
  ledFlash(LED_RX_BLINK);

  // 1. Immediately log the node in the routing table (mutex protects meshNodes[] and _log[])
  lockMeshData();
  updateNodeTable(packet->srcMac);
  logPacket(packet->type, senderMac, packet->srcMac, packet->appId, packet->payload, packet->payloadLen);
  unlockMeshData();

  // 2. Process the packet types
  if (packet->type == MESH_TYPE_PING && packet->payloadLen > 0) {
    // Node announced itself with its name in the PING payload (new library discovery)
    char name[201] = {0};
    uint8_t len = packet->payloadLen < 200 ? packet->payloadLen : 200;
    memcpy(name, packet->payload, len);
    lockMeshData();
    for (int i = 0; i < MAX_NODES; i++) {
      if (meshNodes[i].active && memcmp(meshNodes[i].mac, packet->srcMac, 6) == 0) {
        strncpy(meshNodes[i].name, name, sizeof(meshNodes[i].name) - 1);
        meshNodes[i].name[sizeof(meshNodes[i].name) - 1] = '\0';
        break;
      }
    }
    unlockMeshData();
    char logMsg[80];
    snprintf(logMsg, sizeof(logMsg), "[DISCOVERY] Node joined: %s", name);
    Serial.println(logMsg);
    addSerialLog(logMsg);
  }
  else if (packet->type == MESH_TYPE_PONG) {
    char logMsg[80];
    snprintf(logMsg, sizeof(logMsg), "[DISCOVERY] PONG from %02X:%02X:%02X:%02X:%02X:%02X",
             senderMac[0], senderMac[1], senderMac[2],
             senderMac[3], senderMac[4], senderMac[5]);
    Serial.println(logMsg);
    addSerialLog(logMsg);
  }
  else if (packet->type == MESH_TYPE_DATA) {
    char payloadStr[201] = {0};
    uint8_t len = packet->payloadLen < 200 ? packet->payloadLen : 200;
    memcpy(payloadStr, packet->payload, len);

    if (packet->appId == 0x06) {
      // Node announce — store name against MAC
      lockMeshData();
      for (int i = 0; i < MAX_NODES; i++) {
        if (meshNodes[i].active && memcmp(meshNodes[i].mac, packet->srcMac, 6) == 0) {
          strncpy(meshNodes[i].name, payloadStr, sizeof(meshNodes[i].name) - 1);
          meshNodes[i].name[sizeof(meshNodes[i].name) - 1] = '\0';
          break;
        }
      }
      unlockMeshData();
    }

    char logMsg[128];
    snprintf(logMsg, sizeof(logMsg), "[DATA] src=%02X:%02X:%02X:%02X:%02X:%02X appId=0x%02X payload=%s",
             senderMac[0], senderMac[1], senderMac[2],
             senderMac[3], senderMac[4], senderMac[5],
             packet->appId, payloadStr);
    Serial.println(logMsg);
    addSerialLog(logMsg);
  }
}

void setup() {
  Serial.begin(115200);
  
  // ESP32-C6 Hardware USB CDC Wait
  uint32_t t = millis();
  while (!Serial && (millis() - t) < 5000) { delay(10); }

  rgbLed.begin();
  rgbLed.show();
  ledTimer = millis();

  Serial.println("\n=== MESH MASTER BRIDGE INITIALIZING ===");

  WiFi.setHostname(MESH_HOSTNAME);
  WiFi.mode(WIFI_STA);

  // --- WiFi ---
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("[WIFI] Connecting to %s\n", WIFI_SSID);

  constexpr unsigned long WIFI_TIMEOUT_MS = 15000;
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - wifiStart) < WIFI_TIMEOUT_MS) {
    setColor(COLOR_GREEN); delay(300);
    setColor(COLOR_OFF);   delay(300);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    esp_wifi_set_ps(WIFI_PS_NONE);
    ledState = LED_CONNECTED;
    setColor(COLOR_GREEN);
    Serial.printf("\n[WIFI] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("[WIFI] Channel: %d\n", WiFi.channel());
  } else {
    ledState = LED_NO_WIFI;
    setColor(COLOR_RED);
    Serial.println("\n[WIFI] Connection failed! Running offline.");
  }

  uint8_t chan = (WiFi.status() == WL_CONNECTED) ? WiFi.channel() : 6;

  // 2. Initialize Universal Mesh on the Router's Channel
  if (mesh.begin(chan, MESH_COORDINATOR)) {
    Serial.println("[SYSTEM] Universal Mesh Online.");
    mesh.onReceive(onMeshMessage);
    char meshMsg[64];
    snprintf(meshMsg, sizeof(meshMsg), "[MESH] Online on channel %d", chan);
    addSerialLog(meshMsg);
  } else {
    Serial.println("[ERROR] Mesh Initialization Failed!");
    addSerialLog("[ERROR] Mesh init failed!");
  }

  // 3. Setup REST API Endpoint for Injecting Packets
  server.on("/api/tx", HTTP_POST,
    [](AsyncWebServerRequest *request) {},
    nullptr,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      JsonDocument doc;
      deserializeJson(doc, data, len);

      uint8_t destMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
      const char* macStr = doc["dest"];
      if (macStr) {
        int temp[6];
        if (sscanf(macStr, "%x:%x:%x:%x:%x:%x", &temp[0], &temp[1], &temp[2], &temp[3], &temp[4], &temp[5]) == 6) {
          for (int i = 0; i < 6; i++) destMac[i] = (uint8_t)temp[i];
        }
      }

      uint8_t ttl = doc["ttl"] | 4;
      uint8_t appId = doc["appId"] | 0x00;
      String payloadHex = doc["payload"] | "";

      uint8_t payloadBytes[200] = {0};
      uint8_t payloadLen = payloadHex.length() / 2;
      if (payloadLen > 200) payloadLen = 200;

      for (int i = 0; i < payloadLen; i++) {
        String byteString = payloadHex.substring(i * 2, i * 2 + 2);
        payloadBytes[i] = (uint8_t) strtol(byteString.c_str(), NULL, 16);
      }

      mesh.send(destMac, MESH_TYPE_DATA, appId, payloadBytes, payloadLen, ttl);
      ledFlash(LED_TX_BLINK);
      request->send(200, "application/json", "{\"status\":\"data_sent\"}");
    }
  );

  // 2. Discovery Endpoint
  server.on("/api/discover", HTTP_GET, [](AsyncWebServerRequest *request) {
    uint8_t broadcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    mesh.send(broadcastMac, MESH_TYPE_PING, 0x00, nullptr, 0, 4);
    request->send(200, "application/json", "{\"status\":\"discovery_initiated\"}");
  });

  server.on("/api/nodes", HTTP_GET, [](AsyncWebServerRequest *request) {
    // Snapshot under lock to prevent PSRAM cache coherency race with onMeshMessage
    KnownNode snap[MAX_NODES];
    lockMeshData();
    memcpy(snap, meshNodes, sizeof(snap));
    unlockMeshData();

    JsonDocument doc;
    JsonArray nodesArray = doc["nodes"].to<JsonArray>();
    unsigned long now = millis();
    for (int i = 0; i < MAX_NODES; i++) {
      if (snap[i].active) {
        JsonObject nodeObj = nodesArray.add<JsonObject>();
        char macStr[18];
        snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
                 snap[i].mac[0], snap[i].mac[1], snap[i].mac[2],
                 snap[i].mac[3], snap[i].mac[4], snap[i].mac[5]);
        nodeObj["mac"] = macStr;
        nodeObj["last_seen_seconds_ago"] = (now - snap[i].lastSeen) / 1000;
        if (snap[i].name[0]) nodeObj["name"] = snap[i].name;
      }
    }

    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
  });

  initWebDashboard(server);
  server.begin();
  Serial.println("[SYSTEM] REST API Gateway Ready.");
  addSerialLog("[SYSTEM] REST API ready - waiting for nodes...");
}

void loop() {
  // Let the mesh library do its background work
  mesh.update();

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "SEND") {
      Serial.println("[INJECT] Firing test packet into mesh via Serial...");
      uint8_t destMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
      const char* payload = "Serial Test Pkt";
      mesh.send(destMac, MESH_TYPE_DATA, 0x01, (const uint8_t*)payload, strlen(payload), 4);
      ledFlash(LED_TX_BLINK);
    }
  }

  ledUpdate();
}