#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <esp_wifi.h>
#include <ArduinoJson.h>
#include "secrets.h"
#include "UniversalMesh.h"
#include "web/web.h"

#ifndef MESH_HOSTNAME
  #define MESH_HOSTNAME "universalmesh"
#endif
#ifndef MESH_NETWORK
  #define MESH_NETWORK "mesh"
#endif

#include <ETH.h>
#include <Preferences.h>
#include <esp_now.h>
#include <PubSubClient.h>
extern void setupETH();
extern void loopETH();
extern bool   isEthConnected();
extern bool   isEthLinkUp();
extern bool   isOtaInProgress();
extern void   setOtaBeginCallback(void(*cb)());
extern void   setOtaEndCallback(void(*cb)());
extern String getEthSubnet();
extern String getEthDNS();
extern bool   isNtpSynced();
extern String getNtpTimeStr();

// --- LoRa bridge (lora.cpp) — compiled in only when -D HAS_LORA is set ---
#ifdef HAS_LORA
extern void setupLoRa();
extern void loopLoRa();
extern void loraStandby();
extern void loraResume();
extern bool loraSendPacket(MeshPacket* pkt, float freqMHz = 0.0f);
extern bool loraBridgePacket(MeshPacket* pkt);
extern void loraOnReceive(MeshReceiveCallback cb);
#endif

// --- MQTT client ---
static WiFiClient    _mqttNetClient;
static PubSubClient  _mqtt(_mqttNetClient);
static unsigned long _mqttLastAttempt = 0;
constexpr unsigned long MQTT_RECONNECT_MS = 5000;
static unsigned long _mqttLastCoordPub = 0;
constexpr unsigned long MQTT_COORD_PUB_MS = 60000;

#ifdef HAS_LORA
// MQTT topic HA publishes to in order to trigger a LoRa TX:
//   mesh/universalmesh/lora/tx
// Payload: plain text  OR  JSON {"payload":"...","dest":"FF:FF:FF:FF:FF:FF","appId":1}
#define MQTT_LORA_TX_TOPIC MESH_NETWORK "/" MESH_HOSTNAME "/lora/tx"
static void mqttLoRaTxCallback(const char* topic, byte* payload, unsigned int length) {
  if (length == 0 || length > 190) return;

  char buf[191];
  memcpy(buf, payload, length);
  buf[length] = '\0';

  MeshPacket pkt = {};
  pkt.type   = MESH_TYPE_DATA;
  pkt.ttl    = 3;
  pkt.msgId  = (uint32_t)(millis() ^ (esp_random() & 0xFFFF));
  memset(pkt.destMac, 0xFF, 6);            // broadcast by default
  esp_wifi_get_mac(WIFI_IF_STA, pkt.srcMac);
  pkt.appId  = 0x01;

  // Try JSON first: {"payload":"...","dest":"AA:BB:CC:DD:EE:FF","appId":1}
  JsonDocument doc;
  if (deserializeJson(doc, buf) == DeserializationError::Ok && doc["payload"].is<const char*>()) {
    const char* text = doc["payload"] | "";
    pkt.payloadLen = (uint8_t)strnlen(text, 190);
    memcpy(pkt.payload, text, pkt.payloadLen);
    pkt.appId = doc["appId"] | 0x01;
    const char* destStr = doc["dest"] | "";
    if (strlen(destStr) == 17) {
      int t[6];
      if (sscanf(destStr, "%x:%x:%x:%x:%x:%x", &t[0],&t[1],&t[2],&t[3],&t[4],&t[5]) == 6)
        for (int i = 0; i < 6; i++) pkt.destMac[i] = (uint8_t)t[i];
    }
  } else {
    // Plain text payload
    pkt.payloadLen = (uint8_t)length;
    memcpy(pkt.payload, buf, pkt.payloadLen);
  }

  bool ok = loraSendPacket(&pkt);
  Serial.printf("[MQTT→LoRa] %s — payload='%.*s' queued=%s\n",
                topic, pkt.payloadLen, (char*)pkt.payload, ok ? "yes" : "queue_full");
  char logMsg[80];
  snprintf(logMsg, sizeof(logMsg), "[MQTT→LoRa] '%.*s' %s",
           pkt.payloadLen, (char*)pkt.payload, ok ? "queued" : "queue_full");
  addSerialLog(logMsg);
}
#endif

// Publish queue — enqueued from ESP-NOW callback task, drained in loop()
#define MQTT_QUEUE_SIZE 8
struct MqttPub { char topic[96]; char payload[201]; };
static MqttPub _mqttQueue[MQTT_QUEUE_SIZE];
static int     _mqttQHead = 0;
static int     _mqttQTail = 0;

static Preferences _prefs;
static uint8_t _meshChannel = 1;  // default

static uint8_t loadMeshChannel() {
  _prefs.begin("mesh", true);
  uint8_t ch = _prefs.getUChar("channel", 1);
  _prefs.end();
  return (ch >= 1 && ch <= 13) ? ch : 1;
}

uint8_t getMeshChannel() { return _meshChannel; }

void setMeshChannel(uint8_t ch) {
  if (ch < 1 || ch > 13) return;
  _meshChannel = ch;
  // Persist
  _prefs.begin("mesh", false);
  _prefs.putUChar("channel", ch);
  _prefs.end();
  // Apply to radio
  esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
  // Update ESP-NOW broadcast peer
  static const uint8_t broadcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, broadcast, 6);
  peer.channel = ch;
  esp_now_mod_peer(&peer);
  Serial.printf("[MESH] Channel changed to %d\n", ch);
}

// --- ROUTING TABLE ---
#define MAX_NODES 20

enum Transport : uint8_t { TRANSPORT_ESPNOW, TRANSPORT_LORA_868, TRANSPORT_LORA_2400 };

struct KnownNode {
  uint8_t       mac[6];
  unsigned long lastSeen;
  bool          active;
  char          name[32];
  Transport     transport;   // which radio this node was last heard on
  float         loraRssi;    // last LoRa RSSI in dBm (0 for ESP-NOW nodes)
  float         loraSNR;     // last LoRa SNR in dB   (0 for ESP-NOW nodes)
};
KnownNode meshNodes[MAX_NODES];

// Register or refresh a node seen via ESP-NOW
void updateNodeTable(uint8_t* mac) {
  for (int i = 0; i < MAX_NODES; i++) {
    if (meshNodes[i].active && memcmp(meshNodes[i].mac, mac, 6) == 0) {
      meshNodes[i].lastSeen  = millis();
      meshNodes[i].transport = TRANSPORT_ESPNOW;
      return;
    }
  }
  for (int i = 0; i < MAX_NODES; i++) {
    if (!meshNodes[i].active) {
      memcpy(meshNodes[i].mac, mac, 6);
      meshNodes[i].lastSeen  = millis();
      meshNodes[i].active    = true;
      meshNodes[i].transport = TRANSPORT_ESPNOW;
      meshNodes[i].loraRssi  = 0.0f;
      meshNodes[i].loraSNR   = 0.0f;
      char discMsg[80];
      snprintf(discMsg, sizeof(discMsg), "[ROUTING] New node (ESP-NOW): %02X:%02X:%02X:%02X:%02X:%02X",
               mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
      Serial.println(discMsg);
      addSerialLog(discMsg);
      return;
    }
  }
}

// Called from lora.cpp after a packet arrives — updates transport + signal stats.
// Must be called from loop() context (not ISR); lora.cpp guarantees this.
void updateNodeLoRa(uint8_t* mac, float rssi, float snr) {
  lockMeshData();
  for (int i = 0; i < MAX_NODES; i++) {
    if (meshNodes[i].active && memcmp(meshNodes[i].mac, mac, 6) == 0) {
      meshNodes[i].transport = TRANSPORT_LORA_868;
      meshNodes[i].loraRssi  = rssi;
      meshNodes[i].loraSNR   = snr;
      meshNodes[i].lastSeen  = millis();
      unlockMeshData();
      return;
    }
  }
  // Node not yet in table — add it as a LoRa node
  for (int i = 0; i < MAX_NODES; i++) {
    if (!meshNodes[i].active) {
      memcpy(meshNodes[i].mac, mac, 6);
      meshNodes[i].lastSeen  = millis();
      meshNodes[i].active    = true;
      meshNodes[i].transport = TRANSPORT_LORA_868;
      meshNodes[i].loraRssi  = rssi;
      meshNodes[i].loraSNR   = snr;
      meshNodes[i].name[0]   = '\0';
      char discMsg[80];
      snprintf(discMsg, sizeof(discMsg), "[ROUTING] New node (LoRa): %02X:%02X:%02X:%02X:%02X:%02X",
               mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
      Serial.println(discMsg);
      addSerialLog(discMsg);
      break;
    }
  }
  unlockMeshData();
}


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
#ifdef HAS_LORA
    _mqtt.subscribe(MQTT_LORA_TX_TOPIC);
    Serial.printf("[MQTT] Subscribed to %s\n", MQTT_LORA_TX_TOPIC);
#endif
  } else {
    Serial.printf(" failed rc=%d\n", _mqtt.state());
  }
}

AsyncWebServer server(80);
UniversalMesh mesh;

// --- RECEIVE CALLBACK (shared logic) ---
// fromLora=true  → packet arrived via LoRa,  do NOT re-forward to LoRa (loop prevention)
// fromLora=false → packet arrived via ESP-NOW, forward a copy to LoRa so pager sees it
static void handleIncomingPacket(MeshPacket* packet, uint8_t* senderMac, bool fromLora) {
  lockMeshData();
  if (!fromLora) updateNodeTable(packet->srcMac);
  logPacket(packet->type, senderMac, packet->srcMac, packet->appId, packet->payload, packet->payloadLen);

  // 0x06 announce payload is the plain node name — store it in the routing table
  if (packet->appId == 0x06 && packet->payloadLen > 0) {
    for (int i = 0; i < MAX_NODES; i++) {
      if (meshNodes[i].active && memcmp(meshNodes[i].mac, packet->srcMac, 6) == 0) {
        uint8_t nlen = packet->payloadLen < 31 ? packet->payloadLen : 31;
        memcpy(meshNodes[i].name, packet->payload, nlen);
        meshNodes[i].name[nlen] = '\0';
        break;
      }
    }
  }
  unlockMeshData();

  char payloadStr[201] = {0};
  uint8_t len = packet->payloadLen < 200 ? packet->payloadLen : 200;
  memcpy(payloadStr, packet->payload, len);
  char logMsg[256];
  snprintf(logMsg, sizeof(logMsg), "[%s RX] type=0x%02X src=%02X:%02X:%02X:%02X:%02X:%02X appId=0x%02X payload=%s",
           fromLora ? "LORA" : "ESPNOW",
           packet->type,
           senderMac[0], senderMac[1], senderMac[2],
           senderMac[3], senderMac[4], senderMac[5],
           packet->appId, payloadStr);
  Serial.println(logMsg);
  addSerialLog(logMsg);

  lockMeshData();
  mqttEnqueue(packet);
  unlockMeshData();

#ifdef HAS_LORA
  // Bridge: forward ESP-NOW sensor data (0x01) to LoRa.
  // loraBridgePacket filters non-0x01 packets and deduplicates by srcMac in the
  // TX queue, so chatty nodes don't fill the queue with stale readings.
  if (!fromLora) {
    loraBridgePacket(packet);
  }
#endif
}

// Called by the mesh library for ESP-NOW packets
void onMeshMessage(MeshPacket* packet, uint8_t* senderMac) {
  handleIncomingPacket(packet, senderMac, false);
}

// Called by lora.cpp for LoRa-received packets
static void onLoRaMessage(MeshPacket* packet, uint8_t* senderMac) {
  handleIncomingPacket(packet, senderMac, true);
}

void setup() {
  Serial.begin(115200);
  
  // ESP32-C6 Hardware USB CDC Wait
  uint32_t t = millis();
  while (!Serial && (millis() - t) < 5000) { delay(10); }

  Serial.println("\n=== MESH MASTER BRIDGE INITIALIZING ===");

  WiFi.setHostname(MESH_HOSTNAME);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(false);  // keep radio on for ESP-NOW but don't join any AP

  // --- ETH Elite: Ethernet first, WiFi fallback ---
  setupETH();

  // Phase 1: wait up to 5s for a cable (link up)
  Serial.print("[ETH] Waiting for cable");
  unsigned long ethStart = millis();
  while (!isEthLinkUp() && (millis() - ethStart) < 5000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  // Phase 2: cable detected — wait up to 60s for DHCP
  if (isEthLinkUp()) {
    Serial.print("[ETH] Cable detected, waiting for IP");
    ethStart = millis();
    while (!isEthConnected() && (millis() - ethStart) < 60000) {
      delay(500);
      Serial.print(".");
    }
    Serial.println();
  }

  uint8_t chan;
  if (isEthConnected()) {
    Serial.printf("[ETH] Online! IP: %s\n", ETH.localIP().toString().c_str());
    esp_wifi_set_ps(WIFI_PS_NONE);
    _meshChannel = loadMeshChannel();
    chan = _meshChannel;
    Serial.printf("[MESH] Using channel %d (from NVS)\n", chan);
    esp_wifi_set_channel(chan, WIFI_SECOND_CHAN_NONE);
  } else {
    Serial.println("[ETH] No link — falling back to WiFi...");
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    constexpr unsigned long WIFI_TIMEOUT_MS = 15000;
    unsigned long wifiStart = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - wifiStart) < WIFI_TIMEOUT_MS) {
      delay(300);
      Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
      esp_wifi_set_ps(WIFI_PS_NONE);
      Serial.printf("[WIFI] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
      Serial.println("[WIFI] Connection failed! Running offline.");
    }
    chan = (WiFi.status() == WL_CONNECTED) ? WiFi.channel() : 6;
  }

  // --- MQTT ---
  _mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  _mqtt.setSocketTimeout(1);  // 1s max — fast enough to recover, short enough not to stall loop()
#ifdef HAS_LORA
  _mqtt.setCallback(mqttLoRaTxCallback);
#endif
  mqttConnect();

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

#ifdef HAS_LORA
  // 2b. Initialize LoRa radio and wire its receive path into onLoRaMessage
  //     (separate from ESP-NOW callback to prevent bridging loops)
  loraOnReceive(onLoRaMessage);
  setupLoRa();
  addSerialLog("[LORA] LR1121 bridge active (868 MHz SF12)");
  // Put LoRa radio in standby when OTA starts — stops ISR storms during flash write
  setOtaBeginCallback(loraStandby);
  setOtaEndCallback(loraResume);
#endif

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
#ifdef HAS_LORA
      // Also broadcast over LoRa so LoRa-only nodes receive coordinator messages
      MeshPacket loraPkt = {};
      loraPkt.type       = MESH_TYPE_DATA;
      loraPkt.ttl        = ttl;
      loraPkt.msgId      = (uint32_t)(millis() ^ (esp_random() & 0xFFFF));
      memcpy(loraPkt.destMac, destMac, 6);
      esp_wifi_get_mac(WIFI_IF_STA, loraPkt.srcMac);
      loraPkt.appId      = appId;
      loraPkt.payloadLen = payloadLen;
      memcpy(loraPkt.payload, payloadBytes, payloadLen);
      loraSendPacket(&loraPkt);
#endif
      request->send(200, "application/json", "{\"status\":\"data_sent\"}");
    }
  );

  // 2. Discovery Endpoint
  server.on("/api/discover", HTTP_GET, [](AsyncWebServerRequest *request) {
    uint8_t broadcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    mesh.send(broadcastMac, MESH_TYPE_PING, 0x00, nullptr, 0, 4);
#ifdef HAS_LORA
    // Ping over LoRa too so LoRa-only nodes can announce themselves
    MeshPacket pingPkt = {};
    pingPkt.type   = MESH_TYPE_PING;
    pingPkt.ttl    = 4;
    pingPkt.msgId  = (uint32_t)(millis() ^ (esp_random() & 0xFFFF));
    memset(pingPkt.destMac, 0xFF, 6);
    esp_wifi_get_mac(WIFI_IF_STA, pingPkt.srcMac);
    pingPkt.appId      = 0x00;
    pingPkt.payloadLen = 0;
    loraSendPacket(&pingPkt);
#endif
    request->send(200, "application/json", "{\"status\":\"discovery_initiated\"}");
  });

  // Helper lambda — builds the nodes JSON array (shared by /api/nodes and /api/fast)
  auto buildNodesJson = []() -> String {
    KnownNode snap[MAX_NODES];
    lockMeshData();
    memcpy(snap, meshNodes, sizeof(snap));
    unlockMeshData();
    JsonDocument doc;
    JsonArray arr = doc["nodes"].to<JsonArray>();
    unsigned long now = millis();
    for (int i = 0; i < MAX_NODES; i++) {
      if (!snap[i].active) continue;
      JsonObject o = arr.add<JsonObject>();
      char macStr[18];
      snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
               snap[i].mac[0], snap[i].mac[1], snap[i].mac[2],
               snap[i].mac[3], snap[i].mac[4], snap[i].mac[5]);
      o["mac"] = macStr;
      o["last_seen_seconds_ago"] = (now - snap[i].lastSeen) / 1000;
      if (snap[i].name[0]) o["name"] = snap[i].name;
      o["transport"] = snap[i].transport == TRANSPORT_LORA_868  ? "lora_868"  :
                       snap[i].transport == TRANSPORT_LORA_2400 ? "lora_2400" : "espnow";
      if (snap[i].transport == TRANSPORT_LORA_868 || snap[i].transport == TRANSPORT_LORA_2400) {
        o["rssi_dbm"] = serialized(String(snap[i].loraRssi, 1));
        o["snr_db"]   = serialized(String(snap[i].loraSNR,  1));
      }
    }
    String out; serializeJson(doc, out); return out;
  };

  server.on("/api/nodes", HTTP_GET, [buildNodesJson](AsyncWebServerRequest *request) {
    request->send(200, "application/json", buildNodesJson());
  });

  // /api/fast — nodes + log in one round trip, halving HTTP overhead vs two parallel requests
  server.on("/api/fast", HTTP_GET, [buildNodesJson](AsyncWebServerRequest *request) {
    String nodesJson = buildNodesJson();  // {"nodes":[...]}
    String logJson   = getLogJson();      // {"packets":[...]}
    // Build {"nodes":[...],"log":[...]}
    // - strip closing '}' from nodesJson
    // - extract the array '[...]' from logJson (everything from first '[')
    nodesJson.remove(nodesJson.length() - 1);
    int logStart = logJson.indexOf('[');
    int logEnd   = logJson.lastIndexOf(']');
    String logArray = (logStart >= 0 && logEnd > logStart) ? logJson.substring(logStart, logEnd + 1) : "[]";
    String resp = nodesJson + ",\"log\":" + logArray + "}";
    request->send(200, "application/json", resp);
  });

#ifdef HAS_LORA
  // LoRa direct TX — broadcasts a text message over LoRa as a MeshPacket
  server.on("/api/lora/tx", HTTP_POST,
    [](AsyncWebServerRequest *request) {},
    nullptr,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      JsonDocument doc;
      deserializeJson(doc, data, len);
      const char* text = doc["payload"] | "";
      float freqMHz    = doc["freq"]    | 0.0f;
      uint8_t textLen  = (uint8_t)strnlen(text, 190);

      MeshPacket pkt = {};
      pkt.type       = MESH_TYPE_DATA;
      pkt.ttl        = 3;
      pkt.msgId      = (uint32_t)(millis() ^ (esp_random() & 0xFFFF));
      memset(pkt.destMac, 0xFF, 6);
      esp_wifi_get_mac(WIFI_IF_STA, pkt.srcMac);
      pkt.appId      = 0x01;
      pkt.payloadLen = textLen;
      memcpy(pkt.payload, text, textLen);

      bool ok = loraSendPacket(&pkt, freqMHz);
      char resp[64];
      snprintf(resp, sizeof(resp), "{\"status\":\"%s\",\"freq\":%.3f}",
               ok ? "queued" : "queue_full", freqMHz > 0 ? freqMHz : (float)868.0);
      request->send(200, "application/json", resp);
    }
  );
#endif

  initWebDashboard(server);
  server.begin();
  Serial.println("[SYSTEM] REST API Gateway Ready.");
  addSerialLog("[SYSTEM] REST API ready - waiting for nodes...");
}

void loop() {
  loopETH();  // always — this is where ArduinoOTA.handle() lives

  // During OTA: yield entirely to the OTA handler — no SPI/MQTT/mesh competition
  if (isOtaInProgress()) return;

#ifdef HAS_LORA
  loopLoRa();
#endif
  mqttConnect();
  _mqtt.loop();
  mqttDrain();
  mqttPublishCoordinatorData();
  // Let the mesh library do its background work
  mesh.update();

  // Temporary USB Serial Bypass for testing
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "SEND") {
      Serial.println("[INJECT] Firing test packet into mesh via Serial...");
      uint8_t destMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
      const char* payload = "Serial Test Pkt";
      mesh.send(destMac, MESH_TYPE_DATA, 0x01, (const uint8_t*)payload, strlen(payload), 4);
    }
  }
}