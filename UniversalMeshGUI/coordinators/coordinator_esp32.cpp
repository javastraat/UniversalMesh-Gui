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


AsyncWebServer server(80);
UniversalMesh mesh;

// --- RECEIVE CALLBACK ---
// Triggers when the mesh library catches a packet meant for us (or broadcast)
void onMeshMessage(MeshPacket* packet, uint8_t* senderMac) {
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

  Serial.println("\n=== MESH MASTER BRIDGE INITIALIZING ===");

  WiFi.setHostname(MESH_HOSTNAME);
  WiFi.mode(WIFI_STA);

  // --- WiFi ---
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("[WIFI] Connecting to %s\n", WIFI_SSID);

  constexpr unsigned long WIFI_TIMEOUT_MS = 15000;
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - wifiStart) < WIFI_TIMEOUT_MS) {
    delay(300);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    esp_wifi_set_ps(WIFI_PS_NONE);
    Serial.printf("\n[WIFI] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("[WIFI] Channel: %d\n", WiFi.channel());
  } else {
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
    }
  }
}