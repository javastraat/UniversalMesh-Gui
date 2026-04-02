#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SHT31.h>
#include <ArduinoJson.h>
#include "UniversalMesh.h"
#include "ota_update.h"

#ifdef HAS_DISPLAY_SHIELD
#include <U8g2lib.h>
// WEMOS OLED shield: SSD1306 64x48, I2C 0x3C, reset GPIO0
U8G2_SSD1306_64X48_ER_F_HW_I2C u8g2(U8G2_R0, /* reset= */ 0);
#define PAGE_INTERVAL 5000
uint8_t       currentPage    = 0;
unsigned long lastPageSwitch = 0;
float         lastTempC      = NAN;
float         lastHum        = NAN;
#endif

#ifndef NODE_NAME
  #define NODE_NAME "sensor-node"
#endif
#define HEARTBEAT_INTERVAL  120000
#define SENSOR_INTERVAL      60000

UniversalMesh  mesh;
Adafruit_SHT31 sht30;

uint8_t myMac[6]          = {0};
uint8_t coordinatorMac[6] = {0};
uint8_t meshChannel       = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastSensor    = 0;
volatile bool otaRequested  = false;

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

// -------------------------------------------------------
// Display pages (64x48 WEMOS OLED)
// -------------------------------------------------------
#ifdef HAS_DISPLAY_SHIELD
void showPage(uint8_t page) {
  char buf[12];
  u8g2.clearBuffer();

  if (page == 0) {
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 9, "Temp");
    u8g2.setFont(u8g2_font_helvB18_tf);
    snprintf(buf, sizeof(buf), isnan(lastTempC) ? "--.-" : "%.1f", lastTempC);
    u8g2.drawStr(0, 36, buf);
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 47, "deg C");

  } else if (page == 1) {
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 9, "Humidity");
    u8g2.setFont(u8g2_font_helvB18_tf);
    snprintf(buf, sizeof(buf), isnan(lastHum) ? "--.-" : "%.1f", lastHum);
    u8g2.drawStr(0, 36, buf);
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 47, "%RH");

  } else {
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 9, "wemosd1");
    snprintf(buf, sizeof(buf), "%02X:%02X:%02X", myMac[3], myMac[4], myMac[5]);
    u8g2.drawStr(0, 21, buf);
    snprintf(buf, sizeof(buf), "CH: %d", meshChannel);
    u8g2.drawStr(0, 33, buf);
    u8g2.drawStr(0, 45, mesh.isCoordinatorFound() ? "COORD: OK" : "searching");
  }

  u8g2.sendBuffer();
}
#endif

// -------------------------------------------------------

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

  WiFi.macAddress(myMac);

  lastHeartbeat = millis() - HEARTBEAT_INTERVAL;
  lastSensor    = millis() - SENSOR_INTERVAL;

  mesh.send(coordinatorMac, MESH_TYPE_PING, 0x00, (const uint8_t*)NODE_NAME, strlen(NODE_NAME), 4);
  mesh.send(coordinatorMac, MESH_TYPE_DATA, 0x06, (const uint8_t*)NODE_NAME, strlen(NODE_NAME), 4);
  return true;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  sht30.begin(0x45);

#ifdef HAS_DISPLAY_SHIELD
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 16, "Universal");
  u8g2.drawStr(0, 28, "Mesh");
  u8g2.drawStr(0, 40, "SHT30 node");
  u8g2.sendBuffer();
  delay(1500);
#endif

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.println();
  Serial.println("========================================");
  Serial.println("    UniversalMesh  -  SHT30 Sensor     ");
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

#ifdef HAS_DISPLAY_SHIELD
  showPage(0);
  lastPageSwitch = millis();
#endif
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

  if (now - lastSensor >= SENSOR_INTERVAL) {
    lastSensor = now;
    float tempC = sht30.readTemperature();
    float hum   = sht30.readHumidity();
    if (!isnan(tempC) && !isnan(hum)) {
#ifdef HAS_DISPLAY_SHIELD
      lastTempC = tempC;
      lastHum   = hum;
#endif
      JsonDocument doc;
      doc["name"] = NODE_NAME;
      doc["temp"] = serialized(String(tempC, 1));
      doc["hum"]  = serialized(String(hum, 1));
      String payload;
      serializeJson(doc, payload);
      if (mesh.sendToCoordinator(0x01, payload)) {
        Serial.printf("[TX] Sensor: %s\n", payload.c_str());
      } else {
        Serial.println("[TX] Send failed");
      }
    } else {
      Serial.println("[SHT30] Read error — check wiring/address");
    }
  }

#ifdef HAS_DISPLAY_SHIELD
  if (millis() - lastPageSwitch >= PAGE_INTERVAL) {
    lastPageSwitch = millis();
    currentPage    = (currentPage + 1) % 3;
    showPage(currentPage);
  }
#endif
}
