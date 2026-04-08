#include <Arduino.h>
#include <ArduinoJson.h>
#if defined(ESP32)
  #include <esp_wifi.h>
#endif
#ifdef HAS_RGB_LED
  #include <Adafruit_NeoPixel.h>
#endif
#include "UniversalMesh.h"
#include "ota_update.h"

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

#ifdef HAS_RGB_LED
constexpr uint8_t LED_PIN = 8;
Adafruit_NeoPixel rgbLed(1, LED_PIN, NEO_GRB + NEO_KHZ800);

struct RGB {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

constexpr RGB COLOR_OFF   = {0, 0, 0};
constexpr RGB COLOR_GREEN = {0, 255, 0};
constexpr RGB COLOR_RED   = {255, 0, 0};
constexpr RGB COLOR_BLUE  = {0, 0, 255};
constexpr RGB COLOR_AMBER = {255, 120, 0};

enum LedState { LED_CONNECTING, LED_CONNECTED, LED_NO_COORDINATOR, LED_TX_BLINK, LED_RX_BLINK };
LedState ledState = LED_CONNECTING;
LedState ledPrevState = LED_CONNECTING;
unsigned long ledTimer = 0;
bool ledToggle = false;
int8_t ledFlashRemaining = 0;

constexpr unsigned long BLINK_INTERVAL = 300;
constexpr unsigned long FLASH_ON = 120;
constexpr unsigned long FLASH_OFF = 120;
constexpr uint8_t FLASH_COUNT = 2;

void setColor(const RGB& c, uint8_t brightness = 10) {
  uint16_t scale = (uint16_t)brightness * 255 / 100;
  rgbLed.setPixelColor(0, rgbLed.Color((c.r * scale) / 255, (c.g * scale) / 255, (c.b * scale) / 255));
  rgbLed.show();
}

void setSteadyState(LedState steadyState) {
  ledState = steadyState;
  ledPrevState = steadyState;
  if (steadyState == LED_CONNECTED) {
    setColor(COLOR_GREEN);
  } else if (steadyState == LED_NO_COORDINATOR) {
    setColor(COLOR_RED);
  } else {
    setColor(COLOR_OFF);
  }
}

void ledFlash(LedState flashState) {
  if (ledState == LED_TX_BLINK || ledState == LED_RX_BLINK) return;
  ledPrevState = ledState;
  ledState = flashState;
  ledFlashRemaining = FLASH_COUNT;
  ledToggle = true;
  ledTimer = millis();
  setColor(flashState == LED_TX_BLINK ? COLOR_BLUE : COLOR_AMBER);
}

void ledUpdate() {
  unsigned long now = millis();
  if (ledState == LED_CONNECTING) {
    if (now - ledTimer >= BLINK_INTERVAL) {
      ledToggle = !ledToggle;
      setColor(ledToggle ? COLOR_BLUE : COLOR_OFF);
      ledTimer = now;
    }
    return;
  }

  if (ledState == LED_TX_BLINK || ledState == LED_RX_BLINK) {
    unsigned long phase = ledToggle ? FLASH_ON : FLASH_OFF;
    if (now - ledTimer >= phase) {
      ledToggle = !ledToggle;
      if (ledToggle) {
        ledFlashRemaining--;
        if (ledFlashRemaining <= 0) {
          setSteadyState(ledPrevState);
          return;
        }
        setColor(ledState == LED_TX_BLINK ? COLOR_BLUE : COLOR_AMBER);
      } else {
        setColor(COLOR_OFF);
      }
      ledTimer = now;
    }
  }
}
#endif

// Send all node info as a single JSON (new 200-byte payload fits everything)
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

void onMeshMessage(MeshPacket* packet, uint8_t* senderMac) {
#ifdef HAS_RGB_LED
  ledFlash(LED_RX_BLINK);
#endif

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

// Discover coordinator, configure mesh, register callback. Returns true on success.
static bool connectToCoordinator() {
  meshChannel = mesh.findCoordinatorChannel(NODE_NAME);
  if (meshChannel == 0) {
#ifdef HAS_RGB_LED
    setSteadyState(LED_NO_COORDINATOR);
#endif
    return false;
  }

  mesh.getCoordinatorMac(coordinatorMac);
  mesh.begin(meshChannel);               // re-init on discovered channel
  mesh.setCoordinatorMac(coordinatorMac);
  mesh.onReceive(onMeshMessage);

  #if defined(ESP32)
    esp_wifi_get_mac(WIFI_IF_STA, myMac);
  #else
    WiFi.macAddress(myMac);
  #endif

  lastHeartbeat = millis() - HEARTBEAT_INTERVAL;
  lastTemp      = millis() - TEMP_INTERVAL;

  // Announce presence — PING triggers library PONG reply; DATA 0x06 registers name
  mesh.send(coordinatorMac, MESH_TYPE_PING, 0x00, (const uint8_t*)NODE_NAME, strlen(NODE_NAME), 4);
  mesh.send(coordinatorMac, MESH_TYPE_DATA, 0x06, (const uint8_t*)NODE_NAME, strlen(NODE_NAME), 4);
#ifdef HAS_RGB_LED
  setSteadyState(LED_CONNECTED);
#endif
  return true;
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

#ifdef HAS_RGB_LED
  rgbLed.begin();
  rgbLed.show();
  ledTimer = millis();
  ledState = LED_CONNECTING;
  ledPrevState = LED_CONNECTING;
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
#ifdef HAS_RGB_LED
    ledState = LED_CONNECTING;
    ledPrevState = LED_CONNECTING;
    ledTimer = millis();
#endif
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

  if (!mesh.isCoordinatorFound()) {
#ifdef HAS_RGB_LED
    if (ledState == LED_CONNECTED || ledState == LED_NO_COORDINATOR) {
      ledState = LED_CONNECTING;
      ledPrevState = LED_CONNECTING;
      ledTimer = millis();
      ledToggle = false;
    }
    ledUpdate();
#endif

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
#ifdef HAS_RGB_LED
    ledFlash(LED_TX_BLINK);
#endif
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
#ifdef HAS_RGB_LED
      ledFlash(LED_TX_BLINK);
#endif
      Serial.printf("[TX] Sensor: %s\n", payload.c_str());
    } else {
      Serial.println("[TX] Send failed");
    }
  }

#ifdef HAS_RGB_LED
  ledUpdate();
#endif
}
