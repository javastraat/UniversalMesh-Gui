#include <Arduino.h>
#include <ArduinoJson.h>
#include <esp_wifi.h>
#include <TFT_eSPI.h>
#include <FastLED.h>
#include "UniversalMesh.h"
#include "ota_update.h"

#ifndef NODE_NAME
  #define NODE_NAME "sensor-node"
#endif
#define HEARTBEAT_INTERVAL  120000
#define TEMP_INTERVAL        60000

// --- T-Dongle S3 pin definitions ---
#define TFT_BL   38   // backlight — active LOW
#define LED_DATA 40   // DI (APA102)
#define LED_CLK  39   // CI (APA102)

// --- Hardware ---
TFT_eSPI tft;
CRGB leds;

// --- Mesh ---
UniversalMesh mesh;
uint8_t myMac[6]          = {0};
uint8_t coordinatorMac[6] = {0};
uint8_t meshChannel       = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastTemp      = 0;
volatile bool otaRequested  = false;
static float  lastTempC     = 0.0f;

// --- LED ---
enum LedState { LED_CONNECTING, LED_CONNECTED, LED_NO_COORDINATOR, LED_TX_BLINK, LED_RX_BLINK };
LedState ledState           = LED_CONNECTING;
LedState ledPrevState       = LED_CONNECTING;
unsigned long ledTimer      = 0;
bool          ledToggle     = false;
int8_t        ledFlashRemaining = 0;

constexpr unsigned long BLINK_INTERVAL = 300;
constexpr unsigned long FLASH_ON       = 120;
constexpr unsigned long FLASH_OFF      = 120;
constexpr uint8_t       FLASH_COUNT    = 2;

void setColor(CRGB color) {
  leds = color;
  FastLED.show();
}

void setSteadyState(LedState state) {
  ledState = state;
  ledPrevState = state;
  if      (state == LED_CONNECTED)       setColor(CRGB::Green);
  else if (state == LED_NO_COORDINATOR)  setColor(CRGB::Red);
  else                                   setColor(CRGB::Black);
}

void ledFlash(LedState flashState) {
  if (ledState == LED_TX_BLINK || ledState == LED_RX_BLINK) return;
  ledPrevState = ledState;
  ledState = flashState;
  ledFlashRemaining = FLASH_COUNT;
  ledToggle = true;
  ledTimer = millis();
  setColor(flashState == LED_TX_BLINK ? CRGB::Blue : CRGB::Orange);
}

void ledUpdate() {
  unsigned long now = millis();
  if (ledState == LED_CONNECTING) {
    if (now - ledTimer >= BLINK_INTERVAL) {
      ledToggle = !ledToggle;
      setColor(ledToggle ? CRGB::Blue : CRGB::Black);
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
        if (ledFlashRemaining <= 0) { setSteadyState(ledPrevState); return; }
        setColor(ledState == LED_TX_BLINK ? CRGB::Blue : CRGB::Orange);
      } else {
        setColor(CRGB::Black);
      }
      ledTimer = now;
    }
  }
}

// --- Display (landscape 160x80 after setRotation(1)) ---
void displayUpdate() {
  tft.fillScreen(TFT_BLACK);

  // Header bar
  tft.fillRect(0, 0, 160, 16, TFT_BLUE);
  tft.setTextColor(TFT_WHITE, TFT_BLUE);
  tft.setTextSize(1);
  tft.setCursor(4, 4);
  tft.print("UniversalMesh  -  ");
  tft.print(NODE_NAME);

  // MAC
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           myMac[0], myMac[1], myMac[2], myMac[3], myMac[4], myMac[5]);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(4, 22);
  tft.print(macStr);

  // Coordinator status
  tft.setCursor(4, 36);
  if (mesh.isCoordinatorFound()) {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    char buf[32];
    snprintf(buf, sizeof(buf), "Connected  ch:%d", meshChannel);
    tft.print(buf);
  } else {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.print("Searching for coordinator...");
  }

  // Temp + heap
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(4, 52);
  char tempStr[24];
  snprintf(tempStr, sizeof(tempStr), "Temp: %.1f C", lastTempC);
  tft.print(tempStr);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(4, 64);
  char heapStr[24];
  snprintf(heapStr, sizeof(heapStr), "Heap: %u", ESP.getFreeHeap());
  tft.print(heapStr);
}

// --- Mesh ---
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
  doc["chip"] = ESP.getChipModel();
  doc["rev"]  = (int)ESP.getChipRevision();
  String out;
  serializeJson(doc, out);
  mesh.send(destMac, MESH_TYPE_DATA, appId, out);
}

void onMeshMessage(MeshPacket* packet, uint8_t* senderMac) {
  ledFlash(LED_RX_BLINK);

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
  if (meshChannel == 0) {
    setSteadyState(LED_NO_COORDINATOR);
    return false;
  }
  mesh.getCoordinatorMac(coordinatorMac);
  mesh.begin(meshChannel);
  mesh.setCoordinatorMac(coordinatorMac);
  mesh.onReceive(onMeshMessage);
  esp_wifi_get_mac(WIFI_IF_STA, myMac);
  lastHeartbeat = millis() - HEARTBEAT_INTERVAL;
  lastTemp      = millis() - TEMP_INTERVAL;
  mesh.send(coordinatorMac, MESH_TYPE_PING, 0x00, (const uint8_t*)NODE_NAME, strlen(NODE_NAME), 4);
  mesh.send(coordinatorMac, MESH_TYPE_DATA, 0x06, (const uint8_t*)NODE_NAME, strlen(NODE_NAME), 4);
  setSteadyState(LED_CONNECTED);
  displayUpdate();
  return true;
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // LED (APA102 two-wire)
  FastLED.addLeds<APA102, LED_DATA, LED_CLK, BGR>(&leds, 1);
  FastLED.setBrightness(20);
  setColor(CRGB::Blue);

  // Display — backlight is active LOW
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextWrap(false);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, LOW);  // LOW = backlight ON

  ledTimer = millis();

  Serial.println();
  Serial.println("========================================");
  Serial.println("   UniversalMesh  -  T-Dongle S3 Node  ");
  Serial.println("========================================");

  // Splash
  tft.fillRect(0, 0, 160, 16, TFT_BLUE);
  tft.setTextColor(TFT_WHITE, TFT_BLUE);
  tft.setCursor(4, 4);
  tft.print("UniversalMesh");
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.setCursor(4, 28);
  tft.print("Searching for coordinator...");

  mesh.begin(1);
  mesh.onReceive(onMeshMessage);

  if (connectToCoordinator()) {
    Serial.printf("  Channel     : %d\n", meshChannel);
    Serial.printf("  MAC Address : %02X:%02X:%02X:%02X:%02X:%02X\n",
                  myMac[0], myMac[1], myMac[2], myMac[3], myMac[4], myMac[5]);
    Serial.println("  Coordinator : Found");
  } else {
    Serial.println("  Coordinator : Not found, retrying in loop...");
    ledState = LED_CONNECTING;
    ledPrevState = LED_CONNECTING;
    ledTimer = millis();
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
    if (ledState == LED_CONNECTED || ledState == LED_NO_COORDINATOR) {
      ledState = LED_CONNECTING;
      ledPrevState = LED_CONNECTING;
      ledTimer = millis();
      ledToggle = false;
    }
    ledUpdate();

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
    ledFlash(LED_TX_BLINK);
    Serial.printf("[TX] Heartbeat | %02X:%02X:%02X:%02X:%02X:%02X\n",
                  myMac[0], myMac[1], myMac[2], myMac[3], myMac[4], myMac[5]);
  }

  if (now - lastTemp >= TEMP_INTERVAL) {
    lastTemp = now;
    lastTempC = temperatureRead();
    JsonDocument doc;
    doc["name"] = NODE_NAME;
    doc["temp"] = serialized(String(lastTempC, 1));
    String payload;
    serializeJson(doc, payload);
    if (mesh.sendToCoordinator(0x01, payload)) {
      ledFlash(LED_TX_BLINK);
      Serial.printf("[TX] Sensor: %s\n", payload.c_str());
    } else {
      Serial.println("[TX] Send failed");
    }
    displayUpdate();
  }

  ledUpdate();
}
