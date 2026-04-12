#include <Arduino.h>
#include <ArduinoJson.h>
#include <esp_wifi.h>
#include <TFT_eSPI.h>
#include "UniversalMesh.h"
#include "ota_update.h"

#ifndef NODE_NAME
  #define NODE_NAME "sensor-tdisplay"
#endif
#define HEARTBEAT_INTERVAL  120000
#define TEMP_INTERVAL        60000

// T-Display S3 — power, backlight, button (all TFT pins come from User_Setup_Select.h)
#define PIN_POWER_ON   15   // must be HIGH to power the display board
#define PIN_LCD_BL     38   // backlight, active HIGH
#define PIN_BTN        14   // user button, active LOW (right side of board)
//   Short press  → toggle backlight on / off
//   Long press   → toggle dark / light theme

// Theme palettes (RGB565) — formula: (R>>3)<<11 | (G>>2)<<5 | (B>>3)
//   Index:  BG      CARD    ACCENT  GREEN   RED     MUTED   TEXT    BORDER
static const uint16_t PALETTE_DARK[8]  = {
  0x0882, 0x10C4, 0x5D3F, 0x3D8A, 0xFA89, 0x6BB0, 0xFFFF, 0x10C4
};
static const uint16_t PALETTE_LIGHT[8] = {
  0xF7DF, 0xFFFF, 0x0B5B, 0x1BE6, 0xC905, 0x636E, 0x1905, 0xD69A
};

static uint16_t COL_BG, COL_CARD, COL_ACCENT, COL_GREEN, COL_RED, COL_MUTED, COL_TEXT, COL_BORDER;
static bool     displayOn  = true;
static bool     lightMode  = false;

static void applyTheme(bool light) {
  lightMode  = light;
  const uint16_t* p = light ? PALETTE_LIGHT : PALETTE_DARK;
  COL_BG     = p[0];
  COL_CARD   = p[1];
  COL_ACCENT = p[2];
  COL_GREEN  = p[3];
  COL_RED    = p[4];
  COL_MUTED  = p[5];
  COL_TEXT   = p[6];
  COL_BORDER = p[7];
}

// --- Hardware ---
TFT_eSPI tft;

// --- Mesh ---
UniversalMesh mesh;
uint8_t myMac[6]          = {0};
uint8_t coordinatorMac[6] = {0};
uint8_t meshChannel       = 0;
unsigned long lastHeartbeat   = 0;
unsigned long lastTemp        = 0;
unsigned long lastDisplayTick = 0;
volatile bool otaRequested  = false;
static float  lastTempC     = 0.0f;

// --- Display layout (landscape 320 x 170) ----------------------
//
//  ┌─ Header (navbar) ──────────────────────────── y 0..22  ─┐
//  ├─ Card: Node ──────┬─ Card: Mesh ───────────── y 25..107 ─┤
//  │  MAC              │  ● status / channel                  │
//  │  Heap  (tick)     │  Coord MAC                           │
//  │  Uptime (tick)    │                                      │
//  ├─ Card: Temperature ─────────────────────────── y111..165 ─┤
//  │  Temperature label          25.0 C  (tick)               │
//  └──────────────────────────────────────────────────────────┘
//
// displayTick() : updates only heap, uptime, temperature — no flicker
// displayUpdate(): full redraw — called on connect / coord-change

static void cardBg(int16_t x, int16_t y, int16_t w, int16_t h) {
  tft.fillRoundRect(x, y, w, h, 4, COL_CARD);
  tft.drawRoundRect(x, y, w, h, 4, COL_BORDER);  // visible in light, invisible in dark
}

// Updates only the three live fields — no full-screen clear, no flicker.
void displayTick() {
  if (!displayOn) return;
  lastTempC = temperatureRead();   // always read fresh for display
  tft.setTextSize(1);

  // Heap — inside Card Node (x=4, y=25, w=152)
  tft.fillRect(10, 66, 140, 9, COL_CARD);
  tft.setTextColor(COL_TEXT, COL_CARD);
  tft.setCursor(10, 66);
  char heapStr[24];
  snprintf(heapStr, sizeof(heapStr), "Heap  %uB", ESP.getFreeHeap());
  tft.print(heapStr);

  // Uptime — inside Card Node
  tft.fillRect(10, 78, 140, 9, COL_CARD);
  tft.setCursor(10, 78);
  char upStr[20];
  snprintf(upStr, sizeof(upStr), "Up    %lus", millis() / 1000UL);
  tft.print(upStr);

  // Temperature — inside Card Temp (x=4, y=111, w=312)
  tft.fillRect(10, 130, 200, 26, COL_CARD);
  tft.setTextColor(COL_ACCENT, COL_CARD);
  tft.setTextSize(3);
  tft.setCursor(10, 130);
  char tempStr[12];
  snprintf(tempStr, sizeof(tempStr), "%.1f C", lastTempC);
  tft.print(tempStr);
}

// Full redraw — called on connect, coordinator change, OTA request.
void displayUpdate() {
  if (!displayOn) return;
  tft.fillScreen(COL_BG);

  // ── Header / navbar ─────────────────────────────────────────
  tft.fillRect(0, 0, 320, 22, COL_CARD);
  tft.fillCircle(11, 11, 4, COL_ACCENT);          // blue dot
  tft.setTextColor(COL_TEXT, COL_CARD);
  tft.setTextSize(1);
  tft.setCursor(20, 7);
  tft.print("UniversalMesh");
  tft.setTextColor(COL_MUTED, COL_CARD);
  tft.setCursor(130, 7);
  tft.print(NODE_NAME);

  // ── Card: Node (left) ───────────────────────────────────────
  cardBg(4, 25, 152, 83);
  tft.setTextColor(COL_ACCENT, COL_CARD);
  tft.setTextSize(1);
  tft.setCursor(10, 30);
  tft.print("Node");

  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           myMac[0], myMac[1], myMac[2], myMac[3], myMac[4], myMac[5]);
  tft.setTextColor(COL_MUTED, COL_CARD);
  tft.setCursor(10, 42);
  tft.print("MAC");
  tft.setTextColor(COL_TEXT, COL_CARD);
  tft.setCursor(34, 42);
  tft.print(macStr);

  tft.setTextColor(COL_MUTED, COL_CARD);
  tft.setCursor(10, 54);
  tft.print("Ch");
  tft.setTextColor(COL_TEXT, COL_CARD);
  tft.setCursor(34, 54);
  if (meshChannel) { char chStr[4]; snprintf(chStr,sizeof(chStr),"%d",meshChannel); tft.print(chStr); }
  else             { tft.print("-"); }

  // Heap + uptime drawn by displayTick() below

  // ── Card: Mesh (right) ──────────────────────────────────────
  cardBg(160, 25, 156, 83);
  tft.setTextColor(COL_ACCENT, COL_CARD);
  tft.setTextSize(1);
  tft.setCursor(166, 30);
  tft.print("Mesh");

  if (mesh.isCoordinatorFound()) {
    tft.fillCircle(172, 46, 4, COL_GREEN);
    tft.setTextColor(COL_GREEN, COL_CARD);
    tft.setCursor(180, 42);
    tft.print("Connected");

    // Coordinator MAC
    char coordStr[18];
    snprintf(coordStr, sizeof(coordStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             coordinatorMac[0], coordinatorMac[1], coordinatorMac[2],
             coordinatorMac[3], coordinatorMac[4], coordinatorMac[5]);
    tft.setTextColor(COL_MUTED, COL_CARD);
    tft.setCursor(166, 58);
    tft.print("Coord");
    tft.setTextColor(COL_TEXT, COL_CARD);
    tft.setCursor(166, 68);
    tft.print(coordStr);
  } else {
    tft.fillCircle(172, 46, 4, COL_RED);
    tft.setTextColor(COL_RED, COL_CARD);
    tft.setCursor(180, 42);
    tft.print("Searching...");
  }

  // ── Card: Temperature ────────────────────────────────────────
  cardBg(4, 111, 312, 55);
  tft.setTextColor(COL_MUTED, COL_CARD);
  tft.setTextSize(1);
  tft.setCursor(10, 116);
  tft.print("Temperature");

  // Live value drawn by displayTick() below
  displayTick();
}

// --- Button (short = backlight, long = theme toggle) ---
static void handleButton() {
  static bool     lastState  = HIGH;
  static uint32_t pressStart = 0;
  static bool     longFired  = false;

  bool state = digitalRead(PIN_BTN);

  if (lastState == HIGH && state == LOW) {        // just pressed
    pressStart = millis();
    longFired  = false;
  } else if (state == LOW && !longFired) {        // still held
    if (millis() - pressStart >= 700) {
      longFired = true;
      applyTheme(!lightMode);
      if (displayOn) displayUpdate();
    }
  } else if (lastState == LOW && state == HIGH) { // just released
    if (!longFired) {                             // short press
      displayOn = !displayOn;
      digitalWrite(PIN_LCD_BL, displayOn ? HIGH : LOW);
    }
  }
  lastState = state;
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
  esp_wifi_get_mac(WIFI_IF_STA, myMac);
  lastHeartbeat = millis() - HEARTBEAT_INTERVAL;
  lastTemp      = millis() - TEMP_INTERVAL;
  mesh.send(coordinatorMac, MESH_TYPE_PING, 0x00, (const uint8_t*)NODE_NAME, strlen(NODE_NAME), 4);
  mesh.send(coordinatorMac, MESH_TYPE_DATA, 0x06, (const uint8_t*)NODE_NAME, strlen(NODE_NAME), 4);
  displayUpdate();
  return true;
}

void setup() {
  Serial.begin(115200);

  // Theme — boot default from compile flag, runtime-switchable via button
#ifdef DISPLAY_LIGHT_MODE
  applyTheme(true);
#else
  applyTheme(false);
#endif

  // Button
  pinMode(PIN_BTN, INPUT_PULLUP);

  // Power on the display board first
  pinMode(PIN_POWER_ON, OUTPUT);
  digitalWrite(PIN_POWER_ON, HIGH);

  // Display
  tft.init();
  tft.setRotation(1);           // landscape: 320 x 170
  tft.fillScreen(TFT_BLACK);
  tft.setTextWrap(false);
  pinMode(PIN_LCD_BL, OUTPUT);
  digitalWrite(PIN_LCD_BL, HIGH);  // backlight on (active HIGH)

  // Splash
  tft.fillRect(0, 0, 320, 22, COL_CARD);
  tft.setTextColor(COL_TEXT, COL_CARD);
  tft.setTextSize(1);
  tft.setCursor(6, 7);
  tft.print("UniversalMesh   ");
  tft.print(NODE_NAME);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(6, 36);
  tft.print("Searching for coordinator...");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.println();
  Serial.println("========================================");
  Serial.println("  UniversalMesh  -  T-Display S3 Node  ");
  Serial.println("========================================");

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
  handleButton();

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
      Serial.println("[RETRY] Scanning for coordinator...");
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
    lastTempC = temperatureRead();
    JsonDocument doc;
    doc["name"] = NODE_NAME;
    doc["temp"] = serialized(String(lastTempC, 1));
    String payload;
    serializeJson(doc, payload);
    if (mesh.sendToCoordinator(0x01, payload)) {
      Serial.printf("[TX] Sensor: %s\n", payload.c_str());
    } else {
      Serial.println("[TX] Send failed");
    }
  }

  if (now - lastDisplayTick >= 1000) {
    lastDisplayTick = now;
    displayTick();
  }
}
