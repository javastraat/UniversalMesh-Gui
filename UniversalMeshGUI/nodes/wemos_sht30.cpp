#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SHT31.h>
#include "UniversalMesh.h"

#ifdef HAS_DISPLAY_SHIELD
#include <U8g2lib.h>
// WEMOS OLED shield: SSD1306 64x48, I2C 0x3C, reset GPIO0
// _F_ = full buffer mode, HW_I2C = hardware I2C
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
#define WIFI_CHANNEL        1
#define HEARTBEAT_INTERVAL  60000
#define SENSOR_INTERVAL     30000

UniversalMesh  mesh;
Adafruit_SHT31 sht30;

uint8_t myMac[6]          = {0, 0, 0, 0, 0, 0};
uint8_t coordinatorMac[6] = {0, 0, 0, 0, 0, 0};
bool foundCoordinator     = false;
unsigned long lastAttempt   = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastSensor    = 0;

// -------------------------------------------------------
// Display pages (64x48 WEMOS OLED)
// U8g2: Y coordinate is the text BASELINE
// Fonts: 6x10 for small, helvB18 for large numbers
// -------------------------------------------------------
#ifdef HAS_DISPLAY_SHIELD
void showPage(uint8_t page) {
    char buf[12];
    u8g2.clearBuffer();

    if (page == 0) {
        // --- Page 1: Temperature ---
        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.drawStr(0, 9,  "Temp");
        u8g2.setFont(u8g2_font_helvB18_tf);
        if (!isnan(lastTempC)) {
            snprintf(buf, sizeof(buf), "%.1f", lastTempC);
        } else {
            snprintf(buf, sizeof(buf), "--.-");
        }
        u8g2.drawStr(0, 36, buf);
        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.drawStr(0, 47, "deg C");

    } else if (page == 1) {
        // --- Page 2: Humidity ---
        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.drawStr(0, 9,  "Humidity");
        u8g2.setFont(u8g2_font_helvB18_tf);
        if (!isnan(lastHum)) {
            snprintf(buf, sizeof(buf), "%.1f", lastHum);
        } else {
            snprintf(buf, sizeof(buf), "--.-");
        }
        u8g2.drawStr(0, 36, buf);
        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.drawStr(0, 47, "%RH");

    } else {
        // --- Page 3: Node status (4 lines, small font) ---
        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.drawStr(0, 9,  "wemosd1");
        snprintf(buf, sizeof(buf), "%02X:%02X:%02X", myMac[3], myMac[4], myMac[5]);
        u8g2.drawStr(0, 21, buf);
        snprintf(buf, sizeof(buf), "CH: %d", WIFI_CHANNEL);
        u8g2.drawStr(0, 33, buf);
        u8g2.drawStr(0, 45, foundCoordinator ? "COORD: OK" : "searching");
    }

    u8g2.sendBuffer();
}
#endif

// -------------------------------------------------------

void onMeshMessage(MeshPacket* packet, uint8_t* senderMac) {
    if (packet->type == MESH_TYPE_PONG && packet->payload[0] == 0x01 && !foundCoordinator) {
        memcpy(coordinatorMac, packet->srcMac, 6);
        foundCoordinator = true;
        lastHeartbeat = millis() - HEARTBEAT_INTERVAL;
        lastSensor    = millis() - SENSOR_INTERVAL;
        mesh.send(coordinatorMac, MESH_TYPE_DATA, 0x06, (const uint8_t*)NODE_NAME, strlen(NODE_NAME), 4);
        Serial.printf("[AUTO] Coordinator found at: %02X:%02X:%02X:%02X:%02X:%02X\n",
                      coordinatorMac[0], coordinatorMac[1], coordinatorMac[2],
                      coordinatorMac[3], coordinatorMac[4], coordinatorMac[5]);
    }
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

    if (mesh.begin(WIFI_CHANNEL)) {
        mesh.onReceive(onMeshMessage);

        uint8_t mac[6];
        WiFi.macAddress(mac);
        memcpy(myMac, mac, 6);

        Serial.println();
        Serial.println("========================================");
        Serial.println("    UniversalMesh  -  SHT30 Sensor     ");
        Serial.println("========================================");
        Serial.printf("  MAC Address : %02X:%02X:%02X:%02X:%02X:%02X\n",
                      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        Serial.printf("  Mesh Channel: %d\n", WIFI_CHANNEL);
        Serial.printf("  Heartbeat Interval: %d ms\n", HEARTBEAT_INTERVAL);
        Serial.printf("  Sensor Interval:    %d ms\n", SENSOR_INTERVAL);
        Serial.println("========================================");
        Serial.println("  Searching for Coordinator...");
        Serial.println("========================================");
        Serial.println();

#ifdef HAS_DISPLAY_SHIELD
        showPage(0);
        lastPageSwitch = millis();
#endif

        uint8_t broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        mesh.send(broadcast, MESH_TYPE_PING, 0x00, nullptr, 0, 4);
    }
}

void loop() {
    mesh.update();

    if (foundCoordinator) {
        unsigned long now = millis();

        if (now - lastHeartbeat >= HEARTBEAT_INTERVAL) {
            lastHeartbeat = now;
            uint8_t heartbeat = 0x01;
            mesh.send(coordinatorMac, MESH_TYPE_DATA, 0x05, &heartbeat, 1, 4);
            mesh.send(coordinatorMac, MESH_TYPE_DATA, 0x06, (const uint8_t*)NODE_NAME, strlen(NODE_NAME), 4);
            Serial.printf("[TX] Heartbeat | MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
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
                char payload[56];
                snprintf(payload, sizeof(payload), "N:%s,T:%.1fC,H:%.1f", NODE_NAME, tempC, hum);
                mesh.send(coordinatorMac, MESH_TYPE_DATA, 0x01, (const uint8_t*)payload, strlen(payload), 4);
                Serial.printf("[TX] Sensor: %s\n", payload);
            } else {
                Serial.println("[SHT30] Read error — check wiring/address");
            }
        }
    }
    else if (millis() - lastAttempt > 10000) {
        lastAttempt = millis();
        uint8_t broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        mesh.send(broadcast, MESH_TYPE_PING, 0x00, nullptr, 0, 4);
        Serial.println("[RETRY] Searching for Coordinator...");
    }

#ifdef HAS_DISPLAY_SHIELD
    if (millis() - lastPageSwitch >= PAGE_INTERVAL) {
        lastPageSwitch = millis();
        currentPage    = (currentPage + 1) % 3;
        showPage(currentPage);
    }
#endif
}
