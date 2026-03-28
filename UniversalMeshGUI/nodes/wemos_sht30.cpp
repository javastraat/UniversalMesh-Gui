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
bool pendingLongInfo = false;
uint8_t longInfoDestMac[6] = {0, 0, 0, 0, 0, 0};
uint8_t longInfoAppId = 0;
uint8_t longInfoPart = 0;
unsigned long lastLongInfoTx = 0;
const unsigned long LONG_INFO_TX_INTERVAL = 40;

static bool sendLongInfoPart(uint8_t part, uint8_t* destMac, uint8_t appId) {
    char info[65];

    if (part == 0) {
        snprintf(info, sizeof(info), "info1:n=%s,mac=%02X:%02X:%02X:%02X:%02X:%02X",
                 NODE_NAME,
                 myMac[0], myMac[1], myMac[2], myMac[3], myMac[4], myMac[5]);
    } else if (part == 1) {
        snprintf(info, sizeof(info), "info2:up=%lus,heap=%u,rssi=%d,ch=%d",
                 millis() / 1000UL,
                 ESP.getFreeHeap(),
                 WiFi.RSSI(),
                 WiFi.channel());
    } else {
    #if defined(ESP32)
        snprintf(info, sizeof(info), "info3:chip=%s,rev=%d,sdk=%.24s",
                 ESP.getChipModel(),
                 ESP.getChipRevision(),
                 ESP.getSdkVersion());
    #else
        snprintf(info, sizeof(info), "info3:chip=ESP8266,id=%06X,sdk=%.20s",
                 ESP.getChipId(),
                 ESP.getSdkVersion());
    #endif
    }

    return mesh.send(destMac, MESH_TYPE_DATA, appId, (const uint8_t*)info, strlen(info), 4);
}

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
    if (packet->type == MESH_TYPE_PONG && packet->payloadLen > 0 && packet->payload[0] == 0x01 && !foundCoordinator) {
        memcpy(coordinatorMac, packet->srcMac, 6);
        foundCoordinator = true;
        lastHeartbeat = millis() - HEARTBEAT_INTERVAL;
        lastSensor    = millis() - SENSOR_INTERVAL;
        mesh.send(coordinatorMac, MESH_TYPE_DATA, 0x06, (const uint8_t*)NODE_NAME, strlen(NODE_NAME), 4);
        Serial.printf("[AUTO] Coordinator found at: %02X:%02X:%02X:%02X:%02X:%02X\n",
                      coordinatorMac[0], coordinatorMac[1], coordinatorMac[2],
                      coordinatorMac[3], coordinatorMac[4], coordinatorMac[5]);
    }

    bool directToMe = (memcmp(packet->destMac, myMac, 6) == 0);
    if (packet->type == MESH_TYPE_DATA && directToMe) {
        char msg[65];
        uint8_t len = packet->payloadLen > 64 ? 64 : packet->payloadLen;
        memcpy(msg, packet->payload, len);
        msg[len] = '\0';

        Serial.printf("[RX->ME] From %02X:%02X:%02X:%02X:%02X:%02X | Relay %02X:%02X:%02X:%02X:%02X:%02X | App 0x%02X | %s\n",
                      packet->srcMac[0], packet->srcMac[1], packet->srcMac[2],
                      packet->srcMac[3], packet->srcMac[4], packet->srcMac[5],
                      senderMac[0], senderMac[1], senderMac[2],
                      senderMac[3], senderMac[4], senderMac[5],
                      packet->appId, msg);

        if (len >= 4 && strncmp(msg, "cmd:", 4) == 0) {
            bool fromCoordinator = foundCoordinator && (memcmp(packet->srcMac, coordinatorMac, 6) == 0);
            if (fromCoordinator) {
                const char* command = msg + 4;
                char ack[65];
                snprintf(ack, sizeof(ack), "command received:%s", command);
                mesh.send(packet->srcMac, MESH_TYPE_DATA, packet->appId, (const uint8_t*)ack, strlen(ack), 4);
                Serial.printf("[CMD] Ack sent to %02X:%02X:%02X:%02X:%02X:%02X | %s\n",
                              packet->srcMac[0], packet->srcMac[1], packet->srcMac[2],
                              packet->srcMac[3], packet->srcMac[4], packet->srcMac[5],
                              ack);

                if (strcmp(command, "info") == 0) {
                    char info[65];
                    unsigned long up = millis() / 1000UL;
                    snprintf(info, sizeof(info), "info:u=%lus,h=%u,r=%d,ch=%d,m=%02X%02X",
                             up,
                             ESP.getFreeHeap(),
                             WiFi.RSSI(),
                             WiFi.channel(),
                             myMac[4], myMac[5]);
                    mesh.send(packet->srcMac, MESH_TYPE_DATA, packet->appId, (const uint8_t*)info, strlen(info), 4);
                    Serial.printf("[CMD] Info sent | %s\n", info);
                }

                if (strcmp(command, "info:long") == 0) {
                    memcpy(longInfoDestMac, packet->srcMac, 6);
                    longInfoAppId = packet->appId;
                    longInfoPart = 0;
                    pendingLongInfo = true;
                    lastLongInfoTx = 0;
                    Serial.println("[CMD] Long info queued");
                }

                if (strcmp(command, "reboot") == 0) {
                    Serial.println("[CMD] Reboot requested, restarting...");
                    delay(100);
                    ESP.restart();
                }
            } else {
                Serial.printf("[CMD] Ignored unauthorized command from %02X:%02X:%02X:%02X:%02X:%02X\n",
                              packet->srcMac[0], packet->srcMac[1], packet->srcMac[2],
                              packet->srcMac[3], packet->srcMac[4], packet->srcMac[5]);
            }
        }
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

    if (pendingLongInfo && millis() - lastLongInfoTx >= LONG_INFO_TX_INTERVAL) {
        bool ok = sendLongInfoPart(longInfoPart, longInfoDestMac, longInfoAppId);
        if (!ok) {
            Serial.printf("[CMD] Failed to send info%u\n", longInfoPart + 1);
        }
        lastLongInfoTx = millis();
        longInfoPart++;
        if (longInfoPart >= 3) {
            pendingLongInfo = false;
            Serial.println("[CMD] Long info sent");
        }
    }

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
