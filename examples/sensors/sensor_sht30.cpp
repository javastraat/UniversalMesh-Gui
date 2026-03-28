#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SHT31.h>
#include "UniversalMesh.h"

#define nodeName            "wemosd1-sht30"
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

void onMeshMessage(MeshPacket* packet, uint8_t* senderMac) {
    if (packet->type == MESH_TYPE_PONG && !foundCoordinator) {
        memcpy(coordinatorMac, packet->srcMac, 6);
        foundCoordinator = true;
        lastHeartbeat = millis() - HEARTBEAT_INTERVAL;
        lastSensor    = millis() - SENSOR_INTERVAL;
        Serial.printf("[AUTO] Coordinator found at: %02X:%02X:%02X:%02X:%02X:%02X\n",
                      coordinatorMac[0], coordinatorMac[1], coordinatorMac[2],
                      coordinatorMac[3], coordinatorMac[4], coordinatorMac[5]);
    }
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    sht30.begin(0x45);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (mesh.begin(WIFI_CHANNEL)) {
        mesh.onReceive(onMeshMessage);

        uint8_t mac[6];
        WiFi.macAddress(mac);
        memcpy(myMac, mac, 6);

        Serial.println();
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
            Serial.printf("[TX] Heartbeat sent | MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                          myMac[0], myMac[1], myMac[2], myMac[3], myMac[4], myMac[5]);
        }

        if (now - lastSensor >= SENSOR_INTERVAL) {
            lastSensor = now;
            float tempC = sht30.readTemperature();
            float hum   = sht30.readHumidity();
            if (!isnan(tempC) && !isnan(hum)) {
                char payload[56];
                snprintf(payload, sizeof(payload), "N:%s,T:%.1fC,H:%.1f", nodeName, tempC, hum);
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
}
