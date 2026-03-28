#include <Arduino.h>
#if defined(ESP32)
  #include <esp_wifi.h>
#endif
#include "UniversalMesh.h"

#ifndef NODE_NAME
  #define NODE_NAME "sensor-node"
#endif
#define WIFI_CHANNEL        1
#define HEARTBEAT_INTERVAL  60000
#define TEMP_INTERVAL       30000

UniversalMesh mesh;
uint8_t myMac[6]          = {0, 0, 0, 0, 0, 0};
uint8_t coordinatorMac[6] = {0, 0, 0, 0, 0, 0};
bool foundCoordinator = false;
unsigned long lastAttempt   = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastTemp      = 0;

void onMeshMessage(MeshPacket* packet, uint8_t* senderMac) {
    // If we receive a PONG, we assume it's the Coordinator replying to our discovery ping
    if (packet->type == MESH_TYPE_PONG && packet->payload[0] == 0x01 && !foundCoordinator) {
        memcpy(coordinatorMac, packet->srcMac, 6);
        foundCoordinator = true;
        lastHeartbeat = millis() - HEARTBEAT_INTERVAL;  // fire both immediately
        mesh.send(coordinatorMac, MESH_TYPE_DATA, 0x06, (const uint8_t*)NODE_NAME, strlen(NODE_NAME), 4);
        lastTemp      = millis() - TEMP_INTERVAL;
        Serial.printf("[AUTO] Coordinator found at: %02X:%02X:%02X:%02X:%02X:%02X\n",
                      coordinatorMac[0], coordinatorMac[1], coordinatorMac[2],
                      coordinatorMac[3], coordinatorMac[4], coordinatorMac[5]);
    }
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (mesh.begin(WIFI_CHANNEL)) {
        mesh.onReceive(onMeshMessage);

        uint8_t mac[6];
        #if defined(ESP32)
          esp_wifi_get_mac(WIFI_IF_STA, mac);
        #else
          WiFi.macAddress(mac);
        #endif
        memcpy(myMac, mac, 6);

        Serial.println();
        Serial.println();
        Serial.println("========================================");
        Serial.println("       UniversalMesh  -  Sensor Node   ");
        Serial.println("========================================");
        Serial.printf("  MAC Address : %02X:%02X:%02X:%02X:%02X:%02X\n",
                      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        Serial.printf("  Mesh Channel: %d\n", WIFI_CHANNEL);
        Serial.printf("  Heartbeat Interval: %d ms\n", HEARTBEAT_INTERVAL);
        Serial.printf("  Temp Interval:      %d ms\n", TEMP_INTERVAL);
        Serial.println("========================================");
        Serial.println("  Searching for Coordinator...");
        Serial.println("========================================");
        Serial.println();
        
        // Initial Broadcast Ping to find the Coordinator
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
            Serial.printf("[TX] Heartbeat sent | MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                          myMac[0], myMac[1], myMac[2], myMac[3], myMac[4], myMac[5]);
        }

        if (now - lastTemp >= TEMP_INTERVAL) {
            lastTemp = now;
            #if defined(ESP8266)
            float tempC = 18.0f + (rand() % 100) / 10.0f;
            #else
            float tempC = temperatureRead();
            #endif
            char payload[48];
            snprintf(payload, sizeof(payload), "N:%s,T:%.1fC", NODE_NAME, tempC);
            mesh.send(coordinatorMac, MESH_TYPE_DATA, 0x01, (const uint8_t*)payload, strlen(payload), 4);
            Serial.printf("[TX] Sensor: %s\n", payload);
        }
    }
    else if (millis() - lastAttempt > 10000) {
        // Retry discovery every 10 seconds if not found
        lastAttempt = millis();
        uint8_t broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        mesh.send(broadcast, MESH_TYPE_PING, 0x00, nullptr, 0, 4);
        Serial.println("[RETRY] Searching for Coordinator...");
    }
}