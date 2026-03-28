#include <Arduino.h>
#include "UniversalMesh.h"

#define WIFI_CHANNEL 6
#define SEND_INTERVAL 5000 

UniversalMesh mesh;
uint8_t coordinatorMac[6] = {0, 0, 0, 0, 0, 0};
bool foundCoordinator = false;
unsigned long lastAttempt = 0;

void onMeshMessage(MeshPacket* packet, uint8_t* senderMac) {
    // If we receive a PONG, we assume it's the Coordinator replying to our discovery ping
    if (packet->type == MESH_TYPE_PONG) {
        memcpy(coordinatorMac, packet->srcMac, 6);
        foundCoordinator = true;
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
        Serial.println("[SYSTEM] Sensor Node Online. Searching for Coordinator...");
        
        // Initial Broadcast Ping to find the Coordinator
        uint8_t broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        mesh.send(broadcast, MESH_TYPE_PING, 0x00, nullptr, 0, 4);
    }
}

void loop() {
    mesh.update();

    if (foundCoordinator && (millis() - lastAttempt > SEND_INTERVAL)) {
        lastAttempt = millis();
        
        // Example: Send a "Pulse" AppID: 0x05
        uint8_t heartbeat = 0x01;
        mesh.send(coordinatorMac, MESH_TYPE_DATA, 0x05, &heartbeat, 1, 4);
        Serial.println("[TX] Heartbeat sent to Coordinator");
    } 
    else if (!foundCoordinator && (millis() - lastAttempt > 10000)) {
        // Retry discovery every 10 seconds if not found
        lastAttempt = millis();
        uint8_t broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        mesh.send(broadcast, MESH_TYPE_PING, 0x00, nullptr, 0, 4);
        Serial.println("[RETRY] Searching for Coordinator...");
    }
}