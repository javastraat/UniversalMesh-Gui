#pragma once
#include <ESPAsyncWebServer.h>

void initWebDashboard(AsyncWebServer& server);
void logPacket(uint8_t type, uint8_t* senderMac, uint8_t* origSrcMac, uint8_t appId, const uint8_t* payload, uint8_t payloadLen);
void addSerialLog(const char* msg);
void lockMeshData();
void unlockMeshData();
