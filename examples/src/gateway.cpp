#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "UniversalMesh.h"

// --- CONFIGURATION ---
#define WIFI_CHANNEL 6 // Matches the Master's TechInc channel

// HW364A I2C Pins
#define OLED_SDA 14
#define OLED_SCL 12

UniversalMesh mesh;
Adafruit_SSD1306 display(128, 64, &Wire, -1);

// WDT-Safe Buffer for OLED updates
volatile bool newDataReady = false;
uint8_t lastAppId = 0;
char lastPayload[129] = {0}; // Expanded buffer in case of long Hex strings

// --- RECEIVE CALLBACK ---
void onMeshMessage(MeshPacket* packet, uint8_t* senderMac) {
  // We only care about DATA packets (PINGs are handled silently by the library now)
  if (packet->type == MESH_TYPE_DATA) {
    lastAppId = packet->appId;
    memset(lastPayload, 0, sizeof(lastPayload));
    
    if (packet->payloadLen > 0) {
      if (packet->appId == 1) {
        // Mode 1: Decode as ASCII Text
        memcpy(lastPayload, packet->payload, packet->payloadLen);
      } else {
        // Mode 2: Display as Raw Hex String
        for (int i = 0; i < packet->payloadLen; i++) {
          char hex[3];
          sprintf(hex, "%02X", packet->payload[i]);
          strcat(lastPayload, hex);
        }
      }
    }
    
    newDataReady = true;
  }
}

void setup() {
  Serial.begin(115200);
  
  Wire.begin(OLED_SDA, OLED_SCL);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); 
  }
  
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println("MESH GATEWAY");
  display.printf("CHANNEL: %d\n", WIFI_CHANNEL);
  display.println("INITIALIZING...");
  display.display();

  // Explicitly disconnect Wi-Fi before starting the mesh (The Stability Fix)
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (mesh.begin(WIFI_CHANNEL)) {
    mesh.onReceive(onMeshMessage);
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("MESH GATEWAY");
    display.println("ONLINE & LISTENING");
    display.display();
  }
}

void loop() {
  mesh.update();
  
  if (newDataReady) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.println(">> INCOMING MSG <<");
    display.printf("APP ID: %02X\n", lastAppId);
    
    display.setCursor(0, 25);
    display.setTextSize(1);
    display.println(lastPayload);
    display.display();
    
    newDataReady = false;
  }
  
  delay(10);
}