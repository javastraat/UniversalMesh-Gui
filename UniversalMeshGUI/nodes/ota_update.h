#pragma once
// OTA-over-WiFi update helper.
// Triggered by cmd:update: stops the mesh, connects to WiFi, and blocks in
// ArduinoOTA loop. The IDE uploads the new firmware; the ESP reboots when done.
// Requires -D OTA_SSID and -D OTA_PASSWORD build flags (defined in platformio.ini).

#include <Arduino.h>
#include <ArduinoOTA.h>
#if defined(ESP32)
  #include <WiFi.h>
  #include <esp_now.h>
#else
  #include <ESP8266WiFi.h>
  #include <espnow.h>
#endif

#ifndef OTA_SSID
  #define OTA_SSID ""
#endif
#ifndef OTA_PASSWORD
  #define OTA_PASSWORD ""
#endif

static void startOtaUpdate() {
  Serial.println("[OTA] Stopping mesh...");
  esp_now_deinit();

#if !defined(ESP32)
  // ESP8266: the SDK needs the radio fully released before switching mode.
  // Going through WIFI_OFF lets the internal SDK state settle; without this
  // the soft WDT fires during WiFi.begin() because the radio handoff blocks.
  WiFi.persistent(false);
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(200);
  yield();
#endif

  WiFi.mode(WIFI_STA);
  WiFi.begin(OTA_SSID, OTA_PASSWORD);
  Serial.print("[OTA] Connecting to WiFi");
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 15000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[OTA] WiFi failed — rebooting");
    delay(100);
    ESP.restart();
    return;
  }
  Serial.print("[OTA] IP: ");
  Serial.println(WiFi.localIP());

  ArduinoOTA.setHostname(NODE_NAME);
  ArduinoOTA.onStart([]()                             { Serial.println("[OTA] Start"); });
  ArduinoOTA.onEnd([]()                               { Serial.println("\n[OTA] Done"); });
  ArduinoOTA.onProgress([](unsigned int p, unsigned int tot) {
    Serial.printf("[OTA] %u%%\r", p * 100 / tot);
  });
  ArduinoOTA.onError([](ota_error_t e) {
    Serial.printf("[OTA] Error[%u] — rebooting\n", e);
    delay(100);
    ESP.restart();
  });
  ArduinoOTA.begin();
  Serial.println("[OTA] Ready — open PlatformIO OTA upload or arduino-cli to flash");

  // Block here; ArduinoOTA reboots automatically when the upload finishes
  while (true) {
    ArduinoOTA.handle();
    delay(10);
  }
}
