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
  #include <esp_wifi.h>
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

static const char* wlStatusToText(wl_status_t st) {
  switch (st) {
    case WL_NO_SHIELD: return "WL_NO_SHIELD";
    case WL_IDLE_STATUS: return "WL_IDLE_STATUS";
    case WL_NO_SSID_AVAIL: return "WL_NO_SSID_AVAIL";
    case WL_SCAN_COMPLETED: return "WL_SCAN_COMPLETED";
    case WL_CONNECTED: return "WL_CONNECTED";
    case WL_CONNECT_FAILED: return "WL_CONNECT_FAILED";
    case WL_CONNECTION_LOST: return "WL_CONNECTION_LOST";
    case WL_DISCONNECTED: return "WL_DISCONNECTED";
    default: return "WL_UNKNOWN";
  }
}

static void startOtaUpdate() {
  Serial.println("[OTA] Stopping mesh...");
  esp_now_deinit();

  if (strlen(OTA_SSID) == 0) {
    Serial.println("[OTA] OTA_SSID is empty - rebooting");
    delay(100);
    ESP.restart();
    return;
  }

#if !defined(ESP32)
  // ESP8266: the SDK needs the radio fully released before switching mode.
  // Going through WIFI_OFF lets the internal SDK state settle; without this
  // the soft WDT fires during WiFi.begin() because the radio handoff blocks.
  WiFi.persistent(false);
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(200);
  yield();
#else
  // ESP32-C6 can stay in a stale STA state after ESP-NOW deinit.
  // Fully stop/start Wi-Fi before re-associating for OTA.
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);
  delay(200);
  esp_wifi_stop();
  delay(50);
  esp_wifi_start();
  delay(50);
#endif

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);

  Serial.printf("[OTA] Joining SSID: %s\n", OTA_SSID);
  WiFi.begin(OTA_SSID, OTA_PASSWORD);
  Serial.print("[OTA] Connecting to WiFi");
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 30000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.printf("[OTA] WiFi failed (%s) - rebooting\n", wlStatusToText(WiFi.status()));
    delay(100);
    ESP.restart();
    return;
  }
  Serial.print("[OTA] IP: ");
  Serial.println(WiFi.localIP());

  static bool otaUploadStarted = false;
  otaUploadStarted = false;

  ArduinoOTA.setHostname(NODE_NAME);
  ArduinoOTA.onStart([]()                             { otaUploadStarted = true; Serial.println("[OTA] Start"); });
  ArduinoOTA.onEnd([]()                               { Serial.println("\n[OTA] Done"); });
  ArduinoOTA.onProgress([](unsigned int p, unsigned int tot) {
    Serial.printf("[OTA] %u%%\r", p * 100 / tot);
  });
  ArduinoOTA.onError([](ota_error_t e) {
    Serial.printf("[OTA] Error[%u] - rebooting\n", e);
    delay(100);
    ESP.restart();
  });
  ArduinoOTA.begin();
  Serial.println("[OTA] Ready - open PlatformIO OTA upload or arduino-cli to flash");
  Serial.println("[OTA] Waiting up to 2 minutes before reverting to mesh mode");

  // Block here; ArduinoOTA reboots automatically when the upload finishes.
  // If no upload begins within 2 minutes, reboot back into normal mesh mode.
  unsigned long otaDeadline = millis() + 120000UL;
  while (true) {
    ArduinoOTA.handle();
    if (!otaUploadStarted && millis() > otaDeadline) {
      Serial.println("[OTA] Timeout - no upload received, rebooting to mesh mode");
      delay(100);
      ESP.restart();
    }
    delay(10);
  }
}
