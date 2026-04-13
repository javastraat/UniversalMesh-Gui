#include <Arduino.h>

#ifdef LILYGO_T_ETH_ELITE

#include <ETH.h>
#include <WiFi.h>       // WiFi.onEvent() is the correct bus for ETH events
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <time.h>

#ifndef MESH_HOSTNAME
  #define MESH_HOSTNAME "universalmesh"
#endif
#ifndef OTA_PASSWORD
  #define OTA_PASSWORD "mesh1234"
#endif
#ifndef NTP_SERVER
  #define NTP_SERVER "pool.ntp.org"
#endif
#ifndef NTP_GMT_OFFSET_SEC
  #define NTP_GMT_OFFSET_SEC 0
#endif
#ifndef NTP_DAYLIGHT_OFFSET_SEC
  #define NTP_DAYLIGHT_OFFSET_SEC 0
#endif

// LilyGo T-ETH Elite ESP32-S3 — W5500 SPI Ethernet pin assignments
#define ETH_MISO_PIN   47
#define ETH_MOSI_PIN   21
#define ETH_SCLK_PIN   48
#define ETH_CS_PIN     45
#define ETH_INT_PIN    14
#define ETH_RST_PIN    -1
#define ETH_PHY_ADDR    1

static bool ethConnected = false;
static bool ethLinkUp    = false;
static bool otaStarted   = false;
static bool mdnsStarted  = false;
static bool ntpStarted   = false;
static bool otaActive    = false;

static void (*_otaBeginCb)() = nullptr;
void setOtaBeginCallback(void(*cb)()) { _otaBeginCb = cb; }
bool isOtaInProgress() { return otaActive; }

static void startNTP() {
    if (ntpStarted) return;
    configTime(NTP_GMT_OFFSET_SEC, NTP_DAYLIGHT_OFFSET_SEC, NTP_SERVER, "time.nist.gov");
    ntpStarted = true;
    Serial.printf("[NTP] Sync started — server: %s  offset: %+ds\n", NTP_SERVER, NTP_GMT_OFFSET_SEC);
}

bool isNtpSynced() {
    return time(nullptr) > 1000000000UL;
}

String getNtpTimeStr() {
    if (!isNtpSynced()) return "not synced";
    struct tm t;
    getLocalTime(&t);
    char buf[24];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &t);
    return String(buf);
}

static void startOTA() {
    if (otaStarted) return;
    ArduinoOTA.setHostname(MESH_HOSTNAME);
    // No OTA password — device is on a trusted local mesh network.
    // Password-based auth causes espota.py version skew (SHA256 vs MD5 mismatch).
    ArduinoOTA.onStart([]() {
        otaActive = true;
        if (_otaBeginCb) _otaBeginCb();   // e.g. put LoRa radio in standby
        Serial.println("[OTA] Starting update — pausing mesh/MQTT/LoRa...");
    });
    ArduinoOTA.onEnd([]() {
        Serial.println("\n[OTA] Done — rebooting.");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("[OTA] %u%%\r", progress * 100 / total);
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("[OTA] Error[%u]: ", error);
        if      (error == OTA_AUTH_ERROR)    Serial.println("Auth failed");
        else if (error == OTA_BEGIN_ERROR)   Serial.println("Begin failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive failed");
        else if (error == OTA_END_ERROR)     Serial.println("End failed");
    });
    ArduinoOTA.begin();
    otaStarted = true;
    Serial.printf("[OTA] Ready — hostname: %s\n", MESH_HOSTNAME);
}

static void startMDNS() {
    if (mdnsStarted) return;
    // ArduinoOTA.begin() already calls MDNS.begin() internally;
    // we just add our extra service records here.
    MDNS.addService("http", "tcp", 80);
    mdnsStarted = true;
    Serial.println("[mDNS] Started — universalmesh.local (http._tcp)");
}

bool   isEthConnected()  { return ethConnected; }
bool   isEthLinkUp()     { return ethLinkUp; }
String getEthLocalIP()    { return ETH.localIP().toString(); }
String getEthMAC()        { return ETH.macAddress(); }
String getEthGateway()    { return ETH.gatewayIP().toString(); }
String getEthSubnet()     { return ETH.subnetMask().toString(); }
String getEthDNS()        { return ETH.dnsIP().toString(); }
int    getEthLinkSpeed()  { return ETH.linkSpeed(); }
bool   isEthFullDuplex()  { return ETH.fullDuplex(); }

static void onEthEvent(arduino_event_id_t event) {
    switch (event) {
        case ARDUINO_EVENT_ETH_START:
            Serial.println("[ETH] Started");
            ETH.setHostname("universalmesh");
            break;
        case ARDUINO_EVENT_ETH_CONNECTED:
            Serial.println("[ETH] Link Up");
            ethLinkUp = true;
            break;
        case ARDUINO_EVENT_ETH_GOT_IP:
            Serial.printf("[ETH] IP: %s  MAC: %s  %dMbps %s\n",
                ETH.localIP().toString().c_str(),
                ETH.macAddress().c_str(),
                ETH.linkSpeed(),
                ETH.fullDuplex() ? "FULL_DUPLEX" : "HALF_DUPLEX");
            ethConnected = true;
            startOTA();
            startMDNS();
            startNTP();
            break;
        case ARDUINO_EVENT_ETH_LOST_IP:
            Serial.println("[ETH] Lost IP");
            ethConnected = false;
            if (mdnsStarted) { MDNS.end(); mdnsStarted = false; }
            break;
        case ARDUINO_EVENT_ETH_DISCONNECTED:
            Serial.println("[ETH] Link Down");
            ethConnected = false;
            ethLinkUp    = false;
            if (mdnsStarted) { MDNS.end(); mdnsStarted = false; }
            break;
        case ARDUINO_EVENT_ETH_STOP:
            Serial.println("[ETH] Stopped");
            ethConnected = false;
            break;
        default:
            break;
    }
}

void setupETH() {
    Serial.println("[ETH] Initializing W5500 (LilyGo T-ETH Elite)...");

    // Must use WiFi.onEvent — same event bus the ETH driver fires on
    WiFi.onEvent(onEthEvent);

    if (!ETH.begin(ETH_PHY_W5500, ETH_PHY_ADDR,
                   ETH_CS_PIN, ETH_INT_PIN, ETH_RST_PIN,
                   SPI3_HOST,
                   ETH_SCLK_PIN, ETH_MISO_PIN, ETH_MOSI_PIN)) {
        Serial.println("[ETH] ERROR: Failed to initialize W5500!");
    } else {
        Serial.println("[ETH] W5500 init OK — waiting for link...");
        delay(2000);  // Give the chip time to negotiate link
    }
}

void loopETH() {
    if (otaStarted) ArduinoOTA.handle();
}

#else  // Stub for non-ETH builds (e.g. coordinator_c6, coordinator_s3)

void setupETH() {}
void loopETH() {}

#endif  // LILYGO_T_ETH_ELITE
