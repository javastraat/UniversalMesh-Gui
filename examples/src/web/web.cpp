#include "web.h"
#include "styles.h"
#include "pwa_icon_192_png.h"
#include "pwa_icon_512_png.h"
#include "touch_icon_png.h"
#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <freertos/semphr.h>

#ifdef LILYGO_T_ETH_ELITE
extern bool    isNtpSynced();
extern String  getNtpTimeStr();
extern bool    isEthConnected();
extern String  getEthLocalIP();
extern String  getEthMAC();
extern String  getEthGateway();
extern String  getEthSubnet();
extern String  getEthDNS();
extern int     getEthLinkSpeed();
extern bool    isEthFullDuplex();
extern uint8_t getMeshChannel();
extern void    setMeshChannel(uint8_t);
#endif

// --- Favicon SVG (served at /favicon.ico) ---
static const char FAVICON_SVG[] PROGMEM = "<svg xmlns='http://www.w3.org/2000/svg' viewBox='4 4 32 32'><rect width='40' height='40' fill='black'/><g><path fill='%2358a6ff' d='M22.603,19.604c1.136-0.807,1.88-2.131,1.88-3.626c0-2.452-1.994-4.446-4.444-4.446c-2.451,0-4.445,1.994-4.445,4.446c0,1.495,0.745,2.82,1.881,3.627l-5.008,14.432c-0.132,0.378,0.068,0.791,0.447,0.924C12.991,34.986,13.072,35,13.15,35c0.3,0,0.581-0.188,0.684-0.488l1.02-2.933h10.37l1.017,2.933C26.344,34.813,26.626,35,26.927,35c0.078,0,0.157-0.014,0.237-0.039c0.378-0.133,0.578-0.546,0.446-0.924L22.603,19.604z M20.038,12.991c1.646,0,2.986,1.34,2.986,2.987c0,1.646-1.34,2.986-2.986,2.986c-1.647,0-2.986-1.34-2.986-2.986C17.052,14.331,18.391,12.991,20.038,12.991z M20.038,20.423c0.435,0,0.854-0.064,1.252-0.181l1.603,4.622h-5.709l1.604-4.622C19.185,20.359,19.604,20.423,20.038,20.423z M15.356,30.129l1.324-3.814h6.715l1.325,3.814H15.356z'/><path fill='%2358a6ff' d='M14.12,21.617c0.142,0.142,0.327,0.212,0.511,0.212c0.187,0,0.373-0.07,0.515-0.212c0.282-0.283,0.282-0.743,0-1.025c-1.234-1.231-1.913-2.87-1.913-4.613c0-1.743,0.678-3.381,1.911-4.613c0.282-0.283,0.282-0.742,0-1.026c-0.285-0.282-0.743-0.282-1.025,0c-1.507,1.506-2.336,3.508-2.336,5.639C11.782,18.108,12.612,20.111,14.12,21.617z'/><path fill='%2358a6ff' d='M12.581,23.154c-1.919-1.916-2.974-4.465-2.974-7.176c0-2.71,1.055-5.259,2.973-7.178c0.283-0.283,0.283-0.742,0-1.025c-0.284-0.283-0.743-0.282-1.025,0.001c-2.191,2.191-3.398,5.104-3.398,8.202s1.208,6.011,3.4,8.202c0.142,0.142,0.326,0.212,0.512,0.212s0.372-0.07,0.513-0.212C12.864,23.896,12.864,23.438,12.581,23.154z'/><path fill='%2358a6ff' d='M5.982,15.979c0-3.678,1.432-7.137,4.034-9.741c0.283-0.283,0.283-0.742,0-1.025c-0.283-0.283-0.743-0.283-1.025,0c-2.876,2.877-4.46,6.7-4.46,10.766c0,4.065,1.584,7.889,4.46,10.766c0.142,0.141,0.327,0.211,0.513,0.211s0.372-0.07,0.513-0.211c0.283-0.284,0.283-0.744,0-1.026C7.415,23.115,5.982,19.656,5.982,15.979z'/><path fill='%2358a6ff' d='M26.768,15.979c0,1.742-0.679,3.38-1.911,4.612c-0.282,0.282-0.282,0.741,0,1.024c0.142,0.142,0.328,0.213,0.514,0.213c0.185,0,0.37-0.071,0.512-0.213c1.507-1.506,2.337-3.508,2.337-5.637c0-2.131-0.831-4.134-2.338-5.64c-0.283-0.283-0.741-0.282-1.026,0c-0.282,0.283-0.282,0.742,0.001,1.025C26.089,12.596,26.768,14.235,26.768,15.979z'/><path fill='%2358a6ff' d='M28.445,7.775c-0.285-0.283-0.744-0.282-1.026,0.001c-0.284,0.283-0.284,0.742,0,1.024c1.917,1.917,2.973,4.466,2.973,7.178c0,2.709-1.055,5.258-2.971,7.178c-0.284,0.282-0.284,0.74,0,1.024c0.142,0.142,0.328,0.212,0.513,0.212c0.186,0,0.372-0.07,0.513-0.212c2.19-2.192,3.397-5.105,3.397-8.202C31.844,12.879,30.636,9.966,28.445,7.775z'/><path fill='%2358a6ff' d='M31.007,5.212c-0.282-0.282-0.741-0.282-1.025,0c-0.283,0.283-0.283,0.743,0,1.025c2.603,2.603,4.037,6.063,4.037,9.741c0,3.677-1.434,7.136-4.035,9.74c-0.283,0.283-0.283,0.741,0,1.025c0.141,0.141,0.327,0.211,0.513,0.211c0.185,0,0.371-0.07,0.513-0.211c2.876-2.879,4.459-6.701,4.459-10.766C35.468,11.912,33.884,8.089,31.007,5.212z'/></g></svg>";

// --- Packet ring buffer ---
#ifdef BOARD_HAS_PSRAM
  #define LOG_SIZE 200
#else
  #define LOG_SIZE 10
#endif

struct PacketLog {
  uint8_t       type;
  char          src[18];
  char          origSrc[18];
  uint8_t       appId;
  char          payload[65];
  unsigned long ts;
};

#ifdef BOARD_HAS_PSRAM
  static EXT_RAM_BSS_ATTR PacketLog _log[LOG_SIZE];
#else
  static PacketLog _log[LOG_SIZE];
#endif
static int       _logHead  = 0;
static int       _logCount = 0;

void logPacket(uint8_t type, uint8_t* senderMac, uint8_t* origSrcMac, uint8_t appId, const uint8_t* payload, uint8_t payloadLen) {
  PacketLog& p = _log[_logHead];
  p.type  = type;
  p.appId = appId;
  p.ts    = millis();
  snprintf(p.src, sizeof(p.src), "%02X:%02X:%02X:%02X:%02X:%02X",
           senderMac[0], senderMac[1], senderMac[2], senderMac[3], senderMac[4], senderMac[5]);
  snprintf(p.origSrc, sizeof(p.origSrc), "%02X:%02X:%02X:%02X:%02X:%02X",
           origSrcMac[0], origSrcMac[1], origSrcMac[2], origSrcMac[3], origSrcMac[4], origSrcMac[5]);
  uint8_t len = payloadLen < 64 ? payloadLen : 64;
  memcpy(p.payload, payload, len);
  p.payload[len] = '\0';
  _logHead = (_logHead + 1) % LOG_SIZE;
  if (_logCount < LOG_SIZE) _logCount++;
}

// --- Serial console log buffer (internal RAM, NOT PSRAM) ---
#define SERIAL_LOG_LINES    40
#define SERIAL_LOG_LINE_LEN 120
static char _serialLog[SERIAL_LOG_LINES][SERIAL_LOG_LINE_LEN];
static int  _serialLogHead  = 0;
static int  _serialLogCount = 0;

// --- Mutex protecting meshNodes[] and _log[] across tasks/cores ---
static SemaphoreHandle_t _dataMutex = nullptr;
void lockMeshData()   { if (_dataMutex) xSemaphoreTake(_dataMutex, portMAX_DELAY); }
void unlockMeshData() { if (_dataMutex) xSemaphoreGive(_dataMutex); }

void addSerialLog(const char* msg) {
  strncpy(_serialLog[_serialLogHead], msg, SERIAL_LOG_LINE_LEN - 1);
  _serialLog[_serialLogHead][SERIAL_LOG_LINE_LEN - 1] = '\0';
  _serialLogHead = (_serialLogHead + 1) % SERIAL_LOG_LINES;
  if (_serialLogCount < SERIAL_LOG_LINES) _serialLogCount++;
}

// --- Dashboard HTML (stored in flash) ---
static const char HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html data-theme="dark">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>UniversalMesh</title>
  <meta name="apple-mobile-web-app-capable" content="yes">
  <meta name="apple-mobile-web-app-status-bar-style" content="black-translucent">
  <meta name="apple-mobile-web-app-title" content="UniversalMesh">
  <link rel="apple-touch-icon" href="/apple-touch-icon.png">
  <link rel="manifest" href="/manifest.json">
  <link rel="icon" type="image/svg+xml" href="data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' viewBox='4 4 32 32'%3E%3Crect width='40' height='40' fill='black'/%3E%3Cg%3E%3Cpath fill='%2358a6ff' d='M22.603,19.604c1.136-0.807,1.88-2.131,1.88-3.626c0-2.452-1.994-4.446-4.444-4.446c-2.451,0-4.445,1.994-4.445,4.446c0,1.495,0.745,2.82,1.881,3.627l-5.008,14.432c-0.132,0.378,0.068,0.791,0.447,0.924C12.991,34.986,13.072,35,13.15,35c0.3,0,0.581-0.188,0.684-0.488l1.02-2.933h10.37l1.017,2.933C26.344,34.813,26.626,35,26.927,35c0.078,0,0.157-0.014,0.237-0.039c0.378-0.133,0.578-0.546,0.446-0.924L22.603,19.604z M20.038,12.991c1.646,0,2.986,1.34,2.986,2.987c0,1.646-1.34,2.986-2.986,2.986c-1.647,0-2.986-1.34-2.986-2.986C17.052,14.331,18.391,12.991,20.038,12.991z M20.038,20.423c0.435,0,0.854-0.064,1.252-0.181l1.603,4.622h-5.709l1.604-4.622C19.185,20.359,19.604,20.423,20.038,20.423z M15.356,30.129l1.324-3.814h6.715l1.325,3.814H15.356z'/%3E%3Cpath fill='%2358a6ff' d='M14.12,21.617c0.142,0.142,0.327,0.212,0.511,0.212c0.187,0,0.373-0.07,0.515-0.212c0.282-0.283,0.282-0.743,0-1.025c-1.234-1.231-1.913-2.87-1.913-4.613c0-1.743,0.678-3.381,1.911-4.613c0.282-0.283,0.282-0.742,0-1.026c-0.285-0.282-0.743-0.282-1.025,0c-1.507,1.506-2.336,3.508-2.336,5.639C11.782,18.108,12.612,20.111,14.12,21.617z'/%3E%3Cpath fill='%2358a6ff' d='M12.581,23.154c-1.919-1.916-2.974-4.465-2.974-7.176c0-2.71,1.055-5.259,2.973-7.178c0.283-0.283,0.283-0.742,0-1.025c-0.284-0.283-0.743-0.282-1.025,0.001c-2.191,2.191-3.398,5.104-3.398,8.202s1.208,6.011,3.4,8.202c0.142,0.142,0.326,0.212,0.512,0.212s0.372-0.07,0.513-0.212C12.864,23.896,12.864,23.438,12.581,23.154z'/%3E%3Cpath fill='%2358a6ff' d='M5.982,15.979c0-3.678,1.432-7.137,4.034-9.741c0.283-0.283,0.283-0.742,0-1.025c-0.283-0.283-0.743-0.283-1.025,0c-2.876,2.877-4.46,6.7-4.46,10.766c0,4.065,1.584,7.889,4.46,10.766c0.142,0.141,0.327,0.211,0.513,0.211s0.372-0.07,0.513-0.211c0.283-0.284,0.283-0.744,0-1.026C7.415,23.115,5.982,19.656,5.982,15.979z'/%3E%3Cpath fill='%2358a6ff' d='M26.768,15.979c0,1.742-0.679,3.38-1.911,4.612c-0.282,0.282-0.282,0.741,0,1.024c0.142,0.142,0.328,0.213,0.514,0.213c0.185,0,0.37-0.071,0.512-0.213c1.507-1.506,2.337-3.508,2.337-5.637c0-2.131-0.831-4.134-2.338-5.64c-0.283-0.283-0.741-0.282-1.026,0c-0.282,0.283-0.282,0.742,0.001,1.025C26.089,12.596,26.768,14.235,26.768,15.979z'/%3E%3Cpath fill='%2358a6ff' d='M28.445,7.775c-0.285-0.283-0.744-0.282-1.026,0.001c-0.284,0.283-0.284,0.742,0,1.024c1.917,1.917,2.973,4.466,2.973,7.178c0,2.709-1.055,5.258-2.971,7.178c-0.284,0.282-0.284,0.74,0,1.024c0.142,0.142,0.328,0.212,0.513,0.212c0.186,0,0.372-0.07,0.513-0.212c2.19-2.192,3.397-5.105,3.397-8.202C31.844,12.879,30.636,9.966,28.445,7.775z'/%3E%3Cpath fill='%2358a6ff' d='M31.007,5.212c-0.282-0.282-0.741-0.282-1.025,0c-0.283,0.283-0.283,0.743,0,1.025c2.603,2.603,4.037,6.063,4.037,9.741c0,3.677-1.434,7.136-4.035,9.74c-0.283,0.283-0.283,0.741,0,1.025c0.141,0.141,0.327,0.211,0.513,0.211c0.185,0,0.371-0.07,0.513-0.211c2.876-2.879,4.459-6.701,4.459-10.766C35.468,11.912,33.884,8.089,31.007,5.212z'/%3E%3C/g%3E%3C/svg%3E">
)rawliteral"
WEB_CSS
R"rawliteral(
</head>
<body>
  <nav class="navbar">
    <span class="nav-brand"><span class="dot"></span>&nbsp;UniversalMesh</span>
    <div class="nav-actions">
      <button class="theme-btn" onclick="toggleTopology()" id="topo-btn" title="Toggle mesh topology map">&#x2B21;</button>
      <button class="theme-btn" onclick="toggleConsole()" id="console-btn" title="Toggle serial console">&gt;_</button>
      <button class="theme-btn" onclick="rebootDevice()" id="reboot-btn" title="Reboot coordinator" style="color:#dc3545">&#x21BA;</button>
      <button class="theme-btn" onclick="toggleTheme()" id="theme-btn" title="Toggle dark/light mode">&#x1F319;</button>
    </div>
  </nav>
  <div class="container">
  <h1>Coordinator</h1>
  <div class="grid">
    <div class="card">
      <h2>ESP</h2>
      <div class="row"><span class="lbl">Chip</span><span class="val" id="chip">-</span></div>
      <div class="row"><span class="lbl">Cores</span><span class="val" id="cores">-</span></div>
      <div class="row"><span class="lbl">CPU</span><span class="val" id="cpu">-</span></div>
      <div class="row"><span class="lbl">Flash</span><span class="val" id="flash">-</span></div>
      <div class="row"><span class="lbl">Free Heap</span><span class="val" id="heap">-</span></div>
      <div class="row"><span class="lbl">MAC</span><span class="val" id="esp-mac">-</span></div>
      <div class="row"><span class="lbl">Uptime</span><span class="val" id="uptime">-</span></div>
      <div class="row"><span class="lbl">Time</span><span class="val" id="ntp-time">-</span></div>
    </div>
    <div class="card" id="wifi-card">
      <h2>WiFi</h2>
      <div class="row"><span class="lbl">SSID</span><span class="val" id="ssid">-</span></div>
      <div class="row"><span class="lbl">IP</span><span class="val" id="ip">-</span></div>
      <div class="row"><span class="lbl">Gateway</span><span class="val" id="gw">-</span></div>
      <div class="row"><span class="lbl">RSSI</span><span class="val" id="rssi">-</span></div>
      <div class="row"><span class="lbl">Channel</span><span class="val" id="ch">-</span></div>
    </div>
    <div class="card" id="eth-card" style="display:none">
      <h2>Ethernet</h2>
      <div class="row"><span class="lbl">Status</span><span class="val" id="eth-status">-</span></div>
      <div class="row"><span class="lbl">IP</span><span class="val" id="eth-ip">-</span></div>
      <div class="row"><span class="lbl">Subnet</span><span class="val" id="eth-subnet">-</span></div>
      <div class="row"><span class="lbl">Gateway</span><span class="val" id="eth-gw">-</span></div>
      <div class="row"><span class="lbl">DNS</span><span class="val" id="eth-dns">-</span></div>
      <div class="row"><span class="lbl">MAC</span><span class="val" id="eth-mac">-</span></div>
      <div class="row"><span class="lbl">Speed</span><span class="val" id="eth-speed">-</span></div>
    </div>
    <div class="card">
      <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:10px">
        <h2 style="margin-bottom:0">Mesh Nodes</h2>
        <button id="ping-all-btn" onclick="pingAll()" class="btn btn-primary">Ping All</button>
      </div>
      <div id="nodes-empty" class="empty">No nodes discovered yet</div>
      <table id="nodes-table" style="display:none;table-layout:fixed;width:100%">
        <colgroup><col style="width:auto"><col style="width:100px"></colgroup>
        <thead><tr><th onclick="sortNodes('name')" style="cursor:pointer;user-select:none">Node <span id="sort-node-icon"></span></th><th onclick="sortNodes('seen')" style="cursor:pointer;user-select:none;text-align:right">Last Seen <span id="sort-seen-icon">&#9650;</span></th></tr></thead>
        <tbody id="nodes-body"></tbody>
      </table>
    </div>
    <div class="card" id="mesh-card" style="display:none">
      <h2>Mesh</h2>
      <div class="row">
        <span class="lbl">Channel</span>
        <select id="mesh-ch-sel" class="sel">
          <option value="1">1</option><option value="2">2</option><option value="3">3</option>
          <option value="4">4</option><option value="5">5</option><option value="6">6</option>
          <option value="7">7</option><option value="8">8</option><option value="9">9</option>
          <option value="10">10</option><option value="11">11</option><option value="12">12</option>
          <option value="13">13</option>
        </select>
      </div>
      <div class="action-buttons-vertical">
        <button onclick="saveMeshCh(false)" class="btn btn-success" id="mesh-ch-btn">Save</button>
        <button onclick="saveMeshCh(true)" class="btn btn-danger" id="mesh-ch-reboot-btn">Save &amp; Reboot</button>
      </div>
    </div>
    <div class="card">
      <h2>Send Message</h2>
      <div class="row" style="margin-bottom:8px">
        <span class="lbl">To</span>
        <select id="msg-dest" class="sel" style="flex:1;min-width:0;overflow:hidden;text-overflow:ellipsis">
          <option value="FF:FF:FF:FF:FF:FF">Broadcast — FF:FF:FF:FF:FF:FF</option>
        </select>
      </div>
      <div class="row" style="margin-bottom:8px">
        <span class="lbl">Text</span>
        <input id="msg-text" class="sel" style="flex:1;min-width:0" placeholder="Hello mesh..." maxlength="64" onkeydown="if(event.key==='Enter')sendMsg()">
      </div>
      <div class="action-buttons-vertical">
        <button id="msg-send-btn" onclick="sendMsg()" class="btn btn-primary">Send (App&nbsp;0x01)</button>
      </div>
      <div id="msg-status" style="margin-top:6px;font-size:0.85em;color:var(--sub);text-align:center"></div>
    </div>
  </div>
  <div class="card">
    <h2>Live Packet Log <span id="log-pageinfo" style="float:right;font-size:0.8em;color:var(--sub)"></span></h2>
    <div id="log-empty" class="empty">No packets yet</div>
    <div style="overflow-x:auto">
    <table id="log-table" style="display:none;min-width:480px">
      <thead><tr><th>Type</th><th>From</th><th>App</th><th>Payload</th><th id="log-time-hdr">Age</th></tr></thead>
      <tbody id="log-body"></tbody>
    </table>
    </div>
    <div style="display:flex;justify-content:space-between;align-items:center;margin-top:8px">
      <div>
        <button id="log-prev" onclick="logPage(-1)" class="btn btn-secondary">&laquo; Prev</button>
        <button id="log-next" onclick="logPage(1)"  class="btn btn-secondary">Next &raquo;</button>
      </div>
      <div class="sub" id="tick"></div>
    </div>
  </div>
  <div id="topo-panel" style="display:none;margin-top:15px">
    <div class="card">
      <h2>Mesh Topology</h2>
      <div style="display:flex;gap:6px;margin-bottom:6px;flex-wrap:wrap">
        <button class="btn btn-secondary" onclick="topoFreeze()" id="topo-freeze-btn" title="Freeze/unfreeze layout">&#9646;&#9646; Freeze</button>
        <button class="btn btn-secondary" onclick="topoReset()" title="Reset layout">&#8635; Reset</button>
        <button class="btn btn-secondary" onclick="topoToggleMac()" id="topo-mac-btn" title="Toggle MAC labels" style="opacity:0.55">MAC</button>
        <button class="btn btn-secondary" onclick="topoToggleAge()" id="topo-age-btn" title="Toggle edge age labels" style="opacity:0.55">Age</button>
        <button class="btn btn-secondary" onclick="topoExportPng()" title="Export as PNG">&#8681; PNG</button>
      </div>
      <canvas id="topo-canvas" width="700" height="340" style="width:100%;display:block;border-radius:4px;background:var(--bg);cursor:grab"></canvas>
      <div id="topo-info" style="min-height:2.5em;margin-top:8px;padding:8px;border-radius:4px;background:var(--row-bg);font-size:0.88em;line-height:1.6"></div>
      <div style="margin-top:4px;font-size:0.8em;color:var(--sub)">Inferred from relay paths in packet log &mdash; blue = coordinator &middot; green = node &middot; click to select &middot; drag to pin &middot; double-click to unpin</div>
    </div>
  </div>
  <div id="console-panel" style="display:none;margin-top:15px">
    <div class="card">
      <h2>&gt;_ Serial Console</h2>
      <div id="console-out" class="console-out"></div>
    </div>
  </div>
  </div>
  <script>
    async function rebootDevice(){
      const btn=document.getElementById('reboot-btn');
      if(!confirm('Reboot the coordinator?')) return;
      btn.textContent='…';btn.disabled=true;
      try{ await fetch('/api/reboot',{method:'POST'}); }catch(e){}
      btn.textContent='↺';btn.disabled=false;
    }
    async function sendMsg(){
      const text=document.getElementById('msg-text').value.trim();
      if(!text) return;
      const dest=document.getElementById('msg-dest').value;
      let hex='';
      const bytes=Math.min(text.length,64);
      for(let i=0;i<bytes;i++) hex+=text.charCodeAt(i).toString(16).padStart(2,'0');
      const btn=document.getElementById('msg-send-btn');
      const status=document.getElementById('msg-status');
      btn.disabled=true; status.textContent='Sending…';
      try{
        const r=await fetch('/api/tx',{method:'POST',headers:{'Content-Type':'application/json'},
          body:JSON.stringify({dest,appId:1,payload:hex,ttl:4})});
        const d=await r.json();
        status.style.color='#28a745';
        status.textContent=d.status==='data_sent'?'Sent!':'Error';
        if(d.status==='data_sent') document.getElementById('msg-text').value='';
      }catch(e){
        status.style.color='#dc3545';
        status.textContent='Failed';
      }
      btn.disabled=false;
      setTimeout(()=>{status.textContent='';status.style.color='var(--sub)';},2500);
    }
    function toggleTheme(){
      const isDark=document.documentElement.getAttribute('data-theme')==='dark';
      const next=isDark?'light':'dark';
      document.documentElement.setAttribute('data-theme',next);
      document.getElementById('theme-btn').textContent=isDark?'☀️':'🌙';
      localStorage.setItem('theme',next);
    }
    (function(){
      const t=localStorage.getItem('theme')||'dark';
      document.getElementById('theme-btn').textContent=t==='dark'?'🌙':'☀️';
    })();
    const PAGE_SIZE=10;
    let logPage_=0;
    let logPackets_=[];
    let coordMac_='';
    let nodeNames_={};
    let nodeSort_={col:'seen',asc:true};
    let lastNodes_=[];
    function sortNodes(col){
      if(nodeSort_.col===col) nodeSort_.asc=!nodeSort_.asc;
      else{ nodeSort_.col=col; nodeSort_.asc=(col==='seen'); }
      document.getElementById('sort-node-icon').innerHTML=nodeSort_.col==='name'?(nodeSort_.asc?'&#9650;':'&#9660;'):'';
      document.getElementById('sort-seen-icon').innerHTML=nodeSort_.col==='seen'?(nodeSort_.asc?'&#9650;':'&#9660;'):'';
      renderNodes(lastNodes_);
    }
    function renderNodes(nodes){
      lastNodes_=nodes;
      const nb=document.getElementById('nodes-body');
      nb.innerHTML='';
      if(!nodes||!nodes.length){ document.getElementById('nodes-empty').style.display=''; document.getElementById('nodes-table').style.display='none'; return; }
      document.getElementById('nodes-empty').style.display='none';
      document.getElementById('nodes-table').style.display='';
      const blueDot='<span style="display:inline-block;width:8px;height:8px;border-radius:50%;background:#58a6ff;margin-right:6px"></span>';
      nb.innerHTML='<tr><td>'+blueDot+'<span style="color:#58a6ff;font-weight:bold">coordinator</span><br><span style="font-size:0.85em;color:var(--muted)">'+coordMac_+'</span></td><td style="text-align:right;white-space:nowrap">-</td></tr>';
      const others=nodes.filter(n=>n.mac.toUpperCase()!==coordMac_).sort((a,b)=>{
        if(nodeSort_.col==='name'){
          const na=(a.name||a.mac).toLowerCase(), nb2=(b.name||b.mac).toLowerCase();
          return nodeSort_.asc?na.localeCompare(nb2):nb2.localeCompare(na);
        } else {
          return nodeSort_.asc?a.last_seen_seconds_ago-b.last_seen_seconds_ago:b.last_seen_seconds_ago-a.last_seen_seconds_ago;
        }
      });
      others.forEach(n=>{
        const dot='<span style="display:inline-block;width:8px;height:8px;border-radius:50%;background:'+(n.last_seen_seconds_ago<=120?'#3fb950':'#f85149')+';margin-right:6px"></span>';
        const node=n.name?'<span style="color:#58a6ff;font-weight:bold">'+n.name+'</span><br><span style="font-size:0.85em;color:var(--muted)">'+n.mac+'</span>':n.mac;
        nb.innerHTML+='<tr><td>'+dot+node+'</td><td style="text-align:right;white-space:nowrap">'+n.last_seen_seconds_ago+'s ago</td></tr>';
      });
    }
    function logPage(dir){
      logPage_=Math.max(0,Math.min(logPage_+dir,Math.ceil(logPackets_.length/PAGE_SIZE)-1));
      renderLog();
    }
    function renderLog(){
      const lb=document.getElementById('log-body');
      lb.innerHTML='';
      if(!logPackets_.length){
        document.getElementById('log-empty').style.display='';
        document.getElementById('log-table').style.display='none';
        document.getElementById('log-pageinfo').textContent='';
        document.getElementById('log-prev').style.visibility='hidden';
        document.getElementById('log-next').style.visibility='hidden';
        return;
      }
      const total=Math.ceil(logPackets_.length/PAGE_SIZE);
      const page=logPackets_.slice(logPage_*PAGE_SIZE,(logPage_+1)*PAGE_SIZE);
      document.getElementById('log-empty').style.display='none';
      document.getElementById('log-table').style.display='';
      document.getElementById('log-pageinfo').textContent=(logPage_+1)+' / '+total;
      document.getElementById('log-prev').style.visibility=logPage_>0?'visible':'hidden';
      document.getElementById('log-next').style.visibility=logPage_<total-1?'visible':'hidden';
      page.forEach(p=>{
        const relayed=p.origSrc&&p.src.toUpperCase()!==p.origSrc.toUpperCase();
        lb.innerHTML+='<tr'+(relayed?' class="relayed"':'')+'>'
          +'<td><span class="tag">'+({0x12:'PING',0x13:'PONG',0x15:'DATA'}[p.type]||'0x'+p.type.toString(16).padStart(2,'0'))+'</span></td>'
          +'<td>'+(nodeNames_[p.src.toUpperCase()]||p.src)+'</td>'
          +'<td>0x'+p.appId.toString(16).padStart(2,'0')+'</td>'
          +'<td>'+(p.appId===0x06?'[announce] '+p.payload:p.appId===0x05?'[heartbeat] '+(nodeNames_[p.origSrc.toUpperCase()]||nodeNames_[p.src.toUpperCase()]||p.origSrc):p.appId===0x00?({0x12:'[discovery ping]',0x13:'[discovery pong]'}[p.type]||'[discovery]')+(p.origSrc!==p.src?' <span class="muted">from '+(p.origSrc.toUpperCase()===coordMac_?'[coordinator]':(nodeNames_[p.origSrc.toUpperCase()]||p.origSrc))+'</span>':''):p.payload)+'</td>'
          +'<td>'+(p._time?p._time.toTimeString().slice(0,8):p.age_s+'s ago')+'</td>'
          +'</tr>';
      });
    }
    function pingAll(){
      const btn=document.getElementById('ping-all-btn');
      btn.textContent='...';btn.disabled=true;
      fetch('/api/discover').then(()=>{
        btn.textContent='sent!';
        setTimeout(()=>{btn.textContent='ping all';btn.disabled=false;},2000);
      }).catch(()=>{btn.textContent='ping all';btn.disabled=false;});
    }
    function fmtUptime(ms){
      let s=Math.floor(ms/1000),m=Math.floor(s/60),h=Math.floor(m/60),d=Math.floor(h/24);
      return d?d+'d '+h%24+'h':h?h+'h '+m%60+'m':m?m+'m '+s%60+'s':s+'s';
    }
    function fmtBytes(b){return b>1048576?(b/1048576).toFixed(1)+' MB':b>1024?(b/1024).toFixed(1)+' KB':b+' B'}
    function set(id,v){document.getElementById(id).textContent=v}
    let _ntpRef=null; // Date object representing "now" at last fetch, null if not synced
    function fmtPacketTime(age_s){
      if(!_ntpRef) return age_s+'s ago';
      const t=new Date(_ntpRef.getTime()-age_s*1000);
      return t.toTimeString().slice(0,8); // HH:MM:SS
    }

    // --- Mesh topology force graph ---
    let _topoRunning=false,_topoAnim=null,_topoNodes=[],_topoEdges=[],_topoMap=new Map(),_topoSel=null,_stCache={};
    let _topoPanX=0,_topoPanY=0,_topoDrag=null,_topoPanning=false,_topoPanLast={x:0,y:0},_topoFrozen=false,_topoDragMoved=false,_topoShowMac=false,_topoShowAge=false;
    function toggleTopology(){
      const p=document.getElementById('topo-panel');
      const show=p.style.display==='none';
      p.style.display=show?'':'none';
      document.getElementById('topo-btn').style.opacity=show?'1':'0.5';
      if(show){
        _topoNodes=[];_topoEdges=[];_topoMap=new Map();_topoSel=null;
        mergeTopology(); fixCoord(); _topoRunning=true; topoTick();
        const cv=document.getElementById('topo-canvas');
        cv.addEventListener('mousedown',e=>{
          e.preventDefault();
          const p=topoCanvasCoords(e,cv);
          const hit=topoHitNode(p.x,p.y);
          _topoDragMoved=false;
          if(hit){_topoDrag=hit;cv.style.cursor='grabbing';}
          else{_topoPanning=true;_topoPanLast={x:e.clientX,y:e.clientY};cv.style.cursor='grabbing';}
        });
        cv.addEventListener('mousemove',e=>{
          if(_topoDrag){
            _topoDragMoved=true;
            const p=topoCanvasCoords(e,cv);
            _topoDrag.x=p.x;_topoDrag.y=p.y;_topoDrag.vx=0;_topoDrag.vy=0;_topoDrag.fixed=true;
          } else if(_topoPanning){
            _topoDragMoved=true;
            const s=cv.width/cv.getBoundingClientRect().width;
            _topoPanX+=(e.clientX-_topoPanLast.x)*s;
            _topoPanY+=(e.clientY-_topoPanLast.y)*s;
            _topoPanLast={x:e.clientX,y:e.clientY};
            topoClampView(cv);
          }
        });
        cv.addEventListener('mouseup',e=>{
          if(!_topoDragMoved){
            const p=topoCanvasCoords(e,cv);
            const hit=topoHitNode(p.x,p.y);
            _topoSel=(hit===_topoSel)?null:hit;
            renderTopoInfo();
          }
          _topoDrag=null;_topoPanning=false;cv.style.cursor='grab';
        });
        cv.addEventListener('mouseleave',()=>{_topoDrag=null;_topoPanning=false;cv.style.cursor='grab';});
        cv.addEventListener('dblclick',e=>{
          const p=topoCanvasCoords(e,cv);
          const hit=topoHitNode(p.x,p.y);
          if(hit&&!hit.isCoord){hit.fixed=false;hit.vx=0;hit.vy=0;}
        });
      } else { _topoRunning=false; if(_topoAnim) cancelAnimationFrame(_topoAnim); }
    }
    function topoCanvasCoords(e,cv){
      const r=cv.getBoundingClientRect();
      const sx=(e.clientX-r.left)*(cv.width/r.width);
      const sy=(e.clientY-r.top)*(cv.height/r.height);
      return {x:sx-_topoPanX,y:sy-_topoPanY};
    }
    function topoClampView(cv){
      if(!_topoNodes.length) return;
      let x0=Infinity,x1=-Infinity,y0=Infinity,y1=-Infinity;
      _topoNodes.forEach(n=>{x0=Math.min(x0,n.x);x1=Math.max(x1,n.x);y0=Math.min(y0,n.y);y1=Math.max(y1,n.y);});
      const pad=50,mg=60;
      x0-=pad;x1+=pad;y0-=pad;y1+=pad;
      // minimum zoom = zoom that fits all nodes in canvas (can't zoom out further than that)
      // clamp pan so nodes can't scroll off screen
      _topoPanX=Math.min(_topoPanX,(cv.width-mg)-x0);
      _topoPanX=Math.max(_topoPanX,mg-x1);
      _topoPanY=Math.min(_topoPanY,(cv.height-mg)-y0);
      _topoPanY=Math.max(_topoPanY,mg-y1);
    }
    function topoHitNode(cx,cy){
      let hit=null;
      _topoNodes.forEach(n=>{const rd=n.isCoord?15:10,dx=n.x-cx,dy=n.y-cy;if(dx*dx+dy*dy<=(rd+5)*(rd+5))hit=n;});
      return hit;
    }
    function topoFreeze(){
      _topoFrozen=!_topoFrozen;
      const btn=document.getElementById('topo-freeze-btn');
      btn.innerHTML=_topoFrozen?'&#9658; Resume':'&#9646;&#9646; Freeze';
      btn.style.opacity=_topoFrozen?'0.65':'1';
    }
    function topoToggleMac(){
      _topoShowMac=!_topoShowMac;
      const btn=document.getElementById('topo-mac-btn');
      btn.style.opacity=_topoShowMac?'1':'0.55';btn.style.fontWeight=_topoShowMac?'bold':'normal';
    }
    function topoToggleAge(){
      _topoShowAge=!_topoShowAge;
      const btn=document.getElementById('topo-age-btn');
      btn.style.opacity=_topoShowAge?'1':'0.55';btn.style.fontWeight=_topoShowAge?'bold':'normal';
    }
    function topoExportPng(){
      const cv=document.getElementById('topo-canvas');
      const tmp=document.createElement('canvas');
      tmp.width=cv.width;tmp.height=cv.height;
      const tc=tmp.getContext('2d');
      const dark=document.documentElement.getAttribute('data-theme')==='dark';
      tc.fillStyle=dark?'#0d1117':'#ffffff';
      tc.fillRect(0,0,tmp.width,tmp.height);
      tc.drawImage(cv,0,0);
      tmp.toBlob(blob=>{
        const url=URL.createObjectURL(blob);
        const a=document.createElement('a');
        a.download='topology.png';a.href=url;
        document.body.appendChild(a);a.click();document.body.removeChild(a);
        setTimeout(()=>URL.revokeObjectURL(url),1000);
      },'image/png');
    }
    function topoReset(){
      _topoPanX=0;_topoPanY=0;
      _topoNodes.forEach(n=>{if(!n.isCoord){n.fixed=false;n.vx=0;n.vy=0;}});
      fixCoord();
      if(_topoFrozen){_topoFrozen=false;const btn=document.getElementById('topo-freeze-btn');btn.innerHTML='&#9646;&#9646; Freeze';btn.style.opacity='1';}
    }
function fixCoord(){
      const cv=document.getElementById('topo-canvas');
      const c=_topoMap.get(coordMac_);
      if(c){c.x=cv.width/2;c.y=cv.height/2;c.fixed=true;}
    }
    function renderTopoInfo(){
      const div=document.getElementById('topo-info');
      if(!_topoSel){div.innerHTML='';return;}
      const n=_topoSel;
      let h='<strong style="color:'+(n.isCoord?'#58a6ff':'#3fb950')+'">'+n.name+'</strong>';
      h+=' &nbsp;<span style="font-size:0.85em;color:var(--sub)">'+n.id+'</span><br>';
      if(n.isCoord && _stCache.chip){
        h+=_stCache.chip+' &middot; '+_stCache.cores+' cores &middot; '+_stCache.cpu_mhz+' MHz<br>';
        h+='Heap: '+fmtBytes(_stCache.free_heap)+' &middot; Up: '+fmtUptime(_stCache.uptime_ms)+'<br>';
        if(_stCache.wifi_connected && _stCache.ip)
          h+='IP: '+_stCache.ip+' &middot; SSID: '+(_stCache.ssid||'-');
        else
          h+='MAC: '+(_stCache.esp_mac||'-')+' &middot; Ch: '+(_stCache.channel||'-');
      } else {
        const pkts=logPackets_.filter(p=>p.origSrc.toUpperCase()===n.id||p.src.toUpperCase()===n.id);
        if(pkts.length){
          h+='Last seen: '+pkts[0].age_s+'s ago<br>';
          const data=pkts.filter(p=>p.appId===1&&p.payload&&p.origSrc.toUpperCase()===n.id);
          if(data.length) h+='Last payload: <span style="color:var(--sub)">'+data[0].payload+'</span>';
        } else { h+='No packets in log yet'; }
      }
      div.innerHTML=h;
    }
    function updateTopoLastSeen(nodes){
      if(!nodes) return;
      nodes.forEach(n=>{
        const node=_topoMap.get(n.mac.toUpperCase());
        if(node) node.lastSeen=n.last_seen_seconds_ago;
      });
    }
    function mergeTopology(){
      const cv=document.getElementById('topo-canvas');
      const coord=coordMac_.toUpperCase();
      const edgeSet=new Set(_topoEdges.map(e=>[e[0],e[1]].sort().join('~')));
      const addNode=mac=>{
        mac=mac.toUpperCase();
        if(!_topoMap.has(mac)){
          const n={id:mac,name:mac===coord?'coordinator':(nodeNames_[mac]||mac.slice(-5)),
            isCoord:mac===coord,x:cv.width/2+(Math.random()-.5)*220,y:cv.height/2+(Math.random()-.5)*180,vx:0,vy:0};
          _topoMap.set(mac,n); _topoNodes.push(n);
        } else { const n=_topoMap.get(mac); if(nodeNames_[mac]) n.name=nodeNames_[mac]; }
        return mac;
      };
      const addEdge=(a,b)=>{ const k=[a,b].sort().join('~'); if(!edgeSet.has(k)){edgeSet.add(k);_topoEdges.push([a,b]);} };
      addNode(coord);
      logPackets_.forEach(p=>{ const s=addNode(p.src),o=addNode(p.origSrc); addEdge(s,coord); if(s!==o) addEdge(o,s); });
    }
    function topoTick(){
      if(!_topoRunning) return;
      const cv=document.getElementById('topo-canvas');
      const ctx=cv.getContext('2d');
      const ns=_topoNodes, es=_topoEdges, nm=_topoMap;
      if(!_topoFrozen){
        ns.forEach(n=>{n.fx=0;n.fy=0;});
        for(let i=0;i<ns.length;i++) for(let j=i+1;j<ns.length;j++){
          const a=ns[i],b=ns[j],dx=b.x-a.x,dy=b.y-a.y,d2=dx*dx+dy*dy+1,d=Math.sqrt(d2),f=6000/d2;
          a.fx-=f*dx/d;a.fy-=f*dy/d;b.fx+=f*dx/d;b.fy+=f*dy/d;
        }
        es.forEach(([ai,bi])=>{
          const a=nm.get(ai),b=nm.get(bi); if(!a||!b) return;
          const dx=b.x-a.x,dy=b.y-a.y,d=Math.sqrt(dx*dx+dy*dy)||1,f=0.06*(d-130);
          a.fx+=f*dx/d;a.fy+=f*dy/d;b.fx-=f*dx/d;b.fy-=f*dy/d;
        });
        const ccx=cv.width/2,ccy=cv.height/2;
        ns.forEach(n=>{n.fx+=(ccx-n.x)*.006;n.fy+=(ccy-n.y)*.006;});
        ns.forEach(n=>{
          if(n.fixed) return;
          n.vx=(n.vx+n.fx)*.72; n.vy=(n.vy+n.fy)*.72;
          n.x=Math.max(55,Math.min(cv.width-55,n.x+n.vx));
          n.y=Math.max(22,Math.min(cv.height-22,n.y+n.vy));
        });
      }
      const dark=document.documentElement.getAttribute('data-theme')==='dark';
      ctx.clearRect(0,0,cv.width,cv.height);
      ctx.save();
      ctx.translate(_topoPanX,_topoPanY);
      ctx.strokeStyle=dark?'#3a3a5c':'#bbb'; ctx.lineWidth=1.5;
      es.forEach(([ai,bi])=>{
        const a=nm.get(ai),b=nm.get(bi); if(!a||!b) return;
        ctx.beginPath();ctx.moveTo(a.x,a.y);ctx.lineTo(b.x,b.y);ctx.stroke();
        if(_topoShowAge){
          const node=a.isCoord?b:a;
          if(node.lastSeen!=null){
            const mx=(a.x+b.x)/2,my=(a.y+b.y)/2;
            const lbl=node.lastSeen+'s';
            ctx.font='9px monospace';ctx.textAlign='center';
            ctx.fillStyle=dark?'#30363d':'#e0e0e0';
            ctx.fillRect(mx-14,my-9,28,12);
            ctx.fillStyle=dark?'#8b949e':'#555';
            ctx.fillText(lbl,mx,my);
          }
        }
      });
      ns.forEach(n=>{
        const r=n.isCoord?15:10;
        if(n===_topoSel){
          ctx.beginPath();ctx.arc(n.x,n.y,r+5,0,Math.PI*2);
          ctx.strokeStyle='#f0c040';ctx.lineWidth=2.5;ctx.stroke();
        }
        ctx.beginPath();ctx.arc(n.x,n.y,r,0,Math.PI*2);
        ctx.fillStyle=n.isCoord?'#58a6ff':(n.lastSeen>120?'#f85149':'#3fb950');
        ctx.fill(); ctx.strokeStyle=dark?'#1a1a2e':'#fff'; ctx.lineWidth=2; ctx.stroke();
        if(n.fixed&&!n.isCoord){
          ctx.beginPath();ctx.arc(n.x,n.y-r-2,3,0,Math.PI*2);
          ctx.fillStyle='#f0c040';ctx.fill();
        }
        ctx.fillStyle=dark?'#e6edf3':'#1a1a2e'; ctx.font='bold 11px monospace'; ctx.textAlign='center';
        ctx.fillText(n.name,n.x,n.y+r+14);
        if(_topoShowMac&&!n.isCoord){
          ctx.fillStyle=dark?'#8b949e':'#888'; ctx.font='9px monospace';
          ctx.fillText(n.id,n.x,n.y+r+25);
        }
      });
      ctx.restore();
      _topoAnim=requestAnimationFrame(topoTick);
    }

    async function refreshSlow(){
      try{
        const st=await fetch('/api/status').then(r=>r.json());
        set('chip',    st.chip);
        set('cores',   st.cores);
        set('cpu',     st.cpu_mhz+' MHz');
        set('flash',   fmtBytes(st.flash_size));
        set('heap',    fmtBytes(st.free_heap));
        set('esp-mac', st.esp_mac);
        set('uptime',  fmtUptime(st.uptime_ms));
        set('ntp-time',st.ntp_synced?st.ntp_time:'syncing\u2026');
        set('ssid',    st.ssid);
        set('ip',      st.ip);
        set('gw',      st.gateway);
        set('rssi',    st.rssi+' dBm');
        set('ch',      st.channel);
        document.getElementById('wifi-card').style.display=st.wifi_connected?'':'none';
        if(st.eth_present){
          document.getElementById('eth-card').style.display='';
          set('eth-status',st.eth_connected?'\uD83D\uDFE2 Connected':'\uD83D\uDD34 Disconnected');
          set('eth-ip',    st.eth_ip);
          set('eth-subnet',st.eth_subnet);
          set('eth-gw',    st.eth_gateway);
          set('eth-dns',   st.eth_dns);
          set('eth-mac',   st.eth_mac);
          set('eth-speed', st.eth_connected?st.eth_speed_mbps+'Mbps '+(st.eth_full_duplex?'Full':'Half')+'-Duplex':'-');
          document.getElementById('mesh-card').style.display='';
          const sel=document.getElementById('mesh-ch-sel');
          if(sel&&st.mesh_channel&&!meshChDirty) sel.value=st.mesh_channel;
        }
        _stCache=st;
        _ntpRef=st.ntp_synced?new Date(st.ntp_time.replace(' ','T')):null;
        document.getElementById('log-time-hdr').textContent='Time';
        coordMac_=st.esp_mac.toUpperCase();
      }catch(e){}
    }
    async function refreshFast(){
      try{
        const [nd,lg]=await Promise.all([
          fetch('/api/nodes').then(r=>r.json()),
          fetch('/api/log').then(r=>r.json())
        ]);
        renderNodes(nd.nodes||[]);
        nodeNames_={};
        if(nd.nodes) nd.nodes.forEach(n=>{ if(n.name) nodeNames_[n.mac.toUpperCase()]=n.name; });
        const destSel=document.getElementById('msg-dest');
        const prevDest=destSel.value;
        destSel.innerHTML='<option value="FF:FF:FF:FF:FF:FF">Broadcast \u2014 FF:FF:FF:FF:FF:FF</option>';
        if(nd.nodes) nd.nodes.filter(n=>n.mac.toUpperCase()!==coordMac_).forEach(n=>{
          const opt=document.createElement('option');
          opt.value=n.mac;
          opt.textContent=(n.name||n.mac)+(n.name?' \u2014 '+n.mac:'');
          destSel.appendChild(opt);
        });
        if([...destSel.options].some(o=>o.value===prevDest)) destSel.value=prevDest;
        const _fetchNow=Date.now();
        logPackets_=lg.packets?lg.packets.slice().reverse().map(p=>({...p,_time:new Date(_fetchNow-p.age_s*1000)})):[];
        renderLog();
        if(_topoRunning){ mergeTopology(); updateTopoLastSeen(nd.nodes); renderTopoInfo(); }
        set('tick','last update: '+new Date().toLocaleTimeString());
      }catch(e){}
    }
    let meshChDirty=false;
    document.addEventListener('change',e=>{if(e.target.id==='mesh-ch-sel') meshChDirty=true;});
    async function saveMeshCh(reboot){
      const ch=parseInt(document.getElementById('mesh-ch-sel').value);
      const btn=reboot?document.getElementById('mesh-ch-reboot-btn'):document.getElementById('mesh-ch-btn');
      btn.textContent='...';btn.disabled=true;
      try{
        await fetch('/api/mesh/channel',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({channel:ch,reboot:reboot})});
        btn.textContent=reboot?'rebooting…':'saved!';
        meshChDirty=false;
      }catch(e){btn.textContent='error';}
      if(!reboot) setTimeout(()=>{btn.textContent='save';btn.disabled=false;},2000);
    }
    let _consoleOpen=false,_consoleInterval=null;
    async function refreshConsole(){
      try{
        const d=await fetch('/api/serial').then(r=>r.json());
        const el=document.getElementById('console-out');
        el.textContent=(d.lines||[]).join('\n');
        el.scrollTop=el.scrollHeight;
      }catch(e){}
    }
    function toggleConsole(){
      _consoleOpen=!_consoleOpen;
      document.getElementById('console-panel').style.display=_consoleOpen?'':'none';
      document.getElementById('console-btn').style.opacity=_consoleOpen?'1':'0.5';
      if(_consoleOpen){refreshConsole();_consoleInterval=setInterval(refreshConsole,2000);}
      else{clearInterval(_consoleInterval);_consoleInterval=null;}
    }
    refreshSlow(); refreshFast();
    setInterval(refreshSlow,5000);
    setInterval(refreshFast,1000);
  </script>
</body>
</html>
)rawliteral";

// --- Endpoints ---
void initWebDashboard(AsyncWebServer& server) {
  _dataMutex = xSemaphoreCreateMutex();

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, "text/html", HTML);
  });

  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest* request) {
    String json = "{";
    json += "\"chip\":\""       + String(ESP.getChipModel())        + "\",";
    json += "\"cores\":"        + String(ESP.getChipCores())        + ",";
    json += "\"cpu_mhz\":"      + String(ESP.getCpuFreqMHz())       + ",";
    json += "\"flash_size\":"   + String(ESP.getFlashChipSize())    + ",";
    json += "\"free_heap\":"    + String(ESP.getFreeHeap())         + ",";
    json += "\"esp_mac\":\""    + String(WiFi.macAddress())         + "\",";
    json += "\"uptime_ms\":"    + String(millis())                  + ",";
    json += "\"ssid\":\""       + WiFi.SSID()                       + "\",";
    json += "\"ip\":\""         + WiFi.localIP().toString()         + "\",";
    json += "\"gateway\":\""    + WiFi.gatewayIP().toString()       + "\",";
    json += "\"wifi_connected\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false") + ",";
    json += "\"rssi\":"         + String(WiFi.RSSI())               + ",";
    json += "\"channel\":"      + String(WiFi.channel())            + ",";
#ifdef LILYGO_T_ETH_ELITE
    bool ethOk = isEthConnected();
    json += "\"eth_present\":true,";
    json += "\"eth_connected\":"    + String(ethOk ? "true" : "false")      + ",";
    json += "\"eth_ip\":\""         + (ethOk ? getEthLocalIP() : "")        + "\",";
    json += "\"eth_mac\":\""        + getEthMAC()                           + "\",";
    json += "\"eth_gateway\":\""    + (ethOk ? getEthGateway() : "")        + "\",";
    json += "\"eth_subnet\":\""     + (ethOk ? getEthSubnet() : "")         + "\",";
    json += "\"eth_dns\":\""        + (ethOk ? getEthDNS() : "")            + "\",";
    json += "\"eth_speed_mbps\":"   + String(ethOk ? getEthLinkSpeed() : 0) + ",";
    json += "\"eth_full_duplex\":"  + String(ethOk && isEthFullDuplex() ? "true" : "false") + ",";
    json += "\"mesh_channel\":"     + String(getMeshChannel()) + ",";
    json += "\"ntp_synced\":"       + String(isNtpSynced() ? "true" : "false") + ",";
    json += "\"ntp_time\":\""       + getNtpTimeStr() + "\"";
#else
    json += "\"eth_present\":false";
#endif
    json += "}";
    request->send(200, "application/json", json);
  });

#ifdef LILYGO_T_ETH_ELITE
  server.on("/api/mesh/channel", HTTP_POST,
    [](AsyncWebServerRequest* request) {},
    nullptr,
    [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t, size_t) {
      JsonDocument doc;
      deserializeJson(doc, data, len);
      uint8_t ch = doc["channel"] | 1;
      bool reboot = doc["reboot"] | false;
      setMeshChannel(ch);
      request->send(200, "application/json", "{\"channel\":" + String(ch) + ",\"reboot\":" + String(reboot?"true":"false") + "}");
      if (reboot) { delay(200); ESP.restart(); }
    }
  );
#endif

  server.on("/manifest.json", HTTP_GET, [](AsyncWebServerRequest* request) {
    String json = "{";
    json += "\"name\":\"UniversalMesh\",";
    json += "\"short_name\":\"UniversalMesh\",";
    json += "\"start_url\":\"/\",";
    json += "\"display\":\"standalone\",";
    json += "\"background_color\":\"#1a1a2e\",";
    json += "\"theme_color\":\"#1a1a2e\",";
    json += "\"icons\":[";
    json += "{\"src\":\"/apple-touch-icon.png\",\"sizes\":\"180x180\",\"type\":\"image/png\",\"purpose\":\"any\"},";
    json += "{\"src\":\"/pwa-icon-192.png\",\"sizes\":\"192x192\",\"type\":\"image/png\",\"purpose\":\"any\"},";
    json += "{\"src\":\"/pwa-icon-512.png\",\"sizes\":\"512x512\",\"type\":\"image/png\",\"purpose\":\"any\"}";
    json += "]}";
    request->send(200, "application/json", json);
  });

  server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest* request) {
    AsyncWebServerResponse* response = request->beginResponse(200, "image/svg+xml", FPSTR(FAVICON_SVG));
    response->addHeader("Cache-Control", "max-age=86400");
    request->send(response);
  });

  server.on("/apple-touch-icon.png", HTTP_GET, [](AsyncWebServerRequest* request) {
    AsyncWebServerResponse* response = request->beginResponse(200, "image/png", TOUCH_ICON_PNG, TOUCH_ICON_PNG_LEN);
    response->addHeader("Cache-Control", "max-age=86400");
    request->send(response);
  });

  server.on("/pwa-icon-192.png", HTTP_GET, [](AsyncWebServerRequest* request) {
    AsyncWebServerResponse* response = request->beginResponse(200, "image/png", PWA_ICON_192_PNG, PWA_ICON_192_PNG_LEN);
    response->addHeader("Cache-Control", "max-age=86400");
    request->send(response);
  });

  server.on("/pwa-icon-512.png", HTTP_GET, [](AsyncWebServerRequest* request) {
    AsyncWebServerResponse* response = request->beginResponse(200, "image/png", PWA_ICON_512_PNG, PWA_ICON_512_PNG_LEN);
    response->addHeader("Cache-Control", "max-age=86400");
    request->send(response);
  });

  server.on("/api/reboot", HTTP_POST, [](AsyncWebServerRequest* request) {
    request->send(200, "application/json", "{\"rebooting\":true}");
    delay(200);
    ESP.restart();
  });

  server.on("/api/log", HTTP_GET, [](AsyncWebServerRequest* request) {
    // Snapshot indices under lock only — keep lock time in microseconds, not milliseconds
    lockMeshData();
    int head  = _logHead;
    int count = _logCount;
    unlockMeshData();

    String json = "{\"packets\":[";
    int start = count < LOG_SIZE ? 0 : head;
    for (int i = 0; i < count; i++) {
      const PacketLog& p = _log[(start + i) % LOG_SIZE];
      if (i > 0) json += ",";
      json += "{";
      json += "\"type\":"     + String(p.type)    + ",";
      json += "\"src\":\""   + String(p.src)    + "\",";
      json += "\"origSrc\":\"" + String(p.origSrc) + "\",";
      json += "\"appId\":"  + String(p.appId)  + ",";
      // Escape payload — raw bytes (e.g. 0x01) must not appear unescaped in JSON
      json += "\"payload\":\"";
      for (const char* c = p.payload; *c; c++) {
        uint8_t b = (uint8_t)*c;
        if      (b == '"')  json += "\\\"";
        else if (b == '\\') json += "\\\\";
        else if (b < 0x20)  { char esc[7]; snprintf(esc, sizeof(esc), "\\u%04x", b); json += esc; }
        else                json += *c;
      }
      json += "\",";
      json += "\"age_s\":" + String((millis() - p.ts) / 1000);
      json += "}";
    }
    json += "]}";
    request->send(200, "application/json", json);
  });

  server.on("/api/serial", HTTP_GET, [](AsyncWebServerRequest* request) {
    String json = "{\"lines\":[";
    int start = _serialLogCount < SERIAL_LOG_LINES ? 0 : _serialLogHead;
    int count = _serialLogCount;
    for (int i = 0; i < count; i++) {
      const char* line = _serialLog[(start + i) % SERIAL_LOG_LINES];
      if (i > 0) json += ",";
      json += "\"";
      for (const char* c = line; *c; c++) {
        uint8_t b = (uint8_t)*c;
        if      (b == '"')  json += "\\\"";
        else if (b == '\\') json += "\\\\";
        else if (b < 0x20)  { char esc[7]; snprintf(esc, sizeof(esc), "\\u%04x", b); json += esc; }
        else                json += *c;
      }
      json += "\"";
    }
    json += "]}";
    request->send(200, "application/json", json);
  });
}
