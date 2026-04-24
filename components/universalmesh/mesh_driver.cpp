#include "mesh_driver.h"

#if defined(ESP32)
#include <esp_wifi.h>
#endif

UniversalMesh* UniversalMesh::_instance = nullptr;
uint8_t UniversalMesh::_coordinatorMac[6];

UniversalMesh::UniversalMesh() {
  _instance = this;
  _userCallback = nullptr;
  _role = MESH_NODE;
  _coordinatorFound = false;
  _lastDiscoveryPing = 0;
  _meshKeySet = false;
  memset(_coordinatorMac, 0, 6);
}

bool UniversalMesh::begin(uint8_t channel, MeshRole role) {
  _role = role;
  #if defined(ESP8266)
    wifi_set_channel(channel);
  #elif defined(ESP32)
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
  #endif
  #if defined(ESP32)
    esp_wifi_get_mac(WIFI_IF_STA, _myMac);
  #else
    WiFi.macAddress(_myMac);
  #endif

  esp_now_deinit();  // safe to call even if not initialized; ensures clean reinit after WiFi adapter restart
  if (esp_now_init() != 0) return false;

  #if defined(ESP8266)
    esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
    esp_now_register_recv_cb(espNowRecvWrapper);
    esp_now_add_peer(_broadcastMac, ESP_NOW_ROLE_COMBO, channel, NULL, 0);
  #elif defined(ESP32)
    esp_now_register_recv_cb(espNowRecvWrapper);
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, _broadcastMac, 6);
    peer.channel = 0;  // 0 = follow current WiFi channel
    esp_now_add_peer(&peer);
  #endif
  UM_DEBUG_PRINTF("[MESH] ESP-NOW Initialized %02X:%02X:%02X:%02X:%02X:%02X\n",
                  _myMac[0], _myMac[1], _myMac[2], _myMac[3], _myMac[4], _myMac[5]);
  return true;
}

void UniversalMesh::onReceive(MeshReceiveCallback callback) { _userCallback = callback; }

void UniversalMesh::setNetworkKey(const char* key) {
  memset(_meshKey, 0, 16);
  strncpy((char*)_meshKey, key, 16);
  _meshKeySet = true;
}

bool UniversalMesh::send(uint8_t destMac[6], uint8_t type, uint8_t appId, const uint8_t* payload, uint8_t len, uint8_t ttl, bool encrypt) {
  MeshPacket p = {};
  p.type = type;
  p.ttl = ttl;
  #if defined(ESP8266)
    p.msgId = (uint32_t)os_random();
  #elif defined(ESP32)
    p.msgId = esp_random() % 1000000000;
  #endif
  memcpy(p.destMac, destMac, 6);
  memcpy(p.srcMac, _myMac, 6);
  p.appId = appId;
  if (payload != nullptr && len > 0) {
    p.payloadLen = (len > 200) ? 200 : len;
    if (encrypt && _meshKeySet) {
      mbedtls_aes_context aes;
      mbedtls_aes_init(&aes);
      mbedtls_aes_setkey_enc(&aes, _meshKey, 128);
      uint8_t iv[16] = {0};
      memcpy(iv, &p.msgId, sizeof(p.msgId));
      size_t iv_offset = 0;
      mbedtls_aes_crypt_cfb128(&aes, MBEDTLS_AES_ENCRYPT, p.payloadLen, &iv_offset, iv, payload, p.payload);
      mbedtls_aes_free(&aes);
    } else {
      memcpy(p.payload, payload, p.payloadLen);
    }
  } else {
    p.payloadLen = 0;
  }
  return (esp_now_send(_broadcastMac, (uint8_t*)&p, sizeof(p)) == 0);
}

bool UniversalMesh::send(uint8_t destMac[6], uint8_t type, uint8_t appId, String payload, uint8_t ttl, bool encrypt) {
  return send(destMac, type, appId, (const uint8_t*)payload.c_str(), payload.length(), ttl, encrypt);
}

bool UniversalMesh::sendToCoordinator(uint8_t appId, uint8_t* payload, uint8_t len) {
  if (!_coordinatorFound) return false;
  return send(_coordinatorMac, MESH_TYPE_DATA, appId, payload, len, 4);
}

bool UniversalMesh::sendToCoordinator(uint8_t appId, String payload) {
  return sendToCoordinator(appId, (uint8_t*)payload.c_str(), payload.length());
}

bool UniversalMesh::sendSecureToCoordinator(uint8_t appId, String payload) {
  if (!_coordinatorFound) return false;
  return send(_coordinatorMac, MESH_TYPE_SECURE_DATA, appId, payload, 4, true);
}

uint8_t UniversalMesh::findCoordinatorChannel(const char* nodeName) {
  uint8_t broadcastMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  uint8_t pingData[] = {0x50, 0x49, 0x4E, 0x47};
  #if defined(ESP32)
  // Keep promiscuous ON for the entire scan so WiFi can't change the channel
  // between esp_wifi_set_channel() and esp_now_send().
  esp_wifi_set_promiscuous(true);
  #endif
  for (uint8_t ch = 1; ch <= 13; ch++) {
    UM_DEBUG_PRINTF("[MESH] Scanning channel %d...\n", ch);
    #if defined(ESP8266)
      ESP.wdtFeed();  // 13-channel scan blocks ~3 s — keep HW WDT alive
      wifi_set_channel(ch);
      if (esp_now_is_peer_exist(broadcastMac)) esp_now_del_peer(broadcastMac);
      esp_now_add_peer(broadcastMac, ESP_NOW_ROLE_COMBO, ch, NULL, 0);
    #elif defined(ESP32)
      esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
      esp_now_peer_info_t peerInfo = {};
      memcpy(peerInfo.peer_addr, broadcastMac, 6);
      peerInfo.channel = 0;  // 0 = follow current WiFi channel
      peerInfo.encrypt = false;
      if (esp_now_is_peer_exist(broadcastMac)) esp_now_mod_peer(&peerInfo);
      else esp_now_add_peer(&peerInfo);
    #endif
    delay(20); // Let radio settle
    _pongReceived = false;
    // Send 3 PINGs per channel so a single WiFi probe collision doesn't lose
    // the whole channel — ESP-NOW is fire-and-forget with no retransmits.
    for (uint8_t attempt = 0; attempt < 3; attempt++) {
      if (broadcastMac == nullptr || pingData == nullptr) continue;
      esp_now_send(broadcastMac, pingData, sizeof(pingData));
      unsigned long startWait = millis();
      while (millis() - startWait < 80) {
        if (_pongReceived) return ch;
        delay(5);
        #ifdef ESP8266
        ESP.wdtFeed();
        #endif
      }
    }
  }
  #if defined(ESP32)
  esp_wifi_set_promiscuous(false);
  #endif
  return 0;
}

void UniversalMesh::setCoordinatorMac(uint8_t* mac) { memcpy(_coordinatorMac, mac, 6); _coordinatorFound = true; }
void UniversalMesh::getCoordinatorMac(uint8_t* mac) { memcpy(mac, _coordinatorMac, 6); }
bool UniversalMesh::isCoordinatorFound() { return _coordinatorFound; }

bool UniversalMesh::isSeen(uint32_t id, uint8_t type) {
  for (int i = 0; i < SEEN_SIZE; i++)
    if (_seen[i].msgId == id && _seen[i].type == type) return true;
  return false;
}

void UniversalMesh::markSeen(uint32_t id, uint8_t type) {
  _seen[_sIdx] = {id, type};
  _sIdx = (_sIdx + 1) % SEEN_SIZE;
}

#if defined(ESP8266)
void UniversalMesh::espNowRecvWrapper(uint8_t *mac, uint8_t *data, uint8_t len) {
  if (_instance) _instance->handleReceive(mac, data, len);
}
#elif defined(ESP32)
void UniversalMesh::espNowRecvWrapper(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (_instance) _instance->handleReceive((uint8_t*)info->src_addr, (uint8_t*)data, len);
}
#endif

void UniversalMesh::handleReceive(uint8_t *mac, uint8_t *data, uint8_t len) {
  if (len == 4 && memcmp(data, "PING", 4) == 0) {
    if (_role == MESH_COORDINATOR) {
      uint8_t pongData[] = {0x50, 0x4F, 0x4E, 0x47};
      esp_now_send(_broadcastMac, pongData, 4);
    }
    return;
  }
  if (len == 4 && memcmp(data, "PONG", 4) == 0) {
    _pongReceived = true;
    memcpy(_coordinatorMac, mac, 6);
    return;
  }
  if (len < (int)sizeof(MeshPacket)) return;
  MeshPacket* p = (MeshPacket*)data;
  if (!isSeen(p->msgId, p->type)) {
    markSeen(p->msgId, p->type);
    if (p->type == MESH_TYPE_PING) {
      uint8_t replyAppId = (_role == MESH_COORDINATOR) ? 0xFF : 0x00;
      send(p->srcMac, MESH_TYPE_PONG, replyAppId, nullptr, 0, p->ttl);
      if (_role == MESH_COORDINATOR && p->payloadLen > 0 && _userCallback)
        _userCallback(p, mac);
    }
    if (_role == MESH_NODE && p->type == MESH_TYPE_PONG && p->appId == 0xFF) {
      memcpy(_coordinatorMac, p->srcMac, 6);
      _coordinatorFound = true;
    }
    bool isForMe = (memcmp(p->destMac, _myMac, 6) == 0) || (memcmp(p->destMac, _broadcastMac, 6) == 0);
    bool terminalData = (p->type == MESH_TYPE_DATA && memcmp(p->destMac, _myMac, 6) == 0);
    if (isForMe && _userCallback && p->type != MESH_TYPE_PING)
      _userCallback(p, mac);
    if (p->ttl > 0 && !terminalData) {
      uint8_t relayBuffer[sizeof(MeshPacket)];
      memcpy(relayBuffer, data, len);
      ((MeshPacket*)relayBuffer)->ttl--;
      esp_now_send(_broadcastMac, relayBuffer, len);
    }
  }
}

void UniversalMesh::update() {
  if (_role == MESH_NODE && !_coordinatorFound) {
    if (millis() - _lastDiscoveryPing > 10000) {
      _lastDiscoveryPing = millis();
      send(_broadcastMac, MESH_TYPE_PING, 0x00, nullptr, 0, 4);
    }
  }
}
