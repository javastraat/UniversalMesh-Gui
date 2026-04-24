#ifndef UNIVERSAL_MESH_H
#define UNIVERSAL_MESH_H

#include <Arduino.h>

#if defined(ESP8266)
  #include <stddef.h>
  #define MBEDTLS_AES_ENCRYPT 1
  #define MBEDTLS_AES_DECRYPT 0
  typedef struct { int nr; uint32_t buf[68]; } mbedtls_aes_context;
  inline void mbedtls_aes_init(mbedtls_aes_context *) {}
  inline void mbedtls_aes_free(mbedtls_aes_context *) {}
  inline int  mbedtls_aes_setkey_enc(mbedtls_aes_context *, const unsigned char *, unsigned int) { return 0; }
  inline int  mbedtls_aes_crypt_cfb128(mbedtls_aes_context *, int, size_t, size_t *,
                                       unsigned char *, const unsigned char *, unsigned char *) { return 0; }
#else
  #include <mbedtls/aes.h>
#endif

#if defined(ESP8266)
  #include <ESP8266WiFi.h>
  #include <espnow.h>
#elif defined(ESP32)
  #include <esp_now.h>
  #include <esp_wifi.h>
#endif

#ifdef UM_DEBUG_MODE
  #define UM_DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
  #define UM_DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
  #define UM_DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
  #define UM_DEBUG_PRINT(...)
  #define UM_DEBUG_PRINTLN(...)
  #define UM_DEBUG_PRINTF(...)
#endif

#define MESH_TYPE_PING          0x12
#define MESH_TYPE_PONG          0x13
#define MESH_TYPE_ACK           0x14
#define MESH_TYPE_DATA          0x15
#define MESH_TYPE_KEY_REQ       0x16
#define MESH_TYPE_SECURE_DATA   0x17
#define MESH_TYPE_PARANOID_DATA 0x18

enum MeshRole { MESH_NODE = 0, MESH_COORDINATOR = 1 };

struct __attribute__((packed)) MeshPacket {
  uint8_t  type;
  uint8_t  ttl;
  uint32_t msgId;
  uint8_t  destMac[6];
  uint8_t  srcMac[6];
  uint8_t  appId;
  uint8_t  payloadLen;
  uint8_t  payload[200];
};

typedef void (*MeshReceiveCallback)(MeshPacket* packet, uint8_t* senderMac);

class UniversalMesh {
  public:
    UniversalMesh();
    bool begin(uint8_t channel, MeshRole role = MESH_NODE);
    void setNetworkKey(const char* key);
    bool send(uint8_t destMac[6], uint8_t type, uint8_t appId, const uint8_t* payload, uint8_t len, uint8_t ttl = 4, bool encrypt = false);
    bool send(uint8_t destMac[6], uint8_t type, uint8_t appId, String payload, uint8_t ttl = 4, bool encrypt = false);
    bool sendToCoordinator(uint8_t appId, uint8_t* payload, uint8_t len);
    bool sendToCoordinator(uint8_t appId, String payload);
    bool sendSecureToCoordinator(uint8_t appId, String payload);
    uint8_t findCoordinatorChannel(const char* nodeName = nullptr);
    void setCoordinatorMac(uint8_t* mac);
    void getCoordinatorMac(uint8_t* mac);
    bool isCoordinatorFound();
    void onReceive(MeshReceiveCallback callback);
    void update();

  private:
    uint8_t _myMac[6];
    uint8_t _broadcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    MeshReceiveCallback _userCallback;
    MeshRole _role;
    static uint8_t _coordinatorMac[6];
    volatile bool _pongReceived;
    bool _coordinatorFound;
    unsigned long _lastDiscoveryPing;
    uint8_t _meshKey[16];
    bool _meshKeySet = false;

    struct SeenMessage { uint32_t msgId; uint8_t type; };
    static const int SEEN_SIZE = 30;
    SeenMessage _seen[SEEN_SIZE];
    uint8_t _sIdx = 0;
    bool isSeen(uint32_t id, uint8_t type);
    void markSeen(uint32_t id, uint8_t type);

    static UniversalMesh* _instance;

    #if defined(ESP8266)
      static void espNowRecvWrapper(uint8_t *mac, uint8_t *data, uint8_t len);
    #elif defined(ESP32)
      static void espNowRecvWrapper(const esp_now_recv_info_t *info, const uint8_t *data, int len);
    #endif

    void handleReceive(uint8_t *mac, uint8_t *data, uint8_t len);
};

#endif
