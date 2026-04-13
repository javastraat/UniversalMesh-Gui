// lora.cpp — Dual-band LoRa bridge for Universal Mesh coordinator
// Board: Lilygo T-ETH Elite + LoRa Shield (LR1121)
// Transport: MeshPacket sent/received as raw bytes over LoRa, same struct as ESP-NOW
// Gated by -D HAS_LORA in platformio.ini

#include <Arduino.h>

#ifdef HAS_LORA

#include <SPI.h>
#include <RadioLib.h>
#include "UniversalMesh.h"

// ---------------------------------------------------------------------------
// T-ETH Elite LoRa Shield pin assignments  (from board silkscreen diagram)
// ---------------------------------------------------------------------------
#define RADIO_SCLK   10
#define RADIO_MISO    9
#define RADIO_MOSI   11
#define RADIO_CS     40
#define RADIO_RST    46
#define RADIO_IRQ     8
#define RADIO_BUSY   16

// ---------------------------------------------------------------------------
// RF parameters — override via build_flags if needed
//   -D LORA_FREQ_MHZ=915.0   for US/AU
//   -D LORA_SF=9              for longer range (lower data rate)
//   -D LORA_POWER=22          for maximum TX power (check local regulations)
// ---------------------------------------------------------------------------
#ifndef LORA_FREQ_MHZ
  #define LORA_FREQ_MHZ   868.0f   // EU 868 MHz band
#endif
#ifndef LORA_BW
  #define LORA_BW         125.0f   // 125 kHz bandwidth
#endif
#ifndef LORA_SF
  #define LORA_SF         12       // SF12 = longest range, agreed with node firmware
#endif
#ifndef LORA_CR
  #define LORA_CR         5        // Coding rate 4/5
#endif
#ifndef LORA_POWER
  #define LORA_POWER      14       // dBm — safe default, well within EU limit
#endif
#ifndef LORA_PREAMBLE
  #define LORA_PREAMBLE   8
#endif

// Private sync word — must match every LoRa node in the mesh.
#define LORA_SYNC_WORD  0xCD

// MeshPacket header size (everything except the 200-byte payload buffer)
// type(1) + ttl(1) + msgId(4) + destMac(6) + srcMac(6) + appId(1) + payloadLen(1) = 20
static constexpr uint8_t MESH_HDR_SIZE = sizeof(MeshPacket) - 200;

// ---------------------------------------------------------------------------
// ISR-safe RX queue  (filled from ISR, drained in loopLoRa)
// ---------------------------------------------------------------------------
#define LORA_RX_QUEUE_SIZE 8

struct LoRaRxEntry {
  uint8_t data[sizeof(MeshPacket)];
  uint8_t len;
  float   rssi;
  float   snr;
};

static LoRaRxEntry      _rxQueue[LORA_RX_QUEUE_SIZE];
static volatile int     _rxHead = 0;   // written by ISR via loopLoRa (under flag)
static int              _rxTail = 0;   // read by loopLoRa only

// Non-blocking TX queue  (filled from any context, drained in loopLoRa)
#define LORA_TX_QUEUE_SIZE 4

struct LoRaTxEntry {
  uint8_t data[sizeof(MeshPacket)];
  uint8_t len;
};

static LoRaTxEntry  _txQueue[LORA_TX_QUEUE_SIZE];
static int          _txHead = 0;
static int          _txTail = 0;

// ---------------------------------------------------------------------------
// Radio state flags  (volatile = written by ISR, read by loop)
// ---------------------------------------------------------------------------
static volatile bool _rxReady      = false;
static volatile bool _txDone       = false;
static bool          _transmitting = false;

// ---------------------------------------------------------------------------
// Seen-message deduplication (prevents processing the same msgId twice)
// ---------------------------------------------------------------------------
#define LORA_SEEN_SIZE 30
static uint32_t _seen[LORA_SEEN_SIZE] = {};
static uint8_t  _seenIdx = 0;

static bool isSeenLoRa(uint32_t id) {
  for (int i = 0; i < LORA_SEEN_SIZE; i++) {
    if (_seen[i] == id) return true;
  }
  return false;
}
static void markSeenLoRa(uint32_t id) {
  _seen[_seenIdx] = id;
  _seenIdx = (_seenIdx + 1) % LORA_SEEN_SIZE;
}

// ---------------------------------------------------------------------------
// SPI — use the default Arduino SPI object (ESP32-S3 SPI2).
// W5500 (ETH) uses raw IDF SPI3_HOST through the ETH driver — it does NOT go
// through Arduino's SPIClass, so there is no conflict with the default SPI bus.
// ---------------------------------------------------------------------------
static Module  _loraModule(RADIO_CS, RADIO_IRQ, RADIO_RST, RADIO_BUSY);  // uses default SPI
static LR1121  _radio(&_loraModule);

// ---------------------------------------------------------------------------
// Callback into the coordinator's onMeshMessage handler
// ---------------------------------------------------------------------------
static MeshReceiveCallback _meshCb = nullptr;

// Called from coordinator setup() to wire us into the existing packet pipeline
void loraOnReceive(MeshReceiveCallback cb) {
  _meshCb = cb;
}

// ---------------------------------------------------------------------------
// ISRs — IRAM_ATTR so they survive cache misses; flag-only, NO SPI here
// ---------------------------------------------------------------------------
static void IRAM_ATTR _isrRx() { _rxReady = true; }
static void IRAM_ATTR _isrTx() { _txDone  = true; }

// RF switch table — LR1121 uses DIO5/DIO6 internally to switch antenna paths.
// These are virtual RadioLib pin IDs, not ESP32 GPIO numbers.
// Table matches LilyGO's own LR1121 init (LilyGo_LoRa_Pager.cpp).
static const uint32_t _rfswitch_pins[] = {
  RADIOLIB_LR11X0_DIO5, RADIOLIB_LR11X0_DIO6,
  RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC
};
static const Module::RfSwitchMode_t _rfswitch_table[] = {
  { LR11x0::MODE_STBY,  { LOW,  LOW  } },
  { LR11x0::MODE_RX,    { LOW,  HIGH } },
  { LR11x0::MODE_TX,    { HIGH, LOW  } },
  { LR11x0::MODE_TX_HP, { HIGH, LOW  } },
  { LR11x0::MODE_TX_HF, { LOW,  LOW  } },
  { LR11x0::MODE_GNSS,  { LOW,  LOW  } },
  { LR11x0::MODE_WIFI,  { LOW,  LOW  } },
  END_OF_MODE_TABLE,
};

// ---------------------------------------------------------------------------
// setupLoRa() — call once from coordinator setup(), after mesh.begin()
// Sequence mirrors LilyGO's proven LR1121 init order exactly.
// ---------------------------------------------------------------------------
void setupLoRa() {
  // Initialise SPI with explicit pins, -1 for SS so RadioLib controls CS manually.
  // Passing the CS pin to SPI.begin() enables hardware SS mode which conflicts
  // with RadioLib's manual CS toggling and causes STATUS_CMD_FAILED (-707).
  SPI.begin(RADIO_SCLK, RADIO_MISO, RADIO_MOSI, -1);

  Serial.println("[LORA] Init LR1121...");

  // 1. Hard reset — give the chip 200 ms to fully complete its boot sequence
  //    before begin() sends the first SPI command.
  _radio.reset();
  delay(200);

  // 2. begin() with ALL parameters including tcxoVoltage = 3.0V.
  //    This is critical: modSetup() calls setTCXO(tcxoVoltage) then immediately
  //    runs config() which calls calibrate(ALL). If the TCXO voltage is wrong
  //    at calibration time the chip returns CMD_FAIL (-707). Default is 1.6V;
  //    the T-LR1121 module needs 3.0V.
  //    Calling setTCXO(3.0) *after* begin() is too late — calibration has already run.
  int state = _radio.begin(
    (float)LORA_FREQ_MHZ,
    (float)LORA_BW,
    LORA_SF,
    LORA_CR,
    LORA_SYNC_WORD,
    LORA_POWER,
    LORA_PREAMBLE,
    3.0f              // TCXO voltage for T-LR1121 module
  );
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[LORA] begin() FAILED code=%d\n", state);
    return;
  }

  Serial.printf("[LORA] %.1f MHz  SF%d  BW%.0f  CR4/%d  %ddBm  sync=0x%02X\n",
                (float)LORA_FREQ_MHZ, LORA_SF, (float)LORA_BW,
                LORA_CR, LORA_POWER, LORA_SYNC_WORD);

  // 3. RF switch — must be configured after begin(), before any TX/RX
  _radio.setRfSwitchTable(_rfswitch_pins, _rfswitch_table);

  // 6. Attach ISRs and start listening
  _radio.setPacketReceivedAction(_isrRx);
  _radio.setPacketSentAction(_isrTx);

  state = _radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[LORA] startReceive FAILED code=%d\n", state);
    return;
  }

  Serial.println("[LORA] Online — listening for MeshPackets.");
}

// ---------------------------------------------------------------------------
// loraSend() — enqueue raw bytes for TX (safe to call from any context)
// Returns false if the TX queue is full (packet dropped).
// ---------------------------------------------------------------------------
bool loraSend(const uint8_t* data, uint8_t len) {
  int next = (_txHead + 1) % LORA_TX_QUEUE_SIZE;
  if (next == _txTail) return false;  // full
  if (len > sizeof(MeshPacket)) len = sizeof(MeshPacket);
  memcpy(_txQueue[_txHead].data, data, len);
  _txQueue[_txHead].len = len;
  _txHead = next;
  return true;
}

// loraSendPacket() — convenience wrapper: only sends header + actual payload,
// not the unused tail of the 200-byte payload buffer.  Saves airtime.
bool loraSendPacket(MeshPacket* pkt) {
  uint8_t wireLen = MESH_HDR_SIZE + pkt->payloadLen;
  return loraSend((const uint8_t*)pkt, wireLen);
}

// ---------------------------------------------------------------------------
// loopLoRa() — call every loop() iteration
// ---------------------------------------------------------------------------
void loopLoRa() {

  // --- 1. RX: ISR set _rxReady, read the packet in safe context ---
  if (_rxReady) {
    _rxReady = false;

    int next = (_rxHead + 1) % LORA_RX_QUEUE_SIZE;
    if (next != _rxTail) {  // queue has space
      LoRaRxEntry& e = _rxQueue[_rxHead];
      uint8_t pktLen = (uint8_t)_radio.getPacketLength();
      if (pktLen > sizeof(MeshPacket)) pktLen = sizeof(MeshPacket);

      int state = _radio.readData(e.data, pktLen);
      if (state == RADIOLIB_ERR_NONE) {
        e.len  = pktLen;
        e.rssi = _radio.getRSSI();
        e.snr  = _radio.getSNR();
        _rxHead = next;
      }
      // Silently discard CRC errors — LoRa CRC handles this
    } else {
      Serial.println("[LORA] RX queue full — packet dropped");
      _radio.readData((uint8_t*)nullptr, 0);  // flush the radio buffer
    }

    if (!_transmitting) _radio.startReceive();
  }

  // --- 2. Drain RX queue → dispatch to coordinator pipeline ---
  while (_rxTail != _rxHead) {
    LoRaRxEntry& e = _rxQueue[_rxTail];

    if (e.len >= MESH_HDR_SIZE) {
      MeshPacket* pkt = (MeshPacket*)e.data;

      if (!isSeenLoRa(pkt->msgId)) {
        markSeenLoRa(pkt->msgId);

        Serial.printf("[LORA] RX %.1fMHz RSSI=%.1fdBm SNR=%.1fdB "
                      "src=%02X:%02X:%02X:%02X:%02X:%02X appId=0x%02X len=%d\n",
                      (float)LORA_FREQ_MHZ, e.rssi, e.snr,
                      pkt->srcMac[0], pkt->srcMac[1], pkt->srcMac[2],
                      pkt->srcMac[3], pkt->srcMac[4], pkt->srcMac[5],
                      pkt->appId, e.len);

        // Feed into the coordinator's existing packet handler
        // (routing table, MQTT, web dashboard — all the same pipeline as ESP-NOW)
        if (_meshCb) _meshCb(pkt, pkt->srcMac);

        // Let the coordinator mark this node's transport as LoRa
        extern void updateNodeLoRa(uint8_t* mac, float rssi, float snr);
        updateNodeLoRa(pkt->srcMac, e.rssi, e.snr);
      }
    }

    _rxTail = (_rxTail + 1) % LORA_RX_QUEUE_SIZE;
  }

  // --- 3. TX: start next queued packet when idle ---
  if (!_transmitting && _txTail != _txHead) {
    LoRaTxEntry& tx = _txQueue[_txTail];
    _radio.standby();

    int state = _radio.startTransmit(tx.data, tx.len);
    if (state == RADIOLIB_ERR_NONE) {
      _transmitting = true;
    } else {
      Serial.printf("[LORA] TX start failed code=%d\n", state);
      _txTail = (_txTail + 1) % LORA_TX_QUEUE_SIZE;
      _radio.startReceive();
    }
  }

  // --- 4. TX complete: clean up and go back to RX ---
  if (_transmitting && _txDone) {
    _txDone       = false;
    _transmitting = false;
    _radio.finishTransmit();
    _txTail = (_txTail + 1) % LORA_TX_QUEUE_SIZE;
    _radio.startReceive();
    Serial.println("[LORA] TX complete.");
  }
}

// ---------------------------------------------------------------------------
// Stubs intentionally empty — defined below in the #else block
// ---------------------------------------------------------------------------

#else  // !HAS_LORA — empty stubs so other files compile without #ifdef guards

#include "UniversalMesh.h"

void setupLoRa()                               {}
void loopLoRa()                                {}
bool loraSend(const uint8_t*, uint8_t)         { return false; }
bool loraSendPacket(MeshPacket*)               { return false; }
void loraOnReceive(MeshReceiveCallback)        {}

#endif  // HAS_LORA
