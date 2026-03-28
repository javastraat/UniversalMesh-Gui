# UniversalMesh GUI (v1.0.0)

A lightweight, Layer-3 mesh networking protocol built on top of ESP-NOW for ESP32 and ESP8266. Enables seamless, long-range communication between nodes without a central Wi-Fi router.

## Table of Contents

- [Features](#features)
- [Architecture](#architecture)
  - [Packet Structure](#packet-structure)
  - [Packet Type Reference](#packet-type-reference)
  - [AppId Reference](#appid-reference)
  - [Coordinator Discovery](#coordinator-discovery)
- [Supported Hardware](#supported-hardware)
  - [Coordinators](#coordinators)
  - [Sensor Nodes](#sensor-nodes)
  - [Display Gateway](#display-gateway)
- [Getting Started](#getting-started)
  - [Installation](#installation)
  - [Configuration](#configuration)
  - [Build](#build)
- [Sensor Node Behaviour](#sensor-node-behaviour)
- [Coordinator Web Dashboard](#coordinator-web-dashboard)
  - [Panels](#panels)
  - [UI Features](#ui-features)
  - [REST API](#rest-api)
- [ETH Elite Features](#eth-elite-features)
- [RGB LED](#rgb-led)
- [messenger.py — Python Test Client](#messengerpy--python-test-client)
- [Development Stack](#development-stack)
- [License](#license)

---

## Features

- **Auto-Relay:** Every node acts as a repeater (TTL-based) to extend range.
- **Self-Healing:** No fixed routing tables; packets find their path through broadcast/rebroadcast.
- **Transparent Discovery:** Built-in PING/PONG handling for network mapping.
- **De-duplication:** Prevents broadcast storms using unique Message IDs.
- **Hybrid Coordinator:** Bridge your mesh to the internet via Wi-Fi or Ethernet.
- **Node Announce:** Sensors broadcast their name on boot and heartbeat; coordinator stores and displays it.
- **MQTT Bridge:** Coordinator automatically forwards mesh data to an MQTT broker (ETH Elite).
- **Real-Time Web Dashboard:** Live SSE-pushed updates — no polling. Packet log, node table, topology map and serial console update the instant a packet arrives.
- **Force-Directed Topology Map:** Visual graph of the mesh — nodes and relay paths inferred from packet headers, animated in real-time.
- **NTP Time Sync:** Coordinator syncs to NTP; packet timestamps shown as wall-clock time in the dashboard.
- **PWA Support:** Dashboard is installable on mobile as a standalone app (iOS & Android).

---

## Architecture

The network consists of a **Coordinator** (the bridge) and multiple **Sensor Nodes**.

### Packet Structure

The library uses a fixed-size packed struct:

| Field | Size | Description |
| :--- | :--- | :--- |
| `type` | 1 byte | `0x12` PING, `0x13` PONG, `0x14` ACK, `0x15` DATA |
| `ttl` | 1 byte | Time-to-live hop limit (default 4) |
| `msgId` | 4 bytes | Unique message ID for deduplication |
| `destMac` | 6 bytes | Destination or `FF:FF:FF:FF:FF:FF` for broadcast |
| `srcMac` | 6 bytes | Original sender MAC |
| `appId` | 1 byte | Application multiplexer — see table below |
| `payloadLen` | 1 byte | Payload length (max 64) |
| `payload` | 64 bytes | Raw binary payload |

### Packet Type Reference

| Type | Name | Description |
| :--- | :--- | :--- |
| `0x00–0x11` | *(reserved)* | Reserved for future protocol extensions |
| `0x12` | PING | Broadcast by a node to discover the coordinator. Every node auto-replies with a PONG |
| `0x13` | PONG | Reply to a PING. `payload[0]=0x01` = coordinator, `payload[0]=0x00` = regular node |
| `0x14` | ACK | Acknowledgement (reserved for future use) |
| `0x15` | DATA | Application data packet — see AppId table below |

### AppId Reference

| AppId | Name | Description |
| :--- | :--- | :--- |
| `0x00` | Protocol | Reserved for mesh control (PING/PONG) |
| `0x01` | Text | Human-readable string payload |
| `0x02` | Raw Hex | Raw binary payload |
| `0x05` | Heartbeat | Periodic alive signal (single byte) |
| `0x06` | Node Announce | Node broadcasts its name; coordinator stores it in the node table |

### Coordinator Discovery

When a sensor node boots it has no knowledge of the coordinator's MAC address. Discovery works as follows:

1. The node broadcasts a **PING** (`type=0x12`) to `FF:FF:FF:FF:FF:FF`.
2. Every node on the mesh that receives the PING **auto-replies with a PONG** (`type=0x13`, `appId=0x00`).
3. Only the coordinator sets `payload[0]=0x01` in its PONG. All other nodes send `payload[0]=0x00`.
4. The sensor node ignores PONGs with `payload[0]=0x00` and locks onto the first PONG where `payload[0]=0x01` as its coordinator.
5. If no coordinator PONG is received within 10 seconds, the node retries the broadcast automatically.

This prevents a sensor node from accidentally treating a relay node or another sensor as the coordinator.

To mark a node as coordinator, pass `true` to `begin()`:

```cpp
mesh.begin(WIFI_CHANNEL, true);   // coordinator — PONG replies carry payload[0]=0x01
mesh.begin(WIFI_CHANNEL);         // sensor / relay — PONG replies carry payload[0]=0x00
```

---

## Supported Hardware

### Coordinators

| Environment | Board | Flash | PSRAM | Ethernet | OTA | RGB LED |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: |
| `coordinator_c6` | ESP32-C6 DevKitC-1 | 4 MB | — | — | — | ✓ |
| `coordinator_s3` | ESP32-S3 DevKitC-1 | 4 MB | — | — | — | — |
| `coordinator_t8_s3` | LilyGo T8-S3 | 16 MB | ✓ | — | — | — |
| `coordinator_eth_elite` | LilyGo T-ETH Elite | 16 MB | ✓ | ✓ | ✓ | — |
| `coordinator_eth_elite_ota` | LilyGo T-ETH Elite | 16 MB | ✓ | ✓ | ✓ (IP) | — |

### Sensor Nodes

| Environment | Board | Notes |
| :--- | :--- | :--- |
| `sensor_node_esp32` | Generic ESP32 Dev Module | 4 MB flash; sends internal CPU temperature |
| `sensor_node_c6` | ESP32-C6 DevKitC-1 | USB CDC; sends internal CPU temperature |
| `sensor_node_t8_s3` | LilyGo T8-S3 | 16 MB flash, PSRAM; sends internal CPU temperature |
| `sensor_wemos_d1` | Wemos D1 / mini (ESP8266) | SHT30 real sensor; optional OLED via `HAS_DISPLAY_SHIELD` |
| `sensor_esp8266` | NodeMCU v2 (ESP8266) | Generic ESP8266 sensor node; sends simulated temperature |

### Display Gateway

| Environment | Board | Notes |
| :--- | :--- | :--- |
| `gateway_esp8266` | NodeMCU v2 (ESP8266) | Receives mesh packets and displays them on SSD1306 OLED |

---

## Getting Started

### Installation

1. Clone or copy the `lib/UniversalMesh/` folder into your project's `lib/` directory.
2. Create `include/secrets.h` with your credentials (see [Configuration](#configuration) below).
3. All ESP32 environments use the `pioarduino` platform (set as the default in `[platformio]`). ESP8266 environments (`sensor_wemos_d1`, `gateway_esp8266`) use the standard `espressif8266` platform — no changes needed.

### Configuration

#### `include/secrets.h` — credentials (git-ignored)

Create this file before building. It holds secrets only:

```cpp
// WiFi (all coordinators)
#define WIFI_SSID "your-ssid"
#define WIFI_PASS "your-password"

// MQTT broker (ETH Elite only)
#define MQTT_BROKER "192.168.1.x"   // IP or hostname
#define MQTT_PORT   1883
#define MQTT_USER   ""              // leave empty if not required
#define MQTT_PASS   ""
```

#### `platformio.ini` build flags — non-secret settings

Network identity, OTA and NTP are configured as build flags per environment:

```ini
build_flags =
    -D MESH_HOSTNAME=\"universalmesh\"   ; mDNS name and MQTT path prefix
    -D MESH_NETWORK=\"mymesh\"           ; MQTT topic root
    -D OTA_PASSWORD=\"mesh1234\"         ; keep in sync with upload_flags --auth=
    -D NTP_SERVER=\"pool.ntp.org\"
    -D NTP_GMT_OFFSET_SEC=3600          ; UTC offset in seconds (e.g. 3600 = UTC+1)
```

`NODE_NAME` is set per sensor node environment:

```ini
build_flags =
    -D NODE_NAME=\"my-node\"
```

If not set, it falls back to `"sensor-node"` via an `#ifndef` guard in the source.

### Build

Flash and monitor a specific environment:

```
pio run -e coordinator_c6 -t upload -t monitor
```

OTA upload (ETH Elite):

```
pio run -e coordinator_eth_elite_ota -t upload
```

---

## Sensor Node Behaviour

Each sensor node:

1. On boot, broadcasts a **PING** to discover the coordinator (see [Coordinator Discovery](#coordinator-discovery)). Retries every 10 s until a coordinator PONG is received.
2. Once the coordinator is found, sends an **AppId `0x06`** Node Announce packet with its `NODE_NAME` string.
3. Every `HEARTBEAT_INTERVAL` ms (default 60 s), sends an **AppId `0x05`** heartbeat and re-sends the announce.
4. Sends sensor readings (e.g. temperature/humidity) as **AppId `0x01`** data packets.

---

## Coordinator Web Dashboard

The coordinator serves a responsive single-page dashboard on port 80.

### Panels

| Panel | Description |
| :--- | :--- |
| **ESP** | Chip model, cores, CPU MHz, flash size, free heap, MAC, uptime, NTP time |
| **WiFi** | SSID, IP, gateway, RSSI, channel |
| **Ethernet** | Status, IP, subnet, gateway, DNS, MAC, link speed/duplex *(ETH Elite only)* |
| **Mesh Nodes** | Live list of known nodes — MAC, last-seen counter, resolved name. Coordinator always pinned at top. Click **Node** or **Last Seen** column header to sort. Green dot = seen <120 s, red = stale |
| **Mesh Channel** | Dropdown to switch ESP-NOW channel (1–13), persisted to NVS *(ETH Elite only)* |
| **Send Message** | Inject a text packet to any node or broadcast directly from the browser |
| **Packet Log** | Paginated live log (200 entries with PSRAM, 10 without). Shows type, sender, app ID, payload, timestamp. Relayed packets highlighted |
| **Topology Map** | Force-directed canvas graph of all nodes and relay paths, inferred from packet headers. Click a node for details. Controls: Freeze/Resume physics, Reset layout, toggle MAC labels, toggle edge age labels, drag to pin a node, double-click to unpin, pan by dragging background, export as PNG |
| **Serial Console** | Live stream of the coordinator's internal log — like a web-based serial monitor |

### UI Features

- **SSE push** — the browser opens a single `GET /api/events` stream; the ESP32 pushes `packet` and `serial` events the instant they happen. No polling for the log, topology or console.
- **Dark / light theme** toggle, persisted in `localStorage`
- **Installable PWA** — add to home screen on iOS/Android for a standalone app experience
- **NTP timestamps** — packet log shows wall-clock time (e.g. `23:18:12`) instead of a relative counter when NTP is synced

### REST API

| Endpoint | Method | Description |
| :--- | :--- | :--- |
| `/api/events` | GET (SSE) | Server-Sent Events stream — `packet` and `serial` event types |
| `/api/status` | GET | Coordinator status (uptime, heap, IP, channel, NTP time, ETH info) |
| `/api/nodes` | GET | Node table (MAC, last-seen seconds, name) |
| `/api/log` | GET | Recent packet log (type, src, origSrc, appId, payload, age) |
| `/api/serial` | GET | Recent serial output (last 40 lines) |
| `/api/tx` | POST | Send a packet into the mesh (`dest`, `appId`, hex `payload`, `ttl`) |
| `/api/discover` | GET | Broadcast a PING to discover all mesh nodes |
| `/api/mesh/channel` | POST | Set ESP-NOW channel 1–13, optionally reboot *(ETH Elite only)* |
| `/api/reboot` | POST | Reboot the coordinator |

---

## ETH Elite Features

The `coordinator_eth_elite` / `coordinator_eth_elite_ota` builds add:

- **W5500 Ethernet** as primary network. On boot, waits up to 5 s for cable, then up to 60 s for DHCP. Falls back to WiFi if no cable is detected.
- **MQTT Bridge** — DATA packets (appId `0x01`, `0x05`, `0x06`) are automatically published to:
  ```
  {MESH_NETWORK}/{MESH_HOSTNAME}/nodes/{nodeName}/{appId}
  ```
  Broker, port and credentials are set in `secrets.h`.
- **NTP sync** — wall-clock time displayed in dashboard and packet log.
- **OTA updates** — flash over the network:
  ```
  pio run -e coordinator_eth_elite_ota -t upload
  ```
- **mDNS** — reachable at `universalmesh.local` (hostname set via the `MESH_HOSTNAME` build flag in `platformio.ini`).
- **Mesh channel selector** — change ESP-NOW channel (1–13) via the dashboard, persisted to NVS across reboots.

---

## RGB LED

The ESP32-C6 coordinator uses pin 8 (NeoPixel) as a status indicator:

| State | Colour | Pattern |
| :--- | :--- | :--- |
| Connecting to WiFi | Green | Slow blink |
| WiFi connected | Green | Steady |
| WiFi failed | Red | Steady |
| Packet received | Yellow | 3 quick flashes |
| Packet transmitted | Blue | 3 quick flashes |

---

## `messenger.py` — Python Test Client

A CLI tool to inject packets into the mesh via the coordinator REST API.

```
python3 messenger.py                          # send one random message
python3 messenger.py -t "Hello mesh"          # send custom text
python3 messenger.py -l -i 2.0               # loop, one message every 2 s
python3 messenger.py -l -n 10 -i 1.0         # send 10 messages, 1 s apart
```

Edit `COORDINATOR_IP` at the top of the file to point to your coordinator.

---

## Development Stack

| Component | Technology | Scope |
| :--- | :--- | :--- |
| Core firmware | C++17 (Arduino / ESP-IDF) | All |
| Build system | PlatformIO | All |
| Mesh transport | ESP-NOW (ESP-IDF) | All |
| Web server | ESP Async WebServer (mathieucarbou fork) | Coordinator |
| Async TCP | AsyncTCP (mathieucarbou fork) | Coordinator (ESP32) |
| JSON | ArduinoJson 7.x | All |
| MQTT | PubSubClient 2.x | Coordinator (ETH Elite) |
| RGB LED | Adafruit NeoPixel 1.12.3 | Coordinator C6 |
| Temperature / humidity | Adafruit SHT31 Library | `sensor_wemos_d1` |
| OLED display (ESP32) | U8g2 | `sensor_wemos_d1` |
| OLED display (ESP8266) | Adafruit SSD1306 + GFX | `gateway_esp8266` |
| Coordinator MCU | ESP32-C6, ESP32-S3, LilyGo T8-S3, LilyGo T-ETH Elite | Coordinator |
| Sensor MCU | Generic ESP32, ESP32-C6, LilyGo T8-S3 (ESP32-S3), Wemos D1 (ESP8266) | Sensor nodes |

---

## License

Licensed under the Apache License, Version 2.0.
