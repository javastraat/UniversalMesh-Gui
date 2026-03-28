#!/usr/bin/env python3
import requests
import datetime
import random
import time
import argparse

# --- CONFIGURATION ---
COORDINATOR_IP = "192.168.2.208"
DESTINATION_MAC = "FF:FF:FF:FF:FF:FF" # Broadcast to all nodes
APP_ID_TEXT = 1  # Tells the Gateway to decode as ASCII

# Random message pool
MESSAGES = [
    # Status
    "System Nominal",
    "All Systems Go",
    "Mesh Heartbeat",
    "Network Stable",
    "Amsterdam Lab Online",
    "Uptime Check OK",
    "Watchdog Reset",
    "Low Power Mode Active",
    "Clock Sync Complete",
    "Boot Sequence Done",
    # Nodes
    "Node Deep Sleep Test",
    "Node Wakeup Triggered",
    "Node Battery Critical",
    "Node Rejoined Mesh",
    "Node Timeout: Retry",
    "New Node Discovered",
    "Orphan Node Detected",
    "Relay Node Active",
    "Edge Node Reporting",
    "Node Pairing Request",
    # Signal / RF
    "Signal Strength: High",
    "Signal Strength: Low",
    "RSSI Threshold Exceeded",
    "Channel Noise Detected",
    "Packet Loss > 5%",
    "Link Quality: Stable",
    "Retransmit Attempt 1",
    "Interference Detected",
    # Data / Sensor
    "Entropy update: 0x42",
    "Sensor Read OK",
    "Temp: 21.4 C",
    "Humidity: 58%",
    "Pressure: 1013 hPa",
    "Motion Detected",
    "Door Event: Open",
    "Door Event: Closed",
    "Light Level: 320 lux",
    "CO2: 842 ppm",
    "Air Quality: Good",
    # Mesh ops
    "Routing Table Updated",
    "Broadcast Flood Limit Hit",
    "TTL Expired: Drop",
    "ACK Received",
    "ACK Timeout",
    "Mesh Rebalancing",
    "Gateway Sync Request",
    "OTA Check: No Update",
    "Config Push Received",
    "Mesh Partition Healed",
]

def send_mesh_update(custom_text=None):
    url = f"http://{COORDINATOR_IP}/api/tx"
    
    # 1. Generate the data
    now = datetime.datetime.now().strftime("%H:%M:%S")
    if custom_text:
        msg = custom_text
    else:
        msg = random.choice(MESSAGES)
    full_string = f"[{now}] {msg}"
    
    # 2. Convert string to Hex for the Coordinator API
    # "Hello" -> "48656C6C6F"
    hex_payload = full_string.encode('utf-8').hex().upper()
    
    payload = {
        "dest": DESTINATION_MAC,
        "appId": APP_ID_TEXT,
        "ttl": 4,
        "payload": hex_payload
    }

    print(f"Attempting to send: {full_string}")
    
    try:
        response = requests.post(url, json=payload, timeout=5)
        if response.status_code == 200:
            print(f"Success! Coordinator Response: {response.json()}")
        else:
            print(f"Error: Server returned {response.status_code}")
    except requests.exceptions.ConnectionError:
        print(f"Failed: Could not connect to Coordinator at {COORDINATOR_IP}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Send a mesh update message.")
    parser.add_argument('-t', '--text', type=str, help='Custom text to send instead of a random message')
    parser.add_argument('-l', '--loop', action='store_true', help='Enable loop mode to send messages repeatedly')
    parser.add_argument('-n', '--num', type=int, default=0, help='Number of messages to send (0 = infinite in loop mode)')
    parser.add_argument('-i', '--interval', type=float, default=2.0, help='Interval between messages in seconds (loop mode)')
    args = parser.parse_args()

    if args.loop:
        count = 0
        try:
            while True:
                send_mesh_update(args.text)
                count += 1
                if args.num > 0 and count >= args.num:
                    break
                time.sleep(args.interval)
        except KeyboardInterrupt:
            print("\nLoop stopped by user.")
    else:
        send_mesh_update(args.text)
