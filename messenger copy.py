#!/usr/bin/env python3
import requests
import datetime
import random
import time
import argparse
import sys

# --- CONFIGURATION ---

COORDINATOR_IP = "192.168.2.193"
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


def fetch_nodes():
    url = f"http://{COORDINATOR_IP}/api/nodes"
    try:
        response = requests.get(url, timeout=5)
        if response.status_code == 200:
            return response.json().get("nodes", [])
        else:
            print(f"Error fetching nodes: Server returned {response.status_code}")
            return []
    except Exception as e:
        print(f"Failed to fetch nodes: {e}")
        return []

def select_destination_mac():
    nodes = fetch_nodes()
    if not nodes:
        print("No nodes found. Defaulting to broadcast (FF:FF:FF:FF:FF:FF)")
        return "FF:FF:FF:FF:FF:FF"
    print("\nAvailable nodes:")
    print("(Press Enter to use broadcast by default)")
    for idx, node in enumerate(nodes):
        name = node.get("name", "")
        mac = node.get("mac", "")
        transport = node.get("transport", "")
        last_seen = node.get("last_seen_seconds_ago", "?")
        label = f"{idx+1}. {mac}"
        if name:
            label += f" ({name})"
        label += f" [{transport}, last seen {last_seen}s ago]"
        print(label)
    print(f"{len(nodes)+1}. FF:FF:FF:FF:FF:FF (Broadcast)")
    print("0. Exit")
    while True:
        try:
            choice = input(f"Select destination [1-{len(nodes)+1}, 0=exit]: ").strip()
            if choice == "0":
                print("Exiting.")
                sys.exit(0)
            if not choice:
                return "FF:FF:FF:FF:FF:FF"
            idx = int(choice) - 1
            if idx == len(nodes):
                return "FF:FF:FF:FF:FF:FF"
            if 0 <= idx < len(nodes):
                return nodes[idx].get("mac", "FF:FF:FF:FF:FF:FF")
        except (ValueError, IndexError):
            print("Invalid selection. Try again.")

def send_mesh_update(custom_text=None, dest_mac=None):
    url = f"http://{COORDINATOR_IP}/api/tx"
    now = datetime.datetime.now().strftime("%H:%M:%S")
    if custom_text:
        msg = custom_text
    else:
        msg = random.choice(MESSAGES)
    full_string = f"[{now}] {msg}"
    hex_payload = full_string.encode('utf-8').hex().upper()
    payload = {
        "dest": dest_mac or "FF:FF:FF:FF:FF:FF",
        "appId": APP_ID_TEXT,
        "ttl": 4,
        "payload": hex_payload
    }
    print(f"Attempting to send: {full_string} to {payload['dest']}")
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
    parser.add_argument('-d', '--dest', type=str, help='Destination MAC address (overrides interactive selection)')
    parser.add_argument('--no-prompt', action='store_true', help='Skip interactive node selection and use broadcast')
    args = parser.parse_args()

    if args.dest:
        dest_mac = args.dest
    elif args.no_prompt:
        dest_mac = "FF:FF:FF:FF:FF:FF"
    else:
        dest_mac = select_destination_mac()

    # Prompt for message if not provided
    if args.text is not None:
        msg_text = args.text
    else:
        print("\nChoose message type:")
        print("1. Send random message (default if Enter)")
        print("2. Enter custom text")
        print("(Press Enter to send a random message)")
        while True:
            choice = input("Select [1-2]: ").strip()
            if choice == "1" or choice == "":
                msg_text = None
                break
            elif choice == "2":
                msg_text = input("Enter your message: ")
                break
            else:
                print("Invalid selection. Try again.")

    if args.loop:
        count = 0
        try:
            while True:
                send_mesh_update(msg_text, dest_mac)
                count += 1
                if args.num > 0 and count >= args.num:
                    break
                time.sleep(args.interval)
        except KeyboardInterrupt:
            print("\nLoop stopped by user.")
    else:
        send_mesh_update(msg_text, dest_mac)
