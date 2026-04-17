#!/usr/bin/env python3
import requests
import datetime
import random
import time
import argparse
import sys
import re
try:
    from colorama import init as colorama_init, Fore, Style
    colorama_init()
    COLORAMA = True
except ImportError:
    COLORAMA = False

# --- CONFIGURATION ---

#COORDINATOR_IP = "universalmesh.local"
COORDINATOR_IP = "192.168.2.193"

# Default identification message text (can be overridden via --ident-msg)
DEFAULT_IDENT_MSG_TEXT = "PD2EMC"

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
            print(f"{Fore.RED if COLORAMA else ''}Error fetching nodes: Server returned {response.status_code}{Style.RESET_ALL if COLORAMA else ''}")
            return []
    except Exception as e:
        print(f"{Fore.RED if COLORAMA else ''}Failed to fetch nodes: {e}{Style.RESET_ALL if COLORAMA else ''}")
        return []

def select_destination_mac():
    def color_for_last_seen(secs):
        if not COLORAMA:
            return ""
        if secs == "?":
            return Fore.WHITE
        try:
            secs = int(secs)
        except Exception:
            return Fore.WHITE
        if secs < 30:
            return Fore.GREEN
        elif secs < 120:
            return Fore.YELLOW
        else:
            return Fore.RED

    while True:
        nodes = fetch_nodes()
        if not nodes:
            print(f"{Fore.RED if COLORAMA else ''}No nodes found. Check coordinator connection or try again.{Style.RESET_ALL if COLORAMA else ''}")
            retry = input("Type 'r' to retry, Enter to use broadcast, or 0 to exit: ").strip().lower()
            if retry == '0':
                print("Exiting.")
                sys.exit(0)
            elif retry == 'r':
                continue
            else:
                return "FF:FF:FF:FF:FF:FF"
        print("\nAvailable nodes:")
        print("(Press Enter to use broadcast by default, 'r' to refresh, '?' for help)")
        for idx, node in enumerate(nodes):
            name = node.get("name", "")
            mac = node.get("mac", "")
            transport = node.get("transport", "")
            last_seen = node.get("last_seen_seconds_ago", "?")
            color = color_for_last_seen(last_seen)
            label = f"{color}{idx+1}. {mac}"
            if name:
                label += f" ({name})"
            label += f" [{transport}, last seen {last_seen}s ago]{Style.RESET_ALL if COLORAMA else ''}"
            print(label)
        print(f"{Fore.CYAN if COLORAMA else ''}{len(nodes)+1}. FF:FF:FF:FF:FF:FF (Broadcast){Style.RESET_ALL if COLORAMA else ''}")
        print(f"{Fore.CYAN if COLORAMA else ''}0. Exit{Style.RESET_ALL if COLORAMA else ''}")
        while True:
            choice = input(f"Select destination [1-{len(nodes)+1}, 0=exit, r=refresh, ?=help]: ").strip().lower()
            if choice == "0":
                print("Exiting.")
                sys.exit(0)
            if choice == 'r':
                break
            if choice == '?':
                print("\nHelp: Select a node by number, or press Enter for broadcast. 'r' refreshes the node list. '0' exits.")
                continue
            if not choice:
                confirm = input(f"{Fore.YELLOW if COLORAMA else ''}Send to ALL nodes (broadcast)? [y/N]: {Style.RESET_ALL if COLORAMA else ''}").strip().lower()
                if confirm == 'y':
                    return "FF:FF:FF:FF:FF:FF"
                else:
                    continue
            try:
                idx = int(choice) - 1
                if idx == len(nodes):
                    confirm = input(f"{Fore.YELLOW if COLORAMA else ''}Send to ALL nodes (broadcast)? [y/N]: {Style.RESET_ALL if COLORAMA else ''}").strip().lower()
                    if confirm == 'y':
                        return "FF:FF:FF:FF:FF:FF"
                    else:
                        continue
                if 0 <= idx < len(nodes):
                    return nodes[idx].get("mac", "FF:FF:FF:FF:FF:FF")
            except (ValueError, IndexError):
                print(f"{Fore.RED if COLORAMA else ''}Invalid selection. Try again.{Style.RESET_ALL if COLORAMA else ''}")
        # If we broke out of the inner loop, refresh node list

def is_valid_mac(mac):
    return bool(re.match(r"^([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$", mac))

def send_mesh_update(custom_text=None, dest_mac=None):
    url = f"http://{COORDINATOR_IP}/api/tx"
    now = datetime.datetime.now().strftime("%H:%M:%S")
    mac_to_use = dest_mac or "FF:FF:FF:FF:FF:FF"
    if not is_valid_mac(mac_to_use):
        print(f"{Fore.RED if COLORAMA else ''}Invalid MAC address: {mac_to_use}{Style.RESET_ALL if COLORAMA else ''}")
        return
    # If the message is a JSON string (options 3 or 4), send it as-is
    if custom_text and custom_text.strip().startswith('{') and custom_text.strip().endswith('}'):
        hex_payload = custom_text.encode('utf-8').hex().upper()
        print(f"Attempting to send: {custom_text} to {mac_to_use}")
    else:
        # For normal messages, prepend timestamp and wrap in brackets
        msg = custom_text if custom_text else random.choice(MESSAGES)
        full_string = f"[{now}] {msg}"
        hex_payload = full_string.encode('utf-8').hex().upper()
        print(f"Attempting to send: {full_string} to {mac_to_use}")
    payload = {
        "dest": mac_to_use,
        "appId": APP_ID_TEXT,
        "ttl": 4,
        "payload": hex_payload
    }
    try:
        response = requests.post(url, json=payload, timeout=5)
        if response.status_code == 200:
            print(f"{Fore.GREEN if COLORAMA else ''}Success! Coordinator Response: {response.json()}{Style.RESET_ALL if COLORAMA else ''}")
        else:
            print(f"{Fore.RED if COLORAMA else ''}Error: Server returned {response.status_code} - {response.text}{Style.RESET_ALL if COLORAMA else ''}")
    except requests.exceptions.ConnectionError:
        print(f"{Fore.RED if COLORAMA else ''}Failed: Could not connect to Coordinator at {COORDINATOR_IP}{Style.RESET_ALL if COLORAMA else ''}")
    except Exception as e:
        print(f"{Fore.RED if COLORAMA else ''}Error sending message: {e}{Style.RESET_ALL if COLORAMA else ''}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Send a mesh update message.")
    parser.add_argument('-t', '--text', type=str, help='Custom text to send instead of a random message')
    parser.add_argument('-l', '--loop', action='store_true', help='Enable loop mode to send messages repeatedly')
    parser.add_argument('-n', '--num', type=int, default=0, help='Number of messages to send (0 = infinite in loop mode)')
    parser.add_argument('-i', '--interval', type=float, default=2.0, help='Interval between messages in seconds (loop mode)')
    parser.add_argument('-d', '--dest', type=str, help='Destination MAC address (overrides interactive selection)')
    parser.add_argument('--no-prompt', action='store_true', help='Skip interactive node selection and use broadcast')
    parser.add_argument('--ident', action='store_true', help='Send identification message {"ric":8,"func":3,"msg":"PD2EMC"}')
    parser.add_argument('--ident-msg', type=str, default=None, help='Custom text for identification message (overrides default "PD2EMC")')
    parser.add_argument('--pager-time', action='store_true', help='Send pager date/time set message {"ric":224,"func":3,"msg":"YYYYMMDDHHMMSSddmmyyHHMMSS"}')
    parser.add_argument('--list', action='store_true', help='List all nodes and exit')
    args = parser.parse_args()

    if args.list:
        nodes = fetch_nodes()
        if not nodes:
            print("No nodes found.")
        else:
            print("\nNodes:")
            for idx, node in enumerate(nodes):
                mac = node.get("mac", "")
                name = node.get("name", "")
                transport = node.get("transport", "")
                last_seen = node.get("last_seen_seconds_ago", "?")
                label = f"{idx+1}. {mac}"
                if name:
                    label += f" ({name})"
                label += f" [{transport}, last seen {last_seen}s ago]"
                print(label)
        sys.exit(0)
    args = parser.parse_args()


    if args.dest:
        if not is_valid_mac(args.dest):
            print(f"{Fore.RED if COLORAMA else ''}Invalid MAC address provided with --dest: {args.dest}{Style.RESET_ALL if COLORAMA else ''}")
            sys.exit(1)
        dest_mac = args.dest
    elif args.no_prompt:
        dest_mac = "FF:FF:FF:FF:FF:FF"
    else:
        dest_mac = select_destination_mac()

    # Command-line message options
    if args.ident:
        ident_text = args.ident_msg or DEFAULT_IDENT_MSG_TEXT
        msg_text = f'{{"ric":8,"func":3,"msg":"{ident_text}"}}'
    elif args.pager_time:
        now = datetime.datetime.now()
        ric = 224
        func = 3
        description = "YYYYMMDDHHMMSS"
        dmyhms = now.strftime("%y%m%d%H%M%S")
        msg_text = f"{{\"ric\":{ric},\"func\":{func},\"msg\":\"{description}{dmyhms}\"}}"
    elif args.text is not None:
        msg_text = args.text
    else:
        print("\nChoose message type:")
        print("1. Send random message (default if Enter)")
        print("2. Enter custom text")
        print("3. Send pager date/time set message")
        print("4. Send identification message")
        print("(Press Enter to send a random message)")
        while True:
            choice = input("Select [1-4]: ").strip()
            if choice == "1" or choice == "":
                msg_text = None
                break
            elif choice == "2":
                msg_text = input("Enter your message: ")
                break
            elif choice == "3":
                now = datetime.datetime.now()
                ric = 224
                func = 3
                description = "YYYYMMDDHHMMSS"
                dmyhms = now.strftime("%y%m%d%H%M%S")
                pager_msg = f"{{\"ric\":{ric},\"func\":{func},\"msg\":\"{description}{dmyhms}\"}}"
                print(f"Pager time set message: {pager_msg}")
                msg_text = pager_msg
                break
            elif choice == "4":
                print("\nEnter identification text (or press Enter for default 'PD2EMC'):")
                custom_ident = input("> ").strip() or DEFAULT_IDENT_MSG_TEXT
                msg_text = f'{{"ric":8,"func":3,"msg":"{custom_ident}"}}'
                print(f"Identification message: {msg_text}")
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
