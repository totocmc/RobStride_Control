#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RobStride CAN Bus Scan Tool (Python Version)

Usage: python3 scan_bus.py [channel]
Examples:
    sudo python3 scan_bus.py can0
    sudo python3 scan_bus.py can1

This script will ping all IDs in range 1-254 and report responding motors.
**Note:** Accessing CAN hardware usually requires 'sudo' privileges.
"""

import sys
import os
import time

# --- Import SDK ---
# Assume this script is in the same directory as position_control_mit.py
# (i.e., parent directory is the SDK root)
try:
    # Try to add SDK root to path
    sdk_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if sdk_path not in sys.path:
        sys.path.insert(0, sdk_path)
    
    # 1. Try to import from installed package
    from robstride_dynamics import RobstrideBus
except ImportError:
    # 2. Try to import from local files (if SDK not installed)
    try:
        print("'robstride_dynamics' package not found, trying local import...")
        from bus import RobstrideBus
    except ImportError as e:
        print(f"❌ Failed to import RobstrideBus SDK: {e}")
        print("Make sure the parent directory of this script is the SDK root,")
        print("or SDK has been installed via 'pip install -e .'.")
        sys.exit(1)

def main():
    # --- 1. Get CAN channel ---
    if len(sys.argv) > 1:
        channel = sys.argv[1]
    else:
        channel = "can0"

    print(f"🚀 RobStride Bus Scan Tool")
    print(f"📡 Scanning channel: {channel}")
    print(f"🔍 Search range: ID 1 to 254")
    print("...")
    time.sleep(1) # Pause to let user read

    # --- 2. Run scan ---
    found_motors = None
    try:
        # RobstrideBus.scan_channel implements all logic for us
        # It internally uses tqdm to show progress bar
        found_motors = RobstrideBus.scan_channel(channel, start_id=1, end_id=255) # end_id=255 scans up to 254
    
    except Exception as e:
        print(f"\n❌ Scan error: {e}")
        if "Operation not permitted" in str(e) or "Permission denied" in str(e):
            print("🔑 Permission error: run this script with 'sudo' to access CAN hardware.")
            print(f"   Example: sudo python3 {sys.argv[0]} {channel}")
        elif "No such device" in str(e):
            print(f"🔌 Device error: CAN interface '{channel}' not found.")
        sys.exit(1)

    # --- 3. Print results ---
    if not found_motors:
        print("\n🚫 No responding motors found on bus.")
    else:
        print("\n✅ Scan complete! Found motors:")
        print("=" * 60)
        print(f"{'Motor ID':<10} | {'MCU Unique ID (UUID)':<45}")
        print("-" * 60)
        
        # found_motors is a dict: {id: (id, uuid_bytearray)}
        # Sort by ID
        for motor_id in sorted(found_motors.keys()):
            # value is a tuple (id, uuid)
            _id, uuid = found_motors[motor_id]
            
            # Convert bytearray to readable hex string
            uuid_hex = uuid.hex() # 'hex()' is a bytearray method
            
            print(f"{motor_id:<10} | {uuid_hex}")
            
        print("=" * 60)

if __name__ == "__main__":
    main()