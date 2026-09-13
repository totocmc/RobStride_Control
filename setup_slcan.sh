#!/bin/bash
# SLCAN USB CH340 Setup Script for RobStride
# Run with: sudo ./setup_slcan.sh [serial_device] [can_interface]

set -e

SERIAL_DEVICE=${1:-/dev/ttyUSB0}
CAN_INTERFACE=${2:-can0}
BITRATE=${3:-1000000}

echo "🔧 Setting up SLCAN for RobStride"
echo "=================================="
echo "Serial device: $SERIAL_DEVICE"
echo "CAN interface: $CAN_INTERFACE"
echo "Bitrate: $BITRATE"
echo ""

# Check if serial device exists
if [ ! -e "$SERIAL_DEVICE" ]; then
    echo "❌ Error: Serial device $SERIAL_DEVICE not found"
    echo "Available serial devices:"
    ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "  None found"
    exit 1
fi

# Check if slcand is installed
if ! command -v slcand &> /dev/null; then
    echo "❌ Error: slcand not found"
    echo "Install with: sudo apt-get install can-utils"
    exit 1
fi

# Stop any existing slcand on this interface
echo "🛑 Stopping any existing slcand on $CAN_INTERFACE..."
sudo pkill -f "slcand.*$CAN_INTERFACE" 2>/dev/null || true
sudo ip link set down $CAN_INTERFACE 2>/dev/null || true
sleep 1

# Start slcand
# -o = open immediately
# -s8 = 1Mbps (8 = 1000000 baud for SLCAN)
# -S = bitrate (for kernel)
echo "🚀 Starting slcand..."
sudo slcand -o -s8 -S $BITRATE $SERIAL_DEVICE $CAN_INTERFACE

# Wait for interface to come up
sleep 2

# Bring up the interface
echo "📡 Bringing up $CAN_INTERFACE..."
sudo ip link set up $CAN_INTERFACE

# Verify
echo ""
echo "✅ Setup complete!"
echo ""
echo "Verify with:"
echo "  ip link show $CAN_INTERFACE"
echo "  candump $CAN_INTERFACE"
echo ""
echo "To use with RobStride examples:"
echo "  export CAN_INTERFACE=$CAN_INTERFACE"
echo "  python3 python/examples/basic_usage.py 11"
echo "  # or"
echo "  ./cpp/examples/basic_control 11 $CAN_INTERFACE"