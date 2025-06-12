#!/bin/bash

# Exit immediately on error
set -e

echo "[host] Loading kernel modules..."

# Load required modules if not already loaded
for mod in can can_raw gs_usb; do
    if ! lsmod | grep -q "^$mod"; then
        echo "[host] Loading $mod..."
        sudo modprobe $mod
    fi
done

# Wait for device (optional but safer)
sleep 1

# Check if can0 is present
if ! ip link show can0 &>/dev/null; then
    echo "[host] ERROR: can0 not detected. Is the CAN-USB adapter plugged in?"
    exit 1
fi

# Bring up can0 if not already up
if ! ip link show can0 | grep -q "state UP"; then
    echo "[host] Bringing up can0 at 500k..."
    sudo ip link set can0 up type can bitrate 500000
else
    echo "[host] can0 already up"
fi

# Optional: Confirm CAN traffic
echo "[host] Checking for CAN traffic..."
timeout 1s candump can0 | grep -q . && echo "[host] CAN traffic detected ✅" || echo "[host] No CAN traffic yet (this may be normal if robot is idle)"

echo "[host] Host CAN setup complete."
