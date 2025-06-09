#!/bin/bash

bunker_can_setup() {
    WS=$HOME/base_ws
    SCRIPT_DIR="$WS/src/ugv_sdk/scripts"
    SETUP_MARKER="/tmp/bunker_can_setup_done"

    # 1. Run setup script once only (first-time setup)
    if [ ! -f "$SETUP_MARKER" ]; then
        echo "[bunker] Running first-time CAN setup..."
        bash "$SCRIPT_DIR/setup_can2usb.bash" && touch "$SETUP_MARKER"
    fi

    # 2. Bring up can0 if not already up
    if ! ip link show can0 &>/dev/null || ! ip link show can0 | grep -q "state UP"; then
        echo "[bunker] Bringing up can0 at 500k..."
        bash "$SCRIPT_DIR/bringup_can2usb_500k.bash" || {
            echo "[bunker] Failed to bring up can0"
            return 1
        }
    fi
}

# Run on shell load
bunker_can_setup
