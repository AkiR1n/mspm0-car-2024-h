#!/bin/bash
# BLE serial bridge startup script
# Resets BlueZ cache for BT24, then launches ble-serial

BT_ADDR="48:87:2D:7F:84:8D"
PORT="/tmp/vBT24"
UUID="0000ffe1-0000-1000-8000-00805f9b34fb"
VENV="$(dirname "$0")/../.venv/bin"

rm -f "$PORT"

echo "Clearing BT24 cache..."
bluetoothctl remove "$BT_ADDR" 2>/dev/null

echo "Starting ble-serial bridge..."
exec "$VENV/ble-serial" -d "$BT_ADDR" -p "$PORT" -r "$UUID" -w "$UUID" -t 20
