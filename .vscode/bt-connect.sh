#!/bin/bash
BT_ADDR="48:87:2D:7F:84:8D"
PORT="/tmp/vBT24"
UUID="0000ffe1-0000-1000-8000-00805f9b34fb"
VENV="$(dirname "$0")/../.venv/bin"

rm -f "$PORT"

# Just start ble-serial - bleak handles discovery internally
exec "$VENV/ble-serial" -d "$BT_ADDR" -p "$PORT" -r "$UUID" -w "$UUID" -t 30
