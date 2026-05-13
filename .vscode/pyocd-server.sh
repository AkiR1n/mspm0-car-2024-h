#!/bin/bash
PYOCD="/home/aki/.local/bin/pyocd"
GDB_PORT=3333
LOG="/tmp/pyocd-gdbserver.log"

# Already running — nothing to do
if ss -tlnp 2>/dev/null | grep -q ":$GDB_PORT "; then
    echo "pyOCD already running on port $GDB_PORT"
    exit 0
fi

# Not running but port is busy (stale/crashed instance)
if ss -tlnp 2>/dev/null | grep -q ":$GDB_PORT "; then
    echo "ERROR: port $GDB_PORT is busy but not pyOCD"
    echo "Try: unplug/replug CMSIS-DAP, then F5 again"
    exit 1
fi

# Start fresh
echo "Starting pyOCD gdbserver..."
nohup "$PYOCD" gdbserver -t mspm0g3507 -p "$GDB_PORT" > "$LOG" 2>&1 &
PID=$!
disown $PID 2>/dev/null

for i in $(seq 1 120); do
    if ss -tlnp 2>/dev/null | grep -q ":$GDB_PORT "; then
        echo "pyOCD ready on port $GDB_PORT"
        exit 0
    fi
    if ! kill -0 $PID 2>/dev/null; then
        echo "ERROR: pyOCD died"
        cat "$LOG"
        exit 1
    fi
    sleep 0.5
done

echo "ERROR: timeout"
exit 1
