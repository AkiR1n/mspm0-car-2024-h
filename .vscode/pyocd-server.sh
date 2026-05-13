#!/bin/bash
PYOCD="/home/aki/.local/bin/pyocd"
GDB_PORT=3333
LOG="/tmp/pyocd-gdbserver.log"

# Aggressively kill any stale pyocd
pkill -9 -f "pyocd.*gdbserver" 2>/dev/null
sleep 1

# Give USB device time to reset after previous session
sleep 0.5

# Start detached
nohup "$PYOCD" gdbserver -t mspm0g3507 -p "$GDB_PORT" > "$LOG" 2>&1 &
PID=$!
disown $PID 2>/dev/null

# Wait for GDB port (60s timeout)
for i in $(seq 1 120); do
    if ss -tlnp 2>/dev/null | grep -q ":$GDB_PORT "; then
        echo "pyOCD ready on port $GDB_PORT"
        exit 0
    fi
    # Check if pyocd died
    if ! kill -0 $PID 2>/dev/null; then
        echo "ERROR: pyOCD died during startup"
        cat "$LOG"
        exit 1
    fi
    sleep 0.5
done

echo "ERROR: timeout waiting for pyOCD"
cat "$LOG"
exit 1
