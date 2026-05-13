#!/bin/bash
# Start pyOCD GDB server for MSPM0G3507 and wait until it's ready

PYOCD="/home/aki/.local/bin/pyocd"
GDB_PORT=3333

# Kill any stale instance
pkill -f "pyocd gdbserver" 2>/dev/null
sleep 0.5

# Start pyOCD in background
$PYOCD gdbserver -t mspm0g3507 -p $GDB_PORT &
PID=$!

# Wait for GDB port to be ready (timeout 30s)
for i in $(seq 1 60); do
    if ss -tlnp | grep -q ":$GDB_PORT "; then
        echo "pyOCD GDB server ready on port $GDB_PORT"
        exit 0
    fi
    sleep 0.5
done

echo "ERROR: pyOCD GDB server failed to start"
kill $PID 2>/dev/null
exit 1
