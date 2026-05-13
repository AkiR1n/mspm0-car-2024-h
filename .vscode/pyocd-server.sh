#!/bin/bash
PYOCD="/home/aki/.local/bin/pyocd"
GDB_PORT=3333
LOG="/tmp/pyocd-gdbserver.log"

# If pyocd is already running, just verify port and exit
if ss -tlnp 2>/dev/null | grep -q ":$GDB_PORT "; then
    echo "pyOCD already running on port $GDB_PORT"
    exit 0
fi

# Start detached
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
