#!/bin/bash
# Start pyOCD GDB server, detach from task lifecycle

PYOCD="/home/aki/.local/bin/pyocd"
GDB_PORT=3333
LOG="/tmp/pyocd-gdbserver.log"

# Kill stale instance
pkill -f "pyocd gdbserver" 2>/dev/null
sleep 0.3

# Start detached with nohup so it survives VS Code task termination
nohup "$PYOCD" gdbserver -t mspm0g3507 -p "$GDB_PORT" > "$LOG" 2>&1 &
PID=$!
disown $PID 2>/dev/null

# Wait for GDB port (30s timeout)
for i in $(seq 1 60); do
    if ss -tlnp 2>/dev/null | grep -q "127.0.0.1:$GDB_PORT"; then
        echo "pyOCD ready on port $GDB_PORT (pid $PID)"
        exit 0
    fi
    sleep 0.5
done

echo "ERROR: timeout waiting for pyOCD"
cat "$LOG"
exit 1
