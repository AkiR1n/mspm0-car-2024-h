#!/usr/bin/env bash

set -euo pipefail

source "$(dirname "$0")/common.sh"

PORT="${PYOCD_GDB_PORT:-3333}"
PID_FILE="${DEV_STATE_DIR}/pyocd-gdbserver.pid"
LOG_FILE="${DEV_STATE_DIR}/pyocd-gdbserver.log"

server_pid() {
    [[ -f "$PID_FILE" ]] || return 1
    tr -d '[:space:]' <"$PID_FILE"
}

server_running() {
    local pid
    pid="$(server_pid)" || return 1
    kill -0 "$pid" 2>/dev/null
}

start_server() {
    local pid

    require_linux_execution
    require_command pyocd "install pyOCD on the Linux host"
    ensure_dev_state_dir

    if server_running; then
        dev_info "pyOCD gdbserver already running on 127.0.0.1:${PORT} (pid $(server_pid))"
        dev_info "log: ${LOG_FILE}"
        return 0
    fi

    if tcp_port_open 127.0.0.1 "$PORT"; then
        dev_warn "tcp port ${PORT} is already in use; assuming another gdb server is already available"
        return 0
    fi

    rm -f "$PID_FILE"

    dev_info "starting pyOCD gdbserver on 127.0.0.1:${PORT}"
    nohup pyocd gdbserver -t mspm0g3507 --persist --port "$PORT" >"$LOG_FILE" 2>&1 &
    pid=$!
    printf '%s\n' "$pid" >"$PID_FILE"

    if ! wait_for_tcp_port 127.0.0.1 "$PORT" 10; then
        dev_warn "pyOCD gdbserver did not become ready within 10s"
        if [[ -f "$LOG_FILE" ]]; then
            tail -n 40 "$LOG_FILE" >&2 || true
        fi
        kill "$pid" 2>/dev/null || true
        rm -f "$PID_FILE"
        exit 1
    fi

    dev_info "pyOCD gdbserver ready on 127.0.0.1:${PORT} (pid ${pid})"
    dev_info "log: ${LOG_FILE}"
}

stop_server() {
    if server_running; then
        local pid
        pid="$(server_pid)"
        dev_info "stopping pyOCD gdbserver pid ${pid}"
        kill "$pid" 2>/dev/null || true
        rm -f "$PID_FILE"
        return 0
    fi

    rm -f "$PID_FILE"
    dev_info "pyOCD gdbserver is not running"
}

status_server() {
    if server_running; then
        dev_info "pyOCD gdbserver is running on 127.0.0.1:${PORT} (pid $(server_pid))"
        dev_info "log: ${LOG_FILE}"
        return 0
    fi

    if tcp_port_open 127.0.0.1 "$PORT"; then
        dev_warn "tcp port ${PORT} is in use, but it is not managed by ${PID_FILE}"
        return 0
    fi

    dev_info "pyOCD gdbserver is not running"
}

usage() {
    cat <<'EOF'
usage:
  scripts/dev/debug_server.sh start
  scripts/dev/debug_server.sh stop
  scripts/dev/debug_server.sh status
EOF
}

action="${1:-start}"

case "$action" in
    start)
        start_server
        ;;
    stop)
        stop_server
        ;;
    status)
        status_server
        ;;
    *)
        usage >&2
        exit 1
        ;;
esac
