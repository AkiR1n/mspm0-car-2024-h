#!/usr/bin/env bash

set -euo pipefail

source "$(dirname "$0")/common.sh"

usage() {
    cat <<'EOF'
usage:
  scripts/dev/python_tool.sh imu-monitor [serial-port|auto] [baud]
  scripts/dev/python_tool.sh vehicle-dashboard [serial-port|auto] [baud]
  scripts/dev/python_tool.sh web-dashboard [serial-port|auto] [baud] [listen-port]
EOF
}

require_linux_execution

tool="${1:-}"
serial_port_arg="${2:-auto}"
baud_rate="${3:-$DEFAULT_SERIAL_BAUD}"
python_bin="$(resolve_python)"

case "$tool" in
    imu-monitor)
        serial_port="$(resolve_serial_port "$serial_port_arg")"
        require_file "${REPO_ROOT}/tools/imu_monitor.py" "IMU monitor script"
        dev_info "starting imu monitor on ${serial_port} @ ${baud_rate}"
        exec "$python_bin" "${REPO_ROOT}/tools/imu_monitor.py" "$serial_port" "$baud_rate"
        ;;
    vehicle-dashboard)
        serial_port="$(resolve_serial_port "$serial_port_arg")"
        require_file "${REPO_ROOT}/tools/vehicle_dashboard.py" "vehicle dashboard script"
        dev_info "starting vehicle dashboard on ${serial_port} @ ${baud_rate}"
        exec "$python_bin" "${REPO_ROOT}/tools/vehicle_dashboard.py" "$serial_port" "$baud_rate"
        ;;
    web-dashboard)
        serial_port="$(resolve_serial_port "$serial_port_arg")"
        listen_port="${4:-$DEFAULT_WEB_DASHBOARD_PORT}"
        require_file "${REPO_ROOT}/tools/web_dashboard.py" "web dashboard script"
        dev_info "starting web dashboard on ${DEFAULT_WEB_DASHBOARD_HOST}:${listen_port} for ${serial_port} @ ${baud_rate}"
        exec "$python_bin" "${REPO_ROOT}/tools/web_dashboard.py" \
            --host "$DEFAULT_WEB_DASHBOARD_HOST" \
            --port "$listen_port" \
            --serial-port "$serial_port" \
            --baud "$baud_rate"
        ;;
    *)
        usage >&2
        exit 1
        ;;
esac
