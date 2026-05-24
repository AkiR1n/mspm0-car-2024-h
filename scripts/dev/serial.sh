#!/usr/bin/env bash

set -euo pipefail

source "$(dirname "$0")/common.sh"

require_linux_execution
require_command picocom "install picocom on the Linux host"

serial_port="$(resolve_serial_port "${1:-auto}")"
baud_rate="${2:-$DEFAULT_SERIAL_BAUD}"

dev_info "opening serial console on ${serial_port} @ ${baud_rate}"
exec picocom -b "$baud_rate" "$serial_port"
