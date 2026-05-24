#!/usr/bin/env bash

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${REPO_ROOT}/build"
DEV_STATE_DIR="${BUILD_DIR}/.dev"
TARGET_NAME="mspm0_school_2026"

SYSCFG_CLI="${SYSCFG_CLI:-/opt/ccstudio/ccs/utils/sysconfig_1.26.0/sysconfig_cli.sh}"
SDK_ROOT="${MSPM0_SDK_ROOT:-${HOME}/ti/mspm0_sdk_2_10_00_04}"
SDK_PRODUCT_JSON="${SDK_PRODUCT_JSON:-${SDK_ROOT}/.metadata/product.json}"
DS_LITE_BIN="${DS_LITE_BIN:-/opt/ccstudio/ccs/ccs_base/DebugServer/bin/DSLite}"
TARGET_CONFIG="${TARGET_CONFIG:-${REPO_ROOT}/targetConfigs/MSPM0G3507.ccxml}"
ELF_PATH="${REPO_ROOT}/build/${TARGET_NAME}.elf"
HEX_PATH="${REPO_ROOT}/build/${TARGET_NAME}.hex"
BIN_PATH="${REPO_ROOT}/build/${TARGET_NAME}.bin"
JLINK_FLASH_SCRIPT="${REPO_ROOT}/.vscode/jlink-flash.jlink"

DEFAULT_SERIAL_BAUD="115200"
DEFAULT_WEB_DASHBOARD_HOST="127.0.0.1"
DEFAULT_WEB_DASHBOARD_PORT="8765"

dev_info() {
    printf '[dev] %s\n' "$*"
}

dev_warn() {
    printf '[dev] warn: %s\n' "$*" >&2
}

die() {
    printf '[dev] error: %s\n' "$*" >&2
    exit 1
}

require_linux_execution() {
    if [[ "$(uname -s)" != "Linux" ]]; then
        die "this workflow only supports Linux execution. Open the repository through VS Code Remote SSH to the Linux host and run the task there."
    fi
}

require_command() {
    local cmd="${1:?missing command name}"
    local hint="${2:-}"

    if ! command -v "$cmd" >/dev/null 2>&1; then
        if [[ -n "$hint" ]]; then
            die "missing command '${cmd}'. ${hint}"
        fi
        die "missing command '${cmd}' in PATH"
    fi
}

require_file() {
    local path="${1:?missing path}"
    local label="${2:-file}"

    [[ -f "$path" ]] || die "${label} not found: ${path}"
}

require_executable_file() {
    local path="${1:?missing path}"
    local label="${2:-file}"

    [[ -x "$path" ]] || die "${label} not found or not executable: ${path}"
}

ensure_dev_state_dir() {
    mkdir -p "$DEV_STATE_DIR"
}

list_serial_devices() {
    local found=1
    local cand

    for cand in /dev/ttyACM* /dev/ttyUSB*; do
        [[ -e "$cand" ]] || continue
        printf '%s\n' "$cand"
        found=0
    done

    return "$found"
}

resolve_serial_port() {
    local requested="${1:-auto}"
    local cand

    if [[ "$requested" != "auto" ]]; then
        [[ -e "$requested" ]] || die "serial port does not exist: ${requested}"
        printf '%s\n' "$requested"
        return 0
    fi

    if [[ -n "${MSPM0_SERIAL_PORT:-}" ]]; then
        [[ -e "$MSPM0_SERIAL_PORT" ]] || die "MSPM0_SERIAL_PORT points to a missing device: ${MSPM0_SERIAL_PORT}"
        printf '%s\n' "$MSPM0_SERIAL_PORT"
        return 0
    fi

    for cand in /dev/ttyACM0 /dev/ttyACM1 /dev/ttyACM2 /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyUSB2; do
        [[ -e "$cand" ]] || continue
        printf '%s\n' "$cand"
        return 0
    done

    die "could not auto-detect a serial device. Set MSPM0_SERIAL_PORT or pick a concrete /dev/ttyACM* or /dev/ttyUSB* path."
}

resolve_python() {
    if [[ -x "${REPO_ROOT}/.venv/bin/python" ]]; then
        printf '%s\n' "${REPO_ROOT}/.venv/bin/python"
        return 0
    fi

    require_command python3 "install Python 3 or restore the repository .venv"
    command -v python3
}

tcp_port_open() {
    local host="${1:?missing host}"
    local port="${2:?missing port}"

    (echo >"/dev/tcp/${host}/${port}") >/dev/null 2>&1
}

wait_for_tcp_port() {
    local host="${1:?missing host}"
    local port="${2:?missing port}"
    local timeout_s="${3:-10}"
    local attempts=$((timeout_s * 10))
    local idx

    for ((idx = 0; idx < attempts; idx++)); do
        if tcp_port_open "$host" "$port"; then
            return 0
        fi
        sleep 0.1
    done

    return 1
}
