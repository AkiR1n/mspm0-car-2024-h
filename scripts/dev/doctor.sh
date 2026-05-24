#!/usr/bin/env bash

set -euo pipefail

source "$(dirname "$0")/common.sh"

status=0

check_required_command() {
    local cmd="$1"

    if command -v "$cmd" >/dev/null 2>&1; then
        printf 'ok    %-18s %s\n' "$cmd" "$(command -v "$cmd")"
    else
        printf 'miss  %-18s\n' "$cmd"
        status=1
    fi
}

check_optional_command() {
    local cmd="$1"

    if command -v "$cmd" >/dev/null 2>&1; then
        printf 'ok    %-18s %s\n' "$cmd" "$(command -v "$cmd")"
    else
        printf 'skip  %-18s not installed\n' "$cmd"
    fi
}

check_required_file() {
    local label="$1"
    local path="$2"

    if [[ -e "$path" ]]; then
        printf 'ok    %-18s %s\n' "$label" "$path"
    else
        printf 'miss  %-18s %s\n' "$label" "$path"
        status=1
    fi
}

check_optional_file() {
    local label="$1"
    local path="$2"

    if [[ -e "$path" ]]; then
        printf 'ok    %-18s %s\n' "$label" "$path"
    else
        printf 'skip  %-18s %s\n' "$label" "$path"
    fi
}

require_linux_execution

printf 'repo  %s\n' "$REPO_ROOT"
printf 'host  %s\n' "$(uname -srmo 2>/dev/null || uname -a)"
printf '\n'

printf 'required tools\n'
check_required_command bash
check_required_command cmake
check_required_command ninja
check_required_command arm-none-eabi-gcc
check_required_command arm-none-eabi-gdb
check_required_command python3
printf '\n'

printf 'optional tools\n'
check_optional_command picocom
check_optional_command pyocd
check_optional_command JLinkExe
check_optional_command JLinkGDBServer
check_optional_command openocd
printf '\n'

printf 'required files\n'
check_required_file syscfg_cli "$SYSCFG_CLI"
check_required_file sdk_product "$SDK_PRODUCT_JSON"
check_required_file target_config "$TARGET_CONFIG"
check_required_file openocd_cfg "${REPO_ROOT}/.vscode/openocd.cfg"
printf '\n'

printf 'optional files\n'
check_optional_file repo_venv "${REPO_ROOT}/.venv/bin/python"
check_optional_file jlink_script "$JLINK_FLASH_SCRIPT"
printf '\n'

printf 'serial devices\n'
if serial_list="$(list_serial_devices 2>/dev/null)"; then
    while IFS= read -r dev; do
        printf 'ok    %-18s %s\n' "serial" "$dev"
    done <<<"$serial_list"
else
    printf 'skip  %-18s none detected\n' "serial"
fi
printf '\n'

printf 'build outputs\n'
for artifact in "$ELF_PATH" "$HEX_PATH" "$BIN_PATH"; do
    if [[ -e "$artifact" ]]; then
        printf 'ok    %-18s %s\n' "artifact" "$artifact"
    else
        printf 'skip  %-18s %s\n' "artifact" "$artifact"
    fi
done

exit "$status"
