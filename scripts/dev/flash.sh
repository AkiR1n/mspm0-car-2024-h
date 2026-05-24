#!/usr/bin/env bash

set -euo pipefail

source "$(dirname "$0")/common.sh"

usage() {
    cat <<'EOF'
usage:
  scripts/dev/flash.sh pyocd
  scripts/dev/flash.sh dslite-elf
  scripts/dev/flash.sh dslite-hex
  scripts/dev/flash.sh jlink
EOF
}

require_linux_execution

backend="${1:-}"

case "$backend" in
    pyocd)
        require_command pyocd "install pyOCD on the Linux host"
        require_file "$ELF_PATH" "ELF artifact"
        exec pyocd flash -t mspm0g3507 "$ELF_PATH"
        ;;
    dslite-elf)
        require_executable_file "$DS_LITE_BIN" "DSLite"
        require_file "$TARGET_CONFIG" "target config"
        require_file "$ELF_PATH" "ELF artifact"
        exec "$DS_LITE_BIN" load -c "$TARGET_CONFIG" -f "$ELF_PATH"
        ;;
    dslite-hex)
        require_executable_file "$DS_LITE_BIN" "DSLite"
        require_file "$TARGET_CONFIG" "target config"
        require_file "$HEX_PATH" "HEX artifact"
        exec "$DS_LITE_BIN" load -c "$TARGET_CONFIG" -f "$HEX_PATH"
        ;;
    jlink)
        require_command JLinkExe "install SEGGER J-Link tools on the Linux host"
        require_file "$JLINK_FLASH_SCRIPT" "J-Link flash script"
        exec JLinkExe -CommanderScript "$JLINK_FLASH_SCRIPT"
        ;;
    *)
        usage >&2
        exit 1
        ;;
esac
