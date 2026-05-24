#!/usr/bin/env bash

set -euo pipefail

source "$(dirname "$0")/common.sh"

require_linux_execution
require_executable_file "$SYSCFG_CLI" "SysConfig CLI"
require_file "$SDK_PRODUCT_JSON" "MSPM0 SDK product metadata"
require_file "${REPO_ROOT}/SysConfig/mspm0-school-2026.syscfg" "SysConfig script"

exec "$SYSCFG_CLI" \
    --product "$SDK_PRODUCT_JSON" \
    --device MSPM0G3507 \
    --package "LQFP-64(PM)" \
    --compiler gcc \
    --script "${REPO_ROOT}/SysConfig/mspm0-school-2026.syscfg" \
    --output "${REPO_ROOT}/SysConfig"
