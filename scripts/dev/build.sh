#!/usr/bin/env bash

set -euo pipefail

source "$(dirname "$0")/common.sh"

run_configure() {
    require_linux_execution
    require_command cmake
    require_command ninja "install Ninja on the Linux host"
    require_command arm-none-eabi-gcc "install the Arm embedded toolchain on the Linux host"
    ensure_dev_state_dir

    exec cmake \
        -S "$REPO_ROOT" \
        -B "$BUILD_DIR" \
        -GNinja \
        -DCMAKE_BUILD_TYPE=Debug \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
}

ensure_configured() {
    if [[ -f "${BUILD_DIR}/build.ninja" && -f "${BUILD_DIR}/CMakeCache.txt" ]]; then
        return 0
    fi

    dev_info "build directory is not configured yet; running configure first"
    cmake \
        -S "$REPO_ROOT" \
        -B "$BUILD_DIR" \
        -GNinja \
        -DCMAKE_BUILD_TYPE=Debug \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
}

run_build() {
    require_linux_execution
    require_command cmake
    require_command ninja "install Ninja on the Linux host"
    require_command arm-none-eabi-gcc "install the Arm embedded toolchain on the Linux host"
    ensure_configured
    exec cmake --build "$BUILD_DIR" "$@"
}

usage() {
    cat <<'EOF'
usage:
  scripts/dev/build.sh configure
  scripts/dev/build.sh build
  scripts/dev/build.sh rebuild
  scripts/dev/build.sh target <hex|bin|...>
EOF
}

action="${1:-}"

case "$action" in
    configure)
        run_configure
        ;;
    build)
        run_build
        ;;
    rebuild)
        run_build --clean-first
        ;;
    target)
        target_name="${2:-}"
        [[ -n "$target_name" ]] || die "missing build target name"
        run_build --target "$target_name"
        ;;
    *)
        usage >&2
        exit 1
        ;;
esac
