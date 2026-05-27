# mspm0-car-2024-h

MSPM0 car project for the 2026 school competition, based on the 2024 EDC H challenge.  
The repository contains both the rule-based competition control stack and an ML experimentation branch.

## Project Status

- Hardware platform: `MSPM0G3507 + FreeRTOS`
- Main control path: IMU + encoder + line sensor fusion
- Challenge coverage: `Q1 ~ Q4`
- Q4 measured completion time: about `55 s`
- Stable validation tag: `stable-q1-q4-20260521`

Current `main` is the practical competition branch.  
ML exploration is kept on `ml-data-driven-car`.

## What `stable-q1-q4-20260521` Means

Tag `stable-q1-q4-20260521` points to commit `b6adab5`:

- commit: `b6adab5677831d9092264635fd640b598e7123d3`
- message: `Mark Q1-Q4 challenge validation stable`

That tag marks the point where the project had already reached a stable Q1-Q4 validation baseline.

Changes added after that tag on `main` mainly include:

- event-driven buzzer / LED signaling
- Q4 A-point exit handling adjustments
- direct key mapping for `Q1/Q2/Q3/Q4`
- simplified OLED runtime display
- wheel response probe tool
- report code excerpts under `docs/report.md`

So the tag is a stable control milestone, not the final repository state.

## Repository Layout

```text
Control/              Control algorithms: PID, wheel, chassis, yaw, line
Drivers/              HAL and device drivers
Sources/              Application layer, app_state, tasks, signal handling
SysConfig/            MSPM0 SysConfig project and generated files
docs/                 Project notes, plans, reports, tuning records
tools/                Debugging, visualization, probing, data tools
archive/legacy/       Older control path kept only for reference
```

## Active Runtime Architecture

The current competition control path is based on five FreeRTOS tasks:

- `sensor_task`  
  Refreshes encoder, IMU, and line-sensor feedback every 10 ms

- `control_task`  
  Runs wheel closed-loop control and differential drive execution every 10 ms

- `main_task`  
  Runs the challenge state machine for `Q1 ~ Q4`

- `test_task`  
  Handles keys, serial commands, runtime tuning, and debug output

- `oled_task`  
  Displays compact challenge status and runtime information

Shared runtime data is centralized in:

- `Sources/app_state.h`
- `Sources/app_state.c`

## Control Strategy

The main challenge logic lives in:

- `Sources/tasks/main_task.c`

The control strategy is organized as phase primitives:

- `ALIGN_START`
- `GAP_TRAVERSE`
- `ARC_TRACK`
- `STOP_AND_SIGNAL`

Typical behavior:

- straight gap segments use IMU heading hold
- arc segments use line tracking
- Q4 uses a multi-lap state machine with special handling for repeated A-point passes and final stop

Control object construction and default parameters are concentrated in:

- `Sources/chassis_system.c`

## Key Branches

- `main`  
  Current competition branch

- `ml-data-driven-car`  
  Worktree-backed branch for data collection, offline analysis, and model-driven control experiments

- `vscode-linux-remote-workflow`  
  Separate branch for a Linux-first VS Code / Remote SSH workflow refactor; not merged into `main` yet

## Build Environment

Recommended host environment:

- Linux
- `arm-none-eabi-gcc`
- CMake + Ninja
- MSPM0 SDK
- TI SysConfig CLI

Key paths used in this project:

| Component | Path |
| --- | --- |
| Arm toolchain | `/usr/bin/arm-none-eabi-gcc` |
| MSPM0 SDK | `~/ti/mspm0_sdk_2_10_00_04/` |
| SysConfig CLI | `/opt/ccstudio/ccs/utils/sysconfig_1.26.0/sysconfig_cli.sh` |
| DSLite | `/opt/ccstudio/ccs/ccs_base/DebugServer/bin/DSLite` |

## Build

```sh
# Regenerate SysConfig output after modifying .syscfg
bash /opt/ccstudio/ccs/utils/sysconfig_1.26.0/sysconfig_cli.sh \
    --product ~/ti/mspm0_sdk_2_10_00_04/.metadata/product.json \
    --device MSPM0G3507 --package "LQFP-64(PM)" --compiler gcc \
    --script SysConfig/mspm0-school-2026.syscfg \
    --output SysConfig

# Configure and build
cmake -S . -B build -GNinja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build build

# Optional outputs
cmake --build build --target hex
cmake --build build --target bin
```

Build artifacts:

- `build/mspm0_school_2026.elf`
- `build/mspm0_school_2026.hex`
- `build/mspm0_school_2026.bin`
- `build/memory.map`

## Flashing and Debugging

Typical flashing options:

```sh
# TI DSLite
/opt/ccstudio/ccs/ccs_base/DebugServer/bin/DSLite load \
    -c targetConfigs/MSPM0G3507.ccxml \
    -f build/mspm0_school_2026.elf

# pyOCD
pyocd flash -t mspm0g3507 build/mspm0_school_2026.elf

# J-Link
JLinkExe -CommanderScript .vscode/jlink-flash.jlink
```

## Reports and Notes

Useful documents:

- [docs/H_PROBLEM_PLAN.md](docs/H_PROBLEM_PLAN.md)
- [docs/CONTROL_CODE_SUMMARY.md](docs/CONTROL_CODE_SUMMARY.md)
- [docs/REFACTOR_ARCHIVE_20260422.md](docs/REFACTOR_ARCHIVE_20260422.md)
- [docs/STAGE_SUMMARY.md](docs/STAGE_SUMMARY.md)
- [docs/report.md](docs/report.md)

## Notes

- `archive/legacy/` is kept for historical reference and comparison
- the ML branch is part of the same git repository; the worktree itself is local, but the branch is pushed normally
- the repository currently preserves multiple experiment branches because tuning and path-control validation were done incrementally
