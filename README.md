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

## Branch Policy

- `main` is the protected integration branch
- day-to-day experiments should stay on topic branches
- `ml-data-driven-car` is the long-lived ML research branch
- `vscode-linux-remote-workflow` is a pending infrastructure branch and should be merged only after hardware workflow validation
- historical tuning branches are preserved for traceability, not as the primary development target

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

## 中文说明

### 项目定位

这是一个面向 **2026 校赛** 的 MSPM0 智能小车项目，题目背景基于 **2024 电赛 H 题**。  
仓库同时保留了两条线：

- 比赛主线：基于规则控制的实车可运行版本
- 研究分支：基于数据采集与模型训练的 ML 实验版本

### 当前状态

- 当前主芯片为 `MSPM0G3507`
- 运行框架为 `FreeRTOS`
- 当前主控制方案为 `IMU + 编码器 + 7 路循迹` 融合控制
- `Q1 ~ Q4` 已完成实测验证
- `Q4` 当前实测完成时间约为 `55 s`

### 关键版本说明

- `stable-q1-q4-20260521`  
  表示 Q1~Q4 已经达到稳定验证通过的基线版本

- `school-final-2026`  
  表示校赛最终实际使用版本，对应提交 `87d75e4 Update challenge controls and OLED status`

### 当前主线结构

当前运行主线由 5 个 FreeRTOS 任务组成：

- `sensor_task`：采集编码器、IMU、循迹反馈
- `control_task`：执行双轮闭环与底盘差速控制
- `main_task`：执行 Q1~Q4 赛题状态机
- `test_task`：处理按键、串口命令、调试参数
- `oled_task`：显示题号、阶段和运行状态

共享状态统一收口在：

- `Sources/app_state.h`
- `Sources/app_state.c`

### 控制思路

当前主逻辑主要位于：

- `Sources/tasks/main_task.c`

整体思路是按动作原语组织赛题流程：

- `ALIGN_START`
- `GAP_TRAVERSE`
- `ARC_TRACK`
- `STOP_AND_SIGNAL`

其中：

- 直线无线段主要依靠 IMU 航向保持
- 圆弧段主要依靠循迹闭环
- `Q4` 使用多圈状态机，并对 A 点重复经过和最终停车做了专门处理

### 主要分支

- `main`：当前比赛主线
- `ml-data-driven-car`：ML 数据采集、训练与建模实验分支
- `vscode-linux-remote-workflow`：Linux / Remote SSH 工作流重构分支，尚未合入主线

### 分支策略

- `main` 作为受保护的主集成分支
- 日常实验和调参优先放在专题分支上进行
- `ml-data-driven-car` 作为长期保留的 ML 研究分支
- `vscode-linux-remote-workflow` 属于待验证后再合入的基础设施分支
- 其余历史调参分支主要用于追溯，不作为默认开发入口

### 补充说明

- `archive/legacy/` 目录保留历史控制代码，仅用于参考
- ML worktree 是本地工作方式，真正上传到 GitHub 的仍然是同一个仓库中的 `ml-data-driven-car` 分支
- 仓库中保留了较多实验分支，因为路径控制、循迹和 Q4 终点行为是逐步调出来的
