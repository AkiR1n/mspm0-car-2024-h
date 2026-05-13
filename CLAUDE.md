# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目定位

2026 校赛主工程，目标芯片 **MSPM0G3507**（Cortex-M0+ @ 80 MHz, 128 KB Flash, 32 KB SRAM）。当前运行 FreeRTOS 11.1.0 + CMake + Ninja 构建。

## 构建命令

```sh
# SysConfig 重新生成（改动 .syscfg 后必须执行）
bash /opt/ccstudio/ccs/utils/sysconfig_1.26.0/sysconfig_cli.sh \
    --product ~/ti/mspm0_sdk_2_10_00_04/.metadata/product.json \
    --device MSPM0G3507 --package "LQFP-64(PM)" --compiler gcc \
    --script SysConfig/mspm0-school-2026.syscfg --output SysConfig

# 配置 + 编译
cmake -S . -B build -GNinja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build build

# 导出 hex / bin
cmake --build build --target hex
cmake --build build --target bin

# DSLite 烧录（CCS 自带工具）
/opt/ccstudio/ccs/ccs_base/DebugServer/bin/DSLite load \
    -c targetConfigs/MSPM0G3507.ccxml -f build/mspm0_school_2026.elf

# J-Link 烧录
/usr/bin/JLinkExe -CommanderScript .vscode/jlink-flash.jlink

# 串口（有线 J-Link VCOM）
picocom -b 115200 /dev/ttyACM0
```

VS Code 中上述命令已配置为 `.vscode/tasks.json` 中的 task（`syscfg`, `configure`, `build`, `hex`, `bin`, `flash-elf`, `flash-jlink`, `serial`），`Ctrl+Shift+P → Run Task` 即可。

产物：`build/mspm0_school_2026.{elf,hex,bin}`, `build/memory.map`, `build/compile_commands.json`

## 工具链路径

| 组件 | 路径 |
|------|------|
| arm-none-eabi-gcc | `/usr/bin/arm-none-eabi-gcc`（14.2.0） |
| MSPM0 SDK | `~/ti/mspm0_sdk_2_10_00_04/` |
| SysConfig CLI | `/opt/ccstudio/ccs/utils/sysconfig_1.26.0/sysconfig_cli.sh` |
| DSLite | `/opt/ccstudio/ccs/ccs_base/DebugServer/bin/DSLite` |
| J-Link | `/usr/bin/JLinkExe` + `/usr/bin/JLinkGDBServer` |

## 架构：分层 + 任务

### 分层结构（从下到上）

1. **Drivers/Hal/** — 硬件薄封装（`motor_hal`, `encoder_hal`），直接操作寄存器/PWM
2. **Drivers/Devices/** — 设备对象层（`motor_drv`, `encoder_drv`, `imu_drv`, `line_sensor`），封装 ISR 和状态
3. **Control/** — 控制算法（`pid`, `wheel` 单轮速度环, `chassis` 差速解算, `yaw_controller` 航向保持, `line_controller` 线位置 PID）
4. **Sources/** — 应用层（`app_state` 全局状态, `chassis_system` 控制装配, `bt_uart` 蓝牙串口, `uart_rx`, `uart_printf`）
5. **Sources/tasks/** — 5 个 FreeRTOS 任务，每个周期读取 app_state 写反馈/读命令

### 5 任务表

| 任务 | 优先级 | 栈 words | 周期 | 职责 |
|------|--------|----------|------|------|
| `sensor_task` | 6 | 512 | 10 ms | 刷新编码器/IMU/循迹，组装 `chassis_feedback_t`，写入 app_state |
| `control_task` | 5 | 512 | 10 ms | 根据 mode 执行 `Twist(v,w)` 或 `WHEEL_TEST` 控制，输出电机 duty，写入 debug 状态 |
| `main_task` | 3 | 512 | 10 ms | 赛题状态机（Q1–Q4），执行相位动作（ALIGN/GAP/ARC/STOP），**只写命令不碰电机** |
| `test_task` | 2 | 512 | 50 ms | Key 切换题目/启动停止，串口命令解析（UART0 + BT 双通道），在线改 PID，状态打印 |
| `oled_task` | 1 | 384 | 100 ms | OLED 显示题号/相位/事件/观测量 |

`sensor_task` 和 `control_task` 之间通过 `app_state` 的中介结构（`chassis_feedback_t`, `chassis_command_t`, `chassis_debug_t`）解耦，而非直接共享指针。

### 控制链数据流

```
sensor_task → app_state.feedback  → control_task → app_state.debug
main_task   → app_state.command   → control_task (消费 command)
main_task   → app_state.challenge → test_task/oled_task (读取显示)
```

### 中断绑定

- `GROUP1_IRQHandler → Encoder_OnEdgeIRQ()` — 编码器 GPIO 边沿计数
- `TIMA1_IRQHandler → Encoder_OnSampleTick()` — 编码器周期采样计时
- `UART0_IRQHandler → uart_rx_irq_handler()` — 有线串口接收（115200）
- `UART1_IRQHandler → bt_uart_irq_handler()` — 蓝牙串口接收（9600）
- SysTick 由 FreeRTOS 独占，**应用代码不得定义 SysTick_Handler**

## 引脚分配

### 电机与编码器

| 物理侧 | 软件标识 | PWM | IN1 | IN2 | 编码器 A1 | 编码器 A2 |
|--------|----------|-----|-----|-----|-----------|-----------|
| 右轮 | s_left_* | PB14 | PB9 | PB10 | PB11 | PB4 |
| 左轮 | s_right_* | PA7 | PB7 | PB6 | PB12 | PB5 |

注意：软件侧 `s_left_*` 映射到物理**右**轮，`s_right_*` 映射到物理**左**轮（`Sources/chassis_system.c` 中 HAL ID 交换）。

电机 HAL 已对称化：前进=IN1高/IN2低，后退=IN1低/IN2高，停车=IN1低/IN2低。

### 串口

| 串口 | TX | RX | 波特率 | 用途 |
|------|----|----|--------|------|
| UART0 | PA10 | PA11 | 115200 | J-Link 有线调试 |
| UART1 (BT) | PA8 | PA9 | 9600 | BT24 BLE 无线调试 |

### 其他

| 功能 | 引脚 | 说明 |
|------|------|------|
| I²C MPU6050 | PA12 (SDA), PA13 (SCL) | I2C1 |
| I²C OLED | PA0 (SDA), PA1 (SCL) | I2C2 |
| 循迹 | PA14, PA16, PA25, PB17, PB19, PB20, PB25 | 7 路 |
| Key1 | PB4 附近 | 题号切换 |
| Key2 | — | 启动/停止 |
| LED1, LED2, BEEP | — | 声光指示 |

## 蓝牙 BLE 无线调试

### 架构

- **固件侧**：`Sources/bt_uart.c/h` — UART1 驱动（1024 字节 TX 环形缓冲 + 中断发送，128 字节 RX 环形缓冲 + 中断接收）
- **PC 侧**：`.vscode/bt-connect.sh` — `ble-serial` 桥接 BT24 → `/tmp/vBT24` 虚拟串口
- **双通道**：`test_task` 同时从 UART0 和 BT 读取命令，`uart_printf` 输出镜像到两个通道

### 使用方式

```sh
# VS Code: Run Task → bt-serial（启动 BLE 桥接，后台）
# VS Code: Run Task → bt-monitor（picocom 连 /tmp/vBT24，9600 baud）
# 或终端:
picocom -b 9600 /tmp/vBT24
```

BT24 模块每次连接前最好断电重启，否则 bleak 可能扫不到。

### 常见问题

- **BT 扫不到**：给 MSPM0 断电重启（BT24 跟着重启），等 10 秒再扫
- **9600 较慢**：TX 缓冲满时会整条丢弃消息（不截断），高速刷屏时 BT 可能丢数据；UART0 115200 不受影响
- **VS Code Serial Monitor 只扫 `/dev/ttyUSB*`**：所以改用 picocom + `/tmp/vBT24`

## 赛题状态机关键结构

`main_task.c` 采用「动作原语 + 相位表」架构，不是每题独立写控制代码。

**4 种动作原语：**
- `ALIGN_START` — 起跑前静止确认
- `GAP_TRAVERSE` — 无线段穿越（IMU 航向 + 编码器差 + 速度差修正）
- `ARC_TRACK` — 半圆弧循迹（纯循迹闭环，前/中/后段分区参数）
- `STOP_AND_SIGNAL` — 停车 + 事件发射

**Q1-Q4 题目路径**通过相位枚举串联这些动作原语。每个弧线有独立参数 profile（`k_arc_profile_bc`, `k_arc_profile_cb`, `k_arc_profile_da`），`BC` 与 `CB` 已分开配置。

## 当前模式系统

`app_mode_t` 控制 control_task 的行为：

| 模式 | 含义 | 控制方式 |
|------|------|----------|
| `APP_MODE_STOP` | 停车 | 电机 stop |
| `APP_MODE_TWIST_OPEN` | 开环速度 | 直接给底盘 v, w |
| `APP_MODE_WHEEL_TEST` | 开环占空比 | 直接给左右电机 duty |
| `APP_MODE_WHEEL_SPEED_TEST` | 闭环速度 | 左右轮独立速度闭环 |
| `APP_MODE_MAIN` | 赛题 | 状态机接管所有控制 |

通过 `test_task` 的 Key1/Key2 和串口命令切换。

### 串口调试命令

| 命令 | 示例 | 说明 |
|------|------|------|
| `l%,r%` | `20,20` | 开环占空比 0–100%（WHEEL_TEST） |
| `spd,l,r` | `spd,0.3,0.3` | 左右轮闭环速度 m/s |
| `twist,v,w` | `twist,0.3,0` | 差速底盘 v(m/s), w(rad/s) |
| `stop` | — | 停车 |
| `q1` … `q4` | — | 选择题号 |
| `run` / `main` | — | 启动赛题 |
| `help` | — | 打印帮助 |
| `showpid` | — | 打印 PID 参数 |
| `pidl,kp,ki,kd,ff` | `pidl,0.2,1.7,0,0.8` | 修改左轮 PID |
| `pidr,kp,ki,kd,ff` | `pidr,0.2,1.7,0,0.8` | 修改右轮 PID |

串口输出格式：`duty=(右,左) meas=(右,左) count=(右,左)`。

## 关键参数位置

- **轮子 PID**: `Sources/chassis_system.c` — 左右轮各自 kp/ki/kd/ff
- **航向 PID**: `Sources/chassis_system.c` — yaw_pid_cfg（kp=0.02, kd=0.0005, out=[-6,6]）
- **线位置 PID**: `Sources/chassis_system.c` — line_pid_cfg（kp=0.067, kd=0.0004, out=[-2.6,2.6]）
- **底盘几何**: `Sources/tasks/main_task.c` — 轮半径 0.0325m, 轮距 0.14m, 编码器 1456 ppr（均为**默认值，未实测标定**）
- **弧线 profile**: `Sources/tasks/main_task.c` — k_arc_profile_bc/cb/da
- **IMU 配置**: `Drivers/Devices/imu_drv.c` — warmup 1200ms, stable_hold 300ms
- **电机方向**: `Sources/chassis_system.c` — `motor_cfg_t.direction_sign` 和 `encoder_cfg_t.direction_sign`（正转前进为正显示）
- **电机/编码器 HAL ID 映射**: `Sources/chassis_system.c` — 左/右软件对象对应物理 HAL ID

## 已知约束与风险

1. **FreeRTOS + M0+ 中断优先级**：Cortex-M0+ 只有 2 位优先级（0-3，数字越大优先级越低）。FreeRTOS CM0 端口要求 PendSV/SysTick 在最低优先级（3）。任何调用 `...FromISR()` 的外设 ISR 优先级不得高于此值。
2. **SRAM 紧张**：32 KB SRAM 运行 5 任务 + MPU6050 DMP + OLED buffer，总 heap 16 KB，需注意栈溢出检查（`configCHECK_FOR_STACK_OVERFLOW = 2`）。
3. **底盘几何未标定**：轮半径、轮距均为理论值。轮距不准会使 `Twist(v,w)` 实际转向量与期望不一致。无线段里程判断也会偏。
4. **IMU 安装未定型**：IMU 未固定会晃动、安装偏角未补偿时 `yaw_deg` 只可参考，不能当强约束。
5. **小车上 SysConfig 外设仅通过 CCS Theia 图形编辑**，不要手动改生成文件（`ti_msp_dl_config.*`、`device_linker.lds`、`device.opt`）。
6. **BT24 BLE 模块 9600 baud**：TX 速度受限，高速刷屏时会丢帧但不会截断数据。连接前建议给 BT24 断电重启。

## 文件组织约定

- **活跃主线代码**在 `Sources/`, `Control/`, `Drivers/Hal/`, `Drivers/Devices/`
- **已脱主线旧模块**在 `archive/legacy/`（旧驱动、旧任务、motion-v2 等），**不参与构建**
- `Drivers/LineTracker/` 仍保留在主目录，因为 `line_sensor` 设备层还在间接复用
- `Drivers/MPU6050/` 和 `Drivers/OLED_Hardware_I2C/` 为独立设备驱动
- `docs/CONTROL_CODE_SUMMARY.md` 是控制代码的详细架构文档，修改控制链时应先参考
- SDK 源码通过 `mspm0g350x_base.cmake` 的 glob 引入，不需手工列出
- `docs/` 是 Obsidian vault，存放所有项目文档、笔记和历史记录

## 修改 SysConfig 后的工作流

1. 在 CCS Theia 中修改 `SysConfig/mspm0-school-2026.syscfg`
2. Save → CCS Theia 自动重新生成代码
3. 回到 VS Code 运行 `syscfg` task（或终端执行上述 sysconfig_cli.sh）
4. 运行 `configure` → `build`
