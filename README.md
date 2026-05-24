# mspm0-school-2026

2026 校赛主工程，当前主线固定为 `MSPM0G3507 + FreeRTOS + chassis refactor + MPU6050 + UART0`。

这个目录直接从 `mspm0-car-2026` 收敛而来，但已经变成独立工程：

- 默认运行主线已切到 `sensor / control / mode_debug` 三任务
- 当前主链走 `Drivers/Hal + Drivers/Devices + Control + Sources/tasks`
- 已脱主线的旧驱动与旧任务已移动到 `archive/legacy/`
- `Drivers/LineTracker` 暂时仍保留在主目录，因为新的 `line_sensor` 设备层还在直接复用它
- CCS Theia 只负责 `.syscfg` 图形编辑，日常工作流转到 VS Code

## 构建环境

| 组件 | 版本 / 路径 |
| ---- | ---- |
| arm-none-eabi-gcc | 14.2.0（`/usr/bin`） |
| CMake | ≥ 3.30（Ninja 生成器） |
| MSPM0 SDK | `~/ti/mspm0_sdk_2_10_00_04/` |
| SysConfig CLI | `/opt/ccstudio/ccs/utils/sysconfig_1.26.0/sysconfig_cli.sh` |
| DSLite | `/opt/ccstudio/ccs/ccs_base/DebugServer/bin/DSLite` |

## 构建 / 产物

```sh
# 1. 改动 SysConfig 后重新生成 ti_msp_dl_config.* + linker
bash /opt/ccstudio/ccs/utils/sysconfig_1.26.0/sysconfig_cli.sh \
    --product ~/ti/mspm0_sdk_2_10_00_04/.metadata/product.json \
    --device MSPM0G3507 --package "LQFP-64(PM)" --compiler gcc \
    --script SysConfig/mspm0-school-2026.syscfg \
    --output SysConfig

# 2. 配置 + 编译
cmake -S . -B build -GNinja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build build

# 3. 导出 hex / bin
cmake --build build --target hex
cmake --build build --target bin
```

产物：

- `build/mspm0_school_2026.elf`
- `build/mspm0_school_2026.hex`
- `build/mspm0_school_2026.bin`
- `build/compile_commands.json`
- `build/memory.map`

## 任务架构

| 任务 | 优先级 | 栈 words | 周期 | 说明 |
| ---- | ---- | ---- | ---- | ---- |
| `sensor_task` | 6 | 512 | 10 ms | 刷新编码器 / IMU / 循迹设备，组装 `chassis_feedback_t` |
| `control_task` | 5 | 512 | 10 ms | 执行 `Twist(v,w)` 或 `WHEEL_TEST` 控制，输出电机命令 |
| `mode_debug_task` | 2 | 512 | 20 ms | 处理串口命令，维护模式并输出调试 CSV |

共享状态统一集中在 `Sources/app_state.[ch]`。

## 当前控制主线

- 设备层：`motor_drv` / `encoder_drv` / `imu_drv` / `line_sensor`
- 控制层：`pid` / `wheel` / `chassis` / `yaw_controller` / `line_controller`
- 编码器 GPIO 中断：`GROUP1_IRQHandler -> Encoder_OnEdgeIRQ()`
- 编码器计时中断：`TIMA1_IRQHandler -> Encoder_OnSampleTick()`
- 当前调试模式以 `WHEEL_TEST` 和 `Twist(v,w)` 为主

## SysConfig 范围

首版 `.syscfg` 只保留：

- `PWM_MOTOR`
- `TIMER_CALC`
- `GPIO_ENCODER`
- `I2C_MPU6050`
- `I2C_OLED`
- `UART0`
- 循迹相关 GPIO
- LED / Key GPIO

## VS Code

已补齐：

- `.vscode/tasks.json`
- `.vscode/extensions.json`
- `.vscode/launch.json`
- `.clangd`
- `.vscode/jlink-flash.jlink`

当前 VS Code 工作流默认按 **Linux 远端执行** 维护：

- 推荐方式：在 macOS 上用 VS Code + Remote SSH 打开这台 Linux 机器上的仓库
- `tasks.json` 里的任务默认都在 Linux 远端执行
- 不再把 macOS 本地作为编译 / 烧录 / 调试 / 串口的主支持环境
- 所有任务统一收口到 `scripts/dev/*.sh`

第一次接手环境时，先运行：

```sh
Run Task -> doctor
```

它会检查：

- `cmake` / `ninja` / `arm-none-eabi-gcc` / `arm-none-eabi-gdb`
- SysConfig CLI 与 MSPM0 SDK 路径
- `pyocd` / `JLinkExe` / `picocom` 等可选工具
- 常见串口设备是否在远端 Linux 上可见

串口相关任务支持 `auto` 选项，会优先尝试：

- `/dev/ttyACM0`
- `/dev/ttyACM1`
- `/dev/ttyUSB0`
- `/dev/ttyUSB1`

也可以通过环境变量覆盖：

```sh
export MSPM0_SERIAL_PORT=/dev/ttyACM1
```

推荐日常顺序：

1. 在 CCS Theia 修改 `SysConfig/mspm0-school-2026.syscfg`
2. 在 VS Code 运行 `doctor`
3. 运行 `syscfg`
4. 运行 `configure` / `build`
5. 运行 `hex` 或 `flash-elf`
6. 运行 `serial`

## J-Link

已补充一套 VS Code J-Link 工作流：

- 调试：`Run and Debug -> J-Link Debug`
- 附加：`Run and Debug -> J-Link Attach`
- 烧录：`Task -> flash-jlink`

默认参数：

- device: `MSPM0G3507`
- interface: `SWD`
- speed: `4000 kHz`
- executable: `build/mspm0_school_2026.elf`

## Remote SSH 调试

`CMSIS-DAP Debug (pyOCD)` 现在会先执行 `debug-prepare`：

1. `build`
2. 启动或复用 Linux 远端上的 `pyOCD gdbserver`

`pyOCD gdbserver` 的状态文件和日志放在：

- `build/.dev/pyocd-gdbserver.pid`
- `build/.dev/pyocd-gdbserver.log`

如果需要手动关闭，运行：

```sh
Run Task -> debug-server-kill
```
