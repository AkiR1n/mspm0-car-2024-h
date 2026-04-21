# mspm0-school-2026

2026 校赛主工程，固定为 `MSPM0G3507 + FreeRTOS + motion-v2 + MPU6050 + OLED + UART0`。

这个目录直接从 `mspm0-car-2026` 收敛而来，但已经变成独立工程：

- `motion-v2` 已并入 `Drivers/MotionV2`
- 默认运行主线只保留 `motion / imu / oled / log` 四个任务
- `K230 / Gimbal / Servo` 仅作为归档源码保留，不参与首版默认构建和运行
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
| `motion_task` | 6 | 512 | 10 ms | 初始化 motor/encoder/linetracker/motion；读取循迹、注入 yaw、推进 `motion_step()` |
| `imu_task` | 5 | 512 | 10 ms | `MPU6050_Init()` 重试初始化；成功后轮询 `Read_Quad()` |
| `oled_task` | 2 | 384 | 100 ms | 显示 IMU / yaw / line / mode |
| `log_task` | 1 | 256 | 1 s | 输出 heap / imu / ypr / line / pps / mode |

共享状态统一集中在 `Sources/app_state.[ch]`。

## motion-v2 收敛点

- 电机：`motor_*`
- 编码器：`encoder_*`
- 循迹：`linetracker_*`
- 转弯检测：`turn_detection_*`
- 高级控制：`motion_*`
- yaw 反馈：`motion_set_yaw_feedback()`
- 编码器 GPIO 中断：`GROUP1_IRQHandler -> encoder_on_gpio_irq()`
- 编码器计时中断：`TIMA1_IRQHandler -> encoder_on_tick_irq()`
- `turn_detection_update(now_ms)` 在 `motion_task` 周期调用，不再依赖独立 tracker ISR

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

推荐日常顺序：

1. 在 CCS Theia 修改 `SysConfig/mspm0-school-2026.syscfg`
2. 在 VS Code 运行 `syscfg`
3. 运行 `configure` / `build`
4. 运行 `hex` 或 `flash-elf`
5. 运行 `serial`
