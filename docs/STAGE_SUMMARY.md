# mspm0-school-2026 阶段性汇总

> 本文档为较早阶段的历史汇总，记录的是 `motion-v2 + 四任务主线` 时期的状态。
> 当前工程主线已切换到 `sensor/control/mode_debug` 三任务，且已脱主线的旧驱动、旧任务已移动到 `archive/legacy/`。
> 现阶段请优先参考 [README](../README.md) 和 [[REFACTOR_ARCHIVE_20260422]]。

## 1. 当前定位

`mspm0-school-2026` 已作为 2026 校赛主工程落地，目标是固定一条可持续迭代的主线：

- 芯片：`MSPM0G3507`
- 系统：`FreeRTOS`
- 运动控制：`motion-v2`
- 传感器：`MPU6050`
- 显示：`OLED`
- 日志：`UART0`
- 主环境：`VS Code`
- `.syscfg` 图形编辑环境：`CCS Theia`

当前工程已经从 `mspm0-car-2026` 收敛为独立目录，不再依赖兄弟工程源码或兄弟目录中的 `motion-v2` 实现。

## 2. 目录与构建收敛

### 已完成

- 新工程目录固定为 `mspm0-school-2026`
- `motion-v2` 已并入：
  - `Drivers/MotionV2/include/`
  - `Drivers/MotionV2/src/`
- 默认构建已切换到新主线，不再把以下模块编进首版默认运行路径：
  - `Drivers/Motor_Encoder_PID`
  - `Drivers/LineTracker`
  - `Drivers/K230_UART`
  - `Drivers/Gimbal`
  - `Drivers/Servo`
- 构建目标名已统一为：
  - `mspm0_school_2026.elf`
  - `mspm0_school_2026.hex`
  - `mspm0_school_2026.bin`

### 当前 CMake 行为

默认构建显式包含：

- `Sources/main.c`
- `Sources/app_state.c`
- `Sources/sysmem.c`
- `Sources/uart_printf.c`
- `Sources/tasks/motion_task.c`
- `Sources/tasks/imu_task.c`
- `Sources/tasks/oled_task.c`
- `Sources/tasks/log_task.c`
- `Drivers/MSPM0/*.c`
- `Drivers/MotionV2/src/*.c`
- `Drivers/MPU6050/*.c`
- `Drivers/OLED_Hardware_I2C/oled_hardware_i2c.c`

这意味着旧模块虽然仍保留在目录中作为归档，但已经不进入默认构建。

## 3. 运行时架构

当前主路径只保留 4 个任务：

### `motion_task`

- 周期：`10 ms`
- 负责初始化：
  - `motor_init(NULL)`
  - `encoder_init(NULL)`
  - `linetracker_init(NULL)`
  - `turn_detection_init(NULL)`
  - `motion_init(NULL)`
- 负责周期行为：
  - 读取循迹状态
  - 注入最新 yaw：`motion_set_yaw_feedback()`
  - 推进 `turn_detection_update(now_ms)`
  - 根据状态选择 `LINE_FOLLOW / YAW_HOLD / STOP`
  - 执行 `motion_step(dt)`
  - 发布 line bits / line position / wheel pps / mode / base speed

### `imu_task`

- 周期：`10 ms`
- 负责 `MPU6050_Init()` 的重试式初始化
- 初始化失败时：
  - 不阻塞整机启动
  - 每秒通过串口打印 retry 日志
- 初始化成功后：
  - 周期调用 `Read_Quad()`
  - 发布 `yaw / pitch / roll`
  - 更新 `imu_ready`

### `oled_task`

- 周期：`100 ms`
- 固定显示：
  - IMU ready 状态
  - yaw
  - line bits / position
  - mode / base speed

### `log_task`

- 周期：`1 s`
- 固定输出一行结构化串口日志，内容包括：
  - heap
  - imu ready
  - yaw / pitch / roll
  - line bits / position
  - left/right pps
  - motion mode
  - base speed
  - last turn

### 已移出主路径的任务

- `pid_task`
- `sensor_task`
- `strategy_task`
- `k230_rx_task`

这些文件仍保留在目录中，但不再参与默认构建，也不由 `main()` 创建任务。

## 4. 共享状态设计

共享状态已收敛到统一结构 `g_app_state`，定义于：

- `Sources/app_state.h`
- `Sources/app_state.c`

当前字段包括：

- `imu_ready`
- `yaw_deg`
- `pitch_deg`
- `roll_deg`
- `line_position`
- `line_bits`
- `left_pps`
- `right_pps`
- `mode`
- `base_speed_pps`
- `last_turn`

这一步的意义是把原先散落在各驱动和任务中的 `extern` 状态集中到单一应用层出口。

## 5. motion-v2 接入状态

当前新工程已经完成“完全切换”式接入，主路径统一改走：

- 电机：`motor_*`
- 编码器：`encoder_*`
- 循迹：`linetracker_*`
- 转弯检测：`turn_detection_*`
- 高级控制：`motion_*`

### 当前中断绑定

- `GROUP1_IRQHandler -> encoder_on_gpio_irq()`
- `TIMA1_IRQHandler -> encoder_on_tick_irq()`

### 当前 turn detection 方式

- 不再依赖独立 tracker 定时器 ISR
- 由 `motion_task` 内部周期调用 `turn_detection_update(now_ms)`

### 兼容性现状

当前运行主路径已经不再调用旧接口：

- `MotorControl_*`
- `LineTracker_*`
- `TurnDetection_*`

旧模块仍保留在仓内，仅作为归档对照，不作为现行主线路径。

## 6. SysConfig 收敛

当前 `.syscfg` 文件：

- `SysConfig/mspm0-school-2026.syscfg`

首版保留外设：

- `PWM_MOTOR`
- `TIMER_CALC`
- `GPIO_ENCODER`
- `I2C_MPU6050`
- `I2C_OLED`
- `UART0`
- 循迹 GPIO
- LED / Key GPIO

首版已移除或禁用：

- `TIMER_TRACKER`
- `PWM_SERVO`
- `K230_UART`
- 与 `K230 / Gimbal / Servo` 主路径相关的外设项

已验证重新执行 `syscfg` 后，生成代码中不再出现 `TIMER_TRACKER`。

## 7. VS Code 工作流

工程已补齐以下配置：

- `.vscode/tasks.json`
- `.vscode/extensions.json`
- `.vscode/launch.json`
- `.clangd`
- `.vscode/jlink-flash.jlink`

### 已配置任务

- `syscfg`
- `configure`
- `build`
- `rebuild`
- `hex`
- `bin`
- `flash-elf`
- `flash-hex`
- `flash-jlink`
- `serial`

### 兼容性修正

由于用户实际终端为 `fish`，而 `LQFP-64(PM)` 中的括号会被 `fish` 解析，任务系统已从 `shell` 切换为 `process`，避免参数在 VS Code 中被 shell 二次解析。

## 8. J-Link 工作流

当前已补齐 VS Code 下的 J-Link 调试 / 烧录入口。

### 调试配置

`launch.json` 中已增加：

- `J-Link Debug`
- `J-Link Attach`

### 默认参数

- device: `MSPM0G3507`
- interface: `SWD`
- speed: `4000`
- gdb: `/usr/bin/arm-none-eabi-gdb`
- server: `/usr/bin/JLinkGDBServer`
- executable: `build/mspm0_school_2026.elf`

### 烧录配置

`flash-jlink` 任务通过：

- `/usr/bin/JLinkExe`
- `.vscode/jlink-flash.jlink`

完成标准烧录流程：

- 连接 SWD
- 选择设备
- reset
- halt
- `loadfile build/mspm0_school_2026.elf`
- reset
- go

### 本机确认情况

已确认本机存在：

- `/usr/bin/JLinkGDBServer`
- `/usr/bin/JLinkExe`

并且本机 SEGGER 工具链对 `MSPM0G3507 / MSPM0` 系列具备支持。

说明：

- 目前确认的是“工具链和 VS Code 配置路径正确”
- 未在本阶段完成实际连板 J-Link 调试会话验证

## 9. 本阶段已完成验证

以下步骤已经在本地完成：

### SysConfig

已成功执行：

```sh
bash /opt/ccstudio/ccs/utils/sysconfig_1.26.0/sysconfig_cli.sh \
  --product /home/aki/ti/mspm0_sdk_2_10_00_04/.metadata/product.json \
  --device MSPM0G3507 \
  --package "LQFP-64(PM)" \
  --compiler gcc \
  --script SysConfig/mspm0-school-2026.syscfg \
  --output SysConfig
```

### CMake 配置

已成功执行：

```sh
cmake -S . -B build -GNinja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

### 编译

已成功执行：

```sh
cmake --build build
```

### 产物导出

已成功执行：

```sh
cmake --build build --target hex bin
```

### 当前产物

已确认存在：

- `build/mspm0_school_2026.elf`
- `build/mspm0_school_2026.hex`
- `build/mspm0_school_2026.bin`
- `build/compile_commands.json`
- `build/memory.map`

## 10. 当前未完成项

以下内容尚未在本阶段闭环：

### 板级联调

- 未实际执行 `flash-jlink` 上板烧录验证
- 未确认 `J-Link Debug` 可以稳定连到目标板
- 未实际验证 `UART0` 串口日志输出
- 未实际验证 OLED 显示
- 未实际验证 MPU6050 接入成功后的姿态输出
- 未实际验证编码器速度响应
- 未实际验证循迹 bits / position 的硬件响应
- 未实际验证底盘基本直行与循迹效果

### 目录清理

当前目录状态已经更新为：

- 已脱主线旧模块已移动到 `archive/legacy/`
- `Drivers/LineTracker` 仍保留在主目录，因为当前新的 `line_sensor` 设备层还在直接复用它
- 当前主线任务文件保留在 `Sources/tasks/`：
  - `sensor_task.*`
  - `control_task.*`
  - `mode_debug_task.*`

### 调试增强

- 还没有为 `cortex-debug` 补寄存器级 SVD
- 还没有补 RTT 日志路径
- 还没有补更细的 reset / attach 策略
- 还没有验证 J-Link 低速 SWD 下的稳定性

## 11. 建议的下一阶段顺序

建议下一阶段按以下顺序推进：

1. 实板运行 `flash-jlink`
2. 实测 `J-Link Debug`
3. 实测串口日志
4. 实测 OLED
5. 实测 MPU6050 初始化成功与失败降级
6. 实测编码器读数
7. 实测循迹 bits / position
8. 实测底盘 line follow
9. 记录一版默认 PID / base speed 参数
10. 再决定是否清理旧目录和是否引入更复杂策略层

## 12. 一句话状态

当前工程已经完成“从 `mspm0-car-2026` 迁移到 `mspm0-school-2026` 并切换到 `motion-v2 + 四任务主线 + VS Code/J-Link 工作流`”这一阶段目标；软件侧构建链路已打通，下一阶段重点应转到实板验证与参数整定。
