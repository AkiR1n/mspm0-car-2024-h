# mspm0-school-2026 测试流程

本文档用于验证当前重构后的底盘主线是否满足本阶段目标：

- 双轮速度闭环可运行
- `Twist(v, w)` 控制链打通
- 新任务模型 `sensor_task + control_task + mode_debug_task` 正常工作
- IMU 缺失或初始化失败时，底盘速度闭环主链不崩溃

## 1. 测试目标

本阶段重点不是比赛策略，而是底盘控制基础链路。

本次测试要覆盖：

- 构建与产物生成
- 新主线入口是否正确
- 编码器中断与测速是否正常
- 串口与 OLED 调试输出是否正常
- `Twist(v, 0)` 直行链路
- `Twist(0, w)` 转向链路
- `Twist(v, w)` 差速链路
- IMU 异常时系统是否仍可运行

本阶段不要求：

- 实际循迹闭环跑通
- yaw hold 实际闭环上板效果
- 比赛级状态机验证

## 2. 代码位置

当前主线相关文件：

- 入口：[Sources/main.c](Sources/main.c)
- 状态共享：[Sources/app_state.h](Sources/app_state.h)
- 底盘系统装配：[Sources/chassis_system.c](Sources/chassis_system.c)
- 传感器任务：[Sources/tasks/sensor_task.c](Sources/tasks/sensor_task.c)
- 控制任务：[Sources/tasks/control_task.c](Sources/tasks/control_task.c)
- 模式与调试任务：[Sources/tasks/mode_debug_task.c](Sources/tasks/mode_debug_task.c)
- 编码器中断绑定：[Drivers/MSPM0/interrupt.c](Drivers/MSPM0/interrupt.c)

当前默认测试命令在：

- [Sources/tasks/mode_debug_task.c](Sources/tasks/mode_debug_task.c)

## 3. 构建测试

在工作区根目录执行：

```sh
cmake -S mspm0-school-2026 -B mspm0-school-2026/build -GNinja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build mspm0-school-2026/build
cmake --build mspm0-school-2026/build --target hex bin
```

预期结果：

- 配置成功
- 编译成功
- 成功生成以下产物：
  - `mspm0-school-2026/build/mspm0_school_2026.elf`
  - `mspm0-school-2026/build/mspm0_school_2026.hex`
  - `mspm0-school-2026/build/mspm0_school_2026.bin`

若失败，先检查：

- `SysConfig` 生成文件是否完整
- 工具链是否可用
- `CMakeLists.txt` 是否仍指向新主线模块

## 4. 主线入口检查

确认 `main()` 只创建以下任务：

- `sensor_task`
- `control_task`
- `mode_debug_task`

当前设计要求：

- 不再由 `main()` 创建旧 `motion_task`
- 不再由 `main()` 创建旧 `imu_task`
- 不再由 `main()` 创建旧 `oled_task`
- 不再由 `main()` 创建旧 `log_task`

建议检查：

```sh
rg -n "motion_task|imu_task|oled_task|log_task" mspm0-school-2026/Sources/main.c mspm0-school-2026/CMakeLists.txt
```

预期结果：

- `main.c` 和 `CMakeLists.txt` 中不再包含旧主线路径

## 5. 默认安全启动测试

当前默认模式建议先保持：

```c
#define MODE_DEBUG_DEFAULT_MODE     APP_MODE_STOP
#define MODE_DEBUG_DEFAULT_V_MPS    0.00f
#define MODE_DEBUG_DEFAULT_W_RADPS  0.00f
```

位置：

- [Sources/tasks/mode_debug_task.c](Sources/tasks/mode_debug_task.c)

在此配置下先烧录运行，确认系统静态正常。

预期结果：

- 板子能正常启动
- 串口持续有输出
- OLED 正常刷新
- 电机不转
- 日志中 `stop=1`
- 日志中 `target=(0,0)`
- 日志中 `duty=(0,0)`

如果这一步不稳定，不要进入动态转轮测试。

## 6. 串口日志检查

当前 `mode_debug_task` 会周期打印一行调试日志。

重点关注字段：

- `mode`
- `stop`
- `vw`
- `target`
- `meas`
- `duty`
- `line`
- `imu`
- `yaw`

建议检查：

- `mode` 是否符合当前模式
- `vw` 是否等于设定的命令值
- `target` 是否随 `Twist(v, w)` 变化
- `meas` 是否能反映真实测速
- `duty` 是否有合理输出
- `imu` 是否能反映 IMU 初始化状态

## 7. OLED 检查

OLED 当前应显示基础运行信息。

预期至少能看到：

- 当前模式
- `v/w`
- 左右轮实测速度
- IMU 状态和 yaw

若 OLED 不工作，但串口正常，优先排查：

- OLED 硬件连接
- `I2C_OLED`
- OLED 驱动初始化

## 8. 编码器链路测试

保持默认 `STOP` 模式不动，用手转轮子。

测试方法：

1. 烧录默认 `STOP` 配置
2. 打开串口日志
3. 手动转动左轮
4. 手动转动右轮
5. 分别测试正反方向

预期结果：

- 转左轮时，左轮 `meas` 变化，右轮基本不变
- 转右轮时，右轮 `meas` 变化，左轮基本不变
- 正反方向切换时，速度符号会改变
- 停止转动后，速度回到接近 0

如果测速没有变化，优先检查：

- 编码器接线
- `GROUP1_IRQHandler`
- `TIMA1_IRQHandler`
- `TIMER_CALC` 是否在运行
- 编码器方向逻辑是否正确

## 9. IMU 容错测试

本阶段要求 IMU 失败时，底盘主链仍能工作。

测试方法：

1. 断开 IMU 或模拟 IMU 初始化失败
2. 烧录程序
3. 观察串口输出
4. 继续做手转轮测试

预期结果：

- 串口出现类似 `sensor: imu init failed, retry=...`
- 系统不崩溃
- `sensor_task` 持续运行
- 编码器测速仍正常
- OLED 和串口仍持续刷新
- `control_task` 仍可执行速度闭环

不允许出现：

- 因 IMU 初始化失败导致整个系统卡死
- 编码器和控制链完全停摆

## 10. 直行测试 `Twist(v, 0)`

先做最低风险测试，建议先离地。

修改 [Sources/tasks/mode_debug_task.c](Sources/tasks/mode_debug_task.c)：

```c
#define MODE_DEBUG_DEFAULT_MODE     APP_MODE_TWIST_OPEN
#define MODE_DEBUG_DEFAULT_V_MPS    0.05f
#define MODE_DEBUG_DEFAULT_W_RADPS  0.00f
```

重新编译并烧录。

测试步骤：

1. 将底盘悬空
2. 上电运行
3. 观察串口和轮子转动

预期结果：

- `stop=0`
- `vw=(0.05, 0.00)`
- 左右轮 `target` 基本相同
- 左右轮 `duty` 有输出
- 两轮同方向转动
- 左右轮 `meas` 应接近目标

如果出现一边转一边不转，优先排查：

- 电机驱动接线
- 单轮编码器是否失效
- 电机方向定义是否和机械安装相反

## 11. 原地转向测试 `Twist(0, w)`

修改默认命令：

```c
#define MODE_DEBUG_DEFAULT_MODE     APP_MODE_TWIST_OPEN
#define MODE_DEBUG_DEFAULT_V_MPS    0.00f
#define MODE_DEBUG_DEFAULT_W_RADPS  1.50f
```

重新编译烧录。

测试步骤：

1. 底盘悬空
2. 上电运行
3. 观察左右轮目标速度和实际转向

预期结果：

- 左右轮 `target` 一正一负
- 左右轮方向相反
- 串口 `vw=(0.00, 1.50)`
- 底盘表现为原地或近似原地转向

如果左右轮同向，优先检查：

- `Chassis_SetTwist()` 的差速解算
- 左右轮方向定义
- 电机方向实际接线

## 12. 混合差速测试 `Twist(v, w)`

修改默认命令：

```c
#define MODE_DEBUG_DEFAULT_MODE     APP_MODE_TWIST_OPEN
#define MODE_DEBUG_DEFAULT_V_MPS    0.08f
#define MODE_DEBUG_DEFAULT_W_RADPS  1.00f
```

重新编译烧录。

测试步骤：

1. 先离地验证
2. 再落地低速验证

预期结果：

- 左右轮 `target` 不相等
- 一侧更快，另一侧更慢
- 左右轮 `meas` 随目标变化
- 底盘走弧线

这是 `Twist(v, w)` 差速链路是否真正打通的关键验证。

## 13. 落地低速测试

在离地验证通过后，再落地测试。

建议顺序：

1. `Twist(0.05, 0.00)`
2. `Twist(0.00, 1.00~1.50)`
3. `Twist(0.08, 1.00)`

预期结果：

- 低速直行时能稳定起步
- 低速转向时能明显转向
- 差速命令下能稳定走弧线

注意事项：

- 初期不要给过大的 `v_mps`
- 初期不要在未知地面摩擦条件下高速测试
- 若出现明显抖动，先降低速度，再回头调 PID

## 14. PID 观察重点

串口中重点观察：

- `target`
- `meas`
- `duty`

判断方法：

- `target` 正常但 `meas` 跟不上：可能动力不足、摩擦大、PID 太弱
- `duty` 长期打满：可能目标过高或 PID 参数不合适
- `meas` 大幅振荡：可能 PID 过激
- 目标归零后轮子不能稳定停下：积分或最小占空比策略需调整

## 15. 常见异常与排查

### 15.1 烧录后无串口输出

检查：

- `UART0` 配置
- 板卡串口连接
- 程序是否进入调度器
- `main()` 是否成功创建任务

### 15.2 轮子不转

检查：

- 当前是否仍为 `APP_MODE_STOP`
- `stop` 是否为 1
- `target` 是否为 0
- 电机供电是否正常
- PWM 与方向脚是否正确

### 15.3 编码器速度一直为 0

检查：

- 编码器引脚中断是否进入
- `TIMA1_IRQHandler` 是否触发
- 编码器接线是否正确
- 编码器参数 `pulses_per_revolution` 是否配置错误

### 15.4 IMU 初始化一直失败

检查：

- I2C 接线
- MPU6050 供电
- IMU 地址与初始化时序

注意：

- IMU 失败本阶段不应阻塞底盘速度闭环测试

## 16. 推荐执行顺序

建议严格按以下顺序执行：

1. 构建通过
2. 烧录默认 `STOP` 配置
3. 检查串口输出
4. 检查 OLED
5. 手转轮子验证编码器测速
6. 验证 IMU 失败不崩
7. 离地测试 `Twist(v, 0)`
8. 离地测试 `Twist(0, w)`
9. 离地测试 `Twist(v, w)`
10. 落地低速测试

不要跳过“静态启动”和“手转轮”两步。

## 17. 本阶段验收结论标准

满足以下条件即可认为本阶段主目标达成：

- 工程可构建并生成 `elf/hex/bin`
- 主线只跑新任务模型
- 串口可稳定输出目标速度、测量速度、duty、`v/w`
- 手转轮子时测速正确变化
- `Twist(v, 0)` 时底盘具备基础直行能力
- `Twist(0, w)` 时底盘具备基础转向能力
- IMU 初始化失败不导致主链崩溃

## 18. 后续建议

本文件对应的是当前静态预设测试模式。

后续建议新增：

- 按键切换 `STOP / TWIST`
- 多组速度预设
- 更适合调 PID 的高频日志模式
- 单轮测试模式
- 串口命令下发 `v/w`

