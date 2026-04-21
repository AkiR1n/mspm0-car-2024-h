# mspm0-school-2026 IMU 重构记录

本文档记录本轮 `MPU6050 / imu_drv / sensor_task / mode_debug_task` 调整的目的、实现和实测结论。

## 1. 背景问题

在底盘新主线切到：

- `sensor_task`
- `control_task`
- `mode_debug_task`

之后，IMU 初期测试暴露了两个明显问题：

1. `MPU6050` 上电或复位后存在较长不稳定阶段。
2. 在关闭 DMP 自动陀螺校准以追求快速启动时，静止状态下 `yaw` 会持续单向漂移。

串口上典型现象是：

- `imu=1/0` 长时间不进入稳定态
- `gz` 长期带固定偏置
- 即使设备静止，`yaw` 仍持续累加

## 2. 本轮目标

本轮调整目标是：

- 缩短 IMU 启动不稳定时间
- 保留 FreeRTOS 友好的非阻塞初始化和刷新方式
- 提供“快速启动”和“可选自动校准”的驱动配置入口
- 让串口调试能明确看到 `ready/stable` 状态
- 在当前阶段先得到一个可用的相对 yaw 通道，便于后续接入控制链

本轮不追求：

- 磁力计绝对航向
- 比赛级姿态融合
- yaw hold 闭环实测上线

## 3. 代码改动

### 3.1 `Drivers/MPU6050/mpu6050.[ch]`

新增了 `mpu6050_config_t`，把初始化显式分成两种模式：

- `MPU6050_CONFIG_DEFAULT`
- `MPU6050_CONFIG_FAST_START`

新增接口：

- `MPU6050_InitWithConfig(const mpu6050_config_t *cfg)`
- `MPU6050_GetGyroSens(float *sens)`
- `MPU6050_SetDmpGyroBiasQ16(const long bias_q16[3])`

说明：

- 当前驱动允许选择是否启用 `DMP_FEATURE_GYRO_CAL`
- 为上层提供了读取陀螺灵敏度和写入 DMP 偏置的能力

### 3.2 `Drivers/Devices/imu_drv.[ch]`

新增 `imu_cfg_t`，当前默认配置为：

- `auto_calibration = 0`
- `warmup_ms = 1200`
- `stable_gyro_threshold = 2.0f`
- `stable_hold_ms = 600`
- `estimate_gyro_bias = 1`
- `apply_dmp_bias = 0`
- `zero_yaw_on_stable = 1`

核心行为调整：

- 启动后先在 `warmup_ms` 窗口内累计三轴陀螺偏置
- 偏置估计完成后，将 `gyro_x/y/z` 统一转换为 `deg/s`
- `stable` 判定基于去偏后的三轴角速度阈值
- `yaw` 不再直接依赖 DMP 四元数的航向角，而是改为基于校正后 `gz` 的软件积分相对角
- 进入稳定态时将相对 yaw 归零

这样做的原因是：

- `pitch/roll` 可以继续使用 DMP 四元数结果
- 对于“无磁力计 + 关闭 DMP 8 秒自动校准”的快速启动模式，DMP `yaw` 本身并不可靠
- 采用校正后 `gz` 的软件积分后，静止漂移可显著降低，并且更适合当前相对航向控制需求

### 3.3 `Sources/tasks/sensor_task.c`

当前设计改为：

- `sensor_task` 周期 `10 ms`
- 失败时每 `1 s` 重试一次 `Imu_Init()`
- 初始化成功后只做非阻塞 `Imu_Refresh()`

这保证了：

- IMU 不会因为初始化失败阻塞系统
- 编码器、循迹和底盘主链不会被 IMU 拉死

### 3.4 `Sources/tasks/mode_debug_task.c`

串口调试输出改为：

```text
mode=STOP imu=1/1 ypr=(...) gz=... up=... stable=...
```

其中：

- `imu=ready/stable`
- `up` 表示 IMU 启动后累计运行时间
- `stable` 表示首次进入稳定态的时间点

这便于直接从串口判断：

- 初始化是否完成
- 稳定态是否已经建立
- 当前 `gz` 是否仍然存在显著静态偏置

## 4. 实测结论

本轮修改后的实测结果如下：

### 4.1 启动时间

当前在复位后的典型表现是：

- `imu=1/1` 大约在 `1.5 s ~ 2 s` 内出现

相比此前依赖 DMP 自动校准的长等待阶段，启动时间明显缩短。

### 4.2 静止表现

在最新串口日志中，稳定后的典型值为：

- `gz` 基本在 `-0.1 ~ +0.1 deg/s`
- `yaw` 基本稳定在 `0.00` 附近，小幅波动约 `0.01 ~ 0.02`

这说明：

- 静态零偏已经基本压住
- 之前“静止时 yaw 还会持续单向累加”的问题已经被消除

### 4.3 当前仍可见的现象

日志中的 `ypr` 后两项仍存在固定偏角，例如：

- `pitch` 约 `-1.x`
- `roll` 约 `168.x`

这更像是：

- 安装姿态
- 轴定义
- 显示零点

带来的固定偏置，而不是新的时间漂移问题。

## 5. 当前结论

可以认为当前 IMU 方案已经满足本阶段需要：

- 启动更快
- 非阻塞，FreeRTOS 友好
- 串口状态可直接观测
- 静止相对 yaw 可用
- 可以继续向后接入 yaw 相关控制模块

## 6. 后续建议

建议下一阶段继续做以下整理：

1. 增加显式 IMU 模式枚举或配置入口，例如：
   - `FAST_START`
   - `AUTO_CAL`
   - `FAST_START_WITH_RUNTIME_BIAS`
2. 对 `pitch/roll` 增加显示零点或安装姿态标定逻辑。
3. 在进入控制链前，明确 `yaw` 的单位和语义：
   - 当前为相对角，单位 `deg`
4. 如后续确需长时间绝对航向稳定，需评估：
   - 磁力计接入
   - 更完整姿态融合方案

## 7. 本轮相关文件

- [Drivers/MPU6050/mpu6050.h](Drivers/MPU6050/mpu6050.h)
- [Drivers/MPU6050/mpu6050.c](Drivers/MPU6050/mpu6050.c)
- [Drivers/Devices/imu_drv.h](Drivers/Devices/imu_drv.h)
- [Drivers/Devices/imu_drv.c](Drivers/Devices/imu_drv.c)
- [Sources/tasks/sensor_task.c](Sources/tasks/sensor_task.c)
- [Sources/tasks/mode_debug_task.c](Sources/tasks/mode_debug_task.c)
- [Sources/app_state.h](Sources/app_state.h)
