# 重构阶段归档（2026-04-22）

本文档用于归档 `mspm0-school-2026` 相对于“底盘驱动与控制链重建”计划的当前进度。

归档基线：

- 三任务新主线已经替换旧 `motion_task` 主路径
- 电机驱动、编码器驱动、MPU6050 驱动已可在板上工作
- 轮子/编码器开环扫速测试已完成，结果见：
  [WHEEL_SPEED_SWEEP_RESULTS.md](/home/aki/workspace_ccstheia/mspm0-school-2026/WHEEL_SPEED_SWEEP_RESULTS.md:1)
- 已脱主线的旧驱动与旧任务现已移动到：
  [archive/legacy](/home/aki/workspace_ccstheia/mspm0-school-2026/archive/legacy:1)

## 1. 当前结论

如果按最初阶段目标来衡量，当前状态可以概括为：

- 新架构主线已经建立并实际跑起来
- `HAL/BSP -> Device -> Control -> RTOS/App` 分层已经成形
- 电机、编码器、IMU、循迹设备都已经纳入新架构
- `sensor_task + control_task + mode_debug_task` 已接管主程序
- 双轮速度闭环代码已经具备，但当前板上主要完成的是开环轮子/编码器验证
- `Twist(v,w)` 控制链在代码中已接通，但现阶段调试主入口切到了 `WHEEL_TEST`
- IMU 已可用，但还没有进入最终可交付的底盘稳向主链

换句话说：

- “底盘新架构搭起来了”
- “底层三件套基本打通了”
- “下一步重点已经从结构迁移到控制整定和主模式收口”

## 2. 对照原计划的完成度

### 2.1 分层重组

已完成：

- `Drivers/Hal/`
  - 已存在，用于硬件读写薄层
- `Drivers/Devices/`
  - 已存在并已接入主线：
    - `motor_drv`
    - `encoder_drv`
    - `imu_drv`
    - `line_sensor`
- `Control/`
  - 已存在并已接入主线：
    - `pid`
    - `wheel`
    - `chassis`
    - `yaw_controller`
    - `line_controller`
- `Sources/tasks/`
  - 已切换为：
    - `sensor_task`
    - `control_task`
    - `mode_debug_task`

当前判断：

- 这一项已经达到“主线替换完成”的程度
- 目录组织与原计划基本一致

### 2.2 设备接口边界

已完成：

- `motor_drv`
  - 已有 `Motor_Init`
  - 已有 `Motor_SetDuty`
  - 已有 `Motor_Stop`
- `encoder_drv`
  - 已有 `Encoder_Init`
  - 已有 ISR 驱动采样链
  - 已有 `Encoder_GetSpeedMps`
  - 已有 `Encoder_GetSpeedRps`
  - 已有 `Encoder_GetCount`
- `imu_drv`
  - 已提供：
    - `yaw_deg`
    - `gyro_z`
    - `pitch_deg`
    - `roll_deg`
    - `accel_*`
    - `gyro_*`
    - `ready/stable/uptime`
- `line_sensor`
  - 已提供：
    - `bits`
    - `detected`

当前判断：

- 设备层接口目标基本完成
- IMU 这部分不只是“能读到值”，还已经补了稳定状态和基础辅助字段

### 2.3 控制层接口

已完成：

- `pid`
  - 纯算法模块已存在
- `wheel`
  - 已实现：
    - `Wheel_SetTargetSpeed`
    - `Wheel_UpdateFeedback`
    - `Wheel_ControlStep`
    - `Wheel_Stop`
- `chassis`
  - 已实现：
    - `Chassis_SetTwist`
    - `Chassis_SetWheelSpeed`
    - `Chassis_ControlStep`
    - `Chassis_Stop`
  - 差速解算已按 `Twist(v,w)` 形式实现
- `yaw_controller`
  - 模块已建立
- `line_controller`
  - 模块已建立

当前判断：

- 代码结构上已经达到计划要求
- 但 `yaw_controller` 和 `line_controller` 还没有进入当前主验收运行链

### 2.4 统一控制数据流

已完成：

- `chassis_feedback_t`
  - 已包含：
    - `dt_s`
    - 左右轮速度
    - 左右计数
    - `imu_ready`
    - `imu_stable`
    - `yaw_deg`
    - `gyro_z`
    - `pitch_deg`
    - `roll_deg`
    - `accel_*`
    - `gyro_*`
    - `imu_uptime_ms`
    - `imu_stable_ms`
    - `line_bits`
    - `line_detected`
- `chassis_command_t`
  - 已包含：
    - `stop`
    - `enable_closed_loop`
    - `v_mps`
    - `w_radps`
  - 并额外加入：
    - `left_duty`
    - `right_duty`
  - 这两个字段主要服务当前 `WHEEL_TEST`
- `chassis_debug_t`
  - 已包含：
    - 左右目标速度
    - 左右测量速度
    - 左右电机 duty

当前判断：

- 这一项已经完成
- 并且为了调试轮子/编码器，还做了比原计划更实用的扩展

### 2.5 RTOS 层重构

已完成：

- `main()` 当前只创建 3 个任务：
  - `sensor_task`
  - `control_task`
  - `mode_debug_task`
- `sensor_task`
  - 负责：
    - 编码器反馈读取
    - IMU 初始化/刷新
    - 循迹传感器刷新
    - 组装 `chassis_feedback_t`
- `control_task`
  - 负责：
    - 读取反馈与命令
    - `Twist(v,w)` 底盘控制
    - 闭环控制步进
    - `WHEEL_TEST` 下的开环电机输出
- `mode_debug_task`
  - 负责：
    - 模式维护
    - 串口命令解析
    - CSV 调试输出

当前判断：

- 新任务模型已经落地
- 这一项已经完成主线替换

### 2.6 中断与硬件约束

已完成：

- `GROUP1_IRQHandler`
  - 当前用于编码器 GPIO 事件推进
- `TIMA1_IRQHandler`
  - 当前用于编码器固定周期速度刷新
- 控制计算未放在中断上下文内

当前判断：

- 中断职责划分符合原计划

### 2.7 旧代码替换策略

已完成：

- 新主线已经不再依赖旧 `motion_task`
- 旧 `MotionV2`、`Motor_Encoder_PID`、`K230_UART`、`Gimbal`、`Servo` 已移动到 `archive/legacy/`
- 旧 `motion/imu/oled/log/pid/k230_rx/strategy` 任务已移动到 `archive/legacy/Sources/tasks/`
- 默认构建已切换到新主线

当前判断：

- “直接替换主线，不保留兼容层”这个目标已经实现

## 3. 当前已验证的功能

### 电机

已验证：

- 左右电机可独立设置 duty
- 串口可直接输入 `left,right` 进行轮子测试
- 最小 duty 钳位当前为 `0.10`

### 编码器

已验证：

- 编码器中断与定时采样链已工作
- 左右轮转动时计数与速度会变化
- 已完成全速扫速测试并拿到速度量级数据

当前编码器关键参数：

- `pulses_per_revolution = 1456`
- `wheel_radius_m = 0.0325`
- `sample_period_s = 0.01`

### IMU / MPU6050

已验证：

- `sensor_task` 可完成 IMU 初始化与失败重试
- MPU6050 已可稳定输出姿态与陀螺仪数据
- 已增加稳定标志与启动时间统计
- 驱动已做过一轮 FreeRTOS 友好化处理，避免长时间阻塞式初始化路径直接卡死主线

当前结论：

- IMU 已进入“可用”状态
- 但离“作为底盘控制主链的一部分稳定交付”还有一段距离

### 循迹

已验证：

- `line_sensor` 设备已纳入新系统
- 当前设备输入可读
- 当前仍直接复用 `Drivers/LineTracker`，因此它还不属于本次可归档范围

当前结论：

- 循迹设备接口已完成
- 但循迹控制器尚未重新纳入新主链验收

## 4. 尚未完成的部分

以下内容在代码结构上已有准备，但还不能算本阶段彻底收尾：

- 双轮速度闭环的正式整定还没完成
- `TWIST_OPEN / 闭环底盘模式` 还没完成系统性板上验收
- `yaw_controller` 尚未正式接入底盘主链
- `line_controller` 尚未正式接入底盘主链
- 当前主调试入口仍偏向 `WHEEL_TEST`
- IMU 还需要继续优化启动稳定阶段与最终控制接入策略
- 旧的阶段性汇总文档还没有统一更新到新架构口径

## 5. 当前最值得关注的问题

### 5.1 结构问题已经不是主矛盾

现在的主要问题已经不是“架构该怎么拆”，而是：

- 速度环怎么整定
- 左右轮一致性怎么处理
- IMU 何时、以什么方式重新接入控制链
- 何时从 `WHEEL_TEST` 切回 `TWIST` 主调试路径

### 5.2 当前系统更像“新底盘平台 beta”

更准确地说，当前已经有：

- 稳定的分层
- 稳定的任务模型
- 稳定的设备驱动入口

但控制效果还处在：

- 单模块逐项验证完成
- 整车控制策略正在回填

这个阶段。

## 6. 建议的下一阶段顺序

建议按下面顺序继续推进：

1. 基于现有扫速数据，完成左右轮速度环 PID 初值
2. 在 `TWIST` 模式下恢复双轮闭环调试
3. 统一左右轮方向/符号约定
4. 让 `Chassis_SetTwist(v,w)` 完成基本直行和原地转向验收
5. 再决定是否把 `yaw_controller` 接入 `Twist` 主链
6. 最后再回收 `line_controller`

## 7. 归档结论

截至 2026-04-22，当前重构可以定性为：

- 架构重构：已完成
- 主线替换：已完成
- 电机/编码器：已完成基础可用验证
- MPU6050：已完成基础可用验证
- 循迹设备输入：已完成
- 速度闭环整定：未完成
- `Twist(v,w)` 板上最终验收：未完成
- `yaw_controller / line_controller` 主链接入：未完成

因此，本阶段最准确的归档描述是：

- “新底盘控制架构已经落地并跑通基础硬件链路”
- “控制效果与比赛级主链仍处于下一阶段工作范围”
