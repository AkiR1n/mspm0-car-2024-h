# 控制代码单文件总结

本文档汇总 `mspm0-school-2026` 当前与小车控制主链直接相关的代码，目标是用一个文件说明：

- 当前控制架构怎么分层
- 每个任务负责什么
- 题目状态机怎么跑
- 底盘、循迹、IMU 各自怎么参与控制
- 关键参数写在哪
- 当前已知假设和风险是什么

不覆盖 VS Code、烧录、历史归档等非控制主题。

## 1. 当前主线总览

当前系统是一个基于 FreeRTOS 的五任务控制主线：

- `sensor_task`
- `control_task`
- `main_task`
- `test_task`
- `oled_task`

入口在：

- `Sources/main.c`

主线节拍：

- `sensor_task`: 10 ms，采样编码器 / IMU / 7 路循迹
- `control_task`: 10 ms，执行双轮速度闭环与 `Twist(v,w)` 差速解算
- `main_task`: 10 ms，执行赛题状态机
- `test_task`: 50 ms，处理按键、串口命令、状态打印
- `oled_task`: 100 ms，显示题号 / 相位 / 最近事件 / 关键观测量

这条主线已经替代旧 `motion_task` 风格控制。

## 2. 分层结构

### 2.1 Device 层

直接参与当前控制的设备层：

- `Drivers/Devices/motor_drv.*`
- `Drivers/Devices/encoder_drv.*`
- `Drivers/Devices/imu_drv.*`
- `Drivers/Devices/line_sensor.*`

这些设备层向上提供：

- 电机 duty 输出
- 编码器速度与累计计数
- IMU 的 `yaw_deg / gyro_z / stable / ready`
- 7 路循迹的 `bits / detected / position`

### 2.2 Control 层

控制层组件：

- `Control/pid.*`
- `Control/wheel.*`
- `Control/chassis.*`
- `Control/yaw_controller.*`
- `Control/line_controller.*`

职责拆分：

- `pid`: 通用 PID 算法
- `wheel`: 单轮速度环，输出电机 duty
- `chassis`: 差速底盘解算，把 `v,w` 变成左右轮目标速度
- `yaw_controller`: 航向保持控制器，当前用于无线段短时稳向
- `line_controller`: 线位置 PID，当前用于弧线循迹

### 2.3 App / Task 层

上层业务逻辑位于：

- `Sources/app_state.*`
- `Sources/chassis_system.*`
- `Sources/tasks/*.c`
- `Sources/signal_driver.*`

其中：

- `app_state` 是全局共享状态中心
- `chassis_system` 是控制对象与默认参数装配点
- `main_task` 是赛题状态机
- `signal_driver` 目前是声光事件空实现占位

## 3. 全局状态中心

文件：

- `Sources/app_state.h`
- `Sources/app_state.c`

`app_state` 是当前系统所有任务共享数据的唯一交换中心，集中维护：

- 底盘反馈 `chassis_feedback_t`
- 底盘命令 `chassis_command_t`
- 控制调试量 `chassis_debug_t`
- 当前模式 `app_mode_t`
- 赛题状态 `app_challenge_info_t`

### 3.1 反馈量

当前反馈包含：

- 左右轮速度
- 左右轮计数
- IMU：`ready / stable / yaw_deg / gyro_z / pitch / roll / accel / gyro / uptime`
- 循迹：`line_bits / line_detected / line_position`

### 3.2 运行模式

当前模式：

- `APP_MODE_STOP`
- `APP_MODE_TWIST_OPEN`
- `APP_MODE_WHEEL_TEST`
- `APP_MODE_WHEEL_SPEED_TEST`
- `APP_MODE_MAIN`

含义：

- `STOP`: 停车
- `TWIST_OPEN`: 直接给底盘 `v,w`
- `WHEEL_TEST`: 直接开环给左右电机 duty
- `WHEEL_SPEED_TEST`: 直接给左右轮闭环目标速度
- `MAIN`: 跑赛题状态机

### 3.3 赛题状态

当前赛题选择支持：

- `Q1`
- `Q2`
- `Q3`
- `Q4`

赛题状态字段包含：

- `selected / active`
- `status`
- `phase`
- `action`
- `checkpoint_count`
- `lap_index / lap_total`
- `geometry_heading_deg`
- `hold_heading_deg`
- `heading_error_deg`
- `phase_distance_m`
- `target_speed_mps`
- `last_event / last_event_ms / event_seq`

事件类型：

- `START`
- `PASS_A`
- `PASS_B`
- `PASS_C`
- `PASS_D`
- `STOP`

## 4. 底盘对象与默认参数

文件：

- `Sources/chassis_system.c`

这里集中创建了：

- 左右电机
- 左右编码器
- 左右轮控制器
- 差速底盘对象
- IMU 对象
- 线控制器
- 航向控制器

### 4.1 当前默认几何参数

写死在 `chassis_system.c` 与 `main_task.c` 中：

- 轮半径：`0.0325 m`
- 编码器脉冲：`13 * 28 * 4 = 1456`
- 轮距：`0.14 m`

这几个量当前都还是“默认值”，还不是确认过的实测值。

### 4.2 左右轮速度环默认参数

左右轮分别有独立 PID 和 feedforward。

左轮：

- `kp = 0.206276`
- `ki = 1.748212`
- `kd = 0`
- `ff = 0.825105`

右轮：

- `kp = 0.208663`
- `ki = 1.768348`
- `kd = 0`
- `ff = 0.834652`

### 4.3 航向与循迹控制器默认参数

航向控制器：

- `kp = 0.02`
- `ki = 0`
- `kd = 0.0005`
- `out = [-6, 6]`
- `rate_ff_gain = 0.0005`

线控制器：

- `kp = 0.067`
- `ki = 0`
- `kd = 0.0004`
- `out = [-2.6, 2.6]`

## 5. 底盘控制链

相关文件：

- `Control/wheel.c`
- `Control/chassis.c`
- `Sources/tasks/control_task.c`

### 5.1 单轮控制

`Wheel_ControlStep()` 的逻辑很直接：

1. 从编码器读当前轮速
2. 用 PID 计算闭环输出
3. 加上 `speed_ff_gain * target_speed_mps`
4. 把结果钳到 `[-1, 1]`
5. 输出到电机 duty

如果目标速度接近 0：

- reset PID
- 直接停电机

### 5.2 差速底盘

`Chassis_SetTwist(v, w)` 的逻辑：

- `left = v - w * wheel_base / 2`
- `right = v + w * wheel_base / 2`

所以轮距不准会直接导致实际转向量与期望不一致。

### 5.3 `control_task`

`control_task` 周期 10 ms，执行顺序：

1. 读 `app_state` 中的反馈和命令
2. 按模式决定控制方式
3. 在 `MAIN` / `TWIST_OPEN` 模式下执行底盘 `Twist(v,w)` 控制
4. 在 `WHEEL_SPEED_TEST` 下执行左右轮速度闭环
5. 在 `WHEEL_TEST` 下直接输出电机 duty
6. 把调试量写回 `app_state`

这意味着 `main_task` 本身不直接碰电机，只写目标命令。

## 6. 传感器采样链

文件：

- `Sources/tasks/sensor_task.c`
- `Drivers/Devices/imu_drv.c`

### 6.1 `sensor_task`

`sensor_task` 每 10 ms：

- 刷新 IMU
- 刷新循迹
- 读左右编码器速度和计数
- 拼成 `chassis_feedback_t`
- 写回 `app_state`

### 6.2 当前 IMU 工作方式

当前 IMU 配置：

- `warmup_ms = 1200`
- `stable_hold_ms = 300`
- `estimate_gyro_bias = 1`
- `apply_dmp_bias = 0`
- `zero_yaw_on_stable = 1`

当前 `imu_drv` 的关键事实：

- 上电后先积累陀螺零偏
- 使用 `gyro_z` 积分得到 `yaw_rel_deg`
- 稳定后把当前角度置零
- 当前没有安装姿态补偿矩阵
- 当前没有“IMU 与车体前进方向偏角”的补偿
- 当前默认假设可用的偏航轴就是 `gyro_z`

因此：

- 如果 IMU 只是平面内旋转一个固定角度，后续可以做软件补偿
- 如果 IMU 没固定、会晃，当前 `yaw_deg` 不能可靠用于主控制

## 7. 赛题状态机

核心文件：

- `Sources/tasks/main_task.c`

当前架构是“动作原语 + 相位表”，不是每题单独写一套控制代码。

### 7.1 当前动作原语

相位动作只有四类：

- `ALIGN_START`
- `GAP_TRAVERSE`
- `ARC_TRACK`
- `STOP_AND_SIGNAL`

含义：

- `ALIGN_START`: 起跑前静止确认
- `GAP_TRAVERSE`: 无线段穿越
- `ARC_TRACK`: 半圆弧循迹
- `STOP_AND_SIGNAL`: 结束停车并发事件

### 7.2 当前题目路径

`Q1`:

- `ALIGN_A_TO_B`
- `GAP_AB`
- `STOP_B`

`Q2`:

- `ALIGN_A_TO_B`
- `GAP_AB`
- `ARC_BC`
- `GAP_CD`
- `ARC_DA`
- `STOP_A`

`Q3`:

- `ALIGN_A_TO_C`
- `GAP_AC`
- `ARC_CB`
- `GAP_BD`
- `ARC_DA`
- `STOP_A`

`Q4`:

- 起始相位与 `Q3` 相同
- 其中 `GAP_AC -> ARC_CB -> GAP_BD -> ARC_DA` 连续跑 4 圈

### 7.3 当前几何航向常量

当前主状态机里直接写死了：

- `AB = 0°`
- `CD = 180°`
- `AC = -38.66°`
- `BD = -141.34°`

这些几何角当前只用于相位描述与对齐显示，并没有形成完整“几何航向闭环主导航”。

## 8. 无线段控制

相关函数：

- `run_gap_traverse()`

当前无线段控制输出由三部分组成：

- `YawController_Update()` 的航向控制项
- 左右轮累计计数差修正 `count_error * MAIN_GAP_COUNT_KP`
- 左右轮速度差修正 `speed_error * MAIN_GAP_SPEED_KD`

当前速度策略：

- 巡航速度 `0.46 m/s`
- 末段速度 `0.28 m/s`

当前段退出逻辑：

- 先离开线
- 行驶里程超过 `min_exit_m`
- 重新看到线并持续确认 `40 ms`

当前已写入的典型里程阈值：

- 直线 `AB/CD`: `1.000 m`，最小重获线 `0.78 m`
- 对角 `AC/BD`: `1.281 m`，最小重获线 `1.03 m`

### 8.1 当前无线段的现实限制

虽然代码里接了 IMU 航向控制，但当前有三个实际风险：

- IMU 安装方向未补偿
- IMU 可能未刚性固定
- 轮径与轮距仍是默认值

所以当前无线段更像：

- 编码器直走修正为主
- IMU 航向项为辅助

## 9. 弧线控制

相关函数：

- `run_arc_track()`
- `select_arc_zone_speed()`
- `select_arc_turn_scale()`
- `get_arc_inner_bias_w()`

### 9.1 当前弧线控制结构

弧线段是纯循迹，不使用 IMU 判定弧线。

控制输出：

- `LineController_Update()` 生成基础转向
- 按线误差放大边缘纠偏
- 按分区切换转向放大系数
- 尾段叠加向内偏置

### 9.2 当前弧线分区

每个弧线 profile 包含：

- `front_end_m`
- `rear_start_m`
- `speed_front_mps`
- `speed_mid_mps`
- `speed_rear_mps`
- `turn_scale_front`
- `turn_scale_mid`
- `turn_scale_rear`
- `inner_bias_start_m`
- `inner_bias_full_m`
- `inner_bias_w_radps`
- `lost_confirm_ms`

也就是说每条弧线都已经被建模成：

- 前段
- 中段
- 后段

### 9.3 当前弧线独立参数组

当前独立 profile：

- `k_arc_profile_bc`
- `k_arc_profile_cb`
- `k_arc_profile_da`

说明：

- `BC` 与 `CB` 已经分开
- `Q2` 的正向弧与 `Q3/Q4` 的反向弧不是共用一套参数

### 9.4 当前 `BC` 段参数

目前 `BC` 段已被人为加重“尾段向内保守性”：

- `min_exit_m = 1.08`
- `rear_start_m = 0.86`
- `speed_rear_mps = 0.24`
- `turn_scale_rear = 1.38`
- `inner_bias_w_radps = 0.38`
- `lost_confirm_ms = 70`

这是一套偏“优先覆盖 C 点、宁可保守”的参数。

## 10. 事件、OLED 与声光

### 10.1 事件接口

当前主状态机只发事件，不直接写 GPIO。

事件通过：

- `app_state_emit_event()`

再转给：

- `signal_emit()`

目前 `signal_driver.c` 还是空实现，只做编译占位。

### 10.2 OLED

`oled_task` 当前显示：

- 题号
- 运行状态
- 当前相位
- 当前动作类型
- 最近事件与时间戳
- 无线段时显示航向误差与段内里程
- 弧线段时显示 line position 与段内里程

### 10.3 `test_task`

`test_task` 是当前最重要的调试入口，负责：

- `Key1` 切换 `Q1..Q4`
- `Key2` 启动或安全停止
- 串口命令解析
- 自动采样输出
- 人类可读状态输出
- 事件变化打印

它还能切换：

- `STOP`
- `MAIN`
- `WHEEL_TEST`
- `WHEEL_SPEED_TEST`
- `TWIST_OPEN`

并支持在线改轮速 PID。

## 11. 当前控制代码里的主要硬假设

当前系统默认成立的假设有：

### 11.1 硬件假设

- 左右轮半径接近 `0.0325 m`
- 轮距接近 `0.14 m`
- 编码器方向定义正确
- 7 路循迹居中安装
- IMU 至少是固定的

### 11.2 场地几何假设

- `AB/CD = 1.0 m`
- `AC/BD = 1.281 m`
- 半圆弧长约 `1.257 m`
- 起跑朝向允许人为摆正

### 11.3 控制假设

- 弧线段主要靠循迹闭环
- 无线段主要靠编码器差和短时航向保持
- 每段切换时控制器状态会 reset
- 不跨相位累计弧线误差

## 12. 当前已知风险与调试重点

### 12.1 IMU 相关

当前最大风险不是算法，而是安装质量：

- IMU 未固定
- IMU 与车体前进方向不平行
- 未做安装偏角补偿

如果这些问题存在，当前 `yaw_deg` 只能参考，不能当强约束。

### 12.2 底盘几何相关

当前轮径和轮距不是实测标定值。

后果：

- 无线段距离判断会偏
- `Twist(v,w)` 的实际转向量会偏
- 两轮同速时可能天然跑偏

### 12.3 `BC -> CD` 偏移问题

当前 `Q2` 的主要难点仍然在：

- `BC` 尾段出弯姿态残余偏差
- `CD` 无线段起始角未必被可靠拉回

所以这部分问题本质上是三件事叠加：

- 弧线尾段参数
- 底盘几何未标定
- IMU 不确定性

## 13. 现在最值得做的事

如果目标是尽快把控制链调稳定，优先级建议是：

1. 固定 IMU，至少保证它不晃
2. 实测轮径和轮距
3. 做直线 `1 m` 测试，确认底盘是否天然跑偏
4. 做低速定角转向测试，确认轮距估计是否离谱
5. 再回头调 `Q1/Q2`

在硬件未稳定前，不建议继续靠堆 `BC` 特例参数解决全部偏移问题。

## 14. 当前控制相关文件清单

主入口与状态：

- `Sources/main.c`
- `Sources/app_state.h`
- `Sources/app_state.c`
- `Sources/chassis_system.c`

任务：

- `Sources/tasks/sensor_task.c`
- `Sources/tasks/control_task.c`
- `Sources/tasks/main_task.c`
- `Sources/tasks/test_task.c`
- `Sources/tasks/oled_task.c`

控制层：

- `Control/pid.c`
- `Control/wheel.c`
- `Control/chassis.c`
- `Control/yaw_controller.c`
- `Control/line_controller.c`

设备层：

- `Drivers/Devices/motor_drv.c`
- `Drivers/Devices/encoder_drv.c`
- `Drivers/Devices/imu_drv.c`
- `Drivers/Devices/line_sensor.c`

事件占位：

- `Sources/signal_driver.c`

## 15. 一句话结论

当前控制代码已经从“驱动拼接”进入“可维护的控制框架”阶段：

- 架构是清楚的
- 状态机已经成形
- 无线段 / 弧线段已经分治
- 事件 / OLED / 测试入口已经打通

但离“稳定比赛参数”还有一层硬件标定工作没完成，最核心的短板仍然是：

- IMU 安装未定型
- 轮径 / 轮距未实测
- `Q2` 的 `BC -> CD` 过渡仍受上述误差放大
