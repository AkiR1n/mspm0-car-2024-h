# 轮子/编码器全速扫速测试记录

测试日期：2026-04-22

本文档记录当前开环轮子/编码器全速扫速测试结果，用于后续速度环 PID 初值整定与左右轮补偿。

## 1. 测试目标

- 对左轮、右轮分别完成 `0.01 ~ 1.00` 全占空比扫速
- 记录最小可测速度、最大速度
- 记录当前编码器关键参数
- 作为后续速度环 PID 整定的基线数据

## 2. 测试条件

- 固件模式：`WHEEL_TEST`
- 控制方式：开环直接下发 duty
- 串口命令格式：`left,right\n`
- 扫速方式：
  - 左轮单独从 `0.01` 递增到 `1.00`
  - 中间静止
  - 右轮单独从 `0.01` 递增到 `1.00`
- 每档持续 `1 s`
- 档位总数：每轮 `100` 档

说明：

- 当前电机驱动存在最小占空比钳位，配置为 `min_duty = 0.10f`
- 因此命令占空比 `0.01 ~ 0.10` 都会被实际抬升到 `0.10`
- 所以“最小命令档位”不等于“最小实际电机占空比”

相关代码位置：

- [Sources/chassis_system.c](/home/aki/workspace_ccstheia/mspm0-school-2026/Sources/chassis_system.c:26)
- [Drivers/Devices/motor_drv.c](/home/aki/workspace_ccstheia/mspm0-school-2026/Drivers/Devices/motor_drv.c:42)

## 3. 原始数据

- 结构化 CSV：
  [wheel_step_data_20260422_145024.csv](/home/aki/workspace_ccstheia/mspm0-school-2026/logs/wheel_pid/wheel_step_data_20260422_145024.csv)
- 原始串口日志：
  [wheel_step_raw_20260422_145024.log](/home/aki/workspace_ccstheia/mspm0-school-2026/logs/wheel_pid/wheel_step_raw_20260422_145024.log)
- 分析脚本：
  [tools/analyze_wheel_sweep.py](/home/aki/workspace_ccstheia/mspm0-school-2026/tools/analyze_wheel_sweep.py)
- 采集脚本：
  [tools/encoder_step_test.py](/home/aki/workspace_ccstheia/mspm0-school-2026/tools/encoder_step_test.py)

## 4. 编码器参数

当前编码器配置来自：

- [Sources/chassis_system.c](/home/aki/workspace_ccstheia/mspm0-school-2026/Sources/chassis_system.c:34)

参数如下：

- `pulses_per_revolution = 13 * 28 * 4 = 1456`
- `wheel_radius_m = 0.0325`
- `sample_period_s = 0.01`

推导结果：

- 轮周长：`0.204204 m`
- `counts_per_meter = 7130.141`
- `meters_per_count = 0.00014025 m`

## 5. 关键结果

分析结果如下：

- 左轮最小可测运动：
  - 命令 duty：`0.010`
  - 测得速度：`0.107 m/s`
  - phase：`l001`
- 左轮最大速度：
  - 命令 duty：`1.000`
  - 测得速度：`1.232 m/s`
  - phase：`l100`
- 右轮最小可测运动：
  - 命令 duty：`0.010`
  - 测得速度：`0.100 m/s`
  - phase：`r001`
- 右轮最大速度：
  - 命令 duty：`1.000`
  - 测得速度：`1.220 m/s`
  - phase：`r100`

## 6. 结果解释

### 最小速度

当前记录到的最小可测速度约为：

- 左轮：`0.107 m/s`
- 右轮：`0.100 m/s`

但这里必须注意：

- 这不是“电机真实的最小起转占空比测试结果”
- 这是在 `min_duty = 0.10` 钳位存在时得到的最小稳定速度
- 也就是说，命令 `0.01` 实际已经按 `0.10` 在驱动电机

因此，当前系统可直接用于速度环整定的更准确结论是：

- 最小实际电机占空比：`0.10`
- 对应最小稳定速度大约：`0.10 ~ 0.11 m/s`

### 最大速度

当前空载开环最高速度约为：

- 左轮：`1.232 m/s`
- 右轮：`1.220 m/s`

左右轮满速能力比较接近，差异约 `0.012 m/s`，量级不到 `1%`

### 方向与符号

当前日志中：

- 左轮正转速度为正
- 右轮正转方向被记录为负值

这说明当前右轮编码器/轮速符号定义与左轮相反。做速度环和底盘闭环时必须统一这一点，但不影响本次“速度量级”记录。

## 7. 对 PID 整定的直接意义

这份数据可以直接用于后续速度环初始化：

- 速度环不要把目标速度压到 `0.10 m/s` 以下再期待稳定调节
- 开环前馈可先按：
  - 左轮：`1.232 m/s @ duty=1.00`
  - 右轮：`1.220 m/s @ duty=1.00`
- 若保留 `min_duty = 0.10`，低速段会有明显死区台阶效应

## 8. 后续建议

- 若后续要做真正的“最小起转 duty”测试，需要临时把 `min_duty` 降低或关闭
- 若后续要整速度环，建议先统一左右轮正方向符号
- 若后续要做双轮一致性补偿，当前满速差异不大，重点应先放在低速段
