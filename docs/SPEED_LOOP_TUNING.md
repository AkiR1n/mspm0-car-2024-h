# 速度环 PID 说明

本文档对应当前重构后的轮级速度环主线，核心代码在：

- [Control/chassis.c](Control/chassis.c)
- [Control/wheel.c](Control/wheel.c)
- [Sources/chassis_system.c](Sources/chassis_system.c)
- [Sources/tasks/mode_debug_task.c](Sources/tasks/mode_debug_task.c)
- [tools/tune_speed_loop.py](tools/tune_speed_loop.py)

当前实现里：

- `Chassis_SetWheelSpeed()` 直接下发左右轮目标速度，不额外做软件限速。
- `Wheel_ControlStep()` 使用 `PID + feedforward` 计算 duty，并最终钳位到 `[-1.0, 1.0]`。
- 一旦 duty 已经顶到 `1.0`，再继续提高目标速度也不会得到额外控制余量，只会进入饱和。

## 1. 当前默认参数

当前默认速度环参数已经写回 [Sources/chassis_system.c](Sources/chassis_system.c)：

左轮：

- `kp = 0.206276`
- `ki = 1.748212`
- `kd = 0.0`
- `ff = 0.825105`

右轮：

- `kp = 0.208663`
- `ki = 1.768348`
- `kd = 0.0`
- `ff = 0.834652`

这组参数对应的是 2026-04-22 当天的自动化整定结果，已经重新编译并刷入过板子。

## 2. 当前测速结果

本次基线结果来自：

- [logs/speed_tuning/speed_tune_summary_20260422_170128.json](logs/speed_tuning/speed_tune_summary_20260422_170128.json)
- [logs/speed_tuning/speed_tune_summary_20260422_171306.json](logs/speed_tuning/speed_tune_summary_20260422_171306.json)
- [logs/speed_tuning/speed_tune_summary_20260422_171707.json](logs/speed_tuning/speed_tune_summary_20260422_171707.json)

关键数据：

- 左轮开环最高测速约 `1.218 m/s`
- 右轮开环最高测速约 `1.205 m/s`
- 95% 高速验证目标：
  - 左 `1.158 m/s`
  - 右 `1.146 m/s`
  - 单轮闭环时 applied duty 大约 `0.95 ~ 0.96`
- 98% 高速验证目标：
  - 左 `1.194 m/s`
  - 右 `1.181 m/s`
  - 单轮闭环实测分别约 `1.217 m/s` 和 `1.205 m/s`
  - 单轮和双轮同时跑时都已经长期顶到 `duty = 1.0`

这说明：

- 现在这套 PID 已经基本把当前电机和轮子的上限吃满了。
- `0.98 * 开环最大速度` 这一档可以跑，但已经接近硬顶，没有明显调节余量。
- 如果继续把目标再往上抬，得到的不是更高的可控速度，而是更早进入饱和。

## 3. 实际建议工作上限

如果目标是“长期可控”和“保留一定余量”，建议每轮速度上限先按下面理解：

- 推荐连续工作区：`1.15 ~ 1.18 m/s`
- 贴着上限跑：左约 `1.19 m/s`，右约 `1.18 m/s`

如果是双轮同时贴上限跑，当前系统也能稳定，但此时控制器已经非常接近纯占空比顶满状态，不适合再把它当作有余量的闭环工作点。

## 4. 自动整定脚本现在做什么

[tools/tune_speed_loop.py](tools/tune_speed_loop.py) 当前流程是：

1. 可选 `--flash` 先烧录固件。
2. 可选 `--reset-before-start` 只做一次 J-Link 复位。
3. 打开串口，等待板端进入 ready 状态。
4. 开启 `auto,...` 机器协议模式。
5. 对左右轮分别做开环 duty 扫描。
6. 从开环数据估算：
   - 最小起转 duty
   - 最大速度
   - `speed_per_duty`
   - `ff_gain`
   - 一阶惯性 `tau`
   - 一版 `kp/ki`
7. 把参数下发到板端 RAM。
8. 做闭环低速、中速、高速验证。
9. 保存原始日志、CSV 和 summary JSON。

当前脚本已经不是“板端持续刷流”模式，而是 PC 主机按固定周期发送 `auto,sample` 请求，板端按次返回状态。这样串口更稳，也更适合后续自动化处理。

## 5. 串口协议

人工调试命令：

```text
<left_percent>,<right_percent>
spd,<left_mps>,<right_mps>
twist,<v_mps>,<w_radps>
pid,<kp>,<ki>,<kd>[,<ff>]
pidl,<kp>,<ki>,<kd>[,<ff>]
pidr,<kp>,<ki>,<kd>[,<ff>]
showpid
stop
help
```

自动化命令：

```text
auto,on
auto,off
auto,phase,<label>
auto,duty,<left_percent>,<right_percent>
auto,spd,<left_mps>,<right_mps>
auto,pid
auto,pid,left,<kp>,<ki>,<kd>[,<ff>]
auto,pid,right,<kp>,<ki>,<kd>[,<ff>]
auto,pid,both,<kp>,<ki>,<kd>[,<ff>]
auto,sample
auto,stop
```

自动采样返回格式：

```text
auto,sample,<board_ms>,<phase>,<mode>,<cmd_left_duty>,<cmd_right_duty>,<cmd_left_speed>,<cmd_right_speed>,<target_left_speed>,<target_right_speed>,<measured_left_speed>,<measured_right_speed>,<applied_left_duty>,<applied_right_duty>,<left_count>,<right_count>,<irq_per_s>,<sample_tick>
```

说明：

- 行尾兼容 `LF`、`CR`、`CRLF`
- 当前推荐由脚本统一驱动，不建议手动混发 `auto,...` 命令和普通测试命令

## 6. 推荐命令

仅复位再整定：

```sh
.venv/bin/python -u tools/tune_speed_loop.py \
  --port /dev/ttyACM0 \
  --reset-before-start
```

先烧录再整定：

```sh
.venv/bin/python -u tools/tune_speed_loop.py \
  --port /dev/ttyACM0 \
  --flash
```

拉高到近满速验证：

```sh
.venv/bin/python -u tools/tune_speed_loop.py \
  --port /dev/ttyACM0 \
  --reset-before-start \
  --duty-steps 10,15,20,25,30,40,50,60,70,80,90,100 \
  --verify-high-fraction 0.95
```

更激进地逼近上限：

```sh
.venv/bin/python -u tools/tune_speed_loop.py \
  --port /dev/ttyACM0 \
  --reset-before-start \
  --duty-steps 10,15,20,25,30,40,50,60,70,80,90,100 \
  --verify-high-fraction 0.98
```

注意：

- `--verify-high-fraction` 在脚本里会被限制到 `0.65 ~ 0.98`
- `0.98` 已经接近“硬顶测速”，不应再把它理解成有余量的工作点

## 7. 何时需要重新整定

下面这些变化都应该重新跑一次速度环自动化：

- 更换电机
- 更换减速比
- 更换轮径
- 更换编码器安装方式
- 编码器参数更新
- 供电明显变化
- 从离地测试切到落地带载使用

重新整定前先确认 [Sources/chassis_system.c](Sources/chassis_system.c) 里的基础参数：

- `pulses_per_revolution`
- `wheel_radius_m`
- 左右轮 `duty_polarity`

## 8. 当前已知限制

- 脚本会把参数下发到板端 RAM，但不会自动改源码。
- 默认值是否写回代码，当前仍由人工决定。
- `summary.json` 里的 `verify_both_high` 目前是聚合后的单个标量，不适合拿来分别看左右轮细节；需要看双轮高速细节时，以 `raw log` 或 `csv` 为准。
- 当前速度环只覆盖轮级，不包含底盘级 `v/w` 约束和整车级限速策略。

## 9. 一句话结论

这套速度环现在已经可用，并且已经基本吃满当前电机转速。

如果你要稳定留余量，就按 `1.15 ~ 1.18 m/s` 用；如果你要贴着上限跑，当前这组 PID 也能把轮子推到大约 `1.19 / 1.18 m/s`，但那时 duty 已经接近或达到饱和。
