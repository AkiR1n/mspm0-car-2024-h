# 自动化轮子/编码器采集脚本

本文档说明如何用脚本自动采集电机与编码器数据，用于后续速度环 PID 整定。

脚本位置：

- [tools/encoder_step_test.py](/home/aki/workspace_ccstheia/mspm0-school-2026/tools/encoder_step_test.py)

## 1. 前提

固件应满足以下条件：

- 当前板子运行在 `WHEEL_TEST` 模式
- 串口命令格式为 `left,right\n`
- 固件会周期输出：

```text
csv,<ms>,<left_cmd>,<right_cmd>,<left_duty>,<right_duty>,<left_mps>,<right_mps>,<left_count>,<right_count>,<irq_per_s>,<sample_tick>
```

## 2. 依赖

脚本依赖 `pyserial`：

```sh
python -m pip install pyserial
```

如果你在虚拟环境里跑，就先激活虚拟环境再安装。

## 3. 默认用途

脚本默认会自动跑一组占空比台阶：

- 静止
- 左轮单独 `5/10/20`
- 右轮单独 `5/10/20`
- 双轮 `10/20`
- 每个动作之间自动插入静止段

这组数据足够先估：

- 最小起转 duty
- 左右轮稳态速度增益
- 左右轮不一致程度
- 编码器计数是否线性

## 4. 直接运行

```sh
python tools/encoder_step_test.py --port /dev/ttyACM0
```

默认输出目录：

- `logs/wheel_pid/`

脚本会生成两个文件：

- 原始串口日志：`wheel_step_raw_*.log`
- 结构化 CSV：`wheel_step_data_*.csv`

## 5. 自定义测试序列

你也可以自己指定 phase。

格式有两种：

- `left,right,duration`
- `name:left,right,duration`

示例：

```sh
python tools/encoder_step_test.py \
  --port /dev/ttyACM0 \
  --no-default-phases \
  --phase idle:0,0,2 \
  --phase left_low:8,0,4 \
  --phase idle2:0,0,2 \
  --phase right_low:0,8,4 \
  --phase both:12,12,5
```

## 6. CSV 字段说明

- `host_time_iso`：PC 侧收到该条记录的时间
- `phase_index`：当前动作编号
- `phase_name`：当前动作名称
- `phase_elapsed_s`：当前动作已运行时间
- `board_ms`：板端 `xTaskGetTickCount()` 换算的毫秒
- `left_cmd/right_cmd`：串口下发目标 duty
- `left_duty/right_duty`：电机实际应用 duty
- `left_mps/right_mps`：编码器测得轮速
- `left_count/right_count`：累计计数
- `irq_per_s`：编码器 GPIO 中断速率
- `sample_tick`：测速采样 tick

## 7. 推荐采集方法

建议先离地空载：

1. 跑默认脚本一轮，确认左右轮方向、速度量级、编码器计数都正常。
2. 如果低占空比起不来，把 `5` 改成 `8` 或 `10` 再跑。
3. 如果电机太快或机械抖动明显，把最高档从 `20` 降到 `15`。
4. 数据正常后，再补一轮更细的 duty 梯度，例如 `6/8/10/12/15/18/20`。

## 8. 后续用途

拿到 `wheel_step_data_*.csv` 后，可以继续做：

- 估算左/右轮最小起转 duty
- 拟合 `duty -> speed(m/s)` 的稳态曲线
- 计算左右轮补偿系数
- 给速度环 PID 初值

如果你把采到的 CSV 发回来，我可以直接帮你做第一版整定。
