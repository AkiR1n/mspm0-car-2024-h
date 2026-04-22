# Legacy Archive

这个目录用于存放已经脱离当前主线、但暂时保留作参考的历史驱动和任务代码。

当前已归档内容：

- `Drivers/MotionV2`
- `Drivers/Motor_Encoder_PID`
- `Drivers/K230_UART`
- `Drivers/Gimbal`
- `Drivers/Servo`
- `Sources/tasks/`
  - `motion_task.*`
  - `imu_task.*`
  - `oled_task.*`
  - `log_task.*`
  - `pid_task.*`
  - `k230_rx_task.*`
  - `strategy_task.*`

当前未归档但名称较旧的模块：

- `Drivers/LineTracker`

原因：

- 新设备层 `Drivers/Devices/line_sensor.c` 仍直接调用 `LineTracker_*`
- 在新的独立循迹设备实现完成之前，`Drivers/LineTracker` 仍属于当前主线依赖

使用约定：

- 这里的代码不参与当前默认主线设计
- 若后续需要彻底删除历史实现，应先确认仓库内无构建、代码或文档引用
