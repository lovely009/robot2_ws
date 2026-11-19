# micro-ROS 通信与执行时序

## 频率

| 数据或任务 | 频率 | 配置 |
| --- | --- | --- |
| 主机周期目标 | 50 Hz | ros2_control 控制器 |
| MCU 运动反馈 | 50 Hz | `FIRMWARE_MOTOR_REPORT_HZ` |
| MCU 状态反馈 | 10 Hz | `FIRMWARE_STATUS_REPORT_HZ` |
| MCU 电机执行 | 200 Hz | `FIRMWARE_MOTOR_COMMAND_HZ` |
| IMU | 100 Hz | `FIRMWARE_IMU_REPORT_HZ` |
| 温度 | 2 Hz | `FIRMWARE_TEMPERATURE_REPORT_HZ` |

MCU 固件需另行配套部署，频率由固件配置决定。micro-ROS 任务管理会话与 USB，电机实时任务管理 CAN、周期执行和本地保护。

## 会话与时钟

Agent 探测按 1 秒间隔调度，有效入站命令和授权帧可推迟探测。连续探测失败后重建会话，电机保护独立处理命令超时。

运行期间定期校时。有效映射超过 30 秒未更新或跳变超过 20 ms 时，时钟状态失效。执行截止时间使用 MCU 单调时钟，校时更新不会延长已接收目标。

会话重建清除旧授权和目标。序号、会话、截止时间与载荷均通过校验后，才更新接收活性。

## 消息与发送

当前 Agent 协议为 schema 4，主机消息、Agent 和 MCU 必须配套构建。

SNAPSHOT 可携带已有授权的续租信息，新授权由 AUTHORITY 握手建立。连续发送有效目标时，主机省略独立保活。空闲时继续按授权期限保活。

配置事务使用可靠消息、去重和有限等待。运动目标使用固定容量邮箱保留最新值。可靠消息发送处理、队列与载荷容量由固件宿主测试覆盖。

页面显示 MCU 链路、时钟和控制授权。状态年龄同时考虑源时间和本地接收时间，重复状态及旧会话消息不会刷新新鲜度。

## 验证

相关测试位于 `ros2_ws/src/micro_ros_bridge/test` 和 `ros2_ws/src/web_host/frontend/test`。

台架测量需记录 USB/CAN 延迟、调度抖动、断流停止和重连后的实际状态。已有时钟测量见[2026-11-02 报告](../outputs/2026-11-02-clock-sync/report.md)，报告中的频率对应当时固件。
