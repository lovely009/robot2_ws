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

固件频率定义在 `micro_ros_firmware/include/firmware_config.h`。micro-ROS 任务管理会话与 USB，电机实时任务管理 CAN、周期执行和本地保护。

## 会话与时钟

Agent 探测按 1 秒间隔调度，有效入站命令和授权帧可推迟探测。连续探测失败后重建会话，电机保护独立处理命令超时。

运行期间定期校时。有效映射超过 30 秒未更新或跳变超过 20 ms 时，时钟状态失效。执行截止时间使用 MCU 单调时钟，校时更新不会延长已接收目标。

会话重建清除旧授权和目标。序号、会话、截止时间与载荷均通过校验后，才更新接收活性。

`fdcan_motor_sim.py` 每秒以主机系统时间重校单调时钟到 epoch 的映射。Agent 与模拟器同机运行时，两端使用同一 epoch 基准。反馈调度和耗时统计使用单调时钟，已入队帧保留采集时的 epoch 与时钟代次。系统时间校正超过 5 ms 会增加 `step_count` 和代次，控制 socket 的 `status` 在单通道 `timing` 或多通道 `channels[].timing` 中返回校时年龄、校正量和读时钟不确定度。

CAN SYNC 的接收线程时间戳随反馈窗口保存，降载兜底窗口标记 `source=fallback`，VAXOR 命令应答标记 `source=command`。每台电机的 `last_feedback` 包含 `trigger`、`encoded` 和 `sent`，用于关联触发、反馈编码和传输接口返回。`encoded` 标记当前模型状态被编码的时刻，模型状态自身的新鲜度仍由被控对象反馈判断。CERB 12 字节、VAXOR 12/20 字节反馈使用原有协议布局。

| 时序字段 | 测量区间 |
| --- | --- |
| `sync_period_ms` / `sync_jitter_ms` | 同通道相邻 SYNC 到达间隔 / 相对配置周期的绝对偏差 |
| `rx_queue_ms` | 接收线程交付到主循环处理 |
| `feedback_lateness_ms` | 编码时刻超过窗口 deadline 的时间 |
| `feedback_queue_ms` | 编码完成到传输接口发送返回 |
| `trigger_to_tx_ms` | 窗口触发到传输接口发送返回 |
| `feedback_gap_ms` | 同一电机相邻反馈发送间隔，包含主循环停顿后兜底补发前的空窗 |
| `transport_send_ms` / `transport_receive_ms` | 一次传输接口调用的耗时，包含锁等待、SDK 执行和背压返回 |
| `loop_work_ms` | 一轮主循环工作耗时，覆盖接收处理、反馈生成和发送 |
| `bridge_spin_ms` / `bridge_publish_ms` | 主循环中的 ROS 回调处理 / 仿真命令发布耗时 |
| `control_service_ms` | 一次控制 socket 请求读取、状态生成或响应发送耗时 |

各项保留累计 `count` / `max` 和最近 512 个样本的 `mean` / `p95` / `window_max`。`feedback_over_period` 统计超过配置反馈周期的响应，`out_of_order_sync` 统计被拒绝的旧 SYNC。观测脚本可同时读取 `/micro_ros_up/status` 的 `clock_valid` / `clock_sync_age_ms` / `clock_sync_failures`，以及 `/motor/control_states` 的 `sync_time_ns` / `state_age_ms`，核对 MCU 校时和端到端新鲜度。

多总线共同执行的主循环、ROS 桥和控制口耗时位于顶层 `loop_timing`，每条总线的收发调用及本轮工作耗时位于 `channels[].timing`。这些耗时使用主机单调时钟，时钟校正期间仍可比较阻塞时长。

`response_by_source_ms` 按 `sync`、`fallback`、`command` 分别统计触发到发送的耗时。排查周期反馈时读取 `sync`，检查兜底空窗时同时读取 `feedback_gap_ms`。RX 日志保留接收线程的 `received` 和主循环处理时的 `time`。

这里的 CAN 时间戳边界为 `host_receive` 和 `transport_send_return`，适配器内部缓存和线上的实际收发时刻需要硬件时间戳另行测量。启动期与稳定期可以分别使用累计最大值和滚动最大值核查。

## 消息与发送

当前 Agent 协议为 schema 4，主机消息、Agent 和 MCU 必须配套构建。

SNAPSHOT 可携带已有授权的续租信息，新授权由 AUTHORITY 握手建立。连续发送有效目标时，主机省略独立保活。空闲时继续按授权期限保活。

配置事务使用可靠消息、去重和有限等待。运动目标使用固定容量邮箱保留最新值。可靠消息发送处理、队列与载荷容量由固件宿主测试覆盖。

页面显示 MCU 链路、时钟和控制授权。状态年龄同时考虑源时间和本地接收时间，重复状态及旧会话消息不会刷新新鲜度。

## 验证

相关测试位于 `micro_ros_firmware/test/host`、`ros2_ws/src/micro_ros_bridge/test` 和 `ros2_ws/src/web_host/frontend/test`。

台架测量需记录 USB/CAN 延迟、调度抖动、断流停止和重连后的实际状态。已有时钟测量见[2026-11-02 报告](../outputs/2026-11-02-clock-sync/report.md)，报告中的频率对应当时固件。
