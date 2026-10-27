# 控制板系统参数

系统参数由 `/micro_ros_node` 的标准 ROS 参数服务提供，使用现有 rclc_parameter
低内存模式（5 个服务；每次请求 1 个参数），电机参数继续使用原配置事务。

| 参数 | 可选值 | 默认值 |
| --- | --- | --- |
| `system.agent_ping_hz` | 10、20、50、100 Hz | 100 Hz |
| `system.motor_command_hz` | 50、100、125、200、250、500、1000 Hz | 200 Hz |
| `system.motion_report_hz` | 20、25、50 Hz | 50 Hz |
| `system.status_report_hz` | 10、20、25、50 Hz | 10 Hz |
| `system.imu_report_hz` | 10、20、25、50、100 Hz | 100 Hz |
| `system.temperature_report_hz` | 1、2 Hz | 2 Hz |

所有值均为整数。修改要求时钟有效、电机失能、无正在加载或执行的轨迹、无保存事务。
频率控制发送机会，不保证实际吞吐；重要状态变化仍保留最快 20 ms 的可靠上报机会，
采样、租约与安全超时不随上报频率改变。

`system.motor_command_hz` 直接控制实时任务的轨迹插值、目标保持重发和配置消费边界；
任务仍每 1 ms 服务 CAN RX/TX，控制周期为 1000 / Hz 毫秒。修改热生效，默认仍为
200 Hz，只有保存 W25 才在重启后保留。上限 1 kHz 表示调度配置能力，实际每台电机的
发送频率受总线容量、协议聚合、设备响应和任务耗时影响，需带实际电机测量。

只读参数：`system.can_nominal_kbps=1000`、`system.can_data_kbps=5000`，以及
`system.unsaved`。CAN 使用原有已标定位时序，不提供在线改速或 USB CDC 波特率设置。

在设备工作台选择“固件参数”后可读取、逐项下发并回读、保存 W25。标准参数服务
不提供整批原子写入；若部分项被拒绝，页面会报告已下发数量并要求回读，已生效项不会
自动回滚。旧固件没有服务或参数时不显示默认值为设备真值。

保存复用 `/motor/execute_config` 的整机 SAVE 事务，同时保存已下发的电机参数。
系统配置使用 v3 blob 头部原有 12 字节预留区（SYS2 标识），纳入原 CRC 和双槽
写入/读回核验；电机记录布局不变。旧 SYS1 配置保留原上报频率，控制频率补为默认
200 Hz；没有系统配置时加载系统默认值，仍保留电机记录。
保存失败不更新“已保存”快照。升级前旧固件再次保存可能清除系统扩展；再次升级后回到默认值。

只读检查示例（先启动 Agent 并加载工作区环境）：

```bash
ros2 param list /micro_ros_node
ros2 param get /micro_ros_node system.motion_report_hz
ros2 param get /micro_ros_node system.unsaved
```

需要重新烧录新固件后才会出现这些服务。Host 测试、目标板编译和浏览器模拟服务测试
不代替真机的服务建链、断线重连、上报频率、掉电恢复和任务时序验证。
