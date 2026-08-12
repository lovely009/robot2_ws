# ADR 0006：dev-real「三自适应」闭环与默认 tty

**Status**: accepted（`dev-real` 专用；取代 ADR 0005 中"移除串口"的结论）

## Context

ADR 0005 精简 `dev-real` 时移除了 `BusConfig.transport`/`tty`，串口能力只留
`dev-sim`。真机台架确认 CANable2（slcan-FD）是实际总线接口，默认链路需要 tty；
同时 ros2_control 关节清单在 URDF 手写，与 YAML 双写易漂移。

## Decision

- **接口自适应**：`BusConfig` 恢复 `transport`（缺省 `socketcan`）与 `tty`；
  新增 `CanTransportRegistry` 工厂注册，`with_builtin_transports()` 注册
  `socketcan` → `SocketCan` 与 `serial_slcan` → `SerialSlcanTransport`
  （POSIX termios，无新依赖，固定 S8/Y5）。`MotorDriver` 按 `bus.transport`
  创建传输，未知类型构造期 fail-fast。解析层校验：socketcan 必须带 `interface`、
  serial_slcan 必须带 `tty`、空 transport 非法。
- **默认 tty 双总线**：默认 `motor_canfd.yaml` 为 `left_arm → /dev/ttyACM0`、
  `right_arm → /dev/ttyACM1`（serial_slcan）；`bus_override` 按 transport 覆盖
  设备路径，**传输类型切换必须改 YAML**；`read_buses()` 按 transport 取 tty/interface。
- **协议与数量**：保持 `MotorProtocolRegistry`（YAML `type` 运行时查找，真机协议集
  不固定）；数量由 `DriverConfig` vector 决定，无固定假设。
- **URDF 清单自动生成**：`canfd_hardware.xacro` 新增 `emit_control_joints` 宏，
  `control_joints` 参数注入；`hardware.launch.py` 重构为 `OpaqueFunction`，运行时
  读取实际 `motor_config_file` 生成 controller joints 与 xacro 参数（覆盖路径一致）。
- **台架与文档**：恢复 `scripts/verify_2530_serial.py`（不安装）；全量更新文档，
  `AGENTS.md` 删除"只保留 SocketCAN"条款。

## Considered Options

- 仅文档化 `CanTransport` 抽象：接口选择仍硬编码，未闭环，否决。
- dev-sim 的 if/else 传输选择：扩展后端要改 `open()`，与协议层不对称，升级为工厂注册。
- `bus_override` 可切传输类型：覆盖参数只应改运行时设备路径，混入拓扑性质会掩盖
  配置错误，否决。
- 保留手写 ros2_control 清单：与 YAML 双写、易漂移，与"YAML 唯一拓扑源"矛盾，否决。

## Consequences

- 默认 serial_slcan 双总线一条命令启动；切 SocketCAN 需改 YAML 并配 `ip link`。
- 三自适应闭环（接口/协议/数量）；launch 复杂度上升，覆盖一致性由
  `test_hardware_launch.py` 与 `test_xacro_consistency.py` 兜底。
- ADR 0005 中"移除串口、BusConfig 只剩 name+interface_name"被取代，0005 保留为历史。
