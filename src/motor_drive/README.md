# motor_drive

C++17 电机驱动库（`dev-real` 真机线，无 ROS 依赖）：双传输后端
（`socketcan` / `serial_slcan`）、协议策略注册、动态电机映射。
内置 RP 与 `vaxor` 两套协议。

## 职责

负责：任意数量总线/电机的配置驱动管理、按 `(bus, type)` 分组调用 `MotorProtocol`、
按 `bus.transport` 创建 `CanTransport`、使能/失能/命令/反馈。

不负责：YAML/ROS 参数解析（由 `motor_hardware_interface` 完成）、ROS 2 生命周期
与 Topic、机器人拓扑与 controller 行为。

## 核心模型

```text
DriverConfig (buses[]/motors[]) -> MotorDriver
  -> 按 (bus, type) 分组 -> MotorProtocol
  -> 按 bus.transport 创建 -> CanTransportRegistry -> CanTransport
```

| 组件 | 职责 |
|---|---|
| `MotorDriver` | 配置校验、总线生命周期、分组、统一收发 |
| `MotorProtocol` / `MotorProtocolRegistry` | 电机家族协议策略与工厂注册 |
| `CanTransport` / `CanTransportRegistry` | 传输抽象与工厂注册（`send`/`receive`） |
| `SocketCan` / `SerialSlcanTransport` | Linux SocketCAN / CANable2 串口实现 |
| `protocol.*` | 物理量与线协议字段转换 |

`BusConfig`：`name` + `interface_name`（socketcan）+ `transport` + `tty`
（serial_slcan）。`serial_slcan` 固定 `C->S8(仲裁 1M)->Y5(数据 5M)->O`，POSIX
termios，无额外依赖。RP 单总线最多 8 台（广播槽位限制，仅 RP 策略校验）。

## 公共接口

`include/motor_drive/`：`motor_types.hpp`、`motor_protocol.hpp`、`motor_driver.hpp`、
`can_transport.hpp`（含 `CanTransportRegistry`）、`protocol.hpp`、`socketcan.hpp`、
`slcan.hpp`、`serial_slcan_transport.hpp`。

## 扩展

- **新增协议**：实现 `MotorProtocol` → 注册 `MotorProtocolRegistry` →
  `motor.yaml`（型号参数库）声明型号与参数 → 补测试。
- **新增传输**：实现 `CanTransport` → 注册 `CanTransportRegistry`
  （`std::unique_ptr<CanTransport>(const BusConfig&)`）→ YAML `transport` 声明
  → 补测试。

不在通用收发循环或 ROS 关节映射中加类型分支。

## 台架与手册

- `scripts/verify_2530_serial.py`：CANable2 串口验证 25/30 mm 电机（需 pyserial，
  源码树脚本不安装）。
- `docs/`：RP 系列与 16–30 mm 系列电机资料
  （[`rp-series.md`](docs/rp-series.md)、
  [`motor-2530-series.md`](docs/motor-2530-series.md)）及五份协议/规格 PDF。

## 测试

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select motor_drive
colcon test --packages-select motor_drive && colcon test-result --verbose
```

`test_protocol`（字段编解码）、`test_motor_protocol`（策略帧生成/解码）、
`test_driver`（校验/分组/传输注册）、`test_slcan`（slcan 方言编解码）。
