# USB 与 USB-CANFD 接线指南

仿真主机始终通过三个 USB 串口设备分别承载 `left_arm`、`right_arm`、`legs`。
启动参数必须使用 `/dev/serial/by-id/...`，不要使用可能随插拔变化的 `/dev/ttyUSB0`。

## 模式一：USB CDC/UART 直连

适用于 3588 与仿真主机直接交换 ASCII 帧记录。每条逻辑总线需要一个独立的 USB
CDC/UART 通道；两端可以通过 USB gadget、USB-UART 或等价串口链路连接。

```text
3588/外部控制器  ── USB CDC/UART ──  仿真主机 /dev/serial/by-id/...
```

使用 `usb_sim.launch.py`。该模式没有物理 CAN-FD：不会模拟仲裁、CANH/CANL 电平、
终端电阻、误码、重发、总线负载或线缆故障。它只验证帧格式、控制逻辑和 Gazebo 行为。

## 模式二：USB-CANFD 物理链路

3588 两端均使用适配器时：

```text
3588 ─ USB-CANFD ─ CANH/CANL/GND ─ USB-CANFD ─ 仿真主机 USB
```

带 CAN-FD 外设的 STM32 接入仿真主机时：

```text
STM32 CAN-FD ─ CANH/CANL/GND ─ USB-CANFD ─ 仿真主机 USB
```

使用 `canfd_sim.launch.py allow_external_canfd:=true`。仿真端适配器必须支持本工程使用的
SLCAN-FD 命令和 CAN-FD BRS；节点会依次发送：

```text
C\rS8\rY5\rO\r
```

本工程按仲裁段 1 Mbit/s、数据段 5 Mbit/s 配置。外部适配器或 MCU 必须使用相同位率。

### CAN-FD 布线要求

- 所有节点同名连接：`CANH` 对 `CANH`、`CANL` 对 `CANL`，并连接参考 `GND`。
- 总线两端各放置一个 120 Ω 终端电阻；断电测量 CANH 与 CANL 之间通常应约为 60 Ω。
- 使用双绞线，主干尽量短，支线尽量短；不要星形连接。
- 确认适配器和收发器支持 CAN-FD 以及 5 Mbit/s 数据段，经典 CAN 设备不等价。
- 三条逻辑总线物理隔离时，需要三套仿真端 USB-CANFD 适配器。

## ID 与接入检查

同一物理总线上，协议类型与电机 ID 的组合必须唯一。特别是 `legs` 总线同时容纳左右腿，
RP 使用 ID 1–8，Vaxor 使用 ID 1–4；不要再接入使用相同协议和 ID 的第二个控制器。
仿真节点会检测短时间内内容不同的重复命令并锁存全机零输出，但这不能替代正确布线。

接线后按以下顺序检查：

1. 用 `ls -l /dev/serial/by-id/` 确认三个稳定设备路径；
2. 核对三条路径互不重复，且与 left/right/legs 标签一致；
3. CAN-FD 模式断电检查终端电阻和 CANH/CANL/GND；
4. 先启动 Gazebo，再给单个电机显式 enable 和小幅命令；
5. 逐轴确认 ID、方向、限位、反馈频率和断线停机。
