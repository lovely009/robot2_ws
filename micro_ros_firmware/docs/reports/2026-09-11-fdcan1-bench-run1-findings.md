# FDCAN1 九电机台架：第一轮运行记录与阻塞项（2026-09-11）

目标文档：[2026-09-11-fdcan1-bench-9motors-goal.md](2026-09-11-fdcan1-bench-9motors-goal.md)
工具：[`micro_ros_firmware/tools/fdcan_motor_sim.py`](../micro_ros_firmware/tools/fdcan_motor_sim.py)
（CANable2 ttyACM1 SLCAN 模拟 CREB 1-6 + VAXOR 7-9；micro-ROS agent 在 ttyACM0）

## 已验证通过

- ST-Link SWD、OpenOCD `reset run`、agent serial、SLCAN 通道与双层观测（slcan 抓包 + ROS 话题）全部可用。
- **经典帧 1Mbps 完全稳定**：板子 0x080 SYNC（10ms/0B）被动监听 6s 持续不丢、不 bus-off。
- **单电机 FD 帧长期稳定**：只发 creb id1 反馈（50Hz）时，无论 FD 无 BRS（全程 1M）
  还是 FD+BRS（5M 数据段），topology 持续 `online=true`、12s 以上不掉线；
  固件正确解出并注册（`protocol=CREB, bus_index=0`）。
- **九台全部被正确发现**：topology 出现 motor_id 1..9，1-6 `protocol=1(CREB)`、
  7-9 `protocol=0(VAXOR)`，`conflict=false`；events 出 9×DISCOVERED(0)。
  证明固件 motor_id/协议族归属/注册表/拓扑话题全链路正确，无 ID 冲突。

## 阻塞项：多节点持续流量下物理层位错误 → bus-off

九节点同时上报时 FDCAN1 秒级进入 bus-off（`CCCR.INIT=1`、TEC≈248、
REC≈52–61、PSR LEC=bit/stuff error、BO=EP=EW=1）。对照实验：

| 配置 | 负载 | 结果 |
|---|---|---|
| 纯监听（0 发送） | 无 | SYNC 100Hz ≥6s 稳定 |
| 单电机 12B 反馈 50Hz，无 BRS(1M) | 低 | ≥543 SYNC 稳定 |
| 单电机 12B 反馈 50Hz，BRS(5M, Y7) | 低 | online 稳定 ≥12s |
| 6×CREB 12B 错峰 | 中 | ~8s 后 bus-off |
| 9 节点 450fps / 180fps / SYNC 对齐 | 中高 | 秒级 bus-off |
| 9 节点 `--no-brs`（全程 1M） | 中 | 仍 bus-off |

软件侧已排除：突发密度（错峰 2.2ms）、帧周期（20/50ms）、BRS 与否、SYNC 仲裁对齐。
规律是「错误随累计帧/总线占用增加」，低负载侥幸通过——**物理层信号完整性问题**的典型表现。

## 需要现场检查（软件无法判定）

1. **终端电阻**：断电用万用表量 CANable 端 CANH–CANL，两终端各 120Ω 并联应读到 **≈60Ω**。
   CANable2 的 120Ω 很多版本要焊 0Ω 跳线/拨码才接通——「以为有」不等于接通；
   若只板子单侧一个 120Ω，低速率/低负载能通信、高速持续流量必出错，与现象吻合。
2. **共地**：CANable2 若用隔离收发器，CAN 地与控制板地不连会导致阈值漂移；确认两端共地。
3. **接线质量**：CANH/CANL 是否接到控制板 FDCAN1 收发器（PD0/PD1 对应那一路）、
   杜邦线接触、线长、与动力线并行走线。
4. 排除后重测：稳定判据为九节点 topology 连续 ≥30s `online=true` 且 SWD 读
   `PSR.BO=0`、`TEC/REC` 不持续增长。

## 另一个配置事实（非阻塞）

本机 CANable2 固件 `16e7497-dirty normaldotcom/canable2` 的数据位率索引 **Y7 才是稳定 5Mbps**；
仓库 `motor_drive` 的 serial_slcan 写死 **Y5**，在该固件上是临界近邻速率
（偶发成帧但丢帧致 OFFLINE）。真机/其他 CANable 接入前需核对 Y 索引，
必要时让数据位率可配置。

## 恢复方法

每次 bus-off 后固件不自动恢复（T12 待办）：先开 CANable 通道（模拟器自动 S8/Y7/O），
再 OpenOCD `reset run` 或按复位键。
