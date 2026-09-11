# 工作交接报告：RP 清除 + FDCAN1 九电机台架（2026-09-11）

> 分支 `dev/robot_ws`。本报告覆盖当天两块工作：①全仓清除已弃用的 RP 协议；
> ②在台架上用 CANable2 模拟 9 台电机验证固件全通路（进行中，卡在物理层）。
> 相关文档：[九电机目标](2026-09-11-fdcan1-bench-9motors-goal.md)、
> [第一轮运行记录/阻塞项](2026-09-11-fdcan1-bench-run1-findings.md)。

## 一、已完成：全仓移除 RP 协议

固件只保留 `vaxor`、`creb` 两个协议族（固件代码本就无 RP，仅 `rpm` 子串）。
上层 ROS 三包清理并保持六包 `colcon build` + `colcon test` 全绿：

- **motor_drive**：删 `pack/unpack_rp_slot/state`、`kRp*` 常量、Python 探针 RP 段、
  RP 测试用例；删除 `rp-series.md` 与两份 RP 厂商 PDF。
- **motor_sim**：删 `RpProtocol` 类与编解码（工厂本就未注册）、`rp_feedback_hz`；
  `frame_router`/node 的 RP 分支收敛；关键安全回归（使能/超时刹车/锁存后失能/
  重使能周期）从 RP 帧**改写为双 Vaxor 轴**，覆盖不丢；fixtures 改 vaxor。
- **motor_hardware_interface**：型号库删 rp-40c/50l/50h/70l/70h/90l；
  生产拓扑 8 台肩肘关节 `rp-50h → creb_r58_101` 并补 CrebProtocol 必填的
  `mit_position/velocity/torque_limit`（**示例值，真机前须按电机参数页核对**）；
  10 个测试 fixture 改 vaxor；URDF 注释与 README 同步。
- 顶层 `CLAUDE.md`/`README.md`/`AGENTS.md` 协议清单更新为 vaxor/creb。
- 遗留（用户说先不管）：`docs/research/mit-controller-tuning.md` 整篇 RP 整定调研仍在；
  ADR 0002/0005 各有 1 行 RP 历史叙述（按档案不可变惯例保留）。

## 二、进行中：FDCAN1 九电机全通路台架

### 环境（已实测确认）

- ST-Link/V2 SWD（OpenOCD 可读寄存器、`reset run`）；ttyACM0=板子 micro-ROS CDC；
  ttyACM1=CANable2（slcan 固件 `16e7497-dirty`）→ 控制板 **FDCAN1(PD0/PD1)**，
  两端各称有 120Ω。FDCAN2/3 悬空（上电必 bus-off，属预期）。
- 位率：仲裁 1M / 数据 5M。
- 9 台 ID 连续 1–9：**CREB 1..6、VAXOR 7..9**（固件注册表 motor_id=厂家节点号、
  不含协议族，同号跨协议会 CONFIG_CONFLICT，故 vaxor 不能用 1..3）。

### 已交付/已验证

- 主机模拟器 [`micro_ros_firmware/tools/fdcan_motor_sim.py`](../micro_ros_firmware/tools/fdcan_motor_sim.py)：
  严格按固件 codec/adapter 字节布局；**SYNC 门控**（开通道后静默，收到板子
  0x080 SYNC 才发帧，避免无人 ACK 把 CANable 自己打进 bus-off）；SYNC 对齐错峰；
  支持 `--no-brs`、`--fb-period-ms`、JSONL 抓包日志。
- 观测链打通：micro-ROS agent（ttyACM0）+ `ros2 topic`；固件话题
  motor_topology/motion_states/motor_status_array/motor_events/motor_config_result、
  motor_bind、/motor_command_array 均可见。
- **协议层正确**：9 台全部被发现，topology 中 1–6=CREB、7–9=VAXOR、`conflict=false`、
  bus_index=0，events 出 9×DISCOVERED；经典帧 1M 持续稳定；单电机 FD 帧
  （1M 无 BRS / 5M BRS）可长期 online。

### 两个关键发现

1. **数据位率索引**：本机 CANable 固件 **Y7 才是稳定 5Mbps**；仓库 `motor_drive`
   serial_slcan 写死的 **Y5 在该固件上临界丢帧**。真机/其他 CANable 接入前需核对，
   建议把数据位率做成可配置。
2. **物理层 blocker（当前卡点）**：9 节点持续 FD 流量时板子 FDCAN1 秒级 bus-off
   （TEC≈248、REC≈52–61、bit/stuff error）。软件侧已用对照实验排除
   突发密度/帧周期/BRS/SYNC 对齐；规律是「错误随总线占用增加」，低负载侥幸通过。
   **需现场检查**：①CANH–CANL 断电应量到 ≈60Ω（CANable2 板载 120Ω 常需焊跳线/拨码
   才接通，单边终端的表现与现象吻合）；②隔离收发器共地；③接线到 FDCAN1 收发器、
   接触与线长。判据：9 节点 topology 连续 ≥30s online 且 SWD 读 `PSR.BO=0`、
   TEC/REC 不增长。

## 三、恢复现场后如何继续（goal 剩余验收项）

固定开场时序（顺序不可乱）：
1. 起 agent：`micro_ros_agent/` 下 `source install/setup.bash &&
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0`；
2. 起模拟器（自动 S8/**Y7**/O 开门、静默等 SYNC）：
   `python3 micro_ros_firmware/tools/fdcan_motor_sim.py --log /tmp/sim.jsonl`；
3. OpenOCD `reset run`（或按复位键）——必须在 CANable 开门之后。

物理层排除后，依次验证目标文档清单中尚未做的项：
- 向 `micro_ros/motor_bind` 发 9 条 MotorBind（creb→`creb_r58_101`，
  vaxor→`vaxor_2550`），等 PROFILE_BOUND；否则未绑定时话题运动量为 NaN。
- `motion_states` 稳定 ~100Hz、9 路、数值与模拟器设定在量化误差内、state_age 新鲜；
  `motor_status_array` 无 fault/conflict。
- 下行闭环：经 `/motor_command_array`（或使能通道）下发使能/目标，模拟器收到对应
  CAN 帧（CREB 0x110+id、VAXOR 0x501/0x502+4id），随后该路状态变化、lifecycle 进 ACTIVE/READY。

## 四、其他注意

- 固件 bus-off 只计数不自动恢复（T12 待办），每次 bus-off 后必须复位板子。
- 生产 `motor_driver.yaml` 已改为 creb/vaxor 型号，但**双总线拓扑仍是旧示例**
  （且把 ttyACM0 当 slcan，与「ttyACM0=板子 CDC」冲突）；走 ROS 真机栈前需按实际重写。
- 本次未改固件已有代码（零固件改动）；如后续要 motor_id 支持协议族命名空间（方案 B）
  或 bus-off 自动恢复（T12），属独立任务。
- 533MB 厂商例程快照 `micro_ros_firmware/docs/dm-mc02例程/` 已加入 .gitignore，勿入库。

## 五、提交

- `8218ed1` 固件运行时在途改动 + 归档 superpowers 计划 + 协议 PDF（会话前在途工作）
- `e98f0b5` ROS 栈 RP 清除 + 九电机台架模拟器与文档
- 交接报告为本次追加提交
- 推送：远端 origin = https://github.com/lovely009/robot_ws.git，分支
  `dev/robot_ws`。本机缺 GitHub 凭据（HTTPS 无 token/helper，SSH key 未加入账户），
  需在有凭据的终端执行 `git push origin dev/robot_ws`。
