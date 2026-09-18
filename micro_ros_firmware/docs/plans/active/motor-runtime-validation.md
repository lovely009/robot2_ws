# 电机运行时验证计划（开放项唯一清单）

> **状态：活动，2026-09-18 重构后更新。** 本文件是 ADR 0007 保持 Proposed 期间
> 唯一的开放项清单；原交接文档已归档（[HANDOFF](../archive/HANDOFF-motor-runtime-refactor.md)）。
>
> 现状基线（架构不在此重复）：host CTest 17 个用例全绿（16 个 C++ cassert +
> 1 个 python 线协议测试，`test/host/`）。任务结构、消息面（1 订 4 发 + rclc
> 参数服务器）、构建与台架顺序以 [README](../../README.md)、[AGENTS.md](../../AGENTS.md)、
> [ADR 0007](../../adr/0007-micro-ros-discovery-driven-motor-runtime.md) 2026-09-18 勘误和
> [决策日志](../architecture/motor-runtime-decision-log.md)为准。
> 标「host 已覆盖」表示逻辑有自动测试、仅缺真机证据；其余 `[ ]` 均需台架或设计动作。

## 1. 实时性与总线容量

- [ ] **TBD-RT-01**：实测 `motor_rt_task` WCET、1 ms 周期 jitter、P0/P1 最坏发送延迟。
- [ ] **TBD-CAN-01**：最坏反馈流量下 RX ring / P2 配额 / TX 队列容量，记录 overflow、backlog、队列峰值、总线负载。
- [ ] 三总线约 40 台 Vaxor/CERB 混合持续压测；测量 USB CDC 状态上行带宽余量。
- [ ] 故障注入下 P0 不静默丢失：CAN 无 ACK、RX burst、下游硬件拒收（重入队/抢占逻辑 host 已覆盖）。
- [ ] 长时间热插拔、离线/恢复循环稳定性。

## 2. 安全语义真机确认

- [ ] **TBD-SAFETY-01**：以厂家文档和无负载台架确认两协议 30 ms 软停实际动作。缺席软停、30 ms/100 ms 时基与单电机隔离语义 host 已覆盖。
- [ ] **TBD-SAFETY-02**：确定 RX ring overflow 触发安全降级的阈值与恢复条件。
- [x] Agent 断连立即失能清授权：2026-09-18 九节点台架实测——ACTIVE 中杀 agent，下一帧为失能帧（控制字 0x0c）、运动停止；重连后**不重发 SET_ENABLE**、仅新快照即恢复（会话电平纯派生）。双 AUTHORIZATION_LOST 事件语义 host 钉死。
- [ ] 快照拒绝真机注入：NaN/Inf、未知/重复/跨总线 ID、非法 mode（整快照拒绝且不刷新 freshness，host 已覆盖）；厂家故障锁存、心跳丢失 100 ms OFFLINE 与恢复路径（30/100ms 级联逻辑 host 覆盖，真机注入待做）。
- [ ] CERB 每总线 20 ms 单帧 SYNC、约 20 ms 状态上报、100 ms OFFLINE 边界实测（协议编解码 host 已覆盖）。
- [x] **BUS_OFF 自清恢复**：HAL 错误状态回调清 `CCCR.INIT` 已实现，**2026-09-18 真机自恢复已实测通过**；持续物理故障下的反复 bus-off 计数随 TBD-CAN-01 故障注入观察。
- [ ] BMI088 断线 / 阻塞 SPI / 停 DRDY：传输超时、降级重试、heater off、双缓冲不撕裂。

## 3. 参数链与 Profile

- [ ] **TBD-CONFIG-01（台架实测部分）**：逐台确认上电回读四对象值；故意改一台设备参数 → 判未初始化（CONFIG_CONFLICT，`last_error=40`，拒绝使能）→ 写回恢复闭环；回读失败报 41 不拦。boot 比对/写回/恢复事务 host 已覆盖（决策日志 §6.3）。
- [ ] REFRESH_PARAMS 按总线一问一答、四个 SDO upload 回读：**2026-09-18 模拟器台架已验证字节序列**（607D:01/02→6080→2538 顺序/值正确）；`system.save` W25Q64 双 sector 掉电保持模拟器阶段已验证（SWD 复位），真机驱动器的 0x2539 保存闭环仍待 dev-real。
- [ ] **TBD-PROFILE-02**：CERB 规格书 Kt 三套口径互不一致，反馈扭矩估计仍依赖 Kt 链（真实值若仅名义一半，上层力矩偏大 1 倍）。判别实验：静止重力载荷同时读字典 `0x6077`（力矩传感器，0.01 N·m）与 `0x6078`（实际电流，mA）算 `Kt_eff = τ/(Iq×gear)`——驱动器自带传感器，不必外接；或 MIT 已知前馈扭矩最小二乘拟合。
- [ ] Vaxor 80 速比型号无完整规格，型号目录刻意不收；规格到位并补 Profile 前不得开放控制。
- [ ] 建立上层型号库（motor.yaml）与固件内置 Profile 的版本/内容一致性机制。

## 4. 上层迁移（ROS 2 侧，未动工）

- [ ] `motor_hardware_interface` 迁移：维护 `joint_name ↔ motor_id`、方向、零位与关节语义；发 `MotorCommandArray`、收 `MotorMotionStateArray`，参数经 rclc 参数服务器对接。
- [ ] 消息定义改动先改权威包 `micro_ros_agent/src/can_bridge_msgs` 再同步 ROS 工作区；固件不得恢复 name-to-ID 兼容层。
- [ ] 规模约束：rclc low-mem 参数槽 128 ≈ 17 台，40 台方案落地前须先评估参数面容量。

## 近期已结案（不再追踪）

- [x] 参数读写全链路、SDO 0x1008 读身份自动绑定（bind FIFO 已退役）、设零掉电持久化（2026-09-17 真机结案，§6.7）、DEV_INIT 不一致槽位位图事件、storage 任务接线、1 订 4 发消息面。
- [x] CERB 管理对象确认与快速 SDO 实现（2026-09-16，TBD-CONFIG-01 对象部分）；台架实测部分仍开放，见第 3 节。
- [x] MotorConfigResult 消息已删除，配置结果走 `motor_events` 内核事件与参数错误码（ADR 勘误），本条遗迹销账。

## 验收记录要求

每次台架必须记录：固件 commit、编译宏、时钟与总线位率、节点拓扑（总线/ID/型号）、
运行时长、错误计数（ring drop / 事件丢弃 / last_error）、队列峰值、WCET/jitter 和明确通过判据。
硬件测试不得以软件停机替代急停、机械限位与供电隔离。全部架构级条目闭环后再将
ADR 0007 转 accepted；单次九电机 FDCAN1 通过不代表三总线 40 台最大规模验收。
