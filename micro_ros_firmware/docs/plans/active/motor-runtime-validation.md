# 电机运行时架构验证计划

> **状态：活动。** 核心运行时已实现并通过 host 回归；以下条目是 ADR 0007 保持 Proposed 的真实开放项。已实现功能不再继续标作笼统 TBD。

## 已由代码与自动测试闭环

- [x] `MotorCommandArray` 对 NaN/Inf、未知 ID、重复 ID 整快照拒绝，拒绝时不刷新 freshness；
- [x] `MotorTopology`、`MotorEvent`、`MotorConfigResult` 消息及发布路径；
- [x] `effort` 是 100 Hz 命令中的唯一动态前馈量；
- [x] QoS 具备当前实现基线；
- [x] protection/limit 公共位定义；
- [x] 两协议 100 ms OFFLINE 判定；
- [x] FDCAN adapter 明确 FD/BRS/DLC，硬件过滤器采用接收标准 ID 后由软件路由；
- [x] 按 `model_name` 显式 Profile 绑定；
- [x] P0/P1/P2 静态队列、P1 latest-wins、每总线 RX ring 和单一 TX 所有权。

## 仍需设计确认或实测

### 协议与安全

- [ ] **TBD-SAFETY-01**：以厂家文档和无负载台架确认 CERB/VAXOR 30 ms 软停动作；
- [ ] **TBD-SAFETY-02**：确定 RX ring overflow 触发安全降级的阈值和恢复条件；
- [ ] **TBD-CONFIG-01**：确认厂家管理对象，特别是 CERB 设备侧限位；
- [ ] 设计并验证 FDCAN BUS_OFF 恢复策略，避免自动恢复掩盖持续物理故障；
- [ ] 建立上层型号库与固件 Profile 的版本/内容一致性机制。

### 容量与实时性

- [ ] **TBD-CAN-01**：在最坏反馈流量下验证 RX、P2、TX 配额与队列容量；
- [ ] **TBD-RT-01**：测量 `motor_rt_task` WCET、1 ms jitter、P0/P1 最坏发送延迟；
- [ ] 三路总线约 40 台混合电机的持续压力测试；
- [ ] 测量最大规模下 USB CDC 带宽和状态上行余量；
- [ ] 长时间热插拔、离线/恢复与冲突注入。

## 验收记录要求

每次台架记录必须包含固件提交、编译宏、时钟、总线位率、节点拓扑、运行时长、错误计数、队列峰值、WCET/jitter 和明确判据。硬件测试不得以软件停机替代急停、机械限位和供电隔离。

全部架构级条目闭环后，再更新 [ADR 0007](../../adr/0007-micro-ros-discovery-driven-motor-runtime.md) 状态；单次九电机 FDCAN1 通过不能代替三总线最大规模验收。
