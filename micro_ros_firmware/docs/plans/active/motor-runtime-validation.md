# 电机运行时验证计划（开放项唯一清单）

> **状态：活动，2026-09-18 重构后更新。** 本文件是 ADR 0007 保持 Proposed 期间
> 唯一的开放项清单；原交接文档已归档（[HANDOFF](../archive/HANDOFF-motor-runtime-refactor.md)）。
>
> 现状基线（架构不在此重复）：host CTest 19 个用例全绿（18 个 C++ cassert +
> 1 个 python 线协议测试，`test/host/`）。任务结构、消息面（**3 订 6 发** + rclc
> 参数服务器：静态量在参数树、动态量走 `/micro_ros_do/motor_policy`）、构建与台架顺序以 [README](../../README.md)、[AGENTS.md](../../AGENTS.md)、
> [ADR 0007](../../adr/0007-micro-ros-discovery-driven-motor-runtime.md) 2026-09-18 勘误和
> [决策日志](../architecture/motor-runtime-decision-log.md)为准。
> 标「host 已覆盖」表示逻辑有自动测试、仅缺真机证据；其余 `[ ]` 均需台架或设计动作。

## 1. 实时性与总线容量

- [ ] **TBD-RT-01**：实测 `motor_rt_task` WCET、1 ms 周期 jitter、P0/P1 最坏发送延迟。
  含 **TBD-RT-02**：轨迹步进（每边界 O(电机数×路点数) 线性扫描）与 mailbox 轨迹
  校验（O(路点数×电机数)，在临界区内）的最坏耗时——台架前为静态上界估算。
- [ ] **TBD-CAN-01**：最坏反馈流量下 RX ring / P2 配额 / TX 队列容量，记录 overflow、backlog、队列峰值、总线负载。
- [ ] 三总线约 40 台 Vaxor/CERB 混合持续压测；测量 USB CDC 状态上行带宽余量。
- [ ] 故障注入下 P0 不静默丢失：CAN 无 ACK、RX burst、下游硬件拒收（重入队/抢占逻辑 host 已覆盖）。
- [ ] 长时间热插拔、离线/恢复循环稳定性。
- [ ] **TBD-STORAGE-01（v3 双扇区擦写阻塞）**：save 从擦 1 个扇区变擦 **2** 个，
  阻塞时间约翻倍。README/AGENTS 里的 "~1s" 是 v2 单扇区**旧值**，**v3 双扇区实测值
  待台架**（与"擦写中掉电"同轮记录）。擦写只在 storage 任务、IWDG 由 motor_rt 独立喂。

## 2. 安全语义真机确认

- [ ] **TBD-SAFETY-01**：以厂家文档和无负载台架确认两协议 30 ms 软停实际动作。缺席软停、30 ms/100 ms 时基与单电机隔离语义 host 已覆盖。
- [ ] **TBD-SAFETY-02**：确定 RX ring overflow 触发安全降级的阈值与恢复条件。
- [x] Agent 断连立即失能清授权：2026-09-18 九节点台架实测——ACTIVE 中杀 agent，下一帧为失能帧（控制字 0x0c）、运动停止；重连后**不重发 SET_ENABLE**、仅新快照即恢复（会话电平纯派生）。双 AUTHORIZATION_LOST 事件语义 host 钉死。
- [ ] **策略通道真机**（新）：`/micro_ros_do/motor_policy` 的 enable/kp/kd 迁移后使能链路；**代数门**——故障后重发同一代数必须被拒（回 CONFIG_RESULT INVALID），代数前进才受理；稀疏语义（策略里没列出的电机保持原状）；断开重连后未重新武装不得自动复权。逻辑 host 已覆盖（`test_motor_control` 的策略用例 + `test_policy_mailbox`）。
- [x] **参数槽每槽实测成本**（2026-09-18 台架）：**328 B/槽**（对比结构估算 269 B，差值是分配器块开销）。
  `PARAM_BRIDGE_MAX_PARAMS = 208` 实测把堆吃到只剩 **1,776 B**，已降为 **176**（覆盖 41 台）→ `heap_free = 12,272 B`。
  **此后变更**：176 → 224（2026-09-19，保护开关可写使每台 4→5 项；同轮把 micro_ros
  32 KiB 任务栈改静态分配落 RAM_D1，`heap_free` 12,328 → 29,748 B）→ 256 → 416
  （**未经复核，超出 120 KiB 堆池**）→ **64**（2026-09-20 修正）。当前值以 `include/param_bridge.h` 为准。
- [ ] **堆高水位（残余）**：176 槽下 12,272 B 只是空载观测值；仍需 48 台发现/绑定 +
  长时间运行下的**稳态最低位**，以及最坏路径（并发 SDO 事务 + rosidl 字符串分配）的瞬态低点。
- [ ] **轨迹真机**：单/多关节轨迹期间插值位置与上位期望一致、P1 帧仍严格 50Hz；
  轨迹运行中继续发 100Hz 快照 → 轨迹机组不受影响、集合外组正常；轨迹中杀 agent
  → 立即失能、轨迹作废、不重放；播完后保持末点（**机构无抱闸**，急停与机械
  支撑须就绪）。仲裁与级联语义 host 已覆盖（`test_motor_control` 的 7 个轨迹用例）。
- [ ] **CERB 清错帧真机**（对新硬件参数表）：注入厂家故障 → FAULT →
  `system.clear_fault = <id>` 发 MIT 控制字 bit5 → 状态帧 `fault_active=false`
  → READY_DISABLED → 显式使能 + 新快照恢复。**未决**：bit5 是否要求驱动器处于
  失能态、是否清全部故障位、瞬时还是锁存——固件当前按「不请求使能」发送。
- [ ] **Vaxor 故障恢复**：确认 `system.clear_fault` 回 `UNSUPPORTED`，故障位随
  物理条件消失后自动收敛 READY_DISABLED。
- [ ] 快照拒绝真机注入：NaN/Inf、未知/重复/跨总线 ID、非法 mode（整快照拒绝且不刷新 freshness，host 已覆盖）；厂家故障锁存、心跳丢失 100 ms OFFLINE 与恢复路径（30/100ms 级联逻辑 host 覆盖，真机注入待做）。
- [ ] CERB 每总线 20 ms 单帧 SYNC、约 20 ms 状态上报、100 ms OFFLINE 边界实测（协议编解码 host 已覆盖）。
- [ ] **CERB 线圈温度真机**（2026-09-20 新增）：12B 反馈帧 `data[8..9]`（大端 int16，
  0.1 °C）已由固件解码进 `MotorStatusArray.temperature`，字节→°C 与故障帧照读的语义
  host 已覆盖；真机待确认冷态读数接近室温、带载升温方向正确。**过温保护未接**——
  `protection_flags` 的 bit1 仍无阈值来源（`config/motor.yaml` 型号库无温度阈值项），
  温度到判据之间还差一步。
- [x] **BUS_OFF 自清恢复**：HAL 错误状态回调清 `CCCR.INIT` 已实现，**2026-09-18 真机自恢复已实测通过**；持续物理故障下的反复 bus-off 计数随 TBD-CAN-01 故障注入观察。
- [x] **大局部变量的静态化审计**（2026-09-19，工具 `micro_ros_firmware/tools/stack_audit.py`）：
  用 `-fstack-usage` + `-fcallgraph-info` 产物按任务入口算最坏调用路径栈深
  （构建目录 `.pio/build/stm32h723vgt6`，插桩 flag 用完已撤）。**最坏路径 / 声明栈**：
  `start_task` 384/1024（37.5%）、`sensor` 532/2048（26.0%）、`motor_rt` 680/4096（16.6%）、
  `led` 112/1024（10.9%）、`storage` 328/4096（8.0%）——**没有一处在小栈上放大帧**。
  对已知案例的回归验证：`storage_task_fn` 帧 **16 B**（`views` 已落在 `.bss`，
  `objdump` 的 `stmdb sp!,{r3,r7,fp,lr}` 与 `.su` 一致），即 2026-09-18 的修复在产物里可见。
  **纠正本条此前的两处猜测**：①`motor_rt_task.cpp` 的 `topo_in[64]`（帧 1,312 B）
  虽然**写在** motor_rt_task.cpp，但**不在 motor_rt 栈上**——它只被 micro_ros 侧的
  `publish_topology` 调用；②`param_bridge.cpp` 的 `[64]` 缓冲其实是
  `static MotorSlot g_motors[]`（`.bss`），该文件全部帧 ≤ 144 B。
  **micro_ros 的逐任务数字（656 B / 2.0%）不可信**：ROS executor 经函数指针回调，
  静态图走不到，补上回调起点后最坏是 `bridge_timer_cb` **4,436 B**
  （`publish_topology` 2,848 + `motor_rt_copy_topology` 1,312），仍在 32 KB 内但余量是
  按 4.4 KB 而非 0.66 KB 算。**未覆盖**：间接调用（adapter 虚表 / FreeRTOS 钩子）、
  `.su` 不含调用约定开销（单帧经验 +8~16 B）、ISR/MSP 栈（`HAL_FDCAN_RxFifo0Callback`
  帧 152 B，量级无虞但未正式审计）。栈溢出钩子是**关中断死循环**，表现为 IWDG 复位、
  不会给出任何显式线索。
- [ ] BMI088 断线 / 阻塞 SPI / 停 DRDY：传输超时、降级重试、heater off、双缓冲不撕裂。

## 3. 参数链与 Profile

- [x] **保护开关可写**（2026-09-19）：`0x2650:00` 新增写事务（SDO 写 + 读回核验，
  不跟 `0x2539`），参数项 `m<id>.limit_protect`（正向语义：1 = 保护生效），
  仅未运动时受理，写后记录自动同步。真机实测：`m2.limit_protect=0` 下发 →
  总线 `23 50 26 00 00 …` → 读回 `43 50 26 00 00 …`（`0` = 开启）→
  `CONFIG_RESULT reason=0`，四台仍 `conflict: false`。
  **对象语义（owner 手工核实 + SLCAN 复现）**：读 `40 50 26 00 00 …` → 反馈
  `43 50 26 00 0x …`（`0x` = 当前状态）；`23 50 26 00 00 …` = 开启、
  `23 50 26 00 01 …` = 关闭。
  同日补两项台架实测：**①开保护的自锁风险——假设未成立**（停发全部帧静置 5 秒，
  反馈仍上报、故障位 0、SDO 仍应答；单机空载静态，带载未验）；**②掉电不保存**
  （单变量对照：唯一被写过的那台回退，未碰的三台不变）。`0x2539` 保存码对它
  **无效**——官方 §4.11 明确码 `1` 只保存 "PI 参数和最大电流"。**该设置易失，
  每次上电回到上电初值（台架四台均为 `1` = 关闭）；要"保护必须开着"必须每次
  上电显式发一次。**
  **遗留**：`0x2541` 方向仍在参数面之外（无参数项，写路径手册未给）。
- [~] **TBD-CONFIG-01（台架实测部分）**：**2026-09-19 真机四台已完成"判未初始化"半边**——设备侧回读值逐台拿到（`0x607D:01/:02` = ∓32768 cnt、`0x6080` = 3325 RPM、`0x2538` = 10000 mA（R58）/ 5000 mA（R48）、`0x2650:00` = 1、`0x2541:00` = 0），`EVENT_DEV_INIT` reason=3 且位图 = **12**（槽 2 最大转速 | 槽 3 峰值电流）——W25 记录还是 9-18 模拟器基线（600 RPM / 1000 mA），属**记录陈旧**而非设备异常。**对齐已按 owner 决定结案**（同日）：以设备为准重新基线（`refresh ×4` → `system.save` → 复位），复位后四台 `conflict: false`、`lifecycle 3`、`last_error 0`、`EVENT_DEV_INIT reason=2`。**剩下**：回读失败报 41 不拦的注入复现。原条目：逐台确认上电回读四对象值；故意改一台设备参数 → 判未初始化（CONFIG_CONFLICT，`last_error=40`，拒绝使能）→ 写回恢复闭环；回读失败报 41 不拦。boot 比对/写回/恢复事务 host 已覆盖（决策日志 §6.3）。
- [ ] REFRESH_PARAMS 按总线一问一答、四个 SDO upload 回读：**2026-09-18 模拟器台架已验证字节序列**（607D:01/02→6080→2538 顺序/值正确）；`system.save` W25Q64 双 sector 掉电保持模拟器阶段已验证（SWD 复位），真机驱动器的 0x2539 保存闭环仍待 dev-real。
- [ ] **TBD-PROFILE-02**：CERB 规格书 Kt 三套口径互不一致，反馈扭矩估计仍依赖 Kt 链（真实值若仅名义一半，上层力矩偏大 1 倍）。判别实验：静止重力载荷同时读字典 `0x6077`（力矩传感器，0.01 N·m）与 `0x6078`（实际电流，mA）算 `Kt_eff = τ/(Iq×gear)`——驱动器自带传感器，不必外接；或 MIT 已知前馈扭矩最小二乘拟合。
- [ ] Vaxor 80 速比型号无完整规格，型号目录刻意不收；规格到位并补 Profile 前不得开放控制。
- [ ] 建立上层型号库（motor.yaml）与固件内置 Profile 的版本/内容一致性机制。
- [x] **TBD-CONFIG-02：`0x2650` 的读子索引 = `0x00`**（2026-09-19 真机四台实测结案）。
  读 `:01` / `:02` 一律 ABORT `0x06090011`（子索引不存在），只有 `:00` 应答；回读值 **1**
  （手册语义 = 看门狗 + 限位保护**关**）。手册写命令 `23 50 26 00 01 …` 里子索引本就是
  `0x00`，早期实现把数据字节当成了子索引。**修前后果正是本条预警的**：REFRESH 恒挂在
  安全槽 → UNCONFIRMED → 三次退避后 UNVERIFIED，全部 CERB 到不了 `DEV_INIT_OK`、
  拒绝使能（`last_error = 41`）。已改 `CERB_SUB_SAFETY_PROTECT` 并同步模拟器与 host 测试
  的镜像假设；烧板复测 `last_error` 41 → 40、`EVENT_DEV_INIT` reason 4 → 3 且位图点名
  真实不一致槽位。**模拟器与 host 测试此前照固件假设建模（而非照对端），结构上不可能
  发现此错**——新增只读探针 `micro_ros_agent/tools/cerb_sdo_probe.py` 直接问真机。
  见[台架记录](../../../docs/reports/2026-09-19-cerb-real-hardware-bench.md)。
- [ ] **TBD-STORAGE-02（升级迁移兜底）**：旧固件上 save **奇数次**使活动 blob 落在
  `0x1000`，再烧新固件，确认参数被一次性兜底读回而**不被清零**（不是从零开始）。
  同时确认已知缺口：旧槽 A 有效 + 旧槽 B 更新时会**静默回退一代**（文档记为不修）。
  **2026-09-19 尝试未果（不是通过）**：在本板上按「烧旧固件（`8d60839` worktree）→ save → 烧新固件」
  跑过两轮，新固件始终走的是**遗留的 v3 槽 B(`0x2000`) blob**，旧 v2 blob 被 seq 仲裁淘汰。
  根因是台架状态不可达：本板此前已由 v3 固件写过槽 B 的合法 blob，`ok_b` 恒真 → 兜底（要求两槽皆假）
  与 v2 迁移（需 v2 槽仲裁胜出）**都复现不出来**。要真正验证必须先**擦净 W25Q64 前 16 KB**
  （专用擦除固件或 QSPI 工具），或**在槽 B 写中途掉电**把 `0x2000` 打成无效。
  已确证的一半：旧固件自身的 v2 blob 在真 flash 上**可写可读**（设 `m6.vel_limit=12.5` → save →
  重启后 m6 因「记录 12.5 ≠ 设备默认 600 RPM」单独报 `CONFIG_CONFLICT` / `last_error 40`）。
- [ ] **TBD-STORAGE-03（RAM_D1 基线）**：blob v3 落地后 `pio run` 实测
  **RAM_D1 = 225,108 B / 327,680 B（68.7%）**（v2 前 220,308 B；净增 +4800 B RAM /
  +360 B Flash）。PlatformIO 的百分比分母是整芯片 577,536 B，判余量要用 RAM_D1 口径。

## 4. 上层迁移（ROS 2 侧，未动工）

- [ ] `motor_hardware_interface` 迁移：维护 `joint_name ↔ motor_id`、方向、零位与关节语义；发 `MotorCommandArray` + `/micro_ros_do/motor_policy`、收 `MotorMotionStateArray`；静态量经 rclc 参数服务器对接。
- [ ] 消息定义改动先改权威包 `micro_ros_agent/src/can_bridge_msgs` 再同步 ROS 工作区；固件不得恢复 name-to-ID 兼容层。
- [~] 规模约束：**2026-09-18 的「已解」被推翻**——128 → 208、每电机 4 项时看似覆盖 48 台，
  但台架实测 208 槽只剩 1,776 B 堆余量；随后每台项数因保护开关变 5 项，
  `PARAM_BRIDGE_MAX_PARAMS` 又经历 176 → 224 → 256 → 416（未经复核、超出堆池）→ **64**（2026-09-20）。
  **结论**：rclc 参数槽按 328 B/槽 × max_params 常驻、**与在线上电机数无关**，48 台规模下
  应把电机参数整体撤出参数树（配置改走块话题），参数树只留 `system.*`。
  当前值与推导以 `include/param_bridge.h` 为唯一来源；方案见
  [48 台规模设计](48-motor-scale-design.md)。**堆高水位仍待真机实测**（见第 2 节）。

## 近期已结案（不再追踪）

- [x] 参数读写全链路、SDO 0x1008 读身份自动绑定（bind FIFO 已退役）、设零掉电持久化（2026-09-17 真机结案，§6.7）、DEV_INIT 不一致槽位位图事件、storage 任务接线、1 订 4 发消息面。
- [x] 下行多点轨迹通道（`MotorTrajectory` + 归属仲裁 + 段内插值 + 通道级 freshness，2026-09-18）：消息、mailbox 通道、内核执行与桥接线完成，host 覆盖，固件 `pio run` 通过。真机项见第 1/2 节。
- [x] 逐电机策略通道 `/micro_ros_do/motor_policy`（2026-09-18）：独立 mailbox + 内核批量落地 + 防复权代数门；参数面退役 enable/kp/kd 三项。host 覆盖，`pio run` 通过。
- [x] **storage 任务栈溢出**（既有缺陷，2026-09-18 台架发现并修复）：`perform_save()` 的
  `motor_control_param_view_t views[64]`（5,120 B）放在 4,096 B 栈上，每次 W25 落盘必然溢出 →
  FreeRTOS 栈溢出钩子关中断死循环 → IWDG ~500ms 复位。表观现象是「一落盘就重启」，
  影响 `system.save`/设备参数写回/设零/清错全部落盘路径。改为 static 后修复并实测通过。
  详见 [台架记录](../../../docs/reports/2026-09-18-control-surface-bench.md)。
- [x] 上行可观测性（2026-09-18）：`trajectory_state` 与 `diagnostics` 两条新话题、`target_source`、`event_seq`、`schema_version`、轨迹拒绝事件带 id 与路点下标。host 覆盖，`pio run` 通过。
- [x] 故障清除入口 `system.clear_fault`（参数脉冲，2026-09-18）：内核按成因三分支 + CERB 清错帧编码完成，host 覆盖；真机项见第 2 节。
- [x] CERB 管理对象确认与快速 SDO 实现（2026-09-16，TBD-CONFIG-01 对象部分）；台架实测部分仍开放，见第 3 节。
- [x] MotorConfigResult 消息已删除，配置结果走 `motor_events` 内核事件与参数错误码（ADR 勘误），本条遗迹销账。

## 验收记录要求

每次台架必须记录：固件 commit、编译宏、时钟与总线位率、节点拓扑（总线/ID/型号）、
运行时长、错误计数（ring drop / 事件丢弃 / last_error）、队列峰值、WCET/jitter 和明确通过判据。
硬件测试不得以软件停机替代急停、机械限位与供电隔离。全部架构级条目闭环后再将
ADR 0007 转 accepted；单次九电机 FDCAN1 通过不代表三总线 40 台最大规模验收。
