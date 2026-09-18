# 固件高内聚重构 · 九节点台架回归报告

- **日期**：2026-09-18
- **固件 commit**：e49d8c2（含重构阶段 0–9、11；基线 720d523）
- **台架**：STM32H723VGT6 控制板（ST-LINK/V2 烧录 + USB CDC micro-ROS）+ CANable2 `b158aa7`（slcan S8/Y5/O）+ 主机九节点模拟器（CERB 1–6、VAXOR 7–9，无负载）
- **性质**：重构行为保持回归（非新功能验收）；真机驱动器的保存帧/设零回执不在模拟器范围。

## 重构内容（11 个提交）

host CMake OBJECT 库函数化；注释/文档事实修正；删死代码；容量跨库 static_assert、SDO 宏与 π 单点；板级库去 FreeRTOS 反向依赖、外设时钟单点、octospi 枚举自立；storage_projection 抽出 + 快照注入打破 storage↔motor_rt 环依赖；heater 模块；内核上下文不透明化 + copy_topology 公共 API（构建系统强制封装边界）；session_sync 会话授权每边界纯派生；CERB 配置事务 8 个 switch 收敛为静态阶段描述符表、refresh 收尾去魔数、BUSY 拒绝收 helper；文档单点真重组档（ARD/HANDOFF 归档、AGENTS 285→120 行）。

## 验证方法

- **自动化**：host CTest 17/17（编译固件真实 .c 无桩，含新增 storage_projection 6 例、heater_pid 4 例、session_sync 5 例、copy_topology、描述符表边界）；`pio run` 0 警告。
- **对抗评审**：CERB 描述符表经三路独立评审——差分预言机对新旧 adapter 接口做 19,060,192 次枚举调用，内核可达输入零差异（两处仅不可达直调边界语义变化，已补测试钉死）；refresh 位图判据与 BUSY helper 逐路径等价；完整性/边界无阻塞项。
- **台架**：九节点模拟器，先 CANable 成 ACK 节点后 SWD 复位；micro-ROS agent 接 CDC。

## 台架结果

### 基线功能（位率/时钟单点）

- 九节点 topology：id 1–9 全 online；型号 cerb_r48/r58_101 + vaxor_2550 自动绑定（SDO 0x1008）；协议映射 1=CERB/0=VAXOR 正确；能力位 CERB=9、VAXOR=49。
- motion_states ≈ 95–97 Hz；imu/data ≈ 94 Hz；imu/temp 2 Hz（SPI2 时钟单点正常）。
- m1–4 的 CONFIG_CONFLICT（fault 6/40）为 W25 旧记录与模拟器默认值的历史数据状态（模拟器对 0x2539 保存帧回 abort），非本次回归；m5–9 干净。

### 断连/授权（session_sync，阶段 8）

- 持续命令快照使 m5 进 ACTIVE（0x115 MIT 帧）→ `kill -9` micro-ros_agent：固件 XRCE ping 失败转 LOST，最后一帧控制字 0x0c（使能位清零，P0 失能），运动帧立即停止——断连立即撤权成立。
- 重启 agent，**不重发 SET_ENABLE**、只发新快照：m5 立即恢复运动帧——会话电平每控制边界纯派生、新快照自动重立授权成立。
- host 钉死断连边界产生两次 AUTHORIZATION_LOST（set_agent + set_auth 顺序保留）。

### 配置事务 golden（CERB 描述符表，阶段 9）

设备应答序列（固件请求字节由 host 黄金表 + 差分 oracle 锁定；模拟器只记录自身 tx）：

- REFRESH_PARAMS m5：`43 7d 60 01/02`（0x607D 位置 ±32768cnt）→ `43 80 60 00`（0x6080=600 RPM）→ `43 38 25 00`（0x2538=1000 mA），四阶段顺序与值符合规格。
- SET_TORQUE_LIMIT m6（2.0 A）：写应答 `60 38 25 00` → 读回 `43 38 25 00 d0 07 00 00`（2000 mA，与写值一致）→ 保存帧已发（模拟器回 `80 39 25 00` abort；真机驱动器才落盘）→ 写后全 CERB 自动重扫身份+四对象（设计行为）。
- SET_ZERO：固件发 0x2531 写帧（host 锁字节 `23 31 25 00 01 00 00 00`），模拟器不实现该 SDO 回执 → UNCONFIRMED 终态（口径正确）；真机回执与掉电保持已在 720d523 结案。
- BUS_OFF：停模拟器制造无 ACK → 重开通道，不复位板子节点自动恢复 online（自清 CCCR.INIT，阶段 4 实测）。

### 持久化（阶段 5 回归）

m5/m6 写限值后触发去抖落盘 + system.save，SWD 复位后记录保持、上电四对象回读比对放行（last_error=0；40 仅指向旧记录节点）。

### 长跑

- 窗口：05:24 起，≥10 分钟空闲九节点反馈（待填）。
- 反馈计数均衡性、错误计数、agent 无重连、频率漂移：待填。

## 未覆盖 / 开放

- heater PWM 占空比波形需示波器；WCET/jitter（TBD-RT-01）、三总线 40 台规模、P0 burst、真实负载惯量未覆盖（见 validation plan）。
- 0x2539 保存帧与 0x2531 设零回执需真机驱动器闭环（模拟器不实现）。
- 单次九节点模拟不构成 ADR 0007 架构级闭环，ADR 状态保持 Proposed。

## 结论

固件侧高内聚重构在九节点模拟台架上行为保持：断连撤权/快照恢复、配置事务字节序列、持久化、BUS_OFF 自恢复、时钟与位率全部符合重构前基线；结构目标（环依赖打破、封装边界构建强制、单一事实源、死代码清除）达成。真机驱动器专项（保存/设零/40 台规模）留待 dev-real。
