# 台架目标：FDCAN1 模拟 9 电机全通路验证（2026-09-11）

## 目标

主机经 CANable2（ttyACM1，SLCAN）在控制板 **FDCAN1** 上模拟 **9 台电机**，
验证「主机模拟器 ⇄ 固件 motor runtime ⇄ micro-ROS agent ⇄ ROS 2 话题」全链路：
固件正确发现 9 台、归属正确协议族、周期反馈正确上行、命令/使能正确下行，
各级 ROS 2 话题内容与频率正确。总线上无真机。

## D0 决策点（已定案 2026-09-11）

固件注册表键 `motor_id = 厂家节点号`，**不含协议族**，同节点号跨协议即
CONFIG_CONFLICT 锁存。已定：9 台 ID 连续 **1–9**——

- **CREB 节点 1–6；VAXOR 节点 7、8、9（方案 A，零固件改动）。**

（备选方案 B：motor_id 改为「协议族 + 节点号」复合键，需同步
discovery / can_bridge_msgs / 上层映射，不在本目标范围。）

## 台架与固定开场时序

- ttyACM0 = 板子 USB CDC（micro-ROS serial）；ttyACM1 = CANable2 → FDCAN1(PD0/PD1)。
- 位率：仲裁 1M / 数据 5M（SLCAN `S8`/`Y5`），FD+BRS；两端各 120Ω。
- CANable 必须先 `O` 开通道后再复位板子（否则 FDCAN1 无 ACK 节点，启动即 bus-off
  锁死；固件 bus-off 只计数不自动恢复）。
- 顺序：
  1. 启动 agent：`micro_ros_agent/` 下 `source install/setup.bash` 后
     `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0`；
  2. 主机模拟器打开 ttyACM1，发 `C\r S8\r Y5\r O\r`（命令间隔 ≥150ms）成为 ACK 节点；
  3. 复位控制板（用户手动或经授权的 ST-Link reset）；
  4. 模拟器立即开始周期发送 9 台反馈（发现为被动注册，靠反馈帧建档）。
- FDCAN2/3 悬空必然 bus-off，忽略，不作为失败项。

## 模拟器需求（主机侧新建工具）

- 语言/形态自定（建议 Python 直连 ttyACM1，复用
  `ros2_ws/src/motor_drive/scripts/motor_probe.py` 的 SLCAN 组帧）；
  不依赖 ROS 栈、不占用 ttyACM0。
- 9 个节点（已定）：CREB id=1..6；VAXOR id=7..9。
- 上行（模拟器→固件）：
  - CREB：反馈帧 ID `0x300+id`，12B，大端：位置(182 count/°)/速度(rpm)/Iq(mA)，
    内容用可辨识的设定值（如各台位置不同，便于逐台核对）；周期建议 20–50ms。
  - VAXOR：状态帧 ID `0x702+4*id`（20B）周期发送；另在收到 MIT 命令帧
    `0x501+4*id` 时立即回应答帧 `0x701+4*id`（12B，手册要求逐帧应答）。
  - 帧格式严格按固件 adapter 的 `match_rx_frame`/decode 实现对齐字节布局，
    先读 `lib/motor_protocol/src/creb_adapter.c`、`vaxor_adapter.c`。
- 下行（固件→模拟器）：记录并解码使能/失能/命令帧（CREB `0x110+id` 9B MIT、
  VAXOR 控制帧 `0x502+4*id` 与 MIT 帧），供命令通路核对；模拟器按收到的目标值
  轻微调整后续反馈（闭环不要求真实，能体现「命令影响反馈」即可）。
- 模拟器自带控制台/日志：每节点 TX/RX 计数、最后收到的命令。

## 验收标准

### CAN 线协议层（ttyACM1 抓包）

- [ ] 9 组反馈帧持续上线，ID 与 DLC/FD/BRS 符合两协议定义；
- [ ] 能抓到固件下行的使能/命令帧且 ID、字节布局正确；
- [ ] VAXOR 每帧 MIT 命令后固件收到模拟器应答（一问一答成立）。

### ROS 2 话题层（agent + `ros2 topic`，消息来自 can_bridge_msgs）

- [ ] `ros2 topic list` 出现固件全部话题；
- [ ] `micro_ros/motor_topology`（MotorTopology，发现窗口后发布）：
      **恰好 9 条 entries**；6 条 `protocol=PROTOCOL_CREB(1)` motor_id 1..6、
      3 条 `protocol=PROTOCOL_VAXOR(0)` motor_id 7..9；
      全部 `online=true`、`conflict=false`；bus_index=0；lifecycle 正常推进；
- [ ] `micro_ros/motion_states`（MotorMotionStateArray）：稳定 ~100Hz，
      每次含 9 路 state，motor_id 与拓扑一致，position/velocity 与模拟器
      设定值在量化误差内一致，`state_age_ms` 新鲜（非 0xFFFFFFFF）；
- [ ] `micro_ros/motor_status_array`：9 路，`unified_fault_code=0`，
      online=true，无 fault/conflict；
- [ ] `micro_ros/motor_events`：出现 9 次 DISCOVERED(0)；
      **无 CONFIG_CONFLICT(8)、无 REGISTRY_FULL(14)、无 FAULT/OFFLINE**；
- [ ] 下行：经 `/motor_command_array`（或固件识别/profile 流程所需的配置通道）
      对指定电机下发使能与目标值后，模拟器侧收到对应 CAN 帧，
      `motor_config_result`（如涉及）结果成功，随后 motion_states 该路数值变化、
      status lifecycle 进入 ACTIVE/READY 对应状态。

### 构建与基线

- [ ] 固件当前工作区源码 `pio run` 通过并已烧录（授权改固件/烧录已授予，
      但方案 A 预期零固件改动；如必须改动，先说明）；
- [ ] 不引入 rp 残留；改了固件代码则 host 测试
      （`cmake -S test/host … && ctest`）与 `pio run` 均通过。

## 交付物

1. 主机模拟器脚本（建议放 `micro_ros_firmware/tools/` 或台架脚本目录，注明用法）；
2. 一次完整运行的记录：topology 快照、motion_states 频率统计（`ros2 topic hz`）、
   事件列表、一收一发对照（CAN 帧 hex ↔ 话题字段）；
3. 发现的问题清单（含固件/协议不符项），区分「模拟器问题」与「固件问题」。

## 非目标 / 安全

- 不接真实电机、不验证真实出力/标定；使能命令无机械风险但仍按最小步骤进行；
- 不做 FDCAN2/3、不做总线故障注入（bus-off 恢复 T12 不在本次范围）；
- 不改生产 `motor_driver.yaml`（ROS 真机栈本次不用）；
- 对硬件的写操作（SWD 复位/烧录）按既有授权范围，执行前说明。
