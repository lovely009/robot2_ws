# FDCAN1 四电机模拟台架指南

> **适用范围（2026-10-14 修订）**：STM32H723 控制板 FDCAN1 + 当前现场 CANable `16e7497-dirty` + 无真实电机的模拟台架。默认节点为 CERB 1=R58、2–4=R48；需要旧九节点台架时显式传入 `--cerb 1,2,3,4,5,6 --vaxor 7,8,9`。**两只已知 CANable 版本的 5 Mbps 索引都是 `Y5`**（2026-10-12 实测修正了 `16e7497-dirty`=Y7 的旧说法，见下）。**固件位时序是冻结的标定值：仲裁 1 Mbps / 数据 5 Mbps，板子侧没有台架变体**。历史上反复出现的"整链间歇静默"**不是板子的问题，是那只 SLCAN CANable 的数据段采样点配错**（88.24% vs 板子 81.25%）；换成可在主机侧精确设位时序的 gs_usb/SocketCAN 适配器即根除（见下"整链间歇静默"一节）。本文只定义稳定启动和只读观察流程；Profile 绑定、使能和运动命令必须另行确认总线无真实电机并取得外部操作权限。

## 连接与固定参数

| 项目 | 值 |
|---|---|
| 控制板 micro-ROS CDC | `/dev/ttyACM0` |
| CANable2 SLCAN | `/dev/ttyACM1` |
| 控制板总线 | FDCAN1，PD0/PD1 |
| 位率 | 仲裁 1 Mbps / 数据 5 Mbps（固件冻结标定值，板子侧无台架变体），CAN FD+BRS |
| SLCAN 配置 | `C` → `S8` → **`Y5`** → `M0` → `A1` → `O`（`16e7497-dirty` 与 `b158aa7` 均为 `Y5`），命令间隔至少 150 ms |
| 模拟节点 | CERB id 1=R58；id 2–4=R48 |

先用 `V` 识别 CANable 固件。**2026-10-12 实测修正**：现场这只 CANable（USB 描述符
`b158aa7`、`V` 响应 `16e7497-dirty`）的 5 Mbps 索引是 **`Y5`**，历史注释记的 `Y7`
是错的。判据（同硬件、同固件，只换索引做 A/B）：`Y7` → 九台全部 OFFLINE、fault=5，
固件 tx/rx 计数几十秒内冻结；`Y5` → 全部 READY_DISABLED、fault=0、online=true，
长期稳定。`Y7` 的失败指纹是 FDCAN1 `PSR.DLEC=3`（**数据相位** ACK error）——仲裁
相位对、数据相位位率/采样点不匹配。注意 `0x080` SYNC 是 DLC=0 帧、没有数据相位，
所以"能收到 SYNC"或"CAN 分析仪 0 错误"都**不能**证明数据位率正确，必须看带载荷的
FD 帧（DLC 8/12/20）是否畅通。索引与 CANable 健康自愈逻辑见
`micro_ros_agent/tools/fdcan_motor_sim.py` 的 `SLCAN_VERSION_DATA_BITRATE` 与
`_recover_latched_bus`。

## 整链间歇静默：适配器是总线上一个采样点配错的节点（2026-10-15 寄存器定案）

**长期"间歇静默、全台架 OFFLINE、重刷固件无效"的根因是那只 SLCAN CANable 本身**——它以
88.24% 的数据段采样点挂在总线上当第二个节点，把板子每个**带数据相位的 FD 帧**采样越界 ⇒
发错误帧、不给 ACK ⇒ 板子 `TEC` 累加 ⇒ 负载下冲过 255 进 bus-off。**不是板子余量不足，
不是模拟器，也不是带宽。**

判据来自 SWD 后台访存（`openocd -c "init" -c "mdw ..."`，**不加 `halt`**——加了会扰动：
SYNC 从 50.0 掉到 47.0 Hz，不 halt 则 705 个采样期间恒为 50.0 Hz）：

| 寄存器 | 健康 | 出错前导 | 静默定格 |
|---|---|---|---|
| `PSR.DLEC` | 0 | **4**（数据相位 Bit1Error） | **3**（数据相位 ACK error） |
| `PSR.BO` | 0 | **1**（bus-off） | 0（error-passive） |
| `ECR.TEC` | 0 | REC 冲到 108 | 钉在 **128** |
| `TXBRP` | 0 | 累积 | **0x0000ffff**（16 个发送缓冲全挂起） |
| `TXFQS.TFFL/TFQF` | 16 / 0 | 下降 | **0 / 1**（FIFO 满） |

静默期间 USB CDC 侧完全正常（`snapshot_accepted` 一路照涨），只有 CAN 侧死；主机重开适配器
通道的**同一毫秒** SYNC 恢复——所以那个"稳定的 ~4 s"是主机自己的恢复延迟（3.0 s 检测 +
6×150 ms 重配），不是物理常数。

**修法（已验证）**：把那只 SLCAN CANable 请出总线，改用**可在主机侧精确设位时序**的
gs_usb/SocketCAN 适配器：

```bash
ip link set can0 type can bitrate 1000000 sample-point 0.8 \
    dbitrate 5000000 dsample-point 0.8125 fd on
ip link set can0 txqueuelen 200      # gs_usb 默认 qlen 10，负载下溢出发 ENOBUFS
```

`dsample-point` 取 **0.8125**（26/32）与板子**逐位相同**：160 MHz 时钟下 5 Mbps 取 `dbrp=1`
共 32 tq，恰好 80% 需要 25.6 tq、整数不可达，而 81.25% 正是板子实测的值（`DBTP`：
`DBPR=1/DTSEG1=12/DTSEG2=3`）。改完实测：负载下 `TEC=0`、`DLEC=0`、`BO=0`、`TFQF=0`
（380 个采样全零），静默 0 次，`tx_dropped=0`。

**板子侧一个字节都不该动**：`NBTP`（`NBRP=1/NTSEG1=63/NTSEG2=16` = 80 tq、**80.0%**）与
`DBTP`（**81.25%**）是冻结标定，匹配真机电机的 80%，改板子会把真机链路做坏。那只 SLCAN
CANable 要用的话得改它自己的数据段采样点（`DataTimeSeg1/2` 14/2 → 13/3，17 tq 不变 →
82.35%）后重烧——**尚未实施**，且其固件源码（`/tmp/canable2-fw`）在本机已不存在。

⚠️ 若将来动适配器固件：DBTP 的 `DTSEG1` 只有 5 位、HAL 上界 **32**（仲裁段 `NTSEG1` 是 8 位，
别照搬）。

**gs_usb 适配器的两个已知卡死**（2026-10-15 实测）：跑负载后先 TX URB 停止归还
（`send()` 报 `ENOBUFS`、接口却仍 `ERROR-ACTIVE`、`ip -s` 的 TX 计数冻结），`down/up` 能修好
TX 却会把 **RX** 弄死，只有 **USB unbind/rebind** 才彻底复位：

```bash
echo 1-11 > /sys/bus/usb/drivers/usb/unbind; sleep 2
echo 1-11 > /sys/bus/usb/drivers/usb/bind
```

（`1-11` 是该适配器的 USB 路径，见 `/sys/class/net/can0/device`。）另注：这只 gs_usb 的
`ip -s` **RX 计数不上报**，判断收没收到要以模拟器自身遥测为准。

**已删除的错误绕法**：2026-10-14 曾在固件加编译开关 `-DFDCAN_DATA_2M_BENCH` 把数据段降到
2 Mbps 绕开本问题。当日已从 `lib/fdcan/src/fdcan.c` 与 `platformio.ini` 删除：它是往真机
标定上打补丁，且必须靠人记着"接真实电机前关掉"，漏关就把真机链路做坏。

## 启动顺序

**顺序不可颠倒，而且"板子先于 ACK 节点启动"会永久卡死**：CANable2 必须先成为 ACK 节点，
控制板才复位。板子若在无 ACK 节点的总线上启动，SYNC 拿不到 ACK → `AutoRetransmission`
重传把三路 TX FIFO 全部填满（诊断显示 `tx_frame_count == 16`，恰好是 FIFO 深度）→ 此后
再也发不出任何帧（`rx_frame_count` 也停在 0），**再复位也好不了**（会立刻重演）。

1. 启动 micro-ROS Agent：

   ```bash
   source /opt/ros/humble/setup.bash
   source "$(git rev-parse --show-toplevel)/micro_ros_agent/install/setup.bash"
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
   ```

2. 启动主机侧 bridge（本指南的 agent-only 流程才需要；`robot.launch.py` 的 L1 已
   常驻一个，不要再起第二个——重复的发布者/订阅者会把命令与状态各处理两遍）：

   ```bash
   source "$(git rev-parse --show-toplevel)/ros2_ws/install/setup.bash"
   ros2 run micro_ros_bridge micro_ros_bridge_node
   ```

3. 启动主机模拟器（板子数据段是 5 Mbps，索引走自动探测 `Y5`；自动探测不上再显式加
   `--data-bitrate-index 5`）：

   ```bash
   python3 "$(git rev-parse --show-toplevel)/micro_ros_agent/tools/fdcan_motor_sim.py" \
     --tty /dev/ttyACM1
   ```

   `--log <jsonl>` 会逐帧落盘并把主循环拖慢，反馈窗口可能越过固件 100 ms 离线门而
   触发全体 OFFLINE——只在需要抓原始帧定位时开，抓完即关。

   本指南只验收总线与发现/离线，不跑力矩，所以不传 `--joint-nm-per-amp`：反馈电流
   会恒报 0（模拟器启动时会就此告警）。要看真实电流刻度走
   `robot_bringup` 的 `robot.launch.py`，那条路从型号库自动带上换算系数。

4. 看到 `S8/Y5/M0/A1/O opened`（若数据段配错，`V` 查询与 `PSR.DLEC=3` 会暴露）后，再由人工复位键或已授权的 OpenOCD `reset run` 复位控制板。
5. 模拟器收到首个 `0x080` SYNC 后才开始错峰发送反馈。

## 只读验收

- 模拟器四节点反馈计数应均衡，长期相差不应持续扩大；
- `/micro_ros_up/motor_topology` 应恰好包含 id 1–4，全部 online、无 conflict；
- OpenOCD 只读寄存器应显示 `TEC=REC=0`、`EP=EW=BO=0`；
- FDCAN2/3 悬空导致的 BUS_OFF 不属于本指南的 FDCAN1 失败；
- 反馈节拍**默认照真机**：每个 SYNC 回一帧（真机 TPDO 传输类型 = 1，2026-10-15 实测
  50 Hz 下 48 发 48 收、延迟约 124 µs）。要降低总线负载用 `--fb-period-ms 50` 切到
  降载模式（可配 `--stagger-ms` 错峰）；该周期是**上界**，必须小于固件离线阈值（100 ms）。

## 设备参数上电比对（2026-09-17 起）

固件上电会把每台 CERB 的驱动板参数（`0x607D:01/:02`、`0x6080`、`0x2538`）
回读一次，与 W25 里的固件记录**按设备端原值**比对：

- ⚠ **2026-10-12：设备参数核对不再是安全门（维护者 为台架效率作的取舍）**。
  重构前的语义（2026-09-17 起）：初始化未完成（不一致 `40` / 回读不成功 `41` /
  本机无记录）的那台进 `CONFIG_CONFLICT`、拒绝使能、不出运动帧。**现语义**：只把
  结论如实上报（`dev_init`、`DEV_INIT` 事件位图、`device_param_report`），
  **不再改 lifecycle、不再拒绝使能、不再拦运动帧**。代价：`dev_init ≠ OK` 时该台的
  限位/速度/力矩可能不是核对过的值，兜底只剩硬件急停与机械限位。
  恢复门禁（两处，缺一不可）：`lib/motor_control/src/motor_control_internal.h` 的
  `motor_control_dev_params_allow_motion` 改回 `(dev_init == NA || dev_init == OK)`；
  `lib/motor_control/src/motor_control_config.c` 放开注释掉的
  `freeze_entry_as_conflict(ctx, entry, MOTOR_CONTROL_FAULT_DEV_PARAM, now_ms);`。
  不恢复门禁时，"哪台没核对上"仍可从 `device_param_report` 与 `dev_init` 事件读出。
- 复位/换机/改接线后若记录与模拟器默认值不同（模拟器 CERB 默认
  `0x2538=1000 mA`、`0x6080=600 RPM`、位置限 `±32768 cnt`），走一次参数写即可让
  设备与记录重新一致。

  **参数面自 2026-09-21 起是话题，没有 ROS 参数服务器**（`ros2 param set/get` 全部
  失效，web 参数页同期报废）。主机入口是 `/motor/param_set` 与 `/motor/system_command`，
  由 `micro_ros_bridge` 统一编成线上 `/micro_ros_do/config`（`kind` 区分）。下面的示例
  **要求 bridge 已在跑**（「启动顺序」第 2 步）；只起了 Agent 时这些话题无人订阅，
  `--once` 不报错、命令静默不生效。`field` 用
  **设备槽序**，与 `device_param_report` 的行序同一套：`0`=位置下限 `1`=位置上限
  `2`=转速上限 `3`=力矩上限 `4`=保护开关 `5`=方向。**力矩列的单位是安培**——设备侧
  权威量是 `0x2538` 的最大电流，折 N·m 要过 Kt×减速比×效率，该系数本身有未决争议
  （见 `motor-runtime-decision-log.md` §6.4）。

  ```bash
  # 单字段：改 1 号机的力矩上限（安培）。count 必须显式给，且等于 targets 长度
  ros2 topic pub --once /motor/param_set can_bridge_msgs/msg/ParamSetArray \
    "{count: 1, targets: [{motor_id: 1, field: 3, value: 2.5}]}"

  # 位置限位必须在同一条消息内成对（只有一边 → 本地拒绝，last_error=10）
  ros2 topic pub --once /motor/param_set can_bridge_msgs/msg/ParamSetArray \
    "{count: 2, targets: [{motor_id: 1, field: 0, value: -3.14},
                          {motor_id: 1, field: 1, value: 3.14}]}"
  ```

  **写 ≠ 保存**：上面的写只改设备 RAM 与固件记录（写后立即重判，写齐即转好）。要
  持久化必须再发一条 SAVE，并等 `save_in_progress` 转 false、`save_seq` 前进才叫
  真落盘（W25 擦写有秒级耗时，此前报"完成"是错的）：

  ```bash
  ros2 topic pub --once /motor/system_command can_bridge_msgs/msg/SystemCommand \
    "{command: 0, motor_id: 65535}"       # 0=SAVE；SAVE 只接受 MOTOR_ID_NONE
  ros2 topic echo /micro_ros_up/system_status
  ```

  没保存的台在 `/micro_ros_up/device_param_report` 上是 `record_dirty=true`。
- 运行中（ACTIVE）回读发现不一致走**软停**（阻尼帧、保留使能），不硬失能。
- **设零位**（`command: 2`）：写 0x2531=1，**仅失能态受理**；总线上应看到一条
  `23 31 25 00 01 00 00 00`。零位改变后设备内 0x607D 限位随之移动，须重认限位与机械
  行程。**持久化已实测**（2026-09-17）：设零 → 驱动器断电重上电 → 零位保持，无需额外
  保存码。

  ```bash
  ros2 topic pub --once /motor/system_command can_bridge_msgs/msg/SystemCommand \
    "{command: 2, motor_id: 1}"           # 1=REFRESH 回读, 3=CLEAR_FAULT 清故障
  ```

  完成判据同样看 `system_status` 的 `set_zero_seq` / `refresh_seq` / `clear_fault_seq`。
- 设备侧写限值之后，总线上应能看到 `23 39 25 00 01/02/04` 的保存帧——看不到就说明
  落盘没做，驱动器一断电就回出厂值，下次上电会判"不一致 → 未初始化"。
- 回读失败会自动退避重试（≤3 次、间隔 500ms；SDO upload 是只读的，重试不
  改设备任何状态），用尽才判 `41`——偶发一次 `41` 通常重连或再发一条
  `{command: 1, motor_id: <id>}`（REFRESH）即可。
- 想核对设备真值：`ros2 topic echo /micro_ros_up/system_status` 看 `last_error` /
  `last_error_motor`；或看 `/micro_ros_up/device_param_report` 的 `dev_*`（驱动板回读）
  与 `rec_*`（固件记录）两行，以及 `record_dirty`（未落盘）。

## 停止与恢复

Ctrl-C 停止模拟器时脚本发送 `C` 关闭 CANable 通道并输出每节点统计。FDCAN1 已进入 BUS_OFF 时，应先停止旧进程，重新启动模拟器并确认通道已开，再复位控制板；不要只重启 ROS Agent。

## 已验证基线与剩余操作

**历史基线（九节点台架，已非默认拓扑）**：30 秒运行 `ECR=0`、节点 1–9 全 online、
反馈计数 `1546/1545×8`（2026-09-11 复测，当时数据段索引 `Y5`）。九节点拓扑现已让位于
默认的 CERB 1=R58、2–4=R48 四节点拓扑；要重做九节点须显式传入节点参数。

以下不属于只读指南：发布 `/motor/command_array` 或发送任何电机运动目标
（`MotorBind`/`motor_enable` 话题已删除）。继续前必须逐项记录环境与证据（拓扑、
CANable 版本与索引、固件版本与位时序、每总线计数），并确认总线无真实电机且已取得
外部操作权限。
