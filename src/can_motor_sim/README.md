# can_motor_sim — 基于 VCAN 的电机驱动调试（ROS2）

一个用于 **基于 vcan 虚拟 CAN 总线进行电机驱动调试** 的 ROS2 (Humble) Python 包。

它由三部分组成：

| 组件 | 作用 |
| ---- | ---- |
| `motor_slave` | CAN FD 电机**从站模拟器**，支持协议 A（25–30mm MIT 式）与协议 B（RP 系列），通过 ROS2 topic 收发 CAN 帧 |
| `motor_master` | **交互式调试主站**（CLI），在终端中发送使能/模式/位置/速度/寄存器读写等命令，并打印解码后的物理量 |
| `ros2_socketcan` 桥接 | 把 `vcan0` 网卡与 ROS2 topic（`/to_can_bus_fd`、`/from_can_bus_fd`）双向桥接，是真实 CAN 总线与 ROS2 之间的“翻译层” |

数据流（主从站之间所有帧都真正经过 vcan0 总线，可用 `candump vcan0` 观测）：

```
motor_master ─/to_can_bus_fd─► sender ─write─► vcan0 ─read─► receiver ─/from_can_bus_fd─► motor_slave
     ▲                                                                                    │
     └─────────────────────────────────────/from_can_bus_fd────────────────────────────────┘
```

- 主站命令：发布 `/to_can_bus_fd` → sender 写入 `vcan0` → receiver 读回发布 `/from_can_bus_fd` → 从站处理。
- 从站应答/周期状态：发布 `/to_can_bus_fd` → sender 写入 `vcan0` → receiver 读回 → `/from_can_bus_fd` → 主站解码。
- 因此 **从站的周期状态（A3/B3）与应答帧（A2）也能在 `candump vcan0` 上看到**，与真实电机一致。

---

## 1. 目录结构

```
robot_ws/src/can_motor_sim/
├── package.xml                  # ROS2 包清单
├── setup.py                     # ament_python 打包脚本（安装资源索引/launch/脚本）
├── setup.cfg                    # 脚本安装目录配置
├── resource/can_motor_sim       # ament 资源索引标记
├── launch/
│   ├── socketcan_bridge.launch.py   # 仅启动 vcan0 桥接（CAN FD）
│   ├── motor_sim.launch.py          # 桥接 + motor_slave 仿真从站
│   └── motor_debug.launch.py        # 桥接 + 从站 + (可选)主站 = 一键调试入口
├── scripts/
│   └── init_vcan.sh                 # 创建 vcan0 (MTU=72) 的脚本
├── can_motor_sim/
│   ├── __init__.py
│   ├── motor_master.py              # 交互式调试主站
│   └── motor_slave.py               # 电机从站模拟器
└── test/
    └── test_import.py               # 冒烟测试
```

---

## 2. 环境准备

### 2.1 依赖

```bash
# ROS2 Humble（已安装 ros2_socketcan / ros2_socketcan_msgs）
sudo apt install -y ros-humble-ros2-socketcan can-utils
```

### 2.2 创建 vcan0 虚拟总线（需要 sudo）

```bash
bash src/can_motor_sim/scripts/init_vcan.sh
# 验证：应显示 mtu 72
ip -details link show vcan0
```

> 也可以使用本仓库对应目录外的顶层脚本 `~/init_vcan.sh`（内容相同）。

### 2.3 构建

```bash
source /opt/ros/humble/setup.bash
cd ~/robot_ws
colcon build --packages-select can_motor_sim
source install/setup.bash
# 确认包已注册
ros2 pkg list | grep can_motor_sim
ros2 pkg executables can_motor_sim
```

---

## 3. 快速开始（一键调试）

### 3.1 只需发命令看总线（无需从机）——一个脚本搞定

```bash
# 后台启动 vcan0 桥接 + 前台交互式主站（无需从站）
cd ~/robot_ws/src/can_motor_sim
bash scripts/run_master.sh A 1      # 协议 A, 控制器 1
# 进入后直接输入 enable / set_mit ... 等命令；quit 后自动清理桥接

# 另一个终端观察总线上所有帧：
candump -x vcan0
```

> 原理：命令要写入 vcan0 网卡，必须经过桥接的 **sender** 节点（由 launch/脚本启动）。
> `run_master.sh` 后台启动桥接、前台运行主站，**无需启动从机**。

### 3.2 带从站模拟器的完整调试（三终端）

```bash
# —— 终端 1：启动 vcan 桥接 + 电机从站模拟器 ——
source /opt/ros/humble/setup.bash && source ~/robot_ws/install/setup.bash
ros2 launch can_motor_sim motor_debug.launch.py protocol:=A node_id:=1
# 若不需要从机，可加 start_slave:=false（只留桥接）

# —— 终端 2：启动交互式调试主站 ——
source /opt/ros/humble/setup.bash && source ~/robot_ws/install/setup.bash
ros2 run can_motor_sim motor_master --protocol A --id 1
# 主站默认不打印周期状态（总线用 candump 看），只打印 [TX] 发送帧与即时回帧

# —— 终端 3（可选）：观察 vcan0 上的原始 CAN 帧 ——
candump -x vcan0
```

> `motor_master` 需要从键盘读取命令，而 `ros2 launch` 不把终端输入转给子进程
> （`emulate_tty` 也只影响 stdout，不转发 stdin），因此**主站应在前台终端运行**
> —— 用 `run_master.sh` 或单独 `ros2 run`。launch 里启动的 master 只能作被动观察。

---

## 4. 交互式命令

在 `motor_master` 提示符 `>` 下输入，输入 `help` 可随时查看命令列表。

> **关于打印**：
> - 主站默认**不打印周期性状态帧**（总线请用 `candump vcan0` 观察）；
>   `status` 命令可临时开启节流打印（`status on` 或 `status 0.2`）。
> - 主站每次发送命令都会打印 `[TX:S/E] id=… len=… data=…`，从站每收到一帧
>   打印 `[RX:S/E] …`，可与 `candump` 抓到的总线帧**逐字节对照**。
> - 所有数值参数支持十进制与 `0x` 十六进制（如 `0x7FF`、`0x8000`）。

### 4.1 协议 A（25–30mm MIT 式，`--protocol A`）

| 命令 | 说明 |
| ---- | ---- |
| `enable` / `disable` | 使能 / 失能电机 |
| `mode mit\|pos\|vel` | 切换控制模式（需先停止） |
| `set_pos <code>` | 下发目标位置（int32 码值，自动加 `0x80000000` 偏置） |
| `set_vel <code>` | 下发目标速度（int16 码值，自动加 `0x8000` 偏置） |
| `set_mit <pos> <vel> <kp_raw> <kd_raw> <iff>` | 完整 MIT 控制帧（手册 3.1 原始码值：`Kp=kp_raw*1e-5`，`Kd=kd_raw*0.01`） |
| `status on\|off\|<sec>` | 开关/调整周期性状态打印（协议通用） |
| `quit` | 退出 |

示例（协议 A, id=1，对应手册 3.1 位置模式）：

```text
> enable
> set_mit 2097152 0 800 100 0   # 位置 2097152=360°、速度0、Kp=0.008(800)、Kd=1(100)、电流0
> set_mit 2000000 0 500 200 1
> [TX:S] id=0x00000505 len=12 data=00 00 20 80 00 80 20 03 64 00 00 80
```

> 该帧与手册 3.1 的例子 `01 00 01 80 00 80 20 03 64 00 00 80` 结构一致：
> 位置=0x80200000（含偏置）、速度=0x8000（零）、Kp=0x0320、Kd=0x0064、前馈电流=0x8000（零）。

### 4.2 协议 B（RP 系列，`--protocol B`）

| 命令 | 说明 |
| ---- | ---- |
| `enable` / `disable` | 使能 / 失能电机 |
| `rp_mit <joint 1-8> <pos16> <vel12> <kp> <kd> <torque12>` | 64 字节广播 MIT 控制（ID 0x80，可同时控制 8 个关节） |
| `rp_pp <pos_rad>` | PP 位置模式单节点控制（ID 0x80+N，4 字节 float 位置） |
| `rp_pv <vel_rad_s>` | PV 速度模式单节点控制（ID 0x80+N，4 字节 float 速度） |
| `rp_reg read <addr> [sub]` | 寄存器读（扩展帧 29 位 ID，可带子地址） |
| `rp_reg write <addr> [sub] <value>` | 寄存器写（扩展帧 29 位 ID，值小端 4 字节） |
| `status on\|off\|<sec>` | 开关/调整周期性状态打印（协议通用） |
| `quit` | 退出 |

示例（协议 B, id=1）：

```text
> enable
> rp_mit 1 32768 0x7FF 100 20 0x7FF
> [TX:S] id=0x00000080 len=64 data=80 00 7F F0 64 01 47 FF 00 00 ...   # 关节1段
> rp_reg write 0xA0 0x01 1       # Pn08(0xA0) 子0x01 模式配置 = PP
> rp_pp 1.5                      # PP 位置 1.5rad（0x80+N）
> rp_reg read 0xA0 0x03          # 读故障寄存器
```

> 协议 B 要点：
> - MIT 广播帧每关节 8 字节，打包顺序为 `pos(16) vel(12) kp(12) kd(12) torque(12)`，
>   `rp_mit` 已按 SDK 算法打包（如 `vel12=0x7FF`→0 速度、`torque12=0x7FF`→0 扭矩）。
> - 寄存器读写使用 29 位扩展帧，位域：`bit0-6 设备ID / bit8-14 产品ID / bit15 成功标志 /
>   bit16-23 寄存器地址 / bit24-27 子地址 / bit28 读写(0读1写)`，地址为**两级（寄存器,子地址）**
>   （如 0x03=Pn3、0xA0=Pn08）。从站按两级存储；Pn08 功能寄存器已实现：
>   子0x01=模式配置(MIT/PP/PV)、子0x02=置零位(应答0x01)、子0x03=故障寄存器(bit0=使能)。
> - PP/PV 经典控制帧 `0x80+N` 携带 4 字节 float（PP=位置、PV=速度），按当前模式解释。
> - **寄存器默认值**（文档 3.1）：Pn3 MIT 阈值默认 -10/10/-60/60/-60/60/0/500/0/10；
>   Pn4/Pn5 保护阈值默认取高于模拟状态（MOS 30℃、母线 24V）的值，默认无故障。
> - **保护故障上报**（文档附录 + Pn4/Pn5）：从站按模拟温度/电压/电流与 Pn4(Level1)/Pn5(Level2)
>   阈值比较置故障位（bit1过压 bit2二级过压 bit3过流 bit4二级过流 bit5 MOS一级过温 bit6 MOS二级过温）；
>   Level2 故障主动停止电机；故障寄存器 bit0 反映使能状态；写 0x0000 清除，若条件仍满足则再次置位。
> - **无固定周期上报**（文档 2 通信机制“一问一答”）：协议 B 从站**不做周期状态帧**，仅收到
>   控制帧（B2/PP/PV）后回 B3、收到寄存器命令后回扩展帧应答；因此不像协议 A 有 20ms 周期帧。
> - **一问一答超时 50ms**（文档 2）：主站 `rp_reg` 记录请求，50ms 内无应答打印 `[B TIMEOUT]`。
> - `rp_mit` 广播其余 7 个关节段填充文档零值 `7F FF 7F F0 00 00 07 FF`，避免误发给其它关节危险指令。

### 4.3 从站模拟的电机响应（以协议文档为准）

| 行为 | 协议 A | 协议 B |
| ---- | ------ | ------ |
| 周期上报 | 每 20ms 发 A3（0x702+4N，20 字节） | **无固定周期上报**：一问一答，仅收到控制帧/寄存器命令后回 B3（ID=设备号） |
| 收到控制帧的应答 | A1(0x501+4N)→ 立即回 A2；A4(0x502+4N)→ 回 A2 确认 | B2(0x80 广播)→ 立即回 B3；PP/PV(0x80+N)→ 回 B3；B4 寄存器读写→同 ID 扩展帧应答 |
| 位置响应 | 位置码值低通跟随目标（A2/A3 上报） | 物理量 rad，位置向指令收敛（0x7FFF=0rad）；PP/PV 按 4 字节 float 指令 |
| 模式/功能寄存器 | A4 byte2 切模式 | Pn08(0xA0)：子0x01 模式(MIT/PP/PV)、子0x02 置零(应答0x01)、子0x03 故障寄存器(bit0=使能) |
| 保护/故障 | — | Pn4(Level1)/Pn5(Level2) 阈值触发过压/过流/过温故障；Level2 停止电机；B3 status 上报故障位 |
| 速度/扭矩 | 速度=位置变化率；电流=前馈 | 速度=rad/s（0x7FF=0）；扭矩=指令+Kp·误差+Kd·误差（0x7FF=0Nm） |
| 状态字段 | A3：state(3停止/4运行)、mode、type | B3：byte6-7 状态/故障位、byte5 MOS温度、byte8 电压、byte9 线圈温度 |

> 从站的所有上报/应答都经 vcan0 总线发出，`candump vcan0` 可直接看到（协议 A 的
> 0x705/0x706、协议 B 的设备号应答帧）。

---

## 5. 用 can-utils 观测虚拟总线

用 `candump` 可以直接看到主站下发的 CAN FD 命令帧，方便排障：

```bash
# 终端 A：监听 vcan0（观察主站命令帧）
candump -x vcan0

# 终端 B：在主站里执行 enable / mode pos / set_pos 2097152
```

预期的总线帧（`##4` 表示 CAN FD 帧；与主站 `[TX]` 输出逐字节一致）：

```text
vcan0 506##4010000000000000            # enable：A4 控制帧（run=0x01）
vcan0 505##40000020800080200364000080  # set_mit 2097152 0 800 100 0：A1 MIT 帧
                                       #   00 00 20 80=位置(0x80200000) 00 80=速度0
                                       #   20 03=Kp(0x0320) 64 00=Kd(0x0064) 00 80=电流0
```

> 说明：模拟从站通过桥接**真正挂在 vcan0 总线上**——它订阅 `/from_can_bus_fd`（总线读入）、
> 发布 `/to_can_bus_fd`（总线写出）。因此从站的**周期状态（A3/B3）与应答帧（A2）都能在
> `candump vcan0` 上看到**（实测：A3 每 20ms 一帧、A2 每收到一帧 A1 回一条）。
> 驱动从站请使用 `motor_master` 命令（`cansend` 发送的经典 CAN 帧在 `enable_can_fd:=true`
> 下不会被接收端转发）；真实硬件场景把 `interface` 换成真实网卡即可（见 FAQ）。

---

## 6. 话题与消息

| 话题 | 类型 | 方向 |
| ---- | ---- | ---- |
| `/to_can_bus_fd` | `ros2_socketcan_msgs/msg/FdFrame` | 主站/用户 → CAN |
| `/from_can_bus_fd` | `ros2_socketcan_msgs/msg/FdFrame` | CAN → 主站/用户 |

`FdFrame` 关键字段：`id`、`is_extended`、`len`、`data[<=64]`。
桥接通过 `socketcan_bridge.launch.py` 将默认话题重映射为 `*_fd` 后缀并开启
`enable_can_fd:=true`。**主站与从站都发布到 `/to_can_bus_fd`（写入总线）、订阅
`/from_can_bus_fd`（读总线）**，从而两方向帧都经过 vcan0。

---

## 7. 常见问题（FAQ）

**Q1：`ros2 launch` 报 “Package 'can_motor_sim' not found”？**
A：未 source 工作空间或未注册资源索引。执行 `source ~/robot_ws/install/setup.bash`，
并确认 `ros2 pkg list | grep can_motor_sim` 有输出；若没有则重新 `colcon build`。

**Q2：发送命令后主站没有收到回复？**
A：依次检查：`ip link show vcan0`（须 UP 且 mtu 72）→ `ros2 topic list` 中是否有
`/to_can_bus_fd`、`/from_can_bus_fd` → 主站与从站的 `--protocol` / `--id` 是否一致。

**Q3：需要 sudo 密码执行 init_vcan.sh？**
A：请在自己的终端中手动执行 `sudo bash scripts/init_vcan.sh`，vcan 内核模块加载需要 root。

**Q4：为什么主站不用 launch 启动？**
A：主站是交互式 CLI（读取键盘输入），`ros2 launch` 不把终端输入转给子进程
（`emulate_tty` 也只改 stdout，不转发 stdin），launch 里起的主站无法交互。
因此：
- **只需发命令看总线（无从机）**：用一键脚本 `bash scripts/run_master.sh A 1`
  （后台起桥接 + 前台交互主站，`quit` 自动清理）；
- **需要从站**：launch 起桥接+从站，另开终端 `ros2 run` 起主站。

**Q5：可以连真实 CAN 设备吗？**
A：可以。把 `interface` 改为真实网卡（如 `can0`，`sudo ip link set can0 up type can bitrate 1000000`），
launch 参数 `interface:=can0` 即可；协议与帧定义相同。

**Q6：主站一直刷屏，没法调试？**
A：主站现在默认**不打印周期性状态帧**（总线用 `candump` 看），只打印
`[TX]` 发送帧与即时回帧，不会再刷屏。若需要观察状态，用 `status on` 或
`status 0.2` 临时开启节流打印。

**Q7：launch 终端刷 “Error receiving CAN FD message: vcan0 - CAN Receive Timeout”？**
A：这是 `ros2_socketcan` receiver 在**轮询窗口内没收到帧**时打印的警告。只要
`receiver_interval_sec` **大于从站周期上报间隔（20ms）** 就不会出现——本包 launch
已默认设为 `0.05`，稳态下不刷屏（仅启动前几秒可能有零星 1~2 次，属正常）。若把从站
周期改慢，请相应调大 `receiver_interval_sec:=<上报周期×2>`。
