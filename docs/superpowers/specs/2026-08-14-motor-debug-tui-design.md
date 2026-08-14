# 电机调试 TUI 上位机（无 ROS）设计

**状态**: 已确认
**日期**: 2026-08-14
**分支**: `dev-real`

## 背景与目标

真机调试需要一个类似电机上位机的交互工具，用于逐台查看电机状态、调增益、
点动、使能/失能与设零位。原型曾设想「走 ROS 通道」（独立 C++ 调试节点 +
自定义 msg/srv + Python 客户端），但在真机台架语境下过重：调试时应独占总线、
与 ros2_control 控制栈互斥，且 Humble 的 `hardware_interface::SystemInterface`
拿不到 node 句柄，把增益/使能/设零这类驱动内部状态强行映射进状态接口很别扭。

**最终取舍：纯 Python 直连 serial_slcan，零 ROS、零 C++ 改动。** 复用仓库里
现成的 Python vaxor 编解码（`scripts/verify_2530_serial.py`），新增 RP 编解码，
用 curses 做交互 TUI。RP 编解码照 `motor_drive/protocol.cpp` 复制一份，接受与
C++ 的漂移（调试工具取舍：简单 > 单一来源）。

## 范围

**做**：
- 实时查看每台电机状态（位姿/速度/扭矩；RP 的 MOS 温度/status/电压/线圈温度；
  vaxor 的电流/电压/fault/state/calib/mode/type；使能态）
- 运行时调 kp/kd 增益
- 点动（jog）目标位置
- 使能/失能（vaxor 单台，RP 总线级）
- vaxor 设零位

**不做**：
- ROS 集成（无 ROS node/topic/service）
- socketcan 传输（只 serial_slcan，遇 socketcan 明确报错退出）
- C++ `motor_drive` / `protocol.cpp` 任何改动
- 软限位/轨迹规划（依赖操作者观察 + 硬件限位/急停，与本仓库「软件兜底不能
  替代硬件急停」的口径一致）
- TUI 的自动化单元测试（交互逻辑手动验证）

## 文件布局

全部在 `src/motor_drive/scripts/`：

| 文件 | 职责 |
|---|---|
| `motor_probe.py`（新） | 共享编解码/收发模块：纯函数，无 curses、无硬件，可单测 |
| `motor_debug_tui.py`（新） | curses 交互 TUI 入口 |
| `verify_2530_serial.py`（改） | 删除内联编解码，改为 `import motor_probe`，行为不变 |

## `motor_probe.py` 设计

纯函数 + 常量，无副作用。以下函数从 `verify_2530_serial.py` 迁入（保持现有
数值/命名不变，仅搬移），RP 为新增。

### slcan 帧收发（迁入）

- `fd_len_code(nbytes) -> str`：FD 长度码映射（9=12, A=16, B=20, C=24, D=32,
  E=48, F=64）
- `encode_frame(identifier, data, is_fd, brs) -> str`：标准 ID 11 bit，`t`/`d`/`b`
  帧头 + 3 位十六进制 ID + 长度码 + 大写 hex payload + `\r`
- `decode_frame(line) -> (id, data, is_fd) | None`：解析 t/T/d/D/b/B 帧

### vaxor 编解码（迁入）

常量：`WR_TICKS_PER_REV=2_097_152`、`WR_CURRENT_SCALE_A=11/4096`、
`WR_KP_SCALE=0.00001`、`WR_KD_SCALE=0.01`。ID 规则 `0x501+4n` 命令、
`0x502+4n` 控制、`0x701+4n` 回复、`0x702+4n` 状态、`0x503+4n` 校准、
`0x703+4n` 校准结果。

- `pos_to_code/code_to_pos`、`vel_to_code/code_to_vel`、
  `current_to_code/code_to_current`
- `le16/le32`
- `pack_wrist_command(position, velocity, kp, kd, current_a) -> bytes`（12 字节 LE）
- `pack_wrist_control(run, set_zero=False, mode=0) -> bytes`（8 字节，经典 CAN）
- `decode_vaxor_feedback(frame_id, data, node) -> dict | None`：结构化解出
  pos/vel/current_a，status 帧另解 voltage_v（u16/10 无偏置，实测口径）、
  fault、motor_state、calibration_state、mode、motor_type

### RP 编解码（新增，照 `protocol.cpp`）

常量：`RP_ENABLE_ID=0x00`、`RP_COMMAND_ID=0x80`、`RP_BROADCAST_SLOTS=8`。

定点映射（与 `protocol.cpp::float_to_uint/uint_to_float` 一致，**截断**而非四舍五入）：

| 字段 | 范围 | 位宽 |
|---|---|---|
| position | [-10, 10] rad | 16 |
| velocity | [-60, 60] rad/s | 12 |
| kp | [0, 500] | 12 |
| kd | [0, 10] | 12 |
| torque(前馈) | [-60, 60] Nm | 12 |

- `pack_rp_slot(command) -> bytes(8)`：P16/V12/Kp12/Kd12/Tau12 位域拼接，字节
  布局照 `protocol.cpp`（b0=P>>8, b1=P, b2=V>>4, b3=(V<<4)|(Kp>>8), b4=Kp,
  b5=Kd>>4, b6=(Kd<<4)|(Tau>>8), b7=Tau）
- `pack_rp_command(commands: list[dict]) -> bytes(64)`：8 槽广播，未配置槽填中性
  槽，配置电机按 `id-1` 放入自己的槽
- `pack_rp_enable(enabled) -> bytes(8)`：经典 CAN，`{enabled, 0×7}`
- `unpack_rp_state(data) -> dict`：12 字节状态帧 → pos/vel/torque + MOS 温度
  （`data[5]-40`）/status（`data[6]<<8|data[7]`）/电压（`data[8]`）/线圈温度
  （`data[9]-40`）

### 统一分发

- `decode_motor_state(frame_id, data, motor) -> dict | None`：按 `motor["protocol"]`
  （`rp`/`vaxor`）分发到 `unpack_rp_state` 或 `decode_vaxor_feedback`，统一返回
  状态 dict（含 `position/velocity/effort` 及协议私有遥测键）。

## `motor_debug_tui.py` 设计

### 配置解析

- 读 `motor_driver.yaml`：命令行 `--config PATH` 指定，默认
  `src/motor_hardware_interface/config/motor_driver.yaml`（相对工作区根）。根
  `motor_hardware` 字段 `motor_specs_file`（相对本文件目录）、`timing`、
  `buses`、`motors`。
- 读 `motor.yaml`（由 `motor_specs_file` 解析）：`motor_types`。
- 每台电机：`type`（型号名）→ `motor_types[type].protocol`（缺省取型号名）→
  协议串 `rp`/`vaxor`；合并型号库默认 `parameters` 与单电机覆盖。
- **只允许 serial_slcan**：每条 bus 必须有 `transport: serial_slcan` 与 `tty`；
  任一条为 socketcan 则打印明确错误并退出（不静默降级）。

### 总线打开

每条 serial_slcan bus：`serial.Serial(tty, 115200, timeout=…)`，依次发送
`C\r`→`S8\r`→`Y5\r`→`O\r` 配置总线（仲裁 1M/数据 5M）。权限不足时打印
`usermod -aG dialout` 提示并退出。

### 控制循环

单线程主循环，约 50 Hz 周期发送命令；每轮同时 drain 反馈与读键：

1. drain 所有串口的反馈帧（非阻塞 `read_until(b"\r")` 短超时），
   `decode_motor_state` 更新每台电机最新状态与「最近反馈时间」
2. `curses` 非阻塞读键，处理命令
3. 到发送周期时，向使能的电机发位置命令（kp/kd 用当前增益）；RP 每总线一条
   64 字节广播（未使能槽填零增益中性槽），vaxor 每台一帧

### 状态模型

每台电机维护：`enabled`、`target_position`、`kp`、`kd`、`latest_state`、
`last_feedback_ts`。启动时全失能、`target_position=None`，增益取 YAML
`command.kp/kd`。

### UI 与按键

顶部：配置摘要（文件、总线、电机数）。中部：状态表（每行一台电机：
name/joint/type/id/bus/pos/vel/effort + 协议私有遥测 + enabled 标记），选中行
高亮。底部：命令栏 + 选中电机详情 + 状态/错误行。

| 键 | 动作 |
|---|---|
| `↑`/`↓`（或 `j`/`k`） | 选择电机 |
| `e` | 使能/失能（二次确认；RP 提示总线级） |
| `+`/`-` | jog 目标位置 ± step（默认 0.05 rad，`[`/`]` 调步长） |
| `g` | 改 kp/kd（提示输入，校验有限且非负） |
| `z` | vaxor 设零位（二次确认；须失能） |
| `0` | 全部失能（应急） |
| `q` | 退出（全失能 + 关串口 + 还原终端） |

### 安全行为

- 启动全失能 + 零增益，绝不自动使能。
- 使能前锁存当前位置为目标（避免一使能就追错目标）。
- `e`/`z` 二次确认；`z` 要求 vaxor 且失能。
- 退出/异常路径：全部失能 + 关闭串口 + `curses.endwin()` 还原终端。
- 软件无软限位，依赖操作者观察、硬件限位与急停（与仓库口径一致）。

## 测试

- `motor_probe.py` 纯函数 → pytest 往返/金样测试：
  - RP `pack_rp_slot`/`unpack_rp_state` 与 `protocol.cpp` 的已知字节向量一致
  - vaxor `pos/vel/current` 码值换算、`pack_wrist_command` 字节布局
  - slcan `encode_frame`/`decode_frame` 往返
  - `decode_motor_state` 按协议分发
- 为此给 `motor_drive` 增加 `ament_cmake_pytest`（`package.xml` 加
  `test_depend`，`CMakeLists.txt` 加 `find_package` + `ament_add_pytest_test`），
  测试文件 `test/test_motor_probe.py` 通过 `__file__` 定位 `scripts/` 并 import。
- TUI 交互逻辑手动验证，不写单元测试。

## 验收标准

- `colcon build` 与 `colcon test` 全绿（含新增 pytest）。
- `verify_2530_serial.py` 重构后行为不变（手工跑通现有 `--listen` 流程）。
- 真机（或台架）上 `motor_debug_tui.py` 能列出配置中的电机、显示实时状态、
  逐台使能/失能/jog/调增益，vaxor 能设零位，`q` 安全退出且电机退磁。
