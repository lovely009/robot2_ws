# 电机模拟器收敛为单一后端（C++ `motor_sim`）

**Status**: accepted（2026-10-15 用户裁定：先更正论据、再接受）

> **论据更正（2026-10-15，代码复查）**：本 ADR 初稿称 Python 后端「1770 行、无单测」，与代码不符——
> 实为 **1851 行**，且已有 **523 行 / 121 条 assert** 的 CTest 断言
> （`micro_ros_firmware/test/host/test_fdcan_motor_sim.py`）。初稿行数与今日实测之差也说明：本 ADR
> 写完之后 Python 侧仍在继续改动。据此改写了 Context 的代价 4 与 Decision §1 的移植代价——
> **删 Python 的代价比初稿估计的高**：不是「没有测试可丢」，而是「契约测试寄居在另一个区域、
> 且无法复用到 C++」。

## Context

仓库里有**两个**都能"在 CAN FD 总线上扮 CERB/VAXOR 电机节点"的后端，按 `controller_mode`
分叉使用，能力不重叠：

| | C++ `motor_sim`（`ros2_ws/src/motor_sim/`） | Python `fdcan_motor_sim.py`（`micro_ros_agent/tools/`，1851 行） |
|---|---|---|
| 谁在用 | `controller_mode:=host/external`，`simulation.launch.py:968` 起 `motor_sim_node` | `controller_mode:=firmware`（HIL），`simulation.launch.py:874` 起该脚本；bench/web 的「电机后端」 |
| 配置 | YAML（launch 生成 `motor_sim.yaml`，`simulation.launch.py:349-366`），`TopologyContract` 逐项校验 | 30 项 CLI，由 launch 逐项翻译（`simulation.launch.py:874-925`） |
| 独有能力 | 多总线；YAML 拓扑单源；故障锁存/诊断/门控/`reset_fault`（`safety_monitor.hpp`、`motor_sim_node.cpp`）；13 个 gtest | CERB CANopen **SDO 从站**（对象字典含真机上电初值，`:421`、`:1054`）；运行期控制通道 `status/add/remove/link`（`:1427`）；CANable 适配器探测与 bus-off 恢复（`:891,:1379`）；反馈周期与相位错开（`:175,:1180`）；`--ready-file`；故障注入开关 `--safety-off`/`--dir-ccw` |

两者是**同一个东西的两个宿主**：同一套 CERB/VAXOR 帧语义，差异只在宿主环境与实现进度。当前代价：

1. **同一语义两处实现、两处漂移**：τ_ff 量化满量程（R48 11 / R58 45）、VAXOR 回显帧必须满 12B、
   SDO `0x2650` 只有子索引 `:00` 存在——这些坑都在一侧踩到后手工补到另一侧。
2. **HIL 只能单总线**：Python 侧单 `--tty/--interface`；`simulation.launch.py:651` 只好靠
   「ID 跨协议全局唯一」绕开。
3. **一个字段改三处**：模型 YAML → launch 翻译 → 两实现各读一次。
4. **HIL 的 CAN 侧契约测试寄居在固件区域**：Python 模拟器的断言不在 `micro_ros_agent/tools/`，
   而在 `micro_ros_firmware/test/host/test_fdcan_motor_sim.py`（523 行、121 条 assert，已注册为
   CTest）。代价有二：① 改模拟器要去**另一个区域**的测试树改断言；② 断言是 Python import +
   assert，**无法复用**到 C++ 后端，必须逐条重表达为 gtest。

已核对的事实（2026-10-15，本工作区）：

- `motor_sim` 的分层已经是「内核 + ROS 壳」：`motor_protocol` / `motor_transport` /
  `motor_sim_core` 三层**无 ROS 依赖**（只链 `yaml-cpp`、`Threads`，`CMakeLists.txt:14-46`），
  只有 `motor_sim_ros` 依赖 rclcpp（`:49-59`）。
- `SerialTransport` 的 SLCAN 初始化是**写死**的 `"C\rS8\rY5\rO\r"`（`serial_transport.cpp:100`），
  没有 Python 侧的 `V` 探测、位率索引自适应、`E` 取证、bus-off 恢复与重开通道；TX 是直接写、
  无队列与丢弃策略。`SocketCanTransport` 只做收发与 FD/BRS（`socket_can_transport.cpp:74-94`），
  不看错误帧与总线状态。
- 反馈节拍是**全局**的：`control_timer` 按 `control_hz` 驱动，VAXOR 按 `control_hz /
  vaxor_feedback_hz` 抽帧（`motor_sim_node.cpp:348,431-449`）；没有 Python 侧的逐台周期、相位
  错开与反馈窗口（`fdcan_motor_sim.py:175,1180`）。
- 控制通道现状：Unix socket + JSON 行协议，bench_manager 的 `MotorBackendCommand` 只是转发
  （`bench_msgs/srv/MotorBackendCommand.srv`、`bench_manager/bench_manager/motor_sim_manager.py:41`），
  launch 内嵌的那份落在固定路径 `/tmp/robot_sim_motor_backend.sock`（`simulation.launch.py:855-866`），
  并因此有「external vs owned」的区分。
- 本机依赖：`pybind11`、`cffi` 均未安装；`libjsoncpp-dev 1.9.5` **已装**且带 CMake config
  （`/usr/lib/x86_64-linux-gnu/cmake/jsoncpp/jsoncppConfig.cmake`）。
- 故障字段的语义宿主已存在，无需新造：CERB 错误码 `data[6..7]`（`cerb_adapter.c:128`，固件只透传
  不解释）、权威故障位 `data[11]` bit5（`:196`）、温度 `data[8..9]`（`:186`）；VAXOR `fault u32`
  bit0 驱动芯片 / bit1 过压 / bit2 欠压 / bit3 过流 / bit4 编码器（`motor_drive/docs/motor-2530-series.md:147`）。
- `docs/README.md` 规定**过程性产物（实施计划、任务台账、逐次运行报告）不进仓库**：本决定的实施
  计划留在评审对话里，本 ADR 是唯一入库宿主。
- Python 后端的契约断言寄居在固件区域：`micro_ros_firmware/test/host/test_fdcan_motor_sim.py`
  （523 行、121 条 assert，经 `test/host/CMakeLists.txt:239-240` 注册为 CTest），覆盖 SDO 中止码
  分层、上电初值、温度字段偏移、逐型号 τ_ff 量化、反馈窗口墙钟兜底等——即本 ADR 初稿误记为
  「无单测」的那部分。

## Decision

### 1. 唯一后端 = `motor_sim_node`（C++），Python 后端删除

所有场景（`host` / `external` / `firmware` / `simulator_only` / web「电机后端」）共用**同一个可执行
文件、同一份配置**，差异只在配置内容与外部进程（STM32、Agent、被控对象）。`fdcan_motor_sim.py`
连同其重复实现一并删除；`fdcan_sniffer.py`、`cerb_sdo_probe.py`、`cerb_fullscale_probe.py` 是取证
工具、不是模拟器，留在原地。

选 C++ 侧的理由（用户授权择语言）：

- **结构性能力在 C++ 侧**：多总线、YAML 拓扑单源与 `TopologyContract` 校验、13 个 gtest、
  与 `motor_drive` 的 C++ 协议实现同源。这些在 Python 侧重来一遍的成本远高于反向移植。
- **Python 侧的独有能力是可移植增量**：SDO 从站、控制面、适配器健康、反馈调度都是**有限且可测**
  的功能块，能逐块落到 C++ 并配 gtest；反过来把「ROS 节点的拓扑契约/门控/诊断 + 高帧率」在
  Python 里重建，等于把已验证的 host 闭环路径降级。
- **性能余量**：HIL 负载已是 18 台 × 100 Hz 快照 + 20 ms 反馈（≈1800 + 450 帧/s），加上 200 Hz
  力矩更新；C++ 无线程与解释器开销的顾虑。
- 代价必须写清：**台架现场改一行就要重新 `colcon build`**，Python 时代「改完直接跑」的迭代方式
  消失；以及要把 `micro_ros_firmware/test/host/test_fdcan_motor_sim.py` 里那 121 条断言（如
  「子索引 `:01/:02` 回 ABORT」「上电初值 = 真机实测值」）逐条**重表达**为 gtest——它们已经是测试，
  但宿主语言与位置都不对，转漏了就是回归。

### 2. 配置单源：一份 YAML，schema 由 `motor_sim` 定义与校验

拓扑与型号参数的来源不变（模型目录的 `motor_driver.yaml` + `simulation_topology.yaml` + 型号库
`motor.yaml`，仍由 launch 合成），产物从「YAML + 30 项 CLI」合成**一份**配置，节点独占读取。
现有 CLI 的去处：

| 现有 CLI | 去处 |
|---|---|
| `--cerb` / `--vaxor` / `--cerb-r58` / `--joint-map` / `--tau-limits` | `motors[]`（id / 协议 / 型号与身份串 / 关节名 / MIT 量程与力矩饱和），已在 `motor_sim.yaml` 的 `motors[]` 里 |
| `--period-ms` / `--fb-period-ms` / `--stagger-ms` / `--effort-rate-hz` | `timing`（新增逐台反馈周期与相位错开；与现有 `control_hz` / `vaxor_feedback_hz` / `command_timeout_ms` 合并成一套语义） |
| `--tty` / `--interface` / `--data-bitrate-index` / `--no-brs` / `--probe-adapter` | `buses[]`（传输类型 + 端点 + 位率与数据段 + 探测开关；多总线共用一套字段） |
| `--safety-off` / `--dir-ccw` | `motors[].sdo_overrides`（保护开关 `0x2650:00`、方向 `0x2541:00` 的初值） |
| `--gazebo` / `--gazebo-control-mode` / `--position-topic` / `--effort-topic` | 节点参数（他处已有同一套：`/joint_states` 订阅 + effort 发布 + 拓扑顺序映射） |
| `--ready-file` / `--log` | 节点参数（就绪文件、日志目标） |
| `--control-socket` | 见下条：改为 ROS 服务 |

**两条落实口径（2026-10-15，增量 1 落地时定）**：

- **`timing` 新字段的缺省 = 旧行为**：0 表示沿用 `control_hz`、不错峰、不另设力矩更新率，
  不预置 Python 后端的数值（`--period-ms 20` / `--stagger-ms 2.2` / `--effort-rate-hz 200`）。
  理由：CPU 侧今天只有 `host`/`external` 读这份 YAML，而它们跑的正是旧节拍；把 Python 的数值
  写进缺省，等于替后续的反馈调度增量提前改掉这两个场景。**firmware 分支切过来时那些数值由
  模型拓扑显式声明**，不靠缺省。
- **`sdo_overrides` 在增量 1 只有 schema、没有输入通路**：模型拓扑与 `motor_driver.yaml` 都不
  声明保护开关与方向，launch 写的是真机初值常量（`1` / `0`）。通路随第 4 节的 SDO 从站一起接；
  在那之前生成的 YAML 里这两个值是常量，**不是**"从拓扑取到的"——写成 `motor.get(...)` 会让
  它看起来有权威来源而实际恒等缺省值。

### 3. 控制面从 Unix socket 改为 ROS 服务

模拟器既然是 ROS 节点，运行期控制面就用 ROS 服务：`motor_sim` 包内定义 `srv/MotorSimCommand.srv`
（请求 = 一条 JSON 命令，响应 = `ok` / `error` / `response_json`，**字段语义与现有 JSON 行协议
逐字相同**：`status` / `add` / `remove` / `link`，后续加 `inject`）。bench_manager 的
`MotorBackendCommand` 从「转发 socket」改为「调用服务」；web 面板的 JSON 不动。

连带消失的三样东西：`/tmp/robot_sim_motor_backend.sock` 固定路径与死 socket 清理、launch 与
bench_manager 两处必须一致的路径常量、「external vs owned」这套只在 socket 下才有意义的区分。
JSON 解析用系统 jsoncpp（已装、有 CMake config；`package.xml` 的 rosdep key 在实现时确认）。

### 3b. 电机后端自成一包：由 `motor_sim` 自己的 launch 承载，不再经 `robot_sim`

`bench_manager` 现在为了起一个电机后端，拼的是
`ros2 launch robot_sim simulation.launch.py simulator_only:=true controller_mode:=firmware …`
（`motor_sim_manager.py:257-278`）——bench_manager 因此硬依赖 `robot_sim` 的 launch 接口与一串
`controller_mode` / `transport` / `model` 参数。**判决（2026-10-15 用户裁定）**：电机后端归
`motor_sim` 包自己，出一个自己的 launch（或直接 `ros2 run motor_sim motor_sim_node` 加参数文件），
`bench_manager` 改为起它；web 仿真页「电机后端」的起停、日志与电机增删能力**不得退化**。

**顺序约束（先决条件，不可颠倒）**：`bench_manager` 只有在 C++ 节点具备第 4 节那些 HIL 能力
（至少 SDO 从站与传输健康）之后才能切过去，否则台架当场退化。因此本条的落地位置在第一步的
**收尾**，与「firmware 分支改起同一节点」「删除 Python 后端」同一批。在此之前 `motor_sim`
包可以先有自己的 launch，但 `bench_manager` 仍走旧路径。

**跨区域注意**：Python 后端 `fdcan_motor_sim.py` 在 `micro_ros_agent/tools/`，属**另一个构建
区域**，所以「`motor_sim` 自己的 launch」只有等 C++ 节点接管后才是干净的。把该脚本搬进
`ros2_ws` 是反向做法，不做。

### 4. 内核补齐 HIL 必需能力（逐块落、逐块测）

| 能力 | 现状 | 落地目标 |
|---|---|---|
| CERB SDO 从站 | 仅 Python | 内核 `core/sdo_server`：对象字典（`0x1008` 身份、`0x607D`/`0x6080`/`0x2538`/`0x2650`/`0x2541`/`0x6064`）、分段上传、写后读回、不存在的子索引回 ABORT `0x06090011`、易失对象的上电初值照真机实测值 |
| 反馈调度 | 全局节拍 + VAXOR 抽帧 | 逐台周期、相位错开、反馈窗口（窗口跨度 ≈ 周期），保留 `--no-brs` 这类对照开关 |
| 链路开关与增删 | 仅 Python | 内核：`link down` = 完全静默（不发反馈、不回 SDO、忽略下行），恢复时按重新上电清状态；运行期 `add`/`remove` |
| 传输健康 | 写死 SLCAN 初始化、无恢复 | `V` 探测与位率索引自适应、`E` 取证（探针默认关）、bus-off/静默恢复、重开通道、TX 队列与丢弃策略 |
| ready-file | 仅 Python | 节点参数：接口就绪后落文件供总启动器排序 |

### 5. 两步交付

- **第一步：一个后端覆盖所有场景（行为对齐）**。统一配置落地；第 4 节的五块能力落进内核并配 gtest；
  控制面改 ROS 服务；`simulation.launch.py` 的 `firmware` 分支改为起同一个节点；`bench_up.sh`、
  `bench_manager` 的引用同步改；删除 `fdcan_motor_sim.py` 与 socket 面。验收 = 台架场景改前/改后
  帧级对照（对照夹具进 `motor_sim/test/fixtures/`，结论进本 ADR，过程不入库）。
- **第二步：故障建模与收尾**。故障状态机与统一注入面（下条）；文档迁移（`micro_ros_agent/README.md`
  的「FDCAN 电机节点模拟器」章节迁到 `motor_sim/README.md`，`motor_sim/README.md` 的
  「与真机栈对比」表按新事实改写）；台架回归（发现 → 配置 → 武装 → 下发 → 反馈，18/18）。

### 6. 故障建模：可注入的场景，不做语义解释

内核按对端（真机）建模，产出故障**现象**；语义解释仍归上游（固件 `cerb_adapter.c`、`motor_drive`）：

- CERB 反馈帧：`data[6..7]` 错误码、`data[10]` 模式、`data[11]` 状态位（bit5 故障 + 使能/抱闸/到位）；
- VAXOR 状态帧：`fault u32`（bit0–bit4 按 `motor-2530-series.md:147`）、`motor_state`、电压；
- 温度：线圈温度一阶热模型（电流平方积分 + 冷却），参数可配；
- 保护动作：`0x2650:00 = 0`（保护生效）时看门狗/限位触发 → 置故障位并停止出力；`= 1`（真机默认）
  时不动作——复现真机「保护被关掉」的场景；
- 注入面统一：控制服务 `inject` 命令 + 配置场景段 + gtest 直调；维度 = 哪台 / 哪种故障 / 持续或
  锁存 / 何时开始。原 `--safety-off`、`--dir-ccw` 并入配置，不再有第二套注入。
- **不做**：电流环、热降额曲线、编码器噪声、机械磨损——既无实测数据也无消费方。

### 7. 验收

1. **gtest**：第 4 节每块能力 + 故障状态机都有用例；现有 13 个测试文件不退化。
2. **帧级对照**：台架场景（STM32 + CANable/SocketCAN）改前（Python 后端）与改后（C++ 后端）的下行
   帧序列与 SDO 事务逐字节一致；对照用帧记录进 `test/fixtures/` 成为常驻回归。
3. **场景矩阵**：`host` / `external` / `firmware` / `simulator_only` 四个场景都由同一可执行验证，
   含多总线一条（Python 后端做不到，属新增能力）。
4. **台架回归**：页面「运行 18/18」、`snapshot_stream.py` 的 `hold`/`step` 行为不变。

## Consequences

- **收益**：一类语义一处实现；HIL 顺带获得多总线；字段改动只改一份 schema；台架踩过的坑变成
  gtest 与常驻帧夹具，而不是只存在于 Python 注释里的经验。
- **代价**：台架现场的改动需要重新构建；SDO/适配器健康这类「按对端建模」的细节在移植期有回归
  风险，靠帧级对照与 gtest 兜；控制面从 socket 改服务后，`bench_manager` 与 web 面板的调用路径
  要同批改（两者都已有测试，但测试里的注入点要跟着换）。
- **待实测、不得凭手册写进实现**：① `0x2650:00 = 0` 保护触发时真机的实际动作（停止出力还是
  抱死、反馈什么码）——先按「置故障位 + 停止出力 + 需清错恢复」建模并标注未实测；② 抱闸的物理
  行为（失能后是否自锁）；③ 温度模型时间常数与阈值；④ CERB 报错代码表完整语义（厂家 PDF
  `motor_sim/docs/`，内核只产出码值、不解释）。
- **不改的东西**：`motor_drive` 与真机 bring-up 链路、`robot_sim` 的被控对象后端（Gazebo/MuJoCo）
  与 effort 门控、固件侧的任何字节、micro-ROS Agent 与取证工具。
- **与既有 ADR 的关系**：不改变 0002（拓扑与真机解耦——`motor_sim` 仍不含机器人拓扑）、0011
  （被控对象后端与模拟器实现正交）、0009（重力前馈叠加点）。本 ADR 只改「模拟电机自身的实现
  形态、配置来源与运行期控制面」。
