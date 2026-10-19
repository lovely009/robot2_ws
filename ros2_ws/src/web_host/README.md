# web_host

rosbridge Web 上位机：一个 launch 起 L1 的三个节点——

| 节点 | 可执行 | 作用 |
|---|---|---|
| `rosbridge_websocket` | `rosbridge_shim` | 官方 rosbridge 的**启动垫片**，先给 `rosbridge_library` 补上有界字符串原语（`string<31>` / `string<=31`，见 `web_host/rosbridge_shim.py` 模块注释），再 runpy 运行官方入口；解决 `can_bridge_msgs` 的 `MotorTopology` 等含 `string<=31` 字段时 rosbridge 2.0.7 序列化报 `inst is not a ROS Message, but a str` 的问题 |
| `web_static_server` | `static_server` | 纯标准库 `ThreadingHTTPServer` 托管 `web/`（`index.html` + `js/` + `css/` + `vendor/` + `roslib.min.js`），无 Node 等构建依赖；文件域 API 只余 `/api/models`、`/api/model-joints` 与 `/api/actions*`，静态几何挂在 `model_dir`（默认 robot_sim 的 share，可指源码目录） |
| `command_streamer` | `command_streamer`（`motor_command` 包） | **命令节拍**：页面只下**设定值**，100 Hz 的完整快照流由它出——浏览器把后台标签页的定时器钳到 1 Hz，而固件的快照新鲜度线是 30 ms 软停 / 100 ms 失能，页面一失焦台架就整臂掉力（见 [ADR 0014](../../../docs/adr/0014-host-side-command-pacing.md)）。命令从哪条路出去由 `bench_manager` 改它的 `output_topic` |

> 台架会话的编排者 `bench_manager` **不在本 launch**（2026-10-15 迁出，现由
> [`robot_bringup`](../robot_bringup/README.md) 的 `robot.launch.py` 起）。本 launch 只做
> L1，「只想看页面」不必带上进程管理权限。

**前端只依赖两个通用面**：rosbridge（实时数据 + 台架命令）与静态 HTTP（页面资产、
几何、模型枚举、动作库）。`web/` 因此可以搬到任意静态服务器独立部署；台架管理能力
由 ROS 图承载，`ros2 service call /bench/...` 同样可用。

前端打开页面后按 `location.hostname:9090` 自动拼 WebSocket 地址（见 `web/js/main.js`），
所以从别的机器访问时，浏览器地址栏用主机 IP，rosbridge 也会连到同一个 IP。

页面是**外壳 / 屏 / 面板**三层（纯前端，无 Node 构建依赖，分层决策见 ADR 0010）：外壳 =
顶栏（连接与串口选择）+ 左导航 + 说明浮层；屏只声明挂哪些面板，面板只负责渲染与交互，
二者都不持有跨屏数据。

| 屏 | hash | 面板 |
|---|---|---|
| 运行 | `#/run` | 关节（TF）索引 · 臂视图 · 电机状态表 · 使能与增益 · **重力补偿** · 下发条 · 观察抽屉（曲线 / 事件日志 / 命令历史） |
| 动作 | `#/teach` | 关节（TF）索引 · 臂视图 · 动作库播放 |
| 设备 | `#/device` | 全部参数 · 观察抽屉（事件日志） |
| 仿真 | `#/sim` | 仿真启动器（整宽） |
| 诊断 | `#/diag` | 曲线 · IMU · 事件日志 · 命令历史（整屏铺开） |

跨屏数据全在 `web/js/core/store.js`；订阅在入口全局建立，切屏不丢曲线缓冲与事件列表，
不可见的屏不参与渲染（单一 rAF + 脏标记）。左导航有 1–5 快捷键。

| 区域 | 功能 |
|---|---|
| 顶栏连接区 | rosbridge WebSocket 地址位于控制板串口选择前；地址自动保存在浏览器本地，页面打开或刷新时自动连接/重连 rosbridge。主机侧串口经 `/bench/options` 枚举 `/dev/serial/by-id/` 的稳定路径，并以 `/dev/ttyACM*`、`/dev/ttyUSB*` 补缺（连接建立后取一次，重连自动重取）。「启动并连接」经 `/bench/start_agent` 启动或复用所选端口的 micro-ROS Agent；「断开」经 `/bench/stop_agent` 只停止 bench_manager 管理的 Agent 子进程，两者都不改变 rosbridge 连接。Agent 状态表示进程状态（`/bench/status` 话题），不代表固件客户端已建立会话 |
| 仿真（`#/sim` 屏，整宽） | 三个抽屉。**仿真**：启动/停止 `robot_sim/simulation.launch.py`（经 `/bench/start_simulation` / `/bench/stop_simulation` 服务；状态与日志来自 `/bench/status` 话题，进程存活不等于 Gazebo 已就绪）；**仿真后端可选 Gazebo / MuJoCo**（`backend:=`），并支持 `vcan`、`SocketCAN`、`SLCAN`。控制端可选主机驱动闭环、外部控制器或 STM32 固件 HIL；设备列表来自主机当前 CAN 接口与串口（`/bench/options`）。**模型**：选择加载的模型（按后端过滤——Gazebo 要模型自带 `config/simulation_topology.yaml`，MuJoCo 要 `mujoco/<模型名>.xml` + `standalone.urdf`）并调模型级仿真参数：生成高度 `spawn_z`、重力 `gravity`、固定基座 `fixed_base`。三者只有 Gazebo 有实现（切到 MuJoCo 置灰；缺省不进命令行、给了非默认值由服务端与 launch 双双显式拒绝），参数按浏览器本地记忆，换模型要按模型几何核对生成高度。**电机后端**：总线上扮演 CERB/VAXOR 电机的独立进程（`fdcan_motor_sim.py`，经 `simulator_only:=true` 的 launch 起），参数取仿真抽屉的模型 / CAN 接口 / 设备选择；**HIL 启动仿真时默认拉起、停止时联动收起**（vcan 与其他控制端模式的总线电机由软件 `motor_sim` 节点扮演，不涉及），也可脱离仿真单独起停——只连板子验证固件发现 / 离线 / 重连。运行中列出电机表（id / 协议 / 型号 / 关节 / 链接状态），可添加（joint-map 内的拓扑电机自动挂 Gazebo 关节，其余为裸假节点）、移除、逐台断开/恢复——断开 = 完全静默，固件判其离线、恢复走 RECONNECTED 重识别。命令经 `/bench/motor_backend_command` 转发到模拟器控制 socket（协议语义见 `micro_ros_agent/README.md`） |
| 关节索引（`#/run`、`#/teach` 左栏） | 关节 = **模型 URDF 的关节 × `/tf`**——关节与电机不是一类东西，本栏不读固件的电机流。关节表来自臂视图加载的 URDF（父子 link / 类型 / 轴 / 原点），数值与年龄来自 `/tf`（动态）+ `/tf_static`：用 URDF 的 origin 从 TF 变换反解关节量（revolute/continuous → rad，prismatic → m）。行 = 关节名 / 数值 / TF 年龄，摘要「n/N TF」= 有新鲜 TF 的关节数（动态帧 >1.5 s 没更新算陈旧，年龄用 1 s 定时器补算，停发时不会冻住）。帧对（child，必要时 parent）对不上该行显示「无 TF」；固定关节不入表。点行 = 置顶 + 选中该关节对应的电机（`JOINT_OF_MOTOR` 反查；臂上小球与右栏表随之联动） |
| 臂视图（`#/run`、`#/teach` 的中栏） | `robot_sim` 的 URDF 经 three.js 实时渲染，关节角来自固件 `micro_ros_up/motion_states`（**不是** `/joint_states`：页面本来就订阅它；而 `/joint_states` 要额外起桥，且在 Gazebo 台架上会被 gz 自己的 broadcaster 抢同名话题）。关节小球着色 = 生命周期九态；半透明幽灵 = 页面自己下发的目标，见下「臂视图（3D）」一节。拖拽旋转 / 滚轮缩放 / 点小球选中。图例栏的模型选择器可**页内切换模型**（重载 URDF 与关节映射，选择存 localStorage） |
| 电机状态表（`#/run` 右栏） | **全部电机同屏**（ID / 位置 / 力矩 / 温度 / 状态，18 行不滚动），数据源 = 固件的 `motion_states` / `motor_status_array`（执行器侧；与左栏的关节不是一类东西）。行悬停 title 显示完整读数（速度/电流/跟随误差/命令源/测量年龄）。行序与左栏列表联动（置顶优先）。点击行 = **置顶 + 选中**（与左栏一致）；点臂上的小球只选中不置顶 |
| 全部参数（`#/device` 屏） | **以固件记录为准**，驱动板向记录对齐：位置 **rad**、速度 **rad/s**、力矩 **A**（设备对象 `0x2538` 存的就是电流；位置/速度在固件参数边界是直通，只有力矩过一次 A↔N·m 换算，见决策日志 §6.4）。两行对照取自只读话题 `micro_ros_up/device_param_report`（固件记录列 / 驱动板回读列，同槽位同单位），`dev_init_state` 5（无记录）时固件列整行显示「无记录」。**写 ≠ 保存**——写只改设备 RAM 与固件记录（写后立即重判），要持久化必须再点「保存」（发 `0x2539` + 落 W25），未保存的台标黄。「读取」= 发 `system_command(REPORT)`，固件把**当前参数视图**立刻回一帧 `device_param_report`（不触发 SDO、不访问电机）；电机在线且已核对时显示驱动板真值，不在线/未核对（`dev_*` 为 NaN）时显示的就是板内 W25 参数表；数值用 7 位有效数字显示（内核按 float32 存，原始值会拖出 0.20000000298023224 这类尾巴）。总览条一眼给出「已核对/未初始化/离线」台数与原因（40=与记录不一致、41=回读不成功、42=本机无记录），并**逐台列出不一致的具体项**（位置下限/位置上限/最大转速/峰值电流，来自固件 DEV_INIT 事件的槽位位图）；状态列悬停同样显示该明细；行内「来源」列标出固件记录/驱动板回读/未核对，「状态」列显示生命周期；位置与力矩输入框悬停显示设备端原值（cnt / mA，位置按 65536 cnt/360° 折算）。逐项 `m<id>.{pos_min,pos_max,vel_limit,torque_limit,limit_protect}` 与方向均可写；写事务（写 + 读回核验的 SDO，约 1 秒）收尾后固件自动发一帧新报告，页面无需单独回读。「设零」把该台当前姿态设为设备零点（写 0x2531=1，仅失能态受理，运行中被拒），零点改变后设备内限位随之移动、须重认限位与机械行程；掉电是否持久待实测。|
| 使能与增益（`#/run` 下部，下发上方） | 与参数表<b>同一套行筛选</b>（默认只显示选中那台，勾「显示全部」看整条臂）。逐台 `m<id>.{enable,kp,kd}` 一行一个「下发」按钮单独下发——**改哪台发哪台**；「全选使能」只改勾选状态（不下发），「一键下发」把当前列出的所有行合成**一条**多目标策略（`targets` 上限 64，代数只 +1，而不是 18 台各 +1）。都走 `/micro_ros_do/motor_policy`话题而非参数树（原因见下「为什么使能/kp/kd 不走参数面」）。**固件不回读 kp/kd**，框里是页面最后一次下发出去的值；受理/拒绝是异步的，看事件日志的 `CONFIG_RESULT`（每台一条回执，突发时页面可能收不全，判定以生命周期列与 `diagnostics` 计数为准） |
| 重力补偿（`#/run`，下发上方） | 本页面下行的**接线开关**（ADR 0013）：打开 = 先经 `/bench/start_gravity_feedforward` 起 `gravity_feedforward_overlay`（第四个案台 unit）并确认它 `running`，**服务端**再把命令流改道到 `/robot_dynamics/*`（改 `command_streamer` 的 `output_topic`），页面同时把自己发的那一条（轨迹）切过去，由叠加节点按模型 URDF 加上 τ_g 再转发给固件；关闭 = 页面先把轨迹切回直连，服务端再**先改道回直连、确认成功才**停节点。顺序是安全边界，反过来会出现「话题已切、节点没起」的空窗。**只在 STM32 在环（顶栏 Agent 在线）时可用**——叠加节点的输出话题只有固件订阅，纯 Gazebo / 纯 MuJoCo / 主机驱动闭环三路没有接收者，按钮置灰并写明原因。模型默认取臂视图当前模型（HIL 会话是操作者手起的，页面没有别的途径知道台上是哪台），关节映射按所选模型现取（`/api/model-joints`，与臂视图同源）；增益框运行中改动即时生效（走节点的参数服务，**不重起**——重起会断命令流，撞上固件的命令超时软停）。面板同时说清四件互相独立的事：开关的**意图**、命令**实际发往**哪里（`/motor_command/status` 的 `output_topic`，服务端改的，与意图不符就是「亮着但没切过去」）、节点进程状态（`/bench/status` 的 `gravity_feedforward` 段）、τ_g 的输入流 `/joint_states` 是否新鲜。**断链不自动回退**：节点不在、或接线与开关对不上（或 `/joint_states` 不新鲜、节点跑的是另一个模型）时给红条告警 + 「立即回退」按钮（经 stop 服务切回直连并停节点），改接线必须由人按下。按钮只覆盖本页面的下发通道——`snapshot_stream.py` / `traj_play.py` 是独立命令源，同时跑就是混流（页面不依赖 `rosapi`，检测不到） |
| 下发（`#/run` 的下发条） | 把表格读成**设定值**发给 `/motor_command/setpoint`（[ADR 0014](../../../docs/adr/0014-host-side-command-pacing.md)），100 Hz 的完整快照流由主机侧 `command_streamer` 出——**页面不再持有节拍**，所以页面失焦不影响台架。「下发」= 改设定值并持续保持（旧版单发一条，30 ms 后固件就软停了）；「停止」= 发空目标列表（固件发阻尼帧、100 ms 后失能）；**清空表格不停台架**。目标里含未初始化或离线的电机时点名提示——这些台不会动且不会有报错回显（下行配置话题已随固件裁剪移除——型号绑定由固件读设备自报身份自动完成，授权/增益/限位取固件安全默认，见 `micro_ros_firmware/README.md` 下行配置通道一节）。折叠条下方那行状态显示命令**当前实际发往**哪里。话题名只有**轨迹**那条是运行期可变的，由上面的「重力补偿」开关切（`core/ros.js` 的 `setDownlinkRoute` 是唯一改写点，它同时换发布者；旧版的 `?command_topic=` / `?trajectory_topic=` URL 覆盖已随该开关删除） |
| 曲线（`#/run` 观察抽屉 / `#/diag`） | 位置/速度/力矩/电流实时折线（canvas，多电机可选、时间窗可调） |
| IMU（`#/diag`） | 两个 canvas：**3D 姿态**（四元数 → 立方体 + 三轴投影，X 前红 / Y 上绿 / Z 右蓝）与**人工地平仪**（按 roll 旋转、pitch 平移地平线，含 ±30° 俯仰刻度与机身符号）；配 Roll/Pitch/Yaw（°）、温度（`micro_ros_up/imu/temp`）、更新年龄、四元数与频率。数据源是 `micro_ros_up/imu/data`（`sensor_msgs/Imu`）；首帧前各角显示 `--`、两个 canvas 显示「等待 IMU…」，`render` 只由脏帧驱动，频率与「更新」年龄另用 1 s 定时器刷新（数据停发后年龄仍会走成「超时」，卸屏即清） |
| 事件日志 / 命令历史（观察抽屉 / `#/diag`；事件日志也在 `#/device` 抽屉） | 固件事件流与已发快照 |
| 动作库播放（`#/teach` 屏；观察抽屉与下发条已按 2026-10-14 指令移出本屏） | 列出 `web_host/action_library/` 的 `MotorTrajectory` 载荷（`GET /api/actions`，含台数/点数/时长摘要），一键把整条动作播到臂上：**就绪检查 → 武装 kp/kd（可编辑，默认 30/1；「使能」勾选=播放前自动武装，取消则跳过武装、需自行保证电机已使能）→ 把轨迹首点作为快照目标对齐起点（约 2.5 s）→ 按 ≤256 点/窗切段播放**（每窗独立 `trajectory_id`、首点 t=0、窗界复用末点保证位置连续；用固件回带的 `elapsed_ms` 在窗末交棒——与主机侧参考实现 `micro_ros_agent/tools/traj_play.py` 同口径）。「循环」开关可反复播放；「停止」发空轨迹取消（末点保持由固件负责，取消即释放归属）。**点击文件名就地展开/收起解析预览**（目标电机、时刻数/点数/时长/采样率、各台位置范围 + 时刻×位置表，均匀抽样 ≤40 行），「导入到动作库」把本地 JSON（同载荷格式）存进动作库（同名需确认覆盖）——导入控件与参数同栏，动作多了不会把入口顶下去；服务端只认 `MotorTrajectory` 载荷，文件名限单层 `[A-Za-z0-9_.-]`。播放调度在模块级：切屏不打断已下发的动作 |
| 长轨迹为什么串行 | 固件的路点数组只有 256 个槽（`MOTOR_CONTROL_MAX_TRAJECTORY_POINTS`；点数组在内核 / mailbox 双缓冲 / assembly / bridge 各存一份，加到 1024 要约 84 KB RAM，DTCM 放不下），所以长动作只能拆成多条独立轨迹。**不能背靠背发**：MCU 的 XRCE 输入流是 `MTU(512) × STREAM_HISTORY(16)` = 8 KB 的**分片**窗口，一条 256 点消息约 5.2 KB（≈11 片）放得下，三条 128 点吃到 ≈18 片 —— 落在窗口外的分片被丢，固件拼不齐即整组拒收（既不回执也不发事件），页面只能等到超时。所以一次只让一条轨迹消息在飞；代价是每个段界一次速度归零（2148 点 4 台 → 9 段）。**桥（`firmware_topic_bridge.py`）仍是"一条轨迹一条消息、超 256 整条拒绝"**：标准 `JointTrajectory` 接口这侧没有 completed 握手时序，未同步 |
| 说明（外壳的说明浮层，Esc 或点遮罩关闭） | 安全与操作注记、单位约定、话题清单。**界面本身不写句子**：长解释集中在这里与各控件的 `title=`，面板上只留数字与状态色 |

### 为什么策略和设备参数分开

`enable/kp/kd` 是运行时策略，不属于设备参数记录的六个槽位，因此通过
`/micro_ros_do/motor_policy` 话题逐台下发，并由固件按策略代数拒绝重复或过期命令。
设备限值和校准相关数据则通过 `/micro_ros_do/param_set` 与
`/micro_ros_do/system_command` 下发，状态通过
`/micro_ros_up/device_param_report` 与 `/micro_ros_up/system_status` 返回。

当前固件不提供 ROS 参数树；`ros2 param`、`rosapi` 和参数服务器不是运行时依赖。
页面上「全部参数」表与「使能与增益」抽屉行分别表达“可回读的设备参数记录”和
“运行时策略”，不再依赖参数服务器容量。

观察抽屉用原生 `<details>`：收起时内容仍留在 DOM 上（面板随屏挂载），只是不可见。

## 旧版单页界面已删除

2026-10-12：重构前的单页界面（`web/legacy.html` + `app.js` / `style.css` / `arm3d.js` /
`simulation.js`，共存期的入口是 `/legacy.html`）**已删除**。它的逻辑已逐块改写进
`web/js/`（`arm3d.js` → `js/panels/arm.js`、`simulation.js` → `js/panels/sim.js`），
共存是重构过渡期的回退手段，现在只保留新版一条线。旧文件仍在 git 历史里
（`git show 3d843d1:ros2_ws/src/web_host/web/`）。

> 2026-10-14：旧界面的 **IMU 面板**（3D 姿态 + 人工地平仪 + Roll/Pitch/Yaw/温度/
> 更新年龄/Hz）已移植为 `js/panels/imu.js`，挂在「诊断」屏
> （`docs/superpowers/specs/2026-10-09-web-host-frontend-restructure-design.md` 的诊断屏一行）。
> 数据侧由 `core/ros.js` 订阅 `micro_ros_up/imu/data` 与 `imu/temp` 写入 `store.imu` /
> `store.temp`，面板只渲染、不订阅。

## 臂视图（3D）

`web/js/panels/arm.js`（ES module）+ `web/vendor/` 里的 three 0.160.1 与 urdf-loader 0.13.1，
用浏览器**原生 import map** 解析，所以 `web/` 依旧只有静态文件、没有 Node 构建链。
vendor 那几个文件是一个整体：`urdf-loader` 顶层就 import 了 STLLoader 与
ColladaLoader（后者又拉 TGALoader），换版本要整体换，只换一个会在运行时才炸。

几何（URDF + 约 70 MB 的 STL）**不放在 `web/` 里**：`setup.py` 用 `os.walk('web')`
逐目录列 data_files，而 setuptools 的 symlink-install 处理不了**目录**符号链接
（会把整目录当文件 copy，报 `not a regular file`）。改为由 `static_server` 把
`model_dir`（模型包根，含 `models/`）挂到 `/robot_sim`——于是 URDF 里的
`package://robot_sim/...` 相对页面原样可用。`model_dir:=` 默认取
`get_package_share_directory('robot_sim')`，**可指向源码目录**
（`model_dir:=…/src/robot_sim`，改模型不必重构建）；同一个目录也是 `/api/models`
与 bench_manager 的模型枚举根。取不到时只影响臂视图与模型列表，静态页面照常，
臂视图自己的浮标会报加载失败。

**模型切换**：臂视图图例栏有模型选择器（候选 = `GET /api/models`，即有
`urdf/<名>.urdf` 的目录），切换即重载 URDF 并重取 `/api/model-joints`
（`motor_id → 关节名`）；选择存 localStorage，`?model=` 仍最高优先。映射解析顺序：
① 模型自带 `config/motor_driver.yaml`（id 与 joint 同在一处）；② 回落
`config/simulation_topology.yaml` 的 (电机名→关节) × `motor_hardware_interface`
权威驱动配置的 (电机名→id)——新模型常常先有拓扑、后补驱动配置。

页面 URL 参数（换模型 / 换接线不用改代码）：

| 参数 | 默认 | 作用 |
|---|---|---|
| `?model=` | `half_body_robot`（页内切换会记住） | 初始模型（`models/` 下的目录名）；页内可在臂视图图例栏切换，此参数优先级最高 |
| `?joints=` | `1:joint_link1,…,9:joint_link9` | 电机 id → URDF 关节名（默认对齐默认模型 `half_body_robot` 第一条链的 id N ↔ `joint_linkN`；第二条链的 id 见该模型拓扑） |

**两条已写在界面上的限制**（别把它们当固件真值）：

1. 幽灵与「跟随误差」用的是**页面自己最后一次发出去的目标**（`history[0]`），
   不是固件的设定点——固件侧的钳位、软停、整包拒收，页面看不到。轨迹在播期间
   （`target_source == 2`）目标由固件插值，页面无从得知，此时**宁可不画**。
2. 臂视图与曲线都受 `motion_states` 的 50 Hz 限制。

参数页的「未初始化」处置：固件上电会把每台 CERB 的驱动板参数与 Flash 记录
比对，不一致的那台判为未初始化（状态列 `CONFIG_CONFLICT`、无法使能）。在该行
把限值写回（写事务同时写驱动板与固件记录，读写回核验通过才生效）即可恢复一致；
写成功只表示运行期数据与固件记录已更新，持久化仍必须显式点「保存」。保存会先
等待设备侧保存事务完成，再完成 W25 双槽落盘；`save_in_progress=false` 才表示本次
保存流程收尾，设备 flash 与 W25 的跨介质掉电原子性仍需台架验证。

## 构建

ament_python 包，在 `ros2_ws/` 内构建（改了 `motor_command` 时必须一起构建——本 launch
默认起它的 `command_streamer`；改了 `bench_manager` / `bench_msgs` 也要，那是
`robot_bringup` 起的）：

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install --packages-select bench_msgs bench_manager motor_command web_host
source install/setup.bash
```

> 注意：即使使用 `--symlink-install`，`setup.py` 里 `console_scripts`
> 新增/改名后（例如 `rosbridge_shim` 刚加入时）**必须重跑一次 colcon build**，
> 入口脚本在 build 时生成到 `install/web_host/lib/web_host/`，symlink 不会自动补上，
> 否则 launch 报 `executable 'rosbridge_shim' not found`。
>
> 同理，`web/` 下**新增**文件（例如新增面板 `js/panels/*.js`、`vendor/`）也必须重跑：data_files
> 的清单在构建时求值，新文件不会自动补进 `install/web_host/share/web_host/web/`，
> 表现为页面 404 而构建"成功"。
>
> **静态资产的缓存策略**（`static_server`）：文档 / JS / CSS 一律 `Cache-Control: no-store`
> ——页面是几十个 ES module 拼起来的，只要**一个**是旧的，模块链接期就报
> `does not provide an export named …`，而且报在 import 它的那个面板上，表现成「面板整块
> 消失」；加 `no-store` 后改过的文件刷新即生效，不用清缓存。模型几何（URDF / 约 70 MB STL）
> 走 `no-cache`：每次重验证、命中返 304，不重下内容。
>
> 但 `no-store` **只对新取到的响应生效，清不掉浏览器里已有的旧副本**——那份来自只发
> `Last-Modified` 的旧版本，浏览器按启发式新鲜度认为它还新鲜，刷新时**根本不来问服务器**
> （实测访问日志里那批 js 一条请求都没有）。所以文档响应会带 `Clear-Site-Data: "cache"`
> 让浏览器清一次该源的 HTTP 缓存，用 **cookie `web_ver` 记账**：每个浏览器在每次服务启动
> 后各清一次。之所以不是"每个进程只发一次"——那个一次性标志会被任何一次非浏览器请求
> （`curl` 验证、探活）吃掉，真正要用的浏览器反而收不到。也只清一次：它会把模型几何的
> 缓存一起清掉，每次清等于每次重下 70 MB 的 STL。
>
> 因此**重启服务后浏览器刷新两次**即可回到干净状态（第一次收到清缓存指令并种下 cookie，
> 第二次全部重新取）。`Clear-Site-Data` 要求安全上下文（`http://localhost` 算，局域网 IP
> 不算）；用 IP 访问时仍需手动清一次缓存。
>
> `main.js` 逐个面板隔离挂载失败并把错误写在原位——看到那行红字，就照上面两条做。
>
> **删文件或改名**是同一个坑的镜像：`build/web_host/web/` 里会留下指向已删文件的陈旧符号
> 链接，下次构建直接报 `error: can't copy '.../build/web_host/web/<已删文件>': doesn't exist
> or not a regular file`。此时删掉该包的构建与安装目录再重建即可：
> `rm -rf build/web_host install/web_host && colcon build --symlink-install --packages-select web_host`。

## 启动

```bash
source /opt/ros/humble/setup.bash
source "$(git rev-parse --show-toplevel)/micro_ros_agent/install/setup.bash"
source "$(git rev-parse --show-toplevel)/ros2_ws/install/setup.bash"
ros2 launch web_host web_host.launch.py
```

Agent 启动由 `bench_manager` 节点经 `/bench/start_agent` 服务执行固定命令
`ros2 run micro_ros_agent micro_ros_agent serial --dev <已枚举串口>`；启动 Web 前要构建
`micro_ros_agent` 工作区并 source 其 `install/setup.bash`。`bench_manager` 的运行用户需对所选
串口有读写权限（通常加入 `dialout` 组）。页面只管理自己启动的一个 Agent；换端口前先点
「停止 Agent」。终端里另行启动且占用该串口的 Agent 不会被页面接管。

默认端口（launch 参数可覆盖）：

- WebSocket：`0.0.0.0:9090` —— `rosbridge_port:=<端口>`
- 静态页面：`0.0.0.0:8080` —— `web_port:=<端口>`

```bash
ros2 launch web_host web_host.launch.py rosbridge_port:=9091 web_port:=8081
```

启动成功的标志（前两条都要看到；bench_manager 无独立启动日志，用
`ros2 service list | grep bench` 确认）：

```
[rosbridge_websocket]: Rosbridge WebSocket server started on port 9090
[web_static_server]: Serving .../install/web_host/share/web_host/web at http://0.0.0.0:8080
```

浏览器打开 **http://<主机IP>:8080**（本机即 http://localhost:8080）。

### 从页面启动仿真

「仿真」屏（`#/sim`）会读取已安装模型拓扑、当前 SocketCAN 接口和串口设备。选择**仿真后端**、控制端、传输、模型与端点后点「启动仿真」；日志显示 `Complete arm simulation` 后再使用对应控制端。停止按钮只结束本页面启动的仿真 launch 进程组。

**仿真后端**（`backend:=`，透传给 `simulation.launch.py`）：

| 后端 | 被控对象 | 面板上的差别 |
| --- | --- | --- |
| `Gazebo`（默认） | `gz sim` + `gz_ros2_control` 的 effort 控制器 | 无 |
| `MuJoCo` | 模型自带的 `mujoco/<模型名>.xml` + 自写力矩直通桥 | 模型下拉只列**同时有** `mujoco/<名>.xml` 与 `urdf/<名>.standalone.urdf` 的模型；选别的会被后端拒绝 |

两个后端的话题契约相同（吃 `/arm_effort_controller/commands`、出 `/joint_states`），传输与控制端的规则完全一样。MuJoCo 的 viewer 窗口开在**起仿真进程的那台机器**上（不是打开浏览器的机器），所以从面板起 MuJoCo 时 bench_manager 必须有可用的显示会话（`DISPLAY`）；Wayland 下 launch 会强制 pyGLFW 走 X11 变体，否则窗口自旋、`/joint_states` 一帧不发（桥启动时会打印实际平台）。

| 传输 / 控制端 | 页面端点 |
|---|---|
| `vcan` + 主机驱动或外部控制器 | 填 vcan 接口名；接口由仿真 launch 创建和清理 |
| `SocketCAN` + 主机驱动 | 选择两个已置为 UP 的不同接口，分别给模拟器与驱动端 |
| `SocketCAN` + 外部控制器或 STM32 HIL | 选择一个已置为 UP 的接口 |
| `SLCAN` + 主机驱动 | 选择两只不同串口适配器；bench_manager 用户需有读写权限 |
| `SLCAN` + 外部控制器或 STM32 HIL | 选择模拟器侧 CANable 串口 |

SocketCAN 与 SLCAN 启动前，页面要求确认物理 CAN 总线只连接仿真器和控制端、没有真机电机；该确认也由后端检查。STM32 HIL 不支持 vcan：先启动仿真，再在顶栏选择 STM32 USB CDC 串口并点「启动并连接」启动 micro-ROS Agent。仿真 launch 会复用当前 WebHost，不会再启动一份占用 8080/9090 的 WebHost 或 Agent。

WebHost 进程的 ROS 环境需已 source `ros2_ws/install/setup.bash`，模型及依赖包已构建。前端静态文件（`web/index.html`、`web/js/**`、`web/css/**`）改动后要按上方构建步骤重新构建 `web_host`，否则安装目录的页面不会包含它；**新增**文件还必须重跑 colcon（见「构建」的注意）。

停止：前台 launch 直接 `Ctrl+C`，两个节点一起退出。

## 与 micro-ROS agent 的关系

web_host 只负责 ROS 2 ⇄ 浏览器；固件（STM32 控制板）的话题要另起
`micro_ros_agent`（独立工作区，见 `../../micro_ros_agent/README.md`）才能在
domain 0 看到。两者无启动顺序要求，页面上话题无数据时先确认 agent 已连上板子：

```bash
# 另一个终端；ttyACM0=控制板 USB CDC（VID 0483）
source /opt/ros/humble/setup.bash
source "$(git rev-parse --show-toplevel)/micro_ros_agent/install/setup.bash"
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

agent 会话建立后，`ros2 topic list` 应能看到 `/micro_ros_up/motor_topology`、
`/micro_ros_up/motor_status_array`、`/micro_ros_up/imu/data` 等固件话题，
页面左侧监控区即有数据。需要 `/dev/ttyACM*` 写权限（用户在 `dialout` 组）。

## 验证

```bash
ss -ltn | grep -E ':(9090|8080)'          # 两端口在监听
ros2 topic list                            # 能看到 /micro_ros_up/* 说明 agent 也正常
curl -sI http://localhost:8080 | head -1  # 期望 HTTP/1.0 200 OK
# 臂视图的三个必需资源（缺任何一个页面都会在浮标上报加载失败）
for f in index.html js/panels/arm.js vendor/three.module.min.js \
         robot_sim/models/half_body_robot/urdf/half_body_robot.urdf; do
  curl -o /dev/null -s -w "%{http_code} $f\n" "http://localhost:8080/$f"
done
```
