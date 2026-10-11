# web_host

rosbridge Web 上位机：一个 launch 同时起两个节点——

| 节点 | 可执行 | 作用 |
|---|---|---|
| `rosbridge_websocket` | `rosbridge_shim` | 官方 rosbridge 的**启动垫片**，先给 `rosbridge_library` 补上有界字符串原语（`string<31>` / `string<=31`，见 `web_host/rosbridge_shim.py` 模块注释），再 runpy 运行官方入口；解决 `can_bridge_msgs` 的 `MotorTopology` 等含 `string<=31` 字段时 rosbridge 2.0.7 序列化报 `inst is not a ROS Message, but a str` 的问题 |
| `web_static_server` | `static_server` | 纯标准库 `ThreadingHTTPServer` 托管 `web/`（`index.html` / `app.js` / `style.css` / `roslib.min.js`），无 Node 等构建依赖 |

前端打开页面后按 `location.hostname:9090` 自动拼 WebSocket 地址（见 `web/app.js`），
所以从别的机器访问时，浏览器地址栏用主机 IP，rosbridge 也会连到同一个 IP。

页面是**单页、以臂为中心**的骨架（纯前端，无 Node 构建依赖）。臂恒在中间且是
最大一块；左栏是关节索引，点一行（或点臂上的关节小球）→ 臂上高亮该关节，右栏
就地展开它的实时读数。参数表在臂的**下方**整宽一行（「全部参数」），再往下是
常驻下发条与底部抽屉；「使能与增益」是抽屉里的第一行（策略面只写不回读，与参数
表分开表达）。原来的五个 Tab 拆成「常驻区」+「底部抽屉」。

| 区域 | 功能 |
|---|---|
| 顶栏连接区 | rosbridge WebSocket 地址位于控制板串口选择前；地址自动保存在浏览器本地，页面打开或刷新时自动连接/重连 rosbridge。主机侧枚举 `/dev/serial/by-id/` 的稳定串口路径，并以 `/dev/ttyACM*`、`/dev/ttyUSB*` 补缺。「启动并连接」只启动或复用所选端口的 micro-ROS Agent；「断开」只停止本页面管理的 Agent 子进程，两者都不改变 rosbridge 连接。Agent 状态表示进程状态，不代表固件客户端已建立会话 |
| 仿真（底部抽屉首栏） | 从页面启动或停止 `robot_arm_gazebo/simulation.launch.py`；按模型拓扑列出可用模型，并支持 `vcan`、`SocketCAN`、`SLCAN`。控制端可选主机驱动闭环、外部控制器或 STM32 固件 HIL；设备列表来自主机当前 CAN 接口与串口。页面展示 launch 子进程状态和最近日志，进程存活不等于 Gazebo 已就绪 |
| 臂视图（中栏） | `robot_arm_gazebo` 的 URDF 经 three.js 实时渲染，关节角来自固件 `micro_ros_up/motion_states`（**不是** `/joint_states`：页面本来就订阅它；而 `/joint_states` 要额外起桥，且在 Gazebo 台架上会被 gz 自己的 broadcaster 抢同名话题）。关节小球着色 = 生命周期九态；半透明幽灵 = 页面自己下发的目标，见下「臂视图（3D）」一节。拖拽旋转 / 滚轮缩放 / 点小球选中 |
| 选中关节（右栏） | 位置/速度/力矩/电流/温度/跟随误差/命令源/故障/测量年龄。点左栏或臂上的小球切换 |
| 全部参数（臂下方整宽一行） | **以固件记录为准**，驱动板向记录对齐：位置 **rad**、速度 **rad/s**、力矩 **A**（设备对象 `0x2538` 存的就是电流；位置/速度在固件参数边界是直通，只有力矩过一次 A↔N·m 换算，见决策日志 §6.4）。两行对照取自只读话题 `micro_ros_up/device_param_report`（固件记录列 / 驱动板回读列，同槽位同单位），`dev_init_state` 5（无记录）时固件列整行显示「无记录」。**写 ≠ 保存**——写只改设备 RAM 与固件记录（写后立即重判），要持久化必须再点「保存」（发 `0x2539` + 落 W25），未保存的台标黄。「读取」= 发 `system_command(REPORT)`，固件把**当前参数视图**立刻回一帧 `device_param_report`（不触发 SDO、不访问电机）；电机在线且已核对时显示驱动板真值，不在线/未核对（`dev_*` 为 NaN）时显示的就是板内 W25 参数表；数值用 7 位有效数字显示（内核按 float32 存，原始值会拖出 0.20000000298023224 这类尾巴）。总览条一眼给出「已核对/未初始化/离线」台数与原因（40=与记录不一致、41=回读不成功、42=本机无记录），并**逐台列出不一致的具体项**（位置下限/位置上限/最大转速/峰值电流，来自固件 DEV_INIT 事件的槽位位图）；状态列悬停同样显示该明细；行内「来源」列标出固件记录/驱动板回读/未核对，「状态」列显示生命周期；位置与力矩输入框悬停显示设备端原值（cnt / mA，位置按 65536 cnt/360° 折算）。逐项 `m<id>.{pos_min,pos_max,vel_limit,torque_limit,limit_protect}` 与方向均可写；写事务（写 + 读回核验的 SDO，约 1 秒）收尾后固件自动发一帧新报告，页面无需单独回读。「设零」把该台当前姿态设为设备零点（写 0x2531=1，仅失能态受理，运行中被拒），零点改变后设备内限位随之移动、须重认限位与机械行程；掉电是否持久待实测。|
| 使能与增益（底部抽屉首行） | 与参数表<b>同一套行筛选</b>（默认只显示选中那台，勾「显示全部」看整条臂）。逐台 `m<id>.{enable,kp,kd}` 一行一个「策略」按钮单独下发——**改哪台发哪台**，走 `/micro_ros_do/motor_policy`话题而非参数树（原因见下「为什么使能/kp/kd 不走参数面」）。**固件不回读 kp/kd**，框里是页面最后一次下发出去的值；受理/拒绝是异步的，看事件日志的 `CONFIG_RESULT` |
| 下发（常驻条） | 命令快照单发/100Hz 连发（目标里含未初始化或离线的电机时点名提示——这些台不会动且不会有报错回显）（下行配置话题已随固件裁剪移除——型号绑定由固件读设备自报身份自动完成，授权/增益/限位取固件安全默认，见 `micro_ros_firmware/README.md` 下行配置通道一节） |
| 曲线（抽屉） | 位置/速度/力矩/电流实时折线（canvas，多电机可选、时间窗可调） |
| 事件日志 / 命令历史（抽屉） | 固件事件流与已发快照 |
| 示教（抽屉） | 点位序列（记录当前命令 / 单步 / 循环播放 / 导入导出 JSON）。导入接受两种 JSON：本页导出的**行数组**，或 `MotorTrajectory` **话题载荷**（`{points:[...]}`，按 `time_from_start_ms` 合成行、wait 取相邻时刻差；`torque` 兼容读作 `effort`）。播放经可靠轨迹话题交 MCU 托管；**单条轨迹上限 256 个路点**（固件硬容量，见下），更长的示教自动切成多条**独立轨迹串行下发**——每条 ≤256 点、段界重复一行（上段终点即下段起点，位置连续），一条播完（`trajectory_state.completed` 回执）才发下一条；段界速度必然归零（固件端点切矢规则），所以段尾挑在运动最安静的行上（2026-10-08；此前版本是"超过总量直接拒绝"）。完成后末点保持，点击暂停发送空轨迹取消 |
| 长轨迹为什么串行 | 固件的路点数组只有 256 个槽（`MOTOR_CONTROL_MAX_TRAJECTORY_POINTS`；点数组在内核 / mailbox 双缓冲 / assembly / bridge 各存一份，加到 1024 要约 84 KB RAM，DTCM 放不下），所以长示教只能拆成多条独立轨迹。**不能背靠背发**：MCU 的 XRCE 输入流是 `MTU(512) × STREAM_HISTORY(16)` = 8 KB 的**分片**窗口，一条 256 点消息约 5.2 KB（≈11 片）放得下，三条 128 点吃到 ≈18 片 —— 落在窗口外的分片被丢，固件拼不齐即整组拒收（既不回执也不发事件），页面只能等到超时。所以一次只让一条轨迹消息在飞；代价是每个段界一次速度归零（2148 点 4 台 → 9 段）。**桥（`firmware_topic_bridge.py`）仍是"一条轨迹一条消息、超 256 整条拒绝"**：标准 `JointTrajectory` 接口这侧没有 completed 握手时序，未同步 |
| IMU（抽屉） | IMU 数值 + 3D 姿态/人工地平仪 |
| 说明（抽屉） | 安全与操作注记、单位约定、话题清单。**界面本身不写句子**：长解释集中在这里与各控件的 `title=`，面板上只留数字与状态色 |

### 为什么策略和设备参数分开

`enable/kp/kd` 是运行时策略，不属于设备参数记录的六个槽位，因此通过
`/micro_ros_do/motor_policy` 话题逐台下发，并由固件按策略代数拒绝重复或过期命令。
设备限值和校准相关数据则通过 `/micro_ros_do/param_set` 与
`/micro_ros_do/system_command` 下发，状态通过
`/micro_ros_up/device_param_report` 与 `/micro_ros_up/system_status` 返回。

当前固件不提供 ROS 参数树；`ros2 param`、`rosapi` 和参数服务器不是运行时依赖。
页面上「全部参数」表与「使能与增益」抽屉行分别表达“可回读的设备参数记录”和
“运行时策略”，不再依赖参数服务器容量。

抽屉用原生 `<details>`：收起时内容仍留在 DOM 上——页面的定时渲染一直在往里写。

## 两版界面并存

URL 根 `/` 是重构后的新版（外壳 / 屏 / 面板三层，见
`docs/adr/0010-web-host-frontend-layering-without-build-chain.md` 与同期的重构设计）；
`/legacy.html` 是重构前的单页界面（以臂为中心 + 8 抽屉，含仿真启动器与 Agent 端口
选择器）。两版**共用同一份静态服务与 `/api` 后端**——Agent 与仿真子进程是同一个，
不存在两套状态。

旧版页面加载的 `web/app.js`、`web/style.css`、`web/arm3d.js`、`web/simulation.js` 是它
专用的资源（新版已把这些逻辑搬进 `web/js/`），**不要当成重构后的死代码删掉**；
`test/test_web_assets.py::test_legacy_assets_exist` 兜住这条。它的臂视图几何同样走
`/robot_arm_gazebo` 挂载（重构前的 `web/robot_arm_gazebo` 目录符号链接已退役）。

## 臂视图（3D）

`web/arm3d.js`（ES module）+ `web/vendor/` 里的 three 0.160.1 与 urdf-loader 0.13.1，
用浏览器**原生 import map** 解析，所以 `web/` 依旧只有静态文件、没有 Node 构建链。
vendor 那几个文件是一个整体：`urdf-loader` 顶层就 import 了 STLLoader 与
ColladaLoader（后者又拉 TGALoader），换版本要整体换，只换一个会在运行时才炸。

几何（URDF + 约 20 MB 的 STL）**不放在 `web/` 里**：`setup.py` 用 `os.walk('web')`
逐目录列 data_files，而 setuptools 的 symlink-install 处理不了**目录**符号链接
（会把整目录当文件 copy，报 `not a regular file`）。改为由 `static_server` 把
`robot_arm_gazebo` 的 share 目录挂到 `/robot_arm_gazebo`——于是 URDF 里的
`package://robot_arm_gazebo/...` 相对页面原样可用。挂载点由
`web_host.launch.py` 的 `model_dir:=` 控制（默认取
`get_package_share_directory('robot_arm_gazebo')`）；取不到时只影响臂视图，
静态页面照常，臂视图自己的浮标会报加载失败。

页面 URL 参数（换模型 / 换接线不用改代码）：

| 参数 | 默认 | 作用 |
|---|---|---|
| `?model=` | `chest_upper_00` | `robot_arm_gazebo/models/` 下的模型目录名 |
| `?joints=` | `1:joint_link1,…,9:joint_link9` | 电机 id → URDF 关节名（默认对齐 `chest_upper_00` 的 id N ↔ `joint_linkN`） |

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

ament_python 包，在 `ros2_ws/` 内构建：

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install --packages-select web_host
source install/setup.bash
```

> 注意：即使使用 `--symlink-install`，`setup.py` 里 `console_scripts`
> 新增/改名后（例如 `rosbridge_shim` 刚加入时）**必须重跑一次 colcon build**，
> 入口脚本在 build 时生成到 `install/web_host/lib/web_host/`，symlink 不会自动补上，
> 否则 launch 报 `executable 'rosbridge_shim' not found`。
>
> 同理，`web/` 下**新增**文件（例如 `arm3d.js`、`vendor/`）也必须重跑：data_files
> 的清单在构建时求值，新文件不会自动补进 `install/web_host/share/web_host/web/`，
> 表现为页面 404 而构建"成功"。

## 启动

```bash
source /opt/ros/humble/setup.bash
source /home/ly/robot_ws/micro_ros_agent/install/setup.bash
source /home/ly/robot_ws/ros2_ws/install/setup.bash
ros2 launch web_host web_host.launch.py
```

Agent 自动启动由 `web_static_server` 子进程执行固定命令
`ros2 run micro_ros_agent micro_ros_agent serial --dev <已枚举串口>`；启动 Web 前要构建
`micro_ros_agent` 工作区并 source 其 `install/setup.bash`。`web_host` 的运行用户需对所选
串口有读写权限（通常加入 `dialout` 组）。页面只管理自己启动的一个 Agent；换端口前先点
「停止 Agent」。终端里另行启动且占用该串口的 Agent 不会被页面接管。

默认端口（launch 参数可覆盖）：

- WebSocket：`0.0.0.0:9090` —— `rosbridge_port:=<端口>`
- 静态页面：`0.0.0.0:8080` —— `web_port:=<端口>`

```bash
ros2 launch web_host web_host.launch.py rosbridge_port:=9091 web_port:=8081
```

启动成功的标志（两条都要看到）：

```
[rosbridge_websocket]: Rosbridge WebSocket server started on port 9090
[web_static_server]: Serving .../install/web_host/share/web_host/web at http://0.0.0.0:8080
```

浏览器打开 **http://<主机IP>:8080**（本机即 http://localhost:8080）。

### 从页面启动仿真

底部抽屉的「仿真」栏会读取已安装模型拓扑、当前 SocketCAN 接口和串口设备。选择控制端、传输、模型与端点后点「启动仿真」；日志显示 `Complete arm simulation` 后再使用对应控制端。停止按钮只结束本页面启动的仿真 launch 进程组。

| 传输 / 控制端 | 页面端点 |
|---|---|
| `vcan` + 主机驱动或外部控制器 | 填 vcan 接口名；接口由仿真 launch 创建和清理 |
| `SocketCAN` + 主机驱动 | 选择两个已置为 UP 的不同接口，分别给模拟器与驱动端 |
| `SocketCAN` + 外部控制器或 STM32 HIL | 选择一个已置为 UP 的接口 |
| `SLCAN` + 主机驱动 | 选择两只不同串口适配器；web_host 用户需有读写权限 |
| `SLCAN` + 外部控制器或 STM32 HIL | 选择模拟器侧 CANable 串口 |

SocketCAN 与 SLCAN 启动前，页面要求确认物理 CAN 总线只连接仿真器和控制端、没有真机电机；该确认也由后端检查。STM32 HIL 不支持 vcan：先启动仿真，再在顶栏选择 STM32 USB CDC 串口并点「启动并连接」启动 micro-ROS Agent。仿真 launch 会复用当前 WebHost，不会再启动一份占用 8080/9090 的 WebHost 或 Agent。

WebHost 进程的 ROS 环境需已 source `ros2_ws/install/setup.bash`，模型及依赖包已构建。新增的 `simulation.js` 属于静态文件，改动后按上方构建步骤重新构建 `web_host`，否则安装目录的页面不会包含它。

停止：前台 launch 直接 `Ctrl+C`，两个节点一起退出。

## 与 micro-ROS agent 的关系

web_host 只负责 ROS 2 ⇄ 浏览器；固件（STM32 控制板）的话题要另起
`micro_ros_agent`（独立工作区，见 `../../micro_ros_agent/README.md`）才能在
domain 0 看到。两者无启动顺序要求，页面上话题无数据时先确认 agent 已连上板子：

```bash
# 另一个终端；ttyACM0=控制板 USB CDC（VID 0483）
source /opt/ros/humble/setup.bash
source /home/ly/robot_ws/micro_ros_agent/install/setup.bash
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
for f in arm3d.js vendor/three.module.min.js \
         robot_arm_gazebo/models/chest_upper_00/urdf/chest_upper_00.urdf; do
  curl -o /dev/null -s -w "%{http_code} $f\n" "http://localhost:8080/$f"
done
```
