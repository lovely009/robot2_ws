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
就地展开它的实时读数。参数与策略在臂的**下方**并排两栏（左「设备参数」/ 右
「使能与增益」），再往下是常驻下发条与底部抽屉。原来的五个 Tab 拆成
「常驻区」+「底部抽屉」。

| 区域 | 功能 |
|---|---|
| 臂视图（中栏） | `robot_arm_gazebo` 的 URDF 经 three.js 实时渲染，关节角来自固件 `micro_ros_up/motion_states`（**不是** `/joint_states`：页面本来就订阅它；而 `/joint_states` 要额外起桥，且在 Gazebo 台架上会被 gz 自己的 broadcaster 抢同名话题）。关节小球着色 = 生命周期九态；半透明幽灵 = 页面自己下发的目标，见下「臂视图（3D）」一节。拖拽旋转 / 滚轮缩放 / 点小球选中 |
| 选中关节（右栏） | 位置/速度/力矩/电流/温度/跟随误差/命令源/故障/测量年龄。点左栏或臂上的小球切换 |
| 设备参数（下方左栏） | **以固件记录为准**，驱动板向记录对齐：位置 **rad**、速度 **rad/s**、力矩 **A**（设备对象 `0x2538` 存的就是电流；位置/速度在固件参数边界是直通，只有力矩过一次 A↔N·m 换算，见决策日志 §6.4）。两行对照取自只读话题 `micro_ros_up/device_param_report`（固件记录列 / 驱动板回读列，同槽位同单位），`dev_init_state` 5（无记录）时固件列整行显示「无记录」。**写 ≠ 保存**——写只改设备 RAM 与固件记录（写后立即重判），要持久化必须再点「保存」（发 `0x2539` + 落 W25），未保存的台标黄。「读取」= **先逐台回读驱动板**（在线台，设备真值）+ 再读参数表；数值用 7 位有效数字显示（内核按 float32 存，原始值会拖出 0.20000000298023224 这类尾巴）。总览条一眼给出「已核对/未初始化/离线」台数与原因（40=与记录不一致、41=回读不成功），并**逐台列出不一致的具体项**（位置下限/位置上限/最大转速/峰值电流，来自固件 DEV_INIT 事件的槽位位图）；状态列悬停同样显示该明细；行内「来源」列标出驱动板/记录/未核对，「状态」列显示生命周期；位置与力矩输入框悬停显示设备端原值（cnt / mA，位置按 65536 cnt/360° 折算）。逐项 `m<id>.{pos_min,pos_max,vel_limit,torque_limit,limit_protect}` 读写 + 逐台「回读」。回读是严格一问一答的 SDO 事务（约 1 秒），完成判据是只读的 `system.refresh_seq`（每次收尾 +1），结论取自 `system.last_error`（0=一致 / 40=与固件记录不一致 → 该台未初始化 / 41=回读不成功），随后重载表格。「设零」把该台当前姿态设为设备零点（写 0x2531=1，仅失能态受理，运行中被拒），零点改变后设备内限位随之移动、须重认限位与机械行程；掉电是否持久待实测。⚠️ **「两行对照（固件记录 / 驱动板回读）」与「未保存标黄」是 P2（web 大修）要实现的行为契约**：话题 `micro_ros_up/device_param_report` 已在固件侧实现，页面重写未开工、**当前页面尚未接线** |
| 使能与增益（下方右栏） | 与左栏<b>同一套行筛选</b>（默认只显示选中那台，勾「显示全部」看整条臂）。逐台 `m<id>.{enable,kp,kd}` 一行一个「策略」按钮单独下发——**改哪台发哪台**，走 `/micro_ros_do/motor_policy`话题而非参数树（原因见下「为什么使能/kp/kd 不走参数面」）。**固件不回读 kp/kd**，框里是页面最后一次下发出去的值；受理/拒绝是异步的，看事件日志的 `CONFIG_RESULT` |
| 下发（常驻条） | 命令快照单发/100Hz 连发（目标里含未初始化或离线的电机时点名提示——这些台不会动且不会有报错回显）（下行配置话题已随固件裁剪移除——型号绑定由固件读设备自报身份自动完成，授权/增益/限位取固件安全默认，见 `micro_ros_firmware/README.md` 下行配置通道一节） |
| 曲线（抽屉） | 位置/速度/力矩/电流实时折线（canvas，多电机可选、时间窗可调） |
| 事件日志 / 命令历史（抽屉） | 固件事件流与已发快照 |
| 示教（抽屉） | 点位序列（记录当前命令 / 单步 / 循环播放 / 导入导出 JSON）。导入接受两种 JSON：本页导出的**行数组**，或 `MotorTrajectory` **话题载荷**（`{points:[...]}`，按 `time_from_start_ms` 合成行、wait 取相邻时刻差；`torque` 兼容读作 `effort`）。播放把整表打成**一条**轨迹交 MCU 托管——**上限 256 个路点，且是总点数**（4 台电机时只够 64 行）；固件对超限轨迹是静默丢弃（ROS 桥入口 return，无事件无计数），所以页面在下发前自查并拒绝，不再报"已托管" |
| IMU（抽屉） | IMU 数值 + 3D 姿态/人工地平仪 |
| 说明（抽屉） | 安全与操作注记、单位约定、话题清单。**界面本身不写句子**：长解释集中在这里与各控件的 `title=`，面板上只留数字与状态色 |

### 为什么使能 / kp / kd 不走参数面

`enable/kp/kd` 确实是固件 `param_field_t` 的头三项，但**没进 `kParamMotorFields[]`
注册表**（每台只注册 5 项限值类），而参数面总容量 `PARAM_BRIDGE_MAX_PARAMS = 64`——
再加 3 项会把可支持的台数从 ~11 压到 ~8。所以这三项走 `/micro_ros_do/motor_policy`
话题（每条策略只带一台，稀疏语义），且带「代数严格前进」的防复权门。这也是界面上
把它和设备限值分成两栏的原因：一边可回读、以固件记录为准（驱动板向记录对齐），一边只写、是页面自己的账。

抽屉用原生 `<details>`：收起时内容仍留在 DOM 上——页面的定时渲染一直在往里写。

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
| `?model=` | `zhoubuguanjie_youbi_001` | `robot_arm_gazebo/models/` 下的模型目录名 |
| `?joints=` | `1:joint_link1,2:joint_link2,3:joint_link3,4:joint_link4` | 电机 id → URDF 关节名 |

**两条已写在界面上的限制**（别把它们当固件真值）：

1. 幽灵与「跟随误差」用的是**页面自己最后一次发出去的目标**（`history[0]`），
   不是固件的设定点——固件侧的钳位、软停、整包拒收，页面看不到。轨迹在播期间
   （`target_source == 2`）目标由固件插值，页面无从得知，此时**宁可不画**。
2. 臂视图与曲线都受 `motion_states` 的 50 Hz 限制。

参数页的「未初始化」处置：固件上电会把每台 CERB 的驱动板参数与 Flash 记录
比对，不一致的那台判为未初始化（状态列 `CONFIG_CONFLICT`、无法使能）。在该行
把限值写回（写事务同时写驱动板与固件记录，读写回核验通过才生效）即可恢复一致；
写成功后固件自动落盘，无需再点「保存到 W25」（该按钮仍用于 kp/kd 等本地参数的
显式落盘）。

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
source /home/ly/robot_ws/ros2_ws/install/setup.bash
ros2 launch web_host web_host.launch.py
```

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
         robot_arm_gazebo/models/zhoubuguanjie_youbi_001/urdf/zhoubuguanjie_youbi_001.urdf; do
  curl -o /dev/null -s -w "%{http_code} $f\n" "http://localhost:8080/$f"
done
```
