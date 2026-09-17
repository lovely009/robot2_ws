# web_host

rosbridge Web 上位机：一个 launch 同时起两个节点——

| 节点 | 可执行 | 作用 |
|---|---|---|
| `rosbridge_websocket` | `rosbridge_shim` | 官方 rosbridge 的**启动垫片**，先给 `rosbridge_library` 补上有界字符串原语（`string<31>` / `string<=31`，见 `web_host/rosbridge_shim.py` 模块注释），再 runpy 运行官方入口；解决 `can_bridge_msgs` 的 `MotorTopology` 等含 `string<=31` 字段时 rosbridge 2.0.7 序列化报 `inst is not a ROS Message, but a str` 的问题 |
| `web_static_server` | `static_server` | 纯标准库 `ThreadingHTTPServer` 托管 `web/`（`index.html` / `app.js` / `style.css` / `roslib.min.js`），无 Node 等构建依赖 |

前端打开页面后按 `location.hostname:9090` 自动拼 WebSocket 地址（见 `web/app.js`），
所以从别的机器访问时，浏览器地址栏用主机 IP，rosbridge 也会连到同一个 IP。

页面分五个 Tab（纯前端，无 Node 构建依赖，`web/` 下 `index.html` / `app.js` /
`style.css` / `roslib.min.js`）：

| Tab | 功能 |
|---|---|
| 参数 | 逐电机 `m<id>.{enable,kp,kd,pos_min,pos_max,vel_limit,torque_limit}` 读写 + 设备参数「回读」。**参数以驱动板为准**、**单位是工程单位**（位置 度、速度 度/秒、力矩 安培，换算由固件在参数边界做）；「来源」列标出该行取自驱动板 / 记录（离线回落）/ 未核对，「状态」列显示生命周期。回读是严格一问一答的 SDO 事务（约 1 秒），完成判据是只读的 `system.refresh_seq`（每次收尾 +1），结论取自 `system.last_error`（0=一致 / 40=与固件记录不一致 → 该台未初始化 / 41=回读不成功），随后重载表格。「设零」把该台当前姿态设为设备零点（写 0x2531=1，仅失能态受理，运行中被拒），零点改变后设备内限位随之移动、须重认限位与机械行程；掉电是否持久待实测 |
| 监控 | IMU 数值 + 3D 姿态/人工地平仪、电机状态表、命令历史 |
| 控制 | 命令快照单发/100Hz 连发（下行配置话题已随固件裁剪移除——型号绑定由固件读设备自报身份自动完成，授权/增益/限位取固件安全默认，见 `micro_ros_firmware/README.md` 下行配置通道一节） |
| 曲线 | 位置/速度/力矩/电流实时折线（canvas，多电机可选、时间窗可调） |
| 示教 | 点位序列（记录当前命令 / 单步 / 循环播放 / 导入导出 JSON） |

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

agent 会话建立后，`ros2 topic list` 应能看到 `/micro_ros/motor_topology`、
`/micro_ros/motor_status_array`、`/micro_ros/imu/data` 等固件话题，
页面左侧监控区即有数据。需要 `/dev/ttyACM*` 写权限（用户在 `dialout` 组）。

## 验证

```bash
ss -ltn | grep -E ':(9090|8080)'          # 两端口在监听
ros2 topic list                            # 能看到 /micro_ros/* 说明 agent 也正常
curl -sI http://localhost:8080 | head -1  # 期望 HTTP/1.0 200 OK
```
