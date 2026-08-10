# upper_body_canfd_gazebo

14 自由度上半身双臂的机器人模型、Gazebo Fortress 集成、真机 bring-up 和 ROS 参考动作工具。

## 机器人拓扑

左右臂各 7 自由度、各使用一条独立 CAN-FD 总线：

| 关节 | 电机类型 | 总线内 ID |
|---|---|---:|
| `shoulder_yaw` | RP | 1 |
| `shoulder_roll` | RP | 2 |
| `shoulder_pitch` | RP | 3 |
| `elbow_pitch` | RP | 4 |
| `wrist_yaw` | 25/30 mm | 1 |
| `wrist_pitch` | 25/30 mm | 2 |
| `wrist_roll` | 25/30 mm | 3 |

仿真默认使用 `vcan0/vcan1`，真机默认使用 `can0/can1`。权威电机映射位于
`motor_hardware_interface/config/motor_canfd.yaml`。

## Launch 入口

| 文件 | 启动内容 | 用途 |
|---|---|---|
| `gazebo.launch.py` | Gazebo、模型、`/clock`、Gazebo controllers | 模型开发 |
| `model_sim.launch.py` | `gazebo.launch.py` + C++ 电机模拟器 | 电机模型调试 |
| `driver_loopback.launch.py` | 模型、模拟器、真实硬件插件、position controller | 完整仿真回环 |
| `hardware.launch.py` | 硬件插件、position controller、state publisher | 真机 bring-up |

完整仿真包含两个 controller manager：

- `/controller_manager`：Gazebo 侧 `effort_controller`；
- `/driver/controller_manager`：CAN-FD 驱动侧 `position_controller`。

启动顺序由 launch event handler 编排，不使用固定延时。

## 全链路仿真

```text
/driver/position_controller
  -> motor_hardware_interface
  -> motor_canfd_drive
  -> vcan0 / vcan1
  -> motor_canfd_sim
  -> Gazebo effort_controller
  -> /joint_states
  -> CAN-FD feedback
```

准备并启动：

```bash
bash src/upper_body_canfd_gazebo/scripts/setup_vcan.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch upper_body_canfd_gazebo driver_loopback.launch.py
```

无图形环境可添加 `gui:=false`；渲染兼容问题可通过 `render_engine:=ogre` 覆盖。

## 参考动作

动作播放器读取 `/driver/joint_states`，向
`/driver/position_controller/commands` 发布 14 关节位置目标，不直接访问 SocketCAN。

```bash
ros2 run upper_body_canfd_gazebo ros_actions --list
ros2 run upper_body_canfd_gazebo ros_actions --action 8
ros2 run upper_body_canfd_gazebo ros_actions --speed-scale 0.5
```

关键帧使用 `s(u) = 3u² - 2u³` smoothstep 插值，默认发布频率为 100 Hz，段首尾速度为零。

## 真机

```bash
ros2 launch upper_body_canfd_gazebo hardware.launch.py \
  left_interface:=can0 right_interface:=can1
```

启动前要求：

1. 两条总线配置为仲裁段 1 Mbit/s、数据段 5 Mbit/s，并启用 CAN-FD BRS；
2. 按单关节到单臂、再到双臂的顺序验证；
3. 确认零位、方向、软硬限位、增益、扭矩常数和急停；
4. 确认反馈超时与停用流程能使电机安全失能。

## 主要文件

```text
config/controllers.yaml             Gazebo effort controller
config/driver_controllers.yaml      驱动侧 position controller
launch/                             仿真与真机入口
urdf/upper_body.urdf.xacro          机器人模型和硬件后端选择
worlds/upper_body_world.sdf         Gazebo 世界
upper_body_canfd_gazebo/
  motion_library.py                 参考动作与关键帧
  ros_action_player.py              ROS position command 播放器
scripts/setup_vcan.sh               创建 vcan0/vcan1
```

## 构建与测试

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to upper_body_canfd_gazebo
colcon test --packages-select upper_body_canfd_gazebo
colcon test-result --verbose
```

测试覆盖动作库、命令顺序和 launch 静态结构；测试不得启动 Gazebo 或真实硬件。
不要把上面的命令当作真机首测命令。真机首次通电还需要逐电机 ID、方向、零位、限位、急停和扭矩限制验证。

## 11. 关节 / 总线精确映射

| 总线 | 电机协议地址 | ROS/Gazebo 关节 |
|---|---|---|
| vcan0 左臂 | RP ID 1 / slot 1 | `left_shoulder_yaw_joint` |
| vcan0 左臂 | RP ID 2 / slot 2 | `left_shoulder_roll_joint` |
| vcan0 左臂 | RP ID 3 / slot 3 | `left_shoulder_pitch_joint` |
| vcan0 左臂 | RP ID 4 / slot 4 | `left_elbow_pitch_joint` |
| vcan0 左臂 | 25/30 N=1 | `left_wrist_yaw_joint` |
| vcan0 左臂 | 25/30 N=2 | `left_wrist_pitch_joint` |
| vcan0 左臂 | 25/30 N=3 | `left_wrist_roll_joint` |
| vcan1 右臂 | RP ID 1 / slot 1 | `right_shoulder_yaw_joint` |
| vcan1 右臂 | RP ID 2 / slot 2 | `right_shoulder_roll_joint` |
| vcan1 右臂 | RP ID 3 / slot 3 | `right_shoulder_pitch_joint` |
| vcan1 右臂 | RP ID 4 / slot 4 | `right_elbow_pitch_joint` |
| vcan1 右臂 | 25/30 N=1 | `right_wrist_yaw_joint` |
| vcan1 右臂 | 25/30 N=2 | `right_wrist_pitch_joint` |
| vcan1 右臂 | 25/30 N=3 | `right_wrist_roll_joint` |

## 12. 工程文件

- `urdf/upper_body.urdf.xacro`：半身双臂几何、质量惯量、关节限位、Gazebo ros2_control；
- `worlds/upper_body_world.sdf`：1 ms 物理步长的 Gazebo 世界；
- `config/controllers.yaml`：14 路 effort 控制器；
- `upper_body_canfd_gazebo/protocol_rp.py`：RP 64 Byte MIT 广播和状态帧；
- `upper_body_canfd_gazebo/protocol_2530.py`：腕部 25/30 mm 协议；
- `upper_body_canfd_gazebo/socketcan.py`：不依赖 python-can 的 Linux SocketCAN/CAN-FD 层；
- `upper_body_canfd_gazebo/canfd_gazebo_bridge.py`：CAN-FD <-> Gazebo 双向电机仿真；
- `upper_body_canfd_gazebo/motion_library.py`：0~13 号参考动作、关节限位检查和 smoothstep 插值；
- `upper_body_canfd_gazebo/canfd_action_player.py`：终端编号播放器，等待输入期间持续保持 MIT 目标；
- `upper_body_canfd_gazebo/canfd_demo_commander.py`：双臂 MIT 演示上位机；
- `upper_body_canfd_gazebo/canfd_keyboard_commander.py`：交互式键盘/手动指令遥控器；
- `scripts/setup_vcan.sh`：建立 vcan0/vcan1；
- `test/test_protocols.py`、`test/test_motion_library.py`、`test/test_action_player_packets.py`：协议字节序列、动作库限位/插值、动作到 CAN 帧映射测试（pytest）。

## 13. 当前哪些参数是“仿真假设”

通信字节格式来自你给的两份协议，5 Mbps 则以你的订正为准；但下面这些机械/电机参数尚没有真实机器人 CAD 或完整电机规格书，因此我没有把它们伪装成真值：

- 上臂、前臂、手掌的长度与质量；
- RP 关节实际额定/峰值扭矩（URDF 暂按协议编码上限 60 Nm 限幅）；
- 25/30 mm 腕部电机输出轴扭矩常数（默认仿真值 0.35 Nm/A）；
- 腕部内部位置/速度模式的固定增益。

这些都已经集中成易改参数。下一步拿到机械尺寸、RP 具体型号和腕部 25/30 mm 具体型号后，可以把“协议级仿真”继续升级成“参数接近真机的数字样机”。

## 14. C++ 真机硬件层（upper_body_canfd_hardware）

V4 新增 C++ 包 `upper_body_canfd_hardware`：把现有 Python 协议层（RP MIT 广播、25/30 mm 腕部 MIT、SocketCAN）移植为纯 C++，并实现
`hardware_interface::SystemInterface` 插件 `upper_body_canfd_hardware/CanFdMotorSystem`，使真机走标准 ros2_control 生命周期
（on_init → on_configure → on_activate → read/write → deactivate），URDF 与 `forward_command_controller` 配置不变。
Humble 的 ros2_control 只支持 C++ 硬件插件，这也是本包用 C++ 实现的原因。

### 14.1 仿真/真机切换

`urdf/upper_body.urdf.xacro` 新增 `use_real_hardware` 开关：

- `use_real_hardware:=false`（默认）：`gz_ros2_control/GazeboSimSystem`，走 Gazebo 仿真；
- `use_real_hardware:=true`：左右臂各一个 `CanFdMotorSystem` 实例，分别绑定一条 CAN-FD 总线。

每个关节在硬件块里带参数：`motor_type`（rp|wrist）、`can_node_id`、`direction`、`zero_offset`、`kp`、`kd`、`effort_scale`，
默认 kp/kd 与键盘工具一致（RP 36/2.0、腕部 0.008/1.0）。

### 14.2 无 Gazebo 全链路验证（vcan + 假从站）

```bash
cd ~/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch upper_body_canfd_gazebo hardware_sim.launch.py
```

该 launch 会启动 vcan 假从站（`fake_slaves.py`，模拟 RP 广播应答 + 腕部 MIT 应答/周期状态）、`robot_state_publisher`、
ros2_control（真插件跑在 vcan0/vcan1 上）以及 `joint_state_broadcaster` + `effort_controller`。验证：

```bash
ros2 control list_hardware_interfaces   # 14 个 effort 接口应为 available + claimed
ros2 topic echo /joint_states --once    # 假从站反馈经过插件解码进入状态接口
ros2 topic pub -1 /effort_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0,0,0,0,0,0,0,0.05,0,0,0,0,0,0]}"
```

### 14.3 真机 bring-up

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on restart-ms 100
sudo ip link set can0 up
sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000 dbitrate 5000000 fd on restart-ms 100
sudo ip link set can1 up

ros2 launch upper_body_canfd_gazebo real_hardware.launch.py \
  left_interface:=can0 right_interface:=can1 allow_hardware:=true
```

插件默认拒绝 `can*` 接口，`allow_hardware:=true` 是刻意为之的显式许可。真机首测顺序：先 enable 后零指令确认反馈静止，
再给单个关节（建议左腕 roll，2530 节点 N=3）极小 effort 验证方向与 `effort_scale`，通过后再扩展到 RP 单关节、单臂 7 关节。
`canfd_keyboard`/`canfd_actions` 仍保留直连总线模式，作为调试与动作库通道，与 ros2_control 插件并行，二者不要同时驱动同一批电机。

### 14.4 测试

协议编解码（已知字节序列 + round-trip）和插件全链路（vcan 回环 + 进程内假从站）用 gtest：

```bash
colcon build --symlink-install --packages-select upper_body_canfd_hardware
colcon test --packages-select upper_body_canfd_hardware
colcon test-result --verbose
```

新包详细文档见 `~/robot_ws/src/upper_body_canfd_hardware/README.md`。
