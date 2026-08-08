# 半身双臂 Gazebo + 双 CAN-FD/vcan 仿真工程

这是一个可直接放进 ROS 2 工作空间编译的 14 自由度半身机器人仿真包。模型不依赖 STL：躯干、手臂和手掌全部由 URDF/Xacro 基础几何体构成，方便先把 CAN-FD、MIT 控制、ROS 2 和 Gazebo 整条链路跑通，后续再替换成真实 CAD/STL。

## 1. 本版模型已经确定的结构

坐标约定：`+X` 向前、`+Y` 向左、`+Z` 向上。

每条手臂 7 DoF：

| 本地编号 | 关节 | 仿真电机 | 协议 |
|---|---|---|---|
| RP ID 1 | shoulder_yaw | 意优 RP | RP CAN-FD MIT |
| RP ID 2 | shoulder_roll | 意优 RP | RP CAN-FD MIT |
| RP ID 3 | shoulder_pitch | 意优 RP | RP CAN-FD MIT |
| RP ID 4 | elbow_pitch | 意优 RP | RP CAN-FD MIT |
| N=1 | wrist_yaw | 25/30 mm 驱控一体 | 25/30 mm CAN-FD |
| N=2 | wrist_pitch | 25/30 mm 驱控一体 | 25/30 mm CAN-FD |
| N=3 | wrist_roll | 25/30 mm 驱控一体 | 25/30 mm CAN-FD |

左右两臂分别使用独立总线：

```text
vcan0 / 将来的 can0 = 左臂：RP ID 1..4 + 腕部 N 1..3
vcan1 / 将来的 can1 = 右臂：RP ID 1..4 + 腕部 N 1..3
```

因为左右是两条物理总线，所以两边可复用相同的本地 ID。RP 的 CAN ID 与 25/30 mm 电机的 CAN ID 区间也不同，因此同一条手臂总线上两种协议可以并存。

## 2. CAN-FD 位率： 订正

本项目按下面的真实目标总线配置设计：

- 仲裁段：1 Mbps；
- 数据段：5 Mbps；
- 每条机械臂一条 CAN-FD 总线。

注意： `25-30mm驱控一体使用说明手册_20260204.pdf` 原文仍写“数据域 4M 80%”。订正为 5 Mbps，以本工程为准。`vcan` 不模拟物理波特率/采样点，只模拟 SocketCAN 帧接口；5 Mbps 是以后换成真实 `can0/can1` 时需要配置的物理参数。

真实 CAN-FD 接口示例（不要在 vcan 上执行）：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on restart-ms 100
sudo ip link set can0 up

sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000 dbitrate 5000000 fd on restart-ms 100
sudo ip link set can1 up
```

真实硬件还需要正确的 CANH/CANL、共地和总线两端终端匹配；25/30 mm 手册要求两端各 120 Ω。

## 3. 协议实现了什么

### 意优 RP（肩和肘）

依据《意优RP系列行星关节CAN-FD通信协议》V1.0：

- `CAN ID 0x00`：使能/失能；
- `CAN ID 0x80`：64 Byte 广播 MIT 指令，最多 8 个 8 Byte 关节槽；
- 本模型每条手臂只使用前 4 个槽，对应 RP ID 1..4；
- MIT 公式：`tau = Kp*(q_des-q) + Kd*(dq_des-dq) + tau_ff`；
- 位置映射 `[-10, 10] rad / 16 bit`；
- 速度 `[-60, 60] rad/s / 12 bit`；
- Kp `[0, 500] / 12 bit`；Kd `[0, 10] / 12 bit`；
- 扭矩 `[-60, 60] Nm / 12 bit`；
- 仿真器按电机 ID 1..4 返回 12 Byte MIT 状态帧。

### 25/30 mm 驱控一体（腕部）

依据手册：

- `0x501 + 4*N`：12 Byte MIT 下行；
- `0x502 + 4*N`：运行/停止/模式控制；
- `0x701 + 4*N`：12 Byte MIT 回复；
- `0x702 + 4*N`：每 20 ms 的 20 Byte 电机状态；
- 一圈 2,097,152 编码器刻度，多圈位置；
- 速度码值单位 0.1 rpm；
- `Kp = raw * 0.00001`，建议初调 0.008；
- `Kd = raw * 0.01`，建议初调 1；
- 前馈电流 `I = raw * 11/4096 A`。

腕部手册没有给出“输出轴 Nm/A”的确定值，因此不能把某个扭矩常数冒充真实参数。本仿真把 `wrist_torque_constant_nm_per_amp` 做成 ROS 参数，默认只取 `0.35 Nm/A` 作为可运行的仿真假设。你之后给我 25/30 mm 电机的准确型号/扭矩常数（或额定电流+额定输出扭矩），就应替换这个参数。

## 4. 软件目标环境：ROS 2 Humble + Gazebo Fortress

本版明确锁定为：

- ROS 2 Humble；
- Gazebo Fortress（Fortress / ignition-gazebo6，不是 Gazebo Classic）；
- ROS 2 Humble 官方二进制最匹配的平台是 Ubuntu 22.04 Jammy；
- `ros_gz_sim` 负责启动/生成模型，`gz_ros2_control` 负责把 effort 控制接口连接到 Gazebo 物理关节，`ros_gz_bridge` 把 Gazebo 仿真时钟桥接到 ROS 2 `/clock`。

Gazebo 官方将 Humble + Fortress 列为官方配对。要特别注意一个容易混淆的命名问题：当前 Humble 包名已经使用 `ros_gz` / `gz_ros2_control`，但 Fortress 后端仍是 `ignition-gazebo6`，所以本工程世界文件中的系统插件使用 `ignition-gazebo-*-system` 文件名。这不是把 Gazebo Classic 或旧版本混进来。

安装基础依赖（ROS 2 Humble 已安装的前提下）：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros-gz \
  ros-humble-gz-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  python3-colcon-common-extensions \
  can-utils
```

安装后先确认环境，不要继续使用之前的 Lyrical/Jetty 命令：

```bash
source /opt/ros/humble/setup.bash
echo "$ROS_DISTRO"
ros2 pkg prefix ros_gz_sim
ros2 pkg prefix gz_ros2_control
ign gazebo --versions
```

预期 `ROS_DISTRO` 输出 `humble`，两个 ROS 包都能找到；Fortress 使用 `ign gazebo` 命令行是正常现象。

## 5. 编译

本包已并入 `~/robot_ws` 工作空间，源码位于 `~/robot_ws/src/upper_body_canfd_gazebo`，统一由工作空间构建：

```bash
cd ~/robot_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` 让 Python 源码修改即时生效。跑测试：

```bash
colcon test
colcon test-result --verbose
```

## 6. 创建 vcan0 / vcan1

每次重启后如果 vcan 没有做成系统自启动，执行：

```bash
cd ~/robot_ws/src/upper_body_canfd_gazebo
bash scripts/setup_vcan.sh
```

验证：

```bash
ip -details link show vcan0
ip -details link show vcan1
```

用 can-utils 看总线：

```bash
candump vcan0
```

另开终端：

```bash
candump vcan1
```

## 7. 启动 Gazebo + 双 CAN-FD 电机仿真

终端 A：

```bash
cd ~/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch upper_body_canfd_gazebo sim.launch.py
```

> **WSL2 提示**：Gazebo Fortress 的 OGRE2 渲染在 WSLg 上可能崩溃或画面空白。可改用 OGRE1 后端（配合 Mesa D3D12）：
>
> ```bash
> MESA_D3D12_DEFAULT_ADAPTER_NAME=AMD GALLIUM_DRIVER=d3d12 \
> ros2 launch upper_body_canfd_gazebo sim.launch.py render_engine:=ogre
> ```
>
> `AMD` 为显卡适配器名，按实际硬件修改；也可以在 `.bashrc` 里定义 `igngazebo()` 后直接运行 `ign gazebo`。

正常情况下会出现：

1. Gazebo Fortress 世界；
2. 半身躯干，左臂蓝色、右臂橙色；
3. 14 个转动关节；
4. `joint_state_broadcaster`；
5. `effort_controller`；
6. ROS 2 `/clock` 随 Fortress 仿真时间更新；
7. `canfd_gazebo_bridge` 打开 `vcan0/vcan1`。

检查：

```bash
ros2 control list_controllers
ros2 topic hz /clock
ros2 topic hz /joint_states
ros2 topic echo /joint_states --once
```

## 8. 通过终端输入编号播放参考动作（canfd_actions，推荐）

V3 新增 `canfd_actions`。它不是简单地把关节瞬间写到某个角度，而是在关键帧之间使用三次 smoothstep 插值：

```text
s(u) = 3u² - 2u³
ds/dt = 6u(1-u)/T
```

每段动作的起点、终点目标速度都为 0，并以默认 100 Hz 连续发送 MIT 指令。即使程序正在等待你输入下一个编号，也仍会持续发送当前保持目标，不会触发 `canfd_gazebo_bridge` 的 0.15 s 指令超时。

终端 B：

```bash
cd ~/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run upper_body_canfd_gazebo canfd_actions
```

启动后直接输入编号再按 Enter：

| 编号 | 动作 | 最主要的参考意义 |
|---:|---|---|
| 0 | 回到准备姿态 | 统一基准、复位、稳定保持 |
| 1 | T 字双臂展开 | 检查 `shoulder_roll` 和左右镜像方向 |
| 2 | 双臂向前伸 | 检查 `shoulder_pitch` 正负方向和同步性 |
| 3 | 只抬左臂 | 确认 `vcan0`/左臂不会串到右臂 |
| 4 | 只抬右臂 | 确认 `vcan1`/右臂不会串到左臂 |
| 5 | 双臂对称屈肘 | 检查 RP ID4、重力负载下的 MIT 保持和同步 |
| 6 | 左手挥手 | 动态检查 vcan0、腕部 N1/N3、轨迹连续性 |
| 7 | 右手挥手 | 与 6 对照检查 vcan1 和左右差异 |
| 8 | 腕部 N1/N2/N3 三轴自检 | 依次单独检查 wrist yaw/pitch/roll 的节点映射和方向 |
| 9 | RP ID1~4 肩肘逐轴自检 | 依次检查 yaw/roll/pitch/elbow 与 RP 广播槽映射 |
| 10 | 综合辨识演示 | 整条 CAN-FD→MIT→Gazebo 链路的快速回归动作 |
| 11 | 双手上举（欢呼） | 双臂对称上举过头，检查双总线大角度协调保持 |
| 12 | 双臂左右齐摆 | 双臂同向 yaw 摆动，动态检查左右 shoulder_yaw 同步 |
| 13 | 双手转腕 | 双臂前伸同时转动腕部 N3，核对左右腕部映射与幅度 |

程序启动时会先用“零增益”指令读取 14 个关节当前反馈，再平滑进入准备姿态，避免把程序重启时的当前姿态直接当成全零。输入 `m` 可重新显示菜单；输入 `q` 会平滑回准备姿态再失能退出；输入 `x` 会立即失能退出；`Ctrl+C` 同样执行失能退出。

想让动作整体慢一半：

```bash
ros2 run upper_body_canfd_gazebo canfd_actions --speed-scale 0.5
```

想直接从命令行只播放编号 8，不进入菜单：

```bash
ros2 run upper_body_canfd_gazebo canfd_actions --action 8
```

只查看动作菜单、不打开 CAN 接口：

```bash
ros2 run upper_body_canfd_gazebo canfd_actions --list
```

动作播放器默认和 `canfd_demo` 一样只接受 `vcan*` 接口。**不要同时运行 `canfd_actions` 和 `canfd_demo`/`canfd_keyboard`**，否则多个上位机会同时向同一批电机发送目标。

这些角度目前是依据本工程 URDF 的零位、正方向和关节限位设计的“仿真参考动作”。即使以后把接口从 `vcan0/vcan1` 换成真实 `can0/can1`，也不能直接照搬到真机；必须先逐关节验证真实零位、正方向、软/硬限位、急停和安全扭矩，再建立真机动作库。

## 9. 连续双臂 MIT 演示（canfd_demo）

终端 B：

```bash
cd ~/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run upper_body_canfd_gazebo canfd_demo
```

它会做缓慢的小幅双臂动作，同时持续走真正的 CAN-FD 编解码链路：

```text
canfd_demo
  -> vcan0 / vcan1
  -> 私有 CAN-FD 协议解码
  -> MIT / 电流控制计算
  -> ros2_control effort
  -> Gazebo 关节动力学
  -> /joint_states
  -> 私有协议状态回复
  -> vcan0 / vcan1
```

按 `Ctrl+C` 时 demo 会先发送中性指令，再发送失能/停止帧。

只运行 10 秒：

```bash
ros2 run upper_body_canfd_gazebo canfd_demo --duration 10
```

### 9.1 键盘手动遥控（canfd_keyboard）

想手动控制单个关节，用键盘遥控器代替 demo：

```bash
cd ~/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run upper_body_canfd_gazebo canfd_keyboard
```

后台线程以 50 Hz 持续发送当前指令状态（桥接指令超时 0.15 s，必须持续刷新才能保持目标位置）。命令：

| 命令 | 说明 |
|---|---|
| `enable [left\|right\|all]` | 使能电机（默认 all） |
| `disable [left\|right\|all]` | 安全停止 + 失能 |
| `neutral [left\|right\|all]` | 零目标（放松），保持使能 |
| `pos <side> <joint> <rad> [kp] [kd]` | 位置指令，默认 kp=36/kd=2（腕部 0.008/1.0） |
| `vel <side> <joint> <rad/s>` | 速度指令（kp=0，保留 kd） |
| `mit <side> <joint> <pos> <vel> <kp> <kd> <ff>` | 完整 MIT 指令，RP 的 ff 为扭矩 Nm、腕部为电流 A |
| `act <n>` | 播放预设动作（`act` 查看列表，`stop` 提前结束） |
| `status` | 查看 14 个关节的指令状态 |
| `quit` / Ctrl+C | 安全停止退出（先中性指令再失能） |

`<joint>` 为 1..7：1=shoulder_yaw、2=shoulder_roll、3=shoulder_pitch、4=elbow_pitch（RP 协议），5=wrist_yaw、6=wrist_pitch、7=wrist_roll（25/30 mm 协议），也支持直接输关节名。

示例：

```text
> enable
> pos left 1 0.4            # 左肩 yaw 到 0.4 rad
> mit right 5 0.2 0 0.008 1 0
> status
> disable
> quit
```

**预设动作**（输入 `act <编号>` 播放，未参与的关节自动带刚度保持，不会甩飞）：

| 编号 | 动作 | 时长 | 特征 |
|---|---|---|---|
| 1 | 右手挥手 | 8 s | 右臂侧举后 yaw 轴摆动，左臂保持 |
| 2 | 双臂开合（拥抱） | 10 s | 双臂对称外张再向内合拢 |
| 3 | 交替摆臂（走路） | 10 s | 左右肩 pitch 反相摆动，模拟走路摆臂 |

示例：

```text
> enable
> act 2        # 播放"双臂开合"
> stop         # 提前结束，保持当前姿态
> act 1        # 换动作
```

## 10. 重要安全设计

`canfd_demo`、`canfd_keyboard` 和 `canfd_actions` 默认只允许名字以 `vcan` 开头的接口；仿真反馈桥也默认拒绝真实 `can0/can1`。原因是反馈桥本质上在“扮演电机”，如果误接到真机总线上，它会与真实电机同时发状态帧。

只有做明确规划过的 HIL（Hardware-in-the-Loop）实验时，才考虑开启：

```bash
ros2 run upper_body_canfd_gazebo canfd_demo \
  --left-interface can0 --right-interface can1 --allow-hardware
```

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

- `urdf/upper_body.urdf.xacro`：半身双臂几何、质量惯量、关节限位，以及 Gazebo 仿真 / 真机 CAN-FD 两套 ros2_control（`hardware_backend:=gazebo|canfd` 切换）；
- `worlds/upper_body_world.sdf`：1 ms 物理步长的 Gazebo 世界；
- `config/controllers.yaml`：14 路 effort 控制器；
- `config/hardware_controllers.yaml`：真机 14 路 position 控制器（`hardware.launch.py` 使用）；
- `launch/hardware.launch.py`：真机 ros2_control bring-up（CAN-FD 硬件插件 + 控制器）；
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

真机链路已经拆分为两个活动包：`motor_canfd_drive` 统一维护 SocketCAN 与电机协议策略，
`motor_hardware_interface` 只实现 `ros2_control` 适配并从 YAML 读取总线/电机/关节映射、增益与超时。
旧 `upper_body_canfd_hardware` 已删除，`hardware.launch.py` 只使用新的 `motor_hardware_interface`。
旧的独立 `can_motor_sim` 调试包也已删除；本包自己的 Python `canfd_gazebo_bridge.py` 仍完整保留，
因此原 Gazebo/Python 仿真路径不受影响。协议手册的 PDF 原件仍放在仓库根目录。

## 13. 当前哪些参数是“仿真假设”

通信字节格式来自你给的两份协议，5 Mbps 则以你的订正为准；但下面这些机械/电机参数尚没有真实机器人 CAD 或完整电机规格书，因此我没有把它们伪装成真值：

- 上臂、前臂、手掌的长度与质量；
- RP 关节实际额定/峰值扭矩（URDF 暂按协议编码上限 60 Nm 限幅）；
- 25/30 mm 腕部电机输出轴扭矩常数（默认仿真值 0.35 Nm/A）；
- 腕部内部位置/速度模式的固定增益。

这些都已经集中成易改参数。下一步拿到机械尺寸、RP 具体型号和腕部 25/30 mm 具体型号后，可以把“协议级仿真”继续升级成“参数接近真机的数字样机”。

## 14. 真实硬件 ros2_control 后端

新的 `motor_hardware_interface` 是标准 C++ `hardware_interface::SystemInterface` 插件，
底层 CAN-FD 通信委托给不依赖 ROS 的 `motor_canfd_drive`。仿真仍由 `gz_ros2_control`
提供相同的关节状态抽象；真机启动时 Xacro 通过
`hardware_backend:=canfd` 切换为 CAN-FD 硬件插件（默认 `gazebo`，`sim.launch.py` 无需改动）。
上层控制器不再依赖协议帧或 SocketCAN。真机入口为：

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch upper_body_canfd_gazebo hardware.launch.py \
  left_interface:=can0 right_interface:=can1
```

真机控制器接受 14 路 position 指令（`hardware_controllers.yaml`），并导出
position/velocity/effort 状态。硬件插件在 activate 时先读取 14 个关节当前反馈、以当前姿态
为初始目标再统一使能（不发明零位），deactivate 时统一安全失能，运行期间持续检查每个关节
反馈超时。禁止同时运行 `canfd_actions`、`canfd_demo` 或 `canfd_keyboard` 等直接占用 CAN
总线的旧工具。

> 新增 `motor_canfd_sim/launch/driver_loopback.launch.py` 用于 driver → vcan → C++ motor sim →
> Gazebo 的全回环路径。当前提交环境没有 ROS 2 Humble / Gazebo Fortress 运行时，因此这里完成的是
> 协议/源码静态验证；合并前仍应在目标 Ubuntu 22.04 环境跑 `colcon test` 和该全回环 launch。
> 真机上电仍必须按“先零增益确认反馈、再小力矩微动验证方向”的顺序进行。


## 15. 新 C++ CAN-FD 仿真路径

原 Python `canfd_gazebo_bridge.py` 保持不变。需要使用新 C++ bridge 时运行：

```bash
ros2 launch motor_canfd_sim gazebo_cpp_sim.launch.py
```

需要同时验证新的 `motor_hardware_interface` 时运行：

```bash
ros2 launch motor_canfd_sim driver_loopback.launch.py
```

后者把新 driver controller manager 放在 `/driver` 命名空间，避免与 Gazebo 自己的
`/controller_manager` 冲突；位置命令入口为 `/driver/position_controller/commands`。
旧 Python bridge 与新 driver 仍可通过同一 vcan 线协议通信，不需要 pybind11 或 Python wrapper。
