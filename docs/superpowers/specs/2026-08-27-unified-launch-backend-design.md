# robot_arm_gazebo 统一 Launch 仿真后端设计

## 1. 目标

`robot_arm_gazebo` 作为其他工程可直接接入的完整仿真后端，只保留并使用
`gazebo.launch.py` 这一条启动入口。用户完成构建后执行：

```bash
ros2 launch robot_arm_gazebo gazebo.launch.py
```

即可启动 Gazebo、UR5e 模型、ROS/Gazebo 时钟桥、状态发布器、控制器和
`robot_arm_motor_sim_node`。不再要求另开终端执行 `ros2 run`。

本次改动不新增 ROS package、真机 bring-up、外部硬件插件、动作播放器自动启动或
新的 launch 文件。动作播放器仍只用于模型验收，且不得与电机仿真节点并行控制。

## 2. 单一入口与参数分层

`launch/gazebo.launch.py` 是完整后端的唯一编排入口。文件顶部集中定义带类型的默认值，
方便长期修改；每个默认值通过 `DeclareLaunchArgument` 暴露，方便调用方临时覆盖。

每个 launch 参数声明前必须有中文注释，说明：

- 参数控制的组件或接口；
- 取值格式、单位或允许值；
- 对连接和安全行为有影响时的约束。

运行参数归 launch 管理，包括：

| 参数 | 默认值 | 作用 |
|---|---|---|
| `initial_positions` | `0,-1.57,0,-1.57,0,0` | 六轴初始角，单位 rad |
| `world_file` | 包内 `worlds/ur5e_world.sdf` | Gazebo world 路径 |
| `gz_verbosity` | `2` | Gazebo 日志等级 |
| `transport_mode` | `usb_serial` | `usb_serial`、`serial_slcan` 或 `socket_can` |
| `arm_port` | `/dev/ttyUSB0` | 串口设备路径或 SocketCAN 接口名 |
| `baud_rate` | `3000000` | 串口波特率；SocketCAN 模式忽略 |
| `allow_external_canfd` | `false` | `serial_slcan` 的显式 CAN-FD 授权开关 |
| `fault_latching_enabled` | `true` | 是否保持故障锁存和整臂零输出安全语义 |
| `motor_config_file` | 包内 `config/motors.yaml` | 电机仿真配置文件路径 |
| `robot_namespace` | `robot_arm` | 机器人 ROS namespace |
| `joint_states_topic` | `joint_states` | namespace 内的关节状态接口 |
| `effort_commands_topic` | `effort_controller/commands` | namespace 内的 effort 命令接口 |
| `diagnostics_topic` | `diagnostics` | namespace 内的诊断接口 |

布尔参数必须使用 ROS launch 的条件化类型转换，不能把非空字符串误当成 `true`。
数值参数传给 ROS 节点时保留数值类型。

电机协议、ID、六轴顺序、限位、传动、plant 增益和时序仍只定义在
`config/motors.yaml`。launch 只选择 `motor_config_file`，不复制这些值，保持该 YAML
作为电机仿真配置的单一来源。

## 3. 启动编排与生命周期

启动顺序为：

1. 在创建进程前解析并校验六轴 `initial_positions` 和 launch 层可判定的枚举参数；
2. 启动 Gazebo、`robot_state_publisher` 和 `/clock` bridge；
3. 将 UR5e entity 创建到 Gazebo；
4. entity 创建成功后启动 `joint_state_broadcaster`；
5. broadcaster 成功后启动 `effort_controller`；
6. effort controller 成功后，在机器人 namespace 内启动
   `robot_arm_motor_sim_node`。

模型创建和两个 controller spawner 延续当前 fail-fast 链：任一阶段非零退出，记录阶段和
退出码并关闭整个 launch。电机节点属于长驻关键进程；其参数校验失败、设备打不开、
传输断开或运行期异常退出时，也触发整个 launch 关闭。Gazebo 自身退出继续关闭 launch。

这样不会存在控制器未就绪时提前发布 effort 的竞态，也不会在电机后端已经失效时留下
一个看似可用的 Gazebo 场景。

## 4. 连接模式和校验

三种 transport 由同一组 launch 参数切换：

```bash
# 默认 USB 串口
ros2 launch robot_arm_gazebo gazebo.launch.py

# USB ACM 设备
ros2 launch robot_arm_gazebo gazebo.launch.py arm_port:=/dev/ttyACM0

# 本机 SocketCAN
ros2 launch robot_arm_gazebo gazebo.launch.py \
  transport_mode:=socket_can arm_port:=vcan0

# 外部 SLCAN-FD 适配器
ros2 launch robot_arm_gazebo gazebo.launch.py \
  transport_mode:=serial_slcan arm_port:=/dev/ttyUSB0 \
  allow_external_canfd:=true
```

launch 将参数原样传给 `robot_arm_motor_sim_node`，由现有节点执行权威校验：

- transport 只允许 `usb_serial`、`serial_slcan`、`socket_can`；
- 串口路径必须存在、可打开且为 tty，允许 `/dev/ttyUSB*`、`/dev/ttyACM*`、
  `/dev/serial/by-id/*` 和 PTY；
- SocketCAN 参数必须是不含 `/` 的已存在接口名；
- 串口波特率必须位于节点已有允许集合；
- `serial_slcan` 必须显式设置 `allow_external_canfd:=true`。

默认 `/dev/ttyUSB0` 不存在时启动失败，不自动扫描或猜测其他设备，避免绑定错误硬件。

## 5. ROS 接口

默认接口保持为：

```text
/robot_arm/joint_states
/robot_arm/effort_controller/commands
/robot_arm/diagnostics
/robot_arm/controller_manager
```

`robot_arm_motor_sim_node` 与 `robot_state_publisher` 放在 `robot_namespace` 下。
电机节点通过 launch remapping 消费 `joint_states_topic`、发布
`effort_commands_topic` 和 `diagnostics_topic`。接口参数使用 namespace 内相对名称作为
默认值；调用方也可以传绝对 topic 名称接入既有系统。

controller manager 的 namespace、spawner 目标和 xacro 中的 ROS 2 control namespace
必须由同一个 `robot_namespace` 推导，不能分别写死。Gazebo entity 名 `ur5e` 和六轴 ABI
保持不变。

## 6. 文件改动

- `src/robot_arm_gazebo/launch/gazebo.launch.py`：集中默认值、参数声明、节点参数和
  remapping、完整启动链及关键进程退出处理。
- `src/robot_arm_gazebo/test/test_launch_contract.py`：验证统一入口包含电机节点、参数
  默认值与注释、namespace 接线、fail-fast 链和类型转换。
- `README.md`：将启动说明改为单 launch 工作流，列出常用覆盖示例和参数查看命令。
- `CONTEXT.md`：记录 `gazebo.launch.py` 是完整仿真后端唯一入口及稳定接口。
- `docs/verification/`：记录构建、测试、包数量和静态 launch 契约验收结果。

不新增 launch 文件或第三个 package。

## 7. 测试与验收

开发遵循测试先行：先扩展静态 launch 契约测试并观察其因缺少统一编排而失败，再实现
最小改动使其通过。

验收命令：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
colcon list --names-only
ros2 launch robot_arm_gazebo gazebo.launch.py --show-args
```

验收标准：

- 全部测试通过且 `colcon test-result --verbose` 无失败；
- `colcon list --names-only` 只输出 `motor_sim` 和 `robot_arm_gazebo`；
- `--show-args` 展示本设计列出的全部参数及用途；
- launch 静态契约确认电机节点只在 effort controller 成功后启动；
- launch 静态契约确认电机节点异常退出会关闭整套后端；
- 不要求 Gazebo GUI、真实串口或真实 CAN 设备参与自动测试。

