# 肘部关节 Gazebo 仿真接入设计

## 目标

将 `ros2_ws/src/urdf` 中的四轴肘部关节模型接入现有 ROS 2 Humble + Gazebo
Fortress 仿真栈，使其通过 `gz_ros2_control` 和现有 `motor_sim` 链路接受控制，
同时保持当前 UR5e 仿真入口和行为不变。

## 范围

- 保留 `elbow_joint_description/urdf/joint.urdf` 作为几何、质量、惯性、坐标和
  机械关节限位的来源。
- 为肘部模型新增仿真包装 Xacro、控制器配置、电机拓扑和独立 launch 入口。
- 使用 Gazebo Fortress 6、`ros_gz_sim`、`gz_ros2_control` 和现有
  `robot_arm_gazebo` 电机仿真模块。
- 不修改或替换现有 UR5e launch、UR5e 模型及其配置。
- 不把仿真默认的 effort、velocity、阻尼或电机型号参数当作真机额定参数。

## 方案选择

采用独立肘部仿真入口，而不在现有 UR5e launch 中加入模型选择分支。这样两个模型
具有各自完整而窄小的启动接口，肘部适配的变化集中在自己的文件内，避免扩大现有
UR5e launch 的条件复杂度。

未采用以下方案：

- 单一 launch 通过 `robot_model` 切换：共享编排较多，但会使模型特有的关节数、
  初始位置、控制器和电机拓扑充满条件分支。
- 直接替换 UR5e：会破坏现有六轴仿真入口，不满足兼容要求。

## 模块与接口

### 描述模块

`elbow_joint_description` 提供模型资源和仿真包装 Xacro。包装层负责：

- 通过 Xacro include 引入原始四轴机械描述，避免复制 link、joint 和 mesh 定义；
- 将 `link0` 固定到仿真世界；
- 为 `1J`、`2J`、`3J`、`4J` 声明 effort command interface，以及 position、
  velocity、effort state interface；
- 加入 `gz_ros2_control/GazeboSimSystem` 和
  `gz_ros2_control::GazeboSimROS2ControlPlugin`；
- 为四个活动关节提供仿真用的 `effort=11 N·m`、
  `velocity=3.141592653589793 rad/s` 和 `damping=0.1 N·m·s/rad`。

原始网格、惯性、关节原点、轴向和上下限不因仿真适配而改变。固定关节 `J1` 不进入
控制关节清单。

### 仿真集成模块

`robot_arm_gazebo` 新增 `elbow_joint.launch.py`，其公开接口只暴露运行时确实需要的
参数：世界文件、Gazebo 日志等级、机器人命名空间、电机配置文件、传输端点和故障
策略。默认启动路径为：

1. 渲染肘部 Xacro 并启动 `robot_state_publisher`；
2. 启动 Gazebo Fortress 和 `/clock` bridge；
3. 从 `robot_description` 生成模型；
4. 依次配置 `joint_state_broadcaster` 和 effort controller；
5. 启动现有 `motor_sim`，由其根据关节状态发布 effort 命令；
6. 任一必要阶段失败时停止整条仿真链。

肘部 launch 不调用 UR5e launch，也不通过复制后继续共享隐式状态。可复用且与模型
无关的解析或进程编排逻辑应进入已有 `launch_support` 模块；模型特有默认值保留在
肘部 launch 中。

## YAML 单一来源与映射

新增肘部仿真电机拓扑 YAML，并同步新增对应 controller YAML。默认拓扑为：

| 电机实例 | ROS 关节 | 总线 | ID |
|---|---|---|---|
| `elbow_motor_1` | `1J` | `vcan0` | 1 |
| `elbow_motor_2` | `2J` | `vcan0` | 2 |
| `elbow_motor_3` | `3J` | `vcan0` | 3 |
| `elbow_motor_4` | `4J` | `vcan0` | 4 |

四个关节统一使用现有 `motor_sim` 已支持的 `cerb` protocol 和 `rp_elbow`
profile；`motor_simulation.buses` 为 `[vcan0]`。不新增协议实现。YAML 中明确注释这些
ID、增益和 profile 是仿真默认值，真机接入时必须用已确认的电机拓扑替换。

以下集合必须严格相等：

```text
URDF 活动关节 = ros2_control 关节 = controller joints = motor YAML joints
              = {1J, 2J, 3J, 4J}
```

不在 C++ 或 launch 中写死电机数量；运行时电机实例仍由 YAML 加载。

## 控制与数据流

```text
Gazebo joint state
  → gz_ros2_control
  → joint_state_broadcaster
  → motor_sim（读取肘部电机 YAML）
  → effort controller commands
  → gz_ros2_control
  → Gazebo joints 1J..4J
```

四个关节的初始位置均为 `0 rad`，且都位于原始机械限位内。控制器在模型创建和
controller manager 就绪前保持 inactive，初始化完成后再恢复物理仿真，避免重力下
无控制坠落。

## 错误处理与安全边界

- Xacro 渲染、模型生成、控制器配置或电机仿真进程失败时，launch fail-fast 退出。
- 配置出现未知关节、重复 ID、缺失总线或关节集合不一致时，在启动前拒绝运行。
- `link0` 固定到世界，避免基座自由落体。
- 仿真 effort、velocity、阻尼和增益只用于数值稳定与接口验证，不替代真机标定、
  机械限位或硬件急停。
- `motor_sim` 内部逻辑总线名固定为 `vcan0`，launch 默认
  `transport_mode=socket_can`、`arm_port=vcan0`；启动说明明确要求系统已创建
  `vcan0`。调用方可用现有 launch 参数覆盖传输模式和端点。

## 测试与验收

自动测试覆盖：

- 原始 URDF 继续通过 XML、`check_urdf` 和 SDF 转换；
- 包内所有 mesh URI 可解析；
- 仿真 Xacro 可渲染，包含固定基座、四个活动关节和 Gazebo 插件；
- URDF、ros2_control、controller YAML、电机 YAML 的关节集合完全一致；
- 电机总线存在、ID 唯一且为 1～4；
- 新 launch 静态加载成功，现有 UR5e launch 测试不回归；
- 目标包构建和 `colcon test` 通过。

集成验收使用 Gazebo Fortress：启动独立肘部 launch 后，模型和全部 STL 正常显示，
`1J`～`4J` 出现在 controller manager 与 `/joint_states` 中，发送非零控制命令时至少
一个关节状态发生符合限位约束的变化，日志中没有资源解析、插件加载或关节不匹配
错误。

## 使用接口

最终入口为：

```bash
ros2 launch robot_arm_gazebo elbow_joint.launch.py
```

允许通过 launch 参数覆盖肘部电机 YAML 和仿真传输端点；默认值无需调用方了解包内
文件布局。UR5e 保持原入口：

```bash
ros2 launch robot_arm_gazebo gazebo.launch.py
```
