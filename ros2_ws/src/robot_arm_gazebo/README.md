# robot_arm_gazebo

机械臂 URDF 模型库与 Ignition Gazebo 仿真集成。每个模型是一个**自包含目录**
（`models/<模型名>/`：几何 + 网格 + 控制器参数 + 自己的 README），共享的
`config/`、`launch/`、`test/` 不含任何模型专有名字——加一个模型只需拷一个目录进来。

仿真后端是 Ignition（`ros_gz_sim` + `gz_ros2_control`）。

## 模型

| 模型 | 说明 | 自由度 |
| --- | --- | --- |
| `zhoubuguanjie_youbi_001` | 右臂（SolidWorks 导出，固定基座 + 4 个 revolute 关节） | 4 |

模型细节、CAD 出处、关节表与占位参数告警见各目录自己的 `README.md`。

## 目录结构

```
robot_arm_gazebo/
├── config/
│   └── display.rviz                        # 跨模型共享的 RViz 预览配置
├── launch/
│   ├── display.launch.py                   # RViz 预览（无 Gazebo、无控制器）
│   └── gazebo.launch.py                    # Ignition 仿真 + effort/position 控制器
├── models/
│   └── <模型名>/                            # 自包含：urdf / meshes / config / cad / README
├── dist/
│   └── <模型名>_urdf/                       # ROS 无关的对外发布件（相对路径网格），不安装
└── test/
    ├── test_model_consistency.py           # 模型目录三件套一致性
    └── test_launch_models.py               # launch 的模型选择链路
```

## 构建与使用

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select robot_arm_gazebo
source install/setup.bash

# RViz 预览（滑杆拖动关节）
ros2 launch robot_arm_gazebo display.launch.py

# Ignition 仿真（空世界 + 生成模型 + 控制器）
ros2 launch robot_arm_gazebo gazebo.launch.py
ros2 launch robot_arm_gazebo gazebo.launch.py world_file:=/path/to/world.sdf

# FDCAN/串口仿真电机后端：默认用 effort 控制器接收 MIT 等效力矩
ros2 launch robot_arm_gazebo gazebo.launch.py gravity:=false

# 若要测试 MIT 等效力矩动力学，显式切换两端为 effort
ros2 launch robot_arm_gazebo gazebo.launch.py control_mode:=effort
```

RViz 预览还需要 `joint_state_publisher_gui` / `rviz2`，Ignition 仿真还需要 `ros_gz_sim` +
`gz_ros2_control`。它们都写在 `package.xml` 的 `exec_depend` 里，但 **rosdep 不会自动装**；
缺包时 launch 在启动任何进程之前就抛 `package ... not found`（例如本机缺
`ros-humble-joint-state-publisher-gui`），补装即可，不是模型问题。

两个 launch 都接受 `model:=<模型名>` 选择 `models/` 下的模型，默认
`zhoubuguanjie_youbi_001`：

```bash
ros2 launch robot_arm_gazebo gazebo.launch.py model:=<模型名>
```

`gazebo.launch.py` 的其余参数：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `world_file` | `empty.sdf` | Ignition 世界文件 |
| `spawn_z` | `0.0` | 模型生成高度（m），按模型几何实测值定 |
| `fixed_base` | `true` | 把基座焊到世界原点；`false` = 自由浮动基座 |
| `controllers_file` | `models/<模型>/config/controllers.yaml` | controller_manager 参数 |
| `control_mode` | `effort` | `effort` 接收 FDCAN/Gazebo 电机后端发布的 MIT 等效力矩；`position` 仅用于话题/单位对照 |

## 控制机械臂

`gazebo.launch.py` 默认自动激活 `joint_state_broadcaster` 与
`arm_effort_controller`（MIT 等效力矩）。FDCAN/Gazebo 电机后端的
`--gazebo-control-mode` 必须与这里的 `control_mode` 一致。effort 模式把 MIT 控制律送入
`arm_effort_controller`，适合验收真实电机后端到 Gazebo 的闭环；position 模式把
固件已解码的关节目标送入 position controller，适合做话题、单位、ID、顺序和
轨迹插补对照。模型未内置重力补偿时需要单独调增益/前馈。两种控制器不会同时占用同一组关节命令接口：

```bash
ros2 topic pub --once /arm_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.5, -0.3, 1.0, 0.2]}"

ros2 topic echo /joint_states
```

命令向量顺序 = 该模型 `config/controllers.yaml` 里 `joints` 的顺序 =
URDF 关节串联顺序（三者由 `test_model_consistency.py` 锁死）。当前模型顺序为
`joint_link1` → `joint_link4`。

## 加一个新模型

1. 在 `models/` 下建 `<新模型名>/`，含 `urdf/`（`<名>.urdf`、`<名>.standalone.urdf`、
   `<名>.urdf.xacro`）、`meshes/`、`config/controllers.yaml`、`cad/`、`README.md`；
2. 网格路径写成 `package://robot_arm_gazebo/models/<新模型名>/meshes/...`；
3. 薄壳 xacro 用相对文件名 include 同目录的 URDF（`<xacro:include filename="<名>.urdf"/>`），
   不要用 `$(dirname)`——它只在 `ros2 launch` 的文件上下文里可用，本包 launch 是以子进程
   方式调 `xacro` 的；
4. 跑 `colcon test --packages-select robot_arm_gazebo`，测试会以新目录为参数自动覆盖它。

已知限制：

- **一次只生成一个模型**：控制器名（`joint_state_broadcaster` /
  `arm_position_controller`）是全局的，同一仿真里生成两个模型会撞名。
- `config/display.rviz` 的 Fixed Frame 写死 `base_link`，根连杆不叫 `base_link` 的模型
  需要自行覆盖 `rviz_config`。
- 模型是**仿真/可视化资产**：几何与惯性来自 CAD 未做真机标定，关节限位/力矩/速度是
  占位值，接真机前必须替换。真机链路（`motor_drive` + `motor_hardware_interface` +
  `motor_driver.yaml` 拓扑）与本包没有代码依赖。

## 单文件纯模型

每个模型的 `urdf/<名>.standalone.urdf` 是把薄壳展开、剥掉 `<ros2_control>` 与
`<gazebo>` 插件块后的纯模型，单个文件即可加载（RViz、第三方工具、拖到别处都行），
不需要 xacro 处理：

```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(cat install/robot_arm_gazebo/share/robot_arm_gazebo/models/<模型名>/urdf/<模型名>.standalone.urdf)"
```

网格仍走 `package://robot_arm_gazebo/models/<模型名>/meshes/`，因此加载前须 source
本工作空间；要脱离 ROS 使用，用 `dist/<模型名>_urdf/` 那份（网格是相对路径）。

## 对外发布件（`dist/`）

`dist/<模型名>_urdf/` 是不依赖 ROS 的单文件模型 + 网格，可以直接打包发给别人：

```bash
cd dist/zhoubuguanjie_youbi_001_urdf && zip -r ../zhoubuguanjie_youbi_001_urdf.zip .
```

zip 不入库（内容在目录里都有，随时可重打）；`dist/` 也不随包安装。
