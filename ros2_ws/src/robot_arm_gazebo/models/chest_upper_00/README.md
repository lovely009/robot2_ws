# chest_upper_00

SolidWorks 导出的上身结构装配体，**纯串联 9 轴**：`base_link → Joint_link1 → … → Joint_link9`，
全部 `revolute`，0 个 `fixed`，无闭环，不需要补环节点。

> ⚠ **命名**：link 是 `Joint_link1..9`（大写 J），joint 是 `joint_link1..9`（小写）。
> 导出器原本让 joint 与它驱动的 link 同名，而 SDFormat 里 link 与 joint 共用 `<model>` 的
> 命名空间 —— 同名会让模型在 Ignition 里**直接建不起来**（`Non-unique name` /
> `FrameAttachedToGraph` 环），`gz_ros2_control` 随之起不来、`controller_manager` 不出现。
> 静态测试（`test_model_consistency`）解析的是 URDF 而非 SDF，**抓不到这个缺陷**。

> ⚠ **已接入端到端电机仿真**：`config/simulation_topology.yaml` 给出 9 个关节的电机名映射
> （`joint_linkN ↔ chest_jointN ↔ id N`，id 1–7 为 CERB、8–9 为 vaxor_2550）。这 9 个关节
> **尚未并入** `motor_hardware_interface/urdf/upper_body.urdf.xacro`，所以电机定义放在本目录
> 自带的 `config/motor_driver.yaml`（仿真专用；权威整机配置不含它，也不该含——含了会让
> `test_xacro_consistency` 变红、真机 bring-up 去驱动不存在的关节）。`simulation.launch.py`
> 的默认模型就是本模型，且 `motor_config_file` 留空时会自动优先取本目录那份；
> `gazebo.launch.py`（纯 Gazebo）与 `display.launch.py`（RViz）同样默认本模型。

> ⚠ **关节限位/力矩/速度是占位值**（`±1.5708 rad` / `10 Nm` / `1 rad/s`，取自
> `zhoubuguanjie_youbi_002`），**未经标定**。导出器原始输出是退化的 `0/0/0/0`，会让 RViz
> 滑杆拖不动、Gazebo 锁死关节。接真机前必须按实际机械限位与执行器参数替换。

## 目录

| 文件 | 说明 |
|---|---|
| `urdf/chest_upper_00.urdf` | SolidWorks 导出的原始 URDF，已做三处适配（见下） |
| `urdf/chest_upper_00.urdf.xacro` | 仿真薄壳：注入 `<ros2_control>` 与 `gz_ros2_control` 插件；`gazebo.launch.py` 加载它 |
| `urdf/chest_upper_00.standalone.urdf` | 派生单文件版（无控制器、无 Gazebo 插件），供脱离 ROS 使用 |
| `config/controllers.yaml` | `gz_ros2_control` 参数：`arm_position_controller` / `arm_effort_controller`，9 关节全覆盖 |
| `config/joint_names_chest_upper_00.yaml` | SolidWorks 导出器产出的关节名清单，保留作溯源 |
| `meshes/*.STL` | 10 个二进制 STL（`base_link` + `Joint_link1..9`），visual 与 collision 复用同一份 |
| `export.log` / `package.xml` / `CMakeLists.txt` / `launch/` | 导出器原样产物，保留作溯源；本包的模型不靠它们加载 |

## 相对导出件做的三处适配

重新导出会覆盖 `urdf/chest_upper_00.urdf`，届时这三处必须重做：

1. **网格路径**：`package://chest_upper_00/meshes/…` →
   `package://robot_arm_gazebo/models/chest_upper_00/meshes/…`（20 处，visual+collision）。
   导出器假设的包名不是已安装的 ROS 包，ROS 与 Gazebo 都解析不到。
2. **关节限位**：9 个 `<limit>` 由 `lower/upper/effort/velocity = 0` 改为 `±1.5708 / 10.0 / 1.0`。
3. **关节改名**：9 个 `<joint>` 的 `name` 由 `Joint_linkN` 改为 `joint_linkN`。导出器让 joint
   与它驱动的 link 同名，而 SDFormat 里 link 与 joint 共用 `<model>` 的命名空间 —— 同名会让
   模型在 Ignition 里直接建不起来（实测：`Non-unique name[Joint_linkN] detected 2 times`、
   `joint ... must not specify its own name as the child frame`、
   `FrameAttachedToGraph cycle detected`），`gz_ros2_control` 起不来、`controller_manager`
   不出现、两个 spawner 各等 60 秒后死掉。改完 joint 与 link 名无交集，模型可正常生成。
   **link 名不要动**：mesh 文件名与 `<gazebo reference>` 都按 `Joint_linkN` 索引。

`standalone.urdf` 与 `dist/` 是派生件，由 `urdf/…urdf.xacro`（`fixed_base:=false` 渲染后剥掉
`<ros2_control>` 与 `<gazebo>`）同源生成，由 `test_model_consistency.py` 兜底防止分叉。

## 几何实测

零位下各连杆网格的 z 范围（link 系，`visual` 的 `origin` 全为 `0 0 0`）：

| link | z 范围 | link | z 范围 |
|---|---|---|---|
| `base_link` | `0.0000 .. 0.7029` | `Joint_link6` | `−0.0540 .. 0.0100` |
| `Joint_link1` | `0.0003 .. 0.0353` | `Joint_link7` | `−0.0060 .. 0.1404` |
| `Joint_link2` | `−0.0693 .. 0.0032` | `Joint_link8` | `−0.0195 .. 0.0000` |
| `Joint_link3` | `−0.0002 .. 0.0703` | `Joint_link9` | `−0.0275 .. 0.0075` |
| `Joint_link4` | `−0.0505 .. 0.0110` | | |
| `Joint_link5` | `0.0000 .. 0.1995` | | |

零位下全模型最低点恰为 **z = 0.000000**（`base_link` 底面落在原点平面上），所以
**`spawn_z` 用 `0.0`** —— `gazebo.launch.py` 的默认值已按本模型设为 `0.0`。
换回 `zhoubuguanjie_youbi_002` 需显式 `spawn_z:=0.0027`。

## 用法

```bash
ros2 launch robot_arm_gazebo display.launch.py model:=chest_upper_00
ros2 launch robot_arm_gazebo gazebo.launch.py  model:=chest_upper_00

# 力矩模式（与 position 控制器互斥）
ros2 launch robot_arm_gazebo gazebo.launch.py model:=chest_upper_00 control_mode:=effort
```

> 模型目录名含中文。`IGN_GAZEBO_RESOURCE_PATH` 已由 launch 指向 install share，
> 网格按 `model://robot_arm_gazebo/models/chest_upper_00/meshes/…` 解析。
