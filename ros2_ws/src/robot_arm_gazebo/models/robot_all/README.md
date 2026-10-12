# robot_all

SolidWorks 导出的**上身 + 双臂 18 轴**装配体：`base_link`（躯干，网格 z ∈ `0.0000 .. 0.7029`）
分出**两条互不相连的 9 轴链** —— 第一链 `Joint_link1..9`（躯干 x ≈ +0.06 一侧）与第二链
`Joint_link1.1..9.9`（x ≈ −0.05 一侧）。全部 `revolute`，0 个 `fixed`；两条链都是纯树、
彼此不连，没有闭环，不需要补环节点。

> 需要**力矩控制 + 补齐的仿真物理参数**（关节阻尼、自碰撞、简化碰撞体、effort 状态接口）
> 用派生模型 [`models/full_body/`](../full_body/README.md)（全身双臂 18 轴，
> 带两条手臂的电机型号映射）：机械几何/惯量与
> 本模型逐字相同，只改仿真侧参数与控制接口。本模型保留为历史验收基线，不再改参数。

躯干/连杆的质量与网格取自本次 CAD 导出，与同一装配体更早的导出件**不同**（例如本模型
`base_link` 为 6.333 kg / 50 万三角面）。第二链**不是**第一链的严格镜像（如 `Joint_link1`
的 x 偏移 +0.064 对 `Joint_link1.1` 的 −0.054；`Joint_link4` 的轴 `(0,0,1)` 对
`Joint_link4.4` 的 `(−0.035, 0.039, 0.999)`），**不能**把一条链的标定结果直接套到另一条。

> ⚠ **命名**：link 是 `Joint_linkN` / `Joint_linkN.N`（大写 J，导出器原样），joint 是
> `joint_linkN` / `joint_linkN.N`（小写）。导出器原本让 joint 与它驱动的 link 同名，而
> SDFormat 里 link 与 joint 共用 `<model>` 的命名空间 —— 同名会让模型在 Ignition 里
> **直接建不起来**（`Non-unique name` / `FrameAttachedToGraph` 环），`gz_ros2_control`
> 随之起不来、`controller_manager` 不出现。静态测试解析的是 URDF 而非 SDF，抓不到这个缺陷。
> `.N` 后缀是导出器给第二链的编号，**不是**层级或数值（`joint_link7.7` 是第二链第 7 轴，
> 与 `joint_link7` 无关）。含点号的名字 ROS 2 合法（tf frame_id、`/joint_states` 名都接受），
> 但命令行传参要加引号。

> ⚠ **已接入端到端电机仿真，但只覆盖第一条链**：`config/simulation_topology.yaml` 把
> `joint_link1..9` 映射到 9 台电机（`chest_joint1..9` ↔ id 1..9，id 1–7 为 CERB、8–9 为
> vaxor_2550），电机定义在仿真专用的 `config/motor_driver.yaml`（权威整机配置不含这 9 台，
> 因为它们的关节尚未并入 `upper_body.urdf.xacro`）。第二条链（`joint_link1.1..9.9`）**没有
> 电机名/ID**，在 Gazebo 里恒关重力、不参与闭环。`simulation.launch.py` 默认选本模型；
> `display.launch.py` 与 `gazebo.launch.py` 同样默认选本模型。

> ⚠ **第二链无驱动、第一链无控制器时会下垂**：第二链 9 个连杆的重力在 xacro 里写死
> `false`；第一链受 `gravity` 参数控制，`gazebo.launch.py` 默认 `gravity:=true`，没有控制器
> 时会在重力下自由下垂，做纯外观/运动学检查请用 `gravity:=false` 锁住重力。

> ⚠ **关节限位/力矩/速度是占位值**（`±1.5708 rad` / `10 Nm` / `1 rad/s`，沿用被删除的
> `chest_upper_00` 的占位值），**未经标定**。导出器原始输出是退化的 `0/0/0/0`
> （`export.log` / `robot_all.csv` 里仍是原值），会让 RViz 滑杆拖不动、Gazebo 锁死关节。
> 接真机前必须按实际机械限位与执行器参数替换。

## 目录

| 文件 | 说明 |
|---|---|
| `urdf/robot_all.urdf` | 主 URDF：导出件经下面三处适配后的结果；`display.launch.py` 加载它 |
| `urdf/胸部整体结构装配体-上身部分.urdf` | 导出器**逐字原样**的产物（未适配：网格指向不存在的包、限位退化 `0/0/0/0`、joint 与 link 同名）。保留作溯源与重新适配的对照；ROS/Gazebo 都不加载它 |
| `urdf/robot_all.urdf.xacro` | Gazebo 薄壳：注入 `<ros2_control>`（18 关节，`gz_ros2_control/GazeboSimSystem`）与插件；`gazebo.launch.py`、`simulation.launch.py` 加载它 |
| `urdf/robot_all.canfd.xacro` | 主机 `motor_hardware_interface` 薄壳（同一份 URDF + `canfd_hardware`），`simulation.launch.py` 的 `controller_mode:=host` 用它把命令编码到 CAN；控制关节清单运行期从 `control_joints` 注入 |
| `urdf/robot_all.standalone.urdf` | 派生单文件版（无控制器、无 Gazebo 插件），供脱离 ROS 使用 |
| `urdf/robot_all.csv` | 导出器产出的逐 link 明细（质量/惯量/网格/`SW Components` 等），保留作溯源，**内容是未适配的原值** |
| `config/controllers.yaml` | 纯 Gazebo（`gazebo.launch.py`）的 `gz_ros2_control` 参数：`arm_position_controller` / `arm_effort_controller`，18 关节全覆盖 |
| `config/simulation_topology.yaml` | 端到端电机仿真（`simulation.launch.py`）的电机↔关节映射：第一链 9 关节（第二链无电机） |
| `config/motor_driver.yaml` | 仿真专用电机拓扑（9 台 `chest_jointN`，同一条 chest 总线，id 1..9）；真机 bring-up 不读它 |
| `meshes/*.STL` | 19 个二进制 STL（`base_link` + 两条链各 9 个），visual 与 collision 复用同一份，约 70 MB |
| `export.log` / `package.xml` / `CMakeLists.txt` | 导出器原样产物，保留作溯源；本工作区的构建不使用它们 |

> 导出器原样产出的 ROS 1 部分（`launch/*.launch`、`config/joint_names_*.yaml`）**未保留**：
> 它们引用的是导出时的原文件名 `urdf/胸部整体结构装配体-上身部分.urdf` 与未改名的
> `Joint_linkN` 关节，落在 ROS 2 工作区里既跑不起来、又是会误导人的悬空引用。
> 关节名清单由 `urdf/robot_all.csv` 承担。

## 相对导出件做的三处适配

适配是**文本级**的（导出器的排版原样保留，只有列出的三处不同），`urdf/robot_all.urdf` 与
同目录的 `urdf/胸部整体结构装配体-上身部分.urdf` 逐行对照即可复核。重新导出时导出器会写出
中文名的那个文件，三处适配**必须重做**，做法：

1. **网格路径**：`package://胸部整体结构装配体-上身部分/meshes/…` →
   `package://robot_arm_gazebo/models/robot_all/meshes/…`（38 处 = 19 link × visual+collision）。
   导出器假设的包名不是已安装的 ROS 包，ROS 与 Gazebo 都解析不到，模型会全空白。
2. **关节限位**：18 个 `<limit>` 由 `lower/upper/effort/velocity = 0` 改为 `±1.5708 / 10.0 / 1.0`。
3. **关节改名**：18 个 `<joint>` 的 `name` 由 `Joint_linkN` / `Joint_linkN.N` 改为小写
   `joint_linkN` / `joint_linkN.N`（原因见上面的命名告警）。**link 名不要动**：mesh 文件名与
   `<gazebo reference>` 都按 `Joint_linkN` 索引。

`<robot name=…>` 也一并由导出器的中文装配体名改为 `robot_all`（与目录名一致）。

`standalone.urdf` 与 `dist/robot_all_urdf/` 是派生件，由 `urdf/robot_all.urdf.xacro`
（`fixed_base:=false` 渲染后剥掉 `<ros2_control>` 与 `<gazebo>`）同源生成，
由 `test_model_consistency.py` 兜底防止分叉。

## 关节表

序号 = 命令顺序（`Float64MultiArray` 的元素序）＝ URDF 物理关节顺序，也是
`<ros2_control>` 的声明顺序。两链的父子关系完全相同，差别只在几何。

| # | 第一链 joint | 第二链 joint | 父 → 子 | 类型 |
|---|---|---|---|---|
| 1 | `joint_link1` | `joint_link1.1` | `base_link` → `Joint_link1` / `Joint_link1.1` | revolute |
| 2 | `joint_link2` | `joint_link2.2` | `Joint_link1` → `Joint_link2` … | revolute |
| 3 | `joint_link3` | `joint_link3.3` | … | revolute |
| 4 | `joint_link4` | `joint_link4.4` | … | revolute |
| 5 | `joint_link5` | `joint_link5.5` | … | revolute |
| 6 | `joint_link6` | `joint_link6.6` | … | revolute |
| 7 | `joint_link7` | `joint_link7.7` | … | revolute |
| 8 | `joint_link8` | `joint_link8.8` | … | revolute |
| 9 | `joint_link9` | `joint_link9.9` | … → `Joint_link9` / `Joint_link9.9` | revolute |

关节原点与轴向量直接取自导出器，**未做真机标定**（零位、方向、限位都没在实物上验证过）。

## 几何实测

零位（全部关节角为 0）下，`visual` 的 `origin` 全为 `0 0 0`，各连杆网格在其 link 系里的
z 范围：

| link | z 范围 | link | z 范围 |
|---|---|---|---|
| `base_link` | `0.0000 .. 0.7029` | `Joint_link1.1` | `−0.0002 .. 0.0348` |
| `Joint_link1` | `0.0003 .. 0.0353` | `Joint_link2.2` | `−0.0650 .. 0.0075` |
| `Joint_link2` | `−0.0693 .. 0.0032` | `Joint_link3.3` | `−0.0002 .. 0.0703` |
| `Joint_link3` | `−0.0002 .. 0.0703` | `Joint_link4.4` | `−0.0485 .. 0.0144` |
| `Joint_link4` | `−0.0505 .. 0.0110` | `Joint_link5.5` | `0.0000 .. 0.1995` |
| `Joint_link5` | `0.0000 .. 0.1995` | `Joint_link6.6` | `−0.0100 .. 0.0540` |
| `Joint_link6` | `−0.0540 .. 0.0100` | `Joint_link7.7` | `−0.0000 .. 0.1464` |
| `Joint_link7` | `−0.0060 .. 0.1404` | `Joint_link8.8` | `−0.0195 .. 0.0000` |
| `Joint_link8` | `−0.0195 .. 0.0000` | `Joint_link9.9` | `−0.0322 .. 0.0030` |
| `Joint_link9` | `−0.0275 .. 0.0075` | | |

零位下全模型整体包围盒 `x ∈ [−0.1903, 0.1935]`、`y ∈ [−0.0450, 0.1221]`、
`z ∈ [0.000000, 0.7725]`：**最低点恰为 z = 0.000000**（`base_link` 底面落在原点平面上，
两条链都挂在躯干上部、不触地），所以 **`spawn_z` 用 `0.0`** —— `gazebo.launch.py` 的默认值
无需针对本模型调整。躯干是沿 x 展开的板件（x 宽 0.21、y 厚 0.14、z 高 0.70），两条链沿 x
分列、主要朝 +y 伸展。

## 用法

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws && colcon build --symlink-install --packages-select robot_arm_gazebo
source install/setup.bash

# RViz 预览（滑杆拖动 18 个关节）
ros2 launch robot_arm_gazebo display.launch.py model:=robot_all

# Ignition 仿真（纯 Gazebo，控制器接管 18 关节；第一链无控制器时会垂可用 gravity:=false）
ros2 launch robot_arm_gazebo gazebo.launch.py model:=robot_all

# 端到端电机—机械臂仿真（Gazebo + motor_sim + 第一链 9 台电机，默认走本机 robot_vcan0）
ros2 launch robot_arm_gazebo simulation.launch.py model:=robot_all
```

`display.launch.py` 加载的是纯模型 `urdf/robot_all.urdf`（无控制器），`gazebo.launch.py` 与
`simulation.launch.py` 加载 `urdf/robot_all.urdf.xacro`。网格按 `package://robot_arm_gazebo/…`
解析，所以**必须先 source 本工作空间**；Gazebo 侧由 `IGN_GAZEBO_RESOURCE_PATH` 指向 install
share（launch 已处理）。

`simulation.launch.py` 的电机来源：`config/simulation_topology.yaml` 声明哪些电机（本模型是
第一链 9 台）参与仿真、各接哪个关节；电机本体（总线、id、型号、增益）取自仿真专用的
`config/motor_driver.yaml`（`motor_config_file` 留空时的默认，只对本模型生效，不影响真机
bring-up）。第二条链无电机，仿真里不动作。
