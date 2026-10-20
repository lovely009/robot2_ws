# robot_leg

单条腿（骨盆 → 髋 → 张合 → 大腿 → 膝 → 踝 2 轴，6 个主动关节 + 1 段张合并联连杆）的
SolidWorks 导出件。**当前只做到可视化预览**，不是可上控制/仿真的模型。

> ⚠ **本目录不是契约模型**：没有 `urdf/robot_leg.urdf.xacro`，因此
> `test/test_model_consistency.py` 不把它纳入参数化范围（契约缺项是**静默**的）。
> 四个 launch 里**只有 `display.launch.py` 能用它**；`gazebo` / `mujoco` /
> `simulation` 分别要求 xacro、`mujoco/<名>.xml`、`config/simulation_topology.yaml`，
> 本目录一概没有。

## 目录

| 路径 | 角色 |
| --- | --- |
| `cad/` | **溯源件**：2026-10-19 17:53 的 sw2urdf 原始导出（catkin 包形态，包名是非 ASCII 的 `机器人大腿`）。`urdf/机器人大腿.urdf` + `.csv` 是**几何/惯量/关节原点与轴的唯一来源**，只读不改；`cad/meshes/` 是导出件原名（`Joint_LinkN.STL`） |
| `urdf/robot_leg.urdf` | 适配后的主 URDF（预览用），由 `cad/` 那份机械变换而来，见下「已做的适配」 |
| `meshes/` | 可视网格，文件名与 link 名一致（`Joint_linkN.STL`） |

## 已做的适配（相对 cad/ 导出件）

1. `<robot name>` 改 `robot_leg`。
2. link 名 `Joint_LinkN` → `Joint_linkN`，joint 名 → `joint_linkN`。导出件里 **link 名与
   joint 名完全相同**，而 SDFormat 里 link/joint 共用命名空间，必须拆开。
3. 网格路径 → `package://robot_sim/models/robot_leg/meshes/`。
4. 6 个 revolute 关节的 `<limit>`（导出件全是 `0/0/0/0`）删掉，类型改 `continuous`——原因
   与补法见下「未做」。
5. 保留导出件的张合连杆与 `<mimic>`，见下。
6. 数值按 5 位有效数字书写（与 `half_body_robot` 同风格）。几何/惯量/关节原点与轴已与
   `cad/` 那份逐项核对一致（5 位有效数字内），只有第 4 条是有意丢弃。

## 关节表

`base_link` 是**骨盆组件**（QS-02-00），整条腿挂在它下面。关节轴、质量、组件名均取自
`cad/urdf/机器人大腿.csv`；「电机」是从 CAD 组件名里读到的型号，**未与真机核对**。

| link | joint | 关节轴 (xyz) | 质量 kg | CAD 组件名（节选） | 电机（CAD 名） |
| --- | --- | --- | --- | --- | --- |
| `base_link` | — | — | 12.848 | 骨盆组件、腿部支撑轴 | — |
| `Joint_link1` | `joint_link1` | `0 -0.11422 0.99345` | 0.29343 | 髋关节电机固定法兰盘 | 巨蟹58 |
| `Joint_link2` | `joint_link2` | `0 0 -1` | 0.65039 | 胯部关节旋转轴、腿部张合电机输出法兰盘、RB2008UUCC0 轴承 | 巨蟹58 ×2 |
| `Joint_link3` | `joint_link3` | `0 0 -1` | 0.65983 | 大腿骨架、腿部旋转电机转接轴 | 巨蟹48 |
| `Joint_link4` | `joint_link4` | `0 0 1` | 0.10179 | 膝关节旋转轴（无耳版）、小腿旋转轴限位块 | 巨蟹48 |
| `Joint_link5` | `joint_link5` | `0 0 1` | 0.33789 | 小腿骨架、脚踝 40 电机抱箍 | 巨蟹48 |
| `Joint_link6` | `joint_link6` | `0 0 -1` | 0.06977 | 脚踝连接旋转轴 | X25S-BZ153050-MH-1 |
| `Joint_link2.1` | `joint_link2.1` | `0 0 1` | 0.041575 | 张合连杆输入侧零件 + M6×12 螺钉 | — |
| `Joint_link2.2` | `joint_link2.2` | `0 0 1` | 0.13757 | 腿部张合关节输出连杆2、关节轴承 GE8C | — |

> 关节的功能命名（髋 / 膝 / 踝）只能从 CAD 组件名读出，未逐轴向 CAD 作者确认；`joint_link2`
> 一带列了两台巨蟹58，说明张合是独立电机驱动，**电机数可能多于 6**，接控制前必须核实。

## 张合连杆：joint_link2.1 → joint_link2.2

导出件在 `Joint_Link2.2` 上带 `<mimic joint="Joint_Link2.1" multiplier="1" offset="0"/>`，
即平行四边形张合连杆的 1:1 关系。`joint_state_publisher` 与 `robot_state_publisher`
**都支持 mimic**：前者把带 mimic 的关节放进 `dependent_joints` 而非 `free_joints`
（`joint_state_publisher_gui` 建滑杆只遍历 `free_joints`），后者按源关节算出从动关节值。

所以**拖 `joint_link2.1` 时 `joint_link2.2` 会 1:1 跟随**，2.2 不出滑杆、不散架，
也不需要额外的闭环求解器。实测：喂 `joint_link2.1 = 0.70 rad`，
`Joint_link2 → Joint_link2.2` 的偏航 = **0.7000 rad**。

## 未做（接控制/仿真前必须补）

| 缺项 | 为什么 / 怎么补 |
| --- | --- |
| **关节限位** | 导出件 6 个 revolute 的 `<limit>` 全是 `0/0/0/0`（CAD 里**没配角度配合**，与 `half_body_robot` 那次不同）。`<limit lower=upper>` 会让 `joint_state_publisher_gui` 连滑杆都不建，所以当前 6 个关节写成 `continuous`（JSP 对 continuous 给 ±π），**只为预览能拖，不代表真实行程**。补法：在 SolidWorks 里给关节加角度配合后重导出到 `cad/`，把类型改回 `revolute` 并填 `lower`/`upper` |
| `effort` / `velocity` | 取型号库 `motor_hardware_interface/config/motor.yaml` 的 `rated_torque_nm` / `rated_speed_rad_s`（cerb_r58 20 N·m、cerb_r48 5 N·m / 3.1416 rad·s⁻¹）。`joint_link6` 的 X25S 在该库中无对应档，须先确认型号 |
| `<dynamics>` | 该关节零位等效惯量（Pinocchio CRBA 对角，等效 1 s 时间常数），做法见 `half_body_robot/urdf/half_body_robot.urdf` 头部第 2 条 |
| 碰撞体 | 现用全分辨率网格（单腿 31 MB）。应按 `half_body_robot/tools/build_simplified_collision_meshes.py` 生成 `meshes/collision/` 凸包 |
| xacro / 控制器 / 拓扑 | `urdf/robot_leg.urdf.xacro`、`config/controllers.yaml`、`config/motor_driver.yaml`、`config/simulation_topology.yaml` |
| standalone / dist / MJCF | `tools/regen_derived.py`、`tools/build_mujoco_model.py`。两个工具都按 `models/<目录名>/` 取路径、不写死模型名，理论上可直接复用 |
| 与上身的关系 | 本模型是**独立单腿**，没有把 `base_link` 挂到 `half_body_robot` 的躯干下，也**没有左右镜像**。要合成整机需另行处理（镜像侧还要确认张合连杆的 mimic 符号） |

## 怎么看

```bash
source /opt/ros/humble/setup.bash && cd ros2_ws
colcon build --symlink-install --packages-select robot_sim && source install/setup.bash
ros2 launch robot_sim display.launch.py model:=robot_leg
```

RViz 里 7 个滑杆（`joint_link1..6` + `joint_link2.1`）；`joint_link2.2` 无滑杆，跟随
`joint_link2.1`。Fixed Frame 是 `base_link`，与 `config/display.rviz` 一致。
