# robot_leg（单腿）

单条腿的 SolidWorks 导出件：骨盆 `base_link` → 髋 → 膝 → 踝，**6 个主动关节，纯串联树**
（无闭环、无 mimic）。几何与惯量取自 CAD；`base_link` 是骨盆组件（QS-02-00）。

> 本模型 2026-10-20 从**新导出件**重建，取代同名的旧预览件。旧件（2026-10-19 导出）带一段
> 平行四边形张合连杆（`joint_link2.1` → `joint_link2.2`，靠 `<mimic>` 1:1 跟随），共 8 个
> 关节；新导出件取消了连杆，张合机构的相关零件并入 `Joint_link2`，因此现在只有 6 个关节。
> 旧件是**非契约预览件**（缺 xacro / controllers.yaml，只有 `display.launch.py` 能加载）；
> 本件已补全契约，四个 launch 都能用。

## 目录

| 路径 | 角色 |
| --- | --- |
| `cad/` | **溯源件**：2026-10-20 15:04 的 sw2urdf 原始导出（catkin 包形态，包名是非 ASCII 的 `机器人大腿`）。`urdf/机器人大腿.urdf` + `.csv` 是**几何/惯量/关节原点与轴的唯一来源**，只读不改 |
| `urdf/robot_leg.urdf` | 适配后的主 URDF，由 `cad/` 那份机械变换而来，见下「已做的适配」 |
| `urdf/robot_leg.urdf.xacro` | 仿真薄壳（`<ros2_control>` + gz 插件），机械部分与主 URDF 同源 |
| `urdf/robot_leg.standalone.urdf` | 派生：薄壳以 `fixed_base:=false` 渲染后剥掉仿真块 |
| `config/controllers.yaml` | gz_ros2_control 参数（含 `joints` 表） |
| `meshes/` | 可视网格，文件名与 link 名一致（`Joint_linkN.STL`）；与 `cad/meshes/` 是**硬链接**（同一 inode，不占额外空间） |
| `tools/regen_derived.py` | 重生成 standalone 与 `dist/`（改主 URDF 后必须重跑） |

## 已做的适配（相对 `cad/` 导出件）

1. `<robot name>` 改 `robot_leg`。
2. link 名 `Joint_LinkN` → `Joint_linkN`（导出器给的是**大写 L**），joint 名 → `joint_linkN`。
   导出件里 link 名与 joint 名完全相同，而 SDFormat 里 link/joint 共用命名空间，必须拆开。
3. 网格路径 → `package://robot_sim/models/robot_leg/meshes/`。
4. `<limit>` 四项按「关节限位与 effort/velocity」一节覆写（导出件全是 `0/0/0/0`）。
5. 补 `<dynamics damping="…" friction="0.0" />`：damping 取该关节零位等效惯量（Pinocchio
   CRBA 对角，kg·m²），等效 1 s 时间常数；friction 显式置 0（库仑摩擦未标定）。
6. 数值按 5 位有效数字书写（与 `half_body_robot` 同风格）。几何/惯量/关节原点与轴已与
   `cad/` 那份逐项核对一致，只有第 4 条的 lower/upper 是有意替换的占位值。

## 关节表

`base_link` 是**骨盆组件**（QS-02-00），整条腿挂在它下面。关节轴、质量、组件名均取自
`cad/urdf/机器人大腿.csv`；「电机型号」是从 CAD 组件名里读到的型号，**未与真机核对**。

| link | joint | 关节轴 (xyz) | 质量 kg | 电机型号（推定） | CAD 组件名（节选） |
| --- | --- | --- | --- | --- | --- |
| `base_link` | — | — | 12.926 | — | 骨盆组件、腿部支撑轴 |
| `Joint_link1` | `joint_link1` | `0 -0.11422 0.99345` | 0.29343 | cerb_r58_101 | 髋关节电机固定法兰盘、巨蟹58电机 |
| `Joint_link2` | `joint_link2` | `0 0 -1` | 0.82613 | cerb_r58_101 | 胯部关节旋转轴、腿部张合电机输出法兰盘、关节轴承 GE8C、**巨蟹58电机 ×2** |
| `Joint_link3` | `joint_link3` | `0 0 1` | 0.66354 | cerb_r48_101 | 大腿骨架、腿部旋转电机转接轴、巨蟹48电机、膝关节旋转限位块 |
| `Joint_link4` | `joint_link4` | `0 0 1` | 0.10179 | cerb_r48_101 | 膝关节旋转轴（无耳版）、小腿旋转轴限位块、巨蟹48电机 |
| `Joint_link5` | `joint_link5` | `0 0 1` | 0.33789 | cerb_r48_101 | 小腿骨架、脚踝 40 电机抱箍、巨蟹48电机 |
| `Joint_link6` | `joint_link6` | `0 0 -1` | 0.06977 | vaxor_2550 | 脚踝连接旋转轴、X25S 关节电机 |

> ⚠ **电机型号是按「同 link 内出现的电机」推定的，未逐轴向 CAD 作者确认**，接控制前必须核实。
> 两处具体疑点：
> 1. `Joint_link2` 里有**两台巨蟹58**（一台驱动 `joint_link2`，另一台驱动已取消的张合机构），
>    说明**电机数可能多于 6**；
> 2. 另一种读法是「电机装在父 link、驱动下一个关节」（`zhoubuguanjie_youbi_001` 的导出件就是
>    这个读法）。按那种读法 `joint_link1` 的电机在 `base_link` 的骨盆组件里（CSV 未展开到零件级），
>    本表按「同 link 同名次」的读法取值——**取两者中较小者，偏保守**。
>
> 关节的功能命名（髋 / 膝 / 踝）同样只能从 CAD 组件名读出，未逐轴向 CAD 作者确认。

## 关节限位与 effort/velocity

| 项 | 来源 | 可信度 |
| --- | --- | --- |
| `lower` / `upper` | **占位值 ±1.5708 rad（±90°）** | ⚠ **不是真实行程** |
| `effort` | 型号库 `motor_hardware_interface/config/motor.yaml` 的 `rated_torque_nm` | 型号推定可信则可信 |
| `velocity` | 同上，`rated_speed_rad_s` | 同上 |
| `<dynamics damping>` | Pinocchio CRBA 零位对角 | 由 CAD 惯量导出 |

导出件的 6 个 revolute 关节 `<limit>` 全是 `0/0/0/0`——**CAD 里没配角度配合**，所以
`lower`/`upper` 无实值可抄。`<limit lower=upper>` 会让 `joint_state_publisher_gui` 连滑杆都不建，
而 `test_model_consistency.py` 要求 revolute 关节的 `lower < upper` 且 `effort`/`velocity` 非零，
所以这里写 ±90° 的占位口径（与 `half_body_robot` 仿真侧 `simulation_topology.yaml` 里的占位值同源），
**只为让模型能加载、滑杆能拖，不代表真实行程**。

**补法**：在 SolidWorks 里给 6 个关节加角度配合后重新导出到 `cad/`，再按同样的适配流程
重生成 `urdf/robot_leg.urdf`，把 `lower`/`upper` 换成实值。CAD 里已经有「大腿旋转轴-限位块」
「膝关节旋转-限位块」「小腿旋转轴-限位块」这些**机械限位零件**，角度配合没做而已。

`effort`/`velocity` 逐关节值：`joint_link1/2` = 20 N·m / 3.1416 rad·s⁻¹（cerb_r58_101）；
`joint_link3/4/5` = 5 N·m / 3.1416 rad·s⁻¹（cerb_r48_101）；
`joint_link6` = 1.15 N·m / 4.189 rad·s⁻¹（vaxor_2550，**50 档是推定**——CAD 只给了
`X25S-BZ153050-MH-1`，型号库里 25mm 系列有 2550/2530 两档，选 2550 是沿用
`half_body_robot` 腕关节的同一选择）。

## 几何包络与 spawn_z

零位（各关节角 = 0）下模型世界坐标包络 **x ∈ [−0.4069, 0.0434]、y ∈ [−0.0808, 0.0741]、
z ∈ [0.0000, 1.2721]**，最低点恰为 z = 0，所以 `gazebo.launch.py` 的 `spawn_z` 默认 **0.0**
可以直接用（与 `half_body_robot` 同）。**换导出件要重新核这个值**——`base_link` 一动，
整条腿的最低点就跟着变。

可视网格共 7 个（`base_link` + 6 个连杆），约 62.7 万三角面，`Joint_link2` 一个就 15.6 万面。

## 已知缺口（接控制/仿真前必须补）

| 缺项 | 为什么 / 怎么补 |
| --- | --- |
| **真实关节限位** | 见上「关节限位与 effort/velocity」。当前是占位 ±90° |
| 电机型号核实 | 见上关节表的告警。`joint_link2` 的两台巨蟹58 与「父 link 驱动下一关节」的另一种读法都要核实 |
| 碰撞体 | 现用全分辨率可视网格（7 个共 62.7 万面）。应按 `half_body_robot/tools/build_simplified_collision_meshes.py` 生成 `meshes/collision/` 凸包，并逐个确认零位无假接触 |
| 自碰撞 | 未开 `<self_collide>`（全分辨率碰撞体 + 自碰撞会拖垮接触检测）。补上简化碰撞体后再开 |
| 电机拓扑 | 缺 `config/simulation_topology.yaml` 与 `config/motor_driver.yaml`——**电机 ID、总线、驱动器型号一概未知**，`simulation.launch.py`（端到端电机仿真）起不来 |
| MuJoCo | 缺 `mujoco/robot_leg.xml`，`mujoco.launch.py` 起不来。生成工具 `tools/build_mujoco_model.py` 按 `models/<目录名>/` 取路径、不写死模型名，可复用，但需要先有简化碰撞体 |
| 与上身的关系 | 本模型是**独立单腿**，`base_link` 没有挂到 `half_body_robot` 的躯干下，也**没有左右镜像** |

## 怎么看

```bash
source /opt/ros/humble/setup.bash && cd ros2_ws
colcon build --symlink-install --packages-select robot_sim && source install/setup.bash

ros2 launch robot_sim display.launch.py model:=robot_leg     # RViz 预览，6 个滑杆
ros2 launch robot_sim gazebo.launch.py  model:=robot_leg     # Ignition，力矩控制
```

Fixed Frame 是 `base_link`，与 `config/display.rviz` 一致。命令个数 = 6，顺序 =
`joint_link1` → `joint_link6`。
