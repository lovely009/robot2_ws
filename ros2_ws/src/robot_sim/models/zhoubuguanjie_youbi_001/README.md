# zhoubuguanjie_youbi_001（右臂，肘部以上）

右臂的 SolidWorks 导出件：手臂支撑座 `base_link`（内含 CERB R58 关节模组）→ 肩 ×3 → 肘，
**4 个主动关节，纯串联树**（无闭环、无 mimic）。几何与惯量取自 CAD。

> 本目录是 **2026-09-17 的导出件**。同一装配体后来在 2026-10-09 又导出过一版（10 个 link，
> 带腕部远置驱动并联连杆 + `config/wrist_closure.yaml`），那一版住在旧包
> `ros2_ws/src/robot_arm_gazebo/models/zhoubuguanjie_youbi_001/`，已于 2026-10-10 连同腕部
> 闭环求解器一并移除。**本件不含腕部**——`Joint_link4` 里那台 X25S 驱动的腕关节在这一版
> 导出里根本不存在。要腕部模型得用 2026-10-09 那份重新导出（`git show 459e8b12:...` 可取回）。

## 目录

| 路径 | 角色 |
| --- | --- |
| `cad/` | **溯源件**：2026-09-17 14:56 的 sw2urdf 原始导出（catkin 包形态）。`urdf/zhoubuguanjie_youbi_001.urdf` + `.csv` 是**几何/惯量/关节原点与轴的唯一来源**，只读不改 |
| `urdf/zhoubuguanjie_youbi_001.urdf` | 适配后的主 URDF，由 `cad/` 那份机械变换而来，见下「已做的适配」 |
| `urdf/zhoubuguanjie_youbi_001.urdf.xacro` | 仿真薄壳（`<ros2_control>` + gz 插件） |
| `urdf/zhoubuguanjie_youbi_001.standalone.urdf` | 派生：薄壳以 `fixed_base:=false` 渲染后剥掉仿真块 |
| `config/controllers.yaml` | gz_ros2_control 参数（含 `joints` 表） |
| `meshes/` | 可视网格，文件名与 link 名一致；与 `cad/meshes/` 是**硬链接**（同一 inode） |
| `tools/regen_derived.py` | 重生成 standalone 与 `dist/`（改主 URDF 后必须重跑） |

## 已做的适配（相对 `cad/` 导出件）

1. 网格路径 → `package://robot_sim/models/zhoubuguanjie_youbi_001/meshes/`。
2. joint 名 `Joint_linkN` → `joint_linkN`（link 名本来就是小写 `Joint_linkN`，保留导出器原样）。
   导出件里 link 名与 joint 名完全相同，而 SDFormat 里 link/joint 共用命名空间，必须拆开。
3. `<limit>`：`lower`/`upper` **照抄 CAD 的实值**；`effort`/`velocity` 换成型号库的值，
   见下「关节限位与 effort/velocity」。
4. 补 `<dynamics damping="…" friction="0.0" />`：damping 取该关节零位等效惯量（Pinocchio
   CRBA 对角，kg·m²），等效 1 s 时间常数；friction 显式置 0（库仑摩擦未标定）。
5. 数值按 5 位有效数字书写（与 `half_body_robot` 同风格）。几何/惯量/关节原点与轴已与
   `cad/` 那份逐项核对一致。

## 关节表

关节轴、质量、组件名均取自 `cad/urdf/zhoubuguanjie_youbi_001.csv`。

| link | joint | 关节轴 (xyz) | 质量 kg | 电机型号（推定） | CAD 组件名（节选） |
| --- | --- | --- | --- | --- | --- |
| `base_link` | — | — | 2.2245 | cerb_r58_101 | 手臂支撑座、**CERB R58 关节模组** |
| `Joint_link1` | `joint_link1` | `0 0 -1` | 0.07625 | cerb_r48_101 | 肩部关节连接旋转轴盖、CERB R48 关节模组 |
| `Joint_link2` | `joint_link2` | `0 0 1` | 0.36781 | cerb_r48_101 | 大臂支架、转换板、40 电机抱闸外壳、CERB R48 关节模组 |
| `Joint_link3` | `joint_link3` | `0 0 -1` | 0.13227 | cerb_r48_101 | 肘部关节旋转轴 + 轴盖、CERB R48 关节模组 |
| `Joint_link4` | `joint_link4` | `0 0 -1` | 0.1555 | vaxor_2550 | 小臂骨架、X25S 关节电机 |

> ⚠ **电机型号按「电机装在父 link、驱动下一个关节」的读法推定**（R58 在 `base_link` 里驱动
> `joint_link1`，R48 在 `Joint_linkN` 里驱动 `joint_linkN+1`），未逐轴向 CAD 作者确认。
> 这个读法与导出件手填的 `effort`（20 / 5 / 5 / 5）**恰好一致**——R58 = 20 N·m、
> R48 = 5 N·m——是它的旁证。`Joint_link4` 里那台 X25S 驱动的是**本版导出里不存在的腕关节**。

## 关节限位与 effort/velocity

| 项 | 来源 | 可信度 |
| --- | --- | --- |
| `lower` / `upper` | **CAD 角度配合的实值**（导出件原样） | CAD 作者手配，未与真机核对 |
| `effort` | 型号库 `motor_hardware_interface/config/motor.yaml` 的 `rated_torque_nm` | 型号推定可信则可信 |
| `velocity` | 同上，`rated_speed_rad_s` | 同上 |
| `<dynamics damping>` | Pinocchio CRBA 零位对角 | 由 CAD 惯量导出 |

逐关节值：

| joint | lower / upper (rad) | effort (N·m) | velocity (rad·s⁻¹) | 导出件手填值 |
| --- | --- | --- | --- | --- |
| `joint_link1` | −1.5708 / 1.5708 | 5 | 3.1416 | effort 20、velocity 2 |
| `joint_link2` | −1.5708 / 1.5708 | 5 | 3.1416 | effort 5、velocity 2 |
| `joint_link3` | −2 / 2 | 5 | 3.1416 | effort 5、velocity 2 |
| `joint_link4` | −1.5708 / 1.5708 | 1.15 | 4.189 | effort 5、velocity 2 |

**为什么覆盖导出件手填的 effort/velocity**：`half_body_robot` 定的口径是「effort/velocity 取
型号库的 `rated_torque_nm` / `rated_speed_rad_s`」，本件沿用。导出件里那组值有两个问题：
`velocity` 四个关节一律填 2（型号库的额定是 3.1416 / 4.189），而 `joint_link1` 填 20 是按
「R58 驱动它」的读法给的——本文件按保守的同名次读法取 5。**两种读法见关节表的告警**，
核实后改这一处即可。

## 几何包络与 spawn_z

零位（各关节角 = 0）下模型世界坐标包络 **x ∈ [−0.0290, 0.1726]、y ∈ [−0.0310, 0.0310]、
z ∈ [0.0000, 0.6018]**，最低点恰为 z = 0，所以 `gazebo.launch.py` 的 `spawn_z` 默认 **0.0**
可以直接用。**换导出件要重新核这个值**。

> 与已移除的 2026-10-09 版不同：那一版臂沿 +x 水平伸出、整臂 x 到 0.52 m，`spawn_z` 要 0.03。
> 本版是竖直朝上的装配位形，不需要抬起。

可视网格共 5 个，约 40.7 万三角面。

## 已知缺口（接控制/仿真前必须补）

| 缺项 | 为什么 / 怎么补 |
| --- | --- |
| 电机型号核实 | 见关节表的告警（两种读法）。导出件手填的 effort 支持另一种读法 |
| 腕部 | 本版导出**不含腕**。要腕部（含远置驱动并联连杆与闭环）得用 2026-10-09 的导出件重做 |
| 碰撞体 | 现用全分辨率可视网格（5 个共 40.7 万面）。应按 `half_body_robot/tools/build_simplified_collision_meshes.py` 生成 `meshes/collision/` 凸包，并逐个确认零位无假接触 |
| 自碰撞 | 未开 `<self_collide>`（理由同 `robot_leg`） |
| 电机拓扑 | 缺 `config/simulation_topology.yaml` 与 `config/motor_driver.yaml`——**电机 ID、总线、驱动器型号一概未知**，`simulation.launch.py` 起不来 |
| MuJoCo | 缺 `mujoco/zhoubuguanjie_youbi_001.xml`，`mujoco.launch.py` 起不来。需先有简化碰撞体 |
| 基座安装 | `base_link` 是手臂支撑座，本模型把它焊在世界原点（`fixed_base:=true`）；挂到 `half_body_robot` 的躯干下要另做 |

## 怎么看

```bash
source /opt/ros/humble/setup.bash && cd ros2_ws
colcon build --symlink-install --packages-select robot_sim && source install/setup.bash

ros2 launch robot_sim display.launch.py model:=zhoubuguanjie_youbi_001   # RViz 预览，4 个滑杆
ros2 launch robot_sim gazebo.launch.py  model:=zhoubuguanjie_youbi_001   # Ignition，力矩控制
```

Fixed Frame 是 `base_link`，与 `config/display.rviz` 一致。命令个数 = 4，顺序 =
`joint_link1` → `joint_link4`。
