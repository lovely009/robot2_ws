# full_body（全身机器人）

SolidWorks 导出的**上身 + 双臂 18 轴**：`base_link`（躯干，网格 z ∈ `0.0000 .. 0.7029`）
分出两条 9 轴链 —— 第一链 `Joint_link1..9`（x ≈ +0.06 侧）与第二链 `Joint_link1.1..9.9`
（x ≈ −0.05 侧）。机械几何、关节原点/轴、质量与惯量与 `models/robot_all` **逐字相同**
（同一次 CAD 导出），差别只在「仿真侧参数」「控制接口」与「电机映射」：

| # | 差异 | robot_all | full_body | 为什么 |
|---|---|---|---|---|
| 1 | 命令接口 | `position` + `effort` 两个 | **只有 `effort`**（力矩控制） | 同时声明两个接口时，gz_ros2_control 在加载期即置 EFFORT 位并持续写 `JointForceCmd`，而 gz-sim 在 `JointForceCmd` 存在时忽略 `JointVelocityCmd`（`physics/Physics.cc`：“Only set joint velocity if joint force is not set”）→ **position 命令被静默忽略**（2026-10-12 实测；A/B 对照见文末） |
| 2 | 状态接口 | position / velocity | position / velocity / **effort** | robot_all 的 `/joint_states` 里 effort 恒为 NaN，仿真里看不到关节力矩 |
| 3 | 关节动力学 | 无 `<dynamics>` | 18 个关节都有 `damping` / `friction` | 无阻尼时除控制器 kd 外没有任何耗散，撞限位后持续振荡 |
| 4 | 碰撞体 | 复用全分辨率可视网格（全模型 ≈146 万三角面，`base_link` 一个就 50 万） | 3 个 `<box>`（躯干每边内缩 10 mm）+ 14 个凸包（顶点内缩 3 mm，每个 384–554 面）+ 腕端 `Joint_link9/.9` 不参与 | 接触检测开销；配合自碰撞后还必须留出接触余量，见「碰撞体为什么这么配」 |
| 5 | 自碰撞 | SDF 默认 `false`（两臂可互相穿透） | `<self_collide>true</self_collide>`（必须用 SDF 名：gazebo-classic 的 `<selfCollide>` 会被 sdformat 静默丢弃） | 双臂模型做轨迹验收必须能真的碰到。实测两条链都能被力矩驱动（`joint_link2.2` 施加 −2 N·m：+0.97 → −1.53 rad，速度一路贴着 1 rad/s 速度限），而"撞躯干/自身"的方向会被真实挡住（指令力矩全被接触吸收） |
| 6 | 接触摩擦 | SDF 默认（mu = 1） | 19 个 link 显式 `<mu1>`/`<mu2>` = 0.8 | 参数可查、可标定；`<ode><kp>/<kd>` 对 dartsim 无效故不写 |
| 7 | 电机映射 | 9 台（只第一链，id 1–7 全 r58 + 8–9 vaxor） | **18 台，两条手臂都驱动**（id 1..9 + 10..18，型号序列见下） | 全身模型两臂都要能被力矩驱动 |
| 8 | 重力 | 第一链受 `gravity` 参数控制，第二链恒关（无电机） | 两条链都由 `gravity` 参数统一控制 | 第二链现在有电机，不需要"关重力防空转"的例外 |
| 9 | 关节限位 `lower/upper` | 18 个关节统一占位 `±1.5708` | **CAD 真实角度限位**（逐关节不同、第二链镜像；见「关节限位」节） | 仿真的可达范围要反映真实机械边界，轨迹验收才有意义 |

机械本体的**关节限位 `lower`/`upper` 是 CAD 真实角度限位**（见下一节）；`effort`（10 N·m）与
`velocity`（1 rad·s⁻¹）仍是占位值——导出件里这两项是 0，会让 Gazebo 锁死关节，接真机前必须按
执行器参数替换。

## 关节限位（来自 CAD 角度配合）

来源：`models/胸部整体结构装配体-上身部分/`（2026-10-12 15:51 用 sw2urdf 重新导出的同一份
装配体，带 SolidWorks 的角度配合上下限）。`robot_all` 里是统一占位 `±1.5708`；本模型换成真实值，
**只取 `lower`/`upper`**，`effort`/`velocity` 沿用占位。

| 关节 | 限位 (rad) | ≈ 度 | 关节 | 限位 (rad) | ≈ 度 |
|---|---|---|---|---|---|
| `joint_link1` | −0.17 … +2.62 | −10 … 150 | `joint_link1.1` | −2.62 … +0.17 | −150 … 10 |
| `joint_link2` | −0.26 … +0.26 | ±15 | `joint_link2.2` | −0.26 … +0.26 | ±15 |
| `joint_link3` | −3.14 … +0.52 | −180 … 30 | `joint_link3.3` | −0.52 … +3.14 | −30 … 180 |
| `joint_link4` | 0 … +1.57 | 0 … 90 | `joint_link4.4` | −1.57 … 0 | −90 … 0 |
| `joint_link5` | ±1.57 | ±90 | `joint_link5.5` | ±1.57 | ±90 |
| `joint_link6` | −2.09 … +0.09 | −120 … 5 | `joint_link6.6` | −2.09 … +0.09 | −120 … 5 |
| `joint_link7` | ±1.57 | ±90 | `joint_link7.7` | ±1.57 | ±90 |
| `joint_link8` | ±1.57 | ±90 | `joint_link8.8` | ±1.57 | ±90 |
| `joint_link9` | ±1.57 | ±90 | `joint_link9.9` | ±1.57 | ±90 |

- 数值都落在整十/整五度上，且第二链与第一链**镜像对称**（`−10…150` ↔ `−150…10`、
  `−180…30` ↔ `−30…180`、`0…90` ↔ `−90…0`）——这是"设计值"而非随手填的证据。
- ⚠ **`joint_link6` 与 `joint_link6.6` 是例外**：CAD 里两侧相同（`−120…5`），不满足上面的镜像
  规律。已按 CAD 原值保留；若后续确认应镜像（`−5…120`），改派生来源或直接改主 URDF 后重跑
  `tools/` 的两条验收即可。
- 实测生效：`joint_link2.2` 施加 ±2 N·m 后精确停在 CAD 的 **±0.2600 rad**（旧占位值下会摆到
  ±1.5708）。
- ⚠ 活动范围比占位值**窄很多**（`joint_link2` 从 ±90° 收窄到 ±15°，`joint_link6` 从 ±90° 收窄到
  −120°…5°）：之前用占位限位验过的示教轨迹/动作可能超限，播放前先核对。
- ⚠ `config/simulation_topology.yaml` 里 profile 的硬/命令限位是**型号级**粗检查（±90°/±85°），
  比逐关节的真实限位宽——两者口径不同：真实限位由 Gazebo 物理层强制，profile 只做整体兜底。

> ⚠ **不支持 `control_mode:=position`**：本模型没有 position 命令接口，`arm_position_controller`
> 会在 spawn 时明确报错（有意的失败，而不是静默不动）。仿真统一走力矩：`gazebo.launch.py`
> 默认 `control_mode:=effort`，`simulation.launch.py` 内部恒传 `effort`。

## 电机映射（两条手臂，从上到下 = 靠躯干 → 末端）

型号序列：**rb48 → rb48 → rb58 → rb58 → rb48 → rb48 → rb48 → vaxor → vaxor**
（`rb48` = `cerb_r48_101`、`rb58` = `cerb_r58_101`、`vaxor` = `vaxor_2550`）

| # | 第一链关节 | 电机名 | CAN id | 型号 | 第二链关节 | 电机名 | CAN id | 型号 |
|---|---|---|---|---|---|---|---|---|
| 1 | `joint_link1` | `arm_joint1` | 1 | `cerb_r48_101` | `joint_link1.1` | `arm_joint1.1` | 10 | `cerb_r48_101` |
| 2 | `joint_link2` | `arm_joint2` | 2 | `cerb_r48_101` | `joint_link2.2` | `arm_joint2.2` | 11 | `cerb_r48_101` |
| 3 | `joint_link3` | `arm_joint3` | 3 | `cerb_r58_101` | `joint_link3.3` | `arm_joint3.3` | 12 | `cerb_r58_101` |
| 4 | `joint_link4` | `arm_joint4` | 4 | `cerb_r58_101` | `joint_link4.4` | `arm_joint4.4` | 13 | `cerb_r58_101` |
| 5 | `joint_link5` | `arm_joint5` | 5 | `cerb_r48_101` | `joint_link5.5` | `arm_joint5.5` | 14 | `cerb_r48_101` |
| 6 | `joint_link6` | `arm_joint6` | 6 | `cerb_r48_101` | `joint_link6.6` | `arm_joint6.6` | 15 | `cerb_r48_101` |
| 7 | `joint_link7` | `arm_joint7` | 7 | `cerb_r48_101` | `joint_link7.7` | `arm_joint7.7` | 16 | `cerb_r48_101` |
| 8 | `joint_link8` | `arm_joint8` | 8 | `vaxor_2550` | `joint_link8.8` | `arm_joint8.8` | 17 | `vaxor_2550` |
| 9 | `joint_link9` | `arm_joint9` | 9 | `vaxor_2550` | `joint_link9.9` | `arm_joint9.9` | 18 | `vaxor_2550` |

- ⚠ **两条链用同一型号序列是假设**（第二链是镜像侧）；实机若不对称，改
  `config/motor_driver.yaml` 与 `config/simulation_topology.yaml` 即可，无需改代码。
- 18 台电机挂在**同一条总线**（逻辑名 `arm`，仿真里默认 `/dev/ttyACM0` 或 socketcan 接口）；
  同总线 id 必须唯一（固件注册表以 motor_id 为键）。
- 拓扑唯一来源是 YAML：`config/motor_driver.yaml`（型号/id/增益/MIT 限幅）+
  `config/simulation_topology.yaml`（关节映射 + profile）。
- 仿真 profile 分三档（`cerb_r48` / `cerb_r58` / `vaxor_2550`）：硬/命令限位是**型号级粗兜底**
  （±90°/±85°、1 rad/s）——比 URDF 里逐关节的真实限位宽，真实边界由 Gazebo 物理层强制；
  `effort_nm` 统一取保守的 10 N·m（= URDF 的占位 `effort`；按型号可放宽到型号库峰值：
  r48 11 N·m、r58 45 N·m）。
- MIT 限幅（发给电机的协议参数，见 `motor_driver.yaml` 的 `parameters`）：cerb 取型号库
  `peak_torque_nm`（r58 45 / r48 11），位置/速度限幅沿用 12.5 rad / 30 rad·s⁻¹（未标定）。

## 目录

| 文件 | 说明 |
|---|---|
| `urdf/full_body.urdf` | 主 URDF：双臂 18 关节 + `<dynamics>` + 简化碰撞体 |
| `urdf/full_body.urdf.xacro` | Gazebo 薄壳：`<ros2_control>`（**只 effort 命令接口**）、自碰撞、摩擦、重力开关、`gz_ros2_control` 插件 |
| `urdf/full_body.canfd.xacro` | 主机 `motor_hardware_interface` 薄壳（`controller_mode:=host`） |
| `urdf/full_body.standalone.urdf` | 派生单文件版（无控制器、无 Gazebo 插件） |
| `config/controllers.yaml` | `gz_ros2_control` 参数：`joint_state_broadcaster` + `arm_effort_controller`（`arm_position_controller` 节保留但不可用，见文件内注释） |
| `config/motor_driver.yaml` | **电机拓扑**：18 台电机（型号/id/增益/MIT 限幅），总线逻辑名 `arm` |
| `config/simulation_topology.yaml` | 端到端仿真的电机↔关节映射（18 条）+ 三档 profile |
| `meshes/*.STL` | 19 个二进制 STL：与 `models/robot_all/meshes/` **硬链接同一份文件**（不占额外磁盘，改一处两边同时变，别单独改） |
| `meshes/base_link_simplified.STL` | `base_link` 降采样版（50 万面 → 2.3 万面，MuJoCo 解码上限 20 万面），由 `tools/simplify_mesh.py` 生成 |
| `meshes/collision/*.STL` | 14 个凸包碰撞网格，由 `tools/build_simplified_collision_meshes.py` 生成 |
| `mujoco/full_body.xml` | 生成的 MJCF（供 `mujoco.launch.py` 使用），由 `tools/build_mujoco_model.py` 生成，**别手改** |
| `tools/build_simplified_collision_meshes.py` | 凸包/box 参数生成器（`--check` 只校验不写文件） |
| `tools/check_collision_interpenetration.py` | 零位碰撞体相交验收（SAT，退出码 0 = 无相交） |
| `tools/build_mujoco_model.py` | URDF → MJCF 生成器（转换 + 补碰撞 geom/exclude/actuator + 自检） |
| `tools/simplify_mesh.py` | STL 顶点聚类降采样（生成 `*_simplified.STL`） |

`dist/full_body_urdf/` 是对外发布件（相对路径网格、无 ROS 依赖），由 `test_model_consistency.py`
兜底防分叉。

## 物理参数依据（**全部未标定**，替换时按此表定位）

| 参数 | 取值 | 依据 | 替换建议 |
|---|---|---|---|
| `<limit>` `lower`/`upper` | 逐关节不同（见「关节限位」节） | **CAD 角度配合的真实设计值**（sw2urdf 导出件） | 机械改动后重新导出并重跑派生 |
| `<limit>` `effort` / `velocity` | 10 N·m / 1 rad·s⁻¹（占位） | 导出件里这两项是 0（会锁死关节），暂沿用 robot_all 的占位 | 按执行器额定值替换 |
| `<dynamics damping>` | 逐关节不同（`joint_link2` 0.0473 … `joint_link9/.9` 1.42e-06） | 该关节**零位等效惯量**（Pinocchio CRBA 对角，kg·m²）数值，等效 1 s 时间常数 | 真机可用「同幅值阶跃下的速度衰减时间」反推 |
| `<dynamics friction>` | 0.0（显式写出） | 库仑摩擦没有实测数据；给非零值会让腕部小关节出现未建模死区 | 真机用小力矩斜坡测起动力矩后填回 |
| `<mu1>`/`<mu2>` | 0.8 | 铝/塑料件干摩擦量级估计 | 需要接触滑移类结论时按实测摩擦角替换 |
| `<self_collide>` | true | 双臂模型需要真实自碰撞 | 保持 |
| `<box>` / 凸包 | 见 `urdf/full_body.urdf` 与上一节（躯干每边内缩 10 mm、凸包内缩 3 mm） | 原网格包围盒（box）与顶点凸包（网格），再留接触余量 | 接触位置要求精确时换回原网格（有 trimesh 开销）或按实物外形重做，替换后必须跑零位相交验收 |

## 碰撞体为什么这么配（附验收）

自碰撞打开后，**非相邻** link 的碰撞体只要在零位或常用姿态下相交/贴住，dartsim 的接触刚度
（大）就会把关节顶死。实测过两轮：

- `base_link` 用整高整宽的 box：零位与 `Joint_link2` 假接触 15.8 mm，joint_link2 加 2 N·m
  完全不动；换成原始网格做碰撞虽然零位不相交（CAD 装配本身无穿插），50 万面的 trimesh 参与
  碰撞却会让 ODE 检测刷 `Trimesh-trimesh ... hash table bucket overflow` 并拖垮控制器切换
  （spawner 超时）。
- 截断到 0.68 m 后零位 OK，但**手臂自然下垂**时凸包又贴住躯干（`joint_link2` 0.00 mm、
  `Joint_link7` 1.24 mm），手臂只能垂到 −0.32 rad（真机能垂到 −0.96 rad）。

最终方案：**实心简化形状 + 统一留接触余量**。

| link | 碰撞体 | 余量 |
|---|---|---|
| `base_link` | `<box>` 0.1865 × 0.1195 × 0.66（原包围盒 0.2065 × 0.1395 × 0.7029） | **每边内缩 10 mm**（避开两肩让位区与侧面） |
| `Joint_link1..7`（含 `.N` 镜像，12 个） | 凸包 | **顶点朝质心内缩 3 mm**（`SHRINK_M`） |
| `Joint_link8/.8` | `<box>`（原包围盒） | 小垫片，本身近似方块 |
| `Joint_link9/.9` | **不提供 `<collision>`** | 被 link7 的凸包整个包住，实心形状必然假接触（缩到包围盒 ×0.8 仍穿透 9.3 mm） |

改完碰撞体后必须跑两条验收（在模型目录内）：

```bash
python3 tools/check_collision_interpenetration.py            # 零位非相邻对 0 相交，退出码非 0 即失败
python3 tools/build_simplified_collision_meshes.py --check   # 凸包与原始 STL 仍一致
```

改姿态相关参数（限位/零位/凸包内缩量）后，还建议按实际会出现的姿态复核一遍——`check_collision_interpenetration.py`
只检查零位，把 `POSE` 换成目标姿态即可复用其判定逻辑。

## 用法

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws && colcon build --symlink-install --packages-select robot_arm_gazebo
source install/setup.bash

# RViz 预览（滑杆拖动 18 个关节）
ros2 launch robot_arm_gazebo display.launch.py model:=full_body

# Ignition 仿真（力矩控制；无控制器时会下垂，可用 gravity:=false 锁住重力）
ros2 launch robot_arm_gazebo gazebo.launch.py model:=full_body

# 端到端电机—机械臂仿真（Gazebo + motor_sim + 18 台电机，默认走本机 robot_vcan0）
ros2 launch robot_arm_gazebo simulation.launch.py model:=full_body

# MuJoCo 仿真（独立后端，话题契约与 Gazebo 一致：effort 命令 ↔ joint_states）
ros2 launch robot_arm_gazebo mujoco.launch.py model:=full_body            # 带 viewer
ros2 launch robot_arm_gazebo mujoco.launch.py model:=full_body headless:=true
```

`controller_mode:=firmware`（STM32 HIL）时 18 台电机映射会按 id 传给 FDCAN 模拟器
（`--cerb 1,2,3,... --vaxor 8,9,17,18` 由 launch 按拓扑自动生成）。

## MuJoCo 后端

`mujoco.launch.py` 用**自写的力矩直通桥**（`robot_arm_gazebo/scripts/mujoco_effort_bridge.py`）
驱动 `mujoco/full_body.xml`，订阅 `/arm_effort_controller/commands`、发布 `/joint_states`，
**话题契约与 Gazebo 那条路完全一致**，所以 `simulation.launch.py`、web_host、手工
`ros2 topic pub` 的操作方式都不用改：

```bash
ros2 topic pub -r 50 /arm_effort_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}"   # 顺序 = joint_link1..9 → 1.1..9.9
```

- 力矩直通（`<motor gear="1">`），不依赖 ros2_control；`command_timeout_s`（默认 0.5 s）
  内没有新命令即清零，与 `motor_sim` 的命令超时语义一致。
- `/joint_states` 的 `effort` 取 `qfrc_actuator`（执行器广义力），与 Gazebo 侧读到的
  「关节传递力矩」略有差别。
- 实测（2026-10-12，MuJoCo 3.6.0）：joint_link2 加 2 N·m 到 CAD 限位 0.264（软约束允许
  ~4e-3 rad 穿透）；镜像链 joint_link2.2 加 −2 N·m 对称到 −0.264；停命令后重力把臂拉回
  下部限位，两链对称。
- **为什么不用 `mujoco_ros2_control` 的 ros2_control 集成**：本机 0.1.2（apt）在注册
  actuator 时创建话题 `pid_gains/position/<joint>/<index>/pid_state`，ROS 2 话题名不允许
  纯数字 token → `InvalidTopicNameError` 直接 abort（第 11 个 actuator 起必崩）。
  另外它把 MJCF 无 actuator 的关节注册为 passive，effort 接口会导出失败。
- **MJCF 由脚本生成**（`tools/build_mujoco_model.py`）：转换器只产 visual geom、不产
  actuator、也不产 contact exclude，所以脚本后处理补三样：碰撞 geom（凸包 + `base_link`
  box，`density="0"` 避免重复计质量）、18 个 `<contact><exclude>`（`base_link` 与每个
  可动 link，防自碰撞把基座顶住）、18 个 `<motor>`。生成后有自检
  （nq/nv/ngeom/nexclude/nu 计数 + 关节名清单）。**改 URDF 后必须重跑生成器。**
- MuJoCo 的 STL 解码上限是 20 万面，`base_link` 原始网格 50 万面 → 用
  `tools/simplify_mesh.py` 生成 `meshes/base_link_simplified.STL`（2.3 万面）专供 MJCF。
- viewer 需要 `~/.ros/ros2_control/.venv`（`mujoco_ros2_control` 的 apt 包建的，里面有
  mujoco python 包）；launch 自动挑它，找不到则退回系统 python3 并降级为无头。

## 已知缺口

- **碰撞体是近似**：凸包比原网格「胖」（已统一内缩 3 mm 留余量），躯干 box 每边比实物小
  10 mm，腕端两个小法兰不参与碰撞检测。要精确接触位置就得换回原始网格，但会付出 trimesh
  碰撞开销（见「碰撞体为什么这么配」）。手臂深垂到与躯干重叠的姿态时会被真实挡住。
- **URDF 里写 Gazebo 参数必须用 SDF 名**：`<self_collide>` 生效，而 gazebo-classic 的
  `<selfCollide>` 会被 sdformat 的 URDF→SDF 转换**静默丢弃**（`ign sdf -p` 复核）。`<mu1>`/
  `<mu2>` 则有映射，会被转成 `<surface><friction><ode><mu>/<mu2>`。
- 力矩幅值**不由 URDF 或控制器钳位**：`<limit effort>` / 命令接口的 min/max 在
  gz_ros2_control + dartsim 这条链上都不生效；仿真侧的力矩钳位在 `motor_sim` 的
  `effort_limit_nm`（见 `config/simulation_topology.yaml` 的 profiles）。
- 关节速度限 `1 rad/s` 实测**是生效的**（dartsim 的速度限），快速轨迹验收时注意它是瓶颈。
- 本体惯量未含减速器反射转子惯量（型号库只有 Kt / 减速比 / 效率），仿真里关节比真机「轻」；
  轨迹跟随/整定的绝对结论不能直接搬真机。
- 两条链的型号序列相同是**假设**；id 1..9 / 10..18 的分配是本模型的约定，接实机前请按真机
  节点号核对 `config/motor_driver.yaml`。
- **MuJoCo 后端是简化近似**：MJCF 里的碰撞体沿用 Gazebo 侧的简化形状（凸包 + 躯干 box）；
  关节限位是 MuJoCo 软约束（实测越界 ~4e-3 rad），Gazebo 侧同样是近似；两套引擎的接触刚度
  与摩擦模型不同，**同一组力矩在两边的轨迹不会完全一致**。`damping` 沿用 URDF 导出，
  MuJoCo 侧的 `frictionloss` 为 0（URDF 的 `friction` 没有映射过来）。
- MJCF **不随 URDF 自动更新**：改 URDF/凸包/限位后要重跑
  `tools/build_mujoco_model.py`（生成器自检会拦住明显漏项，但仍需人工复核）。

## 附：position 接口失效的实测（2026-10-12，Gazebo 6.18 + gz_ros2_control Humble）

- `models/robot_all`（position + effort 双接口）：激活 `arm_position_controller` 后发 18 维位置
  指令，关节**完全不动**；`ros2 control` 显示控制器 active。
- A/B 对照：把 18 处 `<command_interface name="effort"/>` 删掉、其余不变，同一套 launch 下
  position 指令 1 s 内精确到位（0.5/0.4/0.3/0.2，误差 < 1e-4）。
- 结论：position 接口本身可用（`position_proportional_gain` 默认 0.1，一阶速度环），失效原因是
  effort 接口抢先创建了 `JointForceCmd`。本模型按「均使用力矩控制」只导出 effort。
