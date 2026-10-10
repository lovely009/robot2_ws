# zhoubuguanjie_youbi_002（右臂 · 无腕部连杆）

右臂模型：R58 肩座 + 支撑座 + **6 个直接驱动的串联关节**。几何与惯性由 SolidWorks
装配体导出（SW2URDF），**本次导出取消了腕部远置驱动连杆机构**。

```
zhoubuguanjie_youbi_002/
├── README.md                       # 本文件
├── config/
│   └── controllers.yaml            # gz_ros2_control 参数（含 joints 表）
├── meshes/*.STL                    # 7 个可视网格，URDF <visual> 按 package:// 引用
├── meshes/collision/<连杆>/*.STL    # 凸分解碰撞网格，URDF <collision> 引用（见「碰撞网络」）
└── urdf/
    ├── zhoubuguanjie_youbi_002.urdf            # 模型主件
    ├── zhoubuguanjie_youbi_002.standalone.urdf # 纯模型单文件（无 ros2_control / 插件）
    └── zhoubuguanjie_youbi_002.urdf.xacro      # 仿真薄壳（注入 ros2_control 与 gz 插件）
```

## 与上一版（zhoubuguanjie_youbi_001，已移除）的关系

**同一个臂的两次导出，不是同一个模型。** 差别是结构性的：

| | 上一版 001 | 002（本模型） |
|---|---|---|
| 腕部 | 远置驱动**并联连杆**机构（两台 X25S 固定在小臂，曲柄—连杆—球铰驱动腕） | **取消了连杆**，腕部退化为直接驱动的串联关节 |
| 关节 | 12 revolute + 4 fixed，**含闭环**（URDF 里被切开） | **6 revolute + 0 fixed，纯树、无任何闭环** |
| 闭环补丁 | 需要 `<equality>` / Gazebo SDF 注入 / `wrist_closure_solver.py` | **完全不需要** |
| 安装基准 | `base_link` 底面 z = −0.030，**不含**支撑座 | **含支撑座**：底面 z = −0.0027、柱顶 z = 0.65 |
| `spawn_z` | 0.03 | **0.0027** |

> 001 及其 `wrist_closure_solver.py`、`test_wrist_closure.py`、`dist/zhoubuguanjie_youbi_001_urdf/`
> 已于 2026-10-10 一并移除（旧版本仍在 git 历史里，见 `3f1839e^`）。本表保留作选型对照。

⚠ **`gazebo.launch.py` 的 `spawn_z` 默认 0.03 是给上一版的。** 用本模型必须显式传
`spawn_z:=0.0027`，否则整条臂会悬空 3 cm。

## 关节

| 关节 | 父 → 子 | 类型 | 轴 | 占位限位 (rad) | 对应电机 |
| --- | --- | --- | --- | --- | --- |
| `joint_link1` | `base_link` → `Joint_link1` | revolute | (0, 0, −1) | ±1.5708 | `right_shoulder_yaw`（CERB R58, id 1） |
| `joint_link2` | `Joint_link1` → `Joint_link2` | revolute | (0, 0, −1) | ±1.5708 | `right_shoulder_roll`（CERB R58, id 2） |
| `joint_link3` | `Joint_link2` → `Joint_link3` | revolute | (0, 0, −1) | ±1.5708 | `right_shoulder_pitch`（CERB R58, id 3） |
| `joint_link4` | `Joint_link3` → `Joint_link4` | revolute | (0, 0, 1) | ±1.5708 | `right_elbow_pitch`（CERB R58, id 4） |
| `joint_link5` | `Joint_link4` → `Joint_link5` | revolute | (0, 0, −1) | ±1.5708 | `right_wrist_yaw`（vaxor_2550, id 1） |
| `joint_link6` | `Joint_link5` → `Joint_link6` | revolute | (0, 0, 1) | ±1.5708 | `right_wrist_pitch`（vaxor_2550, id 2） |

电机映射取自 `motor_hardware_interface/config/motor_driver.yaml` 的 `right_arm` 总线。
真机拓扑里还有一个 `right_wrist_roll`（vaxor_2550, id 3），**本模型不含该自由度**，
因此不参与仿真。

**连杆名与关节名不同名**：连杆保留 CAD 原名 `Joint_linkN`，关节为小写 `joint_linkN`。
导出件里 1~4 号连杆是大写 `Joint_LinkN`、5~6 号是小写，移植时已统一。

## ⚠️ 参数来源与必须修改的地方

- **关节限位、力矩、速度全是占位值**：SolidWorks 原始导出为 `0/0/0/0`（会让 RViz 滑杆
  拖不动、Gazebo 直接锁死关节），本包填入 −1.5708~1.5708 rad、10 Nm、1 rad/s —— 对齐
  `config/simulation_topology.yaml` 里 `cerb_r58` profile（沿用上一版的取值）。
  **接真机前必须按实际机械限位与执行器参数修改。**
- 几何与惯性参数由 SolidWorks 模型导出，**未做真机标定**。
- `base_link` 含手臂支撑座（质量 2.868 kg），**和上一版的安装基准不同**，两版模型的
  关节原点不可直接对比。
- 模型本身不含控制器与传感器；控制器参数在 `config/controllers.yaml`，仿真插件由
  `urdf/*.urdf.xacro` 注入。

## 从 SolidWorks 重新导出后的改写步骤

导出的原始件不能直接用，重新导出后要重复以下改写（`git diff` 上一版可以看清每处改动）：

1. mesh 路径 `package://<SolidWorks 导出名>/meshes/` →
   `package://robot_arm_gazebo/models/zhoubuguanjie_youbi_002/meshes/`，网格文件名统一为
   `Joint_linkN.STL`（导出件里 1~4 号是大写 `Joint_LinkN.STL`）；
2. 连杆名 `Joint_LinkN` → `Joint_linkN`，关节名 → 小写 `joint_linkN`
   （空格与大小写混乱进话题/参数名会出问题）；
3. `<limit>` 的 `0/0/0/0` → 占位值（6 个 revolute）；
4. `<material name="">` → `<连杆名>_material`（导出件里不同颜色共用空名字，
   解析器会按名字去重导致颜色错乱）；
5. 重生成 `urdf/*.standalone.urdf`：把 xacro 渲染后剥掉 `<ros2_control>`/`<gazebo>`
   （渲染时用 `fixed_base:=false`，纯模型里不该有 `world` 连杆）；
6. 重生成 `../../dist/zhoubuguanjie_youbi_002_urdf/` 的对外发布件（同一步骤，但
   mesh 路径改成相对路径 `meshes/`）。

`test/test_model_consistency.py` 会校验 1~6 的结果。

## 碰撞网络（meshes/collision/）

**问题**：原始 URDF 的 `<collision>` 直接复用可视网格。Gazebo/DART 与 MuJoCo 都会把
一个网格碰撞体当**单一凸包**处理，而 base_link 的支撑座＋支架、各连杆的凹形腔体一旦
被凸包填充，连 CAD 上根本不相接的连杆对都会凭空“穿透”（实测零位下 base↔link1 显示
−46 mm，而三角面真值只有 −0.16 mm），仿真里表现为恒定的大接触力、关节锁死。

**做法**：每个连杆的可视网格先用 CoACD 凸分解成若干凸块（`threshold=0.05`，与可视网格
同一 link 坐标系、原点不动），再对每块做**平面内缩（erosion）**——把凸块每个面沿法向
内移 δ，δ 由线性规划解出：约束是零位 + 全行程扫掠下**任意连杆对不穿透**（相邻关节对
由引擎自带过滤，不计入），目标是 Σδ 最小，δ 上限为该块的切比雪夫内切半径 −0.5 mm。
δ 不是经验值，是使穿透归零的最小可行解。

| 连杆 | 凸块数 | 内缩 δ | 碰撞体/原始凸包体积 |
| --- | --- | --- | --- |
| `base_link` | 1（仅安装座支架） | 5.54 mm | 0.7 % |
| `Joint_link1` | 51 | 0 | 54 % |
| `Joint_link2` | 38 | 0 | 62 % |
| `Joint_link3` | 52 | 0 | 54 % |
| `Joint_link4` | 34 | 0 | 53 % |
| `Joint_link5` | 21 | 0 | 52 % |
| `Joint_link6` | 4（共 51 块，仅留远端） | 0 | 1.3 % |

**两处刻意的裁剪（必须知道）**：

1. **`base_link` 碰撞不含支撑柱段**，只有安装座支架（z≈0.56~0.64）。原因是占位限位
   ±1.5708 rad 允许臂折进支撑柱——实测 link4 在折到位形时扎进柱体真值深 30 mm，这不是
   网格问题，任何内缩都清不掉。要让支撑柱参与碰撞，正确做法是**按实际机械限位收窄
   关节范围**，而不是把柱体从碰撞里删掉。
2. **`Joint_link6` 只保留 4 块远端小片**。link4↔link6 是**非相邻**对（不被引擎过滤），
   在占位限位下腕部能折到与 link4 真值重叠约 10 mm（同为限位过宽所致），而 link6 的凸块
   太薄（内切半径中位数仅 1.1 mm）无法靠内缩清除，只能裁掉与 link4 干涉的块。

因此本碰撞网络在**占位限位（±1.5708）全行程**内自穿透为 0，但代价是上面两处保真度
损失；接真机时若先按实际机械限位收窄关节范围，可把支撑柱与完整 link6 加回碰撞。

**验收实测**（`/tmp/mjprobe/conp`，MuJoCo 凸几何，`filterparent` 默认）

- 零位：穿透 0；
- 侦察给出的 31 个“最深穿透”位形（base↔link1..6、link4↔link6）：逐个穿透均为 0；
- 自建扫掠 4463 个位形（零位 + 2⁶ 角点 + 单关节全程 + 腕部网格 + 随机满箱）：最大穿透 0。

## 怎么用

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash

# RViz 预览（本模型是唯一的模型，也是两个 launch 的默认值，model:= 可省）。
ros2 launch robot_arm_gazebo display.launch.py model:=zhoubuguanjie_youbi_002

# Gazebo 仿真。⚠ 必须带 spawn_z:=0.0027（本模型含支撑座，默认的 0.03 是给上一版的）。
ros2 launch robot_arm_gazebo gazebo.launch.py  model:=zhoubuguanjie_youbi_002 spawn_z:=0.0027

# 控制器默认 control_mode:=effort（真正的验收路径，见下）。要在 Gazebo 里做位置验收：
ros2 launch robot_arm_gazebo gazebo.launch.py  model:=zhoubuguanjie_youbi_002 spawn_z:=0.0027 control_mode:=position
```

单独加载纯模型：

```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(cat install/robot_arm_gazebo/share/robot_arm_gazebo/models/zhoubuguanjie_youbi_002/urdf/zhoubuguanjie_youbi_002.standalone.urdf)"
```

网格走 `package://robot_arm_gazebo/models/zhoubuguanjie_youbi_002/meshes/`，加载前须
source 本工作空间；要脱离 ROS（Blender / MuJoCo / PyBullet）用
`dist/zhoubuguanjie_youbi_002_urdf/` 那份相对路径版本 —— **本模型是纯树，MuJoCo 里
不需要任何 `<equality>` 约束，导入即可用**。

## 仿真验收实测（2026-10-10）

在 Gazebo（Ignition）里跑通，硬件 `zhoubuguanjie_youbi_002_system` 正常 configure+activate，
6 个关节的 command/state 接口全部就绪。用 `motor_driver.yaml` 里 `right_arm` 的 MIT
力矩律（`τ = kp(p_des−p) + kd(0−v)`，kp=30、kd=1，由外部节点 100 Hz 写入
`/arm_effort_controller/commands`）跟踪：

| 目标位形 | 最大误差 |
| --- | --- |
| 全零 | 0.018 rad（1.05°） |
| 全 0.4 rad | 0.026 rad（1.47°） |
| ±0.5 / ±0.8 rad | 0.030 rad（1.74°） |

**两条坑（对 001 同样成立）：**

- **`control_mode:=position` 不能用来做跟踪验收。** `arm_position_controller` 会认领
  position 命令接口、`/joint_states` 也正常，但关节**不跟随**命令 —— 实测把
  `joint_link2` 依次命令 0.0 / 0.6 / 1.0 / −0.6，反馈全是 ~0.56 rad，即它只是待在
  重力平衡点上。根因是 gz_ros2_control 对 position 命令接口只给很弱的位置比例增益。
  **能用的是 `effort` 模式 + MIT 力矩律**（也是 launch 的默认）。
- **`gravity:=false` 会让关节完全冻死**（位置与速度精确为 0）：没有重力、又没有有效的
  位置力，就没有任何力矩来源。别把它当成 position 模式的验收配置。
- 另外 `ros2 control switch_controllers` 在目标控制器**未被加载**时会打印
  "Successfully switched controllers" 却把当前控制器停掉、留下一个空档 ——
  `control_mode` 没选中 `arm_effort_controller` 时它就不在列表里。要切换请用
  `ros2 run controller_manager spawner <控制器名>`。
