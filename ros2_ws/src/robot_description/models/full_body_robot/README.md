# full_body_robot

无头部全身模型，原导出名为 `全身装配（不含头部）`，现纳入 `robot_description` 的公共模型目录。
包含腰胯/躯干、双臂和双腿/脚板；38 个连杆、37 个转动关节条目，总声明质量约 11.37637 kg。
用户已确认每个相应 joint 只负责 1 个转动自由度；每侧腕部的 `3.8` 与 `3.9` 两个关节合起来负责 2 个自由度。
37 台电机的编号与关节分组见下方。差动传动关系和闭环约束仍待标定。

- `cad/`：完整保留原始 46 个文件，包括 URDF、CSV、STL、ROS 1 导出包及日志。
- `urdf/full_body_robot.urdf`：主 URDF，腰部串联为 `1 → 3 → 2`，左腕为 `3.7 → 3.9 十字轴 → 3.8 手板`。
- `urdf/full_body_robot.mapping.urdf`：离线映射入口，目前与主 URDF 内容一致。
- `meshes/`：原始 STL 的逐字节副本，未简化或重新估算几何。

两份 URDF 的关节名使用 VDMocap SDK 的 `BN_*` 身体节点名作为前缀。
上身旋转用 `BN_Spine_yaw`，两侧踝关节用 `BN_*Foot_pitch` 和 `BN_*Foot_roll`，
其余机械轴在节点内用 `axis_1`、`axis_2`、`axis_3` 编号。两侧腕部都按从前臂到手板的顺序编号，左腕 `axis_1`、`axis_2` 分别对应原 CAD 的 3.9、3.8。
每个子连杆使用对应关节名加 `_link`，例如 `Joint_Link1` 对应 `BN_Spine_yaw_link`。
`base_link` 是 RViz 的固定参考帧。下文中的 CAD 编号用于追溯原始装配关系。
两份 URDF 均将 joint2 的 parent 改为 Link3，并用 `inverse(T_1_3) · T_1_2` 重表达 origin；
左腕依据 CSV 零件归属和相交的正交轴交换 3.8/3.9 的串联顺序，并重表达固定 origin，
使两个腕轴都能带动实际手板（右手板原本已在末端 3.9r，无需交换）。
所有 link 的零位世界位姿和 visual/collision/inertial frame 保持等价，其他参数逐元素保留。
网格引用统一为 `package://robot_description/models/full_body_robot/meshes/...`，
由现有 Web 模型选择器和 ROS 2 资源路径解析，无需另建 ROS 包。
原始中文包名和 ROS 1 launch 仅作为来源记录保留在 `cad/`。

## 电机拓扑

拓扑配置见 [config/motor_topology.yaml](config/motor_topology.yaml)，待确认参数用 `null` 表示。
表中的名称是 URDF 关节名。单电机条目为一对一分配，双电机条目为两台电机共同控制两个关节。
编号采用本模型的电机编号，CAN 总线和节点 ID 另行确认。

| 电机编号 | 传动分组 | URDF 关节 |
|---|---|---|
| 1 | 一对一 | `BN_Spine_yaw` |
| 2、3 | 腰部差动 | `BN_Spine_axis_1`、`BN_Spine_axis_2` |
| 4、5 | 胯部差动 | `BN_Hips_axis_1`、`BN_Hips_axis_2` |
| 6、7 | 左肩差动 | `BN_LeftShoulder_axis_1`、`BN_LeftShoulder_axis_2` |
| 8 | 一对一 | `BN_LeftUpperArm_axis_1` |
| 9 | 一对一 | `BN_LeftUpperArm_axis_2` |
| 10 | 一对一 | `BN_LeftUpperArm_axis_3` |
| 11 | 一对一 | `BN_LeftLowerArm_axis_1` |
| 12 | 一对一 | `BN_LeftLowerArm_axis_2` |
| 13、14 | 左腕差动 | `BN_LeftHand_axis_1`、`BN_LeftHand_axis_2` |
| 15、16 | 右肩差动 | `BN_RightShoulder_axis_1`、`BN_RightShoulder_axis_2` |
| 17 | 一对一 | `BN_RightUpperArm_axis_1` |
| 18 | 一对一 | `BN_RightUpperArm_axis_2` |
| 19 | 一对一 | `BN_RightUpperArm_axis_3` |
| 20 | 一对一 | `BN_RightLowerArm_axis_1` |
| 21 | 一对一 | `BN_RightLowerArm_axis_2` |
| 22、23 | 右腕差动 | `BN_RightHand_axis_1`、`BN_RightHand_axis_2` |
| 24 | 一对一 | `BN_LeftUpperLeg_axis_1` |
| 25 | 一对一 | `BN_LeftUpperLeg_axis_2` |
| 26 | 一对一 | `BN_LeftUpperLeg_axis_3` |
| 27 | 一对一 | `BN_LeftLowerLeg_axis_1` |
| 28 | 一对一 | `BN_LeftLowerLeg_axis_2` |
| 29 | 一对一 | `BN_LeftFoot_pitch` |
| 30 | 一对一 | `BN_LeftFoot_roll` |
| 31 | 一对一 | `BN_RightUpperLeg_axis_1` |
| 32 | 一对一 | `BN_RightUpperLeg_axis_2` |
| 33 | 一对一 | `BN_RightUpperLeg_axis_3` |
| 34 | 一对一 | `BN_RightLowerLeg_axis_1` |
| 35 | 一对一 | `BN_RightLowerLeg_axis_2` |
| 36 | 一对一 | `BN_RightFoot_pitch` |
| 37 | 一对一 | `BN_RightFoot_roll` |

差动组的传动矩阵（同向与反向分别对应的关节、符号与比例）、零位和机械限位待确认。当前表格用于记录编号和分组。

## 预览和离线映射

### 整机 MuJoCo

[`mujoco/full_body_robot.xml`](mujoco/full_body_robot.xml) 包含主 URDF 的 38 个连杆、
25 路一对一输入及六组差动的 12 路输入。整机保留了 10 根定长连杆的闭环，可在同一窗口调试全身。

在 **仿真工作台**选择 **MuJoCo → 外部控制器 → vcan（纯软件）→ full_body_robot** 后启动。
切到 **调试 → 关节**，等到面板显示 **MuJoCo 位置控制** 后即可拖动 37 个关节滑块。
直驱关节发送位置目标，差动关节按球头与定长连杆几何求解电机角，并保留同组另一轴的目标。
直驱范围约 ±10°，差动输出范围为腰 ±0.075 rad、胯 ±0.035 rad、肩 ±0.070 rad、腕 ±0.045 rad。
仿真发布 `/joint_states`、`/tf` 和 `/clock`。工作台的整机视图和仿真 TF 都使用从 MJCF 生成的
`full_body_robot.standalone.urdf`。仿真发布 67 个关节状态，其中 37 个对应工作台滑块，30 个用于显示差动法兰和连杆。

```bash
~/.ros/ros2_control/.venv/bin/python -m mujoco.viewer \
  --mjcf /home/ly/robot2_ws/ros2_ws/src/robot_description/models/full_body_robot/mujoco/full_body_robot.xml
```

在 **Control** 面板拖动滑块，输入单位 rad，调试范围 ±10°。`motor_01` 等沿用一对一电机编号；
差动滑块以 `waist_`、`hips_`、`left_shoulder_`、`right_shoulder_`、`left_wrist_`、`right_wrist_` 分组。
在 **Simulation** 面板可加载零位及六组同时同向或反向的关键帧。

模型底座固定，重力和接触关闭。腰、胯、左右肩和左右腕的法兰、连杆和平台端球头从原始 CAD 网格拆出。
腰部拆件沿 X 各向内平移 1 mm，以配合名义 100 mm 球头间距。腕部的输出盘与外端盖在 STL 中连成一片，
显示模型按电机轴上半径 8.55 mm 的圆柱面分开两者，内盘和球头随电机转动。
需要拆件或降面的外观网格生成到 `mujoco/visual/`，原始 CAD 网格保留。
MJCF、显示副本和仿真 URDF 可用 MuJoCo 环境执行 `tools/build_mujoco_model.py` 重建，需 VTK、NumPy、SciPy 和 PyYAML。
原始 STL 及六份单组 MJCF 是生成器输入，`mujoco/visual/`、整机 MJCF 和 standalone URDF 一并纳入版本管理。
工作台按上次提交的整组关节目标保持位置。在同一仿真会话中交替操作 Viewer 和工作台时，工作台会恢复其保存的其他关节目标。

### 单组闭环调试

腰部差动可单独用 MuJoCo 调试。模型使用零位 CAD 球头坐标与 100 mm 名义间距，
用两条固定长度约束表示闭环连杆，偏航轴固定。打开方式：

```bash
~/.ros/ros2_control/.venv/bin/python -m mujoco.viewer \
  --mjcf /home/ly/robot2_ws/ros2_ws/src/robot_description/models/full_body_robot/mujoco/waist_differential.xml
```

右侧 **Control** 面板的 `flange_plus_x` 和 `flange_minus_x` 均按共同的 +X 右手方向计角，
单位为 rad，调试范围 ±10°。左侧 **Simulation** 可加载零位、同向和反向三个关键帧。
模型采用简化几何、零重力和仿真伺服参数，用于验证闭环运动方向。物理电机编号对应、编码器正向、机械限位和负载能力待实机标定。

胯部可用同一命令加载 [`mujoco/hips_differential.xml`](mujoco/hips_differential.xml)。两侧法兰按
`base_link` 的 +Z 右手方向计角，滑块名中的 +X、−X 表示安装侧。两根 65 mm 连杆前端接入
前横梁的安装孔，球心沿孔轴高于安装面 4 mm。34 mm 摆臂在仿真零位选取向外的装配分支，
相对 CAD 导出姿态，+X 侧转约 −53.77°，−X 侧转约 +74.88°。等量同向主要驱动 `BN_Hips_axis_1`，
等量反向主要驱动 `BN_Hips_axis_2`。4、5 号对应安装侧及编码器正向仍待确认。

| 上肢模型 | 几何和滑块 |
|---|---|
| [左肩](mujoco/left_shoulder_differential.xml)、[右肩](mujoco/right_shoulder_differential.xml) | 参照试用手臂的空间连杆结构。两侧使用整机胸部 CAD 各自的球心和 URDF 转轴，连杆球心距约 106.906 mm。滑块 `axis_1_direct` 使用各自 URDF 第一轴正向，`flange_linkage` 使用胸部局部 +X 右手方向。 |
| [左腕](mujoco/left_wrist_differential.xml)、[右腕](mujoco/right_wrist_differential.xml) | 分别从两侧前臂 CAD 提取球心，电机轴由安装面和紧固件位置估算。近端、远端连杆球心距分别约 101.500、66.006 mm。滑块 `flange_proximal`、`flange_distal` 的转轴均取前臂坐标下 X 分量为正。 |

上肢模型也使用 ±10° 法兰调试输入和零重力。肩部单独拖动 `axis_1_direct` 时，闭环会同时改变
`axis_2`。按模型的法兰正向，左腕同向主要带动 `axis_1`，右腕同向主要带动 `axis_2`。
这些角度对应 CAD 几何约定，实机电机编号与编码器正向仍待确认。

可以在 Web 模型选择器中选择 `full_body_robot`，或启动 RViz 几何预览：

```bash
ros2 launch robot_sim display.launch.py model:=full_body_robot
```

当前 37 个关节的 `lower/upper/effort/velocity` 均为 0，按限位工作的预览器不能拖动关节。
电机拓扑目前用于记录编号和分组。上面的 MuJoCo 模型为独立几何调试入口，整机驱动和 HIL 配置待实机标定。
原始网格约 330 万个三角面，加载可能较慢，其中 3 个 STL 超过本机 MuJoCo 的面数上限。

已实测调整的手臂仍以 `right_arm_4dof` 的 4.4 来源为基准；本次没有用全身 CAD 参数覆盖它。

以下保留原整合版本已确认的离线映射约定。原专用全身 BVH 工具保留在归档提交 `1c5b98ef` 中。
当前通用动作转换通过 `bvh_broadcaster/config/generic_profiles.yaml` 识别主 URDF 和映射版，目标名称已同步到 `BN_*_link`。

BVH 离线全身映射按功能组合单轴 joint：每臂为肩带2轴、肩3轴、肘/前臂2轴、腕2轴，
每腿为髋3轴、膝/小腿2轴、踝2轴。
用户确认 `3.6(r)` 肘部弯曲、`3.7(r)` 小臂轴向旋转、`5.6(r)` 踝俯仰、`5.7(r)` 踝横滚；
配置以 `joint_roles.status: user_confirmed` 记录功能，不等同于机械限位或传动已标定。
`3.1(r)` 肩俯仰、`3.2(r)` 肩偏航仍标记 `tentative`，只针对 `Joint_Link2` 局部参考：
零位与局部Y/Z轴线分别偏约 `0.00021° / 2.1113°`。显示ROS前左上系里它们近竖直Z/前向X，不能直接同名。
用户确认 `Joint_Link1` 负责上半身旋转，`Joint_Link3 / Joint_Link2` 合成腰平台2轴，三个joint各自只有1个自由度。
胸部是刚体，没有独立胸部关节；`Joint_Link2` 仅作为刚体上身参考frame。
BVH多段脊柱合成上身整体目标，由 `1 → 3 → 2` 的三个轴共同实现。
骨盆 `Joint_Link4 / Joint_Link5` 按URDF独立路径作候选预览，实物是否与平台耦合尚未确认。
16 个功能组覆盖全部 37 个关节坐标。主模型和映射版都将 CAD 原始导出的 2、3 兄弟支路改为串联。

用户确认的显示方向为绕Z轴旋转 `-90°`：在此约定下，`r` 是右侧，无 `r` 是左侧。
映射使用其逆 `Rz(+90°)` 把源姿态变换到原始URDF坐标，不修改模型参数。
以BVH的 `Spine` 为腰参考，去除整体移动和朝向；首帧参考T-pose联合拟合四肢骨段方向与功能转轴，
双手另以实际手板网格的延伸轴、掌面法向对齐BVH指根几何，初始掌心朝下；小臂轴转与腕两轴共同拟合。
再按派生树的固定RPY、原始局部轴向和顺序合成旋转。踝参考角保持0，两轴组保留三维目标的投影误差。
肘轴取 `Joint_link3.6(r)`，源Z-up参考轴线为 `[0,0,1]`；膝轴取 `Joint_link5.4(r)`，源参考轴线为 `[0,1,0]`。
轴线允许双向对齐，左右保留各自原始轴正负号；这是参考姿态校准，不修改机械参数。

原离线配置 `bvh_broadcaster/config/full_body_robot.yaml` 和映射说明保留在上述归档提交中。
这些映射约定仅用于离线预览，不执行原始全零机械限位，不构成电机映射或闭环模型。

2026-11-03 从归档提交 `1c5b98ef27236129c62020fd337348890e2725c3` 恢复完整模型。
名称、原始参数、38 个原始网格、差动闭环和仿真显示树由 [`test_full_body_robot.py`](../../test/test_full_body_robot.py) 检查。
