# 左手板 STEP 装配变更独立审查

审查时间：2026-11-13（Asia/Shanghai）

## 结论

**真实阻塞项：无。**

本次修改不是只翻转显示网格。零位实际模型中，左 `BN_LeftHand_pitch_link` 的物理 link 世界帧、惯量表达、pitch 关节轴、左腕两条平台的球头站点以及对应法兰/杆件都参与了同一装配修正，杆件闭合成立。

本结论仅覆盖仿真装配正确性。硬件零位、电机方向、真实限位和标定未执行，也不在本任务授权范围内；这些仍是仿真模型声明的未标定项，不是本次装配阻塞项。

## 实际产物与审查基线

- 生成器已在后台结束，日志最后一行：
  `Native full model: 37 outputs, 30 auxiliary hinges, 10 closed rods; kinematic workspaces regenerated`
- 实际模型：
  - `ros2_ws/src/robot_description/models/full_body_robot/mujoco/full_body_robot.xml`
  - `ros2_ws/src/robot_description/models/full_body_robot/mujoco/full_body_robot.physics.xml`
  - `ros2_ws/src/robot_description/models/full_body_robot/urdf/full_body_robot.standalone.urdf`
  - `ros2_ws/src/robot_description/models/full_body_robot/urdf/full_body_robot.180deg.standalone.urdf`
- 审查使用 ROS Humble 环境及
  `/home/ly/.ros/ros2_control/.venv/bin/python`。
- 未修改仓库文件。未重新解析 STEP；复用同一 STEP SHA
  `4741b25580a08be2abb7a49a30b21d0f63f4a6926189587785f13a80ef4c86a4`
  的 analytic feature、kinematic axis、display ownership 和 rod audit。

## 1. 板的零位方向

通过真实 STL 的板端局部顶点、MuJoCo 编译后的世界位姿，以及由
`BN_LeftLowerArm_2_link` 到板铰点的前臂外伸方向计算：

| 侧 | 板端沿前臂外伸方向投影 |
|---|---:|
| 左 | +68.471 mm |
| 右 | +70.145 mm |

左侧从原浏览器复现的约 -68.47 mm 变为 +68.47 mm，符号与原前臂轴端缺陷正好相反，且与右侧向外方向一致。左侧 STL 的零位世界帧与
`left_hand_mount_alignment.json` 的最大元素误差为
`2.75e-14`。

URDF 层级确认为：

`BN_LeftLowerArm_2_link -> BN_LeftHand_roll_link -> BN_LeftHand_pitch_link`

当前修正针对用户指出的末级 pitch 板；roll 轴承座保持原 STEP 姿态。这不是仅换视觉的旁路：pitch 子件的物理 body 帧在 `source_tree()` 中重装，后续惯量和几何均以该物理帧重新表达。

## 2. 轴心与 STEP 根据

使用 `kinematic_axes.json` 的 `BN_LeftHand_pitch`：

- STEP cylinder index：3777
- 左 pitch STEP axis：`[0, 0.9440890204, 0.3296906453]`
- 绕该轴旋转量：`pi`
- 实模关节世界轴与 STEP 轴的点积绝对值：`0.99999999999999`
- 实模关节锚点相对同一条 STEP 轴线的距离：`4.73e-15 m`
- 零位关节角：`0`

轴点沿轴向取线位置的变化不影响刚体旋转，实际锚点与 STEP 轴的无限直线一致。右侧对应 pitch 轴方向同为
`[0, +0.9440890204, +0.3296906453]`，因此这个半转与右侧装配方向相容。

## 3. 惯量

`BN_LeftHand_pitch_link` 仍使用 `Robot_body2` 原始物理链的：

- 质量：`0.0264042403632555 kg`
- 质心：按绕 STEP pitch 轴的同一刚体半转变换后重表达
- 惯量张量：随物理 link 世界姿态进行同一次刚体旋转

定向回归
`test_runtime_inertias_preserve_native_export_world_com_and_tensor`
通过。实际 MuJoCo 物理 body 的质量与变换后质心也一致，未发现退回旧世界姿态或改用辅助杆惯量替换物理 link 惯量的情况。

## 4. 左腕分件归属与平台

`BN_LeftLowerArm_2_link.STL` 是左腕平台、法兰、转子和杆的 source body，其 SHA 与
`Robot_body2` manifest 一致：
`fe2d53249d2ff9a98b50a25022e6aa225b18519696273de51ec3e4dacaec66e3`。

当前 ownership 中：

- proximal rod：region 45
- distal rod：region 62
- proximal platform：regions 42, 0..11
- distal platform：regions 41, 12..23
- proximal flange：regions 44, 63
- distal flange：regions 43, 64
- proximal/distal rotor 区域与固定 rotor cover 分开归属
- fixed、part、rotor 三组区域无重复，且覆盖全部 source regions

两个平台 mesh 的实际世界帧均严格等于各自的
`platform_transform @ BN_LeftLowerArm_2_link 原 STEP 世界帧`，最大元素误差
`3.08e-14`。两个 `platform_transform` 与板的半转相同，因此平台球头确实跟板一起移动，不存在“板转了而输出球头留在前臂”的情况。

## 5. 杆件与杆端

两根左腕杆均保持原始杆长：

- proximal：`0.10150000000000184 m`
- distal：`0.0659999999999984 m`

在零位对真实 rod STL 的轴承中心进行变后检查：

| 杆 | flange 网格端与 site 球心误差 | platform 网格端与 site 球心误差 |
|---|---:|---:|
| proximal | `2.48e-13 mm` | `1.51e-12 mm` |
| distal | `7.36e-13 mm` | `1.69e-12 mm` |

两个新球心距离分别与源杆长的数值差为浮点零量级。零位约束最大残差：
`3.39e-15`。

法兰绕各自解析 motor axis 重新配对，采用最小源角变动分支：

- proximal：`-17.1673769 deg`；候选另一支约 `-179.94 deg`
- distal：`+22.3078611 deg`；候选另一支约 `+174.11 deg`

两个选择均显著更接近源零位，避免无意义的近半转分支。

## 6. 动态闭合与回归

在工作台相关的完整 67 轴模型上执行：

- `test_full_body_native.py`：9 passed
- `test_full_body_robot.py`：20 passed
- 模型规模验证：`nq=67, nv=67, nu=37, neq=10`
- 实测 tester：
  - 板的左右外伸方向
  - 真实移动杆网格端与球心跟随
  - 关节锚点/轴线
  - 惯量世界表达
  - 平台/法兰/杆 body 绑定
  - 六组差动小行程闭环及最终目标收敛
  - 主显示模型与物理模型数组一致性

杆件动态闭环测试中 equality 残差上界低于 `3e-5`，满足现有实模构建阈值；未出现 MuJoCo warning，未发现左手板重装引入新的断杆或分支翻转。

## 范围说明

- 已确认：零位仿真几何、STEP 轴心、惯量、分件与杆端闭合。
- 未确认且按任务不执行：真机零位、实际电机方向、硬件可用行程、碰撞边界和实机标定。
- 这些未确认项均由模型自身明确标为未标定，不构成本次“仿真装配修正”的阻塞项。

