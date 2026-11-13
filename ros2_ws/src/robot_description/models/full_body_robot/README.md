# full_body_robot

2026-11-13 主模型使用新 `../robot_body/urdf/cnm.urdf` 和 `../全身不带头.STEP` 的原生装配坐标，胯部采用已核验的安装孔／座面重建，左手板保留新版导出中已向外伸出的姿态。
当前入口为 [`mujoco/full_body_robot.xml`](mujoco/full_body_robot.xml)，保留 37 个输出关节、30 个辅助关节、10 根闭环连杆和 37 个电机编号。
38 个主体和耦合活动件分别绑定到对应运动 body。源 STL 的连通域重新分类，避免区域编号重排、合并或拆分造成错误归属。
原始 robot_body 和 STEP 保留原样；校准矩阵、源文件摘要及 RPY 修正记录在 `config/step_alignment.json`。
新旧 `Base_link.STL` 逐字节相同。基座上方偏航件局部帧转动 19.25°，网格与轴定义同步重表达，世界轴仍为 −Z；实体世界位置不变。偏航网格连通域由 183 变为 182，合并的旧 0／161 均属固定件，活动件编号已按世界几何重新映射。
修正前的 38 件原生参考装配与 STEP 节点的距离中位数最大 0.218 mm、p95 最大 1.224 mm；这是离散点云比对，不是装配间隙。
新版左手板自由端与真实 STEP 节点的最大差约 0.003 mm。板端独立回归避免仅用密集轴承顶点的整件中位数／p95 掩盖板片反折。
新版双肩连杆球心距为 100.684850 mm，腰杆 42 mm、胯杆 65 mm、腕杆 101.5／66 mm。
胯部按实际安装孔和座面重建；两侧外摆分支的曲柄补偿为 −67.575°／+88.687°，轴承外圈随连杆，内圈留在安装侧。

按用户确认，画面左侧整条手臂在 `BN_LeftUpperArm_3` 长轴处重装 +180°，下游肘、前臂、手板、惯量及腕部闭环同步旋转。安装矩阵记录于 `config/arm_mount_alignment.json`。
新版 robot_body 已修复左手板姿态及左腕坐标表达，手板自身额外重装角为 0；旧版的 180° 翻转及 −17.167°／+22.308° 曲柄补偿不再套用。
两根腕杆和平台球头保留新版零位，杆长仍为 101.5／66 mm。左 pitch 正方向采用新导出定义，六组可达域已重算，旧轨迹需按新版 URDF 重新转换。
安装判断与零位记录见 `config/left_hand_mount_alignment.json`；这不代表真机方向／编码器零位标定。
工作台 URDFLoader 实际网格近景：[左腕](cad/assembly_audit/2026-11-13-web-left-wrist-corrected.png)、[右腕](cad/assembly_audit/2026-11-13-web-right-wrist-corrected.png)。

关节按 `robot_body/meshes` 文件名去掉 `.STL`、末尾 `_link` 与首尾空格命名；连杆保留 `_link`，根连杆为 `Base_link`。
例如 `BN_Spine_pitch`、`BN_hips_roll`、`BN_LeftUpperArm_1`、`BN_LeftHand_roll`／`BN_LeftHand_pitch`。
完整旧名／新名对应见 [`config/joint_names.json`](config/joint_names.json)；电机编号、差动组和关节对应见 [`config/motor_topology.yaml`](config/motor_topology.yaml)。
仿真位置接口可解析旧名称别名，但旧动作的坐标定义已经变化，应按新版 URDF 重新转换；别名兼容不代表旧轨迹姿态等价。

左大腿第 2 关节的源 RPY 符号错误已修正为 `-0.0740617322384423 1.5707963267949 0`；仅修改该处便使后续六件同时对齐 STEP。
四处源关节轴心已依据 STEP 和父子件圆柱支持修正：左右肩第 1 轴、左右腕 pitch。
通过重表达 visual／collision／inertial 与子关节坐标，保持零位网格和质心的世界位姿。
轴线记录见 [`config/kinematic_axes.json`](config/kinematic_axes.json)、`cad/assembly_audit/2026-11-13-joint-axes.json`。
其余 31 根轴有父子双侧圆柱支持，两根脚部 roll 轴目前仅有单侧支持，保留源定义并记录待机械核实。

仿真限位按当前整臂朝向重新进行 37 轴扫描与六组二维耦合检查，结果见 `config/simulation_limits.yaml` 和 `mujoco/geometry_limits.json`。使用未降面的 CAD 三角网格，步长 1°、内收余量 2°；其他组保持零位。
按用户授权，只有轴线与实际几何支持的有限电机内部区域允许重叠。有限配合面、球头和轴承区域在每个姿态检查配准；零位接触本身不授予豁免，不再排除整对相邻装配体。平面接触必须同时落在双方当前真实三角面片内，断开的轴向材料段分别处理。
JSON/YAML 以扫描报告 SHA256、逐轴范围、六组掩码和计算规则／CAD 证据摘要绑定，重建时拒绝混用旧结果。主体惯量保留新版 robot_body 值并随装配变换；辅助件惯量仅用于数值预览。这些是仿真软件范围，硬件行程、线缆约束、实体包容及跨组组合姿态尚未标定。

```bash
source ros2_ws/install/setup.bash
~/.ros/ros2_control/.venv/bin/python ros2_ws/src/robot_description/models/full_body_robot/tools/build_mujoco_model.py
tools/mujoco_zh/run.sh --mjcf /home/ly/robot2_ws/ros2_ws/src/robot_description/models/full_body_robot/mujoco/full_body_robot.xml
```

`full_body_robot.urdf` 和 `.mapping.urdf` 为 37 轴骨架；`.standalone.urdf` 额外包含活动法兰、连杆和辅助关节，用于完整 CAD 运动显示。
新版分件证据见 `cad/assembly_audit/2026-11-13-components.json`，杆端与安装检查见 `2026-11-13-step-linkages.json` 和 `2026-11-13-hips-mounting.json`。
显示网格在 `mujoco/visual/native/`，分件归属与零位变换记录在 [`config/native_assembly.json`](config/native_assembly.json)。
肩臂、双腕及胯部细节见 [新版主模型实体渲染](cad/assembly_audit/2026-11-13-native-main-preview.png)。
主模型与 standalone 的新版摘要已注册到 BVH 通用映射配置。
本轮 75 项针对性测试通过，验证分组、源摘要和六组可达采样数见 `cad/assembly_audit/2026-11-13-validation.json`。
新版输入、base／左腕差异和独立审查见 `cad/assembly_audit/2026-11-13-new-body-check.md`；此前 Robot_body2 阶段报告仅保留为历史证据。

另提供 ±180° 仿真副本：[`mujoco/full_body_robot.180deg.xml`](mujoco/full_body_robot.180deg.xml)，以及 `urdf/full_body_robot.180deg.urdf`、`urdf/full_body_robot.180deg.standalone.urdf`。
该副本的 67 个转动关节和 37 个驱动范围均为 ±π，位置接口的 37 个输出范围同步为 ±π，移除旧采样域限制；10 处实际闭环与几何逆解保留，因此部分角度组合仍不可达。当前主模型与其配置保持原范围。
主模型重建后，用 `python3 tools/build_180deg_preview.py` 在本模型目录重新生成副本；用中文 MuJoCo 启动器的 `--mjcf` 参数打开上述 XML。

以下为迁移前的静态装配核对记录；其中“旧运动模型未更新”等状态描述属于当时记录，当前主模型以上方说明为准。

2026-11-12 装配复查更正：旧 `full_body_robot.xml` 的显示配准未通过用户截图复核，肩臂/隔板及两侧手腕存在错位。
之前的单连杆表面与约束测试不能替代整机装配验收。

胯部安装复查进一步更正：原 STEP／`Robot_body1` 中两侧胯部球铰组件本身没有装入平台安装孔。
“20 个杆端球心同心”只证明杆与球铰匹配，不能证明球铰已安装到接收件；之前的通过结论没有覆盖这一段连接。

当前查看胯部安装修正，请打开 [`mujoco/full_body_robot.hips-mounted.xml`](mujoco/full_body_robot.hips-mounted.xml)。
它保留整机 STEP 配准，并按实际孔口和座面重装胯部两根连杆：

- 两侧平台孔轴线位于 X = ±71.52 mm；测量孔口外侧最高完整圆周及有限凸台，不采用下方约 2 mm 的肩面。
- GE8 内圈轴孔为 Ø8 mm，轴向宽度约 8 mm，球心从实际座面沿孔轴外移 4 mm；平台端轴孔同轴、内圈底面贴合座面。
- 保留两根杆的 65 mm 球心距，法兰绕实际电机轴分别转动约 −53.77°／+74.88°，选择外摆装配分支。
- Ø16 mm、宽 5 mm 的轴承外圈归连杆；内圈与紧固组件留在平台或法兰。杆的滚转方向按实测杆孔法向重建。
- 法兰平板侧另测实际 Ø5 mm 孔的最高完整孔口圆周；验收同时检查该侧座面和法兰／固定电机轴同轴。

这是**静态安装重建**；原 STEP、原始导出网格保持原样，运动模型的关节、闭环及碰撞限位尚未完成这一轮装配迁移。
参数和独立验收结果分别见 `config/hips_mount_alignment.json`、`cad/assembly_audit/2026-11-12-hips-mounting.json`。
MuJoCo 实体渲染对比见 [胯部安装前后](cad/assembly_audit/2026-11-12-hips-before-after.png)（左为原 STEP，右为修正后）。
验收从源几何重新测孔口、座高和球心，不以求解参数作为通过依据；还核实实际 MJCF 的网格绑定与源连通域内容。
固定底座也按剔除活动件后的源区域核验，防止原始连杆或紧固组件残留造成重影。
回归覆盖整体移位、求解参数与组件一起装到错误肩面、连杆／平台网格互换、底座重复残留活动件，以及球心仍闭合但连杆绕长轴扭转 20°／180°。

```bash
source ros2_ws/install/setup.bash
~/.ros/ros2_control/.venv/bin/python ros2_ws/src/robot_description/models/full_body_robot/tools/hips_mounting.py
~/.ros/ros2_control/.venv/bin/python ros2_ws/src/robot_description/models/full_body_robot/tools/audit_step_linkages.py
tools/mujoco_zh/run.sh --mjcf /home/ly/robot2_ws/ros2_ws/src/robot_description/models/full_body_robot/mujoco/full_body_robot.linkage-audit.xml
```

当前通过 STEP 零位比对的模型为 [`mujoco/full_body_robot.step-aligned.xml`](mujoco/full_body_robot.step-aligned.xml)。
它按照 `Robot_body1` 的整机坐标组装全部 38 份网格，修正已核实的腰部 pitch 导出姿态，不使用旧的逐连杆 ICP 显示矩阵。
这是**静态装配校验模型**，无关节、驱动器与运动闭环。旧运动模型保留，其关节安装轴、活动件归属和限位尚未按新版装配重新标定。

重建入口为 `tools/build_step_alignment.py`。打开方式：

```bash
tools/mujoco_zh/run.sh --mjcf /home/ly/robot2_ws/ros2_ws/src/robot_description/models/full_body_robot/mujoco/full_body_robot.step-aligned.xml
```

`config/step_alignment.json` 记录整机坐标，`cad/assembly_audit/2026-11-12-step-realignment.json` 记录与实际 STEP 节点的逐连杆距离。
逐连杆距离中位数最大 0.264 mm，95% 分位最大 1.458 mm；这是离散节点比对，不是接触间隙。
实际 STEP 地标点用于肩臂、隔板和腕部的自动回归检查。

STEP 耦合连杆复核：六组、10 根杆、20 个端点都找到实际 STEP 球面与杆端圆柱的同心对应。
检查同时要求杆端圆周覆盖和轴向截面中心一致，避免“无限圆柱轴碰巧经过球心”误判。
几何中心检查容差 0.025 mm，最大轴向截面中心偏差约 0.013 mm；这是几何匹配检查，不是间隙或加工精度验收。
10 个法兰取得大圆柱几何轴候选；轴线正负号没有编码器方向含义，静态几何也不证明转子/壳体的动态归属。

| 连接几何（mm） | STEP | 旧运动模型 |
|---|---:|---:|
| 腰部左右球心跨度 | 102 | 100 |
| 胯部平台左右球心跨度（锚点定义不同） | 178.767043 | 143.040001 |
| 单根肩部连杆球心距（左右相同） | 106.047644 | 106.905873 |
| 胯部单根连杆球心距 | 65 | 65 |
| 手腕长／短连杆球心距 | 101.5／66 | 101.5／66.005954 |

旧模型的腰部球心被向内移 1 mm/侧，双肩用了不同的杆长；不能直接用于这份 STEP 装配。
胯部原 STEP 的球铰并未装入孔位，两列跨度差异不能直接判定旧锚点错误。上方安装重建保留实际杆长，并按真实安装孔和座面确定平台位置及曲柄分支。
原运动模型的锚点、杆长和活动件归属仍需重新标定，本轮没有改动运动控制、限位或保护参数。

带标记的静态校验模型为 `mujoco/full_body_robot.linkage-audit.xml`：透明外形、橙色球心连线、绿色球心标记、蓝色法兰几何轴；胯部采用修正后的安装模型与标记位置。
胯部之外的组目前仍仅通过杆端几何检查，未以此声称安装座面或运动验收通过。
完整坐标、源分件、圆柱轴与检查结果见 `cad/assembly_audit/2026-11-12-step-linkages.json`。
复核命令（STEP 特征缓存已提供；重提取需安装 `cadquery-ocp`）：

```bash
~/.ros/ros2_control/.venv/bin/python ros2_ws/src/robot_description/models/full_body_robot/tools/audit_step_linkages.py
```

以下为前次更新记录，其中“测试通过”仅指当时的单连杆/物理参数检查。

2026-11-12：standalone 显示模型与整机 MJCF 外观已更新为 `../Robot_body1` 的 38 份 STL。
与上一版 `Robot_body` 相比，14 份双臂网格变化，其余 24 份一致。
配准、源文件哈希和活动件的连通域归属记录在 `config/display_meshes.json`；显示网格位于 `mujoco/visual/Robot_body1/`。

本次重点修正了耦合件的显示连接：六组差动共 10 根连杆，30 份法兰/连杆/平台网格分别绑定到原有活动 body，
不再把整件 STL 中的活动件固定在主连杆上。腕部转子与端盖继续分开，防止静止端盖随电机转动。
同时修正旧显示配准中左前臂沿局部 Z 首尾翻转的问题，重新配准两侧前臂。
连通域归属已固化；重建前核对源/参考哈希、区域数量与重复归属，全部拆分成功后才替换显示网格。

全身 STEP 已通过 OpenCascade 解析，包含 528 个实体实例。新版原始 URDF 存在腰部 pitch 导出姿态差异，
导致上身与 STEP 的点云距离达到数十毫米；仅恢复上一版该关节 RPY 后，上身各连杆的距离中位数降至约 0.004～0.13 mm。
这些数值是离散点云比对，不代表装配间隙。新版还有 5 处右臂/右手网格 URI 的文件名前导空格错误。
原导出包保持原样，本次未将其运动学、惯量或零限位直接覆盖到 full。
来源、逐连杆比对和分件证据见 [`cad/assembly_audit/2026-11-12.json`](cad/assembly_audit/2026-11-12.json)。

`full_body_robot.urdf`、映射版、质量/惯量、传动、闭环与限位沿用现有模型。
显示更新不会重新定义碰撞范围；`mujoco/full_body_robot.physics.xml` 继续供原几何扫描使用。
`tools/build_mujoco_model.py` 会重建两份 MJCF 与显示 URDF，生成的 68 个可见网格包含 38 个主体和 30 个耦合活动件。

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
`Base_link` 是 RViz 的固定参考帧。下文中的 CAD 编号用于追溯原始装配关系。
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
用户已确认机械电机编号 1～37 即实际 CAN 节点 ID 1～37，已写入拓扑。
工作台可读取 25 个一对一组和 6 个双电机差动组；方向、零位、差动参数及驱动配置尚未标定，
该拓扑仅描述编号对应关系，不作为可执行传动配置，真机控制保持禁用。

| 电机编号 | 传动分组 | URDF 关节 |
|---|---|---|
| 1 | 一对一 | `BN_Spine_yaw` |
| 2、3 | 腰部差动 | `BN_Spine_pitch`、`BN_Spine_roll` |
| 4、5 | 胯部差动 | `BN_hips_roll`、`BN_hips_pitch` |
| 6、7 | 左肩差动 | `BN_LeftShoulder_1`、`BN_LeftShoulder_2` |
| 8 | 一对一 | `BN_LeftUpperArm_1` |
| 9 | 一对一 | `BN_LeftUpperArm_2` |
| 10 | 一对一 | `BN_LeftUpperArm_3` |
| 11 | 一对一 | `BN_LeftLowerArm_1` |
| 12 | 一对一 | `BN_LeftLowerArm_2` |
| 13、14 | 左腕差动 | `BN_LeftHand_roll`、`BN_LeftHand_pitch` |
| 15、16 | 右肩差动 | `BN_RightShoulder_1`、`BN_RightShoulder_2` |
| 17 | 一对一 | `BN_RightUpperArm_1` |
| 18 | 一对一 | `BN_RightUpperArm_2` |
| 19 | 一对一 | `BN_RightUpperArm_3` |
| 20 | 一对一 | `BN_RightLowerArm_1` |
| 21 | 一对一 | `BN_RightLowerArm_2` |
| 22、23 | 右腕差动 | `BN_RightHand_roll`、`BN_RightHand_pitch` |
| 24 | 一对一 | `BN_LeftUpperLeg_1` |
| 25 | 一对一 | `BN_LeftUpperLeg_2` |
| 26 | 一对一 | `BN_LeftUpperLeg_3` |
| 27 | 一对一 | `BN_LeftLowerLeg_1` |
| 28 | 一对一 | `BN_LeftLowerLeg_2` |
| 29 | 一对一 | `BN_LeftFoot_pitch` |
| 30 | 一对一 | `BN_LeftFoot_roll` |
| 31 | 一对一 | `BN_RightUpperLeg_1` |
| 32 | 一对一 | `BN_RightUpperLeg_2` |
| 33 | 一对一 | `BN_RightUpperLeg_3` |
| 34 | 一对一 | `BN_RightLowerLeg_1` |
| 35 | 一对一 | `BN_RightLowerLeg_2` |
| 36 | 一对一 | `BN_RightFoot_pitch` |
| 37 | 一对一 | `BN_RightFoot_roll` |

差动组的传动矩阵（同向与反向分别对应的关节、符号与比例）、零位和机械限位待确认。当前表格用于记录编号和分组。

## 预览和离线映射

### 整机 MuJoCo

[`mujoco/full_body_robot.xml`](mujoco/full_body_robot.xml) 包含主 URDF 的 38 个连杆、
25 路一对一输入及六组差动的 12 路输入。整机保留了 10 根定长连杆的闭环，可在同一窗口调试全身。

在 **仿真工作台**选择 **MuJoCo → 外部控制器 → vcan（纯软件）→ full_body_robot** 后启动。
切到 **调试 → 关节**，等到面板显示 **MuJoCo 位置控制** 后即可使用关节滑块。
直驱关节发送位置目标，差动关节按球头与定长连杆几何求解电机角，并保留同组另一轴的目标。
37 轴各自使用 [`config/simulation_limits.yaml`](config/simulation_limits.yaml) 的几何扫描范围。
零位时 37 个滑块均可用。六组差动的范围随同组另一轴的目标角度更新，界面显示当前姿态下连续可用的区间。
仿真发布 `/joint_states`、`/tf` 和 `/clock`。工作台的整机视图和仿真 TF 都使用从 MJCF 生成的
`full_body_robot.standalone.urdf`。仿真发布 67 个关节状态，其中 37 个对应工作台滑块，30 个用于显示差动法兰和连杆。

```bash
~/.ros/ros2_control/.venv/bin/python -m mujoco.viewer \
  --mjcf /home/ly/robot2_ws/ros2_ws/src/robot_description/models/full_body_robot/mujoco/full_body_robot.xml
```

在 **Control** 面板拖动滑块，输入单位 rad，每个输入使用 YAML 中各自的 `actuator_limits`。`motor_01` 等沿用一对一电机编号；
差动滑块以 `waist_`、`hips_`、`left_shoulder_`、`right_shoulder_`、`left_wrist_`、`right_wrist_` 分组。
在 **Simulation** 面板可加载零位及六组同时同向或反向的关键帧。

模型底座固定，重力和接触关闭。腰、胯、左右肩和左右腕的法兰、连杆和平台端球头从原始 CAD 网格拆出。
腰部拆件沿 X 各向内平移 1 mm，以配合名义 100 mm 球头间距。腕部的输出盘与外端盖在 STL 中连成一片，
显示模型按电机轴上半径 8.55 mm 的圆柱面分开两者，内盘和球头随电机转动。
需要拆件或降面的外观网格生成到 `mujoco/visual/`，原始 CAD 网格保留。
MJCF、显示副本和仿真 URDF 可用 MuJoCo 环境执行 `tools/build_mujoco_model.py` 重建，需 VTK、NumPy、SciPy 和 PyYAML。
原始 STL 及六份单组 MJCF 是生成器输入，`mujoco/visual/`、整机 MJCF 和 standalone URDF 一并纳入版本管理。
工作台按上次提交的整组关节目标保持位置。在同一仿真会话中交替操作 Viewer 和工作台时，工作台会恢复其保存的其他关节目标。

### CAD 几何仿真限位

原始 CAD URDF 的关节上下限全部为零。扫描器使用装配后的三角网格和定长连杆逆解，
电机输出轴按用户确认的 ±180° 搜索，逐轴以 1° 步长搜索到首次干涉或连杆切向奇异边界，并向内收 2°。
六组差动另外在单轴扫描范围内检查两轴的 1° 网格，在边界内收两格，保留零位连通的二维区域。
差动目标的四个相邻网格点均需有效，工作台根据同组另一轴目标角度更新滑块范围。
其他输出关节保持零位。
完整边界原因、零位配合面排除清单和几何指纹记录在 [`mujoco/geometry_limits.json`](mujoco/geometry_limits.json)。

网格按 CAD 连通零件分开检查，保留全部非退化原始三角面；`mujoco/collision/native/` 为未降面的扫描输入。扫描报告列出每一有限豁免的 STEP 证据、参与组件摘要、材料区间或真实平面片索引；`adjacent_assembly_exclusions` 必须为空。
完整 37 轴范围、各侧首次阻挡原因和六组二维掩码见上述 JSON；便于查看的角度表另见 `cad/assembly_audit/2026-11-13-outward-arm-limits.md`。±178° 表示到达 ±180° 搜索上限后内收余量，不能解释为实测硬限位。二维包络内仍需通过掩码判断组合目标。

在工程根目录重算，使用系统 Python 的 Coal/NumPy 环境和已有 MuJoCo 环境。

```bash
source ros2_ws/install/setup.bash
/usr/bin/python3 ros2_ws/src/robot_description/models/full_body_robot/tools/derive_simulation_limits.py
~/.ros/ros2_control/.venv/bin/python ros2_ws/src/robot_description/models/full_body_robot/tools/build_mujoco_model.py
```

仅重算两侧大腿和手腕时，为扫描命令添加 `--refine-thigh-wrist`，再运行整机生成器。
只重算六组差动时，使用 `--refine-coupled`，再运行整机生成器。
扫描报告中各组的 `envelope_deg` 为整个二维可动区域的投影范围，工作台显示由当前另一轴目标决定的范围。

整机生成器将扫描限位写入 MJCF 和 standalone URDF。下列单组模型保留各自的 ±10° 法兰探索输入。

动作工作台从模型库选择本模型时，使用同一份仿真位置范围和六组耦合区域转换 37 轴动作。
软件预览速度上限为 π rad/s，报告以 `simulation_limits_enforced` 标记。此速度上限用于离线预览。
预览中的 30 个辅助轴由 MuJoCo 几何逆解重建，显示完整 CAD 法兰与连杆。
导出的关节动作保留 37 轴，并提供对应的预览 URDF。

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
`Base_link` 的 +Z 右手方向计角，滑块名中的 +X、−X 表示安装侧。两根 65 mm 连杆前端接入
前横梁的安装孔，球心沿孔轴高于安装面 4 mm。34 mm 摆臂在仿真零位选取向外的装配分支，
相对 CAD 导出姿态，+X 侧转约 −53.77°，−X 侧转约 +74.88°。等量同向主要驱动 `BN_hips_roll`，
等量反向主要驱动 `BN_hips_pitch`。4、5 号对应安装侧及编码器正向仍待确认。

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

主 URDF 和映射版中 37 个关节的 `lower/upper/effort/velocity` 均为 0，按限位工作的原始预览器不能拖动关节。
工作台 MuJoCo 入口使用上述 standalone URDF 的仿真范围。
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
