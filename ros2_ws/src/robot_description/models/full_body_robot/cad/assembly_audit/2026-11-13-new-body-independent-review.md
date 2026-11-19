# 新 robot_body / cnm 独立只读审查

时间：2026-11-13 16:05 CST 左右快照；最后一次生成产物读取约 16:06 CST。  
范围：同一 `全身不带头.STEP` 下的 Base/yaw、上体姿态、左腕 roll/pitch 装配、生成链、当前 MuJoCo/URDF 及测试证据。未重做全量 ICP，未修改仓库。

## 结论

在当前“运动学与装配预览”目标下，**未发现阻塞问题**。Base/yaw 的几何和惯量变化是坐标重表达；新左腕源已向外，新一轮生成没有第二次半转，raw platform fittings 与两 rod 零位保持成立。

有三个必须按作用域保留的未完成项：

1. `BN_LeftFoot_roll`、`BN_RightFoot_roll` 仍无子侧双侧几何支持；现模型沿用 raw URDF 轴。若“37 轴均须双侧 STEP 独立闭合”是验收条件，这两轴仍是阻塞项。
2. `native_assembly.json` 明确写 `collision_validation: pending`，当前接触几何关闭。若目标包含碰撞安全或限位，不能据此验收。
3. 四个手工轴选择仍应保留：左右肩 `BN_*Shoulder_1` 和左右手 pitch。它们与所选 STEP 圆柱线一致，且保存 raw source 正向；不应被“最近圆柱”自动规则覆盖。

## 快照与来源绑定

关键源码/配置哈希：

- `cnm.urdf`: `d127a6994613c80193d93ac194c4fa8d5590139c9972b2ac668b002045651be7`
- `build_native_model.py`: `27659301a9b6df98fc664a74d474f9037871041b8baff38ba80b7758f24fdb6a`
- `build_step_alignment.py`: `ddc69af11694ccb89da9f5bf944aee6dcac6d534d103a96d70473abd79cc8033`
- `left_hand_mounting.py`: `2b3b38e7d3afd28493365291b5bccecd9cef8db86a6cccd5581eebf572eeaf55`
- `step_alignment.json`: `c58bfbd1f340b9feb0ba688e8b9f04b91d1c1e8f19985931b353b990cd12d139`
- `display_meshes.json`: `7e0c223dbb9964144bd556bbf1b4c745898887f6a35d3f41c73992da58c2bf7d`
- `full_body_robot.xml` 与 `full_body_robot.physics.xml`: 均为 `b98a9e6dc3766c02564fd529b95dc88925ade655a369a1f568684d280a3fd46b`

`step_alignment.json` 的 `source_urdf` 当前为 `robot_body/urdf/cnm.urdf`，其 `urdf_sha256` 与实际文件一致；STEP hash 为 `4741b25580a08be2abb7a49a30b21d0f63f4a6926189587785f13a80ef4c86a4`。`display_meshes.json` 的 `source` 为 `robot_body`。

生成链已去掉旧的 `Robot_body1/2` 运行源：

- `build_native_model.py:68-111` 从 `alignment['source_urdf']` 加载，并校验 URDF、每份 mesh hash 与 `alignment.source`。
- `build_step_alignment.py:18-26` 从 alignment 的动态 `source_urdf` 加载；`source_directory = urdf.parents[1]` 对当前 `robot_body/urdf/cnm.urdf` 正确。
- 全仓 production tools/config 中只剩 `display_meshes.json` 的 `previous_source: Robot_body2`、`legacy_registration_source: Robot_body1` 两条 provenance 文本，不是运行依赖。

## Base 与 BN_Spine_yaw

### Base_link

- 新、旧 `Base_link.STL` 均为 `ee2df3d7e307c2d40718b57746c14e593cbadd9db75e30ef53a3acc87c77f318`，**byte identical**。
- `Base_link` 是 URDF 根；MuJoCo 中为 `worldbody` 下固定的 `Base_link`，无 root free joint。其世界位姿仍严格为 identity。
- 新 URDF 的 Base 质量/COM/惯量只是补足小数位：相对旧 Robot_body2 的舍入值，质量差 `1.521e-6 kg`、COM 差 `2.97e-7 m`、惯量最大元素差 `4.53e-9 kg m²`，均落在旧导出分辨率内。
- Base 本身没有物理转动轴；其上方的 yaw 轴是独立关节。

### BN_Spine_yaw

新的 link 帧是 `Trans(0,0,0.011) * Rx(0.335975881008902)`。新 STL 相对 Robot_body2 STL 的刚体坐标变换被拟合为 `Rx(-0.335975881008902)`、平移约 `1e-11 m`；对应顶点残差 RMS `2.614e-9 m`、最大 `8.563e-9 m`。因此：

```text
T_new_link * T_old->new_mesh = T_old_link
```

世界几何没有重定位或重朝向。旧/新 STL hash 分别是 `90f863...e6b5` / `330ddb...a880`，不能继续用旧 hash 做缓存键。

物理不变量也一致：

- yaw 世界轴：`[0,-5.16e-15,-1.0]`，仍为 world `-Z`，误差量级 `1e-15`；轴点 `[0,0,0.011]`。
- 相对 Robot_body2 的 yaw 世界 COM 差 `3.48e-7 m`；世界惯量张量最大元素差 `3.93e-9 kg m²`。这可用旧值舍入和新增精度解释。
- 不能把局部 COM 或局部惯量直接比较来判“换了惯量”；它们已随新 link frame 重表达。

区域变化也已独立核对：183 old regions 映射到 182 new regions，唯一合并是把 **old region 0 与 old 161** 映到 **new region 0**，均为 fixed。当前 metadata 声明 182 regions，yaw 24 moving / 158 fixed，new region 0 在 fixed，无 moving ownership 重叠。`2026-11-13-new-body-components.json` 记录的世界顶点最大差即上述 `8.5567e-9 m`。

上体 `BN_Spine_pitch`、`BN_Spine_roll` 到左 `BN_LeftLowerArm_2` 相对旧 Robot_body2 的零位差仅因 full precision 提升：

| link | world COM 差 | world inertia 最大元素差 | 旋转差 |
|---|---:|---:|---:|
| Spine pitch | 4.18e-7 m | 7.66e-11 kg m² | 0.00118 deg |
| Spine roll | 1.15e-6 m | 7.25e-9 kg m² | 0.000964 deg |
| Left lower arm 2 | — | — | 0.000465 deg |

最新 `step_alignment` 与新 URDF 的这些 link frame 相符；视觉 mesh 的世界姿态在编译模型中以 `1e-6 m` 容差通过。

## 左腕 roll、pitch 与 rod 装配

### roll 帧

新 roll link 的局部表示由倾斜轴改成 `-Z`，但这是一次 frame re-expression：

- old->new roll STL 刚体变换拟合：约 `90.43512 deg`，ERROR RMS `2.71e-10 m`、最大 `1.32e-9 m`。
- new link frame 与新的 STL 局部坐标互相抵消，raw STEP 世界几何保持。
- old roll 世界轴约为 `[0.9848008,0.0572769,-0.1639717]`；new 为 `[0.9848071,0.0572577,-0.1639406]`，共同对应 raw source/STEP motor hinge，方向没有被重新定义。
- roll 世界 COM 差约 `6.83e-6 m`，世界惯量最大差 `1.53e-12 kg m²`，属于旧值舍入范围。

### pitch 手板与向外意图

新 `BN_LeftHand_pitch_link.STL` 本体未换，新 link 姿态相对旧 Robot_body2 的世界变化是约 `180 deg` 绕全局 X 的半转：Y/Z 列反号。用户给出的实际 STEP tip 结果为 max `2.53 um`；旧 Body2 max `10.6 mm`，说明新 source 已经完成手板方向修复。

独立定向几何检查：

| 姿态 | 沿前臂外向投影 min / max |
|---|---:|
| 新 left pitch | `-6.954 / +70.136 mm` |
| 新 right pitch | `-6.954 / +70.136 mm` |
| 对新 left 再套旧 hinge-axis pi | `-70.136 / +6.954 mm` |

因此旧 `left_hand_mounting` 的 hinge 半转不能再应用到新 source；否则会把已向外的左板翻回内折。已确认的浏览器 Loader 结果为左/右板端外伸 `69.826 / 69.828 mm`，与上述方向结论一致。

### 新 source 的 platform fittings 与两 rod 零位

`left_hand_mounting.solve()` 在当前新 alignment 上独立执行结果：

- source plate 的 `-Y` 投影为约 `0.983`，因此 `rotation_rad = 0.0`；
- `hand_transform` 精确为 identity；`native_assembly.remounted_source_link_world_transforms = {}`；
- proximal zero shift `8.88e-16 rad`，最近候选另一支约 `-3.05186 rad`；
- distal zero shift `0 rad`，另一支约 `+2.83781 rad`；
- 两 rod 的 platform transform 精确 identity，flange/rod transform 偏差小于 `1e-15`；
- 当前新 source report 中两个 platform ball 都有双侧支撑，零位闭环残差为机器精度；rod length 为 `0.10150000000000184 m` 和 `0.0659999999999984 m`。

因此，保留 raw platform fittings 与两 rod 原始零位是正确的。这里的依据是当前新 source 的 endpoint audit，不是把旧 source 的 endpoint 直接覆盖过来。

## 生成模型与约束

当前快照的生成结果：

- URDF：38 links / 37 joints；
- MuJoCo：`nq=nv=67`、`nu=37`、`neq=10`、67 joints、59 bodies、68 visual geoms；
- Base 固定在世界系，identity；yaw anchor `[0,0,0.011]`，轴 world `-Z`；
- 37 个 actuator 分别唯一绑定，没有重复；
- 10 个闭环均为 `connect` equality；中性零位 equality residual max `3.91e-15 m`；
- display 与 physics XML 当前 byte identical，相关模型测试也逐字段验证了质量、惯量、关节、actuator 和 equality 数据。

Axis audit：37 joints 中 **35** 具有 parent+child 双侧圆柱支持；仅左右 `Foot_roll` 无 child-side support。两 Foot_roll raw 轴与其 parent candidate 线差分别为约 `7.36e-8 m`、`9.15e-8 m`，但缺少 child-side 支持，不能把该 parent 候选宣称成双侧完成。

四个手工 override（左右 Shoulder_1、左右 Hand_pitch）被保留，并被写入生成 model。对所选 STEP feature cylinder 的当前模型 anchor line error 在 `~1e-15 m` 量级，轴向差低于 `0.000006 deg`；部分显示轴符号与 cylinder feature 相反，是代码有意保留 raw source 正向，不能当作轴错误。自动最近 cylinder 未覆盖这些人工选择。

唯一额外的人工 20.35 mm 肩轴选择也仍显式记录在 `kinematic_axes.json` evidence 中。报告没有把它替换成最近自动候选。

## 旧补偿处置

应迁移：

- `source_urdf = robot_body/urdf/cnm.urdf`、step/source/mesh hashes 与新 182-region metadata；
- Base 全精度惯量；yaw link `Rx(+theta)` 与新 STL `Rx(-theta)` 的成对表达；
- 新左 roll 的 `-Z` 局部轴及其 re-expressed STL；
- 新左 pitch 已向外的 raw 姿态、raw platform fittings 和两 rod 零位。

应保留：

- `BN_LeftUpperLeg_2` 的 `rpy[0] = -0.0740617322384423` 校正。新 raw export 仍是 `+0.0740617322384423`；保留新 hashes 的 `step_alignment` 已明确记录 exported 与 corrected 两个值。
- 左右 Shoulder_1、左右 Hand_pitch 的人工 STEP 轴选择，以及 hips fixture。
- `collision_validation: pending` 与“仿真预览、非硬件零位/限位”的范围声明。

应撤掉或加防误用：

- 新 source 上再次执行旧 `left_hand_mounting.py` 的 pi 重装路径；当前 `solve()` 已按 plate 外向自动变为 identity，测试也断言 `rotation_rad == 0` 与两 rod zero shift `<1e-10`。
- Robot_body1/2 的旧运行源、旧 step mesh hashes，以及旧 yaw mesh 的 identity/local 直配思路。旧 yaw world identity 不能再配给新局部 STL；应当要么完整使用新 link frame + 新 STL，要么重新按同一坐标系重表达。

## 硬编码与遗留

没有在生产生成链中发现阻塞性的旧 source 硬编码：

- `build_native_model.py` 和 `build_step_alignment.py` 均通过 alignment 的 `source_urdf/source` 工作；
- `display_meshes.json` 用 `source: robot_body`；
- 只有 provenance 字段还提到 Robot_body1/2；
- `test_full_body_native.py` 把 `robot_body/urdf/cnm.urdf` 和带日期的 audit JSON 当验收 fixture，这是有意的测试绑定；它们有 hash/STEP 身份守卫，不是 runtime fallback。若以后换日期文件，测试路径需要同步更新，但不是当前模型错误。

## 验证证据

本机模型相关测试日志：`/tmp/robot-new-body/tests-model.log`，当前快照为 **45 passed in 68.07 s**。主任务另行报告 20 项 runtime 与 7 项 geometry/web 相关测试通过，以及真实 served browser 的 MuJoCo URDF Loader 成功加载 67 总铰 / 37 UI 关节并显示左右外伸板端；本轮不再重跑这些范围。

未独立完成项：

- 未重做全量 STEP->mesh ICP；
- 未重新解析 STEP；
- 左右 Foot_roll child-side 证据仍未闭环；
- collision validation 仍为 pending。
