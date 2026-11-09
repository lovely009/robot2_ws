# trial_arm_7dof

工程模型名保持 `trial_arm_7dof`，运行资源由新版 `手臂_mass` 接入，现为 **15 个连杆、7 个转动关节、7 个固定关节**：原14个实体加一个无质量轴框架，总声明质量约 **3.247852541 kg**。7 个运动关节仍为 `Joint_Link1` 至 `Joint_Link7`。

运行 URDF 适配 robot 名称、公共包网格 URI，并通过 `Joint_Link2_axis` 与固定补偿修正关节2轴线；原14个实体参数、网格和零位世界装配保持不变。新版还改变了部分关节坐标系、固定连杆分组及网格，不能把旧版几何装配核对结果视为新版验收。

## 资源与来源

- `urdf/trial_arm_7dof.urdf`、`meshes/`：当前公共运行资源，网格使用 `package://robot_description/models/trial_arm_7dof/...`；Web 模型目录自动发现此模型，不再单独列出中文导出名。
- `cad/mass_export/`：新版来源快照，包括修复后的 URDF/CSV、14 个 STL、原始导出附属文件，以及 `cad/original/` 内未修复的原始 URDF/CSV。其 README 和审查报告记录的是接入前的修复阶段；当前接入状态以本说明为准。
- [近似修复报告](cad/mass_export/cad/inertial_audit/2026-11-05-approximation.md)及[逐项数值和 SHA-256](cad/mass_export/cad/inertial_audit/2026-11-05-approximation.json)：说明三处电机质量属性的估算依据与限制，来源文件未在接入时改写。
- `cad/urdf/试用_手臂.*`、`cad/meshes/`：旧版 CAD 原件；`cad/previous_model/` 保存旧运行 URDF、README 和检查工具。旧版 `cad/inertial_audit/2026-11-05*`（不含文件名中的 `mass`、`integration`）及 `cad/assembly_audit/` 是历史结果，不适用于当前模型。
- `tools/audit_inertia.py`：以新版 CSV 对照当前 14 连杆，重建 `cad/inertial_audit/2026-11-05-mass.json`；需要 NumPy。
- `tools/recheck_inertia.py`：检查关节 3～7 及拆分后的固定电机连杆，重建 `cad/inertial_audit/2026-11-05-mass-recheck.json`；需要 NumPy、SciPy。不再按旧版合并连杆减去整台电机质量，也不覆盖旧报告。

## 使用边界

`Joint_Link3.3`、`Joint_Link4.4`、`Joint_Link5.5` 的质心和惯量采用**均匀密度网格凸包近似**，不是恢复出的真实电机质量分布。两台 R58 保留 0.460 kg；Joint_Link5.5 暂用同型号实例的 0.244 kg，与厂商 0.260 kg 存在待确认差异。其余连杆的来源参数保留，未据此验收整臂动力学。

`config/motor_driver.yaml` 仅提供ID1～7及型号的离线映射；`transmissions.yaml` 将1/2声明为共享空间连杆耦合组，3～7直连。几何以整装STEP为准，57.1745°安装相位包含于球心坐标，不另加命令偏置；数学装配分支不是机械安全限位。纯力矩换算须显式传入输入侧姿态，实时调用须使用经时效/完整性检查的实测姿态。

来源 URDF 七个运动关节的 lower、upper、effort、velocity 仍全部为 0；HIL 使用下节独立仿真 URDF 的几何限位与型号库速度/力矩参数。实测机械限位、零位/方向标定及动力学均未验收；来源模型不可用于实机运动或可信重力补偿；原始映射没有端口、总线或控制增益。

构建并 source 含此模型的 `robot_description` 安装环境后，可离线预览：

```bash
ros2 launch robot_sim display.launch.py model:=trial_arm_7dof
```

不改变默认模型或 `right_arm_4dof`，不增加 Gazebo 配置，不启动硬件。模型来源接入见[接入检查记录](cad/inertial_audit/2026-11-05-integration.md)；新增传动、轴框架、独立安装及审查修复见[离线实施验收](cad/transmission_audit/2026-11-05-implementation.md)。

## MuJoCo 纯软件仿真（2026-11-06）

`mujoco/trial_arm_7dof.xml` 与 `urdf/trial_arm_7dof.standalone.urdf` 是单独的仿真派生件；来源 URDF 的零限位、离线电机映射及传动配置保持不变。离线预览可直接启动 MuJoCo 物理桥，不接 CAN 或控制板；工作台完整仿真现另具独立 HIL 拓扑，见下节。七轴为关节空间力矩执行器，不模拟实际肩部输入电机闭环。

仿真范围由模型网格的近似凸分解接触扫描与肩部球面/电机圆相交可达性得到，记录在 [geometry_limits.json](mujoco/geometry_limits.json)。单轴按 1° 扫描，其他轴固定零位，并向内退让 2°；肩部还复用现有传动反解检查输入半圆与装配分支，再收缩至通过 31×31 网格筛查的矩形。若退让后某方向没有行程，则该侧边界保持零位。扫描止于 ±180°，到达扫描边界不代表存在机械止挡。这些范围不保证任意多轴组合无碰撞，也不等于真实机械行程。

刚性组件、相邻组件的安装接触及零位已有的凸分解重叠按记录排除。原网格中的孔洞经 CoACD 近似保留，碰撞边界仍可能偏差；未建模的连杆实体、电缆和真实止挡不在覆盖范围。动力学沿用已注明的质量属性近似；额外每轴 armature=0.03 kg·m²、damping=0.1 N·m·s/rad 仅用于数值稳定，未作实机辨识。执行器力矩按型号库峰值限幅。

重建前先 source 当前工程的 ROS 安装环境，使 `robot_transmission` 可导入；使用环境中已有的 MuJoCo / NumPy / PyYAML / VTK / trimesh / CoACD：

```bash
~/.ros/ros2_control/.venv/bin/python tools/build_collision_meshes.py
~/.ros/ros2_control/.venv/bin/python tools/build_mujoco_model.py
ros2 launch robot_sim mujoco.launch.py model:=trial_arm_7dof
```

碰撞件的完成标记记录来源、参数与各分件哈希；来源变化、分件损坏或上次中断均会自动重建，不复用半套分件。小于 1e-12 m³ 的退化碰撞碎片不装入 MuJoCo，清单记录在限位报告中。运行验证见 `test/test_trial_mujoco.py`。

## STM32 / can0 HIL（2026-11-06）

工作台选择 MuJoCo、STM32 固件 HIL、SocketCAN 和 can0，在确认总线上没有真实电机后启动仿真，再连接 STM32 USB CDC Agent。`motor_driver.simulation.yaml` 是独立的隔离总线仿真映射，启动器在仿真路径优先读取它；原始 `motor_driver.yaml` 不增加端口、总线或增益。`simulation_topology.yaml` 定义7台假电机，ID/型号和来源一致，肩部继续使用空间连杆传动。

假电机增益 kp=30、kd=1 沿用现有仿真惯例；输入侧行程对肩部31×31输出网格反解取包络，其他轴取仿真URDF范围。包络不是全输入组合可达性保证，实际转换仍执行装配分支及反馈时效检查。非线性力矩换算使用当次新鲜输入侧姿态。所有这些参数仅用于仿真，不作为实机标定或保护参数。

连接与发现验收不使能、不发送运动、不烧录。MuJoCo关节状态为被控对象真值；MCU反馈经传动映射读取，HIL状态发布路径避免争用同一关节状态话题。

### HIL 主机控制限位

仅在固件HIL会话中，存在 `motor_driver.simulation.yaml` 时主机读取该映射及 `trial_arm_7dof.standalone.urdf`，不把来源URDF的零限位带入仿真控制会话。非HIL路径继续读取来源映射和来源URDF。专用映射存在而仿真URDF缺失时明确拒绝，不静默退回。

位置上下限取几何推导记录；速度/力矩逐轴按 `robot_description/config/motor.yaml` 型号库读取：R48（1/2/5/6/7）峰值11 N·m，R58（3/4）峰值45 N·m，两型号额定输出速度均3.1416 rad/s。不把MIT协议量程当机械极限。URDF中的关节侧标量参数不能完整表达1/2轴耦合后的输入约束；电机侧仍经过传动换算和逐电机力矩限幅。MuJoCo桥不额外硬夹真实速度，URDF速度用于主机规划/命令约束，不能把它宣称为物理速度硬止挡。连接或控制会话启动不自动使能，也不自动执行目标。
