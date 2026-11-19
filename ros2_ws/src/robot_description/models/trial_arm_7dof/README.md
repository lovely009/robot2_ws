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

来源 URDF 七个运动关节的 lower、upper、effort、velocity 仍全部为 0。实测机械限位、零位/方向标定及动力学均未验收；来源模型不可用于实机运动或可信重力补偿；原始映射没有端口、总线或控制增益。

`urdf/trial_arm_7dof.standalone.urdf` 与机械主 URDF 同源。仿真模型、几何扫描与 HIL 配置已移除；显示与碰撞网格及离线主机映射继续保留。
