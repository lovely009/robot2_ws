# full_body_robot

本目录保存整机机械 CAD、37 轴 URDF、完整活动件显示骨架、站姿 URDF／初始角度，以及显示和碰撞网格。关节命名遵循 `config/joint_convention.yaml`，基座 +X 向前、+Y 向左、+Z 向上；映射见 `config/joint_names.json` 与 `model_motor_topology.yaml`。

机械装配源为 `../robot_body/urdf/cnm.urdf` 和 `../全身不带头.STEP`。双臂 18 个连杆惯性取自 `../全身实验版_仅双臂质量/`；来源摘要、分件归属、轴心校准和安装矩阵保存在 `config/`，历史机械核验记录保存在 `cad/assembly_audit/`。

`tools/build_native_model.py` 从这些记录重建主 URDF、mapping 和显示／碰撞网格，不加载仿真器。它校验来源摘要，保留当前主 URDF 的关节约束，依赖 NumPy、SciPy、PyYAML、VTK。运行方式：

```bash
python3 tools/build_native_model.py
```

`urdf/full_body_robot.standalone.urdf` 保留完整 CAD 活动件骨架；`full_body_robot.standing.urdf` 须配合 `config/standing.json` 的初始角度显示。`mujoco/visual/` 与 `mujoco/collision/` 仅沿用历史网格路径，主 URDF 仍引用它们；仿真 XML、仿真扫描和 HIL 专用入口已移除。

历史扫描范围及辅助件数值惯量不代表实机验收。机械限位、方向／编码器零位、电缆约束、质量属性和跨组姿态仍须独立核验。

生成器须以现有主 URDF 为限位输入；该文件缺失时拒绝重建。完整 standalone、站姿及安装记录是已核验快照，CAD 变更后须单独重新核验和更新，当前生成器不会自动重算闭环或站姿。
