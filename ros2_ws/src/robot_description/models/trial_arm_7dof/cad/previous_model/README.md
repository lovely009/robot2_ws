# trial_arm_7dof

原名 `试用_手臂`，8 个连杆、7 个串联转动关节，总声明质量 2.784365668 kg。
更名仅涉及运行 URDF 的 robot 名称和网格 URI，关节名、坐标、轴向、质量、质心和惯量逐项保留。

- `cad/`：原始导出包（包括 CSV、URDF、STL、日志和 ROS 1 launch），作为来源记录。
- `urdf/trial_arm_7dof.urdf` 和 `meshes/`：公共模型资源，现有 Web 模型目录可自动枚举。
- `cad/inertial_audit/2026-11-05.md`：质心/惯量核对结论；同名 JSON 是逐连杆数值。
- `tools/audit_inertia.py`：在模型目录无关的位置运行，重建 JSON 检查记录；需要 NumPy。

用户确认关节 1、2（`Joint_Link1`、`Joint_Link2`）采用位置控制；当前用途按几何和运动学验收，不要求这两项质量属性准确。原始惯性参数保留为未验收数据；若以后用于全臂动力学、重力补偿或物理仿真，需重新核实相关质量分布。这是用途说明，尚未写入驱动配置。

构建并 source 含此模型的 robot_description 安装环境后，可离线预览：

```bash
ros2 launch robot_sim display.launch.py model:=trial_arm_7dof
```

7 个关节的 lower、upper、effort、velocity 都为 0，按限位工作的滑杆无法活动。
尚无执行器映射、实测机械限位、闭环/驱动关系或动力学验收；只提供几何预览，不能直接用于电机控制或可信动力学仿真。
本次未改变默认模型或已调试的 right_arm_4dof，也未配置 Gazebo/MuJoCo/HIL。

关节3～7的再次复核见 [质量、质心与惯量复核](cad/inertial_audit/2026-11-05-recheck.md)，重点记录关节7均匀密度假设下约37%～38%的惯量差异，以及关节6电机质量来源差异。

URDF/STL联合核对发现 J2 约5mm偏心、J3 约2.59°法兰轴向差，以及J5约4.282mm偏心/2.475°轴向差；详见 [几何装配核对](cad/assembly_audit/2026-11-05.md) 和其中的局部叠加图。尚未按推测改写运行参数。
