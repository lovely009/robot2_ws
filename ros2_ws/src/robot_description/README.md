# 机器人公共机械模型

`models/<model>/` 提供机械 CAD、URDF、显示与碰撞网格、惯量、关节命名及主机传动配置。本包不提供仿真器、假电机、HIL 模型或仿真控制器。

- `urdf/`：机械关节、惯量及几何；standalone 可包含完整活动件显示骨架。
- `meshes/`：URDF 的 `package://robot_description/models/...` 网格。
- `config/`：执行器离线映射、传动、坐标约定与装配记录。
- `tools/`：机械派生、惯量和几何检查。
- `cad/`：机械来源和历史核验记录，不作为独立 ROS 包构建。

`config/motor.yaml` 是执行器型号库来源；主机映射使用包内相对路径，源码与隔离安装均可解析。模型参数与历史几何报告不代表实机方向、零位、机械限位或动力学验收。

`half_body_robot`、`robot_leg` 和 `robot_waist` 的 `tools/regen_derived.py` 从主 URDF 生成 standalone 和 ROS 无关的 `dist/`，不依赖仿真薄壳。`full_body_robot/tools/build_native_model.py` 从已核验的装配记录重建 37 轴机械骨架和网格，保留当前主 URDF 的关节约束；不会生成或认证新的机械安全限位。

整机 URDF 使用的 `full_body_robot/mujoco/visual/` 与 `mujoco/collision/` 是沿用的网格目录名称，保留其路径以确保显示和动力学几何引用完整；其中没有仿真场景 XML。机械来源中的历史报告仅作追溯。
