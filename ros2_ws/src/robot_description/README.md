# 机器人公共模型

`models/<model>/` 是机械模型和传动配置的唯一来源。MCU 整机启动、TF、网页和仿真都从本包读取；模型包不启动硬件、Agent、控制器或仿真器。

- `urdf/<model>.urdf`：机械关节、惯量、限位和碰撞几何。
- `meshes/`：URDF 使用 `package://robot_description/models/...` 引用的网格。
- `config/`：该模型的执行器映射、传动及仿真配置。
- `tools/`：模型派生与几何检查脚本。
- `cad/`：原始 CAD 导出参考，不作为独立 ROS 包构建。

`config/motor.yaml` 是执行器型号库的权威源。模型配置通过包内相对路径引用它，源码树与隔离安装树都可直接解析，不依赖兄弟包的目录布局。

`robot_sim` 负责仿真启动与桥接；模型资产统一由本包提供。网页兼容 URL `/robot_sim` 也指向本包。

这些模型和默认参数不是装配验收记录。修改模型或传动后需重新核验实机映射、方向和限位。`dist/` 与可选 MJCF 是由工具生成的派生产物，未生成时不构成已验证的仿真后端。

`right_arm_4dof` 是导入的实测 4.4 右臂模型，以其原始 MJCF 为准同步运行 URDF。原件和适配范围见该模型的 [README](models/right_arm_4dof/README.md)；它不使用其他模型的自动生成工具覆盖测试参数。

`full_body_robot` 是无头部全身 CAD 预览模型，包含 38 个连杆和 37 个关节条目。原始参数和网格完整保留；其机械限位和电机映射尚未标定，仅提供几何预览。来源和使用边界见该模型的 [README](models/full_body_robot/README.md)。

`trial_arm_7dof` 已由 `手臂_mass` 新版替换，包含 14 个连杆、7 个转动关节和 6 个固定关节；三处电机质量属性使用经用户批准的均匀密度凸包近似，尚未完成动力学或实机验收。新旧来源、检查结果和使用边界见其 [README](models/trial_arm_7dof/README.md)。
