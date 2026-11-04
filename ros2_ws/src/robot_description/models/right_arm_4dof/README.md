# 右臂四自由度模型

`right_arm_4dof` 由用户提供并经过实机脚本测试的 `4.4` 模型纳入工程，替换旧的 `zhoubuguanjie_youbi_001`。保留 9 个链接、4 个转动关节和 4 个固定关节，关节名保留原始大小写 `Joint_link1` 至 `Joint_link4`。

- `cad/`：全部 24 个原始文件，逐字节保留；校验值见 `source_sha256.json`。其中测试脚本会操作 CAN 电机，不属于工程测试入口。
- `mujoco/right_arm_4dof.xml`：以实测脚本使用的 `cad/urdf/4.4.mjcf.xml` 为权威源，仅修改根模型名。仿真步长、重力、质量、质心、惯量、阻尼、摩擦和执行器限制均保留。
- `urdf/right_arm_4dof.urdf`：同步 MJCF 的第二关节轴向 `0 0 -1`、角度限位精度、阻尼和摩擦；另外修改模型名与网格 URI。原 URDF 的质量、质心、惯量和关节原点已与 MJCF 一致，未重新估算。
- `urdf/right_arm_4dof.standalone.urdf`：与运行 URDF 完全相同，供 MuJoCo 的 TF 发布入口使用。
- `meshes/`：原始 9 个 STL，未简化或重新导出。

工作台可自动发现此模型，`robot_sim` 的 `display.launch.py model:=right_arm_4dof` 和 `mujoco.launch.py model:=right_arm_4dof` 可使用它。未配置 Gazebo xacro、真机电机映射和 HIL 拓扑；旧模型的控制器配置不适用于这些关节名。

原始 URDF 的速度上限均为 `0`，MJCF 未提供实测速率限值，因此继续保留 `0`。执行器力矩上限为 `10 / 5 / 5 / 5 N·m`。原始 MJCF 注释注明质量属性包含网格估算值；本次保留用户选定的实测模型，不将其描述为所有质量属性都已独立标定。接入实机控制前仍需确认电机映射、方向、零位和速度限值。
