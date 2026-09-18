# zhoubuguanjie_youbi_001（右臂）

四自由度机械臂（固定基座 + 4 个 revolute 关节）的 URDF 模型：几何与惯性由 SolidWorks
装配体导出，关节由装在臂上的 CAN-FD 一体化关节模组驱动。

```
zhoubuguanjie_youbi_001/
├── README.md                       # 本文件
├── cad/                            # SolidWorks 导出留档（provenance，不参与仿真）
│   ├── zhoubuguanjie_youbi_001.csv # 逐连杆的质量/惯量/关节原点表
│   └── export.log                  # SW2URDF 导出器日志
├── config/controllers.yaml         # gz_ros2_control 参数（含 joints 表）
├── meshes/*.STL                    # 5 个网格，URDF 按 package:// 引用
└── urdf/
    ├── zhoubuguanjie_youbi_001.urdf            # 原始导出件（本包的主要模型文件）
    ├── zhoubuguanjie_youbi_001.standalone.urdf # 纯模型单文件（无 ros2_control / 插件）
    └── zhoubuguanjie_youbi_001.urdf.xacro      # 仿真薄壳（注入 ros2_control 与 gz 插件）
```

## 关节

| 关节 | 父连杆 → 子连杆 | 轴 | 占位限位 (rad) |
| --- | --- | --- | --- |
| `joint_link1` | `base_link` → `Joint_link1` | (0, 0, −1) | −1.5708 ~ 1.5708 |
| `joint_link2` | `Joint_link1` → `Joint_link2` | (0, 0, 1) | −1.5708 ~ 1.5708 |
| `joint_link3` | `Joint_link2` → `Joint_link3` | (0, 0, −1) | −1.5708 ~ 1.5708 |
| `joint_link4` | `Joint_link3` → `Joint_link4` | (0, 0, −1) | −1.5708 ~ 1.5708 |

关节顺序即 `Joint_link1` → `Joint_link4` 的串联顺序，也是
`arm_position_controller` 的 `Float64MultiArray` 命令顺序（由
`test/test_model_consistency.py` 兜底）。

**连杆名与关节名不同名**：连杆保留 CAD 原名 `Joint_linkN`（它就是关节模组的名字），
关节统一为小写 `joint_linkN`。两者一一对应，不要混用。

各连杆的 CAD 来源（`cad/` 里的 CSV 有完整表）：

| 连杆 | 质量 (kg) | 组成 |
| --- | --- | --- |
| `base_link` | 2.224 | 手臂支撑座 + CERB R58 Lite 关节模组 |
| `Joint_link1` | 0.0763 | 关节连接旋转盖 + CERB R48 Lite |
| `Joint_link2` | 0.368 | 大臂支架 + 转换板 + 40 电机抱闸外壳 + CERB R48 Lite |
| `Joint_link3` | 0.132 | 关节旋转轴/轴盖 + CERB R48 Lite |
| `Joint_link4` | 0.156 | 小臂骨架 + X25S 腕部电机 |

实测网格范围（STL 包围盒，用于定生成高度）：`base_link` 沿 z 覆盖 **0 → 0.600 m**、
底面在 z=0；`Joint_link1` z −0.010~0.052；`Joint_link2` z −0.216~0.006；
`Joint_link3` z −0.010~0.052；`Joint_link4` z −0.128~0.009。整条臂从基座底面往上长，
所以 `gazebo.launch.py` 的 `spawn_z` 默认 0.0。

## ⚠️ 参数来源与必须修改的地方

- **关节限位、力矩、速度全是占位值**：SolidWorks 原始导出为
  `lower=upper=effort=velocity=0`，revolute 关节限位为 0 会导致 RViz 滑杆拖不动、
  Gazebo 等仿真器直接锁死关节，所以本包填入了 −1.5708~1.5708 rad、10 Nm、1 rad/s 的
  占位值让模型能动起来。**接真机前必须按实际机械限位与执行器参数修改。**
- 几何与惯性参数由 SolidWorks 模型导出，**未做真机标定**。
- 模型本身不含控制器与传感器；控制器参数在 `config/controllers.yaml`，仿真插件由
  `urdf/*.urdf.xacro` 注入。

## 从 SolidWorks 重新导出后的改写步骤

原始导出件不能直接用，重新导出后要重复以下改写（`git diff` 上一版可以看清每处改动）：

1. mesh 路径 `package://zhoubuguanjie_youbi_001/meshes/` →
   `package://robot_arm_gazebo/models/zhoubuguanjie_youbi_001/meshes/`（10 处）；
2. 关节名 `Joint_linkN` → `joint_linkN`（4 处，连杆名与 `<child link>` 不动）；
3. `<limit>` 的 `0/0/0/0` → 占位值（4 处）；
4. `<material name="">` → `<连杆名>_material`（5 处，导出件里不同颜色共用空名字，
   解析器会按名字去重导致颜色错乱）；
5. 重生成 `urdf/*.standalone.urdf`：把 xacro 渲染后剥掉 `<ros2_control>`/`<gazebo>`
   （渲染时用 `fixed_base:=false`，纯模型里不该有 `world` 连杆）；
6. 重生成 `../../dist/zhoubuguanjie_youbi_001_urdf/` 的对外发布件（同一步骤，但
   mesh 路径改成相对路径 `meshes/`）。

`test/test_model_consistency.py` 会校验 1~5 的结果（关节清单一致、mesh 存在、限位
非退化、standalone 与 URDF 同源），漏改会在 `colcon test` 里暴露。

## 怎么用

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash

ros2 launch robot_arm_gazebo display.launch.py          # RViz 预览（滑杆驱动）
ros2 launch robot_arm_gazebo gazebo.launch.py           # Ignition 仿真 + 位置控制器
```

单独加载纯模型：

```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(cat install/robot_arm_gazebo/share/robot_arm_gazebo/models/zhoubuguanjie_youbi_001/urdf/zhoubuguanjie_youbi_001.standalone.urdf)"
```

网格走 `package://robot_arm_gazebo/models/zhoubuguanjie_youbi_001/meshes/`，加载前须
source 本工作空间；要脱离 ROS（Blender / MuJoCo / PyBullet）用
`dist/zhoubuguanjie_youbi_001_urdf/` 那份相对路径版本。
