# zhoubuguanjie_youbi_001 机械臂 URDF 模型

右臂模型（固定基座 + 6 关节主链 + 腕部远置驱动连杆机构）的运动学/动力学模型，
单个 URDF 文件 + 网格，不依赖 ROS，解压即用。

```
zhoubuguanjie_youbi_001_urdf/
├── zhoubuguanjie_youbi_001.urdf   # 模型本体（纯文本，可直接编辑）
└── meshes/                        # 13 个 STL 网格，与 URDF 用相对路径关联
```

## 怎么打开

- **在线查看器**：把 `zhoubuguanjie_youbi_001.urdf` 拖进任意 URDF 在线预览网站（需连同 meshes 一起上传）。
- **Blender / MuJoCo / PyBullet / 其他仿真器**：导入 URDF，网格按相对路径自动找到。
- **ROS 2 + RViz**（需已装 ROS 2）：
  ```bash
  cd zhoubuguanjie_youbi_001_urdf        # 必须先进这个目录：相对路径按当前工作目录解析
  ros2 run joint_state_publisher_gui joint_state_publisher_gui &
  ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:="$(cat zhoubuguanjie_youbi_001.urdf)"
  rviz2
  ```
  在 RViz 里 Add → RobotModel，Fixed Frame 选 `base_link`。

## 模型结构

主链（从基座往外的串联顺序）：

| 关节名 | 父 link | 子 link | 轴 | 限位 (rad) |
| --- | --- | --- | --- | --- |
| `joint_link1` | `base_link` | `Joint_link1` | (0,0,-1) | -1.5708 ~ 1.5708 |
| `joint_link2` | `Joint_link1` | `Joint_link2` | (0,0,-1) | -1.5708 ~ 1.5708 |
| `joint_link3` | `Joint_link2` | `Joint_link3` | (0,0,-1) | -1.5708 ~ 1.5708 |
| `joint_link4` | `Joint_link3` | `Joint_link4` | (0,0,1) | -1.5708 ~ 1.5708 |
| `joint_link5` | `Joint_link4` | `Joint_link5` | (0,0,1) | -1.5708 ~ 1.5708 |
| `joint_link6` | `Joint_link5` | `Joint_link6` | (0,0,1) | -1.5708 ~ 1.5708 |

腕部远置驱动分支（两条并联连杆）：

| 关节名 | 父 link | 子 link | 轴 | 限位 (rad) |
| --- | --- | --- | --- | --- |
| `joint_link7` | `Joint_link6` | `Joint_link7` | (0,0,1) | -1.5708 ~ 1.5708 |
| `joint_link9` | `Joint_link7` | `Joint_link9` | (0,0,1) | -1.5708 ~ 1.5708 |
| `joint_link10` | `Joint_link9` | `Joint_link10` | fixed | — |
| `joint_link7_0` | `Joint_link6` | `Joint_link7_0` | (0,0,1) | -1.5708 ~ 1.5708 |
| `joint_link9_0` | `Joint_link7_0` | `Joint_link9_0` | (0,0,1) | -1.5708 ~ 1.5708 |
| `joint_link10_0` | `Joint_link9_0` | `Joint_link10_0` | fixed | — |

连杆名是 CAD 原名 `Joint_linkN`（关节模组的名字，`.0` 后缀写作 `_0`），关节名统一为
小写 `joint_linkN`，两者一一对应。单位：长度 m、角度 rad、质量 kg、惯量 kg·m²。

⚠ **腕部（`joint_link5` 之后）是远置驱动连杆机构**：两台 X25S 电机物理上固定在小臂
（`Joint_link4`）上形成运动学闭环，URDF 是树无法表达，SW2URDF 把环切开在"电机固定于
小臂"这条边上（连杆与电机挂在腕部二轴末端）。因此该模型不能用于腕部动力学或驱动
验证，腕部只能当几何资产；两条连杆在仿真器里会作为被动件随重力摆动。

本文件是 `robot_arm_gazebo` 仓库内 `models/zhoubuguanjie_youbi_001/` 的对外发布形态
（网格改为相对路径，去掉 ROS 依赖）；仓库里那份另带 `ros2_control` 与 Gazebo 集成。

## ⚠️ 参数来源与必须修改的地方

- **关节限位、力矩、速度全是占位值**：SolidWorks 原始导出为
  `lower=upper=effort=velocity=0`，revolute 关节限位为 0 会导致 RViz 滑杆拖不动、
  多数仿真器直接锁死关节，所以本包填入了 -1.5708~1.5708 rad、10 Nm、1 rad/s 的
  占位值让模型能动起来。**接真机前必须按实际机械限位与执行器参数修改。**
- 几何与惯性参数由 SolidWorks 模型导出，**未做真机标定**。
- **两台 X25S 电机的质量不对称**（`Joint_link10` 0.0054 kg vs `Joint_link10_0`
  0.0119 kg），来自 CAD 选件；接真机前请核对。
- 模型不含任何控制器、仿真插件或传感器，是纯模型。
