# zhoubuguanjie_youbi_001 机械臂 URDF 模型

四自由度机械臂（固定基座 + 4 个转动关节）的运动学/动力学模型，
单个 URDF 文件 + 网格，不依赖 ROS，解压即用。

```
zhoubuguanjie_youbi_001_urdf/
├── zhoubuguanjie_youbi_001.urdf   # 模型本体（纯文本，可直接编辑）
└── meshes/                     # 5 个 STL 网格，与 URDF 用相对路径关联
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

| 关节名 | 父 link | 子 link | 轴 | 限位 (rad) |
| --- | --- | --- | --- | --- |
| `joint_link1` | `base_link` | `Joint_link1` | (0,0,-1) | -1.5708 ~ 1.5708 |
| `joint_link2` | `Joint_link1` | `Joint_link2` | (0,0,1) | -1.5708 ~ 1.5708 |
| `joint_link3` | `Joint_link2` | `Joint_link3` | (0,0,-1) | -1.5708 ~ 1.5708 |
| `joint_link4` | `Joint_link3` | `Joint_link4` | (0,0,-1) | -1.5708 ~ 1.5708 |

连杆名是 CAD 原名 `Joint_linkN`（关节模组的名字），关节名统一为小写 `joint_linkN`，
两者一一对应。顺序即从基座往外的串联顺序。单位：长度 m、角度 rad、质量 kg、惯量 kg·m²。

本文件是 `robot_arm_gazebo` 仓库内 `models/zhoubuguanjie_youbi_001/` 的对外发布形态
（网格改为相对路径，去掉 ROS 依赖）；仓库里那份另带 `ros2_control` 与 Gazebo 集成。

## ⚠️ 参数来源与必须修改的地方

- **关节限位、力矩、速度全是占位值**：SolidWorks 原始导出为
  `lower=upper=effort=velocity=0`，revolute 关节限位为 0 会导致 RViz 滑杆拖不动、
  多数仿真器直接锁死关节，所以本包填入了 -1.5708~1.5708 rad、10 Nm、1 rad/s 的
  占位值让模型能动起来。**接真机前必须按实际机械限位与执行器参数修改。**
- 几何与惯性参数由 SolidWorks 模型导出，**未做真机标定**。
- 模型不含任何控制器、仿真插件或传感器，是纯模型。
