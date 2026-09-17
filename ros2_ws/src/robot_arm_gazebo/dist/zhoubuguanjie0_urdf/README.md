# zhoubuguanjie0 机械臂 URDF 模型

四自由度肘关节机械臂的完整运动学/动力学模型，单个 URDF 文件 + 网格，
不依赖 ROS，解压即用。

```
zhoubuguanjie0_urdf/
├── zhoubuguanjie0.urdf   # 模型本体（纯文本，可直接编辑）
└── meshes/               # 5 个 STL 网格，与 URDF 用相对路径关联
```

## 怎么打开

- **在线查看器**：把 `zhoubuguanjie0.urdf` 拖进任意 URDF 在线预览网站即可（需连同 meshes 一起上传）。
- **Blender / MuJoCo / PyBullet / 其他仿真器**：导入 `zhoubuguanjie0.urdf`，网格按相对路径自动找到。
- **ROS 2 + RViz**（需已装 ROS 2）：
  ```bash
  cd zhoubuguanjie0_urdf        # 必须先进这个目录：相对路径按当前工作目录解析
  ros2 run joint_state_publisher_gui joint_state_publisher_gui &
  ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:="$(cat zhoubuguanjie0.urdf)"
  rviz2
  ```
  在 RViz 里 Add → RobotModel，Fixed Frame 选 `base_link`。

## 模型结构

固定基座 `base_link`，往外串 4 个 revolute 关节（绕各自 z 轴），
每个关节 `effort=10 Nm`、`velocity=1 rad/s`：

| 关节名 | 父 link | 子 link | 限位 (rad) |
| --- | --- | --- | --- |
| `joint_link1` | `base_link` | `Joint_Link1` | 0 ~ 1.57 |
| `joint_link2` | `Joint_Link1` | `Joint_Link2` | -1.57 ~ 1.57 |
| `joint_link3` | `Joint_Link2` | `Joint_Link3` | -0.09 ~ 2.09 |
| `joint_link4` | `Joint_Link3` | `Joint_Link4` | -1.57 ~ 1.57 |

单位：长度 m、角度 rad、质量 kg、惯量 kg·m²。

## 参数来源与注意点

- 几何与惯性参数由 SolidWorks 模型导出，**未做真机标定**。
- 关节限位、`effort=10 Nm`、`velocity=1 rad/s` 是**名义占位值**，
  接真实执行器前请按实际参数修改。
- 模型不含任何控制器、仿真插件或传感器，是纯模型。
