# zhoubuguanjie_youbi_001（右臂）

右臂模型：R58 肩座 + 6 个主链 revolute 关节 + 腕部远置驱动连杆机构。几何与惯性由
SolidWorks 装配体导出（2026-10-09 版），关节由装在臂上的 CAN-FD 一体化关节模组驱动。

```
zhoubuguanjie_youbi_001/
├── README.md                       # 本文件
├── cad/                            # SolidWorks 导出留档（provenance，不参与仿真）
│   ├── zhoubuguanjie_youbi_001.csv # 逐连杆的质量/惯量/关节原点表
│   └── export.log                  # SW2URDF 导出器日志
├── config/
│   ├── controllers.yaml            # gz_ros2_control 参数（含 joints 表）
│   └── simulation_topology.yaml    # 电机 → 关节映射（完整仿真入口用）
├── meshes/*.STL                    # 13 个网格，URDF 按 package:// 引用
└── urdf/
    ├── zhoubuguanjie_youbi_001.urdf            # 原始导出件改写版（本包主要模型文件）
    ├── zhoubuguanjie_youbi_001.standalone.urdf # 纯模型单文件（无 ros2_control / 插件）
    └── zhoubuguanjie_youbi_001.urdf.xacro      # 仿真薄壳（注入 ros2_control 与 gz 插件）
```

## 关节

| 关节 | 父连杆 → 子连杆 | 轴（URDF 表达） | 占位限位 (rad) |
| --- | --- | --- | --- |
| `joint_link1` | `base_link` → `Joint_link1` | (0, 0, −1) | −1.5708 ~ 1.5708 |
| `joint_link2` | `Joint_link1` → `Joint_link2` | (0, 0, −1) | −1.5708 ~ 1.5708 |
| `joint_link3` | `Joint_link2` → `Joint_link3` | (0, 0, −1) | −1.5708 ~ 1.5708 |
| `joint_link4` | `Joint_link3` → `Joint_link4` | (0, 0, 1) | −1.5708 ~ 1.5708 |
| `joint_link5` | `Joint_link4` → `Joint_link5` | (0, 0, 1) | −1.5708 ~ 1.5708 |
| `joint_link6` | `Joint_link5` → `Joint_link6` | (0, 0, 1) | −1.5708 ~ 1.5708 |
| `joint_link7` | `Joint_link6` → `Joint_link7` | (0, 0, 1) | −1.5708 ~ 1.5708 |
| `joint_link9` | `Joint_link7` → `Joint_link9` | (0, 0, 1) | −1.5708 ~ 1.5708 |
| `joint_link10` | `Joint_link9` → `Joint_link10` | fixed | — |
| `joint_link7_0` | `Joint_link6` → `Joint_link7_0` | (0, 0, 1) | −1.5708 ~ 1.5708 |
| `joint_link9_0` | `Joint_link7_0` → `Joint_link9_0` | (0, 0, 1) | −1.5708 ~ 1.5708 |
| `joint_link10_0` | `Joint_link9_0` → `Joint_link10_0` | fixed | — |

主链关节顺序即 `joint_link1` → `joint_link6` 的串联顺序；`joint_link7/9`、`joint_link7_0/9_0`
与 `joint_link10/10_0` 是腕部的两条远置驱动分支（见下节）。
`arm_position_controller` 的 `Float64MultiArray` 命令顺序是其中被电机驱动的关节的
有序子序列（当前为 `joint_link1` → `joint_link4`，见 `config/controllers.yaml`，由
`test/test_model_consistency.py` 兜底）。

**连杆名与关节名不同名**：连杆保留 CAD 原名 `Joint_linkN`（它就是关节模组的名字，
`.0` 后缀改为 `_0`），关节统一为小写 `joint_linkN`。两者一一对应，不要混用。

各连杆的 CAD 来源（`cad/` 里的 CSV 有完整表）：

| 连杆 | 质量 (kg) | 组成 |
| --- | --- | --- |
| `base_link` | 1.003 | CERB R58 Lite 关节模组（旧版含手臂支撑座，本次导出已去掉） |
| `Joint_link1` | 0.166 | 肩部关节旋转轴 + 旋转轴盖 + CERB R48 Lite |
| `Joint_link2` | 0.347 | 转换板 + 大臂支架 + 40 电机抱闸外壳 + CERB R48 Lite |
| `Joint_link3` | 0.111 | 旋转轴（二板）+ 旋转轴盖 + CERB R48 Lite |
| `Joint_link4` | 0.110 | 腕部转接盘 + 小臂骨架并联机构（含轴承） |
| `Joint_link5` | 0.0037 | 腕部十字轴（含轴承/螺钉） |
| `Joint_link6` | 0.0093 | 腕部二轴 |
| `Joint_link7` / `Joint_link7_0` | 0.0164 / 0.0113 | 腕部连杆 2 / 连杆 1（各含 GE43 关节轴承） |
| `Joint_link9` / `Joint_link9_0` | 0.0028 | GE43 关节轴承 + 半圆头螺钉（销定） |
| `Joint_link10` / `Joint_link10_0` | 0.0054 / 0.0119 | X25S 关节电机（两台，见已知问题） |

装配位形（各关节角 = 0）下臂沿 +x 水平伸出：`base_link` 几何 z −0.030 ~ 0.033
（底面在 z = −0.030），整臂 x ≈ −0.03 ~ 0.52 m。`gazebo.launch.py` 的 `spawn_z`
默认 **0.03**，让基座底面落在地面上（换模型要重新核这个值）。

## 腕部机构与闭环说明（重要）

`joint_link5` 之后是**远置驱动（并联连杆）腕**：两台 X25S 关节电机固定在小臂上，
各自经"摇臂—连杆"驱动十字轴腕的两个自由度。装配位形下的几何证据（STL 最小间距）：

- `Joint_link10` / `Joint_link10_0`（两台电机）与 `Joint_link4`（小臂）距离
  0.13 mm / 0.01 mm——电机安装面贴在小臂上；
- `Joint_link7` / `Joint_link7_0`（两根连杆）两端铰轴（`joint_link7/9`、`joint_link7_0/9_0`）
  两两平行，杆在垂直于该轴的平面内摆动；
- `joint_link5` 与 `joint_link6` 的轴正交且共点（十字轴万向节）。

因此真实机构的运动学闭环为（每个分支一条）：
`小臂 4 —5— 十字轴 5 —6— 二轴 6 —7— 连杆 7 —9— 轴承 9 —— 电机 10 —— 固定于 4`。
URDF 是树，无法表达闭环。SW2URDF 把环切开在"电机固定于小臂"这条边上，导出树为
`… → joint_link6 → joint_link7 → joint_link9 → joint_link10`，即**连杆与电机挂在
腕部二轴末端**。

**仿真后果**：`joint_link5/6` 与两条连杆分支在 Gazebo 里没有驱动、也没有闭环约束，
会在重力下缓慢偏离零位（2026-10-09 冒烟实测：数分钟后 `joint_link9_0` 漂移约
0.57 rad，其余被动关节 ≲0.1 rad——gz_ros2_control 对声明了 position 命令接口的关节
默认只给 0.1 的位置比例增益，不构成有效支撑）。本模型**不能用于腕部动力学或腕部
驱动验证**，腕部只能当几何/可视化资产。`config/controllers.yaml` 与
`simulation_topology.yaml` 都只接管肩肘 4 台 CERB 电机，与这一现状一致。

## ⚠️ 参数来源与必须修改的地方

- **关节限位、力矩、速度全是占位值**：SolidWorks 原始导出为
  `lower=upper=effort=velocity=0`，revolute 关节限位为 0 会导致 RViz 滑杆拖不动、
  Gazebo 等仿真器直接锁死关节，所以本包填入了 −1.5708~1.5708 rad、10 Nm、1 rad/s 的
  占位值让模型能动起来。**接真机前必须按实际机械限位与执行器参数修改。**
- 几何与惯性参数由 SolidWorks 模型导出，**未做真机标定**。
- **两台 X25S 电机的质量不对称**（`Joint_link10` 0.0054 kg vs `Joint_link10_0`
  0.0119 kg），组件清单差一个 `YUANZHU_1.stp`；请对照 CAD 核对选件是否漏选/多选。
- `joint_link2-3`、`joint_link3-4` 的相邻轴为异面（公垂距 ≈ 6 mm / 9 mm，旧版为 0）
  是本次导出的实测值，接真机前请核对。
- 模型本身不含控制器与传感器；控制器参数在 `config/controllers.yaml`，仿真插件由
  `urdf/*.urdf.xacro` 注入。

## 从 SolidWorks 重新导出后的改写步骤

原始导出件不能直接用，重新导出后要重复以下改写（`git diff` 上一版可以看清每处改动）：

1. mesh 路径 `package://zhoubuguanjie ROBOT-0/meshes/` →
   `package://robot_arm_gazebo/models/zhoubuguanjie_youbi_001/meshes/`，网格文件名
   去点号并统一为 `Joint_linkN.STL` / `Joint_linkN_0.STL`（`N.0` → `N_0`）；
2. 连杆名 `Joint_LinkN` → `Joint_linkN`（含 `.0` → `_0`），关节名 → 小写
   `joint_linkN`（`.` 不是 ROS 名字字符，进话题/参数名会炸）；
3. `<limit>` 的 `0/0/0/0` → 占位值（10 个 revolute 关节）；
4. `<material name="">` → `<连杆名>_material`（导出件里不同颜色共用空名字，
   解析器会按名字去重导致颜色错乱）；
5. 删除 fixed 关节的 `<axis xyz="0 0 0" />`（URDF 对 fixed 无意义）；
6. 重生成 `urdf/*.standalone.urdf`：把 xacro 渲染后剥掉 `<ros2_control>`/`<gazebo>`
   （渲染时用 `fixed_base:=false`，纯模型里不该有 `world` 连杆）；
7. 重生成 `../../dist/zhoubuguanjie_youbi_001_urdf/` 的对外发布件（同一步骤，但
   mesh 路径改成相对路径 `meshes/`）。

`test/test_model_consistency.py` 会校验 1~7 的结果（关节清单一致、mesh 存在、限位
非退化、standalone 与 URDF 同源、dist 与模型同源），漏改会在 `colcon test` 里暴露。

## 怎么用

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash

ros2 launch robot_arm_gazebo display.launch.py          # RViz 预览（滑杆驱动）
ros2 launch robot_arm_gazebo gazebo.launch.py           # Ignition 仿真 + 位置/力矩控制器
```

单独加载纯模型：

```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(cat install/robot_arm_gazebo/share/robot_arm_gazebo/models/zhoubuguanjie_youbi_001/urdf/zhoubuguanjie_youbi_001.standalone.urdf)"
```

网格走 `package://robot_arm_gazebo/models/zhoubuguanjie_youbi_001/meshes/`，加载前须
source 本工作空间；要脱离 ROS（Blender / MuJoCo / PyBullet）用
`dist/zhoubuguanjie_youbi_001_urdf/` 那份相对路径版本。
