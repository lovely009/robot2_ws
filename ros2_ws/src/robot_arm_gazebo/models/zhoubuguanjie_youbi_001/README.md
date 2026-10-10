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

| 关节 | 父 → 子 | 类型 | 轴（URDF 表达） | 占位限位 (rad) |
| --- | --- | --- | --- | --- |
| `joint_link1` | `base_link` → `Joint_link1` | revolute | (0, 0, −1) | ±1.5708 |
| `joint_link2` | `Joint_link1` → `Joint_link2` | revolute | (0, 0, −1) | ±1.5708 |
| `joint_link3` | `Joint_link2` → `Joint_link3` | revolute | (0, 0, −1) | ±1.5708 |
| `joint_link4` | `Joint_link3` → `Joint_link4` | revolute | (0, 0, 1) | ±1.5708 |
| `joint_link5` | `Joint_link4` → `Joint_link5` | revolute | (0, 0, 1) | ±1.5708 |
| `joint_link6` | `Joint_link5` → `Joint_link6` | revolute | (0, 0, 1) | ±1.5708 |
| `joint_wrist_ball_a` | `Joint_link6` → `wrist_ball_a` | revolute | (0, 1, 0) | ±3.1416 |
| `joint_link7` | `wrist_ball_a` → `Joint_link7` | revolute | (0, 0, 1) | ±3.1416 |
| `joint_motor_a` | `Joint_link4` → `Joint_link10` | **fixed** | — | — |
| `joint_crank_a` | `Joint_link10` → `crank_a` | revolute | (0, 0, 1)＝电机输出轴 | ±3.1416 |
| `joint_pin_a` | `crank_a` → `Joint_link9` | **fixed** | — | — |
| `joint_wrist_ball_b` | `Joint_link6` → `wrist_ball_b` | revolute | (0, 1, 0) | ±3.1416 |
| `joint_link7_0` | `wrist_ball_b` → `Joint_link7_0` | revolute | (0, 0, 1) | ±3.1416 |
| `joint_motor_b` | `Joint_link4` → `Joint_link10_0` | **fixed** | — | — |
| `joint_crank_b` | `Joint_link10_0` → `crank_b` | revolute | (0, 0, 1)＝电机输出轴 | ±3.1416 |
| `joint_pin_b` | `crank_b` → `Joint_link9_0` | **fixed** | — | — |

主链是 `joint_link1` → `joint_link6` 的串联；其余是腕部的两条远置驱动分支（见下节）。
带 `crank_*`/`wrist_ball_*` 的是零质量虚拟连杆，`joint_wrist_ball_*` 与 `joint_link7*`
合起来构成连杆腕端的二自由度球铰，`joint_crank_*` 是摇臂转角 —— 这三个被动自由度由
`scripts/wrist_closure_solver.py` 按闭环约束解出。

`arm_position_controller` 的 `Float64MultiArray` 命令顺序是可控关节的有序子序列
（当前为 `joint_link1` → `joint_link4`，见 `config/controllers.yaml`，由
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
| `crank_a` / `crank_b` | 1e−6（虚拟） | 曲柄占位连杆：CAD 里「转子 + 输出法兰 + 销」合在电机装配体中，无独立零件 |

装配位形（各关节角 = 0）下臂沿 +x 水平伸出：`base_link` 几何 z −0.030 ~ 0.033
（底面在 z = −0.030），整臂 x ≈ −0.03 ~ 0.52 m。`gazebo.launch.py` 的 `spawn_z`
默认 **0.03**，让基座底面落在地面上（换模型要重新核这个值）。

## 腕部机构与闭环（重要）

`joint_link5` 之后是**远置驱动（并联连杆）腕**：两台 X25S 关节电机固定在小臂上，
各自经「曲柄—连杆」驱动十字轴腕的一个自由度；**两个电机的转角共同决定腕部的两个
自由度**。装配位形下的几何证据（STL 最小间距 / 圆柱拟合）：

- `Joint_link10` / `Joint_link10_0`（两台电机）与 `Joint_link4`（小臂）距离
  0.13 mm / 0.01 mm——电机安装面贴在小臂上；
- 两台电机是**同半径（17.0 mm）圆柱、轴线几乎平行**（都离世界竖直 13.4°，互相夹角
  2.5°），轴心沿臂向相隔 35 mm —— 典型的双电机并联腕；
- `joint_link5` 与 `joint_link6` 的轴正交且共点（十字轴万向节）。

真实机构的运动学闭环（每分支一条）：
`小臂 4 —(固定) 电机 10 —(摇臂绕输出轴) 曲柄销 9 —(球铰) 连杆 7 —(球铰) 二轴 6 —5— 4`。

### 树里补了什么（2026-10-10 重构后）

URDF 是树，闭环必须切开。**切点选在「连杆末端球心」这条边上** —— 于是电机、曲柄销都
落在小臂一侧，连杆挂在小臂侧当自由端，闭环由求解器在运行时补平。树里补了三处，
**都不改已有几何**（补完后零位世界位姿与改写前逐位相同，偏差 8e-12 m，由
`test_wrist_closure.py` 与 `test_model_consistency.py` 兜底）：

1. **电机固定于小臂**：`joint_motor_a`/`_b`（fixed，`Joint_link4` → `Joint_link10/10_0`），
   origin 取零位时电机在小臂系里的位姿。此前 SW2URDF 把电机挂在链条**末端**（连杆的
   子节点），拖腕部关节时电机会被甩出去 —— 这是 2026-10-10 之前 RViz 里「散架」的根因。
2. **曲柄自由度**：`crank_a`/`crank_b`（零质量虚拟连杆，= 电机转子 + 输出法兰 + 销这个
   回转刚体）+ `joint_crank_a`/`_b`（revolute），轴就是**电机输出轴**，origin 放在轴上、
   局部 z 即输出轴。`joint_pin_a`/`_b`（fixed）把曲柄销 `Joint_link9/9_0` 固接上去。
3. **球铰**：连杆两端都是关节轴承（GE43 向心关节轴承）＝球铰，而 SolidWorks 的树导出
   只能给单轴 revolute。每分支因此插了一个零质量虚拟连杆（`wrist_ball_a`/`wrist_ball_b`）
   与一个正交 revolute（`joint_wrist_ball_a`/`_b`，轴取 (0,1,0)——与既有 (0,0,1) 正交且
   垂直于连杆轴向，避免退化）；原 `joint_link7`/`joint_link7_0` 成为球铰的第二轴，
   其 origin 归零（球铰已经在那一点上了，不归零会把平移算两遍）。
   这两个轴的 `<limit>` 放到 ±π 是**有意**的：它们是球铰的分解角、不是物理量，
   该分解在行程边缘接近奇异（±90° 行程内 α 可达 176°，而连杆轴向真实摆动只有 98°）。

关节数不变：**12 revolute + 4 fixed**（删掉 `joint_link9/_0` 与 `joint_link10/_0`，
新增 `joint_crank_a/_b` 与 `joint_motor_a/_b` + `joint_pin_a/_b`；revolute 净额为零，
`<ros2_control>` 的「声明全部非 fixed 物理关节」契约仍成立）。

### 曲柄半径与可达行程

曲柄销中心到电机输出轴的垂距 = **曲柄半径 22.18 / 21.93 mm**（两分支独立算出、相差
0.25 mm；都恰好超出电机缸面 R=17 mm 约 5 mm）。这个「两条互不相干的机构算出同一个数」
是**摇臂轴 = 电机输出轴**的强旁证。

**可达行程是有限的**：连杆 A 分支把能闭上的范围限在 `|joint_link5|, |joint_link6| ≲
0.85 rad`（≈49°）—— ±0.8 rad 内 225 个位形全可解，±0.9 rad 起出现无解位形，±1.4 rad
时 24/225 无解。**URDF 里 `joint_link5/6` 的 ±1.5708 是占位限位，超过了连杆机构实际
能闭上的范围**。超出可达域时求解器只夹紧并告警，不外推。

### 闭环求解器

`scripts/wrist_closure_solver.py` 订阅上位发布的关节角，按闭环约束解出四个被动关节
（每分支：球铰两轴 + 销轴补偿角）后再发出去。`display.launch.py` 默认接上它：
滑杆节点改发 `/joint_states_raw`，求解器发 `/joint_states`，`robot_state_publisher`
订阅后者；`closure_solver:=false` 可关掉，回到原始的「散架」行为做对照。
解法全是闭式（球面 ∩ 圆 → 摇臂角；球铰两轴 → 连杆指向），无迭代；`test_wrist_closure.py`
不采信求解器自报的指标，而是把解出的角代回真实树 FK，比对「球心位置 + 末端朝向」是否
等于「摇臂绕轴刚性转过 φ」的目标，误差在浮点噪声量级。

### ⚠ 数值问题（接真机前必须解决）

1. **`Joint_link9`/`Joint_link9_0` 的 URDF 原点不在曲柄端球心上**。实测量得球心在
   link9 局部系的 `(-0.378, -9.893, 2.780) mm`（分支 B：`(-1.213, -9.825, 2.980)`），
   而 URDF 原点是 `(0, 0, 0)`，偏差约 **10 mm**。该偏移由两种独立方法互证（轴承盘中心
   切片拟合、连杆远端环形区质心）。求解器用的是实测球心（配置里的 `crank_pin_offset`），
   所以**不要反过来去"修" URDF 的 joint origin**——那会同时改动导出件与派生件，
   而 CAD 里零件本来就在那个位置。
2. **曲柄臂（摇臂）几何不存在**。整个 CAD 零件清单里没有任何叫 `曲柄/摆臂/摇臂` 的零件；
   电机 mesh 是纯回转体（r∈[15,21) mm 占 74%，最大 24.4 mm），r>19 mm 的材料角向分散
   （最大 30° 扇区仅占 28%），**没有臂状凸出**；曲柄销就在电机外缘（垂距 17.9 mm）。
   所以 `crank_a`/`crank_b` 只能是零质量占位连杆：重构后销会绕电机输出轴公转、
   连杆会跟着走，但看不到曲柄臂体在转。**需要从 SolidWorks 单独导出摇臂零件**。
3. **可达行程未做真机标定**：模型给出的 ±0.85 rad 是用现有参数算出的闭环可达域，
   真机行程应以机械限位为准（`joint_link5/6` 现在的 ±1.5708 是占位值）。

> **已作废的历史结论**：2026-10-10 之前的 README 与
> `docs/reports/2026-10-10-wrist-closure-implementation.md` §2 曾断言「电机 mesh 自身的
> 回转轴是局部 z、离世界竖直 75°，作为摇臂轴只有 49%/62% 可解 ⇒ 电机输出轴与摇臂轴不是
> 同一根，中间还有一级直角传动」。**该结论是把 mesh 的法向统计读错了轴**：用「法向零空间
> + 侧表面投影圆拟合」（拟合中位残差 1.5 mm）实测，电机回转轴主要是**局部 y**、离世界竖直
> **13.4°**，两台电机轴互相平行（夹角 2.5°）。以它为摇臂轴，两分支独立算出的曲柄半径吻合
> 到 0.25 mm。**摇臂轴就是电机输出轴**。

### 仿真能力边界

`config/controllers.yaml` 与 `simulation_topology.yaml` 仍只接管肩肘 4 台 CERB 电机。
腕部被动关节（`joint_crank_a/_b` 与四个球铰轴）在 Gazebo 里**没有驱动**，会在重力下飘
（2026-10-09 冒烟实测：数分钟后被动关节漂移 ≲0.57 rad——gz_ros2_control 对声明了 position
命令接口的关节默认只给 0.1 的位置比例增益），此时闭环不成立、连杆会脱开。**Gazebo 侧的闭环驱动尚未接**：
那个增益是硬件级参数，为腕部抬高它会同时改变已被验证的肩肘闭环行为，需要单独决策。
所以：腕部在 **RViz 里能正确联动**（走求解器），在 **Gazebo 里仍只是几何资产**，
本模型依旧不能用于腕部动力学或腕部驱动验证。

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
3. `<limit>` 的 `0/0/0/0` → 占位值（12 个 revolute 关节）；球铰分解轴取 ±π、
   曲柄轴取 ±π（理由见「腕部机构与闭环」）；
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
