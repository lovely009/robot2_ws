# 腕部连杆闭环实施与两处数值缺陷（2026-10-10）

**接手先读**：本报告 → `ros2_ws/src/robot_arm_gazebo/models/zhoubuguanjie_youbi_001/README.md`
（模型权威文档，腕部章节）→ 模型目录 `config/wrist_closure.yaml`（参数与其不确定度）。
本报告取代 [2026-10-09 交接](2026-10-09-wrist-linkage-closure-handoff.md) 的 §2.4/§2.5 数值部分。

## 0. 一句话现状

| 项 | 状态 |
| --- | --- |
| 方案 α（球铰 + 求解器 + RViz 链路） | **已实施**（`colcon test` 全绿，端到端误差在浮点噪声量级） |
| 摇臂轴参数 | **仍未标定**，按纯运动学判据取"近竖直过电机形心"；已发现与电机输出轴矛盾（§2） |
| `joint_link9` 原点 ≠ 曲柄端球心 | **已查实**，偏差约 10 mm；求解器改用实测球心绕开（§1） |
| Gazebo 侧闭环驱动 | **未做**：增益是硬件级参数，抬高它会影响已验证的肩肘闭环（§4） |

## 1. 缺陷一：`joint_link9` 的 URDF 原点不在球心上（≈10 mm）

两种独立方法互证：

| 测量 | 结果 |
| --- | --- |
| link9 mesh 自身系：整个零件是绕局部 `(x, y)` 轴线的回转体（各 z 切片中心一致），轴承盘外径 12 mm、z 向宽 3.35 mm，螺钉穿内孔两端各伸出一段 | 轴线在 `(-0.378, -9.893)`／`(-1.213, -9.825)` mm |
| 连杆远端（曲柄端）环形区质心，连杆局部系 | 分支 A `y = -9.46` mm（换算后一致）；分支 B `-5.70`（换算后 `-6.05`，一致） |
| URDF `joint_link9` 原点在连杆局部系 | 分支 A `y = +0.000036` mm |

⇒ 球心比 URDF 原点在连杆局部 y 上高约 **9.9 mm**（该方向在世界系里近竖直），
连杆方向随之偏约 11°，杆长也从 URDF 的 101.81 mm 变成 **103.57 mm**（分支 B：67.05 → 65.78 mm）。
**处理方式**：不动 URDF 的 joint origin（CAD 里零件本来就在那个位置，改了会与导出件分叉），
而是把实测球心放进 `config/wrist_closure.yaml` 的 `wrist_ball_offset` / `crank_pin_offset`。

## 2. 缺陷二：摇臂轴与电机输出轴不是同一根

写了 band 判据（"杆长球面 ∩ 曲柄圆"必须对**整个腕部行程**有解）的证伪器，
按方向扫全空间：只有离世界竖直 **~20° 以内**的方向满足；θ≥45° 急剧变差、θ≥75° 基本无解。
而 X25S 电机 mesh 自身是绕**局部 z** 的回转体（法向零空间判据：97% 面积平行于 z；
中心另有 r≈3 mm 通孔），局部 z 在世界系里是 `(0.075, -0.962, 0.264)`——**离竖直 75°**，
作为摇臂轴时只有 49%/62% 的位形可解。用实测球心复算，结论不变。

⇒ 要么电机与摇臂之间还有一级直角传动（用户口述的"端面法兰 → 短曲柄"漏了这一段），
要么本次对 mesh 的解读有误。**需要在 SolidWorks 里核对**：输出法兰圆面法线、
摇臂回转中心、摇臂销孔中心到轴的距离。旁证：两分支由此算出的曲柄半径几乎相同
（19.57 / 19.51 mm），轴方向在 link4 系里也几乎一致。

轴心位置在运动学上不可辨识（可行集很大），**方向可以辨识**——这是本次相对
2026-10-09 交接的实质进展：卡点从"三个参数全未知"缩小到"轴心待测 + 方向已定在 20° 内"。

## 3. 实施内容

- **模型**：每分支插一个零质量虚拟连杆（`wrist_ball_a/_b`）+ 一个正交 revolute
  （`joint_wrist_ball_a/_b`，轴 (0,1,0)：与既有 (0,0,1) 正交且垂直于连杆轴向，避免退化），
  原 `joint_link7/_0` 成为球铰第二轴、origin 归零。补完后零位 P7/P9 与改写前逐位相同。
  `<ros2_control>` 声明 12 个 revolute；`standalone`/`dist` 已重生成。
- **参数**：`models/zhoubuguanjie_youbi_001/config/wrist_closure.yaml`（轴 + 球心，带来源与不确定度）。
- **求解器**：`scripts/wrist_closure_solver.py`，全闭式无迭代；订阅 `/joint_states_raw`，
  发 `/joint_states`；无解或超限只夹紧并告警，不放大概率。
- **RViz 链路**：`display.launch.py` 默认启动求解器；`closure_solver:=false` 关掉做对照。
- **测试**：`test/test_wrist_closure.py`（4 个用例 / 6 项）。判据不用求解器自报的指标，
  而是把解出的被动角代回真实树 FK，比对"球心位置 + 末端朝向"是否等于"摇臂绕轴刚性转过 φ"。

## 4. 验证

- `colcon test --packages-select robot_arm_gazebo`：17 项全绿（含新增 4 个用例）；全工作区 238 项全绿。
- 端到端（分支 A/B × 腕部 ±0.3/±0.8/±1.4 rad 网格）：球心位置偏差 **0.0000 µm**、朝向偏差 **2e-6°**。
  位置是闭式精确解；朝向由销轴补偿角闭合——两分支的"连杆自转"两支都精确满足位置约束，
  求解器取姿态残差小的那支，所以不像 2026-10-09 预判的那样有姿态残差。
- `ign sdf -p`（standalone）转换通过，Gazebo 接受零质量虚拟连杆。
- 零位自洽：解出的四个被动角全为 0.000°（装配位形就是 φ=0）。
- **未做**：没有启动 Gazebo 跑闭环（见 §5），没有真机验证。

## 5. 遗留

1. **摇臂轴待标定**（§2）——拿到数值只改 `wrist_closure.yaml`，不动代码。
2. **Gazebo 侧闭环驱动未接**：gz_ros2_control 的 position 比例增益是**硬件级**参数
   （默认 0.1），为腕部抬高它会同时改变已被验证的肩肘闭环行为，需要单独决策（可能要立 ADR）。
   现状：腕部在 RViz 里能正确联动，在 Gazebo 里仍只是几何资产、会漂移。
3. `test_wrist_closure.py` 的杆长断言写的是实测区间（103.6/65.8 mm ±3 mm）——
   球心若被重新标定，这两个数要同步改。
4. 两分支的球心测量不确定度约 ±1 mm（腕端眼孔外圆柱拟合噪声；曲柄端由轴承盘定，
   更稳）。腕端 `wrist_ball_offset` 在连杆局部系里与 link 原点只差 ≤2.1 mm。

## 6. 复现

```bash
cd ~/robot_ws/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
colcon test --packages-select robot_arm_gazebo && colcon test-result --verbose \
  --test-result-base build/robot_arm_gazebo
ros2 launch robot_arm_gazebo display.launch.py            # 腕部跟随闭环
ros2 launch robot_arm_gazebo display.launch.py closure_solver:=false   # 对照：散架
```

分析与反演脚本（一次性工具，路径按本机硬编码）：本次新增的在 `/tmp`（未入库）；
2026-10-09 那批仍在 `docs/reports/2026-10-09-wrist-linkage-closure-assets/`。
