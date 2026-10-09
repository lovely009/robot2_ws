# Handoff：右臂模型更新与腕部连杆闭环求解（2026-10-09）

> **本报告的 §2.4 / §2.5 数值已被 [2026-10-10 腕部连杆闭环实施](2026-10-10-wrist-closure-implementation.md) 取代**（球心偏差约 10 mm、摇臂轴方向收敛到 20° 内）；§1/§3 仍有效。

**接手先读**：本报告 → `ros2_ws/src/robot_arm_gazebo/models/zhoubuguanjie_youbi_001/README.md`
（模型权威文档，含「腕部机构与闭环说明」）→ 本目录 `…-assets/`（本次分析工具）。

## 0. 一句话现状

| 阶段 | 状态 | 说明 |
| --- | --- | --- |
| 一、新 SolidWorks 导出迁移进原模型 | **完成**（未提交 git） | `colcon test` 10/10、Gazebo 冒烟通过 |
| 二、腕部"曲柄-连杆"按真实机构联动 | **未实施**（方案已定） | 需"球铰 + 曲柄自由度 + 求解层"；唯一卡点是摇臂轴参数（见 §2.5） |

工作区：`dev/robot_ws` 分支，所有改动均在工作区，**未 commit**。原始导出目录
`ros2_ws/src/robot_arm_gazebo/zhoubuguanjie ROBOT-0/` 未动（建议后续归档或移出包目录）。

---

## 1. 第一阶段：模型迁移（完成）

### 1.1 背景

用户 2026-10-09 从 SolidWorks 重新导出 `zhoubuguanjie ROBOT-0/`（13 连杆；10 revolute +
2 fixed，含完整腕部并联连杆），要求更新 `models/zhoubuguanjie_youbi_001/` 并修复描述错误。

### 1.2 改动清单

| 文件 | 改动 |
| --- | --- |
| `models/…/urdf/zhoubuguanjie_youbi_001.urdf` | 由导出件改写：robot 名、mesh 路径（去空格包名）、连杆 `Joint_LinkN(.0)→Joint_linkN(_0)`、关节 `joint_linkN(_0)`（`.` 非法字符）、占位限位（10 revolute）、`<material>` 命名、删 fixed 的零轴 |
| `models/…/meshes/` | 13 个 STL（新命名；旧 5 个被覆盖） |
| `models/…/urdf/*.urdf.xacro` | `<ros2_control>` 声明 10 个 revolute（顺序=URDF）；`<gazebo><gravity>` 引用 12 个 link |
| `models/…/config/controllers.yaml` | 仍只列 4 台驱动关节 `joint_link1..4`（注释说明） |
| `models/…/urdf/*.standalone.urdf`、`dist/zhoubuguanjie_youbi_001_urdf/` | 重新生成（脚本 `regen_derived.py`） |
| `models/…/cad/` | 新 CSV + export.log |
| `models/…/README.md`、包 `README.md` | 关节表（12）、闭环说明、spawn_z、契约更新 |
| `launch/gazebo.launch.py` | `spawn_z` 0.0 → **0.03**（新基座底面 z=−0.030） |
| `test/test_model_consistency.py` | 契约改为：`<ros2_control>` 声明全部非 fixed 物理关节（顺序一致）；`controllers` 为其**有序子序列** |
| `test/test_launch_models.py` | spawn_z 断言 0.03 |

### 1.3 验证证据

- `colcon test --packages-select robot_arm_gazebo`：**10/10 通过**（目录完备、关节契约、
  mesh/限位、standalone 同源、dist 同源、launch 模型链路）。
- **逆向验证**：故意交换 controllers 的 joint_link1/2 顺序 → 测试 FAILED；恢复 → PASSED。
- Gazebo headless 冒烟（步骤见 §3.1）：模型生成成功、gz_ros2_control 加载 10 个 revolute
  关节（2 个 fixed 正确跳过）、`joint_state_broadcaster` + `arm_position_controller` 激活、
  `/joint_states` 广播全部 10 个关节。
- 几何复核：改写后 URDF 的 FK 与网格包围盒与原导出**逐项一致**（探针脚本见 assets）。

### 1.4 遗留（需用户核对 CAD，未擅改）

1. **两台 X25S 质量不对称**：`Joint_link10` 0.0054 kg vs `Joint_link10_0` 0.0119 kg
   （组件清单差一个 `YUANZHU_1.stp`）；
2. `joint_link2-3`、`joint_link3-4` 相邻轴为**异面**（公垂距 ≈6 mm / 9 mm，旧版为 0）；
3. 新网格比旧版更大（单文件最大 ~6 MB，共 24 MB），dist 发布件同步变大。

---

## 2. 第二阶段：腕部连杆闭环（进行中）

### 2.1 现象

RViz（`display.launch.py`）里拖 `joint_link7/9/7_0/9_0` 或 `joint_link5/6` 时连杆与电机
"散架"：电机绕杆末端甩出（拖 `joint_link7`=0.72 rad 时电机位移 ≈7 cm）。

### 2.2 三层原因（均已查实）

1. **闭环切口**（URDF 树固有限制）：物理上两台 X25S 电机**固定于小臂**——几何证据：
   `Joint_link10↔Joint_link4` 最小距离 0.13 mm、`Joint_link10_0↔Joint_link4` 0.01 mm
   （安装面贴合）；SW2URDF 只能导出树，切在"电机-小臂"这条边上，于是树里
   `…→6→7→9→10`：电机被挂到杆的末端。
2. **杆端铰类型错误**：真实杆端是**关节轴承（球铰）**——零件名 `GE43 向心关节轴承`、
   `Y-KCE06 杆端关节轴承`，用户口头确认"杆是二力杆"；导出件建为单轴 revolute。
   数值证据：单轴近似下杆端位置误差 **14–93 mm**、杆"离面"摆动角最大 **44°**。
3. **曲柄（摇臂）自由度缺失**：9–10 之间是 fixed，电机输出的曲柄转角没有被任何关节表达。

### 2.3 机构反演（本会话的分析）

用户描述 + 现场照片确认的机构链：
**电机端面法兰 → 短曲柄（摆臂）→ 球头 → 长杆（二力杆）→ 球头 → 腕部框体**。

几何反演（判据："杆长球面（心 P7、半径 L）∩ 曲柄圆（轴 a、过 C、半径 r）"有解）：

- 轴方向扫描（8 个腕位形测试集）：**竖直轴（装配位形下）+ 过电机 bbox 中心**得 8/8 可解
  （r≈17.7/17.5 mm）；"轴沿臂向 + 偏心轴心"网格扫描 ≤5/12；"沿 y" ≤6/12；
- 测试集扩到 12 位形后，"竖直轴过中心"也失效 → **单圆模型 + 单一位形的固有欠定**。

### 2.4 关键数值（零位，世界系）

| 量 | 分支 A（7/9/10） | 分支 B（7_0/9_0/10_0） |
| --- | --- | --- |
| 杆长 L = \|P9−P7\| | 101.81 mm | 67.05 mm |
| P7（杆—腕铰点） | (0.5000, −0.0076, −0.0049) | (0.4970, 0.0308, −0.0154) |
| P9（杆—曲柄铰点） | (0.3986, −0.0087, 0.0047) | (0.4309, 0.0275, −0.0045) |
| 电机 bbox 中心 | (0.3930, 0.0081, 0.0002) | (0.4281, 0.0108, 0.0000) |
| 曲柄半径 r（若轴=竖直过中心） | 17.7 mm | 17.5 mm |

**求解公式**（球铰模型）：圆参数化 `P9(φ) = O + r(û cosφ + ŵ sinφ)`（O = 轴上垂足，
û = (P9₀−O)/r，ŵ = a×û），约束 `|P9(φ) − P7| = L` 展开为
`A cosφ + B sinφ = M`（Rodrigues；A = d·u − (d·a)(a·u)，B = d·(a×u)，K = (d·a)(a·u)，
M = (|u|² + |d|² − L²)/2 − K，d = P7−C，u = P9₀−C），解析 2 解取连续支
（原型用扫描+二分，等价，见 `assets/closure_proto.py`）。

### 2.5 唯一卡点与解法

**卡点**：摇臂轴（方向、轴心、半径）**无法从单一位形几何唯一反演**（单圆 ∩ 球面只给出
一族可行解）。用户在 2026-10-09 已确认"杆是二力杆"，但对轴方向只给了定性描述
（"输出在侧边、对着杆"），仍不够。

**解法（任选其一，拿到即可实施）**：

1. 用户在 SolidWorks 中给出：**输出法兰圆面法线（轴方向）+ 曲柄回转中心 + 曲柄长度**；
2. 用户把腕关节各转 ~20° 后**再导出一份**（如 `zhoubuguanjie ROBOT-1/`）：
   两位形 = 标准机构标定，可唯一确定轴与曲柄（由 P9 的位移 + 旋转反解）。

### 2.6 待实施方案（建议）

- **方案 α（推荐先做，改动最小）**
  - **模型**：每分支在 `Joint_link6 → Joint_link7` 之间插入球铰 = 1 个零质量虚拟 link
    （`<inertial>` 1e-6 kg、无 visual/collision）+ 1 个与 v 轴正交的 revolute
    （origin 与 `joint_link7` 同点）；原 `joint_link7`（v 轴）保留为球铰第二轴。树结构不变。
  - **求解器**（新 `scripts/wrist_closure_solver.py`，原型 `assets/closure_proto.py`）：
    读主链关节 → 解"球 ∩ 圆"得 P9 目标 → 反解"球铰两角 + j9"使 `Joint_link9/10`
    的世界位姿匹配"固定于小臂"（**位置可精确、姿态残差待测**）。
  - **RViz 链路**：`joint_state_publisher_gui` → `/joint_states_raw`；solver 订阅 raw、
    发布修正后的 `/joint_states`；`robot_state_publisher` 订阅 `/joint_states`。
  - **Gazebo 链路**：被动关节加控制器（position 接口已声明；注意默认 0.1 比例增益
    **不够**，需高增益或 effort PD），solver 输出命令。
- **方案 β（更彻底，后续可选）**：另补"曲柄关节"（`Joint_link9` 从 7 下改挂 10 下，
  新增 revolute、轴=摇臂轴），求解器只驱动曲柄角与球铰角；树切点不变（仍在 4–10）。
- 两个方案都保持"10 个 revolute 关节"的既有契约；`test_model_consistency.py` 需同步
  接纳新增关节（契约已支持子序列，改动小）。
- **轴参数做成求解器配置**：先按"竖直过中心"上线，拿到 §2.5 信息后改配置即可。

---

## 3. 环境与流程（本会话实测踩坑）

### 3.1 Gazebo headless 冒烟（已验证可用的完整步骤）

```bash
# 终端 A：server（无 GUI）
source /opt/ros/humble/setup.bash && source ~/robot_ws/ros2_ws/install/setup.bash
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib
export IGN_GAZEBO_RESOURCE_PATH=~/robot_ws/ros2_ws/install/robot_arm_gazebo/share
ign gazebo -s -r empty.sdf

# 终端 B：spawn + 控制器
source /opt/ros/humble/setup.bash && source ~/robot_ws/ros2_ws/install/setup.bash
xacro ~/robot_ws/ros2_ws/src/robot_arm_gazebo/models/zhoubuguanjie_youbi_001/urdf/\
zhoubuguanjie_youbi_001.urdf.xacro \
  controllers_file:=$HOME/robot_ws/ros2_ws/install/robot_arm_gazebo/share/robot_arm_gazebo/models/zhoubuguanjie_youbi_001/config/controllers.yaml \
  fixed_base:=true gravity:=true > /tmp/spawn.urdf
ros2 run ros_gz_sim create -name zhoubuguanjie_youbi_001 -file /tmp/spawn.urdf -z 0.03
# 必须先起 robot_state_publisher（提供 robot_description 服务），否则 controller_manager 永不出现：
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(cat /tmp/spawn.urdf)" -p use_sim_time:=true
ros2 run controller_manager spawner joint_state_broadcaster --controller-manager-timeout 60
ros2 run controller_manager spawner arm_position_controller --controller-manager-timeout 60
ros2 topic echo /joint_states --once --field name
```

三个坑：

1. 不设 `IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib` →
   `Failed to load system plugin [gz_ros2_control-system]`（ign 不搜 LD_LIBRARY_PATH）；
2. `IGN_GAZEBO_RESOURCE_PATH` 必须是 `…/install/robot_arm_gazebo/share`
   （其下有 `robot_arm_gazebo/models/…` 供 `model://` 解析）——注意不是 `install`；
3. **gz_ros2_control 会等 `robot_state_publisher` 的 `robot_description` 服务**：
   不启动它，`/controller_manager` 永不出现（spawner 只会超时）。

另：无控制器时被动关节仍受 gz_ros2_control 默认 **0.1 位置比例增益**，实测数分钟内
`joint_link9_0` 漂移 ≈0.57 rad、其余 ≲0.1 rad（写进了模型 README）。

### 3.2 其它

- 常规构建/测试：`colcon build --symlink-install --packages-select robot_arm_gazebo`；
  `colcon test …`；结果查 `build/robot_arm_gazebo/test_results/`（`--test-result-base`）。
- `--symlink-install` 下改 `models/` 无需 rebuild；但 launch/test 读 install 路径，
  包外新增文件仍需 rebuild（本会话未涉及）。
- **本会话在用户机器上启动过的进程均已关闭**（ign gazebo、robot_state_publisher）；
  用户自有的 micro_ros_node / rosbridge / web_static_server 未动。

---

## 4. 复现资产（`docs/reports/2026-10-09-wrist-linkage-closure-assets/`）

| 文件 | 用途 |
| --- | --- |
| `migrate_model.py` | SW 导出件 → 模型库改写（第一阶段工具，重新导出后可重用） |
| `regen_derived.py` | standalone / dist 重生成（渲染 xacro → 剥插件 → 相对路径） |
| `urdf_geometry_probe.py` | FK + 网格包围盒探针（几何一致性复核） |
| `urdf_contact_graph.py` | 全链接触图（识别树边/缺失连接，闭环切口定位） |
| `closure_proto.py` | 闭环求解原型（球铰 + 曲柄圆；含轴心/轴方向评估） |
| `axis_center_scan2.py` | 摇臂轴标定扫描（竖直/横向轴心网格） |
| `wrist_detail.py` | 腕部局部渲染（给用户指认零件的标注图） |

均为**一次性分析工具**（只读、不入 colcon 安装）；脚本内路径按本机（`/home/ly/robot_ws`）
硬编码，换机需改。依赖：Python3 + numpy / scipy / matplotlib。

---

## 5. 事实速查（模型）

- **12 关节**：`joint_link1..6`（主链）+ `joint_link7/9/10` 与 `joint_link7_0/9_0/10_0`
  （腕部两分支；`joint_link10/10_0` 为 fixed）。
- **树**：`base_link → 1 → … → 6 → {7→9→10, 7_0→9_0→10_0}`。
- **驱动**：`controllers.yaml` 仅 `joint_link1..4`（4 台 CERB）；`simulation_topology.yaml`
  同（腕部关节无电机映射）。
- **闭环切点**：`Joint_link4 ↔ Joint_link10/10_0`（物理固定、树中缺失）。
- `spawn_z=0.03`；`gravity:=false` 关闭 12 个 link 的重力。
- 命令顺序（`Float64MultiArray`）= `joint_link1..4`；模型 README 有完整关节表与质心表。
