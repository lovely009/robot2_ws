# 仿真后端选择（Gazebo / MuJoCo）设计

- 日期：2026-10-13
- 状态：已实施，待端到端验收（2026-10-13）
- 范围：`simulation.launch.py` 增后端参数、MuJoCo 桥补齐契约、web 面板加后端选择、默认模型收敛到 `full_body`、删除 `models/robot_all`
- 关联：`docs/adr/0011-simulation-backend-selection.md`

## 背景

`web_host` 的「仿真」屏只能起 Gazebo 回路（`simulation_manager.py` 拼出
`ros2 launch robot_arm_gazebo simulation.launch.py transport:=… controller_mode:=… model:=…`），
而 MuJoCo 只有独立入口 `mujoco.launch.py`：它不接 `motor_sim`、不接驱动端、也不在面板里。

两条路的被控对象接口其实同构：都是「订阅 `/arm_effort_controller/commands`
（`Float64MultiArray`）、发布 `/joint_states`」。Gazebo 侧由 `gazebo.launch.py` 里的
`gz sim` + `gz_ros2_control` 提供，MuJoCo 侧由 `mujoco_effort_bridge.py` 自己步进物理后提供。
差异集中在三处：命令数组的**映射方式**、`/clock` 的有无、以及 pyGLFW 后端选择（Wayland 自旋）。

## 决策

### 1. 一个入口、一个参数：`backend:=gazebo|mujoco`

`simulation.launch.py` 新增 `backend` 参数（默认 `gazebo`），它只决定被控对象入口 include 哪一个：

| backend | 被控对象入口 | 说明 |
|---|---|---|
| `gazebo`（默认） | `include gazebo.launch.py`（`control_mode:="effort"`） | 现状不变 |
| `mujoco` | `include mujoco.launch.py`（`model:=<model>`） | 复用现有入口，不复制 |

不变的接线（两个后端共用）：`_load_simulation()` / `_bind_motor_topology()` 的拓扑校验与生成、
`_write_runtime_configs()`（`motor_sim.yaml` + 驱动端 `motor_driver.yaml`）、`motor_sim` 节点、
`simulation_gate.py`、`controller_mode:=host` 时的 `hardware.launch.py` 驱动端、
`controller_mode:=firmware` 时的 `fdcan_motor_sim.py`、vcan 创建/清理、runtime 目录清理、
关键进程退出时的整体 shutdown。

后端相关的差异只有两处实现分支：

- `backend:=gazebo`：继续调 `_write_gazebo_controller_config()` 生成该 backends 的控制器参数文件，
  并用 `_model_joint_names()`（读 `controllers.yaml`）校验拓扑关节是模型关节的子集。
- `backend:=mujoco`：不生成 Gazebo 控制器配置，改为**解析 MJCF 的关节名**做同一件事——
  既校验拓扑关节都在 MJCF 里，也把它作为 `joint_names` 传给桥（见 2）。启动前失败要报出
  「哪个关节不在 MJCF 里」和「MJCF 路径」，而不是等桥在运行期静默丢命令。

### 2. 桥按关节名吃命令：`joint_names` 参数

`mujoco_effort_bridge.py` 新增参数 `joint_names`（有序字符串列表，默认空）：

- **给了**：命令数组按该列表的下标语义解释，逐个按**关节名**查到 actuator 写 `data.ctrl`；
  长度不符即拒绝并 warning（沿用现有「长度不对就忽略」的语义）。不在列表里的 actuator 一律写 0。
- **留空**：保持现有语义（数组顺序 = MJCF 关节顺序，长度 = MJCF 关节数），
  README 里手工 `ros2 topic pub` 的 18 个数用法不受影响。

映射逻辑抽到独立纯模块（不 import `mujoco` / `rclpy`），使 pytest 能在系统 python3 下直接单测
——pytest 环境里没有 venv 的 mujoco，桥脚本本身无法被 import。

### 3. MuJoCo 路补 `/clock`

桥按物理时间 `data.time` 发布 `rosgraph_msgs/Clock`（桥自身**不设** `use_sim_time`，
`/joint_states` 的时间戳仍用系统时钟）。

⚠ 理由已按实测更正：**不是为了修 TF**。实测 robot_state_publisher 3.0.3 把收到的
`/joint_states` 的 `header.stamp` 原样搬到 `/tf`，并不用自身（被 `use_sim_time` 冻住的）时钟，
所以缺 `/clock` 不会让 TF 时间戳坏掉。补它的目的是**两条路的时间基一致**（Gazebo 路 rsp 与
joint_states 都是仿真时间），以及将来若有节点开 `use_sim_time` 不会因为没有 `/clock` 冻在 0。

### 3b. 连带修复：`fdcan_motor_sim.py` 的命令顺序

`micro_ros_agent/tools/fdcan_motor_sim.py` 原来用 `sorted(set(joint_map.values()))` 决定命令数组
顺序，而订阅端按拓扑顺序解释。对 `chest_joint1..9` 这类名字排序恰好等于声明顺序，但 full_body
第二链的名字带点号（`joint_link1.1..9.9`）时字典序是 `[link1, link1.1, link2, …]` → 除首尾关节外
力矩全部错位，且长度相同不报错。改为保留 `--joint-map` 的声明顺序并去重。

这条**与后端无关**：`controller_mode:=firmware` + full_body 在 Gazebo 与 MuJoCo 两条路上都会错位，
所以不修就无法声称「两个后端都能正常使用」。同时 `bench_up.sh` 的默认档从 `position` 改为 `effort`
（`robot_all` 删除后没有带 position 命令接口的模型），并新增可选第二参数指定模型。

### 3c. 启动期校验补齐：standalone URDF

`mujoco.launch.py` 的 robot_state_publisher 用 `models/<模型>/urdf/<模型>.standalone.urdf` 渲染
`robot_description`。只校验 MJCF 会让「缺 standalone」的组合通过校验、然后在 `cat` 处静默得到空
`robot_description`（rsp 起来了但模型是空的）。因此 launch 与 web 的能力探测都要求
`mujoco/<模型名>.xml` **与** `urdf/<模型名>.standalone.urdf` 同时存在。

### 4. pyGLFW 变体改在进程外注入

`mujoco.launch.py` 给桥进程传 `additional_env`：当 `XDG_SESSION_TYPE == wayland` 且有 `DISPLAY`
时，置 `PYGLFW_LIBRARY_VARIANT=x11` 并**删除** `WAYLAND_DISPLAY`。脚本里那段同名设置（写在
`from mujoco import viewer` 之前、但在 `import mujoco` 之后）**是空操作，删除**。

### 5. viewer 一律开

`backend:=mujoco` 不传 `headless`（即 `mujoco.launch.py` 的默认 `false`），窗口出现在 web_host
所在机器上。无本地显示时桥自己 catch 异常降级无头并打 warning（既有行为，不改）。
`mujoco.launch.py` 的 `headless` 参数保留，供手工启动使用。

### 6. 默认模型收敛到 `full_body`，删除 `models/robot_all/`

- 4 个 launch 的 `default_value`、`test/test_launch_models.py` 的 `MODEL`、`web/js/core/store.js`
  的 `export const MODEL`、`CLAUDE.md`、`robot_arm_gazebo/README.md`、`web_host/README.md`
  一并改成 `full_body`。
- 删除 `models/robot_all/`（已核实：`full_body` 只 include 自己的 `full_body.urdf`；`robot_all`
  的其它引用全是注释或默认值）。`install/` 里的陈旧副本一并清理，否则面板仍会列出它。
- `dist/robot_all_urdf/`（未入 git，70 MB）**保留**，是否删除由用户决定——它未入版本库，
  删了不可恢复。

### 7. web：面板加后端下拉，模型按后端能力过滤

- `index.html` 的 sim 面板模板里加 `id="simBackend"`（选项 `gazebo` / `mujoco`）；DOM 单一来源，
  JS 不生成 id（ADR 0010 的契约测试会挡）。
- `sim.js`：读取/写入该值，纳入既有的 localStorage 配置持久化与「配置相同则不重复启动」的判断；
  提交时放进请求体。
- `simulation_manager.py`：校验 `backend ∈ {gazebo, mujoco}`；`backend == mujoco` 时要求
  `models/<model>/mujoco/<model>.xml` 存在，否则报「该模型没有 MuJoCo 模型」；拼 `backend:=<值>`；
  `backend` 进 config（状态展示与幂等比较都用）。
- `/api/simulation/options` **保持** `models` 为字符串数组，**新增** `mujoco_models`（有 MJCF 的
  模型名列表）。前端在 `backend == mujoco` 时用它过滤下拉，并给出「该模型只支持 Gazebo」的提示。
  不改既有键的形态，避免牵连现有资产契约测试。

## 文件与边界

新增：

- `ros2_ws/src/robot_arm_gazebo/scripts/mujoco_command_mapping.py`：命令数组 → `(关节名, 值)` 的
  纯映射（无 mujoco/rclpy 依赖）。
- `ros2_ws/src/robot_arm_gazebo/test/test_mujoco_command_mapping.py`：上面的单测。
- `ros2_ws/src/web_host/test/test_simulation_manager.py`：`SimulationProcessManager` 的单测
  （现状是零覆盖）。
- `ros2_ws/src/web_host/test/conftest.py`：守卫——web_host 的单测不得真起子进程。
- `ros2_ws/src/robot_arm_gazebo/test/test_mujoco_command_mapping.py`：纯映射单测。
- 本 spec / plan 与 `docs/adr/0011-…`。

修改：

- `launch/simulation.launch.py`：`backend` 参数、被控对象入口分支、MJCF 校验与 `joint_names` 透传。
- `launch/mujoco.launch.py`：`additional_env`（pyGLFW 变体）、`joint_names` 参数透传、默认模型。
- `launch/gazebo.launch.py`、`launch/display.launch.py`：默认模型、注释。
- `scripts/mujoco_effort_bridge.py`：按名映射、发布 `/clock`、删除无效的环境变量块、`joint_names` 参数。
- `CMakeLists.txt`（robot_arm_gazebo）：安装新的 `scripts/mujoco_command_mapping.py`（`install(PROGRAMS)`）。
- `test/test_launch_models.py`：`MODEL` 改 `full_body`，加「默认模型有 `mujoco/<model>.xml`」断言。
- `web/index.html`、`web/js/panels/sim.js`、`web_host/simulation_manager.py`：后端选择。
- `web/js/core/store.js`：`MODEL` 默认值。
- `CLAUDE.md`、`robot_arm_gazebo/README.md`、`web_host/README.md`、`models/full_body/README.md`、
  `docs/adr/README.md`（索引行）：文档同步。
- `models/full_body/` 下若干注释：去掉对将被删除的 `robot_all` 的指代，改写成本模型自身的理由
  （只改注释语义，不动参数）。

删除：

- `models/robot_all/`（整目录，git 跟踪）+ `install/` 里的陈旧副本。`dist/robot_all_urdf/` 不动。

跨区域改（连带修复，见 3b）：

- `micro_ros_agent/tools/fdcan_motor_sim.py`：命令数组顺序改为保留声明顺序。
- `micro_ros_agent/tools/bench_up.sh`：默认档 `position` → `effort`，新增可选模型参数。

明确不做：

- 不为 `robot_all` 或其它模型生成 MJCF；不引入第二个 MJCF 生成器。
- 不改 `motor_sim` / `motor_hardware_interface` / `motor_drive` / 固件 / `micro_ros_agent`。
- 不做 web 端「MuJoCo 画面回传」（浏览器里看不到 MuJoCo 渲染；画面在服务器显示器上）。
- 不追求两后端的数值一致性（MuJoCo 侧是简化近似，见 `models/full_body/README.md`）。
- 不为「web 页面的臂视图在仿真后端下显示关节角」加桥（页面现在读固件话题
  `micro_ros_up/motion_states`，Gazebo 后端下同样没数据；这是既有边界，不在本次范围）。

## 验收标准

1. `colcon build --symlink-install` 全绿；`colcon test` 全绿（含新增用例），且不启动 Gazebo、
   不开 MuJoCo 窗口、不占 vcan/串口。
2. 面板选 `gazebo`：行为与改动前一致（回归），`/api/simulation/start` 的启动参数与日志不变形。
3. 面板选 `mujoco` + `vcan` + `host`：MuJoCo 窗口正常（非 Wayland 自旋），`/joint_states` ~100 Hz，
   `/clock` 有输出，effort 命令经 `motor_sim` 门控后按**关节名**落到正确关节。
4. 面板选 `mujoco` 且模型无 MJCF：后端返回明确错误，不启动进程。
5. `grep -rn robot_all` 在 **`ros2_ws/src/` 与 `micro_ros_agent/` 下**只剩：`dist/`（未入 git 的
   发布件）、以及明确写「已删除 / 历史件」的说明性文字。`docs/` 下的 ADR/spec/plan 里保留
   `robot_all` 是**有意**的（它们记录这次删除的来龙去脉），不参与该判据。
