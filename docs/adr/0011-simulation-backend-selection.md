# 仿真后端可选（Gazebo / MuJoCo）与默认模型收敛到 half_body_robot

**Status**: accepted

> **包更名（2026-10-13）**：本 ADR 写就时包名为 `robot_arm_gazebo`，当日更名为 `robot_sim`
> （只改名，内部布局不变）。正文里的 `robot_arm_gazebo` 一律对应现在的 `robot_sim`。

> **模型更名（2026-10-13，同日稍后）**：本 ADR 的「默认模型收敛到 `full_body`」这条决定的
> **实质仍然有效**（四个 launch 的 `model:=` 默认值 + `web_host` 的 `MODEL` 常量收敛到同一个
> 模型），但该模型的**名字与内容变了**：CAD 重新导出后 `full_body` 更名为 `half_body_robot`
> （`models/full_body/` 与 `dist/full_body_urdf/` 已删除），并且 10 个关节的 `lower`/`upper`
> 收窄。正文与 Decision/Consequences 里的 `full_body` 一律对应现在的 `half_body_robot`。
> 细节见 `models/half_body_robot/README.md` 与 `docs/adr/README.md` 的索引行。

## Context

`web_host` 的「仿真」屏（`#/sim`）原来只能启动一条 Gazebo 回路：面板收参数 → `SimulationProcessManager`
拼出 `ros2 launch robot_sim simulation.launch.py transport:=… controller_mode:=… model:=…`
（`ros2_ws/src/web_host/web_host/simulation_manager.py:259`），而 `simulation.launch.py` 在
`launch_simulation()` 里**无条件** include `gazebo.launch.py` 并把 `control_mode:="effort"` 钉死。
MuJoCo 那条路已经存在，但只是一个**独立入口**：`launch/mujoco.launch.py`（rsp +
`scripts/mujoco_effort_bridge.py`），没有和电机仿真/驱动端联动，也没在面板里出现。

设计前核对过的事实（2026-10-13，本工作区；凡标「实测」的都有可复现命令）：

1. **两条路的被控对象接口同构**。Gazebo 路由 `gazebo.launch.py` 提供 `gz sim` + rsp
   + `/clock` 桥 + 模型生成 + controller spawner（`launch/gazebo.launch.py:74-135`），对外只有
   「吃 `/arm_effort_controller/commands`、出 `/joint_states`」；MuJoCo 路的桥自己步进物理、订阅
   同一 effort 话题、发布同一 `/joint_states`（`scripts/mujoco_effort_bridge.py:101,120-139`）。
   启动期接口并不完全等价：Gazebo 还要一份 `controllers_file`（由仿真入口生成并注入），
   MuJoCo 入口不接受它——所以后端分叉点在**参数集**上，不只是一个文件名。
2. **命令数组的顺序不是天然对齐的**。`motor_sim` 按**它自己配置里的电机顺序**发
   （`motor_sim/src/core/frame_router.cpp:179-185`、`motor_sim/src/ros/motor_sim_node.cpp:458-462`，
   `last_efforts_` 以 `config().motors` 下标索引），而 MuJoCo 桥原来按 **MJCF 关节下标**吃
   （`mujoco_effort_bridge.py:127-129`）。Gazebo 路免疫，是因为 `simulation.launch.py` 会把
   `arm_position_controller`/`arm_effort_controller` 的 `joints` 按拓扑顺序重写进临时
   `controllers.yaml`（`simulation.launch.py:249-266, 681-686`）；MuJoCo 路没有这一步，于是依赖
   「MJCF 顺序恰好等于拓扑顺序」这个**巧合**（full_body 上确实相等，但没有任何测试或校验强制它，
   且两边长度相同，错了也不会报错）。
3. **MuJoCo 路没有 `/clock`，但后果不是「TF 不发」**。`mujoco.launch.py` 的 rsp 用
   `use_sim_time:=true`，这条路没有任何 `/clock` 发布者。**实测**（robot_state_publisher 3.0.3）：
   rsp 把收到的 `/joint_states` 的 `header.stamp` 原样搬到 `/tf`，并不用自身（被 `use_sim_time`
   冻住的）时钟给 TF 打戳，所以缺 `/clock` 不会污染 TF 时间戳。真正的问题是**时间基不一致**：
   rsp 被设成仿真时间、桥用系统时间给 `/joint_states` 打戳，而 Gazebo 路两者都是仿真时间。今天
   没有别的 sim-time 消费者，所以暂时不爆；一旦有节点也开 `use_sim_time`，它会因为没有
   `/clock` 而冻在 0。
4. **桥里的 pyGLFW 变体修复无效（实测）**。`mujoco_effort_bridge.py` 曾在 `from mujoco import
   viewer` 之前设 `PYGLFW_LIBRARY_VARIANT=x11` 并清 `WAYLAND_DISPLAY`，但 `import mujoco`
   **本身就会先载入 glfw**（实测 import 后 `'glfw' in sys.modules` 为真、库版本串
   `3.4.0 Wayland …`），那段是空操作。Wayland 会话下窗口自旋：主线程 ~39% CPU、`sync()` 不返回、
   `/joint_states` 一帧不发。在**进程外**预设同样两个变量后实测 `glfw.get_platform()` 返回
   `0x60004`（X11），窗口与 `/joint_states` 均正常。
5. **`fdcan_motor_sim.py` 的命令顺序是 sorted，与拓扑顺序不等价**。
   `micro_ros_agent/tools/fdcan_motor_sim.py:436` 原本是
   `self.joints = sorted(set(joint_map.values()))`，并按该顺序发 effort（`:480-487`）。对
   `chest_joint1..9` 这类名字排序恰好等于声明顺序，但 full_body 的第二链关节名带点号，
   `sorted` 给出 `[link1, link1.1, link2, link2.2, …]`，与订阅端期望的
   `[link1..9, link1.1..9.9]` 不同 → **除首尾关节外力矩全部错位，且长度相同不报错**。
   这条与后端无关：`controller_mode:=firmware` + full_body 在 Gazebo 与 MuJoCo 两条路上都会错位。
6. **只有 `full_body` 有 MJCF**：`models/full_body/mujoco/full_body.xml` 由
   `models/full_body/tools/build_mujoco_model.py` 生成；`robot_all` 有仿真拓扑但既无 `mujoco/`
   也无生成器，它的 URDF 连 `<collision>` 都没有。
7. **默认模型是 `robot_all`，而它是唯一声明 position 命令接口的模型**。默认值散落在 4 个 launch、
   `test/test_launch_models.py:24`、`web/js/core/store.js` 的 `MODEL`、多份 README 与 `CLAUDE.md`；
   而 `micro_ros_agent/tools/bench_up.sh:14` 的默认档是 `position` 且调用 `gazebo.launch.py` 时
   **不传** `model:=`（即用 launch 默认模型）。
8. **`models/` 是整目录安装**（`CMakeLists.txt` 的 `install(DIRECTORY … models)`），删掉某个模型
   目录后 `install/` 里会留下上一次构建的陈旧副本，colcon 不会自动清——不清理的话面板仍会列出它。

## Decision

1. **后端是 `simulation.launch.py` 的一个参数**：新增 `backend:=gazebo|mujoco`（默认 `gazebo`），
   只决定被控对象入口 include 哪个文件。按后端分叉的只有三处：include 的文件、传给它的
   `launch_arguments`（Gazebo: `world_file`/`controllers_file`/`control_mode`/`gravity`；
   MuJoCo: `model`/`joint_names`）、以及 Gazebo 控制器配置的生成与关节来源校验
   （Gazebo 读 `controllers.yaml`，MuJoCo 解析 MJCF）。传输、控制端、`motor_sim`、effort 门控、
   驱动端接线、vcan 生命周期一律共用，**不进任何后端分支**。后端与控制端**正交**（三者组合都允许，
   接口同构；`firmware` 本身另有「必须 socketcan/slcan、不能 vcan」的既有约束）。
2. **MuJoCo 桥按关节名吃命令**：新增 `joint_names` 参数（逗号分隔的有序关节名），
   `simulation.launch.py` 传拓扑顺序的那份。映射逻辑抽到不依赖 mujoco/rclpy 的
   `scripts/mujoco_command_mapping.py`（pytest 跑在系统 python3，import 不到 venv 里的 mujoco）。
   留空时保持旧语义（顺序 = MJCF 关节顺序、长度 = 关节数），手工 `ros2 topic pub` 的用法不变。
   名字缺失/重复/长度不符一律拒绝整帧并 warning，不静默少发。
3. **MuJoCo 路补 `/clock`**：桥按物理时间 `data.time` 发布 `rosgraph_msgs/Clock`
   （桥自身不设 `use_sim_time`，`/joint_states` 时间戳仍取系统时钟）。目的是让两条路的时间基
   一致、且将来开 `use_sim_time` 的消费者不会冻在 0——**不是**为了修 TF（见 Context 3）。
4. **pyGLFW 变体改在进程外注入**：`mujoco.launch.py` 给桥进程传一份清理过的 `env`
   （Wayland 会话且有 `DISPLAY` 时置 `PYGLFW_LIBRARY_VARIANT=x11` 并**删除** `WAYLAND_DISPLAY`；
   `additional_env` 无法表达「删除」，所以传整份 `env`）。桥启动时打印实际平台，便于核对。
5. **viewer 一律开**：`backend:=mujoco` 不传 `headless`。窗口出现在 web_host 所在机器的显示器上，
   因此「从面板起 MuJoCo」隐含前提是 web_host 从有 `DISPLAY` 的会话启动；无显示时桥自己降级为
   无头并打 warning（既有行为，不改）。
6. **默认模型收敛到 `full_body`，删除 `models/robot_all/`**：4 个 launch 的默认值、
   `test_launch_models.py`、`store.js` 的 `MODEL`、`CLAUDE.md` / `AGENTS.md` /
   各 README 同步改，并清理 `install/` 里的陈旧副本。删除依据：它的全部引用都是注释或默认值，
   `full_body` 只 include 自己的 URDF，`models/full_body/meshes/` 的硬链接指向的是
   `dist/full_body_urdf/`（实测 19/19 同 inode）而不是 `models/robot_all/`。
   `dist/robot_all_urdf/`（70 MB 发布件）当时保留不动；**2026-10-13 稍后按用户指示删除**。
   ⚠ **更正**：本条与当日 spec/plan 都写它「未入 git，删了不可恢复」——**这是错的**。它的
   20 个文件全在 git HEAD 里（旧路径 `ros2_ws/src/robot_arm_gazebo/dist/robot_all_urdf/`），
   19 个网格与当时工作区逐字相同，只有 `robot_all.urdf` 差一行注释里的包名
   （`robot_arm_gazebo` → `robot_sim`），即**删除是可恢复的**。取回方式：
   `git show HEAD:ros2_ws/src/robot_arm_gazebo/dist/robot_all_urdf/robot_all.urdf`。
7. **`micro_ros_agent/tools/bench_up.sh` 的默认档从 `position` 改为 `effort`**：删掉 `robot_all`
   后仓库里没有带 position 命令接口的模型，`control_mode:=position` 会在 spawn
   `arm_position_controller` 处失败（full_body 只导出 effort，理由见该模型 README）。
   脚本保留 `position` 选项并新增可选的第二个参数指定模型。这条变更**扩大了本 ADR 的范围**
   到 `micro_ros_agent/tools/`，原因是「收敛默认模型」这一决定直接打断了台架脚本的默认档。
8. **修 `fdcan_motor_sim.py` 的命令顺序**：改为保留 `--joint-map` 的声明顺序并去重
   （`list(dict.fromkeys(...))`）。对文档里的台架用法行为不变（`joint_link1..4` 的声明序等于排序序），
   对 `firmware` + full_body 是修好错位。同样属于「本决定的连带修复」，不是独立特性。
9. **web 面板加「仿真后端」下拉**：标记写在 `index.html` 的 sim 面板模板（DOM 单一来源，JS 不生成
   id），`sim.js` 读写并随既有 localStorage 配置持久化、提交进请求体、按后端过滤模型下拉、
   运行中与其他控件一起禁用；`simulation_manager.py` 校验 `backend` 取值、校验「模型有
   MJCF 且**有 standalone URDF**」、拼 `backend:=` 参数、把 `backend` 纳入 config 幂等比较。
   `/api/simulation/options` 保持 `models` 为字符串数组，**新增** `mujoco_models` 键。
10. **测试**：`SimulationProcessManager` 此前零覆盖，补 pytest（后端取值、MuJoCo 能力校验、
    命令行拼接、config 幂等与「已在跑」冲突）；桥的映射补 pytest；`test_launch_models.py` 的
    `MODEL` 改 `full_body` 并加「默认模型有 MJCF」断言；`test_model_consistency.py` 加
    「MJCF 的关节集/actuator 顺序 == URDF 可控关节顺序」和「MJCF 网格能解析」断言；
    `web_host/test/conftest.py` 加「测试不得真起进程」守卫。全部测试不启动 Gazebo、
    不开 MuJoCo 窗口、不占 vcan/串口。

## Consequences

- 一个入口、一个面板覆盖两个后端；再加第三个后端只需再加一个 include 分支与一份能力声明。
- **两个后端不会逐位一致**：MuJoCo 侧是简化近似（碰撞体沿用 Gazebo 的简化形状、关节限位是软约束且
  实测越界 ~4e-3 rad、`frictionloss` 为 0），验收标准是**接口与闭环行为对等**，不是轨迹逐点相同。
- `backend:=mujoco` + `controller_mode:=firmware` 组合未被实机验证（需要 CANable + STM32 台架）；
  代码层面它与 host/external 走同一套接线，但**只按代码审阅对待**，不算已验证组合。
- **`/clock` 与时间基**：桥发 `/clock`（仿真时间）、`/joint_states` 用系统时间戳。Gazebo 路两者都是
  仿真时间。今天没有 sim-time 消费者时无影响；`motor_sim` 只写不读 `header.stamp`（
  `motor_sim_node.cpp:532`），所以反馈回环不受影响。
- **viewer 在无显示的机器上静默降级无头**，面板仍显示 `running`；「窗口正常」这条验收只在有
  `DISPLAY` 的机器上成立。
- **与 ADR 0009（重力前馈）的关系**：0009 把前馈叠加点定死在 Gazebo 的
  `/arm_effort_controller/commands` 话题上，而该 ADR 至今未落地（全仓无 `gravity_feedforward_mixer.py`
  /`gravity_torque_node`）。本 ADR 不改变 0009 的边界，但**声明其边界的缺口**：实现 0009 时若要
  覆盖 MuJoCo 后端，需要重新确认叠加点是否仍 Gazebo 专属（两后端共用同一话题，因此大概率仍然成立），
  并另立 ADR 或修订 0009。`gravity:=false` 在 MuJoCo 后端被显式拒绝（MJCF 重力生成期固定）。
- 删掉 `models/robot_all/` 后，本模型（当时的 `full_body`）的多处注释失去被指对象，已改写为
  本模型自身的理由；该模型仍在 git 历史里可回滚。`robot_all` 与本模型一样也留有历史记录。
- 真机路径、`motor_drive` / `motor_sim` / `motor_hardware_interface` / 固件一律不动；
  仅 `micro_ros_agent/tools/` 的上述两处（属连带修复，见 Decision 7–8）。
