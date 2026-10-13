# 仿真后端选择（Gazebo / MuJoCo）实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让 `web_host` 的仿真面板能在 Gazebo 与 MuJoCo 两个后端之间选择，两者都能正常起停并跑通
「position 控制器 → 驱动端 → CAN → `motor_sim` → effort → 被控对象 → `/joint_states`」这条闭环；
默认模型收敛到 `full_body`，删除 `models/robot_all`。

**Architecture:** `simulation.launch.py` 新增 `backend` 参数，只在「被控对象」这一层分叉
（include `gazebo.launch.py` 或 `mujoco.launch.py`），其余接线（拓扑校验、运行期 YAML、`motor_sim`、
effort 门控、驱动端、vcan 生命周期）完全共用。MuJoCo 桥补齐两处契约：按**关节名**吃命令数组、
发布 `/clock`；pyGLFW 变体改由 launch 以 `additional_env` 在进程外注入。

**Tech Stack:** ROS 2 Humble / Python 3 launch、rclpy、MuJoCo 3.6（`~/.ros/ros2_control/.venv`）、
标准库 `ThreadingHTTPServer`、原生 ES module（无构建链）、pytest（静态契约与单测）。

**Spec:** `docs/superpowers/specs/2026-10-13-simulation-backend-selection-design.md`
**ADR:** `docs/adr/0011-simulation-backend-selection.md`

## Global Constraints

- **测试不得启动 Gazebo、不得开 MuJoCo 窗口、不得占用 vcan 或串口、不得连真实硬件。**
  `SimulationProcessManager` 的测试一律注入 `popen_factory` 假体，不产生真实子进程。
- **后端只替换被控对象入口**：`motor_sim`、`simulation_gate.py`、`fdcan_motor_sim.py`、
  `hardware.launch.py`、运行期 YAML 生成不得出现 `backend` 分支。
- **兼容既有手工用法**：`mujoco.launch.py` 直接起（不传 `joint_names`）时，命令数组语义保持
  现状（顺序 = MJCF 关节顺序、长度 = MJCF 关节数）。
- **DOM 单一来源**（ADR 0010）：新下拉写在 `index.html` 的面板模板里；JS 不得生成带 `id` 的元素。
- **`web/` 下新增或改名任何文件后必须重跑 `colcon build --packages-select web_host`**，
  `--symlink-install` 不兜 `setup.py` 清单与新增静态文件。
- **默认模型只认 `full_body`**：改完后 `grep -rn robot_all` 不得再命中任何会被执行的代码路径。
- 代码规范：Python snake_case、测试带类型注解、中文注释写意图；提交用 conventional 前缀并写原因。
- 不得覆盖、暂存或回滚工作区里已有的用户改动（当前 `scripts/mujoco_effort_bridge.py` 有未提交改动，
  本次会在其基础上继续修改）。

---

### Task 1: 命令数组 → 关节的纯映射模块

**Files:**
- Create: `ros2_ws/src/robot_arm_gazebo/scripts/mujoco_command_mapping.py`
- Create: `ros2_ws/src/robot_arm_gazebo/test/test_mujoco_command_mapping.py`
- Modify: `ros2_ws/src/robot_arm_gazebo/CMakeLists.txt`（`install(PROGRAMS …)` 加新脚本）

**Interfaces:**
- Produces: `resolve_command_indices(joint_names: list[str] | None, actuator_joint_names: list[str]) -> list[tuple[int, int]] | None`
  —— 返回 `[(命令下标, actuator 下标)]`；`joint_names` 为空时返回 `None`（表示「按 MJCF 顺序、等长」）。
  以及 `mapping_error(joint_names, actuator_joint_names, command_length) -> str | None`：长度不符或
  名字对不上时给出人话错误，否则 `None`。
- Consumes: 无（不 import `mujoco` / `rclpy`，pytest 直接 import）。

- [ ] **Step 1: 先写测试**（并注册进 `ament_add_pytest_test`，否则不进 `colcon test`）：覆盖 ①空 `joint_names` + 长度相等 → `None`；②空 + 长度不等 → 错误；
  ③有序名字全命中 → 正确 `(命令下标, actuator 下标)` 对，且**乱序**时按名字对齐（不是按下标）；
  ④名字不在 MJCF 里 → 错误信息含该名字；⑤`joint_names` 有重复 → 错误。
- [ ] **Step 2: 实现**：纯函数，无 I/O。
- [ ] **Step 3: 验证**：`colcon build --packages-select robot_arm_gazebo && colcon test --packages-select robot_arm_gazebo --pytest-args "-k mujoco_command_mapping"`

### Task 2: 桥补齐契约（按名映射 / `/clock` / pyGLFW 变体）

**Files:**
- Modify: `ros2_ws/src/robot_arm_gazebo/scripts/mujoco_effort_bridge.py`

**Interfaces:**
- Consumes: Task 1 的纯映射。
- Produces: 新参数 `joint_names`（默认 `[]`）；新发布者 `rosgraph_msgs/Clock`（话题 `/clock`，
  值取 `data.time`）；`use_sim_time` 保持不设。

- [ ] **Step 1:** 用 Task 1 的函数替换现有的「按关节下标写 `ctrl`」逻辑；长度不符/名字缺失走
      现成的 warning + 忽略语义（`throttle_duration_sec` 保留）。
- [ ] **Step 2:** 每拍发布 `/clock`（在 `mj_step` 之后、用 `data.time`）。
- [ ] **Step 3:** 删除 `__init__` 里那段无效的 `PYGLFW_LIBRARY_VARIANT` / `WAYLAND_DISPLAY` 设置，
      并留一行注释说明「改由 `mujoco.launch.py` 的 `additional_env` 在进程外注入（脚本内太晚：
      `import mujoco` 已先载入 glfw）」。
- [ ] **Step 4:** 验证：`python3 -m py_compile` + 手工起一次
      `ros2 launch robot_arm_gazebo mujoco.launch.py model:=full_body`，
      确认 `/joint_states` 与 `/clock` 都有输出、无 GLFW Wayland 警告。

### Task 3: `mujoco.launch.py` 注入环境与参数

**Files:**
- Modify: `ros2_ws/src/robot_arm_gazebo/launch/mujoco.launch.py`

- [ ] **Step 1:** 给桥的 `ExecuteProcess` 加 `additional_env`：`XDG_SESSION_TYPE == "wayland"`
      且有 `DISPLAY` 时传 `PYGLFW_LIBRARY_VARIANT=x11`；`WAYLAND_DISPLAY` 需要「删除」而不是置空，
      实现上用 `additional_env` 无法删除，改为在 launch 里用 `SetEnvironmentVariable` 对该进程组
      生效或在 `_launch_setup` 里构造一份清理过的 `env` 字典传给 `ExecuteProcess(env=…)`。
- [ ] **Step 2:** 新增 `joint_names` launch 参数（逗号分隔字符串，默认空）并透传成桥参数列表。
- [ ] **Step 3:** 默认模型改 `full_body`，更新 docstring（控制方式示例改成 18 个数）。
- [ ] **Step 4:** 验证：`colcon build --packages-select robot_arm_gazebo`；
      `ros2 launch robot_arm_gazebo mujoco.launch.py model:=full_body` 看窗口与话题。

### Task 4: `simulation.launch.py` 的 `backend` 参数

**Files:**
- Modify: `ros2_ws/src/robot_arm_gazebo/launch/simulation.launch.py`

**Interfaces:**
- Produces: `backend` launch 参数（`gazebo` | `mujoco`，默认 `gazebo`）；`backend:=mujoco` 时
  向 `mujoco.launch.py` 传 `model` 与 `joint_names`（拓扑顺序的关节名，逗号分隔）。

- [ ] **Step 1:** 校验 `backend` 取值；取值非法即 fail fast 并说明可选值。
- [ ] **Step 2:** `backend == "mujoco"` 时：校验 `models/<model>/mujoco/<model>.xml` 存在、
      解析其关节名、校验拓扑关节 ⊆ MJCF 关节（错误信息点名缺失关节与 MJCF 路径）；
      用 `IncludeLaunchDescription` 引入 `mujoco.launch.py` 替代 `gazebo.launch.py`；
      **不**调 `_write_gazebo_controller_config()`。
- [ ] **Step 3:** `backend == "gazebo"` 走现状（含 Gazebo 控制器配置生成与 `controllers.yaml` 校验）。
- [ ] **Step 4:** 更新 docstring：新增后端说明与两后端的接口契约。
- [ ] **Step 5:** 验证：静态加载两个后端各一次（`OpaqueFunction` 需要真实 context，用现有
      `test_launch_models.py` 的加载方式核对参数声明），再各手工起一次。

### Task 5: `SimulationProcessManager` 单测 + 实现

**Files:**
- Create: `ros2_ws/src/web_host/test/test_simulation_manager.py`
- Modify: `ros2_ws/src/web_host/web_host/simulation_manager.py`

**Interfaces:**
- Produces: `options()` 新增 `mujoco_models: list[str]`；`_validate_and_build()` 接受
  `backend`（必填，`gazebo` | `mujoco`），`mujoco` 时校验模型有 MJCF；`start()` 的命令行末尾带
  `backend:=<值>`；`config` 含 `backend`（幂等比较与状态展示）。

- [ ] **Step 1: 先写测试**（注入 `popen_factory`、`can_provider`、`serial_provider`、`model_provider`）：
      ①`backend` 缺失/非法 → `ValueError`；②`mujoco` + 无 MJCF 的模型 → `ValueError` 且**未** spawn；
      ③`gazebo` → 命令行含 `backend:=gazebo`，参数顺序稳定；④`mujoco` → 命令行含 `backend:=mujoco`；
      ⑤同一 config 重复 `start()` 返回原状态、不重新 spawn；不同 `backend` → `RuntimeError`；
      ⑥`options()['mujoco_models']` 只列有 MJCF 的模型。
      模型能力探测用可注入的 provider（不要在测试里读真实 `share/`）。
- [ ] **Step 2:** 实现（含 `enumerate_mujoco_models()`，与 `enumerate_simulation_models()` 同风格）。
- [ ] **Step 3:** 验证：`colcon build --packages-select web_host && colcon test --packages-select web_host`

### Task 6: web 面板的后端选择

**Files:**
- Modify: `ros2_ws/src/web_host/web/index.html`、`web/js/panels/sim.js`、`web/js/core/store.js`
- Test: `ros2_ws/src/web_host/test/test_web_assets.py`（既有契约测试必须继续绿）

- [ ] **Step 1:** `index.html` 的 sim 面板模板里加「后端」`<select id="simBackend">`（两项），
      位置放在「控制端」之前；**不**在屏模板里加 id。
- [ ] **Step 2:** `sim.js`：纳入 `ui` 映射与配置持久化、随请求体提交、按 `deviceOptions.mujoco_models`
      在 `backend == mujoco` 时过滤模型下拉；模型不支持该后端时给出提示而不是发请求。
- [ ] **Step 3:** `store.js` 的 `MODEL` 默认值改 `full_body`（注释同步）。
- [ ] **Step 4:** **重跑** `colcon build --symlink-install --packages-select web_host`（脚本与 `setup.py`
      清单之外的改动靠 symlink 兜住，但重跑才能保证安装目录与源码一致），再
      `colcon test --packages-select web_host`，确认 DOM/资产契约测试绿。

### Task 6b: 连带修复（跨区域）

**Files:**
- Modify: `micro_ros_agent/tools/fdcan_motor_sim.py`、`micro_ros_agent/tools/bench_up.sh`

- [ ] **Step 1:** `fdcan_motor_sim.py` 的 `self.joints = sorted(set(...))` 改为保留 `--joint-map`
      声明顺序并去重（`list(dict.fromkeys(...))`），注释写清为什么（拓扑序 vs 字典序、点号关节名）。
- [ ] **Step 2:** `bench_up.sh` 默认档 `position` → `effort`，注释说明理由（唯一带 position 接口的
      `robot_all` 已删），并新增可选第二参数 `模型名`。
- [ ] **Step 3:** 验证：`bash -n bench_up.sh`；命令行拼接在「给模型 / 不给模型」两种情况下都打印
      期望结果；`python3 -m py_compile fdcan_motor_sim.py`。
      firmware + full_body 的实测需要 CANable + STM32 台架，本轮**不实跑**，按代码审阅对待。

### Task 7: 默认模型收敛与删除 `models/robot_all`

**Files:**
- Modify: `launch/display.launch.py`、`launch/gazebo.launch.py`、`test/test_launch_models.py`、
  `CLAUDE.md`、`robot_arm_gazebo/README.md`、`web_host/README.md`、`models/full_body/**`（注释）
- Delete: `ros2_ws/src/robot_arm_gazebo/models/robot_all/`（整目录）
- 保留：`ros2_ws/src/robot_arm_gazebo/dist/robot_all_urdf/`（未入 git，删了不可恢复）

- [ ] **Step 1:** 4 个 launch 的默认模型 + `MODEL` 常量 + `store.js` 改 `full_body`；
      `test_launch_models.py` 增加「默认模型存在 `mujoco/<model>.xml`」断言（`mujoco.launch.py`
      归属这条链）。
- [ ] **Step 2:** 删除 `models/robot_all/`，并清理 `install/robot_arm_gazebo/share/robot_arm_gazebo/models/robot_all`
      这个上一次构建留下的陈旧副本。
- [ ] **Step 3:** 改写 `models/full_body/` 里失去被指对象的注释（`urdf/full_body.urdf`、
      `urdf/full_body.urdf.xacro`、`urdf/full_body.canfd.xacro`、`config/*.yaml`、
      `README.md`）——只说本模型自己的理由，不再引用已删除的模型名。
- [ ] **Step 4:** 验证：`colcon build --symlink-install` 全仓 + `colcon test` 全绿；
      `grep -rn "robot_all" --include=*.py --include=*.js --include=*.html --include=*.yaml --include=*.xacro --include=*.urdf ros2_ws/src/`
      只剩 `dist/` 未跟踪件。

### Task 8: 文档与 ADR 状态

**Files:**
- Modify: `CLAUDE.md`（仿真 bring-up 段落加 `backend:=`）、`robot_arm_gazebo/README.md`（新增后端一节）、
  `web_host/README.md`（仿真屏加后端选择说明）、`models/full_body/README.md`、
  `docs/adr/README.md`（索引加 0011）、`docs/adr/0011-…`（`Status` 改 `accepted`）

- [ ] **Step 1:** 按实际实现回填文档（不写未验证的行为）。
- [ ] **Step 2:** 验证：文档里给出的每条命令至少手工跑过一次；`web_host/README.md` 的截图/字段名与
      面板实际 id 一致。

### Task 9: 端到端验收

- [ ] **Step 1:** 面板 + `gazebo` + `vcan` + `host`：起停正常，日志无回归。
- [ ] **Step 2:** 面板 + `mujoco` + `vcan` + `host`：桥日志打印 `platform=0x60004`（X11）且窗口可用、
      `/joint_states` ~100 Hz、`/clock` 有输出、给某个**末端**关节发单点 effort 时只有该关节动
      （验证按名映射而不是按顺序碰巧对上）。
- [ ] **Step 3:** 面板 + `mujoco` + 无 MJCF 的模型：被拒绝且没有子进程。
- [ ] **Step 4:** 从面板停止 `mujoco`：进程组干净退出、窗口关闭、无残留 `ros2 launch`。
- [ ] **Step 5:** `colcon test` 全量 + `colcon test-result --verbose` 全绿；记录实际命令与输出。
      注意 `test_model_consistency.py` 的 dist 断言依赖本机存在 `dist/`（未入 git 的派生发布件），
      干净 checkout 上该断言先天失败，与本次改动无关。
