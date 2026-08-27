# robot_arm_gazebo Unified Launch Backend Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让 `ros2 launch robot_arm_gazebo gazebo.launch.py` 单命令启动可供外部工程接入的完整 UR5e 电机仿真后端，并集中声明、注释和覆盖全部运行参数。

**Architecture:** 保留 `gazebo.launch.py` 作为唯一编排入口，在文件顶部集中默认值，通过带中文说明的 launch arguments 暴露给调用方。现有 entity → broadcaster → effort controller 的 fail-fast 链扩展到 motor node；motor node 置于机器人 namespace，并通过参数和 remapping 接入 Gazebo 控制接口。

**Tech Stack:** ROS 2 Humble Python launch、`launch_ros`、Gazebo Fortress / `ros_gz_sim`、pytest、ament/colcon、Markdown。

**Spec:** `docs/superpowers/specs/2026-08-27-unified-launch-backend-design.md`

## Global Constraints

- 仓库只包含 `motor_sim` 和 `robot_arm_gazebo` 两个 ament package，不新增第三个 package。
- `launch/` 只保留 `gazebo.launch.py`，不新增其他 launch 文件。
- `config/motors.yaml` 继续作为六轴顺序、总线、电机协议、ID、限位及仿名义参数的唯一来源。
- transport 只允许 `usb_serial`、`serial_slcan` 和 `socket_can`，一个进程只能选择一种。
- `serial_slcan` 必须显式设置 `allow_external_canfd:=true`。
- 保持显式 enable、150 ms 命令超时、断线停机及锁存安全语义；`fault_latching_enabled:=false` 仅用于调试。
- 动作播放器不得由 launch 启动，也不得与电机仿真节点并行控制。
- launch 每个参数必须有中文注释和可由 `--show-args` 显示的中文 `description`。
- 自动测试不得要求 Gazebo GUI、真实串口或真实 CAN 设备。

---

### Task 1: 用静态契约定义统一 launch 的公开接口

**Files:**
- Modify: `src/robot_arm_gazebo/test/test_launch_contract.py`
- Modify: `src/robot_arm_gazebo/launch/gazebo.launch.py`

**Interfaces:**
- Consumes: 现有 `generate_launch_description()`、`_build_runtime(context)` 和 `parse_initial_positions()`。
- Produces: `LAUNCH_DEFAULTS: dict[str, object]`；launch arguments `initial_positions`、`world_file`、`gz_verbosity`、`transport_mode`、`arm_port`、`baud_rate`、`allow_external_canfd`、`fault_latching_enabled`、`motor_config_file`、`robot_namespace`、`joint_states_topic`、`effort_commands_topic`、`diagnostics_topic`。

- [ ] **Step 1: 写统一参数表的失败测试**

在 `test_launch_contract.py` 增加：

```python
EXPECTED_ARGUMENTS = {
    "initial_positions",
    "world_file",
    "gz_verbosity",
    "transport_mode",
    "arm_port",
    "baud_rate",
    "allow_external_canfd",
    "fault_latching_enabled",
    "motor_config_file",
    "robot_namespace",
    "joint_states_topic",
    "effort_commands_topic",
    "diagnostics_topic",
}


def test_gazebo_launch_declares_documented_backend_arguments() -> None:
    module = load_launch_module()
    launch_description = module.generate_launch_description()
    declarations = {
        entity.name: entity
        for entity in launch_description.entities
        if entity.__class__.__name__ == "DeclareLaunchArgument"
    }
    assert declarations.keys() == EXPECTED_ARGUMENTS
    for declaration in declarations.values():
        assert declaration.description


def test_gazebo_launch_has_expected_connection_defaults() -> None:
    module = load_launch_module()
    assert module.LAUNCH_DEFAULTS["transport_mode"] == "usb_serial"
    assert module.LAUNCH_DEFAULTS["arm_port"] == "/dev/ttyUSB0"
    assert module.LAUNCH_DEFAULTS["baud_rate"] == "3000000"
    assert module.LAUNCH_DEFAULTS["allow_external_canfd"] == "false"
    assert module.LAUNCH_DEFAULTS["fault_latching_enabled"] == "true"


def test_transport_mode_rejects_unknown_value() -> None:
    module = load_launch_module()
    with pytest.raises(ValueError, match="transport_mode"):
        module.validate_transport_mode("automatic")
```

- [ ] **Step 2: 运行测试并确认因参数接口缺失而失败**

Run:

```bash
source /opt/ros/humble/setup.bash
python3 -m pytest src/robot_arm_gazebo/test/test_launch_contract.py -q
```

Expected: FAIL，指出 `LAUNCH_DEFAULTS` 不存在或声明集合缺少新参数；原有初始姿态测试仍通过。

- [ ] **Step 3: 在 launch 顶部增加集中默认值和参数声明**

在 imports 后定义字符串默认值，包内路径使用 substitutions 留在 `_build_runtime` 解析：

```python
LAUNCH_DEFAULTS = {
    "initial_positions": "0,-1.57,0,-1.57,0,0",
    "world_file": "",
    "gz_verbosity": "2",
    "transport_mode": "usb_serial",
    "arm_port": "/dev/ttyUSB0",
    "baud_rate": "3000000",
    "allow_external_canfd": "false",
    "fault_latching_enabled": "true",
    "motor_config_file": "",
    "robot_namespace": "robot_arm",
    "joint_states_topic": "joint_states",
    "effort_commands_topic": "effort_controller/commands",
    "diagnostics_topic": "diagnostics",
}
```

增加 launch 层枚举校验，并在 `_build_runtime` 的第一段、创建任何 action 前对
`LaunchConfiguration("transport_mode").perform(context)` 调用它：

```python
def validate_transport_mode(value: str) -> str:
    supported = {"usb_serial", "serial_slcan", "socket_can"}
    if value not in supported:
        raise ValueError(
            "transport_mode must be usb_serial, serial_slcan or socket_can"
        )
    return value
```

节点仍保留权威参数和设备校验；launch 的提前校验只负责在 Gazebo 进程创建前拒绝非法
枚举，不复制 tty、SocketCAN 或波特率规则。

为 13 个参数逐一创建 `DeclareLaunchArgument`。`world_file` 和
`motor_config_file` 的声明默认值分别使用：

```python
PathJoinSubstitution(
    [FindPackageShare("robot_arm_gazebo"), "worlds", "ur5e_world.sdf"]
)
PathJoinSubstitution(
    [FindPackageShare("robot_arm_gazebo"), "config", "motors.yaml"]
)
```

每个声明前写中文代码注释，并设置面向 `--show-args` 的中文 `description`，例如：

```python
# 外部电机协议端点的传输方式；只允许 usb_serial、serial_slcan、socket_can。
DeclareLaunchArgument(
    "transport_mode",
    default_value=LAUNCH_DEFAULTS["transport_mode"],
    description="电机协议传输方式：usb_serial、serial_slcan 或 socket_can。",
)
```

- [ ] **Step 4: 运行 launch 契约测试并确认参数测试通过**

Run:

```bash
source /opt/ros/humble/setup.bash
python3 -m pytest src/robot_arm_gazebo/test/test_launch_contract.py -q
```

Expected: PASS。

- [ ] **Step 5: 提交公开参数接口**

```bash
git add src/robot_arm_gazebo/launch/gazebo.launch.py \
  src/robot_arm_gazebo/test/test_launch_contract.py
git commit -m "feat: declare unified arm backend launch parameters"
```

---

### Task 2: 将 motor node 接入控制器成功链和 ROS 接口

**Files:**
- Modify: `src/robot_arm_gazebo/test/test_launch_contract.py`
- Modify: `src/robot_arm_gazebo/launch/gazebo.launch.py`

**Interfaces:**
- Consumes: Task 1 的 13 个 `LaunchConfiguration`；现有 `robot_arm_motor_sim_node` ROS 参数 `transport_mode`、`arm_port`、`baud_rate`、`config_file`、`allow_external_canfd`、`fault_latching_enabled`。
- Produces: namespace 内的 motor node；remapping `joint_states`、`effort_controller/commands`、`diagnostics`；effort spawner 成功后启动 motor node；motor node 退出后 `Shutdown`。

- [ ] **Step 1: 写 motor node 编排的失败测试**

在 `test_launch_contract.py` 增加源码级契约断言：

```python
def test_gazebo_launch_starts_motor_backend_after_effort_controller() -> None:
    text = LAUNCH_FILE.read_text(encoding="utf-8")
    assert 'executable="robot_arm_motor_sim_node"' in text
    assert '"transport_mode": LaunchConfiguration("transport_mode")' in text
    assert '"arm_port": LaunchConfiguration("arm_port")' in text
    assert '"config_file": LaunchConfiguration("motor_config_file")' in text
    assert '("joint_states", LaunchConfiguration("joint_states_topic"))' in text
    assert 'LaunchConfiguration("effort_commands_topic")' in text
    assert '("diagnostics", LaunchConfiguration("diagnostics_topic"))' in text
    assert '_start_after_success(motor_sim, "启动 effort_controller")' in text


def test_motor_backend_exit_shuts_down_unified_launch() -> None:
    text = LAUNCH_FILE.read_text(encoding="utf-8")
    assert "target_action=motor_sim" in text
    assert "_shutdown_on_exit(\"电机仿真后端\")" in text
```

- [ ] **Step 2: 运行测试并确认因 motor node 未纳入 launch 而失败**

Run:

```bash
source /opt/ros/humble/setup.bash
python3 -m pytest \
  src/robot_arm_gazebo/test/test_launch_contract.py::test_gazebo_launch_starts_motor_backend_after_effort_controller \
  src/robot_arm_gazebo/test/test_launch_contract.py::test_motor_backend_exit_shuts_down_unified_launch -q
```

Expected: 两项 FAIL，缺少 executable 和退出事件处理。

- [ ] **Step 3: 构造带类型参数和 remapping 的 motor node**

在 `_build_runtime` 中创建：

```python
motor_sim = Node(
    package="robot_arm_gazebo",
    executable="robot_arm_motor_sim_node",
    namespace=LaunchConfiguration("robot_namespace"),
    output="screen",
    parameters=[
        {
            "transport_mode": LaunchConfiguration("transport_mode"),
            "arm_port": LaunchConfiguration("arm_port"),
            "baud_rate": ParameterValue(
                LaunchConfiguration("baud_rate"), value_type=int
            ),
            "config_file": LaunchConfiguration("motor_config_file"),
            "allow_external_canfd": ParameterValue(
                LaunchConfiguration("allow_external_canfd"), value_type=bool
            ),
            "fault_latching_enabled": ParameterValue(
                LaunchConfiguration("fault_latching_enabled"), value_type=bool
            ),
        }
    ],
    remappings=[
        ("joint_states", LaunchConfiguration("joint_states_topic")),
        (
            "effort_controller/commands",
            LaunchConfiguration("effort_commands_topic"),
        ),
        ("diagnostics", LaunchConfiguration("diagnostics_topic")),
    ],
)
```

将 effort spawner 的成功动作从 `None` 改为 `motor_sim`。motor node 不直接放入返回动作
列表，保证只由 success handler 启动一次。

- [ ] **Step 4: 增加长驻关键进程退出处理**

增加 helper：

```python
def _shutdown_on_exit(stage: str):
    def _on_exit(event, _context):
        reason = f"{stage} 已退出（退出码 {event.returncode}），停止 UR5e 仿真后端"
        return [LogInfo(msg=reason), Shutdown(reason=reason)]

    return _on_exit
```

在 `_build_runtime` 返回列表中注册：

```python
RegisterEventHandler(
    OnProcessExit(
        target_action=motor_sim,
        on_exit=_shutdown_on_exit("电机仿真后端"),
    )
)
```

- [ ] **Step 5: 从 launch 参数生成 Gazebo、world 和 namespace 配置**

将固定 world 与 verbosity 改为：

```python
"gz_args": [
    "-r -v ",
    LaunchConfiguration("gz_verbosity"),
    " ",
    LaunchConfiguration("world_file"),
]
```

将 `PushRosNamespace("robot_arm")`、spawn node namespace 和 controller manager 路径
统一由 `LaunchConfiguration("robot_namespace")` 推导。controller manager 参数使用
`PythonExpression` 组成 `/<namespace>/controller_manager`，避免第二份 namespace 默认值：

```python
from launch.substitutions import PythonExpression

robot_namespace = LaunchConfiguration("robot_namespace")
controller_manager = PythonExpression(
    ["'/' + '", robot_namespace, "'.strip('/') + '/controller_manager'"]
)
```

两个 spawner 的 `--controller-manager` 后都传 `controller_manager`。state publisher 使用
`GroupAction([PushRosNamespace(robot_namespace), robot_state_publisher])`，spawn node 使用
`namespace=robot_namespace`，motor node 也使用同一个 `robot_namespace`。

- [ ] **Step 6: 运行完整 launch 契约测试**

Run:

```bash
source /opt/ros/humble/setup.bash
python3 -m pytest src/robot_arm_gazebo/test/test_launch_contract.py -q
```

Expected: PASS，且无 collection error 或 launch import warning。

- [ ] **Step 7: 提交完整编排**

```bash
git add src/robot_arm_gazebo/launch/gazebo.launch.py \
  src/robot_arm_gazebo/test/test_launch_contract.py
git commit -m "feat: start motor backend from gazebo launch"
```

---

### Task 3: 更新用户入口和领域契约文档

**Files:**
- Modify: `README.md`
- Modify: `CONTEXT.md`

**Interfaces:**
- Consumes: Task 1–2 的 launch 参数和稳定 topic。
- Produces: 单命令启动说明、USB/ACM/SocketCAN/SLCAN-FD 示例、调试覆盖示例和统一入口领域约束。

- [ ] **Step 1: 写文档契约的失败测试**

在 `test_launch_contract.py` 增加：

```python
REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]


def test_docs_use_only_unified_launch_for_motor_backend() -> None:
    readme = (REPOSITORY_ROOT / "README.md").read_text(encoding="utf-8")
    context = (REPOSITORY_ROOT / "CONTEXT.md").read_text(encoding="utf-8")
    assert "ros2 run robot_arm_gazebo robot_arm_motor_sim_node" not in readme
    assert "ros2 launch robot_arm_gazebo gazebo.launch.py" in readme
    assert "--show-args" in readme
    assert "完整仿真后端的唯一启动入口" in context
```

- [ ] **Step 2: 运行测试并确认旧的双终端工作流导致失败**

Run:

```bash
source /opt/ros/humble/setup.bash
python3 -m pytest \
  src/robot_arm_gazebo/test/test_launch_contract.py::test_docs_use_only_unified_launch_for_motor_backend -q
```

Expected: FAIL，README 仍包含单独 `ros2 run`，CONTEXT 尚未声明统一入口。

- [ ] **Step 3: 重写 README 启动章节**

启动章节以默认命令开头：

```bash
ros2 launch robot_arm_gazebo gazebo.launch.py
```

说明默认 `usb_serial + /dev/ttyUSB0`，设备不存在时整套启动失败；补充
`/dev/ttyACM0`、`socket_can + vcan0`、`serial_slcan + allow_external_canfd`、
`fault_latching_enabled:=false` 示例。加入：

```bash
ros2 launch robot_arm_gazebo gazebo.launch.py --show-args
```

删除所有将 `robot_arm_motor_sim_node` 作为正常用户入口的 `ros2 run` 示例；保留协议帧
人工调试命令，但将终端 1 改为统一 launch。

- [ ] **Step 4: 更新 CONTEXT.md**

在包边界或三种连接模式前加入精确契约：

```markdown
`robot_arm_gazebo/launch/gazebo.launch.py` 是完整仿真后端的唯一启动入口；它编排
Gazebo、UR5e、控制器和电机协议节点。外部工程通过 launch 参数选择 transport、设备和
ROS topic，不单独启动 `robot_arm_motor_sim_node`。
```

- [ ] **Step 5: 运行文档与 launch 契约测试**

Run:

```bash
source /opt/ros/humble/setup.bash
python3 -m pytest src/robot_arm_gazebo/test/test_launch_contract.py -q
```

Expected: PASS。

- [ ] **Step 6: 提交文档同步**

```bash
git add README.md CONTEXT.md src/robot_arm_gazebo/test/test_launch_contract.py
git commit -m "docs: document unified robot arm launch workflow"
```

---

### Task 4: 构建、全量测试和验收记录

**Files:**
- Create: `docs/verification/2026-08-27-unified-launch-backend.md`

**Interfaces:**
- Consumes: 完成后的两个 ament packages 和统一 launch。
- Produces: 可复核的构建、测试、包清单及 `--show-args` 验收记录。

- [ ] **Step 1: 检查工作空间 package 边界**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon list --names-only
```

Expected exactly:

```text
motor_sim
robot_arm_gazebo
```

- [ ] **Step 2: 全量构建**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

Expected: 两个 package 均 `Finished`，退出码 0。

- [ ] **Step 3: 全量测试**

Run:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test
colcon test-result --verbose
```

Expected: `colcon test` 退出码 0，结果为 0 failures、0 errors。

- [ ] **Step 4: 验证 launch 参数帮助**

Run:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_arm_gazebo gazebo.launch.py --show-args
```

Expected: 输出全部 13 个参数，每个参数都有中文描述；默认连接显示
`transport_mode=usb_serial`、`arm_port=/dev/ttyUSB0`、`baud_rate=3000000`。

- [ ] **Step 5: 写验收记录**

创建 `docs/verification/2026-08-27-unified-launch-backend.md`，逐项记录上述命令、执行时间、
退出码和摘要。若环境没有 `/dev/ttyUSB0`，明确说明未执行真实设备启动；这不是自动测试
失败，因为契约测试不依赖硬件。

- [ ] **Step 6: 检查差异并提交验收记录**

Run:

```bash
git diff --check
git status --short
```

Expected: `git diff --check` 无输出；status 只包含预期验收记录。

```bash
git add docs/verification/2026-08-27-unified-launch-backend.md
git commit -m "test: record unified launch backend verification"
```
