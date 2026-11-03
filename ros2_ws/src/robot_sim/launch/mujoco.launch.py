"""启动 MuJoCo 仿真（自写力矩直通桥）+ 关节状态发布。

模型由 `model:=` 选择，需要 `models/<模型名>/mujoco/<模型名>.xml`
（用 `models/<模型名>/tools/build_mujoco_model.py` 生成）。

使用 `scripts/mujoco_effort_bridge.py`，与 Gazebo 保持相同话题契约：

  订阅 /arm_effort_controller/commands（Float64MultiArray，18 个数 = 物理关节顺序）
  发布 /joint_states 与 /clock

`joint_names` 参数（逗号分隔的有序关节名，默认空）会交给桥做**按名字**的命令映射：
`simulation.launch.py` 的 `backend:=mujoco` 会传拓扑顺序的关节名，避免依赖
「MJCF 关节顺序恰好等于拓扑顺序」这个巧合。留空则按 MJCF 顺序（手工操作时用）。

控制方式（命令 18 个数，顺序 = joint_link1..9 → joint_link1.1..9.9）：

  ros2 topic pub -r 50 /arm_effort_controller/commands \\
    std_msgs/msg/Float64MultiArray "{data: [1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}"

参数：model / mujoco_model / joint_names / headless / control_hz / sim_speed_factor /
command_timeout_s。

Wayland 下通过进程环境强制 pyGLFW 使用 X11，避免 viewer 自旋阻塞状态发布。
必须在导入 mujoco/glfw 前设置，并删除 WAYLAND_DISPLAY；因此传完整 env。
"""

import os
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# mujoco_ros2_control 的 apt 包会建这个 venv（里面有 mujoco python 包）；
# 找不到就退回系统 python3，由桥脚本自己报告 import 失败。
VENV_PYTHON = Path.home() / ".ros/ros2_control/.venv/bin/python"
TRUTHY = {"1", "true", "yes", "on"}


def _bridge_environment() -> dict:
    """给桥进程一份清理过的环境（见模块 docstring 里 pyGLFW 那段）。

    必须删除 `WAYLAND_DISPLAY` 而不是置空：glfw 只看变量在不在。`additional_env` 只能
    增改不能删，所以这里传整份 `env`。
    """
    environment = dict(os.environ)
    if environment.get("XDG_SESSION_TYPE") == "wayland" and environment.get("DISPLAY"):
        environment["PYGLFW_LIBRARY_VARIANT"] = "x11"
        environment.pop("WAYLAND_DISPLAY", None)
    return environment


def _launch_setup(context):
    """把启动参数写成临时 params 文件交给桥节点（避免在命令行里拼长参数串）。"""
    parameters = {
        "mujoco_effort_bridge": {
            "ros__parameters": {
                "mujoco_model": LaunchConfiguration("mujoco_model").perform(context),
                "effort_topic": "/arm_effort_controller/commands",
                "joint_states_topic": "/joint_states",
                "joint_names": LaunchConfiguration("joint_names").perform(context),
                "control_hz": float(LaunchConfiguration("control_hz").perform(context)),
                "command_timeout_s": float(
                    LaunchConfiguration("command_timeout_s").perform(context)
                ),
                "sim_speed_factor": float(
                    LaunchConfiguration("sim_speed_factor").perform(context)
                ),
                "headless": LaunchConfiguration("headless").perform(context).lower() in TRUTHY,
            }
        }
    }
    directory = Path(tempfile.mkdtemp(prefix="mujoco_bridge_"))
    params_file = directory / "bridge.yaml"
    params_file.write_text(yaml.safe_dump(parameters, sort_keys=False), encoding="utf-8")

    python = str(VENV_PYTHON) if VENV_PYTHON.is_file() else "python3"
    bridge_script = (
        Path(get_package_prefix("robot_sim"))
        / "lib"
        / "robot_sim"
        / "mujoco_effort_bridge.py"
    )
    bridge = ExecuteProcess(
        cmd=[python, "-u", str(bridge_script), "--ros-args", "--params-file", str(params_file)],
        output="both",
        emulate_tty=True,
        env=_bridge_environment(),
    )
    cleanup = ExecuteProcess(
        cmd=["python3", "-c",
             "import shutil,sys; shutil.rmtree(sys.argv[1], ignore_errors=True)",
             str(directory)],
        output="log",
    )
    return [bridge, RegisterEventHandler(OnShutdown(on_shutdown=[cleanup]))]


def generate_launch_description():
    pkg_share = FindPackageShare("robot_description")
    model = LaunchConfiguration("model")
    # MuJoCo 侧基座固定（MJCF 无 free joint），与 Gazebo 的 fixed_base:=true 等价，
    # 因此用不带 world 的 standalone 模型
    urdf_file = PathJoinSubstitution(
        [pkg_share, "models", model, "urdf", [model, ".standalone.urdf"]]
    )
    robot_description = ParameterValue(Command(["cat ", urdf_file]), value_type=str)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("model", default_value="half_body_robot",
                                  description="models/ 下的模型目录名"),
            DeclareLaunchArgument(
                "mujoco_model",
                default_value=PathJoinSubstitution(
                    [pkg_share, "models", model, "mujoco", [model, ".xml"]]
                ),
                description="MJCF 路径（默认取模型目录的 mujoco/<模型名>.xml）",
            ),
            DeclareLaunchArgument(
                "joint_names",
                default_value="",
                description="逗号分隔的有序关节名，交给桥按名字映射命令数组；"
                            "空 = 按 MJCF 关节顺序（手工 pub 时用）",
            ),
            DeclareLaunchArgument("headless", default_value="false",
                                  description="true = 不开 MuJoCo viewer 窗口"),
            DeclareLaunchArgument("control_hz", default_value="100.0",
                                  description="控制/发布频率。HIL 闭环里它决定"
                                              "被控对象状态帧的节拍，是 PD 回路的"
                                              "死区主项（100 Hz ⇒ 10ms 一拍）；"
                                              "kp=30/kd=1 在合成 ~20ms 死区下正好"
                                              "压线（2026-10-14 实测 ±1 rad 极限环）。"
                                              "物理步数同比减少，CPU 基本不变"),
            DeclareLaunchArgument("sim_speed_factor", default_value="1.0",
                                  description="仿真速度倍率（1.0 = 实时）"),
            DeclareLaunchArgument("command_timeout_s", default_value="0.5",
                                  description="命令超时（秒）后力矩清零"),
            robot_state_publisher,
            OpaqueFunction(function=_launch_setup),
        ]
    )
