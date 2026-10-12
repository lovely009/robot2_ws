"""启动 MuJoCo 仿真（自写力矩直通桥）+ 关节状态发布。

模型由 `model:=` 选择，需要 `models/<模型名>/mujoco/<模型名>.xml`
（用 `models/<模型名>/tools/build_mujoco_model.py` 生成）。

为什么不用 `mujoco_ros2_control` 的 ros2_control 集成：本机 0.1.2 在注册 actuator 时会创建
非法话题名（`pid_gains/position/<joint>/<index>/pid_state`，ROS 2 不允许纯数字 token）直接
abort。本入口改用 `scripts/mujoco_effort_bridge.py`，**话题契约与 gazebo 那条路完全一致**：

  订阅 /arm_effort_controller/commands（Float64MultiArray，18 个数 = 物理关节顺序）
  发布 /joint_states

所以 `simulation.launch.py`、web_host、手工 `ros2 topic pub` 的操作方式都不用改。

控制方式（命令 18 个数，顺序 = joint_link1..9 → joint_link1.1..9.9）：

  ros2 topic pub -r 50 /arm_effort_controller/commands \\
    std_msgs/msg/Float64MultiArray "{data: [1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}"

参数：model / mujoco_model / headless / control_hz / sim_speed_factor / command_timeout_s。
"""

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


def _launch_setup(context):
    """把启动参数写成临时 params 文件交给桥节点（避免在命令行里拼长参数串）。"""
    parameters = {
        "mujoco_effort_bridge": {
            "ros__parameters": {
                "mujoco_model": LaunchConfiguration("mujoco_model").perform(context),
                "effort_topic": "/arm_effort_controller/commands",
                "joint_states_topic": "/joint_states",
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
        Path(get_package_prefix("robot_arm_gazebo"))
        / "lib"
        / "robot_arm_gazebo"
        / "mujoco_effort_bridge.py"
    )
    bridge = ExecuteProcess(
        cmd=[python, "-u", str(bridge_script), "--ros-args", "--params-file", str(params_file)],
        output="both",
        emulate_tty=True,
    )
    cleanup = ExecuteProcess(
        cmd=["python3", "-c",
             "import shutil,sys; shutil.rmtree(sys.argv[1], ignore_errors=True)",
             str(directory)],
        output="log",
    )
    return [bridge, RegisterEventHandler(OnShutdown(on_shutdown=[cleanup]))]


def generate_launch_description():
    pkg_share = FindPackageShare("robot_arm_gazebo")
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
            DeclareLaunchArgument("model", default_value="robot_all",
                                  description="models/ 下的模型目录名"),
            DeclareLaunchArgument(
                "mujoco_model",
                default_value=PathJoinSubstitution(
                    [pkg_share, "models", model, "mujoco", [model, ".xml"]]
                ),
                description="MJCF 路径（默认取模型目录的 mujoco/<模型名>.xml）",
            ),
            DeclareLaunchArgument("headless", default_value="false",
                                  description="true = 不开 MuJoCo viewer 窗口"),
            DeclareLaunchArgument("control_hz", default_value="100.0",
                                  description="控制/发布频率"),
            DeclareLaunchArgument("sim_speed_factor", default_value="1.0",
                                  description="仿真速度倍率（1.0 = 实时）"),
            DeclareLaunchArgument("command_timeout_s", default_value="0.5",
                                  description="命令超时（秒）后力矩清零"),
            robot_state_publisher,
            OpaqueFunction(function=_launch_setup),
        ]
    )
