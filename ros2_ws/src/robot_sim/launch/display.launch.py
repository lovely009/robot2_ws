"""启动 RViz 模型预览（无 Gazebo）：滑杆驱动关节，验证模型外观与运动学。

模型由 `model:=` 选择（`models/<模型名>/`）。本文件读的是纯模型 URDF，
不含 ros2_control 与仿真插件——要在 Ignition 里被控制器驱动请用 gazebo.launch.py。

关节状态发布器直接发 /joint_states，robot_state_publisher 按 URDF 树发 TF。
full_body_robot 默认精确发布 Standing 初始角，可用 joint_state_gui:=true 调关节。

默认模型为 half_body_robot（两条互不相连的 9 轴链共 18 轴，link 名大写 `Joint_linkN` /
`Joint_linkN.N`，joint 名小写）。
"""

import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _display_nodes(context):
    model = LaunchConfiguration("model").perform(context)
    model_dir = Path(get_package_share_directory("robot_description")) / "models" / model
    standing = model == "full_body_robot"
    gui = LaunchConfiguration("joint_state_gui").perform(context)
    publisher = "joint_state_publisher_gui" if gui == "true" or (gui == "auto" and not standing) \
        else "joint_state_publisher"
    urdf_name = f"{model}.standing.urdf" if standing else f"{model}.urdf"
    robot_description = ParameterValue((model_dir / "urdf" / urdf_name).read_text(), value_type=str)
    joint_parameters = {"robot_description": robot_description}
    rviz_arguments = ["-d", LaunchConfiguration("rviz_config")]
    if standing:
        positions = json.loads((model_dir / "config/standing.json").read_text())
        joint_parameters["zeros"] = positions
        rviz_arguments.extend(["-f", "world"])
    return [
        Node(package=publisher, executable=publisher,
             output="screen", parameters=[joint_parameters]),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             output="screen", parameters=[{"robot_description": robot_description}]),
        Node(package="rviz2", executable="rviz2", output="screen", arguments=rviz_arguments),
    ]


def generate_launch_description():
    pkg_share = FindPackageShare("robot_sim")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value="half_body_robot",
                description="models/ 下的模型目录名",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [pkg_share, "config", "display.rviz"]
                ),
                description="RViz 配置文件",
            ),
            DeclareLaunchArgument(
                "joint_state_gui", default_value="auto", choices=["auto", "true", "false"],
                description="是否显示关节滑杆，auto 在 full 整机预览中精确发布初始角",
            ),
            OpaqueFunction(function=_display_nodes),
        ]
    )
