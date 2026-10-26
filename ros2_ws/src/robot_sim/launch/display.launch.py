"""启动 RViz 模型预览（无 Gazebo）：滑杆驱动关节，验证模型外观与运动学。

模型由 `model:=` 选择（`models/<模型名>/`）。本文件读的是纯模型 URDF，
不含 ros2_control 与仿真插件——要在 Ignition 里被控制器驱动请用 gazebo.launch.py。

滑杆节点（joint_state_publisher_gui）直接发 /joint_states，
robot_state_publisher 按 URDF 树发 TF。

默认模型为 half_body_robot（两条互不相连的 9 轴链共 18 轴，link 名大写 `Joint_linkN` /
`Joint_linkN.N`，joint 名小写）。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("robot_sim")
    model = LaunchConfiguration("model")
    # 文件名带模型名：models/<model>/urdf/<model>.urdf
    urdf_file = PathJoinSubstitution(
        [pkg_share, "models", model, "urdf", [model, ".urdf"]]
    )

    # LaunchConfiguration 在运行期求值，因此延迟读取 URDF。
    robot_description = ParameterValue(
        Command([FindExecutable(name="cat"), " ", urdf_file]),
        value_type=str,
    )

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
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", LaunchConfiguration("rviz_config")],
            ),
        ]
    )
