"""重力前馈叠加节点的独立启动入口（`robot_dynamics/gravity_feedforward_overlay`）。

只起这一个节点；**不** include 也不修改 `robot_sim` 的 launch。启用时把命令源的话题
指到 `/robot_dynamics/command_array` 与 `/robot_dynamics/trajectory`（节点默认输入），
节点加上 τ_g 后转发到 `micro_ros_bridge` 的 `/motor/*` 主机入口。

  ros2 launch robot_dynamics gravity_feedforward.launch.py \
    urdf_path:=/path/to/model.urdf joint_map:=1:joint_link1,2:joint_link2

joint_map 留空 = 节点内置的 half_body_robot 兜底表，只对默认模型正确；换模型必须传参。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "urdf_path",
            description="URDF 文件路径（必填；空或解析失败节点起不来）",
        ),
        DeclareLaunchArgument(
            "joint_map",
            default_value="",
            description="motor_id:关节名 逗号分隔；留空用节点内置的 "
            "half_body_robot 兜底表（换模型必须传参）",
        ),
    ]
    node = Node(
        package="robot_dynamics",
        executable="gravity_feedforward_overlay",
        name="gravity_feedforward_overlay",
        output="screen",
        parameters=[
            {
                "urdf_path": LaunchConfiguration("urdf_path"),
                "joint_map": LaunchConfiguration("joint_map"),
            }
        ],
    )
    return LaunchDescription(args + [node])
