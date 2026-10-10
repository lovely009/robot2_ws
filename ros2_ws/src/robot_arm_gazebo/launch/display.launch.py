"""启动 RViz 模型预览（无 Gazebo）：滑杆驱动关节，验证模型外观与运动学。

模型由 `model:=` 选择（`models/<模型名>/`）。本文件读的是纯模型 URDF，
不含 ros2_control 与仿真插件——要在 Ignition 里被控制器驱动请用 gazebo.launch.py。

腕部闭环：URDF 是树，两条远置驱动连杆分支的闭环在导出时被切开，直接用滑杆拖
joint_link5/6 会让连杆与电机「散架」。因此默认启动 wrist_closure_solver：
滑杆节点改发 /joint_states_raw，求解器按闭环约束补出四个被动关节后再发 /joint_states，
robot_state_publisher 订阅后者。求解器只做运动学重算，不发任何真机命令；
`closure_solver:=false` 可关掉它，回到原始（散架）行为做对照。
求解器的摇臂轴参数来自模型目录的 config/wrist_closure.yaml，**该数值尚未标定**。

⚠ 没有腕部闭环的模型（例如 zhoubuguanjie_youbi_002，导出时取消了连杆）**没有
config/wrist_closure.yaml**，必须显式传 `closure_solver:=false`，否则求解器节点
会因为读不到配置文件而退出。这类模型本来也不需要求解器。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
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
    pkg_share = FindPackageShare("robot_arm_gazebo")
    model = LaunchConfiguration("model")
    # 文件名带模型名：models/<model>/urdf/<model>.urdf
    urdf_file = PathJoinSubstitution(
        [pkg_share, "models", model, "urdf", [model, ".urdf"]]
    )
    closure_config = PathJoinSubstitution(
        [pkg_share, "models", model, "config", "wrist_closure.yaml"]
    )

    # LaunchConfiguration 在构造期没有值，不能再像单模型时代那样 open().read()；
    # 用 cat 在运行期读文件，与 gazebo.launch.py 用 xacro 的做法一致。
    robot_description = ParameterValue(
        Command([FindExecutable(name="cat"), " ", urdf_file]),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value="zhoubuguanjie_youbi_001",
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
                "closure_solver",
                default_value="true",
                description="是否启动腕部闭环求解器（false = 看原始「散架」行为做对照）",
            ),
            # 滑杆节点两版，按 closure_solver 二选一：
            #   启用求解器 -> 先发 /joint_states_raw，由求解器补出被动关节后再发 /joint_states；
            #   关掉求解器 -> **必须直发 /joint_states**，否则没有任何节点产出关节状态，
            #     robot_state_publisher 收不到输入，RViz 里整条臂都不成树。
            # （这条 remap 早先是无条件加的，于是 closure_solver:=false 实际上从来没生效过：
            #   滑杆发到 /joint_states_raw 后就断了。没有腕部闭环的模型——例如
            #   zhoubuguanjie_youbi_002——必须走下面这一支。）
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                output="screen",
                parameters=[{"robot_description": robot_description}],
                remappings=[("joint_states", "joint_states_raw")],
                condition=IfCondition(LaunchConfiguration("closure_solver")),
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                output="screen",
                parameters=[{"robot_description": robot_description}],
                condition=UnlessCondition(LaunchConfiguration("closure_solver")),
            ),
            Node(
                package="robot_arm_gazebo",
                executable="wrist_closure_solver.py",
                output="screen",
                parameters=[
                    {
                        "urdf_path": ParameterValue(urdf_file, value_type=str),
                        "config_path": ParameterValue(closure_config, value_type=str),
                        "input_topic": "/joint_states_raw",
                        "output_topic": "/joint_states",
                    }
                ],
                condition=IfCondition(LaunchConfiguration("closure_solver")),
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
