"""CAN-FD 真机 bring-up 入口（motor_hardware_interface 自足，不依赖仿真栈）。

启动 ros2_control（motor_hardware_interface/CanFdSystem）、robot_state_publisher、
joint_state_broadcaster 与 position_controller，直接驱动两条 SocketCAN/CAN-FD 总线。

model_xacro 是必填参数：指向能渲染 CAN-FD 后端 ros2_control 块的机器人模型 xacro，
要求该 xacro 暴露 hardware_backend/left_interface/right_interface/motor_config_file
参数（本仓库的 upper_body_canfd_gazebo/urdf/upper_body.urdf.xacro 即满足）。

真机部署只需底层驱动 + 本适配包 + 模型描述，不需要模拟器或 Gazebo。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from motor_hardware_interface.helpers import (
    motor_joints_from_config,
    write_position_joints_params_file,
)

# 驱动侧 position controller 的 joints 由权威真机拓扑配置生成（单一来源），
# 不在此处手抄。若调用方覆盖 motor_config_file，需保证拓扑顺序与默认一致。
POSITION_JOINTS = motor_joints_from_config(
    Path(get_package_share_directory("motor_hardware_interface"))
    / "config"
    / "motor_canfd.yaml"
)
# launch_ros 会把参数 dict 里的列表序列化成 !!python/tuple，rcl 解析后为空；
# 因此把 joints 写进标准嵌套格式的临时参数文件，随 manager 一起加载。
POSITION_JOINTS_PARAM_FILE = write_position_joints_params_file(POSITION_JOINTS)


def generate_launch_description() -> LaunchDescription:
    share = FindPackageShare("motor_hardware_interface")
    model_xacro = LaunchConfiguration("model_xacro")
    left_interface = LaunchConfiguration("left_interface")
    right_interface = LaunchConfiguration("right_interface")
    motor_config_file = LaunchConfiguration("motor_config_file")
    default_motor_config = PathJoinSubstitution([share, "config", "motor_canfd.yaml"])

    description = ParameterValue(Command([
        FindExecutable(name="xacro"), " ",
        model_xacro,
        " hardware_backend:=canfd left_interface:=", left_interface,
        " right_interface:=", right_interface,
        " motor_config_file:=", motor_config_file,
    ]), value_type=str)
    controller_config = PathJoinSubstitution([share, "config", "driver_controllers.yaml"])

    manager = Node(package="controller_manager", executable="ros2_control_node", output="screen",
                   parameters=[
                       {"robot_description": description},
                       controller_config,
                       POSITION_JOINTS_PARAM_FILE,
                   ])
    state_publisher = Node(package="robot_state_publisher", executable="robot_state_publisher",
                           output="screen", parameters=[{"robot_description": description}])
    jsb = Node(package="controller_manager", executable="spawner", output="screen",
               arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"])
    position = Node(package="controller_manager", executable="spawner", output="screen",
                    arguments=["position_controller", "--controller-manager", "/controller_manager"])

    return LaunchDescription([
        DeclareLaunchArgument(
            "model_xacro",
            default_value="",
            description="必填：渲染 CAN-FD 后端 ros2_control 块的机器人模型 xacro 路径",
        ),
        DeclareLaunchArgument("left_interface", default_value="can0"),
        DeclareLaunchArgument("right_interface", default_value="can1"),
        DeclareLaunchArgument(
            "motor_config_file",
            default_value=default_motor_config,
            description="真机电机拓扑配置；默认 motor_hardware_interface/config/motor_canfd.yaml",
        ),
        manager,
        state_publisher,
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[position])),
        jsb,
    ])
