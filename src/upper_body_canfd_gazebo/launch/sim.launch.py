"""启动 Gazebo Fortress、机器人模型和 Gazebo 侧 effort controller。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("upper_body_canfd_gazebo")
    xacro_file = PathJoinSubstitution([pkg_share, "urdf", "upper_body.urdf.xacro"])
    controller_file = PathJoinSubstitution([pkg_share, "config", "controllers.yaml"])
    world_file = PathJoinSubstitution([pkg_share, "worlds", "upper_body_world.sdf"])

    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                xacro_file,
                " controllers_file:=",
                controller_file,
            ]
        ),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            "gz_args": [
                # Server-only mode when gui:=false (e.g. headless containers).
                PythonExpression(["'-s ' if '", LaunchConfiguration("gui"), "'.lower() == 'false' else ''"]),
                # Optional render engine override, e.g. render_engine:=ogre for
                # environments where the OGRE2 backend fails (WSLg + Mesa d3d12).
                PythonExpression(
                    [
                        "'--render-engine ' + '",
                        LaunchConfiguration("render_engine"),
                        "' + ' ' if '",
                        LaunchConfiguration("render_engine"),
                        "' else ''",
                    ]
                ),
                "-r -v 2 ",
                world_file,
            ]
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    # Gazebo Fortress publishes simulation time on Gazebo Transport. ros_gz_bridge
    # exposes it as ROS /clock so every use_sim_time node advances with the simulator.
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-name", "upper_body_canfd_robot", "-topic", "robot_description"],
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    effort_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["effort_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Humble/Fortress startup is event-driven instead of relying on fixed delays:
    # create model -> joint state broadcaster -> effort controller.
    # The controller spawners wait for /controller_manager when necessary.
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument(
                "render_engine",
                default_value="",
                description="Gazebo rendering engine (e.g. ogre, ogre2); empty keeps the default",
            ),
            gazebo,
            robot_state_publisher,
            clock_bridge,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_robot,
                    on_exit=[joint_state_spawner],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_spawner,
                    on_exit=[effort_spawner],
                )
            ),
            # Event handlers are registered before create starts, so even a fast
            # successful spawn cannot race past the controller startup chain.
            spawn_robot,
        ]
    )
