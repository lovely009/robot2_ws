#!/usr/bin/env python3
# Copyright 2026 can_motor_sim contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""One-launch entry point for vcan-based motor drive debugging.

Starts:
  1. the ros2_socketcan bridge on ``vcan0`` (CAN FD enabled)
  2. (optional) a motor_slave simulator node (start_slave:=false to disable)
  3. (optional) a motor_master node (start_master:=true)

The master uses ``emulate_tty`` so its stdin is attached to this launch's
terminal, letting you type commands right here. If that is not convenient,
run the master in a second terminal with ``ros2 run can_motor_sim motor_master``.

Examples:
  # bridge + slave only
  ros2 launch can_motor_sim motor_debug.launch.py
  # bridge + master only (no slave needed to drive/observe the bus)
  ros2 launch can_motor_sim motor_debug.launch.py start_slave:=false start_master:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('can_motor_sim')
    protocol = LaunchConfiguration('protocol')
    node_id = LaunchConfiguration('node_id')
    interface = LaunchConfiguration('interface')
    start_slave = LaunchConfiguration('start_slave')
    start_master = LaunchConfiguration('start_master')

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'socketcan_bridge.launch.py')),
        launch_arguments={
            'interface': interface,
        }.items(),
    )

    slave = Node(
        package='can_motor_sim',
        executable='motor_slave',
        name='motor_slave',
        output='screen',
        arguments=['--protocol', protocol, '--id', node_id],
        condition=IfCondition(start_slave),
    )

    # emulate_tty gives motor_master a pty attached to this terminal's stdin,
    # so it can accept interactive commands from the launch terminal.
    master = Node(
        package='can_motor_sim',
        executable='motor_master',
        name='motor_master',
        output='screen',
        emulate_tty=True,
        arguments=['--protocol', protocol, '--id', node_id],
        condition=IfCondition(start_master),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'protocol', default_value='A',
            description='Motor protocol to simulate (A or B)'),
        DeclareLaunchArgument(
            'node_id', default_value='1',
            description='Device id (A: 1..60, B: 1..8)'),
        DeclareLaunchArgument(
            'interface', default_value='vcan0'),
        DeclareLaunchArgument(
            'start_slave', default_value='true',
            description='Start the motor_slave simulator (not required to drive/observe)'),
        DeclareLaunchArgument(
            'start_master', default_value='false',
            description='Also start motor_master in this launch (interactive via pty)'),
        bridge,
        slave,
        master,
    ])
