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

"""Bridge + interactive motor_master in ONE launch (no slave required).

Useful when you only want to send commands and watch the (virtual) bus with
``candump vcan0``, or when a real/other node answers on the bus:

  ros2 launch can_motor_sim motor_master.launch.py protocol:=A node_id:=1

The master node is started with ``emulate_tty`` so its stdin is attached to
this launch's terminal - type commands (enable / set_mit / rp_mit ...) right
here. Press Ctrl-C to stop the launch.

Note: ``ros2 launch`` stdin forwarding via pty can be flaky in some terminals;
if typing does not reach the master, start it separately:
  Terminal 1: ros2 launch can_motor_sim socketcan_bridge.launch.py
  Terminal 2: ros2 run can_motor_sim motor_master --protocol A --id 1
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('can_motor_sim')
    protocol = LaunchConfiguration('protocol')
    node_id = LaunchConfiguration('node_id')
    interface = LaunchConfiguration('interface')

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'socketcan_bridge.launch.py')),
        launch_arguments={
            'interface': interface,
        }.items(),
    )

    master = Node(
        package='can_motor_sim',
        executable='motor_master',
        name='motor_master',
        output='screen',
        emulate_tty=True,
        arguments=['--protocol', protocol, '--id', node_id],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'protocol', default_value='A',
            description='Motor protocol (A or B)'),
        DeclareLaunchArgument(
            'node_id', default_value='1',
            description='Device id (A: 1..60, B: 1..8)'),
        DeclareLaunchArgument(
            'interface', default_value='vcan0'),
        bridge,
        master,
    ])
