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

"""Launch the ros2_socketcan bridge on a (v)can interface with CAN FD enabled.

The bridge connects a Linux SocketCAN interface (default ``vcan0``) to two
ROS2 topics:
  * ``/to_can_bus_fd``   : FdFrame messages to be sent on the bus
  * ``/from_can_bus_fd`` : FdFrame messages received from the bus

These topic names match what ``motor_master`` / ``motor_slave`` use.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    socketcan_share = get_package_share_directory('ros2_socketcan')
    interface = LaunchConfiguration('interface')
    enable_can_fd = LaunchConfiguration('enable_can_fd')
    from_topic = LaunchConfiguration('from_can_bus_topic')
    to_topic = LaunchConfiguration('to_can_bus_topic')
    receiver_interval = LaunchConfiguration('receiver_interval_sec')
    sender_timeout = LaunchConfiguration('sender_timeout_sec')

    socketcan_bridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(socketcan_share, 'launch',
                         'socket_can_bridge.launch.xml')),
        launch_arguments={
            'interface': interface,
            'enable_can_fd': enable_can_fd,
            'from_can_bus_topic': from_topic,
            'to_can_bus_topic': to_topic,
            'receiver_interval_sec': receiver_interval,
            'sender_timeout_sec': sender_timeout,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'interface', default_value='vcan0',
            description='SocketCAN interface to bridge (vcan0 = virtual bus)'),
        DeclareLaunchArgument(
            'enable_can_fd', default_value='true',
            description='Use CAN FD (FdFrame). The interface MTU must be 72.'),
        DeclareLaunchArgument(
            'from_can_bus_topic', default_value='from_can_bus_fd',
            description='Topic with frames received from the CAN bus'),
        DeclareLaunchArgument(
            'to_can_bus_topic', default_value='to_can_bus_fd',
            description='Topic with frames to be sent on the CAN bus'),
        DeclareLaunchArgument(
            'receiver_interval_sec', default_value='0.05',
            description='Receiver poll window. MUST be larger than the '
                        'periodic status period (slave reports every 20ms), '
                        'otherwise the bridge logs "CAN Receive Timeout" '
                        'repeatedly. 0.05 avoids the spam.'),
        DeclareLaunchArgument(
            'sender_timeout_sec', default_value='0.05'),
        socketcan_bridge,
    ])
