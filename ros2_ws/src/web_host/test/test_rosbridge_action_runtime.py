"""Real local rosbridge/standard Action-service transport; no device access.

The process guard is explicitly lifted only for the rosbridge shim. The ROS
services below are test doubles on an isolated DDS domain, not motor drivers.
"""
from __future__ import annotations

import asyncio
import copy
import json
import os
import signal
import socket
import subprocess
import sys
import threading
import time

import pytest


REAL_POPEN = subprocess.Popen
rclpy = pytest.importorskip('rclpy')
pytest.importorskip('rosbridge_server')
pytest.importorskip('tornado.websocket')

from control_msgs.action import FollowJointTrajectory
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from robot_system_msgs.msg import LeasedPolicy
from std_msgs.msg import String
from tornado.websocket import websocket_connect
import yaml


ACTION = '/robot/follow_joint_trajectory/_action/'


def test_real_rosbridge_standard_goal_result_and_internal_publish_denial(monkeypatch, tmp_path):
    # This subprocess owns only a loopback WebSocket and fake ROS service
    # clients; it never launches an Agent, a controller or a hardware backend.
    monkeypatch.setattr(subprocess, 'Popen', REAL_POPEN)
    context = Context()
    rclpy.init(args=[], context=context, domain_id=205)
    node = Node('fake_action_services', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    requests, policies, forbidden = [], [], []

    def goal(request, response):
        requests.append(copy.deepcopy(request))
        response.accepted = True
        response.stamp.sec = 123
        return response

    def result(request, response):
        assert list(request.goal_id.uuid) == list(range(1, 17))
        response.status = 4
        response.result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        response.result.error_string = 'fake MCU trajectory finished'
        return response

    handles = [
        node.create_service(FollowJointTrajectory.Impl.SendGoalService, ACTION + 'send_goal', goal),
        node.create_service(FollowJointTrajectory.Impl.GetResultService, ACTION + 'get_result', result),
        node.create_subscription(LeasedPolicy, '/robot/control/policy', policies.append, 10),
        node.create_subscription(String, '/robot/managed/motor/command_array', forbidden.append, 10),
        node.create_subscription(String, '/micro_ros_do/low_cmd', forbidden.append, 10),
    ]
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    with socket.socket() as port_probe:
        port_probe.bind(('127.0.0.1', 0))
        port = port_probe.getsockname()[1]
    params = tmp_path / 'rosbridge.yaml'
    params.write_text(yaml.safe_dump({'/**': {'ros__parameters': {
        'port': port, 'address': '127.0.0.1', 'topics_glob': '[]',
        # The installed rosbridge parser expects Python-list strings.
        'topics_pub_glob': repr(['/robot/control/policy']),
        'topics_sub_glob': repr(['/robot/state']),
        'services_glob': repr([ACTION + 'send_goal', ACTION + 'get_result']),
        'call_services_in_new_thread': True,
    }}}), encoding='utf-8')
    environment = os.environ.copy()
    environment.update(ROS_DOMAIN_ID='205', ROS_LOCALHOST_ONLY='1')
    process = None
    log_path = tmp_path / 'rosbridge.log'

    async def exchange():
        ws = None
        until = time.monotonic() + 5.
        while time.monotonic() < until:
            if process.poll() is not None:
                raise AssertionError('rosbridge exited: ' + log_path.read_text()[-4000:])
            try:
                ws = await websocket_connect(f'ws://127.0.0.1:{port}', connect_timeout=.2)
                break
            except (ConnectionError, OSError):
                await asyncio.sleep(.05)
        assert ws is not None, 'rosbridge did not listen within five seconds'
        try:
            async def call(identifier, suffix, service_type, arguments):
                discovery_until = time.monotonic() + 2.
                while True:
                    ws.write_message(json.dumps(dict(op='call_service', id=identifier,
                        service=ACTION + suffix, type=service_type, args=arguments)))
                    while True:
                        raw = await ws.read_message()
                        assert raw is not None, 'WebSocket closed during service call'
                        message = json.loads(raw)
                        if message.get('id') == identifier and message.get('op') == 'service_response':
                            break
                    if message.get('result') is True:
                        return message['values']
                    # WebSocket readiness can precede DDS discovery. Only this
                    # explicit pre-dispatch rejection is retryable: no service
                    # request was sent, so it cannot duplicate a motion goal.
                    undiscovered = message.get('values') == f'Service {ACTION + suffix} does not exist'
                    if not undiscovered or time.monotonic() >= discovery_until:
                        raise AssertionError(message)
                    await asyncio.sleep(.05)

            uuid = {'uuid': list(range(1, 17))}
            response = await call('goal', 'send_goal',
                'control_msgs/action/FollowJointTrajectory_SendGoal', {
                    'goal_id': uuid, 'goal': {'trajectory': {'joint_names': ['elbow'], 'points': [
                        {'positions': [.1], 'time_from_start': {'sec': 0, 'nanosec': 0}},
                        {'positions': [.2], 'time_from_start': {'sec': 1, 'nanosec': 0}},
                    ]}, 'goal_time_tolerance': {'sec': 1, 'nanosec': 0}},
                })
            assert response['accepted'] is True
            assert response['stamp']['sec'] == 123
            response = await call('result', 'get_result',
                'control_msgs/action/FollowJointTrajectory_GetResult', {'goal_id': uuid})
            assert response['status'] == 4
            assert response['result']['error_code'] == 0
            assert response['result']['error_string'] == 'fake MCU trajectory finished'
            assert requests[0].goal.trajectory.joint_names == ['elbow']
            assert list(requests[0].goal.trajectory.points[1].positions) == [.2]
            assert len(requests) == 1

            ws.write_message(json.dumps(dict(op='publish', topic='/robot/control/policy',
                type='robot_system_msgs/msg/LeasedPolicy', msg={
                    'lease_id': 'transport-test', 'epoch': 17,
                    'session_id': 1_800_000_000_000, 'sequence': 1,
                    'policy': {'count': 0, 'targets': []},
                })))
            for topic in ('/robot/managed/motor/command_array', '/micro_ros_do/low_cmd'):
                for operation in ('advertise', 'publish'):
                    ws.write_message(json.dumps(dict(op=operation, topic=topic,
                        type='std_msgs/msg/String', msg={'data': 'must never reach DDS'})))
            until = time.monotonic() + 2.
            while not policies and time.monotonic() < until:
                await asyncio.sleep(.02)
            assert policies and policies[0].lease_id == 'transport-test'
            await asyncio.sleep(.2)
            assert forbidden == []
            assert node.count_publishers('/robot/managed/motor/command_array') == 0
            assert node.count_publishers('/micro_ros_do/low_cmd') == 0
        finally:
            ws.close()

    try:
        with log_path.open('w') as log:
            process = REAL_POPEN([sys.executable, '-m', 'web_host.rosbridge_shim',
                '--ros-args', '--params-file', str(params)],
                env=environment, stdout=log, stderr=subprocess.STDOUT)
            asyncio.run(asyncio.wait_for(exchange(), timeout=12.))
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.)
        executor.shutdown(timeout_sec=1.)
        thread.join(timeout=1.)
        node.destroy_node()
        context.shutdown()
    assert handles
