"""隔离 ROS domain 验证参数提交与状态；不发布运动或设备命令。"""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String


SCRIPT = Path(__file__).resolve().parents[1]/'scripts'/'firmware_topic_bridge.py'
DOMAIN = 219


def _files(tmp_path):
    driver, urdf = tmp_path/'motor_driver.yaml', tmp_path/'robot.urdf'
    driver.write_text('motor_hardware:\n  motors:\n'
                      '    m2: {id: 2, joint: joint_link2}\n'
                      '    m3: {id: 3, joint: spin}\n', encoding='utf-8')
    urdf.write_text('<robot name="arm"><joint name="joint_link2" type="revolute">'
                    '<limit lower="-0.12" upper="0.14"/></joint>'
                    '<joint name="spin" type="continuous"/></robot>', encoding='utf-8')
    return ['-p', f'driver_file:={driver}', '-p', f'urdf_file:={urdf}', '-p', 'model_name:=arm']


def _payload(lower):
    return json.dumps({'model': 'arm', 'joint_limits': {'joint_link2': {'lower': lower, 'upper': .14}}})


def _wait(node, predicate):
    deadline = time.monotonic()+5.
    while time.monotonic() < deadline and not predicate():
        node.executor.spin_once(timeout_sec=.1)
    assert predicate()


def _set(node, client, *values, enabled=None):
    request = SetParametersAtomically.Request()
    request.parameters = [Parameter('joint_limits_json', value=value).to_parameter_msg() for value in values]
    if enabled is not None:
        request.parameters.append(Parameter('enable_trajectory', value=enabled).to_parameter_msg())
    future = client.call_async(request)
    _wait(node, future.done)
    return future.result().result


def test_committed_joint_limits_status_and_atomic_rejection(tmp_path, monkeypatch):
    env = {**os.environ, 'ROS_DOMAIN_ID': str(DOMAIN), 'ROS_LOCALHOST_ONLY': '1'}
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    context = Context()
    rclpy.init(args=[], context=context, domain_id=DOMAIN)
    node = Node('joint_limit_parameter_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    statuses = []
    node.create_subscription(String, '/robot_transmission/status',
                             lambda msg: statuses.append(json.loads(msg.data)),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    client = node.create_client(SetParametersAtomically, '/firmware_topic_bridge/set_parameters_atomically')
    with (tmp_path/'bridge.log').open('w') as log:
        process = subprocess.Popen([sys.executable, str(SCRIPT), '--ros-args', *_files(tmp_path)],
                                   env=env, stdout=log, stderr=subprocess.STDOUT)
        try:
            assert client.wait_for_service(timeout_sec=10.)
            _wait(node, lambda: bool(statuses))
            assert statuses[-1]['joint_limits']['joint_link2'] == {'lower': -.12, 'upper': .14}
            assert statuses[-1]['joint_limits']['spin'] == {'lower': None, 'upper': None}
            wrong_model = json.dumps({'model': 'other', 'joint_limits': {}})
            assert not _set(node, client, wrong_model, enabled=True).successful
            statuses.clear()
            _wait(node, lambda: bool(statuses))
            assert not statuses[-1]['trajectory_enabled']
            assert statuses[-1]['joint_limits']['joint_link2']['lower'] == -.12

            assert _set(node, client, _payload(-.5)).successful
            _wait(node, lambda: statuses[-1]['joint_limits']['joint_link2']['lower'] == -.5
                  and not statuses[-1]['trajectory_enabled'])
            assert not _set(node, client, _payload(-.8), wrong_model).successful
            assert not _set(node, client, _payload(float('-inf'))).successful
            statuses.clear()
            _wait(node, lambda: bool(statuses))
            assert statuses[-1]['joint_limits']['joint_link2']['lower'] == -.5

            assert _set(node, client, '{}', enabled=False).successful
            _wait(node, lambda: statuses[-1]['joint_limits']['joint_link2']['lower'] == -.12
                  and not statuses[-1]['trajectory_enabled'])
        finally:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5.)
            executor.shutdown()
            node.destroy_node()
            context.shutdown()


def test_invalid_startup_override_fails_before_status_timer(tmp_path):
    result = subprocess.run(
        [sys.executable, str(SCRIPT), '--ros-args', *_files(tmp_path),
         '-p', 'joint_limits_json:=\'{"model":"other","joint_limits":{}}\''],
        env={**os.environ, 'ROS_DOMAIN_ID': str(DOMAIN), 'ROS_LOCALHOST_ONLY': '1'},
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=10.)
    assert result.returncode != 0
    assert '关节限位模型与主机当前模型不一致' in result.stdout


def test_custom_limits_without_transmission_model_fail_at_startup():
    result = subprocess.run(
        [sys.executable, str(SCRIPT), '--ros-args', '-p', 'motor_ids:=[2]',
         '-p', 'joint_names:=[joint_link2]', '-p', f"joint_limits_json:='{_payload(-.5)}'"],
        env={**os.environ, 'ROS_DOMAIN_ID': str(DOMAIN), 'ROS_LOCALHOST_ONLY': '1'},
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=10.)
    assert result.returncode != 0
    assert '当前未加载传动模型，不能设置关节限位' in result.stdout
