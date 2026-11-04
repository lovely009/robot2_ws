"""真实 Humble lifecycle 回调；systemd 注入假体，不触碰设备或进程。"""
import json

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState

from conftest import FakeSystemd
from bench_manager import node as manager_module
from bench_manager import simulation_manager as simulation_module
from bench_manager.systemd_units import SystemdUnits


def test_repeated_lifecycle_request_keeps_bench_services_alive(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '197')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    systemd = SystemdUnits(runner=FakeSystemd())
    monkeypatch.setattr(manager_module, 'SystemdUnits', lambda: systemd)
    monkeypatch.setattr(manager_module, 'enumerate_serial_ports', lambda: [])
    monkeypatch.setattr(simulation_module, 'enumerate_can_interfaces', lambda: [])
    monkeypatch.setattr(simulation_module, 'enumerate_serial_ports', lambda: [])
    monkeypatch.setattr(manager_module, 'enumerate_simulation_models', lambda _root: [])
    monkeypatch.setattr(manager_module, 'enumerate_mujoco_models', lambda _root: [])
    rclpy.init(args=[])
    node = manager_module.BenchManager()
    try:
        for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
            response = node._service_change_state.callback(
                ChangeState.Request(transition=Transition(id=transition)),
                ChangeState.Response())
            assert response.success
        for transition in (Transition.TRANSITION_CONFIGURE,
                           Transition.TRANSITION_ACTIVATE, 255):
            response = node._service_change_state.callback(
                ChangeState.Request(transition=Transition(id=transition)),
                ChangeState.Response())
            assert not response.success
            state = node._service_get_state.callback(GetState.Request(), GetState.Response())
            assert state.current_state.id == State.PRIMARY_STATE_ACTIVE
            names = {service.srv_name for service in node.services}
            assert {'/bench/options', '/bench/agent_status'} <= names
        response = node._on_options(None, manager_module.GetBenchOptions.Response())
        assert json.loads(response.serial_ports_json) == []
        response = node._service_change_state.callback(
            ChangeState.Request(transition=Transition(id=Transition.TRANSITION_DEACTIVATE)),
            ChangeState.Response())
        assert response.success
    finally:
        node.destroy_node()
        rclpy.shutdown()
