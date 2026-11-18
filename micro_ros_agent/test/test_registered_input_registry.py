"""后端注册表发布：从 models.keys() 映射到 MJCF 输入轴（joint_map 的值）。

纯逻辑测试：用 __new__ 跳过 rclpy 节点构造，只验证注册表名字与定期发布。
"""
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
from types import SimpleNamespace

import pytest

SCRIPT_PATH = Path(__file__).parents[1] / "tools" / "fdcan_motor_sim.py"
SPEC = spec_from_file_location("fdcan_motor_sim_registry", SCRIPT_PATH)
MODULE = module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class _Pub:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class _Clock:
    def now(self):
        return SimpleNamespace(to_msg=lambda: SimpleNamespace(sec=0, nanosec=0))


def _bridge(joint_map, motor_actuators=None):
    b = MODULE.GazeboBridge.__new__(MODULE.GazeboBridge)
    b.joint_map = dict(joint_map)
    if motor_actuators is not None:
        b.motor_actuators = dict(motor_actuators)
    b.registry_pub = _Pub()
    b.registry_interval = 0.0
    b._last_registry = 0.0
    b._joint_state_type = lambda: SimpleNamespace(
        header=SimpleNamespace(stamp=None), name=[])
    b.node = SimpleNamespace(get_clock=lambda: _Clock())
    return b


def test_registered_names_follow_models_keys_order():
    b = _bridge({1: "BN_Spine_yaw", 2: "waist_flange_plus_x",
                 3: "waist_flange_minus_x"})
    names = b.registered_input_names({3: object(), 1: object()})
    assert names == ["BN_Spine_yaw", "waist_flange_minus_x"]


def test_removed_motor_disappears_from_registry():
    b = _bridge({1: "a", 2: "b"})
    assert b.registered_input_names({1: object(), 2: object()}) == ["a", "b"]
    # remove id 1
    assert b.registered_input_names({2: object()}) == ["b"]


def test_registered_names_dedupes_shared_input_axis():
    # 两台电机意外映射到同一输入轴时只报一次，避免下游重复名。
    b = _bridge({1: "shared", 2: "shared"})
    assert b.registered_input_names({1: object(), 2: object()}) == ["shared"]


def test_publish_registry_publishes_jointstate_names():
    b = _bridge({1: "a", 2: "b"})
    b.publish_registry({1: object(), 2: object()})
    assert len(b.registry_pub.messages) == 1
    assert b.registry_pub.messages[0].name == ["a", "b"]
