"""HIL 只验证坐标、同组门控与反馈，不打开 CAN、串口或物理后端。"""
from __future__ import annotations

import importlib.util
from pathlib import Path
from types import SimpleNamespace
import time

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[4]
SPEC = importlib.util.spec_from_file_location(
    "transmission_hil_sim", ROOT / "micro_ros_agent/tools/fdcan_motor_sim.py")
SIM = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SIM)


def test_hil_shared_transmission_preserves_joint_order_and_group_zeroing() -> None:
    driver = Path(__file__).resolve().parents[2] / "robot_description/models/half_body_robot/config/motor_driver.yaml"
    motors = yaml.safe_load(driver.read_text())["motor_hardware"]["motors"]
    joint_map = {m["id"]: m["joint"] for m in motors.values()}
    bridge = SIM.GazeboBridge.__new__(SIM.GazeboBridge)
    bridge.joint_map = joint_map
    bridge.joints = list(joint_map.values())
    order = list(reversed(bridge.joints))
    bridge.enable_transmission(driver, joint_names=order)
    assert bridge.joints == order
    bridge.control_mode = "effort"
    bridge.tau_limits = {}
    bridge.tau_limit = 10.0
    bridge.fw_lifecycle = {}
    bridge._msg_type = SimpleNamespace
    outputs = []
    bridge.pub = SimpleNamespace(publish=lambda msg: outputs.append(msg.data))
    now = time.monotonic()
    bridge.state_at = {j: now for j in bridge.joints}
    bridge.pos["joint_link8"] = 0.2
    bridge.pos["joint_link9"] = 0.1
    models = {mid: SIM.MotorModel(mid, "cerb", 1.0, 1.0) for mid in joint_map}
    for mid, target in ((8, 0.5), (9, 0.1)):
        motor = models[mid]
        motor.enabled = True
        motor.last_cmd_at = now
        motor.last_cmd = {"pos_rad": target, "vel_rad_s": 0.0,
                          "kp": 1.0, "kd": 0.0, "tau_nm": 0.0}
    assert bridge.joint_position(8) == pytest.approx(0.3)
    assert bridge.joint_position(9) == pytest.approx(0.1)
    bridge.publish_transmission_commands(models, now)
    assert outputs[-1][order.index("joint_link8")] == pytest.approx(0.2)
    assert outputs[-1][order.index("joint_link9")] == pytest.approx(0.2)
    assert models[8].applied_joint_nm == pytest.approx(0.2)
    models[9].enabled = False
    bridge.publish_transmission_commands(models, now)
    assert outputs[-1] == [0.0] * len(order)
    assert models[8].applied_joint_nm == 0.0
    models[9].enabled = True
    bridge.state_at["joint_link9"] = now - 0.6
    bridge.publish_transmission_commands(models, now)
    assert outputs[-1] == [0.0] * len(order)
    assert not bridge.feedback_available(8)
    bridge.state_at["joint_link9"] = now
    bridge.fw_lifecycle = {8: SIM.LIFECYCLE_ACTIVE, 9: 3}
    bridge.fw_status_at = now
    bridge.publish_transmission_commands(models, now)
    assert outputs[-1] == [0.0] * len(order)


def test_hil_position_dependent_effort_uses_current_actuator_positions() -> None:
    driver = Path(__file__).resolve().parents[2] / "robot_description/models/trial_arm_7dof/config/motor_driver.yaml"
    motors = yaml.safe_load(driver.read_text())["motor_hardware"]["motors"]
    joint_map = {m["id"]: m["joint"] for m in motors.values()}
    bridge = SIM.GazeboBridge.__new__(SIM.GazeboBridge)
    bridge.joint_map = joint_map
    bridge.joints = list(joint_map.values())
    bridge.enable_transmission(driver)
    bridge.control_mode = "effort"
    bridge.tau_limits = {}
    bridge.tau_limit = 10.0
    bridge.fw_lifecycle = {}
    bridge._msg_type = SimpleNamespace
    outputs = []
    bridge.pub = SimpleNamespace(publish=lambda msg: outputs.append(msg.data))
    now = time.monotonic()
    for joint, position in (("Joint_Link1", 0.17), ("Joint_Link2", 0.08)):
        bridge.pos[joint] = position
        bridge.state_at[joint] = now
    models = {mid: SIM.MotorModel(mid, "cerb", 1.0, 1.0) for mid in (1, 2)}
    actuator_torques = {}
    for mid, torque in ((1, 2.0), (2, -0.5)):
        motor = models[mid]
        motor.enabled = True
        motor.last_cmd_at = now
        motor.last_cmd = {"pos_rad": 0.0, "vel_rad_s": 0.0,
                          "kp": 0.0, "kd": 0.0, "tau_nm": torque}
        actuator_torques[bridge.motor_actuators[mid]] = torque
    measured = bridge.transmission.joint_to_actuator({
        joint: bridge.axis_type(bridge.pos[joint], 0.0, 0.0)
        for joint in ("Joint_Link1", "Joint_Link2")})
    actuator_positions = {name: axis.position for name, axis in measured.items()}
    expected = bridge.transmission.actuator_effort_to_joint(
        actuator_torques, actuator_positions=actuator_positions)
    zero_pose = bridge.transmission.joint_to_actuator({
        joint: bridge.axis_type(0.0, 0.0, 0.0)
        for joint in ("Joint_Link1", "Joint_Link2")})
    zero_pose_expected = bridge.transmission.actuator_effort_to_joint(
        actuator_torques,
        actuator_positions={name: axis.position for name, axis in zero_pose.items()})

    bridge.publish_transmission_commands(models, now)

    assert outputs[-1] == pytest.approx([expected.get(j, 0.0) for j in bridge.joints])
    assert outputs[-1] != pytest.approx(
        [zero_pose_expected.get(j, 0.0) for j in bridge.joints])
    assert models[1].applied_joint_nm == 2.0
    assert models[2].applied_joint_nm == -0.5

    bridge.state_at["Joint_Link2"] = now - 0.6
    bridge.publish_transmission_commands(models, now)
    assert outputs[-1] == [0.0] * len(bridge.joints)
    assert all(motor.applied_joint_nm == 0.0 for motor in models.values())
