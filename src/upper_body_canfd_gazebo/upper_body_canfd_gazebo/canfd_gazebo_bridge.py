"""Bridge two per-arm SocketCAN buses to Gazebo effort commands.

Each arm owns one CAN-FD bus and mixes two non-conflicting private protocols:

* RP IDs 1..4: shoulder yaw / roll / pitch + elbow pitch.
* 25/30 mm nodes N=1..3: wrist yaw / pitch / roll.

The node decodes motor packets, computes MIT actuator effort, publishes the
14 joint torques to ros2_control, and emits simulated motor feedback packets.
"""

from dataclasses import dataclass, field
import time
from typing import Dict

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from . import protocol_2530 as wrist_protocol
from . import protocol_rp as rp_protocol
from .socketcan import CanFrame, SocketCanBus


ARM_NAMES = ("left", "right")

RP_JOINT_SUFFIX_BY_ID = {
    1: "shoulder_yaw_joint",
    2: "shoulder_roll_joint",
    3: "shoulder_pitch_joint",
    4: "elbow_pitch_joint",
}

WRIST_JOINT_SUFFIX_BY_NODE = {
    1: "wrist_yaw_joint",
    2: "wrist_pitch_joint",
    3: "wrist_roll_joint",
}

ARM_JOINT_ORDER = [
    "shoulder_yaw_joint",
    "shoulder_roll_joint",
    "shoulder_pitch_joint",
    "elbow_pitch_joint",
    "wrist_yaw_joint",
    "wrist_pitch_joint",
    "wrist_roll_joint",
]

ALL_JOINTS = [f"{arm}_{suffix}" for arm in ARM_NAMES for suffix in ARM_JOINT_ORDER]


@dataclass
class JointSample:
    position: float = 0.0
    velocity: float = 0.0
    effort: float = 0.0


@dataclass
class RpDriveState:
    enabled: bool = False
    command: rp_protocol.RpMitCommand = field(default_factory=rp_protocol.RpMitCommand)
    last_command_time: float = 0.0


@dataclass
class WristDriveState:
    enabled: bool = False
    mode: int = wrist_protocol.MODE_MIT
    zero_offset: float = 0.0
    command: wrist_protocol.WristMitCommand = field(default_factory=wrist_protocol.WristMitCommand)
    last_command_time: float = 0.0
    current_a: float = 0.0
    # Heavier yaw/pitch axes default to 30 mm; roll defaults to 25 mm.
    motor_type: int = 0x01


def _clamp(value: float, limit: float) -> float:
    return min(max(value, -limit), limit)


class CanFdGazeboBridge(Node):
    def __init__(self) -> None:
        super().__init__("canfd_gazebo_bridge")

        self.declare_parameter("left_interface", "vcan0")
        self.declare_parameter("right_interface", "vcan1")
        self.declare_parameter("allow_hardware_interfaces", False)
        self.declare_parameter("control_rate_hz", 500.0)
        self.declare_parameter("status_rate_hz", 50.0)
        self.declare_parameter("command_timeout_s", 0.15)
        self.declare_parameter("rp_torque_limit_nm", 60.0)
        # The communication manual does not provide an output torque constant.
        # This value is therefore intentionally a tunable simulation parameter.
        self.declare_parameter("wrist_torque_constant_nm_per_amp", 0.35)
        self.declare_parameter("wrist_current_limit_a", 11.0)
        self.declare_parameter("wrist_torque_limit_nm", 5.0)
        self.declare_parameter("wrist_position_mode_gain_a_per_rad", 5.0)
        self.declare_parameter("wrist_position_mode_damping_a_per_rad_s", 0.08)
        self.declare_parameter("wrist_velocity_mode_gain_a_per_rad_s", 0.45)

        self._command_timeout = float(self.get_parameter("command_timeout_s").value)
        self._rp_torque_limit = float(self.get_parameter("rp_torque_limit_nm").value)
        self._wrist_kt = float(self.get_parameter("wrist_torque_constant_nm_per_amp").value)
        self._wrist_current_limit = float(self.get_parameter("wrist_current_limit_a").value)
        self._wrist_torque_limit = float(self.get_parameter("wrist_torque_limit_nm").value)
        self._position_gain = float(self.get_parameter("wrist_position_mode_gain_a_per_rad").value)
        self._position_damping = float(
            self.get_parameter("wrist_position_mode_damping_a_per_rad_s").value
        )
        self._velocity_gain = float(self.get_parameter("wrist_velocity_mode_gain_a_per_rad_s").value)

        interfaces = {
            "left": str(self.get_parameter("left_interface").value),
            "right": str(self.get_parameter("right_interface").value),
        }
        allow_hardware = bool(self.get_parameter("allow_hardware_interfaces").value)
        for interface in interfaces.values():
            if not interface.startswith("vcan") and not allow_hardware:
                raise RuntimeError(
                    f"Refusing to run a motor emulator on '{interface}'. "
                    "Set allow_hardware_interfaces:=true only for an intentional HIL setup."
                )

        self._buses: Dict[str, SocketCanBus] = {}
        try:
            for arm, interface in interfaces.items():
                self._buses[arm] = SocketCanBus(interface)
        except OSError as exc:
            for bus in self._buses.values():
                bus.close()
            raise RuntimeError(
                f"Cannot open SocketCAN interface. Create vcan0/vcan1 first: {exc}"
            ) from exc

        self._samples: Dict[str, JointSample] = {name: JointSample() for name in ALL_JOINTS}
        self._rp: Dict[str, Dict[int, RpDriveState]] = {
            arm: {motor_id: RpDriveState() for motor_id in RP_JOINT_SUFFIX_BY_ID}
            for arm in ARM_NAMES
        }
        self._wrist: Dict[str, Dict[int, WristDriveState]] = {
            arm: {
                1: WristDriveState(motor_type=0x01),
                2: WristDriveState(motor_type=0x01),
                3: WristDriveState(motor_type=0x02),
            }
            for arm in ARM_NAMES
        }

        self._effort_pub = self.create_publisher(
            Float64MultiArray, "/effort_controller/commands", 10
        )
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 50)

        control_rate = float(self.get_parameter("control_rate_hz").value)
        status_rate = float(self.get_parameter("status_rate_hz").value)
        if control_rate <= 0.0 or status_rate <= 0.0:
            raise ValueError("control_rate_hz and status_rate_hz must be positive")
        self.create_timer(1.0 / control_rate, self._control_tick)
        self.create_timer(1.0 / status_rate, self._status_tick)

        self.get_logger().info(
            "CAN-FD emulator ready: left=%s, right=%s; RP IDs 1..4 + wrist N 1..3 per bus"
            % (interfaces["left"], interfaces["right"])
        )

    def _joint_name(self, arm: str, suffix: str) -> str:
        return f"{arm}_{suffix}"

    def _on_joint_states(self, msg: JointState) -> None:
        for index, name in enumerate(msg.name):
            sample = self._samples.get(name)
            if sample is None:
                continue
            if index < len(msg.position):
                sample.position = float(msg.position[index])
            if index < len(msg.velocity):
                sample.velocity = float(msg.velocity[index])

    def _control_tick(self) -> None:
        now = time.monotonic()
        self._drain_can(now)
        self._calculate_efforts(now)

        output = Float64MultiArray()
        output.data = [self._samples[name].effort for name in ALL_JOINTS]
        self._effort_pub.publish(output)

    def _drain_can(self, now: float) -> None:
        for arm, bus in self._buses.items():
            for frame in bus.recv_available():
                self._handle_frame(arm, frame, now)

    def _handle_frame(self, arm: str, frame: CanFrame, now: float) -> None:
        if frame.is_extended_id:
            return

        # RP enable is a standard 8-byte frame, shared by the RP devices on this arm.
        if frame.arbitration_id == rp_protocol.ENABLE_ID and len(frame.data) >= 1:
            enabled = bool(frame.data[0])
            for drive in self._rp[arm].values():
                drive.enabled = enabled
            return

        # RP broadcasts 8 slots / 64 bytes. This robot uses the first 4 slots.
        if frame.arbitration_id == rp_protocol.MIT_BROADCAST_ID and len(frame.data) == 64:
            commands = rp_protocol.unpack_broadcast(frame.data)
            for motor_id, drive in self._rp[arm].items():
                drive.command = commands[motor_id - 1]
                drive.last_command_time = now
            self._send_rp_feedback(arm)
            return

        for node in WRIST_JOINT_SUFFIX_BY_NODE:
            if frame.arbitration_id == wrist_protocol.motor_control_id(node) and len(frame.data) == 8:
                self._handle_wrist_control(arm, node, frame.data)
                return
            if frame.arbitration_id == wrist_protocol.mit_command_id(node) and len(frame.data) == 12:
                drive = self._wrist[arm][node]
                drive.command = wrist_protocol.unpack_mit_command(frame.data)
                drive.last_command_time = now
                self._send_wrist_reply(arm, node)
                return

    def _handle_wrist_control(self, arm: str, node: int, data: bytes) -> None:
        drive = self._wrist[arm][node]
        run = bool(data[0])
        set_zero = bool(data[1])
        requested_mode = int(data[2])

        # Manual section 3.4: changing mode is accepted only while stopped.
        if not run and requested_mode in (
            wrist_protocol.MODE_MIT,
            wrist_protocol.MODE_VELOCITY,
            wrist_protocol.MODE_POSITION,
        ):
            drive.mode = requested_mode

        if set_zero and not run:
            joint_name = self._joint_name(arm, WRIST_JOINT_SUFFIX_BY_NODE[node])
            drive.zero_offset = self._samples[joint_name].position

        drive.enabled = run
        if not run:
            drive.current_a = 0.0

    def _calculate_efforts(self, now: float) -> None:
        for arm in ARM_NAMES:
            for motor_id, suffix in RP_JOINT_SUFFIX_BY_ID.items():
                joint_name = self._joint_name(arm, suffix)
                sample = self._samples[joint_name]
                drive = self._rp[arm][motor_id]
                torque = 0.0
                if drive.enabled and (now - drive.last_command_time) <= self._command_timeout:
                    command = drive.command
                    torque = (
                        command.kp * (command.position - sample.position)
                        + command.kd * (command.velocity - sample.velocity)
                        + command.torque_ff
                    )
                    torque = _clamp(torque, self._rp_torque_limit)
                sample.effort = torque

            for node, suffix in WRIST_JOINT_SUFFIX_BY_NODE.items():
                joint_name = self._joint_name(arm, suffix)
                sample = self._samples[joint_name]
                drive = self._wrist[arm][node]
                current = 0.0
                if drive.enabled and (now - drive.last_command_time) <= self._command_timeout:
                    command = drive.command
                    relative_position = sample.position - drive.zero_offset
                    if drive.mode == wrist_protocol.MODE_MIT:
                        current = (
                            wrist_protocol.kp_to_current_gain(command.kp)
                            * (command.position - relative_position)
                            + wrist_protocol.kd_to_current_gain(command.kd)
                            * (command.velocity - sample.velocity)
                            + command.current_ff
                        )
                    elif drive.mode == wrist_protocol.MODE_POSITION:
                        current = (
                            self._position_gain * (command.position - relative_position)
                            - self._position_damping * sample.velocity
                        )
                    elif drive.mode == wrist_protocol.MODE_VELOCITY:
                        current = self._velocity_gain * (command.velocity - sample.velocity)

                drive.current_a = _clamp(current, self._wrist_current_limit)
                sample.effort = _clamp(
                    drive.current_a * self._wrist_kt,
                    self._wrist_torque_limit,
                )

    def _send_rp_feedback(self, arm: str) -> None:
        bus = self._buses[arm]
        for motor_id, suffix in RP_JOINT_SUFFIX_BY_ID.items():
            sample = self._samples[self._joint_name(arm, suffix)]
            payload = rp_protocol.pack_state(
                sample.position,
                sample.velocity,
                sample.effort,
            )
            bus.send(motor_id, payload, force_fd=True)

    def _send_wrist_reply(self, arm: str, node: int) -> None:
        drive = self._wrist[arm][node]
        sample = self._samples[self._joint_name(arm, WRIST_JOINT_SUFFIX_BY_NODE[node])]
        payload = wrist_protocol.pack_mit_reply(
            node,
            sample.position - drive.zero_offset,
            sample.velocity,
            drive.current_a,
        )
        self._buses[arm].send(wrist_protocol.mit_reply_id(node), payload, force_fd=True)

    def _status_tick(self) -> None:
        for arm in ARM_NAMES:
            bus = self._buses[arm]
            for node, suffix in WRIST_JOINT_SUFFIX_BY_NODE.items():
                drive = self._wrist[arm][node]
                sample = self._samples[self._joint_name(arm, suffix)]
                payload = wrist_protocol.pack_status(
                    node,
                    sample.position - drive.zero_offset,
                    sample.velocity,
                    drive.current_a,
                    motor_state=4 if drive.enabled else 3,
                    mode=drive.mode,
                    motor_type=drive.motor_type,
                )
                bus.send(wrist_protocol.status_id(node), payload, force_fd=True)

    def destroy_node(self):  # type: ignore[override]
        for bus in self._buses.values():
            bus.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CanFdGazeboBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
