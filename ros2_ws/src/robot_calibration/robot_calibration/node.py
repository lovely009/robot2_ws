"""Publish configuration/commissioning checks; never send a hardware command."""
from __future__ import annotations

import signal
import time
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from robot_system_msgs.msg import CalibrationState

from .validation import validate_configuration
from .diagnostics import calibration_diagnostic


class CalibrationValidator(Node):
    def __init__(self, **kwargs):
        super().__init__('calibration_validator', **kwargs)
        for name in ('driver_file', 'urdf_file', 'manifest_file', 'model_name', 'robot_id'):
            self.declare_parameter(name, '')
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._publisher = self.create_publisher(CalibrationState, '/robot/calibration_state', qos)
        self._diagnostics = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self._last_blockers = None
        self._last_diagnostics_at = -float('inf')
        self._last_diagnostics_signature = None
        self._timer = self.create_timer(0.5, self._check)
        self._check()

    def _check(self):
        values = {name: self.get_parameter(name).value for name in (
            'driver_file', 'urdf_file', 'manifest_file', 'model_name', 'robot_id')}
        result = validate_configuration(**values)
        output = CalibrationState()
        output.header.stamp = self.get_clock().now().to_msg()
        for name, value in result.asdict().items():
            setattr(output, name, value)
        self._publisher.publish(output)
        blockers = tuple(result.blockers)
        now = time.monotonic()
        signature = (result.valid, result.commissioned, result.robot_id,
                     result.model_name, result.calibration_id, result.config_hash, blockers)
        if (now - self._last_diagnostics_at >= 1.
                or (signature != self._last_diagnostics_signature
                    and now - self._last_diagnostics_at >= .5)):
            diagnostics = DiagnosticArray()
            diagnostics.header.stamp = output.header.stamp
            diagnostics.status = [calibration_diagnostic(
                result, node_name=self.get_fully_qualified_name().lstrip('/'))]
            self._diagnostics.publish(diagnostics)
            self._last_diagnostics_at = now
            self._last_diagnostics_signature = signature
        if blockers != self._last_blockers:
            if blockers:
                self.get_logger().warning('Startup checks block commissioning: ' + '; '.join(blockers))
            else:
                self.get_logger().info('Configuration matches the accepted commissioning record')
            self._last_blockers = blockers


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationValidator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
