"""命令控制器：把「设定值」变成固定节拍的命令快照流。

节拍必须由主机侧进程持有：网页 JS 的定时器在标签页失去焦点时被浏览器钳到 1 Hz
（实测前台 88 Hz / 后台 1.0 Hz），而固件的快照新鲜度线是 30 ms 软停 / 100 ms 失能
——页面一失焦台架就整臂掉力。理由与验收见 README.md。
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Int32, String

from can_bridge_msgs.msg import MotorCommandArray

# 出·流：与固件对 motor_command_array 的订阅（lib 的 qos_stream()）同一条判据——
# 高频流、latest-wins，depth 1 保证不积压过期快照。订阅方要用 best-effort 才收得到
# （ros2 topic echo --qos-reliability best_effort）；常态观察看 ~/status。
STREAM_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# 入·设定值：低频状态声明，**不允许静默丢**——丢一条就会拿旧目标继续跑。与固件对
# trajectory / policy 的处理同判据（qos_reliable(1)）。发布侧必须 reliable。
SETPOINT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class CommandStreamer(Node):
    """持有最近一次设定值，按 stream_hz 出完整快照。

    `count == 0`（空目标列表）**照发**：完整快照语义下它是"全部软停"这条真消息，
    显式停止不该靠断流实现。
    """

    def __init__(self):
        super().__init__("command_streamer")
        self.stream_hz = float(self.declare_parameter("stream_hz", 100.0).value)
        if self.stream_hz <= 0.0:
            raise ValueError("stream_hz 必须为正")
        setpoint_topic = self.declare_parameter(
            "setpoint_topic", "/motor_command/setpoint").value
        output_topic = self.declare_parameter(
            "output_topic", "/micro_ros_do/motor_command_array").value
        client_count_topic = self.declare_parameter(
            "client_count_topic", "/client_count").value
        status_topic = self.declare_parameter(
            "status_topic", "/motor_command/status").value
        # 客户端归零即停流（网页关了）。**没出现过 /client_count 时保持 None 不拦**：
        # 没有 rosbridge 就没有网页这条路，操作者只可能从命令行驱动。
        self.require_client = bool(
            self.declare_parameter("require_client", True).value)

        self.setpoint = None
        self.setpoint_sequence = 0
        self.setpoint_at = None
        # 不叫 clients：那是 rclpy Node 的只读属性（本节点的 service client 列表）。
        self.connected_clients = None
        self.frames = 0
        # reason 是**状态**（为什么现在没在发），由每拍的 _tick 决定；拒收是**事件**，
        # 单独粘住——否则被下一拍的 "ok" 冲掉，操作者永远看不到设定值没生效。
        self.reason = "no_setpoint"
        self.reject_reason = None
        self.rejected_setpoints = 0

        self.create_subscription(MotorCommandArray, setpoint_topic,
                                 self._on_setpoint, SETPOINT_QOS)
        self.create_subscription(Int32, client_count_topic,
                                 self._on_client_count, SETPOINT_QOS)
        self.pub = self.create_publisher(MotorCommandArray, output_topic,
                                         STREAM_QOS)
        self.status_pub = self.create_publisher(String, status_topic,
                                                SETPOINT_QOS)
        self.create_timer(1.0 / self.stream_hz, self._tick)
        self.create_timer(1.0, self._publish_status)

    def _on_client_count(self, msg):
        self.connected_clients = int(msg.data)

    def _on_setpoint(self, msg):
        error = self._validate(msg)
        if error:
            self.reject_reason = error
            self.rejected_setpoints += 1
            self.get_logger().warning(
                f"设定值拒收（{error}）——不发到总线，继续沿用上一条")
            return
        self.reject_reason = None
        self.setpoint = msg
        self.setpoint_sequence = int(msg.sequence)
        self.setpoint_at = self.get_clock().now()

    @staticmethod
    def _validate(msg):
        """挡住固件会整条拒收的快照，把静默失败变成看得见的状态。

        固件对 NaN/Inf 是**整快照拒收**（MotorCommandArray.msg 的 TBD-ROS-01=A），
        照发只会让总线白跑；这里先拦下来并记在 status 里。
        """
        if msg.count != len(msg.targets):
            return f"count={msg.count} 与 targets 长度 {len(msg.targets)} 不一致"
        for target in msg.targets:
            for name in ("position", "velocity", "effort"):
                if not math.isfinite(getattr(target, name)):
                    return f"m{target.motor_id} 的 {name} 非有限值"
        return None

    def _tick(self):
        if self.setpoint is None:
            self.reason = "no_setpoint"
            return
        if self.require_client and self.connected_clients == 0:
            self.reason = "no_client"
            return
        self.reason = "ok"
        self.frames += 1
        out = MotorCommandArray()
        # 每帧盖当前时刻与自增序号：两个字段都只作日志关联（固件 freshness 走本地
        # 单调时基、序号门 2026-09-15 已移除），重放旧值会让日志对齐失真。
        out.sync_time_ns = self.get_clock().now().nanoseconds
        out.sequence = self.frames & 0xFFFFFFFF
        out.count = self.setpoint.count
        out.targets = list(self.setpoint.targets)
        self.pub.publish(out)

    def _publish_status(self):
        now = self.get_clock().now()
        age = (None if self.setpoint_at is None
               else (now - self.setpoint_at).nanoseconds / 1e9)
        self.status_pub.publish(String(data=json.dumps({
            "streaming": self.reason == "ok",
            "reason": self.reason,
            "reject": self.reject_reason,
            "rejected_setpoints": self.rejected_setpoints,
            "stream_hz": self.stream_hz,
            "frames": self.frames,
            "clients": self.connected_clients,
            "setpoint_sequence": self.setpoint_sequence,
            "setpoint_age_s": None if age is None else round(age, 3),
            "targets": 0 if self.setpoint is None else int(self.setpoint.count),
        }, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = CommandStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
