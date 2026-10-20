"""命令控制器的行为测试。

发布端用替身捕获、定时器不 spin——只测本节点的裁决逻辑，不起进程、不碰串口/CAN。
"""

import json

import pytest
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Int32

from can_bridge_msgs.msg import MotorCommandArray, MotorCommandTarget
from motor_command.command_streamer import (CLIENT_COUNT_QOS, COMMAND_OUTPUT_QOS,
                                            SETPOINT_QOS, CommandStreamer)


class _Recorder:
    """替身 publisher：只记下发过什么。"""

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _setpoint(sequence=1, targets=((3, -1.0, 0.0, 0.0),)):
    message = MotorCommandArray()
    message.sync_time_ns = 1
    message.sequence = sequence
    message.count = len(targets)
    for motor_id, position, velocity, effort in targets:
        target = MotorCommandTarget()
        target.motor_id = motor_id
        target.position = position
        target.velocity = velocity
        target.effort = effort
        message.targets.append(target)
    return message


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def streamer():
    node = CommandStreamer()
    node.pub = _Recorder()
    node.status_pub = _Recorder()
    yield node
    node.destroy_node()


def _status(streamer):
    streamer._publish_status()
    return json.loads(streamer.status_pub.messages[-1].data)


def test_no_setpoint_means_no_output(streamer):
    """没收到过设定值就不发——台架保持未使能，不是"发一条空的"。"""
    for _ in range(5):
        streamer._tick()
    assert streamer.pub.messages == []
    assert _status(streamer)["reason"] == "no_setpoint"


def test_each_tick_repeats_the_same_snapshot(streamer):
    streamer._on_setpoint(_setpoint(targets=((3, -1.0, 0.0, 0.25),)))
    for _ in range(3):
        streamer._tick()
    assert len(streamer.pub.messages) == 3
    frames = streamer.pub.messages
    # 内容逐帧相同：固件按快照受理，重复发同一目标是"保持"，不是新证据
    assert all(f.count == 1 for f in frames)
    assert all(f.targets[0].motor_id == 3 for f in frames)
    assert all(f.targets[0].effort == pytest.approx(0.25) for f in frames)
    # 帧序号自增（只作日志关联），固件侧的序号门 2026-09-15 已移除
    assert [f.sequence for f in frames] == [1, 2, 3]
    assert all(f.sync_time_ns > 0 for f in frames)


def test_new_setpoint_takes_effect_on_the_next_tick(streamer):
    streamer._on_setpoint(_setpoint(sequence=7, targets=((3, -1.0, 0.0, 0.0),)))
    streamer._tick()
    streamer._on_setpoint(_setpoint(sequence=8, targets=((3, -0.5, 0.0, 0.0),)))
    streamer._tick()
    assert streamer.pub.messages[-1].targets[0].position == pytest.approx(-0.5)
    assert _status(streamer)["setpoint_sequence"] == 8


def test_empty_target_list_is_streamed(streamer):
    """空目标列表是"全部软停"这条真消息，必须照发——显式停止不靠断流。"""
    streamer._on_setpoint(_setpoint(targets=()))
    streamer._tick()
    assert len(streamer.pub.messages) == 1
    assert streamer.pub.messages[0].count == 0


def test_client_zero_stops_the_stream_and_it_resumes(streamer):
    streamer._on_setpoint(_setpoint())
    streamer._tick()
    assert len(streamer.pub.messages) == 1

    streamer._on_client_count(Int32(data=0))
    streamer._tick()
    assert len(streamer.pub.messages) == 1, "客户端归零后不该继续发"
    assert _status(streamer)["reason"] == "no_client"

    streamer._on_client_count(Int32(data=1))
    streamer._tick()
    assert len(streamer.pub.messages) == 2, "重连后应自动续流"


def test_unknown_client_count_does_not_block(streamer):
    """没有 rosbridge 就没有 /client_count——命令行驱动时不能被它拦住。"""
    streamer._on_setpoint(_setpoint())
    streamer._tick()
    assert len(streamer.pub.messages) == 1
    assert _status(streamer)["clients"] is None


def test_snapshot_the_firmware_would_reject_is_refused_early(streamer):
    """count 与长度不一致、或含 NaN/Inf 的快照固件是**整条拒收**，别往总线上白发。"""
    streamer._on_setpoint(_setpoint())
    streamer._tick()

    bad_count = _setpoint()
    bad_count.count = 2
    streamer._on_setpoint(bad_count)
    streamer._tick()
    status = _status(streamer)
    assert status["reject"] == "count=2 与 targets 长度 1 不一致"
    assert status["rejected_setpoints"] == 1
    # 拒收不覆盖上一条有效设定值：旧目标继续跑，而不是悄悄变成"没目标"
    assert streamer.pub.messages[-1].targets[0].position == pytest.approx(-1.0)
    assert status["streaming"] is True

    bad_nan = _setpoint(targets=((3, float("nan"), 0.0, 0.0),))
    streamer._on_setpoint(bad_nan)
    streamer._tick()
    assert "非有限值" in _status(streamer)["reject"]
    assert streamer.pub.messages[-1].targets[0].position == pytest.approx(-1.0)

    # 拒收是事件、不是状态：下一拍 reason 仍是 ok，所以它单独粘住
    assert _status(streamer)["reason"] == "ok"
    assert _status(streamer)["rejected_setpoints"] == 2


def test_status_reports_stream_state(streamer):
    streamer._on_setpoint(_setpoint(sequence=42, targets=((3, 0.0, 0.0, 0.0),)))
    streamer._tick()
    status = _status(streamer)
    assert status["streaming"] is True
    assert status["reason"] == "ok"
    assert status["stream_hz"] == pytest.approx(100.0)
    assert status["frames"] == 1
    assert status["setpoint_sequence"] == 42
    assert status["targets"] == 1
    assert status["setpoint_age_s"] >= 0.0


def test_command_output_qos_is_reliable():
    """命令流在主机内走：bridge 订阅端是 reliable，节拍器必须同一条判据。

    也顺带修掉叠加路线：overlay 的命令输入订阅是 reliable，发 best-effort
    在 ROS 2 里**不建连接**（静默失效）。
    """
    assert COMMAND_OUTPUT_QOS.reliability == ReliabilityPolicy.RELIABLE
    assert COMMAND_OUTPUT_QOS.durability == DurabilityPolicy.VOLATILE
    assert COMMAND_OUTPUT_QOS.depth == 1


def test_setpoint_qos_is_reliable():
    """设定值丢一条就会拿旧目标继续跑，必须 reliable（同固件对 trajectory/policy）。"""
    assert SETPOINT_QOS.reliability == ReliabilityPolicy.RELIABLE
    assert SETPOINT_QOS.durability == DurabilityPolicy.VOLATILE
    assert SETPOINT_QOS.depth == 1


def test_client_count_qos_matches_the_rosbridge_publisher():
    """客户端数的订阅必须与 rosbridge 的 publisher 同为 TRANSIENT_LOCAL。

    那个 publisher 是 latched（client_manager.py 的 ClientManager），VOLATILE 订阅者
    收不到迟到的锁存样本——浏览器已连着时启动本节点就会读不到当前值，require_client
    在启动那一刻失效。这条断言锁住的是"启动即可见"这个行为的前提。
    """
    assert CLIENT_COUNT_QOS.durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert CLIENT_COUNT_QOS.reliability == ReliabilityPolicy.RELIABLE
    assert CLIENT_COUNT_QOS.depth == 1


def test_output_topic_switches_at_runtime():
    """重力补偿开关切接线时改的是本节点的输出话题（ADR 0014），不重起进程。

    用真实发布者断言 topic_name——换话题是"发布者被换掉"，替身捕获看不出这件事。
    """
    node = CommandStreamer()
    try:
        assert node.pub.topic_name == "/motor/command_array"

        result = node._on_set_parameters(
            [Parameter("output_topic", value="/robot_dynamics/command_array")])
        assert result.successful
        # 推迟到下一拍才换：回调跑在参数值落定之前，那里动发布者会让
        # "参数已改"与"实际发往哪"短暂不一致
        assert node.pub.topic_name == "/motor/command_array"
        node._tick()
        assert node.pub.topic_name == "/robot_dynamics/command_array"
        assert node.output_topic == "/robot_dynamics/command_array"

        # 切回来（关开关走同一条路）
        node._on_set_parameters(
            [Parameter("output_topic", value="/motor/command_array")])
        node._tick()
        assert node.pub.topic_name == "/motor/command_array"
    finally:
        node.destroy_node()


def test_malformed_output_topic_is_refused(streamer):
    """空串或相对名会让命令发到没人订阅的话题上——参数面就拒掉，别等断流。"""
    for bad in ("", "robot_dynamics/command_array", 3):
        result = streamer._on_set_parameters(
            [Parameter("output_topic", value=bad)])
        assert not result.successful, bad
        assert streamer._pending_output_topic is None
    # 被拒的设定值不改变现状：还是发原话题
    streamer._tick()
    assert streamer.output_topic == "/motor/command_array"


def test_status_reports_the_current_output_topic(streamer):
    """页面要能看见命令当前发往哪里——断链时不静默换路，就得让它可核对。"""
    assert _status(streamer)["output_topic"] == "/motor/command_array"
