"""接口契约及生成消息的真实序列化；不启动节点、不访问设备。"""
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
EPOCH_MILLISECONDS = 1_793_275_200_000


def declarations(relative_path):
    return [
        line.split('#', 1)[0].strip()
        for line in (ROOT / relative_path).read_text(encoding='utf-8').splitlines()
        if line.split('#', 1)[0].strip()
    ]


def test_mcu_sessions_do_not_truncate_epoch_milliseconds():
    # The wire session is epoch milliseconds, already larger than uint32.
    for path in ('msg/RobotState.msg', 'msg/LeasedCommand.msg',
                 'msg/LeasedPolicy.msg', 'msg/LeasedTrajectory.msg',
                 'srv/AcquireControl.srv'):
        fields = declarations(path)
        assert 'uint64 session_id' in fields
        assert 'uint32 session_id' not in fields


@pytest.mark.parametrize('name,payload', [
    ('LeasedCommand', 'MotorCommandArray command'),
    ('LeasedPolicy', 'MotorPolicyArray policy'),
    ('LeasedTrajectory', 'MotorTrajectory trajectory'),
])
def test_motion_envelopes_share_the_lease_contract(name, payload):
    assert declarations(f'msg/{name}.msg') == [
        'std_msgs/Header header', 'string lease_id', 'uint64 epoch',
        'uint64 session_id', 'uint64 sequence', f'can_bridge_msgs/{payload}',
    ]


def test_generated_messages_round_trip_full_width_session_and_payload():
    messages = pytest.importorskip('robot_system_msgs.msg')
    serialization = pytest.importorskip('rclpy.serialization')
    from can_bridge_msgs.msg import MotorCommandTarget

    source = messages.LeasedCommand()
    source.lease_id = 'one-control-lease'
    source.epoch = EPOCH_MILLISECONDS + 1
    source.session_id = EPOCH_MILLISECONDS
    source.sequence = 2**32 + 7
    source.command.count = 1
    target = MotorCommandTarget()
    target.motor_id = 47
    target.position = 0.25
    source.command.targets = [target]
    restored = serialization.deserialize_message(
        serialization.serialize_message(source), messages.LeasedCommand)
    assert restored == source
    assert restored.session_id == EPOCH_MILLISECONDS
    assert restored.command.targets[0].position == 0.25


def test_generated_services_preserve_modes_and_full_width_session():
    services = pytest.importorskip('robot_system_msgs.srv')
    serialization = pytest.importorskip('rclpy.serialization')
    request = services.AcquireControl.Request()
    assert (request.MANUAL, request.TRAJECTORY, request.TASK) == (1, 2, 3)
    request.owner = 'motion_executor'
    request.mode = request.TRAJECTORY
    request.lease_ms = 1000
    response = services.AcquireControl.Response()
    response.success = True
    response.lease_id = 'one-control-lease'
    response.epoch = EPOCH_MILLISECONDS + 1
    response.session_id = EPOCH_MILLISECONDS
    response.lease_ms = request.lease_ms
    restored = serialization.deserialize_message(
        serialization.serialize_message(response), services.AcquireControl.Response)
    assert restored == response
    # All remaining service types must have generated runtime support as well.
    for name in ('RenewControl', 'ReleaseControl', 'SetArmed'):
        service = getattr(services, name)
        assert serialization.deserialize_message(
            serialization.serialize_message(service.Request()), service.Request
        ) == service.Request()
