"""SDK regression against local typed services; no MCU topics or hardware."""
from __future__ import annotations

import threading
import time
import uuid

import pytest

from robot_supervisor.client import ControlClient, ControlError, ControlTimeout, _parser


def test_cli_requires_explicit_lease_credentials():
    parser = _parser()
    with pytest.raises(SystemExit):
        parser.parse_args(['arm', '--lease-id', 'token', '--epoch', '42'])
    args = parser.parse_args([
        'renew', '--lease-id', 'token', '--epoch', str(2**40),
        '--session-id', str(2**41), '--lease-ms', '1000',
    ])
    assert args.epoch == 2**40
    assert args.session_id == 2**41


@pytest.fixture
def ros_fixture():
    rclpy = pytest.importorskip('rclpy')
    messages = pytest.importorskip('robot_system_msgs.msg')
    services = pytest.importorskip('robot_system_msgs.srv')
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from std_srvs.srv import Trigger

    context = Context()
    rclpy.init(args=[], context=context)
    namespace = '/foundation_client_test_' + uuid.uuid4().hex[:8]
    server = rclpy.create_node('client_test_server', context=context)
    node = rclpy.create_node('client_test_sdk', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(server)
    requests, received = [], []
    flags = {'reject': False}
    epoch, session_id = 2**42 + 1, 1_793_275_200_000

    def handler(name):
        def callback(request, response):
            requests.append((name, request))
            response.success = not flags['reject']
            if hasattr(response, 'reason'):
                response.reason = 'test_rejected' if flags['reject'] else ''
            if name == 'acquire_control':
                response.lease_id = 'test-lease'
                response.epoch, response.session_id = epoch, session_id
                response.lease_ms = request.lease_ms
            elif name == 'renew_control':
                response.lease_remaining_ms = request.lease_ms
            return response
        return callback

    for name, service in (
            ('acquire_control', services.AcquireControl),
            ('renew_control', services.RenewControl),
            ('release_control', services.ReleaseControl),
            ('authorize_motion', services.AuthorizeMotion),
            ('set_armed', services.SetArmed), ('stop', Trigger), ('reset_fault', Trigger)):
        server.create_service(service, f'{namespace}/{name}', handler(name))
    for channel, envelope in (
            ('command', messages.LeasedCommand), ('policy', messages.LeasedPolicy),
            ('trajectory', messages.LeasedTrajectory)):
        server.create_subscription(envelope, f'{namespace}/control/{channel}',
                                   lambda message, channel=channel: received.append((channel, message)), 1)
    state_publisher = server.create_publisher(messages.RobotState, f'{namespace}/state', 1)
    state = messages.RobotState()
    state.state, state.ready = state.READY, True
    state.epoch, state.session_id = epoch, session_id
    server.create_timer(0.02, lambda: state_publisher.publish(state))
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    client = ControlClient('test_owner', lease_ms=2000, namespace=namespace,
                           node=node, timeout_sec=1.0)
    try:
        yield type('Fixture', (), dict(
            client=client, node=node, server=server, executor=executor,
            namespace=namespace, requests=requests, received=received,
            flags=flags, epoch=epoch, session_id=session_id,
        ))
    finally:
        client.close()
        executor.shutdown(timeout_sec=1.0)
        thread.join(timeout=1.0)
        assert not thread.is_alive()
        node.destroy_node()
        server.destroy_node()
        context.shutdown()


def test_services_keep_full_width_session_and_do_not_renew_in_background(ros_fixture):
    fixture = ros_fixture
    client = fixture.client
    response = client.acquire()
    assert response.success
    assert (client.epoch, client.session_id) == (fixture.epoch, fixture.session_id)
    assert client.node.executor is None  # Temporary waits must release the node.
    client.arm()
    client.disarm()
    client.renew(1000)
    assert [name for name, _ in fixture.requests] == [
        'acquire_control', 'set_armed', 'set_armed', 'renew_control',
    ]
    time.sleep(0.04)
    assert len(fixture.requests) == 4
    client.release()
    assert client.lease_id == ''
    with pytest.raises(ControlError, match='acquire or bind'):
        client.arm()
    assert fixture.requests[-1][0] == 'release_control'


def test_envelopes_use_one_sequence_and_fresh_stamps_without_mutating_inputs(ros_fixture):
    from can_bridge_msgs.msg import MotorCommandArray, MotorPolicyArray, MotorTrajectory
    fixture = ros_fixture
    client = fixture.client
    client.acquire()
    command, policy, trajectory = MotorCommandArray(), MotorPolicyArray(), MotorTrajectory()
    policy.policy_generation = 17
    before = client.node.get_clock().now().nanoseconds
    outgoing = [client.publish_command(command), client.publish_policy(policy),
                client.publish_trajectory(trajectory)]
    assert [message.sequence for message in outgoing] == [1, 2, 3]
    assert all(message.session_id == fixture.session_id for message in outgoing)
    assert all(message.epoch == fixture.epoch and message.lease_id == 'test-lease'
               for message in outgoing)
    assert outgoing[0].command.sync_time_ns >= before
    assert outgoing[1].policy.sync_time_ns >= before
    assert outgoing[2].trajectory.session_id == fixture.session_id
    assert outgoing[1].policy.policy_generation == 17  # Supervisor assigns the final generation.
    assert command.sync_time_ns == policy.sync_time_ns == trajectory.sync_time_ns == 0
    assert policy.session_id == trajectory.session_id == 0
    deadline = time.monotonic() + 1.0
    while len(fixture.received) < 3 and time.monotonic() < deadline:
        time.sleep(0.01)
    assert sorted(channel for channel, _ in fixture.received) == ['command', 'policy', 'trajectory']
    # Explicitly bound credentials preserve a caller's prior sequence.
    client.bind_lease('test-lease', fixture.epoch, fixture.session_id, sequence=3)
    assert client.publish_command(command).sequence == 4


def test_attached_executor_is_never_spun_by_the_sdk(ros_fixture, monkeypatch):
    fixture = ros_fixture
    fixture.executor.add_node(fixture.node)
    from rclpy import executors
    def forbidden(*_args, **_kwargs):
        raise AssertionError('an existing executor must not be spun a second time')
    monkeypatch.setattr(executors, 'SingleThreadedExecutor', forbidden)
    assert fixture.client.acquire().success
    assert fixture.client.status().session_id == fixture.session_id


def test_discovery_timeout_is_bounded_and_rejection_keeps_no_lease(ros_fixture):
    fixture = ros_fixture
    fixture.flags['reject'] = True
    with pytest.raises(ControlError, match='test_rejected'):
        fixture.client.acquire()
    assert fixture.client.lease_id == ''
    missing = ControlClient('missing', namespace=fixture.namespace + '_missing',
                            node=fixture.node, timeout_sec=0.05)
    started = time.monotonic()
    try:
        with pytest.raises(ControlTimeout, match='unavailable'):
            missing.acquire()
        assert time.monotonic() - started < 0.5
    finally:
        missing.close()


def test_expired_lease_blocks_publication_and_stop_reset_need_no_lease(ros_fixture):
    from can_bridge_msgs.msg import MotorCommandArray
    fixture = ros_fixture
    fixture.client.acquire()
    fixture.client._lease_deadline = time.monotonic() - 1.0
    with pytest.raises(ControlError, match='expired'):
        fixture.client.publish_command(MotorCommandArray())
    assert not fixture.received
    assert fixture.client.stop().success
    assert fixture.client.lease_id == ''
    assert fixture.client.reset().success
    assert [name for name, _ in fixture.requests][-2:] == ['stop', 'reset_fault']


def test_motion_authorization_uses_global_sequence_and_never_arms_or_renews(ros_fixture):
    from can_bridge_msgs.msg import MotorCommandArray
    fixture = ros_fixture
    client = fixture.client
    client.mode = 2
    client.acquire()
    client.publish_command(MotorCommandArray())
    goal_id = uuid.uuid4()
    assert client.authorize_motion(goal_id).success
    name, request = fixture.requests[-1]
    assert name == 'authorize_motion'
    assert bytes(request.goal_id) == goal_id.bytes
    assert request.lease_id == client.lease_id
    assert (request.epoch, request.session_id, request.sequence) == (
        fixture.epoch, fixture.session_id, 2)
    assert [name for name, _ in fixture.requests] == ['acquire_control', 'authorize_motion']
    fixture.flags['reject'] = True
    with pytest.raises(ControlError, match='test_rejected'):
        client.authorize_motion(uuid.uuid4())
    assert client.sequence == 3
    with pytest.raises(ValueError, match='nonzero'):
        client.authorize_motion(bytes(16))
    assert client.sequence == 3


def test_standard_action_helper_sends_the_same_authorized_transport_uuid(ros_fixture):
    from control_msgs.action import FollowJointTrajectory
    from rclpy.action import ActionServer

    fixture = ros_fixture
    client = fixture.client
    client.mode = 2
    client.acquire()
    actual_ids = []

    def execute(handle):
        actual_ids.append(bytes(handle.goal_id.uuid))
        handle.succeed()
        return FollowJointTrajectory.Result(error_code=FollowJointTrajectory.Result.SUCCESSFUL)

    server = ActionServer(fixture.server, FollowJointTrajectory,
                          fixture.namespace + '/follow_joint_trajectory', execute_callback=execute)
    try:
        handle = client.follow_joint_trajectory(FollowJointTrajectory.Goal())
        assert handle.accepted
        future = handle.get_result_async()
        client._wait(future, time.monotonic() + 1.)
        assert future.done()
        assert actual_ids == [bytes(fixture.requests[-1][1].goal_id)]
        assert [name for name, _ in fixture.requests] == ['acquire_control', 'authorize_motion']
    finally:
        server.destroy()
