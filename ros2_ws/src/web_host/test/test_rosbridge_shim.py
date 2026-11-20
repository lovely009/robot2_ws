"""Exercise rosbridge with real generated standard Action request types."""
import pytest


def test_disconnect_during_wait_keeps_ros_callbacks_running(monkeypatch):
    from rclpy.context import Context
    from rclpy.exceptions import InvalidHandle
    from web_host.rosbridge_shim import _RosbridgeExecutor

    context = Context()
    context.init()
    executor = _RosbridgeExecutor(context=context)
    try:
        wait = executor._wait_for_ready_callbacks
        disconnected = False

        def disconnect(*args, **kwargs):
            nonlocal disconnected
            if not disconnected:
                disconnected = True
                raise InvalidHandle('subscription destroyed during wait set construction')
            yield from wait(*args, **kwargs)

        monkeypatch.setattr(executor, '_wait_for_ready_callbacks', disconnect)
        future = executor.create_task(lambda: 'next callback')
        executor.spin_once(timeout_sec=0)
        executor.spin_once(timeout_sec=0)
        assert future.result() == 'next callback'
    finally:
        executor.shutdown()
        context.shutdown()


def test_disconnect_after_wait_releases_event_and_callback_group():
    from rclpy.context import Context
    from rclpy.exceptions import InvalidHandle
    from rclpy.node import Node
    from web_host.rosbridge_shim import _RosbridgeExecutor

    context = Context()
    context.init()
    executor = _RosbridgeExecutor(context=context)
    node = Node('test_rosbridge_disconnect', context=context)
    guard = node.create_guard_condition(lambda: None)
    executed = []

    def take_destroyed(_):
        raise InvalidHandle('event destroyed before take_data')

    async def execute(_, value):
        executed.append(value)

    try:
        for take in (take_destroyed, lambda _: 'next callback'):
            task = executor._make_handler(guard, node, take, execute)
            task()
            task.result()
            assert not guard._executor_event
            assert guard.callback_group.can_execute(guard)
        assert executed == ['next callback']
    finally:
        executor.shutdown()
        node.destroy_node()
        context.shutdown()


def test_standard_trajectory_action_services_and_nested_payload_resolve():
    pytest.importorskip('rosbridge_library')
    from control_msgs.action import FollowJointTrajectory
    from rosbridge_library.internal import message_conversion, ros_loader
    from web_host.rosbridge_shim import _patch_standard_trajectory_action

    _patch_standard_trajectory_action()
    _patch_standard_trajectory_action()  # startup patch remains idempotent
    service = ros_loader.get_service_class('control_msgs/action/FollowJointTrajectory_SendGoal')
    assert service is FollowJointTrajectory.Impl.SendGoalService
    request = service.Request()
    payload = {'goal_id': {'uuid': list(range(16))}, 'goal': {
        'trajectory': {'joint_names': ['joint'], 'points': [
            {'positions': [.1], 'time_from_start': {'sec': 0, 'nanosec': 0}},
            {'positions': [.2], 'time_from_start': {'sec': 1, 'nanosec': 0}},
        ]}, 'path_tolerance': [], 'goal_tolerance': [],
        'goal_time_tolerance': {'sec': 1, 'nanosec': 0}}}
    message_conversion.populate_instance(payload, request)
    assert list(request.goal_id.uuid) == list(range(16))
    assert list(request.goal.trajectory.points[-1].positions) == [.2]
    result_type = ros_loader.get_service_class('control_msgs/action/FollowJointTrajectory_GetResult')
    result = result_type.Response(status=4, result=FollowJointTrajectory.Result(error_code=0))
    converted = message_conversion.extract_values(result)
    assert converted['status'] == 4 and converted['result']['error_code'] == 0


@pytest.mark.parametrize('origin,allowed', [
    ('http://localhost:8080', True), ('http://127.0.0.1:8080', True),
    ('http://[::1]:8080', True), ('http://LOCALHOST:8080', True),
    ('https://unrelated.invalid', False), ('http://localhost:8081', False),
    ('https://localhost:8080', False), ('http://user@localhost:8080', False),
    ('http://localhost:8080/path', False), ('null', False), (None, False),
])
def test_websocket_accepts_only_local_workbench_origin(origin, allowed):
    pytest.importorskip('rosbridge_server')
    from rosbridge_server.websocket_handler import RosbridgeWebSocket
    from web_host import rosbridge_shim
    rosbridge_shim._patch_workbench_origin(8080)
    handler = object.__new__(RosbridgeWebSocket)
    assert handler.check_origin(origin) is allowed


def test_origin_check_uses_configured_web_port():
    from rosbridge_server.websocket_handler import RosbridgeWebSocket
    from web_host import rosbridge_shim
    rosbridge_shim._patch_workbench_origin(8180)
    handler = object.__new__(RosbridgeWebSocket)
    assert handler.check_origin('http://localhost:8180')
    assert not handler.check_origin('http://localhost:8080')


# 只替代 ROS 连接后的通信，握手与来源门禁执行真实 Tornado/rosbridge 代码。
RosbridgeWebSocket = pytest.importorskip('rosbridge_server.websocket_handler').RosbridgeWebSocket
from tornado.testing import AsyncHTTPTestCase, gen_test
from tornado.web import Application
from tornado.httpclient import HTTPRequest, HTTPClientError
from tornado.websocket import websocket_connect


class TestOriginHandshake(AsyncHTTPTestCase):
    def get_app(self):
        from web_host.rosbridge_shim import _patch_workbench_origin
        _patch_workbench_origin(8080)
        class HandshakeOnly(RosbridgeWebSocket):
            def open(self):
                pass
            def on_close(self):
                pass
        return Application([(r'/', HandshakeOnly)])

    @gen_test
    async def test_only_workbench_origin_can_complete_handshake(self):
        url = self.get_url('/').replace('http:', 'ws:')
        connection = await websocket_connect(HTTPRequest(url, headers={'Origin': 'http://localhost:8080'}))
        connection.close()
        for origin in (None, 'http://unrelated.invalid', 'http://localhost:8081'):
            headers = {} if origin is None else {'Origin': origin}
            with self.assertRaises(HTTPClientError) as caught:
                connection = await websocket_connect(HTTPRequest(url, headers=headers))
                connection.close()
            assert caught.exception.code == 403
