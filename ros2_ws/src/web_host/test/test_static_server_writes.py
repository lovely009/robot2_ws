"""真实 HTTP 写请求处理器的回归：设备与 ROS 使用替身，不开端口或设备。"""
from __future__ import annotations

import importlib.util
import io
import json
import sys
from email.message import Message
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock

import pytest

from web_host.actions import read_action, save_action


@pytest.fixture
def handler(monkeypatch, tmp_path):
    web_root = tmp_path / 'web'
    web_root.mkdir()
    parameters = {'web_root': str(web_root)}
    node = Mock()
    node.declare_parameter.side_effect = lambda name, default: parameters.setdefault(name, default)
    node.get_parameter.side_effect = lambda name: SimpleNamespace(value=parameters[name])
    rclpy = ModuleType('rclpy')
    rclpy.init = Mock()
    rclpy.ok = lambda: False
    node_module = ModuleType('rclpy.node')
    node_module.Node = lambda _name: node
    models = ModuleType('web_host.models')
    for name in ('enumerate_arm_models', 'model_transmissions', 'motor_library',
                 'transmission_model_joints'):
        setattr(models, name, Mock())
    for name, module in [('rclpy', rclpy), ('rclpy.node', node_module),
                         ('web_host.models', models)]:
        monkeypatch.setitem(sys.modules, name, module)
    source = Path(__file__).resolve().parents[1] / 'web_host' / 'static_server.py'
    spec = importlib.util.spec_from_file_location('web_host._http_write_test', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    captured = []
    server = Mock()
    module.ThreadingHTTPServer = lambda _address, cls: captured.append(cls) or server
    monkeypatch.setattr(module.signal, 'signal', Mock())
    module.main()

    def post(path, payload, *, origin='http://localhost:8080',
             host='localhost:8080', content_type='application/json'):
        request = object.__new__(captured[0])
        request.path = path
        request.headers = Message()
        request.headers['Host'] = host
        if origin is not None:
            request.headers['Origin'] = origin
        request.headers['Content-Type'] = content_type
        body = json.dumps(payload).encode()
        request.headers['Content-Length'] = str(len(body))
        request.rfile = io.BytesIO(body)
        replies = []
        request._send_json = lambda status, result: replies.append((status, result))
        request.do_POST()
        assert len(replies) == 1
        return replies[0]

    return SimpleNamespace(post=post, web_root=web_root)


def _action(position):
    return json.dumps({'target_ids': [1], 'points': [
        {'motor_id': 1, 'time_from_start_ms': 0, 'position': position}]})


def test_package_import_defers_service_dependencies_and_preserves_main(monkeypatch):
    source = Path(__file__).resolve().parents[1] / 'web_host' / '__init__.py'
    spec = importlib.util.spec_from_file_location('web_host', source)
    module = importlib.util.module_from_spec(spec)
    # A fresh import must work without ROS or the HTTP service, while the public
    # entry point still forwards the caller's arguments when it is invoked.
    monkeypatch.setitem(sys.modules, 'rclpy', None)
    monkeypatch.setitem(sys.modules, 'web_host.static_server', None)
    spec.loader.exec_module(module)
    service = ModuleType('web_host.static_server')
    service.main = Mock(return_value='served')
    monkeypatch.setitem(sys.modules, 'web_host.static_server', service)
    assert module.main(args=['--ros-args']) == 'served'
    service.main.assert_called_once_with(args=['--ros-args'])


def test_cross_origin_simple_post_cannot_overwrite_action(handler):
    original = _action(0.1)
    save_action(handler.web_root, 'motion.json', original)
    status, _ = handler.post('/api/actions', {
        'name': 'motion.json', 'text': _action(0.8), 'overwrite': True,
    }, origin='http://foreign.example', content_type='text/plain')
    assert status == 403
    assert read_action(handler.web_root, 'motion.json') == original


@pytest.mark.parametrize('path', ['/api/actions', '/api/device-can/heartbeat'])
@pytest.mark.parametrize('origin', [
    'http://foreign.example:8080', 'http://localhost:8081',
    'https://localhost:8080', 'null', '', 'http://localhost:invalid',
    'http://user@localhost:8080', 'http://localhost:8080/path',
])
def test_foreign_or_malformed_origin_rejected_before_write(handler, path, origin):
    status, _ = handler.post(path, {}, origin=origin)
    assert status == 403
    assert not (handler.web_root.parent / 'action_library').exists()


@pytest.mark.parametrize('path', ['/api/actions', '/api/device-can/heartbeat'])
@pytest.mark.parametrize('content_type', ['text/plain', 'application/x-www-form-urlencoded'])
def test_json_required_for_all_write_endpoints(handler, path, content_type):
    status, _ = handler.post(path, {}, content_type=content_type)
    assert status == 415


@pytest.mark.parametrize('origin,host', [
    ('http://localhost:8080', 'localhost:8080'),
    ('http://LOCALHOST', 'localhost:80'),
    ('http://localhost:80', 'localhost'),
    ('http://[::1]:8080', '[::1]:8080'),
    (None, 'localhost:8080'),
])
def test_same_origin_or_cli_json_can_save_and_overwrite(handler, origin, host):
    for position in (0.1, 0.8):
        status, response = handler.post('/api/actions', {
            'name': 'motion.json', 'text': _action(position), 'overwrite': True,
        }, origin=origin, host=host, content_type='application/json; charset=utf-8')
        assert status == 200
        assert response['ok'] is True
        saved = json.loads(read_action(handler.web_root, 'motion.json'))
        assert saved['points'][0]['position'] == position


@pytest.mark.parametrize('origin', ['http://localhost:8080', None])
def test_device_session_json_request_cannot_reopen_retired_direct_path(handler, origin):
    status, result = handler.post('/api/device-can/heartbeat', {'session': 'token'}, origin=origin)
    assert status == 410
    assert 'micro-ROS' in result['error']
