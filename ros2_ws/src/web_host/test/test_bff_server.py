"""Comprehensive unit, security, recorder, and real ROS 2 integration tests for web_host.bff."""
from __future__ import annotations

import asyncio
import json
from pathlib import Path
import socket
import threading
import time

from fastapi.testclient import TestClient
import pytest
import websockets

from web_host.bff.app import create_app
from web_host.bff.lease import ControlLeaseManager
from web_host.bff.recorder import RosbagRecorder
from web_host.bff.ros_gateway import MockRosGateway, RosGatewayThread


def _free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(('127.0.0.1', 0))
        return int(sock.getsockname()[1])


def test_lease_lifecycle_pin_preemption_and_disconnect(tmp_path):
    gw = MockRosGateway()
    app = create_app(
        ros_gateway=gw,
        web_root=tmp_path / 'web',
        motion_data_dir=tmp_path / 'data',
        recordings_dir=tmp_path / 'rec',
        pin='2468',
        lease_ttl_s=2.0,
    )
    with TestClient(app) as client:
        assert gw.client_counts == [0]
        assert client.get('/api/control/lease').json()['active_controllers'] == 0

        assert client.post('/api/control/lease/acquire', json={
            'client_id': 'c1', 'pin': 'wrong',
        }).status_code == 403

        r1 = client.post('/api/control/lease/acquire', json={
            'client_id': 'c1', 'label': 'Pad-1', 'pin': '2468',
        })
        assert r1.status_code == 200
        d1 = r1.json()
        assert d1['lease']['active_controllers'] == 1
        assert gw.client_counts[-1] == 1

        # Conflict without force
        assert client.post('/api/control/lease/acquire', json={
            'client_id': 'c2', 'pin': '2468',
        }).status_code == 409

        # Force preemption stops BEFORE granting new owner
        gw.events_log.clear()
        r2 = client.post('/api/control/lease/acquire', json={
            'client_id': 'c2', 'label': 'Pad-2', 'force': True, 'pin': '2468',
        })
        assert r2.status_code == 200
        d2 = r2.json()
        assert d2['preempted_previous'] is True
        assert d2['lease']['epoch'] > d1['lease']['epoch']
        assert gw.events_log[0] == 'stop'

        # Owner WS disconnect drops /client_count 1->0 while viewers remain connected
        with client.websocket_connect('/ws?client_id=viewer-1') as v1:
            _ = v1.receive_json()
            with client.websocket_connect('/ws?client_id=c2') as owner_ws:
                _ = owner_ws.receive_json()
            assert gw.client_counts[-1] == 0
            assert client.get('/api/control/lease').json()['connected_clients'] == 1


def test_lease_ttl_expiration():
    now_mono = [50.0]
    counts: list[int] = []
    stops: list[str] = []
    mgr = ControlLeaseManager(
        ttl_s=1.0,
        clock=lambda: now_mono[0],
        wall_ms=lambda: int(now_mono[0] * 1000),
        on_active_count_changed=counts.append,
        on_stop_requested=stops.append,
    )
    _state, token, _ = mgr.acquire('c1', 'Pad')
    assert counts == [1]
    now_mono[0] += 0.5
    assert mgr.heartbeat('c1', token) is True
    assert mgr.expire_if_needed() is False
    now_mono[0] += 1.05
    assert mgr.expire_if_needed() is True
    assert counts == [1, 0]
    assert stops == ['lease_timeout']


def test_write_gating_bench_and_readonly_modes(tmp_path):
    gw = MockRosGateway()
    gw.register_service_handler('/motor/get_device_info', lambda req: {'topology': [], 'id': req['motor_id']})
    gw.register_service_handler('/bench/options', lambda _req: {'models': ['half_body_robot']})
    gw.register_service_handler('/motor/submit_policy', lambda _req: {'accepted': True})
    gw.register_service_handler('/bench/start_simulation', lambda _req: {'ok': True})
    gw.register_service_handler('/bench_manager/set_parameters_atomically', lambda _req: {'successful': True})

    app = create_app(
        ros_gateway=gw,
        web_root=tmp_path / 'web',
        motion_data_dir=tmp_path / 'data',
        recordings_dir=tmp_path / 'rec',
        control_mode='bench',
    )
    with TestClient(app) as client:
        # Unowned write calls must fail with 403 lease_required
        for url, body in (
            ('/api/control/joint-target', {'names': ['j1'], 'positions': [0.1]}),
            ('/api/control/trajectory', {'joint_names': ['j1'], 'points': [{'positions': [0.1], 'time_from_start_ms': 100}]}),
            ('/api/motor/policy', {'session_id': 1, 'targets': [{'motor_id': 1, 'enabled': True}]}),
            ('/api/bench/simulation/start', {'payload': {'model': 'half_body_robot'}}),
            ('/api/control/parameters', {'node': '/bench_manager', 'parameters': []}),
        ):
            r = client.post(url, json=body)
            assert r.status_code == 403
            assert r.json()['detail']['code'] == 'lease_required'

        # Read-only queries and universal E-Stop succeed for viewers
        assert client.get('/api/motor/device-info').status_code == 200
        assert client.get('/api/bench/options').status_code == 200
        assert client.post('/api/control/stop', json={'reason': 'viewer_estop'}).status_code == 200

        # WS uplink without lease returns lease_required error
        with client.websocket_connect('/ws?client_id=ctrl-1') as ws:
            _ = ws.receive_json()
            ws.send_json({'op': 'control:joint_target', 'names': ['j1'], 'positions': [0.2]})
            err = None
            for _ in range(5):
                pkt = ws.receive_json()
                if pkt.get('op') == 'error':
                    err = pkt
                    break
            assert err is not None
            assert err['code'] == 'lease_required'

            # Acquire lease -> REST + WS writes succeed
            acq = client.post('/api/control/lease/acquire', json={'client_id': 'ctrl-1'}).json()
            headers = {'X-Client-Id': 'ctrl-1', 'X-Lease-Token': acq['token']}
            assert client.post('/api/control/joint-target', json={'names': ['j1'], 'positions': [0.1], 'stamp_ns': 123456, 'frame_id': 'manual-1'}, headers=headers).status_code == 200
            assert client.post('/api/sim/joint-target', json={'names': ['j1'], 'positions': [0.2], 'frame_id': 'sim-session-9'}, headers=headers).status_code == 200
            assert client.post('/api/control/trajectory', json={'joint_names': ['j1'], 'points': [{'positions': [0.2], 'time_from_start_ms': 50}], 'stamp_ns': 987654321, 'frame_id': 'workbench-trajectory/42'}, headers=headers).status_code == 200
            assert client.post('/api/motor/policy', json={'session_id': 1, 'targets': [{'motor_id': 1, 'enabled': True}]}, headers=headers).status_code == 200
            assert client.post('/api/bench/simulation/start', json={'payload': {'model': 'half_body_robot'}}, headers=headers).status_code == 200
            assert client.post('/api/control/parameters', json={'node': '/bench_manager', 'parameters': []}, headers=headers).status_code == 200

            assert any(p.topic == '/control/joint_target' and p.payload['frame_id'] == 'manual-1' and p.payload['stamp_ns'] == 123456 for p in gw.publishes)
            assert any(p.topic == '/robot_sim/joint_position_target' and p.payload['frame_id'] == 'sim-session-9' for p in gw.publishes)
            assert any(p.topic == '/control/trajectory' and p.payload['frame_id'] == 'workbench-trajectory/42' and p.payload['stamp_ns'] == 987654321 for p in gw.publishes)

            ws.send_json({
                'op': 'control:joint_target',
                'token': acq['token'],
                'names': ['j1'],
                'positions': [0.3],
                'frame_id': 'ws-frame-1',
            })
            ws.send_json({
                'op': 'sim:joint_target',
                'token': acq['token'],
                'names': ['j1'],
                'positions': [0.4],
                'frame_id': 'sim-session-9',
            })
            # Drain lease:state broadcast if any, then ping to ensure server processed uplink
            ws.send_json({'op': 'ping', 'client_ts_ms': 1})
            for _ in range(5):
                pkt = ws.receive_json()
                if pkt.get('op') == 'pong':
                    break
            assert any(p.topic == '/control/joint_target' and p.payload['positions'] == [0.3] and p.payload['frame_id'] == 'ws-frame-1' for p in gw.publishes)
            assert any(p.topic == '/robot_sim/joint_position_target' and p.payload['positions'] == [0.4] and p.payload['frame_id'] == 'sim-session-9' for p in gw.publishes)

    # Readonly mode blocks writes even with valid lease token
    ro_app = create_app(
        ros_gateway=MockRosGateway(),
        web_root=tmp_path / 'web',
        motion_data_dir=tmp_path / 'data',
        recordings_dir=tmp_path / 'rec',
        control_mode='readonly',
    )
    with TestClient(ro_app) as ro_client:
        acq = ro_client.post('/api/control/lease/acquire', json={'client_id': 'c1'}).json()
        r = ro_client.post(
            '/api/control/joint-target',
            json={'names': ['j1'], 'positions': [0.1]},
            headers={'X-Client-Id': 'c1', 'X-Lease-Token': acq['token']},
        )
        assert r.status_code == 403
        assert r.json()['detail']['code'] == 'readonly_mode'


def test_diagnostics_omits_sim_position_when_sim_disabled(tmp_path):
    app = create_app(
        ros_gateway=MockRosGateway(enable_sim=False),
        web_root=tmp_path / 'web',
        motion_data_dir=tmp_path / 'data',
        recordings_dir=tmp_path / 'rec',
        control_mode='bench',
    )
    with TestClient(app) as client:
        diag = client.get('/api/diagnostics').json()
        assert 'telemetry:sim_position' not in diag['topics']
        assert 'telemetry:motor_state' in diag['topics']


def test_sim_joint_target_returns_published_false_when_sim_disabled(tmp_path):
    pytest.importorskip('rclpy')
    app = create_app(
        ros_gateway=RosGatewayThread(node_name='web_host_bff_no_sim', enable_sim=False),
        web_root=tmp_path / 'web',
        motion_data_dir=tmp_path / 'data',
        recordings_dir=tmp_path / 'rec',
        control_mode='bench',
    )
    with TestClient(app) as client:
        acq = client.post('/api/control/lease/acquire', json={'client_id': 'ctrl-nosim'}).json()
        headers = {'X-Client-Id': 'ctrl-nosim', 'X-Lease-Token': acq['token']}
        resp = client.post(
            '/api/sim/joint-target',
            json={'names': ['j1'], 'positions': [0.2], 'frame_id': 'sim-disabled'},
            headers=headers,
        )
        assert resp.status_code == 200
        body = resp.json()
        assert body['accepted'] is True
        assert body['published'] is False

        with client.websocket_connect('/ws?client_id=ctrl-nosim') as ws:
            _ = ws.receive_json()
            ws.send_json({
                'op': 'sim:joint_target',
                'token': acq['token'],
                'names': ['j1'],
                'positions': [0.4],
                'frame_id': 'sim-disabled',
            })
            ws.send_json({'op': 'ping', 'client_ts_ms': 1})
            pong = None
            for _ in range(5):
                pkt = ws.receive_json()
                if pkt.get('op') == 'pong':
                    pong = pkt
                    break
            assert pong is not None


def test_unified_same_origin_and_static_cache_headers(tmp_path):
    web_root = tmp_path / 'web'
    (web_root / 'workbench' / 'assets').mkdir(parents=True)
    (web_root / 'workbench' / 'index.html').write_text('<html><body>ok</body></html>', encoding='utf-8')
    hashed_js = web_root / 'workbench' / 'assets' / 'workbench-AbCdEf12.js'
    hashed_js.write_text('console.log("ok");' * 80, encoding='utf-8')
    mesh = web_root / 'mesh.stl'
    mesh.write_bytes(b'solid test\nendsolid test\n')

    app = create_app(
        ros_gateway=MockRosGateway(),
        web_root=web_root,
        motion_data_dir=tmp_path / 'data',
        recordings_dir=tmp_path / 'rec',
    )
    with TestClient(app) as client:
        # Cross-origin and malformed origins rejected across all mutating endpoints
        for bad_origin in ('http://evil.example', 'null', 'http://testserver:9999'):
            assert client.post('/api/control/lease/acquire', json={'client_id': 'c1'}, headers={'Origin': bad_origin}).status_code == 403
            assert client.post('/api/actions', json={'name': 'a.json', 'text': '{}'}, headers={'Origin': bad_origin}).status_code == 403
            assert client.post('/api/motion/jobs', json={}, headers={'Origin': bad_origin}).status_code == 403
            assert client.post('/api/motion/sources/status', json={}, headers={'Origin': bad_origin}).status_code == 403

        # Cross-origin WS rejected
        with pytest.raises(Exception):
            with client.websocket_connect('/ws?client_id=bad', headers={'Origin': 'http://evil.example'}):
                pass

        # Matching origin accepted
        assert client.post('/api/control/lease/acquire', json={'client_id': 'c1'}, headers={'Origin': 'http://testserver'}).status_code == 200

        # Static cache headers & gzip
        r_html = client.get('/workbench/')
        assert r_html.status_code == 200
        assert r_html.headers['cache-control'] == 'no-store'

        r_js = client.get('/workbench/assets/workbench-AbCdEf12.js', headers={'Accept-Encoding': 'gzip'})
        assert r_js.status_code == 200
        assert r_js.headers['cache-control'] == 'public, max-age=31536000, immutable'
        assert r_js.headers.get('content-encoding') == 'gzip'

        r_mesh = client.get('/mesh.stl')
        assert r_mesh.status_code == 200
        assert r_mesh.headers['cache-control'] == 'no-cache'

        # Symlinked assets (as installed by colcon --symlink-install) must serve 200
        ext_js = tmp_path / 'external_src.js'
        ext_js.write_text('export const ok = true;', encoding='utf-8')
        sym_js = web_root / 'workbench' / 'assets' / 'symlinked-12345678.js'
        sym_js.symlink_to(ext_js)
        r_sym = client.get('/workbench/assets/symlinked-12345678.js')
        assert r_sym.status_code == 200
        assert r_sym.headers['cache-control'] == 'public, max-age=31536000, immutable'

        # Path traversal blocked
        assert client.get('/../../etc/passwd').status_code == 404
        assert client.get('/workbench/%2e%2e/%2e%2e/etc/passwd').status_code == 404


def test_rosbag_recorder_lifecycle_and_download(tmp_path):
    pytest.importorskip('rclpy')
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import JointState

    gw = MockRosGateway()
    gw.start()
    rec_dir = tmp_path / 'recordings'
    recorder = RosbagRecorder(rec_dir, gw)

    app = create_app(
        ros_gateway=gw,
        web_root=tmp_path / 'web',
        motion_data_dir=tmp_path / 'data',
        recordings_dir=rec_dir,
    )
    with TestClient(app) as client:
        r_start = client.post('/api/recordings/start', json={'name': 'test_bag_01'})
        assert r_start.status_code == 200
        assert r_start.json()['active'] is True

        # Double start -> 409
        assert client.post('/api/recordings/start', json={'name': 'test_bag_02'}).status_code == 409

        msg = JointState()
        msg.name = ['joint_1']
        msg.position = [0.42]
        raw = serialize_message(msg)
        gw.inject_channel('telemetry:joints', {'names': ['joint_1'], 'positions': [0.42]}, raw_bytes=raw)

        r_stop = client.post('/api/recordings/stop')
        assert r_stop.status_code == 200
        info = r_stop.json()
        assert info['name'] == 'test_bag_01'
        assert info['message_count'] == 1
        assert info['size_bytes'] > 0

        r_list = client.get('/api/recordings')
        assert r_list.status_code == 200
        assert len(r_list.json()['recordings']) == 1

        r_dl = client.get('/api/recordings/test_bag_01/download')
        assert r_dl.status_code == 200
        assert len(r_dl.content) > 0

        assert client.get('/api/recordings/..%2Fetc/download').status_code in (400, 404)
        assert client.delete('/api/recordings/test_bag_01').status_code == 200
        assert client.get('/api/recordings').json()['recordings'] == []
    assert recorder.storage_format in ('mcap', 'sqlite3')


def test_real_ros2_dds_integration_and_clean_shutdown(tmp_path):
    rclpy = pytest.importorskip('rclpy')
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Imu, JointState
    from std_msgs.msg import Empty, Int32
    import httpx
    import uvicorn

    port = _free_port()
    gw = RosGatewayThread(node_name='web_host_bff_it')
    app = create_app(
        ros_gateway=gw,
        web_root=tmp_path / 'web',
        motion_data_dir=tmp_path / 'data',
        recordings_dir=tmp_path / 'rec',
        broadcast_hz=40.0,
    )
    server = uvicorn.Server(uvicorn.Config(app, host='127.0.0.1', port=port, log_level='warning'))
    srv_thread = threading.Thread(target=server.run, daemon=True)
    srv_thread.start()

    ext_ctx = rclpy.Context()
    rclpy.init(context=ext_ctx)
    ext_node = rclpy.create_node('bff_it_ext', context=ext_ctx)
    ext_exec = SingleThreadedExecutor(context=ext_ctx)
    ext_exec.add_node(ext_node)

    seen_counts: list[int] = []
    seen_stops = 0
    seen_targets: list[list[float]] = []

    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    ext_node.create_subscription(Int32, '/client_count', lambda m: seen_counts.append(int(m.data)), latched)

    def _on_stop(_m):
        nonlocal seen_stops
        seen_stops += 1

    ext_node.create_subscription(Empty, '/control/stop', _on_stop, 10)
    ext_node.create_subscription(
        JointState, '/control/joint_target', lambda m: seen_targets.append(list(m.position)), 10,
    )
    joint_pub = ext_node.create_publisher(JointState, '/joint_states', 10)
    sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
    imu_pub = ext_node.create_publisher(Imu, '/imu/data', sensor_qos)

    ext_stop = threading.Event()
    ext_thread = threading.Thread(
        target=lambda: [
            ext_exec.spin_once(timeout_sec=0.02)
            for _ in iter(lambda: ext_stop.is_set() or not ext_ctx.ok(), True)
        ],
        daemon=True,
    )
    ext_thread.start()

    try:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            try:
                if httpx.get(f'http://127.0.0.1:{port}/api/health', timeout=0.5).status_code == 200:
                    break
            except Exception:
                time.sleep(0.05)
        else:
            pytest.fail('uvicorn BFF server did not become healthy')

        pub_stop = threading.Event()

        def _pub_joints():
            while not pub_stop.is_set():
                msg = JointState()
                ns = time.time_ns()
                msg.header.stamp.sec = ns // 1_000_000_000
                msg.header.stamp.nanosec = ns % 1_000_000_000
                msg.name = ['joint_1', 'joint_2']
                msg.position = [0.25, -0.5]
                joint_pub.publish(msg)
                imu_msg = Imu()
                imu_msg.orientation.w = 1.0
                imu_pub.publish(imu_msg)
                time.sleep(0.01)

        pub_thread = threading.Thread(target=_pub_joints, daemon=True)
        pub_thread.start()

        async def _scenario():
            async with httpx.AsyncClient(base_url=f'http://127.0.0.1:{port}') as hc:
                async with websockets.connect(f'ws://127.0.0.1:{port}/ws?client_id=viewer') as viewer:
                    _ = json.loads(await viewer.recv())
                    await viewer.send(json.dumps({'op': 'subscribe', 'channels': ['telemetry:joints', 'telemetry:imu']}))
                    _ = json.loads(await viewer.recv())

                    async with websockets.connect(f'ws://127.0.0.1:{port}/ws?client_id=ctrl') as ctrl:
                        _ = json.loads(await ctrl.recv())
                        r_acq = await hc.post('/api/control/lease/acquire', json={'client_id': 'ctrl'})
                        assert r_acq.status_code == 200
                        token = r_acq.json()['token']
                        t_wait1 = time.monotonic() + 1.0
                        while 1 not in seen_counts and time.monotonic() < t_wait1:
                            await asyncio.sleep(0.02)

                        # Verify viewer receives JointState and BEST_EFFORT Imu fan-out
                        got_joint = None
                        got_imu = None
                        for _ in range(30):
                            pkt = json.loads(await asyncio.wait_for(viewer.recv(), timeout=2.0))
                            if pkt.get('channel') == 'telemetry:joints':
                                got_joint = pkt
                            elif pkt.get('channel') == 'telemetry:imu':
                                got_imu = pkt
                            if got_joint and got_imu:
                                break
                        assert got_joint is not None
                        assert got_joint['data']['names'] == ['joint_1', 'joint_2']
                        assert got_imu is not None
                        assert isinstance(got_imu['data']['orientation']['w'], float)

                        # Controller publishes joint target via WS uplink -> real DDS
                        await ctrl.send(json.dumps({
                            'op': 'control:joint_target',
                            'token': token,
                            'names': ['joint_1', 'joint_2'],
                            'positions': [0.55, -0.15],
                        }))

                        # Viewer triggers E-Stop -> real DDS /control/stop
                        assert (await hc.post('/api/control/stop', json={'reason': 'it_estop'})).status_code == 200

                    wait_end = time.monotonic() + 2.0
                    while time.monotonic() < wait_end:
                        if seen_counts and seen_counts[-1] == 0 and seen_stops >= 1 and seen_targets:
                            break
                        await asyncio.sleep(0.02)

        asyncio.run(_scenario())
        pub_stop.set()
        pub_thread.join(timeout=1.0)

        assert 1 in seen_counts
        assert seen_counts[-1] == 0
        assert seen_stops >= 1
        assert seen_targets and seen_targets[-1] == pytest.approx([0.55, -0.15])
    finally:
        pub_stop.set()
        pub_thread.join(timeout=1.0)
        t0 = time.monotonic()
        server.should_exit = True
        srv_thread.join(timeout=3.0)
        shutdown_s = time.monotonic() - t0
        ext_stop.set()
        ext_thread.join(timeout=1.0)
        ext_exec.shutdown()
        ext_node.destroy_node()
        ext_ctx.try_shutdown()
        assert not srv_thread.is_alive()
        assert shutdown_s < 2.0
