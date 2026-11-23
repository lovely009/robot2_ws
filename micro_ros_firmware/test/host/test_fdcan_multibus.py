"""Multi-channel routing tests; no USB, physical CAN, or motors."""
import importlib.util
from pathlib import Path
import sys

import pytest

TOOLS = Path(__file__).parents[3] / 'micro_ros_agent/tools'
sys.path.insert(0, str(TOOLS))


def load_routes():
    spec = importlib.util.spec_from_file_location('motor_sim_routes', TOOLS / 'motor_sim_routes.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_explicit_nodes_and_remainder_are_partitioned():
    routes = load_routes().assign_nodes([
        {'bus': 0, 'transport': 'usbcanfd', 'device': 0, 'channel': 0, 'ids': '*'},
        {'bus': 1, 'transport': 'usbcanfd', 'device': 0, 'channel': 1, 'ids': '3-4'},
    ], [1, 2, 3, 4])
    assert [r['node_ids'] for r in routes] == [[1, 2], [3, 4]]


def test_can4_can_be_used_with_all_native_routes():
    rows = [
        {'bus': i, 'transport': 'socketcan', 'endpoint': f'can{i}', 'ids': str(i + 1)}
        for i in range(4)
    ]
    assign = load_routes().assign_nodes
    assert [r['node_ids'] for r in assign(rows, [1, 2, 3, 4])] == [[1], [2], [3], [4]]
    rows[-1]['bus'] = 4
    with pytest.raises(ValueError, match='MCU bus'):
        assign(rows, [1, 2, 3, 4])


@pytest.mark.parametrize('ids,expected', [('1-3', [1, 2, 3]), ('', [])])
def test_explicit_subset_leaves_other_model_nodes_unloaded(ids, expected):
    routes = load_routes().assign_nodes([
        {'bus': 0, 'transport': 'usbcanfd', 'device': 0, 'channel': 0, 'ids': ids},
    ], list(range(1, 38)))
    assert routes[0]['node_ids'] == expected


@pytest.mark.parametrize('rows,match', [
    ([{'bus': 0, 'transport': 'socketcan', 'endpoint': 'can0', 'ids': '1'},
      {'bus': 1, 'transport': 'socketcan', 'endpoint': 'can1', 'ids': '1'}], 'duplicate'),
    ([{'bus': 0, 'transport': 'socketcan', 'endpoint': 'can0', 'ids': '8'}], 'unknown'),
    ([{'bus': 0, 'transport': 'socketcan', 'endpoint': 'can0', 'ids': '*'},
      {'bus': 1, 'transport': 'socketcan', 'endpoint': 'can1', 'ids': '*'}], 'remainder'),
    ([{'bus': 0, 'transport': 'usbcanfd', 'device': 0, 'channel': 2, 'ids': '*'}], 'channel'),
    ([{'bus': 0, 'transport': 'socketcan', 'endpoint': 'can0', 'ids': '*'},
      {'bus': 1, 'transport': 'socketcan', 'endpoint': 'can0', 'ids': ''}], 'endpoint'),
])
def test_invalid_routes_fail_before_open(rows, match):
    with pytest.raises(ValueError, match=match):
        load_routes().assign_nodes(rows, [1, 2])

class FakeSDK:
    def __init__(self):
        self.calls = []
        self.fail_channel = None
        self.fd_packet = None

    def __getattr__(self, name):
        def call(*args):
            self.calls.append((name, args))
            if name == 'ZCAN_OpenDeviceEx':
                return 100
            if name == 'ZCAN_InitCAN':
                return 0 if args[1] == self.fail_channel else 200 + args[1]
            if name == 'ZCAN_ReceiveFD':
                if self.fd_packet is None:
                    return 0
                packet = args[1][0]
                packet.frame.can_id = self.fd_packet[0]
                packet.frame.length = len(self.fd_packet[1])
                packet.frame.flags = 1
                packet.frame.data[:packet.frame.length] = self.fd_packet[1]
                self.fd_packet = None
                return 1
            if name == 'ZCAN_Receive':
                return 0
            return 1
        return call


def test_partial_full_body_backend_only_responds_for_selected_nodes(monkeypatch):
    import ctypes
    import canfd_transports as transport
    from fdcan_motor_sim import build_multibus_sim

    api = FakeSDK()
    sent = []

    def transmit(handle, packets, _count):
        packet = ctypes.cast(packets, ctypes.POINTER(transport.TxFD)).contents
        sent.append((handle, packet.frame.can_id))
        return 1

    api.ZCAN_TransmitFD = transmit
    monkeypatch.setattr(transport, 'CtypesSDK', lambda _: api)
    monkeypatch.setattr(transport, 'usb_devices', lambda: [
        {'bus': 999, 'address': 103}])
    vaxor_ids = [13, 14, 22, 23, 30, 37]
    group = build_multibus_sim([
        {'bus': 0, 'transport': 'usbcanfd', 'device': 0, 'channel': 0, 'ids': '1-9'},
        {'bus': 1, 'transport': 'usbcanfd', 'device': 0, 'channel': 1, 'ids': '10-18'},
    ], [mid for mid in range(1, 38) if mid not in vaxor_ids], vaxor_ids, period_ms=20)
    try:
        status = group.apply_control({'cmd': 'status'})
        assert status['ok']
        assert [(m['id'], m['bus']) for m in status['motors']] == [
            (mid, 0 if mid <= 9 else 1) for mid in range(1, 19)]
        assert [r['nodes'] for r in status['channels']] == [
            list(range(1, 10)), list(range(10, 19))]
        assert sorted(mid for mid, motor in group.models.items()
                      if motor.kind == 'vaxor') == [13, 14]
        assert all(not motor.enabled for motor in group.models.values())
        for bus in group.buses.values():
            for mid in range(1, 38):
                bus.handle_sdo(mid, bytes.fromhex('4008100000000000'))
            bus._pump_tx()
        assert sorted(sent) == [(200, 0x580 + mid) for mid in range(1, 10)] + [
            (201, 0x580 + mid) for mid in range(10, 19) if mid not in vaxor_ids]
    finally:
        group.close()


def test_usb_device_opens_once_and_configures_before_init():
    from canfd_transports import CxDevice
    api = FakeSDK()
    device = CxDevice(api, 1, 17)
    a = device.open_channel(0)
    b = device.open_channel(1)
    calls = [name for name, _ in api.calls]
    assert calls.count('ZCAN_OpenDeviceEx') == 1
    first_init = calls.index('ZCAN_InitCAN')
    assert calls.index('ZCAN_SetAbitBaud') < first_init
    assert calls.index('ZCAN_SetDbitBaud') < first_init
    assert calls.index('ZCAN_SetResistanceEnable') < first_init
    a.close(); b.close(); device.close(); device.close()
    assert [n for n, _ in api.calls].count('ZCAN_CloseDevice') == 1
    assert [n for n, _ in api.calls].count('ZCAN_ResetCAN') == 2


def test_usb_startup_failure_releases_all_open_channels():
    from canfd_transports import CxDevice
    api = FakeSDK(); api.fail_channel = 1
    device = CxDevice(api, 1, 17)
    with pytest.raises(RuntimeError, match='InitCAN'):
        with device:
            device.open_channel(0)
            device.open_channel(1)
    assert [n for n, _ in api.calls].count('ZCAN_ResetCAN') == 1
    assert [n for n, _ in api.calls].count('ZCAN_CloseDevice') == 1


def test_usb_error_passive_marker_does_not_drop_valid_fd_sync():
    from canfd_transports import CxDevice
    from fdcan_motor_sim import parse_socketcan_frame
    api = FakeSDK()
    with CxDevice(api, 1, 17) as device:
        channel = device.open_channel(0)
        api.fd_packet = (0x20000080, b'')
        assert parse_socketcan_frame(channel.recv(72)) == (0x80, b'', True)
        with pytest.raises(BlockingIOError):
            channel.recv(72)

class FakeFrameTransport:
    polling = True
    label = 'fake CAN'

    def __init__(self):
        self.sent = []
        self.closed = False

    def send(self, packet):
        self.sent.append(packet)
        return len(packet)

    def recv(self, _size):
        raise BlockingIOError()

    def close(self):
        self.closed = True


def make_bus(node):
    from fdcan_motor_sim import CanMotorSim
    transport = FakeFrameTransport()
    sim = CanMotorSim(None, [node], [], 20, None, bus_transport=transport)
    return sim, transport


def test_sync_and_sdo_stay_on_the_receiving_bus():
    from fdcan_motor_sim import CanMotorSimGroup, parse_socketcan_frame
    a, ta = make_bus(1); b, tb = make_bus(2)
    group = CanMotorSimGroup({0: a, 1: b})
    try:
        a.handle_downlink(0x80, b'', True)
        assert a.sync_seen and not b.sync_seen
        a.handle_sdo(1, bytes.fromhex('4008100000000000'))
        a._pump_tx()
        assert any(parse_socketcan_frame(f)[0] == 0x581 for f in ta.sent)
        assert tb.sent == []
        status = group.apply_control({'cmd': 'status'})
        assert [(m['id'], m['bus']) for m in status['motors']] == [(1, 0), (2, 1)]
    finally:
        group.close()
    assert ta.closed and tb.closed


def test_dynamic_nodes_require_bus_and_keep_ids_globally_unique():
    from fdcan_motor_sim import CanMotorSimGroup
    a, _ = make_bus(1); b, _ = make_bus(2)
    group = CanMotorSimGroup({0: a, 1: b})
    try:
        assert not group.apply_control({'cmd': 'add', 'id': 3, 'kind': 'cerb'})['ok']
        assert not group.apply_control({'cmd': 'add', 'id': 1, 'kind': 'cerb', 'bus': 1})['ok']
        assert group.apply_control({'cmd': 'add', 'id': 3, 'kind': 'cerb', 'bus': 1})['ok']
        assert 3 in b.models and 3 not in a.models
        b.next_send[3] = [1.0]
        assert group.apply_control({'cmd': 'remove', 'id': 3})['ok']
        assert 3 not in b.next_send and 3 not in group.models
    finally:
        group.close()


def test_multiple_usb_devices_use_distinct_slots_and_serialize_sdk(monkeypatch):
    import canfd_transports as transport
    import threading
    import time
    from fdcan_motor_sim import pack_socketcan_frame
    api = FakeSDK()
    monkeypatch.setattr(transport, 'CtypesSDK', lambda _: api)
    monkeypatch.setattr(transport, 'usb_devices', lambda: [
        {'bus': 999, 'address': 101}, {'bus': 999, 'address': 102}])
    pool = transport.CxPool()
    active = 0; peak = 0
    def transmit(*_):
        nonlocal active, peak
        active += 1; peak = max(peak, active)
        time.sleep(.005)
        active -= 1
        return 1
    api.ZCAN_TransmitFD = transmit
    try:
        a = pool.channel(0, 0); b = pool.channel(1, 0)
        packet = pack_socketcan_frame(0x301, b'abc', True, True)
        workers = [threading.Thread(target=ch.send, args=(packet,)) for ch in (a, b)]
        for worker in workers: worker.start()
        for worker in workers: worker.join()
        assert peak == 1
        assert [args[1] for name, args in api.calls if name == 'ZCAN_OpenDeviceEx'] == [0, 1]
    finally:
        pool.close()


def test_group_close_preserves_control_socket_it_did_not_bind(tmp_path):
    from fdcan_motor_sim import CanMotorSimGroup
    a, _ = make_bus(1)
    path = tmp_path / 'other-owner.sock'; path.write_text('owned by another process')
    group = CanMotorSimGroup({0:a}, str(path))
    group.close()
    assert path.exists()


def test_group_close_releases_other_buses_when_one_close_fails():
    from fdcan_motor_sim import CanMotorSimGroup
    a, _ = make_bus(1); b, tb = make_bus(2)
    group = CanMotorSimGroup({0:a,1:b})
    def fail_close(): raise OSError('disconnected adapter')
    a.close = fail_close
    with pytest.raises(OSError): group.close()
    assert tb.closed


def test_bad_slcan_recovery_does_not_block_healthy_bus(monkeypatch):
    from fdcan_motor_sim import CanMotorSimGroup, parse_socketcan_frame
    import fdcan_motor_sim as module
    a, _ = make_bus(1); b, tb = make_bus(2)
    group = CanMotorSimGroup({0:a,1:b})
    monkeypatch.setattr(module.time, 'sleep', lambda _: (_ for _ in ()).throw(AssertionError('blocking recovery')))
    monkeypatch.setattr(module.os, 'write', lambda fd, data: len(data))
    try:
        a.start_loop(); b.start_loop()
        a.data_bitrate_index = 5
        a._recover_latched_bus()
        now = module.time.monotonic()
        a.tick(now)
        b._enqueue_rx((0x80,b'',True))
        b.tick(now)
        b.tick(now + 0.005)
        assert b.sync_seen
        assert any(parse_socketcan_frame(f)[0] == 0x302 for f in tb.sent)
        assert a._recovery is not None
    finally:
        group.close()


def test_control_rejects_unhashable_protocol_without_stopping_buses():
    from fdcan_motor_sim import CanMotorSimGroup
    a, _ = make_bus(1); b, _ = make_bus(2)
    group = CanMotorSimGroup({0:a,1:b})
    try:
        assert not group.apply_control({'cmd':'add','id':3,'kind':[],'bus':1})['ok']
        assert group.apply_control({'cmd':'status'})['ok']
    finally:
        group.close()


def test_runtime_io_failure_is_visible_without_stopping_other_buses():
    from fdcan_motor_sim import CanMotorSimGroup
    import time
    a, _ = make_bus(1); b, tb = make_bus(2)
    group = CanMotorSimGroup({0:a,1:b})
    a.models[1].enabled = True
    def failed_tick(*_, **__): raise TimeoutError('recovery write timed out')
    a.tick = failed_tick
    try:
        a.start_loop(); b.start_loop()
        b._enqueue_rx((0x80,b'',True))
        group.tick(time.monotonic())
        group.tick(time.monotonic() + .005)
        status = group.apply_control({'cmd':'status'})
        assert status['channels'][0]['state'] == 'failed'
        assert 'timed out' in status['channels'][0]['error']
        assert not group.stop and not b.stop and not a.models[1].enabled
        assert b.sync_seen and tb.sent
    finally: group.close()


def test_recovery_waits_from_actual_write_completion(monkeypatch):
    import fdcan_motor_sim as module
    from types import SimpleNamespace
    clock = [100.0]
    monkeypatch.setattr(module, 'time', SimpleNamespace(monotonic=lambda:clock[0]))
    a, _ = make_bus(1)
    group = module.CanMotorSimGroup({0:a})
    def delayed_write(fd, payload):
        clock[0] += .2
        return len(payload)
    monkeypatch.setattr(module.os,'write',delayed_write)
    try:
        a.data_bitrate_index = 5
        a._recover_latched_bus()
        a.tick(100.0)
        assert a._recovery['ready_at'] >= 100.35
    finally:group.close()


def test_partial_control_request_does_not_pause_bus_service(tmp_path):
    import socket,json,time
    from fdcan_motor_sim import CanMotorSimGroup
    a, _ = make_bus(1); b, tb = make_bus(2)
    group = CanMotorSimGroup({0:a,1:b}, str(tmp_path/'control.sock'))
    client=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
    try:
        group._setup_control_socket(); a.start_loop(); b.start_loop()
        client.connect(group.control_socket_path)
        client.sendall(b'{"cmd":')
        start=time.monotonic(); group.tick(start)
        assert time.monotonic()-start < .1
        b._enqueue_rx((0x80,b'',True)); group.tick(time.monotonic()); group.tick(time.monotonic())
        assert b.sync_seen and tb.sent
        client.sendall(b'"status"}\n')
        client.setblocking(False)
        response = bytearray()
        for _ in range(20):
            group.tick(time.monotonic())
            try:
                response.extend(client.recv(4096))
            except BlockingIOError:
                continue
            if b'\n' in response:
                break
        assert b'\n' in response
        assert json.loads(response)['ok']
    finally:client.close();group.close()


class SimTime:
    def __init__(self):
        self.now_ns = 100_000_000_000
        self.epoch_offset_ns = 1_790_000_000_000_000_000

    def monotonic_ns(self):
        return self.now_ns

    def time_ns(self):
        return self.now_ns + self.epoch_offset_ns

    def advance_ms(self, milliseconds):
        self.now_ns += round(milliseconds * 1_000_000)


def make_timed_bus(node=1, *, fb_period_ms=0):
    from fdcan_motor_sim import CanMotorSim, HostEpochClock
    source = SimTime()
    clock = HostEpochClock(source.monotonic_ns, source.time_ns)
    transport = FakeFrameTransport()
    sim = CanMotorSim(None, [node], [], 20, None, bus_transport=transport,
                      clock=clock, fb_period_ms=fb_period_ms, stagger_ms=0)
    sim.start_loop()
    sim._last_step = source.now_ns / 1e9
    return sim, transport, source


def test_host_epoch_clock_refresh_preserves_captured_sample_across_wall_step():
    from fdcan_motor_sim import HostEpochClock
    source = SimTime()
    clock = HostEpochClock(source.monotonic_ns, source.time_ns)
    first = clock.stamp()
    assert first['epoch_ns'] == 1_790_000_100_000_000_000
    source.advance_ms(1000)
    source.epoch_offset_ns -= 2_000_000_000
    clock.refresh()
    second = clock.stamp()
    assert second['monotonic_ns'] - first['monotonic_ns'] == 1_000_000_000
    assert second['epoch_ns'] == first['epoch_ns'] - 1_000_000_000
    assert first['epoch_ns'] == 1_790_000_100_000_000_000
    assert second['clock_generation'] != first['clock_generation']
    status = clock.status()
    assert status['step_count'] == 1
    assert status['last_correction_ns'] == -2_000_000_000


def test_sync_receive_timestamp_survives_main_loop_delay_and_can_backpressure():
    sim, transport, source = make_timed_bus()
    try:
        sim._enqueue_rx((0x80, b'', True))
        source.advance_ms(12)
        sim.tick(source.now_ns / 1e9)
        status = sim.apply_control({'cmd': 'status'})['timing']
        assert status['rx_queue_ms']['last'] == 12
        assert status['feedback_lateness_ms']['last'] == 12
        assert status['trigger_to_tx_ms']['last'] == 12
        frame = status['last_feedback']['1']
        assert frame['trigger']['epoch_ns'] == 1_790_000_100_000_000_000
        assert frame['encoded']['epoch_ns'] == 1_790_000_100_012_000_000
        assert frame['sent']['epoch_ns'] == frame['encoded']['epoch_ns']
        assert len(transport.sent[-1]) == 72
        assert transport.sent[-1][4] == 12

        def full(_packet):
            raise BlockingIOError()

        transport.send = full
        source.advance_ms(8)
        sim._enqueue_rx((0x80, b'', True))
        sim.tick(source.now_ns / 1e9)
        source.advance_ms(35)
        transport.send = lambda packet: len(packet)
        sim._pump_tx()
        status = sim.apply_control({'cmd': 'status'})['timing']
        assert status['feedback_queue_ms']['last'] == 35
        assert status['trigger_to_tx_ms']['last'] == 35
        assert status['feedback_over_period'] == 1
    finally:
        sim.close()


def test_sync_period_uses_receive_time_and_rejects_older_sync():
    sim, _, source = make_timed_bus()
    try:
        first = sim.clock.stamp()
        sim._enqueue_rx((0x80, b'', True))
        source.advance_ms(20)
        sim._enqueue_rx((0x80, b'', True))
        source.advance_ms(30)
        sim._drain_rx()
        sim.handle_downlink(0x80, b'', True, received=first)
        status = sim.apply_control({'cmd': 'status'})['timing']
        assert sim.sync_count == 2
        assert status['sync_period_ms']['last'] == 20
        assert status['sync_jitter_ms']['last'] == 0
        assert status['rx_queue_ms']['max'] == 50
        assert status['out_of_order_sync'] == 1
    finally:
        sim.close()


def test_multibus_status_keeps_sync_clocks_and_feedback_separate():
    from fdcan_motor_sim import CanMotorSimGroup
    a, _, sa = make_timed_bus(1)
    b, _, sb = make_timed_bus(2)
    group = CanMotorSimGroup({0: a, 1: b})
    try:
        a._enqueue_rx((0x80, b'', True))
        sa.advance_ms(5)
        a.tick(sa.now_ns / 1e9)
        status = group.apply_control({'cmd': 'status'})
        assert status['channels'][0]['timing']['trigger_to_tx_ms']['last'] == 5
        assert status['channels'][1]['timing']['last_sync'] is None
        assert status['channels'][1]['timing']['last_feedback'] == {}
        sb.advance_ms(10)
        b._enqueue_rx((0x80, b'', True))
        b.tick(sb.now_ns / 1e9)
        status = group.apply_control({'cmd': 'status'})
        assert set(status['channels'][0]['timing']['last_feedback']) == {'1'}
        assert set(status['channels'][1]['timing']['last_feedback']) == {'2'}
        assert group.apply_control({'cmd': 'remove', 'id': 1})['ok']
        assert group.apply_control({'cmd': 'status'})['channels'][0]['timing']['last_feedback'] == {}
    finally:
        group.close()


def test_fallback_feedback_is_identified_independently_of_last_sync():
    sim, _, source = make_timed_bus(fb_period_ms=40)
    try:
        sim._enqueue_rx((0x80, b'', True))
        sim.tick(source.now_ns / 1e9)
        source.advance_ms(41)
        sim.tick(source.now_ns / 1e9)
        timing = sim.apply_control({'cmd': 'status'})['timing']
        assert timing['last_feedback']['1']['trigger']['source'] == 'fallback'
        assert timing['last_feedback']['1']['trigger']['epoch_ns'] == 1_790_000_100_041_000_000
        assert timing['last_sync']['epoch_ns'] == 1_790_000_100_000_000_000
    finally:
        sim.close()


def test_queued_feedback_retains_original_sync_after_another_sync_arrives():
    sim, _, source = make_timed_bus()
    try:
        sim._enqueue_rx((0x80, b'', True))
        source.advance_ms(20)
        sim._enqueue_rx((0x80, b'', True))
        source.advance_ms(10)
        sim.tick(source.now_ns / 1e9)
        timing = sim.apply_control({'cmd': 'status'})['timing']
        assert timing['last_sync']['sequence'] == 2
        assert timing['last_feedback']['1']['trigger']['sequence'] == 1
        assert timing['trigger_to_tx_ms']['last'] == 30
        sim.tick(source.now_ns / 1e9)
        timing = sim.apply_control({'cmd': 'status'})['timing']
        assert timing['last_feedback']['1']['trigger']['sequence'] == 2
        assert timing['trigger_to_tx_ms']['last'] == 10
    finally:
        sim.close()


def test_vaxor_command_reply_has_receive_and_send_epoch_without_wire_change():
    import fdcan_motor_sim as module
    source = SimTime()
    clock = module.HostEpochClock(source.monotonic_ns, source.time_ns)
    transport = FakeFrameTransport()
    sim = module.CanMotorSim(None, [], [13], 20, None,
                             bus_transport=transport, clock=clock)
    try:
        sim._enqueue_rx((0x501 + 4 * 13, bytes(12), True))
        source.advance_ms(7)
        sim._drain_rx()
        sim._pump_tx()
        timing = sim.apply_control({'cmd': 'status'})['timing']
        frame = timing['last_feedback']['13']
        assert frame['trigger']['source'] == 'command'
        assert frame['encoded']['epoch_ns'] - frame['trigger']['epoch_ns'] == 7_000_000
        can_id, payload, is_fd = module.parse_socketcan_frame(transport.sent[-1])
        assert (can_id, len(payload), is_fd) == (0x735, 12, True)
    finally:
        sim.close()


def test_timing_statistics_are_bounded_and_keep_lifetime_maximum():
    from fdcan_motor_sim import TimingMetric
    metric = TimingMetric()
    metric.add(100_000_000)
    for _ in range(600):
        metric.add(2_000_000)
    status = metric.status()
    assert status['count'] == 601
    assert status['window_count'] == 512
    assert status['max'] == 100
    assert status['window_max'] == status['p95'] == status['mean'] == 2


def test_fallback_after_stall_reports_actual_motor_feedback_gap():
    sim, _, source = make_timed_bus(fb_period_ms=40)
    try:
        sim._enqueue_rx((0x80, b'', True))
        sim.tick(source.now_ns / 1e9)
        source.advance_ms(160)
        sim.tick(source.now_ns / 1e9)
        timing = sim.apply_control({'cmd': 'status'})['timing']
        assert timing['feedback_gap_ms']['last'] == 160
        assert timing['last_feedback']['1']['gap_ms'] == 160
        assert timing['response_by_source_ms']['fallback']['last'] == 0
    finally:
        sim.close()


def test_mixed_feedback_separates_sync_and_command_latencies_and_rx_log():
    import io
    import json
    sim, _, source = make_timed_bus()
    log = io.StringIO()
    sim.log_f = log
    try:
        assert sim.apply_control({'cmd': 'add', 'id': 13, 'kind': 'vaxor'})['ok']
        sim._enqueue_rx((0x80, b'', True))
        source.advance_ms(10)
        sim.tick(source.now_ns / 1e9)
        sim._enqueue_rx((0x501 + 4 * 13, bytes(12), True))
        source.advance_ms(3)
        sim._drain_rx()
        sim._pump_tx()
        timing = sim.apply_control({'cmd': 'status'})['timing']
        assert timing['response_by_source_ms']['sync']['count'] == 2
        assert timing['response_by_source_ms']['sync']['mean'] == 10
        assert timing['response_by_source_ms']['command']['count'] == 1
        assert timing['response_by_source_ms']['command']['mean'] == 3
        rx = next(json.loads(line) for line in log.getvalue().splitlines()
                  if json.loads(line)['dir'] == 'rx')
        assert rx['received']['epoch_ns'] == 1_790_000_100_010_000_000
        assert rx['time']['monotonic_ns'] - rx['received']['monotonic_ns'] == 3_000_000
    finally:
        sim.close()


def test_transport_send_duration_includes_blocking_and_backpressure():
    sim, transport, source = make_timed_bus()
    try:
        def blocked_send(_packet):
            source.advance_ms(350)
            raise BlockingIOError()

        transport.send = blocked_send
        sim._enqueue_rx((0x80, b'', True))
        sim.tick(source.now_ns / 1e9)
        timing = sim.timing_status()
        assert timing['transport_send_ms']['last'] == 350
        assert timing['last_feedback'] == {}

        transport.send = lambda packet: len(packet)
        sim._pump_tx()
        timing = sim.timing_status()
        assert timing['transport_send_ms']['last'] == 0
        assert timing['transport_send_ms']['max'] == 350
        assert timing['feedback_queue_ms']['last'] == 350
    finally:
        sim.close()


def test_transport_receive_duration_includes_failed_poll():
    sim, transport, source = make_timed_bus()
    try:
        def blocked_receive(_size):
            source.advance_ms(350)
            sim.stop = True
            raise BlockingIOError()

        transport.recv = blocked_receive
        sim._reader_loop()
        assert sim.timing_status()['transport_receive_ms']['last'] == 350
    finally:
        sim.close()


def test_group_loop_timing_includes_control_service_delay():
    import time
    from fdcan_motor_sim import CanMotorSimGroup
    sim, _ = make_bus(1)
    group = CanMotorSimGroup({0: sim})

    def delayed_control_service():
        time.sleep(0.02)
        group.stop = True

    group._service_control_socket = delayed_control_service
    group.run()
    timing = group.apply_control({'cmd': 'status'})['loop_timing']
    assert timing['control_service_ms']['last'] >= 20
    assert timing['loop_work_ms']['count'] == 1
    assert timing['loop_work_ms']['last'] >= timing['control_service_ms']['last']
