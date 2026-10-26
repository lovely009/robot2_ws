"""直连参数会话测试：仅使用内存假体，不打开真实 CAN 或串口。"""
from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import Mock

import canopen
import pytest

from web_host.direct_can import DirectCanSession, FdSdoNetwork


@pytest.fixture
def direct():
    clients = {}

    class Network:
        def __init__(self):
            self.subscriptions = {}
            self.send_message = Mock()
            self.disconnect = Mock()

        def __len__(self):
            return len(clients)

        def subscribe(self, cob_id, callback):
            self.subscriptions[cob_id] = callback

        def __contains__(self, node_id):
            return node_id in clients

        def add_node(self, node_id):
            def upload(index, sub):
                if index == 0x1008:
                    return f'R48LITE_V{node_id}'.encode()
                if index == 0x607D:
                    return (-32768 if sub == 1 else 32768).to_bytes(4, 'little', signed=True)
                return (314 if index in (0x2542, 0x2543) else 500).to_bytes(4, 'little')
            client = SimpleNamespace(sdo=SimpleNamespace(upload=Mock(side_effect=upload)))
            clients[node_id] = client
            return client

        def __getitem__(self, node_id):
            return clients[node_id]

    network = Network()
    factory = Mock(return_value=network)
    session = DirectCanSession(factory, lambda: {'socketcan': ['fixture-can'], 'slcan': ['/fixture-tty']})
    yield session, network, clients, factory
    session.close()


def test_connect_does_not_send_or_read_device_objects(direct):
    session, _, clients, factory = direct
    result = session.connect('socketcan', 'fixture-can')
    assert result['read_only'] is True
    assert clients == {}
    assert session.heartbeat(result['session'])['connected'] is True
    assert session.discover(result['session'])['motors'] == []
    factory.assert_called_once_with('socketcan', 'fixture-can')
    with pytest.raises(ValueError, match='已有直连会话'):
        session.connect('socketcan', 'fixture-can')


def test_read_decodes_each_node_and_keeps_object_units(direct):
    session, _, clients, _ = direct
    token = session.connect('slcan', '/fixture-tty')['session']
    result = session.read(token, [2, 7])
    assert [m['id'] for m in result['motors']] == [2, 7]
    fields = {f['key']: f for f in result['motors'][0]['fields']}
    assert fields['identity']['value'] == 'R48LITE_V2'
    assert fields['position_min']['value'] == -32768
    assert fields['p_scale']['value'] == 3.14
    assert fields['kp_scale']['value'] == 500
    assert fields['current_max']['unit'] == 'mA'
    assert clients[2].sdo.upload.call_args_list[0].args == (0x1008, 0)


def test_no_reply_is_reported_without_default_parameters(direct):
    session, network, clients, _ = direct
    token = session.connect('socketcan', 'fixture-can')['session']
    network.add_node(4)
    clients[4].sdo.upload.side_effect = canopen.SdoCommunicationError('No SDO response received')
    motor = session.read(token, [4])['motors'][0]
    assert motor['online'] is False
    assert len(motor['fields']) == 1
    assert 'value' not in motor['fields'][0]
    assert 'No SDO response' in motor['fields'][0]['error']


def test_session_tokens_and_foreign_master_block_reads(direct):
    session, network, clients, _ = direct
    token = session.connect('socketcan', 'fixture-can')['session']
    with pytest.raises(ValueError, match='会话已失效'):
        session.read('old-token', [1])
    network.subscriptions[0x601](0x601, b'', 0)
    with pytest.raises(ValueError, match='其他 SDO 主站'):
        session.read(token, [1])
    assert clients == {}
    session.disconnect(token)
    with pytest.raises(ValueError, match='会话已失效'):
        session.heartbeat(token)


@pytest.mark.parametrize('transport, endpoint', [('shell', 'fixture-can'), ('socketcan', 'not-a-device'), ('slcan', 'socket://remote:123')])
def test_only_enumerated_local_endpoints_are_accepted(direct, transport: str, endpoint: str):
    session, _, _, factory = direct
    with pytest.raises(ValueError):
        session.connect(transport, endpoint)
    factory.assert_not_called()


@pytest.mark.parametrize('ids', [[], [0], [128], [True], ['1'], list(range(1, 18)), None])
def test_invalid_node_lists_never_open_device(direct, ids):
    session, _, _, factory = direct
    with pytest.raises(ValueError):
        session.read('', ids)
    factory.assert_not_called()


def test_network_sends_only_standard_fd_sdo_uploads():
    bus = Mock()
    network = FdSdoNetwork(bus)
    network.send_message(0x601, bytes.fromhex('4008100000000000'))
    message = bus.send.call_args.args[0]
    assert message.is_fd and message.bitrate_switch and not message.is_extended_id
    for can_id, data in [(0, bytes(8)), (0x201, bytes(8)), (0x601, bytes.fromhex('2300100000000000'))]:
        with pytest.raises(ValueError):
            network.send_message(can_id, data)
    assert bus.send.call_count == 1


def test_canopen_segmented_identity_upload_over_in_memory_bus():
    """真实 SDO 分段状态机，仅连接 python-can 内存总线。"""
    import uuid
    from canopen import objectdictionary as od

    channel = f'device-test-{uuid.uuid4()}'
    dictionary = canopen.ObjectDictionary()
    identity = od.ODVariable('Device name', 0x1008, 0)
    identity.data_type = od.VISIBLE_STRING
    identity.access_type = 'ro'
    identity.default = 'R48LITE_V2'
    dictionary.add_object(identity)
    server = canopen.Network()
    client = FdSdoNetwork()
    try:
        server.connect(interface='virtual', channel=channel)
        server.add_node(canopen.LocalNode(1, dictionary))
        client.connect(interface='virtual', channel=channel)
        node = client.add_node(1)
        assert node.sdo.upload(0x1008, 0) == b'R48LITE_V2'
    finally:
        client.disconnect()
        server.disconnect()


def test_discovery_uses_cerb_status_and_vaxor_reports_without_node_input(direct):
    import struct
    session, network, clients, _ = direct
    token = session.connect('socketcan', 'fixture-can')['session']
    # SDO 应答不建立设备，只有固件认可的存在证据帧建立设备。
    assert session.discover(token)['motors'] == []
    network.subscriptions[0x309](0x309, bytes(12), 0)
    status = bytes(2) + struct.pack('<IHHhIBBBB', 0x80000000, 0x8000, 0x8000, 240, 0, 0, 1, 2, 50)
    network.subscriptions[0x702 + 4 * 5](0x702 + 4 * 5, status, 0)
    motors = session.discover(token)['motors']
    assert [(m['protocol'], m['id']) for m in motors] == [('cerb', 9), ('vaxor', 5)]
    assert all(m['online'] for m in motors)
    fields = {f['key']: f['value'] for f in motors[1]['fields']}
    assert fields['position'] == 0
    assert fields['voltage'] == 24
    assert fields['motor_type'] == 50
    assert clients == {}


def test_discovery_marks_stale_feedback_offline(direct):
    import time
    session, network, _, _ = direct
    token = session.connect('socketcan', 'fixture-can')['session']
    network.subscriptions[0x301](0x301, bytes(12), 0)
    with session._rx_lock:
        session._seen['cerb:1']['last_seen'] = time.monotonic() - 1
    motor = session.discover(token)['motors'][0]
    assert motor['online'] is False
    assert motor['age_ms'] >= 1000


def test_second_session_cannot_steal_endpoint_and_lock_is_released(direct):
    session, _, _, factory = direct
    token = session.connect('socketcan', 'fixture-can')['session']
    other = DirectCanSession(factory, lambda: {'socketcan': ['fixture-can'], 'slcan': []})
    try:
        with pytest.raises(ValueError, match='已被另一个直连会话占用'):
            other.connect('socketcan', 'fixture-can')
        session.disconnect(token)
        result = other.connect('socketcan', 'fixture-can')
        assert other.heartbeat(result['session'])['connected']
    finally:
        other.close()
