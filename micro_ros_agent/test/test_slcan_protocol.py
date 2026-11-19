"""保留真机工具的 SLCAN 线协议，测试不打开设备。"""
import importlib.util
from pathlib import Path

import pytest


SPEC = importlib.util.spec_from_file_location(
    "slcan_protocol", Path(__file__).parents[1] / "tools" / "slcan_protocol.py")
assert SPEC is not None
assert SPEC.loader is not None
protocol = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(protocol)


@pytest.mark.parametrize("length", [0, 1, 8, 12, 16, 20, 24, 32, 48, 64])
@pytest.mark.parametrize("brs", [False, True])
def test_fd_round_trip(length, brs):
    data = bytes(range(length))
    line = protocol.slcan_line(0x321, data, True, brs)
    assert line.startswith(b"b" if brs else b"d")
    assert protocol.parse_slcan(line.decode()) == (0x321, data, True)


def test_classic_and_extended_receive():
    assert protocol.slcan_line(0x123, b"\x01\x02", False, False) == b"t12320102\r"
    assert protocol.parse_slcan("T0000012320102\r") == (0x123, b"\x01\x02", False)


@pytest.mark.parametrize("line", ["", "V1234", "t1238AA", "tXYZ0", "d123Z"])
def test_invalid_receive_is_ignored(line):
    assert protocol.parse_slcan(line) is None


def test_send_validation_and_channel_configuration():
    with pytest.raises(ValueError):
        protocol.slcan_line(0x800, b"", True, True)
    with pytest.raises(ValueError):
        protocol.slcan_line(1, bytes(9), True, True)
    with pytest.raises(ValueError):
        protocol.slcan_config_commands(10)
    assert protocol.slcan_config_commands() == (
        b"C\r", b"S8\r", b"Y5\r", b"M0\r", b"A1\r", b"O\r")
