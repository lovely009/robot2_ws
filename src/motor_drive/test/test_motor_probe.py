"""motor_probe 的 slcan 帧收发与 vaxor 编解码测试。"""
import sys
from pathlib import Path

import pytest

SCRIPTS = Path(__file__).resolve().parent.parent / "scripts"
sys.path.insert(0, str(SCRIPTS))

import motor_probe as mp


# ---- slcan 帧收发 ----
def test_fd_len_code_mapping():
    assert mp.fd_len_code(8) == "8"
    assert mp.fd_len_code(12) == "9"
    assert mp.fd_len_code(20) == "B"
    assert mp.fd_len_code(64) == "F"


def test_fd_len_code_unknown_raises():
    with pytest.raises(ValueError):
        mp.fd_len_code(10)


def test_encode_classic_frame():
    data = bytes([1, 0, 0, 0, 0, 0, 0, 0])
    assert mp.encode_frame(0x506, data, is_fd=False, brs=False) == \
        "t50680100000000000000\r"


def test_encode_fd_frame_uses_b_and_len_code():
    data = bytes(12)
    assert mp.encode_frame(0x501, data, is_fd=True, brs=True) == \
        "b5019" + "00" * 12 + "\r"


def test_decode_frame_roundtrip_classic():
    frame = "t50680100000000000000"
    assert mp.decode_frame(frame) == (0x506, bytes([1, 0, 0, 0, 0, 0, 0, 0]), False)


def test_decode_frame_roundtrip_fd():
    frame = mp.encode_frame(0x501, bytes(12), is_fd=True, brs=True)
    assert mp.decode_frame(frame) == (0x501, bytes(12), True)


def test_decode_frame_ignores_garbage():
    assert mp.decode_frame("Z123") is None
    assert mp.decode_frame("") is None


# ---- vaxor 码值换算 ----
def test_vaxor_position_zero_bias():
    assert mp.pos_to_code(0.0) == 0x80000000
    assert mp.code_to_pos(0x80000000) == 0.0


def test_vaxor_velocity_zero_bias():
    assert mp.vel_to_code(0.0) == 0x8000
    assert mp.code_to_vel(0x8000) == 0.0


def test_vaxor_current_zero_bias():
    assert mp.current_to_code(0.0) == 0x8000
    assert mp.code_to_current(0x8000) == 0.0


# ---- vaxor 组帧/控制 ----
def test_pack_wrist_command_golden():
    # 12 字节：pos(4LE)=0x80000000, vel(2)=0x8000, kp(2)=0, kd(2)=0, cur(2)=0x8000
    assert mp.pack_wrist_command(0.0, 0.0, 0.0, 0.0, 0.0) == \
        bytes.fromhex("000000800080000000000080")


def test_pack_wrist_control_bytes():
    assert mp.pack_wrist_control(True) == bytes([1, 0, 0, 0, 0, 0, 0, 0])
    assert mp.pack_wrist_control(False, set_zero=True) == bytes([0, 1, 0, 0, 0, 0, 0, 0])


# ---- vaxor 反馈解码 ----
def _status_frame():
    return bytes.fromhex("0505000000800080008078000100000004020101")


def test_decode_vaxor_status_frame():
    state = mp.decode_vaxor_feedback(0x702, _status_frame(), node=0)
    assert state is not None
    assert state["position"] == 0.0
    assert state["velocity"] == 0.0
    assert state["current_a"] == 0.0
    assert state["voltage_v"] == 12.0
    assert state["fault"] == 1
    assert state["motor_state"] == 4
    assert state["calibration_state"] == 2
    assert state["mode"] == 1
    assert state["motor_type"] == 1


def test_decode_vaxor_reply_frame():
    reply = bytes.fromhex("050500000080008000800000")
    state = mp.decode_vaxor_feedback(0x701, reply, node=0)
    assert state is not None
    assert "voltage_v" not in state  # reply 帧无状态字段
    assert state["current_a"] == 0.0


def test_decode_vaxor_wrong_id_returns_none():
    assert mp.decode_vaxor_feedback(0x706, _status_frame(), node=0) is None
