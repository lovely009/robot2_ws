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


# ---- RP 编解码 ----
_ZERO = {"position": 0.0, "velocity": 0.0, "kp": 0.0, "kd": 0.0, "feedforward": 0.0}


def test_pack_rp_slot_zero_golden():
    assert mp.pack_rp_slot(_ZERO) == bytes.fromhex("7FFF7FF0000007FF")


def test_pack_rp_slot_clamps_kp():
    # kp 上限 500 -> 0xFFF；kd 上限 10 -> 0xFFF
    packed = mp.pack_rp_slot({"position": 0.0, "velocity": 0.0, "kp": 999.0,
                              "kd": 999.0, "feedforward": 0.0})
    assert (packed[3] & 0x0F) == 0x0F and packed[4] == 0xFF  # kp 满量程
    assert (packed[5] >> 4) == 0x0F and (packed[6] & 0xF0) == 0xF0  # kd 满量程


def test_pack_rp_command_fills_slots():
    payload = mp.pack_rp_command({1: _ZERO})
    assert len(payload) == 64
    assert payload[0:8] == mp.pack_rp_slot(_ZERO)   # slot 1（id=1）
    assert payload[8:16] == mp.pack_rp_slot(_ZERO)  # 未配置槽填中性槽
    assert payload[8:16] == payload[0:8]


def test_pack_rp_enable_bytes():
    assert mp.pack_rp_enable(True) == bytes([1, 0, 0, 0, 0, 0, 0, 0])
    assert mp.pack_rp_enable(False) == bytes([0, 0, 0, 0, 0, 0, 0, 0])


def test_unpack_rp_state_golden():
    data = bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x50,
                  0x12, 0x34, 0x18, 0x50, 0x00, 0x00])
    state = mp.unpack_rp_state(data)
    assert state["position"] == -10.0
    assert state["velocity"] == -60.0
    assert state["effort"] == -60.0
    assert state["mos_temperature_c"] == 40.0
    assert state["status"] == 0x1234
    assert state["voltage_v"] == 24.0
    assert state["coil_temperature_c"] == 40.0
