#!/usr/bin/env python3
"""Regression checks for FDCAN simulator startup, setup and feedback scheduling."""

import json
import os
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
from unittest import mock

SCRIPT_PATH = Path(__file__).parents[3] / "micro_ros_agent" / "tools" / "fdcan_motor_sim.py"
SPEC = spec_from_file_location("fdcan_motor_sim", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)

# 非中点速度必须独立于 Kp 的高半字节，缩放采用厂家指南默认值。
_guide_frame = bytes.fromhex("cc a8 c3 ab c0 31 00 88 00 00 00 00")
_guide_cmd = MODULE.cerb_decode_command(_guide_frame)
assert _guide_cmd is not None
assert abs(_guide_cmd["pos_rad"] - 1.0) <= 6.28 / 65535
assert abs(_guide_cmd["vel_rad_s"] - (0xABC / 4095 * 6.28 - 3.14)) < 1e-12
assert abs(_guide_cmd["kp"] - 49 / 4095 * 2500) < 1e-12
assert abs(_guide_cmd["kd"] - 8 / 4095 * 500) < 1e-12
assert MODULE.cerb_decode_command(bytes([0xCC]) + bytes(11)) is None
assert MODULE.cerb_decode_command(bytes([0xC2]) + _guide_frame[1:]) is None

assert MODULE.DEFAULT_DATA_BITRATE_INDEX == 5
# CANable 固件 16e7497 的 5Mbps 索引为 Y5，非 Y7；按固件版本表选择。
assert MODULE.SLCAN_VERSION_DATA_BITRATE["16e7497"] == 5
assert MODULE.SLCAN_VERSION_DATA_BITRATE["b158aa7"] == 5
assert MODULE.slcan_config_commands(5) == (
    b"C\r", b"S8\r", b"Y5\r", b"M0\r", b"A1\r", b"O\r",
)
assert MODULE.slcan_config_commands(7) == (
    b"C\r", b"S8\r", b"Y7\r", b"M0\r", b"A1\r", b"O\r",
)

try:
    MODULE.slcan_config_commands(10)
except ValueError:
    pass
else:
    raise AssertionError("multi-digit SLCAN data bitrate index was accepted")

pending = {}
MODULE.schedule_feedback_window(pending, [7, 8, 9], 1.0, 0.0022)
assert pending == {7: [1.0], 8: [1.0022], 9: [1.0044]}

# 下一 SYNC 窗口既不能覆盖旧 deadline，也不能丢掉当前窗口的新 deadline。
pending[7].pop(0)
pending[8].pop(0)
MODULE.schedule_feedback_window(pending, [7, 8, 9], 1.02, 0.0022)
assert pending[9] == [1.0044, 1.0244]
assert pending[7] == [1.02]
assert pending[8] == [1.0222]

# 身份分段上传字节格式须与固件 cerb_adapter 匹配。
sim = MODULE.CanMotorSim.__new__(MODULE.CanMotorSim)
sim.models = {1: MODULE.MotorModel(1, "cerb")}
sim.use_brs = True
sim.sent = []
sim.send = lambda can_id, payload, is_fd=True, brs=True, model=None, tag="": \
    sim.sent.append((can_id, payload, tag))

sim.handle_sdo(1, bytes([0x40, 0x08, 0x10, 0x00, 0, 0, 0, 0]))
assert sim.sent[-1][0] == 0x581
assert sim.sent[-1][1] == bytes([0x41, 0x08, 0x10, 0x00, 10, 0, 0, 0])  # 字节4-5 = 总长

sim.handle_sdo(1, bytes([0x60, 0, 0, 0, 0, 0, 0, 0]))  # 切换位 0，取首段
assert sim.sent[-1][0] == 0x581
assert sim.sent[-1][1] == bytes([0x00]) + b"R48LITE"  # 非末段：t=0/c=0

sim.handle_sdo(1, bytes([0x70, 0, 0, 0, 0, 0, 0, 0]))  # 切换位 1，取续段
# 末段 head = t(1)<<4 | 未用字节(4)<<1 | c(1) = 0x19，数据 "_V2" + 4 字节填充
assert sim.sent[-1][1] == bytes([0x19]) + b"_V2" + b"\x00" * 4

# 非 0x1008 的对象请求 → SDO abort（固件据此判读失败，不拿别的对象绑型号）。
sim.handle_sdo(1, bytes([0x40, 0x00, 0x10, 0x00, 0, 0, 0, 0]))
assert sim.sent[-1][1][0] == 0x80
# 设备限值对象（0x607D:01/02、0x6080、0x2538）：写 0x23 → 0x60 ack 并存储；
# 读 0x40 → 0x43 + 存储值（固件写后读回核验与 REFRESH 参数回读的对端）。
sim.handle_sdo(1, bytes([0x23, 0x7D, 0x60, 0x01, 0x42, 0xD7, 0xFF, 0xFF]))
assert sim.sent[-1][1] == bytes([0x60, 0x7D, 0x60, 0x01, 0, 0, 0, 0])
sim.handle_sdo(1, bytes([0x40, 0x7D, 0x60, 0x01, 0, 0, 0, 0]))
assert sim.sent[-1][1] == bytes([0x43, 0x7D, 0x60, 0x01, 0x42, 0xD7, 0xFF, 0xFF])
sim.handle_sdo(1, bytes([0x23, 0x80, 0x60, 0x00, 0x58, 0x02, 0, 0]))
assert sim.sent[-1][1] == bytes([0x60, 0x80, 0x60, 0x00, 0, 0, 0, 0])
sim.handle_sdo(1, bytes([0x40, 0x80, 0x60, 0x00, 0, 0, 0, 0]))
assert sim.sent[-1][1] == bytes([0x43, 0x80, 0x60, 0x00, 0x58, 0x02, 0, 0])
sim.handle_sdo(1, bytes([0x23, 0x38, 0x25, 0x00, 0xE8, 0x03, 0, 0]))
sim.handle_sdo(1, bytes([0x40, 0x38, 0x25, 0x00, 0, 0, 0, 0]))
assert sim.sent[-1][1] == bytes([0x43, 0x38, 0x25, 0x00, 0xE8, 0x03, 0, 0])
# 出厂默认：0x607D:01/02 = ±32768 cnt（±180°）。
sim2 = MODULE.MotorModel(3, "cerb")
assert sim2.sdo_params[(0x607D, 1)] == -32768
assert sim2.sdo_params[(0x607D, 2)] == 32768

# 中止码分层：不存在的索引 0x9999:00 → 0x06020000；
# 已有索引的无效子索引 0x6080:01 → 0x06090011。
sim.handle_sdo(1, bytes([0x40, 0x99, 0x99, 0x00, 0, 0, 0, 0]))
assert sim.sent[-1][1] == bytes([0x80, 0x99, 0x99, 0x00, 0x00, 0x00, 0x02, 0x06])
sim.handle_sdo(1, bytes([0x40, 0x80, 0x60, 0x01, 0, 0, 0, 0]))
assert sim.sent[-1][1] == bytes([0x80, 0x80, 0x60, 0x01, 0x11, 0x00, 0x09, 0x06])
# 身份串只挂在 0x1008:00；真机 :01/:02 回"子索引不存在"，这里不能回身份串。
sim.handle_sdo(1, bytes([0x40, 0x08, 0x10, 0x01, 0, 0, 0, 0]))
assert sim.sent[-1][1] == bytes([0x80, 0x08, 0x10, 0x01, 0x11, 0x00, 0x09, 0x06])

# 实测出厂值：R58 为 3325RPM/13300mA，R48 为 3325RPM/5000mA，身份与参数同源。
r58 = MODULE.MotorModel(1, "cerb")
MODULE.apply_cerb_device_model(r58, "r58")
assert r58.identity == b"R58LITE_V2"
assert r58.sdo_params[(0x6080, 0)] == 3325
assert r58.sdo_params[(0x2538, 0)] == 13300
r48 = MODULE.MotorModel(2, "cerb")
MODULE.apply_cerb_device_model(r48, "r48")
assert r48.identity == b"R48LITE_V2"
assert r48.sdo_params[(0x6080, 0)] == 3325
assert r48.sdo_params[(0x2538, 0)] == 5000

# 非 CERB 节点（无此模型/是 vaxor）不回 SDO。
sent_before = len(sim.sent)
sim.handle_sdo(9, bytes([0x40, 0x08, 0x10, 0x00, 0, 0, 0, 0]))
assert len(sim.sent) == sent_before

# 反馈温度位于 data[8..9]，大端 int16、0.1°C；名义值与 C++ motor_sim 一致。
frame = MODULE.MotorModel(4, "cerb").cerb_frame()
assert len(frame) == 12
assert frame[8:10] == round(MODULE.CERB_NOMINAL_TEMPERATURE_C * 10).to_bytes(2, "big")
assert frame[11] & 0x20 == 0  # 无故障

# R58 身份分配：--cerb-r58 指定的节点自报 R58LITE_V2。
sim.models[2] = MODULE.MotorModel(2, "cerb")
sim.models[2].identity = b"R58LITE_V2"
sim.handle_sdo(2, bytes([0x40, 0x08, 0x10, 0x00, 0, 0, 0, 0]))
assert sim.sent[-1][1] == bytes([0x41, 0x08, 0x10, 0x00, 10, 0, 0, 0])

# VAXOR 命令从字节 0 起为位置、速度、kp、kd、Iq，并验证电机到关节的 50:1 换算。
_vaxor_payload = (
    MODULE.vaxor_pos_code(1.25)
    + MODULE.vaxor_vel_code(2.0)
    + int(round(0.2 / MODULE.VAXOR_KP_SCALE)).to_bytes(2, "little")
    + int(round(0.8 / MODULE.VAXOR_KD_SCALE)).to_bytes(2, "little")
    + MODULE.vaxor_cur_code(0.4)
)
_vaxor_cmd = MODULE.vaxor_decode_command(_vaxor_payload)
assert _vaxor_cmd is not None
assert abs(_vaxor_cmd["position_motor_rad"] - 1.25) < 2e-5
assert abs(_vaxor_cmd["pos_rad"] - 1.25 / 50.0) < 2e-5
assert abs(_vaxor_cmd["velocity_motor_rad_s"] - 2.0) < 2e-3
assert abs(_vaxor_cmd["kp"] - 0.2) < 1e-6
assert abs(_vaxor_cmd["kd"] - 0.8) < 1e-6
assert abs(_vaxor_cmd["tau_nm"] - 0.4 * 0.042 * 50.0 * 0.65) < 2e-3
assert MODULE.vaxor_decode_command(_vaxor_payload[:-1]) is None

# 多轴 0x210 一帧的两个 ID 必须各自更新对应模型；空槽不可触碰其他节点。
multi = MODULE.CanMotorSim.__new__(MODULE.CanMotorSim)
multi.models = {i: MODULE.MotorModel(i, "cerb") for i in (1, 2, 3)}
multi.log_f = None
multi.mit_torque_limits = {}
mit1 = bytes([0xcc, 0x80, 0, 0x80, 3, 0xe8, 0x14, 0x18, 0])
mit2 = bytes([0x0c, 0x80, 1, 0x80, 3, 0xe8, 0x14, 0x18, 1])
payload = (mit1 + mit2 + bytes(36) + bytes(2)
           + bytes([1, 2, 0, 0, 0, 0]) + bytes(2))
assert len(payload) == 64
multi.handle_downlink(0x210, payload, True)
assert multi.models[1].last_cmd is not None and multi.models[1].enabled
assert multi.models[2].last_cmd is not None and not multi.models[2].enabled
assert multi.models[3].last_cmd is None

# MIT 解码量程取逐型号配置，R58/R48 默认均为 300N·m，不能用物理峰值 45/11 替代。
def _mit_tau_frame(tau_nm: float, limit: float) -> bytes:
    code = round((tau_nm + limit) / (2.0 * limit) * 4095)
    frame = bytearray.fromhex("cc 80 00 80 00 00 00 00 00 00 00 00")
    frame[7] = (code >> 8) & 0x0F
    frame[8] = code & 0xFF
    return bytes(frame)


for _limit in (300.0, 11.0, 45.0):
    _decoded = MODULE.cerb_decode_command(_mit_tau_frame(5.0, _limit), _limit)
    assert abs(_decoded["tau_nm"] - 5.0) < 0.08, (_limit, _decoded["tau_nm"])

# 未逐台指定量程时按厂家真值 300 解码（不能退回物理峰值 45/11）。
assert abs(MODULE.cerb_decode_command(_mit_tau_frame(5.0, 300.0))["tau_nm"] - 5.0) < 0.08
_scale = MODULE.CanMotorSim.__new__(MODULE.CanMotorSim)
_scale.models = {1: MODULE.MotorModel(1, "cerb")}
_scale.log_f = None
_scale.mit_torque_limits = {}
_scale.handle_downlink(0x111, _mit_tau_frame(5.0, 300.0), True)
assert abs(_scale.models[1].last_cmd["tau_nm"] - 5.0) < 0.08, \
    _scale.models[1].last_cmd["tau_nm"]
_scale.mit_position_limits = {1: 6.28}
_scale.mit_velocity_limits = {1: 6.28}
_scale.mit_kp_limits = {1: 5000.0}
_scale.mit_kd_limits = {1: 1000.0}
_scale.handle_downlink(0x111, _guide_frame, True)
assert abs(_scale.models[1].last_cmd["pos_rad"] - 2.0) < 0.0002
assert abs(_scale.models[1].last_cmd["kp"] - 49 / 4095 * 5000) < 1e-12


# 实测上电初值：保护原值 1（关）、方向 0（CW）；由固件上电归一。
_cerb = MODULE.MotorModel(1, "cerb")
assert _cerb.sdo_params[(0x2650, 0x00)] == 1, "保护上电初值应为 1（关）"
assert _cerb.sdo_params[(0x2541, 0x00)] == 0, "方向上电初值应为 0（CW）"


# joint-map 重号由后者静默覆盖，不报冲突；与固件的重号检测语义不同。
_jm = MODULE.parse_joint_map("1:joint_link1,2:joint_link2")
assert _jm == {1: "joint_link1", 2: "joint_link2"}
assert MODULE.parse_joint_map("1:joint_a,1:joint_b") == {1: "joint_b"}  # 后者覆盖
assert MODULE.parse_joint_map("1:a,,2:b,") == {1: "a", 2: "b"}  # 空项跳过
assert MODULE.parse_joint_map(" 1 : a ") == {1: "a"}  # 两侧空白容忍


# 降载反馈的墙钟兜底：fb-period-ms 是窗口周期上界，不依赖实际 SYNC 频率，
# 避免反馈间隔超过固件 100ms 离线门。
def _bare_sim(fb_period_ms=60, stagger_ms=2.2):
    s = MODULE.CanMotorSim.__new__(MODULE.CanMotorSim)
    s.models = {1: MODULE.MotorModel(1, "cerb"),
                2: MODULE.MotorModel(2, "vaxor")}
    s.next_send = {}
    s.fb_period = fb_period_ms / 1000.0
    s.stagger = stagger_ms / 1000.0
    s.last_window_at = 100.0
    s.window_count = 0
    return s


# 默认保真模式：实测 TPDO 0x1800:02=1，每个 SYNC 回一帧，无错峰或墙钟兜底。
assert MODULE.feedback_cadence(20, 0, 2.2) == (True, 1, 0.0, float("inf"))
_dev, _interval, _stagger, _period = MODULE.feedback_cadence(20, 60, 2.2)
assert (_dev, _interval, abs(_stagger - 0.0022) < 1e-9) == (False, 3, True)
assert abs(_period - 0.06) < 1e-9
# 降载模式下 fb_period 是**上界**：SYNC 快于假定值时不额外抽帧（round 到 1）。
assert MODULE.feedback_cadence(20, 20, 0.0)[1] == 1
for _bad in ((0, 60, 2.2), (20, -1, 2.2)):
    try:
        MODULE.feedback_cadence(*_bad)
    except ValueError:
        pass
    else:
        raise AssertionError(f"非法节拍参数被接受：{_bad}")

_w = _bare_sim()
# 未到周期：不补，且不留任何 deadline
assert not _w.maybe_fallback_window(100.0 + 0.059)
assert _w.window_count == 0 and _w.next_send == {}
# 到周期：补一轮，两台各一条 deadline，按 stagger 错峰
assert _w.maybe_fallback_window(100.0 + 0.060)
assert _w.window_count == 1
assert len(_w.next_send[1]) == 1 and abs(_w.next_send[1][0] - 100.06) < 1e-9
assert len(_w.next_send[2]) == 1
assert abs(_w.next_send[2][0] - _w.next_send[1][0] - 0.0022) < 1e-9
# "到点才补"，不是每拍都补：紧接着的下一拍不补
assert not _w.maybe_fallback_window(100.0 + 0.061)
assert _w.window_count == 1
# 无 SYNC 时仍按 fb_period 出窗口；用二进制精确的 1/16 秒避免边界浮点误差。
_we = _bare_sim()
_we.fb_period = 0.0625
for _i in range(5):
    assert _we.maybe_fallback_window(_we.last_window_at + 0.0625)
assert _we.window_count == 5, _we.window_count
assert _we.last_window_at == 100.0 + 0.0625 * 5

# 同一 21ms 间隔在 20/60ms 周期下结论相反；避开 20ms 浮点边界。
_w60_ref = _bare_sim()          # 默认 60ms
_w20 = _bare_sim(fb_period_ms=20)
assert not _w20.maybe_fallback_window(100.0 + 0.019)
assert not _w60_ref.maybe_fallback_window(100.0 + 0.021)  # 60ms 周期：还没到
assert _w20.maybe_fallback_window(100.0 + 0.021)          # 20ms 周期：到点了
assert _w20.window_count == 1
assert _w60_ref.window_count == 0 and _w60_ref.next_send == {}


# 反馈电流/转速取本拍实际施加的关节力矩与速度，换算与 C++ frame_router 一致。
_fb = MODULE.MotorModel(1, "cerb", gear_ratio=101.0, joint_nm_per_amp=7.777)
# ① 未施加力矩（未使能 / 无命令 / 固件未授权）→ 电流 0、转速 0。
assert _fb.report_current_a() == 0.0
assert _fb.report_motor_rpm() == 0.0
_fb_zero = _fb.cerb_frame()
assert _fb_zero[2:4] == (0).to_bytes(2, "big")                   # 转速
assert _fb_zero[4:6] == (0).to_bytes(2, "big")                   # Iq
# ② 施加 7.777 N·m（= 1 A × Kt×速比×效率）→ 上报 50 码（20 mA/码 = 1 A）。
_fb.applied_joint_nm = 7.777
assert abs(_fb.report_current_a() - 1.0) < 1e-9
assert _fb.cerb_frame()[4:6] == (50).to_bytes(2, "big")
# ③ 转速按速比换算到电机侧：关节 1 rad/s × 101 → 964.4 RPM。
_fb.vel_joint_rad_s = 1.0
assert abs(_fb.report_motor_rpm() - 101.0 * 60.0 / (2 * 3.141592653589793)) < 1e-6
# ④ 缺型号参数（型号库里查不到这台）时报 0，不猜系数——vaxor 有内置速比 50，
#    cerb 没有可退的速比，两者都不能凭空造一个 Kt。
_unknown = MODULE.MotorModel(9, "cerb")
assert _unknown.report_current_a() == 0.0 and _unknown.report_motor_rpm() == 0.0
_vaxor_fb = MODULE.MotorModel(7, "vaxor", joint_nm_per_amp=1.365)
assert abs(_vaxor_fb.report_motor_rad_s() - 0.0) < 1e-12   # 静止
_vaxor_fb.vel_joint_rad_s = 0.5
assert abs(_vaxor_fb.report_motor_rad_s() - 25.0) < 1e-9   # 内置速比 50
_vaxor_fb.applied_joint_nm = 1.365
assert abs(_vaxor_fb.report_current_a() - 1.0) < 1e-9
_vaxor_frame = _vaxor_fb.vaxor_reply_frame()
assert len(_vaxor_frame) == 12
# 帧布局：id(2) | 位置(4) | 速度(2) | 电流(2) | 填充(2)。固件只读 2..9。
assert abs(MODULE.vaxor_decode_vel(_vaxor_frame[6:8]) - 25.0) < 0.02
assert abs(MODULE.vaxor_decode_current(_vaxor_frame[8:10]) - 1.0) < 0.01


# CERB 反馈越界须钳位到 int16，不能因 struct.error 中断模拟器。
_cb = MODULE.MotorModel(1, "cerb", gear_ratio=101.0, joint_nm_per_amp=7.777)
_cb.pos_deg = 400.0     # 远超 ±180°（int16 只能表示 ±180.04°）
_cb.vel_joint_rad_s = 10 ** 9
_cb.applied_joint_nm = -(10 ** 9)
_cb.temp_c = 10 ** 6
_frame = _cb.cerb_frame()  # 修复前这里抛 struct.error
assert len(_frame) == 12
assert _frame[0:2] == (32767).to_bytes(2, "big")                 # 正越界 → 上端
assert _frame[2:4] == (32767).to_bytes(2, "big")                 # rpm 正越界
assert _frame[4:6] == (-32768).to_bytes(2, "big", signed=True)   # Iq 负越界
assert _frame[8:10] == (32767).to_bytes(2, "big")                # 温度越界

_cb2 = MODULE.MotorModel(2, "cerb")
_cb2.pos_deg = -400.0
assert _cb2.cerb_frame()[0:2] == (-32768).to_bytes(2, "big", signed=True)

# 量程内不得变形（夹取只对越界生效）
_cb3 = MODULE.MotorModel(3, "cerb")
_cb3.pos_deg = 90.0
assert _cb3.cerb_frame()[0:2] == (90 * 182).to_bytes(2, "big", signed=True)


# 运行期控制：add 新建、remove 移除；link=False 不发反馈、不回 SDO、不处理下行。
# 恢复连接清命令，按重新上电处理。
_cnt = {1: MODULE.MotorModel(1, "cerb")}
_joints = {1: "joint_link1"}
_r = MODULE.apply_sim_control(_cnt, {"cmd": "status"}, _joints)
assert _r["ok"] and _r["motors"][0]["joint"] == "joint_link1"

_r = MODULE.apply_sim_control(_cnt, {"cmd": "add", "id": 7, "kind": "vaxor"})
assert _r["ok"] and _cnt[7].kind == "vaxor" and _cnt[7].linked
# 重号拒绝：不能静默覆盖已有模型
assert not MODULE.apply_sim_control(
    _cnt, {"cmd": "add", "id": 7, "kind": "vaxor"})["ok"]
# R58 身份由 add 参数指定（缺省 R48，与启动期 --cerb-r58 的语义一致）
assert MODULE.apply_sim_control(
    _cnt, {"cmd": "add", "id": 5, "kind": "cerb", "model": "r58"})["ok"]
assert _cnt[5].identity == b"R58LITE_V2"
assert MODULE.apply_sim_control(
    _cnt, {"cmd": "add", "id": 6, "kind": "cerb"})["ok"]
assert _cnt[6].identity == b"R48LITE_V2"
# id 上限：CERB ≤127（0x300+id 与 SDO 0x580+node），VAXOR ≤63（0x501+4n 不出 0x600）
assert not MODULE.apply_sim_control(
    _cnt, {"cmd": "add", "id": 128, "kind": "cerb"})["ok"]
assert not MODULE.apply_sim_control(
    _cnt, {"cmd": "add", "id": 64, "kind": "vaxor"})["ok"]
assert not MODULE.apply_sim_control(_cnt, {"cmd": "add", "id": 0, "kind": "cerb"})["ok"]
assert not MODULE.apply_sim_control(_cnt, {"cmd": "frobnicate"})["ok"]

# link=False 三处拦截：下行路由、SDO 应答、反馈窗口
_sim = MODULE.CanMotorSim.__new__(MODULE.CanMotorSim)
_sim.models = _cnt
_sim.next_send = {}
_r = MODULE.apply_sim_control(_cnt, {"cmd": "link", "id": 1, "up": False})
assert _r["ok"] and not _cnt[1].linked
assert _sim._model_for_downlink(0x110 + 1) is None       # 下行不再归属
assert _sim._model_for_downlink(0x110 + 5) is not None
_sim.use_brs = True
_sim.sent = []
_sim.send = lambda *a, **kw: _sim.sent.append(a)
_sim.handle_sdo(1, bytes([0x40, 0x08, 0x10, 0x00, 0, 0, 0, 0]))
assert not _sim.sent, "静默电机的 SDO 不应答"
_sim.models[1].linked = True
_sim.handle_sdo(1, bytes([0x40, 0x08, 0x10, 0x00, 0, 0, 0, 0]))
assert _sim.sent, "恢复后 SDO 照常应答"

# link=True = 重新上电：清 enabled/last_cmd（不残留旧运动授权）
_cnt[1].enabled = True
_cnt[1].last_cmd = {"enabled": True}
_r = MODULE.apply_sim_control(_cnt, {"cmd": "link", "id": 1, "up": True})
assert _r["ok"] and _cnt[1].linked and not _cnt[1].enabled \
    and _cnt[1].last_cmd is None

# apply_control 随 link/remove 清掉残留的待发 deadline
_purge = MODULE.CanMotorSim.__new__(MODULE.CanMotorSim)
_purge.models = {1: MODULE.MotorModel(1, "cerb")}
_purge.next_send = {1: [0.0]}
assert _purge.apply_control({"cmd": "link", "id": 1, "up": False})["ok"]
assert _purge.next_send == {}
assert _purge.apply_control({"cmd": "remove", "id": 1})["ok"]
assert 1 not in _purge.models

# register_feedback_window 只排 linked 电机（静默电机不占窗口）
_win = MODULE.CanMotorSim.__new__(MODULE.CanMotorSim)
_win.models = {1: MODULE.MotorModel(1, "cerb"),
               2: MODULE.MotorModel(2, "cerb")}
_win.models[1].linked = False
_win.next_send = {}
_win.stagger = 0.0022
_win.last_window_at = 0.0
_win.window_count = 0
_win.register_feedback_window(10.0)
assert sorted(_win.next_send) == [2]

# 0x210 多轴快照对静默电机不生效（帧里仍带它的槽位）
_multi = MODULE.CanMotorSim.__new__(MODULE.CanMotorSim)
_multi.models = {1: MODULE.MotorModel(1, "cerb")}
_multi.models[1].linked = False
_multi.log_f = None
_multi.mit_torque_limits = {}
_multi_payload = _mit_tau_frame(0.0, 300.0)[:9] + bytes(47) + bytes([1, 0, 0, 0, 0, 0]) + bytes(2)
assert len(_multi_payload) == 64
_multi.handle_downlink(0x210, _multi_payload, True)
assert _multi.models[1].last_cmd is None
_multi.models[1].linked = True
_multi.handle_downlink(0x210, _multi_payload, True)
assert _multi.models[1].last_cmd is not None

# 控制 socket 端到端（真实 Unix socket，不碰 CAN 总线）：一行 JSON 一问一答、
# 0600 权限、teardown 后死文件可复用。
_ctrl = MODULE.CanMotorSim.__new__(MODULE.CanMotorSim)
_ctrl.models = {1: MODULE.MotorModel(1, "cerb")}
_ctrl.next_send = {}
_ctrl.control_socket_path = os.path.join(tempfile.mkdtemp(), "sim.sock")
_ctrl._control_srv = None
_ctrl._setup_control_socket()
assert os.stat(_ctrl.control_socket_path).st_mode & 0o777 == 0o600
_client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
_client.settimeout(2.0)
_client.connect(_ctrl.control_socket_path)
_client.sendall(b'{"cmd":"add","id":9,"kind":"cerb","model":"r58"}\n')
_ctrl._service_control_socket()
_reply = b""
while b"\n" not in _reply:
    _reply += _client.recv(4096)
assert json.loads(_reply.split(b"\n", 1)[0])["ok"]
assert _ctrl.models[9].identity == b"R58LITE_V2"
# 服务端一连接一命令（答完即关），第二条命令要重新连接
_client.close()
_client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
_client.settimeout(2.0)
_client.connect(_ctrl.control_socket_path)
_client.sendall(b"not json\n")
_ctrl._service_control_socket()
_reply = b""
while b"\n" not in _reply:
    _reply += _client.recv(4096)
assert json.loads(_reply.split(b"\n", 1)[0])["ok"] is False
_client.close()
_ctrl._teardown_control_socket()
assert not os.path.exists(_ctrl.control_socket_path)
# 路径被活实例占用 → 拒绝启动（两个模拟器同一条总线会互相打断）
_ctrl2 = MODULE.CanMotorSim.__new__(MODULE.CanMotorSim)
_ctrl2.control_socket_path = os.path.join(
    os.path.dirname(_ctrl.control_socket_path), "taken.sock")
_ctrl2._control_srv = None
_ctrl2._setup_control_socket()
_ctrl3 = MODULE.CanMotorSim.__new__(MODULE.CanMotorSim)
_ctrl3.control_socket_path = _ctrl2.control_socket_path
_ctrl3._control_srv = None
try:
    _ctrl3._setup_control_socket()
except SystemExit:
    pass
else:
    raise AssertionError("控制 socket 被占用时应拒绝启动")
_ctrl2._teardown_control_socket()


# 反馈电流 = 本拍实际施加力矩 ÷ (Kt×速比×效率)。
# 仅实际发布的 effort 记入出力，未使能/无命令/撤权/位置模式均为 0A。
class _Pub:
    def __init__(self):
        self.msg = None

    def publish(self, msg):
        self.msg = msg


class _Arr:
    def __init__(self):
        self.data = []


def _bridge():
    """不碰 rclpy 的桥（host 测试无 ROS）：只装 publish_commands 用到的字段。"""
    b = MODULE.GazeboBridge.__new__(MODULE.GazeboBridge)
    b.joint_map = {1: "joint_link1", 2: "joint_link2"}
    b.joints = ["joint_link1", "joint_link2"]
    b.pos = {"joint_link1": 0.0, "joint_link2": 0.0}
    b.vel = {"joint_link1": 0.0, "joint_link2": 0.0}
    b.hold_pos = {"joint_link1": 0.0, "joint_link2": 0.0}
    b.control_mode = "effort"
    b.tau_limit = 10.0
    b.tau_limits = {}
    b.publish_interval = 0.0
    b.last_pub = 0.0
    b.fw_lifecycle = {}
    b._msg_type = _Arr
    b.pub = _Pub()
    return b


def _model(mid, kind="cerb"):
    m = MODULE.MotorModel(mid, kind, gear_ratio=101.0, joint_nm_per_amp=7.777)
    m.enabled = True
    m.last_cmd = {"kp": 30.0, "kd": 1.0, "pos_rad": 0.0, "vel_rad_s": 0.0,
                  "tau_nm": 0.5}
    return m


_b = _bridge()
_ms = {1: _model(1), 2: _model(2)}
_ms[2].enabled = False                      # 未使能
_b.publish_commands(_ms)
assert _ms[1].applied_joint_nm == 0.5, _ms[1].applied_joint_nm   # 前馈直通
assert _ms[2].applied_joint_nm == 0.0, "未使能的电机必须报 0 力矩"

# 撤掉命令后下一拍出力归零。
_ms[1].last_cmd = None
_b.publish_commands(_ms)
assert _ms[1].applied_joint_nm == 0.0, "没有命令就不该出力"

# 固件撤权（非 ACTIVE）即使主机侧还有命令也不发力矩。
_ms[1].last_cmd = {"kp": 30.0, "kd": 1.0, "pos_rad": 0.0, "vel_rad_s": 0.0,
                   "tau_nm": 0.5}
_b.fw_lifecycle = {1: 1}                    # 非 LIFECYCLE_ACTIVE
_b.publish_commands(_ms)
assert _ms[1].applied_joint_nm == 0.0, "固件撤权后不得有幽灵力矩"

# 授权恢复 ⇒ 力矩恢复；饱和后记的必须是**钳位后**的值（反馈与出力一致）。
_b.fw_lifecycle = {1: MODULE.LIFECYCLE_ACTIVE}
_ms[1].last_cmd["tau_nm"] = 100.0
_b.publish_commands(_ms)
assert _ms[1].applied_joint_nm == 10.0, _ms[1].applied_joint_nm
assert _b.pub.msg.data == [10.0, 0.0]

# 逐台物理峰值（--tau-limits）仍按型号峰值钳位——MIT 解码量程分离后不能顺带
# 把出力权限改成解码值 300。
_b.tau_limits = {1: 45.0, 2: 11.0}
_b.publish_commands(_ms)
assert _ms[1].applied_joint_nm == 45.0, _ms[1].applied_joint_nm
assert _b.pub.msg.data == [45.0, 0.0]


# 用真实子进程验证 main 能走到设备打开阶段，而非提前因变量名错误失败。
_startup = subprocess.run(
    [sys.executable, str(SCRIPT_PATH), "--tty", "/nonexistent/robot_ws-probe"],
    capture_output=True, text=True, encoding="utf-8", errors="replace",
    timeout=15,  # 挂住即判失败；run() 超时会杀掉子进程并抛 TimeoutExpired
)
assert _startup.returncode != 0, _startup.stdout
assert "UnboundLocalError" not in _startup.stderr, _startup.stderr
assert "NameError" not in _startup.stderr, _startup.stderr
# 失败须为探针路径的 FileNotFoundError，证明已完成构造前的启动路径。
assert "FileNotFoundError" in _startup.stderr, _startup.stderr
assert "/nonexistent/robot_ws-probe" in _startup.stderr, _startup.stderr

# tau-limits 仅作物理饱和，MIT 解码量程独立使用 mit-torque-limits。
class _StopMain(Exception):
    pass


def _capture_main_sim_and_bridge(argv):
    sim_kwargs = {}
    bridge_args = []

    class _FakeSim:
        def __init__(self, *args, **kwargs):
            sim_kwargs.update(kwargs)

    class _FakeBridge:
        def __init__(self, *args, **kwargs):
            bridge_args.append((args, kwargs))
            raise _StopMain

    old_argv = sys.argv
    sys.argv = [str(SCRIPT_PATH), *argv]
    try:
        with mock.patch.object(MODULE, "CanMotorSim", _FakeSim), \
                mock.patch.object(MODULE, "GazeboBridge", _FakeBridge):
            MODULE.main()
    except _StopMain:
        pass
    else:
        raise AssertionError("main() 未走到 GazeboBridge 构造")
    finally:
        sys.argv = old_argv
    assert len(bridge_args) == 1, bridge_args
    return sim_kwargs, bridge_args[0]


_COMMON_ARGV = [
    "--tty", "/nonexistent/robot_ws-probe", "--cerb", "1,2",
    "--joint-map", "1:joint_link1,2:joint_link2",
    "--joint-nm-per-amp", "1:5.0753,2:5.05",
    "--gazebo", "--effort-rate-hz", "0",
]
_sim_kwargs, (_bridge_pos, _) = _capture_main_sim_and_bridge(
    _COMMON_ARGV + [
        "--tau-limits", "1:45,2:11",
        "--mit-torque-limits", "1:300,2:300",
    ])
# 解码量程逐台来自型号库 mit_torque_limit_nm。
assert _sim_kwargs["mit_torque_limits"] == {1: 300.0, 2: 300.0}, \
    _sim_kwargs["mit_torque_limits"]
# 物理饱和仍是 peak_torque_nm（GazeboBridge 第 6 个位置参数）。
assert _bridge_pos[5] == {1: 45.0, 2: 11.0}, _bridge_pos[5]

# 未传 mit-torque-limits 时使用空表，由解码器默认 300 兜底，不复用物理峰值。
_sim_kwargs, _ = _capture_main_sim_and_bridge(
    _COMMON_ARGV + ["--tau-limits", "1:45,2:11"])
assert _sim_kwargs["mit_torque_limits"] == {}, _sim_kwargs["mit_torque_limits"]

# SocketCAN 打包须可往返，解析拒绝扩展帧；设备只处理标准帧。
sys.modules.setdefault("fdcan_motor_sim", MODULE)
PROBE_PATH = SCRIPT_PATH.parent / "cerb_sdo_probe.py"
PROBE_SPEC = spec_from_file_location("cerb_sdo_probe", PROBE_PATH)
assert PROBE_SPEC is not None and PROBE_SPEC.loader is not None
PROBE = module_from_spec(PROBE_SPEC)
PROBE_SPEC.loader.exec_module(PROBE)

_req = bytes([0x40, 0x08, 0x10, 0x00, 0, 0, 0, 0])
_frame = PROBE.pack_socketcan_fd(0x601, _req)
assert len(_frame) == 72, "canfd_frame 是 72 字节定长（8B 头 + 64B 数据）"
assert PROBE.parse_socketcan_fd(_frame) == (0x601, _req)
_e = PROBE.parse_socketcan_fd(PROBE.pack_socketcan_fd(0x80000000 | 0x601, _req))
assert _e is None, "扩展帧必须被解析层拒绝"

print("[test_fdcan_motor_sim] OK")

# 用假传输验证身份分段上传：0x41 发起，0x60/0x70 交替取段，末段按总长截断。
class _FakeProbe(PROBE.SlcanSdoProbe):
    def __init__(self, identity):
        self.node = 1
        self.identity = identity
        self.offset = 0
        self.replies = []

    def send_request(self, req):
        if req[0] == 0x40:                       # 上传发起
            size = len(self.identity)
            self.offset = 0
            self.replies.append(bytes([0x41, req[1], req[2], req[3],
                                       size & 0xFF, (size >> 8) & 0xFF, 0, 0]))
            return
        chunk = self.identity[self.offset:self.offset + 7]
        self.offset += len(chunk)
        last = self.offset >= len(self.identity)
        unused = (7 - len(chunk)) if last else 0
        head = (((req[0] >> 4) & 1) << 4) | (unused << 1) | (1 if last else 0)
        self.replies.append(bytes([head]) + chunk.ljust(7, b"\x00"))

    def drain(self, timeout):
        out, self.replies = self.replies, []
        return [(0x581, r) for r in out]


for _identity in (b"R48LITE_V2", b"R58LITE_V2", b"AB"):   # 含不足一段的短串
    _fp = _FakeProbe(_identity)
    _status, _size, _body, _tries = _fp.read(0x1008, 0x00)
    assert _status == "upload", (_status, _identity)
    assert _size == len(_identity)
    assert _body == _identity, (_body, _identity)

print("[test_fdcan_motor_sim] OK")
