#!/usr/bin/env python3
"""Regression checks for FDCAN simulator setup and feedback scheduling."""

import json
import os
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import socket
import tempfile

SCRIPT_PATH = Path(__file__).parents[3] / "micro_ros_agent" / "tools" / "fdcan_motor_sim.py"
SPEC = spec_from_file_location("fdcan_motor_sim", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)

assert MODULE.DEFAULT_DATA_BITRATE_INDEX == 5
# 2026-10-12 现场实测：这只 CANable（V=16e7497-dirty）的 5 Mbps 索引是 Y5，
# 不是历史注释里的 Y7——Y7 会让带上数据载荷的 FD 帧数据相位 ACK error，
# 9 台全部 OFFLINE、fault=5。判据见 fdcan_motor_sim.py 的 SLCAN_VERSION_DATA_BITRATE。
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

# 身份读取应答（SDO 0x1008 分段上传）：字节格式必须与固件 cerb_adapter 对向，
# 否则台架上固件永远读不到身份串、全部电机停在未绑定。
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

# 非 CERB 节点（无此模型/是 vaxor）不回 SDO。
sent_before = len(sim.sent)
sim.handle_sdo(9, bytes([0x40, 0x08, 0x10, 0x00, 0, 0, 0, 0]))
assert len(sim.sent) == sent_before

# 12B 周期反馈帧：温度必须落在 data[8..9]（大端 int16，0.1 °C）——固件
# cerb_adapter 从这里取 MotorStatusArray.temperature，模拟器不填则页面恒 "--"。
frame = MODULE.MotorModel(4, "cerb").cerb_frame()
assert len(frame) == 12
assert frame[8:10] == round(38.0 * 10).to_bytes(2, "big")  # 30 + 2×4 = 38.0 °C
assert frame[11] & 0x20 == 0  # 无故障

# R58 身份分配：--cerb-r58 指定的节点自报 R58LITE_V2。
sim.models[2] = MODULE.MotorModel(2, "cerb")
sim.models[2].identity = b"R58LITE_V2"
sim.handle_sdo(2, bytes([0x40, 0x08, 0x10, 0x00, 0, 0, 0, 0]))
assert sim.sent[-1][1] == bytes([0x41, 0x08, 0x10, 0x00, 10, 0, 0, 0])

# VAXOR MIT 命令的字段布局必须与固件相同：位置从字节 0 开始，随后是
# 速度、kp、kd、Iq。这个回归用例能抓住“误从 data[2:6] 取位置且丢失增益”
# 的错误；解码结果同时验证电机轴量到关节量的 50:1 换算。
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

# MIT 力矩量化上限**逐型号**（规格书：P/V/T Max 必须与电机参数页一致）。固件按 Profile
# 的 `mit_torque_limit_nm` 编码，解码端必须跟着对应——写死单一常数会让另一个型号的 τ_ff
# 被整体缩放（R48 真值 11 用 45 解 = +4.09×，R58 真值 45 用 11 解 = +0.24×）。
# 两个型号都断言，任一侧写死都会红。
def _mit_tau_frame(tau_nm: float, limit: float) -> bytes:
    code = round((tau_nm + limit) / (2.0 * limit) * 4095)
    frame = bytearray(12)
    frame[7] = (code >> 8) & 0x0F
    frame[8] = code & 0xFF
    return bytes(frame)


for _limit in (11.0, 45.0):
    _decoded = MODULE.cerb_decode_command(_mit_tau_frame(5.0, _limit), _limit)
    assert abs(_decoded["tau_nm"] - 5.0) < 0.02, (_limit, _decoded["tau_nm"])


# 上电初值必须与**真机实测**一致（进程重启 = 掉电复现）：保护开关初值 1（关），
# 方向初值 0（CW）。固件靠上电归一把它们写成定态——模拟器初值给错，台架就永远
# 复现不出"上电后保护是关的"这个前提。
_cerb = MODULE.MotorModel(1, "cerb")
assert _cerb.sdo_params[(0x2650, 0x00)] == 1, "保护上电初值应为 1（关）"
assert _cerb.sdo_params[(0x2541, 0x00)] == 0, "方向上电初值应为 0（CW）"


# ── --joint-map 的重号是**静默**的（2026-10-13 固化）────────────────────────
# 与固件侧"重号进 CONFIG_CONFLICT"完全不同：模拟器这一侧 dict 赋值直接覆盖，
# 不报错、不告警。接线把两个关节接到同一个 motor_id 时，表现是"某台电机不跟"，
# 而不是任何错误码——台架上极易误判成固件或动力学问题。要让行为变响（告警或
# 硬失败）之前，先确认不会打断既有台架命令行：simulation.launch.py 是按
# simulation_topology 的电机顺序拼这一串的。
_jm = MODULE.parse_joint_map("1:joint_link1,2:joint_link2")
assert _jm == {1: "joint_link1", 2: "joint_link2"}
assert MODULE.parse_joint_map("1:joint_a,1:joint_b") == {1: "joint_b"}  # 后者覆盖
assert MODULE.parse_joint_map("1:a,,2:b,") == {1: "a", 2: "b"}  # 空项跳过
assert MODULE.parse_joint_map(" 1 : a ") == {1: "a"}  # 两侧空白容忍


# ── 反馈窗口的墙钟兜底（2026-10-13 修复的回归测试）────────────────────────
# 窗口原本**只**由 SYNC 计数驱动，周期 = interval × **实际** SYNC 周期。SYNC 是
# 固件侧事件，实测节拍可远慢于 --period-ms 假定值（本台架 2026-10-13 实测仅
# 12~24Hz），窗口于是被拉长到 3×SYNC ≈ 120ms，越过固件 100ms 离线门
# （vaxor_adapter.c / cerb_adapter.c 的 offline_timeout_ms）→ 全台架
# OFFLINE↔重连抖动 → 重连清空 profile → 速度/力矩大面积 NaN。
# 兜底让 --fb-period-ms 成为周期的**上界**，与 SYNC 快慢解耦。
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
# SYNC 长期不来（模拟实测的 12~24Hz）：仍严格按 fb_period 出窗口。
# 推进量用 1/16 秒（二进制精确）而非 60ms：0.06 不是精确值，
# "刚好到点"会被浮点算成差 1e-14 → 误判成"还没到点"（本用例第一版即踩此坑，
# 与产品代码无关，纯断言写法问题）。
_we = _bare_sim()
_we.fb_period = 0.0625
for _i in range(5):
    assert _we.maybe_fallback_window(_we.last_window_at + 0.0625)
assert _we.window_count == 5, _we.window_count
assert _we.last_window_at == 100.0 + 0.0625 * 5

# 周期参数生效：--fb-period-ms 20 把阈值降到 20ms。用"同一个 21ms 间隔在两种
# 周期下结论相反"来断言，既绕开浮点（100.02-100.0 会算成 0.0199999999999996），
# 也比单点比较更能说明阈值真的跟着参数走。
_w60_ref = _bare_sim()          # 默认 60ms
_w20 = _bare_sim(fb_period_ms=20)
assert not _w20.maybe_fallback_window(100.0 + 0.019)
assert not _w60_ref.maybe_fallback_window(100.0 + 0.021)  # 60ms 周期：还没到
assert _w20.maybe_fallback_window(100.0 + 0.021)          # 20ms 周期：到点了
assert _w20.window_count == 1
assert _w60_ref.window_count == 0 and _w60_ref.next_send == {}


# ── 反馈编码越界不得打死进程（2026-10-13 台架实测的崩溃）──────────────────
# 武装后关节瞬时摆过 ±180°，cerb_frame 的 pos_deg*182 溢出 int16 → struct.error
# 打死整个模拟器进程，launch 随即关掉全部（含 MuJoCo 桥，exit -11），台架全下线、
# 固件侧 18 台全 OFFLINE/fault=5。vaxor 的三个编码函数一直有 clamp，CERB 这条路
# 漏了——而它的失效方式是**崩进程**，不是丢一帧。
_cb = MODULE.MotorModel(1, "cerb")
_cb.pos_deg = 400.0     # 远超 ±180°（int16 只能表示 ±180.04°）
_cb.rpm = 10 ** 9
_cb.iq_ma = -(10 ** 9)
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


# ── 运行期控制（--control-socket 的纯逻辑层，2026-10-14 加）────────────────
# web 上位机仿真页的功能面：add 往总线上加电机（id 在 joint_map 里自动挂关节，
# 否则是只走线协议的裸假节点）；remove 从模拟集合移除（再 add 回来是全新电机）；
# link 断/连。linked=False 时**完全静默**——不发反馈、不回 SDO、忽略下行帧，
# 对固件等价于设备掉电（心跳超时判 OFFLINE）；恢复时清命令状态像重新上电
# （固件走 RECONNECTED 重识别）。
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
_multi_payload = bytes(56) + bytes([1, 0, 0, 0, 0, 0]) + bytes(2)
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
