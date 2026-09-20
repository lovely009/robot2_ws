#!/usr/bin/env python3
"""Regression checks for FDCAN simulator setup and feedback scheduling."""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

SCRIPT_PATH = Path(__file__).parents[3] / "micro_ros_agent" / "tools" / "fdcan_motor_sim.py"
SPEC = spec_from_file_location("fdcan_motor_sim", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)

assert MODULE.DEFAULT_DATA_BITRATE_INDEX == 5
assert MODULE.slcan_config_commands(5) == (
    b"C\r", b"S8\r", b"Y5\r", b"O\r",
)
assert MODULE.slcan_config_commands(7) == (
    b"C\r", b"S8\r", b"Y7\r", b"O\r",
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
sim = MODULE.SlcanSim.__new__(MODULE.SlcanSim)
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


# 上电初值必须与**真机实测**一致（进程重启 = 掉电复现）：保护开关初值 1（关），
# 方向初值 0（CW）。固件靠上电归一把它们写成定态——模拟器初值给错，台架就永远
# 复现不出"上电后保护是关的"这个前提。
_cerb = MODULE.MotorModel(1, "cerb")
assert _cerb.sdo_params[(0x2650, 0x00)] == 1, "保护上电初值应为 1（关）"
assert _cerb.sdo_params[(0x2541, 0x00)] == 0, "方向上电初值应为 0（CW）"
