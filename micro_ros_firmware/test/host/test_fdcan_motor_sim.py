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
