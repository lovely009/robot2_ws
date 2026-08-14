# 电机调试 TUI 上位机 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 `src/motor_drive/scripts/` 新增一个纯 Python、无 ROS 的交互式电机调试上位机（curses TUI + 共享编解码模块），支持 RP 与 vaxor 电机的状态查看、调增益、点动、使能/失能与设零位。

**Architecture:** 抽取 `verify_2530_serial.py` 里现成的 vaxor/slcan 编解码到共享模块 `motor_probe.py`，照 `protocol.cpp` 新增 RP 编解码与统一分发；新增 `motor_debug_tui.py` 读 `motor_driver.yaml`/`motor.yaml` 直连 serial_slcan，用 curses 显示实时状态并响应键盘命令。零 C++ 改动。

**Tech Stack:** Python 3.10（stdlib `curses`、`pathlib`、`argparse`）、`pyserial` 3.5、`pyyaml`、pytest（ament_cmake_pytest）。

## Global Constraints

- 只支持 `serial_slcan` 传输；遇到 `socketcan` 必须打印明确错误并退出，不静默降级。
- 不新增 pip 依赖（`pyserial`/`pyyaml` 已存在）；只用 stdlib `curses`。
- 不改任何 C++ 文件（`motor_drive` 的 `protocol.cpp` 等保持不动）。
- Python 命名 snake_case，函数名 `test_<behavior>()`；注释用中文写意图（数据流/单位/安全边界），协议名/标识符保留英文。
- 安全口径：启动全失能 + 零增益；使能前锁存当前位置为目标；`e`/`z` 二次确认；退出/异常路径全部失能 + 关串口 + `curses.endwin()`。软件无软限位，依赖操作者观察 + 硬件限位/急停。
- 构建/测试命令：`source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select motor_drive && colcon test --packages-select motor_drive && colcon test-result --verbose`。

---

## File Structure

| 文件 | 动作 | 职责 |
|---|---|---|
| `src/motor_drive/scripts/motor_probe.py` | 新建 | 共享编解码/收发模块：slcan 帧、vaxor、RP、统一分发。纯函数，无 curses/硬件。 |
| `src/motor_drive/scripts/motor_debug_tui.py` | 新建 | curses 交互 TUI：配置解析、总线打开、控制循环、UI、按键、安全。 |
| `src/motor_drive/scripts/verify_2530_serial.py` | 修改 | 删除内联编解码，改 `from motor_probe import ...`，行为不变。 |
| `src/motor_drive/test/test_motor_probe.py` | 新建 | `motor_probe` 纯函数 pytest。 |
| `src/motor_drive/test/test_motor_debug.py` | 新建 | `motor_debug_tui` 的 `load_config`/`build_command_frames` 纯逻辑 pytest。 |
| `src/motor_drive/package.xml` | 修改 | 加 `ament_cmake_pytest` test_depend。 |
| `src/motor_drive/CMakeLists.txt` | 修改 | 注册两个 pytest。 |
| `src/motor_drive/README.md` | 修改 | 加调试工具使用说明。 |

---

### Task 1: pytest 基础设施 + `motor_probe.py`（slcan + vaxor 迁入）

**Files:**
- Create: `src/motor_drive/scripts/motor_probe.py`
- Create: `src/motor_drive/test/test_motor_probe.py`
- Modify: `src/motor_drive/package.xml`
- Modify: `src/motor_drive/CMakeLists.txt`

**Interfaces:**
- Produces: 模块 `motor_probe`，导出 `fd_len_code`、`encode_frame`、`decode_frame`、
  `pos_to_code`/`code_to_pos`、`vel_to_code`/`code_to_vel`、
  `current_to_code`/`code_to_current`、`le16`/`le32`、`pack_wrist_command`、
  `pack_wrist_control`、`decode_vaxor_feedback`、`wrist_command_id`/`wrist_control_id`/
  `wrist_reply_id`/`wrist_status_id`/`wrist_calib_id`/`wrist_calib_result_id`、
  常量 `WR_TICKS_PER_REV`、`WR_CURRENT_SCALE_A`、`WR_KP_SCALE`、`WR_KD_SCALE`。

- [ ] **Step 1: 写失败测试** — 创建 `test/test_motor_probe.py`（只含本任务范围：slcan + vaxor）

```python
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
    state = mp.decode_vaxor_feedback(0x702, _status_frame(), node=1)
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
    reply = bytes.fromhex("0505000000800080008000")
    state = mp.decode_vaxor_feedback(0x701, reply, node=1)
    assert state is not None
    assert "voltage_v" not in state  # reply 帧无状态字段
    assert state["current_a"] == 0.0


def test_decode_vaxor_wrong_id_returns_none():
    assert mp.decode_vaxor_feedback(0x706, _status_frame(), node=1) is None
```

- [ ] **Step 2: 运行测试确认失败**

Run: `cd /home/love009/robot_ws && python3 -m pytest src/motor_drive/test/test_motor_probe.py -q`
Expected: 全部 `ModuleNotFoundError: No module named 'motor_probe'`

- [ ] **Step 3: 实现 `motor_probe.py`（slcan + vaxor 部分）**

```python
"""电机调试共享模块：slcan 帧收发与 RP / vaxor（25/30mm）编解码。

纯函数，无 curses、无硬件依赖，可直接单测。vaxor 部分从
verify_2530_serial.py 迁入并保持数值一致；RP 部分照 motor_drive/protocol.cpp
复制（接受与 C++ 的漂移）。
"""

from __future__ import annotations

# ---------------------------------------------------------------------------
# slcan 帧收发
# ---------------------------------------------------------------------------

_FD_LEN_CODE = {
    0: "0", 1: "1", 2: "2", 3: "3", 4: "4", 5: "5", 6: "6", 7: "7", 8: "8",
    12: "9", 16: "A", 20: "B", 24: "C", 32: "D", 48: "E", 64: "F",
}
_DECODE_LEN_CODE = {v: k for k, v in _FD_LEN_CODE.items()}


def fd_len_code(nbytes: int) -> str:
    if nbytes not in _FD_LEN_CODE:
        raise ValueError(f"unsupported slcan FD length {nbytes}")
    return _FD_LEN_CODE[nbytes]


def encode_frame(identifier: int, data: bytes, is_fd: bool, brs: bool) -> str:
    """按 CANable2 slcan 方言组帧（标准 ID 11 bit）。"""
    if identifier > 0x7FF:
        raise ValueError(f"standard ID 0x{identifier:X} out of range")
    if is_fd:
        cmd = "b" if brs else "d"
    else:
        cmd = "t"
    return f"{cmd}{identifier:03X}{fd_len_code(len(data))}{data.hex().upper()}\r"


def decode_frame(line: str):
    """解码 CANable2 上报帧，返回 (id, data, is_fd) 或 None。"""
    line = line.strip()
    if len(line) < 5:
        return None
    cmd, rest = line[0], line[1:]
    if cmd in "tT":
        is_fd = False
        id_len = 3 if cmd == "t" else 8
    elif cmd in "dDbB":
        is_fd = True
        id_len = 3 if cmd in "db" else 8
    else:
        return None
    if len(rest) < id_len + 1:
        return None
    try:
        identifier = int(rest[:id_len], 16)
        len_code = rest[id_len]
        payload = rest[id_len + 1:]
    except ValueError:
        return None
    nbytes = _DECODE_LEN_CODE.get(len_code)
    if nbytes is None or len(payload) != nbytes * 2:
        return None
    try:
        data = bytes.fromhex(payload)
    except ValueError:
        return None
    return identifier, data, is_fd


# ---------------------------------------------------------------------------
# vaxor（25/30 mm）编解码
# ---------------------------------------------------------------------------

WR_TICKS_PER_REV = 2_097_152
WR_CURRENT_SCALE_A = 11.0 / 4096.0
WR_KP_SCALE = 0.00001
WR_KD_SCALE = 0.01
_WR_PI = 3.141592653589793


def wrist_command_id(node: int) -> int:
    return 0x501 + 4 * node


def wrist_control_id(node: int) -> int:
    return 0x502 + 4 * node


def wrist_reply_id(node: int) -> int:
    return 0x701 + 4 * node


def wrist_status_id(node: int) -> int:
    return 0x702 + 4 * node


def wrist_calib_id(node: int) -> int:
    return 0x503 + 4 * node


def wrist_calib_result_id(node: int) -> int:
    return 0x703 + 4 * node


def pos_to_code(rad: float) -> int:
    return round(rad / (2.0 * _WR_PI) * WR_TICKS_PER_REV) + 0x80000000


def code_to_pos(code: int) -> float:
    return (code - 0x80000000) / WR_TICKS_PER_REV * 2.0 * _WR_PI


def vel_to_code(rad_s: float) -> int:
    rpm10 = round(rad_s * 60.0 / (2.0 * _WR_PI) * 10.0)
    return max(-30000, min(30000, rpm10)) + 0x8000


def code_to_vel(code: int) -> float:
    return (code - 0x8000) / 10.0 * 2.0 * _WR_PI / 60.0


def current_to_code(amp: float) -> int:
    return max(-32768, min(32767, round(amp / WR_CURRENT_SCALE_A))) + 0x8000


def code_to_current(code: int) -> float:
    return (code - 0x8000) * WR_CURRENT_SCALE_A


def le16(data: bytes, off: int) -> int:
    return int.from_bytes(data[off:off + 2], "little")


def le32(data: bytes, off: int) -> int:
    return int.from_bytes(data[off:off + 4], "little")


def pack_wrist_command(position_rad: float, velocity_rad_s: float,
                       kp: float, kd: float, current_a: float) -> bytes:
    """与 protocol.cpp::pack_wrist_command 相同的 12 字节 MIT 下行。"""
    return (
        pos_to_code(position_rad).to_bytes(4, "little")
        + vel_to_code(velocity_rad_s).to_bytes(2, "little")
        + round(kp / WR_KP_SCALE).to_bytes(2, "little")
        + round(kd / WR_KD_SCALE).to_bytes(2, "little")
        + current_to_code(current_a).to_bytes(2, "little")
    )


def pack_wrist_control(run: bool, set_zero: bool = False, mode: int = 0) -> bytes:
    return bytes([int(run), int(set_zero), mode, 0, 0, 0, 0, 0])


def decode_vaxor_feedback(frame_id: int, data: bytes, node: int):
    """解码 vaxor 回复/状态帧，返回统一状态 dict 或 None。"""
    if frame_id == wrist_reply_id(node) and len(data) >= 12:
        return {
            "position": code_to_pos(le32(data, 2)),
            "velocity": code_to_vel(le16(data, 6)),
            "current_a": code_to_current(le16(data, 8)),
        }
    if frame_id == wrist_status_id(node) and len(data) >= 20:
        return {
            "position": code_to_pos(le32(data, 2)),
            "velocity": code_to_vel(le16(data, 6)),
            "current_a": code_to_current(le16(data, 8)),
            "voltage_v": le16(data, 10) / 10.0,
            "fault": le32(data, 12),
            "motor_state": data[16],
            "calibration_state": data[17],
            "mode": data[18],
            "motor_type": data[19],
        }
    return None
```

- [ ] **Step 4: 加 pytest 基础设施** — 修改 `package.xml` 与 `CMakeLists.txt`

`package.xml`：在 `<test_depend>ament_cmake_gtest</test_depend>` 后加一行：

```xml
  <test_depend>ament_cmake_pytest</test_depend>
```

`CMakeLists.txt`：在 `if(BUILD_TESTING)` 块内、`find_package(ament_cmake_gtest REQUIRED)` 后加：

```cmake
  find_package(ament_cmake_pytest REQUIRED)
```

并在 `ament_add_gtest(test_slcan ...)` 及其 `target_link_libraries` 之后加：

```cmake
  ament_add_pytest_test(test_motor_probe test/test_motor_probe.py)
```

- [ ] **Step 5: 运行测试确认通过**

Run: `cd /home/love009/robot_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select motor_drive && colcon test --packages-select motor_drive && colcon test-result --verbose`
Expected: `test_motor_probe` 通过，`test_motor_probe.py` 内全部用例 PASS。

- [ ] **Step 6: 提交**

```bash
git add src/motor_drive/scripts/motor_probe.py src/motor_drive/test/test_motor_probe.py \
        src/motor_drive/package.xml src/motor_drive/CMakeLists.txt
git commit -m "test(motor_drive): 新增 motor_probe slcan/vaxor 编解码与 pytest"
```

---

### Task 2: `motor_probe.py` 增加 RP 编解码

**Files:**
- Modify: `src/motor_drive/scripts/motor_probe.py`（追加 RP 段）
- Modify: `src/motor_drive/test/test_motor_probe.py`（追加 RP 测试）

**Interfaces:**
- Produces: `RP_ENABLE_ID=0x00`、`RP_COMMAND_ID=0x80`、`RP_BROADCAST_SLOTS=8`、
  `pack_rp_slot(command: dict) -> bytes(8)`、`pack_rp_command(slot_commands: dict) -> bytes(64)`、
  `pack_rp_enable(enabled: bool) -> bytes(8)`、`unpack_rp_state(data: bytes) -> dict`。
  command dict 键为 `position`/`velocity`/`kp`/`kd`/`feedforward`；`slot_commands`
  键为电机 `id`（1-8）。

- [ ] **Step 1: 写失败测试** — 追加到 `test_motor_probe.py`

```python
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
```

- [ ] **Step 2: 运行测试确认失败**

Run: `cd /home/love009/robot_ws && python3 -m pytest src/motor_drive/test/test_motor_probe.py -q -k rp`
Expected: `AttributeError: module 'motor_probe' has no attribute 'pack_rp_slot'`

- [ ] **Step 3: 追加 RP 段到 `motor_probe.py`**（文件末尾）

```python
# ---------------------------------------------------------------------------
# RP 编解码（照 motor_drive/protocol.cpp）
# ---------------------------------------------------------------------------

RP_ENABLE_ID = 0x00
RP_COMMAND_ID = 0x80
RP_BROADCAST_SLOTS = 8

_RP_POS_RANGE = (-10.0, 10.0, 16)
_RP_VEL_RANGE = (-60.0, 60.0, 12)
_RP_KP_RANGE = (0.0, 500.0, 12)
_RP_KD_RANGE = (0.0, 10.0, 12)
_RP_TORQUE_RANGE = (-60.0, 60.0, 12)

_RP_ZERO_COMMAND = {"position": 0.0, "velocity": 0.0, "kp": 0.0, "kd": 0.0,
                    "feedforward": 0.0}


def _float_to_uint(value: float, low: float, high: float, bits: int) -> int:
    # 与 protocol.cpp::float_to_uint 一致：先限幅，再线性映射并截断（非四舍五入）。
    value = max(low, min(high, value))
    maximum = (1 << bits) - 1
    return int((value - low) * maximum / (high - low))


def _uint_to_float(value: int, low: float, high: float, bits: int) -> float:
    maximum = (1 << bits) - 1
    return value * (high - low) / maximum + low


def pack_rp_slot(command: dict) -> bytes:
    """RP MIT 8 Byte 槽：P(16)+V(12)+Kp(12)+Kd(12)+Tau(12)。"""
    position = _float_to_uint(command.get("position", 0.0), *_RP_POS_RANGE)
    velocity = _float_to_uint(command.get("velocity", 0.0), *_RP_VEL_RANGE)
    kp = _float_to_uint(command.get("kp", 0.0), *_RP_KP_RANGE)
    kd = _float_to_uint(command.get("kd", 0.0), *_RP_KD_RANGE)
    torque = _float_to_uint(command.get("feedforward", 0.0), *_RP_TORQUE_RANGE)
    return bytes([
        position >> 8, position & 0xFF,
        velocity >> 4, ((velocity & 0x0F) << 4) | (kp >> 8), kp & 0xFF,
        kd >> 4, ((kd & 0x0F) << 4) | (torque >> 8), torque & 0xFF,
    ])


def pack_rp_command(slot_commands: dict) -> bytes:
    """RP 8 槽 64 字节广播：slot_commands 键为电机 id（1-8），未配置槽填中性槽。"""
    payload = bytearray(RP_BROADCAST_SLOTS * 8)
    for slot in range(RP_BROADCAST_SLOTS):
        command = slot_commands.get(slot + 1, _RP_ZERO_COMMAND)
        payload[slot * 8:(slot + 1) * 8] = pack_rp_slot(command)
    return bytes(payload)


def pack_rp_enable(enabled: bool) -> bytes:
    return bytes([int(enabled), 0, 0, 0, 0, 0, 0, 0])


def unpack_rp_state(data: bytes) -> dict:
    """RP 12 字节状态帧 -> 位姿/速度/扭矩 + MOS 温度/status/电压/线圈温度。"""
    if len(data) < 12:
        raise ValueError("RP state must be 12 bytes")
    return {
        "position": _uint_to_float((data[0] << 8) | data[1], *_RP_POS_RANGE),
        "velocity": _uint_to_float((data[2] << 4) | (data[3] >> 4), *_RP_VEL_RANGE),
        "effort": _uint_to_float(((data[3] & 0x0F) << 8) | data[4], *_RP_TORQUE_RANGE),
        "mos_temperature_c": data[5] - 40.0,
        "status": (data[6] << 8) | data[7],
        "voltage_v": data[8],
        "coil_temperature_c": data[9] - 40.0,
    }
```

- [ ] **Step 4: 运行测试确认通过**

Run: `cd /home/love009/robot_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select motor_drive && colcon test --packages-select motor_drive && colcon test-result --verbose`
Expected: `test_motor_probe` 全部 PASS。

- [ ] **Step 5: 提交**

```bash
git add src/motor_drive/scripts/motor_probe.py src/motor_drive/test/test_motor_probe.py
git commit -m "feat(motor_drive): motor_probe 新增 RP 编解码"
```

---

### Task 3: `motor_probe.py` 增加统一分发 `decode_motor_state`

**Files:**
- Modify: `src/motor_drive/scripts/motor_probe.py`（追加分发函数）
- Modify: `src/motor_drive/test/test_motor_probe.py`（追加分发测试）

**Interfaces:**
- Produces: `decode_motor_state(frame_id: int, data: bytes, motor: dict) -> dict | None`。
  按 `motor["protocol"]`（`rp`/`vaxor`）分发；vaxor 用
  `motor["parameters"]` 里的 `torque_constant_nm_per_amp`/`gear_ratio`/`gear_efficiency`
  把电流 A 换算为关节侧 Nm 存入 `effort`。RP 校验 `frame_id == motor["id"]` 且 12 字节。

- [ ] **Step 1: 写失败测试** — 追加到 `test_motor_probe.py`

```python
# ---- 统一分发 decode_motor_state ----
_RP_STATE = bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x50,
                   0x12, 0x34, 0x18, 0x50, 0x00, 0x00])


def test_decode_motor_state_rp():
    motor = {"protocol": "rp", "id": 1}
    state = mp.decode_motor_state(1, _RP_STATE, motor)
    assert state is not None
    assert state["effort"] == -60.0
    assert state["status"] == 0x1234


def test_decode_motor_state_rp_wrong_id_returns_none():
    motor = {"protocol": "rp", "id": 2}
    assert mp.decode_motor_state(1, _RP_STATE, motor) is None


def test_decode_motor_state_vaxor_computes_effort():
    # 用 current_to_code(1.0) 构造 reply 帧，验证 effort = current_a * Kt * gear * eff
    reply = bytearray(12)
    reply[2:6] = mp.pos_to_code(0.0).to_bytes(4, "little")
    reply[6:8] = mp.vel_to_code(0.0).to_bytes(2, "little")
    reply[8:10] = mp.current_to_code(1.0).to_bytes(2, "little")
    motor = {
        "protocol": "vaxor", "id": 1,
        "parameters": {
            "torque_constant_nm_per_amp": 0.042,
            "gear_ratio": 50.0,
            "gear_efficiency": 0.65,
        },
    }
    state = mp.decode_motor_state(mp.wrist_reply_id(1), bytes(reply), motor)
    assert state is not None
    assert state["current_a"] == pytest.approx(1.0, abs=0.01)
    assert state["effort"] == pytest.approx(
        state["current_a"] * 0.042 * 50.0 * 0.65, rel=1e-9)


def test_decode_motor_state_unknown_protocol_returns_none():
    assert mp.decode_motor_state(1, _RP_STATE, {"protocol": "nope", "id": 1}) is None
```

- [ ] **Step 2: 运行测试确认失败**

Run: `cd /home/love009/robot_ws && python3 -m pytest src/motor_drive/test/test_motor_probe.py -q -k decode_motor_state`
Expected: `AttributeError: module 'motor_probe' has no attribute 'decode_motor_state'`

- [ ] **Step 3: 追加 `decode_motor_state` 到 `motor_probe.py` 末尾**

```python
# ---------------------------------------------------------------------------
# 统一分发
# ---------------------------------------------------------------------------

def decode_motor_state(frame_id: int, data: bytes, motor: dict):
    """按协议类型把一帧解码为统一状态 dict；不匹配返回 None。

    统一状态含 position/velocity/effort；协议私有遥测键（如 current_a、
    voltage_v、mos_temperature_c、status、fault）按需附带。
    """
    protocol = motor["protocol"]
    if protocol == "rp":
        if frame_id != motor["id"] or len(data) != 12:
            return None
        return unpack_rp_state(data)
    if protocol == "vaxor":
        state = decode_vaxor_feedback(frame_id, data, motor["id"])
        if state is None:
            return None
        # 反馈给的是电流 A，统一 effort 对上层表示关节侧 Nm：
        # Kt * 减速比 * 效率（gear_ratio / gear_efficiency 缺省 1）。
        params = motor.get("parameters", {})
        kt = float(params.get("torque_constant_nm_per_amp", 0.0))
        gear = float(params.get("gear_ratio", 1.0))
        efficiency = float(params.get("gear_efficiency", 1.0))
        state["effort"] = state["current_a"] * kt * gear * efficiency
        return state
    return None
```

- [ ] **Step 4: 运行测试确认通过**

Run: `cd /home/love009/robot_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select motor_drive && colcon test --packages-select motor_drive && colcon test-result --verbose`
Expected: `test_motor_probe` 全部 PASS。

- [ ] **Step 5: 提交**

```bash
git add src/motor_drive/scripts/motor_probe.py src/motor_drive/test/test_motor_probe.py
git commit -m "feat(motor_drive): motor_probe 新增统一状态分发 decode_motor_state"
```

---

### Task 4: 重构 `verify_2530_serial.py` 复用 `motor_probe`

**Files:**
- Modify: `src/motor_drive/scripts/verify_2530_serial.py`

**Interfaces:**
- Consumes: Task 1/3 的 `motor_probe` 导出。
- Produces: 行为与重构前完全一致（`--listen`/`--motion-test`/`--calibrate` 输出不变）。

- [ ] **Step 1: 用 `motor_probe` 替换内联编解码**

删除 verify 脚本里第 37–173 行的内联实现（常量 `WR_TICKS_PER_REV`/`WR_CURRENT_SCALE_A`/
`WR_KP_SCALE`/`WR_KD_SCALE`、ID 函数、`pos_to_code`/`code_to_pos`/`vel_to_code`/
`code_to_vel`/`current_to_code`/`code_to_current`/`le16`/`le32`、`fd_len_code`/
`encode_frame`/`decode_frame`、`pack_wrist_command`、`describe_feedback`），在文件顶部
import 后改为：

```python
from motor_probe import (
    decode_frame,
    encode_frame,
    decode_vaxor_feedback,
    pack_wrist_command,
    wrist_calib_id,
    wrist_calib_result_id,
    wrist_command_id,
    wrist_control_id,
    wrist_reply_id,
    wrist_status_id,
)
```

保留 `WR_CALIB_PAYLOAD`、`send()`、`listen()`、`run_calibration()`、`main()` 与
`ARGS` 全局。`describe_feedback` 改为复用 `decode_vaxor_feedback`：

```python
def describe_feedback(frame_id: int, data: bytes, node: int) -> str:
    state = decode_vaxor_feedback(frame_id, data, node)
    if state is None:
        return f"帧 0x{frame_id:03X} len={len(data)} data={data.hex().upper()}"
    base = (f"pos={state['position']:+.4f} rad "
            f"vel={state['velocity']:+.3f} rad/s "
            f"cur={state['current_a']:+.3f} A")
    if "voltage_v" in state:  # 状态帧（20 字节）
        return (f"状态 0x{frame_id:03X} {base} "
                f"voltage={state['voltage_v']:.1f} V fault=0x{state['fault']:08X} "
                f"motor_state={state['motor_state']} calib={state['calibration_state']} "
                f"mode={state['mode']} type={state['motor_type']} [{data.hex().upper()}]")
    return f"回复 0x{frame_id:03X} {base} [{data.hex().upper()}]"
```

`main()` 内对 `wr_calib_id`/`wr_calib_result_id`/`wr_command_id`/`wr_control_id` 的
调用继续沿用 import 的 ID 函数（`wrist_calib_id` 等已由 `motor_probe` 提供）。

- [ ] **Step 2: 语法与导入自检**

Run: `cd /home/love009/robot_ws && python3 -m py_compile src/motor_drive/scripts/verify_2530_serial.py && python3 src/motor_drive/scripts/verify_2530_serial.py --help`
Expected: 编译通过，`--help` 打印原参数说明，无 import 错误。

- [ ] **Step 3: 提交**

```bash
git add src/motor_drive/scripts/verify_2530_serial.py
git commit -m "refactor(motor_drive): verify_2530_serial 复用 motor_probe 编解码"
```

---

### Task 5: `motor_debug_tui.py` — `load_config` 与 `build_command_frames`（纯逻辑）

**Files:**
- Create: `src/motor_drive/scripts/motor_debug_tui.py`
- Create: `src/motor_drive/test/test_motor_debug.py`
- Modify: `src/motor_drive/CMakeLists.txt`

**Interfaces:**
- Produces: `load_config(path: str) -> dict` 返回
  `{"buses": [{"name","transport","tty"}], "motors": [{"name","joint","bus","type","protocol","id","kp","kd","parameters"}], "timing": {...}}`；
  `build_command_frames(bus_motors, states, targets, enabled, gains) -> list[(id, data, is_fd, brs)]`。
  `states` 键为电机名、值为 `decode_motor_state` 的 dict（可能为空 `{}`）；
  `targets` 键为电机名、值为 `float | None`；`enabled` 键为电机名、值为 bool；
  `gains` 键为电机名、值为 `[kp, kd]`。

- [ ] **Step 1: 写失败测试** — 创建 `test/test_motor_debug.py`

```python
"""motor_debug_tui 的 load_config 与 build_command_frames 纯逻辑测试。"""
import sys
from pathlib import Path

import pytest

SCRIPTS = Path(__file__).resolve().parent.parent / "scripts"
sys.path.insert(0, str(SCRIPTS))

from motor_debug_tui import build_command_frames, load_config
from motor_probe import RP_COMMAND_ID, pack_rp_command, wrist_command_id


def _write_config(tmp_path: Path) -> str:
    specs = tmp_path / "motor.yaml"
    specs.write_text(
        "motor_types:\n"
        "  rp-50h:\n"
        "    protocol: rp\n"
        "    parameters:\n"
        "      rated_torque_nm: 10.0\n"
        "  vaxor_2550:\n"
        "    protocol: vaxor\n"
        "    parameters:\n"
        "      torque_constant_nm_per_amp: 0.042\n"
        "      gear_ratio: 50\n",
        encoding="utf-8",
    )
    topo = tmp_path / "motor_driver.yaml"
    topo.write_text(
        "motor_hardware:\n"
        "  motor_specs_file: motor.yaml\n"
        "  timing: {feedback_timeout_s: 0.10, initial_feedback_wait_s: 1.0}\n"
        "  buses:\n"
        "    left_arm: {transport: serial_slcan, tty: /dev/ttyACM0}\n"
        "  motors:\n"
        "    m_rp: {joint: j_rp, bus: left_arm, type: rp-50h, id: 1, command: {kp: 30.0, kd: 1.0}}\n"
        "    m_vx: {joint: j_vx, bus: left_arm, type: vaxor_2550, id: 1, command: {kp: 0.008, kd: 1.0}}\n",
        encoding="utf-8",
    )
    return str(topo)


def test_load_config_maps_type_to_protocol(tmp_path):
    cfg = load_config(_write_config(tmp_path))
    by_name = {m["name"]: m for m in cfg["motors"]}
    assert by_name["m_rp"]["protocol"] == "rp"
    assert by_name["m_vx"]["protocol"] == "vaxor"
    assert by_name["m_vx"]["parameters"]["torque_constant_nm_per_amp"] == 0.042
    assert cfg["buses"][0]["tty"] == "/dev/ttyACM0"


def test_load_config_rejects_socketcan(tmp_path):
    topo = tmp_path / "motor_driver.yaml"
    topo.write_text(
        "motor_hardware:\n"
        "  motor_specs_file: motor.yaml\n"
        "  timing: {feedback_timeout_s: 0.10, initial_feedback_wait_s: 1.0}\n"
        "  buses:\n"
        "    b0: {transport: socketcan, interface: can0}\n"
        "  motors:\n"
        "    m: {joint: j, bus: b0, type: t, id: 1, command: {kp: 1.0, kd: 0.0}}\n",
        encoding="utf-8",
    )
    (tmp_path / "motor.yaml").write_text(
        "motor_types:\n  t: {protocol: rp, parameters: {}}\n", encoding="utf-8")
    with pytest.raises(ValueError):
        load_config(str(topo))


def test_build_command_frames_rp_disabled_zero_gain():
    bus_motors = [{"name": "m_rp", "protocol": "rp", "id": 1}]
    frames = build_command_frames(
        bus_motors,
        states={"m_rp": {"position": 0.5}},
        targets={"m_rp": 0.5},
        enabled={"m_rp": False},
        gains={"m_rp": (30.0, 1.0)},
    )
    assert len(frames) == 1
    fid, data, is_fd, brs = frames[0]
    assert (fid, is_fd, brs) == (RP_COMMAND_ID, True, True)
    assert data == pack_rp_command({1: {"position": 0.5, "velocity": 0.0,
                                        "kp": 0.0, "kd": 0.0, "feedforward": 0.0}})


def test_build_command_frames_vaxor_enabled():
    bus_motors = [{"name": "m_vx", "protocol": "vaxor", "id": 1}]
    frames = build_command_frames(
        bus_motors,
        states={"m_vx": {}},
        targets={"m_vx": None},
        enabled={"m_vx": True},
        gains={"m_vx": (0.008, 1.0)},
    )
    assert len(frames) == 1
    fid, data, is_fd, brs = frames[0]
    assert (fid, is_fd, brs) == (wrist_command_id(1), True, True)
    assert len(data) == 12
```

- [ ] **Step 2: 运行测试确认失败**

Run: `cd /home/love009/robot_ws && python3 -m pytest src/motor_drive/test/test_motor_debug.py -q`
Expected: `ModuleNotFoundError: No module named 'motor_debug_tui'`

- [ ] **Step 3: 实现 `motor_debug_tui.py` 的纯逻辑部分**

```python
"""交互式电机调试上位机（curses，无 ROS，只 serial_slcan）。

读 motor_driver.yaml + motor.yaml 直连 CANable2 串口，实时显示每台电机状态，
支持调增益、点动、使能/失能与 vaxor 设零位。启动即全失能 + 零增益。
"""

from __future__ import annotations

import argparse
from pathlib import Path

import yaml

from motor_probe import (
    pack_rp_command,
    pack_wrist_command,
    RP_COMMAND_ID,
    wrist_command_id,
)

BAUD = 115200
DEFAULT_CONFIG = "src/motor_hardware_interface/config/motor_driver.yaml"


def load_config(path: str) -> dict:
    """读拓扑 + 型号库，返回 buses/motors/timing；只允许 serial_slcan。"""
    topo_path = Path(path)
    hw = yaml.safe_load(topo_path.read_text(encoding="utf-8"))["motor_hardware"]

    specs_path = topo_path.parent / hw["motor_specs_file"]
    motor_types = yaml.safe_load(specs_path.read_text(encoding="utf-8"))["motor_types"]

    buses = []
    for name, node in hw["buses"].items():
        transport = node.get("transport", "socketcan")
        if transport != "serial_slcan":
            raise ValueError(
                f"总线 '{name}' 使用 transport={transport}，本工具只支持 serial_slcan；"
                f"请改用 serial_slcan 或直接用 verify_2530_serial.py")
        buses.append({"name": name, "transport": transport, "tty": node["tty"]})

    motors = []
    for name, node in hw["motors"].items():
        model = node["type"]
        type_node = motor_types.get(model)
        if type_node is None:
            raise ValueError(f"电机 '{name}' 引用未知型号 '{model}'")
        protocol = type_node.get("protocol", model)
        parameters = dict(type_node.get("parameters", {}))
        parameters.update(node.get("parameters", {}))
        motors.append({
            "name": name,
            "joint": node["joint"],
            "bus": node["bus"],
            "type": model,
            "protocol": protocol,
            "id": node["id"],
            "kp": node["command"]["kp"],
            "kd": node["command"]["kd"],
            "parameters": parameters,
        })

    timing = hw.get("timing", {"feedback_timeout_s": 0.1, "initial_feedback_wait_s": 1.0})
    return {"buses": buses, "motors": motors, "timing": timing}


def build_command_frames(bus_motors, states, targets, enabled, gains):
    """给定一条总线的电机，构造本周期要发送的帧列表 [(id, data, is_fd, brs)]。

    未使能电机发零增益（kp/kd=0），避免产生保持力矩；target 未锁存时用当前反馈位置。
    """
    frames = []
    rp_motors = [m for m in bus_motors if m["protocol"] == "rp"]
    vaxor_motors = [m for m in bus_motors if m["protocol"] == "vaxor"]

    if rp_motors:
        slots = {}
        for m in rp_motors:
            state = states.get(m["name"], {})
            pos = targets.get(m["name"])
            if pos is None:
                pos = state.get("position", 0.0)
            kp, kd = gains.get(m["name"], (0.0, 0.0))
            if not enabled.get(m["name"], False):
                kp, kd = 0.0, 0.0
            slots[m["id"]] = {"position": pos, "velocity": 0.0,
                              "kp": kp, "kd": kd, "feedforward": 0.0}
        frames.append((RP_COMMAND_ID, pack_rp_command(slots), True, True))

    for m in vaxor_motors:
        state = states.get(m["name"], {})
        pos = targets.get(m["name"])
        if pos is None:
            pos = state.get("position", 0.0)
        kp, kd = gains.get(m["name"], (0.0, 0.0))
        if not enabled.get(m["name"], False):
            kp, kd = 0.0, 0.0
        frames.append((wrist_command_id(m["id"]),
                       pack_wrist_command(pos, 0.0, kp, kd, 0.0), True, True))
    return frames
```

- [ ] **Step 4: 注册测试** — 在 `CMakeLists.txt` 的 `ament_add_pytest_test(test_motor_probe ...)` 后加：

```cmake
  ament_add_pytest_test(test_motor_debug test/test_motor_debug.py)
```

- [ ] **Step 5: 运行测试确认通过**

Run: `cd /home/love009/robot_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select motor_drive && colcon test --packages-select motor_drive && colcon test-result --verbose`
Expected: `test_motor_debug` 全部 PASS。

- [ ] **Step 6: 提交**

```bash
git add src/motor_drive/scripts/motor_debug_tui.py src/motor_drive/test/test_motor_debug.py \
        src/motor_drive/CMakeLists.txt
git commit -m "feat(motor_drive): motor_debug_tui 新增配置解析与命令组帧"
```

---

### Task 6: `motor_debug_tui.py` — curses TUI、总线打开、控制循环、安全

**Files:**
- Modify: `src/motor_drive/scripts/motor_debug_tui.py`（追加运行时与 curses 逻辑）

**Interfaces:**
- Consumes: Task 5 的 `load_config`/`build_command_frames`，`motor_probe` 的
  `encode_frame`/`decode_frame`/`decode_motor_state`/`pack_rp_enable`/
  `pack_wrist_control`/`RP_ENABLE_ID`/`wrist_control_id`。
- Produces: 可运行的 `python3 src/motor_drive/scripts/motor_debug_tui.py --config ...`。

- [ ] **Step 1: 追加运行时与 curses 逻辑到 `motor_debug_tui.py`**

在 `build_command_frames` 之后追加：

```python
def configure_slcan(ser) -> None:
    for cmd in ("C\r", "S8\r", "Y5\r", "O\r"):
        ser.write(cmd.encode("ascii"))
        time.sleep(0.15)


def open_bus(bus: dict):
    try:
        ser = serial.Serial(bus["tty"], BAUD, timeout=0.01, write_timeout=1.0)
    except PermissionError as exc:
        raise PermissionError(
            f"无法打开 {bus['tty']}（权限不足），请 sudo 运行或先 "
            f"`sudo usermod -aG dialout $USER` 后重新登录") from exc
    configure_slcan(ser)
    return ser


def _prompt(stdscr, label: str) -> str:
    h, _ = stdscr.getmaxyx()
    curses.echo()
    curses.curs_set(1)
    stdscr.addstr(h - 2, 0, f"{label} " + " " * 40)
    stdscr.clrtoeol()
    value = stdscr.getstr(h - 2, len(label) + 1, 24).decode("utf-8", "ignore")
    curses.noecho()
    curses.curs_set(0)
    return value


def main(stdscr, config_path: str) -> int:
    curses.curs_set(0)
    stdscr.nodelay(True)

    try:
        config = load_config(config_path)
    except (OSError, ValueError, KeyError, yaml.YAMLError) as exc:
        curses.endwin()
        print(f"配置读取失败：{exc}")
        return 2

    motors = config["motors"]
    motors_by_bus = {}
    for bus in config["buses"]:
        motors_by_bus[bus["name"]] = [m for m in motors if m["bus"] == bus["name"]]

    opened = []  # [(bus_dict, serial)]
    try:
        for bus in config["buses"]:
            opened.append((bus, open_bus(bus)))
    except Exception as exc:
        for _, ser in opened:
            ser.close()
        curses.endwin()
        print(f"打开总线失败：{exc}")
        return 2

    states = {m["name"]: {} for m in motors}
    targets = {m["name"]: None for m in motors}
    enabled = {m["name"]: False for m in motors}
    gains = {m["name"]: [m["kp"], m["kd"]] for m in motors}
    step = 0.05
    selected = 0
    message = "启动：全失能 + 零增益"
    last_send = time.monotonic()
    send_period = 0.02  # 50 Hz

    def send_frames(bus: dict, ser) -> None:
        frames = build_command_frames(
            motors_by_bus[bus["name"]], states, targets, enabled, gains)
        for fid, data, is_fd, brs in frames:
            ser.write(encode_frame(fid, data, is_fd, brs).encode("ascii"))

    def rp_motor_names_on_same_bus(motor) -> list:
        return [m["name"] for m in motors_by_bus[motor["bus"]]
                if m["protocol"] == "rp"]

    def toggle_enable(idx: int) -> None:
        nonlocal message
        motor = motors[idx]
        names = [motor["name"]]
        if motor["protocol"] == "rp":
            names = rp_motor_names_on_same_bus(motor)  # RP 使能是总线级广播
        new_state = not enabled[motor["name"]]
        for name in names:
            enabled[name] = new_state
        for bus, ser in opened:
            if bus["name"] != motor["bus"]:
                continue
            if motor["protocol"] == "rp":
                ser.write(encode_frame(RP_ENABLE_ID, pack_rp_enable(new_state),
                                       False, False).encode("ascii"))
            else:
                ser.write(encode_frame(
                    wrist_control_id(motor["id"]),
                    pack_wrist_control(new_state), False, False).encode("ascii"))
        scope = "总线级" if motor["protocol"] == "rp" else "单台"
        message = f"{motor['name']} 已{'使能' if new_state else '失能'}（{scope}）"

    def do_set_zero(idx: int) -> None:
        nonlocal message
        motor = motors[idx]
        if motor["protocol"] != "vaxor":
            message = "设零位仅支持 vaxor"
            return
        if enabled[motor["name"]]:
            message = "设零位要求电机先失能"
            return
        for bus, ser in opened:
            if bus["name"] == motor["bus"]:
                ser.write(encode_frame(
                    wrist_control_id(motor["id"]),
                    pack_wrist_control(False, set_zero=True), False, False).encode("ascii"))
        message = f"{motor['name']} 已发送设零位指令"

    def render() -> None:
        stdscr.erase()
        stdscr.addstr(0, 0, f"motor_debug  config={config_path}  "
                            f"总线={len(config['buses'])} 电机={len(motors)}")
        header = (f"{'':>2} {'name':<20} {'joint':<22} {'type':<10} {'bus':<10} "
                  f"{'pos':>9} {'vel':>9} {'eff':>9} {'tmp/cur':>10} {'V':>6} {'fault':>8} en")
        stdscr.addstr(2, 0, header)
        for i, m in enumerate(motors):
            s = states[m["name"]]
            attr = curses.A_REVERSE if i == selected else curses.A_NORMAL
            pos = s.get("position", float("nan"))
            vel = s.get("velocity", float("nan"))
            eff = s.get("effort", float("nan"))
            extra = s.get("mos_temperature_c", s.get("current_a", float("nan")))
            volt = s.get("voltage_v", float("nan"))
            fault = s.get("fault", s.get("status", 0))
            en = "E" if enabled[m["name"]] else "-"
            line = (f"{i:>2} {m['name']:<20} {m['joint']:<22} {m['type']:<10} "
                    f"{m['bus']:<10} {pos:>9.3f} {vel:>9.3f} {eff:>9.3f} "
                    f"{extra:>10.1f} {volt:>6.1f} {fault:>8} {en}")
            stdscr.addstr(3 + i, 0, line, attr)
        stdscr.addstr(3 + len(motors) + 1, 0, message)
        stdscr.addstr(3 + len(motors) + 2, 0,
                      "↑↓选择  e使能/失能  +/-点动  [ ]步长  g调增益  z设零(vaxor)  "
                      "0全失能  q退出")
        stdscr.refresh()

    try:
        while True:
            # 1. drain 反馈
            for bus, ser in opened:
                for _ in range(20):
                    line = ser.read_until(b"\r")
                    if not line:
                        break
                    decoded = decode_frame(line.decode("ascii", errors="ignore"))
                    if decoded is None:
                        continue
                    fid, data, _ = decoded
                    for m in motors_by_bus[bus["name"]]:
                        state = decode_motor_state(fid, data, m)
                        if state is not None:
                            states[m["name"]] = state
                            if targets[m["name"]] is None and "position" in state:
                                targets[m["name"]] = state["position"]  # 锁存
                            break
            # 2. 读键
            key = stdscr.getch()
            if key != -1:
                if key in (ord("q"), 27):
                    break
                if key in (curses.KEY_UP, ord("j")) and motors:
                    selected = (selected - 1) % len(motors)
                elif key in (curses.KEY_DOWN, ord("k")) and motors:
                    selected = (selected + 1) % len(motors)
                elif key == ord("e") and motors:
                    if enabled[motors[selected]["name"]] or _confirm(stdscr):
                        toggle_enable(selected)
                elif key in (ord("+"), ord("=")) and motors:
                    if enabled[motors[selected]["name"]]:
                        targets[motors[selected]["name"]] = \
                            (targets[motors[selected]["name"]] or 0.0) + step
                        message = f"{motors[selected]['name']} target += {step}"
                elif key == ord("-") and motors:
                    if enabled[motors[selected]["name"]]:
                        targets[motors[selected]["name"]] = \
                            (targets[motors[selected]["name"]] or 0.0) - step
                        message = f"{motors[selected]['name']} target -= {step}"
                elif key == ord("[") and step > 0.001:
                    step /= 2.0
                    message = f"步长 {step:.4f} rad"
                elif key == ord("]") and step < 1.0:
                    step *= 2.0
                    message = f"步长 {step:.4f} rad"
                elif key == ord("g") and motors:
                    kp_s = _prompt(stdscr, f"kp[{motors[selected]['name']}]:")
                    kd_s = _prompt(stdscr, f"kd[{motors[selected]['name']}]:")
                    try:
                        kp, kd = float(kp_s), float(kd_s)
                        if kp < 0 or kd < 0 or kp != kp or kd != kd:
                            raise ValueError
                        gains[motors[selected]["name"]] = [kp, kd]
                        message = f"{motors[selected]['name']} kp={kp} kd={kd}"
                    except ValueError:
                        message = "增益无效：须为有限非负数"
                elif key == ord("z") and motors and _confirm(stdscr):
                    do_set_zero(selected)
                elif key == ord("0"):
                    for name in enabled:
                        enabled[name] = False
                    for bus, ser in opened:
                        for m in motors_by_bus[bus["name"]]:
                            if m["protocol"] == "rp":
                                ser.write(encode_frame(RP_ENABLE_ID,
                                                       pack_rp_enable(False),
                                                       False, False).encode("ascii"))
                                break  # RP 广播一次即可
                            ser.write(encode_frame(
                                wrist_control_id(m["id"]),
                                pack_wrist_control(False), False, False).encode("ascii"))
                    message = "已全部失能"
            # 3. 周期发命令
            now = time.monotonic()
            if now - last_send >= send_period:
                for bus, ser in opened:
                    send_frames(bus, ser)
                last_send = now
            render()
            time.sleep(0.002)
    finally:
        # 安全收尾：全失能 + 关串口 + 还原终端
        for name in enabled:
            enabled[name] = False
        try:
            for bus, ser in opened:
                for m in motors_by_bus[bus["name"]]:
                    if m["protocol"] == "rp":
                        ser.write(encode_frame(RP_ENABLE_ID, pack_rp_enable(False),
                                               False, False).encode("ascii"))
                        break
                    ser.write(encode_frame(
                        wrist_control_id(m["id"]),
                        pack_wrist_control(False), False, False).encode("ascii"))
        except Exception:
            pass
        for _, ser in opened:
            ser.close()
        curses.endwin()
    return 0


def _confirm(stdscr) -> bool:
    value = _prompt(stdscr, "确认？(y/N):")
    return value.strip().lower() in ("y", "yes")


def parse_args():
    parser = argparse.ArgumentParser(description="电机调试上位机（无 ROS，只 serial_slcan）")
    parser.add_argument("--config", default=DEFAULT_CONFIG,
                        help="motor_driver.yaml 路径（相对工作区根）")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    try:
        sys.exit(curses.wrapper(main, args.config))
    except KeyboardInterrupt:
        sys.exit(130)
```

实现时把 `motor_debug_tui.py` 顶部 import 扩充为如下（`import curses`/`import serial`
等只是普通 import，不会初始化终端，pytest 可安全 import 本模块；只有
`curses.wrapper`/`initscr` 才初始化终端）：

```python
import argparse
import curses
import sys
import time
from pathlib import Path

import serial
import yaml

from motor_probe import (
    decode_frame,
    decode_motor_state,
    encode_frame,
    pack_rp_command,
    pack_rp_enable,
    pack_wrist_command,
    pack_wrist_control,
    RP_COMMAND_ID,
    RP_ENABLE_ID,
    wrist_command_id,
    wrist_control_id,
)
```

- [ ] **Step 2: 语法/导入自检**

Run: `cd /home/love009/robot_ws && python3 -m py_compile src/motor_drive/scripts/motor_debug_tui.py && python3 src/motor_drive/scripts/motor_debug_tui.py --help`
Expected: 编译通过，`--help` 打印 `--config` 说明。

- [ ] **Step 3: 冒烟测试（无硬件，验证配置解析路径）**

Run: `cd /home/love009/robot_ws && python3 - <<'PY'
import sys; sys.path.insert(0, "src/motor_drive/scripts")
from motor_debug_tui import load_config
cfg = load_config("src/motor_hardware_interface/config/motor_driver.yaml")
print("buses:", [b["name"] for b in cfg["buses"]])
print("motors:", [(m["name"], m["protocol"], m["id"]) for m in cfg["motors"]])
PY`
Expected: 打印 `buses: ['left_arm', 'right_arm']` 与 14 台电机的 (name, protocol, id)，无异常。

- [ ] **Step 4: 提交**

```bash
git add src/motor_drive/scripts/motor_debug_tui.py
git commit -m "feat(motor_drive): motor_debug_tui curses 交互与安全收尾"
```

---

### Task 7: 文档

**Files:**
- Modify: `src/motor_drive/README.md`

**Interfaces:**
- Consumes: Task 6 完成的工具。

- [ ] **Step 1: 在 `motor_drive/README.md` 的「台架与手册」小节后补充使用说明**

在 `scripts/verify_2530_serial.py` 那条后追加：

```markdown
- `scripts/motor_debug_tui.py`：交互式电机调试上位机（curses，无 ROS，只
  serial_slcan）。读 `motor_driver.yaml` + `motor.yaml`，实时查看 RP/vaxor 每台
  电机状态，支持调增益、点动、使能/失能与 vaxor 设零位。从工作区根运行：

  ```bash
  python3 src/motor_drive/scripts/motor_debug_tui.py \
      --config src/motor_hardware_interface/config/motor_driver.yaml
  ```

  需 `/dev/ttyACM*` 写权限（dialout/sudo）。启动即全失能 + 零增益；`e` 使能前会
  锁存当前位置，`q`/异常退出自动全失能。软件无软限位，依赖硬件限位与急停。
```

- [ ] **Step 2: 提交**

```bash
git add src/motor_drive/README.md
git commit -m "docs(motor_drive): 补充 motor_debug_tui 使用说明"
```

---

## Self-Review

**Spec coverage:**
- 实时状态（RP/vaxor 私有遥测）→ Task 1/2/3（`decode_motor_state` 统一 dict）+ Task 6（渲染表）。✓
- 运行时调增益 → Task 6（`g` 键改写 `gains`，`build_command_frames` 每周期读取）。✓
- 点动 → Task 6（`+`/`-` 改 `targets`）。✓
- 使能/失能（vaxor 单台 / RP 总线级）→ Task 6（`toggle_enable` 按协议分组）。✓
- vaxor 设零 → Task 6（`do_set_zero`）。✓
- 只 serial_slcan、遇 socketcan 报错 → Task 5（`load_config` 抛 `ValueError`）。✓
- 安全（启动全失能、锁存、二次确认、退出全失能）→ Task 6。✓
- pytest → Task 1/2/3/5。✓
- 重构 verify → Task 4。✓

**Placeholder scan:** 无 TBD/TODO/「类似 Task N」。所有代码步骤含实际代码。

**Type consistency:** `decode_motor_state` 返回 dict 键（position/velocity/effort/
current_a/voltage_v/mos_temperature_c/coil_temperature_c/status/fault/motor_state/
calibration_state/mode/motor_type）在 Task 1/2/3 定义、Task 5/6 消费一致；
`build_command_frames` 返回 `(id, data, is_fd, brs)` 在 Task 5 定义、Task 6 消费一致；
`load_config` 返回键（buses/motors/timing，motor 含 name/joint/bus/type/protocol/id/
kp/kd/parameters）在 Task 5 定义、Task 6 消费一致。
