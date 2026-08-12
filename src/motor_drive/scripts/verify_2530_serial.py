#!/usr/bin/env python3
"""通过 CANable2 的串口 slcan-FD 方言直接验证 25/30 mm（motor_2530）电机。

不依赖 SocketCAN、不刷固件：把 /dev/ttyACM0 当作 CAN 通道，
按固件命令集（C/S8/Y5/O + t/b 帧）收发控制与反馈帧。

用法：
    sudo python3 verify_2530_serial.py [--id 1] [--listen 3] [--motion-test]

默认流程（安全）：
    1. 配置总线：C（关闭）→ S8（仲裁 1M）→ Y5（数据 5M）→ O（打开）
    2. 静默监听 1.5 s，确认反馈通路
    3. 发送使能帧（run=1，经典 CAN，ID 0x506）
    4. 监听并解码反馈（回复 0x705 / 状态 0x706）
    5. 发送零力矩命令（Kp=0/Kd=0/前馈=0，电机不会主动转动）
    6. 退出前发送关闭帧（run=0），电机断电退磁

--motion-test 会在拿到当前位置后，以 Kp=0.008/Kd=1 向当前位置 +
--angle-deg 步进一个小角度保持 2 s，然后回到零力矩并关闭。
电机使能后会有保持力矩，请固定好电机、手不要伸入活动范围。

需要 /dev/ttyACM0 的写权限：当前用户不在 dialout 组时用 sudo 运行，
或先执行 `sudo usermod -aG dialout $USER` 并重新登录。
"""

from __future__ import annotations

import argparse
import sys
import time

import serial

TTY_DEFAULT = "/dev/ttyACM0"
BAUD = 115200  # USB CDC 实际按 USB 全速 bulk 传输，波特率只是名义值

# ---- 25/30 mm 协议常量（与 motor_drive/protocol.hpp 保持一致）----
WR_TICKS_PER_REV = 2_097_152
WR_CURRENT_SCALE_A = 11.0 / 4096.0
WR_KP_SCALE = 0.00001
WR_KD_SCALE = 0.01


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


# 编码器自校准指令（16-30mm 手册 V20260720 §3.7，视觉核验 Data2=0xCC）
WR_CALIB_PAYLOAD = bytes([0x55, 0xAA, 0xCC, 0x33, 0x77, 0x11, 0xFF, 0x00])


def pos_to_code(rad: float) -> int:
    ticks = round(rad / (2.0 * 3.141592653589793) * WR_TICKS_PER_REV)
    return ticks + 0x80000000


def code_to_pos(code: int) -> float:
    return (code - 0x80000000) / WR_TICKS_PER_REV * 2.0 * 3.141592653589793


def vel_to_code(rad_s: float) -> int:
    rpm10 = round(rad_s * 60.0 / (2.0 * 3.141592653589793) * 10.0)
    return max(-30000, min(30000, rpm10)) + 0x8000


def code_to_vel(code: int) -> float:
    return (code - 0x8000) / 10.0 * 2.0 * 3.141592653589793 / 60.0


def current_to_code(amp: float) -> int:
    return max(-32768, min(32767, round(amp / WR_CURRENT_SCALE_A))) + 0x8000


def code_to_current(code: int) -> float:
    return (code - 0x8000) * WR_CURRENT_SCALE_A


def le32(data: bytes, off: int) -> int:
    return int.from_bytes(data[off : off + 4], "little")


def le16(data: bytes, off: int) -> int:
    return int.from_bytes(data[off : off + 2], "little")


def fd_len_code(nbytes: int) -> str:
    """slcan FD 长度码：0-8 原样，9=12, A=16, B=20, C=24, D=32, E=48, F=64。"""
    mapping = {0: "0", 1: "1", 2: "2", 3: "3", 4: "4",
               5: "5", 6: "6", 7: "7", 8: "8", 12: "9",
               16: "A", 20: "B", 24: "C", 32: "D", 48: "E", 64: "F"}
    try:
        return mapping[nbytes]
    except KeyError:
        raise ValueError(f"unsupported slcan FD length {nbytes}") from None


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
    """解码 CANable2 上报的帧，返回 (id, data, is_fd) 或 None。"""
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
        return None  # 状态/错误帧等，忽略
    if len(rest) < id_len + 1:
        return None
    try:
        identifier = int(rest[:id_len], 16)
        len_code = rest[id_len]
        payload = rest[id_len + 1 :]
    except ValueError:
        return None
    mapping = {"0": 0, "1": 1, "2": 2, "3": 3, "4": 4, "5": 5, "6": 6,
               "7": 7, "8": 8, "9": 12, "A": 16, "B": 20, "C": 24,
               "D": 32, "E": 48, "F": 64}
    nbytes = mapping.get(len_code)
    if nbytes is None or len(payload) != nbytes * 2:
        return None
    try:
        data = bytes.fromhex(payload)
    except ValueError:
        return None
    return identifier, data, is_fd


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


def describe_feedback(frame_id: int, data: bytes, node: int) -> str:
    reply_id = wrist_reply_id(node)
    status_id = wrist_status_id(node)
    if frame_id == reply_id and len(data) >= 12:
        return (
            f"回复 0x{frame_id:03X} pos={code_to_pos(le32(data, 2)):+.4f} rad "
            f"vel={code_to_vel(le16(data, 6)):+.3f} rad/s "
            f"cur={code_to_current(le16(data, 8)):+.3f} A "
            f"[{data.hex().upper()}]"
        )
    if frame_id == status_id and len(data) >= 20:
        # 实测固件（V20260204 及以后）电压字段为无偏置 u16/10 V；
        # 手册里带 0x8000 偏置的公式与实际不符（实测 12V 供电读到 119 -> 11.9V）。
        voltage = le16(data, 10) / 10.0
        fault = le32(data, 12)
        return (
            f"状态 0x{frame_id:03X} pos={code_to_pos(le32(data, 2)):+.4f} rad "
            f"vel={code_to_vel(le16(data, 6)):+.3f} rad/s "
            f"cur={code_to_current(le16(data, 8)):+.3f} A "
            f"voltage={voltage:.1f} V fault=0x{fault:08X} "
            f"motor_state={data[16]} calib={data[17]} mode={data[18]} type={data[19]} "
            f"[{data.hex().upper()}]"
        )
    return f"帧 0x{frame_id:03X} len={len(data)} data={data.hex().upper()}"


def send(ser: serial.Serial, line: str) -> None:
    ser.write(line.encode("ascii"))
    print(f"  TX {line.strip()}")


def listen(ser: serial.Serial, seconds: float, label: str) -> int:
    deadline = time.monotonic() + seconds
    count = 0
    while time.monotonic() < deadline:
        line = ser.read_until(b"\r")
        if not line:
            continue
        decoded = decode_frame(line.decode("ascii", errors="ignore"))
        if decoded is None:
            continue
        frame_id, data, is_fd = decoded
        count += 1
        kind = "FD" if is_fd else "经典"
        print(f"  RX [{kind}] {describe_feedback(frame_id, data, ARGS.node)}")
    print(f"  {label}: 共收到 {count} 帧")
    return count


def run_calibration(ser: serial.Serial, node: int) -> int:
    """发送编码器自校准指令并等待结果帧（约 10s，期间严禁断电）。"""
    calib_id = wrist_calib_id(node)
    result_id = wrist_calib_result_id(node)
    control_id = wrist_control_id(node)
    print(f"=== 编码器自校准：节点 {node}（ID 0x{calib_id:X}）===")
    print("注意：校准约 10s，瞬时电流 >1A，全程禁止断电；电机可能轻微转动/振动。")
    ser.reset_input_buffer()
    # 手册 4.4：0005（停机+切 MIT）→ 0006（校准指令），停机指令必须先发。
    send(ser, encode_frame(control_id, bytes([0, 0, 0, 0, 0, 0, 0, 0]),
                           is_fd=False, brs=False))
    time.sleep(0.6)
    send(ser, encode_frame(calib_id, WR_CALIB_PAYLOAD, is_fd=False, brs=False))

    deadline = time.monotonic() + 25.0
    while time.monotonic() < deadline:
        line = ser.read_until(b"\r")
        decoded = decode_frame(line.decode("ascii", errors="ignore"))
        if decoded is None:
            continue
        frame_id, data, _ = decoded
        if frame_id == result_id and len(data) >= 8:
            if data[0] == 0x02:
                print(f"  [OK] 校准完成（0x{result_id:X} Data0=0x02）")
                return 0
            if data[0] == 0x03:
                print(f"  [FAIL] 校准错误（0x{result_id:X} Data0=0x03）")
                return 1
        if frame_id == wrist_status_id(node) and len(data) >= 20:
            calib = data[17]
            print(f"  [状态] state={data[16]} calib={calib} "
                  f"fault=0x{le32(data, 12):08X} volt={le16(data, 10) / 10.0:.1f}V")
            if calib == 2:
                print("  [OK] 状态帧确认编码器零点校准完成（calib=2）")
                return 0
            if calib == 3:
                print("  [FAIL] 状态帧报校准错误（calib=3）")
                return 1
    print("  [TIMEOUT] 25s 内未收到校准结果，请检查供电是否稳定后重试")
    return 2


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tty", default=TTY_DEFAULT)
    parser.add_argument("--id", type=int, default=1, dest="node",
                        help="25/30 mm 节点 ID，默认 1")
    parser.add_argument("--listen", type=float, default=3.0,
                        help="使能后监听秒数，默认 3")
    parser.add_argument("--motion-test", action="store_true",
                        help="做一次小幅位置保持测试（默认只做零力矩验证）")
    parser.add_argument("--calibrate", action="store_true",
                        help="仅执行编码器自校准（10s），不做运动测试")
    parser.add_argument("--angle-deg", type=float, default=10.0,
                        help="motion-test 的步进角度（度），默认 10")
    global ARGS
    ARGS = parser.parse_args()

    if ARGS.node < 1 or ARGS.node > 60:
        print(f"错误：节点 ID 必须在 [1, 60]，收到 {ARGS.node}")
        return 2

    try:
        ser = serial.Serial(ARGS.tty, BAUD, timeout=0.2, write_timeout=1.0)
    except PermissionError:
        print(
            f"无法打开 {ARGS.tty}（权限不足）。\n"
            f"请用 sudo 运行本脚本，或先执行：\n"
            f"    sudo usermod -aG dialout $USER\n"
            f"然后重新登录后再运行。"
        )
        return 2
    except serial.SerialException as exc:
        print(f"打开 {ARGS.tty} 失败：{exc}")
        print("请确认 CANable2 已插入且枚举为串口（lsusb 应显示 16d0:117e）。")
        return 2

    cmd_id = wrist_command_id(ARGS.node)
    ctl_id = wrist_control_id(ARGS.node)

    if ARGS.calibrate:
        ser.reset_input_buffer()
        print("配置总线：C -> S8(1M) -> Y5(5M) -> O")
        for cmd in ("C\r", "S8\r", "Y5\r", "O\r"):
            send(ser, cmd)
            time.sleep(0.15)
        rc = run_calibration(ser, ARGS.node)
        ser.close()
        return rc

    print(f"=== CANable2 串口验证：25/30 mm 电机（节点 {ARGS.node}）===")
    print(f"命令 ID 0x{cmd_id:X}（FD+BRS）  控制 ID 0x{ctl_id:X}（经典）")
    ser.reset_input_buffer()

    print("[1/6] 配置总线：C -> S8(1M) -> Y5(5M) -> O")
    for cmd in ("C\r", "S8\r", "Y5\r", "O\r"):
        send(ser, cmd)
        time.sleep(0.15)

    print("[2/6] 静默监听 1.5s（不使能，确认反馈通路）")
    listen(ser, 1.5, "基线监听")

    print("[3/6] 发送使能帧（run=1）")
    enable_payload = bytes([1, 0, 0, 0, 0, 0, 0, 0])
    send(ser, encode_frame(ctl_id, enable_payload, is_fd=False, brs=False))

    print(f"[4/6] 监听 {ARGS.listen}s，解码反馈")
    n = listen(ser, ARGS.listen, "使能后监听")
    if n == 0:
        print("!! 未收到任何反馈帧：请检查 CAN 接线（CANH/CANL/120Ω 终端）、")
        print("   电机供电与节点 ID，以及总线位速是否与电机匹配。")

    print("[5/6] 发送零力矩命令（Kp=0/Kd=0/前馈=0，电机不会主动转动）")
    safe_cmd = pack_wrist_command(0.0, 0.0, 0.0, 0.0, 0.0)
    send(ser, encode_frame(cmd_id, safe_cmd, is_fd=True, brs=True))
    time.sleep(0.2)

    if ARGS.motion_test and n > 0:
        print(f"[6/6] 位置保持测试：向当前位置 + {ARGS.angle_deg}° 步进 2s")
        print("     （Kp=0.008/Kd=1，固定好电机，手不要伸入活动范围）")
        # 从最新反馈解出位置：再听 0.5s 取最后出现的位置码
        latest_pos = 0.0
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            line = ser.read_until(b"\r")
            decoded = decode_frame(line.decode("ascii", errors="ignore"))
            if decoded is None:
                continue
            frame_id, data, _ = decoded
            if frame_id in (wrist_reply_id(ARGS.node), wrist_status_id(ARGS.node)) \
                    and len(data) >= 12:
                latest_pos = code_to_pos(le32(data, 2))
        target = latest_pos + ARGS.angle_deg * 3.141592653589793 / 180.0
        hold = pack_wrist_command(target, 0.0, 0.008, 1.0, 0.0)
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            send(ser, encode_frame(cmd_id, hold, is_fd=True, brs=True))
            time.sleep(0.1)
        safe_cmd = pack_wrist_command(0.0, 0.0, 0.0, 0.0, 0.0)
        send(ser, encode_frame(cmd_id, safe_cmd, is_fd=True, brs=True))

    print("[收尾] 发送关闭帧（run=0），电机退磁")
    disable_payload = bytes([0, 0, 0, 0, 0, 0, 0, 0])
    send(ser, encode_frame(ctl_id, disable_payload, is_fd=False, brs=False))
    ser.close()
    print("=== 验证结束 ===")
    return 0


if __name__ == "__main__":
    sys.exit(main())
