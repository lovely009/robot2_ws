"""真机 CAN 工具共用的 SLCAN 编解码。"""

DEFAULT_DATA_BITRATE_INDEX = 5

FD_LEN_CODE = {0: "0", 1: "1", 2: "2", 3: "3", 4: "4", 5: "5", 6: "6",
               7: "7", 8: "8", 12: "9", 16: "A", 20: "B", 24: "C",
               32: "D", 48: "E", 64: "F"}

_CODE_LEN = {v: k for k, v in FD_LEN_CODE.items()}

def slcan_config_commands(
        data_bitrate_index: int = DEFAULT_DATA_BITRATE_INDEX) -> tuple[bytes, ...]:
    """构造关闭、位率、正常模式、自动重发和开通道命令。"""
    if not 0 <= data_bitrate_index <= 9:
        raise ValueError("SLCAN data bitrate index must be one decimal digit")
    return (b"C\r", b"S8\r", f"Y{data_bitrate_index}\r".encode(),
            b"M0\r", b"A1\r", b"O\r")

def slcan_line(can_id: int, payload: bytes, is_fd: bool, brs: bool) -> bytes:
    """标准 11bit ID 的 SLCAN 行（CANable2 方言），以 \\r 结尾。"""
    if can_id > 0x7FF:
        raise ValueError("standard ID out of range")
    cmd = "t" if not is_fd else ("b" if brs else "d")
    # 非法长度明确报错，便于定位发送帧。
    code = FD_LEN_CODE.get(len(payload))
    if code is None:
        raise ValueError(
            f"CAN FD 非法负载长度 {len(payload)}（id=0x{can_id:03X}，"
            f"合法：{sorted(FD_LEN_CODE)}）")
    return f"{cmd}{can_id:03X}{code}".encode() \
        + payload.hex().upper().encode() + b"\r"

def parse_slcan(line: str):
    """解析固件上报帧 -> (id, data, is_fd)；非数据帧返回 None。"""
    line = line.strip()
    if len(line) < 5:
        return None
    cmd, rest = line[0], line[1:]
    if cmd in "tT":
        is_fd, id_len = False, 3 if cmd == "t" else 8
    elif cmd in "dDbB":
        is_fd, id_len = True, 3 if cmd in "db" else 8
    else:
        return None  # a/w/f/错误状态等忽略
    try:
        can_id = int(rest[:id_len], 16)
        n = _CODE_LEN[rest[id_len]]
        data = bytes.fromhex(rest[id_len + 1:id_len + 1 + 2 * n])
    except (ValueError, KeyError):
        return None
    if len(data) != n:
        return None
    return can_id, data, is_fd
