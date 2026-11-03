#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""巨蟹 58lite/48lite：SocketCAN can0 将当前位置标为零点。

电机节点 ID：1、2、3、4；标准 CAN ID = 0x600 + 节点 ID。
厂家零点指令：23 31 25 00 01 00 00 00 (SDO 写 0x2531:00 = 1)。

先将四个关节固定在想作为零点的姿态，并确保电机全部失能。
程序不负责运动、使能或失能电机；修改硬件零位会影响后续运动。

SocketCAN：通过 Linux can0 网络接口收发，兼容 CAN-FD 与经典 CAN。
安装：pip install python-can mujoco
Viewer：python3 zero_socketcan.py --viewer
只读：python3 zero_socketcan.py --check
标零：python3 zero_socketcan.py --apply
自测：python3 zero_socketcan.py --self-test

默认采用 CAN-FD + BRS。如果设备要求经典 CAN 2.0，添加 --classic。
如厂家固件不回复 SDO 读取/写入，可在电机已失能且可靠支撑的前提下，
使用 --send-only：只发送标零帧，不读取或声称标零成功。
"""

import argparse
import os
import sys
import time
from pathlib import Path

SOCKETCAN_BACKEND = 'socketcan'  # Linux SocketCAN（can0 / can1 等）
NODES = (1, 2, 3, 4)
SET_ZERO = bytes.fromhex('23 31 25 00 01 00 00 00')
READ_STATUS = bytes.fromhex('40 41 60 00 00 00 00 00')
READ_POSITION = bytes.fromhex('40 64 60 00 00 00 00 00')

# 唯一需要根据实际放置位置修改的模型路径。
MJCF_PATH = Path(__file__).resolve().parent / 'urdf' / '4.4.mjcf.xml'
# 真实电机角度 -> MuJoCo 关节角度，若某轴方向相反，将对应的 1 改为 -1。
JOINT_SIGNS = (1, 1, 1, 1)
COUNTS_PER_DEGREE = 182.0  # 延用原 zero.py 的厂家示例比例；请核对真实电机


def read_live_position(bus, can_module, node, use_fd, timeout=0.06):
    """只读 0x6064 位置。无响应返回 None，不改变电机状态。"""
    request = can_module.Message(
        arbitration_id=0x600 + node, is_extended_id=False,
        is_fd=use_fd, bitrate_switch=use_fd, data=READ_POSITION, check=True)
    bus.send(request, timeout=timeout)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=max(0, deadline - time.monotonic()))
        if msg is None:
            break
        if msg.is_extended_id or msg.arbitration_id != 0x580 + node:
            continue
        dat = bytes(msg.data)
        if len(dat) != 8 or dat[1:4] != bytes.fromhex('64 60 00'):
            continue
        lengths = {0x43: 4, 0x47: 3, 0x4B: 2, 0x4F: 1}
        if dat[0] == 0x80:
            raise RuntimeError(f'节点 {node} 读取位置被拒绝: {dat.hex(" ")}')
        if dat[0] in lengths:
            return int.from_bytes(dat[4:4 + lengths[dat[0]]], 'little', signed=True)
    return None


def show_viewer(iface, use_fd, timeout, mjcf_path):
    """真实关节 -> MuJoCo Viewer；仅显示，不发运动指令。"""
    try:
        import can
        import mujoco
        import mujoco.viewer
    except ImportError as exc:
        print(f'缺少依赖：{exc}。执行 pip install python-can mujoco', file=sys.stderr)
        return 2
    if not mjcf_path.is_file():
        print(f'找不到 MJCF 文件: {mjcf_path}', file=sys.stderr)
        return 2
    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    data = mujoco.MjData(model)
    names = [f'Joint_link{i}' for i in range(1, 5)]
    adr = []
    for name in names:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            raise RuntimeError(f'MJCF 中没有关节 {name}')
        adr.append(model.jnt_qposadr[jid])
    ensure_interface(iface, use_fd)
    print('Viewer 仅显示真实电机角度，不会驱动真实电机。拖动电机，观察仿真方向是否一致。')
    print('电机 ID 1-4 对应 Joint_link1-4；角度比例默认 182 码/度。')
    with can.Bus(channel=iface, interface=SOCKETCAN_BACKEND, fd=use_fd,
                 receive_own_messages=False) as bus:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            last_report = 0.0
            angles = [None] * 4
            while viewer.is_running():
                for i, node in enumerate(NODES):
                    counts = read_live_position(bus, can, node, use_fd,
                                                timeout=min(timeout, 0.06))
                    if counts is not None:
                        angle_deg = counts / COUNTS_PER_DEGREE
                        angles[i] = angle_deg
                        data.qpos[adr[i]] = JOINT_SIGNS[i] * angle_deg * 3.141592653589793 / 180.0
                mujoco.mj_forward(model, data)  # 不运行动力学，防止重力造成模型自行下垂
                viewer.sync()
                if time.monotonic() - last_report >= 0.5:
                    print('  '.join(f'J{i+1}={a:+.2f}°' if a is not None else f'J{i+1}=无反馈'
                                    for i, a in enumerate(angles)), flush=True)
                    last_report = time.monotonic()
                time.sleep(0.02)
    return 0



def print_commands(iface, nodes, use_fd):
    print('标零指令（标准11位CAN ID，数据8字节）：')
    for node in nodes:
        can_id = 0x600 + node
        sep = '##1' if use_fd else '#'
        print(f'  cansend {iface} {can_id:03X}{sep}{SET_ZERO.hex().upper()}')


def ensure_interface(iface, use_fd):
    """CAN-FD socket 的链路 MTU 应是72，经典 CAN 链路 MTU 是16。"""
    path = f'/sys/class/net/{iface}/mtu'
    if not os.path.isfile(path):
        raise RuntimeError(f'找不到接口 {iface}。请运行 ip -details link show {iface}')
    with open(path, encoding='ascii') as f:
        mtu = int(f.read().strip())
    print(f'SocketCAN {iface}: MTU={mtu}，发送格式={"CAN-FD+BRS" if use_fd else "经典CAN"}')
    if use_fd and mtu != 72:
        raise RuntimeError('can0没有开启CAN-FD (MTU应为72)。请检查驱动及 fd on 配置，或使用 --classic。')


def tx(bus, can_module, node, data, use_fd, timeout):
    """关键：FD模式下，不仅消息 is_fd=True，打开 Bus 时也必须 fd=True。"""
    msg = can_module.Message(
        arbitration_id=0x600 + node,
        is_extended_id=False,
        is_fd=use_fd,
        bitrate_switch=use_fd,
        data=data,
        check=True,
    )
    bus.send(msg, timeout=timeout)
    print(f'    TX 0x{msg.arbitration_id:03X}: {bytes(msg.data).hex(" ").upper()}')


def rx_sdo(bus, node, index, timeout):
    """接收指定电机、指定对象字典索引的 SDO 回复。"""
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(f'电机{node}：等待0x{0x580+node:03X}的0x{index:04X}回复超时')
        msg = bus.recv(timeout=remaining)
        if msg is None:
            continue
        if msg.is_error_frame:
            print(f'    CAN错误帧: {msg}', file=sys.stderr)
            continue
        if msg.is_extended_id or msg.arbitration_id != 0x580 + node:
            continue
        dat = bytes(msg.data)
        if len(dat) != 8 or dat[1:3] != index.to_bytes(2, 'little') or dat[3] != 0:
            continue
        print(f'    RX 0x{msg.arbitration_id:03X}: {dat.hex(" ").upper()}')
        if dat[0] == 0x80:
            error = int.from_bytes(dat[4:8], 'little')
            raise RuntimeError(f'电机{node}：SDO 0x{index:04X}中止码0x{error:08X}')
        return dat


def read_sdo(bus, can_module, node, index, request, use_fd, timeout):
    tx(bus, can_module, node, request, use_fd, timeout)
    dat = rx_sdo(bus, node, index, timeout)
    sizes = {0x43: 4, 0x47: 3, 0x4B: 2, 0x4F: 1}
    if dat[0] not in sizes:
        raise RuntimeError(f'电机{node}：意外的SDO读取响应 {dat.hex(" ")}')
    return int.from_bytes(dat[4:4 + sizes[dat[0]]], 'little', signed=True)


def inspect(bus, can_module, node, use_fd, timeout):
    status = read_sdo(bus, can_module, node, 0x6041, READ_STATUS, use_fd, timeout)
    print(f'  电机{node}：状态字=0x{status & 0xFFFF:04X}')
    # CiA402 状态字 bit2=Operation enabled；拒绝在运行使能状态标零。
    if status & 0x0004:
        raise RuntimeError(f'电机{node}仍处于 Operation enabled，请先失能并固定关节')
    pos = read_sdo(bus, can_module, node, 0x6064, READ_POSITION, use_fd, timeout)
    print(f'  电机{node}：当前0x6064={pos:+d}计数，约{pos/182:+.3f}度（厂家示例比例）')
    return pos


def write_zero(bus, can_module, node, use_fd, timeout, verify):
    tx(bus, can_module, node, SET_ZERO, use_fd, timeout)
    if not verify:
        print(f'  电机{node}：已请求发送标零帧，未验证电机是否执行成功')
        return
    response = rx_sdo(bus, node, 0x2531, timeout)
    if response[0] != 0x60:
        raise RuntimeError(f'电机{node}：未收到0x60标准写入确认')
    time.sleep(0.10)
    pos = read_sdo(bus, can_module, node, 0x6064, READ_POSITION, use_fd, timeout)
    print(f'  电机{node}：标零后0x6064={pos:+d}计数，约{pos/182:+.3f}度')
    if abs(pos / 182.0) > 1.0:
        raise RuntimeError(f'电机{node}：位置仍偏离零点超过1度，请勿盲目重复标零')


def self_test():
    assert SET_ZERO.hex() == '2331250001000000'
    assert READ_STATUS.hex() == '4041600000000000'
    assert READ_POSITION.hex() == '4064600000000000'
    assert [0x600+n for n in NODES] == [0x601, 0x602, 0x603, 0x604]
    class FakeCan:
        class Message:
            def __init__(self, **kwargs):
                self.__dict__.update(kwargs)
    class FakeBus:
        def __init__(self): self.messages = []
        def send(self, msg, timeout): self.messages.append(msg)
    bus = FakeBus()
    tx(bus, FakeCan, 1, SET_ZERO, True, 0.1)
    assert bus.messages[0].is_fd and bus.messages[0].bitrate_switch
    assert bus.messages[0].arbitration_id == 0x601
    print('自测通过：CAN ID、8字节标零指令及FD帧字段正确。')


def main():
    parser = argparse.ArgumentParser(description='巨蟹四关节 SocketCAN 当前角度标零')
    parser.add_argument('--interface', default='can0')
    parser.add_argument('--nodes', nargs='+', type=int, default=list(NODES))
    parser.add_argument('--classic', action='store_true', help='改用经典CAN 2.0帧')
    parser.add_argument('--check', action='store_true', help='只读取状态和位置，不标零')
    parser.add_argument('--apply', action='store_true', help='检查失能后逐台标零并验证')
    parser.add_argument('--send-only', action='store_true', help='跳过SDO读取/回复，仅发送标零指令，结果未验证')
    parser.add_argument('--viewer', action='store_true', help='打开4.4 MuJoCo Viewer，显示真实电机角度')
    parser.add_argument('--mjcf', type=Path, default=MJCF_PATH, help='4.4 MJCF路径')
    parser.add_argument('--self-test', action='store_true')
    parser.add_argument('--timeout', type=float, default=1.5)
    args = parser.parse_args()

    if args.self_test:
        self_test()
        return 0
    if sum([args.check, args.apply, args.send_only, args.viewer]) > 1:
        parser.error('--check、--apply、--send-only、--viewer只能选一个')
    if not args.nodes or len(set(args.nodes)) != len(args.nodes) or not all(1 <= n <= 127 for n in args.nodes):
        parser.error('节点ID必须互不重复，且在1～127')
    if args.timeout <= 0:
        parser.error('--timeout必须大于零')

    use_fd = not args.classic
    if args.viewer:
        return show_viewer(args.interface, use_fd, args.timeout, args.mjcf.resolve())
    print_commands(args.interface, args.nodes, use_fd)
    if not (args.check or args.apply or args.send_only):
        print('\n以上只是预览，未打开CAN设备，也未修改零点。')
        return 0

    try:
        import can
    except ImportError:
        print('缺少依赖，请执行 pip install python-can', file=sys.stderr)
        return 2

    if args.apply or args.send_only:
        print('\n警告：这将修改电机机械零点！')
        print('请将各轴固定在正确零位，确认四台电机均已失能、机械臂有支撑。')
        if args.send_only:
            print('SEND-ONLY 不会读取状态/位置，也无法判断标定是否成功。')
        confirm = input('确认完成后，输入 SET ZERO：').strip()
        if confirm != 'SET ZERO':
            print('已取消，未发送任何CAN帧。')
            return 0

    writes_started = False
    try:
        ensure_interface(args.interface, use_fd)
        # !!! 关键修复：打开 SocketCAN 时设置 fd=True !!!
        with can.Bus(channel=args.interface, interface=SOCKETCAN_BACKEND,
                     fd=use_fd, receive_own_messages=False) as bus:
            if not args.send_only:
                print('\n[标零前只读检查]')
                for node in args.nodes:
                    inspect(bus, can, node, use_fd, args.timeout)
            if args.check:
                print('\n只读检查完成，未修改零点。')
                return 0
            print('\n[开始设置零点]')
            for node in args.nodes:
                # 注意：发送成功不等于驱动器一定执行了标零命令。
                writes_started = True
                write_zero(bus, can, node, use_fd, args.timeout, verify=not args.send_only)
                time.sleep(0.15)
            print('\n四台电机的标零帧已依次发送。请重启后复查是否持久保存。')
            return 0
    except (Exception, KeyboardInterrupt) as exc:
        print(f'\n停止：{exc}', file=sys.stderr)
        if writes_started:
            print('可能已有部分电机完成标零，请先读取位置，勿直接重复执行！', file=sys.stderr)
        else:
            print('此次运行未发送标零写入帧。', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
