#!/usr/bin/env python3
"""
motor_master.py

Interactive CLI master for testing Protocol A and Protocol B over CAN FD via ROS2 topics.
Sends frames to /to_can_bus_fd and listens to /from_can_bus_fd to print decoded physical values.

Run:
  ros2 run can_motor_sim motor_master --protocol A --id 1

"""

import argparse
import struct
import sys
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node


def _hex(data):
    return ' '.join('%02X' % b for b in data)

# Message type from ros2_socketcan
try:
    from ros2_socketcan_msgs.msg import FdFrame
except Exception:
    class FdFrame:
        def __init__(self):
            self.id = 0
            self.is_extended = False
            self.dlc = 0
            self.data = bytearray()


class MotorMaster(Node):
    def __init__(self, protocol='A', node_id=1, status_interval=1.0):
        super().__init__('motor_master_%s_%d' % (protocol.lower(), node_id))
        self.protocol = protocol.upper()
        self.node_id = int(node_id)

        self.pub = self.create_publisher(FdFrame, '/to_can_bus_fd', 10)
        self.sub = self.create_subscription(FdFrame, '/from_can_bus_fd', self.on_frame, 50)

        # Periodic status frames are NOT printed by default: watch the bus with
        # `candump vcan0` instead. `status on` re-enables a throttled print
        # (default once per second) for convenience.
        self.status_interval = float(status_interval)
        self.status_enabled = False
        self._last_status_time = 0.0
        self.quit_requested = False  # set by the 'quit' command to stop the node

        # Loopback guard: the bridge echoes our own frames back on
        # /from_can_bus_fd. Keep a short queue of recently sent frames so the
        # echo of ANY of them is ignored even when commands are sent back to
        # back (a single last-frame slot would miss earlier echoes).
        self._recent_tx = deque()  # (time, id, data)

        # Register read/write is one-question-one-answer with a 50ms reply
        # timeout (doc section 2). Track pending requests and report timeouts.
        self._pending_reply = {}   # id -> send time
        self.create_timer(0.02, self._check_reply_timeout)

        self.get_logger().info('Motor master started: protocol=%s id=%s status_print=off(interval=%.1fs)'
                               % (self.protocol, self.node_id, self.status_interval))

    def _log_status(self, text):
        """Log a periodic status line, throttled to at most one per interval."""
        if not self.status_enabled:
            return
        now = time.time()
        if now - self._last_status_time >= self.status_interval:
            self._last_status_time = now
            self.get_logger().info(text)

    def make_frame(self, can_id, data_bytes, is_extended=False):
        msg = FdFrame()
        msg.id = int(can_id)
        if hasattr(msg, 'is_extended'):
            msg.is_extended = bool(is_extended)
        elif hasattr(msg, 'is_extended_id'):
            msg.is_extended_id = bool(is_extended)
        # support both 'dlc' and 'len' fields
        if hasattr(msg, 'dlc'):
            msg.dlc = len(data_bytes)
        elif hasattr(msg, 'len'):
            msg.len = len(data_bytes)
        msg.data = bytearray(data_bytes)
        return msg

    def publish(self, can_id, data_bytes, is_extended=False):
        self._recent_tx.append((time.time(), int(can_id), bytes(data_bytes)))
        if len(self._recent_tx) > 128:
            self._recent_tx.popleft()
        msg = self.make_frame(can_id, data_bytes, is_extended=is_extended)
        self.pub.publish(msg)
        # Echo every transmitted frame so it can be cross-checked against candump.
        print('[TX:%s] id=0x%08X len=%d data=%s'
              % ('E' if is_extended else 'S', int(can_id), len(data_bytes), _hex(data_bytes)))

    def _check_reply_timeout(self):
        """Report register requests that received no reply within 50ms."""
        now = time.time()
        for rid in list(self._pending_reply):
            if now - self._pending_reply[rid] > 0.05:
                self.get_logger().warning('[B TIMEOUT] no reply within 50ms, id=0x%08X' % rid)
                del self._pending_reply[rid]

    def _is_loopback(self, can_id, data):
        """True if this inbound frame matches one we sent within the last 0.2s."""
        now = time.time()
        for i in range(len(self._recent_tx) - 1, -1, -1):
            t, fid, fdata = self._recent_tx[i]
            if now - t > 0.2:
                del self._recent_tx[i]
                continue
            if can_id == fid and data == fdata:
                return True
        return False

    def on_frame(self, msg):
        can_id = int(msg.id)
        data = bytes(msg.data) if msg.data is not None else b''
        is_ext = getattr(msg, 'is_extended', False)
        # Ignore the bridge's echo of our own just-sent frames (loopback).
        if self._is_loopback(can_id, data):
            return
        # decode based on protocol and incoming id
        if self.protocol == 'A':
            # A2 replies: 0x701 + 4*N
            # A3 status: 0x702 + 4*N
            expected_reply = 0x701 + 4 * self.node_id
            expected_status = 0x702 + 4 * self.node_id
            # A2 reply (DLC12): byte0=0x01+4N, byte1=0x05, then pos/vel/current
            # with 0x80000000 / 0x8000 offsets (see manual 3.2).
            if can_id == expected_reply and len(data) >= 12:
                pos = struct.unpack_from('<I', data, 2)[0] - 0x80000000
                vel = struct.unpack_from('<H', data, 6)[0] - 0x8000
                current = struct.unpack_from('<H', data, 8)[0] - 0x8000
                angle_deg = pos / 2097152.0 * 360.0
                rpm = vel / 10.0
                currentA = current * 11.0 / 4096.0
                self.get_logger().info('[A REPLY] pos_code=%d angle=%.4fdeg vel_code=%d rpm=%.2f current=%.3fA'
                                       % (pos, angle_deg, vel, rpm, currentA))
            elif can_id == expected_status and len(data) >= 20:
                # A3 status (DLC20): pos/vel/current/vbus all offset-encoded
                pos = struct.unpack_from('<I', data, 2)[0] - 0x80000000
                vel = struct.unpack_from('<H', data, 6)[0] - 0x8000
                current = struct.unpack_from('<H', data, 8)[0] - 0x8000
                vbus = struct.unpack_from('<H', data, 10)[0] - 0x8000
                fault = struct.unpack_from('<I', data, 12)[0]
                motor_state = data[16]
                enc_calib = data[17]
                mode = data[18]
                mtype = data[19]
                angle_deg = pos / 2097152.0 * 360.0
                rpm = vel / 10.0
                currentA = current * 11.0 / 4096.0
                vbusV = vbus / 10.0
                self._log_status('[A STATUS] angle=%.4fdeg rpm=%.2f current=%.3fA Vbus=%.1fV state=%d mode=%d type=0x%02X fault=0x%08X' % (angle_deg, rpm, currentA, vbusV, motor_state, mode, mtype, fault))
            else:
                self.get_logger().debug('A: Received frame id=0x%X dlc=%d' % (can_id, len(data)))
        else:
            # Protocol B
            # B3: standard reply id == device id
            if not is_ext and (can_id & 0xFF) in range(1, 9):
                dev = can_id & 0xFF
                if len(data) >= 10:
                    pos16 = (data[0] << 8) | data[1]
                    # map pos16 to radians -10..10
                    pos_rad = (pos16 / 65535.0) * 20.0 - 10.0
                    # velocity decode
                    vel_tmp = ((data[2] << 4) | (data[3] >> 4)) & 0xFFF
                    vel = (vel_tmp - 0x7FF) * (60.0 / 0x7FF)
                    # torque decode
                    torque_tmp = (((data[3] & 0x0F) << 8) | data[4]) & 0xFFF
                    torque = (torque_tmp - 0x7FF) * (60.0 / 0x7FF)
                    # doc 4.1.3: byte5 MOS temp, byte6-7 status, byte8 voltage, byte9 coil
                    mos = data[5] - 40
                    status = (data[7] << 8) | data[6]
                    volt = data[8]
                    coil = data[9] - 40
                    self._log_status('[B DEV %d] pos=%+.3frad vel=%+.1frad/s tor=%+.2fNm '
                                     'MOS=%dC st=0x%04X V=%dV coil=%dC'
                                     % (dev, pos_rad, vel, torque, mos, status, volt, coil))
                else:
                    self.get_logger().info('[B DEV %d] short frame id=0x%X dlc=%d' % (can_id & 0xFF, can_id, len(data)))
            elif is_ext:
                # register read/write response, parse 29-bit id per doc layout
                id29 = can_id & 0x1FFFFFFF
                op = (id29 >> 28) & 0x1
                sub = (id29 >> 24) & 0xF
                reg_addr = (id29 >> 16) & 0xFF
                succ = (id29 >> 15) & 0x1
                product_id = (id29 >> 8) & 0x7F
                dev_id = id29 & 0x7F
                val = int.from_bytes(data, byteorder='little', signed=False) if len(data) > 0 else 0
                latency = ''
                if id29 in self._pending_reply:
                    dt = (time.time() - self._pending_reply.pop(id29)) * 1000.0
                    latency = ' latency=%.1fms' % dt
                self.get_logger().info('[B REG RESP] dev=%d reg=0x%02X sub=0x%X op=%s succ=%d val=%s%s'
                                       % (dev_id, reg_addr, sub, 'R' if op == 0 else 'W', succ, val, latency))
            else:
                self.get_logger().debug('B: Received frame id=0x%X dlc=%d' % (can_id, len(data)))


def interactive_loop(master: MotorMaster):
    print('Enter commands. Type "help" for a list, "quit" to exit.')
    while True:
        try:
            line = input('> ')
        except EOFError:
            break
        parts = line.strip().split()
        if not parts:
            continue
        cmd = parts[0].lower()
        if cmd == 'help':
            print('Protocol A commands: enable, disable, mode mit|pos|vel,')
            print('  set_pos <pos_code>          (pos_code: 0x80000000 offset)')
            print('  set_vel <vel_code>          (vel_code: 0x8000 offset)')
            print('  set_mit <pos> <vel> <kp_raw> <kd_raw> <iff>   (manual 3.1 raw codes)')
            print('Protocol B commands: enable, disable,')
            print('  rp_mit <joint 1-8> <pos16> <vel12> <kp> <kd> <torque12>')
            print('  rp_pp <pos_rad>            (PP: 0x80+N, 4B float position)')
            print('  rp_pv <vel_rad_s>          (PV: 0x80+N, 4B float velocity)')
            print('  rp_reg read <addr> [sub]')
            print('  rp_reg write <addr> [sub] <value>')
            print('Common: status on|off|<interval_sec>, quit')
            continue
        if cmd == 'status':
            if len(parts) < 2:
                print('Usage: status on|off|<interval_sec> (current: enabled=%s interval=%.1fs)'
                      % (master.status_enabled, master.status_interval))
                continue
            if parts[1] == 'on':
                master.status_enabled = True
                print('Status logging ON (every %.1fs)' % master.status_interval)
            elif parts[1] == 'off':
                master.status_enabled = False
                print('Status logging OFF')
            else:
                try:
                    master.status_interval = max(0.0, float(parts[1]))
                    master.status_enabled = True
                    print('Status logging every %.2fs' % master.status_interval)
                except ValueError:
                    print('Usage: status on|off|<interval_sec>')
            continue
        if cmd == 'quit' or cmd == 'exit':
            master.quit_requested = True
            break
        if master.protocol == 'A':
            if cmd == 'enable' or cmd == 'disable':
                # send B1-like? For protocol A enabling is via control command A4
                N = master.node_id
                control_cmd_id = 0x502 + 4 * N
                run_byte = 0x01 if cmd == 'enable' else 0x00
                data = bytearray(8)
                data[0] = run_byte
                # leave others zero
                master.publish(control_cmd_id, data)
                print('Sent A control run=%s' % run_byte)
                continue
            if cmd == 'mode' and len(parts) >= 2:
                mode_map = {'mit':0x00, 'vel':0x01, 'pos':0x02}
                m = parts[1].lower()
                if m not in mode_map:
                    print('Unknown mode. Valid: mit,pos,vel')
                    continue
                N = master.node_id
                control_cmd_id = 0x502 + 4 * N
                data = bytearray(8)
                data[2] = mode_map[m]
                # ensure stop before switching per spec (we simply send stop then set mode)
                data[0] = 0x00
                master.publish(control_cmd_id, data)
                # then set mode while stopped
                master.publish(control_cmd_id, data)
                print('Sent mode change to %s' % m)
                continue
            if cmd == 'set_pos' and len(parts) >= 2:
                try:
                    val = int(parts[1], 0)   # base 0: accept 0x/0o/dec
                except ValueError:
                    print('Invalid integer')
                    continue
                N = master.node_id
                mit_ctrl_id = 0x501 + 4 * N
                data = bytearray(12)
                # protocol A encodes position with 0x80000000 offset (manual 3.1);
                # unset fields hold their "zero" codes (vel/Iff = 0x8000, Kp/Kd = 0)
                struct.pack_into('<I', data, 0, (val + 0x80000000) & 0xFFFFFFFF)
                struct.pack_into('<H', data, 4, 0x8000)   # vel = 0
                struct.pack_into('<H', data, 6, 0)        # Kp = 0
                struct.pack_into('<H', data, 8, 0)        # Kd = 0
                struct.pack_into('<H', data, 10, 0x8000)  # Iff = 0
                master.publish(mit_ctrl_id, data)
                print('Sent MIT control pos_code=%d' % val)
                continue
            if cmd == 'set_vel' and len(parts) >= 2:
                try:
                    val = int(parts[1], 0)   # base 0: accept 0x/0o/dec
                except ValueError:
                    print('Invalid integer')
                    continue
                N = master.node_id
                mit_ctrl_id = 0x501 + 4 * N
                data = bytearray(12)
                # velocity with 0x8000 offset; position/Iff zero codes
                struct.pack_into('<I', data, 0, 0x80000000)   # pos = 0
                struct.pack_into('<H', data, 4, (val + 0x8000) & 0xFFFF)
                struct.pack_into('<H', data, 6, 0)            # Kp = 0
                struct.pack_into('<H', data, 8, 0)            # Kd = 0
                struct.pack_into('<H', data, 10, 0x8000)      # Iff = 0
                master.publish(mit_ctrl_id, data)
                print('Sent MIT control vel_code=%d' % val)
                continue
            if cmd == 'set_mit' and len(parts) >= 6:
                # A1 frame full MIT control (raw codes, manual 3.1):
                #   set_mit <pos_code> <vel_code> <kp_raw> <kd_raw> <iff_code>
                #   Kp = kp_raw*0.00001 (e.g. 0.008 -> 800)
                #   Kd = kd_raw*0.01     (e.g. 1     -> 100)
                try:
                    p = int(parts[1], 0); v = int(parts[2], 0)
                    kp = int(parts[3], 0); kd = int(parts[4], 0); iff = int(parts[5], 0)
                except ValueError:
                    print('Invalid integer')
                    continue
                N = master.node_id
                mit_ctrl_id = 0x501 + 4 * N
                data = bytearray(12)
                struct.pack_into('<I', data, 0, (p + 0x80000000) & 0xFFFFFFFF)
                struct.pack_into('<H', data, 4, (v + 0x8000) & 0xFFFF)
                struct.pack_into('<H', data, 6, kp & 0xFFFF)
                struct.pack_into('<H', data, 8, kd & 0xFFFF)
                struct.pack_into('<H', data, 10, (iff + 0x8000) & 0xFFFF)
                master.publish(mit_ctrl_id, data)
                print('Sent MIT pos=%d vel=%d Kp_raw=%d Kd_raw=%d Iff=%d' % (p, v, kp, kd, iff))
                continue
        # Protocol B commands
        if master.protocol == 'B':
            if cmd == 'enable' or cmd == 'disable':
                data = bytearray(8)
                data[0] = 0x01 if cmd == 'enable' else 0x00
                master.publish(0x00, data)
                print('Sent B mode switch enabled=%s' % (cmd == 'enable'))
                continue
            if cmd == 'rp_mit' and len(parts) >= 7:
                try:
                    idx = int(parts[1])
                    pos16 = int(parts[2], 0) & 0xFFFF
                    vel12 = int(parts[3], 0) & 0xFFF
                    kp = int(parts[4], 0) & 0xFFF
                    kd = int(parts[5], 0) & 0xFFF
                    torque12 = int(parts[6], 0) & 0xFFF
                except ValueError:
                    print('Invalid numeric')
                    continue
                if not (1 <= idx <= 8):
                    print('joint index must be 1..8')
                    continue
                # Build 64-byte broadcast frame (0x80): 8 joints x 8 bytes.
                # Fill ALL joints with the doc zero-value segment (0 pos/vel/tor,
                # 0 kp/kd -> 7F FF 7F F0 00 00 07 FF) so other joints are not
                # commanded to dangerous values (pos 0x0000 = -10rad).
                data = bytearray(64)
                zero_seg = bytes((0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00, 0x07, 0xFF))
                for j in range(8):
                    data[j*8:(j+1)*8] = zero_seg
                # Packing per SDK test_mit_mode.cpp / protocol doc 4.1.4:
                #   pos(16) data[0:1]  vel(12) data[2:3]  kp(12) data[3:4]
                #   kd(12) data[5:6]   torque(12) data[6:7]
                base = (idx-1) * 8
                data[base + 0] = (pos16 >> 8) & 0xFF
                data[base + 1] = pos16 & 0xFF
                data[base + 2] = (vel12 >> 4) & 0xFF
                data[base + 3] = ((vel12 & 0xF) << 4) | ((kp >> 8) & 0x0F)
                data[base + 4] = kp & 0xFF
                data[base + 5] = (kd >> 4) & 0xFF
                data[base + 6] = ((kd & 0xF) << 4) | ((torque12 >> 8) & 0x0F)
                data[base + 7] = torque12 & 0xFF
                master.publish(0x80, data)
                print('Sent RP MIT broadcast for joint %d' % idx)
                continue
            if cmd == 'rp_pp' and len(parts) >= 2:
                # PP single-node position control (doc 4.1.2.2): 0x80+N, 4B float
                try:
                    pos_f = float(parts[1])
                except ValueError:
                    print('Invalid float')
                    continue
                master.publish(0x80 + master.node_id, struct.pack('<f', pos_f))
                print('Sent RP PP pos=%.3frad' % pos_f)
                continue
            if cmd == 'rp_pv' and len(parts) >= 2:
                # PV single-node velocity control (doc 4.1.2.3): 0x80+N, 4B float
                try:
                    vel_f = float(parts[1])
                except ValueError:
                    print('Invalid float')
                    continue
                master.publish(0x80 + master.node_id, struct.pack('<f', vel_f))
                print('Sent RP PV vel=%.3frad/s' % vel_f)
                continue
            if cmd == 'rp_reg' and len(parts) >= 3:
                op = parts[1].lower()
                # 29-bit extended id layout (protocol doc, section 2):
                #   bit28=rw, bit24-27=sub, bit16-23=reg, bit15=success,
                #   bit8-14=product, bit0-6=device
                def _mk_id(rw, addr, sub=0x0):
                    return ((rw & 0x1) << 28) | ((sub & 0xF) << 24) \
                        | ((addr & 0xFF) << 16) | (0 << 15) \
                        | (0x1 << 8) | (master.node_id & 0x7F)
                if op == 'read' and len(parts) in (3, 4):
                    addr = int(parts[2], 0)
                    sub = int(parts[3], 0) & 0xF if len(parts) == 4 else 0x0
                    id29 = _mk_id(0, addr, sub)
                    master.publish(id29, b'', is_extended=True)
                    master._pending_reply[id29] = time.time()
                    print('Sent RP reg read id=0x%08X reg=0x%02X sub=0x%X' % (id29, addr & 0xFF, sub))
                    continue
                if op == 'write' and len(parts) in (4, 5):
                    addr = int(parts[2], 0)
                    sub = int(parts[3], 0) & 0xF if len(parts) == 5 else 0x0
                    val = int(parts[-1], 0)
                    id29 = _mk_id(1, addr, sub)
                    payload = int(val).to_bytes(4, byteorder='little')
                    master.publish(id29, payload, is_extended=True)
                    master._pending_reply[id29] = time.time()
                    print('Sent RP reg write id=0x%08X reg=0x%02X sub=0x%X val=%s' % (id29, addr & 0xFF, sub, val))
                    continue
        print('Unknown command or invalid for current protocol. Type help for commands.')


def start_input_thread(master):
    t = threading.Thread(target=interactive_loop, args=(master,), daemon=True)
    t.start()
    return t


def _strip_ros_args(argv):
    """Drop the trailing '--ros-args ...' block appended by ros2 launch.

    ros2 launch / ros2 run inject ``--ros-args -r __node:=<name>`` after the
    node arguments; argparse must not see them.
    """
    if '--ros-args' in argv:
        return argv[:argv.index('--ros-args')]
    return argv


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--protocol', choices=['A', 'B'], default='A')
    parser.add_argument('--id', type=int, default=1)
    parser.add_argument('--status-interval', type=float, default=1.0,
                        help='Seconds between periodic status prints (default 1.0, 0=every frame)')
    args = parser.parse_args(_strip_ros_args(sys.argv[1:]))

    rclpy.init()
    master = MotorMaster(protocol=args.protocol, node_id=args.id,
                         status_interval=args.status_interval)
    input_thread = start_input_thread(master)

    try:
        # spin until the 'quit' command is issued (interactive thread sets the flag)
        while rclpy.ok() and not master.quit_requested:
            rclpy.spin_once(master, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        master.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
