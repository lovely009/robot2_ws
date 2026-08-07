#!/usr/bin/env python3
"""
motor_slave.py

Generic motor slave simulator supporting Protocol A (25-30mm MIT-style) and
Protocol B (RP series) over CAN FD via ROS2 topics.

Publish/subscribe topics (attached to the CAN bus via the bridge):
- Subscribes to: /from_can_bus_fd (frames received from the bus)
- Publishes to: /to_can_bus_fd (replies/status written onto the bus, visible in candump)

Run:
  ros2 run can_motor_sim motor_slave --protocol A --id 1

"""

import argparse
import struct
import sys
import time
from collections import defaultdict, deque

import rclpy
from rclpy.node import Node

# Message type from ros2_socketcan
try:
    from ros2_socketcan_msgs.msg import FdFrame
except Exception:
    # Provide a fallback stub type name for environments where the message isn't installed.
    class FdFrame:
        def __init__(self):
            self.id = 0
            self.is_extended = False
            self.dlc = 0
            self.data = bytearray()


class MotorState:
    def __init__(self):
        # Protocol A model: raw encoder code (int32), velocity code, current (A)
        self.target_pos = 0
        self.pos = 0
        self.vel = 0
        self.current = 0
        self.kp = 0.0
        self.kd = 0.0
        self.mode = 0  # 0 MIT,1 vel,2 pos
        self.enabled = False
        # Protocol B physical model: rad / (rad/s) / Nm (RP doc 4.1.2.1)
        self.pos_rad = 0.0
        self.target_pos_rad = 0.0
        self.vel_cmd = 0.0
        self.vel_rad = 0.0
        self.torque_cmd = 0.0


class MotorSlave(Node):
    def __init__(self, protocol='A', node_id=1):
        super().__init__('motor_slave_%s_%d' % (protocol.lower(), node_id))
        self.protocol = protocol.upper()
        self.node_id = int(node_id)

        # Attach to the (virtual) CAN bus through the bridge:
        #  - subscribe to frames received from the bus (/from_can_bus_fd)
        #  - publish replies/periodic status to the bus (/to_can_bus_fd),
        #    so everything the slave sends is visible on vcan0 via candump.
        self.pub = self.create_publisher(FdFrame, '/to_can_bus_fd', 10)
        self.sub = self.create_subscription(FdFrame, '/from_can_bus_fd', self.on_frame, 50)

        self.state = MotorState()
        # RP registers keyed by (reg_addr, sub_addr); doc uses two-level addressing.
        self.registers = defaultdict(int)
        # Simulated analog states used for status report + fault detection.
        self.mos_temp = 30.0      # MOS temperature (deg C)
        self.coil_temp = 30.0     # coil temperature (deg C)
        self.bus_voltage = 24.0   # bus voltage (V)
        self.fault = 0            # 16-bit fault register (Pn08/0xA0 sub 0x03); bit0=enable
        self._load_rp_defaults()

        # Loopback guard: keep a short queue of recently sent frames so the
        # bridge's echo of any of them is ignored (not re-processed / logged).
        self._recent_tx = deque()  # (time, id, data)

        self._last_time = time.time()
        self.create_timer(0.02, self.periodic_status)  # 20ms

        # smoothing factor for position (low-pass)
        self.alpha = 0.05
        self.get_logger().info('Motor slave started: protocol=%s id=%s' % (self.protocol, self.node_id))

    def _f32(self, reg, sub):
        """Read a stored value back as IEEE-754 float (registers store raw bits)."""
        bits = int(self.registers.get((reg, sub), 0)) & 0xFFFFFFFF
        return struct.unpack('<f', struct.pack('<I', bits))[0]

    def _load_rp_defaults(self):
        """Load RP register default values (doc 3.1). Floats stored as raw bits."""
        fbits = lambda x: struct.unpack('<I', struct.pack('<f', x))[0]
        # Pn3 MIT thresholds (0x03)
        p3 = {0x01: -10.0, 0x02: 10.0, 0x03: -60.0, 0x04: 60.0,
              0x05: -60.0, 0x06: 60.0, 0x07: 0.0, 0x08: 500.0,
              0x09: 0.0, 0x0A: 10.0}
        for s, v in p3.items():
            self.registers[(0x03, s)] = fbits(v)
        # Pn4 Level1 protection thresholds (0x04). Defaults chosen above the
        # simulated analog state (MOS 30C / bus 24V / ~0A) so the drive is
        # fault-free by default; user may lower them to trigger protection.
        p4 = {0x01: 90.0, 0x02: 75.0, 0x03: 12.0, 0x04: 9.0, 0x05: 30.0, 0x06: 28.0}
        for s, v in p4.items():
            self.registers[(0x04, s)] = fbits(v)
        # Pn5 Level2 protection thresholds (0x05)
        p5 = {0x01: 100.0, 0x02: 80.0, 0x03: 15.0, 0x04: 10.0, 0x05: 33.0, 0x06: 30.0}
        for s, v in p5.items():
            self.registers[(0x05, s)] = fbits(v)

    def _update_fault(self):
        """Evaluate Pn4/Pn5 protection thresholds against simulated analog state.

        Fault bits (doc appendix): bit1=1级过压 bit2=2级过压 bit3=1级过流
        bit4=2级过流 bit5=MOS1级过温 bit6=MOS2级过温. Level2 faults stop the
        motor; bit0 always reflects the enable state.
        """
        mos, volt = self.mos_temp, self.bus_voltage
        cur = abs(self.state.torque_cmd) if self.state.enabled else 0.0
        f = 0x0001 if self.state.enabled else 0x0000
        if mos >= self._f32(0x04, 0x01):
            f |= 0x0020                      # bit5 MOS1级过温
        if mos >= self._f32(0x05, 0x01):
            f |= 0x0040                      # bit6 MOS2级过温
        if volt >= self._f32(0x04, 0x05):
            f |= 0x0002                      # bit1 1级过压
        if volt >= self._f32(0x05, 0x05):
            f |= 0x0004                      # bit2 2级过压
        if cur >= self._f32(0x04, 0x03):
            f |= 0x0008                      # bit3 1级过流
        if cur >= self._f32(0x05, 0x03):
            f |= 0x0010                      # bit4 2级过流
        self.fault = f
        # Level2 faults actively stop the motor (doc appendix)
        if f & 0x0004 or f & 0x0010 or f & 0x0040:
            if self.state.enabled:
                self.get_logger().warning('Protocol B: Level2 fault 0x%04X -> motor stopped' % f)
            self.state.enabled = False

    # Helpers to create FdFrame message
    def make_frame(self, can_id, data_bytes, is_extended=False):
        msg = FdFrame()
        msg.id = int(can_id)
        # some ros2_socketcan versions use is_extended, some use is_extended_id; try attribute names
        if hasattr(msg, 'is_extended'):
            msg.is_extended = bool(is_extended)
        elif hasattr(msg, 'is_extended_id'):
            msg.is_extended_id = bool(is_extended)
        # different message definitions may use 'dlc' or 'len' for data length
        if hasattr(msg, 'dlc'):
            msg.dlc = len(data_bytes)
        elif hasattr(msg, 'len'):
            msg.len = len(data_bytes)
        # ensure data is a bytes/bytearray
        msg.data = bytearray(data_bytes)
        return msg

    def publish_frame(self, can_id, data_bytes, is_extended=False):
        self._recent_tx.append((time.time(), int(can_id), bytes(data_bytes)))
        if len(self._recent_tx) > 128:
            self._recent_tx.popleft()
        msg = self.make_frame(can_id, data_bytes, is_extended=is_extended)
        self.pub.publish(msg)

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
        try:
            can_id = int(msg.id)
        except Exception:
            return

        # Extract data as bytes/bytearray
        data = bytes(msg.data) if msg.data is not None else b''
        is_ext = getattr(msg, 'is_extended', False)

        # Ignore the bridge's echo of our own just-sent frames (loopback).
        if self._is_loopback(can_id, data):
            return

        # Echo every received command frame so it can be cross-checked with candump.
        self.get_logger().info('[RX:%s] id=0x%08X len=%d data=%s'
                               % ('E' if is_ext else 'S', can_id, len(data),
                                  ' '.join('%02X' % b for b in data)))

        if self.protocol == 'A':
            self.handle_protocol_a(can_id, data)
        else:
            self.handle_protocol_b(can_id, data, is_extended=is_ext)

    # ===== Protocol A handlers =====
    def handle_protocol_a(self, can_id, data: bytes):
        N = self.node_id
        mit_ctrl_id = 0x501 + 4 * N   # A1
        control_cmd_id = 0x502 + 4 * N  # A4
        mit_reply_id = 0x701 + 4 * N  # A2

        if can_id == mit_ctrl_id:
            # MIT control instruction (host -> slave), DLC 12 (manual 3.1).
            # Position offset 0x80000000, velocity/current offset 0x8000.
            if len(data) < 12:
                self.get_logger().warning('Protocol A: MIT control frame too short')
                return
            pos_code = struct.unpack_from('<I', data, 0)[0] - 0x80000000
            vel_code = struct.unpack_from('<H', data, 4)[0] - 0x8000
            kp_raw = struct.unpack_from('<H', data, 6)[0]
            kd_raw = struct.unpack_from('<H', data, 8)[0]
            ff_code = struct.unpack_from('<H', data, 10)[0] - 0x8000

            # update target and gains
            self.state.target_pos = int(pos_code)
            self.state.kp = kp_raw * 0.00001
            self.state.kd = kd_raw * 0.01
            # convert feedforward current (code -> A)
            self.state.current = ff_code * 11.0 / 4096.0

            # send immediate MIT reply (A2, manual 3.2): 0x701+4N, DLC12
            reply = bytearray(12)
            reply[0] = (0x01 + 4 * N) & 0xFF
            reply[1] = 0x05
            struct.pack_into('<I', reply, 2, (int(self.state.pos) + 0x80000000) & 0xFFFFFFFF)
            struct.pack_into('<H', reply, 6, (int(self.state.vel) + 0x8000) & 0xFFFF)
            current_code = int(self.state.current * 4096.0 / 11.0)
            struct.pack_into('<H', reply, 8, (current_code + 0x8000) & 0xFFFF)
            self.publish_frame(mit_reply_id, reply)
            self.get_logger().info('Protocol A: MIT ctrl pos_code=%d vel_code=%d Kp=%.4f Kd=%.3f Iff=%.3fA'
                                   % (pos_code, vel_code, self.state.kp, self.state.kd, self.state.current))

        elif can_id == control_cmd_id:
            # Motor control command (A4, manual 3.4), DLC 8
            if len(data) < 8:
                return
            run_byte = data[0]
            zero_byte = data[1]
            mode_byte = data[2]
            if run_byte == 0x01:
                self.state.enabled = True
            else:
                self.state.enabled = False
            if zero_byte == 0x01:
                # set current position as zero
                self.state.pos = 0
                self.state.target_pos = 0
            # only allow mode change when stopped
            if not self.state.enabled:
                self.state.mode = int(mode_byte)
            # immediate ack via MIT reply (A2)
            reply = bytearray(12)
            reply[0] = (0x01 + 4 * N) & 0xFF
            reply[1] = 0x05
            struct.pack_into('<I', reply, 2, (int(self.state.pos) + 0x80000000) & 0xFFFFFFFF)
            struct.pack_into('<H', reply, 6, (int(self.state.vel) + 0x8000) & 0xFFFF)
            struct.pack_into('<H', reply, 8, (int(self.state.current) + 0x8000) & 0xFFFF)
            self.publish_frame(mit_reply_id, reply)
            self.get_logger().info('Protocol A: ctrl cmd run=%d zero=%d mode=%d -> enabled=%s mode=%d'
                                   % (run_byte, zero_byte, mode_byte, self.state.enabled, self.state.mode))

    def periodic_status(self):
        # smooth position toward target
        # simple first-order low-pass
        delta = self.state.target_pos - self.state.pos
        self.state.pos += int(delta * self.alpha)
        # velocity estimation (code units per second approximate)
        now = time.time()
        dt = now - self._last_time if now - self._last_time > 1e-6 else 1e-6
        self.state.vel = int((delta * self.alpha) / dt)
        self._last_time = now

        if self.protocol == 'A':
            N = self.node_id
            status_id = 0x702 + 4 * N  # A3
            # Build 20-byte status frame
            data = bytearray(20)
            data[0] = (0x01 + 4 * N) & 0xFF
            data[1] = 0x05
            struct.pack_into('<I', data, 2, (int(self.state.pos) + 0x80000000) & 0xFFFFFFFF)
            # velocity (offset 0x8000)
            vel_code = max(-32768, min(32767, int(self.state.vel)))
            struct.pack_into('<H', data, 6, (vel_code + 0x8000) & 0xFFFF)
            # current code (offset 0x8000)
            current_code = int(self.state.current * 4096.0 / 11.0)
            struct.pack_into('<H', data, 8, (current_code + 0x8000) & 0xFFFF)
            # bus voltage code: (u16 - 0x8000)/10 V; simulate 24V -> 240+0x8000
            vbus_code = int(24.0 * 10) + 0x8000
            struct.pack_into('<H', data, 10, vbus_code)
            # fault register
            struct.pack_into('<I', data, 12, 0)
            data[16] = 4 if self.state.enabled else 3  # motor state (4 running, 3 stopped)
            data[17] = 2  # encoder calibrated
            data[18] = self.state.mode
            data[19] = 0x02  # motor type 25mm
            self.publish_frame(status_id, data)

        else:
            # Protocol B is ONE-QUESTION-ONE-ANSWER (doc section 2): there is
            # NO fixed periodic status report. Update the physical model and
            # fault detection here on the internal 20ms tick, but do NOT
            # publish a periodic B3 frame - replies are only sent in response
            # to control/register frames (B2/PP/PV/register handlers).
            delta_rad = self.state.target_pos_rad - self.state.pos_rad
            self.state.pos_rad += delta_rad * self.alpha
            self.state.vel_rad = delta_rad * self.alpha / dt
            # fault detection against Pn4/Pn5 protection thresholds
            self._update_fault()

    # ===== Protocol B handlers =====
    def _rp_encode_reply(self, pos_rad, vel_rad, tor, status):
        """Encode a 12-byte RP joint reply (doc 4.1.3) from physical values.

        Layout: pos(16, -10..10rad) | vel(12, -60..60rad/s) | tor(12, -60..60Nm)
                | MOS temp | status(2B) | voltage | coil temp | reserved.
        Zero-points: 0x7FFF=0rad, 0x7FF=0 rad/s, 0x7FF=0 Nm.
        Temperature/voltage come from the simulated analog state.
        """
        data = bytearray(12)
        pos16 = int(max(0, min(0xFFFF, (pos_rad + 10.0) * 65535.0 / 20.0)))
        vel_12 = int(max(0, min(0xFFF, (vel_rad + 60.0) * 4095.0 / 120.0)))
        tor_12 = int(max(0, min(0xFFF, (tor + 60.0) * 4095.0 / 120.0)))
        data[0] = (pos16 >> 8) & 0xFF
        data[1] = pos16 & 0xFF
        data[2] = (vel_12 >> 4) & 0xFF
        data[3] = ((vel_12 & 0xF) << 4) | ((tor_12 >> 8) & 0x0F)
        data[4] = tor_12 & 0xFF
        data[5] = int(max(0, min(255, self.mos_temp + 40.0)))     # MOS temp (-40..215C)
        data[6] = status & 0xFF                                    # status/fault low byte
        data[7] = (status >> 8) & 0xFF                             # status/fault high byte
        data[8] = int(max(0, min(255, self.bus_voltage)))          # bus voltage (0..255V)
        data[9] = int(max(0, min(255, self.coil_temp + 40.0)))     # coil temp (-40..215C)
        return data

    def _reg_read(self, reg, sub):
        """Read a register. Pn08(0xA0) functional sub-registers are live."""
        if reg == 0xA0 and sub == 0x01:
            return self.state.mode           # mode config (MIT/PP/PV)
        if reg == 0xA0 and sub == 0x03:
            # fault register: bit0 reflects enable state, rest from fault flags
            return (self.fault & 0xFFFE) | (0x0001 if self.state.enabled else 0x0000)
        return self.registers.get((reg, sub), 0)

    def _reg_write(self, reg, sub, val):
        """Write a register; handle Pn08(0xA0) functional sub-registers.

        Returns an optional ack byte for functional writes (0x01 success), or
        None for plain data registers.
        """
        if reg == 0xA0:
            if sub == 0x01:  # mode config: 0=MIT, 1=PP, 2=PV
                self.state.mode = val & 0xFF
                self.get_logger().info('Protocol B: mode config -> mode=%d' % self.state.mode)
                return 0x01
            if sub == 0x02:  # zero position (doc: 0x01 success / 0x00 fail)
                if val & 0x01:
                    self.state.pos_rad = 0.0
                    self.state.target_pos_rad = 0.0
                    self.get_logger().info('Protocol B: zero position set')
                    return 0x01
                return 0x00
            if sub == 0x03:  # fault clear (write 0x0000 clears all)
                self.fault &= ~(val & 0xFFFF)
                return 0x01
        self.registers[(reg, sub)] = val
        return None

    def handle_protocol_b(self, can_id, data: bytes, is_extended=False):
        # B1: mode switching (standard id 0x00)
        if not is_extended and can_id == 0x00:
            if len(data) >= 1:
                if data[0] == 0x01:
                    self.state.enabled = True
                else:
                    self.state.enabled = False
                self.get_logger().info('Protocol B: mode set enabled=%s' % self.state.enabled)
            return

        # PP / PV single-node control (doc 4.1.2.2 / 4.1.2.3): CAN ID = 0x80 + N
        #   PP -> 4-byte float target position, PV -> 4-byte float target velocity
        if not is_extended and can_id == (0x80 + self.node_id):
            if len(data) < 4:
                return
            val = struct.unpack('<f', data[:4])[0]
            if self.state.mode == 0x02:   # PV mode: velocity command
                self.state.vel_cmd = val
                self.get_logger().info('Protocol B: PV vel_cmd=%.3frad/s' % val)
            else:                          # PP / MIT: position command
                self.state.target_pos_rad = val
                self.get_logger().info('Protocol B: PP pos_cmd=%.3frad' % val)
            # one-question-one-answer: reply immediately with current state + fault
            self._update_fault()
            reply = self._rp_encode_reply(self.state.pos_rad, self.state.vel_rad,
                                          self.state.torque_cmd, self.fault)
            self.publish_frame(self.node_id & 0xFF, reply)
            return

        # B2: MIT broadcast control (standard id 0x80), dlc 64 bytes controlling 8 joints
        if not is_extended and can_id == 0x80:
            if len(data) < 64:
                self.get_logger().warning('Protocol B: broadcast MIT frame too short')
                return
            # find the segment for this device. Device IDs are 1..8 map to index 0..7
            idx = self.node_id - 1
            if not (0 <= idx < 8):
                return
            seg = data[idx*8:(idx+1)*8]
            # decode 8-byte joint command into physical units (doc 4.1.2.1)
            pos16 = (seg[0] << 8) | seg[1]
            pos_cmd = pos16 / 65535.0 * 20.0 - 10.0              # rad
            vel_tmp = ((seg[2] << 4) | (seg[3] >> 4)) & 0xFFF
            vel_cmd = (vel_tmp - 0x7FF) * (60.0 / 0x7FF)         # rad/s
            self.state.kp = (((seg[3] & 0x0F) << 8) | seg[4]) * (500.0 / 0xFFF)
            kd_tmp = ((seg[5] << 4) | (seg[6] >> 4)) & 0xFFF
            self.state.kd = kd_tmp * (10.0 / 0xFFF)
            torque_tmp = (((seg[6] & 0x0F) << 8) | seg[7]) & 0xFFF
            tor_cmd = (torque_tmp - 0x7FF) * (60.0 / 0x7FF)      # Nm
            self.state.target_pos_rad = pos_cmd
            self.state.vel_cmd = vel_cmd
            self.state.torque_cmd = tor_cmd
            self.get_logger().info('Protocol B: MIT ctrl joint=%d pos_cmd=%+.3frad '
                                   'vel_cmd=%+.1frad/s Kp=%.2f Kd=%.2f tor_cmd=%+.2fNm'
                                   % (idx + 1, pos_cmd, vel_cmd,
                                      self.state.kp, self.state.kd, tor_cmd))
            # one-question-one-answer: reply immediately with current state + fault
            self._update_fault()
            reply = self._rp_encode_reply(self.state.pos_rad, self.state.vel_rad,
                                          self.state.torque_cmd, self.fault)
            self.publish_frame(self.node_id & 0xFF, reply)
            return

        # B4: register read/write (extended 29-bit IDs)
        # Layout (doc section 2): bit28=rw, bit24-27=sub, bit16-23=reg,
        #   bit15=success, bit8-14=product, bit0-6=device
        if is_extended:
            id29 = can_id & 0x1FFFFFFF
            op = (id29 >> 28) & 0x1
            sub = (id29 >> 24) & 0xF
            reg_addr = (id29 >> 16) & 0xFF
            succ = (id29 >> 15) & 0x1
            product_id = (id29 >> 8) & 0x7F
            dev_id = id29 & 0x7F
            # only respond if device id matches
            if dev_id != self.node_id:
                return
            if op == 0:
                # read: reply with same id (extended), little-endian value
                val = self._reg_read(reg_addr, sub)
                resp = struct.pack('<I', int(val))
                self.publish_frame(can_id, resp, is_extended=True)
                self.get_logger().info('Protocol B: reg read reg=0x%02X sub=0x%X val=%s' % (reg_addr, sub, val))
            else:
                # write: data contains little-endian payload
                if len(data) == 0:
                    return
                val = int.from_bytes(data, byteorder='little', signed=False)
                ack_data = self._reg_write(reg_addr, sub, val)
                # acknowledge: zero-length echo, or functional ack byte (e.g. zero-pos success)
                ack = bytes([ack_data]) if ack_data is not None else b''
                self.publish_frame(can_id, ack, is_extended=True)
                self.get_logger().info('Protocol B: reg write reg=0x%02X sub=0x%X val=%s' % (reg_addr, sub, val))
            return

        # B3: device standard reply with id == device id (1..8)
        if not is_extended and (can_id & 0xFF) == self.node_id:
            # Inbound frames with our own reply id are ignored (no loop-back).
            return


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
    parser.add_argument('--id', type=int, default=1, help='Device/controller id (A: 1..60, B:1..8)')
    args = parser.parse_args(_strip_ros_args(sys.argv[1:]))

    rclpy.init()
    node = MotorSlave(protocol=args.protocol, node_id=args.id)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
