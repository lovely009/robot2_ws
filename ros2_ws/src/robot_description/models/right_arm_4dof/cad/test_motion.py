#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
四关节动作库连续循环播放 + 柔顺阻抗 + MuJoCo重力补偿（重新打包V2）

目标：
  - 动作按JSON中的绝对时间播放，不因人手掰动而暂停。
  - 支持单次播放或循环播放，循环时按设定时间平滑衔接末帧与首帧。
  - 手动偏离目标时，位置恢复力矩到达上限后保持恒定。
  - 松手后追踪当前时刻的动作位置，不补放掰动期间的旧帧。
  - CAN-FD+BRS / SocketCAN can0 / 电机ID 1~4 / MIT仅下发tff。

动作文件：默认 ./motion.json（也兼容原始中文文件名）
MJCF：默认 ./urdf/4.4.mjcf.xml

准备：pip install mujoco python-can pillow
中文字体：sudo apt install fonts-noto-cjk

运行：
  python3 robot_motion_loop_v2.py --self-test
  python3 robot_motion_loop_v2.py --check-trajectory
  python3 robot_motion_loop_v2.py --check-model
  python3 robot_motion_loop_v2.py --demo
  python3 robot_motion_loop_v2.py

注意：
  1. JSON的position单位是弧度，velocity单位是弧度/秒；torque字段全为0，本版不使用。
  2. 纯外环力矩控制：MIT P=V=Kp=Kd=0，全部计算结果放在MIT tff。
  3. qfrc_bias(q,0)用于四关节联合重力补偿，不包含惯性前馈M、C。
  4. 力矩上限为软件指令限幅，不是可靠的接触力或驱动器硬件限矩。
  5. 上电MIT 0xCC也可能释放抱闸；必须支撑机械臂，并保持硬件急停可用。
"""

import argparse
import bisect
import json
import math
import os
import subprocess
import threading
import time
import tkinter as tk
import xml.etree.ElementTree as ET
from collections import deque
from tkinter import filedialog, ttk

# ----------------------------- 1. 基础设置 -----------------------------
CAN_CHANNEL = 'can0'
MODEL_PATH = './urdf/4.4.mjcf.xml'
MOTION_PATH = './motion.json'
NODE_IDS = [1, 2, 3, 4]
JOINT_NAMES = [f'Joint_link{i}' for i in NODE_IDS]
JOINT_LIMITS = [(-90, 90), (-180, 180), (-135, 135), (-180, 180)]

# 你的驱动器MIT tff物理量程：J1=58lite，J2~4=48lite。
T_MAX = [50.0, 12.0, 12.0, 12.0]

# 刚刚设置的零点若正好对应MJCF零位，以下保持零即可。
# 方向必须与你已经做过的重力补偿实验一致。
ZERO_DEG = [0.0, 0.0, 0.0, 0.0]
ANGLE_SIGN = [1, 1, 1, 1]
TORQUE_SIGN = [1, 1, 1, 1]

CONTROL_DT = 0.005          # 5ms / 200Hz，Linux+Python并非硬实时
FEEDBACK_TIMEOUT = 0.20    # 任何一路反馈长期过期则请求停止
FF_STEP = [0.06, 0.03, 0.03, 0.03]  # 每轮指令最大变化，N.m
PREPARE_TOL_DEG = 3.0       # 四关节都接近动作首帧3°范围后进入正式播放
PREPARE_SPEED_DEG_S = 8.0
PREPARE_STABLE_S = 0.30
LOOP_BRIDGE_S = 1.5       # 默认首尾衔接时间(s)，可在上位机修改

# K:外环刚度(Nm/rad); cap:回弹项限幅; D:速度误差阻尼(Nm.s/rad)
# gravity:重力补偿倍率(0.7=70%); total:最终合成tff软件限幅(Nm)
# 初始值只是调参起点，需要结合各关节电机、实机摩擦和受力调节。
DEFAULT_SETTINGS = [
    dict(k=15.0, cap=1.5, d=0.30, gravity=0.0, total=5.0),
    dict(k=15.0, cap=1.2, d=0.25, gravity=0.0, total=3.0),
    dict(k=20.0, cap=2.5, d=0.35, gravity=0.0, total=3.5),
    dict(k=12.0, cap=1.0, d=0.20, gravity=0.0, total=2.5),
]


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def encode(value, low, high, bits):
    return int(math.floor((clamp(value, low, high) - low) *
                          ((1 << bits) - 1) / (high - low) + 0.5 + 1e-9))


def decode(raw, low, high, bits):
    return low + raw * (high - low) / ((1 << bits) - 1)


# ----------------------------- 2. 动作库 -------------------------------
class MotionLibrary:
    """时间轴上的4轴位置/速度（弧度/弧度每秒）；使用三次Hermite插值。"""

    def __init__(self, path, times, positions, velocities):
        self.path = path
        self.times = times
        self.positions = positions
        self.velocities = velocities
        self.duration = times[-1]
        self.count = len(times)

    @classmethod
    def load(cls, path):
        with open(path, 'r', encoding='utf-8-sig') as f:
            doc = json.load(f)
        points = doc['points']
        if doc.get('target_count') != 4 or set(doc.get('target_ids', [])) != set(NODE_IDS):
            raise ValueError('动作库target_ids必须是1、2、3、4')
        if doc.get('count') != len(points):
            raise ValueError('动作库count与points数量不一致')
        frames = {}
        for item in points:
            stamp = int(item['time_from_start_ms'])
            motor = int(item['motor_id'])
            if motor not in NODE_IDS or stamp < 0:
                raise ValueError('动作库包含非法电机ID或负时间')
            frame = frames.setdefault(stamp, {})
            if motor in frame:
                raise ValueError(f'{stamp}ms重复出现ID{motor}')
            q = float(item['position'])
            v = float(item['velocity'])
            if not (math.isfinite(q) and math.isfinite(v)):
                raise ValueError('动作库含NaN或无穷数值')
            frame[motor] = (q, v)
        stamps = sorted(frames)
        if len(stamps) < 2 or stamps[0] != 0:
            raise ValueError('动作库至少需要2个时间点，且首帧时间为0ms')
        positions, velocities = [], []
        for stamp in stamps:
            frame = frames[stamp]
            if set(frame) != set(NODE_IDS):
                raise ValueError(f'{stamp}ms不是完整的四轴帧')
            q = [frame[i][0] for i in NODE_IDS]
            v = [frame[i][1] for i in NODE_IDS]
            for i in range(4):
                deg = math.degrees(q[i])
                lo, hi = JOINT_LIMITS[i]
                if not lo - 1e-4 <= deg <= hi + 1e-4:
                    raise ValueError(f'{stamp}ms: J{i+1}目标角度{deg:.2f}°超过模型关节范围')
            positions.append(q)
            velocities.append(v)
        return cls(os.path.abspath(path), [t / 1000.0 for t in stamps],
                   positions, velocities)

    def sample(self, t):
        """输入播放时间(s)，输出四关节目标位置(rad)、目标速度(rad/s)。"""
        if t <= 0.0:
            return self.positions[0].copy(), self.velocities[0].copy()
        if t >= self.duration:
            return self.positions[-1].copy(), [0.0] * 4

        j = bisect.bisect_right(self.times, t) - 1
        t0, t1 = self.times[j], self.times[j + 1]
        h = t1 - t0
        s = (t - t0) / h
        s2, s3 = s * s, s * s * s
        # p(s) = h00*p0 + h10*h*v0 + h01*p1 + h11*h*v1
        h00 = 2*s3 - 3*s2 + 1
        h10 = s3 - 2*s2 + s
        h01 = -2*s3 + 3*s2
        h11 = s3 - s2
        # 对t求导后得到速度（注意对s求导再除以h）。
        d00 = 6*s2 - 6*s
        d10 = 3*s2 - 4*s + 1
        d01 = -6*s2 + 6*s
        d11 = 3*s2 - 2*s
        qd, vd = [], []
        for i in range(4):
            p0 = self.positions[j][i]
            p1 = self.positions[j + 1][i]
            v0 = self.velocities[j][i]
            v1 = self.velocities[j + 1][i]
            qd.append(h00*p0 + h10*h*v0 + h01*p1 + h11*h*v1)
            vd.append((d00*p0 + d10*h*v0 + d01*p1 + d11*h*v1) / h)
        return qd, vd

    def sample_bridge(self, t, bridge_s):
        """末帧→首帧的三次Hermite衔接；两端角度、速度连续。"""
        if bridge_s <= 0:
            raise ValueError('首尾衔接时间必须大于0')
        s = clamp(t / bridge_s, 0.0, 1.0)
        s2, s3 = s * s, s * s * s
        h00, h10 = 2*s3 - 3*s2 + 1, s3 - 2*s2 + s
        h01, h11 = -2*s3 + 3*s2, s3 - s2
        d00, d10 = 6*s2 - 6*s, 3*s2 - 4*s + 1
        d01, d11 = -6*s2 + 6*s, 3*s2 - 2*s
        qd, vd = [], []
        for i in range(4):
            p0, p1 = self.positions[-1][i], self.positions[0][i]
            v0, v1 = self.velocities[-1][i], self.velocities[0][i]
            qd.append(h00*p0 + h10*bridge_s*v0 + h01*p1 + h11*bridge_s*v1)
            vd.append((d00*p0 + d10*bridge_s*v0 + d01*p1 + d11*bridge_s*v1)
                      / bridge_s)
        return qd, vd


def locate_path(configured, also=None):
    base = os.path.dirname(os.path.abspath(__file__))
    candidates = [configured, os.path.join(base, configured)]
    if also:
        candidates.extend(also)
    for path in candidates:
        if path and os.path.isfile(path):
            return os.path.abspath(path)
    raise FileNotFoundError('找不到文件：' + configured)


def load_motion(configured=MOTION_PATH):
    # ZIP工程使用ASCII文件名motion.json；单独下载本脚本时，
    # 也可直接使用用户原目录下的“原始0.5倍速3号反.json”。
    if configured == MOTION_PATH:
        original = '原始0.5倍速3号反.json'
        basedir = os.path.dirname(os.path.abspath(__file__))
        return MotionLibrary.load(locate_path(configured, [
            './' + original,
            os.path.join(basedir, original),
        ]))
    return MotionLibrary.load(locate_path(configured))


# --------------------------- 3. MuJoCo重力 -----------------------------
def load_model():
    import mujoco
    path = locate_path(MODEL_PATH, ['./4.4.mjcf.xml',
                                    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                 '4.4.mjcf.xml')])
    try:
        model = mujoco.MjModel.from_xml_path(path)
    except Exception as exc:
        # 某些场景只带MJCF没带STL：删除几何体后加载，保留完整惯性和关节。
        if not any(token in str(exc).lower() for token in
                   ('mesh', 'stl', 'resource', 'asset', 'no such file')):
            raise
        root = ET.parse(path).getroot()
        for parent in root.iter():
            for child in list(parent):
                if child.tag in ('geom', 'asset'):
                    parent.remove(child)
        model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
        print('未找到STL网格，已用MJCF中保留的质量、质心、惯性数据进行动力学计算。')
    if model.nq != 4 or model.nv != 4:
        raise ValueError(f'MJCF需要4个活动关节，得到nq={model.nq}、nv={model.nv}')
    for i, name in enumerate(JOINT_NAMES):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0 or model.jnt_qposadr[jid] != i or model.jnt_dofadr[jid] != i:
            raise ValueError('MJCF关节顺序不匹配：' + name)
    return mujoco, model, mujoco.MjData(model)


def gravity_torques(mujoco, model, data, q_deg):
    data.qpos[:] = [math.radians(a) for a in q_deg]
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    return [float(a) for a in data.qfrc_bias]


# -------------------------- 4. SocketCAN MIT --------------------------
def mit_payload(tff_nm, tmax, enable=True):
    """MIT协议12字节；P、V对应0物理量，Kp/Kd原始码0，只有tff变化。"""
    p = encode(0.0, -3.14, 3.14, 16)
    v = encode(0.0, -3.14, 3.14, 12)
    t = encode(tff_nm, -tmax, tmax, 12)
    payload = bytes([
        0xCC if enable else 0x0C,
        p >> 8, p & 0xff,
        v >> 4, (v & 15) << 4, 0,
        0, (t >> 8) & 15, t & 255,
        0, 0, 0,
    ])
    return payload, t, decode(t, -tmax, tmax, 12)


def send_mit(bus, can, i, torque, enable=True):
    packet, raw, actual = mit_payload(torque, T_MAX[i], enable)
    bus.send(can.Message(arbitration_id=0x110 | NODE_IDS[i],
                         is_extended_id=False, is_fd=True,
                         bitrate_switch=True, data=packet, check=True),
             timeout=0.04)
    return raw, actual


def parse_feedback(can_id, payload, stamp=None):
    """兼容0x301~0x304及0x001~0x004的12B角度/转速/Iq回复。"""
    if len(payload) != 12:
        return None
    for i, node in enumerate(NODE_IDS):
        if can_id in (0x300 | node, node):
            t = time.monotonic() if stamp is None else stamp
            angle = int.from_bytes(payload[0:2], 'big', signed=True) * 180.0 / 32768.0
            rpm = int.from_bytes(payload[2:4], 'big', signed=True)
            iq = int.from_bytes(payload[4:6], 'big', signed=True)
            fault = int.from_bytes(payload[6:8], 'big', signed=False)
            return i, (t, angle, rpm, iq, fault, payload[11])
    return None


# --------------------------- 5. 限幅阻抗 -------------------------------
def calculate_impedance(q_deg, dq_deg_s, targets_rad, velocities_rad_s,
                        parameters, gravity_nm):
    """全程使用外环：重力项+限幅恢复项+目标速度阻尼，经总力矩限幅后下发。"""
    rows = []
    for i in range(4):
        p = parameters[i]
        target_deg = math.degrees(targets_rad[i])
        target_vel_deg_s = math.degrees(velocities_rad_s[i])
        error_deg = target_deg - q_deg[i]
        restore = clamp(p['k'] * math.radians(error_deg), -p['cap'], p['cap'])
        damp = p['d'] * (velocities_rad_s[i] - math.radians(dq_deg_s[i]))
        gravity_ff = p['gravity'] * gravity_nm[i]
        desired = TORQUE_SIGN[i] * (gravity_ff + restore + damp)
        actual = clamp(desired, -p['total'], p['total'])
        rows.append(dict(
            angle=q_deg[i], target=target_deg, error=error_deg,
            speed=dq_deg_s[i], target_speed=target_vel_deg_s,
            gravity=gravity_nm[i], gravity_ff=gravity_ff,
            restore=restore, damping=damp,
            desired_tff=desired, capped_tff=actual,
            limited=(abs(desired - actual) > 1e-8),
        ))
    return rows


# ------------------------- 6. 播放时钟状态机 --------------------------
class PlaybackClock:
    """单次/循环时间轴；与外部掰动无关，循环时末首帧位置和速度连续。"""

    def __init__(self, motion):
        self.motion = motion
        self.phase = '等待反馈'
        self.hold_q = [0.0]*4
        self.origin = 0.0            # 此轮播放开始时的monotonic时间
        self.playhead = 0.0          # 从本次播放开始累计的时间(s)，暂停时冻结
        self.elapsed = 0.0           # 当前循环内的时间(s)，含衔接段
        self.loop = False
        self.bridge_s = LOOP_BRIDGE_S
        self.cycle_duration = motion.duration
        self.cycle_index = 0         # 已完成循环次数，从0开始
        self.segment = '未播放'       # 动作播放 / 首尾衔接
        self.progress = 0.0
        self.near_since = None
        self.max_err = [0.0]*4
        self.sum_err2 = [0.0]*4
        self.samples = 0

    def first_feedback(self, q_deg):
        if self.phase == '等待反馈':
            self.hold_q = q_deg.copy()
            self.phase = '保持当前位置'

    def command(self, operation, now, q_deg):
        # PLAY命令可以是 'PLAY' 或 ('PLAY', 是否循环, 衔接秒数)。
        if isinstance(operation, tuple):
            name = operation[0]
        else:
            name = operation

        if name == 'PLAY':
            if isinstance(operation, tuple):
                _, loop_enabled, bridge_s = operation
                self.loop = bool(loop_enabled)
                self.bridge_s = float(bridge_s)
            self.cycle_duration = self.motion.duration + (self.bridge_s if self.loop else 0.0)
            self.phase = '准备起始姿态'
            self.near_since = None
            self.playhead = 0.0
            self.elapsed = 0.0
            self.cycle_index = 0
            self.progress = 0.0
            self.segment = '回到首帧'
            self.max_err = [0.0]*4
            self.sum_err2 = [0.0]*4
            self.samples = 0
        elif name == 'PAUSE' and self.phase == '播放中':
            self.playhead = max(0.0, now - self.origin)
            if not self.loop:
                self.playhead = min(self.playhead, self.motion.duration)
            self.phase = '已暂停'
        elif name == 'RESUME' and self.phase == '已暂停':
            self.origin = now - self.playhead
            self.phase = '播放中'
        elif name == 'HOLD':
            self.hold_q = q_deg.copy()
            self.phase = '保持当前位置'
            self.playhead = 0.0
            self.elapsed = 0.0
            self.cycle_index = 0
            self.segment = '保持'
            self.progress = 0.0
            self.near_since = None

    def target_at(self, playhead):
        """根据累计播放时间确定当前循环/衔接位置；从不根据实际关节角度回退。"""
        self.playhead = max(0.0, playhead)
        if self.loop:
            self.cycle_index = int(self.playhead // self.cycle_duration)
            self.elapsed = self.playhead % self.cycle_duration
            self.progress = 100.0*self.elapsed/self.cycle_duration
            if self.elapsed < self.motion.duration:
                self.segment = '动作播放'
                return self.motion.sample(self.elapsed)
            self.segment = '首尾平滑衔接'
            return self.motion.sample_bridge(self.elapsed-self.motion.duration, self.bridge_s)

        self.elapsed = min(self.playhead, self.motion.duration)
        self.progress = 100.0*self.elapsed/self.motion.duration
        self.cycle_index = 0
        self.segment = '动作播放' if self.elapsed < self.motion.duration else '保持末帧'
        if self.playhead >= self.motion.duration:
            return self.motion.positions[-1].copy(), [0.0]*4
        return self.motion.sample(self.elapsed)

    def sample(self, now, q_deg, dq_deg_s):
        if self.phase == '准备起始姿态':
            target = [math.degrees(v) for v in self.motion.positions[0]]
            in_place = all(abs(q_deg[i]-target[i]) <= PREPARE_TOL_DEG
                           and abs(dq_deg_s[i]) <= PREPARE_SPEED_DEG_S
                           for i in range(4))
            if in_place:
                if self.near_since is None:
                    self.near_since = now
                elif now-self.near_since >= PREPARE_STABLE_S:
                    self.origin = now
                    self.playhead = 0.0
                    self.elapsed = 0.0
                    self.phase = '播放中'
            else:
                self.near_since = None
            if self.phase == '准备起始姿态':
                return self.motion.positions[0].copy(), [0.0]*4

        if self.phase == '播放中':
            qd, vd = self.target_at(now-self.origin)
            if not self.loop and self.playhead >= self.motion.duration:
                self.phase = '播放完成（保持末帧）'
            return qd, vd
        if self.phase == '已暂停':
            qd, _ = self.target_at(self.playhead)
            return qd, [0.0]*4
        if self.phase == '播放完成（保持末帧）':
            return self.motion.positions[-1].copy(), [0.0]*4
        return [math.radians(x) for x in self.hold_q], [0.0]*4

    def add_errors(self, rows):
        if self.phase != '播放中':
            return
        self.samples += 1
        for i, r in enumerate(rows):
            a = abs(r['error'])
            self.max_err[i] = max(self.max_err[i], a)
            self.sum_err2[i] += a*a

    def rms(self):
        return [math.sqrt(a/self.samples) if self.samples else 0.0
                for a in self.sum_err2]


# ------------------------ 7. 实机控制与反馈线程 -----------------------
lock = threading.Lock()
stop_event = threading.Event()
commands = deque()
worker = None
settings = [p.copy() for p in DEFAULT_SETTINGS]
feedback = [None]*4
feedback_speed = [0.0]*4
rx_stamps = [deque(maxlen=200) for _ in range(4)]
rx_count = [0]*4
history = deque(maxlen=4000)
state = dict(phase='未启动', error='', rows=[{} for _ in range(4)],
             tx_count=0, elapsed=0.0, duration=0.0, peak=[0.0]*4, rms=[0.0]*4,
             filename='', progress=0.0, cycle=1, segment='未播放', loop=False, total_play_s=0.0)


def receive_loop(bus):
    """独立接收线程；每次MIT指令触发的反馈都独立计数。"""
    try:
        while not stop_event.is_set():
            msg = bus.recv(timeout=0.03)
            if msg is None or msg.is_extended_id or msg.is_error_frame:
                continue
            item = parse_feedback(msg.arbitration_id, bytes(msg.data))
            if item is None:
                continue
            i, fb = item
            with lock:
                old = feedback[i]
                if old:
                    dt = fb[0] - old[0]
                    if 0.0005 <= dt <= 0.15:
                        delta = (fb[1] - old[1] + 180.0) % 360.0 - 180.0
                        measured = delta / dt
                        feedback_speed[i] = 0.78*feedback_speed[i] + 0.22*measured
                    else:
                        feedback_speed[i] = 0.0
                feedback[i] = fb
                rx_stamps[i].append(fb[0])
                rx_count[i] += 1
    except Exception as exc:
        with lock:
            state['error'] = 'CAN接收线程异常：' + str(exc)


def rx_hz(stamps):
    if len(stamps) < 2:
        return 0.0
    span = stamps[-1]-stamps[0]
    return (len(stamps)-1)/span if span > 0 else 0.0


def push_operation(op):
    with lock:
        commands.append(op)


def run_real(motion):
    bus = None
    rx_thread = None
    tx_count = 0
    last_ff = [0.0]*4
    try:
        import can
        mj, model, data = load_model()
        bus = can.Bus(interface='socketcan', channel=CAN_CHANNEL,
                      fd=True, receive_own_messages=False)
        rx_thread = threading.Thread(target=receive_loop, args=(bus,), daemon=True)
        rx_thread.start()
        clock = PlaybackClock(motion)
        pending_ops = []
        begin = time.monotonic()
        next_tick = begin
        while not stop_event.is_set():
            now = time.monotonic()
            with lock:
                fbs = feedback.copy()
                speeds = feedback_speed.copy()
                params = [p.copy() for p in settings]
                stamps = [list(x) for x in rx_stamps]
                ops = list(commands)
                commands.clear()
                thread_error = state['error']
            if thread_error:
                raise RuntimeError(thread_error)
            ready = all(fb is not None for fb in fbs)
            if not ready and now - begin > 3.0:
                raise TimeoutError('等待四台电机反馈超时，ID=' + ','.join(
                    str(NODE_IDS[i]) for i, fb in enumerate(fbs) if fb is None))
            if ready and any(now - fb[0] > FEEDBACK_TIMEOUT for fb in fbs):
                raise TimeoutError('反馈超时，电机ID=' + ','.join(
                    str(NODE_IDS[i]) for i, fb in enumerate(fbs)
                    if now - fb[0] > FEEDBACK_TIMEOUT))
            if not ready:
                pending_ops.extend(ops)
            rows = [{} for _ in range(4)]
            q = [0.0]*4
            if ready:
                q = [ANGLE_SIGN[i]*(fbs[i][1]-ZERO_DEG[i]) for i in range(4)]
                dq = [ANGLE_SIGN[i]*speeds[i] for i in range(4)]
                clock.first_feedback(q)
                pending_ops.extend(ops)
                for op in pending_ops:
                    clock.command(op, now, q)
                pending_ops.clear()
                qd, vd = clock.sample(now, q, dq)
                gravity = gravity_torques(mj, model, data, q)
                rows = calculate_impedance(q, dq, qd, vd, params, gravity)
                clock.add_errors(rows)
            for i in range(4):
                desired = rows[i]['capped_tff'] if ready else 0.0
                last_ff[i] += clamp(desired - last_ff[i], -FF_STEP[i], FF_STEP[i])
                t_raw, t_sent = send_mit(bus, can, i, last_ff[i])
                tx_count += 1
                if ready:
                    rows[i].update(sent=t_sent, t_raw=t_raw,
                                   iq=fbs[i][3], rpm=fbs[i][2],
                                   rx_hz=rx_hz(stamps[i]),
                                   rx_age_ms=max(0.0, (now-fbs[i][0])*1000.0),
                                   max_error=clock.max_err[i], rms=clock.rms()[i])
            with lock:
                state.update(phase=clock.phase, rows=rows, tx_count=tx_count,
                             elapsed=clock.elapsed, duration=clock.cycle_duration,
                             progress=clock.progress, cycle=clock.cycle_index+1,
                             segment=clock.segment, loop=clock.loop,
                             total_play_s=clock.playhead,
                             peak=clock.max_err.copy(), rms=clock.rms())
                if ready:
                    history.append((now, q.copy(), [r['target'] for r in rows]))
            next_tick += CONTROL_DT
            delta = next_tick - time.monotonic()
            if delta < -CONTROL_DT:
                next_tick = time.monotonic()
            else:
                stop_event.wait(max(0.0, delta))
    except Exception as exc:
        print('控制停止原因：', exc, flush=True)
        with lock:
            state['phase'] = '已停止：' + str(exc)
    finally:
        stop_event.set()
        if bus is not None:
            for i in range(4):
                try:
                    send_mit(bus, can, i, 0.0, enable=False)
                except Exception:
                    pass
            if rx_thread is not None:
                rx_thread.join(timeout=0.2)
            bus.shutdown()
        with lock:
            if not state['phase'].startswith('已停止：'):
                state['phase'] = '已停止（已请求四轴失能）'


def run_demo(motion):
    """离线演示：让示意角度滞后目标，并模拟一段手动干预；不控制电机。"""
    mj = model = data = None
    try:
        mj, model, data = load_model()
    except Exception as exc:
        print('演示模式未加载MuJoCo，示意重力项设为0：', str(exc))
    clock = PlaybackClock(motion)
    q = [math.degrees(v) for v in motion.positions[0]]
    dq = [0.0]*4
    clock.first_feedback(q)
    next_tick = time.monotonic()
    while not stop_event.is_set():
        now = time.monotonic()
        with lock:
            ops = list(commands)
            commands.clear()
            ps = [p.copy() for p in settings]
        for op in ops:
            clock.command(op, now, q)
        qd, vd = clock.sample(now, q, dq)
        target = [math.degrees(z) for z in qd]
        # 一段人为施加的示意偏差，动作时间仍持续前进。
        disturbance = 17.0 if clock.phase == '播放中' and 3.0 <= clock.elapsed <= 4.8 else 0.0
        desired_q = target.copy()
        desired_q[2] += disturbance
        for i in range(4):
            prev = q[i]
            q[i] += 0.05*(desired_q[i] - q[i])
            dq[i] = (q[i]-prev)/0.02
        gravity = gravity_torques(mj, model, data, q) if mj else [0.0]*4
        rows = calculate_impedance(q, dq, qd, vd, ps, gravity)
        clock.add_errors(rows)
        for i, row in enumerate(rows):
            raw = encode(row['capped_tff'], -T_MAX[i], T_MAX[i], 12)
            row.update(sent=decode(raw, -T_MAX[i], T_MAX[i], 12), t_raw=raw,
                       iq=0, rpm=0, rx_hz=50.0, rx_age_ms=0.0,
                       max_error=clock.max_err[i], rms=clock.rms()[i])
        with lock:
            state.update(phase='【演示】' + clock.phase, elapsed=clock.elapsed,
                         duration=clock.cycle_duration, progress=clock.progress,
                         cycle=clock.cycle_index+1, segment=clock.segment,
                         loop=clock.loop, total_play_s=clock.playhead,
                         rows=rows, peak=clock.max_err.copy(), rms=clock.rms())
            history.append((now, q.copy(), target))
        next_tick += 0.02
        stop_event.wait(max(0.0, next_tick-time.monotonic()))


# ----------------------------- 8. 中文上位机 ---------------------------
def get_chinese_font():
    """WSL/Tk可能显示\\uXXXX，使用Pillow渲染中文标签。"""
    try:
        out = subprocess.check_output(['fc-match', '-f', '%{file}\n',
                                       'Noto Sans CJK SC:lang=zh-cn'],
                                      text=True, timeout=3)
        p = out.strip().splitlines()[0]
        if os.path.isfile(p):
            return p
    except (OSError, subprocess.SubprocessError, IndexError):
        pass
    for p in ('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
              '/usr/share/fonts/truetype/wqy/wqy-microhei.ttc'):
        if os.path.isfile(p):
            return p
    raise RuntimeError('缺少中文字体：sudo apt install fonts-noto-cjk')


def gui(demo=False, initial_motion=None):
    global worker
    from PIL import Image, ImageDraw, ImageFont, ImageTk

    BG='#F1F5FA'; WHITE='#FFFFFF'; TEXT='#26374A'; BLUE='#1159B6'; GREY='#68788C'
    root = tk.Tk()
    root.title('四关节动作库柔顺播放 · MuJoCo / CAN-FD')
    root.geometry('1500x830')
    root.minsize(1130, 660)
    root.configure(bg=BG)
    font_path = get_chinese_font()
    pictures = {}
    fonts = {}

    def image_label(value, size=14, color=TEXT):
        key = (str(value), size, color)
        if key not in pictures:
            if size not in fonts:
                fonts[size] = ImageFont.truetype(font_path, size)
            font = fonts[size]
            box = font.getbbox(str(value))
            pic = Image.new('RGBA', (max(1, box[2]-box[0]+12),
                                     max(1, box[3]-box[1]+12)))
            ImageDraw.Draw(pic).text((6-box[0],6-box[1]), str(value), font=font, fill=color)
            pictures[key] = ImageTk.PhotoImage(pic, master=root)
        return pictures[key]

    def zh(parent, text, size=14, bg=WHITE, color=TEXT):
        return tk.Label(parent, image=image_label(text, size, color), bg=bg, bd=0)

    # 顶部说明
    top = tk.Frame(root, bg=BG, padx=18, pady=12)
    top.pack(fill='x')
    zh(top, '四关节动作库播放 · 限幅阻抗 · MuJoCo重力补偿', 22, bg=BG).pack(anchor='w')
    zh(top, '掰动时动作时间不暂停；松手后追赶当前时间目标。MIT仅下发tff，200Hz。',
       12, bg=BG, color=GREY).pack(anchor='w', pady=(5,0))

    # 滚动区域
    body = tk.Frame(root, bg=BG); body.pack(fill='both', expand=True)
    scroll = tk.Scrollbar(body, orient='vertical'); scroll.pack(side='right', fill='y')
    panel = tk.Canvas(body, bg=BG, highlightthickness=0, yscrollcommand=scroll.set)
    panel.pack(side='left', fill='both', expand=True)
    scroll.config(command=panel.yview)
    inner = tk.Frame(panel, bg=BG, padx=18)
    window = panel.create_window((0,0), window=inner, anchor='nw')
    inner.bind('<Configure>', lambda e: panel.config(scrollregion=panel.bbox('all')))
    panel.bind('<Configure>', lambda e: panel.itemconfigure(window,width=e.width))
    panel.bind_all('<MouseWheel>', lambda e: panel.yview_scroll(-int(e.delta/120) or (-1 if e.delta>0 else 1), 'units'))
    panel.bind_all('<Button-4>', lambda e: panel.yview_scroll(-3, 'units'))
    panel.bind_all('<Button-5>', lambda e: panel.yview_scroll(3, 'units'))

    motion_box = tk.Frame(inner, bg=WHITE, padx=14, pady=12)
    motion_box.pack(fill='x', pady=(0,10))
    zh(motion_box, '动作库与播放控制', 17, color=BLUE).pack(anchor='w')
    motion_var = tk.StringVar(value='动作文件：' + (os.path.basename(initial_motion.path) if initial_motion else '未加载'))
    tk.Label(motion_box, textvariable=motion_var, bg=WHITE, fg=TEXT,
             font=('DejaVu Sans Mono',11), anchor='w').pack(fill='x', pady=5)
    info_var = tk.StringVar(value='')
    tk.Label(motion_box, textvariable=info_var, bg=WHITE, fg=GREY,
             font=('DejaVu Sans Mono',11),anchor='w').pack(fill='x')
    progress = ttk.Progressbar(motion_box, orient='horizontal', maximum=100, mode='determinate')
    progress.pack(fill='x', pady=(8,4))

    loop_controls = tk.Frame(motion_box, bg=WHITE)
    loop_controls.pack(fill='x', pady=(8,1))
    loop_var = tk.BooleanVar(value=True)  # 默认开启连续循环，可关闭切换单次播放
    ttk.Checkbutton(loop_controls, variable=loop_var).pack(side='left', padx=(2,2))
    zh(loop_controls, '循环播放（末帧自动平滑返回首帧）', 13).pack(side='left', padx=(2,18))
    zh(loop_controls, '首尾衔接时间', 13).pack(side='left', padx=(0,7))
    bridge_var = tk.StringVar(value=f'{LOOP_BRIDGE_S:g}')
    ttk.Entry(loop_controls, width=7, justify='center', textvariable=bridge_var).pack(side='left')
    zh(loop_controls, '秒（点击“播放动作”时生效）', 12, color=GREY).pack(side='left', padx=(7,0))

    motion = initial_motion

    def choose_json():
        nonlocal motion
        if worker and worker.is_alive():
            set_ui_status('请先停止电机，才能更换动作库')
            return
        file = filedialog.askopenfilename(title='Select JSON trajectory',
                                          filetypes=[('JSON','*.json'),('All','*.*')])
        if not file:
            return
        try:
            motion = MotionLibrary.load(file)
            motion_var.set('动作文件：'+os.path.basename(motion.path))
            set_ui_status(f'动作库已加载：{motion.count}帧，{motion.duration:.3f}秒')
        except Exception as exc:
            set_ui_status('载入失败：'+str(exc))

    # 参数设置（允许运行时修改）
    param_box = tk.Frame(inner, bg=WHITE, padx=14, pady=10)
    param_box.pack(fill='x',pady=(0,10))
    zh(param_box,'四关节柔顺控制参数（重力比例0.7 = 70%）',17,color=BLUE).pack(anchor='w',pady=(0,10))
    grid = tk.Frame(param_box,bg=WHITE); grid.pack(fill='x')
    labels=['关节','刚度 K (Nm/rad)','回弹上限 (Nm)','阻尼 D (Nm·s/rad)',
            '重力比例 (0~1.3)','总 tff 上限 (Nm)']
    for c,label in enumerate(labels):
        zh(grid,label,12).grid(row=0,column=c,padx=7,pady=(0,8))
        grid.columnconfigure(c,weight=1)
    edits=[]
    for i,p in enumerate(settings):
        zh(grid,f'J{i+1} / '+('58lite' if i==0 else '48lite'),12).grid(row=i+1,column=0,padx=4,pady=4)
        entry_map={}
        for c,key in enumerate(('k','cap','d','gravity','total'),1):
            entry=ttk.Entry(grid,width=13,justify='center')
            entry.insert(0,f'{p[key]:g}')
            entry.grid(row=i+1,column=c,sticky='ew',padx=7,pady=3)
            entry_map[key]=entry
        edits.append(entry_map)

    def apply_settings():
        try:
            new=[]
            for i,widgets in enumerate(edits):
                item={k:float(w.get().strip()) for k,w in widgets.items()}
                if not all(math.isfinite(v) for v in item.values()):
                    raise ValueError('参数必须是有限数字')
                if not 0 <= item['k'] <= 100 or not 0 <= item['d'] <= 10:
                    raise ValueError(f'J{i+1}的K范围0~100、D范围0~10')
                if not 0 <= item['gravity'] <= 1.3:
                    raise ValueError(f'J{i+1}重力比例范围0~1.3')
                if not 0 <= item['cap'] <= T_MAX[i] or not 0 < item['total'] <= T_MAX[i]:
                    raise ValueError(f'J{i+1}力矩上限需要在有效范围内（T_MAX={T_MAX[i]}）')
                new.append(item)
            with lock:
                for i in range(4):
                    settings[i].update(new[i])
            set_ui_status('参数已应用')
            return True
        except Exception as exc:
            set_ui_status('参数错误：'+str(exc))
            return False

    param_buttons=tk.Frame(param_box,bg=WHITE)
    param_buttons.pack(fill='x',pady=(9,0))

    def make_button(parent,title,action):
        b=ttk.Button(parent,image=image_label(title,14,BLUE),command=action)
        b.pack(side='left',padx=(0,9),ipadx=8,ipady=4)
        return b

    make_button(param_buttons,'应用控制参数',apply_settings)
    zh(param_buttons,'回弹力矩上限与总tff上限分别控制；实际下发值受限幅影响',
       11,color=GREY).pack(side='left',padx=9)

    # 实时数据表
    table_box=tk.Frame(inner,bg=WHITE,padx=14,pady=12)
    table_box.pack(fill='x',pady=(0,10))
    zh(table_box,'实时四轴数据：目标和实际角度 / 误差 / 力矩',17,color=BLUE).pack(anchor='w',pady=(0,10))
    table=tk.Frame(table_box,bg=WHITE);table.pack(fill='x')
    column_names=['关节','当前(°)','目标(°)','误差(°)','目标速(°/s)','实际速(°/s)',
                  '重力前馈','回弹力矩','阻尼力矩','下发tff','Iq(mA)','反馈Hz','播放峰值误差']
    for c,title in enumerate(column_names):
        zh(table,title,11).grid(row=0,column=c,padx=2,pady=(0,10))
        table.columnconfigure(c,weight=1)
    vars_by_joint=[]
    for i in range(4):
        zh(table,f'J{i+1}',12,color=BLUE).grid(row=i+1,column=0,padx=2,pady=9)
        cells=[]
        for c in range(1,len(column_names)):
            v=tk.StringVar(value='--')
            tk.Label(table,textvariable=v,bg=WHITE,fg=TEXT,
                     font=('DejaVu Sans Mono',9),width=11,anchor='center').grid(
                         row=i+1,column=c,padx=0,pady=9)
            cells.append(v)
        vars_by_joint.append(cells)

    # 曲线
    chart_box=tk.Frame(inner,bg=WHITE,padx=14,pady=10)
    chart_box.pack(fill='x',pady=(0,10))
    zh(chart_box,'最近10秒关节角度：实线=实际，虚线=动作库目标',14,color=BLUE).pack(anchor='w')
    chart=tk.Canvas(chart_box,height=220,bg='#172235',highlightthickness=0)
    chart.pack(fill='x',pady=(8,0))

    # 底部固定操作栏
    footer=tk.Frame(root,bg=BG,padx=18,pady=10)
    footer.pack(side='bottom',fill='x')
    footer_left=tk.Frame(footer,bg=BG);footer_left.pack(side='left')
    footer_right=tk.Frame(footer,bg=BG);footer_right.pack(side='right')
    status_label=zh(footer_left,'未启动',13,bg=BG,color=BLUE)
    status_label.pack(anchor='w')
    count_var=tk.StringVar(value='TX 0')
    tk.Label(footer_left,textvariable=count_var,bg=BG,fg=GREY,
             font=('DejaVu Sans Mono',10)).pack(anchor='w')
    status_override=None
    override_until=0.0

    def set_ui_status(text):
        nonlocal status_override,override_until
        status_override=text
        override_until=time.monotonic()+3.0

    def start():
        global worker
        if worker and worker.is_alive():
            return
        if motion is None:
            set_ui_status('先加载JSON动作库')
            return
        if not apply_settings():
            return
        stop_event.clear()
        with lock:
            commands.clear()
            for i in range(4):
                feedback[i]=None
                feedback_speed[i]=0.0
                rx_stamps[i].clear()
                rx_count[i]=0
            history.clear()
            state.update(phase='启动中',error='',rows=[{} for _ in range(4)],
                         tx_count=0,elapsed=0.0,progress=0.0,
                         cycle=1, segment='未播放',loop=False,total_play_s=0.0,
                         peak=[0.0]*4,rms=[0.0]*4,
                         duration=motion.duration,filename=os.path.basename(motion.path))
        worker=threading.Thread(target=(run_demo if demo else run_real),
                                args=(motion,),daemon=False)
        worker.start()

    def operation(name):
        if not (worker and worker.is_alive()):
            set_ui_status('请先点击连接/启动')
            return
        if name == 'PLAY':
            try:
                seconds = float(bridge_var.get().strip())
                if not math.isfinite(seconds) or not 0.5 <= seconds <= 5.0:
                    raise ValueError('首尾衔接时间应为0.5~5秒')
                push_operation(('PLAY', loop_var.get(), seconds))
            except ValueError as exc:
                set_ui_status('参数错误：'+str(exc))
        else:
            push_operation(name)

    def stop():
        stop_event.set()

    make_button(footer_right,'停止/四轴失能',stop)
    make_button(footer_right,'停止动作并保持',lambda:operation('HOLD'))
    make_button(footer_right,'继续播放',lambda:operation('RESUME'))
    make_button(footer_right,'暂停动作',lambda:operation('PAUSE'))
    make_button(footer_right,'播放动作',lambda:operation('PLAY'))
    make_button(footer_right,'连接/启动',start)
    make_button(motion_box,'选择JSON动作库',choose_json)

    colors=['#FFBF69','#77D3FF','#92E2A5','#FF8FAA']
    last_status=None
    def refresh():
        nonlocal last_status
        with lock:
            snap=dict(state)
            rows=[r.copy() for r in state['rows']]
            points=list(history)
        current_status=snap['phase']
        if status_override and time.monotonic()<override_until:
            current_status=status_override
        if current_status!=last_status:
            status_label.config(image=image_label(current_status,13,BLUE))
            last_status=current_status
        count_var.set(f"TX {snap['tx_count']}")
        progress['value']=snap['progress']
        segment_en = ('BRIDGE' if snap['segment'] == '首尾平滑衔接' else
                      'MOTION' if snap['segment'] == '动作播放' else
                      'PREPARE' if snap['segment'] == '回到首帧' else 'HOLD')
        loop_desc = (f"LOOP #{snap['cycle']} / {segment_en}"
                     if snap['loop'] else f"ONCE / {segment_en}")
        info_var.set(f"{loop_desc}    "
                     f"{snap['elapsed']:.3f} / {snap['duration']:.3f} s    "
                     f"{snap['progress']:.1f}%    "
                     f"总播放 {snap['total_play_s']:.1f}s    "
                     f"Peak(4 axes): {', '.join(f'{v:.2f}' for v in snap['peak'])} deg")
        for i,r in enumerate(rows):
            if 'sent' not in r:
                continue
            values=[f"{r['angle']:+.2f}",f"{r['target']:+.2f}",
                    f"{r['error']:+.2f}",f"{r['target_speed']:+.1f}",
                    f"{r['speed']:+.1f}",f"{r['gravity_ff']:+.2f}",
                    f"{r['restore']:+.2f}",f"{r['damping']:+.2f}",
                    f"{r['sent']:+.2f}",str(r['iq']),f"{r['rx_hz']:.0f}",
                    f"{r['max_error']:.2f}"]
            for v,x in zip(vars_by_joint[i],values):
                v.set(x)
        chart.delete('all')
        w=max(600,chart.winfo_width());h=max(200,chart.winfo_height())
        for i in range(4):
            cy=h*(i+0.5)/4
            chart.create_line(44,cy,w-8,cy,fill='#35455B')
            chart.create_text(12,cy,text=f'J{i+1}',fill=colors[i],anchor='w')
        if len(points)>1:
            end=points[-1][0]
            valid=[p for p in points if p[0] >= end-10.0]
            valid=valid[::max(1,len(valid)//300)]
            for i in range(4):
                actual=[]; target=[]
                for t,qs,td in valid:
                    x=44+(t-(end-10.0))/10.0*(w-55)
                    cy=h*(i+0.5)/4
                    mag=JOINT_LIMITS[i][1]
                    actual.extend((x,cy-clamp(qs[i]/mag,-1,1)*h/4*0.37))
                    target.extend((x,cy-clamp(td[i]/mag,-1,1)*h/4*0.37))
                if len(actual)>=4:
                    chart.create_line(*actual,fill=colors[i],width=2)
                    chart.create_line(*target,fill=colors[i],width=1,dash=(4,4))
        root.after(100,refresh)

    def close():
        stop_event.set()
        if worker and worker.is_alive():
            root.after(100,close)
        else:
            root.destroy()
    root.protocol('WM_DELETE_WINDOW',close)
    if demo:
        root.after(250,start)
        root.after(700,lambda:operation('PLAY'))
    refresh()
    root.mainloop()


# ------------------------------- 9. 自测 -------------------------------
def self_test(motion):
    assert motion.count >= 2 and motion.duration > 0
    for i in range(4):
        pkt,raw,torque=mit_payload(0.0,T_MAX[i])
        assert len(pkt)==12 and raw==2048
        assert pkt.hex().upper()=='CC8000800000000800000000'
        assert abs(torque)<=T_MAX[i]/4095.0+1e-9
        blob=(int(30/180*32768)).to_bytes(2,'big',signed=True)+bytes(10)
        assert parse_feedback(NODE_IDS[i],blob)[0]==i
        assert parse_feedback(0x300|NODE_IDS[i],blob)[0]==i
    for t in (0.0,0.017,0.232,2.1,motion.duration/2,motion.duration):
        q,v=motion.sample(t)
        assert len(q)==4 and len(v)==4
        assert all(math.isfinite(x) for x in q+v)
    # 恰好在原采样时刻，Hermite插值必须还原位置与速度。
    for j in sorted({0, 1, motion.count//4, motion.count//2,
                     3*motion.count//4, motion.count-1}):
        t=motion.times[j]
        q,v=motion.sample(t)
        assert max(abs(a-b) for a,b in zip(q,motion.positions[j]))<1e-7
        if j < motion.count-1 and j>0:
            assert max(abs(a-b) for a,b in zip(v,motion.velocities[j]))<1e-6
    ps=[dict(k=20,cap=2.5,d=0.35,gravity=0.7,total=3.5)]*4
    rows=calculate_impedance([0]*4,[0]*4,[math.radians(60)]*4,[0]*4,ps,[0]*4)
    assert all(abs(r['restore']-2.5)<1e-9 for r in rows)
    clock=PlaybackClock(motion)
    clock.first_feedback([0]*4)
    clock.command('PLAY',5.0,[0]*4)
    clock.sample(5.0,[0]*4,[0]*4)
    clock.sample(5.4,[0]*4,[0]*4)
    assert clock.phase=='播放中'
    dt = min(1.8, motion.duration/2.0)
    a,_=clock.sample(5.4+dt,[30]*4,[0]*4)
    b,_=motion.sample(dt)
    assert max(abs(x-y) for x,y in zip(a,b))<1e-5, '被拖动不应改变播放时钟'

    # 循环模式：起止拼接位置/速度连续、播放不中断、暂停/继续保留进度。
    bridge_s=1.5
    start_q, start_v=motion.positions[0], motion.velocities[0]
    end_q, end_v=motion.positions[-1], motion.velocities[-1]
    for tau, expected_q, expected_v in ((0, end_q, end_v),
                                        (bridge_s, start_q, start_v)):
        q_b, v_b=motion.sample_bridge(tau, bridge_s)
        assert max(abs(x-y) for x,y in zip(q_b, expected_q)) < 1e-9
        assert max(abs(x-y) for x,y in zip(v_b, expected_v)) < 1e-9
    for k in range(101):
        q_b,v_b=motion.sample_bridge(bridge_s*k/100, bridge_s)
        for i in range(4):
            lo,hi=JOINT_LIMITS[i]
            assert math.radians(lo)<=q_b[i]<=math.radians(hi), '循环衔接超出关节范围'
            assert math.isfinite(q_b[i]) and math.isfinite(v_b[i])
    clock=PlaybackClock(motion)
    t0=10.0
    q0=[math.degrees(x) for x in motion.positions[0]]
    clock.first_feedback(q0)
    clock.command(('PLAY', True, bridge_s),t0,q0)
    clock.sample(t0,q0,[0]*4)
    clock.sample(t0+0.31,q0,[0]*4)
    assert clock.phase=='播放中'
    origin=clock.origin
    q_before,v_before=clock.sample(origin+motion.duration-0.000001,[40]*4,[0]*4)
    q_after,v_after=clock.sample(origin+motion.duration+0.000001,[-40]*4,[0]*4)
    assert max(abs(x-y) for x,y in zip(q_before,q_after)) < 1e-4
    assert max(abs(x-y) for x,y in zip(v_before,v_after)) < 1e-3
    assert clock.phase=='播放中' and clock.segment=='首尾平滑衔接'
    q_before,v_before=clock.sample(origin+motion.duration+bridge_s-0.000001,[40]*4,[0]*4)
    q_after,v_after=clock.sample(origin+motion.duration+bridge_s+0.000001,[-40]*4,[0]*4)
    assert max(abs(x-y) for x,y in zip(q_before,q_after)) < 1e-4
    assert max(abs(x-y) for x,y in zip(v_before,v_after)) < 1e-3
    assert clock.cycle_index==1 and clock.phase=='播放中'
    assert clock.elapsed<0.01 and clock.segment=='动作播放'
    clock.command('PAUSE',origin+motion.duration+bridge_s+0.2,q0)
    clock.sample(origin+motion.duration+bridge_s+0.2,q0,[0]*4)
    paused=clock.elapsed
    clock.sample(origin+motion.duration+bridge_s+4.2,q0,[0]*4)
    assert clock.phase=='已暂停' and abs(clock.elapsed-paused)<1e-6
    clock.command('RESUME',origin+motion.duration+bridge_s+4.2,q0)
    clock.sample(origin+motion.duration+bridge_s+4.4,q0,[0]*4)
    assert clock.phase=='播放中' and clock.elapsed>paused+0.15

    print('PASS：动作库解析、Hermite插值、MIT封包、四轴反馈、限幅回弹、首尾平滑衔接与循环时钟')
    print(f'动作库：{motion.count}个时间点，{motion.duration:.3f}秒，4个关节')


def check_model():
    mj,model,data=load_model()
    print('模型:',MODEL_PATH,'nq=',model.nq,'nv=',model.nv)
    for qs in ([0]*4,[15,-20,-40,10],[-20,15,-30,-5]):
        print('q=',qs,' g[Nm]=',
              [round(x,5) for x in gravity_torques(mj,model,data,qs)])
    print('仅计算模型，没有连接CAN')


def check_trajectory(motion):
    print('文件：',motion.path)
    print('时间点：',motion.count,'动作时长：',motion.duration,'秒')
    for i in range(4):
        poses=[math.degrees(row[i]) for row in motion.positions]
        vel=[math.degrees(row[i]) for row in motion.velocities]
        print(f'J{i+1} 起点={poses[0]:+.2f}°,末尾={poses[-1]:+.2f}°,'
              f'范围={min(poses):+.2f}~{max(poses):+.2f}°,'
              f'最大绝对速度={max(abs(x) for x in vel):.2f}°/s')
    bridge_s=LOOP_BRIDGE_S
    print(f'循环播放：每轮{motion.duration+bridge_s:.3f}秒（原动作{motion.duration:.3f}秒 + ' 
          f'末首平滑衔接{bridge_s:.3f}秒）')
    for i in range(4):
        diff=math.degrees(motion.positions[0][i]-motion.positions[-1][i])
        print(f'J{i+1} 末帧→首帧角度差：{diff:+.3f}°')
    print('默认启用连续循环；上位机取消勾选可恢复单次播放；手动拖动不暂停播放时钟')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description='四关节柔顺动作库播放')
    parser.add_argument('--self-test',action='store_true',help='纯算法自测，不连接电机')
    parser.add_argument('--check-trajectory',action='store_true',help='检查JSON内容，不连接电机')
    parser.add_argument('--check-model',action='store_true',help='检查MuJoCo重力，不连接电机')
    parser.add_argument('--demo',action='store_true',help='离线界面演示')
    parser.add_argument('--json',default=MOTION_PATH,help='动作JSON路径')
    parser.add_argument('--model',default=MODEL_PATH,help='MuJoCo MJCF模型路径')
    args=parser.parse_args()
    MODEL_PATH=args.model
    try:
        motion=load_motion(args.json)
    except (OSError,ValueError,KeyError,TypeError) as exc:
        parser.error(f'动作库加载失败：{exc}')
    if args.self_test:
        self_test(motion)
    elif args.check_trajectory:
        check_trajectory(motion)
    elif args.check_model:
        check_model()
    else:
        gui(demo=args.demo,initial_motion=motion)
