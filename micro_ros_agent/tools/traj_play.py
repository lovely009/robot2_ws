#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任意长度轨迹播放器：把 web 示教导出的 MotorTrajectory JSON 连续播到臂上。

为什么需要它：固件整条轨迹上限 = MOTOR_CONTROL_MAX_TRAJECTORY_POINTS(256) 个
路点，且**四台电机共用这一个预算**（4 台 = 64 个时刻）。而示教导出的文件动辄
600/1080/1800 点。要"随便播放"，只能在主机侧把整条轨迹切成 ≤256 点的窗口，
**在前一窗口跑完的瞬间交棒**下一个窗口——固件受理新轨迹时把 t=0 重设到受理
时刻，所以窗口边界必须是同一个点（本工具让下一窗口的首点 = 上一窗口的末点），
并且发送时刻要压在边界上，否则交棒处就是一个位置阶跃。

用法：
  python3 traj_play.py --json ../../ros2_ws/src/web_host/4bvh_keyframes.json
  python3 traj_play.py --json <file> --kp 30 --kd 1.0 --hold-s 2 --loop 2
  python3 traj_play.py --json <file> --dry-run      # 只解析+切窗，不动臂
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from can_bridge_msgs.msg import (MotorCommandArray, MotorCommandTarget,
                                 MotorMotionStateArray, MotorPolicyArray,
                                 MotorPolicyTarget, MotorTrajectory,
                                 MotorTrajectoryPoint, MotorTrajectoryState)

MAX_POINTS = 256          # 固件整条轨迹总路点上限（四台共用）
WINDOW_STEPS = 60         # 每窗口时间步数 → 4 台时 240 点，留 16 点余量


def log(*a):
    print(*a, flush=True)


def load_series(path):
    """读 MotorTrajectory 载荷 → (target_ids, [(t_ms, {mid: (pos, vel, eff)})])。"""
    with open(path, "r", encoding="utf-8") as f:
        d = json.load(f)
    ids = list(d["target_ids"])
    buckets = {}
    for p in d["points"]:
        t = int(p["time_from_start_ms"])
        eff = p.get("effort", p.get("torque", 0.0))   # web 导出写 torque
        buckets.setdefault(t, {})[int(p["motor_id"])] = (
            float(p["position"]), float(p.get("velocity", 0.0)), float(eff))
    steps = []
    for t in sorted(buckets):
        # 任一目标在某一时刻缺席 → 该时刻不完整，固件会拒；缺的补上一时刻的值
        if len(buckets[t]) != len(ids):
            prev = steps[-1][1] if steps else {i: (0.0, 0.0, 0.0) for i in ids}
            merged = dict(prev)
            merged.update(buckets[t])
            buckets[t] = merged
        steps.append((t, buckets[t]))
    return ids, steps


def make_windows(steps, max_steps=WINDOW_STEPS, overlap=True):
    """切窗：每窗最多 max_steps 个时刻；下一窗首点复用上一窗末点（同一时刻同一
    位置），保证交棒处位置连续。返回 [(窗口步列表, 窗口首时刻在整条里的偏移ms)]。"""
    wins, i, n = [], 0, len(steps)
    while i < n:
        chunk = steps[i:i + max_steps]
        if overlap and wins and i > 0:
            chunk = [steps[i - 1]] + chunk       # 边界点重复一次
        wins.append(chunk)
        i += max_steps
    return wins


class Player(Node):
    def __init__(self, kp, kd):
        super().__init__("traj_play")
        self.kp, self.kd = kp, kd
        self.pub_tr = self.create_publisher(MotorTrajectory, "/micro_ros_do/motor_trajectory", 10)
        self.pub_cmd = self.create_publisher(MotorCommandArray, "/micro_ros_do/motor_command_array", 10)
        self.pub_pol = self.create_publisher(MotorPolicyArray, "/micro_ros_do/motor_policy", 10)
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=50)
        self.motion = None
        self.tstate = None
        self.create_subscription(MotorMotionStateArray, "/micro_ros_up/motion_states",
                                 self._on_motion, be)
        self.create_subscription(MotorTrajectoryState, "/micro_ros_up/trajectory_state",
                                 self._on_tstate, be)
        self.trace = []

    def _on_motion(self, m):
        self.motion = (time.monotonic(), m)
        for s in m.states:
            self.trace.append((time.monotonic(), s.motor_id, s.position))

    def _on_tstate(self, m):
        self.tstate = (time.monotonic(), m)

    def spin(self, sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.005)

    def pos(self, mid):
        if not self.motion:
            return None
        for s in self.motion[1].states:
            if s.motor_id == mid:
                return s.position
        return None

    def src(self):
        """逐台 target_source：0=NONE 1=SNAPSHOT 2=TRAJECTORY。

        这是台架上「某台根本不跟」的第一判据——NONE 表示该台没过
        entry_may_move 的门（policy_enable / motion_authorized /
        dev_params_allow_motion 之一），此时发再多命令它都不动，
        而 ROS 侧没有任何报错（快照仍被受理、计数照涨）。
        """
        if not self.motion:
            return {}
        return {s.motor_id: s.target_source for s in self.motion[1].states}

    def age(self):
        if not self.motion:
            return {}
        return {s.motor_id: s.state_age_ms for s in self.motion[1].states}

    def elapsed(self, tid):
        if self.tstate and self.tstate[1].trajectory_id == tid:
            return self.tstate[1].elapsed_ms, self.tstate[1].active, self.tstate[1].completed
        return None, False, False

    def arm(self, ids):
        m = MotorPolicyArray()
        m.sync_time_ns = time.time_ns()
        m.request_id = int(time.time()) & 0xFFFF
        m.policy_generation = (int(time.time()) // 1) & 0x7FFFFFFF or 1
        m.count = len(ids)
        m.targets = [MotorPolicyTarget(motor_id=i, enabled=True, kp=self.kp, kd=self.kd,
                                       confirm_unprotected=True) for i in ids]
        for _ in range(5):
            self.pub_pol.publish(m)
            self.spin(0.1)
        self.spin(1.0)

    def hold(self, ids, poss, seconds):
        """用快照把臂稳在给定位置（轨迹通道接管前的起点对齐）。"""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            c = MotorCommandArray()
            c.sync_time_ns = time.time_ns()
            c.count = len(ids)
            c.targets = [MotorCommandTarget(motor_id=i, position=poss[i],
                                            velocity=0.0, effort=0.0) for i in ids]
            self.pub_cmd.publish(c)
            self.spin(0.05)

    def cancel_traj(self):
        m = MotorTrajectory()
        m.trajectory_id = 0
        m.segment_count = 1
        m.count = 0
        m.target_count = 0
        self.pub_tr.publish(m)
        self.spin(0.8)

    def send_window(self, tid, ids, chunk):
        """发一窗：chunk 首点重设为 t=0（固件受理时把 t=0 设到受理时刻）。"""
        t0 = chunk[0][0]
        m = MotorTrajectory()
        m.sync_time_ns = time.time_ns()
        m.sequence = 0
        m.trajectory_id = tid
        m.segment_index = 0
        m.segment_count = 1
        m.target_count = len(ids)
        m.target_ids = ids
        pts = []
        for t, vals in chunk:
            for i in ids:
                p, v, e = vals.get(i, (0.0, 0.0, 0.0))
                pts.append(MotorTrajectoryPoint(time_from_start_ms=int(t - t0),
                                                motor_id=i, position=p,
                                                velocity=v, effort=e))
        m.count = len(pts)
        m.points = pts
        self.pub_tr.publish(m)
        return len(pts), chunk[-1][0] - t0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", required=True)
    ap.add_argument("--kp", type=float, default=30.0)
    ap.add_argument("--kd", type=float, default=1.0)
    ap.add_argument("--hold-s", type=float, default=2.5, help="起点对齐保持时长")
    ap.add_argument("--lead-ms", type=int, default=0,
                    help="提前多少 ms 交棒。固件受理新轨迹时把 t=0 重设到受理时刻，"
                         "所以交棒越贴近上一窗的末点、位置越连续；提前量补偿的是"
                         "下行延迟，给大了反而让新窗起点跑到臂前面形成台阶")
    ap.add_argument("--loop", type=int, default=1)
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()

    ids, steps = load_series(a.json)
    wins = make_windows(steps)
    span = steps[-1][0]
    log(f"[play] {a.json}")
    log(f"       目标 {ids} | {len(steps)} 个时刻（{len(steps)*len(ids)} 点）"
        f" | 时长 {span} ms | 切 {len(wins)} 窗，每窗 ≤{WINDOW_STEPS} 时刻")
    for k, w in enumerate(wins):
        log(f"       窗{k}: {len(w)} 时刻 t={w[0][0]}..{w[-1][0]}ms "
            f"({len(w)*len(ids)} 点){'  含边界重叠点' if k else ''}")
    if any(len(w) * len(ids) > MAX_POINTS for w in wins):
        log("[play] 错误：有窗口超过 256 点上限"); return 2
    if a.dry_run:
        log("[play] dry-run 结束（未动臂）"); return 0

    rclpy.init()
    n = Player(a.kp, a.kd)
    log(f"[play] 武装 kp={a.kp} kd={a.kd}")
    n.cancel_traj()
    n.arm(ids)
    start = {i: steps[0][1][i][0] for i in ids}
    log(f"[play] 起点对齐到轨迹首点 {start}，保持 {a.hold_s}s")
    n.hold(ids, start, a.hold_s)
    settled = {i: n.pos(i) for i in ids}
    src, age = n.src(), n.age()
    log(f"[play] 对齐后实测 {({k: round(v,4) for k,v in settled.items() if v is not None})}")
    log(f"[play] target_source {src}（1=快照 0=无目标 2=轨迹）")
    stale = [i for i, a in age.items() if a > 200]
    if stale:
        log(f"[play] ⚠ 这些台的运动反馈已过期 { {i: age[i] for i in stale} } ms——"
            f"它们不会跟随任何命令")
    if any(src.get(i) == 0 for i in ids):
        log("[play] ⚠ 有台 target_source=0（未被下发目标）：快照/轨迹命令会被静默忽略，"
            "先查该台的 policy_enable / motion_authorized / dev_params_allow_motion")

    for rep in range(a.loop):
        n.trace.clear()
        for k, w in enumerate(wins):
            tid = 90000 + rep * 100 + k
            npts, dur = n.send_window(tid, ids, w)
            log(f"[play] 第{rep+1}轮 窗{k}/{len(wins)-1} 已发 {npts} 点 时长 {dur}ms")
            # 等这一窗跑完（用固件回带的 elapsed 计时，别用本地钟）
            end = time.monotonic() + dur / 1000.0 + 3.0
            while time.monotonic() < end:
                n.spin(0.01)
                el, act, done = n.elapsed(tid)
                nxt = wins[k + 1][0][0] - w[0][0] if k + 1 < len(wins) else None
                if el is not None and el + a.lead_ms >= dur:
                    break
        n.spin(1.0)

    # 结论
    per = {}
    for t, mid, p in n.trace:
        per.setdefault(mid, []).append((t, p))
    log("\n=== 结果 ===")
    # 交棒是唯一可能产生位置台阶的地方：逐电机列出最大的几次单采样位移及其时刻，
    # 台阶出现在窗口边界附近（时间戳与窗口起点的关系）就说明交棒时刻没压准。
    for i in ids:
        tr = per.get(i, [])
        if len(tr) < 20:
            continue
        jumps = sorted(((abs(tr[j][1] - tr[j-1][1]), tr[j][0]) for j in range(1, len(tr))),
                       reverse=True)[:3]
        med = sorted(abs(tr[j][1] - tr[j-1][1]) for j in range(1, len(tr)))[len(tr)//2]
        log(f"  m{i} 最大单采样位移 {jumps[0][0]:.4f} rad（中位 {med:.5f}）" +
            ("  ⚠ 比值 >20× 提示有台阶" if med > 1e-6 and jumps[0][0] / med > 20 else ""))
    for i in ids:
        tr = per.get(i, [])
        if len(tr) < 10:
            log(f"  m{i}: 采样不足"); continue
        d = [abs(tr[j][1] - tr[j-1][1]) for j in range(1, len(tr))]
        med = sorted(d)[len(d)//2]
        vs = [(tr[j][1] - tr[j-1][1]) / max(tr[j][0] - tr[j-1][0], 1e-6)
              for j in range(1, len(tr))]
        vm = max(abs(x) for x in vs) or 1e-9
        rev = sum(1 for j in range(1, len(vs))
                  if vs[j] * vs[j-1] < 0 and min(abs(vs[j]), abs(vs[j-1])) > 0.1 * vm)
        log(f"  m{i}: 采样 {len(tr)} | 终点 {tr[-1][1]:.4f} | |v|max {vm:.2f} rad/s "
            f"| 速度反号 {rev} | 最大单步/中位 {max(d)/max(med,1e-9):.1f}×")
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
