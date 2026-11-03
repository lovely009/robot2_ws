#!/usr/bin/env python3
"""MIT 驱动器替身：在 Gazebo 上复现 CERB 的 MIT 律，用来对照「线性插值 vs C¹ 插补」。

真机链路：固件(插补) → FDCAN → 驱动器(内部 MIT 律) → 关节
本脚本：  路点 → 插值(linear|hermite) → τ = kp(p_des−p) + kd(v_des−v) + τ_ff → Gazebo effort

为什么要它：CAN 模拟器把关节速度写死（fdcan_motor_sim.py:169），看不出插补的效果；
Gazebo 的关节会因力矩真实运动，于是 C¹ 插补**消除速度台阶 → 消除力矩台阶**这件事
是可观测、可量的。

用法（需先切到 effort 控制器）：
  ros2 control switch_controllers --deactivate arm_position_controller \
                                  --activate arm_effort_controller
  python3 scripts/mit_gazebo_probe.py --interp linear
  python3 scripts/mit_gazebo_probe.py --interp hermite
输出：/tmp/mit_<interp>.csv（t, p_des, v_des, p_act, v_act, tau）与终端摘要。
"""
import argparse
import csv
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

JOINTS = ["joint_link1", "joint_link2", "joint_link3", "joint_link4"]
KP, KD, TAU_FF = 8.0, 0.5, 0.0
TAU_LIMIT = 20.0
CTRL_HZ = 50.0                      # 与固件 20ms 控制边界一致
WAYPOINTS_MS = [(0, 0.0), (1000, 0.6), (2000, 0.3)]   # 相对起点的偏移（rad）


def monotone_tangents(pts):
    """与固件 trajectory_monotone_tangent() 同算法的切矢（Fritsch–Carlson 限幅）。

    pts: [(t_s, x)]，按时间升序。端点恒 0（起止零速）。
    """
    n = len(pts)
    m = [0.0] * n
    for k in range(1, n - 1):
        t0, x0 = pts[k - 1]
        t1, x1 = pts[k]
        t2, x2 = pts[k + 1]
        d0 = (x1 - x0) / (t1 - t0)
        d1 = (x2 - x1) / (t2 - t1)
        if d0 * d1 <= 0.0:
            m[k] = 0.0
            continue
        a0, a1 = abs(d0), abs(d1)
        limit = 3.0 * min(a0, a1)
        v = 0.5 * (d0 + d1)
        m[k] = max(-limit, min(limit, v))
    return m


class MitProbe(Node):
    def __init__(self, interp):
        super().__init__("mit_gazebo_probe")
        self.interp = interp
        self.pos = {j: 0.0 for j in JOINTS}
        self.vel = {j: 0.0 for j in JOINTS}
        self.seen = False
        self.create_subscription(JointState, "/joint_states", self.on_state, 10)
        self.pub = self.create_publisher(Float64MultiArray,
                                         "/arm_effort_controller/commands", 10)

    def on_state(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.pos and i < len(msg.position):
                self.pos[name] = msg.position[i]
                self.vel[name] = msg.velocity[i] if i < len(msg.velocity) else 0.0
                self.seen = True

    def spin_once(self, timeout=0.1):
        rclpy.spin_once(self, timeout_sec=timeout)

    def apply(self, tau):
        out = Float64MultiArray()
        out.data = [float(tau.get(j, 0.0)) for j in JOINTS]
        self.pub.publish(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--interp", choices=["linear", "hermite"], default="hermite")
    ap.add_argument("--target", default="joint_link1")
    ap.add_argument("--settle", type=float, default=1.5)
    args = ap.parse_args()

    rclpy.init()
    node = MitProbe(args.interp)
    while not node.seen:
        node.spin_once(0.1)
    node.get_logger().info("已收到 /joint_states，开始")

    p_start = dict(node.pos)
    hold = {j: p_start[j] for j in JOINTS}

    # 路点：相对被测关节起点
    pts = [(ms / 1000.0, p_start[args.target] + dx) for ms, dx in WAYPOINTS_MS]
    tangents = monotone_tangents(pts) if args.interp == "hermite" else None
    total = pts[-1][0]

    # 先把非被测关节稳住（否则它们会因重力垮下去）
    end = time.time() + args.settle
    while time.time() < end:
        node.spin_once(0.02)
        node.apply({j: KP * (hold[j] - node.pos[j]) - KD * node.vel[j]
                    for j in JOINTS})

    def sample(t_s):
        """返回 (p_des, v_des)，单位 rad / rad·s⁻¹。"""
        if t_s <= pts[0][0]:
            return pts[0][1], 0.0
        if t_s >= total:
            return pts[-1][1], 0.0
        k = 0
        while k + 2 < len(pts) and pts[k + 1][0] <= t_s:
            k += 1
        (t0, x0), (t1, x1) = pts[k], pts[k + 1]
        h = t1 - t0
        u = (t_s - t0) / h
        if args.interp == "linear":
            # 与固件改动前的行为一致：位置线性、速度 = 段斜率（分片常数）
            return x0 + u * (x1 - x0), (x1 - x0) / h
        m0, m1 = tangents[k], tangents[k + 1]
        u2, u3 = u * u, u * u * u
        p = ((2 * u3 - 3 * u2 + 1) * x0 + (u3 - 2 * u2 + u) * h * m0
             + (-2 * u3 + 3 * u2) * x1 + (u3 - u2) * h * m1)
        v = ((6 * u2 - 6 * u) * x0 + (3 * u2 - 4 * u + 1) * h * m0
             + (-6 * u2 + 6 * u) * x1 + (3 * u2 - 2 * u) * h * m1) / h
        return p, v

    rows = []
    t0 = time.time()
    period = 1.0 / CTRL_HZ
    while True:
        t_s = time.time() - t0
        node.spin_once(0.0)
        p_des, v_des = sample(t_s)
        tau = {}
        for j in JOINTS:
            pd = p_des if j == args.target else hold[j]
            vd = v_des if j == args.target else 0.0
            e = KP * (pd - node.pos[j]) + KD * (vd - node.vel[j]) + TAU_FF
            tau[j] = max(-TAU_LIMIT, min(TAU_LIMIT, e))
        node.apply(tau)
        rows.append((t_s, p_des, v_des, node.pos[args.target],
                     node.vel[args.target], tau[args.target]))
        if t_s > total + 0.5:
            break
        time.sleep(max(0.0, period - (time.time() - t0 - t_s)))

    # 松手：别再压着力矩
    for _ in range(10):
        node.apply({j: 0.0 for j in JOINTS})
        node.spin_once(0.01)

    path = "/tmp/mit_%s.csv" % args.interp
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "p_des", "v_des", "p_act", "v_act", "tau"])
        w.writerows(rows)

    dtau = [abs(rows[i + 1][5] - rows[i][5]) for i in range(len(rows) - 1)]
    print("插值=%s  采样 %d 点  关节=%s" % (args.interp, len(rows), args.target))
    print("  tau 范围 %.3f .. %.3f N·m" % (min(r[5] for r in rows),
                                           max(r[5] for r in rows)))
    print("  相邻 |Δtau|: 中位 %.4f  95%% %.4f  最大 %.4f" %
          (sorted(dtau)[len(dtau) // 2], sorted(dtau)[int(len(dtau) * .95)],
           max(dtau)))
    print("  端点 tau: t=0 %.3f   t=%.2f %.3f" %
          (rows[0][5], rows[-1][0], rows[-1][5]))
    print("  CSV: %s" % path)

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
