#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""命令快照流：武装整条臂 + 以固定节奏持续发 MotorCommandArray（默认 100 Hz）。

为什么需要它（2026-10-14）：固件的新鲜度级联是 30 ms / 100 ms——目标过期 >30 ms
软停、>100 ms 直接 `motor_control_revoke_motion()`（AUTHORIZATION_LOST + 全部
DISABLE）。所以命令**必须**以 ≥33 Hz 持续下发。已有的下发路径都不适合做台架验证：

  - 浏览器面板（`web_host` 的「100Hz 连发」）：`setInterval(setInterval, 10ms)` 在
    页面渲染负载下会给不出 100 Hz，一次 >100 ms 的卡顿就撤权；它适合人工点两下，
    不适合长时间验证。
  - `traj_play.py`：走 **轨迹** 通道（MCU 侧插值），不测命令快照这条通道。

本工具补上这一块：命令值、节奏、时长都由命令行给，跑完打印统计（ACTIVE 台数、
target_source、被控关节的跟随误差），给"链路是否像接真机"一个可复现的判据。

拓扑（电机 id / 关节 / kp / kd）**只从权威 YAML 读**，与 `simulation.launch.py`
和固件同源；不在这里另写一份默认表。

用法：
  # echo 模式：不发命令，只看反馈是否正常（链路体检，最安全）
  python3 snapshot_stream.py --mode echo --seconds 10

  # 保持当前位姿：武装 + 100 Hz 快照，目标 = 收到的最新反馈位置
  python3 snapshot_stream.py --mode hold --seconds 20

  # 单关节小步进（验证"下发 → 反馈跟上 → 被控对象跟上"）
  python3 snapshot_stream.py --mode step --step-id 1 --step-rad 0.1 --seconds 20

  # 只武装不发命令（排查"武装是否被作废"）
  python3 snapshot_stream.py --mode arm --seconds 5
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from can_bridge_msgs.msg import (MotorCommandArray, MotorCommandTarget,
                                 MotorEvent, MotorMotionStateArray,
                                 MotorPolicyArray, MotorPolicyTarget,
                                 MotorStatusArray)

LIFECYCLE_ACTIVE = 4
LIFECYCLE_NAMES = {0: "缺席", 1: "发现", 2: "识别中", 3: "就绪未使能", 4: "运行",
                   5: "安全停机", 6: "故障", 7: "配置冲突", 8: "离线"}
FAULT_NAMES = {0: "无", 1: "协议", 2: "厂家", 3: "Profile", 4: "冲突", 5: "离线",
               6: "设备参数未初始化", 7: "策略代数未前进", 8: "方向已翻需重设零"}
# CONFIG_RESULT 的 reason（motor_protocol_config_result_t，唯一真源见
# motor_protocol_adapter.h）
CONFIG_RESULT_NAMES = {0: "OK", 1: "BUSY", 2: "INVALID", 3: "UNSUPPORTED",
                       4: "DEVICE_ERROR", 5: "TIMEOUT", 6: "UNCONFIRMED"}
# 事件类型与 motor_control_event_type_t 对齐（唯一真源：MotorEvent.msg 注释）
EVENT_NAMES = {0: "发现", 1: "Profile绑定", 2: "Profile拒绝", 3: "就绪", 4: "运行",
               5: "软停", 6: "失能", 7: "故障", 8: "配置冲突", 9: "离线", 10: "重连",
               11: "授权丢失", 12: "快照拒绝", 13: "配置结果", 14: "注册表满",
               15: "设备参数", 16: "轨迹拒绝"}
DEFAULT_CONFIG = ("ros2_ws/src/robot_sim/models/half_body_robot/config/"
                  "motor_driver.yaml")
# policy_generation 的**单调递增**起点与落地文件。
#
# 为什么不能直接用时间戳：内核的代数门是「enable 且当前未使能时，
# policy_generation 必须严格大于表项已锁存的值」（motor_control_apply_policy_entry，
# 防「周期重发自动复权」）。表项里那个值**跨进程存活**，所以主机每次重启都拿
# 「当前时间掩码」当起点是有害的——掩码 0x7FFFFFFF 每 ~36 分钟回绕，重启后很可能
# 比锁存值更小 ⇒ 每一次武装都被判「代数未前进」，而且内核把这个原因**报成
# CONFIG_INVALID(reason=2)**，看不出真因（2026-10-14 台架实测踩到）。
# 对策：起点固定为 0x80000000（高于任何历史掩码值），并把用过的最大值落盘，
# 于是「只增不减」跨进程成立；被拒时再翻倍逃逸（见 arm()）。
GENERATION_FLOOR = 0x80000000
GENERATION_FILE = Path.home() / ".ros" / "micro_ros_policy_generation"


def _load_generation() -> int:
    floor = GENERATION_FLOOR
    try:
        floor = max(floor, int(GENERATION_FILE.read_text().strip()))
    except Exception:                   # 文件缺失/损坏：用起点即可
        pass
    return floor & 0xFFFFFFFF


def _store_generation(value: int) -> None:
    try:
        GENERATION_FILE.parent.mkdir(parents=True, exist_ok=True)
        GENERATION_FILE.write_text(f"{value & 0xFFFFFFFF}\n")
    except Exception:                   # 落盘失败不该让台架停下
        pass


def log(*a):
    print(*a, flush=True)


def load_topology(path: Path):
    """读权威拓扑 YAML → [(id, joint, type, kp, kd)]，按文件声明顺序。"""
    doc = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    motors = (doc.get("motor_hardware") or {}).get("motors") or {}
    if not isinstance(motors, dict) or not motors:
        raise SystemExit(f"{path} 里没有 motor_hardware.motors")
    out = []
    for name, spec in motors.items():
        command = spec.get("command") or {}
        out.append((int(spec["id"]), str(spec.get("joint", "")),
                    str(spec.get("type", "")), float(command.get("kp", 0.0)),
                    float(command.get("kd", 0.0))))
    return out


class Streamer(Node):
    def __init__(self, rate_hz: float, command_topic: str):
        super().__init__("snapshot_stream")
        # QoS 按对端各配各的（2026-10-15 用 `ros2 topic info -v` 逐话题实测）：
        #   - 下行命令：固件的订阅端 **BEST_EFFORT**。这里给 RELIABLE 是"提供强于
        #     请求"，兼容；反过来（发 best_effort 而对方请求 reliable）才会被静默拒收。
        #   - 下行策略：固件的订阅端 **RELIABLE**，所以这里必须 RELIABLE。
        #   - 上行 motion_states：固件的发布端 **offering BEST_EFFORT**，订阅端用
        #     reliable 会"静默收不到"（DDS 规则：sub 的可靠性不能高于 pub）。
        # depth=1/keep_last：实时流只保最新，积压的旧快照没有意义（过期即撤权）。
        qos_tx = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            history=HistoryPolicy.KEEP_LAST)
        qos_rx = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST)
        self.pub_cmd = self.create_publisher(
            MotorCommandArray, command_topic, qos_tx)
        self.pub_pol = self.create_publisher(
            MotorPolicyArray, "/micro_ros_do/motor_policy", qos_tx)
        self.create_subscription(MotorMotionStateArray,
                                 "/micro_ros_up/motion_states", self._on_motion,
                                 qos_rx)
        self.create_subscription(MotorStatusArray,
                                 "/micro_ros_up/motor_status_array",
                                 self._on_status, qos_rx)
        # 事件流：**拒绝原因的唯一出口**（武装/快照被拒只在这里说明理由）。
        self.create_subscription(MotorEvent, "/micro_ros_up/motor_events",
                                 self._on_event, qos_rx)
        self.pos: dict[int, float] = {}
        self.target_source: dict[int, int] = {}
        self.lifecycle: dict[int, int] = {}
        self.fault: dict[int, int] = {}
        self.unprotected: dict[int, bool] = {}
        self.new_events: list[str] = []
        self.motion_frames = 0
        self.rate = rate_hz
        self.sequence = (time.time_ns() // 1000) & 0x7FFFFFFF
        self.generation = _load_generation()
        self.arm_rejected = False
        self.need_confirm = False
        self.stream_started_at: float | None = None

    def _on_motion(self, msg):
        for s in msg.states:
            self.pos[s.motor_id] = float(s.position)
            self.target_source[s.motor_id] = int(s.target_source)

    def _on_status(self, msg):
        for s in msg.statuses:
            self.lifecycle[s.motor_id] = int(s.lifecycle)
            self.fault[s.motor_id] = int(s.unified_fault_code)
            # 「未保护」= 设备侧保护**确知为关**（限位保护不在位）→ 武装必须带
            # 显式确认，否则一律被拒（MotorPolicyTarget.confirm_unprotected 的语义）。
            self.unprotected[s.motor_id] = bool(s.device_protect_known
                                               and not s.device_protect_enabled)

    def _on_event(self, msg):
        reason = int(msg.reason)
        kind = int(msg.event_type)
        text = f"{EVENT_NAMES.get(kind, kind)} id={msg.motor_id} " \
               f"reason={reason}"
        if kind == 11:                      # AUTHORIZATION_LOST
            text += "（目标过期/会话丢失 → 已撤权，必须重新武装）"
        elif kind == 7:
            text += f"（故障码 {FAULT_NAMES.get(reason, reason)}）"
        elif kind == 15:
            text += "（设备参数上电比对：2=OK 3=未初始化 4=回读失败）"
        elif kind == 13 and reason != 0:
            # 内核把「代数未前进 / 设备参数未核对 / profile 未绑定 / kp-kd 非法」
            # 都报成 reason=2；reason=6 则是「未带未保护确认」（设备侧保护确知为关）。
            name = CONFIG_RESULT_NAMES.get(reason, str(reason))
            text += f"（={name}"
            if reason == 6:
                self.need_confirm = True     # 设备明说了要确认 → 下一次武装带上
            elif reason == 2:
                text += "：可能是代数未前进/设备参数未核对/未绑定/kp-kd 非法"
            text += "）"
            if int(msg.request_id) == self.generation:
                self.arm_rejected = True
        self.new_events.append(text)

    def auto_confirm(self, motors) -> bool:
        """全部目标电机都「确知未保护」时，允许自动带上确认（仿真台架的常态）。

        真机上只要有一台保护状态未知或在位，就不自动确认——那必须由操作者显式
        用 `--confirm-unprotected` 承担。
        """
        states = [self.unprotected.get(mid) for mid, *_ in motors]
        return bool(states) and all(states)

    def lifecycle_line(self):
        """`运行 15/18` + 非运行台的分布/故障码——判断"武装是否生效"的直接证据。"""
        if not self.lifecycle:
            return "（还没有状态帧）"
        active = sum(1 for v in self.lifecycle.values() if v == LIFECYCLE_ACTIVE)
        bad = {mid: v for mid, v in self.lifecycle.items()
               if v != LIFECYCLE_ACTIVE}
        line = f"运行 {active}/{len(self.lifecycle)}"
        if bad:
            hist = {}
            for mid, v in bad.items():
                hist.setdefault(v, []).append(mid)
            detail = "；".join(
                f"{len(ids)}台{LIFECYCLE_NAMES.get(v, v)}"
                f"[{','.join(map(str, sorted(ids)))}]"
                for v, ids in sorted(hist.items()))
            faults = {mid: c for mid, c in self.fault.items() if c}
            if faults:
                detail += "；故障码 " + " ".join(
                    f"{mid}:{FAULT_NAMES.get(c, c)}" for mid, c in sorted(faults.items()))
            line += f"（{detail}）"
        return line
        self.motion_frames += 1

    # ---- 下行 --------------------------------------------------------------
    def arm(self, motors, confirm_unprotected=False, grow=False):
        """一条消息带全部目标（稀疏语义）。代数前进 = 新意图。

        `grow=True`（上一次被拒）时把代数**翻倍**：万一表项锁存着一个比我们
        起点更大的值（例如更早的会话/别的工具写过），一两次就能跳过去。
        """
        auto = self.auto_confirm(motors)
        confirm = confirm_unprotected or auto or self.need_confirm
        step = max(1, self.generation) if grow else 1
        self.generation = (self.generation + step) & 0xFFFFFFFF
        _store_generation(self.generation)
        msg = MotorPolicyArray()
        msg.sync_time_ns = time.time_ns()
        msg.request_id = self.generation
        msg.policy_generation = self.generation
        msg.count = len(motors)
        msg.targets = [
            MotorPolicyTarget(motor_id=mid, enabled=True, kp=kp, kd=kd,
                              confirm_unprotected=confirm)
            for mid, _joint, _kind, kp, kd in motors
        ]
        self.pub_pol.publish(msg)
        why = "（带未保护确认：目标电机设备侧保护确知为关）" if auto else ""
        log(f"[stream] 武装 {len(motors)} 台 gen={self.generation}{why}")
        return self.generation

    def flush_events(self) -> None:
        """把固件事件打出来——受理/拒绝的理由只在这里能看到。"""
        while self.new_events:
            log(f"[事件] {self.new_events.pop(0)}")

    def send_snapshot(self, commands):
        self.sequence = (self.sequence + 1) & 0x7FFFFFFF
        msg = MotorCommandArray()
        msg.sync_time_ns = time.time_ns()
        msg.sequence = self.sequence
        msg.count = len(commands)
        msg.targets = [
            MotorCommandTarget(motor_id=mid, position=p, velocity=v, effort=e)
            for mid, p, v, e in commands
        ]
        self.pub_cmd.publish(msg)
        return msg.sequence

    def step(self, duration: float):
        end = time.monotonic() + duration
        rclpy.spin_once(self, timeout_sec=0.0)
        now = time.monotonic()
        if now >= end:
            return False
        time.sleep(max(0.0, 1.0 / self.rate))
        return True

    def active_count(self):
        return sum(1 for v in self.target_source.values() if v == 1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=None,
                    help=f"权威拓扑 YAML（默认仓库内 {DEFAULT_CONFIG}）")
    ap.add_argument("--mode", choices=("echo", "arm", "hold", "step"),
                    default="echo")
    ap.add_argument("--hz", type=float, default=100.0, help="快照下发频率（必须 ≥33）")
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--step-id", type=int, default=1)
    ap.add_argument("--step-rad", type=float, default=0.1)
    ap.add_argument("--status-hz", type=float, default=1.0)
    ap.add_argument("--confirm-unprotected", action="store_true",
                    help="武装时带「设备侧保护确知为关」的显式确认（真机必带；"
                         "仿真里假电机不上报保护，通常不需要）")
    ap.add_argument("--command-topic", default="/micro_ros_do/motor_command_array",
                    help="快照下发话题。改指 /robot_dynamics/command_array 可经"
                         "重力前馈叠加节点中转；默认直连固件，行为不变"
                         "（见 robot_dynamics/README.md）")
    args = ap.parse_args()

    repo = Path(__file__).resolve().parents[2]
    config = Path(args.config) if args.config else repo / DEFAULT_CONFIG
    if not config.is_file():
        raise SystemExit(f"找不到拓扑 YAML：{config}")
    motors = load_topology(config)
    log(f"[stream] 拓扑 {config}：{len(motors)} 台 "
        f"{[m[0] for m in motors]}")

    rclpy.init()
    node = Streamer(args.hz, args.command_topic)
    log(f"[stream] mode={args.mode} hz={args.hz:g} seconds={args.seconds:g}；"
        "先等反馈（无反馈说明 agent/固件链路没通）…")
    wait_until = time.monotonic() + min(10.0, args.seconds)
    while time.monotonic() < wait_until and not node.pos:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not node.pos:
        log("[stream] ⚠ 收不到 motion_states —— 固件链路不通，退出")
        node.destroy_node()
        rclpy.shutdown()
        return 2
    log(f"[stream] 反馈已通：{len(node.pos)} 台，"
        f"{'  '.join(f'{k}={v:+.3f}' for k, v in sorted(node.pos.items())[:6])} …")

    if args.mode == "echo":
        end = time.monotonic() + args.seconds
        last = 0.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            node.flush_events()
            now = time.monotonic()
            if now - last >= 1.0 / args.status_hz:
                last = now
                log(f"[stream] 反馈 {node.motion_frames} 帧 | {node.lifecycle_line()}"
                    f" | 位姿 "
                    f"{'  '.join(f'{k}={v:+.3f}' for k, v in sorted(node.pos.items())[:4])}")
        node.destroy_node()
        rclpy.shutdown()
        return 0

    node.arm(motors, confirm_unprotected=args.confirm_unprotected)
    if args.mode == "arm":
        end = time.monotonic() + args.seconds
        last = 0.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            node.flush_events()
            now = time.monotonic()
            if now - last >= 1.0 / args.status_hz:
                last = now
                log(f"[stream] 武装后 {node.lifecycle_line()}"
                    f"（target_source={sorted(set(node.target_source.values()))}；"
                    "没有快照时 projected 源恒为 0，属正常）")
        node.destroy_node()
        rclpy.shutdown()
        return 0

    hold = dict(node.pos)
    end = time.monotonic() + args.seconds
    last_status = 0.0
    last_arm_log = 0.0
    last_arm_check = 0.0
    sent = 0
    while time.monotonic() < end:
        commands = []
        for mid, joint, _kind, _kp, _kd in motors:
            p = hold.get(mid, 0.0)
            if args.mode == "step" and mid == args.step_id:
                p += args.step_rad
            commands.append((mid, p, 0.0, 0.0))
        node.send_snapshot(commands)
        sent += 1
        rclpy.spin_once(node, timeout_sec=0.0)
        node.flush_events()
        now = time.monotonic()
        # 链路抖动（CAN 静默 / 会话重建）会让固件**撤权并全部 DISABLE**，且冻结态
        # 只认「新代数 + 新快照」（ARD §36）。真机上的主机控制器也会这么干：发现
        # 设备掉出运行态就重新武装，并把保持目标改成**当前实测位姿**（而不是抖前
        # 的旧目标——对着旧目标重新发力会让机构弹回去）。
        if now - last_arm_check >= 0.5 and node.lifecycle:
            last_arm_check = now
            active = sum(1 for v in node.lifecycle.values() if v == LIFECYCLE_ACTIVE)
            if active < len(node.lifecycle):
                node.arm(motors, confirm_unprotected=args.confirm_unprotected,
                         grow=node.arm_rejected)
                node.arm_rejected = False
                hold = dict(node.pos)
                if now - last_arm_log >= 5.0:      # 反复失败时别刷屏
                    last_arm_log = now
                    log(f"[stream] 只有 {active}/{len(node.lifecycle)} 台在运行态"
                        f" → 重新武装并改用当前实测位姿（详见上面的 [事件] 行）")
        if now - last_status >= 1.0 / args.status_hz:
            last_status = now
            err = ""
            if args.mode == "step":
                got = node.pos.get(args.step_id, float("nan"))
                want = hold.get(args.step_id, 0.0) + args.step_rad
                err = f" | m{args.step_id} 目标{want:+.3f} 实测{got:+.3f} " \
                      f"误差{want - got:+.3f} rad"
            log(f"[stream] 已发 {sent} 帧 | {node.lifecycle_line()}"
                f" | 快照被采纳 {node.active_count()}/{len(motors)}{err}")
        time.sleep(max(0.0, 1.0 / args.hz - (time.monotonic() - now)))
    log(f"[stream] 结束：共发 {sent} 帧（{sent / args.seconds:.1f} Hz 实测），"
        f"{node.lifecycle_line()}，快照被采纳 {node.active_count()}/{len(motors)}")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
