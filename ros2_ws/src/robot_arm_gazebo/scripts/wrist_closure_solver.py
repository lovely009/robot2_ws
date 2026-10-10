#!/usr/bin/env python3
"""腕部闭环求解器：把切开的闭环补回去，让被动关节角始终满足「二力杆 + 曲柄圆」。

背景见 models/zhoubuguanjie_youbi_001/README.md 的腕部章节与 config/wrist_closure.yaml。

机构：两台 X25S 关节电机**固定在小臂（Joint_link4）上**，各自经「曲柄—连杆」驱动十字轴腕的
一个自由度；两个电机的转角共同决定腕部的两个自由度。URDF 是树，闭环切在「连杆末端球心」
这条边上，于是树里：

    小臂 4 ─┬─(fixed) 电机 10 ─(crank_a, 绕电机输出轴)─ 曲柄销 link9
            └─ 5 ─ 6 (十字轴腕) ─(球铰 2 轴)─ 连杆 7    ← 连杆末端是自由端

本节点订阅上位发布的关节角（例如 RViz 的 joint_state_publisher_gui），按闭环约束解出
每分支三个被动关节（曲柄转角 + 球铰两轴），把修正后的完整关节状态再发出去。

每分支的解法（全部闭式，无迭代）：
  1. 腕端球心**就是连杆原点**（与被动角无关：球铰的回转中心钉在 link6 上，连杆原点与它重合）；
  2. 曲柄销中心只能落在「以电机输出轴为轴、半径 r 的圆」上 —— 该圆在**电机系**里是常量，
     r 与相位由零位位形 + 配置里的 crank_pin_offset 推出（URDF 不表达曲柄销球心）；
  3. 杆长守恒 |P9(φ) − P7| = L ⇒ 解出曲柄转角 φ（闭式，两支取与前一刻连续的那支）；
  4. 球铰两轴把连杆「指向」P9(φ)：2 个自由度对着球面上一点，位置可精确满足；
     连杆绕自身轴的自转是球铰的**自由自由度**，两支都精确满足位置，按与前一刻连续选取。

因为曲柄现在是树里真实的 revolute（joint_crank_a/b），本版**不再需要**旧版的「销轴补偿角」
——旧版把整台电机当摇臂绕轴转（读数取「Joint_link9/10 随摇臂刚性转动」），既与「电机固定于
小臂」的物理矛盾，也留下一个结构性的姿态残差。

安全：任何一步无解或超限都只降级（夹紧 + 告警），不放大输出；本节点不发任何写命令，
真机链路不经过它。
"""

import math
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState

# ---------------------------------------------------------------------------
# 纯运动学（不依赖 ROS，可直接单测）
# ---------------------------------------------------------------------------


def rpy_to_R(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
            @ np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
            @ np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]]))


def origin_T(element):
    origin = element.find("origin")
    xyz = np.zeros(3)
    rpy = np.zeros(3)
    if origin is not None:
        if origin.get("xyz"):
            xyz = np.array([float(v) for v in origin.get("xyz").split()])
        if origin.get("rpy"):
            rpy = np.array([float(v) for v in origin.get("rpy").split()])
    T = np.eye(4)
    T[:3, :3] = rpy_to_R(*rpy)
    T[:3, 3] = xyz
    return T


def rodrigues(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    K = np.array([[0.0, -axis[2], axis[1]],
                  [axis[2], 0.0, -axis[0]],
                  [-axis[1], axis[0], 0.0]])
    return np.eye(3) + math.sin(angle) * K + (1.0 - math.cos(angle)) * (K @ K)


def rot_z(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rot_y(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


class Chain:
    """极简 URDF 运动学：按 joint 树做 FK，够用即可（模型只有 16 个关节）。"""

    def __init__(self, urdf_text):
        root = ET.fromstring(urdf_text)
        self.joints = {}
        self.children = {}
        self.links = [link.get("name") for link in root.findall("link")]
        for joint in root.findall("joint"):
            name = joint.get("name")
            axis = joint.find("axis")
            self.joints[name] = {
                "parent": joint.find("parent").get("link"),
                "child": joint.find("child").get("link"),
                "T": origin_T(joint),
                "axis": (np.array([float(v) for v in axis.get("xyz").split()])
                         if axis is not None else np.zeros(3)),
                "type": joint.get("type"),
                "limit": (float(joint.find("limit").get("lower")),
                          float(joint.find("limit").get("upper")))
                if joint.find("limit") is not None else None,
            }
            self.children.setdefault(self.joints[name]["parent"], []).append(name)
        child_links = {j["child"] for j in self.joints.values()}
        self.root = next(link for link in self.links if link not in child_links)

    def world_poses(self, angles):
        """返回 {link: T_world}；angles 里没有的关节取 0。"""
        poses = {}

        def walk(link, T):
            poses[link] = T
            for joint_name in self.children.get(link, []):
                joint = self.joints[joint_name]
                T_joint = T @ joint["T"]
                if joint["type"] in ("revolute", "continuous"):
                    T_joint = T_joint @ _axis_T(joint["axis"], angles.get(joint_name, 0.0))
                walk(joint["child"], T_joint)

        walk(self.root, np.eye(4))
        return poses


def _axis_T(axis, angle):
    T = np.eye(4)
    T[:3, :3] = rodrigues(axis, angle)
    return T


def solve_crank_angle(p7, axis_dir, axis_point, pin_zero, rod_length):
    """解曲柄转角 φ：使曲柄圆上的点与 P7 的距离等于杆长。

    圆参数化为 P9(φ) = O + r(u_hat cosφ + w_hat sinφ)（O = 销心到轴的垂足，u_hat 指向零位
    销心）。不可行（球面与圆不相交）时 delta 取 0，即取最接近的点（夹紧而非放大）。
    """
    axis_dir = axis_dir / np.linalg.norm(axis_dir)
    d0 = pin_zero - axis_point
    height0 = float(d0 @ axis_dir)
    offset = d0 - height0 * axis_dir
    r = float(np.linalg.norm(offset))
    O = axis_point + height0 * axis_dir
    if r < 1e-9:
        # 销心落在轴上 ⇒ 曲柄半径为零，机构退化
        return 0.0, 0.0, False, (O, axis_dir, np.zeros(3), 0.0)
    u_hat = offset / r
    w_hat = np.cross(axis_dir, u_hat)
    d = p7 - axis_point
    radial = d - float(d @ axis_dir) * axis_dir
    rho = float(np.linalg.norm(radial))
    if rho < 1e-12:
        return 0.0, 0.0, False, (O, u_hat, w_hat, r)
    height_rel = float(d @ axis_dir) - height0
    # |O + r(u cosφ + w sinφ) − P7| = L 展开成 cos(φ − ψ) = base/rho
    base = (r * r + rho * rho + height_rel * height_rel - rod_length * rod_length) / (2.0 * r)
    feasible = abs(base / rho) <= 1.0
    ratio = min(1.0, max(-1.0, base / rho))
    psi = math.atan2(float(radial @ w_hat), float(radial @ u_hat))
    return psi, math.acos(ratio), feasible, (O, u_hat, w_hat, r)


def pick_crank_branch(psi, delta, previous):
    """两支 φ = ψ ± δ 里取与 previous 连续的那支。

    previous 为 None（第一帧）时取最接近 0 的那支 —— 模型是在零位装配下建的，
    零位对应 φ = 0；另一支是同一个圆的「另一侧」解，几何上同样成立但装配不同。
    """
    candidates = [psi - delta, psi + delta]
    reference = 0.0 if previous is None else previous
    return min(candidates, key=lambda phi: abs(_wrap(phi - reference)))


def aim_rod(d_local, d6, flip):
    """解球铰两轴 (α, β) 使连杆轴向 d_local 在 link6 系里指向 d6。

    T_rod = T_link6 · Rot((0,1,0), α) · Rot((0,0,1), β)（两个关节 origin 的 rpy 都为 0），
    故 Ry(α) Rz(β) d_local = d6。Ry 保 y、Rz 保 z，于是 m = Rz(β) d_local 满足
    m_y = d6_y、m_z = d_local_z，m_x 由单位长度定。

    m_x 的符号对应连杆绕自身轴的自转 —— 那是球铰的自由自由度，两支都精确满足位置约束，
    差别只在连杆姿态，所以由 flip 指定、由调用方按连续性取。返回 (α, β, 是否夹紧)。

    α、β 回绕进 [-π, π]：它们是**球铰的分解角**，加 2π 是同一个位形。不回绕会算出 +352°
    这种值，再被 <limit> 误当成超限 —— 那正是本文件早期版本的假故障。真正有物理意义的是
    连杆轴向相对零位的**总摆动角**（自检里量化过：腕部 ±90° 行程内约 98°），不是这两个分角。
    """
    m_z = float(d_local[2])
    residual = 1.0 - float(d6[1]) ** 2 - m_z ** 2
    saturated = residual < 0.0
    m_x = flip * math.sqrt(max(residual, 0.0))
    m_y = float(d6[1])
    beta = math.atan2(m_y, m_x) - math.atan2(float(d_local[1]), float(d_local[0]))
    s = m_x * m_x + m_z * m_z
    if s < 1e-12:
        alpha = 0.0
    else:
        cos_a = (m_x * float(d6[0]) + m_z * float(d6[2])) / s
        sin_a = (m_z * float(d6[0]) - m_x * float(d6[2])) / s
        alpha = math.atan2(sin_a, cos_a)
    return _wrap(alpha), _wrap(beta), saturated


def zero_pose_flip(d_local):
    """零位（d6 == d_local）处给出装配位形的那个 flip：此时 m_x 应等于 d_local_x。"""
    return 1.0 if float(d_local[0]) >= 0.0 else -1.0


def _wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class Branch:
    """一条腕部分支的闭环几何：曲柄圆（在电机系里是常量）+ 连杆球心偏移。"""

    def __init__(self, chain, cfg):
        self.cfg = cfg
        self.rod_link = cfg["rod_link"]
        self.motor_link = chain.joints[cfg["crank_joint"]]["parent"]
        self.crank_pin_offset = np.array(cfg["crank_pin_offset"], dtype=float)
        # 腕端球心就是连杆原点（球铰回转中心钉在 link6 上，与连杆原点重合），所以「二力杆」
        # 的长度是**从连杆原点到曲柄销心**的距离。配置里不再有 wrist_ball_offset ——
        # 网格反演出的腕端球心与连杆原点只差 ≤2.1 mm（拟合噪声量级），把它当成连杆系里的
        # 偏移会随连杆自转一起转，而连杆原点是钉死的，两者矛盾（实测会漏 1~3 mm 的位置残差）。
        self.rod_length = float(np.linalg.norm(self.crank_pin_offset))
        self.d_local = self.crank_pin_offset / self.rod_length

        # 曲柄圆的常量部分，全在**曲柄系**里表达（局部 z 即电机输出轴，见 URDF 注释）
        T_origin = chain.joints[cfg["crank_joint"]]["T"]
        self.T_origin = T_origin
        self.axis_local = T_origin[:3, :3] @ chain.joints[cfg["crank_joint"]]["axis"]
        self.axis_local = self.axis_local / np.linalg.norm(self.axis_local)
        self.axis_point = T_origin[:3, 3]

        # 零位下把曲柄销球心换算进曲柄系：轴向分量 pz 定圆心，横向分量定半径 r 与相位
        zero_poses = chain.world_poses({})
        T_motor0 = zero_poses[self.motor_link]
        T_rod0 = zero_poses[self.rod_link]
        pin_world0 = (T_rod0 @ np.append(self.crank_pin_offset, 1.0))[:3]
        pin_in_motor0 = T_motor0[:3, :3].T @ (pin_world0 - T_motor0[:3, 3])
        pin_in_crank = T_origin[:3, :3].T @ (pin_in_motor0 - T_origin[:3, 3])
        self.radius = float(math.hypot(pin_in_crank[0], pin_in_crank[1]))
        self.center_local = self.axis_point + float(pin_in_crank[2]) * self.axis_local
        if self.radius < 1e-9:
            raise ValueError(f"{self.rod_link}: 曲柄半径为零，机构退化（检查 crank_pin_offset）")
        u_crank = np.array([pin_in_crank[0], pin_in_crank[1], 0.0]) / self.radius
        self.u_local = T_origin[:3, :3] @ u_crank
        self.w_local = np.cross(self.axis_local, self.u_local)

        self._flip = zero_pose_flip(self.d_local)
        self._phi = None
        self._rod_rotation = None

    def pin_at(self, T_motor, phi):
        """给定电机位姿与曲柄转角，曲柄销球心的世界位置。"""
        local = self.center_local + self.radius * (
            self.u_local * math.cos(phi) + self.w_local * math.sin(phi))
        return (T_motor @ np.append(local, 1.0))[:3]

    def solve(self, poses, ball_limit, crank_limit):
        """解一条分支的被动关节角。返回 (dict 关节名→角, 诊断 dict)。"""
        T_motor = poses[self.motor_link]
        T_rod = poses[self.rod_link]
        T_link6 = poses["Joint_link6"]

        p7 = T_rod[:3, 3]
        pin_zero = self.pin_at(T_motor, 0.0)
        axis_dir = T_motor[:3, :3] @ self.axis_local
        axis_point = (T_motor @ np.append(self.axis_point, 1.0))[:3]

        psi, delta, feasible, _ = solve_crank_angle(
            p7, axis_dir, axis_point, pin_zero, self.rod_length)
        # 曲柄角同样回绕进 [-π, π]：加 2π 是同一个位形，不回绕会在行程边缘撞上假超限
        phi = _wrap(pick_crank_branch(psi, delta, self._phi))
        clamped_phi = max(-crank_limit, min(crank_limit, phi))
        target = self.pin_at(T_motor, clamped_phi)

        d6 = T_link6[:3, :3].T @ ((target - p7) / self.rod_length)
        candidates = []
        for flip in (self._flip, -self._flip):
            alpha, beta, saturated = aim_rod(self.d_local, d6, flip)
            over = abs(alpha) > ball_limit or abs(beta) > ball_limit
            alpha = max(-ball_limit, min(ball_limit, alpha))
            beta = max(-ball_limit, min(ball_limit, beta))
            rotation = T_link6[:3, :3] @ rot_y(alpha) @ rot_z(beta)
            candidates.append(((alpha, beta), rotation, saturated or over))
        # 连杆自转是自由自由度：两支都精确满足位置，取与上一刻连续的那支
        if self._rod_rotation is None:
            (alpha, beta), rotation, degraded = candidates[0]
        else:
            (alpha, beta), rotation, degraded = min(
                candidates,
                key=lambda item: _rotation_angle(item[1], self._rod_rotation))

        reached = rotation @ self.d_local * self.rod_length + p7
        position_error = float(np.linalg.norm(reached - target))
        if position_error > 1e-6:
            degraded = True
        self._phi = clamped_phi
        self._rod_rotation = rotation
        return (
            {self.cfg["crank_joint"]: clamped_phi,
             self.cfg["ball_joint"]: alpha,
             self.cfg["rod_joint"]: beta},
            {
                "phi": clamped_phi,
                "feasible": feasible and not degraded,
                "position_error_m": position_error,
                "radius_m": self.radius,
            },
        )


def _rotation_angle(a, b):
    cosine = (float(np.trace(a.T @ b)) - 1.0) / 2.0
    return abs(math.acos(min(1.0, max(-1.0, cosine))))


# ---------------------------------------------------------------------------
# ROS 节点
# ---------------------------------------------------------------------------


class WristClosureSolver(Node):
    def __init__(self):
        super().__init__("wrist_closure_solver")
        model = self.declare_parameter("model", "zhoubuguanjie_youbi_001").value
        share = get_package_share_directory("robot_arm_gazebo")
        default_urdf = f"{share}/models/{model}/urdf/{model}.urdf"
        default_cfg = f"{share}/models/{model}/config/wrist_closure.yaml"
        urdf_path = self.declare_parameter("urdf_path", default_urdf).value
        config_path = self.declare_parameter("config_path", default_cfg).value
        input_topic = self.declare_parameter("input_topic", "/joint_states_raw").value
        output_topic = self.declare_parameter("output_topic", "/joint_states").value

        with open(urdf_path, encoding="utf-8") as handle:
            self._chain = Chain(handle.read())
        with open(config_path, encoding="utf-8") as handle:
            config = yaml.safe_load(handle)

        self._branches = [Branch(self._chain, dict(config[key]))
                          for key in ("branch_a", "branch_b")]
        solver_cfg = config.get("solver", {})
        self._ball_limit = float(solver_cfg.get("ball_limit_rad", 1.0))
        self._crank_limit = float(solver_cfg.get("crank_limit_rad", 3.15))
        self._warn_m = float(solver_cfg.get("position_warn_m", 1e-6))

        self._publisher = self.create_publisher(JointState, output_topic, 10)
        self.create_subscription(JointState, input_topic, self._on_joint_state, 10)
        for branch in self._branches:
            self.get_logger().info(
                f"{branch.rod_link}: 曲柄半径 {branch.radius * 1000:.2f} mm "
                f"杆长 {branch.rod_length * 1000:.2f} mm（轴取自 URDF {branch.cfg['crank_joint']}）")
        self.get_logger().info(
            f"闭环求解器已启动：{input_topic} → {output_topic}；"
            f"球心参数来自 {config_path}（数值未标定，见文件头说明）")

    def _on_joint_state(self, message):
        angles = dict(zip(message.name, message.position))
        poses = self._chain.world_poses(angles)
        solved = dict(angles)
        for branch in self._branches:
            joints, diag = branch.solve(poses, self._ball_limit, self._crank_limit)
            solved.update(joints)
            level = self.get_logger().warn if (
                not diag["feasible"] or diag["position_error_m"] > self._warn_m
            ) else self.get_logger().debug
            level(
                f"{branch.rod_link}: 曲柄角={diag['phi']:+.3f} rad r={diag['radius_m'] * 1000:.2f} mm "
                f"位置误差={diag['position_error_m'] * 1e6:.1f} µm "
                f"{'' if diag['feasible'] else '（闭环无解或超限，已夹紧）'}")

        out = JointState()
        out.header.stamp = message.header.stamp
        out.header.frame_id = message.header.frame_id
        out.name = sorted(solved)
        out.position = [float(solved[n]) for n in out.name]
        out.velocity = [0.0] * len(out.name)
        out.effort = [0.0] * len(out.name)
        self._publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = WristClosureSolver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
